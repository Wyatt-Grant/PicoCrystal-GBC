#include <math.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"


#ifdef PIXEL_DOUBLE
  #include "screen_double.pio.h"
#else
  #include "screen.pio.h"
#endif

#include "picosystem.hpp"

namespace picosystem {

  PIO               screen_pio  = pio0;
  uint              screen_sm   = 0;
  uint32_t          dma_channel;
  volatile int16_t  dma_scanline = -1;

  uint32_t         _audio_pwm_wrap = 5000;
  struct repeating_timer _audio_update_timer;

  enum pin {
    RED = 14, GREEN = 13, BLUE = 15,                  // user rgb led
    CS = 5, SCK = 6, MOSI  = 7,                       // spi
    VSYNC = 8, DC = 9, LCD_RESET = 4, BACKLIGHT = 12, // screen
    AUDIO = 11,                                       // audio
    CHARGE_LED = 2, CHARGING = 24, BATTERY_LEVEL = 26 // battery / charging
  };

  void init_inputs(uint32_t pin_mask) {
    for(uint8_t i = 0; i < 32; i++) {
      uint32_t pin = 1U << i;
      if(pin & pin_mask) {
        gpio_set_function(i, GPIO_FUNC_SIO);
        gpio_set_dir(i, GPIO_IN);
        gpio_pull_up(i);
      }
    }
  }

  volatile bool _in_flip = false;
  volatile bool _flip_armed = false;

  // A|B|X|... below are raw pin numbers (see enum button), not pre-shifted
  // bits, so OR-ing them directly is not a valid bitmask -- it collapses to
  // a handful of low bits instead of covering pins 16-23. Shift each one
  // into place before combining.
  constexpr uint32_t BUTTON_PIN_MASK =
    (1U << A) | (1U << B) | (1U << X) | (1U << Y) |
    (1U << UP) | (1U << DOWN) | (1U << LEFT) | (1U << RIGHT);

  // ---------------------------------------------------------------------
  // Button input: debounced edge queue
  //
  // The consumer (main.cpp's run loop) can only present ONE joypad state per
  // emulated GB frame -- gb.direct.joypad is set once and the whole frame runs
  // against it. So everything that happens to a button between two polls has
  // to survive as a *sequence*, not a level.
  //
  // The old scheme latched falling edges into a single bit per button, which
  // collapsed any number of edges into "pressed for one frame". That loses
  // real input three ways: two taps in one window become one; a tap followed
  // by a press-and-hold is absorbed into the hold; and a release+re-press
  // while the pin is low at both polls is invisible entirely (the game sees
  // one unbroken hold and never re-triggers). The last one is the mash-through
  // -dialogue case.
  //
  // Instead: the IRQ (both edges now) debounces and counts *net level
  // transitions* per pin, and _poll_io() hands the consumer at most one
  // transition per frame. Every tap therefore gets at least one frame pressed
  // AND one frame released, however tightly it was squeezed between polls.
  //
  // Self-correcting by design: transitions strictly alternate (the IRQ drops
  // any edge that doesn't change the debounced level), so applying them by
  // XOR reproduces the sequence exactly -- and whenever a button's queue is
  // empty _poll_io() resyncs that bit to the raw pin instead, so a dropped
  // transition (saturation, or an edge lost while flash ops had IRQs off)
  // heals on the next frame rather than leaving the button stuck.
  // ---------------------------------------------------------------------

  // Pending net transitions per pin, oldest-first by parity from _io's bit.
  // Saturating: >4 transitions inside one 16.7ms frame is not physically
  // reachable on a mechanical switch, and dropping one only costs a resync.
  static constexpr uint8_t IO_EDGE_MAX = 4;
  static volatile uint8_t  _io_edges[32];
  // Last accepted (post-debounce) pin levels, 1 = released. Only the IRQ
  // writes this; it is the reference the next edge is compared against.
  static volatile uint32_t _io_debounced = 0xffffffff;
  // time_us_32() of each pin's last accepted edge, for the bounce lockout.
  static volatile uint32_t _io_edge_us[32];
  // Tactile switches on the PicoSystem settle in ~1-5ms. Anything shorter
  // than this after an accepted edge is contact bounce, not a new press --
  // release bounce in particular used to fire a phantom extra press (and,
  // via the Y+X / X+B chords, phantom menu opens and flash saves).
  static constexpr uint32_t IO_DEBOUNCE_US = 4000;

  // Shared GPIO IRQ, two jobs:
  //
  // Buttons: debounce the edge and, if it is a genuine level change, queue one
  // transition for the run loop to consume (see the block comment above).
  //
  // Marked __not_in_flash_func: XIP is disabled while save_storage's flash
  // erase/program run. Those wrap themselves in save_and_disable_interrupts()
  // so this cannot currently be entered from flash, but a RAM-resident handler
  // is the correct guarantee rather than a lucky one -- and it also keeps XIP
  // cache misses out of the VSYNC flip path below.
  //
  // VSYNC (the ST7789 TE pin, rising edge = beam entering the off-glass dead
  // zone at scanline 240 -- see the STE command in init): starts an armed
  // display flip there, for main.cpp's tear-free vsync setting. The GRAM
  // write the DMA feeds (~19.65 rows/ms -- 53 PIO cycles per 2px at 125MHz,
  // see screen.pio) is slightly SLOWER than the panel's own 60Hz scan-out
  // (~20.6 rows/ms), but a flip started at line 240 has the 80 dead lines +
  // back porch (~4.5ms) of head start, which the beam's ~2.45us/row gain
  // never claws back within one frame -- no panel-level tear anywhere.
  // Armed-but-still-flipping should be impossible (flips are ~12.2ms, TE
  // period ~16.7ms); if it ever happens the arm survives for the next TE.
  static void __not_in_flash_func(_gpio_irq_handler)(uint gpio, uint32_t event_mask) {
    if(gpio == VSYNC) {
      #ifndef PIXEL_DOUBLE
        if(_flip_armed && !_in_flip) {
          _flip_armed = false;
          // The same transfer _flip() starts, but read_addr/count are
          // published BEFORE _in_flip is raised: main.cpp's core1 chase loop
          // keys on (_in_flip && read_addr), and the DMA registers still hold
          // the END of the previous transfer until rewritten -- raising
          // _in_flip first would let a concurrent chase sail past a stale
          // address and overwrite rows this flip hasn't streamed yet.
          dma_channel_set_read_addr(dma_channel, SCREEN->data, false);
          dma_channel_set_trans_count(dma_channel, SCREEN->w * SCREEN->h / 2, false);
          _in_flip = true;
          dma_channel_start(dma_channel);
        }
      #endif
      return;
    }
    // Bounce lockout. Note this can also swallow the *release* of an
    // implausibly short (<4ms) tap; that costs nothing, because the queued
    // press is consumed next frame and the bit then resyncs to the raw
    // (released) pin on the frame after.
    uint32_t now = time_us_32();
    if(now - _io_edge_us[gpio] < IO_DEBOUNCE_US)
      return;

    // Only net level changes count -- this is what keeps the queued
    // transitions strictly alternating, so the consumer can apply them by XOR.
    uint32_t level = gpio_get(gpio) ? (1U << gpio) : 0;
    if(level == (_io_debounced & (1U << gpio)))
      return;

    _io_edge_us[gpio] = now;
    _io_debounced ^= (1U << gpio);
    if(_io_edges[gpio] < IO_EDGE_MAX)
      _io_edges[gpio]++;
  }

  // Seed _io/_lio and the debounce reference from the live pins, discarding
  // any queued transitions. Use at every point that (re)enters an input loop
  // cold: without it, presses made in whatever ran *before* the loop are still
  // queued and get replayed as phantom input on its first frames -- picking a
  // ROM with A used to hand the game an A press on frame 1.
  void _reset_io() {
    uint32_t ints = save_and_disable_interrupts();
    uint32_t raw = gpio_get_all();
    // Backdated a full lockout: seeding with "now" would instead open every
    // reset with a 4ms window in which a real press is discarded as bounce.
    uint32_t seed_us = time_us_32() - IO_DEBOUNCE_US;
    _io_debounced = raw;
    for(uint8_t i = 0; i < 32; i++) {
      if((1U << i) & BUTTON_PIN_MASK) {
        _io_edges[i] = 0;
        _io_edge_us[i] = seed_us;
      }
    }
    _io = _lio = raw;
    restore_interrupts(ints);
  }

  // One input sample: advance each button by at most one queued transition,
  // and resync to the live pin where there is nothing queued. IRQs are held
  // off across the whole sample so a concurrent edge can't be lost to the
  // read-modify-write on _io_edges (the old latch had exactly that race
  // between reading the latch word and zeroing it).
  void _poll_io() {
    uint32_t ints = save_and_disable_interrupts();
    uint32_t raw = gpio_get_all();
    uint32_t next = _io;
    for(uint8_t i = 0; i < 32; i++) {
      uint32_t bit = 1U << i;
      if(!(bit & BUTTON_PIN_MASK))
        continue;
      if(_io_edges[i]) {
        _io_edges[i]--;
        next ^= bit;
      } else {
        next = (next & ~bit) | (raw & bit);
      }
    }
    _lio = _io;
    _io = next;
    restore_interrupts(ints);
  }

  void init_button_input(uint32_t pin_mask) {
    for(uint8_t i = 0; i < 32; i++) {
      if((1U << i) & pin_mask)
        gpio_set_irq_enabled_with_callback(
          i, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, _gpio_irq_handler);
    }
    _reset_io();
  }

  void init_outputs(uint32_t pin_mask) {
    for(uint8_t i = 0; i < 32; i++) {
      uint32_t pin = 1U << i;
      if(pin & pin_mask) {
        gpio_set_function(i, GPIO_FUNC_SIO);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, 0);
      }
    }
  }

  bool pressed(uint32_t b) {
    return !(_io & (1U << b)) && (_lio & (1U << b));
  }

  bool button(uint32_t b) {
    return !(_io & (1U << b));
  }

  void _reset_to_dfu() {
    reset_usb_boot(0, 0);
  }

  float _battery_voltage() {
    // convert adc reading to voltage
    adc_select_input(0);
    float v = (float(adc_read()) / (1 << 12)) * 3.3f;
    return v * 3.0f; // correct for voltage divider on board
  }

  uint32_t time() {
    absolute_time_t t = get_absolute_time();
    return to_ms_since_boot(t);
  }

  uint32_t time_us() {
    absolute_time_t t = get_absolute_time();
    return to_us_since_boot(t);
  }

  void sleep(uint32_t d) {
    sleep_ms(d);
  }

  uint32_t battery() {
    // convert to 0..1 range for battery between 2.8v and 4.1v
    float c = (_battery_voltage() - 2.8f) / 1.3f;
    return std::max(0.0f, std::min(1.0f, c)) * 100;
  }

  void _wait_vsync() {
    while(gpio_get(VSYNC))  {}  // if in vsync already wait for it to end
    while(!gpio_get(VSYNC)) {}  // now wait for vsync to occur
  }

  // Measure the panel's real TE (vblank) period by timing 16 consecutive
  // rising edges on the TE pin. The ST7789's frame rate comes from its
  // internal RC oscillator, so the true period varies unit to unit around
  // the nominal value -- main.cpp uses this boot-time measurement both to
  // sanity-check the panel rate and to lock its vsync pacer / audio pacing
  // to the panel's actual clock. Returns the average period in microseconds,
  // or 0 if the TE pin never pulses (timeout ~700ms, comfortably above 16
  // intervals even at the old 40Hz rate). Blocking; call once at startup,
  // after the screen is initialised.
  uint32_t _measure_te_period_us() {
    constexpr uint32_t INTERVALS = 16;
    const uint64_t deadline = time_us_64() + 700000;
    auto wait_rising_edge = [&]() -> bool {
      while(gpio_get(VSYNC))  { if(time_us_64() > deadline) return false; }
      while(!gpio_get(VSYNC)) { if(time_us_64() > deadline) return false; }
      return true;
    };
    if(!wait_rising_edge()) return 0;
    const uint64_t t0 = time_us_64();
    for(uint32_t i = 0; i < INTERVALS; i++) {
      if(!wait_rising_edge()) return 0;
    }
    return (uint32_t)((time_us_64() - t0) / INTERVALS);
  }

  bool _is_flipping() {
    return _in_flip;
  }

  // in pixel doubling mode...
  //
  // scanline data is sent via dma to the pixel doubling pio program which then
  // writes the data to the st7789 via an spi-like interface. the pio program
  // doubles pixels horizontally, but we need to double them vertically by
  // sending each scanline to the pio twice.
  //
  // to minimise the number of dma transfers we transmit the current scanline
  // and the previous scanline in every transfer. the exceptions are the first
  // and final scanlines which are sent on their own to start and complete the
  // write.
  //
  // - transfer #1: scanline 0
  // - transfer #2: scanline 0 + scanline 1
  // - transfer #3: scanline 1 + scanline 2
  // ...
  // - transfer #n - 1: scanline (n - 1) + scanline n
  // - transfer #n: scanline n

  // sets up dma transfer for current and previous scanline (except for
  // scanlines 0 and 120 which are sent on their own.)
  void transmit_scanline() {
    // start of data to transmit
    uint32_t *s = (uint32_t *)&SCREEN->data[
      ((dma_scanline - 1) < 0 ? 0 : (dma_scanline - 1)) * 120
    ];
    // number of 32-bit words to transmit
    uint16_t c = (dma_scanline == 0 || dma_scanline == 120) ? 60 : 120;

    dma_channel_transfer_from_buffer_now(dma_channel, s, c);
  }

  // once the dma transfer of the scanline is complete we move to the
  // next scanline (or quit if we're finished)
  void __isr dma_complete() {
    if(dma_channel_get_irq0_status(dma_channel)) {
      dma_channel_acknowledge_irq0(dma_channel); // clear irq flag

      #ifdef PIXEL_DOUBLE
        if(++dma_scanline > 120) {
          // all scanlines done. reset counter and exit
          dma_scanline = -1;
          _in_flip = false;
          return;
        }
        transmit_scanline();
      #else
        _in_flip = false;
      #endif
    }
  }

  void _flip() {
    if(!_is_flipping()) {
      _in_flip = true;
      #ifdef PIXEL_DOUBLE
        // start the dma transfer of scanline data
        dma_scanline = 0;
        transmit_scanline();
      #else
        // if dma transfer already in process then skip
        uint32_t c = SCREEN->w * SCREEN->h / 2;
        dma_channel_transfer_from_buffer_now(dma_channel, SCREEN->data, c);
      #endif
    }
  }

  void screen_program_init(PIO pio, uint sm) {
    pio_clear_instruction_memory(pio);
  
    #ifdef PIXEL_DOUBLE
      uint offset = pio_add_program(pio, &screen_double_program);
      pio_sm_config c = screen_double_program_get_default_config(offset);
    #else
      uint offset = pio_add_program(pio, &screen_program);
      pio_sm_config c = screen_program_get_default_config(offset);
    #endif

    pio_sm_set_consecutive_pindirs(pio, sm, MOSI, 2, true);

    #ifndef NO_OVERCLOCK
      // dividing the clock by two ensures we keep the spi transfer to
      // around 62.5mhz as per the st7789 datasheet when overclocking
      sm_config_set_clkdiv_int_frac(&c, 2, 1);
    #endif

    // osr shifts left, autopull off, autopull threshold 32
    sm_config_set_out_shift(&c, false, false, 32);

    // configure out, set, and sideset pins
    sm_config_set_out_pins(&c, MOSI, 1);
    sm_config_set_sideset_pins(&c, SCK);

    pio_sm_set_pins_with_mask(
      pio, sm, 0, (1u << SCK) | (1u << MOSI));

    pio_sm_set_pindirs_with_mask(
      pio, sm, (1u << SCK) | (1u << MOSI), (1u << SCK) | (1u << MOSI));

    // join fifos as only tx needed (gives 8 deep fifo instead of 4)
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_gpio_init(pio, MOSI);
    pio_gpio_init(pio, SCK);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
  }

  uint16_t _gamma_correct(uint8_t v) {
    float gamma = 2.8;
    return (uint16_t)(pow((float)(v) / 100.0f, gamma) * 65535.0f + 0.5f);
  }

  void backlight(uint8_t b) {
    pwm_set_gpio_level(BACKLIGHT, _gamma_correct(b));
  }

  void _play_note(uint32_t f, uint32_t v) {
    // adjust the clock divider to achieve this desired frequency
    #ifndef NO_OVERCLOCK
      float clock = 250000000.0f;
    #else
      float clock = 125000000.0f;
    #endif

    float pwm_divider = clock / _audio_pwm_wrap / f;
    pwm_set_clkdiv(pwm_gpio_to_slice_num(AUDIO), pwm_divider);
    pwm_set_wrap(pwm_gpio_to_slice_num(AUDIO), _audio_pwm_wrap);

    // work out usable range of volumes at this frequency. the piezo speaker
    // isn't driven in a way that can control volume easily however if we're
    // clever with the duty cycle we can ensure that the ceramic doesn't have
    // time to fully deflect - effectively reducing the volume.
    //
    // through experiment it seems that constraining the deflection period of
    // the piezo to between 0 and 1/10000th of a second gives reasonable control
    // over the volume. the relationship is non linear so we also apply a
    // correction curve which is tuned so that the result sounds reasonable.
    uint32_t max_count = (f * _audio_pwm_wrap) / 10000;

    // the change in volume isn't linear - we correct for this here
    float curve = 1.8f;
    uint32_t level = (pow((float)(v) / 100.0f, curve) * max_count);
    pwm_set_gpio_level(AUDIO, level);
  }

  void led(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(RED,   _gamma_correct(r));
    pwm_set_gpio_level(GREEN, _gamma_correct(g));
    pwm_set_gpio_level(BLUE,  _gamma_correct(b));
  }

  void _screen_command(uint8_t c, size_t len = 0, const char *data = NULL) {
    gpio_put(CS, 0);
    gpio_put(DC, 0); // command mode
    spi_write_blocking(spi0, &c, 1);
    if(data) {
      gpio_put(DC, 1); // data mode
      spi_write_blocking(spi0, (const uint8_t*)data, len);
    }
    gpio_put(CS, 1);
  }

  uint32_t _gpio_get() {
    return gpio_get_all();
  }

  bool _audio_update_callback(struct repeating_timer *t) {
    _update_audio();
    return true;
  }

  void _start_audio() {
    add_repeating_timer_ms(-1, _audio_update_callback, NULL, &_audio_update_timer);
  }

  void _init_hardware() {
    // configure backlight pwm and disable backlight while setting up
    pwm_config cfg = pwm_get_default_config();
    pwm_set_wrap(pwm_gpio_to_slice_num(BACKLIGHT), 65535);
    pwm_init(pwm_gpio_to_slice_num(BACKLIGHT), &cfg, true);
    gpio_set_function(BACKLIGHT, GPIO_FUNC_PWM);
    backlight(0);

    #ifndef NO_OVERCLOCK
      // Apply a modest overvolt, default is 1.10v.
      // this is required for a stable 250MHz on some RP2040s
      vreg_set_voltage(VREG_VOLTAGE_1_20);
	    sleep_ms(10);
      // overclock the rp2040 to 250mhz
      set_sys_clock_khz(250000, true);
    #endif

    // configure control io pins
    init_inputs(BUTTON_PIN_MASK);
    init_button_input(BUTTON_PIN_MASK);
    init_outputs(CHARGE_LED);

    // configure adc channel used to monitor battery charge
    adc_init(); adc_gpio_init(BATTERY_LEVEL);

    // configure pwm channels for red, green, blue led channels
    pwm_set_wrap(pwm_gpio_to_slice_num(RED), 65535);
    pwm_init(pwm_gpio_to_slice_num(RED), &cfg, true);
    gpio_set_function(RED, GPIO_FUNC_PWM);

    pwm_set_wrap(pwm_gpio_to_slice_num(GREEN), 65535);
    pwm_init(pwm_gpio_to_slice_num(GREEN), &cfg, true);
    gpio_set_function(GREEN, GPIO_FUNC_PWM);

    pwm_set_wrap(pwm_gpio_to_slice_num(BLUE), 65535);
    pwm_init(pwm_gpio_to_slice_num(BLUE), &cfg, true);
    gpio_set_function(BLUE, GPIO_FUNC_PWM);

    // configure the spi interface used to initialise the screen
    spi_init(spi0, 8000000);

    // reset cycle the screen before initialising
    gpio_set_function(LCD_RESET, GPIO_FUNC_SIO);
    gpio_set_dir(LCD_RESET, GPIO_OUT);
    gpio_put(LCD_RESET, 0); sleep_ms(100); gpio_put(LCD_RESET, 1);

    // configure screen io pins
    gpio_set_function(DC, GPIO_FUNC_SIO); gpio_set_dir(DC, GPIO_OUT);
    gpio_set_function(CS, GPIO_FUNC_SIO); gpio_set_dir(CS, GPIO_OUT);
    gpio_set_function(SCK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    // setup the st7789 screen driver
    gpio_put(CS, 1);

    // initialise the screen configuring it as 12-bits per pixel in RGB order
    enum st7789 {
      SWRESET   = 0x01, TEON      = 0x35, MADCTL    = 0x36, COLMOD    = 0x3A,
      GCTRL     = 0xB7, VCOMS     = 0xBB, LCMCTRL   = 0xC0, VDVVRHEN  = 0xC2,
      VRHS      = 0xC3, VDVS      = 0xC4, FRCTRL2   = 0xC6, PWRCTRL1  = 0xD0,
      FRMCTR1   = 0xB1, FRMCTR2   = 0xB2, GMCTRP1   = 0xE0, GMCTRN1   = 0xE1,
      INVOFF    = 0x20, SLPOUT    = 0x11, DISPON    = 0x29, GAMSET    = 0x26,
      DISPOFF   = 0x28, RAMWR     = 0x2C, INVON     = 0x21, CASET     = 0x2A,
      RASET     = 0x2B, STE       = 0x44, DGMEN     = 0xBA,
    };

    _screen_command(SWRESET);
    sleep_ms(5);
    _screen_command(MADCTL,    1, "\x04");
    _screen_command(TEON,      1, "\x00");
    // Move the TE pulse from vblank start to scanline 240 (STE, big-endian
    // N). The controller scans 320 lines but only 0..239 are bonded to
    // glass, so with the stock vblank pulse a flip DMA gets only the ~12
    // back-porch lines (~0.6ms) of head start before the beam re-enters row
    // 0 -- and at 60Hz the beam (~20.6 rows/ms) is slightly faster than the
    // DMA (~19.65 rows/ms), leaving the two essentially tied at row 239:
    // unit-to-unit oscillator variance then shows as occasional bottom-edge
    // tear. Pulsing at line 240 instead starts the flip at the top of the
    // 80-line dead zone, growing the head start to ~92 line-times (~4.5ms)
    // -- the beam can no longer catch the DMA anywhere on the panel.
    _screen_command(STE,       2, "\x00\xF0");
    _screen_command(FRMCTR2,   5, "\x0C\x0C\x00\x33\x33");
    _screen_command(COLMOD,    1, "\x03");
    _screen_command(GAMSET,    1, "\x01");

    _screen_command(GCTRL,     1, "\x14");
    _screen_command(VCOMS,     1, "\x25");
    _screen_command(LCMCTRL,   1, "\x2C");
    _screen_command(VDVVRHEN,  1, "\x01");
    _screen_command(VRHS,      1, "\x12");
    _screen_command(VDVS,      1, "\x20");
    _screen_command(PWRCTRL1,  2, "\xA4\xA1");
    // Frame rate control: RTNA=0x0F (the ST7789 power-on default) runs the
    // panel at a nominal 60Hz instead of the stock SDK's 0x1E (~39Hz). At
    // 60Hz the TE period (~16.7ms) nearly matches the GBC's 59.73Hz frame
    // rate, so main.cpp's vsync mode can present essentially every emulated
    // frame and pace itself off TE (see the pacer there). The flip DMA
    // (~12.2ms full frame) still fits inside one TE period, and started at
    // the STE tear scanline it stays ahead of the faster 60Hz beam (see the
    // TE IRQ note above _gpio_irq_handler).
    _screen_command(FRCTRL2,   1, "\x0F");
    _screen_command(GMCTRP1,  14, "\xD0\x04\x0D\x11\x13\x2B\x3F\x54\x4C\x18\x0D\x0B\x1F\x23");
    _screen_command(GMCTRN1,  14, "\xD0\x04\x0C\x11\x13\x2C\x3F\x44\x51\x2F\x1F\x1F\x20\x23");
    _screen_command(INVON);
    sleep_ms(115);
    _screen_command(SLPOUT);
    _screen_command(CASET,     4, "\x00\x00\x00\xef");
    _screen_command(RASET,     4, "\x00\x00\x00\xef");

    // Clear GRAM to black BEFORE switching the display on. GRAM powers up
    // holding random noise and nothing else in this sequence writes it, so
    // a DISPON here would put confetti on the glass until main() pushes its
    // first frame -- normally hidden by the backlight being off, but any
    // light leaking in early (the backlight pin floats until _init_hardware
    // claims it, and panels retain charge across quick power cycles) makes
    // it flash at boot. 240x240 @ 12bpp = 86,400 bytes; bump the SPI clock
    // for the bulk write (~22ms at 31.25MHz) and drop back for commands.
    {
      uint8_t zeros[360]; // one 240px row at 12bpp
      memset(zeros, 0, sizeof(zeros));
      gpio_put(CS, 0);
      gpio_put(DC, 0); // command mode
      uint8_t ramwr = RAMWR;
      spi_write_blocking(spi0, &ramwr, 1);
      gpio_put(DC, 1); // data mode
      spi_set_baudrate(spi0, 31250000);
      for(int i = 0; i < 240; i++) {
        spi_write_blocking(spi0, zeros, sizeof(zeros));
      }
      spi_set_baudrate(spi0, 8000000);
      gpio_put(CS, 1);
    }

    _screen_command(DISPON);
    _screen_command(RAMWR);

    // switch st7789 into data mode so that we can start transmitting frame
    // data - no need to issue any more commands
    gpio_put(CS, 0);
    gpio_put(DC, 1);

    // at this stage the screen is configured and expecting to receive
    // pixel data. each time we write a screen worth of data the
    // st7789 resets the data pointer back to the start meaning that
    // we can now just leave the screen in data writing mode and
    // reassign the spi pins to our pixel doubling pio. so long as
    // we always write the entire screen we'll never get out of sync.

    // enable vsync input pin, we use this to synchronise screen updates
    // ensuring no tearing
    gpio_init(VSYNC);
    gpio_set_dir(VSYNC, GPIO_IN);

    // route the TE rising edge (vblank start) into the shared GPIO IRQ so an
    // armed flip starts exactly at vblank (see _gpio_irq_handler). The
    // callback itself was registered by init_button_input() above; always
    // enabled -- when nothing is armed the handler is a compare and return,
    // ~60 times a second.
    gpio_set_irq_enabled(VSYNC, GPIO_IRQ_EDGE_RISE, true);

    // setup the screen updating pio program
    screen_program_init(screen_pio, screen_sm);

    // initialise dma channel for transmitting pixel data to screen
    // via the screen updating pio program
    dma_channel = 0;
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(screen_pio, screen_sm, true));
    dma_channel_configure(
      dma_channel, &config, &screen_pio->txf[screen_sm], nullptr, 0, false);
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_enabled(pio_get_dreq(screen_pio, screen_sm, true), true);

    irq_remove_handler(DMA_IRQ_0, dma_complete);
    irq_add_shared_handler(DMA_IRQ_0, dma_complete, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    // initialise audio pwm pin
    int audio_pwm_slice_number = pwm_gpio_to_slice_num(AUDIO);
    pwm_config audio_pwm_cfg = pwm_get_default_config();
    pwm_init(audio_pwm_slice_number, &audio_pwm_cfg, true);
    gpio_set_function(AUDIO, GPIO_FUNC_PWM);
    pwm_set_gpio_level(AUDIO, 0);
  }

}

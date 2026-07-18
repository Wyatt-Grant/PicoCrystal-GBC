// PicoCrystal main.cpp -- NOT the stock PicoSystem init()/update()/draw()
// model. This project runs a Game Boy Color emulator (Walnut-CGB) rather
// than a native game, so it needs a custom main() driving its own loop and
// direct low-level access to the display flip/vsync primitives -- the same
// shape the sibling PicoSystem_InfoNes project uses for the same reason.
// See /home/wyatt/.claude/plans/i-want-to-port-fluttering-meadow.md.
//
// Milestones 3-6 done: frame data, input, and audio work correctly on real
// hardware. Milestone 7 (current): save persistence.

#include <array>
#include <cstdint>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/platform.h" // for __not_in_flash_func()
#include "pico/multicore.h"
#include "hardware/dma.h"  // dma_hw->ch[].read_addr, chased by the vsync mode

#include "picosystem.hpp"

// Diagnostic step (Milestone 8 performance push): fully disabled, not just
// muted. audio_output_mute()/volume=0 only skips the final PSG->PCM mix
// (minigb_apu_audio_callback); Polished Crystal's sound engine still writes
// individual APU registers (frequency, envelope, duty cycle, ...) many times
// per frame regardless, and each of those writes still called into
// minigb_apu_audio_write via __gb_write's `#if ENABLE_SOUND` branch even at
// volume 0 -- which is almost certainly why muting alone barely moved the
// needle. Setting ENABLE_SOUND to 0 compiles those calls out of __gb_read/
// __gb_write entirely, removing that per-register-access traffic too, not
// just the final mixdown. Expect this alone still isn't the whole story
// (per the user: "we'll need more than that") -- it isolates how much the
// APU subsystem as a whole (not just synthesis) is actually costing.
#define ENABLE_SOUND 1
#include "core/minigb_apu/minigb_apu.h" // MINIGB_APU_AUDIO_FORMAT_S16SYS set via CMakeLists.txt

#if ENABLE_SOUND
// Walnut-CGB calls audio_read()/audio_write() directly (not through function
// pointers), so -- matching Walnut-CGB's own reference SDL example -- these
// must be declared before walnut_cgb.h is included, and defined somewhere
// in this translation unit.
uint8_t audio_read(uint16_t addr);
void audio_write(uint16_t addr, uint8_t val);
#endif

#include "save_storage.hpp" // save_storage_mark_dirty(), used by WGB_CART_RAM_WRITE below

// ---------------------------------------------------------------------
// ROM / cart-RAM storage, plus the WGB_* direct-access overrides that
// walnut_cgb.h's hot read/write paths expand to (Milestone 8 performance
// push). Both live above the walnut_cgb.h include because the macros are
// baked into __gb_read/__gb_read16/__gb_read32/__gb_write at their
// definitions inside that header.
//
// The stock route for every ROM byte is an indirect call through
// gb->gb_rom_read* -- pointer load + blx + callee prologue/epilogue on every
// emulated instruction fetch and operand read, ~15 M0+ cycles that these
// macros replace with a bare XIP array load inlined into the (RAM-resident)
// caller. The gb_rom_read*/gb_cart_ram_* callback functions further down
// still exist and are still passed to gb_init(): cold init-time code (header
// parse, checksum) keeps using them.
// ---------------------------------------------------------------------

// Cart RAM is static, not heap-allocated, per this project's embedded
// conventions. 32KB (4 banks x 8KB, the largest MBC3/MBC5 layout in common
// use -- Polished Crystal's save, confirmed via constants/ram_constants.asm
// and the host_test harness) is the ceiling for every rom_catalog entry:
// tools/gen_rom_data.py rejects at build time any ROM whose header declares
// more, so this one shared buffer covers every title.
static uint8_t cart_ram[32 * 1024];

// Set once at boot by rom_select() (see below), before gb_init() runs. All
// ROM access -- the WGB_ROM_READ* macros below and the gb_rom_read*
// callbacks -- indexes through this rather than a fixed compile-time symbol,
// since which ROM is active is a runtime choice.
static const uint8_t *g_rom_data = nullptr;

// SRAM mirror of the first 2KB of the active ROM, filled at boot in main().
// That window is the GB memory map's hottest code: the RST vectors (pokecrystal-
// style engines `rst FarCall`/`rst Bankswitch` on nearly every cross-bank call),
// the interrupt handlers, and the top of home-bank code -- all executed
// constantly from every bank. Serving it from SRAM instead of XIP both removes
// those stalls and relieves pressure on the 16KB XIP cache (shared with all
// banked ROM data) for everything else. The `addr < sizeof` guard costs 2
// cycles on every ROM read; a single avoided QSPI cache miss (~40+ cycles)
// pays for ~20 of them. Sized 2KB because that's what fits: RAM is 99%+
// spoken for after the core1 render offload, and the linker still needs
// room for the (unused but reserved) default heap.
static uint8_t rom_mirror[2048];

static inline uint8_t wgb_rom_read8_direct(uint_fast32_t addr) {
	if (addr < sizeof(rom_mirror))
		return rom_mirror[addr];
	return g_rom_data[addr];
}

static inline uint16_t wgb_rom_read16_direct(uint_fast32_t addr) {
	// The straddling read at addr == sizeof-1 falls through to flash for
	// both bytes (same values -- the mirror is a copy, never the truth).
	const uint8_t *src = (addr < sizeof(rom_mirror) - 1) ? &rom_mirror[addr]
							     : &g_rom_data[addr];
	return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static inline uint32_t wgb_rom_read32_direct(uint_fast32_t addr) {
	const uint8_t *src = (addr < sizeof(rom_mirror) - 3) ? &rom_mirror[addr]
							     : &g_rom_data[addr];
	return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

#define WGB_ROM_READ(gb, a)      wgb_rom_read8_direct(a)
#define WGB_ROM_READ16(gb, a)    wgb_rom_read16_direct(a)
#define WGB_ROM_READ32(gb, a)    wgb_rom_read32_direct(a)
#define WGB_CART_RAM_READ(gb, a) (cart_ram[(a)])
// Same body as the gb_cart_ram_write callback: the dirty mark must never be
// separated from the write, or autosave silently stops seeing changes.
#define WGB_CART_RAM_WRITE(gb, a, v) \
	do { cart_ram[(a)] = (v); save_storage_mark_dirty(); } while (0)

// Puts walnut_cgb.h's per-instruction lookup tables (op_cycles/TAC_CYCLES)
// in SRAM instead of .rodata/XIP -- see the hook's comment in walnut_cgb.h.
#define WGB_RAM_DATA __not_in_flash("wgb_hot_tables")

// ---------------------------------------------------------------------
// Core1 scanline-render offload, phase 2 (Milestone 9): the full PPU line
// render (BG + window + sprites), not just the 1.5x scale/convert, now runs
// on core1. __gb_draw_line's per-line work on core0 shrinks to snapshotting
// the PPU registers + OAM into the line ring (WGB_LCD_LINE_OVERRIDE below;
// the OAM copy is the same 160 bytes the old pixel copy was).
//
// This stays *exact*, not approximately-right: everything mutable the
// renderer reads travels by value in the snapshot except VRAM, and VRAM is
// covered by WGB_VRAM_WRITE_FENCE -- every CPU/DMA write to VRAM first spins
// until core1 has drained the ring, so no queued line can ever observe a
// VRAM byte from its future. The fence is 2 volatile loads + compare when
// the ring is empty (the common case; core1 drains far faster than the
// emulator fills). When it does wait -- e.g. HBlank DMA streaming tiles
// right after a line was queued -- core0 stalls at most for the render of
// the lines already queued, which is work core0 itself would have done
// inline under the old design, so it can't be slower than that.
//
// The ring indexes live up here (rather than with the ring further down)
// because the fence macro must be fully expandable inside walnut_cgb.h's
// __gb_write bodies. SPSC discipline unchanged: core0 writes _line_wr,
// core1 writes _line_rd.
// ---------------------------------------------------------------------
#define ENABLE_LCD 1 // walnut_cgb.h's default, made explicit for the hooks here

static volatile uint32_t _line_wr = 0;
static volatile uint32_t _line_rd = 0;

struct gb_s; // walnut_cgb.h's context type, defined inside the include below
static void wgb_lcd_enqueue(struct gb_s *gb);
#define WGB_LCD_LINE_OVERRIDE(gb) wgb_lcd_enqueue(gb)

#define WGB_VRAM_WRITE_FENCE() \
	do { while (_line_rd != _line_wr) tight_loop_contents(); } while (0)

// Drive the MBC3 RTC from wall time (time_us_64()) instead of emulated
// cycles: cycle ticking loses real time whenever emulation falls behind
// (dropped-frame debt is discarded by the pacer) or is paused (settings
// menu), making Pokemon Crystal's clock run slow. See rtc_host_tick() below.
#define WGB_RTC_EXTERNAL_TICK 1

#include "core/walnut_cgb.h"

#include "rom_data.hpp" // generated into the build dir by tools/gen_rom_data.py
#include "audio_output.hpp"

using namespace picosystem;

// ---------------------------------------------------------------------
// picosystem.cpp is excluded from this build (see picosystem/picosystem.cmake)
// because it defines int main() and statically allocates SCREEN -- both of
// which this project needs to own itself. Replicate just the piece of its
// global state that hardware.cpp actually touches: SCREEN (for _flip()),
// _io/_lio (for button()/pressed()), and _io_press_latch (set by hardware.cpp's
// button IRQ handler; see init_button_latch()).
// ---------------------------------------------------------------------
namespace picosystem {
	static color_t _fb[240 * 240] __attribute__((aligned(4)));
	static buffer_t _screen_storage = { .w = 240, .h = 240, .data = _fb, .alloc = false };
	buffer_t *SCREEN = &_screen_storage;
	uint32_t _io = 0, _lio = 0;
	volatile uint32_t _io_press_latch = 0;
}

// ---------------------------------------------------------------------
// Walnut-CGB glue
// ---------------------------------------------------------------------
static struct gb_s gb;

// These are only reached from cold init-time paths now (gb_init's header
// parse/checksum still calls through the function pointers); every hot
// runtime access was devirtualized via the WGB_* macros above. They keep
// their __not_in_flash_func markings anyway -- they're tiny, and gb_error's
// LED blink must work even if flash is mid-erase when a fault hits.
static uint8_t __not_in_flash_func(gb_rom_read)(struct gb_s *, const uint_fast32_t addr) {
	return g_rom_data[addr];
}

static uint16_t __not_in_flash_func(gb_rom_read_16bit)(struct gb_s *, const uint_fast32_t addr) {
	const uint8_t *src = &g_rom_data[addr];
	return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t __not_in_flash_func(gb_rom_read_32bit)(struct gb_s *, const uint_fast32_t addr) {
	const uint8_t *src = &g_rom_data[addr];
	return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static uint8_t __not_in_flash_func(gb_cart_ram_read)(struct gb_s *, const uint_fast32_t addr) {
	return cart_ram[addr];
}

static void __not_in_flash_func(gb_cart_ram_write)(struct gb_s *, const uint_fast32_t addr, const uint8_t val) {
	cart_ram[addr] = val;
	save_storage_mark_dirty();
}

#if ENABLE_SOUND
static struct minigb_apu_ctx apu;

// RAM-resident: called from the RAM-resident __gb_read/__gb_write on every
// APU register access (Polished Crystal's sound engine makes many per
// frame), and the minigb_apu functions they forward to are already in RAM --
// leaving these trampolines in flash would bounce that hot path through XIP.
uint8_t __not_in_flash_func(audio_read)(uint16_t addr) {
	return minigb_apu_audio_read(&apu, addr);
}

void __not_in_flash_func(audio_write)(uint16_t addr, uint8_t val) {
	minigb_apu_audio_write(&apu, addr, val);
}
#endif

static void gb_error(struct gb_s *, const enum gb_error_e, const uint16_t) {
	// No display-worthy error path yet (that needs text() -- not linked in
	// this narrowed build). Flash the LED red so a hardware run reports a
	// core fault visibly instead of hanging silently.
	while (true) {
		led(100, 0, 0);
		sleep(150);
		led(0, 0, 0);
		sleep(150);
	}
}

#if ENABLE_SOUND
// Volume step used by the settings menu's VOLUME row (settings_step()
// below). audio_output's native range is 0..800, a "percent of unity" that
// goes up to 8x -- see audio_output.hpp.
constexpr uint16_t VOLUME_STEP = 40; // 0..800 in 20 steps -- 5% of unity per step
constexpr uint16_t VOLUME_MAX = 800;

static inline void adjust_volume(int32_t dir) { // dir > 0: up, dir < 0: down
	uint16_t vol = audio_output_get_volume();
	if (dir > 0)
		audio_output_set_volume(vol + VOLUME_STEP > VOLUME_MAX ? VOLUME_MAX : vol + VOLUME_STEP);
	else if (dir < 0)
		audio_output_set_volume(vol < VOLUME_STEP ? 0 : vol - VOLUME_STEP);
}
#endif

// backlight() (picosystem/hardware.cpp) is write-only PWM -- no getter -- so
// this tracks the level ourselves. Mirrors main()'s initial backlight(75).
// Floored above 0 (never fully black): there's no other way to see the
// picker or a running game well enough to bring brightness back up again.
static uint8_t g_brightness = 75;
constexpr uint8_t BRIGHTNESS_STEP = 5; // 0..100 in 20 steps -- 5% per step
constexpr uint8_t BRIGHTNESS_MIN = 15; // backlight PWM cuts out entirely below ~15%

static inline void adjust_brightness(int32_t dir) { // dir > 0: up, dir < 0: down
	int32_t b = (int32_t)g_brightness + dir * BRIGHTNESS_STEP;
	if (b < BRIGHTNESS_MIN) b = BRIGHTNESS_MIN;
	if (b > 100) b = 100;
	g_brightness = (uint8_t)b;
	backlight(g_brightness);
}

// Tear-free display toggle (settings menu VSYNC row, persisted). This never
// paces emulation -- the game runs at its authentic 59.73Hz with normal audio
// in both modes; the toggle only decides *when the panel gets a copy* of the
// framebuffer.
//
// OFF (default): the stock path. _flip() fires straight from the main loop,
// racing both the panel scan-out and core1's scanline writes -- zero overhead,
// but shear is visible on scrolls.
//
// ON: the main loop arms _flip_armed instead and the ST7789 TE interrupt
// starts the flip DMA at panel vblank (hardware.cpp _gpio_irq_handler) --
// GRAM writes started at vblank outrun the panel beam, so no panel-level
// tear. Buffer-level tear (emulator overwriting rows mid-stream) is prevented
// by core1 chasing the flip DMA's read_addr before storing each scaled row
// (see render_line_job). Cost when the chase engages: core1's stores briefly
// throttle to the DMA's ~21 rows/ms streaming rate. The arming policy in the
// main loop keeps that rare -- this is what the reverted always-on version
// got wrong (it armed every frame, so catch-up sprints were permanently
// throttled and core0 stalled behind the chase through the VRAM fence).
//
// volatile: written by core0 (settings menu), read by core1's store path.
static volatile bool g_vsync = false;

// Header toggles (settings menu FPS/BATTERY PERCENTAGE rows, persisted).
// g_show_fps gates the FPS counter in the in-game header band
// (draw_status_bar(), main()'s run loop); g_show_battery gates the battery
// "<n>%" text in every header -- in-game, boot menu, and settings menu alike.
// The battery icon itself always shows, its fill conveying the level.
static bool g_show_fps = true;
static bool g_show_battery = true;

// BOOT LAST GAME (settings menu row, persisted): when on, main() skips the
// boot ROM picker and boots straight into the last-played game; holding B
// during power-on forces the picker anyway. g_last_slot tracks the last
// game by its save_slot (stable across catalog reshuffles), 0xFF = none yet.
static bool g_boot_last = false;
static uint8_t g_last_slot = 0xFF;

// The LED reports battery charge: a green->red gradient (green ~= full,
// red ~= nearly empty) at ~15% brightness so it glows rather than glares.
// The usable ~5..100 span maps to the gradient, blended through yellow.
// Shared by the in-game run loop and the menus (boot ROM picker, settings).
static void led_show_battery(int level) {
	if (level < 5)   level = 5;
	if (level > 100) level = 100;
	constexpr int BRIGHTNESS = 15;      // ~15% max brightness
	int fill = (level - 5) * 100 / 95;  // 0 (empty) .. 100 (full)
	led((100 - fill) * BRIGHTNESS / 100, // red rises as it drains
	    fill * BRIGHTNESS / 100,         // green rises as it fills
	    0);
}

// Poll PicoSystem's 8 buttons and pack them into the GBC joypad byte (active
// low, per JOYPAD_* in walnut_cgb.h). PicoSystem has no Start/Select of its
// own, so per the plan: X->Start, Y->Select. Holding Y and X together opens
// the settings menu -- detected in the run loop before this is called, so a
// completed chord never reaches the game as Start+Select.
static inline void update_joypad() {
	uint8_t joypad = 0xFF;
	if (button(picosystem::UP))    joypad &= ~JOYPAD_UP;
	if (button(picosystem::DOWN))  joypad &= ~JOYPAD_DOWN;
	if (button(picosystem::LEFT))  joypad &= ~JOYPAD_LEFT;
	if (button(picosystem::RIGHT)) joypad &= ~JOYPAD_RIGHT;
	if (button(picosystem::A))     joypad &= ~JOYPAD_A;
	if (button(picosystem::B))     joypad &= ~JOYPAD_B;
	if (button(picosystem::X))     joypad &= ~JOYPAD_START;
	if (button(picosystem::Y))     joypad &= ~JOYPAD_SELECT;
	gb.direct.joypad = joypad;
}

#if ENABLE_LCD
// RGB565 (Walnut-CGB's resolved gb->cgb.fixPalette format) -> PicoSystem's
// packed RGBA4444 color_t. Plain per-pixel bit math, no LUT needed: Walnut
// already maintains fixPalette incrementally as the game writes palette RAM,
// so there's nothing left to precompute per frame.
static inline color_t rgb565_to_color(uint16_t c) {
	uint8_t r4 = (c >> 12) & 0xF; // top 4 of 5 red bits
	uint8_t g4 = (c >> 7) & 0xF;  // top 4 of 6 green bits
	uint8_t b4 = (c >> 1) & 0xF;  // top 4 of 5 blue bits
	return r4 | (0xF << 4) | (b4 << 8) | (g4 << 12);
}

// GBC's 160x144 frame scaled 1.5x to 240x216 -- an exact clean multiple of
// both dimensions (240/160 == 216/144 == 1.5). All 24px of leftover height
// goes to the top: the header band above the game canvas is then the exact
// same 24px band the menu screens use, so the chrome doesn't jump around
// between menus and gameplay.
constexpr int32_t SCALED_W = 240;
constexpr int32_t SCALED_H = 216;
constexpr int32_t OFFSET_X = (240 - SCALED_W) / 2;
constexpr int32_t OFFSET_Y = 240 - SCALED_H;

// 1.5x is a clean 3:2 ratio, so no per-pixel division or index table is needed
// -- source pixels pair up (s0, s1) and expand to three output pixels
// (s0, mid, s1), where the synthesised middle sample is the average of its two
// neighbours instead of a duplicated source pixel. That single blended column
// (and, vertically, a blended row) is what turns the old nearest-neighbour
// blockiness into a softer bilinear-style look, for the cost of one average
// per pair -- far cheaper than a true per-pixel resample and with no divides.

// Per-channel average of two RGBA4444 pixels without unpacking: the standard
// packed-pixel mean. (a & b) keeps the bits both share; ((a ^ b) & 0xEEEE) >> 1
// adds half the differing bits, and masking off each nibble's low bit before
// the shift stops the halving from carrying across channel boundaries. Alpha
// (always 0xF for palette colours) averages back to 0xF, a no-op. Inlined into
// the RAM-resident caller below, so it runs from RAM too.
static inline color_t blend_avg(color_t a, color_t b) {
	return (color_t)((a & b) + (((a ^ b) & 0xEEEE) >> 1));
}

// One previous horizontally-scaled row, kept so an odd source line can
// synthesise the smoothed midpoint row between it and the even line before it.
static color_t _prev_row[SCALED_W];

// Converted-once color palette. fixPalette (64 RGB565 entries) is maintained
// incrementally by the emulator as the game writes CGB palette RAM, and only a
// handful of entries change on any given frame. Converting it to PicoSystem
// color_t per *scaled pixel* meant 34560 rgb565_to_color() calls/frame (240 x
// 144); instead we convert the 64 entries once and reuse them as a lookup.
// Rebuilds are driven by cgb.fixPaletteDirty (set by the emulator's own
// BCPD/OCPD write handlers -- the only two places fixPalette changes), which
// replaced the old 64-entry compare-per-scanline change detection: exact, and
// costs nothing on the (vast majority of) scanlines with no palette writes.
// Mid-frame palette swaps (LYC/HBlank effects) stay correct because the dirty
// snapshot travels with each line job below.
static color_t _pal_lut[0x40];

// ---------------------------------------------------------------------
// Core1 scanline-render offload (Milestones 8+9).
//
// Milestone 8 moved the 1.5x scale/convert to core1, with core0 still
// rendering each line's pixels inline. Milestone 9 moves the render itself:
// __gb_draw_line on core0 now only snapshots the PPU state (registers,
// palettes, OAM -- see struct wgb_scanline_state in walnut_cgb.h) into this
// ring via wgb_lcd_enqueue(), and core1 runs the full BG/window/sprite
// render (__gb_render_scanline, reading VRAM live) followed by the scale.
// Exactness is preserved by WGB_VRAM_WRITE_FENCE -- see the block comment
// above the walnut_cgb.h include.
//
// The palette snapshot rides in the job only when dirty, so a line's colors
// are exactly the palette state at the moment the PPU emitted it, no matter
// how far core1 lags -- mid-frame palette effects render identically to the
// old synchronous code.
//
// Ring discipline: core0 is the only writer of _line_wr, core1 the only
// writer of _line_rd (classic SPSC indexes, declared above the walnut_cgb.h
// include so the fence macro can see them; both monotonically increasing,
// wrapped only at index time). The Cortex-M0+ executes and retires stores
// in order and the RP2040 bus fabric preserves per-master ordering, so a
// volatile index publish after the payload stores is sufficient -- the
// compiler barrier stops the compiler itself from reordering.
struct line_job {
	wgb_scanline_state st;      // PPU register/palette snapshot; st.oam -> oam[]
	uint8_t oam[0xA0];          // OAM snapshot (same 160 bytes the old pixel copy was)
	uint8_t pal_dirty;          // palette[] below is valid and must be applied
	uint16_t palette[0x40];     // fixPalette snapshot, only when pal_dirty
};
constexpr uint32_t LINE_RING_LEN = 8; // power of two
static line_job _line_ring[LINE_RING_LEN];

// Producer: called by __gb_draw_line via WGB_LCD_LINE_OVERRIDE (see above
// the walnut_cgb.h include), 144x/frame from RAM-resident emulation code on
// core0 -- so it stays RAM-resident, but is just the state snapshot now.
static void __not_in_flash_func(wgb_lcd_enqueue)(struct gb_s *gb) {
	// Backpressure guard. Shouldn't trigger in practice (core1 renders a
	// line in a fraction of the time the emulator takes to produce one),
	// but spinning here is correct if it ever does -- e.g. core1 briefly
	// parked by save_storage's flash lockout.
	while (_line_wr - _line_rd >= LINE_RING_LEN)
		tight_loop_contents();

	line_job *job = &_line_ring[_line_wr % LINE_RING_LEN];
	wgb_snapshot_scanline_state(gb, &job->st);
	memcpy(job->oam, gb->oam, sizeof(job->oam));
	job->st.oam = job->oam; // render must read the frozen copy, not live OAM
	job->pal_dirty = gb->cgb.fixPaletteDirty;
	if (gb->cgb.fixPaletteDirty) {
		memcpy(job->palette, gb->cgb.fixPalette, sizeof(job->palette));
		gb->cgb.fixPaletteDirty = 0;
	}
	// PPU state evolution that used to happen inside the render -- must
	// run here on core0, exactly once per emitted line (see walnut_cgb.h).
	wgb_advance_window_state(gb);
	__compiler_memory_barrier();
	_line_wr = _line_wr + 1;
}

// gb_init_lcd() demands a non-NULL lcd_draw_line callback (its NULL check is
// __gb_draw_line's "LCD not initialised" early-out), but with
// WGB_LCD_LINE_OVERRIDE in place the callback itself can never be invoked.
static void lcd_callback_unused(struct gb_s *, const uint8_t *, const uint_fast8_t) {}

// Consumer, on core1: full scanline render (reading VRAM live -- safe under
// the VRAM write fence) followed by the 1.5x scale/convert. RAM-resident
// both because it's real per-pixel work and because core1 flash fetches
// would contend with core0's XIP (ROM + remaining emulator code) on the
// shared QSPI bus.
static uint8_t _c1_pixels[LCD_WIDTH]; // core1-private render output buffers
static uint8_t _c1_prio[LCD_WIDTH];

static void __not_in_flash_func(render_line_job)(const line_job *job) {
	__gb_render_scanline(&gb, &job->st, _c1_pixels, _c1_prio);

	if (job->pal_dirty) {
		for (int32_t i = 0; i < 0x40; i++)
			_pal_lut[i] = rgb565_to_color(job->palette[i]);
	}

	// Everything that reads VRAM or the job slot is done -- retire the job
	// NOW, before the framebuffer stores below. This releases core0's VRAM
	// write fence and frees the ring slot while the stores (which the vsync
	// chase below may briefly block) are still pending, so core0 only ever
	// waits on renders, never on the display DMA. The store phase reads only
	// the locals captured here plus core1-private state (_c1_pixels,
	// _pal_lut, _prev_row).
	const uint32_t ly = job->st.ly;
	const bool cgb = job->st.cgb_mode;
	__compiler_memory_barrier();
	_line_rd = _line_rd + 1;

	// Vertical placement (1.5x, 3:2). Source lines pair up (2k, 2k+1) onto three
	// dest rows: the even line is row 3k, the odd line is row 3k+2, and row 3k+1
	// between them is their average. So an even line writes its own row and gets
	// stashed in _prev_row; the next odd line writes its row and then fills the
	// smoothed midpoint from _prev_row + itself. (OFFSET_X == 0, so a scaled row
	// is one contiguous framebuffer row -- we build straight into it.)
	const int32_t k = (int32_t)(ly >> 1);
	const bool odd = ly & 1;

	// Vsync chase: never overwrite framebuffer rows an in-flight flip DMA
	// hasn't streamed yet. read_addr is published before _in_flip is raised
	// (see hardware.cpp's TE handler), so this can never pass on a stale
	// address. In steady state it never spins: the DMA streams ~21 rows/ms
	// while the emulator writes ~14 rows/ms at authentic pace, and the
	// 24-row letterbox gives the reader a head start -- the spin only
	// engages when emulation bursts ahead within a frame, and the main
	// loop's arming policy keeps flips away from catch-up sprints.
	if (g_vsync) {
		const uint32_t last_row = (uint32_t)(OFFSET_Y + 3 * k) + (odd ? 2u : 0u);
		const uintptr_t chase_end = (uintptr_t)SCREEN->data +
			(uintptr_t)(last_row + 1) * SCALED_W * sizeof(color_t);
		while (_in_flip && dma_hw->ch[dma_channel].read_addr < chase_end)
			tight_loop_contents();
	}

	color_t *dst = SCREEN->p(OFFSET_X, OFFSET_Y + 3 * k + (odd ? 2 : 0));

	// Horizontal scale 1.5x with a blended middle column per pixel pair.
	if (cgb) {
		for (int32_t s = 0, d = 0; s < LCD_WIDTH; s += 2, d += 3) {
			color_t c0 = _pal_lut[_c1_pixels[s]];
			color_t c1 = _pal_lut[_c1_pixels[s + 1]];
			dst[d]     = c0;
			dst[d + 1] = blend_avg(c0, c1);
			dst[d + 2] = c1;
		}
	} else {
		// Shouldn't happen for this ROM (it's CGB-flagged), but keep a
		// visibly-distinct fallback rather than silently rendering black.
		static const color_t grey[4] = {
			rgb565_to_color(0xFFFF), rgb565_to_color(0xAD55),
			rgb565_to_color(0x52AA), rgb565_to_color(0x0000)
		};
		for (int32_t s = 0, d = 0; s < LCD_WIDTH; s += 2, d += 3) {
			color_t c0 = grey[_c1_pixels[s] & 3];
			color_t c1 = grey[_c1_pixels[s + 1] & 3];
			dst[d]     = c0;
			dst[d + 1] = blend_avg(c0, c1);
			dst[d + 2] = c1;
		}
	}

	if (odd) {
		// Row 3k+1: smoothed average of the stored even row and this odd row.
		color_t *mid = SCREEN->p(OFFSET_X, OFFSET_Y + 3 * k + 1);
		for (int32_t x = 0; x < SCALED_W; x++)
			mid[x] = blend_avg(_prev_row[x], dst[x]);
	} else {
		memcpy(_prev_row, dst, sizeof(_prev_row));
	}
}

// ---------------------------------------------------------------------
// UI drawing: in-game status bar + menu screens. text() isn't linked in this
// narrowed build (see picosystem.cmake), so this carries its own tiny 3x5
// font -- drawn at 2x (6x10 per glyph) by the fast status_text() path, and at
// arbitrary scale/letterspacing by menu_text() for the menu chrome -- plus a
// 7x7 icon set for the menu rows.
//
// The @ui-draw / @menu-draw markers fence pure-drawing code (touches only
// SCREEN, no GPIO/pico calls) so tools/render_menus.cpp can sed-extract and
// compile it on the host to render the screens as PNGs. Keep hardware access
// out of the fenced ranges.
// ---------------------------------------------------------------------
// @ui-draw-begin
static const uint8_t _font3x5[][5] = {
	{0b111, 0b101, 0b101, 0b101, 0b111}, // 0
	{0b010, 0b110, 0b010, 0b010, 0b111}, // 1
	{0b111, 0b001, 0b111, 0b100, 0b111}, // 2
	{0b111, 0b001, 0b111, 0b001, 0b111}, // 3
	{0b101, 0b101, 0b111, 0b001, 0b001}, // 4
	{0b111, 0b100, 0b111, 0b001, 0b111}, // 5
	{0b111, 0b100, 0b111, 0b101, 0b111}, // 6
	{0b111, 0b001, 0b001, 0b010, 0b010}, // 7
	{0b111, 0b101, 0b111, 0b101, 0b111}, // 8
	{0b111, 0b101, 0b111, 0b001, 0b111}, // 9
	{0b111, 0b100, 0b110, 0b100, 0b100}, // F  (10)
	{0b110, 0b101, 0b110, 0b100, 0b100}, // P  (11)
	{0b011, 0b100, 0b010, 0b001, 0b110}, // S  (12)
	{0b101, 0b001, 0b010, 0b100, 0b101}, // %  (13)
	// Remaining letters (A-Z minus F/P/S above), added for the boot ROM-select
	// screen's title text -- crude at 3px wide but legible for the short,
	// all-caps game names in rom_catalog.
	{0b010, 0b101, 0b111, 0b101, 0b101}, // A  (14)
	{0b110, 0b101, 0b110, 0b101, 0b110}, // B  (15)
	{0b011, 0b100, 0b100, 0b100, 0b011}, // C  (16)
	{0b110, 0b101, 0b101, 0b101, 0b110}, // D  (17)
	{0b111, 0b100, 0b110, 0b100, 0b111}, // E  (18)
	{0b011, 0b100, 0b101, 0b101, 0b011}, // G  (19)
	{0b101, 0b101, 0b111, 0b101, 0b101}, // H  (20)
	{0b111, 0b010, 0b010, 0b010, 0b111}, // I  (21)
	{0b001, 0b001, 0b001, 0b101, 0b010}, // J  (22)
	{0b101, 0b101, 0b110, 0b101, 0b101}, // K  (23)
	{0b100, 0b100, 0b100, 0b100, 0b111}, // L  (24)
	{0b101, 0b111, 0b101, 0b101, 0b101}, // M  (25)
	{0b101, 0b111, 0b111, 0b101, 0b101}, // N  (26)
	{0b010, 0b101, 0b101, 0b101, 0b010}, // O  (27)
	{0b010, 0b101, 0b101, 0b011, 0b001}, // Q  (28)
	{0b110, 0b101, 0b110, 0b101, 0b101}, // R  (29)
	{0b111, 0b010, 0b010, 0b010, 0b010}, // T  (30)
	{0b101, 0b101, 0b101, 0b101, 0b011}, // U  (31)
	{0b101, 0b101, 0b101, 0b101, 0b010}, // V  (32)
	{0b101, 0b101, 0b111, 0b111, 0b101}, // W  (33)
	{0b101, 0b101, 0b010, 0b101, 0b101}, // X  (34)
	{0b101, 0b101, 0b010, 0b010, 0b010}, // Y  (35)
	{0b111, 0b001, 0b010, 0b100, 0b111}, // Z  (36)
	{0b100, 0b010, 0b001, 0b010, 0b100}, // >  (37) -- menu selection marker
	{0b000, 0b010, 0b111, 0b010, 0b000}, // +  (38) -- "Y+X" footer hints
	{0b001, 0b010, 0b100, 0b010, 0b001}, // <  (39) -- settings "adjust" hint
	{0b000, 0b010, 0b000, 0b010, 0b000}, // :  (40) -- clock separator
	{0b000, 0b000, 0b000, 0b000, 0b010}, // .  (41) -- long-title ellipsis
};

static int32_t status_glyph(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	switch (c) {
		case 'F': return 10;
		case 'P': return 11;
		case 'S': return 12;
		case '%': return 13;
		case 'A': return 14;
		case 'B': return 15;
		case 'C': return 16;
		case 'D': return 17;
		case 'E': return 18;
		case 'G': return 19;
		case 'H': return 20;
		case 'I': return 21;
		case 'J': return 22;
		case 'K': return 23;
		case 'L': return 24;
		case 'M': return 25;
		case 'N': return 26;
		case 'O': return 27;
		case 'Q': return 28;
		case 'R': return 29;
		case 'T': return 30;
		case 'U': return 31;
		case 'V': return 32;
		case 'W': return 33;
		case 'X': return 34;
		case 'Y': return 35;
		case 'Z': return 36;
		case '>': return 37;
		case '+': return 38;
		case '<': return 39;
		case ':': return 40;
		case '.': return 41;
	}
	return -1; // anything else (space) just advances
}

// 8px advance per glyph (6px of ink + 2px gap).
constexpr int32_t STATUS_GLYPH_ADV = 8;

static void status_text(int32_t x, int32_t y, const char *s, color_t col) {
	for (; *s; s++, x += STATUS_GLYPH_ADV) {
		int32_t g = status_glyph(*s);
		if (g < 0)
			continue;
		for (int32_t ry = 0; ry < 5; ry++) {
			uint8_t row = _font3x5[g][ry];
			color_t *d = SCREEN->p(x, y + ry * 2);
			for (int32_t rx = 0; rx < 3; rx++) {
				if (row & (0b100 >> rx)) {
					d[rx * 2]                  = col;
					d[rx * 2 + 1]              = col;
					d[rx * 2 + SCREEN->w]      = col;
					d[rx * 2 + 1 + SCREEN->w]  = col;
				}
			}
		}
	}
}

// Digits only (both values are 0..~9999 unsigned); avoids pulling printf
// machinery into the binary for two numbers.
static int32_t status_fmt_uint(char *dst, uint32_t v) {
	char tmp[10];
	int32_t n = 0;
	do {
		tmp[n++] = '0' + v % 10;
		v /= 10;
	} while (v);
	for (int32_t i = 0; i < n; i++)
		dst[i] = tmp[n - 1 - i];
	return n;
}

// Zero-padded fixed-width variant, for the settings clock ("07", "014").
static void status_fmt_uint_pad(char *dst, uint32_t v, int32_t width) {
	for (int32_t i = width - 1; i >= 0; i--) {
		dst[i] = (char)('0' + v % 10);
		v /= 10;
	}
	dst[width] = '\0';
}

// Rendered width of an n-glyph string (drops the trailing inter-glyph gap),
// for right-aligning and centering text.
static constexpr int32_t text_w(int32_t n, int32_t scale = 2,
				int32_t adv = STATUS_GLYPH_ADV) {
	return n * adv - (adv - 3 * scale);
}

static void fill_rect(int32_t x, int32_t y, int32_t w, int32_t h, color_t c) {
	for (int32_t ry = 0; ry < h; ry++) {
		color_t *d = SCREEN->p(x, y + ry);
		for (int32_t rx = 0; rx < w; rx++)
			d[rx] = c;
	}
}

// Scalable/letterspaced variant of status_text for the menu screens: `scale`
// is pixels per font pixel (status_text is hardwired to 2), `adv` the
// per-glyph advance -- pass more than 4*scale to letterspace a heading.
// Built on fill_rect, so slower than status_text's unrolled path; menus
// redraw at ~60fps with frame time to spare, the in-game bar keeps the fast
// path.
static void menu_text(int32_t x, int32_t y, const char *s, color_t col,
		      int32_t scale, int32_t adv) {
	for (; *s; s++, x += adv) {
		int32_t g = status_glyph(*s);
		if (g < 0)
			continue;
		for (int32_t ry = 0; ry < 5; ry++) {
			uint8_t row = _font3x5[g][ry];
			for (int32_t rx = 0; rx < 3; rx++)
				if (row & (0b100 >> rx))
					fill_rect(x + rx * scale, y + ry * scale,
						  scale, scale, col);
		}
	}
}

// 7x7 1-bit icons for the menu rows, drawn at 2x (14x14) by draw_icon().
// MSB is the leftmost pixel.
enum icon_t : uint32_t { ICON_CART, ICON_SLIDERS, ICON_SUN, ICON_SPEAKER, ICON_CLOCK, ICON_SCREEN, ICON_BOLT, ICON_BATTERY, ICON_SWATCHES, ICON_CONTRAST };

static const uint8_t _icons7[][7] = {
	{ 0b1111110,  // ICON_CART -- GB cartridge, notched top-right corner
	  0b1000011,
	  0b1000001,
	  0b1011101,
	  0b1011101,
	  0b1000001,
	  0b1111111 },
	{ 0b0100000,  // ICON_SLIDERS -- two mixer sliders (settings rows)
	  0b1111111,
	  0b0100000,
	  0b0000000,
	  0b0000010,
	  0b1111111,
	  0b0000010 },
	{ 0b0001000,  // ICON_SUN -- brightness
	  0b0100010,
	  0b0011100,
	  0b1011101,
	  0b0011100,
	  0b0100010,
	  0b0001000 },
	{ 0b0001000,  // ICON_SPEAKER -- volume
	  0b0011010,
	  0b1111001,
	  0b1111101,
	  0b1111001,
	  0b0011010,
	  0b0001000 },
	{ 0b0011100,  // ICON_CLOCK
	  0b0100010,
	  0b1001001,
	  0b1001101,
	  0b1000001,
	  0b0100010,
	  0b0011100 },
	{ 0b1111111,  // ICON_SCREEN -- display panel on a stand (VSYNC row)
	  0b1000001,
	  0b1000001,
	  0b1000001,
	  0b1111111,
	  0b0001000,
	  0b0111110 },
	{ 0b0001100,  // ICON_BOLT -- lightning bolt (FPS row)
	  0b0011000,
	  0b0110000,
	  0b1111100,
	  0b0011000,
	  0b0110000,
	  0b1100000 },
	{ 0b0011100,  // ICON_BATTERY -- vertical battery, nub on top (BATTERY row)
	  0b0111110,
	  0b1000001,
	  0b1000001,
	  0b1000001,
	  0b1000001,
	  0b0111110 },
	{ 0b1110111,  // ICON_SWATCHES -- 2x2 grid of color swatches (THEME row)
	  0b1110111,
	  0b1110111,
	  0b0000000,
	  0b1110111,
	  0b1110111,
	  0b1110111 },
	{ 0b0011100,  // ICON_CONTRAST -- circle, left half filled (APPEARANCE row)
	  0b0111010,
	  0b1111001,
	  0b1111001,
	  0b1111001,
	  0b0111010,
	  0b0011100 },
};

static void draw_icon(int32_t x, int32_t y, icon_t icon, color_t col) {
	for (int32_t ry = 0; ry < 7; ry++)
		for (int32_t rx = 0; rx < 7; rx++)
			if (_icons7[icon][ry] & (0x40 >> rx))
				fill_rect(x + rx * 2, y + ry * 2, 2, 2, col);
}

// Recolor just the sticker area of ICON_CART (its inner 3x2 block, rows 3-4 /
// cols 2-4) after the cart body is drawn. The boot menu calls this when a ROM
// pins a `color` in roms.json, so each game's cart gets a distinct label.
static void draw_cart_label(int32_t x, int32_t y, color_t col) {
	for (int32_t ry = 3; ry <= 4; ry++)
		for (int32_t rx = 2; rx <= 4; rx++)
			fill_rect(x + rx * 2, y + ry * 2, 2, 2, col);
}

// RGBA4444 layout per rgb565_to_color: R in bits 0-3, A 4-7, B 8-11, G 12-15.
// STATUS_WHITE/GREY carry the primary/secondary text roles; along with the six
// UI_* neutrals below they are swapped between dark and light by apply_mode()
// (settings APPEARANCE row), so they are mutable globals, not constexpr. Their
// initializers are the dark-mode values, so the UI is correct before the stored
// mode is applied at boot.
static color_t STATUS_WHITE = 0xFFFF;
static color_t STATUS_GREY  = 0x88F8;

constexpr color_t status_rgb(uint32_t r4, uint32_t g4, uint32_t b4) {
	return (color_t)(r4 | 0xF0 | (b4 << 8) | (g4 << 12));
}

// UI palette: dark full-screen panels with a single accent color, picked from
// the themes below (settings THEME row). Everything accent-tinted -- icons,
// meter fills, header rule, selection pill, battery fill -- follows it.
struct ui_theme_t {
	const char *name;
	color_t accent;
};

constexpr ui_theme_t UI_THEMES[] = {
	{ "MINT",      status_rgb( 4, 13,  8) }, // the classic green
	{ "GRAPE",     status_rgb( 9,  5, 15) },
	{ "BERRY",     status_rgb(15,  4,  7) },
	{ "PEACH",     status_rgb(15,  9,  4) },
	{ "LEMON",     status_rgb(14, 13,  3) },
	{ "BLUEBERRY", status_rgb( 4, 10, 15) },
	{ "BUBBLEGUM", status_rgb(15,  7, 11) },
	{ "VANILLA",   status_rgb(14, 13, 10) },
};
constexpr uint32_t THEME_COUNT = sizeof(UI_THEMES) / sizeof(UI_THEMES[0]);
// Pseudo-theme past the last fixed entry: cycles UI_ACCENT through the hue
// wheel instead of a static color, so it can't live in UI_THEMES[] above.
constexpr uint32_t THEME_RGB = THEME_COUNT;
constexpr uint32_t THEME_OPTION_COUNT = THEME_COUNT + 1;

// Selected-row pill (boot menu highlight). Dark mode: the accent knocked down
// to a dark tint (each channel /4 -- MINT's (4,13,8) gives the (1,3,2) pill the
// UI always had). Light mode: the accent lifted toward white instead, so the
// highlight reads as a soft tint on the light card rather than a dark blob.
constexpr color_t theme_pill(color_t a) {
	return status_rgb((a & 0xF) >> 2, ((a >> 12) & 0xF) >> 2,
			  ((a >> 8) & 0xF) >> 2);
}
constexpr uint32_t lift4(uint32_t c) { return 15 - (15 - c) * 3 / 16; } // 13/16 toward white
constexpr color_t light_pill(color_t a) {
	return status_rgb(lift4(a & 0xF), lift4((a >> 12) & 0xF),
			  lift4((a >> 8) & 0xF));
}

static uint8_t g_theme = 0;     // UI_THEMES index, persisted in device settings
static uint8_t g_dark_mode = 1; // 1 = dark, 0 = light; persisted in device settings
static color_t UI_ACCENT  = UI_THEMES[0].accent;
static color_t UI_ROW_SEL = theme_pill(UI_THEMES[0].accent);

static void apply_theme(uint32_t idx) {
	g_theme = (uint8_t)(idx < THEME_OPTION_COUNT ? idx : 0);
	// RGB pseudo-theme: seed a starting hue here; update_rgb_theme() takes
	// over recomputing UI_ACCENT every LED-update tick while it's selected.
	UI_ACCENT = g_theme < THEME_COUNT ? UI_THEMES[g_theme].accent
					   : hsv(0.0f, 1.0f, 1.0f);
	UI_ROW_SEL = g_dark_mode ? theme_pill(UI_ACCENT) : light_pill(UI_ACCENT);
}

// RGB pseudo-theme: cycles UI_ACCENT (and, via led_show_rgb(), the power LED)
// through the hue wheel over time. Called from the same throttled ~0.75s
// cadence as the battery-LED update (menu_battery_poll::poll(), the run
// loop's BATTERY_UPDATE_FRAMES block) rather than every frame -- a step per
// tick still reads as a smooth cycle without adding a per-frame cost.
static void update_rgb_theme() {
	if (g_theme != THEME_RGB)
		return;
	static uint32_t step = 0;
	step = (step + 1) % 32; // 32 hue steps per revolution
	UI_ACCENT = hsv((float)step / 32.0f, 1.0f, 1.0f);
	UI_ROW_SEL = g_dark_mode ? theme_pill(UI_ACCENT) : light_pill(UI_ACCENT);
}

// Drives the power LED from the current (just-updated) UI_ACCENT, mirroring
// the RGB pseudo-theme on screen. Same brightness cap as led_show_battery()
// so it glows rather than glares.
static void led_show_rgb() {
	constexpr int BRIGHTNESS = 15;
	led((uint8_t)((UI_ACCENT & 0xF) * BRIGHTNESS / 15),
	    (uint8_t)(((UI_ACCENT >> 12) & 0xF) * BRIGHTNESS / 15),
	    (uint8_t)(((UI_ACCENT >> 8) & 0xF) * BRIGHTNESS / 15));
}

// Neutral background/text ramp, swapped as a set by apply_mode() (see below).
// Initialized to the dark-mode values so the UI is correct before the stored
// mode is applied at boot; light mode inverts them (light card, dark text).
static color_t UI_CARD    = status_rgb(1, 1, 1);  // panel body
static color_t UI_HEADER  = status_rgb(2, 2, 2);  // header band
static color_t UI_TRACK   = status_rgb(3, 3, 3);  // meter tracks, header rule
static color_t UI_VALUE   = status_rgb(6, 6, 6);   // unselected value text (e.g. ROM size)
static color_t UI_FILL    = status_rgb(6, 6, 6);   // unselected meter fill (kept dark on the light track)
static color_t UI_BRIGHT  = status_rgb(11, 11, 11); // clock digits (unselected)
static color_t STATUS_DIM = status_rgb(5, 5, 5);   // faint text: hints, colons

// Dark/light appearance (settings APPEARANCE row), orthogonal to the accent
// theme above: it swaps only the neutral ramp and the pill tint direction,
// leaving the chosen accent (and the amber/red low-battery literals) alone.
// g_dark_mode (declared with the theme state above) defaults to 1 so a fresh
// device -- or one whose settings record was reset by an upgrade -- keeps the
// classic dark look.
struct ui_mode_t {
	color_t white, grey, card, header, track, value, fill, bright, dim;
};
constexpr ui_mode_t UI_MODES[2] = {
	// [0] DARK -- the values the UI has always used.
	{ 0xFFFF, 0x88F8, status_rgb(1, 1, 1), status_rgb(2, 2, 2),
	  status_rgb(3, 3, 3), status_rgb(6, 6, 6), status_rgb(6, 6, 6),
	  status_rgb(11, 11, 11), status_rgb(5, 5, 5) },
	// [1] LIGHT -- light card, dark text. Unselected text (grey/value) stays
	// light so the accent-colored selected row reads as the focus; the meter
	// fill stays dark so it's legible against the lighter track.
	{ status_rgb(1, 1, 1), status_rgb(10, 10, 10), status_rgb(15, 15, 15),
	  status_rgb(13, 13, 13), status_rgb(11, 11, 11), status_rgb(10, 10, 10),
	  status_rgb(6, 6, 6), status_rgb(3, 3, 3), status_rgb(9, 9, 9) },
};

static void apply_mode(uint32_t dark) {
	g_dark_mode = dark ? 1 : 0;
	const ui_mode_t &m = UI_MODES[g_dark_mode ? 0 : 1];
	STATUS_WHITE = m.white;
	STATUS_GREY  = m.grey;
	UI_CARD   = m.card;
	UI_HEADER = m.header;
	UI_TRACK  = m.track;
	UI_VALUE  = m.value;
	UI_FILL   = m.fill;
	UI_BRIGHT = m.bright;
	STATUS_DIM = m.dim;
	UI_ROW_SEL = g_dark_mode ? theme_pill(UI_ACCENT) : light_pill(UI_ACCENT);
}

// Battery icon: a 17x13 rounded body outline plus a 3x5 terminal nub (20px
// total), its interior filled proportionally to the charge level -- a fixed
// green/yellow/red gradient, independent of the UI accent theme (including
// the RGB pseudo-theme) so the charge state always reads the same way.
constexpr int32_t BATT_ICON_W = 20;
constexpr color_t BATT_GREEN = status_rgb(4, 13, 8); // >= 66%

static void draw_battery_icon(int32_t x, int32_t y, uint32_t batt) {
	for (int32_t i = 1; i < 16; i++) {
		*SCREEN->p(x + i, y)      = STATUS_GREY;
		*SCREEN->p(x + i, y + 12) = STATUS_GREY;
	}
	for (int32_t i = 1; i < 12; i++) {
		*SCREEN->p(x, y + i)      = STATUS_GREY;
		*SCREEN->p(x + 16, y + i) = STATUS_GREY;
	}
	for (int32_t i = 4; i <= 8; i++) {
		*SCREEN->p(x + 17, y + i) = STATUS_GREY;
		*SCREEN->p(x + 18, y + i) = STATUS_GREY;
		*SCREEN->p(x + 19, y + i) = STATUS_GREY;
	}
	color_t fill = batt >= 66 ? BATT_GREEN            // healthy
		     : batt >= 11 ? status_rgb(13, 10, 2) // 11..65%
				  : status_rgb(13, 3, 2);    // <= 10%
	int32_t fw = (int32_t)(batt * 13 + 50) / 100; // rounded, 0..13 -- leaves a
							// 1px gap on both sides of
							// the interior at full charge
	for (int32_t ry = 2; ry <= 10; ry++) {
		color_t *d = SCREEN->p(x + 2, y + ry);
		for (int32_t rx = 0; rx < fw; rx++)
			d[rx] = fill;
	}
}

// "<n>% [icon]", right-aligned so the icon's right edge lands at `right`.
// Shared by the in-game status bar and the menu card headers. show_pct=false
// drops the "<n>%" text and keeps the icon (the in-game BATTERY PERCENTAGE
// toggle -- the icon's fill still shows the level at a glance).
//
// The icon (17x13, see draw_battery_icon) is 3px taller than the "<n>%"
// text (6x10 glyphs at status_text's 2x scale), so drawing both at the same
// `top` leaves their vertical centers off by 1.5px -- the icon reads as
// sitting low against the text. BATT_ICON_Y_ADJ shifts the icon up to
// re-center it against the text's midline.
constexpr int32_t BATT_ICON_Y_ADJ = 2;
static void draw_battery_block(int32_t right, int32_t top, uint32_t batt,
				bool show_pct = true) {
	char buf[12];
	draw_battery_icon(right - BATT_ICON_W, top - BATT_ICON_Y_ADJ, batt);
	if (!show_pct)
		return;
	int32_t n = status_fmt_uint(buf, batt);
	buf[n] = '%';
	buf[n + 1] = '\0';
	status_text(right - BATT_ICON_W - 5 - text_w(n + 1), top, buf, STATUS_WHITE);
}

// Shared header band: the top OFFSET_Y (24px) rows, used identically by the
// in-game status bar and the menu screens -- dark strip, hairline rule along
// its bottom edge, letterspaced label left, battery right. In-game it sits in
// the letterbox above the scaled GBC frame, so it can't race core1's scanline
// rendering and the emulator never overdraws it.
constexpr int32_t HDR_TOP = 7;    // (24 - 10px glyph height) / 2
constexpr int32_t UI_MARGIN = 12; // screen-edge margin for all UI content
constexpr int32_t UI_RIGHT = 240 - UI_MARGIN;

// In-game header rule: always dark, regardless of light/dark mode. This
// hairline sits directly above the scaled GBC frame, not other UI chrome, so
// it shouldn't lighten with UI_TRACK's light-mode ramp -- in light mode
// UI_TRACK is near-white there and reads as a stray bright line against the
// game content below it.
constexpr color_t STATUS_RULE = status_rgb(3, 3, 3);

// bottom_gap reserves that many rows at the band's bottom edge (painted in the
// band color, below the rule) so the hairline rule doesn't sit flush against
// whatever follows. Used in-game to keep the rule off the top of the scaled
// GBC frame; menus leave it 0 so the rule butts the card as intended.
static void draw_header_band(const char *title, uint32_t batt, color_t rule,
			      bool show_pct = true, int32_t bottom_gap = 0) {
	int32_t rule_y = OFFSET_Y - 1 - bottom_gap;
	fill_rect(0, 0, SCREEN->w, rule_y, UI_HEADER);
	fill_rect(0, rule_y, SCREEN->w, 1, rule);
	if (bottom_gap > 0)
		fill_rect(0, rule_y + 1, SCREEN->w, bottom_gap, UI_HEADER);
	menu_text(UI_MARGIN, HDR_TOP, title, STATUS_GREY, 2, 12);
	draw_battery_block(UI_RIGHT, HDR_TOP, batt, show_pct);
}

// In-game status bar. Repainted only when a shown value changes (~1x/sec), so
// the cost is negligible and it stays off the per-frame budget. The settings
// menu independently toggles the FPS readout (g_show_fps -- an empty title
// skips the label/number) and the battery percentage text (g_show_battery --
// the icon itself always shows).
static void draw_status_bar(uint32_t fps, uint32_t batt, bool saving) {
	// An in-flight autosave commit replaces the FPS readout with a WRITING
	// TO FLASH indicator -- shown even with the FPS toggle off, so a
	// power-off during the flash write window is always signposted. Normal
	// 8px advance, not the header's letterspaced 12px: at 16 glyphs the
	// letterspaced version would run into the battery block.
	if (saving) {
		draw_header_band("", batt, STATUS_RULE, g_show_battery, 1);
		status_text(UI_MARGIN, HDR_TOP, "WRITING TO FLASH", UI_ACCENT);
		return;
	}
	draw_header_band(g_show_fps ? "FPS" : "", batt, STATUS_RULE, g_show_battery, 1);
	if (g_show_fps) {
		char buf[12];
		int32_t n = status_fmt_uint(buf, fps);
		buf[n] = '\0';
		status_text(UI_MARGIN + 3 * 12 + 4, HDR_TOP, buf, STATUS_WHITE);
	}
}

// ---------------------------------------------------------------------
// Menus: boot-time ROM picker + settings screen -- full-screen dark panels
// under the shared header band, with dim control hints along the bottom.
// Both are modal: each owns a tiny poll/draw loop (mirroring the
// _io/_lio/_io_press_latch dance the real run loop uses for pressed()) and
// only returns when the user leaves. Opened in-game, the settings menu
// therefore pauses emulation by construction: core0 simply isn't calling
// gb_run_frame_dualfetch() while it's open.
// ---------------------------------------------------------------------

// Fills the whole screen with the panel color and draws the header band with
// a green accent rule (the in-game FPS overlay draws its own grey-ruled band
// directly via draw_header_band()). Menu headers follow the BATTERY
// PERCENTAGE toggle just like the in-game header.
static void draw_menu_frame(const char *title, uint32_t batt) {
	fill_rect(0, OFFSET_Y, SCREEN->w, SCREEN->h - OFFSET_Y, UI_CARD);
	draw_header_band(title, batt, UI_ACCENT, g_show_battery);
}

// Dim, centered control hints along the bottom of the panel.
static void draw_menu_hints(const char *s) {
	int32_t n = 0;
	while (s[n])
		n++;
	status_text((SCREEN->w - text_w(n)) / 2, 226, s, STATUS_DIM);
}
// @ui-draw-end

// Menus poll the battery on the same slow cadence the in-game loop uses (see
// BATTERY_UPDATE_FRAMES in main()) -- the level changes slowly, so reading
// the ADC every menu frame would be waste. The counter starts at the
// threshold so the very first frame paints a real level instead of leaving
// the icon blank for ~0.5s. Each fresh reading also drives the battery LED,
// so the gradient shows from the boot ROM picker onward, not just in-game.
struct menu_battery_poll {
	static constexpr int INTERVAL_FRAMES = 30;
	int count = INTERVAL_FRAMES;
	uint32_t level = 0;
	uint32_t poll() {
		if (++count >= INTERVAL_FRAMES) {
			count = 0;
			int b = battery();
			if (b < 0)   b = 0;
			if (b > 100) b = 100;
			level = (uint32_t)b;
			if (g_theme == THEME_RGB) {
				update_rgb_theme();
				led_show_rgb();
			} else {
				led_show_battery(b);
			}
		}
		return level;
	}
};

// RTC fields shown/edited by the settings menu. The MBC3 has no calendar,
// just a raw day counter -- and the GBC Pokemon games only consume it mod 7
// as the day of the week -- so the menu edits a weekday + time instead of a
// meaningless 0..511 day number. The day counter is seeded to the weekday
// index (Sunday == 0).
//
// Before a game boots these are staged values, applied to the MBC3 clock
// right after gb_init() (which leaves it zeroed; the board has no
// battery-backed RTC) -- but only as the FALLBACK seed: a save that carries
// a persisted RTC record (see save_storage's rtc_record_t) wins, so the
// clock resumes where the last committed save left it instead of jumping to
// the staged time on every boot. That keeps Pokemon Crystal's SRAM-stored
// time offset valid across power cycles (in-game time simply doesn't
// advance while the device is off; nudge it here when the lag matters).
// Once in-game, settings_menu() re-reads the live clock on open and writes
// edits straight back, so the game sees changes immediately. The values as
// last set are also persisted to flash (see save_storage_settings_*) and
// restored at boot.
static uint8_t g_rtc_dow = 0;  // 0..6, Sunday == 0
static uint8_t g_rtc_hour = 0; // 0..23
static uint8_t g_rtc_min = 0;  // 0..59

static uint64_t g_rtc_last_us = 0; // wall-clock baseline for rtc_host_tick()

static void rtc_apply_to_gb() {
	gb.rtc_real.reg.min = g_rtc_min;
	gb.rtc_real.reg.hour = g_rtc_hour;
	gb.rtc_real.reg.yday = g_rtc_dow;
	// Keep the halt (bit 6) and day-overflow (bit 7) flags; clear day bit 8.
	gb.rtc_real.reg.high = (gb.rtc_real.reg.high & 0xC0);
	// A manual set restarts the current second (like the MBC3 prescaler)
	// and discards wall time elapsed before the edit.
	gb.counter.rtc_count = 0;
	g_rtc_last_us = time_us_64();
}

// Advances the MBC3 clock by real elapsed time (WGB_RTC_EXTERNAL_TICK
// repurposes counter.rtc_count as a microsecond remainder). Core0 only, and
// only OUTSIDE gb_run_frame_dualfetch(): rtc_real, the latch memcpy, and
// the RTC register writes all execute inside the frame step on this core,
// so ticking between frames can never race them. Core1 never touches
// gb.rtc_* or gb.counter.
static void rtc_host_tick() {
	uint64_t now = time_us_64();
	uint64_t elapsed = now - g_rtc_last_us;
	g_rtc_last_us = now;
	if (gb.mbc != 3)
		return;
	if (gb.rtc_real.reg.high & 0x40)
		return; // HALT: elapsed time is discarded, like a stopped watch
	gb.counter.rtc_count += elapsed;
	while (gb.counter.rtc_count >= 1000000) {
		gb.counter.rtc_count -= 1000000;
		gb_rtc_tick_second(&gb);
	}
}

// Snapshot source for the RTC record committed with each autosave (see
// save_storage_set_rtc_provider). Runs inside save_storage_poll on core0
// between frames, so the read is coherent.
static bool rtc_snapshot(save_rtc_t &out) {
	if (gb.mbc != 3)
		return false;
	memcpy(out.reg, gb.rtc_real.bytes, sizeof(out.reg));
	out.subsec_us = (uint32_t)gb.counter.rtc_count;
	return true;
}

// @menu-draw-begin
enum settings_row_t : uint32_t {
	SET_ROW_BRIGHT,
#if ENABLE_SOUND
	SET_ROW_VOLUME,
#endif
	SET_ROW_VSYNC,
	SET_ROW_FPS,
	SET_ROW_BATTERY,
	SET_ROW_BOOT_LAST,
	SET_ROW_THEME,
	SET_ROW_MODE, // APPEARANCE: dark/light
	SET_ROW_DOW, // the clock rows render as the big segmented clock below
	SET_ROW_HOUR,
	SET_ROW_MIN,
	SET_ROW_COUNT,
};

static const char *const RTC_DOW_NAMES[7] = {
	"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT",
};

// The segmented clock: DOW : HH : MM, 9 glyph cells at 4x (12x20 per glyph).
constexpr int32_t CLK_SCALE = 4;
constexpr int32_t CLK_ADV = 4 * CLK_SCALE;

// Every setting row is a single text line now -- BRIGHT/VOLUME carry a compact
// inline meter bar on the same line rather than a full-width bar underneath --
// so all rows share one height. Uniform 18px keeps the RTC clock section below
// clear of draw_menu_hints()'s fixed y=226 (with rows through APPEARANCE the
// clock anchors at y=178 and the digits end at ~y=212).
constexpr int32_t ROW_H = 18;

static int32_t settings_row_y(uint32_t row) {
	return OFFSET_Y + 10 + (int32_t)row * ROW_H;
}

// Selected-row highlight pill -- same two-rect shape as the boot menu's row
// highlight (draw_boot_menu), offset for settings rows' icon/text baseline:
// boot's `y` is the row top with its icon drawn at y+4, while settings rows
// draw their icon directly at `y`, so this shifts by that same 4px to keep
// the pill centered on the row content.
static void draw_row_highlight(int32_t y) {
	fill_rect(UI_MARGIN - 6, y - 3, UI_RIGHT - UI_MARGIN + 12, 18, UI_ROW_SEL);
	fill_rect(UI_MARGIN - 5, y - 4, UI_RIGHT - UI_MARGIN + 10, 20, UI_ROW_SEL);
}

// Value row: icon + label, a right-aligned text value where the meter rows
// have a bar. Shared by the toggles (ON/OFF) and the THEME row (theme name).
static void draw_value_row(uint32_t row, uint32_t sel, icon_t icon,
			    const char *label, const char *value) {
	int32_t y = settings_row_y(row);
	bool is_sel = (row == sel);
	int32_t n = 0;
	while (value[n])
		n++;
	if (is_sel)
		draw_row_highlight(y);
	draw_icon(UI_MARGIN, y, icon, is_sel ? UI_ACCENT : STATUS_GREY);
	status_text(UI_MARGIN + 22, y + 2, label, is_sel ? UI_ACCENT : STATUS_GREY);
	status_text(UI_RIGHT - text_w(n), y + 2, value,
		    is_sel ? UI_ACCENT : STATUS_GREY);
}

static void draw_toggle_row(uint32_t row, uint32_t sel, icon_t icon,
			     const char *label, bool value) {
	draw_value_row(row, sel, icon, label, value ? "ON" : "OFF");
}

static void draw_settings_menu(uint32_t sel, uint32_t batt) {
	draw_menu_frame("SETTINGS", batt);

	char buf[8];

	// Meter rows: icon + label, then an inline bar and its % value, all on one
	// line. Selection reads by color -- the selected row's value and fill go
	// accent green. The % text is right-aligned at UI_RIGHT, but its width
	// changes with the digit count (9%..100%), so the bar is anchored to a
	// fixed slot sized for the widest value ("100%") and holds still as the
	// number ticks rather than shifting with it.
	constexpr int32_t MINI_BAR_W = 64, MINI_BAR_H = 8;
	constexpr int32_t bar_x = UI_RIGHT - text_w(4) - 8 - MINI_BAR_W;
	for (uint32_t row = 0; row < SET_ROW_VSYNC; row++) {
		int32_t y = settings_row_y(row);
		bool is_sel = (row == sel);

		icon_t icon = ICON_SUN;
		const char *label = "BRIGHTNESS";
		uint32_t value = g_brightness;
#if ENABLE_SOUND
		if (row == SET_ROW_VOLUME) {
			icon = ICON_SPEAKER;
			label = "VOLUME";
			// Normalized to 0-100 (native range is 0-800) so it
			// reads on the same scale as brightness.
			value = ((uint32_t)audio_output_get_volume() * 100 + VOLUME_MAX / 2) / VOLUME_MAX;
		}
#endif
		if (is_sel)
			draw_row_highlight(y);
		draw_icon(UI_MARGIN, y, icon, is_sel ? UI_ACCENT : STATUS_GREY);
		status_text(UI_MARGIN + 22, y + 2, label,
			    is_sel ? UI_ACCENT : STATUS_GREY);
		int32_t n = status_fmt_uint(buf, value);
		buf[n] = '%';
		buf[n + 1] = '\0';
		status_text(UI_RIGHT - text_w(n + 1), y + 2, buf,
			    is_sel ? UI_ACCENT : STATUS_GREY);
		// Bar vertically centered on the 10px-tall glyphs (y+2..y+12).
		int32_t by = y + 2;
		fill_rect(bar_x, by, MINI_BAR_W, MINI_BAR_H, UI_TRACK);
		fill_rect(bar_x, by, (int32_t)(value * MINI_BAR_W / 100), MINI_BAR_H,
			  is_sel ? UI_ACCENT : UI_FILL);
	}

	// VSYNC: ON trades a little emulation headroom for tear-free scrolling
	// (TE-synchronised flips; see g_vsync's comment). FPS/BATTERY toggle the
	// matching piece of the in-game header (draw_status_bar()).
	draw_toggle_row(SET_ROW_VSYNC, sel, ICON_SCREEN, "VSYNC", g_vsync);
	draw_toggle_row(SET_ROW_FPS, sel, ICON_BOLT, "FPS", g_show_fps);
	draw_toggle_row(SET_ROW_BATTERY, sel, ICON_BATTERY, "BATTERY PERCENTAGE",
			g_show_battery);
	// BOOT LAST GAME: ON skips the boot ROM picker and auto-boots the
	// last-played game (hold B during power-on to get the picker back).
	draw_toggle_row(SET_ROW_BOOT_LAST, sel, ICON_CART, "BOOT LAST GAME",
			g_boot_last);
	// THEME: the value is the theme's name (or "RGB" for the cycling
	// pseudo-theme); every accent-tinted element on screen (including this
	// value) recolors live as < > cycles it.
	draw_value_row(SET_ROW_THEME, sel, ICON_SWATCHES, "THEME",
		       g_theme < THEME_COUNT ? UI_THEMES[g_theme].name : "RGB");
	// APPEARANCE: dark/light -- swaps the neutral ramp live, so the whole
	// panel (and this row) recolors as < > toggles it.
	draw_value_row(SET_ROW_MODE, sel, ICON_CONTRAST, "APPEARANCE",
		       g_dark_mode ? "DARK" : "LIGHT");

	// RTC clock section: DOW : HH : MM as a big segmented clock with a small
	// caption over each group and the clock icon in the left margin beside
	// the digits; the selected group (the one < > adjusts) goes accent
	// green. No label row -- the captions + icon carry the meaning, and the
	// saved height (plus the 18px row pitch) is what keeps the toggle rows
	// and this clock clear of draw_menu_hints()'s fixed y=226.
	bool clk_sel = (sel >= SET_ROW_DOW);

	struct {
		const char *text; // pre-rendered group text (weekday name / digits)
		const char *label;
		int32_t label_len;
		uint32_t row;
	} grp[3] = {
		{ RTC_DOW_NAMES[g_rtc_dow], "DAY", 3, SET_ROW_DOW  },
		{ buf,                      "HR",  2, SET_ROW_HOUR },
		{ buf + 4,                  "MIN", 3, SET_ROW_MIN  },
	};
	status_fmt_uint_pad(buf, g_rtc_hour, 2);
	status_fmt_uint_pad(buf + 4, g_rtc_min, 2);

	int32_t gy = settings_row_y(SET_ROW_DOW); // group captions
	int32_t dy = gy + 14;                     // big glyphs
	// Icon vertically centered on the 20px-tall digits ((20 - 14px icon)/2).
	draw_icon(UI_MARGIN, dy + 3, ICON_CLOCK, clk_sel ? UI_ACCENT : STATUS_GREY);
	int32_t x = (SCREEN->w - text_w(9, CLK_SCALE, CLK_ADV)) / 2;
	for (int32_t i = 0; i < 3; i++) {
		bool is_sel = (grp[i].row == sel);
		int32_t glyphs = i == 0 ? 3 : 2;
		int32_t gw = text_w(glyphs, CLK_SCALE, CLK_ADV);
		menu_text(x + (gw - text_w(grp[i].label_len)) / 2, gy, grp[i].label,
			  is_sel ? UI_ACCENT : STATUS_DIM, 2, STATUS_GLYPH_ADV);
		menu_text(x, dy, grp[i].text, is_sel ? UI_ACCENT : UI_BRIGHT,
			  CLK_SCALE, CLK_ADV);
		x += glyphs * CLK_ADV;
		if (i < 2) {
			menu_text(x, dy, ":", STATUS_DIM, CLK_SCALE, CLK_ADV);
			x += CLK_ADV;
		}
	}

	draw_menu_hints("< > ADJUST   B BACK");
}

// Small solid triangle, apex up or down -- the boot menu's scroll indicator.
static void draw_scroll_arrow(int32_t x, int32_t y, bool down, color_t c) {
	for (int32_t k = 0; k < 3; k++)
		fill_rect(x - k, down ? y - k : y + k, 1 + 2 * k, 1, c);
}

// Boot ROM-select body: catalog rows (icon, name, size) plus the trailing
// SETTINGS row, the selected row drawn as a highlight pill. Catalogs taller
// than the screen scroll: a sticky window offset follows the selection.
static void draw_boot_menu(uint32_t sel, uint32_t batt) {
	constexpr int32_t ROW_H = 20;
	// Rows that fit between the header and draw_menu_hints()'s fixed y=226
	// (the last one lands at y=196..216 even with the SETTINGS gap).
	constexpr uint32_t VISIBLE_ROWS = 9;
	const uint32_t total = ROM_COUNT + 1; // + trailing SETTINGS row
	static uint32_t first = 0; // window start, persists across frames
	if (sel < first)
		first = sel;
	if (sel >= first + VISIBLE_ROWS)
		first = sel - VISIBLE_ROWS + 1;

	draw_menu_frame("BOOT", batt);

	char buf[8];
	const uint32_t rows = total < VISIBLE_ROWS ? total : VISIBLE_ROWS;
	for (uint32_t r = 0; r < rows; r++) {
		uint32_t i = first + r;
		// The SETTINGS row sits apart by a small extra gap.
		int32_t y = OFFSET_Y + 12 + (int32_t)r * ROW_H + (i == ROM_COUNT ? 4 : 0);
		bool is_sel = (i == sel);
		if (is_sel) {
			// Rounded highlight pill across the row.
			fill_rect(UI_MARGIN - 6, y + 1, UI_RIGHT - UI_MARGIN + 12, 18, UI_ROW_SEL);
			fill_rect(UI_MARGIN - 5, y, UI_RIGHT - UI_MARGIN + 10, 20, UI_ROW_SEL);
		}
		draw_icon(UI_MARGIN, y + 4,
			  i < ROM_COUNT ? ICON_CART : ICON_SLIDERS,
			  is_sel ? UI_ACCENT : STATUS_DIM);
		// Per-ROM cart label tint (roms.json "color"), 0 = leave default.
		// The selected row stays fully theme-accent, so skip the overlay there.
		if (i < ROM_COUNT && !is_sel && rom_catalog[i].label_color)
			draw_cart_label(UI_MARGIN, y + 4, rom_catalog[i].label_color);

		const char *name = i < ROM_COUNT ? rom_catalog[i].name : "SETTINGS";
		int32_t name_x = UI_MARGIN + 22;
		int32_t name_max = UI_RIGHT - name_x;
		if (i < ROM_COUNT) {
			// Right-aligned size column, accent on the selected row.
			// GBC ROM sizes are always a power of two (per the cartridge
			// header size field), so an exact multiple of 1024K is shown
			// in whole megabytes instead ("1M" rather than "1024K").
			uint32_t kb = (rom_catalog[i].size + 1023) / 1024;
			int32_t n;
			if (kb % 1024 == 0) {
				n = status_fmt_uint(buf, kb / 1024);
				buf[n] = 'M';
			} else {
				n = status_fmt_uint(buf, kb);
				buf[n] = 'K';
			}
			buf[n + 1] = '\0';
			int32_t vx = UI_RIGHT - text_w(n + 1);
			status_text(vx, y + 6, buf, is_sel ? UI_ACCENT : UI_VALUE);
			name_max = vx - 4 - name_x;
		}

		// Clip long titles to the space left of the size column: the name's
		// ink (text_w, no trailing glyph gap) must fit in name_max, so short
		// size strings ("2M" vs "512K") buy extra name characters.
		int32_t max_chars = (name_max + STATUS_GLYPH_ADV - 6) / STATUS_GLYPH_ADV;
		int32_t len = 0;
		while (name[len])
			len++;
		char clipped[32];
		if (len > max_chars) {
			for (int32_t c = 0; c < max_chars - 2; c++)
				clipped[c] = name[c];
			clipped[max_chars - 2] = '.';
			clipped[max_chars - 1] = '.';
			clipped[max_chars] = '\0';
			name = clipped;
		}
		status_text(name_x, y + 6, name, is_sel ? UI_ACCENT : STATUS_GREY);
	}

	// Scroll indicators when rows extend past the window.
	if (first > 0)
		draw_scroll_arrow(UI_RIGHT + 4, OFFSET_Y + 5, false, STATUS_DIM);
	if (first + VISIBLE_ROWS < total)
		draw_scroll_arrow(UI_RIGHT + 4, 221, true, STATUS_DIM);

	draw_menu_hints("A SELECT   Y+X SETTINGS");
}
// @menu-draw-end

// Set by settings_step() so settings_menu() knows whether anything actually
// changed and is worth persisting to flash on close.
static bool g_settings_edited = false;

// Set alongside it for the clock rows specifically: an in-game clock edit
// must also force an autosave, or the (older) RTC record already on flash
// would override the edit on the next boot.
static bool g_rtc_edited = false;

// Snapshot the settings globals and persist them: one synchronous ~50ms
// flash write, skipped inside the store when nothing actually differs from
// the record already on flash. Shared by the settings-menu close and main()'s
// last-booted-game recording. The initializer order mirrors device_settings_t.
static void store_device_settings() {
	device_settings_t ds = {
		g_brightness, g_rtc_dow, g_rtc_hour, g_rtc_min,
#if ENABLE_SOUND
		audio_output_get_volume(),
#else
		0,
#endif
		(uint8_t)(g_vsync ? 1 : 0),
		(uint8_t)(g_show_fps ? 1 : 0),
		(uint8_t)(g_show_battery ? 1 : 0),
		g_theme,
		g_dark_mode,
		(uint8_t)(g_boot_last ? 1 : 0),
		g_last_slot,
	};
	save_storage_settings_store(ds);
}

static void settings_step(uint32_t row, int32_t dir, bool in_game) {
	g_settings_edited = true;
	switch (row) {
	case SET_ROW_BRIGHT: adjust_brightness(dir); return;
#if ENABLE_SOUND
	case SET_ROW_VOLUME: adjust_volume(dir); return;
#endif
	case SET_ROW_VSYNC: g_vsync = !g_vsync; return; // either direction toggles
	case SET_ROW_FPS: g_show_fps = !g_show_fps; return; // either direction toggles
	case SET_ROW_BATTERY: g_show_battery = !g_show_battery; return; // either direction toggles
	case SET_ROW_BOOT_LAST: g_boot_last = !g_boot_last; return; // either direction toggles
	case SET_ROW_THEME:
		apply_theme((g_theme + THEME_OPTION_COUNT + (uint32_t)dir) % THEME_OPTION_COUNT);
		return;
	case SET_ROW_MODE: apply_mode(!g_dark_mode); return; // either direction toggles
	// RTC fields wrap so either direction reaches any value quickly.
	case SET_ROW_DOW:  g_rtc_dow  = (uint8_t)((g_rtc_dow + 7 + dir) % 7);    break;
	case SET_ROW_HOUR: g_rtc_hour = (uint8_t)((g_rtc_hour + 24 + dir) % 24); break;
	case SET_ROW_MIN:  g_rtc_min  = (uint8_t)((g_rtc_min + 60 + dir) % 60);  break;
	}
	g_rtc_edited = true; // only the clock rows fall through the switch
	if (in_game)
		rtc_apply_to_gb();
}

// Modal settings screen, shared by the boot menu (its SETTINGS row, or Y+X)
// and in-game (Y+X chord, detected in the run loop).
static void settings_menu(bool in_game) {
	uint32_t sel = 0;
	menu_battery_poll batt;

	if (in_game) {
		// Show the live MBC3 clock, not the stale boot-time staging. The
		// day counter ticks past 6 as play spans midnights; fold it back
		// to a weekday (the only meaning the games give it).
		g_rtc_dow = (uint8_t)(((gb.rtc_real.reg.yday |
					((uint16_t)(gb.rtc_real.reg.high & 1) << 8))) % 7);
		g_rtc_hour = gb.rtc_real.reg.hour;
		g_rtc_min = gb.rtc_real.reg.min;
	}

	// The buttons that opened the menu (Y+X, or A on the boot menu's
	// SETTINGS row) are still held on the first iterations -- ignore input
	// until everything is released so opening can't immediately toggle back
	// out or activate a row.
	bool wait_release = true;
	uint64_t next_repeat = 0;

	while (true) {
		_lio = _io;
		_io = _gpio_get() & ~_io_press_latch;
		_io_press_latch = 0;

		if (wait_release) {
			if (!(button(picosystem::A) || button(picosystem::B) ||
			      button(picosystem::X) || button(picosystem::Y)))
				wait_release = false;
		} else {
			if (pressed(picosystem::B) ||
			    (button(picosystem::X) && button(picosystem::Y)))
				break;
			if (pressed(picosystem::UP))
				sel = (sel + SET_ROW_COUNT - 1) % SET_ROW_COUNT;
			if (pressed(picosystem::DOWN))
				sel = (sel + 1) % SET_ROW_COUNT;

			// Left/Right adjust the selected row, with hold-to-repeat
			// (initial delay, then fast) so ranges like DAY 0..511 aren't
			// hundreds of individual taps.
			int32_t dir = 0;
			uint64_t now = time_us_64();
			if (pressed(picosystem::LEFT) || pressed(picosystem::RIGHT)) {
				dir = pressed(picosystem::RIGHT) ? +1 : -1;
				next_repeat = now + 350000;
			} else if ((button(picosystem::LEFT) || button(picosystem::RIGHT)) &&
				   now >= next_repeat) {
				dir = button(picosystem::RIGHT) ? +1 : -1;
				next_repeat = now + 60000;
			}
			if (dir)
				settings_step(sel, dir, in_game);
		}

		if (in_game) {
#if ENABLE_SOUND
			// Hold the speaker at a clean silent level while paused
			// instead of stuck at whatever it last played.
			audio_output_mute();
#endif
			// Wall time keeps flowing into the clock while the game is
			// paused here (real MBC3 behavior), and an in-menu autosave
			// then commits a fresh RTC snapshot.
			rtc_host_tick();
			// Keep autosave flushing -- the player may well open this
			// right after an in-game save.
			save_storage_poll(cart_ram, sizeof(cart_ram));
		}

		draw_settings_menu(sel, batt.poll());
		_flip();
		sleep(16); // no timing requirements -- just enough for ~60fps input polling
	}

	// Persist the settings (brightness/volume/clock) if anything was
	// actually edited -- one synchronous ~50ms flash write, unnoticeable
	// here where nothing is animating. Skipped entirely on a look-only
	// visit so browsing the menu never wears flash.
	// An in-game clock edit must reach flash even if the game never saves
	// again this session -- otherwise the older RTC record wins on next
	// boot and undoes the edit. One extra 32KB autosave cycle, only on an
	// actual clock edit.
	if (in_game && g_rtc_edited)
		save_storage_mark_dirty();
	g_rtc_edited = false;

	if (g_settings_edited) {
		g_settings_edited = false;
		store_device_settings();
	}

	// Drain the closing press so B (or the Y+X chord) doesn't leak into
	// whatever screen resumes underneath (game or boot menu).
	do {
		_lio = _io;
		_io = _gpio_get() & ~_io_press_latch;
		_io_press_latch = 0;
		sleep(8);
	} while (button(picosystem::A) || button(picosystem::B) ||
		 button(picosystem::X) || button(picosystem::Y));
}

// Boot-time ROM picker. Runs once in main(), before gb_init() -- nothing
// else is driving the display yet. The list is the ROM catalog plus a
// trailing SETTINGS row (set apart by a half-row gap).
static uint32_t rom_select() {
	uint32_t sel = 0;
	constexpr uint32_t MENU_ROWS = ROM_COUNT + 1; // + trailing SETTINGS row

	// Seed _io from the current GPIO level so the loop's first pressed()
	// poll below reflects real state instead of the zero-initialized
	// default (which would read as "every button just released").
	_io = _gpio_get();

	menu_battery_poll batt;

	while (true) {
		_lio = _io;
		_io = _gpio_get() & ~_io_press_latch;
		_io_press_latch = 0;

		if (pressed(picosystem::A)) {
			if (sel < ROM_COUNT)
				return sel;
			settings_menu(false); // the SETTINGS row
		}
		if (button(picosystem::X) && button(picosystem::Y))
			settings_menu(false);

		if (pressed(picosystem::DOWN))
			sel = (sel + 1) % MENU_ROWS;
		if (pressed(picosystem::UP))
			sel = (sel + MENU_ROWS - 1) % MENU_ROWS;

		draw_boot_menu(sel, batt.poll());

		_flip();
		sleep(16); // menu has no timing requirements -- just enough for ~60fps input polling
	}
}
#endif

#if ENABLE_SOUND
// Set by core0 once per frame (when unmuted); core1 clears it after
// synthesizing. Plain flag, no FIFO -- the inter-core hardware FIFO is
// reserved for save_storage's multicore_lockout protocol.
static volatile bool _synth_requested = false;
#endif

// Core1 worker loop: drain scanline jobs (priority -- they're latency
// sensitive within a frame), then handle the once-per-frame audio synthesis
// request. IRQs stay enabled throughout, so the audio DMA completion IRQ and
// save_storage's lockout FIFO IRQ both still preempt this loop normally.
static void __not_in_flash_func(core1_main)() {
	// Registers core1 as able to respond to multicore_lockout_start_blocking()
	// from core0 -- save_storage.cpp uses this to safely pause core1 during
	// flash erase/program (flash is unreadable, including for instruction
	// fetch, on both cores while that's happening).
	multicore_lockout_victim_init();

#if ENABLE_SOUND
	audio_output_core1_init(); // DMA completion IRQ must be bound on this core
#endif

	while (true) {
#if ENABLE_LCD
		if (_line_rd != _line_wr) {
			// render_line_job advances _line_rd itself, between its render
			// and store phases (see its retire comment).
			render_line_job(&_line_ring[_line_rd % LINE_RING_LEN]);
			continue;
		}
#endif
#if ENABLE_SOUND
		if (_synth_requested) {
			_synth_requested = false;
			audio_output_render_frame(&apu);
		}
#endif
	}
}

int main() {
	_init_hardware(); // includes the 250MHz/1.20V overclock (see hardware.cpp)

	for (int32_t y = 0; y < SCREEN->h; y++)
		for (int32_t x = 0; x < SCREEN->w; x++)
			*SCREEN->p(x, y) = 0; // black border around the GBC frame

	// Restore the persisted device settings (brightness, volume, staged RTC)
	// before anything is displayed, so the boot menu already comes up with
	// the user's values. Defaults (g_brightness = 75, volume muted, clock
	// Sunday midnight) stand when nothing was ever stored.
	{
		device_settings_t ds;
		if (save_storage_settings_load(ds)) {
			g_brightness = ds.brightness < BRIGHTNESS_MIN ? BRIGHTNESS_MIN
				     : ds.brightness > 100 ? 100 : ds.brightness;
			g_rtc_dow  = ds.rtc_dow  % 7;
			g_rtc_hour = ds.rtc_hour % 24;
			g_rtc_min  = ds.rtc_min  % 60;
			g_vsync = ds.vsync != 0;
			g_show_fps = ds.show_fps != 0;
			g_show_battery = ds.show_battery != 0;
			apply_theme(ds.theme); // out-of-range falls back to MINT
			apply_mode(ds.dark_mode); // restore dark/light appearance
			g_boot_last = ds.boot_last != 0;
			g_last_slot = ds.last_slot;
#if ENABLE_SOUND
			audio_output_set_volume(ds.volume); // clamps internally
#endif
		}
	}

	// _init_hardware() explicitly sets backlight(0) as part of setup (screen
	// off while the caller gets ready) -- the stock boot sequence in the
	// excluded picosystem.cpp ramps it back up after its logo animation, but
	// we skip that whole sequence. Turned on here (rather than after gb_init,
	// as before) so the boot ROM-select screen below is actually visible;
	// with stored settings this applies the restored level directly.
	backlight(g_brightness);

#if ENABLE_LCD
	// BOOT LAST GAME: skip the picker and jump straight to the last-played
	// game, unless B is held during power-on (the escape hatch back to the
	// menu -- X can't serve, held at power-on it forces DFU boot). A last
	// slot that no longer matches the catalog (never recorded, or the ROM
	// list changed) falls through to the picker as before.
	uint32_t rom_index = UINT32_MAX;
	if (g_boot_last) {
		// Read live GPIO -- the same seeding rom_select() does -- so the
		// held-B check reflects real state, not the zero-initialized _io.
		_io = _gpio_get();
		if (!button(picosystem::B))
			for (uint32_t i = 0; i < ROM_COUNT; i++)
				if (rom_catalog[i].save_slot == g_last_slot) {
					rom_index = i;
					break;
				}
	}
	if (rom_index == UINT32_MAX)
		rom_index = rom_select();

	// Record the booted game so BOOT LAST GAME knows where to return -- kept
	// current even while the toggle is off, so enabling it later already
	// points at the right game. Auto-boots leave the slot unchanged, so the
	// store's no-op check means flash is only written when switching games.
	if ((uint8_t)rom_catalog[rom_index].save_slot != g_last_slot) {
		g_last_slot = (uint8_t)rom_catalog[rom_index].save_slot;
		store_device_settings();
	}
#else
	// No display to pick from in this diagnostic build -- always boot the
	// first catalog entry.
	uint32_t rom_index = 0;
#endif
	const rom_entry_t &rom = rom_catalog[rom_index];
	g_rom_data = rom.data;
	// Fill the SRAM mirror of ROM 0x0000-0x0FFF (see rom_mirror up top) --
	// must happen before gb_init(), whose header parse already reads ROM.
	memcpy(rom_mirror, g_rom_data, sizeof(rom_mirror));

	save_storage_load(cart_ram, sizeof(cart_ram), rom.save_slot); // restore a prior save, if any, before boot

	enum gb_init_error_e ret = gb_init(&gb, &gb_rom_read, &gb_rom_read_16bit,
					    &gb_rom_read_32bit, &gb_cart_ram_read,
					    &gb_cart_ram_write, &gb_error, nullptr);
	if (ret != GB_INIT_NO_ERROR) {
		// ROM failed checksum/MBC validation -- distinct from a runtime
		// gb_error() so it's diagnosable from hardware: solid red vs blink.
		led(100, 0, 0);
		while (true) {}
	}

#if ENABLE_LCD
	// Restore the MBC3 clock. Preferred source is the RTC record committed
	// with the loaded save (freeze-while-off: the clock resumes exactly
	// where the last save left it, keeping the game's SRAM time-offset
	// valid across power cycles). Saves without a record -- fresh files, or
	// saves written by pre-RTC-record firmware -- fall back to the staged
	// boot-menu clock, the only other way a game starts anywhere but day 0
	// midnight (no battery-backed RTC on the board). Harmless for non-MBC3
	// carts (the registers are never mapped).
	save_rtc_t boot_rtc;
	if (save_storage_load_rtc(boot_rtc)) {
		memcpy(gb.rtc_real.bytes, boot_rtc.reg, sizeof(gb.rtc_real.bytes));
		gb.counter.rtc_count = boot_rtc.subsec_us;
	} else {
		rtc_apply_to_gb();
	}
	// Snapshot the clock into every committed save from here on.
	save_storage_set_rtc_provider(&rtc_snapshot);
#endif

#if ENABLE_LCD
	// The callback is a placeholder: WGB_LCD_LINE_OVERRIDE routes every line
	// to wgb_lcd_enqueue() instead (see the ring section above).
	gb_init_lcd(&gb, &lcd_callback_unused);
	gb.cgb.fixPaletteDirty = 1; // force the first line job to carry a palette snapshot

	// Frame-skip rendering (currently OFF -- rendering every frame to gauge the
	// full-rate cost): when set, __gb_draw_line renders every scanline on
	// alternate frames only (whole frames skipped, not individual lines),
	// halving the PPU + scale/convert cost without the per-scanline shimmer that
	// interlacing produced, at the cost of a ~30fps visible refresh. Either way
	// the emulated CPU steps every frame, so game timing/accuracy is unaffected.
	// The vertical smoothing below works in both modes: it needs both source
	// lines of each pair present in order, which full-frame draws always give.
	gb.direct.frame_skip = 0;
#endif

#if ENABLE_SOUND
	minigb_apu_audio_init(&apu);
	audio_output_init(); // PWM setup only; core1 is launched below
#endif

	// Core1 worker: scanline scale/convert + audio synthesis + the audio
	// DMA IRQ all live there now. Launched after gb_init/gb_init_lcd so the
	// ring's producer can't run before the consumer exists, and before the
	// first gb_run_frame_dualfetch() for the same reason.
	multicore_launch_core1(core1_main);

	led(0, 15, 0); // solid green once init succeeds, before the run loop

	// picosystem.cpp's excluded stock loop is what normally refreshes
	// _io/_lio each frame -- without this, button()/pressed() silently read
	// stale (always-zero) state and button() would report "pressed" forever.
	_io = _gpio_get();

	// The LED reports battery charge: a green->red gradient (green ~= full,
	// red ~= nearly empty) at ~15% brightness. Updated on a cadence rather than
	// every frame -- battery() reads the ADC and the charge level changes
	// slowly, so per-frame polling would only waste cycles and jitter the LED.
	// Starting the counter at the threshold makes the first loop iteration do
	// an immediate update instead of sitting on the boot-success green for ~0.5s.
	constexpr int BATTERY_UPDATE_FRAMES = 30;
	int battery_count = BATTERY_UPDATE_FRAMES;

#if ENABLE_LCD
	// Status bar state: the values currently painted in the top letterbox,
	// plus a ~1s wall-clock window for the FPS count. batt_shown = -1 makes
	// the first battery poll (immediate, see battery_count above) paint the
	// bar on the first frame instead of leaving it blank for a second.
	uint32_t fps_shown = 0;
	bool saving_shown = false;       // WRITING TO FLASH indicator painted
	uint64_t saving_hold_until = 0;  // keeps it up past a fast commit
	int32_t batt_shown = -1;
	uint64_t batt_hold_until = 0; // displayed level pinned until this time
	uint32_t fps_frames = 0;
	uint64_t fps_window_start = time_us_64();
#endif

	// Real-time pacer (replaces _wait_vsync()). The panel refreshes at 40Hz
	// but the GBC runs at 59.73Hz -- gating each emulated frame on vsync
	// capped the game at 40/59.73 = 67% speed no matter how fast emulation
	// got, and silently swallowed every other optimization's gains. Instead,
	// pace to the GBC's true frame period: run flat-out while behind real
	// time, and only wait when a frame finishes early, so the game can never
	// run *faster* than authentic speed. The cost is losing vsync alignment
	// on _flip() (possible shear on fast scrolls); tearing was already
	// half-accepted anyway, since emulation draws into the same buffer the
	// flip DMA reads from.
	constexpr uint64_t GB_FRAME_US = 16742; // 70224 cycles / 4.194304MHz
	uint64_t frame_due = time_us_64();
	g_rtc_last_us = frame_due; // discard boot/menu time predating the game

	// Consecutive behind-schedule frames that skipped arming a vsync flip --
	// see the arming policy at the bottom of the loop.
	uint32_t vsync_skipped = 0;

	while (true) {
		_lio = _io;
		// _io_press_latch (set from a GPIO falling-edge IRQ, see
		// picosystem/hardware.cpp) catches any button that went down and back
		// up entirely between two polls -- this loop's cadence isn't constant
		// (it stalls on core1 and on save-flash erases), so a plain level
		// sample of _gpio_get() can miss a tap that fits inside one of those
		// gaps. Fold it in here, then clear it for the next window.
		_io = _gpio_get() & ~_io_press_latch;
		_io_press_latch = 0;

#if ENABLE_LCD
		bool status_dirty = false;
		// Y+X chord (either button completing it) opens the settings menu,
		// pausing emulation until it closes -- settings_menu() runs its own
		// loop in place of this one. Checked before update_joypad() so the
		// completed chord never reaches the game as Start+Select (though the
		// first button of the chord does, alone, for the frames before its
		// partner lands -- unavoidable when the chord is built from game
		// buttons).
		if (button(picosystem::X) && button(picosystem::Y) &&
		    (pressed(picosystem::X) || pressed(picosystem::Y))) {
			// The menu drives _flip() directly -- a pending armed flip
			// firing mid-menu-draw would stream a half-painted panel.
			_flip_armed = false;
			settings_menu(true);
			// The menu painted over the whole screen and the pause stalled
			// real time. The emulator repaints its 216 rows every frame,
			// but the letterbox bands are only painted on change -- clear
			// everything and force a status-bar repaint. Restart the pacer
			// and FPS window at "now" so the game doesn't fast-forward to
			// repay the pause (and the next FPS reading isn't nonsense).
			// Let the menu's last flip finish first so the clear doesn't
			// blank rows the DMA is still streaming to the panel.
			while (_in_flip)
				tight_loop_contents();
			memset(SCREEN->data, 0, SCREEN->w * SCREEN->h * sizeof(color_t));
			status_dirty = true;
			frame_due = time_us_64();
			fps_window_start = frame_due;
			fps_frames = 0;
		}
#endif
		update_joypad();

		gb_run_frame_dualfetch(&gb);

		// Feed real elapsed time into the MBC3 clock -- before the save
		// poll below, so a flush always snapshots a current clock.
		rtc_host_tick();

#if ENABLE_SOUND
		// PSG synthesis (~549 samples of 4-channel frequency/envelope/duty
		// work) now runs on core1 -- this is just a request flag. Still
		// skipped entirely when muted rather than synthesizing audio nobody
		// will hear; audio_output_mute() forces a clean silent level instead
		// of leaving the speaker stuck at whatever it last played.
		if (audio_output_get_volume() > 0) {
			_synth_requested = true;
		} else {
			audio_output_mute();
		}
#endif

		save_storage_poll(cart_ram, sizeof(cart_ram));

#if ENABLE_LCD
		// Repaint the header when the autosave commit window opens or
		// closes -- draw_status_bar() swaps the FPS readout for WRITING TO
		// FLASH. The commit programs ~1s of 512B chunks; hold the indicator
		// so it stays on screen a beat past the write and is easy to notice.
		constexpr uint64_t SAVING_HOLD_US = 2000000;
		uint64_t saving_now = time_us_64();
		if (save_storage_saving() && !saving_shown)
			saving_hold_until = saving_now + SAVING_HOLD_US;
		bool saving = save_storage_saving() || saving_now < saving_hold_until;
		if (saving != saving_shown) {
			saving_shown = saving;
			status_dirty = true;
		}
#endif

		if (++battery_count >= BATTERY_UPDATE_FRAMES) {
			battery_count = 0;
			int level = battery();
			if (level < 0)   level = 0;
			if (level > 100) level = 100;
#if ENABLE_LCD
			// Anti-flicker: the ADC reading jitters between adjacent
			// levels, so once a level is painted it stays pinned for 3s
			// before a different reading may replace it. batt_hold_until
			// starts at 0, so the very first reading paints immediately.
			uint64_t batt_now = time_us_64();
			if (level != batt_shown && batt_now >= batt_hold_until) {
				batt_shown = level;
				batt_hold_until = batt_now + 3000000;
				status_dirty = true;
			}
#endif
			if (g_theme == THEME_RGB) {
				// The battery icon's fill is fixed (not accent-tied), and
				// nothing else in the in-game header reads UI_ACCENT, so
				// the new hue needs no forced repaint here -- only the LED
				// changes on this tick.
				update_rgb_theme();
				led_show_rgb();
			} else {
				led_show_battery(level);
			}
		}

#if ENABLE_LCD
		// FPS = emulated frames completed over the last ~1s wall-clock
		// window. The pacer below caps the loop at the GBC's authentic
		// 59.73Hz, so "60" means full speed and anything lower is genuine
		// slowdown. Repaint only when a displayed value actually changed.
		fps_frames++;
		uint64_t fps_now = time_us_64();
		uint64_t fps_elapsed = fps_now - fps_window_start;
		if (fps_elapsed >= 1000000) {
			uint32_t fps = (uint32_t)((fps_frames * 1000000ull + fps_elapsed / 2) / fps_elapsed);
			fps_window_start = fps_now;
			fps_frames = 0;
			if (fps != fps_shown) {
				fps_shown = fps;
				status_dirty = true;
			}
		}
		if (status_dirty) {
			// The header band is rows 0..OFFSET_Y-1 -- the very rows an
			// in-flight vsync flip streams first. Same chase as core1's;
			// ~1x/sec and worst-case ~1ms, so off the frame budget.
			if (g_vsync) {
				const uintptr_t hdr_end = (uintptr_t)SCREEN->data +
					(uintptr_t)OFFSET_Y * SCREEN->w * sizeof(color_t);
				while (_in_flip && dma_hw->ch[dma_channel].read_addr < hdr_end)
					tight_loop_contents();
			}
			draw_status_bar(fps_shown, batt_shown < 0 ? 0 : (uint32_t)batt_shown,
					saving_shown);
		}

		// Let core1 finish this frame's queued scanlines before flipping,
		// so the DMA never streams a frame with lines still landing. Core1
		// runs well ahead of the emulator, so this is normally a no-op wait.
		while (_line_rd != _line_wr)
			tight_loop_contents();
#endif

		frame_due += GB_FRAME_US;
		uint64_t now = time_us_64();

		// Vsync arming policy (the frame is complete in the buffer here --
		// the ring drain above guarantees it). On-schedule frames always
		// arm: the writer then paces at the authentic ~14 rows/ms next
		// frame, which the flip DMA's ~21 rows/ms outruns, so core1's chase
		// stays a no-op. Behind-schedule frames (catch-up sprints, where
		// emulation runs flat-out and WOULD collide with the DMA) skip
		// arming so the chase never throttles the sprint -- that throttling
		// is what got the always-on version reverted. Every 3rd sprint
		// frame arms anyway so the screen keeps moving through sustained
		// slowdown; cheap there, because an emulator that can't hit 60fps
		// writes rows slower than the DMA streams them.
		if (g_vsync && (now < frame_due || ++vsync_skipped >= 3)) {
			vsync_skipped = 0;
			_flip_armed = true;
		}

		if (now < frame_due) {
			// Ahead of real time: hold the line at authentic game speed.
			while (time_us_64() < frame_due)
				tight_loop_contents();
		} else if (now - frame_due > 4 * GB_FRAME_US) {
			// Fell badly behind (e.g. a save-flash erase pause): drop the
			// accumulated debt instead of fast-forwarding to repay it.
			frame_due = now;
		}

		if (!g_vsync)
			_flip();
	}
}

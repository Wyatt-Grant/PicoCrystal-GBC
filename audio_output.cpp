#include "audio_output.hpp"

#include <cstdint>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h" // hardware_alarm_*, paces the startup chime
#include "pico/platform.h" // __not_in_flash_func()
#include "pico/time.h"     // time_us_64(), paces audio_output_play_pcm_blocking

namespace {

// GPIO 11 -- the PicoSystem's piezo speaker PWM pin. Not exposed via
// picosystem.hpp (it's a private enum inside hardware.cpp); confirmed by
// reading picosystem/hardware.cpp's `enum pin { ... AUDIO = 11 ... }`.
constexpr uint AUDIO_PIN = 11;

// 12-bit duty-cycle resolution, carrier ~61kHz at the 250MHz overclock
// (250e6 / 4096) -- comfortably above both audible range and the piezo's own
// bandwidth, so it reads as a clean crude DAC rather than an audible whine.
constexpr uint16_t PWM_WRAP = 4095;

// The screen driver (picosystem/hardware.cpp) hardcodes DMA channel 0
// without claiming it, so dma_claim_unused_channel() could hand that same
// channel back to us. Claim a fixed channel 1 instead (panics on conflict,
// which is what we want if another user ever appears).
constexpr uint AUDIO_DMA_CH = 1;

// GPIO 11 is PWM slice 5, channel B: duty lives in bits 31:16 of the slice's
// CC register. Each playback word is (duty << 16); the low half writes
// channel A's compare, which drives nothing (GPIO 10 is unconnected on the
// PicoSystem and never set to GPIO_FUNC_PWM).
uint32_t duty_buf[2][AUDIO_SAMPLES];

// Double-buffer state, all core1-only after init (render_frame in thread
// context vs the DMA IRQ handler -- same core, so a brief IRQ-disable is a
// sufficient critical section).
volatile uint8_t play_buf_idx = 0; // buffer the DMA is (or was last) playing
volatile bool dma_idle = true;     // no transfer in flight; PWM holds its last duty
volatile bool buf_pending = false; // a filled buffer is published but not yet started

// Synthesis staging buffer (interleaved stereo from minigb_apu). Owned here
// rather than by the caller: only core1 ever touches it.
audio_sample_t stage_buf[AUDIO_SAMPLES_TOTAL];

// One-pole high-pass state (core1 only). See render_frame for why.
int32_t hpf_x1 = 0, hpf_y1 = 0;

// Playback pacing (see audio_output_set_frame_period). frame_period_us is
// how often the emulator actually produces a frame's worth of samples --
// 16742us at authentic GBC pace, or the measured panel TE period when
// main.cpp's TE-paced vsync mode locks emulation to the panel clock.
// pace_timer is the claimed DREQ timer index, -1 until core1_init runs.
volatile uint32_t frame_period_us = 16742;
volatile int pace_timer = -1;

// Program the DREQ timer so consumption runs a hair (+~0.05%) above the
// production rate of AUDIO_SAMPLES per frame_period_us. Drift then always
// lands on the graceful side -- a few-microsecond PWM hold at a buffer
// boundary -- rather than the bad side, where an unconsumed buffer is
// eventually overwritten and a whole frame of audio skips. Divisor is
// sys_clk cycles per sample: frame_period_us * 250 / AUDIO_SAMPLES cycles
// at 250MHz, minus 1/2500th for the margin (16742us -> 7637 - 3 = 7634,
// ~32757Hz against ~32732Hz production -- matching the retired hardcoded
// 1/7635 setting).
void retune_pace_timer() {
	if (pace_timer < 0)
		return;
	uint32_t div = (uint32_t)((uint64_t)frame_period_us * 250u / AUDIO_SAMPLES);
	div -= div / 2500;
	dma_timer_set_fraction((uint)pace_timer, 1, (uint16_t)div);
}

// minigb_apu deliberately keeps each of its (up to 4) channels quiet
// internally -- VOL_INIT_MAX in minigb_apu.h is AUDIO_SAMPLE_MAX/8 -- so that
// summing several at once doesn't overflow. In practice most frames don't
// have all 4 channels active at once, so even 100% ("unity", no additional
// gain) plays back well under full scale. VOLUME_MAX_PERCENT allows real
// amplification above that, not just attenuation; the soft-knee limiter in
// render_frame keeps boosted peaks from wrapping or hard-clipping harshly.
// 800% drives typical loud passages deep into the limiter's knee -- i.e.
// deliberate compression: peaks are pinned near full scale while everything
// quieter comes up toward them, which is the only way to raise *average*
// loudness once peaks already span the full 3.3V swing.
constexpr uint16_t VOLUME_MAX_PERCENT = 800;
volatile uint16_t volume_percent = 0; // muted by default; raised from the
                                      // settings menu's VOLUME row

// DMA completion handler, on core1 (audio_output_core1_init installs it
// there). RAM-resident: a flash-resident handler would stall on XIP misses
// and steal QSPI bandwidth from core0's ROM fetches.
//
// If the next frame's buffer is already published, chain straight into it --
// back-to-back samples, no gap. If it isn't (the pacing DREQ deliberately
// runs a hair fast -- see core1_init), go idle: the PWM simply holds the
// last duty level (no writes arrive), which stretches one sample by a few
// microseconds instead of clicking, and render_frame restarts us.
void __not_in_flash_func(audio_dma_irq_handler)() {
	dma_irqn_acknowledge_channel(1, AUDIO_DMA_CH);
	if (buf_pending) {
		buf_pending = false;
		play_buf_idx ^= 1;
		dma_channel_set_read_addr(AUDIO_DMA_CH, duty_buf[play_buf_idx], false);
		dma_channel_set_trans_count(AUDIO_DMA_CH, AUDIO_SAMPLES, true);
	} else {
		dma_idle = true;
	}
}

} // namespace

void audio_output_init() {
	gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

	// Max out the pad drive. The piezo is a capacitive load being switched
	// at the 61kHz carrier; the default 4mA drive (and slew limiting) can't
	// fully charge it each period, which droops the effective swing and
	// with it the acoustic output. 12mA + fast slew delivers the fullest
	// rail-to-rail swing the pad can manage. (If the board buffers the
	// piezo behind a transistor this is simply a no-op -- the schematic
	// isn't public, so set it either way; it costs nothing.)
	gpio_set_drive_strength(AUDIO_PIN, GPIO_DRIVE_STRENGTH_12MA);
	gpio_set_slew_rate(AUDIO_PIN, GPIO_SLEW_RATE_FAST);

	uint slice = pwm_gpio_to_slice_num(AUDIO_PIN);
	pwm_config cfg = pwm_get_default_config();
	pwm_config_set_clkdiv(&cfg, 1.0f);
	pwm_config_set_wrap(&cfg, PWM_WRAP);
	pwm_init(slice, &cfg, true);
	pwm_set_gpio_level(AUDIO_PIN, PWM_WRAP / 2); // centered = silence
}

void audio_output_core1_init() {
	// Hardware-paced playback: a DMA channel, paced by a DMA DREQ timer at
	// the sample rate, writes duty words straight into the PWM slice's CC
	// register. This replaces the old 30us repeating alarm on core1, which
	// had two audible defects: (a) 30us != the true 30.52us sample period,
	// so playback ran ~1.7% fast and drained each frame's buffer ~300us
	// early -- the PWM then flat-lined until the next frame, a guaranteed
	// ~60Hz buzz -- and (b) every sample was delivered by a software timer
	// IRQ that core1's scanline rendering could delay, adding jitter (FM
	// grit) on top. DMA pacing is sample-exact and jitter-free, and also
	// removes ~549 IRQs/frame of core1 overhead.
	//
	// Rate choice: production is AUDIO_SAMPLES per paced frame -- 548
	// samples per frame_period_us (16742us authentic = ~32732Hz -- not the
	// nominal 32768Hz, minigb_apu's integer truncation of samples-per-vsync).
	// Consumption is paced a touch above production; see retune_pace_timer()
	// for the rationale and audio_output_set_frame_period() for how the rate
	// follows the emulator's actual pace.
	pace_timer = dma_claim_unused_timer(true);
	retune_pace_timer();

	dma_channel_claim(AUDIO_DMA_CH);
	dma_channel_config cfg = dma_channel_get_default_config(AUDIO_DMA_CH);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg, true);
	channel_config_set_write_increment(&cfg, false);
	channel_config_set_dreq(&cfg, dma_get_timer_dreq((uint)pace_timer));
	dma_channel_configure(AUDIO_DMA_CH, &cfg,
			      &pwm_hw->slice[pwm_gpio_to_slice_num(AUDIO_PIN)].cc,
			      nullptr, AUDIO_SAMPLES, false);

	// The completion IRQ must be taken on core1 (render_frame relies on
	// IRQ-disable as its critical section against the handler, which only
	// works same-core), hence DMA_IRQ_1 -- the screen driver already owns
	// DMA_IRQ_0 on core0. irq_set_exclusive_handler binds to the calling
	// core's vector table, which is why this whole function must run on
	// core1.
	dma_channel_set_irq1_enabled(AUDIO_DMA_CH, true);
	irq_set_exclusive_handler(DMA_IRQ_1, audio_dma_irq_handler);
	irq_set_enabled(DMA_IRQ_1, true);
}

void __not_in_flash_func(audio_output_render_frame)(struct minigb_apu_ctx *ctx) {
	minigb_apu_audio_callback(ctx, stage_buf);

	// Claim the fill slot atomically. If a previous frame is still pending
	// (production ran ahead -- e.g. core0 catching up after a stall), we are
	// about to overwrite that same slot, so revoke it first: otherwise the
	// completion IRQ could chain into the buffer mid-write and play a torn
	// frame. Revoked means the DMA goes idle at completion instead and our
	// publish below restarts it -- the older frame is dropped, which is the
	// right call when production runs ahead.
	uint32_t save = save_and_disable_interrupts();
	buf_pending = false;
	uint8_t fill_idx = play_buf_idx ^ 1;
	restore_interrupts(save);

	uint32_t *out = duty_buf[fill_idx];
	uint16_t vol = volume_percent;
	int32_t x1 = hpf_x1, y1 = hpf_y1;

	for (unsigned i = 0; i < AUDIO_SAMPLES; i++) {
		// Mono downmix.
		int32_t x = ((int32_t)stage_buf[i * 2] + stage_buf[i * 2 + 1]) / 2;

		// One-pole high-pass, ~200Hz (a = 246/256 at ~32.7kHz). The piezo
		// radiates essentially nothing below a few hundred Hz, but GB bass
		// lines and channel-gating DC steps still swing the signal hard --
		// wasting headroom the audible mids then lose to the limiter.
		// Filtering the dead band out first lets the same volume setting
		// play noticeably louder and cleaner.
		int32_t y = ((y1 + x - x1) * 246) >> 8;
		x1 = x;
		y1 = y;

		int32_t s = y * vol / 100;

		// Soft-knee limiter: progressively halve the overshoot in stages
		// instead of slamming into a hard clip. Above ~100% volume loud
		// frames legitimately exceed int16 range; the knee turns what would
		// be harsh flat-topping into ordinary-sounding compression, so the
		// volume can be pushed much further before it turns nasty. The
		// cascaded stages give slopes of 1/2, 1/4, 1/8, then 1/32 for
		// successive overshoot bands -- at 800% even worst-case all-channel
		// peaks (~229k in) compress to ~37k, so the waveform rounds off
		// toward full scale and only its topmost sliver hits the final
		// clamp.
		int32_t mag = s < 0 ? -s : s;
		if (mag > 20000) mag = 20000 + ((mag - 20000) >> 1);
		if (mag > 26000) mag = 26000 + ((mag - 26000) >> 1);
		if (mag > 30000) mag = 30000 + ((mag - 30000) >> 1);
		if (mag > 32000) mag = 32000 + ((mag - 32000) >> 2);
		if (mag > 32767) mag = 32767;
		s = s < 0 ? -mag : mag;

		// -32768..32767 -> 0..4095 duty, pre-shifted into the CC register's
		// channel-B half so the DMA can copy words verbatim.
		out[i] = (((uint32_t)(s + 32768)) >> 4) << 16;
	}
	hpf_x1 = x1;
	hpf_y1 = y1;

	// Publish. Same-core race with the DMA IRQ handler: without the guard,
	// the handler could fire between our dma_idle check and the kick and
	// double-start the channel.
	save = save_and_disable_interrupts();
	if (dma_idle) {
		dma_idle = false;
		play_buf_idx = fill_idx;
		dma_channel_set_read_addr(AUDIO_DMA_CH, out, false);
		dma_channel_set_trans_count(AUDIO_DMA_CH, AUDIO_SAMPLES, true);
	} else {
		// Still draining the previous buffer -- the handler will chain into
		// this one at completion.
		buf_pending = true;
	}
	restore_interrupts(save);
}

void audio_output_set_frame_period(uint32_t frame_us) {
	if (frame_us == 0)
		return;
	frame_period_us = frame_us;
	// Before core1_init has claimed the timer this just stores the period
	// (core1_init retunes from it); afterwards it reprograms the DREQ timer
	// live -- a single aligned register write, safe from either core while
	// playback runs.
	retune_pace_timer();
}

void audio_output_set_volume(uint16_t percent) {
	volume_percent = percent > VOLUME_MAX_PERCENT ? VOLUME_MAX_PERCENT : percent;
}

uint16_t audio_output_get_volume() {
	return volume_percent;
}

void audio_output_play_boot_jingle() {
	uint32_t vol = volume_percent;
	if (vol == 0)
		return;
	// A full-swing square at unity is already the loudest signal the piezo
	// can reproduce, so the >100% loudness-compression territory (meant
	// for the APU's quiet mix) is clamped off rather than limited here.
	if (vol > 100)
		vol = 100;

	constexpr uint32_t RATE = 32768;
	constexpr int32_t FULL = 2032; // full DAC swing around the center duty

	// Based on GBC boot (see the header): NR12=$F3 steps the 4-bit envelope
	// down every 3/64s -- exactly 1536 samples at this rate -- and a full
	// 15..0 ring-down takes ~703ms.
	constexpr uint32_t ENV_STEP = RATE * 3 / 64;
	// Q16 phase increments for the two notes: 131072/125 Hz ($783) and
	// 131072/63 Hz ($7C1); inc = freq * 65536 / RATE = 2^18 / divider.
	constexpr uint32_t NOTE1_INC = (1u << 18) / 125;
	constexpr uint32_t NOTE2_INC = (1u << 18) / 63;
	// Note 2 triggers two scroll-counter ticks (4 GB frames, ~67ms) after
	// note 1, restarting the envelope at 15.
	constexpr uint32_t NOTE2_AT = RATE * 67 / 1000;
	constexpr uint32_t LEN = NOTE2_AT + 15 * ENV_STEP; // note 2 rings to 0

	uint32_t phase = 0, inc = NOTE1_INC;
	int32_t env = 15;
	uint32_t env_count = 0;

	uint64_t t0 = time_us_64();
	for (uint32_t i = 0; i < LEN; i++) {
		uint64_t due = t0 + (uint64_t)i * 1000000u / RATE;
		while (time_us_64() < due)
			tight_loop_contents();
		if (i == NOTE2_AT) { // retrigger: new freq, envelope back to 15
			inc = NOTE2_INC;
			env = 15;
			env_count = 0;
		}
		if (++env_count >= ENV_STEP) {
			env_count = 0;
			if (env > 0)
				env--;
		}
		// 50% duty square (NR11=$80) scaled by the envelope level.
		int32_t s = FULL * env / 15 * (int32_t)vol / 100;
		if (!(phase & 0x8000))
			s = -s;
		pwm_set_gpio_level(AUDIO_PIN, (uint16_t)(PWM_WRAP / 2 + 1 + s));
		phase += inc;
	}
	pwm_set_gpio_level(AUDIO_PIN, PWM_WRAP / 2);
}

namespace {

// --- PicoCrystal startup chime ------------------------------------------
//
// Unlike the GB jingle this one plays *underneath* the boot animation, so it
// cannot busy-wait: a splash frame takes far longer than one sample period,
// and pumping the synth from the animation loop would drop out on every
// frame. It runs from a hardware-alarm IRQ instead, one sample per firing,
// leaving core0 free to keep drawing between them.
//
// 16384Hz rather than the APU path's 32768: the highest partner here is
// ~2.1kHz, so Nyquist is met with room to spare, and halving the rate halves
// an interrupt that now competes with the splash's rendering. A raw
// hardware alarm, not an alarm pool -- pool callbacks fire on the core that
// *created* the pool rather than the one that armed them, which has bitten
// this project before; hardware_alarm_set_callback binds to the caller.
constexpr uint32_t CHIME_RATE = 16384;
constexpr uint32_t CHIME_LEN = CHIME_RATE * 800 / 1000;
constexpr int32_t CHIME_FULL = 2032; // full DAC swing around the center duty

// Rising C6-G6-C7 arpeggio. Phase is Q32, so a Hz-to-increment conversion is
// just freq * (2^32 / CHIME_RATE) == freq << 18.
constexpr uint32_t CHIME_VOICES = 3;
const uint32_t CHIME_FREQ_HZ[CHIME_VOICES] = { 1046, 1568, 2093 };
const uint32_t CHIME_ONSET[CHIME_VOICES] = {
	0, CHIME_RATE * 60 / 1000, CHIME_RATE * 120 / 1000,
};

// Per-voice envelope: a short linear attack (a hard start on a bell tone
// clicks through the piezo) into an exponential decay applied as
// amp -= amp >> CHIME_DECAY_SHIFT, i.e. a time constant of 2^12 samples
// (~250ms). The last voice in still gets ~2.7 time constants before the end,
// which is quiet but not silent -- hence CHIME_FADE.
constexpr int32_t CHIME_AMP_ONE = 1 << 15; // envelope is Q15
constexpr uint32_t CHIME_ATTACK = CHIME_RATE * 5 / 1000;
constexpr uint32_t CHIME_DECAY_SHIFT = 12;
// Forced fade over the final stretch so the tail reaches true zero rather
// than stepping there.
constexpr uint32_t CHIME_FADE = CHIME_RATE * 100 / 1000;

// Normalizing by CHIME_VOICES would assume all three peak in phase at once,
// which this arpeggio never does -- it left 43% of the DAC swing unused on a
// speaker with none to spare. CHIME_NORM is the actual worst-case |acc| over
// the whole 800ms instead, measured by rendering this same integer loop on
// the host, so the loudest sample lands just under full scale. Retune it if
// the notes, onsets, envelope or sample rate change.
constexpr int32_t CHIME_NORM = 54600;

struct chime_voice_t {
	uint32_t ph, det_ph, inc, det_inc;
	int32_t amp;
};

// Chime state. Written by the start/stop calls and then owned by the alarm
// IRQ; chime_active gates the handler so a cancel racing a firing is inert.
chime_voice_t chime_v[CHIME_VOICES];
uint32_t chime_pos = 0;
uint32_t chime_vol = 0;
uint64_t chime_t0 = 0;
int chime_alarm = -1;
volatile bool chime_active = false;

// Parabolic sine: with x sweeping -2^15..2^15 across one cycle, 4x(1-|x|)
// traces a full period peaking at +-2^15, within a few percent of a real
// sine. Cheaper than a table lookup and, on a piezo driven through a 12-bit
// PWM DAC, indistinguishable from one.
int32_t __not_in_flash_func(chime_psin)(uint32_t phase) {
	int32_t x = (int32_t)(phase >> 16) - 32768;
	int32_t a = x < 0 ? -x : x;
	return (4 * x * (32768 - a)) >> 15;
}

// Synthesize sample chime_pos, push it to the DAC, and advance.
void __not_in_flash_func(chime_render_sample)() {
	int32_t acc = 0;
	for (uint32_t n = 0; n < CHIME_VOICES; n++) {
		if (chime_pos < CHIME_ONSET[n])
			continue;
		chime_voice_t &v = chime_v[n];
		uint32_t age = chime_pos - CHIME_ONSET[n];
		if (age < CHIME_ATTACK)
			v.amp = (int32_t)(CHIME_AMP_ONE * age / CHIME_ATTACK);
		else
			v.amp -= v.amp >> CHIME_DECAY_SHIFT;
		// Voice and detuned partner, averaged so a voice at full
		// envelope still spans only +-2^15.
		int32_t osc = (chime_psin(v.ph) + chime_psin(v.det_ph)) / 2;
		acc += (osc * v.amp) >> 15;
		v.ph += v.inc;
		v.det_ph += v.det_inc;
	}

	// CHIME_DRIVE over-drives the normalized signal into the soft knee
	// below. Straight gain was not available: normalization already put
	// peaks at ~98% of the rail, so the only way left to get louder is to
	// raise the *average* level and let the knee round off the peaks. A
	// decaying bell has a high crest factor, so there is a lot to reclaim
	// -- the ring-down, well under the knee, gets the full 2x, while the
	// attack transient compresses instead of clipping.
	constexpr int32_t CHIME_DRIVE = 2;
	int32_t s = acc * CHIME_FULL / CHIME_NORM * CHIME_DRIVE;
	s = s * (int32_t)chime_vol / 100;
	uint32_t left = CHIME_LEN - chime_pos;
	if (left < CHIME_FADE)
		s = s * (int32_t)left / (int32_t)CHIME_FADE;

	// The APU mix's cascaded soft knee, with its thresholds scaled from
	// int16 range to this signal's CHIME_FULL rail. Slopes of 1/2, 1/4,
	// 1/8, 1/32 over successive overshoot bands: at CHIME_DRIVE 2 the
	// worst-case peak lands just inside the rail, so the final clamp never
	// actually fires and nothing flat-tops.
	int32_t mag = s < 0 ? -s : s;
	if (mag > 1240) mag = 1240 + ((mag - 1240) >> 1);
	if (mag > 1610) mag = 1610 + ((mag - 1610) >> 1);
	if (mag > 1860) mag = 1860 + ((mag - 1860) >> 1);
	if (mag > 1985) mag = 1985 + ((mag - 1985) >> 2);
	if (mag > CHIME_FULL) mag = CHIME_FULL;
	s = s < 0 ? -mag : mag;

	pwm_set_gpio_level(AUDIO_PIN, (uint16_t)(PWM_WRAP / 2 + 1 + s));
	chime_pos++;
}

void __not_in_flash_func(chime_alarm_irq)(uint alarm_num) {
	(void)alarm_num;
	if (!chime_active)
		return;
	uint64_t due;
	do {
		chime_render_sample();
		if (chime_pos >= CHIME_LEN) {
			// Rung out. Leave the alarm claimed -- the stop call
			// releases it, so ownership has exactly one owner.
			chime_active = false;
			pwm_set_gpio_level(AUDIO_PIN, PWM_WRAP / 2);
			return;
		}
		due = chime_t0 + (uint64_t)chime_pos * 1000000u / CHIME_RATE;
		// set_target returns true when the target is already in the
		// past: a slot was missed (something held IRQs off, e.g. a
		// screen flip). Render it immediately and try the next rather
		// than rearming late, so the notes keep their pitch instead of
		// stretching -- the overwritten DAC write is harmless.
	} while (hardware_alarm_set_target((uint)chime_alarm, from_us_since_boot(due)));
}

} // namespace

void audio_output_start_startup_chime() {
	if (chime_alarm >= 0)
		return; // already running
	uint32_t vol = volume_percent;
	if (vol == 0)
		return;
	// Same reasoning as the GB jingle: the >100% range compensates for the
	// APU's deliberately quiet mix and has nothing to give a signal that is
	// already mixed to full scale here.
	if (vol > 100)
		vol = 100;
	chime_vol = vol;

	for (uint32_t i = 0; i < CHIME_VOICES; i++) {
		chime_v[i].ph = chime_v[i].det_ph = 0;
		chime_v[i].inc = CHIME_FREQ_HZ[i] << 18;
		// The detuned partner, ~0.6% sharp: 1046Hz beats against its
		// twin at ~6Hz, the higher voices proportionally faster.
		chime_v[i].det_inc = chime_v[i].inc + chime_v[i].inc / 170;
		chime_v[i].amp = 0;
	}
	chime_pos = 0;
	chime_t0 = time_us_64();

	chime_alarm = (int)hardware_alarm_claim_unused(true);
	hardware_alarm_set_callback((uint)chime_alarm, chime_alarm_irq);
	chime_active = true;
	hardware_alarm_set_target((uint)chime_alarm,
				  from_us_since_boot(chime_t0 + 1000000u / CHIME_RATE));
}

void audio_output_stop_startup_chime() {
	if (chime_alarm < 0)
		return;
	// Clear the gate before cancelling: a firing that slips through in
	// between then returns without touching the DAC or rearming.
	chime_active = false;
	hardware_alarm_cancel((uint)chime_alarm);
	hardware_alarm_set_callback((uint)chime_alarm, nullptr);
	hardware_alarm_unclaim((uint)chime_alarm);
	chime_alarm = -1;
	pwm_set_gpio_level(AUDIO_PIN, PWM_WRAP / 2);
}

void audio_output_mute() {
	// Called every frame while muted (render_frame isn't). Any in-flight DMA
	// drains within a frame and goes idle -- no new buffers get published --
	// after which this write sticks and the speaker sits at a clean center
	// level instead of whatever it last played.
	pwm_set_gpio_level(AUDIO_PIN, PWM_WRAP / 2);
}

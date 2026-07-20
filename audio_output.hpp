#pragma once
#include "core/minigb_apu/minigb_apu.h"

// Drives GBC audio out through the PicoSystem's piezo speaker by bypassing
// the SDK's voice()/play() single-tone ADSR synth entirely (it can't play
// arbitrary PCM) and instead treating the AUDIO GPIO's PWM as a crude DAC:
// a fixed-frequency PWM carrier has its duty cycle rewritten at the audio
// sample rate from a double buffer. See the plan's Architecture > Audio
// section for why.
//
// Playback is hardware-paced: a DMA channel, driven by a DMA DREQ timer at
// the exact sample rate, streams duty-cycle words into the PWM compare
// register. This replaced the original 30us repeating-alarm-per-sample
// design, whose ~1.7% rate error (30us vs the true 30.52us period) drained
// the buffer early every frame (a constant ~60Hz buzz as the output
// flat-lined) and whose per-sample software IRQs both jittered audibly and
// cost core1 ~549 interrupts per frame. See audio_output_core1_init() in
// audio_output.cpp for the full rate-matching rationale.
//
// Everything expensive runs on core1: PSG synthesis (minigb_apu's
// audio_callback), the mono downmix + high-pass + soft-limit + duty
// conversion (audio_output_render_frame), and the DMA completion IRQ.

// Configures the AUDIO pin's PWM as a fixed-carrier DAC. Call once from
// core0, after _init_hardware(). No longer launches core1 -- main.cpp owns
// the core1 worker loop and calls audio_output_core1_init() from it.
void audio_output_init();

// Claims the audio DMA channel + pacing timer and installs the completion
// IRQ handler. Must be called once from core1 itself: the handler and
// audio_output_render_frame() synchronize by disabling IRQs, which only
// excludes a handler bound to the same core's vector table.
void audio_output_core1_init();

// Synthesizes one GBC frame of PSG audio from the shared APU context,
// downmixes to mono, high-passes (piezo-dead lows would only waste
// headroom), applies volume with a soft-knee limiter, and publishes the
// duty-converted buffer to the DMA channel (chaining seamlessly if a
// transfer is still in flight). Called from core1's worker loop. The
// emulator on core0 keeps writing APU registers (via minigb_apu_audio_write)
// while this reads the same context; that lock-free sharing is deliberate
// and matches how other RP2040 GB ports drive minigb_apu cross-core -- a
// racing register write can at worst smear one frame's envelope/frequency
// by a sample, never corrupt memory (all context fields are fixed-size
// scalars/arrays).
void audio_output_render_frame(struct minigb_apu_ctx *ctx);

// Tells the playback side how often the emulator actually produces a frame
// of AUDIO_SAMPLES samples, in microseconds -- 16742 (the default) at
// authentic GBC pace, or the measured panel TE period while main.cpp's
// TE-paced vsync mode locks emulation to the panel clock. Playback DREQ
// pacing is retuned to sit just above that production rate so drift always
// resolves as a harmless microsecond-scale hold instead of a dropped audio
// frame. Callable from core0 at any time (before or after core1 starts).
void audio_output_set_frame_period(uint32_t frame_us);

// Master volume as a percentage of the raw mixed signal (0-800 -- see
// audio_output.cpp for why 100/"unity" is too quiet), applied during the
// downmix in audio_output_render_frame(); the soft-knee limiter there keeps
// boosted peaks musical instead of wrapping, and turns the upper half of the
// range into progressively heavier loudness compression. Written from core0,
// read by core1's downmix (single aligned 16-bit value, so cross-core reads
// are atomic).
void audio_output_set_volume(uint16_t percent);
uint16_t audio_output_get_volume();

// One sine component of the synthesized boot chime (see below): frequency,
// relative amplitude 0-255, when it starts sounding and how fast it swells
// in, and its ring-out rate -- the partial loses amp >> decay_shift per
// sample once its attack ends (an exponential decay with a time constant of
// 2^decay_shift samples at 32768Hz -- 13 is ~250ms, 14 ~500ms). Per-partial
// onsets let two overlapping bell strikes render as one seamless pass (the
// first strike keeps ringing under the second, like the recording), and
// per-partial decay reproduces how struck chimes shed their high partials
// faster than the fundamental.
struct chime_partial_t {
	uint16_t freq_hz;
	uint8_t amp;
	uint8_t decay_shift;
	uint16_t start_ms;
	uint16_t attack_ms;
};

// Synthesizes the whole chime -- a sum of delayed, swelling, decaying sine
// partials -- in one pass, streamed straight through the PWM DAC,
// busy-waiting on core0 until it finishes. A ~80ms master fade at the end
// takes whatever is still ringing cleanly to silence (no cut-off click).
// Only valid between audio_output_init() (the carrier must be configured)
// and audio_output_core1_init() (afterwards the DMA engine owns the CC
// register). Software-paced: at boot nothing else runs, so a time_us_64()
// spin per sample is jitter-free enough. Applies the master volume
// (unity-clamped -- the 0-800 loudness-compression range only makes sense
// against the APU's deliberately quiet mix, not a full-scale tone) and
// recenters the output to silence when done. No-op at volume 0.
void audio_output_play_chime_blocking(const chime_partial_t *partials,
				      uint32_t n_partials, uint32_t dur_ms);

// Forces the PWM output to its centered (silent) level. Intended for
// skipping synthesis entirely while volume is 0 (real per-sample PSG
// synthesis, not free) without leaving the speaker stuck at whatever
// non-zero level it last played -- call this every frame while muted,
// rather than just stopping submission; any in-flight DMA drains within a
// frame and the level then sticks.
void audio_output_mute();

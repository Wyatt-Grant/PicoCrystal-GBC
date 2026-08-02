// PicoCrystal main.cpp -- NOT the stock PicoSystem init()/update()/draw()
// model. This project runs a Game Boy Color emulator (Walnut-CGB) rather
// than a native game, so it needs a custom main() driving its own loop and
// direct low-level access to the display flip/vsync primitives.

#include <array>
#include <cstdint>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/platform.h" // for __not_in_flash_func()
#include "pico/multicore.h"
#include "hardware/dma.h"  // dma_hw->ch[].read_addr, chased by the vsync mode

#include "picosystem.hpp"

// Compile-time switch for the whole APU subsystem, distinct from muting.
// Volume 0 only skips the final PSG->PCM mix (minigb_apu_audio_callback);
// a game's sound engine still writes APU registers (frequency, envelope,
// duty cycle, ...) many times per frame, and each of those writes reaches
// minigb_apu_audio_write through __gb_read/__gb_write's `#if ENABLE_SOUND`
// branch. Setting this to 0 compiles that per-register traffic out too --
// useful for isolating what the APU as a whole costs on a perf run.
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
// walnut_cgb.h's hot read/write paths expand to. Both live above the
// walnut_cgb.h include because the macros are baked into
// __gb_read/__gb_read16/__gb_read32/__gb_write at their definitions inside
// that header.
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
// Core1 scanline-render offload: the full PPU line render (BG + window +
// sprites), not just the 1.5x scale/convert, runs on core1. __gb_draw_line's
// per-line work on core0 is just snapshotting the PPU registers + OAM into
// the line ring (WGB_LCD_LINE_OVERRIDE below).
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
// _io/_lio (the state behind button()/pressed(), driven here by _poll_io() --
// see init_button_input() in hardware.cpp for how those are fed).
// ---------------------------------------------------------------------
namespace picosystem {
	static color_t _fb[240 * 240] __attribute__((aligned(4)));
	static buffer_t _screen_storage = { .w = 240, .h = 240, .data = _fb, .alloc = false };
	buffer_t *SCREEN = &_screen_storage;
	uint32_t _io = 0, _lio = 0;
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

// Tear-free display toggle (settings menu VSYNC row, persisted).
//
// OFF (default): the stock path. _flip() fires straight from the main loop,
// racing both the panel scan-out and core1's scanline writes -- zero overhead,
// but shear is visible on scrolls. The game runs at its authentic 59.73Hz off
// the wall-clock pacer.
//
// ON: normally fully TE-LOCKED (g_te_pace; the panel is driven at ~60Hz by
// hardware.cpp's FRCTRL2 setting, so the lock costs no game speed). Every
// frame arms _flip_armed and the pacer then WAITS for the ST7789 TE
// interrupt to consume the arm and start the flip DMA (at STE tear scanline
// 240, where the beam enters the off-glass dead zone -- the ~4.5ms head
// start keeps the GRAM write ahead of the beam, so no panel-level tear).
// The wait is what makes the lock airtight against buffer-level tear too:
// the next frame's rows only start landing once the flip DMA is already
// streaming, and core1's per-row read_addr chase (render_line_job) keeps the
// writer behind it -- a flip can never stream a half-written frame. Cost:
// the chase briefly paces core1's stores to the DMA's ~19.65 rows/ms.
// Emulation runs at the panel's measured rate (audio pacing retunes to
// match -- tempo, not pitch); the settings row shows the rate.
//
// Arming without waiting is not enough: a pending arm can fire mid-write and
// stream a mixed buffer, which is what "still tears with vsync on" looks
// like. The wait is the load-bearing part.
//
// volatile: written by core0 (settings menu), read by core1's store path.
static volatile bool g_vsync = false;

// Authentic GBC frame period: 70224 cycles / 4.194304MHz.
constexpr uint64_t GB_FRAME_US = 16742;

// Boot-time measurement of the panel's real TE period (its RC oscillator
// varies unit to unit around the nominal ~60Hz), and whether it passed the
// sanity gate (54-66Hz) that enables TE-locked vsync: with vsync ON, every
// frame arms a flip and emulation waits for the TE IRQ to consume it before
// continuing -- the game runs at exactly the panel's rate, every frame is
// presented, and the shared framebuffer is coherent at every flip (see the
// pacer). Game speed and music tempo (not pitch -- audio pacing retunes,
// see apply_audio_pacing) follow the panel's true rate; the settings VSYNC
// row displays it. When the measurement fails the gate, vsync ON falls back
// to the wall-clock pacer, which can still tear (pending arm consumed
// mid-write). Set once at boot, read-only after.
static uint32_t g_te_period_us = 0;
static bool g_te_pace = false;

// Audio playback pacing follows the emulator's actual frame period: the
// panel's measured TE period while TE-paced vsync is active, authentic pace
// otherwise. Keeping consumption tuned just above true production is what
// prevents both dropped audio frames (production ahead) and audible
// flat-line buzz (consumption far ahead) -- see audio_output.cpp.
static void apply_audio_pacing() {
#if ENABLE_SOUND
	audio_output_set_frame_period(
		g_vsync && g_te_pace ? g_te_period_us : (uint32_t)GB_FRAME_US);
#endif
}

// STATUS BAR mode (settings menu row, persisted). Picks what the in-game
// header band shows: the FPS counter, the battery "<n>%" text, both, or
// neither -- the battery icon itself always shows in every bar mode, its fill
// conveying the level. The percent choice also applies to the boot/settings
// menu headers. FULLSCREEN drops the in-game header entirely and centers the
// scaled GBC frame vertically (see g_game_offset_y); menus keep their band.
// FS BATTERY is FULLSCREEN plus a 2px-tall battery meter spanning the whole
// width of the top letterbox band (see draw_fs_battery_bar).
enum status_bar_mode_t : uint8_t {
	STATUS_FPS_PCT,
	STATUS_FPS,
	STATUS_PCT,
	STATUS_ICON,
	STATUS_FULLSCREEN,
	STATUS_FS_BATTERY,
	STATUS_MODE_COUNT,
};
static uint8_t g_status_bar = STATUS_FPS_PCT;
static inline bool status_show_fps() { return g_status_bar <= STATUS_FPS; }
static inline bool status_show_pct() {
	return g_status_bar == STATUS_FPS_PCT || g_status_bar == STATUS_PCT;
}
// True for every mode that drops the in-game header band and centers the game
// frame -- FULLSCREEN and FS BATTERY alike (the latter's 2px meter lives inside
// the letterbox the centering creates, not in a band of its own).
static inline bool status_fullscreen() { return g_status_bar >= STATUS_FULLSCREEN; }
static inline bool status_fs_battery() { return g_status_bar == STATUS_FS_BATTERY; }
// Height of that meter, in rows. The game frame is pushed down by the same
// amount in FS BATTERY (see apply_game_offset) so the bar sits above the
// canvas rather than over the top of it.
constexpr int32_t FS_BATT_BAR_H = 2;
// Menu (boot picker / settings) header percentage. FULLSCREEN only drops the
// *in-game* header, so it says nothing about what the menus should show -- the
// menus keep their band and their "<n>%", same as the FPS-only default. ICON
// is the one mode that deliberately hides the number everywhere.
static inline bool menu_show_pct() {
	return status_show_pct() || status_fullscreen();
}

// BOOT LAST GAME (settings menu row, persisted): when on, main() skips the
// boot ROM picker and boots straight into the last-played game; holding B
// during power-on forces the picker anyway. g_last_slot tracks the last
// game by its save_slot (stable across catalog reshuffles), 0xFF = none yet.
static bool g_boot_last = false;
static uint8_t g_last_slot = 0xFF;

// NOSTALGIC BOOT (settings menu row, persisted): when on, launching a game
// plays a Game Boy-style boot splash -- the PicoCrystal logo scrolls down to
// center screen and two chimes ring out before emulation starts: our own
// arpeggio under the animation, then the GB "ba-ding" once it settles. See
// nostalgic_boot_splash().
static bool g_nostalgic_boot = true;

// SAVE INTERVAL (settings menu row, persisted): the minimum spacing between
// flash commits of the save file -- the wear throttle described in
// save_storage.hpp. Index 0 is MANUAL (commit only on the in-game X+B chord);
// the rest are the seconds a commit is held off for, however continuously the
// game writes cart RAM. 3s is the historical fixed behaviour and stays the
// default.
static const uint16_t SAVE_INTERVAL_SECS[] = { 0, 3, 5, 10, 30, 60, 120 };
constexpr uint32_t SAVE_INTERVAL_COUNT =
	sizeof(SAVE_INTERVAL_SECS) / sizeof(SAVE_INTERVAL_SECS[0]);
constexpr uint8_t SAVE_INTERVAL_DEFAULT = 1; // 3 seconds
static uint8_t g_save_interval = SAVE_INTERVAL_DEFAULT;
static inline bool save_interval_manual() { return g_save_interval == 0; }

// Push g_save_interval down into the save state machine. Called on boot (after
// the stored settings load) and on every edit of the row.
static void apply_save_interval() {
	save_storage_set_interval_ms(SAVE_INTERVAL_SECS[g_save_interval] * 1000u);
}

// Shared LED brightness cap: ~45% at full backlight, scaled down with the
// screen brightness setting so a dimmed screen gets a matching dim LED.
// Used by every LED indicator (battery, RGB theme, flash-write blink).
static inline int led_brightness() {
	return 45 * g_brightness / 100;
}

// Poll PicoSystem's 8 buttons and pack them into the GBC joypad byte (active
// low, per JOYPAD_* in walnut_cgb.h). PicoSystem has no Start/Select of its
// own, so X->Start, Y->Select. Holding Y and X together opens
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
static inline constexpr color_t rgb565_to_color(uint16_t c) {
	uint8_t r4 = (c >> 12) & 0xF; // top 4 of 5 red bits
	uint8_t g4 = (c >> 7) & 0xF;  // top 4 of 6 green bits
	uint8_t b4 = (c >> 1) & 0xF;  // top 4 of 5 blue bits
	return r4 | (0xF << 4) | (b4 << 8) | (g4 << 12);
}

// COLOR FILTER (settings row): picture modes, each recoloring the game's
// palette on its way to the framebuffer. GBC, the original, approximates a real
// GBC panel's response instead of showing raw palette values -- a game's
// palette writes were authored for a reflective STN screen whose primaries
// bleed heavily into each other, and feeding those same values straight to this
// device's LCD gives colors far more saturated (and greens far purer) than the
// artists ever saw. The rest are frank effects: monochrome ramps, the classic
// DMG greens, a saturation boost. Read by core1's LUT rebuild; only changed
// from the settings menu while emulation is paused, so no synchronisation
// beyond the volatile is needed.
enum color_filter_t : uint8_t {
	CF_OFF = 0,   // raw palette values, converted straight through
	CF_GBC = 1,   // pinned: flash records predate the mode list and stored
		      // this byte as the old ON/OFF toggle, so 0 and 1 have to
		      // keep meaning OFF and GBC (device_settings_t::color_filter)
	CF_VIVID,
	CF_CANDY,
	CF_PASTEL,
	CF_MONO,
	CF_SEPIA,
	CF_NIGHT,
	CF_GREEN,
	CF_POCKET,
	CF_INVERT,
	CF_COUNT,
};
static volatile uint8_t g_color_filter = CF_OFF;

// Every mode runs the same two stages, so there is one conversion path and no
// per-mode branching anywhere that matters:
//
//   5-bit channel -> _cf_x5 -> 0..255 -> 3x3 matrix -> clamp -> curve -> 4 bits
//
// The matrix does everything cross-channel (the GBC bleed, desaturation toward
// luma, saturation boosts); the per-channel curves do everything else
// (wash-out, tints, inversion, posterizing). That split is what lets ten fairly
// different looks share one function.
//
// Invariant: every matrix row sums to exactly 256, so greys stay perfectly
// neutral. A matrix whose rows sum differently tints the greyscale, which is
// the usual failure mode of the hand-rolled GBC matrices floating around -- and
// here it would also break the two DMG modes, whose four shades are picked by
// the (necessarily neutral) luma the matrix produces. Tints belong in the
// curves.

// Luma weights in 1/256ths, summing to 256: the neutral row every monochrome
// mode uses, and the desaturation target of the saturation family below.
constexpr int32_t CF_LR = 77, CF_LG = 150, CF_LB = 29;

// The active pipeline, rebuilt by cf_build_tables() on a mode change:
//   _cf_x5   5-bit channel -> 0..255, i.e. what "v * 255 / 31" used to compute
//            inline. The M0+ has no divide instruction and GCC can't magic-
//            number its way around one without a long multiply, so each of
//            those three divides compiled to an __aeabi_idiv call -- together
//            they cost more than everything else in the function.
//   _cf_mat  the matrix, row-major, in 1/256ths. Signed, because a saturation
//            boost is exactly a matrix with negative off-diagonals. int32
//            rather than the int16 the values would fit in: Thumb-1 has no
//            immediate-offset form of LDRSH, so a halfword table costs an extra
//            MOVS per coefficient -- 9 wasted instructions to save 18 bytes.
//   _cf_out  post-matrix channel (0..255) -> the final 4-bit output nibble,
//            one curve per channel so a mode can tint.
// Plain .bss, so they sit in SRAM where core1 reads them without touching the
// QSPI bus. 836 bytes all told, which is the point: folding the matrix into the
// input lookups would drop the multiplies, but it costs ~1.2KB against the
// ~2.8KB of slack the RAM region has left, and the multiplies are worth tens of
// cycles per *changed palette entry* -- this runs once per entry on a palette
// rebuild, never per pixel.
static uint8_t _cf_x5[32];
static int32_t _cf_mat[9];
static uint8_t _cf_out[3][256];

// Branchless saturate to 0..255: three data-processing pairs, versus the
// compare-and-branch GCC emits for the ternary form (which costs a taken branch
// on the common in-range path, on a core with no branch prediction).
// `v & ~(v >> 31)` floors at 0; the same trick on 255 - v then ceilings at 255.
static inline int32_t cf_clamp8(int32_t v) {
	v &= ~(v >> 31);
	const int32_t t = 255 - v;
	return 255 - (t & ~(t >> 31));
}

// Wash-out. An accurate GBC emulation is noticeably *darker* than raw output,
// and this device's panel is already dim -- stacking the two would bury the
// shadows. So instead of the usual darkening gamma we lift: blend v amount/256
// of the way toward its mirrored-square curve (v -> 255 - (255-v)^2/255), which
// leaves black at black and white at white but pulls the midtones up, for the
// slightly faded, sun-bleached look of a real screen rather than a dim one.
// (255-v)^2/255 is approximated as (255-v)^2 >> 8; the 1/255-vs-1/256 error is
// under one part in 256, well inside the 4-bit output. GBC's 128 is bumped past
// what a straight "authentic" filter would do, per how dark this panel reads.
static int32_t cf_lift(int32_t v, int32_t amount) {
	const int32_t iv = 255 - v;
	return v + ((((255 - (iv * iv >> 8)) - v) * amount) >> 8);
}

// Saturation blend: s/256 of the identity plus the remainder of luma. s == 256
// is the identity (hue and saturation untouched), s == 0 collapses every row to
// luma (monochrome), and s > 256 pushes past the identity -- which is where the
// negative off-diagonals of a saturation boost come from. The off-diagonals are
// computed first and the diagonal takes the remainder, so a row lands on
// exactly 256 however the rounding falls.
static void cf_sat_matrix(int32_t s) {
	static const int32_t L[3] = { CF_LR, CF_LG, CF_LB };
	for (int32_t i = 0; i < 3; i++) {
		int32_t row = 0;
		for (int32_t j = 0; j < 3; j++) {
			if (i == j)
				continue;
			const int32_t v = ((256 - s) * L[j] + 128) >> 8;
			_cf_mat[i * 3 + j] = v;
			row += v;
		}
		_cf_mat[i * 3 + i] = 256 - row;
	}
}

// The four shades of a DMG-class panel, darkest first, as 8-bit RGB. Both
// posterizing modes quantize luma into four equal bands and emit these -- the
// real thing, detail loss included, rather than a smooth ramp between them.
static const uint8_t CF_RAMP_GREEN[4][3] = {
	{ 0x0F, 0x38, 0x0F }, { 0x30, 0x62, 0x30 },
	{ 0x8B, 0xAC, 0x0F }, { 0x9B, 0xBC, 0x0F },
};
static const uint8_t CF_RAMP_POCKET[4][3] = {
	{ 0x1F, 0x1F, 0x1F }, { 0x4D, 0x53, 0x3C },
	{ 0x8B, 0x95, 0x6D }, { 0xC4, 0xCF, 0xA1 },
};

static void cf_build_tables(uint8_t mode) {
	for (int32_t v = 0; v < 32; v++)
		_cf_x5[v] = (uint8_t)(v * 255 / 31);

	// Matrix. Everything but GBC is a saturation blend.
	switch (mode) {
	case CF_GBC: {
		// The cross-channel bleed, in 1/256ths. Each output channel is
		// mostly its own input plus a slice of the other two -- which is
		// what desaturates the picture the way the panel did. Blue keeps
		// the least of itself and picks up the most green: the GBC's blue
		// was the weakest, most cyan-shifted primary.
		static const int32_t GBC_BLEED[9] = {
			200,  40,  16,
			 32, 184,  40,
			 32,  48, 176,
		};
		memcpy(_cf_mat, GBC_BLEED, sizeof(_cf_mat));
		break;
	}
	case CF_CANDY:  cf_sat_matrix(448); break; // x1.75
	case CF_VIVID:  cf_sat_matrix(352); break; // x1.375
	case CF_NIGHT:  cf_sat_matrix(192); break; // x0.75
	case CF_PASTEL: cf_sat_matrix(141); break; // x0.55
	case CF_INVERT: cf_sat_matrix(256); break; // identity: colours untouched
	default:        cf_sat_matrix(0);   break; // monochrome: luma rows
	}

	// Curves. Cold path (a mode change, with emulation paused), so this is
	// written for legibility -- the switch is per table entry, not per pixel.
	for (int32_t v = 0; v < 256; v++) {
		int32_t c[3];
		switch (mode) {
		case CF_GBC:
			c[0] = c[1] = c[2] = cf_lift(v, 128);
			break;
		case CF_VIVID:
			// Gentle S-contrast about mid-grey (x9/8) -- the punch
			// half of "vivid"; the matrix does the colour half.
			c[0] = c[1] = c[2] = 128 + ((v - 128) * 9 >> 3);
			break;
		case CF_CANDY:
			// VIVID turned up: a much harder saturation boost in the
			// matrix, and here a lift (so it reads bright and sugary
			// rather than merely contrasty) followed by a x5/4 pivot
			// about 132. Both ends run past 0..255 on purpose -- the
			// clamp is what clips highlights to pure primaries, and
			// that clipping is most of the "pop".
			c[0] = c[1] = c[2] = 132 + ((cf_lift(v, 80) - 132) * 5 >> 2);
			break;
		case CF_PASTEL:
			// Heavily lifted: high-key, milky, low-contrast.
			c[0] = c[1] = c[2] = cf_lift(v, 190);
			break;
		case CF_SEPIA: {
			// Warm monochrome: per-channel gains on a lifted ramp.
			const int32_t l = cf_lift(v, 64);
			c[0] = l * 275 >> 8;
			c[1] = l * 238 >> 8;
			c[2] = l * 184 >> 8;
			break;
		}
		case CF_NIGHT: {
			// Blue cut hard, red and green nudged up: an amber wash
			// for playing in the dark. Keeps a trace of the original
			// colour (the matrix only desaturates to 75%).
			const int32_t l = cf_lift(v, 96);
			c[0] = l * 272 >> 8;
			c[1] = l * 230 >> 8;
			c[2] = l * 140 >> 8;
			break;
		}
		case CF_GREEN:
		case CF_POCKET: {
			const uint8_t (*ramp)[3] = mode == CF_GREEN ? CF_RAMP_GREEN
								   : CF_RAMP_POCKET;
			const int32_t shade = v >> 6; // four equal luma bands
			c[0] = ramp[shade][0];
			c[1] = ramp[shade][1];
			c[2] = ramp[shade][2];
			break;
		}
		case CF_INVERT:
			c[0] = c[1] = c[2] = 255 - v;
			break;
		default: // CF_MONO -- and CF_OFF, which never reads these tables
			c[0] = c[1] = c[2] = v;
			break;
		}
		for (int32_t i = 0; i < 3; i++)
			_cf_out[i][v] = (uint8_t)(cf_clamp8(c[i]) >> 4);
	}
}

// RGB565 -> RGBA4444 with the filter applied. Not marked inline: it is called
// once per *changed* palette entry on a rebuild (see the delta loop in
// render_line_job), never per pixel, so the hot path is untouched -- but it is
// called from core1, hence RAM-resident to keep it off the QSPI bus.
static color_t __not_in_flash_func(rgb565_to_color_filtered)(uint16_t c) {
	// Take all three channels at 5 bits (green's 6th bit is below the 4-bit
	// output's precision anyway) and work in 0..255.
	const int32_t r = _cf_x5[(c >> 11) & 0x1F];
	const int32_t g = _cf_x5[(c >> 6)  & 0x1F];
	const int32_t b = _cf_x5[c         & 0x1F];

	// Matrix, then the curve and the drop to 4 bits in a single lookup. The
	// clamp only ever bites for a saturation boost (VIVID, CANDY), whose
	// negative off-diagonals can push a channel outside 0..255; every other
	// matrix has non-negative coefficients and rows summing to 256, so there
	// it is dead weight the compiler still has to carry.
	const uint32_t nr = _cf_out[0][cf_clamp8(
		(r * _cf_mat[0] + g * _cf_mat[1] + b * _cf_mat[2]) >> 8)];
	const uint32_t ng = _cf_out[1][cf_clamp8(
		(r * _cf_mat[3] + g * _cf_mat[4] + b * _cf_mat[5]) >> 8)];
	const uint32_t nb = _cf_out[2][cf_clamp8(
		(r * _cf_mat[6] + g * _cf_mat[7] + b * _cf_mat[8]) >> 8)];

	return (color_t)(nr | (0xF << 4) | (nb << 8) | (ng << 12));
}

static inline color_t rgb565_to_color_maybe_filtered(uint16_t c) {
	return g_color_filter != CF_OFF ? rgb565_to_color_filtered(c)
					: rgb565_to_color(c);
}

// Non-CGB (DMG) titles get a fixed four-shade grey ramp rather than palette
// indices, so it needs the same treatment -- rebuilt by apply_color_filter()
// instead of per palette write, since nothing else ever changes it.
static const uint16_t DMG_GREY565[4] = { 0xFFFF, 0xAD55, 0x52AA, 0x0000 };
static color_t _grey_lut[4] = {
	rgb565_to_color(0xFFFF), rgb565_to_color(0xAD55),
	rgb565_to_color(0x52AA), rgb565_to_color(0x0000)
};

// Forces the next CGB palette rebuild to convert all 64 entries instead of
// just the ones whose RGB565 changed (see the delta loop in render_line_job).
// Starts set so the first rebuild after boot is a full one -- the shadow it
// diffs against is only meaningful once something has filled it -- and is set
// again on a filter toggle, where every entry's *colour* changes although none
// of the source values do. Written only with emulation paused, cleared by
// core1 inside the rebuild.
static volatile bool _pal_lut_stale = true;

// Select a filter mode. The DMG ramp is rebuilt here; the CGB palette LUT is
// rebuilt by core1 the usual way, by marking the emulator's palette dirty so
// the next line job carries a snapshot. Call only with emulation paused (the
// settings menu, or before the run loop starts) -- core1 must not be mid-job.
static void apply_color_filter(uint8_t mode) {
	// The only route to a filtered conversion runs through here, so this is
	// the one place the tables have to be ready by. Rebuilding them for
	// CF_OFF too is a few thousand cycles once, and saves having a second
	// state to reason about.
	cf_build_tables(mode);
	g_color_filter = mode;
	for (int32_t i = 0; i < 4; i++)
		_grey_lut[i] = rgb565_to_color_maybe_filtered(DMG_GREY565[i]);
	_pal_lut_stale = true;
	gb.cgb.fixPaletteDirty = 1;
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

// Where the game frame actually lands: OFFSET_Y (under the header band)
// normally, or centered (12px letterbox top and bottom) in the FULLSCREEN
// status-bar mode. FS BATTERY shifts that down by the meter's height so the
// bar gets clear rows of its own (14px above, 10px below) instead of the
// canvas's first rows sitting directly under it. Read by core1's scanline
// writer; only changed while emulation is paused (settings menu open / before
// the run loop starts), when core1's line callback can't be running and the
// line ring is drained.
static volatile int32_t g_game_offset_y = OFFSET_Y;
static void apply_game_offset() {
	if (!status_fullscreen()) {
		g_game_offset_y = OFFSET_Y;
		return;
	}
	g_game_offset_y = (240 - SCALED_H) / 2 +
			  (status_fs_battery() ? FS_BATT_BAR_H : 0);
}

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

// The RGB565 each _pal_lut entry was converted from, so a rebuild only has to
// convert the entries that actually changed. Core1-private (only the rebuild
// below touches it), and kept exactly in step with _pal_lut. The dirty flag
// says *something* in the palette moved; in practice one or two entries did,
// while a rebuild converted all 64. That gap is what made the COLOR FILTER
// expensive: its conversion is several times the cost of the plain one, and a
// scene doing per-scanline palette work paid for it 64 entries at a time,
// stalling core0 on the VRAM fence behind core1's rebuild.
static uint16_t _pal_src[0x40];

// ---------------------------------------------------------------------
// Core1 scanline-render offload: the line ring.
//
// __gb_draw_line on core0 only snapshots the PPU state (registers, palettes,
// OAM -- see struct wgb_scanline_state in walnut_cgb.h) into this ring via
// wgb_lcd_enqueue(); core1 runs the full BG/window/sprite render
// (__gb_render_scanline, reading VRAM live) followed by the 1.5x scale.
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
	uint8_t oam[0xA0];          // OAM snapshot; the render must not see live OAM
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
		// Convert only what moved (see _pal_src) -- the compare is a
		// couple of cycles per entry against a conversion that is an
		// order of magnitude more, and it is rare for a palette write to
		// touch more than one or two of the 64. `all` covers the cases
		// where the shadow can't be trusted: the first rebuild after
		// boot, and a filter change, which changes every output colour
		// without changing a single input value.
		const bool all = _pal_lut_stale;
		_pal_lut_stale = false;
		// Branch hoisted out of the loop: the filter can't change
		// mid-rebuild (it only moves with emulation paused).
		if (g_color_filter != CF_OFF) {
			for (int32_t i = 0; i < 0x40; i++) {
				const uint16_t c = job->palette[i];
				if (!all && c == _pal_src[i])
					continue;
				_pal_src[i] = c;
				_pal_lut[i] = rgb565_to_color_filtered(c);
			}
		} else {
			for (int32_t i = 0; i < 0x40; i++) {
				const uint16_t c = job->palette[i];
				if (!all && c == _pal_src[i])
					continue;
				_pal_src[i] = c;
				_pal_lut[i] = rgb565_to_color(c);
			}
		}
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
	// address. In steady state it never spins: the DMA streams ~19.65
	// rows/ms while the emulator writes ~14 rows/ms at authentic pace, and
	// the top letterbox (24 rows; 12 in FULLSCREEN) gives the reader a
	// head start -- the spin only
	// engages when emulation bursts ahead within a frame, and the main
	// loop's arming policy keeps flips away from catch-up sprints.
	if (g_vsync) {
		const uint32_t last_row = (uint32_t)(g_game_offset_y + 3 * k) + (odd ? 2u : 0u);
		const uintptr_t chase_end = (uintptr_t)SCREEN->data +
			(uintptr_t)(last_row + 1) * SCALED_W * sizeof(color_t);
		while (_in_flip && dma_hw->ch[dma_channel].read_addr < chase_end)
			tight_loop_contents();
	}

	color_t *dst = SCREEN->p(OFFSET_X, g_game_offset_y + 3 * k + (odd ? 2 : 0));

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
		// Non-CGB (DMG) title: the emulator emits 2-bit shades rather
		// than fixPalette indices, so map them to a fixed grey ramp
		// (_grey_lut, which apply_color_filter() keeps in step).
		for (int32_t s = 0, d = 0; s < LCD_WIDTH; s += 2, d += 3) {
			color_t c0 = _grey_lut[_c1_pixels[s] & 3];
			color_t c1 = _grey_lut[_c1_pixels[s + 1] & 3];
			dst[d]     = c0;
			dst[d + 1] = blend_avg(c0, c1);
			dst[d + 2] = c1;
		}
	}

	if (odd) {
		// Row 3k+1: smoothed average of the stored even row and this odd row.
		color_t *mid = SCREEN->p(OFFSET_X, g_game_offset_y + 3 * k + 1);
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
	// Up/down triangles for the clock editor's "adjust" hint, mirrored about
	// the same two rows so the pair reads as one "^v" unit.
	{0b000, 0b000, 0b010, 0b111, 0b000}, // ^  (42)
	{0b000, 0b000, 0b111, 0b010, 0b000}, // v  (43)
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
		case '^': return 42;
		case 'v': return 43;
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
enum icon_t : uint32_t { ICON_CART, ICON_SLIDERS, ICON_SUN, ICON_SPEAKER, ICON_CLOCK, ICON_SCREEN, ICON_NOTE, ICON_STATUSBAR, ICON_SWATCHES, ICON_CONTRAST, ICON_DISK, ICON_DROPLET };

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
	{ 0b0001111,  // ICON_NOTE -- eighth note: the boot chime (NOSTALGIC BOOT
	  0b0001111,  // row). The bolt it replaces read as "fast/power", which is
	  0b0001100,  // the opposite of what the setting does -- it *adds* the
	  0b0001000,  // Game Boy logo scroll and jingle before a game starts.
	  0b1111000,
	  0b1111000,
	  0b1110000 },
	{ 0b1111111,  // ICON_STATUSBAR -- screen with a solid band across the top
	  0b1111111,  // (STATUS BAR row). Was a vertical battery, which named only
	  0b1111111,  // one of the five modes; the band is the row's actual subject.
	  0b0000000,  // The band is 3 rows (6px at 2x) against the hollow body
	  0b1000001,  // below, so it reads as a filled header rather than as the
	  0b1000001,  // thick top edge of a plain rectangle.
	  0b1111111 },
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
	{ 0b1111111,  // ICON_DISK -- floppy disk (SAVE INTERVAL row): solid body
	  0b1100011,  // with the shutter cut out of the top and the label cut out
	  0b1100011,  // of the bottom. The old version drew both as thin outlined
	  0b1111111,  // slots, which at 2x muddled into a striped block.
	  0b1000001,
	  0b1000001,
	  0b1111111 },
	{ 0b0001000,  // ICON_DROPLET -- ink drop (COLOR FILTER row): a tint being
	  0b0011100,  // applied. Solid rather than outlined, like ICON_DISK -- a
	  0b0011100,  // 7x7 outline shrinks to a couple of lit pixels per row and
	  0b0111110,  // muddles at 2x.
	  0b1111111,
	  0b1111111,
	  0b0111110 },
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

// Per-game boot-menu art: an 8x8 RGBA4444 tile out of assets/icons.png (see
// gen_rom_data.py), drawn at 2x like the 7x7 glyphs above so it occupies a
// 16x16 box. The tiles carry their own colors, so unlike draw_icon() this
// takes no tint -- the artwork is the same on the selected row and in either
// appearance mode. Transparent source pixels pack to 0 and are skipped, so
// the row highlight shows through around the art.
static void draw_rom_icon(int32_t x, int32_t y, const uint16_t *icon) {
	for (int32_t ry = 0; ry < 8; ry++)
		for (int32_t rx = 0; rx < 8; rx++)
			if (icon[ry * 8 + rx])
				fill_rect(x + rx * 2, y + ry * 2, 2, 2, icon[ry * 8 + rx]);
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

// Ordered by hue so < > walks the wheel rather than jumping around it: greens
// -> yellows -> oranges/browns -> reds/pinks -> purples -> blues -> cyans, with
// the low-saturation neutrals last. Every accent has to stay legible as *text*
// on both the dark and the light card, so nothing here goes very dark -- the
// low-value flavors (cocoa, taro) are pitched as milky tints instead -- and
// nothing goes near-white either, or it vanishes on the light card (vanilla is
// already at that edge, so frost is pitched a few steps below it).
constexpr ui_theme_t UI_THEMES[] = {
	{ "MINT",      status_rgb( 4, 13,  8) }, // the classic green
	{ "KIWI",      status_rgb( 6, 13,  4) }, // true green, between mint and matcha
	{ "MATCHA",    status_rgb( 9, 13,  4) }, // yellow-green
	{ "LEMON",     status_rgb(14, 13,  3) },
	{ "HONEY",     status_rgb(15, 11,  3) }, // amber, the gap between lemon and peach
	{ "PEACH",     status_rgb(15,  9,  4) },
	{ "COCOA",     status_rgb(12,  8,  6) }, // desaturated orange = milk chocolate
	{ "CHERRY",    status_rgb(15,  4,  5) }, // true red; berry sits on its pink side
	{ "BERRY",     status_rgb(15,  4,  7) },
	{ "BUBBLEGUM", status_rgb(15,  7, 11) },
	{ "COTTON",    status_rgb(15, 10, 13) }, // candy floss: bubblegum, milkier
	{ "PLUM",      status_rgb(13,  5, 14) }, // magenta, between cotton and taro
	{ "TARO",      status_rgb(12,  9, 15) }, // pale violet
	{ "GRAPE",     status_rgb( 9,  5, 15) },
	{ "SLUSHIE",   status_rgb( 5,  7, 15) }, // indigo, between grape and blueberry
	{ "BLUEBERRY", status_rgb( 4, 10, 15) },
	{ "ICING",     status_rgb( 9, 13, 15) }, // pale sky blue
	{ "LAGOON",    status_rgb( 4, 13, 15) }, // bright cyan, between icing and soda
	{ "SODA",      status_rgb( 4, 14, 13) }, // teal, the one hue nothing else covered
	{ "VANILLA",   status_rgb(14, 13, 10) },
	{ "FROST",     status_rgb(10, 12, 14) }, // cool neutral to vanilla's warm one

};
constexpr uint32_t THEME_COUNT = sizeof(UI_THEMES) / sizeof(UI_THEMES[0]);

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
	g_theme = (uint8_t)(idx < THEME_COUNT ? idx : 0);
	UI_ACCENT = UI_THEMES[g_theme].accent;
	UI_ROW_SEL = g_dark_mode ? theme_pill(UI_ACCENT) : light_pill(UI_ACCENT);
}

// Neutral background/text ramp, swapped as a set by apply_mode() (see below).
// Initialized to the dark-mode values so the UI is correct before the stored
// mode is applied at boot; light mode inverts them (light card, dark text).
static color_t UI_CARD    = status_rgb(1, 1, 1);  // panel body
static color_t UI_HEADER  = status_rgb(2, 2, 2);  // header band
static color_t UI_TRACK   = status_rgb(3, 3, 3);  // meter tracks, header rule
static color_t UI_VALUE   = status_rgb(6, 6, 6);   // unselected value text (currently unused)
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

// Charge-state color, shared by the battery icon's fill and the power LED
// (led_show_battery() below) so the two always agree at a glance -- a fixed
// green -> amber -> red ramp, independent of the UI accent theme so the charge
// state always reads the same way.
//
// The ramp is continuous in the charge level rather than a few coarse bands:
// green at 100%, amber at the 50% pivot, red at empty, linearly interpolated
// per channel in between. RGBA4444's 4-bit channels are what actually limit
// the resolution -- the endpoints below are chosen so the moving channel
// walks ~10 values per half (red 4->14 climbing 100..50%, green 11->2 falling
// 50..0%), i.e. ~19 distinct colors across the range, well past the ~10 the
// eye can tell apart on a 17x13 icon.
constexpr color_t BATT_GREEN = status_rgb(4, 13, 8);  // 100%, the mint full-charge green
constexpr color_t BATT_AMBER = status_rgb(14, 11, 1); // 50% pivot
constexpr color_t BATT_RED   = status_rgb(14, 2, 1);  // empty

// Per-channel lerp on the packed 4-bit channels: t/n of the way from a to b,
// rounded. n is 50 (half the charge range) for both halves of the ramp.
constexpr uint32_t batt_lerp4(uint32_t a, uint32_t b, uint32_t t, uint32_t n) {
	return (a * (n - t) + b * t + n / 2) / n;
}
constexpr color_t batt_mix(color_t a, color_t b, uint32_t t, uint32_t n) {
	return status_rgb(batt_lerp4(a & 0xF, b & 0xF, t, n),
			  batt_lerp4((a >> 12) & 0xF, (b >> 12) & 0xF, t, n),
			  batt_lerp4((a >> 8) & 0xF, (b >> 8) & 0xF, t, n));
}
constexpr color_t battery_color(uint32_t batt) {
	return batt >= 100 ? BATT_GREEN
	     : batt >= 50  ? batt_mix(BATT_AMBER, BATT_GREEN, batt - 50, 50)
			   : batt_mix(BATT_RED, BATT_AMBER, batt, 50);
}

// The LED reports battery charge at the shared led_brightness() cap so it
// glows rather than glares, unpacking battery_color()'s 4-bit channels onto
// the LED's 0-100 per-channel scale. Shared by the in-game run loop and the
// menus (boot ROM picker, settings).
//
// FS BATTERY is the exception: that mode already shows the charge as the
// on-screen meter, so the LED stays dark and is left free for the
// flash-commit blink (which overrides this everywhere anyway).
static void led_show_battery(int level) {
	if (status_fs_battery()) {
		led(0, 0, 0);
		return;
	}
	if (level < 0)   level = 0;
	if (level > 100) level = 100;
	const color_t c = battery_color((uint32_t)level);
	const int brightness = led_brightness();
	led((c & 0xF) * brightness / 15,
	    ((c >> 12) & 0xF) * brightness / 15,
	    ((c >> 8) & 0xF) * brightness / 15);
}

// Battery icon: a 17x13 rounded body outline plus a 3x5 terminal nub (20px
// total), its interior filled proportionally to the charge level in the
// matching battery_color().
constexpr int32_t BATT_ICON_W = 20;

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
	color_t fill = battery_color(batt);
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
// drops the "<n>%" text and keeps the icon (STATUS BAR modes without PCT --
// the icon's fill still shows the level at a glance).
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
// the cost is negligible and it stays off the per-frame budget. The STATUS BAR
// setting (g_status_bar) picks the text shown: the FPS readout (an empty title
// skips the label/number), the battery percentage, both, or neither -- the
// icon itself always shows. In FULLSCREEN mode the run loop never calls this
// (there is no header band); the guard below is just belt-and-braces.
static void draw_status_bar(uint32_t fps, uint32_t batt) {
	if (status_fullscreen())
		return;
	draw_header_band(status_show_fps() ? "FPS" : "", batt, STATUS_RULE,
			 status_show_pct(), 1);
	if (status_show_fps()) {
		char buf[12];
		int32_t n = status_fmt_uint(buf, fps);
		buf[n] = '\0';
		status_text(UI_MARGIN + 3 * 12 + 4, HDR_TOP, buf, STATUS_WHITE);
	}
}

// FS BATTERY's meter: the top 2 rows of the screen become one screen-wide
// battery gauge, filled from the left in the same battery_color() ramp the
// icon uses. It sits in the 14px letterbox above the game frame (the 12px
// FULLSCREEN centering leaves, plus the 2 rows apply_game_offset() adds back
// for the bar), so like the header band it can't collide with core1's
// scanline writes. The unfilled remainder uses the fixed dark STATUS_RULE
// rather than the theme's UI_TRACK -- the backdrop here is the black
// letterbox, and light mode's near-white track would read as a stray bright
// bar across the top of the screen.
static void draw_fs_battery_bar(uint32_t batt) {
	if (batt > 100)
		batt = 100;
	int32_t fw = (int32_t)(batt * SCREEN->w + 50) / 100;
	fill_rect(0, 0, fw, FS_BATT_BAR_H, battery_color(batt));
	fill_rect(fw, 0, SCREEN->w - fw, FS_BATT_BAR_H, STATUS_RULE);
}

// ---------------------------------------------------------------------
// Menus: boot-time ROM picker + settings screen -- full-screen dark panels
// under the shared header band, with dim control hints along the bottom.
// Both are modal: each owns a tiny poll/draw loop (mirroring the
// same _poll_io() sampling the real run loop uses for pressed()) and
// only returns when the user leaves. Opened in-game, the settings menu
// therefore pauses emulation by construction: core0 simply isn't calling
// gb_run_frame_dualfetch() while it's open.
// ---------------------------------------------------------------------

// Fills the whole screen with the panel color and draws the header band with
// a green accent rule (the in-game FPS overlay draws its own grey-ruled band
// directly via draw_header_band()). Menu headers follow the STATUS BAR mode's
// percent choice just like the in-game header, except in FULLSCREEN -- there
// the menus keep both their band and the percentage (see menu_show_pct());
// only the in-game header disappears.
static void draw_menu_frame(const char *title, uint32_t batt) {
	fill_rect(0, OFFSET_Y, SCREEN->w, SCREEN->h - OFFSET_Y, UI_CARD);
	draw_header_band(title, batt, UI_ACCENT, menu_show_pct());
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
			led_show_battery(b);
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
// Seconds are staged like the rest but deliberately NOT persisted with them:
// adding a field to device_settings_t invalidates every stored record (CRC
// over the whole payload), which is a poor trade for second-precision on a
// seed that only ever applies to a save with no RTC record of its own.
// A boot with no persisted RTC record therefore starts at :00.
static uint8_t g_rtc_sec = 0;  // 0..59

static uint64_t g_rtc_last_us = 0; // wall-clock baseline for rtc_host_tick()

static void rtc_apply_to_gb() {
	gb.rtc_real.reg.sec = g_rtc_sec;
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

// Before a game boots there is no MBC3 clock to tick (gb_init() hasn't run),
// so the staged g_rtc_* would sit frozen at whatever flash restored for as
// long as the boot menu's settings/clock screens are open. Advance them off
// the same wall clock rtc_host_tick() uses, so the readout ticks there too --
// and so a clock set at the boot menu is still right when a game finally
// starts. Whole seconds only; the remainder stays in the baseline.
static uint64_t g_rtc_stage_last_us = 0;

// Restart the staged second: once at boot to anchor the baseline, and on a
// pre-boot edit, mirroring rtc_apply_to_gb()'s reset of the sub-second
// remainder. Deliberately NOT called when a menu opens -- the baseline runs
// free from boot, so time that passes with no menu on screen is caught up the
// next time one is.
static void rtc_stage_tick_reset() { g_rtc_stage_last_us = time_us_64(); }

static void rtc_stage_tick() {
	uint64_t now = time_us_64();
	uint64_t elapsed = now - g_rtc_stage_last_us;
	if (elapsed < 1000000)
		return;
	uint32_t secs = (uint32_t)(elapsed / 1000000);
	g_rtc_stage_last_us += (uint64_t)secs * 1000000;
	// Roll the whole staged value as one second-of-week counter rather than
	// carrying field by field -- the weekday is the day counter's only
	// meaning here, so wrapping at 7 days is the correct wrap.
	constexpr uint32_t WEEK = 7 * 24 * 60 * 60;
	uint32_t t = (((uint32_t)g_rtc_dow * 24 + g_rtc_hour) * 60 + g_rtc_min) * 60 +
		     g_rtc_sec;
	t = (t + secs) % WEEK;
	g_rtc_sec = (uint8_t)(t % 60);
	g_rtc_min = (uint8_t)((t / 60) % 60);
	g_rtc_hour = (uint8_t)((t / 3600) % 24);
	g_rtc_dow = (uint8_t)(t / 86400);
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
	SET_ROW_COLOR, // COLOR FILTER: GBC-panel color emulation
	SET_ROW_STATUS,
	SET_ROW_SAVE,
	SET_ROW_BOOT_LAST,
	SET_ROW_NOSTALGIC,
	SET_ROW_THEME,
	SET_ROW_MODE, // APPEARANCE: dark/light
	SET_ROW_CLOCK, // A opens the segmented editor (draw_clock_menu)
	SET_ROW_COUNT,
};

// Fields of the clock editor -- its own screen (clock_menu), reached with A
// from SET_ROW_CLOCK. Separate from settings_row_t because the two screens
// have separate selections; the shared row metrics below don't apply here.
enum clock_field_t : uint32_t {
	CLK_ROW_DOW,
	CLK_ROW_HOUR,
	CLK_ROW_MIN,
	CLK_ROW_SEC,
	CLK_ROW_COUNT,
};

static const char *const RTC_DOW_NAMES[7] = {
	"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT",
};

// The segmented clock: DOW : HH : MM, 9 glyph cells at 4x (12x20 per glyph).
constexpr int32_t CLK_SCALE = 4;
constexpr int32_t CLK_ADV = 4 * CLK_SCALE;

// Every setting row is a single text line -- BRIGHT/VOLUME carry a compact
// inline meter bar on the same line rather than a full-width bar underneath --
// so all rows share one height. 17px is what the eleventh row (COLOR FILTER)
// cost: at 18px it would have landed at y=214 with its selection pill running
// to y=230, into draw_menu_hints()'s fixed y=226. At 17px the last row sits at
// y=204, glyphs ending at y=216 and the pill at y=220, still clear. The 10px
// glyphs leave a 7px gap, and the pill (20px tall, drawn from y-4) overlaps
// only the neighbouring pill's slot, never its text (which starts at y+19).
constexpr int32_t ROW_H = 17;

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
	// (TE-synchronised flips; see g_vsync's comment). When the boot TE
	// measurement passed the lock gate, the ON value shows the panel's
	// measured refresh rate ("60HZ") -- emulation locks 1:1 to it; a plain
	// "ON" means the panel measured outside 54-66Hz and vsync is running
	// the degraded wall-clock fallback.
	if (g_vsync && g_te_pace) {
		char hz[13]; // status_fmt_uint worst case (10 digits) + "HZ" + NUL
		int32_t n = status_fmt_uint(hz,
			(1000000u + g_te_period_us / 2) / g_te_period_us);
		hz[n] = 'H';
		hz[n + 1] = 'Z';
		hz[n + 2] = '\0';
		draw_value_row(SET_ROW_VSYNC, sel, ICON_SCREEN, "VSYNC", hz);
	} else {
		draw_toggle_row(SET_ROW_VSYNC, sel, ICON_SCREEN, "VSYNC", g_vsync);
	}
	// COLOR FILTER: picture modes (see color_filter_t) -- GBC is the
	// panel-response approximation, the rest range from a saturation boost
	// to the classic DMG greens. Costs nothing per frame whichever is
	// chosen: the whole effect is baked into the 64-entry palette LUT.
	static const char *const CF_NAMES[CF_COUNT] = {
		"OFF", "GBC", "VIVID", "CANDY", "PASTEL", "MONO",
		"SEPIA", "NIGHT", "GB GREEN", "POCKET", "INVERT",
	};
	draw_value_row(SET_ROW_COLOR, sel, ICON_DROPLET, "COLOR FILTER",
		       CF_NAMES[g_color_filter]);
	// STATUS BAR: what the in-game header shows -- FPS and/or battery % text
	// (the icon always shows), just the icon, FULLSCREEN (no header, game
	// frame centered) or FS BATTERY (fullscreen plus a 2px screen-wide
	// battery meter). The percent choice also drives the menu headers.
	static const char *const STATUS_MODE_NAMES[STATUS_MODE_COUNT] = {
		"FPS+PCT", "FPS", "PERCENT", "ICON", "FULLSCREEN", "FS BATTERY",
	};
	draw_value_row(SET_ROW_STATUS, sel, ICON_STATUSBAR, "STATUS BAR",
		       STATUS_MODE_NAMES[g_status_bar]);
	// SAVE INTERVAL: how often the save file may be committed to flash --
	// the wear throttle (see save_storage.hpp). MANUAL commits nothing on
	// its own; the X+B chord in-game does it instead.
	{
		char iv[6]; // "120S" + NUL
		const char *value = "MANUAL";
		if (!save_interval_manual()) {
			int32_t n = status_fmt_uint(iv, SAVE_INTERVAL_SECS[g_save_interval]);
			iv[n] = 'S';
			iv[n + 1] = '\0';
			value = iv;
		}
		draw_value_row(SET_ROW_SAVE, sel, ICON_DISK, "SAVE INTERVAL", value);
	}
	// BOOT LAST GAME: ON skips the boot ROM picker and auto-boots the
	// last-played game (hold B during power-on to get the picker back).
	draw_toggle_row(SET_ROW_BOOT_LAST, sel, ICON_CART, "BOOT LAST GAME",
			g_boot_last);
	// NOSTALGIC BOOT: Game Boy-style logo-scroll + chimes when a game launches.
	draw_toggle_row(SET_ROW_NOSTALGIC, sel, ICON_NOTE, "NOSTALGIC BOOT",
			g_nostalgic_boot);
	// THEME: the value is the accent's name; every accent-tinted element on
	// screen (including this value) recolors live as < > cycles it.
	draw_value_row(SET_ROW_THEME, sel, ICON_SWATCHES, "THEME",
		       UI_THEMES[g_theme].name);
	// APPEARANCE: dark/light -- swaps the neutral ramp live, so the whole
	// panel (and this row) recolors as < > toggles it.
	draw_value_row(SET_ROW_MODE, sel, ICON_CONTRAST, "APPEARANCE",
		       g_dark_mode ? "DARK" : "LIGHT");
	// CLOCK: a summary row ("MON 13:45") that opens the segmented editor on
	// A -- the big display was a fifth of the panel for a setting touched
	// about once, and the space it freed is what put ROW_H back to 18.
	// In-game the caller refreshes g_rtc_* from the live MBC3 clock each
	// frame, so this value ticks rather than showing the boot-time staging.
	{
		char clk[10]; // "DOW HH:MM" + NUL
		const char *dow = RTC_DOW_NAMES[g_rtc_dow];
		clk[0] = dow[0];
		clk[1] = dow[1];
		clk[2] = dow[2];
		clk[3] = ' ';
		status_fmt_uint_pad(clk + 4, g_rtc_hour, 2);
		clk[6] = ':';
		status_fmt_uint_pad(clk + 7, g_rtc_min, 2);
		clk[9] = '\0';
		draw_value_row(SET_ROW_CLOCK, sel, ICON_CLOCK, "CLOCK", clk);
	}

	// Two rows say something the row itself doesn't: CLOCK is the only one
	// that opens a screen rather than cycling in place, and MANUAL's save
	// chord is invisible otherwise.
	draw_menu_hints(sel == SET_ROW_CLOCK
			? "A EDIT   B BACK"
			: (sel == SET_ROW_SAVE && save_interval_manual()
			   ? "< > ADJUST   X+B SAVES"
			   : "< > ADJUST   B BACK"));
}

// Solid triangle, apex up or down, `size` rows tall (2*size-1 px wide at the
// base): the boot menu's scroll indicator at the default size, the clock
// editor's up/down adjust markers a notch bigger.
static void draw_scroll_arrow(int32_t x, int32_t y, bool down, color_t c,
			       int32_t size = 3) {
	for (int32_t k = 0; k < size; k++)
		fill_rect(x - k, down ? y - k : y + k, 1 + 2 * k, 1, c);
}

// ---- Analog dial (clock editor) -------------------------------------------
// sin(k * 6 degrees) * 256 for the 60 minute positions around a clock face.
// Everything on the dial -- ticks, both hands -- lands on one of those 60
// angles, so the whole thing is integer math: no <cmath> in a per-frame path
// on an FPU-less part, and no trig stub needed in tools/render_menus.cpp.
// Cosine is the same table a quarter turn along: cos(k) == SIN60[(k + 15) % 60].
static const int16_t SIN60[60] = {
	   0,   27,   53,   79,  104,  128,  150,  171,  190,  207,
	 222,  234,  243,  250,  255,  256,  255,  250,  243,  234,
	 222,  207,  190,  171,  150,  128,  104,   79,   53,   27,
	   0,  -27,  -53,  -79, -104, -128, -150, -171, -190, -207,
	-222, -234, -243, -250, -255, -256, -255, -250, -243, -234,
	-222, -207, -190, -171, -150, -128, -104,  -79,  -53,  -27,
};

// Point at dial angle `k` (0 = 12 o'clock, increasing clockwise), `r` from the
// centre. Screen y grows downward, hence the negated cosine.
static inline int32_t dial_x(int32_t cx, int32_t r, int32_t k) {
	return cx + r * SIN60[k % 60] / 256;
}
static inline int32_t dial_y(int32_t cy, int32_t r, int32_t k) {
	return cy - r * SIN60[(k + 15) % 60] / 256;
}

// Filled disc (r_in = 0) or annulus: every pixel whose squared distance from
// the centre falls in [r_in^2, r_out^2]. A per-pixel distance test over the
// bounding box -- ~13k iterations at the dial's radius, nothing next to the
// menu loop's 16ms frame, and it keeps the shape obviously correct.
static void fill_ring(int32_t cx, int32_t cy, int32_t r_out, int32_t r_in,
		       color_t c) {
	const int32_t o2 = r_out * r_out, i2 = r_in * r_in;
	for (int32_t dy = -r_out; dy <= r_out; dy++) {
		color_t *row = SCREEN->p(cx, cy + dy);
		for (int32_t dx = -r_out; dx <= r_out; dx++) {
			int32_t d2 = dx * dx + dy * dy;
			if (d2 <= o2 && d2 >= i2)
				row[dx] = c;
		}
	}
}

// Bresenham, 1px wide. Only ever called with both endpoints inside the dial,
// so like fill_rect it doesn't clip.
static void draw_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, color_t c) {
	int32_t dx = x1 - x0, dy = y1 - y0;
	int32_t sx = dx < 0 ? -1 : 1, sy = dy < 0 ? -1 : 1;
	dx = dx < 0 ? -dx : dx;
	dy = dy < 0 ? -dy : dy;
	int32_t err = dx - dy;
	while (true) {
		*SCREEN->p(x0, y0) = c;
		if (x0 == x1 && y0 == y1)
			return;
		int32_t e2 = 2 * err;
		if (e2 > -dy) { err -= dy; x0 += sx; }
		if (e2 <  dx) { err += dx; y0 += sy; }
	}
}

// A clock hand: `thick` parallel lines from the centre out to dial angle `k`.
// The copies are offset along the shallower axis (vertically for a mostly
// horizontal hand, horizontally otherwise), which is close enough to
// perpendicular at these lengths to keep the hand an even width all the way
// round, and centred so the hand grows about its own axis rather than
// drifting off the pivot.
static void draw_hand(int32_t cx, int32_t cy, int32_t k, int32_t len,
		       int32_t thick, color_t c) {
	int32_t x1 = dial_x(cx, len, k), y1 = dial_y(cy, len, k);
	bool wide = (x1 - cx) * (x1 - cx) >= (y1 - cy) * (y1 - cy);
	for (int32_t i = 0; i < thick; i++) {
		int32_t o = i - thick / 2;
		int32_t ox = wide ? 0 : o, oy = wide ? o : 0;
		draw_line(cx + ox, cy + oy, x1 + ox, y1 + oy, c);
	}
}

// The analog face at the top of the clock editor: a dial plate with hour ticks
// (the quarters longer and brighter), an AM/PM marker and the weekday in
// little windows above and below the pivot, and the hour/minute hands. The
// group currently being edited lights up here too -- adjusting HR swings an
// accent-colored hour hand, MIN the minute hand, DAY the weekday window -- so
// the dial doubles as the selection indicator rather than just decoration.
static void draw_clock_face(int32_t cx, int32_t cy, int32_t r, uint32_t field) {
	fill_ring(cx, cy, r - 3, 0, UI_HEADER);  // plate
	fill_ring(cx, cy, r, r - 2, STATUS_DIM); // bezel

	for (int32_t h = 0; h < 12; h++) {
		bool quarter = (h % 3) == 0;
		int32_t k = h * 5;
		int32_t outer = r - 6, inner = outer - (quarter ? 7 : 3);
		draw_hand(dial_x(cx, inner, k), dial_y(cy, inner, k), k,
			  outer - inner, quarter ? 2 : 1,
			  quarter ? STATUS_GREY : STATUS_DIM);
	}

	// AM/PM above the pivot, weekday below -- the two things the digits'
	// 24-hour "14" and "TUE" say, restated where a watch face would put them.
	status_text(cx - text_w(2) / 2, cy - 27, g_rtc_hour < 12 ? "AM" : "PM",
		    STATUS_DIM);
	status_text(cx - text_w(3) / 2, cy + 17, RTC_DOW_NAMES[g_rtc_dow],
		    field == CLK_ROW_DOW ? UI_ACCENT : STATUS_GREY);

	// Hands last, over the windows. The hour hand carries the minutes as a
	// fifth of an hour, so it sits between the ticks instead of snapping.
	// The seconds hand is thin and dim under the other two (a sweep hand,
	// not a reading), reaching nearly to the ticks.
	draw_hand(cx, cy, g_rtc_sec, r - 8, 1,
		  field == CLK_ROW_SEC ? UI_ACCENT : STATUS_GREY);
	draw_hand(cx, cy, (g_rtc_hour % 12) * 5 + g_rtc_min / 12, r - 26, 3,
		  field == CLK_ROW_HOUR ? UI_ACCENT : UI_BRIGHT);
	draw_hand(cx, cy, g_rtc_min, r - 12, 2,
		  field == CLK_ROW_MIN ? UI_ACCENT : UI_BRIGHT);
	fill_ring(cx, cy, 3, 0, UI_ACCENT); // pivot cap
	fill_ring(cx, cy, 1, 0, UI_HEADER);
}

// Selected-group highlight behind the big digits -- the settings list's
// two-rect pill shape (draw_row_highlight), sized to a group instead of a row.
static void draw_group_pill(int32_t x, int32_t y, int32_t w, int32_t h) {
	fill_rect(x + 1, y, w - 2, h, UI_ROW_SEL);
	fill_rect(x, y + 1, w, h - 2, UI_ROW_SEL);
}

// The clock editor: an analog face over DOW : HH : MM as a big segmented
// display, with seconds trailing it at half size the way a watch puts small
// seconds off to the side -- four groups at CLK_SCALE would need 15 cells
// (240px, the whole screen edge to edge), and seconds are the field you read
// rather than the one you read at a glance. A small caption sits above each
// group and up/down arrows flank the selected one. < > walk the groups,
// UP/DOWN adjust the selected value -- the arrows are drawn where the D-pad
// presses them, and the dial's matching hand (or its weekday window) goes
// accent as it moves. Its own screen since the settings list outgrew hosting
// it inline.
static void draw_clock_menu(uint32_t field, uint32_t batt) {
	draw_menu_frame("CLOCK", batt);

	char buf[12];
	status_fmt_uint_pad(buf, g_rtc_hour, 2);
	status_fmt_uint_pad(buf + 4, g_rtc_min, 2);
	status_fmt_uint_pad(buf + 8, g_rtc_sec, 2);

	// Face on top, readout under it. The stack below the dial is fixed:
	// captions at 146, up arrows at 162, the 20px digits at 175 (their pill
	// 171..199), down arrows ending at 207 -- clear of draw_menu_hints()'s
	// fixed y=226. That leaves the card's top 120px for the dial, which
	// centres at y=84 with r=52 and still clears the header rule at
	// OFFSET_Y - 1 by 8px.
	constexpr int32_t gy = 146;      // group captions
	constexpr int32_t dy = 175;      // big glyphs
	constexpr int32_t ARROW_UP = 162, ARROW_DN = 207;
	// Seconds: half scale, sitting on the big digits' baseline (dy + 20).
	constexpr int32_t SEC_SCALE = CLK_SCALE / 2;
	constexpr int32_t SEC_ADV = CLK_ADV / 2;
	constexpr int32_t SEC_Y = dy + 5 * (CLK_SCALE - SEC_SCALE);
	constexpr int32_t SEC_GAP = 6; // breathing room after the big MM

	draw_clock_face(SCREEN->w / 2, 84, 52, field);

	struct group_t {
		const char *text; // pre-rendered group text (weekday name / digits)
		const char *label;
		int32_t label_len;
		int32_t glyphs;
		int32_t scale;
		int32_t adv;
		int32_t y;
		uint32_t field;
	};
	const group_t grp[4] = {
		{ RTC_DOW_NAMES[g_rtc_dow], "DAY", 3, 3, CLK_SCALE, CLK_ADV, dy,
		  CLK_ROW_DOW },
		{ buf,     "HR",  2, 2, CLK_SCALE, CLK_ADV, dy,     CLK_ROW_HOUR },
		{ buf + 4, "MIN", 3, 2, CLK_SCALE, CLK_ADV, dy,     CLK_ROW_MIN  },
		{ buf + 8, "SEC", 3, 2, SEC_SCALE, SEC_ADV, SEC_Y,  CLK_ROW_SEC  },
	};

	// Total run: the 9 big cells (DOW : HH : MM), then the gap and the
	// seconds' 3 small cells (: SS), centred as one block.
	int32_t x = (SCREEN->w - (text_w(9, CLK_SCALE, CLK_ADV) + SEC_GAP +
				  text_w(3, SEC_SCALE, SEC_ADV))) / 2;
	for (int32_t i = 0; i < 4; i++) {
		bool is_sel = (grp[i].field == field);
		int32_t gw = text_w(grp[i].glyphs, grp[i].scale, grp[i].adv);
		int32_t gh = 5 * grp[i].scale;
		int32_t pad = grp[i].scale + 2; // pill inset, to scale
		if (is_sel) {
			draw_group_pill(x - pad, grp[i].y - 5, gw + 2 * pad,
					gh + 10);
			int32_t ax = x + gw / 2;
			draw_scroll_arrow(ax, ARROW_UP, false, UI_ACCENT, 4);
			draw_scroll_arrow(ax, ARROW_DN, true, UI_ACCENT, 4);
		}
		menu_text(x + (gw - text_w(grp[i].label_len)) / 2, gy, grp[i].label,
			  is_sel ? UI_ACCENT : STATUS_DIM, 2, STATUS_GLYPH_ADV);
		menu_text(x, grp[i].y, grp[i].text, is_sel ? UI_ACCENT : UI_BRIGHT,
			  grp[i].scale, grp[i].adv);
		x += grp[i].glyphs * grp[i].adv;
		if (i < 3) {
			// The separator before SEC is drawn at the smaller
			// scale, on the small group's baseline.
			int32_t ss = (i == 2) ? SEC_SCALE : CLK_SCALE;
			int32_t sa = (i == 2) ? SEC_ADV : CLK_ADV;
			if (i == 2)
				x += SEC_GAP;
			menu_text(x, (i == 2) ? SEC_Y : dy, ":", STATUS_DIM,
				  ss, sa);
			x += sa;
		}
	}

	draw_menu_hints("^v ADJUST  <> FIELD  B BACK");
}

// Boot ROM-select body: catalog rows (icon, name) plus the trailing
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

	draw_menu_frame("PICK A GAME", batt);

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
		// Games with art in assets/icons.png show it instead of the cart
		// glyph; the 16x16 tile sits a pixel higher than the 14x14 glyph so
		// both stay centered on the name. Slots with no tile (and the
		// SETTINGS row) keep the drawn icons.
		const uint16_t *art = i < ROM_COUNT ? rom_catalog[i].icon : nullptr;
		if (art) {
			draw_rom_icon(UI_MARGIN, y + 3, art);
		} else {
			draw_icon(UI_MARGIN, y + 4,
				  i < ROM_COUNT ? ICON_CART : ICON_SLIDERS,
				  is_sel ? UI_ACCENT : STATUS_DIM);
			// Per-ROM cart label tint (roms.json "color"), 0 = leave default.
			// The selected row stays fully theme-accent, so skip the overlay
			// there.
			if (i < ROM_COUNT && !is_sel && rom_catalog[i].label_color)
				draw_cart_label(UI_MARGIN, y + 4, rom_catalog[i].label_color);
		}

		const char *name = i < ROM_COUNT ? rom_catalog[i].name : "SETTINGS";
		int32_t name_x = UI_MARGIN + 22;
		int32_t name_max = UI_RIGHT - name_x;

		// Clip long titles to the full row width: the name's ink (text_w, no
		// trailing glyph gap) must fit in name_max. At the current metrics
		// that's 24 glyphs -- the same cap gen_rom_data.py applies to catalog
		// names -- so in practice nothing gets ".."-clipped anymore (the old
		// right-aligned size column used to cost ~4 name characters).
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

	// Scrollbar down the right edge, drawn only when the catalog is taller
	// than the window. Track spans the row area; the thumb's height is the
	// visible fraction and its offset the window position, so it doubles as
	// the "how much more is there" cue the old up/down arrows gave.
	if (total > VISIBLE_ROWS) {
		constexpr int32_t BAR_X = 235;   // right of the selection pill (ends at 233)
		constexpr int32_t BAR_W = 3;
		constexpr int32_t BAR_Y = OFFSET_Y + 12;             // first row's top
		constexpr int32_t BAR_H = VISIBLE_ROWS * ROW_H + 4;  // + the SETTINGS gap
		constexpr int32_t THUMB_MIN = 10;

		int32_t th = (int32_t)(BAR_H * VISIBLE_ROWS / total);
		if (th < THUMB_MIN)
			th = THUMB_MIN;
		// first ranges 0..total-VISIBLE_ROWS over the track's spare
		// travel. The ternary is only to keep the divisor non-zero for
		// the constant-folder when a build's catalog fits the window --
		// this branch doesn't run then.
		const uint32_t travel = total > VISIBLE_ROWS ? total - VISIBLE_ROWS : 1;
		int32_t ty = BAR_Y + (int32_t)((BAR_H - th) * first / travel);

		// Same track/fill pair the settings meters use, so the bar reads
		// as chrome rather than as a selected control -- and the fill
		// stays dark on light mode's near-white track.
		fill_rect(BAR_X, BAR_Y, BAR_W, BAR_H, UI_TRACK);
		fill_rect(BAR_X, ty, BAR_W, th, UI_FILL);
	}

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
		g_status_bar,
		g_theme,
		g_dark_mode,
		(uint8_t)(g_boot_last ? 1 : 0),
		g_last_slot,
		(uint8_t)(g_nostalgic_boot ? 1 : 0),
		g_save_interval,
		g_color_filter,
	};
	save_storage_settings_store(ds);
}

static void settings_step(uint32_t row, int32_t dir, bool in_game) {
	// CLOCK opens clock_menu on A and has nothing to cycle in place --
	// checked before the dirty flag so browsing onto it and pressing < >
	// can't schedule a flash write that changes nothing.
	if (row == SET_ROW_CLOCK)
		return;
	g_settings_edited = true;
	switch (row) {
	case SET_ROW_BRIGHT: adjust_brightness(dir); return;
#if ENABLE_SOUND
	case SET_ROW_VOLUME: adjust_volume(dir); return;
#endif
	case SET_ROW_VSYNC: // either direction toggles
		g_vsync = !g_vsync;
		apply_audio_pacing(); // TE-paced vsync shifts the production rate
		return;
	case SET_ROW_COLOR: // < > cycle OFF/GBC/VIVID/../INVERT, wrapping
		// Safe to rebuild the palette LUTs from here: the settings menu
		// only runs with emulation (and so core1's renderer) paused.
		apply_color_filter((uint8_t)((g_color_filter + CF_COUNT +
					      (uint32_t)dir) % CF_COUNT));
		return;
	case SET_ROW_STATUS: // < > cycle the five modes, wrapping
		g_status_bar = (uint8_t)((g_status_bar + STATUS_MODE_COUNT +
					  (uint32_t)dir) % STATUS_MODE_COUNT);
		apply_game_offset();
		// Kill the battery LED the instant FS BATTERY is selected rather
		// than waiting up to a battery-poll interval for it to go dark.
		// (Switching back out restores the gradient on the next poll.)
		if (status_fs_battery())
			led(0, 0, 0);
		return;
	case SET_ROW_SAVE: // < > cycle MANUAL/3S/../120S, wrapping
		g_save_interval = (uint8_t)((g_save_interval + SAVE_INTERVAL_COUNT +
					     (uint32_t)dir) % SAVE_INTERVAL_COUNT);
		apply_save_interval();
		return;
	case SET_ROW_BOOT_LAST: g_boot_last = !g_boot_last; return; // either direction toggles
	case SET_ROW_NOSTALGIC: g_nostalgic_boot = !g_nostalgic_boot; return; // either direction toggles
	case SET_ROW_THEME:
		apply_theme((g_theme + THEME_COUNT + (uint32_t)dir) % THEME_COUNT);
		return;
	case SET_ROW_MODE: apply_mode(!g_dark_mode); return; // either direction toggles
	}
}

// The clock editor's UP/DOWN, split out of settings_step because the fields live
// on their own screen. Marks both dirty flags: g_settings_edited persists the
// staged clock with the rest of the settings, g_rtc_edited additionally forces
// the autosave that keeps an in-game edit from losing to the older RTC record
// already on flash (see settings_menu's close path).
static void clock_step(uint32_t field, int32_t dir, bool in_game) {
	// RTC fields wrap so either direction reaches any value quickly.
	switch (field) {
	case CLK_ROW_DOW:  g_rtc_dow  = (uint8_t)((g_rtc_dow + 7 + dir) % 7);    break;
	case CLK_ROW_HOUR: g_rtc_hour = (uint8_t)((g_rtc_hour + 24 + dir) % 24); break;
	case CLK_ROW_MIN:  g_rtc_min  = (uint8_t)((g_rtc_min + 60 + dir) % 60);  break;
	case CLK_ROW_SEC:  g_rtc_sec  = (uint8_t)((g_rtc_sec + 60 + dir) % 60);  break;
	}
	g_settings_edited = true;
	g_rtc_edited = true;
	if (in_game)
		rtc_apply_to_gb();
	else
		rtc_stage_tick_reset(); // a set restarts the second here too
}

// Stage the live MBC3 clock into the g_rtc_* globals the menus display and
// edit, instead of the stale boot-time staging. The day counter ticks past 6
// as play spans midnights; fold it back to a weekday (the only meaning the
// games give it). Called every frame while a menu is open in-game, so the
// CLOCK row and the editor both tick with the game's clock.
static void rtc_stage_from_gb() {
	g_rtc_dow = (uint8_t)(((gb.rtc_real.reg.yday |
				((uint16_t)(gb.rtc_real.reg.high & 1) << 8))) % 7);
	g_rtc_hour = gb.rtc_real.reg.hour;
	g_rtc_min = gb.rtc_real.reg.min;
	g_rtc_sec = gb.rtc_real.reg.sec;
}

// One input sample, shared by both modal menu loops.
static void menu_read_input() {
	_poll_io();
}

// Per-frame housekeeping while a menu is open over a paused game. Without it
// the emulator's clock and autosave stall for as long as the menu is up, so
// every menu loop that can run in-game must call it.
static void menu_pause_tick() {
#if ENABLE_SOUND
	// Hold the speaker at a clean silent level while paused instead of
	// stuck at whatever it last played.
	audio_output_mute();
#endif
	// Wall time keeps flowing into the clock while the game is paused here
	// (real MBC3 behavior), and an in-menu autosave then commits a fresh
	// RTC snapshot.
	rtc_host_tick();
	// Keep autosave flushing -- the player may well open this right after
	// an in-game save.
	save_storage_poll(cart_ram, sizeof(cart_ram));
	rtc_stage_from_gb();
}

// Block until every action button is released. Used both on entry (the press
// that opened a screen is still held and would otherwise act on it) and on
// exit (so the closing press doesn't leak into whatever resumes underneath).
static void menu_drain_buttons() {
	do {
		menu_read_input();
		sleep(8);
	} while (button(picosystem::A) || button(picosystem::B) ||
		 button(picosystem::X) || button(picosystem::Y));
}

// An opposed button pair with hold-to-repeat (initial delay, then fast) so the
// wider ranges (MIN 0..59, HR 0..23) aren't dozens of individual taps. Returns
// the direction to step this frame, or 0.
static int32_t menu_repeat_dir(uint32_t neg, uint32_t pos, uint64_t &next_repeat) {
	uint64_t now = time_us_64();
	if (pressed(pos) || pressed(neg)) {
		next_repeat = now + 350000;
		return pressed(pos) ? +1 : -1;
	}
	if ((button(pos) || button(neg)) && now >= next_repeat) {
		next_repeat = now + 60000;
		return button(pos) ? +1 : -1;
	}
	return 0;
}

// The settings list's < > adjust.
static int32_t menu_adjust_dir(uint64_t &next_repeat) {
	return menu_repeat_dir(picosystem::LEFT, picosystem::RIGHT, next_repeat);
}

// The clock editor, opened with A from the settings list's CLOCK row. Edits
// the same g_rtc_* staging and sets the same dirty flags, so settings_menu's
// close path persists whatever happens here -- this screen writes nothing to
// flash itself.
static void clock_menu(bool in_game) {
	uint32_t field = CLK_ROW_HOUR; // the field most likely being fixed
	menu_battery_poll batt;
	uint64_t next_repeat = 0;

	// The A that opened this screen is still held; ignore input until it
	// (and anything else) is released.
	menu_drain_buttons();

	while (true) {
		menu_read_input();

		if (pressed(picosystem::B) ||
		    (button(picosystem::X) && button(picosystem::Y)))
			break;
		// < > walk the fields (they read left-to-right on screen),
		// UP/DOWN spin the selected value -- the transpose of the
		// settings list, but the one that matches a row of adjustable
		// segments.
		if (pressed(picosystem::LEFT))
			field = (field + CLK_ROW_COUNT - 1) % CLK_ROW_COUNT;
		if (pressed(picosystem::RIGHT))
			field = (field + 1) % CLK_ROW_COUNT;

		int32_t dir = menu_repeat_dir(picosystem::DOWN, picosystem::UP,
					      next_repeat);
		if (dir)
			clock_step(field, dir, in_game);

		if (in_game)
			menu_pause_tick();
		else
			rtc_stage_tick(); // no MBC3 clock yet -- tick the staging

		draw_clock_menu(field, batt.poll());
		_flip();
		sleep(16); // no timing requirements -- just input polling headroom
	}

	// The closing B is still held: drain it here or settings_menu reads the
	// same press as its own "back" and closes out from under this return.
	menu_drain_buttons();
}

// Modal settings screen, shared by the boot menu (its SETTINGS row, or Y+X)
// and in-game (Y+X chord, detected in the run loop).
static void settings_menu(bool in_game) {
	uint32_t sel = 0;
	menu_battery_poll batt;

	if (in_game)
		rtc_stage_from_gb();

	// The buttons that opened the menu (Y+X, or A on the boot menu's
	// SETTINGS row) are still held on the first iterations -- ignore input
	// until everything is released so opening can't immediately toggle back
	// out or activate a row.
	bool wait_release = true;
	uint64_t next_repeat = 0;

	while (true) {
		menu_read_input();

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
			// A opens the row's own screen -- only CLOCK has one.
			if (pressed(picosystem::A) && sel == SET_ROW_CLOCK) {
				clock_menu(in_game);
				next_repeat = 0;
				continue; // its drain already consumed this frame's input
			}

			int32_t dir = menu_adjust_dir(next_repeat);
			if (dir)
				settings_step(sel, dir, in_game);
		}

		if (in_game)
			menu_pause_tick();
		else
			rtc_stage_tick();

		draw_settings_menu(sel, batt.poll());
		_flip();
		sleep(16); // no timing requirements -- just enough for ~60fps input polling
	}

	// An in-game clock edit must reach flash even if the game never saves
	// again this session -- otherwise the older RTC record wins on next
	// boot and undoes the edit. One extra 32KB autosave cycle, only on an
	// actual clock edit. The request forces the commit past the SAVE
	// INTERVAL throttle -- with a long interval (or MANUAL) the edit would
	// otherwise sit unwritten until the next scheduled save, which in
	// MANUAL may never come.
	if (in_game && g_rtc_edited) {
		save_storage_mark_dirty();
		save_storage_request_save();
	}
	g_rtc_edited = false;

	// Persist the settings (brightness/volume/clock) if anything was
	// actually edited -- one synchronous ~50ms flash write, unnoticeable
	// here where nothing is animating. Skipped entirely on a look-only
	// visit so browsing the menu never wears flash.
	if (g_settings_edited) {
		g_settings_edited = false;
		store_device_settings();
	}

	// Drain the closing press so B (or the Y+X chord) doesn't leak into
	// whatever screen resumes underneath (game or boot menu).
	menu_drain_buttons();
}

// Boot-time ROM picker. Runs once in main(), before gb_init() -- nothing
// else is driving the display yet. The list is the ROM catalog plus a
// trailing SETTINGS row (set apart by a half-row gap).
static uint32_t rom_select() {
	uint32_t sel = 0;
	constexpr uint32_t MENU_ROWS = ROM_COUNT + 1; // + trailing SETTINGS row

	// Seed _io from the current GPIO level so the loop's first pressed()
	// poll below reflects real state instead of the zero-initialized
	// default (which would read as "every button just released"), and drop
	// any transitions queued by whatever ran before this picker.
	_reset_io();

	menu_battery_poll batt;

	while (true) {
		menu_read_input();

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

// ---------------------------------------------------------------------
// NOSTALGIC BOOT splash (settings toggle g_nostalgic_boot): a Game Boy
// Advance-style boot screen played after a ROM is picked and before
// core1/emulation starts. The PicoCrystal logo -- bold italic sans in the
// GBC "GAME BOY" lettering style -- flies in on a wave that runs from the
// bottom left to the right: each column of the logo swoops up from below,
// staggered so the arrival reads as a wavefront, and rides a travelling
// ripple that stretches and squashes it vertically, so the flat bitmap reads
// as a ribbon turning in 3D. Once it has landed a band of every UI accent
// (settings THEME row) sweeps left to right through the letters, each column
// running the whole wheel before settling on its own color -- the whole
// sequence tuned to ~2.5s, the same length as the descent-and-sweep it
// replaced. Two chimes ring out through the PWM DAC across it, both
// synthesized rather than played back as sampled PCM: our own arpeggio
// (audio_output_start_startup_chime) rings under the fly-in from the first
// frame, then the "ba-ding" based on GBC boot
// (audio_output_play_boot_jingle) once the colors settle.
//
// "By Wyatt" sits in small print at the bottom, in the spot the GBC boot
// screen gives its byline. Follows the APPEARANCE setting: black-on-white in
// light mode, white-on-black in dark (the letter colors read on either
// field).
//
// Bitmaps are 1-bpp MSB-first. The logo is stored as one layer per color, so
// each layer is a plain single-color pass and the per-column wave geometry is
// recomputed (cheaply, from an integer sine table) rather than buffered.
// ---------------------------------------------------------------------

constexpr int32_t BOOT_LOGO_W = 81, BOOT_LOGO_H = 15;
static const uint8_t boot_logo_b_bits[165] = { // "Pic" -- blue
	0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
	0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x77,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xee, 0x7c, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xce, 0xfe, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x1d, 0xcc, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x1d, 0xc0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x38, 0x1d, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x70, 0x3b, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x70, 0x3b, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x70, 0x39, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t boot_logo_g_bits[165] = { // "oC" -- green
	0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x71, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c,
	0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xcd, 0xc0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xcd, 0xc0, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xcd, 0xc0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x9b, 0x8c, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0xf9, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0xf0, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t boot_logo_m_bits[165] = { // "ry" -- magenta
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x76, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7e, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf1,
	0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe1, 0xcc,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe1, 0xcc, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc3, 0x98, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc3, 0xf8, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc1, 0xf8, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t boot_logo_r_bits[165] = { // "sta" -- red
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x7d, 0xf3, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xfd, 0xc7, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0xc3, 0x80, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0xf3, 0x87, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb,
	0x8f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x77, 0x1c,
	0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf7, 0xdf, 0xc0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe3, 0xcf, 0xc0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t boot_logo_y_bits[165] = { // "l" -- yellow, the Y-tail nod
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// The five color layers, left to right, and their settled GBC-style colors.
constexpr int32_t BOOT_LOGO_LAYERS = 5;
static const uint8_t *const boot_logo_layer_bits[BOOT_LOGO_LAYERS] = {
	boot_logo_b_bits, boot_logo_g_bits, boot_logo_m_bits,
	boot_logo_r_bits, boot_logo_y_bits,
};

constexpr int32_t BOOT_BYLINE_W = 66, BOOT_BYLINE_H = 15;
static const uint8_t boot_byline_bits[135] = {
	0xfc, 0x00, 0x00, 0xe1, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00,
	0xe1, 0xc0, 0x00, 0x03, 0x87, 0x00, 0xe7, 0x00, 0x00, 0xe1, 0xc0, 0x00,
	0x03, 0x87, 0x00, 0xe7, 0x00, 0x00, 0xe1, 0xc0, 0x00, 0x03, 0xe7, 0xc0,
	0xfe, 0x39, 0x80, 0xe1, 0xce, 0x63, 0xe3, 0xe7, 0xc0, 0xfe, 0x39, 0x80,
	0xed, 0xce, 0x67, 0xf3, 0x87, 0x00, 0xe7, 0x39, 0x80, 0xed, 0xce, 0x60,
	0x73, 0x87, 0x00, 0xe7, 0x39, 0x80, 0xed, 0xce, 0x63, 0xf3, 0x87, 0x00,
	0xe7, 0x39, 0x80, 0xed, 0xce, 0x67, 0xf3, 0x87, 0x00, 0xe7, 0x39, 0x80,
	0xed, 0xce, 0x67, 0x33, 0x87, 0x00, 0xfe, 0x3f, 0x80, 0xff, 0xcf, 0xe7,
	0xf3, 0xe7, 0xc0, 0xfc, 0x1f, 0x80, 0x73, 0x87, 0xe3, 0xf1, 0xe3, 0xc0,
	0x00, 0x01, 0x80, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x33, 0x00,
	0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x0f, 0x80,
	0x00, 0x00, 0x00,
};

// 1-bpp blitter for the splash bitmaps. Unlike fill_rect it clips vertically:
// the scrolling logo starts fully above the screen, and fill_rect's raw
// SCREEN->p() writes would land out of bounds there.
static void draw_bitmap_1bpp(int32_t x, int32_t y, int32_t w, int32_t h,
			     int32_t scale, const uint8_t *bits, color_t c) {
	int32_t stride = (w + 7) / 8;
	for (int32_t ry = 0; ry < h; ry++) {
		int32_t dy = y + ry * scale, dh = scale;
		if (dy < 0) {
			dh += dy;
			dy = 0;
		}
		if (dy + dh > SCREEN->h)
			dh = SCREEN->h - dy;
		if (dh <= 0)
			continue;
		const uint8_t *row = bits + ry * stride;
		for (int32_t rx = 0; rx < w; rx++)
			if (row[rx / 8] & (0x80 >> (rx & 7)))
				fill_rect(x + rx * scale, dy, scale, dh, c);
	}
}

// Clipped fill for the flying logo: columns swing off the left edge and below
// the bottom one on their way in, and fill_rect's raw SCREEN->p() writes would
// land out of bounds there.
static void fill_rect_clipped(int32_t x, int32_t y, int32_t w, int32_t h,
			      color_t c) {
	if (x < 0) {
		w += x;
		x = 0;
	}
	if (y < 0) {
		h += y;
		y = 0;
	}
	if (x + w > SCREEN->w)
		w = SCREEN->w - x;
	if (y + h > SCREEN->h)
		h = SCREEN->h - y;
	if (w > 0 && h > 0)
		fill_rect(x, y, w, h, c);
}

// Integer sine for the boot wave: `a` is the angle in 1/256ths of a turn,
// result is sin * 256. A quarter-turn table folded four ways -- no FPU here,
// and the splash asks for a few hundred of these per frame.
static int32_t boot_sin(int32_t a) {
	static const int16_t q[65] = {
		0, 6, 13, 19, 25, 31, 38, 44, 50, 56, 62, 68,
		74, 80, 86, 92, 98, 104, 109, 115, 121, 126, 132, 137,
		142, 147, 152, 157, 162, 167, 172, 177, 181, 185, 190, 194,
		198, 202, 206, 209, 213, 216, 220, 223, 226, 229, 231, 234,
		237, 239, 241, 243, 245, 247, 248, 250, 251, 252, 253, 254,
		255, 255, 256, 256, 256,
	};
	a &= 255;
	if (a < 64)
		return q[a];
	if (a < 128)
		return q[128 - a];
	if (a < 192)
		return -q[a - 128];
	return -q[256 - a];
}
static int32_t boot_cos(int32_t a) { return boot_sin(a + 64); }

// Per-channel mix of two RGBA4444 colors, `t` 0-256 from `a` to `b`. The
// splash gradient is the only place that needs it; the UI palette elsewhere is
// all fixed swatches.
static color_t boot_mix(color_t a, color_t b, int32_t t) {
	auto ch = [&](int32_t sh) {
		int32_t x = (a >> sh) & 0xF, y = (b >> sh) & 0xF;
		return (uint32_t)(x + ((y - x) * t >> 8)) & 0xF;
	};
	return status_rgb(ch(0), ch(12), ch(8));
}

static void nostalgic_boot_splash() {
	// Field and small print follow the APPEARANCE setting: black-on-white in
	// light mode (the GBC's own look), white-on-black in dark.
	const bool dark = g_dark_mode != 0;
	const color_t bg = dark ? status_rgb(0, 0, 0) : status_rgb(15, 15, 15);
	const color_t ink = dark ? status_rgb(15, 15, 15) : status_rgb(1, 1, 1);
	// The settled coloring: the vaporwave palette (#01cdfe #05ffa1 #b967ff
	// #ff71ce #fffb96, quantized to 4-bit channels) as evenly spaced stops
	// of a left-to-right gradient rather than one flat color per layer --
	// the letters ramp through it instead of changing color in five hard
	// steps. All five stops are bright enough to read on either field.
	constexpr int32_t GRAD_STOPS = 5;
	const color_t grad_stops[GRAD_STOPS] = {
		status_rgb(0, 12, 15), // cyan   #01cdfe
		status_rgb(0, 15, 9),  // mint   #05ffa1
		status_rgb(11, 6, 15), // purple #b967ff
		status_rgb(15, 7, 12), // pink   #ff71ce
		status_rgb(15, 15, 9), // yellow #fffb96
	};
	// Gradient color for source column `rx`: the stop pair it falls between,
	// mixed per 4-bit channel.
	auto grad_at = [&](int32_t rx) {
		int32_t s = rx * (GRAD_STOPS - 1) * 256 / (BOOT_LOGO_W - 1);
		int32_t i = s >> 8, f = s & 255;
		if (i >= GRAD_STOPS - 1)
			return grad_stops[GRAD_STOPS - 1];
		return boot_mix(grad_stops[i], grad_stops[i + 1], f);
	};
	constexpr int32_t SCALE = 2;
	constexpr int32_t logo_x = (240 - BOOT_LOGO_W * SCALE) / 2;
	constexpr int32_t logo_land_y = (240 - BOOT_LOGO_H * SCALE) / 2;
	constexpr int32_t logo_mid_y = logo_land_y + BOOT_LOGO_H * SCALE / 2;

	// Two phases, adding up to the ~1.3s the descent-and-sweep used to take
	// before the chime: the wave carries the logo in, then the accent band
	// runs through it.
	constexpr uint32_t FLY_MS = 760, BAND_MS = 540;

	// Wavefront: a source-column index that travels left to right, reaching
	// the right edge as FLY_MS ends and continuing past it (which damps the
	// residual ripple in the same left-to-right order). SPREAD columns are
	// mid-flight at any moment -- wide enough that the diagonal reads.
	constexpr int32_t SPREAD = 50;
	constexpr int32_t FLY_DY = 64; // px below its landing spot a column starts
	constexpr int32_t FLY_DX = 26; // ...and px to the left, so it arrives from
				       // the bottom *left*. Both stay modest: a
				       // long drop fans the in-flight columns so
				       // far apart that the letters stop reading.
	constexpr int32_t WAVE_OVER = 42;    // px a column overshoots *above* its
					     // landing line mid-flight, so the wave
					     // crest rides through the top of the
					     // screen before dropping back into place
	constexpr int32_t WAVE_AMP = 30;     // px of ripple travel on top of that
	constexpr int32_t WAVE_SQUASH = 110; // 8.8: vertical scale swing, the 3D turn
	constexpr int32_t WAVE_HZ_256 = 280; // ripple phase per second, 1/256 turns
	constexpr int32_t WAVE_K = 3;        // ...and per source column

	// Accent band: each column runs the whole THEME wheel, starting from the
	// user's own theme, before coming to rest on the gradient. The band is
	// BAND_W source columns per accent and sweeps far enough (one extra stop
	// for the lead-in out of ink) that the last column reaches the gradient
	// exactly as BAND_MS ends.
	constexpr int32_t BAND_W = 10;
	constexpr int32_t BAND_TRAVEL = ((int32_t)THEME_COUNT + 1) * BAND_W + BOOT_LOGO_W;

	// Stop `k` of that run for column `rx`: ink before the band, then the
	// accents from the user's theme onward, then the settled gradient.
	auto band_stop = [&](int32_t k, int32_t rx) -> color_t {
		if (k < 0)
			return ink;
		if (k >= (int32_t)THEME_COUNT)
			return grad_at(rx);
		return UI_THEMES[(g_theme + k) % THEME_COUNT].accent;
	};

	// Perspective: a column enters ZOOM_IN times life size -- as if it were
	// nearer the camera -- and shrinks to 1x as the wave passes it. 8.8,
	// eased on the same (1-a)^2 as the rest of the arrival.
	constexpr int32_t ZOOM_IN = 600; // ~2.3x
	auto col_zoom = [](int32_t ease) { return 256 + ((ZOOM_IN - 256) * ease >> 8); };

	// Where source column `rx` sits horizontally for a wavefront at `frontq`
	// (8.8 columns). The zoom has to show up here too or the letters shear:
	// magnifying a column vertically without widening its spacing to match
	// shreds it. Spacing is col_zoom() per column, so x is that integrated
	// from the right -- which works out to a lead that decays as (1-a)^3 and
	// is gone by the time a column lands. ZOOM_LEAD is the integral's
	// constant: how far right the newest column rides. Then the leftward
	// lead-in it has yet to give back. Called for rx and rx+1 so a column
	// can be stretched to meet its neighbor.
	constexpr int32_t ZOOM_LEAD = SCALE * (ZOOM_IN - 256) * SPREAD / (3 * 256);
	auto col_x = [&](int32_t rx, int32_t frontq) {
		int32_t a = (frontq - (rx << 8)) / SPREAD; // arrival, 8.8
		a = a < 0 ? 0 : (a > 256 ? 256 : a);
		int32_t inv = 256 - a;
		int32_t ease = inv * inv >> 8;        // (1-a)^2, 8.8
		int32_t lead = ease * inv >> 8;       // (1-a)^3, 8.8
		return logo_x + rx * SCALE + (ZOOM_LEAD * lead >> 8)
		     - (ease * FLY_DX >> 8);
	};

	// One splash frame at elapsed time `el`: field, byline, then the logo
	// column by column -- each with its own position, vertical scale and
	// color, all derived from where the wave and the accent band have got to.
	auto draw_frame = [&](uint32_t el) {
		fill_rect(0, 0, SCREEN->w, SCREEN->h, bg);
		draw_bitmap_1bpp((240 - BOOT_BYLINE_W) / 2, 214,
				 BOOT_BYLINE_W, BOOT_BYLINE_H, 1, boot_byline_bits, ink);

		// Wavefront in 8.8 columns: the zoom is anchored to it, so a
		// whole-column step would visibly jerk the magnified letters.
		int32_t frontq = (int32_t)((uint64_t)(BOOT_LOGO_W + SPREAD) * 256 * el / FLY_MS);
		int32_t wave_t = (int32_t)((uint64_t)el * WAVE_HZ_256 / 1000);
		// Global fade-out of the ripple, finishing before the band does
		// so the gradient comes to rest on an already still logo.
		int32_t tail = (int32_t)(FLY_MS + BAND_MS) - 120 - (int32_t)el;
		int32_t fade = tail >= 400 ? 256 : (tail <= 0 ? 0 : tail * 256 / 400);
		int32_t band = el < FLY_MS ? -1
			     : (int32_t)((uint64_t)BAND_TRAVEL * (el - FLY_MS) / BAND_MS);

		for (int32_t rx = 0; rx < BOOT_LOGO_W; rx++) {
			int32_t d = frontq - (rx << 8); // 8.8 columns behind the front
			int32_t a = d / SPREAD;
			a = a < 0 ? 0 : (a > 256 ? 256 : a);
			if (a == 0)
				continue; // the wave has not reached this column yet
			int32_t inv = 256 - a;
			int32_t ease = inv * inv >> 8;

			// Ripple envelope: decays over a window ~2.4x the arrival
			// stagger, so a column keeps swaying briefly after it lands.
			int32_t env = 256 - d / (SPREAD * 12 / 5);
			env = env < 0 ? 0 : (env > 256 ? 256 : env);
			env = env * fade >> 8;
			int32_t ph = wave_t - rx * WAVE_K;
			// Flight path: up from below (ease), arcing past the
			// landing line and back down (a half sine over the
			// column's own arrival), plus the travelling ripple.
			int32_t cy = logo_mid_y + (ease * FLY_DY >> 8)
				   - (WAVE_OVER * boot_sin(a >> 1) >> 8)
				   + ((WAVE_AMP * boot_sin(ph) >> 8) * env >> 8);
			// Vertical scale: the ripple's squash times the perspective
			// zoom, so a column that is still oversized is oversized in
			// both axes -- the horizontal half falls out of col_x.
			int32_t vs = 256 + ((WAVE_SQUASH * boot_cos(ph) >> 8) * env >> 8);
			vs = vs * col_zoom(ease) >> 8;
			if (vs > 660) // the two stack; past ~2.6x the crest
				vs = 660; // stops reading as letters at all

			// Stretch each column to meet its neighbor: mid-flight the
			// per-column x offsets pull apart, and the smear that fills
			// the gap is what sells the depth.
			int32_t cx = col_x(rx, frontq);
			int32_t cw = col_x(rx + 1, frontq) - cx;
			if (cw < SCALE)
				cw = SCALE;
			else if (cw > 16)
				cw = 16;

			// Where this column is on the band's run: ink until the band
			// reaches it, then one stop per accent, ending on the
			// gradient. Adjacent stops are mixed rather than swapped, so
			// what travels through the letters is a continuous ribbon of
			// hue that comes to rest on the gradient.
			color_t c = ink;
			if (band >= 0) {
				int32_t d = band - rx + BAND_W; // -1 stop of ink lead-in
				if (d >= 0) {
					int32_t k = d / BAND_W - 1;
					c = boot_mix(band_stop(k, rx), band_stop(k + 1, rx),
						     (d % BAND_W) * 256 / BAND_W);
				}
			}

			for (int32_t i = 0; i < BOOT_LOGO_LAYERS; i++) {
				const uint8_t *bits = boot_logo_layer_bits[i];
				int32_t stride = (BOOT_LOGO_W + 7) / 8;
				for (int32_t ry = 0; ry < BOOT_LOGO_H; ry++) {
					if (!(bits[ry * stride + rx / 8] & (0x80 >> (rx & 7))))
						continue;
					// Row span under the column's vertical scale,
					// measured out from the logo's midline.
					int32_t y0 = cy + (((2 * ry - BOOT_LOGO_H) * SCALE * vs) >> 9);
					int32_t y1 = cy + (((2 * (ry + 1) - BOOT_LOGO_H) * SCALE * vs) >> 9);
					fill_rect_clipped(cx, y0, cw, y1 > y0 ? y1 - y0 : 1, c);
				}
			}
		}
		_flip();
		while (_in_flip)
			tight_loop_contents();
	};

	// Our own chime rings out under the fly-in. It is alarm-driven rather
	// than blocking precisely so it can overlap the animation -- the
	// drawing below keeps running between its per-sample interrupts -- and
	// it has rung down long before the loop ends. The matching stop() is
	// below, before anything else drives the DAC.
#if ENABLE_SOUND
	audio_output_start_startup_chime();
#endif

	uint32_t t0 = time();
	for (;;) {
		uint32_t el = time() - t0;
		if (el >= FLY_MS + BAND_MS)
			break;
		draw_frame(el);
		sleep(4);
	}
	draw_frame(FLY_MS + BAND_MS); // settled: the gradient, held for the chime

	// Colors settled -- hand the DAC over to the second chime: the "ba-ding"
	// based on GBC boot (audio_output_play_boot_jingle), whose ring-down
	// doubles as the hold before the cut. Release our chime's alarm first; it
	// finished ~1.7s ago, but the jingle drives the same PWM register from
	// a blocking loop, so ownership is ended explicitly rather than left to
	// timing. Muted (or soundless build): hold the same beat silently so
	// the splash still reads.
#if ENABLE_SOUND
	audio_output_stop_startup_chime();
	if (audio_output_get_volume() > 0)
		audio_output_play_boot_jingle();
	else
		sleep(800);
#else
	sleep(800);
#endif
	sleep(150); // beat of silence before the cut, like the original

	// Cut to black before emulation, same push-and-wait as the power-on
	// GRAM clear, so the game's first frame doesn't tear out of the splash.
	fill_rect(0, 0, SCREEN->w, SCREEN->h, 0);
	_flip();
	while (_in_flip)
		tight_loop_contents();
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

	// Measure the panel's true TE period (averaged over 16 pulses, so ~270ms
	// spent here with the backlight still off) and
	// decide whether TE-locked vsync is usable -- see g_te_period_us /
	// g_te_pace. Gate: 54-66Hz, generous enough for the ST7789 RC
	// oscillator's unit-to-unit spread; outside it means the measurement
	// itself is suspect (TE glitching), not just an off-nominal panel.
	g_te_period_us = _measure_te_period_us();
	g_te_pace = g_te_period_us >= 15152 && g_te_period_us <= 18519;

	for (int32_t y = 0; y < SCREEN->h; y++)
		for (int32_t x = 0; x < SCREEN->w; x++)
			*SCREEN->p(x, y) = 0; // black border around the GBC frame

	// Push the black frame to the panel before the backlight ever comes on:
	// nothing in the init sequence clears GRAM, so from DISPON until the
	// first flip the panel is scanning power-on noise. The picker used to
	// flip within a frame of backlight-on, masking it; with autoboot and
	// TE-armed vsync the first flip can now land tens of ms later, long
	// enough for the noise to flash visibly at boot.
	_flip();
	while (_in_flip)
		tight_loop_contents();

	// Restore the persisted device settings (brightness, volume, staged RTC)
	// before anything is displayed, so the boot menu already comes up with
	// the user's values. Defaults (g_brightness = 75, volume muted, clock
	// Sunday midnight) stand when nothing was ever stored.
	{
		// Anchor the staged clock's wall-clock baseline before any menu
		// can display it (rtc_stage_tick()).
		rtc_stage_tick_reset();
		device_settings_t ds;
		if (save_storage_settings_load(ds)) {
			g_brightness = ds.brightness < BRIGHTNESS_MIN ? BRIGHTNESS_MIN
				     : ds.brightness > 100 ? 100 : ds.brightness;
			g_rtc_dow  = ds.rtc_dow  % 7;
			g_rtc_hour = ds.rtc_hour % 24;
			g_rtc_min  = ds.rtc_min  % 60;
			g_vsync = ds.vsync != 0;
			g_status_bar = ds.status_bar < STATUS_MODE_COUNT
				     ? ds.status_bar : (uint8_t)STATUS_FPS_PCT;
			apply_game_offset();
			apply_theme(ds.theme); // out-of-range falls back to MINT
			apply_mode(ds.dark_mode); // restore dark/light appearance
			g_boot_last = ds.boot_last != 0;
			g_last_slot = ds.last_slot;
			g_nostalgic_boot = ds.nostalgic_boot != 0;
			g_save_interval = ds.save_interval < SAVE_INTERVAL_COUNT
					? ds.save_interval : SAVE_INTERVAL_DEFAULT;
			// Rebuilds the DMG ramp and marks the CGB palette dirty;
			// both LUTs are rebuilt before the first frame renders.
			apply_color_filter(ds.color_filter < CF_COUNT
					   ? ds.color_filter : (uint8_t)CF_OFF);
#if ENABLE_SOUND
			audio_output_set_volume(ds.volume); // clamps internally
#endif
		}
		// Apply unconditionally -- the defaults matter just as much as a
		// restored value, since the state machine starts at its own.
		apply_save_interval();
	}

	// With the restored vsync setting and the TE measurement both in hand,
	// pin the audio production-rate assumption before core1 (which claims
	// and tunes the pacing timer) is launched.
	apply_audio_pacing();

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
		_reset_io();
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
	// No display to pick from with ENABLE_LCD off -- always boot the first
	// catalog entry.
	uint32_t rom_index = 0;
#endif
	const rom_entry_t &rom = rom_catalog[rom_index];
	g_rom_data = rom.data;
	// Fill the SRAM mirror of the ROM's first 2KB (see rom_mirror up top) --
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

	// Frame-skip rendering, off: when set, __gb_draw_line renders every
	// scanline on alternate frames only (whole frames skipped, not individual lines),
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

#if ENABLE_LCD
	// NOSTALGIC BOOT: the Game Boy-style splash runs here, after the PWM DAC
	// is configured (the chime drives it directly) and before core1 launches
	// (after which the audio DMA owns that register and the scanline worker
	// owns the framebuffer).
	if (g_nostalgic_boot)
		nostalgic_boot_splash();
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
	// Resetting (rather than just sampling) also drops the transitions queued
	// while the boot menu and ROM load ran, which would otherwise replay as
	// phantom input on the game's first frames.
	_reset_io();

	// The LED reports battery charge: a green->red gradient (green ~= full,
	// red ~= nearly empty), capped by led_brightness(). Updated on a cadence rather than
	// every frame -- battery() reads the ADC and the charge level changes
	// slowly, so per-frame polling would only waste cycles and jitter the LED.
	// Starting the counter at the threshold makes the first loop iteration do
	// an immediate update instead of sitting on the boot-success green for ~0.5s.
	constexpr int BATTERY_UPDATE_FRAMES = 30;
	int battery_count = BATTERY_UPDATE_FRAMES;
	int batt_level = 0; // last polled level, so the save blink can hand the
			    // LED straight back without waiting for the next poll

	// Flash-commit LED blink state (see the blink block in the loop).
	constexpr uint64_t SAVING_HOLD_US = 2000000;
	// Cap on how long a *queued* save (requested, nothing written yet) may
	// hold the blink up. The commit itself re-arms the hold for as long as it
	// actually runs, but a request can sit behind a multi-second slot
	// pre-erase, and re-arming across all of that -- then again on the next
	// request -- is what let repeated X+B presses run the blink together into
	// one that never visibly ended.
	constexpr uint64_t SAVING_PENDING_MAX_US = 3000000;
	uint64_t saving_hold_until = 0;  // blink runs while now < this
	uint64_t pending_since = 0;      // when the queued save was first seen (0 = none)
	bool saving_led = false;         // save blink owns the LED

#if ENABLE_LCD
	// Status bar state: the values currently painted in the top letterbox,
	// plus a ~1s wall-clock window for the FPS count. batt_shown = -1 makes
	// the first battery poll (immediate, see battery_count above) paint the
	// bar on the first frame instead of leaving it blank for a second.
	uint32_t fps_shown = 0;
	int32_t batt_shown = -1;
	uint64_t batt_hold_until = 0; // displayed level pinned until this time
	uint32_t fps_frames = 0;
	uint64_t fps_window_start = time_us_64();
#endif

	// Real-time pacer (replaces _wait_vsync()). Pace to the GBC's true frame
	// period: run flat-out while behind real time, and only wait when a
	// frame finishes early, so the game can never run *faster* than
	// authentic speed. (The original blanket _wait_vsync() gating dated from
	// the 40Hz panel days, where it capped the game at 40/59.73 = 67% speed
	// no matter how fast emulation got; the panel now runs at ~60Hz, and
	// when its measured rate is close enough -- g_te_pace -- the vsync-ON
	// path waits on the TE-consumed arm instead of the wall clock, locking
	// emulation 1:1 to the panel with no speed cap. See the arming policy at
	// the bottom of the loop.) With vsync OFF the cost is losing vsync
	// alignment on _flip() (possible shear on fast scrolls); tearing was
	// already half-accepted anyway, since emulation draws into the same
	// buffer the flip DMA reads from.
	uint64_t frame_due = time_us_64();
	g_rtc_last_us = frame_due; // discard boot/menu time predating the game

	// Consecutive behind-schedule frames that skipped arming a vsync flip --
	// see the arming policy at the bottom of the loop.
	uint32_t vsync_skipped = 0;

#if ENABLE_LCD
	// The boot menu's pixels are still in the framebuffer. The game only
	// repaints its own SCALED_H rows and the header (when shown) covers the
	// rest -- but in FULLSCREEN the 12px letterbox bands are never painted,
	// so menu remnants would peek out above and below the game. Clear to
	// black before the first frame, letting any in-flight menu flip finish
	// first so the wipe can't blank rows the DMA is still streaming.
	while (_in_flip)
		tight_loop_contents();
	memset(SCREEN->data, 0, SCREEN->w * SCREEN->h * sizeof(color_t));
#endif

	while (true) {
		// One joypad state per emulated frame is all gb.direct.joypad can
		// carry, and this loop's cadence isn't constant (it stalls on core1
		// and on save-flash erases), so anything that happened to a button
		// since the last poll has to be delivered as a sequence rather than a
		// level. _poll_io() advances each button by one queued transition per
		// call, which is what guarantees a fast tap is seen as press *and*
		// release instead of being merged into its neighbours -- see the
		// edge-queue block comment in picosystem/hardware.cpp.
		_poll_io();

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
		// X+B chord (either button completing it): commit the save to
		// flash now. Only bound in MANUAL mode -- on every other setting
		// the interval already writes on its own, and X+B is an ordinary
		// in-game combination (Start + B) that shouldn't touch flash
		// behind the player's back. Unlike the Y+X menu chord this one
		// isn't swallowed: both buttons still reach the game below, so a
		// save can be triggered without disturbing what's on screen. The
		// LED's save blink is the acknowledgement -- armed here rather
		// than left to the commit window alone, so a press lands visibly
		// even when there is nothing dirty to write (the request is then a
		// no-op, and a silent one would be indistinguishable from a chord
		// that didn't register). A press while a request is already
		// queued is dropped rather than re-armed: the save it is asking
		// for is already on its way, and re-arming from a combination the
		// game itself uses is how a stray Start+B (or a run of them) used
		// to chain 2s holds together into a blink that never let up.
		if (save_interval_manual() && !save_storage_pending() &&
		    button(picosystem::X) && button(picosystem::B) &&
		    (pressed(picosystem::X) || pressed(picosystem::B))) {
			save_storage_request_save();
			saving_hold_until = time_us_64() + SAVING_HOLD_US;
		}

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

		// Track the commit window for the LED blink below. The commit
		// programs ~1s of 512B chunks; the hold slides along with it and
		// keeps the window open a beat past the last one, so the blink is
		// easy to notice however short the write was. A *requested* save
		// counts as active before any flash op starts (see
		// save_storage_pending) -- otherwise a manual X+B save waiting on
		// the slot pre-erase or the quiet gate would sit dark for a second
		// or two, reading as "the chord did nothing" -- but only for
		// SAVING_PENDING_MAX_US, so a queue that is waiting on a slow
		// pre-erase can't hold the LED up indefinitely. The write itself
		// (save_storage_saving) re-arms unconditionally: that is the window
		// a power-off would actually damage, and it is self-limiting.
		uint64_t saving_now = time_us_64();
		if (save_storage_pending()) {
			if (!pending_since)
				pending_since = saving_now;
		} else {
			pending_since = 0;
		}
		if (save_storage_saving() ||
		    (pending_since && saving_now - pending_since < SAVING_PENDING_MAX_US))
			saving_hold_until = saving_now + SAVING_HOLD_US;
		bool saving = saving_now < saving_hold_until;

		if (++battery_count >= BATTERY_UPDATE_FRAMES) {
			battery_count = 0;
			int level = battery();
			if (level < 0)   level = 0;
			if (level > 100) level = 100;
			batt_level = level;
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
			// While the save blink owns the LED, skip the write so no
			// battery color sneaks into the blue/magenta blink.
			if (!saving)
				led_show_battery(level);
		}

		// The LED signposts the flash-commit window: a fast blue/magenta
		// blink, in every screen mode -- so a power-off during the write
		// window is always signposted. The closing edge hands the LED
		// straight back to the battery gradient (or dark, in FS BATTERY)
		// from the last polled level, rather than forcing the battery
		// counter and waiting a frame for it: the deferred version left
		// the LED sitting on whichever blink colour was last written
		// until that tick came round.
		if (saving) {
			const int b = led_brightness();
			if ((saving_now >> 17) & 1) // ~131ms half-period, ~3.8Hz
				led(0, 0, b);  // blue
			else
				led(b, 0, b);  // magenta
			saving_led = true;
		} else if (saving_led) {
			saving_led = false;
			led_show_battery(batt_level);
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
		if (status_dirty && (!status_fullscreen() || status_fs_battery())) {
			// The header band is rows 0..OFFSET_Y-1 (FS BATTERY's meter
			// is the first 2 of them) -- the very rows an in-flight
			// vsync flip streams first. Same chase as core1's; ~1x/sec
			// and worst-case ~1ms, so off the frame budget.
			if (g_vsync) {
				const int32_t rows = status_fs_battery()
						   ? FS_BATT_BAR_H : OFFSET_Y;
				const uintptr_t hdr_end = (uintptr_t)SCREEN->data +
					(uintptr_t)rows * SCREEN->w * sizeof(color_t);
				while (_in_flip && dma_hw->ch[dma_channel].read_addr < hdr_end)
					tight_loop_contents();
			}
			if (status_fs_battery())
				draw_fs_battery_bar(batt_shown < 0 ? 0 : (uint32_t)batt_shown);
			else
				draw_status_bar(fps_shown, batt_shown < 0 ? 0 : (uint32_t)batt_shown);
		}

		// Let core1 finish this frame's queued scanlines before flipping,
		// so the DMA never streams a frame with lines still landing. Core1
		// runs well ahead of the emulator, so this is normally a no-op wait.
		while (_line_rd != _line_wr)
			tight_loop_contents();
#endif

		frame_due += GB_FRAME_US;
		uint64_t now = time_us_64();

		if (g_vsync && g_te_pace) {
			// TE lock: the frame is complete in the buffer (ring drained
			// above), so arm unconditionally and wait for the TE IRQ to
			// consume the arm before emulating any further. Emulation runs
			// at exactly the panel's measured rate, every frame is
			// presented, and -- the part no arming heuristic can deliver --
			// the buffer is *coherent at every flip*: the next frame's rows
			// only start landing after the flip DMA is already streaming,
			// so the writer stays behind the DMA (core1's chase enforces
			// it) and a flip can never stream a half-written frame. The
			// earlier wall-clock arming schemes all shared that hole: an
			// arm pending while emulation ran ahead meant TE could fire
			// mid-write and stream a mixed buffer (new rows on top, old
			// below) -- the residual "still tears with vsync on" reports.
			//
			// No catch-up sprints in this mode: after a stall (flash erase,
			// settings menu) we simply resume at panel rate. Wall-clock
			// debt repayment bought nothing a player can see -- gameplay
			// isn't wall-anchored and the RTC ticks off the wall clock
			// independently (rtc poll below).
			//
			// A frame that overruns one TE period just misses that TE: the
			// panel repeats the previous (complete) frame once -- a hitch,
			// never a tear. The deadline only guards against TE dying
			// mid-game (never seen; boot measurement proved the pin
			// pulses).
			_flip_armed = true;
			const uint64_t deadline = now + 2 * GB_FRAME_US;
			while (_flip_armed && time_us_64() < deadline)
				tight_loop_contents();
			// Keep the wall-clock bookkeeping current so toggling vsync
			// OFF mid-game doesn't inherit stale debt.
			frame_due = time_us_64();
		} else {
			// Wall-clock pacer -- vsync OFF, or ON with a panel whose TE
			// measurement failed the (wide, 54-66Hz) lock gate. In the
			// latter, should-never-happen case the old arming policy
			// applies: on-schedule frames arm; behind-schedule frames skip
			// arming (so the chase never throttles a catch-up sprint)
			// except every 3rd, so the screen keeps moving through
			// sustained slowdown. Tear-free is NOT guaranteed here -- a
			// pending arm can be consumed mid-write of the next frame --
			// it's strictly a degraded fallback.
			if (g_vsync && (now < frame_due || ++vsync_skipped >= 3)) {
				vsync_skipped = 0;
				_flip_armed = true;
			}

			if (now < frame_due) {
				// Ahead of real time: hold at authentic game speed.
				while (time_us_64() < frame_due)
					tight_loop_contents();
			} else if (now - frame_due > 4 * GB_FRAME_US) {
				// Fell badly behind (e.g. a save-flash erase pause): drop
				// the accumulated debt instead of fast-forwarding.
				frame_due = now;
			}
		}

		if (!g_vsync)
			_flip();
	}
}

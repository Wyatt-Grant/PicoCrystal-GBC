// Host-side renderer for the UI screens in main.cpp -- draws the boot menu,
// settings menu, and in-game status bar into a 240x240 RGBA4444 buffer and
// writes them out as PPMs, so UI changes can be eyeballed without flashing
// hardware. It compiles the *actual* drawing code: build_render_menus.sh
// sed-extracts the @ui-draw / @menu-draw fenced ranges from main.cpp into
// .inc files that this file includes between its stubs.
//
//   tools/build_render_menus.sh <out_dir>
//
// The stubs below mirror just enough of the firmware environment (SCREEN,
// OFFSET_Y, volume/brightness/RTC state, rom_catalog) for those ranges to
// compile standalone; keep them in sync if the fenced code grows new
// dependencies.

#include <cstdint>
#include <cstring>
#include <cstdio>

typedef uint16_t color_t;

struct buffer_t {
	int32_t w, h;
	color_t *data;
	color_t *p(int32_t x, int32_t y) { return data + y * w + x; }
};

static color_t _fb[240 * 240];
static buffer_t _screen_storage = { 240, 240, _fb };
static buffer_t *SCREEN = &_screen_storage;

constexpr int32_t OFFSET_Y = 24;

#define ENABLE_SOUND 1
constexpr uint16_t VOLUME_MAX = 800;
static uint16_t audio_output_get_volume() { return 320; } // reads back as 40%
static uint8_t g_brightness = 70;
// Mirrors main.cpp's STATUS BAR mode enum + helpers (they sit outside the
// fenced ranges, next to the other globals stubbed here).
enum status_bar_mode_t : uint8_t {
	STATUS_FPS_PCT,
	STATUS_FPS,
	STATUS_PCT,
	STATUS_ICON,
	STATUS_FULLSCREEN,
	STATUS_FS_BATTERY,
	STATUS_MODE_COUNT,
};
static uint8_t g_status_bar = STATUS_FPS_PCT; // both header pieces render
static inline bool status_show_fps() { return g_status_bar <= STATUS_FPS; }
static inline bool status_show_pct() {
	return g_status_bar == STATUS_FPS_PCT || g_status_bar == STATUS_PCT;
}
static inline bool status_fullscreen() { return g_status_bar >= STATUS_FULLSCREEN; }
static inline bool status_fs_battery() { return g_status_bar == STATUS_FS_BATTERY; }
constexpr int32_t FS_BATT_BAR_H = 2;
static inline bool menu_show_pct() {
	return status_show_pct() || status_fullscreen();
}
// LED brightness cap (led_show_battery() in the fenced range calls it); the
// value only reaches hardware, so any stub-consistent number does here.
static inline int led_brightness() { return 45 * g_brightness / 100; }

// led() only drives hardware, so it's a no-op here.
static void led(int, int, int) {}

#include "ui_draw.inc"

static uint8_t g_rtc_dow = 2; // TUE
static uint8_t g_rtc_hour = 14;
static uint8_t g_rtc_min = 32;
static uint8_t g_rtc_sec = 7;
static bool g_vsync = true; // render the settings screen with the toggle ON
static bool g_color_filter = true; // COLOR FILTER row rendered ON
static bool g_boot_last = true; // BOOT LAST GAME row rendered ON
static bool g_nostalgic_boot = true; // NOSTALGIC BOOT row rendered ON
// SAVE INTERVAL row: rendered at the 10S option (a plain seconds value; index
// 0 would render "MANUAL" instead).
static const uint16_t SAVE_INTERVAL_SECS[] = { 0, 3, 5, 10, 30, 60, 120 };
static uint8_t g_save_interval = 3;
static inline bool save_interval_manual() { return g_save_interval == 0; }
static bool g_te_pace = true;             // render the VSYNC row TE-locked...
static uint32_t g_te_period_us = 16667;   // ...showing "60HZ"

struct rom_entry_t {
	const char *name;
	const void *data;
	uint32_t size;
	uint32_t save_slot;
	uint16_t label_color; // RGBA4444 cart-label tint, 0 = default accent
	// 8x8 RGBA4444 tile from assets/icons.png, or nullptr for the drawn cart
	// glyph (see the two catalogs below).
	const uint16_t *icon;
};

#ifdef REAL_ROM_CATALOG
// build_render_menus.sh found a built firmware's generated rom_data.cpp and
// stripped its ROM payloads: real names, cart tints and icons.png tiles, so
// the boot menu screenshot shows the menu as it looks on this device.
#include "rom_catalog.inc"
constexpr uint32_t ROM_COUNT = sizeof rom_catalog / sizeof rom_catalog[0];
#else
// No firmware build to borrow a catalog from, so this placeholder list stands
// in -- what a fresh checkout renders. A couple of entries carry a
// label_color to exercise the per-ROM cart tint; the rest fall back to the
// accent (0), and every icon is null, i.e. the no-icons.png cart glyph.
constexpr uint32_t ROM_COUNT = 8;
static const rom_entry_t rom_catalog[ROM_COUNT] = {
	{ "POKEMON CRYSTAL", nullptr, 8 * 1024 * 1024, 0, 0xF3F1 },  // yellow-ish
	{ "CHROMATIC TETRIS", nullptr, 512 * 1024, 1, 0xFF04 },      // blue
	{ "SUPER MARIO BROS", nullptr, 1024 * 1024, 3, 0 },
	{ "ZELDA ORACLE OF SEASONS", nullptr, 1024 * 1024, 4, 0 },
	{ "POKEMON PRISM", nullptr, 2 * 1024 * 1024, 7, 0 },
	{ "KIRBY DREAM LAND 2", nullptr, 1024 * 1024, 2, 0 },
	{ "STAR OCEAN BLUE SPHERE", nullptr, 4 * 1024 * 1024, 5, 0 },
	{ "POKEMON PINBALL", nullptr, 2 * 1024 * 1024, 6, 0 },
};
#endif

#include "menu_draw.inc"

// ---------------------------------------------------------------------
// Game canvas: a 160x144 GBC frame (tools/grab_frame.c dumps one as a PPM)
// blitted into the framebuffer through the same 1.5x scaler the device uses,
// so the in-game screenshot shows the real thing -- blended columns, blended
// rows, header band and all -- rather than a bare nearest-neighbour upscale.
// The scaler mirrors core1's scanline writer in main.cpp (blend_avg + the
// 3:2 even/odd/mid row split); it lives outside the fenced ranges because on
// the device it is fused into the line-render hot path.

constexpr int32_t GB_W = 160, GB_H = 144;
constexpr int32_t SCALED_W = 240, SCALED_H = 216;

static inline color_t blend_avg(color_t a, color_t b) {
	return (color_t)((a & b) + (((a ^ b) & 0xEEEE) >> 1));
}

// Reads a P6 PPM with the trivial header grab_frame.c/headless_test.c write
// ("P6\n<w> <h>\n255\n"); anything else is a caller error, not user input.
static bool read_gb_ppm(const char *path, color_t out[GB_H][GB_W]) {
	FILE *f = fopen(path, "rb");
	if (!f) {
		perror(path);
		return false;
	}
	int w = 0, h = 0, maxval = 0;
	if (fscanf(f, "P6 %d %d %d", &w, &h, &maxval) != 3 || w != GB_W ||
	    h != GB_H || maxval != 255) {
		fprintf(stderr, "%s: not a %dx%d 8-bit P6 PPM\n", path, GB_W, GB_H);
		fclose(f);
		return false;
	}
	fgetc(f); // the single whitespace byte after maxval
	for (int32_t y = 0; y < GB_H; y++) {
		for (int32_t x = 0; x < GB_W; x++) {
			int r = fgetc(f), g = fgetc(f), b = fgetc(f);
			if (b == EOF) {
				fprintf(stderr, "%s: truncated\n", path);
				fclose(f);
				return false;
			}
			// 8-bit -> the top 4 bits of each channel, the same
			// truncation rgb565_to_color() does on the device.
			out[y][x] = (color_t)((r >> 4) | (0xF << 4) |
					      ((b >> 4) << 8) | ((g >> 4) << 12));
		}
	}
	fclose(f);
	return true;
}

// 1.5x (3:2): source columns pair up into (s0, avg, s1) and source rows into
// (row 3k, the average of the pair, row 3k+2), landing at y = offset_y.
static void blit_game(const color_t src[GB_H][GB_W], int32_t offset_y) {
	static color_t prev_row[SCALED_W];
	for (int32_t ly = 0; ly < GB_H; ly++) {
		const int32_t k = ly >> 1;
		const bool odd = ly & 1;
		color_t *dst = _screen_storage.p(0, offset_y + 3 * k + (odd ? 2 : 0));
		for (int32_t s = 0, d = 0; s < GB_W; s += 2, d += 3) {
			color_t c0 = src[ly][s], c1 = src[ly][s + 1];
			dst[d]     = c0;
			dst[d + 1] = blend_avg(c0, c1);
			dst[d + 2] = c1;
		}
		if (odd) {
			color_t *mid = _screen_storage.p(0, offset_y + 3 * k + 1);
			for (int32_t x = 0; x < SCALED_W; x++)
				mid[x] = blend_avg(prev_row[x], dst[x]);
		} else {
			memcpy(prev_row, dst, sizeof prev_row);
		}
	}
}

// RGBA4444 layout per main.cpp's rgb565_to_color: R 0-3, A 4-7, B 8-11, G 12-15.
static void write_ppm(const char *path) {
	FILE *f = fopen(path, "wb");
	if (!f) {
		perror(path);
		return;
	}
	fprintf(f, "P6\n%d %d\n255\n", SCREEN->w, SCREEN->h);
	for (int32_t i = 0; i < SCREEN->w * SCREEN->h; i++) {
		color_t c = _fb[i];
		uint8_t rgb[3] = {
			(uint8_t)((c & 0xF) * 17),
			(uint8_t)(((c >> 12) & 0xF) * 17),
			(uint8_t)(((c >> 8) & 0xF) * 17),
		};
		fwrite(rgb, 1, 3, f);
	}
	fclose(f);
	printf("wrote %s\n", path);
}

// Lowercased theme name, for the per-theme output filenames.
static void theme_slug(const char *name, char *out, size_t n) {
	size_t i = 0;
	for (; name[i] && i + 1 < n; i++)
		out[i] = (char)(name[i] >= 'A' && name[i] <= 'Z' ? name[i] + 32
								: name[i]);
	out[i] = '\0';
}

int main(int argc, char **argv) {
	const char *dir = argc > 1 ? argv[1] : ".";
	// Optional 160x144 GBC frame (tools/grab_frame.c) for the in-game shots.
	const char *game_ppm = argc > 2 ? argv[2] : nullptr;
	char path[512];

	draw_boot_menu(1, 82);
	snprintf(path, sizeof path, "%s/boot.ppm", dir);
	write_ppm(path);

	draw_settings_menu(SET_ROW_BRIGHT, 82);
	snprintf(path, sizeof path, "%s/settings.ppm", dir);
	write_ppm(path);

	// The CLOCK row selected in the list, and the editor it opens on A.
	draw_settings_menu(SET_ROW_CLOCK, 82);
	snprintf(path, sizeof path, "%s/settings_clock.ppm", dir);
	write_ppm(path);

	// The editor with DAY selected -- the group the dial answers with its
	// weekday window rather than a hand.
	draw_clock_menu(CLK_ROW_DOW, 82);
	snprintf(path, sizeof path, "%s/clock_menu.ppm", dir);
	write_ppm(path);

	// SAVE INTERVAL on MANUAL with its row selected -- the one row that
	// swaps the hint line (for the B+Y save chord).
	g_save_interval = 0;
	draw_settings_menu(SET_ROW_SAVE, 82);
	snprintf(path, sizeof path, "%s/settings_manual.ppm", dir);
	write_ppm(path);
	g_save_interval = 3;

	memset(_fb, 0, sizeof _fb);
	draw_status_bar(60, 82);
	snprintf(path, sizeof path, "%s/statusbar.ppm", dir);
	write_ppm(path);

	// STATUS BAR mode FPS: "<n>%" text gone from every header (in-game,
	// boot, settings), FPS readout and icon stay.
	g_status_bar = STATUS_FPS;
	memset(_fb, 0, sizeof _fb);
	draw_status_bar(60, 82);
	snprintf(path, sizeof path, "%s/statusbar_nopct.ppm", dir);
	write_ppm(path);

	draw_settings_menu(SET_ROW_STATUS, 82);
	snprintf(path, sizeof path, "%s/settings_nopct.ppm", dir);
	write_ppm(path);

	// ICON mode: header carries only the battery icon.
	g_status_bar = STATUS_ICON;
	memset(_fb, 0, sizeof _fb);
	draw_status_bar(60, 82);
	snprintf(path, sizeof path, "%s/statusbar_icon.ppm", dir);
	write_ppm(path);

	// FULLSCREEN mode: no in-game header at all, but the menus keep their
	// band *and* the "<n>%" (menu_show_pct()).
	g_status_bar = STATUS_FULLSCREEN;
	draw_boot_menu(1, 82);
	snprintf(path, sizeof path, "%s/boot_fullscreen.ppm", dir);
	write_ppm(path);

	// A non-default theme with the THEME row selected -- every
	// accent-tinted element should recolor.
	g_status_bar = STATUS_FPS_PCT;
	apply_theme(13); // GRAPE
	draw_settings_menu(SET_ROW_THEME, 82);
	snprintf(path, sizeof path, "%s/settings_grape.ppm", dir);
	write_ppm(path);

	// Light mode with the APPEARANCE row selected -- the whole neutral ramp
	// inverts (light card, dark text) while the accent theme stays put.
	apply_theme(0); // back to MINT
	apply_mode(0);  // LIGHT
	draw_settings_menu(SET_ROW_MODE, 82);
	snprintf(path, sizeof path, "%s/settings_light.ppm", dir);
	write_ppm(path);
	// Boot menu in light mode: the selection pill should read as a soft tint,
	// not the dark dark-mode blob.
	draw_boot_menu(1, 82);
	snprintf(path, sizeof path, "%s/boot_light.ppm", dir);
	write_ppm(path);
	apply_mode(1); // restore DARK for any later frames

	// ---- README screenshot set (make_screenshots.sh) --------------------
	// Everything below is rendered for the repo's screenshots/ gallery: the
	// full accent-theme sweep, the light/dark pair, and -- when a game frame
	// is supplied -- the in-game canvas under the real device chrome.

	// One settings screen per accent theme, THEME row selected so the name
	// and every accent-tinted element are both visible in the same shot.
	char slug[32];
	for (uint32_t t = 0; t < THEME_COUNT; t++) {
		apply_theme(t);
		draw_settings_menu(SET_ROW_THEME, 82);
		theme_slug(UI_THEMES[t].name, slug, sizeof slug);
		snprintf(path, sizeof path, "%s/theme_%s.ppm", dir, slug);
		write_ppm(path);
	}
	apply_theme(0); // back to MINT

	// Light/dark pair, same theme and same selected row in both, so the two
	// shots differ only in the neutral ramp.
	apply_mode(0); // LIGHT
	draw_boot_menu(1, 82);
	snprintf(path, sizeof path, "%s/boot_light_mint.ppm", dir);
	write_ppm(path);
	draw_settings_menu(SET_ROW_BRIGHT, 82);
	snprintf(path, sizeof path, "%s/settings_light_mint.ppm", dir);
	write_ppm(path);
	// The dial's plate/bezel/hands all come out of the neutral ramp, so the
	// clock editor gets a light-mode shot of its own.
	draw_clock_menu(CLK_ROW_MIN, 82);
	snprintf(path, sizeof path, "%s/clock_light.ppm", dir);
	write_ppm(path);
	apply_mode(1); // DARK

	// The clock editor, the screen A opens from the list's CLOCK row.
	draw_clock_menu(CLK_ROW_HOUR, 82);
	snprintf(path, sizeof path, "%s/clock.ppm", dir);
	write_ppm(path);

	// SEC selected: the half-scale seconds group is the one whose pill and
	// arrows aren't sized off CLK_SCALE, so it gets a shot of its own.
	draw_clock_menu(CLK_ROW_SEC, 82);
	snprintf(path, sizeof path, "%s/clock_sec.ppm", dir);
	write_ppm(path);

	if (game_ppm) {
		static color_t frame[GB_H][GB_W];
		if (!read_gb_ppm(game_ppm, frame))
			return 1;

		// Normal: 24px header band above the 240x216 game canvas.
		g_status_bar = STATUS_FPS_PCT;
		memset(_fb, 0, sizeof _fb);
		blit_game(frame, OFFSET_Y);
		draw_status_bar(60, 82);
		snprintf(path, sizeof path, "%s/in_game.ppm", dir);
		write_ppm(path);

		// FULLSCREEN: no header at all, canvas centered in a 12px
		// letterbox top and bottom.
		g_status_bar = STATUS_FULLSCREEN;
		memset(_fb, 0, sizeof _fb);
		blit_game(frame, (240 - SCALED_H) / 2);
		draw_status_bar(60, 82); // no-ops in FULLSCREEN, kept for parity
		snprintf(path, sizeof path, "%s/in_game_fullscreen.ppm", dir);
		write_ppm(path);

		// FS BATTERY: the centered canvas pushed down by the meter's
		// height (apply_game_offset()), with the 2px screen-wide
		// battery meter in the letterbox above it.
		g_status_bar = STATUS_FS_BATTERY;
		memset(_fb, 0, sizeof _fb);
		blit_game(frame, (240 - SCALED_H) / 2 + FS_BATT_BAR_H);
		draw_fs_battery_bar(82);
		snprintf(path, sizeof path, "%s/in_game_fs_battery.ppm", dir);
		write_ppm(path);
		g_status_bar = STATUS_FPS_PCT;
	}

	return 0;
}

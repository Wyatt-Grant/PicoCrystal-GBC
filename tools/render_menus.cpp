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
	STATUS_MODE_COUNT,
};
static uint8_t g_status_bar = STATUS_FPS_PCT; // both header pieces render
static inline bool status_show_fps() { return g_status_bar <= STATUS_FPS; }
static inline bool status_show_pct() {
	return g_status_bar == STATUS_FPS_PCT || g_status_bar == STATUS_PCT;
}
static inline bool status_fullscreen() { return g_status_bar == STATUS_FULLSCREEN; }
static inline bool menu_show_pct() {
	return status_show_pct() || status_fullscreen();
}
// LED brightness cap (led_show_rgb()/led_show_battery() in the fenced range
// call it); the value only reaches hardware, so any stub-consistent number
// does here.
static inline int led_brightness() { return 45 * g_brightness / 100; }

// PicoSystem SDK color/hardware stubs for the RGB pseudo-theme path: hsv()
// packs like the firmware's color_t (R 0-3, A 4-7, B 8-11, G 12-15, see
// write_ppm below); led() only drives hardware, a no-op here.
static color_t hsv(float h, float s, float v) {
	float r = v, g = v, b = v;
	if (s > 0.0f) {
		h = (h - (int)h) * 6.0f;
		int i = (int)h;
		float f = h - (float)i;
		float p = v * (1.0f - s);
		float q = v * (1.0f - s * f);
		float t = v * (1.0f - s * (1.0f - f));
		switch (i) {
		case 0:  r = v; g = t; b = p; break;
		case 1:  r = q; g = v; b = p; break;
		case 2:  r = p; g = v; b = t; break;
		case 3:  r = p; g = q; b = v; break;
		case 4:  r = t; g = p; b = v; break;
		default: r = v; g = p; b = q; break;
		}
	}
	uint16_t R = (uint16_t)(r * 15.0f), G = (uint16_t)(g * 15.0f),
		 B = (uint16_t)(b * 15.0f);
	return (color_t)((R & 0xF) | (0xF << 4) | ((B & 0xF) << 8) | ((G & 0xF) << 12));
}
static void led(int, int, int) {}

#include "ui_draw.inc"

static uint8_t g_rtc_dow = 2; // TUE
static uint8_t g_rtc_hour = 14;
static uint8_t g_rtc_min = 32;
static bool g_vsync = true; // render the settings screen with the toggle ON
static bool g_boot_last = true; // BOOT LAST GAME row rendered ON
static bool g_nostalgic_boot = true; // NOSTALGIC BOOT row rendered ON
static bool g_te_pace = true;             // render the VSYNC row TE-locked...
static uint32_t g_te_period_us = 16667;   // ...showing "60HZ"

struct rom_entry_t {
	const char *name;
	const void *data;
	uint32_t size;
	uint32_t save_slot;
	uint16_t label_color; // RGBA4444 cart-label tint, 0 = default accent
};

constexpr uint32_t ROM_COUNT = 8;
// A couple of entries carry a label_color so the rendered boot.ppm exercises
// the per-ROM cart tint; the rest fall back to the accent (0).
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

#include "menu_draw.inc"

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

int main(int argc, char **argv) {
	const char *dir = argc > 1 ? argv[1] : ".";
	char path[512];

	draw_boot_menu(1, 82);
	snprintf(path, sizeof path, "%s/boot.ppm", dir);
	write_ppm(path);

	draw_settings_menu(SET_ROW_BRIGHT, 82);
	snprintf(path, sizeof path, "%s/settings.ppm", dir);
	write_ppm(path);

	draw_settings_menu(SET_ROW_HOUR, 82);
	snprintf(path, sizeof path, "%s/settings_clock.ppm", dir);
	write_ppm(path);

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
	apply_theme(1); // GRAPE
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

	return 0;
}

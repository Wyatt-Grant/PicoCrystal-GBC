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
static bool g_show_fps = true;     // draw_status_bar renders both header pieces
static bool g_show_battery = true;

#include "ui_draw.inc"

static uint8_t g_rtc_dow = 2; // TUE
static uint8_t g_rtc_hour = 14;
static uint8_t g_rtc_min = 32;
static bool g_vsync = true; // render the settings screen with the toggle ON

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
	{ "ORACLE OF SEASONS", nullptr, 1024 * 1024, 4, 0 },
	{ "POKEMON PRISM", nullptr, 2 * 1024 * 1024, 7, 0 },
	{ "KIRBY DREAM LAND 2", nullptr, 1024 * 1024, 2, 0 },
	{ "STAR OCEAN BLUE", nullptr, 4 * 1024 * 1024, 5, 0 },
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
	draw_status_bar(60, 82, false);
	snprintf(path, sizeof path, "%s/statusbar.ppm", dir);
	write_ppm(path);

	// The BATTERY PERCENTAGE toggle off: "<n>%" text gone from every
	// header (in-game, boot, settings), icon stays.
	g_show_battery = false;
	memset(_fb, 0, sizeof _fb);
	draw_status_bar(60, 82, false);
	snprintf(path, sizeof path, "%s/statusbar_nopct.ppm", dir);
	write_ppm(path);

	draw_settings_menu(SET_ROW_BATTERY, 82);
	snprintf(path, sizeof path, "%s/settings_nopct.ppm", dir);
	write_ppm(path);

	// A non-default theme with the THEME row selected -- every
	// accent-tinted element should recolor.
	g_show_battery = true;
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

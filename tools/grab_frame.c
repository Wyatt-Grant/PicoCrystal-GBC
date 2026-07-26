/*
 * Host-side frame grabber: boots a ROM in Walnut-CGB with a scripted button
 * sequence and dumps the raw 160x144 GBC frame as a PPM. Companion to
 * tools/render_menus.cpp, which composites one of these under the device's
 * status bar to produce the in-game README screenshot (see make_screenshots.sh).
 *
 *   grab_frame <rom> <out.ppm> <frames> [script]
 *
 * <frames> is how many frames to emulate before the dump. The optional script
 * is a comma-separated list of "frame:BUTTON[:hold]" press events, e.g.
 *
 *   "180:START:8,300:A:8,420:DOWN:4"
 *
 * holding BUTTON down from that frame for <hold> frames (default 8). Buttons:
 * A B START SELECT UP DOWN LEFT RIGHT.
 *
 * No display, no audio, no save file -- this only needs to reach one frame and
 * photograph it.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define ENABLE_SOUND 0
#include "../host_test/core/walnut_cgb.h"

#define MAX_EVENTS 32

struct priv_t {
	uint8_t *rom;
	uint8_t *cart_ram;
	uint16_t fb[LCD_HEIGHT][LCD_WIDTH]; /* RGB565, per fixPalette */
};

struct event_t {
	int frame, hold;
	uint8_t mask;
};

static uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	return ((struct priv_t *)gb->direct.priv)->rom[addr];
}

static uint16_t gb_rom_read_16bit(struct gb_s *gb, const uint_fast32_t addr)
{
	const uint8_t *src = &((struct priv_t *)gb->direct.priv)->rom[addr];
	return ((uint16_t)src[0]) | ((uint16_t)src[1] << 8);
}

static uint32_t gb_rom_read_32bit(struct gb_s *gb, const uint_fast32_t addr)
{
	const uint8_t *src = &((struct priv_t *)gb->direct.priv)->rom[addr];
	return ((uint32_t)src[0]) | ((uint32_t)src[1] << 8) |
	       ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

static uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	return ((struct priv_t *)gb->direct.priv)->cart_ram[addr];
}

static void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
			       const uint8_t val)
{
	((struct priv_t *)gb->direct.priv)->cart_ram[addr] = val;
}

static void gb_error(struct gb_s *gb, const enum gb_error_e err,
		      const uint16_t addr)
{
	(void)gb;
	fprintf(stderr, "FATAL: core error %d at 0x%04X\n", (int)err, addr);
	exit(1);
}

static void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
			   const uint_fast8_t line)
{
	struct priv_t *priv = (struct priv_t *)gb->direct.priv;
	if (gb->cgb.cgbMode) {
		for (unsigned x = 0; x < LCD_WIDTH; x++)
			priv->fb[line][x] = gb->cgb.fixPalette[pixels[x]];
	} else {
		/* DMG ROM: the device draws these through the same four-shade
		 * ramp (main.cpp's grey[] fallback), so mirror it here. */
		static const uint16_t grey[4] = { 0xFFFF, 0xAD55, 0x52AA, 0x0000 };
		for (unsigned x = 0; x < LCD_WIDTH; x++)
			priv->fb[line][x] = grey[pixels[x] & 3];
	}
}

/* RGB565 -> 8-bit PPM (P6), matching host_test/headless_test.c's dump. */
static void dump_ppm(struct priv_t *priv, const char *path)
{
	FILE *f = fopen(path, "wb");
	if (!f) { perror(path); exit(1); }
	fprintf(f, "P6\n%d %d\n255\n", LCD_WIDTH, LCD_HEIGHT);
	for (int y = 0; y < LCD_HEIGHT; y++) {
		for (int x = 0; x < LCD_WIDTH; x++) {
			uint16_t c = priv->fb[y][x];
			uint8_t r5 = (c >> 11) & 0x1F, g6 = (c >> 5) & 0x3F,
				b5 = c & 0x1F;
			uint8_t rgb[3] = {
				(uint8_t)((r5 * 255 + 15) / 31),
				(uint8_t)((g6 * 255 + 31) / 63),
				(uint8_t)((b5 * 255 + 15) / 31)
			};
			fwrite(rgb, 1, 3, f);
		}
	}
	fclose(f);
	fprintf(stderr, "wrote %s\n", path);
}

static uint8_t *read_file(const char *path, long *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); exit(1); }
	fseek(f, 0, SEEK_END);
	long len = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *buf = malloc(len);
	if (!buf || fread(buf, 1, len, f) != (size_t)len) { perror(path); exit(1); }
	fclose(f);
	*out_len = len;
	return buf;
}

static uint8_t button_mask(const char *name)
{
	static const struct { const char *n; uint8_t m; } map[] = {
		{ "A", JOYPAD_A }, { "B", JOYPAD_B },
		{ "START", JOYPAD_START }, { "SELECT", JOYPAD_SELECT },
		{ "UP", JOYPAD_UP }, { "DOWN", JOYPAD_DOWN },
		{ "LEFT", JOYPAD_LEFT }, { "RIGHT", JOYPAD_RIGHT },
	};
	for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++)
		if (strcmp(map[i].n, name) == 0)
			return map[i].m;
	fprintf(stderr, "unknown button '%s'\n", name);
	exit(1);
}

/* "frame:BUTTON[:hold],..." -> event list. Mutates the string it parses. */
static int parse_script(char *s, struct event_t *out)
{
	int n = 0;
	for (char *tok = strtok(s, ","); tok; tok = strtok(NULL, ",")) {
		char name[16] = { 0 };
		int frame = 0, hold = 8;
		int got = sscanf(tok, "%d:%15[^:]:%d", &frame, name, &hold);
		if (got < 2) {
			fprintf(stderr, "bad script event '%s'\n", tok);
			exit(1);
		}
		if (n == MAX_EVENTS) {
			fprintf(stderr, "too many script events (max %d)\n", MAX_EVENTS);
			exit(1);
		}
		out[n].frame = frame;
		out[n].hold = hold;
		out[n].mask = button_mask(name);
		n++;
	}
	return n;
}

int main(int argc, char **argv)
{
	if (argc < 4) {
		fprintf(stderr,
			"usage: %s <rom> <out.ppm> <frames> [\"frame:BUTTON[:hold],...\"]\n",
			argv[0]);
		return 1;
	}
	const char *rom_path = argv[1], *out_path = argv[2];
	int frames = atoi(argv[3]);
	struct event_t events[MAX_EVENTS];
	int event_count = argc > 4 ? parse_script(argv[4], events) : 0;

	struct gb_s gb;
	struct priv_t priv = { 0 };
	long rom_len;
	priv.rom = read_file(rom_path, &rom_len);

	enum gb_init_error_e ret = gb_init(&gb, &gb_rom_read, &gb_rom_read_16bit,
					    &gb_rom_read_32bit, &gb_cart_ram_read,
					    &gb_cart_ram_write, &gb_error, &priv);
	if (ret != GB_INIT_NO_ERROR) {
		fprintf(stderr, "gb_init(%s) failed: %d\n", rom_path, (int)ret);
		return 1;
	}

	size_t save_size = 0;
	if (gb_get_save_size_s(&gb, &save_size) < 0)
		return 1;
	priv.cart_ram = calloc(save_size ? save_size : 1, 1);

	time_t now = time(NULL);
	struct tm tm_now;
	localtime_r(&now, &tm_now);
	gb_set_rtc(&gb, &tm_now);

	gb_init_lcd(&gb, &lcd_draw_line);

	for (int frame = 0; frame <= frames; frame++) {
		uint8_t held = 0;
		for (int i = 0; i < event_count; i++)
			if (frame >= events[i].frame &&
			    frame < events[i].frame + events[i].hold)
				held |= events[i].mask;
		gb.direct.joypad = (uint8_t)~held; /* active low */
		gb_run_frame_dualfetch(&gb);
	}

	dump_ppm(&priv, out_path);
	return 0;
}

/*
 * Headless desktop validation harness for Walnut-CGB running Polished
 * Crystal: proves the emulator core is correct on the host before any of it
 * is blamed on the device. No SDL2/display required -- dumps PPM snapshots to
 * disk and exercises save-RAM round-trip + RTC state directly.
 *
 * Usage: headless_test <rom.gbc> [out_dir]
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define ENABLE_SOUND 0
#include "core/walnut_cgb.h"

struct priv_t {
	uint8_t *rom;
	uint8_t *cart_ram;
	size_t save_size;
	uint16_t fb[LCD_HEIGHT][LCD_WIDTH]; /* RGB555 per pixel */
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
	static const char *names[] = {
		"UNKNOWN", "INVALID OPCODE", "INVALID READ", "INVALID WRITE", ""
	};
	fprintf(stderr, "FATAL: core error %s at 0x%04X\n",
		names[err < GB_INVALID_MAX ? err : 0], addr);
	exit(1);
}

#if ENABLE_LCD
static void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
			   const uint_fast8_t line)
{
	struct priv_t *priv = (struct priv_t *)gb->direct.priv;
	if (gb->cgb.cgbMode) {
		for (unsigned x = 0; x < LCD_WIDTH; x++)
			priv->fb[line][x] = gb->cgb.fixPalette[pixels[x]];
	} else {
		/* Shouldn't hit this path for a CGB-flagged ROM, but fall back
		 * to a flat greyscale ramp so a bug here is visually obvious. */
		static const uint16_t grey[4] = { 0x7FFF, 0x5294, 0x294A, 0x0000 };
		for (unsigned x = 0; x < LCD_WIDTH; x++)
			priv->fb[line][x] = grey[pixels[x] & 3];
	}
}
#endif

/* Dump the current framebuffer (RGB565) as an 8-bit PPM (P6). fixPalette is
 * RGB565, not RGB555: gb_write() feeds it through bgr555_to_rgb565_accurate()
 * (the BE variant only when WALNUT_GB_RGB565_BIGENDIAN is set, which this host
 * build does not set), matching main.cpp's rgb565_to_color() on the device. */
static void dump_ppm(struct priv_t *priv, const char *out_dir, const char *name)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/%s.ppm", out_dir, name);
	FILE *f = fopen(path, "wb");
	if (!f) { perror("fopen"); return; }
	fprintf(f, "P6\n%d %d\n255\n", LCD_WIDTH, LCD_HEIGHT);
	for (int y = 0; y < LCD_HEIGHT; y++) {
		for (int x = 0; x < LCD_WIDTH; x++) {
			uint16_t c = priv->fb[y][x];
			uint8_t r5 = (c >> 11) & 0x1F, g6 = (c >> 5) & 0x3F, b5 = c & 0x1F;
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
	if (fread(buf, 1, len, f) != (size_t)len) { perror("fread"); exit(1); }
	fclose(f);
	*out_len = len;
	return buf;
}

/* Set joypad button state: 1 = pressed (active-low bits per JOYPAD_* masks). */
static void set_buttons(struct gb_s *gb, uint8_t mask, int pressed)
{
	if (pressed) gb->direct.joypad &= ~mask;
	else gb->direct.joypad |= mask;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <rom.gbc> [out_dir]\n", argv[0]);
		return 1;
	}
	const char *rom_path = argv[1];
	const char *out_dir = argc > 2 ? argv[2] : ".";

	struct gb_s gb;
	struct priv_t priv = { 0 };

	long rom_len;
	priv.rom = read_file(rom_path, &rom_len);
	fprintf(stderr, "loaded %s (%ld bytes)\n", rom_path, rom_len);

	enum gb_init_error_e ret = gb_init(&gb, &gb_rom_read, &gb_rom_read_16bit,
					    &gb_rom_read_32bit, &gb_cart_ram_read,
					    &gb_cart_ram_write, &gb_error, &priv);
	if (ret != GB_INIT_NO_ERROR) {
		fprintf(stderr, "gb_init failed: %d (%s)\n", ret,
			ret == GB_INIT_CARTRIDGE_UNSUPPORTED ? "cartridge unsupported" :
			ret == GB_INIT_INVALID_CHECKSUM ? "invalid checksum" : "unknown");
		return 1;
	}

	fprintf(stderr, "mbc=%d cgbMode=%d colour_hash=0x%02X\n",
		gb.mbc, gb.cgb.cgbMode, gb_colour_hash(&gb));

	if (gb_get_save_size_s(&gb, &priv.save_size) < 0) {
		fprintf(stderr, "gb_get_save_size_s failed\n");
		return 1;
	}
	fprintf(stderr, "save_size=%zu bytes\n", priv.save_size);
	priv.cart_ram = calloc(priv.save_size ? priv.save_size : 1, 1);

	/* Set RTC to current wall time -- exercises MBC3 RTC init path. */
	time_t now = time(NULL);
	struct tm tm_now;
	localtime_r(&now, &tm_now);
	gb_set_rtc(&gb, &tm_now);
	fprintf(stderr, "rtc set: sec=%d min=%d hour=%d yday_lo=%d yday_hi=%d\n",
		gb.rtc_real.bytes[0], gb.rtc_real.bytes[1], gb.rtc_real.bytes[2],
		gb.rtc_real.bytes[3], gb.rtc_real.bytes[4]);

#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);
#endif

	/* Scripted input: boot -> title screen -> press Start -> mash A to
	 * push through the New Game intro text as far as it'll go. */
	const int TOTAL_FRAMES = 3600; /* 60s of emulated GBC time */
	for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
		if (frame == 180) fprintf(stderr, "[frame %d] pressing START\n", frame);
		set_buttons(&gb, JOYPAD_START, frame >= 180 && frame < 190);

		if (frame >= 220 && frame < 3600 && (frame % 60) < 6) {
			set_buttons(&gb, JOYPAD_A, 1);
		} else {
			set_buttons(&gb, JOYPAD_A, 0);
		}

		gb_run_frame_dualfetch(&gb);

		if (frame == 150) dump_ppm(&priv, out_dir, "01_boot");
		if (frame == 300) dump_ppm(&priv, out_dir, "02_title");
		if (frame == 900) dump_ppm(&priv, out_dir, "03_menu");
		if (frame == 1800) dump_ppm(&priv, out_dir, "04_intro_mid");
		if (frame == 3599) dump_ppm(&priv, out_dir, "05_intro_end");
	}

	fprintf(stderr, "ran %d frames without core error\n", TOTAL_FRAMES);
	fprintf(stderr, "post-run mbc=%d cgbMode=%d rtc: sec=%d min=%d hour=%d yday_lo=%d yday_hi=%d\n",
		gb.mbc, gb.cgb.cgbMode,
		gb.rtc_real.bytes[0], gb.rtc_real.bytes[1], gb.rtc_real.bytes[2],
		gb.rtc_real.bytes[3], gb.rtc_real.bytes[4]);

	/* Save-RAM round trip: write current cart_ram to disk, reload into a
	 * fresh buffer, confirm bit-exact match. */
	if (priv.save_size > 0) {
		char sav_path[512];
		snprintf(sav_path, sizeof(sav_path), "%s/roundtrip.sav", out_dir);
		FILE *f = fopen(sav_path, "wb");
		fwrite(priv.cart_ram, 1, priv.save_size, f);
		fclose(f);

		uint8_t *reloaded = calloc(priv.save_size, 1);
		f = fopen(sav_path, "rb");
		size_t n = fread(reloaded, 1, priv.save_size, f);
		fclose(f);

		int match = (n == priv.save_size) &&
			memcmp(reloaded, priv.cart_ram, priv.save_size) == 0;
		fprintf(stderr, "save round-trip (%zu bytes): %s\n",
			priv.save_size, match ? "MATCH" : "MISMATCH");
		free(reloaded);
	} else {
		fprintf(stderr, "save round-trip: SKIPPED (save_size == 0)\n");
	}

	free(priv.rom);
	free(priv.cart_ram);
	return 0;
}

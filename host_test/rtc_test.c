/*
 * Host validation for WGB_RTC_EXTERNAL_TICK (wall-clock driven MBC3 RTC).
 *
 * Built with -DWGB_RTC_EXTERNAL_TICK=1, so the core's per-instruction cycle
 * tick is compiled out and counter.rtc_count is the host's microsecond
 * remainder. Covers:
 *   1. no internal ticking while frames run
 *   2. gb_rtc_tick_second rollover chain (carries + invalid-value guards)
 *   3. simulated ragged wall-clock feed (the main.cpp rtc_host_tick logic)
 *      including HALT discarding elapsed time
 *   4. MBC-mapped seconds write resets the sub-second remainder
 *
 * Usage: rtc_test <rom.gbc>   (needs an MBC3 ROM, e.g. polishedcrystal.gbc)
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define ENABLE_SOUND 0
#define ENABLE_LCD 0
#include "core/walnut_cgb.h"

static int failures = 0;

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		failures++; \
		fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
		fprintf(stderr, __VA_ARGS__); \
		fprintf(stderr, "\n"); \
	} \
} while (0)

struct priv_t {
	uint8_t *rom;
	uint8_t *cart_ram;
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
	fprintf(stderr, "FATAL: core error %d at 0x%04X\n", err, addr);
	exit(1);
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

static void set_rtc(struct gb_s *gb, uint8_t sec, uint8_t min, uint8_t hour,
		     uint8_t yday, uint8_t high)
{
	gb->rtc_real.reg.sec = sec;
	gb->rtc_real.reg.min = min;
	gb->rtc_real.reg.hour = hour;
	gb->rtc_real.reg.yday = yday;
	gb->rtc_real.reg.high = high;
}

/* --- 2. Rollover matrix (pure gb_rtc_tick_second, no ROM needed) --------- */
static void test_rollover(struct gb_s *gb)
{
	/* Minute carry. */
	set_rtc(gb, 59, 4, 3, 2, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.sec == 0 && gb->rtc_real.reg.min == 5,
	      "minute carry: sec=%d min=%d", gb->rtc_real.reg.sec, gb->rtc_real.reg.min);

	/* Hour + day carry: 23:59:59 -> day+1 00:00:00. */
	set_rtc(gb, 59, 59, 23, 6, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.sec == 0 && gb->rtc_real.reg.min == 0 &&
	      gb->rtc_real.reg.hour == 0 && gb->rtc_real.reg.yday == 7,
	      "day carry: %d:%d:%d yday=%d", gb->rtc_real.reg.hour,
	      gb->rtc_real.reg.min, gb->rtc_real.reg.sec, gb->rtc_real.reg.yday);

	/* Day 255 -> 256 sets day-counter bit 8. */
	set_rtc(gb, 59, 59, 23, 255, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.yday == 0 && gb->rtc_real.reg.high == 0x01,
	      "day bit 8: yday=%d high=0x%02X", gb->rtc_real.reg.yday, gb->rtc_real.reg.high);

	/* Day 511 -> 0 sets the overflow carry (bit 7) and clears bit 8. */
	set_rtc(gb, 59, 59, 23, 255, 0x01);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.yday == 0 && gb->rtc_real.reg.high == 0x80,
	      "day overflow: yday=%d high=0x%02X", gb->rtc_real.reg.yday, gb->rtc_real.reg.high);

	/* Invalid-value guards: 63/63/31 reset to 0 without carrying. */
	set_rtc(gb, 63, 10, 10, 10, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.sec == 0 && gb->rtc_real.reg.min == 10,
	      "invalid sec guard: sec=%d min=%d", gb->rtc_real.reg.sec, gb->rtc_real.reg.min);

	set_rtc(gb, 59, 63, 10, 10, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.min == 0 && gb->rtc_real.reg.hour == 10,
	      "invalid min guard: min=%d hour=%d", gb->rtc_real.reg.min, gb->rtc_real.reg.hour);

	set_rtc(gb, 59, 59, 31, 10, 0);
	gb_rtc_tick_second(gb);
	CHECK(gb->rtc_real.reg.hour == 0 && gb->rtc_real.reg.yday == 10,
	      "invalid hour guard: hour=%d yday=%d", gb->rtc_real.reg.hour, gb->rtc_real.reg.yday);
}

/* --- 3. Simulated wall clock --------------------------------------------- */
/* Mirror of main.cpp's rtc_host_tick(), with an injectable "now". */
static uint64_t fake_now_us;
static uint64_t last_us;

static void host_tick(struct gb_s *gb)
{
	uint64_t elapsed = fake_now_us - last_us;
	last_us = fake_now_us;
	if (gb->mbc != 3) return;
	if (gb->rtc_real.reg.high & 0x40) return; /* HALT: discard elapsed */
	gb->counter.rtc_count += elapsed;
	while (gb->counter.rtc_count >= 1000000) {
		gb->counter.rtc_count -= 1000000;
		gb_rtc_tick_second(gb);
	}
}

static void test_wall_clock(struct gb_s *gb)
{
	set_rtc(gb, 0, 0, 12, 3, 0);
	gb->counter.rtc_count = 0;
	fake_now_us = 0;
	last_us = 0;

	/* Feed exactly one hour of wall time in ragged chunks: normal frames,
	 * a 2-minute "settings menu pause" gap, and dropped-frame bursts. */
	const uint64_t TOTAL = 3600000000ull;
	uint64_t fed = 0;
	int i = 0;
	while (fed < TOTAL) {
		uint64_t chunk;
		if (i % 1000 == 500)
			chunk = 120000000ull;              /* 2 min menu pause */
		else if (i % 37 == 0)
			chunk = 95000ull;                  /* dropped-frame burst */
		else
			chunk = 16742ull;                  /* ~59.73 fps frame */
		if (chunk > TOTAL - fed)
			chunk = TOTAL - fed;
		fed += chunk;
		fake_now_us += chunk;
		host_tick(gb);
		i++;
	}
	CHECK(gb->rtc_real.reg.hour == 13 && gb->rtc_real.reg.min == 0 &&
	      gb->rtc_real.reg.sec == 0 && gb->counter.rtc_count == 0,
	      "1h ragged feed: %d:%02d:%02d rem=%u", gb->rtc_real.reg.hour,
	      gb->rtc_real.reg.min, gb->rtc_real.reg.sec,
	      (unsigned)gb->counter.rtc_count);

	/* HALT: elapsed time while halted is discarded, then time resumes. */
	gb->rtc_real.reg.high |= 0x40;
	fake_now_us += 30000000ull; /* 30s while halted */
	host_tick(gb);
	CHECK(gb->rtc_real.reg.sec == 0 && gb->counter.rtc_count == 0,
	      "HALT discard: sec=%d rem=%u", gb->rtc_real.reg.sec,
	      (unsigned)gb->counter.rtc_count);
	gb->rtc_real.reg.high &= ~0x40;
	fake_now_us += 2500000ull; /* 2.5s running again */
	host_tick(gb);
	CHECK(gb->rtc_real.reg.sec == 2 && gb->counter.rtc_count == 500000,
	      "resume after HALT: sec=%d rem=%u", gb->rtc_real.reg.sec,
	      (unsigned)gb->counter.rtc_count);
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <rom.gbc>\n", argv[0]);
		return 1;
	}

	static struct gb_s gb;
	struct priv_t priv = { 0 };
	long rom_len;
	priv.rom = read_file(argv[1], &rom_len);
	priv.cart_ram = calloc(0x8000, 1);

	enum gb_init_error_e ret = gb_init(&gb, &gb_rom_read, &gb_rom_read_16bit,
					    &gb_rom_read_32bit, &gb_cart_ram_read,
					    &gb_cart_ram_write, &gb_error, &priv);
	if (ret != GB_INIT_NO_ERROR) {
		fprintf(stderr, "gb_init failed: %d\n", ret);
		return 1;
	}
	CHECK(gb.mbc == 3, "test ROM must be MBC3, got mbc=%d", gb.mbc);

	/* --- 1. No internal ticking: run frames, RTC must not advance ---- */
	set_rtc(&gb, 30, 15, 6, 2, 0);
	gb.counter.rtc_count = 0;
	for (int frame = 0; frame < 600; frame++)
		gb_run_frame_dualfetch(&gb);
	CHECK(gb.counter.rtc_count == 0,
	      "internal tick leaked: rtc_count=%u", (unsigned)gb.counter.rtc_count);
	fprintf(stderr, "after 600 frames: rtc %d:%02d:%02d yday=%d high=0x%02X rem=%u\n",
		gb.rtc_real.reg.hour, gb.rtc_real.reg.min, gb.rtc_real.reg.sec,
		gb.rtc_real.reg.yday, gb.rtc_real.reg.high,
		(unsigned)gb.counter.rtc_count);

	test_rollover(&gb);
	test_wall_clock(&gb);

	/* --- 4. MBC-mapped seconds write resets the remainder ------------ */
	gb.counter.rtc_count = 987654;
	__gb_write(&gb, 0x0000, 0x0A); /* enable cart RAM/RTC access */
	__gb_write(&gb, 0x4000, 0x08); /* select RTC seconds register */
	__gb_write(&gb, 0xA000, 25);   /* write seconds */
	CHECK(gb.rtc_real.reg.sec == 25 && gb.counter.rtc_count == 0,
	      "sec write: sec=%d rem=%u", gb.rtc_real.reg.sec,
	      (unsigned)gb.counter.rtc_count);
	/* Non-seconds register write must NOT reset the remainder. */
	gb.counter.rtc_count = 123456;
	__gb_write(&gb, 0x4000, 0x09); /* select RTC minutes register */
	__gb_write(&gb, 0xA000, 42);
	CHECK(gb.rtc_real.reg.min == 42 && gb.counter.rtc_count == 123456,
	      "min write: min=%d rem=%u", gb.rtc_real.reg.min,
	      (unsigned)gb.counter.rtc_count);

	/* Latch: 0->1 on 0x6000 copies real -> latched. */
	set_rtc(&gb, 11, 22, 3, 44, 0x01);
	__gb_write(&gb, 0x6000, 0x00);
	__gb_write(&gb, 0x6000, 0x01);
	CHECK(memcmp(gb.rtc_latched.bytes, gb.rtc_real.bytes,
		     sizeof(gb.rtc_latched.bytes)) == 0,
	      "latch copy mismatch");

	if (failures) {
		fprintf(stderr, "rtc_test: %d FAILURE(S)\n", failures);
		return 1;
	}
	fprintf(stderr, "rtc_test: all checks passed\n");
	return 0;
}

#pragma once
#include <cstdint>
#include <cstddef>

// Flash-backed persistence for a game's cart RAM (the save file). Real
// cartridges keep SRAM continuously backed by a coin cell, so there's no
// discrete "save now" signal to hook -- this instead autosaves periodically
// whenever the RAM has been written since the last save.
//
// No reboot is needed after a flash write, despite what some RP2040 ports do:
// pico-sdk's hardware_flash.h docs are explicit that the requirement is just
// serialising against XIP flash reads on both cores during the erase/program
// call (via multicore_lockout + disabling interrupts) -- execution continues
// safely afterward. Rebooting per autosave would drop the running game back
// to its title screen. See save_storage.cpp for the guarded implementation.

// Saves are double-buffered across two flash slots (A/B ping-pong): each
// autosave writes the slot that is NOT currently newest, so a power-off during
// the non-atomic erase+program can never destroy the last good save -- the
// interrupted slot is simply rejected on load and the intact one is used.

// Each bootable ROM (see the generated rom_data.hpp's rom_catalog) gets its
// own A/B slot pair, selected by `rom_save_slot` (that ROM's
// rom_entry_t::save_slot, assigned by tools/gen_rom_data.py and pinnable via
// assets/roms.json). Without this, switching titles would load/overwrite
// whichever game last wrote the shared slot pair -- silently corrupting one
// game's save with another's cart_ram layout. MAX_ROM_SAVE_SLOTS bounds the
// flash reserved for this up front: 14 slot regions plus the settings sector
// is the most that fits in the 1MB between the linker script's 15MB FLASH
// (code + ROMs) region and the top of the 16MB chip. Keep it in sync with
// gen_rom_data.py's copy, and prefer never changing it -- the settings
// sector's offset derives from it, so a change strands stored settings
// (they harmlessly reset to defaults on next boot). Historical note: was 8
// until 2026-07; that bump reset settings once but left game saves intact
// (their offsets depend only on their own slot number).
constexpr uint32_t MAX_ROM_SAVE_SLOTS = 14;

// Loads the newest valid saved game (highest header sequence with a matching
// checksum) into cart_ram. Leaves cart_ram untouched (already zero-initialized,
// i.e. a fresh save) if neither slot holds a valid save. Call once at startup,
// after the boot ROM selection is known.
void save_storage_load(uint8_t *cart_ram, size_t size, uint32_t rom_save_slot);

// Call from gb_cart_ram_write() to mark the save data as changed.
void save_storage_mark_dirty();

// --- Autosave pacing ---------------------------------------------------------
// Flash endurance is finite (~100k erase cycles per sector) and some titles
// treat cart RAM as scratch work RAM, dirtying it continuously -- with a fixed
// 3s interval that is a commit (one full slot erase + program) every 3 seconds
// for as long as the game runs. This throttle is the "buffer": no matter how
// often the game writes, at most one flash commit per `ms`. The A/B ping-pong
// halves the per-sector rate again (each slot is erased every other commit).
//
// `ms` == SAVE_INTERVAL_MANUAL disables autosaving entirely: dirty data waits
// (the target slot is still pre-erased, so the commit itself is quick) until
// save_storage_request_save() is called -- main.cpp binds that to the in-game
// X+B chord. Manual mode means an unsaved session is lost on power-off, and it
// widens the window in which only one of the two slots holds a valid save; both
// are the deliberate trade for touching flash only when asked.
constexpr uint32_t SAVE_INTERVAL_MANUAL = 0;

// Sets the minimum spacing between flash commits. Safe to call any time
// (including mid-save -- it only gates the start of the next commit).
void save_storage_set_interval_ms(uint32_t ms);

// True from a save_storage_request_save() until that commit actually starts
// programming -- the requested write is still waiting on the slot pre-erase or
// on the game to go quiet. main.cpp keeps the LED's save blink up across this
// window, so a manual save is signposted from the moment it's asked for rather
// than only once the flash writes begin (which can be a second or two later).
// False when there was nothing dirty to write, i.e. the request is a no-op.
bool save_storage_pending();

// Requests a commit at the next opportunity, bypassing the interval. Used by
// the manual-save chord and by edits that must reach flash immediately (an
// in-game RTC change). No-ops when there is nothing dirty to write.
void save_storage_request_save();

// --- MBC3 RTC persistence ------------------------------------------------
// The MBC3 clock is snapshotted into each committed save (freeze-while-off:
// the board has no battery-backed RTC, so on boot the clock resumes exactly
// where the last save left it; in-game time lags by however long the device
// was off, nudged via the settings menu). The record rides in the otherwise
// unused space of the save header sector, so the save payload format is
// unchanged and saves round-trip with older firmware in both directions.
struct save_rtc_t {
	uint8_t reg[5];     // rtc_real.bytes: sec, min, hour, day low,
	                    // high (bit0 = day bit 8, 0x40 = HALT, 0x80 = carry)
	uint32_t subsec_us; // sub-second remainder, 0..999999
};

// Registers the snapshot source, called at commit time (just before the
// header/commit-record program). Return false to omit the record (non-MBC3
// title).
void save_storage_set_rtc_provider(bool (*fn)(save_rtc_t &out));

// True iff the slot chosen by save_storage_load() carried a valid RTC
// record; fills `out`. Old saves (pre-RTC-record) return false -- callers
// fall back to seeding the clock from the staged settings.
bool save_storage_load_rtc(save_rtc_t &out);

// Call once per frame from the main loop. Autosaves cart_ram to flash if
// it's dirty and the autosave interval has elapsed; otherwise returns
// immediately or advances an in-progress save. The save is incremental --
// at most ONE small flash op per call (a single 4KB sector erase, ~45ms, or a
// page-program burst, ~1.4ms), each briefly pausing core1 (audio) and
// interrupts -- doing the whole slot in one gulp instead freezes both cores
// for ~half a second, felt as a stutter a few seconds after starting a game
// or saving in-game. The header (magic+CRC+seq) is written
// last as the commit record, so a power-off mid-save still falls back to the
// previous good slot on load.
void save_storage_poll(const uint8_t *cart_ram, size_t size);

// True while save_storage_poll() is mid-commit (programming the new snapshot
// to flash, before the header/commit record lands). main.cpp blinks the LED
// blue/magenta for the duration, so a power-off during the write window is
// always signposted.
bool save_storage_saving();

// --- Device settings ---------------------------------------------------------
// The settings menu's values (brightness, volume, staged RTC), persisted in
// one dedicated flash sector directly below the ROM save reservations so
// they survive power cycles. Single-copy (no A/B ping-pong): a torn write
// just falls back to defaults, which for a handful of preferences is an
// acceptable trade for a sector of flash.

struct device_settings_t {
	uint8_t brightness; // backlight percent, BRIGHTNESS_MIN..100
	uint8_t rtc_dow;    // staged RTC weekday, 0..6 (Sunday == 0)
	uint8_t rtc_hour;   // 0..23
	uint8_t rtc_min;    // 0..59
	uint16_t volume;    // audio_output native range, 0..800
	uint8_t vsync;        // 0/1: tear-free TE-synchronised flips (main.cpp g_vsync)
	uint8_t status_bar;   // status_bar_mode_t (main.cpp g_status_bar): 0 FPS+PCT,
			      // 1 FPS, 2 PERCENT, 3 ICON, 4 FULLSCREEN (no in-game
			      // header, game frame centered)
	uint8_t theme;        // UI accent theme: a UI_THEMES index (main.cpp
			      // g_theme); apply_theme() falls back to 0 == MINT
			      // if it's out of range. Was reserved0, always stored
			      // 0 == MINT, so old records decode unchanged.
	uint8_t dark_mode;    // 1 = dark, 0 = light appearance (main.cpp g_dark_mode).
			      // New field: grows the struct, so old records fail
			      // the CRC and reset settings to defaults (dark) once.
	uint8_t boot_last;    // 0/1: skip the boot menu and auto-boot the last
			      // game (main.cpp g_boot_last)
	uint8_t last_slot;    // save_slot of the last-booted ROM -- the stable
			      // per-game id, not the catalog index, which
			      // reshuffles when the ROM list is regenerated.
			      // 0xFF = no game booted yet (main.cpp g_last_slot)
	uint8_t nostalgic_boot; // 0/1: Game Boy-style boot splash + chimes when a
			      // game launches (main.cpp g_nostalgic_boot)
	uint8_t save_interval; // index into main.cpp's SAVE_INTERVAL_SECS
			      // (g_save_interval): 0 = MANUAL, then 3/5/10/30/
			      // 60/120 seconds. Stored as the index, not the
			      // seconds, so the option list can be retuned
			      // without reinterpreting stored records.
	uint8_t color_filter; // picture mode: an index into main.cpp's
			      // color_filter_t (g_color_filter) -- 0 = OFF,
			      // 1 = GBC panel emulation, then VIVID/PASTEL/
			      // MONO/... Was a 0/1 toggle before the mode list
			      // existed, and 0/1 still mean the same two
			      // things, so old records decode unchanged.
};
// Growing this struct invalidates the previously stored record (the CRC is
// computed over the whole payload), so settings reset to defaults once on
// first boot after an upgrade. Acceptable for a handful of preferences.

// Reads the persisted settings into `out`. Returns false (leaving `out`
// untouched) if none have ever been stored or the record is corrupt.
bool save_storage_settings_load(device_settings_t &out);

// Persists `s`, synchronously (one sector erase + page program, ~50ms,
// pausing both cores). Call from the settings menu on close -- never from
// the per-frame loop. No-ops if `s` matches what flash already holds.
void save_storage_settings_store(const device_settings_t &s);


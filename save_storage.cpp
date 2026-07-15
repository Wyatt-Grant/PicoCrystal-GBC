#include "save_storage.hpp"

#include "flash_sliced.hpp"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/time.h"

namespace {

constexpr uint32_t MAGIC = 0x50434352; // "PCCR"
constexpr uint32_t AUTOSAVE_INTERVAL_MS = 3000;

// Each save slot reserves 9 flash sectors (36864 bytes): 1 sector for a small
// header, 8 sectors (32768 bytes) for Polished Crystal's MBC3 save (4 banks x
// 8KB, confirmed via constants/ram_constants.asm and the Milestone 2 host_test
// harness).
constexpr uint32_t SLOT_SECTORS = 9;
constexpr uint32_t SLOT_BYTES = SLOT_SECTORS * FLASH_SECTOR_SIZE;

// Two slots, written alternately (A/B ping-pong). A flash write is NOT atomic:
// the slot is erased before it is refilled, and on a handheld the normal way to
// stop playing is to flip the power switch -- which can land inside that
// erase+program window (a save happens every few seconds). With a single slot
// that meant the only copy of the save was gone and the game came back to a
// fresh file ("save deleted"). Instead we always write to the slot that is NOT
// currently the newest valid one, so the previous good save is left completely
// untouched until the new one is fully committed. A torn write only corrupts
// the older slot; save_storage_load() then falls back to the still-intact newer
// one (or vice versa). A monotonically increasing `seq` in the header records
// which slot is newest.
constexpr uint32_t NUM_SLOTS = 2;
constexpr uint32_t RESERVED_BYTES_PER_ROM = NUM_SLOTS * SLOT_BYTES;

// --- Incremental-save pacing -------------------------------------------------
//
// Erasing and programming flash stalls EVERYTHING: XIP instruction fetch is
// unavailable on both cores for the duration, so each flash op runs with core1
// locked out and interrupts disabled. Done as one synchronous gulp (the
// original design), a save was 9 sector erases (~45ms typical each on the
// W25Q128) plus ~129 page programs -- roughly half a second of hard freeze,
// felt as the "stutter a few seconds after starting/saving a game". So
// save_storage_poll() is now a state machine that issues AT MOST ONE small
// flash op per call (per emulated frame):
//
//   IDLE -> ERASING      first cart-RAM write after a save (dirty) starts a
//                        pre-erase of the target slot, one 4KB sector at a
//                        time. On erase-suspend-capable flash (see
//                        flash_sliced.*) each sector erase runs for only a
//                        ~1.5ms on-time per poll, then suspends so both cores
//                        keep running between polls -- the erase no longer drops
//                        a frame at all. Otherwise it falls back to one blocking
//                        ~45ms sector erase every ERASE_SPACING_MS (the old
//                        behaviour, still the one op that drops a frame or two).
//                        Already-blank sectors are detected via XIP and skipped
//                        for free, so in the common power-cycle case (slot
//                        pre-erased by the previous session) this phase is a
//                        no-op.
//   ERASING -> READY     slot fully blank, waiting for the autosave to be due.
//   READY -> PROGRAMMING interval elapsed and the game has gone quiet (no
//                        cart-RAM writes for QUIET_POLLS_REQUIRED polls, so we
//                        don't snapshot mid-save-burst; capped by
//                        QUIET_WAIT_MAX_MS for titles that use SRAM as work
//                        RAM and are never quiet).
//   PROGRAMMING          one PROGRAM_CHUNK_BYTES page-program (~3ms typical)
//                        per poll; the CRC is accumulated over the exact bytes
//                        sent to flash, then the header -- magic + CRC + seq,
//                        i.e. the commit record -- is programmed LAST. If the
//                        game writes cart RAM mid-programming the committed
//                        snapshot may mix bytes from different frames, but its
//                        CRC still matches the flash contents (loadable, and
//                        the game's own in-save checksums cover the rest), and
//                        `dirty` is already set again so a clean snapshot
//                        follows one interval later.
//
// With sliced erase the worst per-frame flash stall is now one erase on-time
// (~1.5ms, set by ERASE_ON_US in flash_sliced.cpp) or a PROGRAM_CHUNK_BYTES
// program (~1.4ms) -- no dropped frames. On non-suspend-capable flash the
// fallback still pays one ~45ms sector erase per ERASE_SPACING_MS.
constexpr uint32_t ERASE_SPACING_MS = 200;   // fallback (non-sliced) path only
constexpr uint32_t PROGRAM_CHUNK_BYTES = 2 * FLASH_PAGE_SIZE; // 512B, ~1.4ms typ
constexpr uint32_t QUIET_POLLS_REQUIRED = 4;
constexpr uint32_t QUIET_WAIT_MAX_MS = 2000;

// This sits well past our own code+ROM (~2.17MB) and inside the linker script's
// nominally-unused "FAT" region (see the plan's Open Risks) -- nothing else in
// this build touches it. Note rom_save_slot 0's offset is unchanged from the
// old single-ROM layout, so an existing Polished Crystal save written by a
// previous build is still picked up on upgrade (its header reads back with
// seq == 0, which load happily accepts as "the only valid slot").
//
// Each rom_save_slot (see the generated rom_data.hpp) gets its own RESERVED_BYTES_PER_ROM
// region, stacked downward from the top of flash, so different ROMs' saves
// can never alias the same sectors.
uint32_t rom_save_slot = 0; // set by save_storage_load(), reused by poll()

uint32_t save_flash_offset() {
	return PICO_FLASH_SIZE_BYTES - (rom_save_slot + 1) * RESERVED_BYTES_PER_ROM;
}

// Sanity bound: MAX_ROM_SAVE_SLOTS worth of reservations must stay well clear
// of the 15MB code+ROM region (memmap_picosystem.ld's FLASH). Comfortably
// true today (288KB reserved vs. headroom below it) -- this just catches the
// reservation ever being widened past what the chip has.
static_assert(MAX_ROM_SAVE_SLOTS * RESERVED_BYTES_PER_ROM < 4 * 1024 * 1024,
	      "rom save reservation has grown into the FAT-region assumption above");

struct save_header_t {
	uint32_t magic;
	uint32_t size;
	uint32_t crc32;
	uint32_t seq; // higher == newer; 0 also accepted (pre-ping-pong saves)
};

// MBC3 RTC snapshot, stored at a fixed offset in the header sector's first
// page -- programmed in the same flash op as the commit record, so a save
// either carries a coherent (payload, RTC) pair or is rejected whole. Old
// saves hold 0x00 here (header_page is zero-filled), which fails the magic
// check and reads as "no record". Independently CRC'd so it can be dropped
// or extended without touching the save payload format.
constexpr uint32_t RTC_RECORD_OFFSET = 32; // within the header page
constexpr uint32_t RTC_MAGIC = 0x50435254; // "PCRT"

struct rtc_record_t {
	uint32_t magic;
	uint8_t reg[5];
	uint8_t pad[3];     // keeps subsec_us aligned; written as 0
	uint32_t subsec_us; // 0..999999
	uint32_t crc32;     // over the 16 bytes above (magic..subsec_us)
};
static_assert(sizeof(rtc_record_t) == 20, "record layout is on-flash ABI");
static_assert(RTC_RECORD_OFFSET >= sizeof(save_header_t) &&
	      RTC_RECORD_OFFSET + sizeof(rtc_record_t) <= FLASH_PAGE_SIZE,
	      "RTC record must fit in the header page after the header");

bool (*rtc_provider)(save_rtc_t &out) = nullptr;
save_rtc_t loaded_rtc;
bool loaded_rtc_valid = false;

// Standard bit-wise CRC32 (IEEE 802.3 polynomial), streamable so the
// programming phase can accumulate it chunk by chunk. Not table-based -- the
// biggest single call is one 1KB chunk (~0.3ms), not a per-frame hot path, so
// simplicity wins over speed here.
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
	}
	return crc;
}

uint32_t crc32(const uint8_t *data, size_t len) {
	return ~crc32_update(0xFFFFFFFFu, data, len);
}

volatile bool dirty = false;       // unsaved cart-RAM changes exist
volatile bool write_pulse = false; // any cart-RAM write since the last poll
uint32_t last_save_ms = 0;
uint32_t current_seq = 0; // seq of the newest valid save on flash (0 == none)
uint32_t next_slot = 0;   // slot index the next autosave will erase+program

enum class save_state_t { IDLE, ERASING, READY, PROGRAMMING };
save_state_t state = save_state_t::IDLE;
uint32_t erase_sector = 0;   // next sector index within next_slot to erase
uint32_t last_erase_ms = 0;  // paces fallback sector erases ERASE_SPACING_MS apart
bool erase_in_flight = false; // a sliced erase of erase_sector is suspended on-chip
uint32_t quiet_polls = 0;    // polls since the last cart-RAM write
bool due_waiting = false;    // save is due but gated on quiet_polls
uint32_t due_wait_start_ms = 0;
uint32_t prog_offset = 0;    // bytes of cart_ram programmed so far
uint32_t prog_crc = 0;       // running CRC of the bytes actually programmed
uint32_t pending_seq = 0;    // seq the in-flight save will commit with

// Just the header page (256 bytes, flash_range_program()'s minimum
// granularity) -- cart_ram is programmed directly from its own array below,
// no staging copy needed. This used to be a combined 36KB
// (header-sector + cart_ram-sectors) staging buffer; freeing that up is what
// makes moving __gb_step_cpu (the ~25KB opcode dispatcher, see
// core/walnut_cgb.h) into RAM affordable -- that's a much bigger win than
// the copy this buffer used to avoid duplicating.
uint8_t header_page[FLASH_PAGE_SIZE];

uint32_t slot_offset(uint32_t slot) {
	return save_flash_offset() + slot * SLOT_BYTES;
}

bool sector_is_blank(uint32_t off) {
	// Safe to read via XIP: flash_range_erase/program flush the XIP cache
	// before re-enabling it, so nothing stale survives our own writes.
	const uint32_t *p = (const uint32_t *)(XIP_BASE + off);
	for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / sizeof(uint32_t); i++)
		if (p[i] != 0xFFFFFFFFu)
			return false;
	return true;
}

// Pause core1 (if it's actually running -- it isn't while audio is
// disabled, see main.cpp's ENABLE_SOUND) and disable our own interrupts
// for the duration of the erase/program: required because flash is
// unreadable (including for instruction fetch) on both cores while it's
// being erased/programmed. multicore_lockout_start_blocking() blocks
// forever waiting for an acknowledgement from core1 if core1 never
// called multicore_lockout_victim_init() -- confirmed this exact hang:
// core1 is only launched by audio_output_init(), which isn't called at
// all with audio disabled, so an unconditional lockout call
// froze the very first autosave. multicore_lockout_victim_is_initialized()
// is pico-sdk's supported way to check before calling it.
//
// No reboot needed afterward: flash_range_erase/program restore XIP mode
// themselves before returning (see hardware/flash.h's documented
// behaviour), so execution just continues normally.
void flash_locked_erase(uint32_t off, size_t bytes) {
	bool core1_locked_out = multicore_lockout_victim_is_initialized(1);
	if (core1_locked_out)
		multicore_lockout_start_blocking();
	uint32_t ints = save_and_disable_interrupts();

	flash_range_erase(off, bytes);

	restore_interrupts(ints);
	if (core1_locked_out)
		multicore_lockout_end_blocking();
}

void flash_locked_program(uint32_t off, const uint8_t *data, size_t bytes) {
	bool core1_locked_out = multicore_lockout_victim_is_initialized(1);
	if (core1_locked_out)
		multicore_lockout_start_blocking();
	uint32_t ints = save_and_disable_interrupts();

	flash_range_program(off, data, bytes);

	restore_interrupts(ints);
	if (core1_locked_out)
		multicore_lockout_end_blocking();
}

// Drive any suspended sliced erase (ERASING state) to completion. A suspended
// erase leaves the chip in a state where NO other flash op is legal -- neither
// flash_range_erase/program above nor a fresh sliced erase -- so any code path
// that is about to touch flash outside the ERASING state must call this first.
// It also keeps the state machine's bookkeeping (erase_sector) consistent so
// the next poll continues cleanly.
void finish_pending_erase() {
	if (!erase_in_flight)
		return;
	flash_sliced_erase_finish();
	erase_in_flight = false;
	erase_sector++;
}

} // namespace

void save_storage_load(uint8_t *cart_ram, size_t size, uint32_t save_slot) {
	// Probe the flash for erase-suspend support once, at boot, before any
	// autosave can start a (sliced) erase -- the JEDEC probe uses flash_do_cmd,
	// which is illegal once an erase is suspended.
	flash_sliced_init();

	rom_save_slot = save_slot;
	loaded_rtc_valid = false;

	int best_slot = -1;
	uint32_t best_seq = 0;

	for (uint32_t slot = 0; slot < NUM_SLOTS; slot++) {
		const uint8_t *flash = (const uint8_t *)(XIP_BASE + slot_offset(slot));
		save_header_t hdr;
		__builtin_memcpy(&hdr, flash, sizeof(hdr));

		if (hdr.magic != MAGIC || hdr.size != size)
			continue; // empty slot, or size mismatch (different ROM/build)

		const uint8_t *saved_data = flash + FLASH_SECTOR_SIZE;
		if (crc32(saved_data, size) != hdr.crc32)
			continue; // corrupt/torn write in this slot -- skip it

		// Pick the newest valid slot. In steady state the two seqs differ by
		// exactly 1, so a plain comparison is unambiguous; seq wraparound would
		// take 2^32 autosaves (millennia) and never occurs in practice.
		if (best_slot < 0 || hdr.seq >= best_seq) {
			best_slot = (int)slot;
			best_seq = hdr.seq;
		}
	}

	if (best_slot < 0) {
		// No valid save -- stay zeroed (a fresh file). First autosave writes slot 0.
		current_seq = 0;
		next_slot = 0;
		return;
	}

	const uint8_t *saved_data =
	    (const uint8_t *)(XIP_BASE + slot_offset(best_slot) + FLASH_SECTOR_SIZE);
	__builtin_memcpy(cart_ram, saved_data, size);

	// Pick up the RTC record committed with this slot, if any (absent on
	// saves written by older firmware -- the zero-filled bytes fail the
	// magic check).
	rtc_record_t rec;
	__builtin_memcpy(&rec, (const uint8_t *)(XIP_BASE + slot_offset(best_slot) +
						 RTC_RECORD_OFFSET), sizeof(rec));
	if (rec.magic == RTC_MAGIC && rec.subsec_us < 1000000 &&
	    crc32((const uint8_t *)&rec, offsetof(rtc_record_t, crc32)) == rec.crc32) {
		__builtin_memcpy(loaded_rtc.reg, rec.reg, sizeof(loaded_rtc.reg));
		loaded_rtc.subsec_us = rec.subsec_us;
		loaded_rtc_valid = true;
	}

	current_seq = best_seq;
	// Write the OTHER slot next, so this loaded save survives the next write.
	next_slot = ((uint32_t)best_slot + 1) % NUM_SLOTS;
}

void save_storage_set_rtc_provider(bool (*fn)(save_rtc_t &out)) {
	rtc_provider = fn;
}

bool save_storage_load_rtc(save_rtc_t &out) {
	if (!loaded_rtc_valid)
		return false;
	out = loaded_rtc;
	return true;
}

// RAM-resident: called (via main.cpp's WGB_CART_RAM_WRITE) from the
// RAM-resident __gb_write on every cart-RAM byte the game writes -- some
// titles use battery-backed SRAM as general work RAM, making this genuinely
// hot, and a flash-resident body would drag that path through XIP.
void __not_in_flash_func(save_storage_mark_dirty)() {
	dirty = true;
	write_pulse = true;
}

bool save_storage_saving() {
	return state == save_state_t::PROGRAMMING;
}

void save_storage_poll(const uint8_t *cart_ram, size_t size) {
	bool wrote = write_pulse;
	write_pulse = false;
	if (wrote)
		quiet_polls = 0;
	else if (quiet_polls < QUIET_POLLS_REQUIRED)
		quiet_polls++;

	uint32_t now = to_ms_since_boot(get_absolute_time());

	switch (state) {
	case save_state_t::IDLE:
		if (!dirty)
			return;
		// A save is coming: start pre-erasing the target slot now so the
		// erase cost is spread out (and usually long done) before the
		// autosave interval expires. Only reached once dirty is set, so a
		// slot holding an older save is never destroyed unless a newer save
		// is actually on its way -- the same guarantee as the old design.
		erase_sector = 0;
		last_erase_ms = now - ERASE_SPACING_MS; // first erase is immediate
		state = save_state_t::ERASING;
		[[fallthrough]];

	case save_state_t::ERASING:
		// Advance a sliced erase already suspended on-chip before touching any
		// other sector (its contents are undefined while suspended, so no
		// sector_is_blank() until it completes). One on-time per poll.
		if (erase_in_flight) {
			if (flash_sliced_erase_step()) {
				erase_in_flight = false;
				erase_sector++;
			}
			return; // one erase on-time per poll
		}
		// Skipping already-blank sectors is nearly free (an XIP read).
		while (erase_sector < SLOT_SECTORS &&
		       sector_is_blank(slot_offset(next_slot) + erase_sector * FLASH_SECTOR_SIZE))
			erase_sector++;
		if (erase_sector < SLOT_SECTORS) {
			uint32_t off = slot_offset(next_slot) + erase_sector * FLASH_SECTOR_SIZE;
			if (flash_sliced_supported()) {
				// Sliced: run this sector's first on-time now, then step it
				// across following polls. Handles the "done in one on-time" case.
				if (flash_sliced_erase_begin(off))
					erase_sector++;
				else
					erase_in_flight = true;
			} else {
				// Fallback: one blocking ~45ms sector erase, paced.
				if (now - last_erase_ms < ERASE_SPACING_MS)
					return;
				flash_locked_erase(off, FLASH_SECTOR_SIZE);
				erase_sector++;
				last_erase_ms = now;
			}
			return; // at most one sector's worth of flash work per poll
		}
		state = save_state_t::READY;
		due_waiting = false;
		[[fallthrough]];

	case save_state_t::READY:
		if (!dirty || now - last_save_ms < AUTOSAVE_INTERVAL_MS)
			return;
		if (quiet_polls < QUIET_POLLS_REQUIRED) {
			// The game is actively writing SRAM (likely mid save-burst);
			// wait for it to finish so the snapshot is frame-consistent --
			// but not forever, for titles that treat SRAM as work RAM.
			if (!due_waiting) {
				due_waiting = true;
				due_wait_start_ms = now;
			}
			if (now - due_wait_start_ms < QUIET_WAIT_MAX_MS)
				return;
		}
		pending_seq = current_seq + 1;
		if (pending_seq == 0)
			pending_seq = 1; // never store 0 (reserved for "none"); only matters on wrap
		// Cleared before programming: any write during the multi-frame
		// program phase re-sets it, queueing a fresh save one interval later.
		dirty = false;
		prog_offset = 0;
		prog_crc = 0xFFFFFFFFu;
		state = save_state_t::PROGRAMMING;
		[[fallthrough]];

	case save_state_t::PROGRAMMING:
		if (prog_offset < size) {
			uint32_t chunk = PROGRAM_CHUNK_BYTES;
			if (chunk > size - prog_offset)
				chunk = (uint32_t)(size - prog_offset);
			// CRC exactly the bytes handed to flash, so the header always
			// validates the slot's real contents even if cart_ram changes
			// between chunks.
			prog_crc = crc32_update(prog_crc, cart_ram + prog_offset, chunk);
			flash_locked_program(slot_offset(next_slot) + FLASH_SECTOR_SIZE + prog_offset,
					     cart_ram + prog_offset, chunk);
			prog_offset += chunk;
			return;
		}

		// All data pages are on flash -- program the header last. The header
		// is the commit record: until it lands, load ignores this slot
		// entirely, so a power-off anywhere in the phases above just falls
		// back to the previous save.
		{
			save_header_t hdr = { MAGIC, (uint32_t)size, ~prog_crc, pending_seq };
			__builtin_memset(header_page, 0, sizeof(header_page));
			__builtin_memcpy(header_page, &hdr, sizeof(hdr));
			// Snapshot the MBC3 clock into the same page program as the
			// commit record (see rtc_record_t above). Reading the live
			// clock here is coherent: poll() only runs on core0 between
			// emulated frames, never concurrent with the CPU step that
			// mutates it.
			save_rtc_t rtc;
			if (rtc_provider && rtc_provider(rtc)) {
				rtc_record_t rec = {};
				rec.magic = RTC_MAGIC;
				__builtin_memcpy(rec.reg, rtc.reg, sizeof(rec.reg));
				rec.subsec_us = rtc.subsec_us;
				rec.crc32 = crc32((const uint8_t *)&rec,
						  offsetof(rtc_record_t, crc32));
				__builtin_memcpy(header_page + RTC_RECORD_OFFSET, &rec, sizeof(rec));
			}
			flash_locked_program(slot_offset(next_slot), header_page, sizeof(header_page));
		}

		current_seq = pending_seq;
		next_slot = (next_slot + 1) % NUM_SLOTS;
		last_save_ms = now;
		state = save_state_t::IDLE;
		return;
	}
}

// --- Device settings ---------------------------------------------------------

namespace {

constexpr uint32_t SETTINGS_MAGIC = 0x50435354; // "PCST"

// One sector directly below the lowest possible ROM save reservation. The
// static_assert above already keeps the whole stack (this sector included)
// far clear of the code+ROM region.
constexpr uint32_t settings_flash_offset() {
	return PICO_FLASH_SIZE_BYTES -
	       MAX_ROM_SAVE_SLOTS * RESERVED_BYTES_PER_ROM - FLASH_SECTOR_SIZE;
}

struct settings_record_t {
	uint32_t magic;
	uint32_t crc32; // of the payload bytes
	device_settings_t payload;
};

} // namespace

bool save_storage_settings_load(device_settings_t &out) {
	settings_record_t rec;
	__builtin_memcpy(&rec, (const void *)(XIP_BASE + settings_flash_offset()),
			 sizeof(rec));
	if (rec.magic != SETTINGS_MAGIC ||
	    crc32((const uint8_t *)&rec.payload, sizeof(rec.payload)) != rec.crc32)
		return false;
	out = rec.payload;
	return true;
}

void save_storage_settings_store(const device_settings_t &s) {
	// Spare the erase when nothing changed (settings menu closed without
	// edits, or re-set to the stored values).
	device_settings_t cur;
	if (save_storage_settings_load(cur) &&
	    __builtin_memcmp(&cur, &s, sizeof(s)) == 0)
		return;

	// The in-game settings menu keeps save_storage_poll() running, so an
	// autosave sector erase can be suspended on-chip right now. No other flash
	// op is legal while an erase is suspended -- drive it to completion first.
	finish_pending_erase();

	settings_record_t rec = {
		SETTINGS_MAGIC,
		crc32((const uint8_t *)&s, sizeof(s)),
		s,
	};
	uint8_t page[FLASH_PAGE_SIZE];
	__builtin_memset(page, 0xFF, sizeof(page));
	__builtin_memcpy(page, &rec, sizeof(rec));

	flash_locked_erase(settings_flash_offset(), FLASH_SECTOR_SIZE);
	flash_locked_program(settings_flash_offset(), page, sizeof(page));
}

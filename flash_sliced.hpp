#pragma once
#include <cstdint>

// Sliced 4KB sector erase using the flash chip's Erase Suspend (0x75) / Erase
// Resume (0x7A) commands (W25Q family). A normal flash_range_erase() holds XIP
// down for the WHOLE erase (~45ms typ on the W25Q128) -- both cores stall the
// entire time because instruction fetch runs from flash, which is what causes
// the audible/visible save stutter. Instead we run the erase in short slices
// (ERASE_ON_US, ~1.5ms -- see flash_sliced.cpp): issue/resume the erase, let
// it run for one slice, then SUSPEND it and
// restore XIP so both cores run normally again until the next slice. Reads of
// OTHER sectors (code, the game ROM) are legal while an erase is suspended, so
// the emulator keeps going between slices.
//
// See flash_sliced.cpp for the datasheet constraints and the hard rule that
// nothing may read the sector being erased, or issue any other flash op, while
// an erase is suspended (flash_sliced_erase_in_flight() == true).

// Probe the JEDEC ID once at boot and record whether this is a Winbond part
// that supports erase suspend. MUST be called before any erase is started
// (uses flash_do_cmd, which is illegal while an erase is suspended). Returns
// flash_sliced_supported().
bool flash_sliced_init();

// True iff the chip supports sliced erase. When false, callers must fall back
// to a plain (blocking) flash_range_erase.
bool flash_sliced_supported();

// Begin erasing the 4KB sector at flash offset `offset` (must be
// FLASH_SECTOR_SIZE-aligned). Runs one slice, then suspends. Returns true if
// the erase already fully completed within the first slice (does not happen for
// a real erase; handled for robustness) -- in that case no erase is in flight.
bool flash_sliced_erase_begin(uint32_t offset);

// Resume the in-flight erase for one more slice, then suspend again. Returns
// true when the erase has fully completed (nothing left in flight).
bool flash_sliced_erase_step();

// Drive the in-flight erase to completion, one slice per iteration. Each slice
// is its own core1-lockout window, so audio/DMA still breathe between slices.
void flash_sliced_erase_finish();

// True while a sector erase is suspended on-chip (between begin/step calls).
bool flash_sliced_erase_in_flight();

#include "flash_sliced.hpp"

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/io_qspi.h"
#include "hardware/structs/pads_qspi.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/xip.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"

// This file replicates the RP2040 low-level flash access pattern from pico-sdk's
// hardware_flash/flash.c (the internal helpers there are static, so they can't
// be reused). Line references below point at that file
// (build/_deps/pico_sdk-src/src/rp2_common/hardware_flash/flash.c). Everything
// here is RAM-resident (__no_inline_not_in_flash_func / __force_inline into a
// RAM caller): once XIP is exited, any flash-resident code fetch would fault.

namespace {

// --- W25Q command set / status bits (W25Q128JV) ------------------------------
constexpr uint8_t CMD_WREN = 0x06;
constexpr uint8_t CMD_RDSR1 = 0x05; // bit0 = WIP (busy)
constexpr uint8_t CMD_RDSR2 = 0x35; // bit7 = SUS (erase/program suspended)
constexpr uint8_t CMD_SECTOR_ERASE = 0x20; // 4KB, 24-bit address
constexpr uint8_t CMD_SUSPEND = 0x75;
constexpr uint8_t CMD_RESUME = 0x7A;
constexpr uint8_t CMD_JEDEC_ID = 0x9F;

constexpr uint8_t SR1_WIP = 0x01;
constexpr uint8_t SR2_SUS = 0x80;

constexpr uint8_t WINBOND_MFR = 0xEF;

// How long each poll lets the erase actually run before we suspend it and hand
// the cores back. This is the per-poll stall (XIP down, both cores paused for
// the whole on-time). ~1.5ms is well under the ~16.7ms frame/audio budget, and
// a 4KB erase (~45ms typ) finishes in ~30 polls (~0.5s), spread invisibly
// across frames.
constexpr uint32_t ERASE_ON_US = 1500;
// Safety bound on the post-SUSPEND wait for WIP to clear. Normally WIP clears in
// ~tSUS (~20us). We must NEVER re-enable XIP while WIP is set (the flash cannot
// serve reads mid-erase, so the next instruction fetch would hang the bus), so
// this is deliberately larger than tSE max (~400ms): past that the erase itself
// has finished and WIP is guaranteed clear either way. If a SUSPEND is ever
// missed, this degrades to waiting out that one erase (a one-time stutter),
// never a lockup.
constexpr uint32_t WIP_WAIT_MAX_US = 450000;

bool g_supported = false;
bool g_in_flight = false;

// --- boot2 copyout, so XIP can be re-entered after we exit it (flash.c:37-61) -
constexpr int BOOT2_SIZE_WORDS = 64;
uint32_t g_boot2_copyout[BOOT2_SIZE_WORDS];
bool g_boot2_valid = false;

void __no_inline_not_in_flash_func(init_boot2_copyout)() {
	if (g_boot2_valid)
		return;
	// Runs while XIP is still up (window not yet exited), so this read from
	// XIP_BASE is a normal cached flash read.
	const volatile uint32_t *from = (const volatile uint32_t *)XIP_BASE;
	for (int i = 0; i < BOOT2_SIZE_WORDS; i++)
		g_boot2_copyout[i] = from[i];
	__compiler_memory_barrier();
	g_boot2_valid = true;
}

void __no_inline_not_in_flash_func(enable_xip_via_boot2)() {
	// boot2 accepts a return vector in LR and does not trash r4-r7 (flash.c:59).
	((void (*)(void))((intptr_t)g_boot2_copyout + 1))();
}

// --- QSPI pad + XIP control save/restore (flash.c:143-175, RP2040 branch) -----
struct hw_state_t {
	uint32_t xip_ctrl;
	uint32_t qspi_pads[count_of(pads_qspi_hw->io)];
};

void __no_inline_not_in_flash_func(save_hw_state)(hw_state_t *s) {
	for (size_t i = 0; i < count_of(pads_qspi_hw->io); i++)
		s->qspi_pads[i] = pads_qspi_hw->io[i];
	s->xip_ctrl = xip_ctrl_hw->ctrl;
}

void __no_inline_not_in_flash_func(restore_hw_state)(const hw_state_t *s) {
	for (size_t i = 0; i < count_of(pads_qspi_hw->io); i++)
		pads_qspi_hw->io[i] = s->qspi_pads[i];
	xip_ctrl_hw->ctrl = s->xip_ctrl;
}

// Bit-bang the QSPI chip select via IO override (flash.c:273-291, RP2040).
__force_inline void cs_force(bool high) {
	uint32_t field = high ? IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH
			      : IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
	hw_write_masked(&io_qspi_hw->io[1].ctrl,
			field << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
			IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
}

// Send one CS-framed command over the SSI, full-duplex (flash.c:306-344 loop).
// Assumes XIP is already exited (a window is open). rx may be scratch.
void __no_inline_not_in_flash_func(qspi_cmd)(const uint8_t *tx, uint8_t *rx, size_t n) {
	cs_force(false);
	size_t tx_rem = n, rx_rem = n;
	const size_t max_in_flight = 16 - 2; // don't overflow the FIFO if IRQs fire
	while (tx_rem || rx_rem) {
		uint32_t sr = ssi_hw->sr;
		if ((sr & SSI_SR_TFNF_BITS) && tx_rem &&
		    rx_rem - tx_rem < max_in_flight) {
			ssi_hw->dr0 = *tx++;
			tx_rem--;
		}
		if ((sr & SSI_SR_RFNE_BITS) && rx_rem) {
			*rx++ = (uint8_t)ssi_hw->dr0;
			rx_rem--;
		}
	}
	cs_force(true);
}

// Open a low-speed-command window: exit XIP so raw SSI commands work. Must be
// paired with window_close(). init_boot2_copyout() reads flash and so must run
// before flash_exit_xip (while XIP is still up).
void __no_inline_not_in_flash_func(window_open)(hw_state_t *st) {
	rom_connect_internal_flash_fn connect =
	    (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
	rom_flash_exit_xip_fn exit_xip =
	    (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
	init_boot2_copyout();
	save_hw_state(st);
	__compiler_memory_barrier(); // no flash access past here
	connect();
	exit_xip();
}

void __no_inline_not_in_flash_func(window_close)(const hw_state_t *st) {
	rom_flash_flush_cache_fn flush =
	    (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
	flush(); // also releases the CS IO force (flash.c:232) + drops stale cache lines
	enable_xip_via_boot2();
	restore_hw_state(st);
}

__force_inline uint8_t read_sr(uint8_t cmd) {
	uint8_t tx[2] = { cmd, 0 };
	uint8_t rx[2] = { 0, 0 };
	qspi_cmd(tx, rx, 2);
	return rx[1];
}

__force_inline void cmd0(uint8_t cmd) {
	uint8_t rx[1];
	qspi_cmd(&cmd, rx, 1);
}

__force_inline void busy_wait_us(uint32_t us) {
	uint32_t t0 = timer_hw->timerawl; // raw us timer, XIP-safe register read
	while ((uint32_t)(timer_hw->timerawl - t0) < us)
		; // timer_hw fields are volatile, so this is not optimized away
}

// Let the erase run for ERASE_ON_US, then SUSPEND it and -- critically -- wait
// until WIP actually clears before returning, because the caller re-enables XIP
// immediately after and a read issued while WIP is still set would hang the bus.
// Returns true iff the erase fully completed (nothing left to resume). Must be
// called inside an open window.
bool __no_inline_not_in_flash_func(run_on_and_park)() {
	busy_wait_us(ERASE_ON_US);
	if (!(read_sr(CMD_RDSR1) & SR1_WIP))
		return true; // erase finished within this on-time; XIP is safe
	// Park the erase. Wait for WIP to clear (suspend takes ~tSUS); the bound
	// only guards a pathological missed-suspend, and even then WIP is clear by
	// the time it expires (past tSE max the erase itself has completed).
	cmd0(CMD_SUSPEND);
	uint32_t t0 = timer_hw->timerawl;
	while ((read_sr(CMD_RDSR1) & SR1_WIP) &&
	       (uint32_t)(timer_hw->timerawl - t0) < WIP_WAIT_MAX_US)
		;
	// SUS set => really parked (more to do); SUS clear => the erase completed.
	return !(read_sr(CMD_RDSR2) & SR2_SUS);
}

// Pause core1 (only if it registered as a lockout victim -- with sound disabled
// core1 is never launched, see save_storage.cpp's flash_locked_erase) and
// disable our own interrupts for the duration of one on-time, so nothing fetches
// from flash while XIP is exited. Same guarded pattern as flash_locked_erase.
struct lockout_guard {
	bool locked;
	uint32_t ints;
	lockout_guard() {
		locked = multicore_lockout_victim_is_initialized(1);
		if (locked)
			multicore_lockout_start_blocking();
		ints = save_and_disable_interrupts();
	}
	~lockout_guard() {
		restore_interrupts(ints);
		if (locked)
			multicore_lockout_end_blocking();
	}
};

// One poll's worth of erasing, wrapped in the lockout + XIP-off window. `start`
// issues WREN + Sector Erase for a fresh erase; otherwise RESUME continues an
// already-parked one. Returns true when the erase is fully complete.
bool __no_inline_not_in_flash_func(erase_on_and_park)(bool start, uint32_t offset) {
	lockout_guard guard;
	hw_state_t st;
	window_open(&st);
	if (start) {
		cmd0(CMD_WREN);
		uint8_t tx[4] = { CMD_SECTOR_ERASE, (uint8_t)(offset >> 16),
				  (uint8_t)(offset >> 8), (uint8_t)offset };
		uint8_t rx[4];
		qspi_cmd(tx, rx, 4);
	} else {
		cmd0(CMD_RESUME);
	}
	bool done = run_on_and_park();
	window_close(&st);
	return done;
}

} // namespace

bool flash_sliced_init() {
	// flash_do_cmd handles its own XIP exit/restore; safe here because no erase
	// can be in flight this early (called from save_storage_load() at boot).
	uint8_t tx[4] = { CMD_JEDEC_ID, 0, 0, 0 };
	uint8_t rx[4] = { 0, 0, 0, 0 };
	flash_do_cmd(tx, rx, 4);
	g_supported = (rx[1] == WINBOND_MFR);
	return g_supported;
}

bool flash_sliced_supported() {
	return g_supported;
}

bool flash_sliced_erase_begin(uint32_t offset) {
	bool done = erase_on_and_park(/*start=*/true, offset);
	g_in_flight = !done;
	return done;
}

bool flash_sliced_erase_step() {
	if (!g_in_flight)
		return true;
	bool done = erase_on_and_park(/*start=*/false, 0);
	g_in_flight = !done;
	return done;
}

void flash_sliced_erase_finish() {
	while (g_in_flight)
		flash_sliced_erase_step();
}

bool flash_sliced_erase_in_flight() {
	return g_in_flight;
}

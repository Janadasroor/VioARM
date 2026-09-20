//! Phase-C `stm32f031` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOF` answers `MODER` + `BSRR`-driven `ODR`.
//! * `TIM3`/`TIM14`/`TIM16`/`TIM17` count to overflow.
//! * `RTC` runs the new-generation calendar (init + alarm A).
//! * `TSC` latches end-of-acquisition and clears it.
//! * `CEC` latches transmit-end and clears it.
//! * `Flash` unlocks, erases, and programs through the gate.
//! * `DBGMCU` reports the documented `IDCODE`.
//!
//! Register offsets come from the vendored `STM32F031` table
//! (`device::svd::stm32f031`); bit positions mirror the driver sources
//! (CEC `TXEND` is bit 9 per the table).

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

fn pump(mem: &mut board::BoardMemory, n: u64) {
    for _ in 0..n {
        mem.tick_devices(1);
    }
}

// --- GPIO ---

const GPIOF: u32 = 0x4800_1400;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpiof_answers() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    ww(mem, GPIOF + MODER, 0x5555_5555);
    assert_eq!(rw(mem, GPIOF + MODER), 0x5555_5555, "MODER latches");
    ww(mem, GPIOF + BSRR, 0x0000_0005);
    assert_eq!(rw(mem, GPIOF + ODR), 0x05, "BSRR sets ODR");
    ww(mem, GPIOF + BSRR, 0x0005_0000);
    assert_eq!(rw(mem, GPIOF + ODR), 0x00, "BSRR resets ODR");
}

// --- Timers ---

const TIM3: u32 = 0x4000_0400;
const TIM14: u32 = 0x4000_2000;
const TIM16: u32 = 0x4001_4400;
const TIM17: u32 = 0x4001_4800;
const CR1_T: u32 = 0x00;
const SR_T: u32 = 0x10;
const ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;
const UIF: u32 = 1 << 0;

#[test]
fn tim3_14_16_17_count() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    for base in [TIM3, TIM14, TIM16, TIM17] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM3, TIM14, TIM16, TIM17] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- RTC (new-gen calendar) ---

const RTC: u32 = 0x4000_2800;
const TR: u32 = 0x00;
const ISR_R: u32 = 0x0C;
const ALRMAR: u32 = 0x1C;
const WPR: u32 = 0x24;
const INIT: u32 = 1 << 7;
const INITF: u32 = 1 << 6;
const INITS: u32 = 1 << 4;
const ALRAF: u32 = 1 << 8;
const ALRAIE: u32 = 1 << 12;

#[test]
fn rtc_calendar_and_alarm() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    ww(mem, RTC + WPR, 0xCA);
    ww(mem, RTC + WPR, 0x53);
    ww(mem, RTC + ISR_R, INIT);
    assert_ne!(rw(mem, RTC + ISR_R) & INITF, 0, "INITF in init");
    ww(mem, RTC + TR, 0x00_0000);
    ww(mem, RTC + ISR_R, 0);
    assert_ne!(rw(mem, RTC + ISR_R) & INITS, 0, "INITS after init");
    ww(mem, RTC + ALRMAR, 0x03);
    ww(mem, RTC + ISR_R, ALRAIE);
    pump(mem, 1);
    pump(mem, 1);
    pump(mem, 1);
    assert_eq!(rw(mem, RTC + TR) & 0xF, 3, "TR.SU == 3");
    assert_ne!(rw(mem, RTC + ISR_R) & ALRAF, 0, "ALRAF fires");
}

// --- TSC ---

const TSC: u32 = 0x4002_4000;
const CR_C: u32 = 0x00;
const ICR: u32 = 0x08;
const ISR_C: u32 = 0x0C;
const TSCE: u32 = 1 << 0;
const START: u32 = 1 << 1;
const EOAF: u32 = 1 << 0;

#[test]
fn tsc_acquisition_completes() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    ww(mem, TSC + CR_C, TSCE | START);
    pump(mem, 4);
    assert_ne!(rw(mem, TSC + ISR_C) & EOAF, 0, "EOAF latches");
    ww(mem, TSC + ICR, EOAF);
    assert_eq!(rw(mem, TSC + ISR_C) & EOAF, 0, "EOAF clears");
}

// --- CEC ---

const CEC: u32 = 0x4000_7800;
const CR_E: u32 = 0x00;
const TXDR: u32 = 0x08;
const ISR_E: u32 = 0x10;
const CECEN: u32 = 1 << 0;
const TXEND: u32 = 1 << 9;

#[test]
fn cec_transmit_ends() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    ww(mem, CEC + CR_E, CECEN);
    ww(mem, CEC + TXDR, 0xAA);
    pump(mem, 4);
    assert_ne!(rw(mem, CEC + ISR_E) & TXEND, 0, "TXEND latches");
    ww(mem, CEC + ISR_E, TXEND);
    assert_eq!(rw(mem, CEC + ISR_E) & TXEND, 0, "TXEND clears");
}

// --- Flash ---

const FLASH_R: u32 = 0x4002_2000;
const KEYR: u32 = 0x04;
const SR_F: u32 = 0x0C;
const CR_F: u32 = 0x10;
const AR: u32 = 0x14;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const PG: u32 = 1 << 0;
const PER: u32 = 1 << 1;
const STRT: u32 = 1 << 6;
const EOP: u32 = 1 << 5;
const MAIN_FLASH: u32 = 0x0800_0000;

#[test]
fn flash_erase_programs() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    ww(mem, FLASH_R + KEYR, KEY1);
    ww(mem, FLASH_R + KEYR, KEY2);
    ww(mem, FLASH_R + CR_F, PER);
    ww(mem, FLASH_R + AR, MAIN_FLASH);
    ww(mem, FLASH_R + CR_F, PER | STRT);
    pump(mem, 100);
    assert_ne!(rw(mem, FLASH_R + SR_F) & EOP, 0, "erase completes");
    ww(mem, FLASH_R + SR_F, 0);
    ww(mem, FLASH_R + CR_F, PG);
    mem.write(MAIN_FLASH, 0xEF).unwrap();
    mem.write(MAIN_FLASH + 1, 0xBE).unwrap();
    pump(mem, 100);
    assert_ne!(rw(mem, FLASH_R + SR_F) & EOP, 0, "program completes");
    assert_eq!(rw(mem, MAIN_FLASH) & 0xFFFF, 0xBEEF, "halfword lands");
}

// --- DBGMCU ---

const DBGMCU: u32 = 0x4001_5800;
const IDCODE: u32 = 0x00;
const CR_D: u32 = 0x04;

#[test]
fn dbgmcu_reports_f031() {
    let mut m = Machine::armv7m(board::stm32f031());
    let mem = m.core.mem();
    assert_eq!(rw(mem, DBGMCU + IDCODE), 0x1000_0444, "REV_ID + DEV_ID");
    assert_eq!(rw(mem, DBGMCU + CR_D), 0, "CR resets clear");
    ww(mem, DBGMCU + CR_D, 0x0000_0003);
    assert_eq!(rw(mem, DBGMCU + CR_D), 0x0000_0003, "CR latches");
}

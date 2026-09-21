//! Phase-C `stm32h743` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `RTC` runs the new-gen calendar + alarm A.
//! * `UART4`/`5`/`7`/`8` move bytes CPU-side (TX drains, RX injects).
//! * `SYSCFG` runs the I/O compensation-cell `READY` handshake.
//! * `Flash` unlocks bank 1, programs a word, and sector-erases it.
//! * The three `DELAY_Block` windows report `LNGF` once enabled.
//! * `CAN_CCU` walks calibration to precision and barks its watchdog.
//! * `DBGMCU` reports `IDC 0x10006450`.
//!
//! `AXI` stays a documented latch: interconnect issuing/QoS tuning with no
//! firmware-observable dynamics (reset values are the working config).

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

// --- RTC ---

const RTC: u32 = 0x5800_4000;
const TR: u32 = 0x00;
const ISR_R: u32 = 0x0C;
const ALRMAR: u32 = 0x1C;
const WPR: u32 = 0x24;
const INIT: u32 = 1 << 7;
const INITF: u32 = 1 << 6;
const INITS: u32 = 1 << 4;
const ALRAIE: u32 = 1 << 12;
const ALRAF: u32 = 1 << 8;

#[test]
fn rtc_calendar_and_alarm() {
    let mut m = Machine::armv7em(board::stm32h743());
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

// --- UART4/5/7/8 ---

const UART4: u32 = 0x4000_4C00;
const UART5: u32 = 0x4000_5000;
const UART7: u32 = 0x4000_7800;
const UART8: u32 = 0x4000_7C00;
const TDR: u32 = 0x28;
const RDR: u32 = 0x24;
const CR1_U: u32 = 0x00;
const ISR_U: u32 = 0x1C;

fn uart_moves(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1_U, (1 << 0) | (1 << 3) | (1 << 2)); // UE|TE|RE
        ww(mem, base + TDR, u32::from(b'Z'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device(name), b"Z", "TX drains");
    m.drive_device(name, b"Q");
    let mem = m.core.mem();
    pump(mem, 10_000);
    assert_ne!(rw(mem, base + ISR_U) & (1 << 5), 0, "RXNE latches");
    assert_eq!(rw(mem, base + RDR) & 0xFF, u32::from(b'Q'), "RX byte");
}

#[test]
fn uart4_moves_bytes() {
    let mut m = Machine::armv7em(board::stm32h743());
    uart_moves(&mut m, UART4, "uart4");
}

#[test]
fn uart5_7_8_answer() {
    let mut m = Machine::armv7em(board::stm32h743());
    for (base, name) in [(UART5, "uart5"), (UART7, "uart7"), (UART8, "uart8")] {
        uart_moves(&mut m, base, name);
    }
}

// --- SYSCFG comp cell ---

const SYSCFG: u32 = 0x5800_0400;
const CCCSR: u32 = 0x20;

#[test]
fn syscfg_comp_cell_ready() {
    let mut m = Machine::armv7em(board::stm32h743());
    let mem = m.core.mem();
    ww(mem, SYSCFG + CCCSR, 1); // EN
    assert_eq!(rw(mem, SYSCFG + CCCSR) & (1 << 8), 0, "READY waits");
    pump(mem, 8);
    assert_ne!(rw(mem, SYSCFG + CCCSR) & (1 << 8), 0, "READY latches");
    ww(mem, SYSCFG + CCCSR, 0);
    assert_eq!(rw(mem, SYSCFG + CCCSR) & (1 << 8), 0, "EN off drops READY");
}

// --- Flash ---

const FLASH_R: u32 = 0x5200_2000;
const KEYR1: u32 = 0x04;
const CR1: u32 = 0x0C;
const SR1: u32 = 0x10;
const CCR1: u32 = 0x14;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

#[test]
fn flash_programs_and_erases_bank1() {
    let mut m = Machine::armv7em(board::stm32h743());
    let mem = m.core.mem();
    // Locked first: the program write is rejected, WRPERR latches.
    mem.write(0x0800_0010, 0xAA).unwrap();
    assert_ne!(rw(mem, FLASH_R + SR1) & (1 << 17), 0, "WRPERR1 locked");
    ww(mem, FLASH_R + CCR1, 1 << 17);
    // Unlock, program one word in sector 7 (top of bank 1).
    ww(mem, FLASH_R + KEYR1, KEY1);
    ww(mem, FLASH_R + KEYR1, KEY2);
    assert_eq!(rw(mem, FLASH_R + CR1) & 1, 0, "LOCK1 clears");
    ww(mem, FLASH_R + CR1, 1 << 1); // PG1
    const ADDR: u32 = 0x080F_0000;
    mem.write(ADDR, 0x5A).unwrap();
    assert_eq!(mem.read(ADDR).unwrap(), 0x5A, "program lands");
    assert_ne!(rw(mem, FLASH_R + SR1) & (1 << 16), 0, "EOP1");
    ww(mem, FLASH_R + CCR1, 1 << 16);
    // Sector-erase SNB=7, verify zeroes.
    ww(mem, FLASH_R + CR1, (7 << 8) | (1 << 2) | (1 << 7));
    assert_eq!(mem.read(ADDR).unwrap(), 0, "erase zeroes");
    assert_ne!(rw(mem, FLASH_R + SR1) & (1 << 16), 0, "EOP1 after erase");
}

// --- Delay blocks ---

#[test]
fn delay_blocks_report_lngf() {
    let mut m = Machine::armv7em(board::stm32h743());
    let mem = m.core.mem();
    for base in [0x5200_8000u32, 0x4802_2800, 0x5200_6000] {
        ww(mem, base + 0x04, 0x0342_1105);
        assert_eq!(rw(mem, base + 0x04) & (1 << 31), 0, "{base:#x} disabled");
        ww(mem, base, 0x03); // DEN + SEN
        assert_ne!(rw(mem, base + 0x04) & (1 << 31), 0, "{base:#x} LNGF");
    }
}

// --- CAN_CCU ---

const CCU: u32 = 0x4000_A800;

#[test]
fn ccu_calibrates_and_watchdogs() {
    let mut m = Machine::armv7em(board::stm32h743());
    let mem = m.core.mem();
    // Bypass: no walk.
    ww(mem, CCU + 0x04, 1 << 6);
    pump(mem, 50);
    assert_eq!(rw(mem, CCU + 0x08) & (0x3 << 30), 0, "bypass quiet");
    // Calibrate: basic then precision, CSC flags.
    ww(mem, CCU + 0x04, 0);
    pump(mem, 8);
    assert_eq!(rw(mem, CCU + 0x08) & (0x3 << 30), 1 << 30, "basic");
    pump(mem, 8);
    assert_eq!(rw(mem, CCU + 0x08) & (0x3 << 30), 2 << 30, "precision");
    assert_ne!(rw(mem, CCU + 0x10) & 0x02, 0, "CSC latched");
    ww(mem, CCU + 0x10, 0x02);
    // Watchdog: start 3, reload 3 → barks, reloads, barks again.
    ww(mem, CCU + 0x0C, (3 << 16) | 3);
    pump(mem, 3);
    assert_ne!(rw(mem, CCU + 0x10) & 0x01, 0, "CWE on expiry");
    assert_eq!(rw(mem, CCU + 0x0C) & 0xFFFF, 3, "reloaded");
}

// --- DBGMCU ---

#[test]
fn dbgmcu_reports_id() {
    let mut m = Machine::armv7em(board::stm32h743());
    assert_eq!(rw(m.core.mem(), 0x5C00_1000), 0x1000_6450, "IDC");
}

//! Phase-C `stm32f072` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `USART4` moves bytes CPU-side (TX drains, RX injects, new-gen layout).
//! * `ADC` converts a driven channel (F0 layout).
//! * `TIM14`–`TIM17` count to overflow.
//! * `RTC` runs the new-generation calendar (init + alarm A).
//! * `CEC` latches transmit-end and clears it.
//! * `CAN` walks init, holds filters, and lands an injected frame.
//! * `USB` ticks enumeration (`DADDR.EF` → `ISTR.CTR`).
//! * `CRS` latches sync-ok and clears it.
//! * `Flash` unlocks, erases, and programs through the gate.
//! * `DBGMCU` reports the documented `IDCODE` (`DEV_ID 0x448`).
//!
//! Register offsets come from the vendored `STM32F072` table
//! (`device::svd::stm32f072`); bit positions mirror the driver sources.

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

// --- USART4 (new-gen layout) ---

const USART4: u32 = 0x4000_4C00;
const CR1: u32 = 0x00;
const ISR: u32 = 0x1C;
const RDR: u32 = 0x24;
const TDR: u32 = 0x28;
const UE: u32 = 1 << 0;
const RE: u32 = 1 << 2;
const TE: u32 = 1 << 3;
const RXNE: u32 = 1 << 5;

#[test]
fn usart4_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f072());
    {
        let mem = m.core.mem();
        ww(mem, USART4 + CR1, UE | RE | TE);
        ww(mem, USART4 + TDR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device("usart4"), b"A", "TX drains");
    m.drive_device("usart4", b"Q");
    let (sr, byte) = {
        let mem = m.core.mem();
        pump(mem, 10_000);
        let mut sr = 0;
        for _ in 0..100 {
            sr = rw(mem, USART4 + ISR);
            if sr & RXNE != 0 {
                break;
            }
            pump(mem, 100);
        }
        (sr, rw(mem, USART4 + RDR) & 0xFF)
    };
    assert_ne!(sr & RXNE, 0, "RXNE latches");
    assert_eq!(byte, u32::from(b'Q'), "RX byte");
}

// --- ADC (F0 layout) ---

const ADC: u32 = 0x4001_2400;
const ISR_A: u32 = 0x00;
const CR_A: u32 = 0x08;
const DR_A: u32 = 0x40;
const ADEN: u32 = 1 << 0;
const ADSTART: u32 = 1 << 2;
const ADRDY: u32 = 1 << 0;
const EOC: u32 = 1 << 2;

#[test]
fn adc_converts_driven_channel() {
    let mut m = Machine::armv7m(board::stm32f072());
    m.drive_device("adc", &[208]);
    let mem = m.core.mem();
    ww(mem, ADC + CR_A, ADEN);
    pump(mem, 100);
    assert_ne!(rw(mem, ADC + ISR_A) & ADRDY, 0, "ADRDY after ADEN");
    ww(mem, ADC + CR_A, ADEN | ADSTART);
    pump(mem, 1000);
    assert_ne!(rw(mem, ADC + ISR_A) & EOC, 0, "EOC latches");
    assert_eq!(rw(mem, ADC + DR_A) & 0xFFF, 208, "driven code lands");
}

// --- Timers ---

const TIM14: u32 = 0x4000_2000;
const TIM15: u32 = 0x4001_4000;
const TIM16: u32 = 0x4001_4400;
const TIM17: u32 = 0x4001_4800;
const CR1_T: u32 = 0x00;
const SR_T: u32 = 0x10;
const ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;
const UIF: u32 = 1 << 0;

#[test]
fn tim14_thru_17_count() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    for base in [TIM14, TIM15, TIM16, TIM17] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM14, TIM15, TIM16, TIM17] {
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
    let mut m = Machine::armv7m(board::stm32f072());
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

// --- CEC ---

const CEC: u32 = 0x4000_7800;
const CR_E: u32 = 0x00;
const TXDR: u32 = 0x08;
const ISR_E: u32 = 0x10;
const CECEN: u32 = 1 << 0;
const TXEND: u32 = 1 << 9;

#[test]
fn cec_transmit_ends() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    ww(mem, CEC + CR_E, CECEN);
    ww(mem, CEC + TXDR, 0xAA);
    pump(mem, 4);
    assert_ne!(rw(mem, CEC + ISR_E) & TXEND, 0, "TXEND latches");
    ww(mem, CEC + ISR_E, TXEND);
    assert_eq!(rw(mem, CEC + ISR_E) & TXEND, 0, "TXEND clears");
}

// --- CAN ---

const CAN: u32 = 0x4000_6400;
const MCR: u32 = 0x00;
const MSR: u32 = 0x04;
const BTR: u32 = 0x1C;
const FMR: u32 = 0x200;
const FA1R: u32 = 0x21C;
const F0R1: u32 = 0x240;
const RF0R: u32 = 0x0C;
const RI0R: u32 = 0x1B0;
const RDL0R: u32 = 0x1B8;
const INRQ: u32 = 1 << 0;
const INAK: u32 = 1 << 0;
const FINIT: u32 = 1;

#[test]
fn can_init_filters_and_rx() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    ww(mem, CAN + MCR, INRQ);
    pump(mem, 10);
    assert_ne!(rw(mem, CAN + MSR) & INAK, 0, "INAK answers INRQ");
    ww(mem, CAN + BTR, 0x001C_0000);
    assert_eq!(rw(mem, CAN + BTR), 0x001C_0000, "BTR latches");
    ww(mem, CAN + FMR, FINIT);
    ww(mem, CAN + FA1R, 1);
    ww(mem, CAN + F0R1, 0);
    ww(mem, CAN + F0R1 + 4, 0);
    ww(mem, CAN + FMR, 0);
    ww(mem, CAN + MCR, 0);
    pump(mem, 10);
    assert_eq!(rw(mem, CAN + MSR) & INAK, 0, "normal mode");
    let (fmp0, ident, payload) = {
        let mut frame = [0u8; 16];
        frame[0..4].copy_from_slice(&0x123u32.to_le_bytes());
        frame[4] = 1;
        frame[8] = 0xAA;
        m.drive_device("can", &frame);
        let mem = m.core.mem();
        pump(mem, 10);
        (
            rw(mem, CAN + RF0R) & 0b11,
            rw(mem, CAN + RI0R),
            rw(mem, CAN + RDL0R) & 0xFF,
        )
    };
    assert_eq!(fmp0, 1, "FMP0 hit");
    assert_eq!(ident, 0x123, "identifier");
    assert_eq!(payload, 0xAA, "payload");
}

// --- USB ---

const USB: u32 = 0x4000_5C00;
const CNTR: u32 = 0x40;
const ISTR: u32 = 0x44;
const DADDR: u32 = 0x4C;
const CTRM: u32 = 1 << 15;
const CTR: u32 = 1 << 15;
const EF: u32 = 1 << 7;

#[test]
fn usb_fs_enumeration_tick() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    ww(mem, USB + DADDR, EF | 0x05);
    assert_eq!(rw(mem, USB + DADDR), EF | 0x05, "DADDR holds");
    ww(mem, USB + CNTR, CTRM);
    pump(mem, 4);
    assert_ne!(rw(mem, USB + ISTR) & CTR, 0, "CTR latches after EF");
    ww(mem, USB + ISTR, 0);
    assert_eq!(rw(mem, USB + ISTR) & CTR, 0, "CTR clears");
}

// --- CRS ---

const CRS: u32 = 0x4000_6C00;
const CR_C: u32 = 0x00;
const ISR_C: u32 = 0x08;
const ICR: u32 = 0x0C;
const CEN_C: u32 = 1 << 5;
const AUTOTRIMEN: u32 = 1 << 7;
const SYNCOKF: u32 = 1 << 0;

#[test]
fn crs_sync_ok_latches() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    ww(mem, CRS + CR_C, AUTOTRIMEN | CEN_C);
    pump(mem, 4);
    assert_ne!(rw(mem, CRS + ISR_C) & SYNCOKF, 0, "SYNCOKF latches");
    ww(mem, CRS + ICR, SYNCOKF);
    assert_eq!(rw(mem, CRS + ISR_C) & SYNCOKF, 0, "SYNCOKF clears");
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
    let mut m = Machine::armv7m(board::stm32f072());
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
fn dbgmcu_reports_f072() {
    let mut m = Machine::armv7m(board::stm32f072());
    let mem = m.core.mem();
    assert_eq!(rw(mem, DBGMCU + IDCODE), 0x1000_0448, "REV_ID + DEV_ID");
    assert_eq!(rw(mem, DBGMCU + CR_D), 0, "CR resets clear");
    ww(mem, DBGMCU + CR_D, 0x0000_0003);
    assert_eq!(rw(mem, DBGMCU + CR_D), 0x0000_0003, "CR latches");
}

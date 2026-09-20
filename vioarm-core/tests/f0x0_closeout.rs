//! Phase-C `stm32f0x0` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! The `f0x0` table is a generic donor (no single silicon behind it), so
//! this file proves wiring + shared-driver behavior, not a device.
//!
//! * `GPIOC`/`D`/`F` answer `MODER` + `BSRR`-driven `ODR`.
//! * `USART2`/`3`/`4`/`5`/`6` move bytes CPU-side (new-gen layout).
//! * `SPI1` shifts (peerless `0x00`, staged slave byte).
//! * `I2C1` runs the new-generation engine (peerless NACK).
//! * `TIM1`/`3`/`6`/`7`/`14`–`17` count to overflow.
//! * `EXTI` raises a software interrupt (`SWIER` → `PR`).
//! * `DMA1` copies memory-to-memory with `TCIF1`.
//! * `ADC` converts a driven channel (F0 layout).
//! * `PWR`/`CRC` answer their windows (`CR` latch, computed `DR`).
//! * `RTC` runs the new-generation calendar (init + alarm A).
//! * `USB` ticks enumeration (`DADDR.EF` → `ISTR.CTR`).
//! * `Flash` unlocks, erases, and programs through the gate.
//! * `DBGMCU` reports the placeholder `IDCODE`.
//!
//! Register offsets come from the vendored `STM32F0X0` table
//! (`device::svd::stm32f0x0`); bit positions mirror the driver sources.

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

const GPIOC: u32 = 0x4800_0800;
const GPIOD: u32 = 0x4800_0C00;
const GPIOF: u32 = 0x4800_1400;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_c_d_f_answer() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    for base in [GPIOC, GPIOD, GPIOF] {
        ww(mem, base + MODER, 0x5555_5555);
        assert_eq!(rw(mem, base + MODER), 0x5555_5555, "MODER latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- USARTs (new-gen layout) ---

const USART2: u32 = 0x4000_4400;
const USART3: u32 = 0x4000_4800;
const USART4: u32 = 0x4000_4C00;
const USART5: u32 = 0x4000_5000;
const USART6: u32 = 0x4001_1400;
const CR1: u32 = 0x00;
const ISR: u32 = 0x1C;
const RDR: u32 = 0x24;
const TDR: u32 = 0x28;
const UE: u32 = 1 << 0;
const RE: u32 = 1 << 2;
const TE: u32 = 1 << 3;
const RXNE: u32 = 1 << 5;

fn uart_loopback(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1, UE | RE | TE);
        ww(mem, base + TDR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device(name), b"A", "TX drains");
    m.drive_device(name, b"Q");
    let (sr, byte) = {
        let mem = m.core.mem();
        pump(mem, 10_000);
        let mut sr = 0;
        for _ in 0..100 {
            sr = rw(mem, base + ISR);
            if sr & RXNE != 0 {
                break;
            }
            pump(mem, 100);
        }
        (sr, rw(mem, base + RDR) & 0xFF)
    };
    assert_ne!(sr & RXNE, 0, "RXNE latches");
    assert_eq!(byte, u32::from(b'Q'), "RX byte");
}

#[test]
fn usart2_thru_6_loopback() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    for (base, name) in [
        (USART2, "usart2"),
        (USART3, "usart3"),
        (USART4, "usart4"),
        (USART5, "usart5"),
        (USART6, "usart6"),
    ] {
        uart_loopback(&mut m, base, name);
    }
}

// --- SPI1 ---

const SPI1: u32 = 0x4001_3000;
const CR1_S: u32 = 0x00;
const SR_S: u32 = 0x08;
const DR_S: u32 = 0x0C;
const SPE: u32 = 1 << 6;
const MSTR: u32 = 1 << 2;
const SSI: u32 = 1 << 8;
const SSM: u32 = 1 << 9;
const RXNE_S: u32 = 1 << 0;

#[test]
fn spi1_shifts() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    ww(mem, SPI1 + CR1_S, SPE | MSTR | SSI | SSM);
    ww(mem, SPI1 + DR_S, 0xA5);
    pump(mem, 100);
    assert_ne!(rw(mem, SPI1 + SR_S) & RXNE_S, 0, "RXNE after frame");
    assert_eq!(rw(mem, SPI1 + DR_S) & 0xFF, 0x00, "peerless reads zero");
    m.drive_device("spi1", &[0x5A]);
    let byte = {
        let mem = m.core.mem();
        ww(mem, SPI1 + DR_S, 0xA5);
        pump(mem, 100);
        rw(mem, SPI1 + DR_S) & 0xFF
    };
    assert_eq!(byte, 0x5A, "slave byte");
}

// --- I2C1 (new-gen engine) ---

const I2C1: u32 = 0x4000_5400;
const CR1_I: u32 = 0x00;
const CR2_I: u32 = 0x04;
const ISR_I: u32 = 0x18;
const PE: u32 = 1 << 0;
const NACKF: u32 = 1 << 4;

#[test]
fn i2c1_nacks_peerless() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    ww(mem, I2C1 + CR1_I, PE);
    ww(mem, I2C1 + CR2_I, 0x0001_2050 | (1 << 16));
    pump(mem, 8);
    pump(mem, 18);
    assert_ne!(rw(mem, I2C1 + ISR_I) & NACKF, 0, "peerless NACKs");
}

// --- Timers ---

const TIM1: u32 = 0x4001_2C00;
const TIM3: u32 = 0x4000_0400;
const TIM6: u32 = 0x4000_1000;
const TIM7: u32 = 0x4000_1400;
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
fn timers_count() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    for base in [TIM1, TIM3, TIM6, TIM7, TIM14, TIM15, TIM16, TIM17] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM1, TIM3, TIM6, TIM7, TIM14, TIM15, TIM16, TIM17] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- EXTI ---

const EXTI: u32 = 0x4001_0400;
const SWIER: u32 = 0x10;
const PR: u32 = 0x14;

#[test]
fn exti_software_interrupt_pends() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    ww(mem, EXTI + SWIER, 0x0000_0001);
    pump(mem, 4);
    assert_ne!(rw(mem, EXTI + PR) & 0x1, 0, "PR latches from SWIER");
    ww(mem, EXTI + PR, 0x0000_0001);
    assert_eq!(rw(mem, EXTI + PR) & 0x1, 0, "PR clears on write");
}

// --- DMA1 ---

const DMA1: u32 = 0x4002_0000;
const ISR_D: u32 = 0x00;
const CCR1: u32 = 0x08;
const CNDTR1: u32 = 0x0C;
const CPAR1: u32 = 0x10;
const CMAR1: u32 = 0x14;
const EN: u32 = 1 << 0;
const MINC: u32 = 1 << 7;
const PINC: u32 = 1 << 6;
const MEM2MEM: u32 = 1 << 14;
const TCIF1: u32 = 1 << 1;
const SRC: u32 = 0x2000_0100;
const DST: u32 = 0x2000_0200;

#[test]
fn dma_mem2mem_copy() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    for (i, w) in [0x1111_1111u32, 0x2222_2222].iter().enumerate() {
        ww(mem, SRC + (i as u32) * 4, *w);
    }
    ww(mem, DMA1 + CPAR1, SRC);
    ww(mem, DMA1 + CMAR1, DST);
    ww(mem, DMA1 + CNDTR1, 2);
    ww(
        mem,
        DMA1 + CCR1,
        EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
    );
    pump(mem, 10);
    assert_eq!(rw(mem, DST), 0x1111_1111, "word 0 lands");
    assert_eq!(rw(mem, DST + 4), 0x2222_2222, "word 1 lands");
    assert_ne!(rw(mem, DMA1 + ISR_D) & TCIF1, 0, "TCIF1 latches");
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
    let mut m = Machine::armv7m(board::stm32f0x0());
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

// --- PWR + CRC windows ---

const PWR: u32 = 0x4000_7000;
const CR_P: u32 = 0x00;
const DBP: u32 = 1 << 8;
const CRC: u32 = 0x4002_3000;
const DR_C: u32 = 0x00;

#[test]
fn pwr_crc_answer() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    ww(mem, PWR + CR_P, DBP);
    assert_eq!(rw(mem, PWR + CR_P) & DBP, DBP, "DBP latches");
    let before = rw(mem, CRC + DR_C);
    ww(mem, CRC + DR_C, 0x01);
    let after = rw(mem, CRC + DR_C);
    assert_ne!(before, after, "CRC engine runs");
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
    let mut m = Machine::armv7m(board::stm32f0x0());
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
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    ww(mem, USB + DADDR, EF | 0x05);
    assert_eq!(rw(mem, USB + DADDR), EF | 0x05, "DADDR holds");
    ww(mem, USB + CNTR, CTRM);
    pump(mem, 4);
    assert_ne!(rw(mem, USB + ISTR) & CTR, 0, "CTR latches after EF");
    ww(mem, USB + ISTR, 0);
    assert_eq!(rw(mem, USB + ISTR) & CTR, 0, "CTR clears");
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
    let mut m = Machine::armv7m(board::stm32f0x0());
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
fn dbgmcu_reports_placeholder() {
    let mut m = Machine::armv7m(board::stm32f0x0());
    let mem = m.core.mem();
    // Generic donor table: F030-family placeholder, not a device claim.
    assert_eq!(rw(mem, DBGMCU + IDCODE), 0x1000_0444, "placeholder IDCODE");
    assert_eq!(rw(mem, DBGMCU + CR_D), 0, "CR resets clear");
    ww(mem, DBGMCU + CR_D, 0x0000_0003);
    assert_eq!(rw(mem, DBGMCU + CR_D), 0x0000_0003, "CR latches");
}

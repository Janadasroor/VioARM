//! Phase-C `stm32f0x1` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! The `f0x1` table is a generic donor (no single silicon behind it), so
//! this file proves wiring + shared-driver behavior, not a device.
//!
//! * `GPIOA`–`GPIOF` answer `MODER` + `BSRR`-driven `ODR`.
//! * `USART1`–`8` move bytes CPU-side (new-gen layout).
//! * `SPI1` shifts (peerless `0x00`, staged slave byte).
//! * `I2C1` runs the new-generation engine (peerless NACK).
//! * `TIM1`/`2`/`3`/`6`/`7`/`14`–`17` count to overflow.
//! * `EXTI` raises a software interrupt (`SWIER` → `PR`).
//! * `DMA1`/`DMA2` copy memory-to-memory with `TCIF1`.
//! * `SYSCFG_COMP` latches routing (`EXTICR1` round-trips).
//! * `ADC` converts a driven channel (F0 layout).
//! * `DAC` converts immediately (`DHR` → `DOR`).
//! * `PWR`/`CRC` answer their windows (`CR` latch, computed `DR`).
//! * `RTC` runs the new-generation calendar (init + alarm A).
//! * `TSC` latches end-of-acquisition and clears it.
//! * `CEC` latches transmit-end and clears it.
//! * `CAN` walks init, holds filters, and lands an injected frame.
//! * `USB` ticks enumeration (`DADDR.EF` → `ISTR.CTR`).
//! * `CRS` latches sync-ok and clears it.
//! * `Flash` unlocks, erases, and programs through the gate.
//! * `DBGMCU` reports the placeholder `IDCODE`.
//!
//! Register offsets come from the vendored `STM32F0X1` table
//! (`device::svd::stm32f0x1`); bit positions mirror the driver sources.

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

const GPIOA: u32 = 0x4800_0000;
const GPIOB: u32 = 0x4800_0400;
const GPIOC: u32 = 0x4800_0800;
const GPIOD: u32 = 0x4800_0C00;
const GPIOE: u32 = 0x4800_1000;
const GPIOF: u32 = 0x4800_1400;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_all_ports_answer() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    for base in [GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF] {
        ww(mem, base + MODER, 0x5555_5555);
        assert_eq!(rw(mem, base + MODER), 0x5555_5555, "MODER latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- USARTs (new-gen layout) ---

const USART1: u32 = 0x4001_3800;
const USART2: u32 = 0x4000_4400;
const USART3: u32 = 0x4000_4800;
const USART4: u32 = 0x4000_4C00;
const USART5: u32 = 0x4000_5000;
const USART6: u32 = 0x4001_1400;
const USART7: u32 = 0x4001_1800;
const USART8: u32 = 0x4001_1C00;
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
fn usart1_thru_8_loopback() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    for (base, name) in [
        (USART1, "usart1"),
        (USART2, "usart2"),
        (USART3, "usart3"),
        (USART4, "usart4"),
        (USART5, "usart5"),
        (USART6, "usart6"),
        (USART7, "usart7"),
        (USART8, "usart8"),
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    ww(mem, I2C1 + CR1_I, PE);
    ww(mem, I2C1 + CR2_I, 0x0001_2050 | (1 << 16));
    pump(mem, 8);
    pump(mem, 18);
    assert_ne!(rw(mem, I2C1 + ISR_I) & NACKF, 0, "peerless NACKs");
}

// --- Timers ---

const TIM1: u32 = 0x4001_2C00;
const TIM2: u32 = 0x4000_0000;
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
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    for base in [TIM1, TIM2, TIM3, TIM6, TIM7, TIM14, TIM15, TIM16, TIM17] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM1, TIM2, TIM3, TIM6, TIM7, TIM14, TIM15, TIM16, TIM17] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- EXTI ---

const EXTI: u32 = 0x4001_0400;
const SWIER: u32 = 0x10;
const PR: u32 = 0x14;

#[test]
fn exti_software_interrupt_pends() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    ww(mem, EXTI + SWIER, 0x0000_0001);
    pump(mem, 4);
    assert_ne!(rw(mem, EXTI + PR) & 0x1, 0, "PR latches from SWIER");
    ww(mem, EXTI + PR, 0x0000_0001);
    assert_eq!(rw(mem, EXTI + PR) & 0x1, 0, "PR clears on write");
}

// --- DMA1 + DMA2 ---

const DMA1: u32 = 0x4002_0000;
const DMA2: u32 = 0x4002_0400;
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

fn dma_copy(mem: &mut board::BoardMemory, base: u32, word: u32) {
    ww(mem, SRC, word);
    ww(mem, base + CPAR1, SRC);
    ww(mem, base + CMAR1, DST);
    ww(mem, base + CNDTR1, 1);
    ww(
        mem,
        base + CCR1,
        EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
    );
    pump(mem, 10);
    assert_eq!(rw(mem, DST), word, "word lands via {base:#X}");
    assert_ne!(rw(mem, base + ISR_D) & TCIF1, 0, "TCIF1 latches");
}

#[test]
fn dma1_dma2_mem2mem_copy() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    dma_copy(m.core.mem(), DMA1, 0x1111_1111);
    dma_copy(m.core.mem(), DMA2, 0x2222_2222);
}

// --- SYSCFG_COMP ---

const SYSCFG_COMP: u32 = 0x4001_0000;
const EXTICR1: u32 = 0x08;

#[test]
fn syscfg_comp_routing_latches() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    ww(mem, SYSCFG_COMP + EXTICR1, 0x0000_0123);
    assert_eq!(
        rw(mem, SYSCFG_COMP + EXTICR1),
        0x0000_0123,
        "EXTICR1 round-trips"
    );
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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

// --- DAC ---

const DAC: u32 = 0x4000_7400;
const CR_DAC: u32 = 0x00;
const DHR: u32 = 0x08;
const DOR: u32 = 0x2C;
const EN_DAC: u32 = 1 << 0;

#[test]
fn dac_converts_immediately() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    ww(mem, DAC + CR_DAC, EN_DAC);
    ww(mem, DAC + DHR, 0x0ABC);
    pump(mem, 10);
    assert_eq!(rw(mem, DAC + DOR) & 0xFFF, 0xABC, "DOR follows DHR");
}

// --- PWR + CRC windows ---

const PWR: u32 = 0x4000_7000;
const CR_P: u32 = 0x00;
const DBP: u32 = 1 << 8;
const CRC: u32 = 0x4002_3000;
const DR_C: u32 = 0x00;

#[test]
fn pwr_crc_answer() {
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
const CR_S: u32 = 0x00;
const ISR_S: u32 = 0x08;
const ICR_S: u32 = 0x0C;
const CEN_C: u32 = 1 << 5;
const AUTOTRIMEN: u32 = 1 << 6;
const SYNCOKF: u32 = 1 << 0;

#[test]
fn crs_sync_ok_latches() {
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    ww(mem, CRS + CR_S, AUTOTRIMEN | CEN_C);
    pump(mem, 4);
    assert_ne!(rw(mem, CRS + ISR_S) & SYNCOKF, 0, "SYNCOKF latches");
    ww(mem, CRS + ICR_S, SYNCOKF);
    assert_eq!(rw(mem, CRS + ISR_S) & SYNCOKF, 0, "SYNCOKF clears");
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
    let mut m = Machine::armv7m(board::stm32f0x1());
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
    let mut m = Machine::armv7m(board::stm32f0x1());
    let mem = m.core.mem();
    // Generic donor table: F030-family placeholder, not a device claim.
    assert_eq!(rw(mem, DBGMCU + IDCODE), 0x1000_0444, "placeholder IDCODE");
    assert_eq!(rw(mem, DBGMCU + CR_D), 0, "CR resets clear");
    ww(mem, DBGMCU + CR_D, 0x0000_0003);
    assert_eq!(rw(mem, DBGMCU + CR_D), 0x0000_0003, "CR latches");
}

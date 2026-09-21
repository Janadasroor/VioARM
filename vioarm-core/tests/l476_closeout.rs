//! Phase-C `stm32l476` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! Register offsets come from the vendored `STM32L476xx` table
//! (`device::svd::stm32l476`); bit positions mirror the driver sources.

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
const GPIOG: u32 = 0x4800_1800;
const GPIOH: u32 = 0x4800_1C00;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_f_g_h_answer() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    for base in [GPIOF, GPIOG, GPIOH] {
        ww(mem, base + MODER, 1 << 10); // pin 5 output
        ww(mem, base + BSRR, 1 << 5);
        assert_eq!(rw(mem, base + ODR) & (1 << 5), 1 << 5, "BSRR sets ODR");
        ww(mem, base + BSRR, 1 << 21);
        assert_eq!(rw(mem, base + ODR) & (1 << 5), 0, "BSRR resets ODR");
    }
}

// --- UART4 ---

const UART4: u32 = 0x4000_4C00;
const CR1_U: u32 = 0x00;
const ISR_U: u32 = 0x1C;
const RDR: u32 = 0x24;
const TDR: u32 = 0x28;

#[test]
fn uart4_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32l476());
    {
        let mem = m.core.mem();
        ww(mem, UART4 + CR1_U, (1 << 0) | (1 << 3) | (1 << 2)); // UE|TE|RE
        ww(mem, UART4 + TDR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device("uart4"), b"A", "TX drains");
    m.drive_device("uart4", b"Q");
    let mem = m.core.mem();
    pump(mem, 10_000);
    assert_ne!(rw(mem, UART4 + ISR_U) & (1 << 5), 0, "RXNE latches");
    assert_eq!(rw(mem, UART4 + RDR) & 0xFF, u32::from(b'Q'), "RX byte");
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
    let mut m = Machine::armv7m(board::stm32l476());
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

// --- I2C1 ---

const I2C1: u32 = 0x4000_5400;
const CR1_I: u32 = 0x00;
const CR2_I: u32 = 0x04;
const ISR_I: u32 = 0x18;
const PE: u32 = 1 << 0;
const NACKF: u32 = 1 << 4;

#[test]
fn i2c1_nacks_peerless() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, I2C1 + CR1_I, PE);
    ww(mem, I2C1 + CR2_I, 0x0001_2050 | (1 << 16));
    pump(mem, 8);
    pump(mem, 18);
    assert_ne!(rw(mem, I2C1 + ISR_I) & NACKF, 0, "peerless NACKs");
}

// --- ADC1 ---

const ADC1: u32 = 0x5004_0000;
const ISR_A: u32 = 0x00;
const CR_A: u32 = 0x08;
const DR_A: u32 = 0x40;
const ADEN: u32 = 1 << 0;
const ADSTART: u32 = 1 << 2;
const ADRDY: u32 = 1 << 0;
const EOC: u32 = 1 << 2;

#[test]
fn adc1_converts_driven_channel() {
    let mut m = Machine::armv7m(board::stm32l476());
    m.drive_device("adc1", &[208]);
    let mem = m.core.mem();
    ww(mem, ADC1 + CR_A, ADEN);
    pump(mem, 100);
    assert_ne!(rw(mem, ADC1 + ISR_A) & ADRDY, 0, "ADRDY after ADEN");
    ww(mem, ADC1 + CR_A, ADEN | ADSTART);
    pump(mem, 1000);
    assert_ne!(rw(mem, ADC1 + ISR_A) & EOC, 0, "EOC latches");
    assert_eq!(rw(mem, ADC1 + DR_A) & 0xFFF, 208, "driven code lands");
}

// --- DAC ---

const DAC: u32 = 0x4000_7400;
const CR_D: u32 = 0x00;
const DHR12R1: u32 = 0x08;
const DOR1: u32 = 0x2C;
const EN1: u32 = 1 << 0;

#[test]
fn dac1_tracks_holding_register() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, DAC + CR_D, EN1);
    ww(mem, DAC + DHR12R1, 0xABC);
    assert_eq!(rw(mem, DAC + DOR1) & 0xFFF, 0xABC, "DOR follows DHR");
}

// --- DMA1 ---

const DMA1: u32 = 0x4002_0000;
const ISR_D: u32 = 0x00;
const CCR1: u32 = 0x08;
const CNDTR1: u32 = 0x0C;
const CPAR1: u32 = 0x10;
const CMAR1: u32 = 0x14;
const TCIF1: u32 = 1 << 1;

#[test]
fn dma1_mem_to_mem() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    for (i, w) in [0x1111_1111u32, 0x2222_2222].iter().enumerate() {
        // Cast safety: test-only small index.
        #[allow(clippy::cast_possible_truncation)]
        let off = i as u32 * 4;
        ww(mem, 0x2000_0000 + off, *w);
    }
    ww(mem, DMA1 + CPAR1, 0x2000_0000);
    ww(mem, DMA1 + CMAR1, 0x2000_0010);
    ww(mem, DMA1 + CNDTR1, 2);
    // EN | MEM2MEM | MINC | PINC | 32-bit PSIZE/MSIZE.
    ww(
        mem,
        DMA1 + CCR1,
        1 | (1 << 14) | (1 << 7) | (1 << 6) | (2 << 8) | (2 << 10),
    );
    pump(mem, 100);
    assert_eq!(rw(mem, 0x2000_0010), 0x1111_1111, "word 0 copies");
    assert_eq!(rw(mem, 0x2000_0014), 0x2222_2222, "word 1 copies");
    assert_ne!(rw(mem, DMA1 + ISR_D) & TCIF1, 0, "TCIF1 latches");
}

// --- RTC ---

const RTC: u32 = 0x4000_2800;
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
    let mut m = Machine::armv7m(board::stm32l476());
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

// --- LCD ---

const LCD: u32 = 0x4000_2400;
const CR_L: u32 = 0x00;
const FCR_L: u32 = 0x04;
const SR_L: u32 = 0x08;
const CLR_L: u32 = 0x0C;
const RAM0_L: u32 = 0x14;

#[test]
fn lcd_enable_and_update() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, LCD + CR_L, 1); // LCDEN
    assert_ne!(rw(mem, LCD + SR_L) & 0x01, 0, "ENS follows");
    ww(mem, LCD + FCR_L, 0);
    pump(mem, 2);
    assert_ne!(rw(mem, LCD + SR_L) & (1 << 5), 0, "FCRSF syncs");
    ww(mem, LCD + RAM0_L, 0xDEAD_BEEF);
    assert_eq!(rw(mem, LCD + RAM0_L), 0xDEAD_BEEF, "segments latch");
    assert_ne!(rw(mem, LCD + SR_L) & (1 << 3), 0, "UDD on RAM write");
    ww(mem, LCD + CLR_L, 1 << 3);
    assert_eq!(rw(mem, LCD + SR_L) & (1 << 3), 0, "CLR clears UDD");
}

// --- FIREWALL ---

const FWALL: u32 = 0x4001_1C00;
const CSSA: u32 = 0x00;
const CR_FW: u32 = 0x20;

#[test]
fn firewall_arms_and_freezes() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, FWALL + CSSA, 0x0800_0000);
    ww(mem, FWALL + CR_FW, 1); // FPA
    ww(mem, FWALL + CSSA, 0xDEAD_BEEF);
    assert_eq!(rw(mem, FWALL + CSSA), 0x0800_0000, "frozen while armed");
    assert_eq!(rw(mem, FWALL + CR_FW) & 0x02, 0, "no violations");
}

// --- CAN1 ---

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
fn can1_init_filters_and_rx() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, CAN + MCR, INRQ);
    pump(mem, 10);
    assert_ne!(rw(mem, CAN + MSR) & INAK, 0, "INAK answers INRQ");
    ww(mem, CAN + BTR, 0x001C_0000);
    ww(mem, CAN + FMR, FINIT);
    ww(mem, CAN + FA1R, 1);
    ww(mem, CAN + F0R1, 0);
    ww(mem, CAN + F0R1 + 4, 0);
    ww(mem, CAN + FMR, 0);
    ww(mem, CAN + MCR, 0);
    pump(mem, 10);
    assert_eq!(rw(mem, CAN + MSR) & INAK, 0, "normal mode");
    let mut frame = [0u8; 16];
    frame[0..4].copy_from_slice(&0x123u32.to_le_bytes());
    frame[4] = 1;
    frame[8] = 0xAA;
    m.drive_device("can1", &frame);
    let mem = m.core.mem();
    pump(mem, 10);
    assert_eq!(rw(mem, CAN + RF0R) & 0b11, 1, "FMP0 hit");
    assert_eq!(rw(mem, CAN + RI0R), 0x123, "identifier");
    assert_eq!(rw(mem, CAN + RDL0R) & 0xFF, 0xAA, "payload");
}

// --- Flash ---

const FLASH_R: u32 = 0x4002_2000;
const KEYR: u32 = 0x08;
const SR_F: u32 = 0x10;
const CR_F: u32 = 0x14;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const PG: u32 = 1 << 0;
const PER: u32 = 1 << 1;
const STRT: u32 = 1 << 16;
const EOP: u32 = 1 << 0;
const MAIN_FLASH: u32 = 0x0800_0000;

#[test]
fn flash_erase_programs() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, FLASH_R + KEYR, KEY1);
    ww(mem, FLASH_R + KEYR, KEY2);
    // Erase page 0 (PER + PNB=0 + START).
    ww(mem, FLASH_R + CR_F, PER | STRT);
    pump(mem, 100);
    assert_ne!(rw(mem, FLASH_R + SR_F) & EOP, 0, "erase completes");
    ww(mem, FLASH_R + SR_F, EOP);
    ww(mem, FLASH_R + CR_F, PG);
    mem.write(MAIN_FLASH, 0xEF).unwrap();
    mem.write(MAIN_FLASH + 1, 0xBE).unwrap();
    pump(mem, 100);
    assert_ne!(rw(mem, FLASH_R + SR_F) & EOP, 0, "program completes");
    assert_eq!(rw(mem, MAIN_FLASH) & 0xFFFF, 0xBEEF, "halfword lands");
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
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, TSC + CR_C, TSCE | START);
    pump(mem, 4);
    assert_ne!(rw(mem, TSC + ISR_C) & EOAF, 0, "EOAF latches");
    ww(mem, TSC + ICR, EOAF);
    assert_eq!(rw(mem, TSC + ISR_C) & EOAF, 0, "EOAF clears");
}

// --- RNG ---

const RNG: u32 = 0x5006_0800;
const CR_R: u32 = 0x00;
const SR_R: u32 = 0x04;
const DR_R: u32 = 0x08;
const RNGEN: u32 = 1 << 2;
const DRDY: u32 = 1 << 0;

#[test]
fn rng_produces_data() {
    let mut m = Machine::armv7m(board::stm32l476());
    let mem = m.core.mem();
    ww(mem, RNG + CR_R, RNGEN);
    pump(mem, 4);
    assert_ne!(rw(mem, RNG + SR_R) & DRDY, 0, "DRDY latches");
    let a = rw(mem, RNG + DR_R);
    pump(mem, 4);
    let b = rw(mem, RNG + DR_R);
    assert_ne!((a, b), (0, 0), "stream nonzero");
    assert_ne!(a, b, "stream advances");
}

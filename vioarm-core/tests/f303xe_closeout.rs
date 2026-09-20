//! Phase-C `stm32f303xe` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOG`/`H` answer `MODER` + `BSRR`-driven `ODR`.
//! * `UART4`/`5` move bytes CPU-side (new-gen layout).
//! * `RTC` runs the new-generation calendar (init + alarm A).
//! * `SPI3`/`SPI4` shift (peerless `0x00`, staged slave byte).
//! * `I2S2ext`/`I2S3ext` shift as SPI windows (I2S protocol unmodeled).
//! * `I2C3` runs the new-generation engine (peerless NACK).
//! * `ADC3` converts a driven channel; `ADC3_4` latches common control.
//! * `Flash` unlocks, erases, and programs through the gate (2 KiB pages).
//!
//! Register offsets come from the vendored `STM32F303XE` table
//! (`device::svd::stm32f303xe`); bit positions mirror the driver sources.

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

const GPIOG: u32 = 0x4800_1800;
const GPIOH: u32 = 0x4800_1C00;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_g_h_answer() {
    let mut m = Machine::armv7m(board::stm32f303xe());
    let mem = m.core.mem();
    for base in [GPIOG, GPIOH] {
        ww(mem, base + MODER, 0x5555_5555);
        assert_eq!(rw(mem, base + MODER), 0x5555_5555, "MODER latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- UART4/5 (new-gen layout) ---

const UART4: u32 = 0x4000_4C00;
const UART5: u32 = 0x4000_5000;
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
fn uart4_5_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f303xe());
    uart_loopback(&mut m, UART4, "uart4");
    uart_loopback(&mut m, UART5, "uart5");
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
    let mut m = Machine::armv7m(board::stm32f303xe());
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

// --- SPI3/4 + I2S extensions ---

const SPI3: u32 = 0x4000_3C00;
const SPI4: u32 = 0x4001_3C00;
const I2S2EXT: u32 = 0x4000_3400;
const I2S3EXT: u32 = 0x4000_4000;
const CR1_S: u32 = 0x00;
const SR_S: u32 = 0x08;
const DR_S: u32 = 0x0C;
const SPE: u32 = 1 << 6;
const MSTR: u32 = 1 << 2;
const SSI: u32 = 1 << 8;
const SSM: u32 = 1 << 9;
const RXNE_S: u32 = 1 << 0;

fn spi_shifts(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1_S, SPE | MSTR | SSI | SSM);
        ww(mem, base + DR_S, 0xA5);
        pump(mem, 100);
        assert_ne!(rw(mem, base + SR_S) & RXNE_S, 0, "RXNE after frame");
        assert_eq!(rw(mem, base + DR_S) & 0xFF, 0x00, "peerless reads zero");
    }
    m.drive_device(name, &[0x5A]);
    let byte = {
        let mem = m.core.mem();
        ww(mem, base + DR_S, 0xA5);
        pump(mem, 100);
        rw(mem, base + DR_S) & 0xFF
    };
    assert_eq!(byte, 0x5A, "slave byte");
}

#[test]
fn spi3_4_and_i2s_ext_shift() {
    let mut m = Machine::armv7m(board::stm32f303xe());
    spi_shifts(&mut m, SPI3, "spi3");
    spi_shifts(&mut m, SPI4, "spi4");
    spi_shifts(&mut m, I2S2EXT, "i2s2ext");
    spi_shifts(&mut m, I2S3EXT, "i2s3ext");
}

// --- I2C3 (new-gen engine) ---

const I2C3: u32 = 0x4000_7800;
const CR1_I: u32 = 0x00;
const CR2_I: u32 = 0x04;
const ISR_I: u32 = 0x18;
const PE: u32 = 1 << 0;
const NACKF: u32 = 1 << 4;

#[test]
fn i2c3_nacks_peerless() {
    let mut m = Machine::armv7m(board::stm32f303xe());
    let mem = m.core.mem();
    ww(mem, I2C3 + CR1_I, PE);
    ww(mem, I2C3 + CR2_I, 0x0001_2050 | (1 << 16));
    pump(mem, 8);
    pump(mem, 18);
    assert_ne!(rw(mem, I2C3 + ISR_I) & NACKF, 0, "peerless NACKs");
}

// --- ADC3 + ADC3_4 common ---

const ADC3: u32 = 0x5000_0400;
const ADC34: u32 = 0x5000_0700;
const ISR_A: u32 = 0x00;
const CR_A: u32 = 0x08;
const DR_A: u32 = 0x40;
const CCR: u32 = 0x08;
const ADEN: u32 = 1 << 0;
const ADSTART: u32 = 1 << 2;
const ADRDY: u32 = 1 << 0;
const EOC: u32 = 1 << 2;

#[test]
fn adc3_converts_and_common_latches() {
    let mut m = Machine::armv7m(board::stm32f303xe());
    m.drive_device("adc3", &[208]);
    let mem = m.core.mem();
    ww(mem, ADC34 + CCR, 0x0000_0000);
    assert_eq!(rw(mem, ADC34 + CCR), 0, "CCR resets clear");
    ww(mem, ADC3 + CR_A, ADEN);
    pump(mem, 100);
    assert_ne!(rw(mem, ADC3 + ISR_A) & ADRDY, 0, "ADRDY after ADEN");
    ww(mem, ADC3 + CR_A, ADEN | ADSTART);
    pump(mem, 1000);
    assert_ne!(rw(mem, ADC3 + ISR_A) & EOC, 0, "EOC latches");
    assert_eq!(rw(mem, ADC3 + DR_A) & 0xFFF, 208, "driven code lands");
}

// --- Flash (2 KiB pages) ---

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
    let mut m = Machine::armv7m(board::stm32f303xe());
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

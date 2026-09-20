//! Phase-C `stm32f105` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOD`/`GPIOE` answer `CRL` + `BSRR`-driven `ODR` (F1 layout).
//! * `UART4`/`UART5` move bytes CPU-side (F1 `SR`/`DR`).
//! * `TIM1`/`4`–`7` count to overflow.
//! * `I2C1` runs the F1 engine (peerless `AF`).
//! * `SPI2`/`SPI3` shift (peerless `0x00`, staged slave byte).
//! * `ADC2` converts a driven channel (F1 layout).
//! * `DAC` converts immediately (`DHR` → `DOR`).
//! * `RTC`/`BKP` answer (backup register round-trips the domain).
//! * `USB_OTG_FS` runs the device-mode init path.
//! * `PWR`/`CRC` answer their windows (`CR` latch, computed `DR`).
//!
//! Register offsets come from the vendored `STM32F105` table
//! (`device::svd::stm32f105`); bit positions mirror the driver sources.

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

// --- GPIO (F1 layout) ---

const GPIOD: u32 = 0x4001_1400;
const GPIOE: u32 = 0x4001_1800;
const CRL: u32 = 0x00;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;

#[test]
fn gpio_d_e_answer() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    for base in [GPIOD, GPIOE] {
        ww(mem, base + CRL, 0x3333_3333);
        assert_eq!(rw(mem, base + CRL), 0x3333_3333, "CRL latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- UART4/5 (F1 SR/DR) ---

const UART4: u32 = 0x4000_4C00;
const UART5: u32 = 0x4000_5000;
const SR: u32 = 0x00;
const DR: u32 = 0x04;
const CR1: u32 = 0x0C;
const UE: u32 = 1 << 13;
const TE: u32 = 1 << 3;
const RE: u32 = 1 << 2;
const RXNE: u32 = 1 << 5;

fn uart_loopback(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1, UE | TE | RE);
        ww(mem, base + DR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device(name), b"A", "TX drains");
    m.drive_device(name, b"Q");
    let (sr, byte) = {
        let mem = m.core.mem();
        pump(mem, 10_000);
        let mut sr = 0;
        for _ in 0..100 {
            sr = rw(mem, base + SR);
            if sr & RXNE != 0 {
                break;
            }
            pump(mem, 100);
        }
        (sr, rw(mem, base + DR) & 0xFF)
    };
    assert_ne!(sr & RXNE, 0, "RXNE latches");
    assert_eq!(byte, u32::from(b'Q'), "RX byte");
}

#[test]
fn uart4_5_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f105());
    uart_loopback(&mut m, UART4, "uart4");
    uart_loopback(&mut m, UART5, "uart5");
}

// --- Timers ---

const TIM1: u32 = 0x4001_2C00;
const TIM4: u32 = 0x4000_0800;
const TIM5: u32 = 0x4000_0C00;
const TIM6: u32 = 0x4000_1000;
const TIM7: u32 = 0x4000_1400;
const CR1_T: u32 = 0x00;
const SR_T: u32 = 0x10;
const ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;
const UIF: u32 = 1 << 0;

#[test]
fn timers_count() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    for base in [TIM1, TIM4, TIM5, TIM6, TIM7] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM1, TIM4, TIM5, TIM6, TIM7] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- I2C1 (F1 engine) ---

const I2C1: u32 = 0x4000_5400;
const CR1_I: u32 = 0x00;
const SR1_I: u32 = 0x14;
const DR_I: u32 = 0x10;
const PE: u32 = 1 << 0;
const START: u32 = 1 << 8;
const SB: u32 = 1 << 0;
const AF: u32 = 1 << 10;

#[test]
fn i2c1_acks_absent_slave_with_af() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    ww(mem, I2C1 + CR1_I, PE | START);
    pump(mem, 8);
    assert_ne!(rw(mem, I2C1 + SR1_I) & SB, 0, "SB rises");
    ww(mem, I2C1 + DR_I, 0xA0);
    pump(mem, 8);
    assert_ne!(rw(mem, I2C1 + SR1_I) & AF, 0, "peerless AF latches");
}

// --- SPI2/3 ---

const SPI2: u32 = 0x4000_3800;
const SPI3: u32 = 0x4000_3C00;
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
fn spi2_3_shift() {
    let mut m = Machine::armv7m(board::stm32f105());
    spi_shifts(&mut m, SPI2, "spi2");
    spi_shifts(&mut m, SPI3, "spi3");
}

// --- ADC2 (F1 layout) ---

const ADC2: u32 = 0x4001_2800;
const SR_A: u32 = 0x00;
const CR2_A: u32 = 0x08;
const DR_A: u32 = 0x4C;
const ADON: u32 = 1 << 0;
const SWSTART: u32 = 1 << 22;
const EOC: u32 = 1 << 1;

#[test]
fn adc2_converts_driven_channel() {
    let mut m = Machine::armv7m(board::stm32f105());
    m.drive_device("adc2", &[208]);
    let mem = m.core.mem();
    ww(mem, ADC2 + CR2_A, ADON);
    pump(mem, 100);
    ww(mem, ADC2 + CR2_A, ADON | SWSTART);
    pump(mem, 1000);
    assert_ne!(rw(mem, ADC2 + SR_A) & EOC, 0, "EOC latches");
    assert_eq!(rw(mem, ADC2 + DR_A) & 0xFFF, 208, "driven code lands");
}

// --- DAC ---

const DAC: u32 = 0x4000_7400;
const CR_DAC: u32 = 0x00;
const DHR: u32 = 0x08;
const DOR: u32 = 0x2C;
const EN_DAC: u32 = 1 << 0;

#[test]
fn dac_converts_immediately() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    ww(mem, DAC + CR_DAC, EN_DAC);
    ww(mem, DAC + DHR, 0x0ABC);
    pump(mem, 10);
    assert_eq!(rw(mem, DAC + DOR) & 0xFFF, 0xABC, "DOR follows DHR");
}

// --- RTC/BKP ---

const BKP: u32 = 0x4000_6C04;
const BKP_DR1: u32 = 0x00;

#[test]
fn bkp_domain_roundtrips() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    ww(mem, BKP + BKP_DR1, 0x1234);
    assert_eq!(rw(mem, BKP + BKP_DR1), 0x1234, "backup register holds");
}

// --- USB OTG FS device-mode init ---

const FS: u32 = 0x5000_0000;
const GUSBCFG: u32 = 0x0C;
const GRSTCTL: u32 = 0x10;
const GAHBCFG: u32 = 0x08;
const FDMOD: u32 = 1 << 30;
const CSRST: u32 = 1;
const GINT: u32 = 1;

#[test]
fn usb_otg_fs_init_path() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    // Force device mode; a soft reset returns the core to reset values.
    ww(mem, FS + GUSBCFG, FDMOD);
    assert_eq!(rw(mem, FS + GUSBCFG) & FDMOD, FDMOD, "FDMOD holds");
    ww(mem, FS + GRSTCTL, CSRST);
    pump(mem, 4);
    assert_eq!(
        rw(mem, FS + GUSBCFG) & FDMOD,
        0,
        "soft reset restores GUSBCFG"
    );
    ww(mem, FS + GUSBCFG, FDMOD);
    ww(mem, FS + GAHBCFG, GINT);
    assert_eq!(rw(mem, FS + GAHBCFG) & GINT, GINT, "GINT latches");
}

// --- PWR + CRC windows ---

const PWR: u32 = 0x4000_7000;
const CR_P: u32 = 0x00;
const DBP: u32 = 1 << 8;
const CRC: u32 = 0x4002_3000;
const DR_C: u32 = 0x00;

#[test]
fn pwr_crc_answer() {
    let mut m = Machine::armv7m(board::stm32f105());
    let mem = m.core.mem();
    ww(mem, PWR + CR_P, DBP);
    assert_eq!(rw(mem, PWR + CR_P) & DBP, DBP, "DBP latches");
    let before = rw(mem, CRC + DR_C);
    ww(mem, CRC + DR_C, 0x01);
    let after = rw(mem, CRC + DR_C);
    assert_ne!(before, after, "CRC engine runs");
}

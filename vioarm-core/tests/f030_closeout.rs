//! Phase-C `stm32f030` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOD`/`GPIOF` answer `MODER` + `BSRR`-driven `ODR`.
//! * `USART2` moves bytes CPU-side (TX drains, RX injects, new-gen layout).
//! * `SPI2` shifts (peerless `0x00`, staged slave byte).
//! * `I2C2` runs the new-generation engine (peerless NACK).
//! * `TIM6`/`TIM14`–`TIM17` count to overflow.
//! * `RTC` runs the new-generation calendar (init, `TR`, alarm A).
//! * `Flash` unlocks, erases, and programs through the gate.
//! * `DBGMCU` reports the documented `IDCODE`.
//!
//! Register offsets come from the vendored `STM32F030` table
//! (`device::svd::stm32f030`); bit positions mirror the driver sources.

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

const GPIOD: u32 = 0x4800_0C00;
const GPIOF: u32 = 0x4800_1400;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_d_f_answer() {
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    for base in [GPIOD, GPIOF] {
        ww(mem, base + MODER, 0x5555_5555);
        assert_eq!(rw(mem, base + MODER), 0x5555_5555, "MODER latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- USART2 (new-gen layout) ---

const USART2: u32 = 0x4000_4400;
const CR1: u32 = 0x00;
const ISR: u32 = 0x1C;
const RDR: u32 = 0x24;
const TDR: u32 = 0x28;
const UE: u32 = 1 << 0;
const RE: u32 = 1 << 2;
const TE: u32 = 1 << 3;
const RXNE: u32 = 1 << 5;

#[test]
fn usart2_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f030());
    {
        let mem = m.core.mem();
        ww(mem, USART2 + CR1, UE | RE | TE);
        ww(mem, USART2 + TDR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device("usart2"), b"A", "TX drains");
    m.drive_device("usart2", b"Q");
    let (sr, byte) = {
        let mem = m.core.mem();
        pump(mem, 10_000);
        let mut sr = 0;
        for _ in 0..100 {
            sr = rw(mem, USART2 + ISR);
            if sr & RXNE != 0 {
                break;
            }
            pump(mem, 100);
        }
        (sr, rw(mem, USART2 + RDR) & 0xFF)
    };
    assert_ne!(sr & RXNE, 0, "RXNE latches");
    assert_eq!(byte, u32::from(b'Q'), "RX byte");
}

// --- SPI2 ---

const SPI2: u32 = 0x4000_3800;
const CR1_S: u32 = 0x00;
const SR_S: u32 = 0x08;
const DR_S: u32 = 0x0C;
const SPE: u32 = 1 << 6;
const MSTR: u32 = 1 << 2;
const SSI: u32 = 1 << 8;
const SSM: u32 = 1 << 9;
const RXNE_S: u32 = 1 << 0;

#[test]
fn spi2_shifts() {
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    ww(mem, SPI2 + CR1_S, SPE | MSTR | SSI | SSM);
    // Peerless: MISO floats low.
    ww(mem, SPI2 + DR_S, 0xA5);
    pump(mem, 100);
    assert_ne!(rw(mem, SPI2 + SR_S) & RXNE_S, 0, "RXNE after frame");
    assert_eq!(rw(mem, SPI2 + DR_S) & 0xFF, 0x00, "peerless reads zero");
    // Staged slave byte comes back instead.
    m.drive_device("spi2", &[0x5A]);
    let byte = {
        let mem = m.core.mem();
        ww(mem, SPI2 + DR_S, 0xA5);
        pump(mem, 100);
        rw(mem, SPI2 + DR_S) & 0xFF
    };
    assert_eq!(byte, 0x5A, "slave byte");
}

// --- I2C2 (new-gen engine) ---

const I2C2: u32 = 0x4000_5800;
const CR1_I: u32 = 0x00;
const CR2_I: u32 = 0x04;
const ISR_I: u32 = 0x18;
const PE: u32 = 1 << 0;
const NACKF: u32 = 1 << 4;

#[test]
fn i2c2_nacks_peerless() {
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    ww(mem, I2C2 + CR1_I, PE);
    // START, SADD=0x50 write, NBYTES=1.
    ww(mem, I2C2 + CR2_I, 0x0001_2050 | (1 << 16));
    pump(mem, 8);
    pump(mem, 18);
    assert_ne!(rw(mem, I2C2 + ISR_I) & NACKF, 0, "peerless NACKs");
}

// --- Timers ---

const TIM6: u32 = 0x4000_1000;
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
fn tim6_14_thru_17_count() {
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    for base in [TIM6, TIM14, TIM15, TIM16, TIM17] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM6, TIM14, TIM15, TIM16, TIM17] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- RTC (new-gen calendar) ---

const RTC: u32 = 0x4000_2800;
const TR: u32 = 0x00;
const CR_R: u32 = 0x08;
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
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    // Unlock, enter init, set midnight, leave.
    ww(mem, RTC + WPR, 0xCA);
    ww(mem, RTC + WPR, 0x53);
    ww(mem, RTC + ISR_R, INIT);
    assert_ne!(rw(mem, RTC + ISR_R) & INITF, 0, "INITF in init");
    ww(mem, RTC + TR, 0x00_0000);
    ww(mem, RTC + ISR_R, 0);
    assert_ne!(rw(mem, RTC + ISR_R) & INITS, 0, "INITS after init");
    // Alarm A at seconds 03 with its IRQ armed.
    ww(mem, RTC + ALRMAR, 0x03);
    ww(mem, RTC + ISR_R, ALRAIE);
    pump(mem, 1);
    pump(mem, 1);
    pump(mem, 1);
    assert_eq!(rw(mem, RTC + TR) & 0xF, 3, "TR.SU == 3");
    assert_ne!(rw(mem, RTC + ISR_R) & ALRAF, 0, "ALRAF fires");
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
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    // Unlock through the key sequence.
    ww(mem, FLASH_R + KEYR, KEY1);
    ww(mem, FLASH_R + KEYR, KEY2);
    // Erase page 0.
    ww(mem, FLASH_R + CR_F, PER);
    ww(mem, FLASH_R + AR, MAIN_FLASH);
    ww(mem, FLASH_R + CR_F, PER | STRT);
    pump(mem, 100);
    assert_ne!(rw(mem, FLASH_R + SR_F) & EOP, 0, "erase completes");
    ww(mem, FLASH_R + SR_F, 0);
    // Program one halfword.
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
fn dbgmcu_reports_f030x6() {
    let mut m = Machine::armv7m(board::stm32f030());
    let mem = m.core.mem();
    assert_eq!(rw(mem, DBGMCU + IDCODE), 0x1000_0444, "REV_ID + DEV_ID");
    assert_eq!(rw(mem, DBGMCU + CR_D), 0, "CR resets clear");
    ww(mem, DBGMCU + CR_D, 0x0000_0003);
    assert_eq!(rw(mem, DBGMCU + CR_D), 0x0000_0003, "CR latches");
}

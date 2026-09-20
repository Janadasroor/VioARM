//! Phase-C `stm32f100` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOD`–`GPIOG` answer `CRL`/`CRH` + `BSRR`-driven `ODR` (F1 layout).
//! * `UART4`/`UART5` move bytes CPU-side (F1 `SR`/`DR`).
//! * `TIM1`/`5`–`7`/`12`–`17` count to overflow.
//! * `I2C2` runs the F1 engine (peerless `AF`).
//! * `SPI2`/`SPI3` shift (peerless `0x00`, staged slave byte).
//! * `DAC` converts immediately (`DHR` → `DOR`).
//! * `CEC` runs the F1 engine (`TXD` + `TSOM` → `TEOM`).
//! * `RTC`/`BKP` answer (backup register round-trips the domain).
//! * `DMA2` copies memory-to-memory with `TCIF1`.
//! * `FSMC` gates its banks, while `DWT`/`SysTick` stay reachable
//!   (regression guard: the table's `0xfffff400` window must not shadow
//!   the core block).
//!
//! Register offsets come from the vendored `STM32F100` table
//! (`device::svd::stm32f100`); bit positions mirror the driver sources.

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
const GPIOF: u32 = 0x4001_1C00;
const GPIOG: u32 = 0x4001_2000;
const CRL: u32 = 0x00;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;

#[test]
fn gpio_d_thru_g_answer() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    for base in [GPIOD, GPIOE, GPIOF, GPIOG] {
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
    let mut m = Machine::armv7m(board::stm32f100());
    uart_loopback(&mut m, UART4, "uart4");
    uart_loopback(&mut m, UART5, "uart5");
}

// --- Timers ---

const TIM1: u32 = 0x4001_2C00;
const TIM5: u32 = 0x4000_0C00;
const TIM6: u32 = 0x4000_1000;
const TIM7: u32 = 0x4000_1400;
const TIM12: u32 = 0x4000_1800;
const TIM13: u32 = 0x4000_1C00;
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
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    for base in [
        TIM1, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17,
    ] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [
        TIM1, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17,
    ] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- I2C2 (F1 engine) ---

const I2C2: u32 = 0x4000_5800;
const CR1_I: u32 = 0x00;
const SR1_I: u32 = 0x14;
const DR_I: u32 = 0x10;
const PE: u32 = 1 << 0;
const START: u32 = 1 << 8;
const SB: u32 = 1 << 0;
const AF: u32 = 1 << 10;

#[test]
fn i2c2_acks_absent_slave_with_af() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    ww(mem, I2C2 + CR1_I, PE | START);
    pump(mem, 8);
    assert_ne!(rw(mem, I2C2 + SR1_I) & SB, 0, "SB rises");
    ww(mem, I2C2 + DR_I, 0xA0);
    pump(mem, 8);
    assert_ne!(rw(mem, I2C2 + SR1_I) & AF, 0, "peerless AF latches");
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
        // Reading DR clears RXNE (else the next frame overruns by design).
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
    let mut m = Machine::armv7m(board::stm32f100());
    spi_shifts(&mut m, SPI2, "spi2");
    spi_shifts(&mut m, SPI3, "spi3");
}

// --- DAC ---

const DAC: u32 = 0x4000_7400;
const CR_DAC: u32 = 0x00;
const DHR: u32 = 0x08;
const DOR: u32 = 0x2C;
const EN_DAC: u32 = 1 << 0;

#[test]
fn dac_converts_immediately() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    ww(mem, DAC + CR_DAC, EN_DAC);
    ww(mem, DAC + DHR, 0x0ABC);
    pump(mem, 10);
    assert_eq!(rw(mem, DAC + DOR) & 0xFFF, 0xABC, "DOR follows DHR");
}

// --- CEC (F1 engine) ---

const CEC: u32 = 0x4000_7800;
const CFGR: u32 = 0x00;
const TXD: u32 = 0x14;
const CSR: u32 = 0x10;
const PE_C: u32 = 1 << 0;
const IE_C: u32 = 1 << 1;
const TSOM: u32 = 1 << 0;
const TEOM: u32 = 1 << 1;

#[test]
fn cec_f1_tx_handshake() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    ww(mem, CEC + CFGR, PE_C | IE_C);
    ww(mem, CEC + TXD, 0x55);
    ww(mem, CEC + CSR, TSOM);
    pump(mem, 4);
    let csr = rw(mem, CEC + CSR);
    assert_ne!(csr & TEOM, 0, "TEOM latches");
    assert_eq!(csr & TSOM, 0, "TSOM self-clears");
    assert_eq!(m.drain_device("cec"), [0x55], "byte drains");
}

// --- RTC/BKP ---

const BKP: u32 = 0x4000_6C04;
const BKP_DR1: u32 = 0x00;

#[test]
fn bkp_domain_roundtrips() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    ww(mem, BKP + BKP_DR1, 0x1234);
    assert_eq!(rw(mem, BKP + BKP_DR1), 0x1234, "backup register holds");
}

// --- DMA2 ---

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

#[test]
fn dma2_mem2mem_copy() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    for (i, w) in [0x1111_1111u32, 0x2222_2222].iter().enumerate() {
        ww(mem, SRC + (i as u32) * 4, *w);
    }
    ww(mem, DMA2 + CPAR1, SRC);
    ww(mem, DMA2 + CMAR1, DST);
    ww(mem, DMA2 + CNDTR1, 2);
    ww(
        mem,
        DMA2 + CCR1,
        EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
    );
    pump(mem, 10);
    assert_eq!(rw(mem, DST), 0x1111_1111, "word 0 lands");
    assert_eq!(rw(mem, DST + 4), 0x2222_2222, "word 1 lands");
    assert_ne!(rw(mem, DMA2 + ISR_D) & TCIF1, 0, "TCIF1 latches");
}

// --- FSMC + core-block reachability ---

const FSMC: u32 = 0xA000_0000;
const BCR1: u32 = 0x00;
const MBKEN: u32 = 1 << 0;
const NOR1: u32 = 0x6000_0000;
const DWT_CTRL: u32 = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;
const LARGO: u32 = 1 << 0;
const SYST_RVR: u32 = 0xE000_E014;

#[test]
fn fsmc_gates_without_shadowing_core() {
    let mut m = Machine::armv7m(board::stm32f100());
    let mem = m.core.mem();
    // Enable NOR bank 1: the external window floats.
    ww(mem, FSMC + BCR1, 0x0000_30D0 | MBKEN);
    assert_eq!(rw(mem, NOR1), 0xFFFF_FFFF, "enabled bank floats");
    // The core block stays reachable past the (trimmed) FSMC window:
    // DWT CYCCNT is R/W, SysTick RVR latches.
    ww(mem, DWT_CTRL, LARGO);
    ww(mem, DWT_CYCCNT, 0x1234_5678);
    assert_eq!(rw(mem, DWT_CYCCNT), 0x1234_5678, "DWT answers");
    ww(mem, SYST_RVR, 0x00FF_FFFF);
    assert_eq!(rw(mem, SYST_RVR), 0x00FF_FFFF, "SysTick answers");
}

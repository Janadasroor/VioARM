//! Phase-C `stm32f103` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOD`/`E`/`F`/`G` answer config + `BSRR`-driven `ODR`.
//! * `UART4`/`UART5` move bytes CPU-side (TX drains, RX injects).
//! * `TIM5` is 32-bit wide; `TIM8` routes update to line 1 (break-first,
//!   like `TIM1`) raising `TIM8_UP` (IRQ 44); `TIM9`–`TIM14` count.
//! * `DMA2` copies memory-to-memory and raises `TCIF1`.
//! * `CAN` walks the init handshake, holds filter config, and lands an
//!   injected frame in `FIFO0`.
//! * `ADC3` converts the driven bus into RAM through `DMA2` channel 5.
//!
//! Register offsets come from the vendored `STM32F103xx` table
//! (`device::svd::stm32f103`); bit positions mirror the driver sources.

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

const GPIOD: u32 = 0x4001_1400;
const GPIOE: u32 = 0x4001_1800;
const GPIOF: u32 = 0x4001_1C00;
const GPIOG: u32 = 0x4001_2000;
const CRL: u32 = 0x00;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;

#[test]
fn gpio_d_e_f_g_answer() {
    let mut m = Machine::armv7m(board::stm32f103());
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

// --- USART ---

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
fn uart4_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f103());
    uart_loopback(&mut m, UART4, "uart4");
}

#[test]
fn uart5_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f103());
    uart_loopback(&mut m, UART5, "uart5");
}

// --- Timers ---

const TIM5: u32 = 0x4000_0C00;
const TIM8: u32 = 0x4001_3400;
const TIM9: u32 = 0x4001_4C00;
const TIM10: u32 = 0x4001_5000;
const TIM11: u32 = 0x4001_5400;
const TIM12: u32 = 0x4000_1800;
const TIM13: u32 = 0x4000_1C00;
const TIM14: u32 = 0x4000_2000;
const CR1_T: u32 = 0x00;
const DIER: u32 = 0x0C;
const SR_T: u32 = 0x10;
const ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;
const UIE: u32 = 1 << 0;
const UIF: u32 = 1 << 0;
const NVIC_ISPR1: u32 = 0xE000_E204;
const TIM8_UP_BIT: u32 = 1 << (44 - 32);

#[test]
fn tim5_is_32bit_and_counts() {
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    ww(mem, TIM5 + ARR, 0xFFFF_FFFF);
    assert_eq!(
        rw(mem, TIM5 + ARR),
        0xFFFF_FFFF,
        "TIM5 holds a 32-bit period"
    );
    ww(mem, TIM5 + ARR, 10);
    ww(mem, TIM5 + CR1_T, CEN);
    pump(mem, 100_000);
    assert_ne!(rw(mem, TIM5 + SR_T) & UIF, 0, "TIM5 overflows");
    ww(mem, TIM5 + SR_T, 0);
    assert_eq!(rw(mem, TIM5 + SR_T) & UIF, 0, "UIF clears");
}

#[test]
fn tim8_update_raises_line_1() {
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    ww(mem, TIM8 + ARR, 10);
    ww(mem, TIM8 + DIER, UIE);
    ww(mem, TIM8 + CR1_T, CEN);
    pump(mem, 100_000);
    assert_ne!(rw(mem, TIM8 + SR_T) & UIF, 0, "TIM8 overflows");
    assert_ne!(
        rw(mem, NVIC_ISPR1) & TIM8_UP_BIT,
        0,
        "TIM8_UP (IRQ 44) pends on line 1"
    );
}

#[test]
fn tim9_thru_14_count() {
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    for base in [TIM9, TIM10, TIM11, TIM12, TIM13, TIM14] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM9, TIM10, TIM11, TIM12, TIM13, TIM14] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- DMA2 ---

const DMA2: u32 = 0x4002_0400;
const ISR: u32 = 0x00;
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
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    for (i, w) in [0x1111_1111u32, 0x2222_2222, 0x3333_3333, 0x4444_4444]
        .iter()
        .enumerate()
    {
        ww(mem, SRC + (i as u32) * 4, *w);
    }
    ww(mem, DMA2 + CPAR1, SRC);
    ww(mem, DMA2 + CMAR1, DST);
    ww(mem, DMA2 + CNDTR1, 4);
    ww(
        mem,
        DMA2 + CCR1,
        EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
    );
    pump(mem, 10);
    for (i, w) in [0x1111_1111u32, 0x2222_2222, 0x3333_3333, 0x4444_4444]
        .iter()
        .enumerate()
    {
        assert_eq!(rw(mem, DST + (i as u32) * 4), *w, "word {i} lands");
    }
    assert_ne!(rw(mem, DMA2 + ISR) & TCIF1, 0, "TCIF1 latches");
}

// --- CAN ---

const CAN: u32 = 0x4000_6400;
const MCR: u32 = 0x00;
const MSR: u32 = 0x04;
const BTR: u32 = 0x1C;
const FMR: u32 = 0x200;
const FM1R: u32 = 0x204;
const FS1R: u32 = 0x20C;
const FFA1R: u32 = 0x214;
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
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    // Init handshake.
    ww(mem, CAN + MCR, INRQ);
    pump(mem, 10);
    assert_ne!(rw(mem, CAN + MSR) & INAK, 0, "INAK answers INRQ");
    // Baud register holds while held in init.
    ww(mem, CAN + BTR, 0x001C_0000);
    assert_eq!(rw(mem, CAN + BTR), 0x001C_0000, "BTR latches");
    // Bank 0: 32-bit mask mode, accept-all, FIFO0.
    ww(mem, CAN + FMR, FINIT);
    ww(mem, CAN + FA1R, 1);
    let (fs1r, fm1r, ffa1r) = (
        rw(mem, CAN + FS1R),
        rw(mem, CAN + FM1R),
        rw(mem, CAN + FFA1R),
    );
    ww(mem, CAN + FS1R, fs1r | 1);
    ww(mem, CAN + FM1R, fm1r & !1);
    ww(mem, CAN + FFA1R, ffa1r & !1);
    ww(mem, CAN + F0R1, 0);
    ww(mem, CAN + F0R1 + 4, 0);
    ww(mem, CAN + FMR, 0);
    assert_eq!(rw(mem, CAN + F0R1), 0, "filter word holds");
    // Back to normal: INAK drops.
    ww(mem, CAN + MCR, 0);
    pump(mem, 10);
    assert_eq!(rw(mem, CAN + MSR) & INAK, 0, "normal mode");
    let (fmp0, ident, payload) = {
        // One frame on the wire lands in FIFO0.
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

// --- ADC3 + DMA2 ---
const ADC3: u32 = 0x4001_3C00;
const SR_A: u32 = 0x00;
const CR2: u32 = 0x08;
const DR_A: u32 = 0x4C;
const ADON: u32 = 1 << 0;
const CONT: u32 = 1 << 1;
const DMA_BIT: u32 = 1 << 8;
const SWSTART: u32 = 1 << 22;
const EOC: u32 = 1 << 1;
const CCR5: u32 = 0x58;
const CNDTR5: u32 = 0x5C;
const CPAR5: u32 = 0x60;
const CMAR5: u32 = 0x64;
const ADC_DST: u32 = 0x2000_0300;

#[test]
fn adc3_dma_into_ram_via_dma2() {
    let mut m = Machine::armv7m(board::stm32f103());
    // DMA2 channel 5 moves one half-word from ADC3 DR to RAM.
    ww(m.core.mem(), DMA2 + CPAR5, ADC3 + DR_A);
    ww(m.core.mem(), DMA2 + CMAR5, ADC_DST);
    ww(m.core.mem(), DMA2 + CNDTR5, 1);
    ww(m.core.mem(), DMA2 + CCR5, EN | MINC | (1 << 8) | (1 << 10));
    // ADC3 wakes, runs continuously with DMA, and converts the bus.
    ww(m.core.mem(), ADC3 + CR2, ADON | CONT | DMA_BIT);
    pump(m.core.mem(), 100);
    m.analog_drive_mv(1600);
    ww(m.core.mem(), ADC3 + CR2, ADON | CONT | DMA_BIT | SWSTART);
    pump(m.core.mem(), 10_000);
    assert_ne!(rw(m.core.mem(), ADC3 + SR_A) & EOC, 0, "EOC latches");
    assert_eq!(
        rw(m.core.mem(), ADC_DST) & 0xFFFF,
        1985,
        "1600 mV lands through DMA2"
    );
}

// --- FSMC + USB ---

const FSMC_CTRL: u32 = 0xA000_0000;
const BCR1: u32 = 0x00;
const PCR2: u32 = 0x60;
const MBKEN: u32 = 1 << 0;
const PBKEN: u32 = 1 << 2;
const NOR1: u32 = 0x6000_0000;
const NAND2: u32 = 0x7000_0000;

const USB: u32 = 0x4000_5C00;
const CNTR: u32 = 0x40;
const ISTR: u32 = 0x44;
const DADDR: u32 = 0x4C;
const CTRM: u32 = 1 << 15;
const CTR: u32 = 1 << 15;
const EF: u32 = 1 << 7;

#[test]
fn fsmc_banks_gate_at_board_level() {
    use vioarm_core::error::BusError;
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    assert!(
        matches!(mem.read_word(NOR1), Err(BusError::Reserved)),
        "NOR bank 1 gated at reset"
    );
    ww(mem, FSMC_CTRL + BCR1, 0x0000_30D0 | MBKEN);
    assert_eq!(rw(mem, NOR1), 0xFFFF_FFFF, "enabled bank floats");
    assert!(
        matches!(mem.read_word(NAND2), Err(BusError::Reserved)),
        "NAND bank 2 still gated"
    );
    ww(mem, FSMC_CTRL + PCR2, 0x0000_0018 | PBKEN);
    assert_eq!(rw(mem, NAND2), 0xFFFF_FFFF, "enabled NAND floats");
}

#[test]
fn usb_fs_enumeration_tick() {
    let mut m = Machine::armv7m(board::stm32f103());
    let mem = m.core.mem();
    // Address assigned and CTR interrupt armed: the next pump latches CTR.
    ww(mem, USB + DADDR, EF | 0x05);
    assert_eq!(rw(mem, USB + DADDR), EF | 0x05, "DADDR holds");
    ww(mem, USB + CNTR, CTRM);
    pump(mem, 4);
    assert_ne!(rw(mem, USB + ISTR) & CTR, 0, "CTR latches after EF");
    // Write-0 clears the flag, like the toggle/write-clear endpoint flow.
    ww(mem, USB + ISTR, 0);
    assert_eq!(rw(mem, USB + ISTR) & CTR, 0, "CTR clears");
}

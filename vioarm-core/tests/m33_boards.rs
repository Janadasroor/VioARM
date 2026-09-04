//! M33 boards: behaviors live on own-generation U5/H5/WBA/L5 tables.
//!
//! Hand-rolled register pokes (no CMSIS headers are vendored for these
//! families) prove GPIO/USART/TIM/RTC behaviors drive real silicon windows:
//! BSRR sets ODR, USART transmits (TXE/TC), TIM counts, and the RTC WPR
//! unlock + INIT flow works. Runs on the mainline core.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const USART1: u32 = 0x4001_3800;
const TIM2: u32 = 0x4000_0000;
const RTC: u32 = 0x4600_7800;
const GPIOA: u32 = 0x4202_0000;

fn u575() -> Machine<board::BoardMemory> {
    // No firmware: poke registers directly. Peripherals construct reset.
    // (M33 boots Secure; PPB windows are MPU-exempt like the oracle.)
    Machine::armv8m_mainline(board::stm32u575())
}

#[test]
fn gpio_bsrr_sets_odr() {
    let mut m = u575();
    let mem = m.core.mem();
    // PA5 output: MODER bits 11:10 = 01.
    let moder = mem.read_word(GPIOA).unwrap();
    mem.write_word(GPIOA, (moder & !(0x3 << 10)) | (0x1 << 10))
        .unwrap();
    mem.write_word(GPIOA + 0x18, 1 << 5).unwrap();
    assert_eq!(mem.read_word(GPIOA + 0x14).unwrap() & (1 << 5), 1 << 5);
    mem.write_word(GPIOA + 0x18, 1 << (5 + 16)).unwrap();
    assert_eq!(mem.read_word(GPIOA + 0x14).unwrap() & (1 << 5), 0);
}

#[test]
fn usart_transmits_byte() {
    let mut m = u575();
    let mem = m.core.mem();
    // UE + TE, 115200-ish BRR (value immaterial, no baud model).
    mem.write_word(USART1, 0x1 | (0x1 << 3)).unwrap();
    mem.write_word(USART1 + 0x0C, 0x22).unwrap();
    // TXE set when idle and enabled.
    assert_ne!(mem.read_word(USART1 + 0x1C).unwrap() & (1 << 7), 0);
    mem.write_word(USART1 + 0x28, b'Z' as u32).unwrap();
    // The byte queues for output; TC reads set while UE holds.
    let _ = mem;
    assert_eq!(m.drain_channel("usart1"), b"Z");
    assert_ne!(m.core.mem().read_word(USART1 + 0x1C).unwrap() & (1 << 6), 0);
}

#[test]
fn tim2_counts_when_enabled() {
    let mut m = u575();
    let mem = m.core.mem();
    mem.write_word(TIM2 + 0x28, 0).unwrap(); // PSC = 0: every tick
    mem.write_word(TIM2 + 0x2C, 0xFFFF).unwrap(); // ARR
    mem.write_word(TIM2, 0x1).unwrap(); // CEN
    let c0 = mem.read_word(TIM2 + 0x24).unwrap();
    let _ = mem;
    for _ in 0..10 {
        m.step().unwrap();
    }
    let c1 = m.core.mem().read_word(TIM2 + 0x24).unwrap();
    assert!(c1.wrapping_sub(c0) >= 5, "tim advanced {c0:#x} -> {c1:#x}");
}

#[test]
fn fdcan_loopback_roundtrip() {
    // Cut-down fixed layout: filters at RAM+0, RX FIFO0 at RAM+176,
    // TX buffers at RAM+632; TXBAR is at 0xCC here, not 0xD0.
    let mut m = u575();
    const FDCAN1: u32 = 0x4000_A400;
    const RAM: u32 = 0x4000_AC00;
    // INIT + CCE + test mode; RXGFC: one standard filter, accept rest FIFO0.
    m.core.mem().write_word(FDCAN1 + 0x18, 0x83).unwrap();
    m.core
        .mem()
        .write_word(FDCAN1 + 0x80, (1 << 16) | (1 << 4))
        .unwrap();
    m.core.mem().write_word(FDCAN1 + 0x10, 1 << 4).unwrap();
    // TX buffer 0 at RAM+632: std ID 0x123 (T0[28:18]), DLC=8.
    for (i, b) in [0x00u8, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x08, 0x00]
        .into_iter()
        .chain([0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44])
        .enumerate()
    {
        m.core.mem().write(RAM + 632 + i as u32, b).unwrap();
    }
    m.core.mem().write_word(FDCAN1 + 0xCC, 1).unwrap();
    for _ in 0..10 {
        m.step().unwrap();
        if m.core.mem().read_word(FDCAN1 + 0xD4).unwrap() & 1 != 0 {
            break;
        }
    }
    assert_ne!(m.core.mem().read_word(FDCAN1 + 0xD4).unwrap() & 1, 0);
    for _ in 0..10 {
        m.step().unwrap();
        if m.core.mem().read_word(FDCAN1 + 0x90).unwrap() & 0xF != 0 {
            break;
        }
    }
    // Loopback frame in FIFO0 with its payload in message RAM.
    assert_eq!(m.core.mem().read_word(FDCAN1 + 0x90).unwrap() & 0xF, 1);
    let mut payload = vec![0u8; 8];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = m.core.mem().read(RAM + 176 + 8 + i as u32).unwrap();
    }
    assert_eq!(payload, &[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]);
}

#[test]
fn rtc_unlock_and_init_flow() {
    let mut m = u575();
    let mem = m.core.mem();
    // WPR unlock: 0xCA then 0x53.
    mem.write_word(RTC + 0x24, 0xCA).unwrap();
    mem.write_word(RTC + 0x24, 0x53).unwrap();
    // Enter init, set 12:34:56-ish seconds field, leave init.
    let isr = mem.read_word(RTC + 0x0C).unwrap();
    mem.write_word(RTC + 0x0C, isr | (1 << 7)).unwrap();
    assert_ne!(mem.read_word(RTC + 0x0C).unwrap() & (1 << 6), 0);
    mem.write_word(RTC, 0x56).unwrap();
    assert_eq!(mem.read_word(RTC).unwrap() & 0xFF, 0x56);
    mem.write_word(RTC + 0x0C, isr & !(1 << 7)).unwrap();
    assert_ne!(mem.read_word(RTC + 0x0C).unwrap() & (1 << 4), 0);
}

//! `stm32g474xx` close-out: 82/82 behavioral, zero latch.
//!
//! Board-level pokes through a live `Machine` proving the G4-native
//! models agree end to end: FDCAN1/2/3 loopback round-trips over the
//! shared message RAM, one CORDIC cosine, one FMAC FIR tap, the TAMP
//! backup/tamper pair, a UCPD frame send, and a GPIO spot check.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

fn step_until(m: &mut Machine<board::BoardMemory>, addr: u32, mask: u32, want: u32) {
    for _ in 0..200 {
        m.step().unwrap();
        if rw(m.core.mem(), addr) & mask == want {
            return;
        }
    }
    panic!("{addr:#x} never reached {want:#x}");
}

/// One cut-down loopback round-trip on `base` with message RAM at `ram`.
fn fdcan_loopback(m: &mut Machine<board::BoardMemory>, base: u32, ram: u32) {
    // INIT + CCE + test mode; RXGFC: one standard filter, rest to FIFO0.
    ww(m.core.mem(), base + 0x18, 0x83);
    ww(m.core.mem(), base + 0x80, (1 << 16) | (1 << 4));
    ww(m.core.mem(), base + 0x10, 1 << 4);
    // TX buffer 0 at TFQSA=632: std ID 0x123, DLC=2, payload DE AD.
    for (i, b) in [0x00u8, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x02, 0x00]
        .into_iter()
        .chain([0xDE, 0xAD])
        .enumerate()
    {
        m.core.mem().write(ram + 632 + i as u32, b).unwrap();
    }
    ww(m.core.mem(), base + 0xCC, 1);
    step_until(&mut *m, base + 0xD4, 1, 1);
    step_until(&mut *m, base + 0x90, 0xF, 1);
    assert_eq!(m.core.mem().read(ram + 176 + 8).unwrap(), 0xDE);
    assert_eq!(m.core.mem().read(ram + 176 + 9).unwrap(), 0xAD);
}

#[test]
fn fdcan1_loopback() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    fdcan_loopback(&mut m, 0x4000_6400, 0x4000_A400);
}

#[test]
fn fdcan2_loopback() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    fdcan_loopback(&mut m, 0x4000_6800, 0x4000_A400 + 848);
}

#[test]
fn fdcan3_loopback() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    fdcan_loopback(&mut m, 0x4000_6C00, 0x4000_A400 + 2 * 848);
}

#[test]
fn cordic_cos60() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    const CORDIC: u32 = 0x4002_0C00;
    // Cosine, two results, one argument; θ = 60°.
    ww(m.core.mem(), CORDIC, 1 << 19);
    ww(m.core.mem(), CORDIC + 0x04, 715827883);
    step_until(&mut m, CORDIC, 1 << 31, 1 << 31);
    let cos = rw(m.core.mem(), CORDIC + 0x08);
    let sin = rw(m.core.mem(), CORDIC + 0x08);
    assert!((cos as i32 - 0x4000_0000).abs() <= 64, "cos60 {cos:#x}");
    assert!((sin as i32 - 1859775393).abs() <= 64, "sin60 {sin:#x}");
}

#[test]
fn fmac_fir_average() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    const FMAC: u32 = 0x4002_1400;
    // y[n] = 0.5·x[n] + 0.5·x[n-1].
    ww(m.core.mem(), FMAC, 8 << 8);
    ww(m.core.mem(), FMAC + 0x04, 8 << 8);
    ww(m.core.mem(), FMAC + 0x0C, 2 << 24);
    ww(m.core.mem(), FMAC + 0x18, 0x4000);
    ww(m.core.mem(), FMAC + 0x18, 0x4000);
    ww(m.core.mem(), FMAC + 0x0C, (8 << 24) | 2 | (1 << 31));
    ww(m.core.mem(), FMAC + 0x18, 0x4000);
    ww(m.core.mem(), FMAC + 0x18, 0x4000);
    assert_eq!(
        rw(m.core.mem(), FMAC + 0x1C) & 0xFFFF,
        0x4000,
        "(0.5+0.5)/2"
    );
    ww(m.core.mem(), FMAC + 0x18, 0x0000);
    assert_eq!(rw(m.core.mem(), FMAC + 0x1C) & 0xFFFF, 0x2000, "(0.5+0)/2");
}

#[test]
fn tamp_backup_and_tamper() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    const TAMP: u32 = 0x4000_2400;
    ww(m.core.mem(), TAMP + 0x100, 0x1234_5678);
    assert_eq!(rw(m.core.mem(), TAMP + 0x100), 0x1234_5678, "BKP0R latches");
    ww(m.core.mem(), TAMP, 1); // TAMP1E
    assert!(m.drive_device("tamp", &[0x01]), "tamp host channel");
    for _ in 0..10 {
        m.step().unwrap();
    }
    assert_ne!(rw(m.core.mem(), TAMP + 0x30) & 1, 0, "TAMP1F latches");
    assert_eq!(rw(m.core.mem(), TAMP + 0x100), 0, "tamper wipes backup");
    ww(m.core.mem(), TAMP + 0x3C, 1);
    assert_eq!(rw(m.core.mem(), TAMP + 0x30) & 1, 0, "SCR clears");
}

#[test]
fn ucpd_frame_sends() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    const UCPD: u32 = 0x4000_A000;
    ww(m.core.mem(), UCPD + 0x0C, 1 << 10); // CC1
    ww(m.core.mem(), UCPD + 0x1C, 0x1111);
    ww(m.core.mem(), UCPD + 0x20, 2);
    ww(m.core.mem(), UCPD + 0x24, 0xC0FF_EE00);
    ww(m.core.mem(), UCPD + 0x0C, (1 << 10) | (1 << 2)); // TXSEND
    step_until(&mut m, UCPD + 0x14, 1 << 2, 1 << 2);
    assert_eq!(
        m.drain_device("ucpd"),
        [0x11, 0x11, 0x00, 0x00, 0x00, 0xEE],
        "ordset LE + paysz bytes"
    );
}

#[test]
fn gpioa_bsrr() {
    let mut m = Machine::armv7m(board::stm32g474xx());
    const GPIOA: u32 = 0x4800_0000;
    ww(m.core.mem(), GPIOA, 1 << 10); // PA5 output
    ww(m.core.mem(), GPIOA + 0x18, 1 << 5);
    assert_eq!(
        rw(m.core.mem(), GPIOA + 0x14) & (1 << 5),
        1 << 5,
        "BSRR sets ODR"
    );
    ww(m.core.mem(), GPIOA + 0x18, 1 << 21);
    assert_eq!(
        rw(m.core.mem(), GPIOA + 0x14) & (1 << 5),
        0,
        "BSRR resets ODR"
    );
}

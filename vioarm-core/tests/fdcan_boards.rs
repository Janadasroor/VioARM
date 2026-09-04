//! FDCAN board wiring: loopback round-trips through real bus plumbing.
//!
//! The cut-down path runs on a G0C1 board (M0+ core, verified CAN RAM at
//! `0x4000B400`); the full path on an H743 board (M7 core, RAM at
//! `0x4000AC00`). Both programs are hand-rolled register pokes proving the
//! mailbox, RAM storage, and behavior agree end to end.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn step_until(m: &mut Machine<board::BoardMemory>, addr: u32, mask: u32, want: u32) {
    for _ in 0..50 {
        m.step().unwrap();
        if m.core.mem().read_word(addr).unwrap() & mask == want {
            return;
        }
    }
    panic!("{addr:#x} never reached {want:#x}");
}

#[test]
fn g0c1_cut_loopback_roundtrip() {
    let mut m = Machine::new(board::stm32g0c1());
    const FDCAN1: u32 = 0x4000_6400;
    const RAM: u32 = 0x4000_B400;
    // INIT + CCE + test mode; RXGFC: one standard filter, accept rest FIFO0.
    m.core.mem().write_word(FDCAN1 + 0x18, 0x83).unwrap();
    m.core
        .mem()
        .write_word(FDCAN1 + 0x80, (1 << 16) | (1 << 4))
        .unwrap();
    m.core.mem().write_word(FDCAN1 + 0x10, 1 << 4).unwrap();
    // TX buffer 0 at RAM+632: std ID 0x123, DLC=2.
    for (i, b) in [0x00u8, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x02, 0x00]
        .into_iter()
        .chain([0xDE, 0xAD])
        .enumerate()
    {
        m.core.mem().write(RAM + 632 + i as u32, b).unwrap();
    }
    m.core.mem().write_word(FDCAN1 + 0xCC, 1).unwrap();
    step_until(&mut m, FDCAN1 + 0xD4, 1, 1);
    step_until(&mut m, FDCAN1 + 0x90, 0xF, 1);
    // Payload in message RAM at RF0SA+8.
    assert_eq!(m.core.mem().read(RAM + 176 + 8).unwrap(), 0xDE);
    assert_eq!(m.core.mem().read(RAM + 176 + 9).unwrap(), 0xAD);
}

#[test]
fn h743_full_loopback_roundtrip() {
    let mut m = Machine::armv7em(board::stm32h743());
    const FDCAN1: u32 = 0x4000_A000;
    const RAM: u32 = 0x4000_AC00;
    // INIT + CCE + test mode; no filters; TXBC TBSA offset 0x100, queue 2.
    m.core.mem().write_word(FDCAN1 + 0x18, 0x83).unwrap();
    m.core.mem().write_word(FDCAN1 + 0x84, 0x300).unwrap();
    m.core
        .mem()
        .write_word(FDCAN1 + 0xC0, 0x100 | (2 << 24))
        .unwrap();
    m.core.mem().write_word(FDCAN1 + 0xC8, 0).unwrap();
    m.core
        .mem()
        .write_word(FDCAN1 + 0xA0, (4 << 16) | 0x200)
        .unwrap();
    m.core.mem().write_word(FDCAN1 + 0x10, 1 << 4).unwrap();
    // TX element 0 at RAM+0x100: std ID 0x123, DLC=8.
    for (i, b) in [0x00u8, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x08, 0x00]
        .into_iter()
        .chain([0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44])
        .enumerate()
    {
        m.core.mem().write(RAM + 0x100 + i as u32, b).unwrap();
    }
    m.core.mem().write_word(FDCAN1 + 0xD0, 1).unwrap();
    step_until(&mut m, FDCAN1 + 0xD8, 1, 1);
    step_until(&mut m, FDCAN1 + 0xA4, 0x7F, 1);
    // Payload in message RAM at F0SA+8.
    let mut payload = vec![0u8; 8];
    for (i, b) in payload.iter_mut().enumerate() {
        *b = m.core.mem().read(RAM + 0x208 + i as u32).unwrap();
    }
    assert_eq!(payload, &[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]);
}

//! Backtrace ring tests (WP-R2): retired-instruction history for fault triage.
//!
//! All instruction byte sequences are verified against
//! `llvm-mc -triple=thumbv6m-none-eabi`.

use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::flat::FlatMemory;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_8000;

/// Boot `code` at ENTRY, with a reset vector table above it.
fn boot(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::new(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, ENTRY | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    m
}

/// `movs r0, #7; subs r0, r0, #3; muls r0, r0, r0; b .`
/// (bytes: 07 20 | c0 1e | 40 43 | fe e7).
const MOV_SUB_MUL_LOOP: &[u8] = &[0x07, 0x20, 0xc0, 0x1e, 0x40, 0x43, 0xfe, 0xe7];

#[test]
fn retires_land_oldest_first_with_opcodes_and_time() {
    let mut m = boot(MOV_SUB_MUL_LOOP);
    for _ in 0..5 {
        m.step().expect("step retires");
    }
    let bt = m.backtrace(32);
    let pcs: Vec<u32> = bt.iter().map(|e| e.pc).collect();
    assert_eq!(pcs, vec![ENTRY, ENTRY + 2, ENTRY + 4, ENTRY + 6, ENTRY + 6]);
    let h0: Vec<u16> = bt.iter().map(|e| e.h0.unwrap()).collect();
    assert_eq!(h0, vec![0x2007, 0x1ec0, 0x4340, 0xe7fe, 0xe7fe]);
    assert!(bt.windows(2).all(|w| w[0].cyccnt < w[1].cyccnt));
}

#[test]
fn ring_bounds_at_capacity() {
    let mut m = boot(MOV_SUB_MUL_LOOP);
    m.set_backtrace(3);
    for _ in 0..5 {
        m.step().expect("step retires");
    }
    let bt = m.backtrace(32);
    assert_eq!(bt.len(), 3);
    let pcs: Vec<u32> = bt.iter().map(|e| e.pc).collect();
    assert_eq!(pcs, vec![ENTRY + 4, ENTRY + 6, ENTRY + 6]);
}

#[test]
fn disabled_records_nothing() {
    let mut m = boot(MOV_SUB_MUL_LOOP);
    m.set_backtrace(0);
    for _ in 0..5 {
        m.step().expect("step retires");
    }
    assert!(m.backtrace(32).is_empty());
}

#[test]
fn reset_clears_the_ring_but_keeps_capacity() {
    let mut m = boot(MOV_SUB_MUL_LOOP);
    m.step().expect("step retires");
    assert_eq!(m.backtrace(32).len(), 1);
    m.reset().expect("reboot from vector table");
    assert!(m.backtrace(32).is_empty());
    m.step().expect("step retires after reboot");
    assert_eq!(m.backtrace(32).len(), 1);
}

/// `movs r0, #7; subs r0, r0, #3; bkpt #0`: the faulting `bkpt` never
/// retires, so the ring holds the two-instruction prefix while the live pc
/// names the fault site.
#[test]
fn fault_path_keeps_prefix_and_live_pc_names_the_site() {
    let mut m = boot(&[0x07, 0x20, 0xc0, 0x1e, 0x00, 0xbe]);
    assert!(m.run(10).is_err());
    let bt = m.backtrace(32);
    let pcs: Vec<u32> = bt.iter().map(|e| e.pc).collect();
    assert_eq!(pcs, vec![ENTRY, ENTRY + 2]);
    let (regs, _, _) = m.core.snapshot();
    assert_eq!(regs.pc, ENTRY + 4);
}

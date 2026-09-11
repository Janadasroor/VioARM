//! WP-K1: Thumb-2 `.w` modified-immediate data processing on ARMv7-M.
//!
//! `orr.w r3,r3,#1` (`F043 0301`) and `adds.w r3,r3,#1` (`F113 0301`)
//! decode via `decode_dp_modified` and retire to `Advance(pc+4)` on the
//! ARMv7-M core. The CLI previously built every board on the ARMv6-M core,
//! so these faulted even on M3/M4 boards; core selection now follows the
//! board family (`machine_for`), and these tests lock the `armv7m` path
//! with code relocated out of the vector table (vectors live at `0x8/0xC`).
//!
//! The last two tests lock the harness-shape diagnosis for the original
//! probe bins: code placed at `0x8/0xC` faults *into* the vector table it
//! overlaps — an even `HardFault` vector stops with `NonThumbTarget`
//! (0 insn retired), an odd one stacks a frame and the next fetch aborts.

use vioarm_core::cpu::decode32::{decode32, decode32_v7m, DpOp, Inst32};
use vioarm_core::error::HaltCause;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
/// Code base clear of the vector table (`0x0..0x40` on flash-at-0 boards).
const ENTRY: u32 = 0x0000_0100;
/// `orr.w r3,r3,#1` (little-endian `F043 0301`).
const ORR_W: [u8; 4] = [0x43, 0xF0, 0x01, 0x03];
/// `adds.w r3,r3,#1` (little-endian `F113 0301`).
const ADDS_W: [u8; 4] = [0x13, 0xF1, 0x01, 0x03];

fn boot_v7m(code: &[u8]) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(ENTRY, code));
    m.core.regs.pc = ENTRY;
    m.core.xpsr.set(0x0100_0000);
    m
}

fn boot_v6m(code: &[u8]) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]);
    let mut m = Machine::new(mem);
    m.load_image(&load_flat(ENTRY, code));
    m.core.regs.pc = ENTRY;
    m.core.xpsr.set(0x0100_0000);
    m
}

#[test]
fn orr_w_decodes_to_modified_imm_orr_on_v7m_only() {
    match decode32_v7m(0xF043, 0x0301) {
        Ok(Inst32::DpImm {
            op,
            rd,
            rn,
            imm12,
            expand,
            set_flags,
        }) => {
            assert_eq!(op, DpOp::Orr);
            assert_eq!((rd, rn), (3, 3));
            assert_eq!(imm12, 0x001);
            assert!(expand, "modified-immediate form must ThumbExpandImm");
            assert!(!set_flags, "orr.w without .s must not set flags");
        }
        other => panic!("orr.w must decode as DpImm Orr, got {other:?}"),
    }
    assert!(
        decode32(0xF043, 0x0301).is_err(),
        "ARMv6-M must reject Thumb-2 orr.w"
    );
    assert!(
        decode32(0xF113, 0x0301).is_err(),
        "ARMv6-M must reject Thumb-2 adds.w"
    );
}

#[test]
fn orr_w_modified_imm_advances_with_exact_regs() {
    let mut m = boot_v7m(&ORR_W);
    m.core.regs.r[3] = 0x10;
    m.step().expect("orr.w must retire on ARMv7-M");
    assert_eq!(m.core.regs.r[3], 0x11);
    assert_eq!(m.core.regs.pc, ENTRY + 4);
}

#[test]
fn adds_w_modified_imm_advances_with_exact_result() {
    let mut m = boot_v7m(&ADDS_W);
    m.core.regs.r[3] = 0x0F;
    m.step().expect("adds.w must retire on ARMv7-M");
    assert_eq!(m.core.regs.r[3], 0x10);
    assert_eq!(m.core.regs.pc, ENTRY + 4);
}

#[test]
fn v6m_rejects_thumb2_dp() {
    let mut m = boot_v6m(&ORR_W);
    m.core.regs.r[3] = 0x10;
    assert!(
        m.step().is_err(),
        "ARMv6-M must fault Thumb-2 orr.w (no silent mis-execution)"
    );
}

/// The `orrw3.bin` shape: `B@0x8→0xC` over a `NOP`, `ORR.W@0xC` overlapping
/// the `HardFault` vector at `0xC` (whose word `0x0301F043` is odd, so the
/// fault stacks a frame and lands at `0x0301F042`, where the next fetch
/// aborts). Locks the diagnosis byte-for-byte: 2 retired steps, `sp -32`,
/// then `FetchAbort`.
#[test]
fn vector_overlap_with_odd_hardfault_vector_aborts_after_two_steps() {
    let mem = FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]);
    let mut m = Machine::new(mem);
    let img: [u8; 18] = [
        0x00, 0x10, 0x00, 0x20, // 0x00: SP = 0x20001000
        0x09, 0x00, 0x00, 0x00, // 0x04: reset vector -> 0x9 (pc 0x8)
        0x00, 0xE0, // 0x08: B -> 0x0C
        0x00, 0xBF, // 0x0A: NOP (skipped)
        0x43, 0xF0, 0x01, 0x03, // 0x0C: ORR.W (= HardFault vector word, odd)
        0x00, 0xBE, // 0x10: BKPT (never reached)
    ];
    m.load_image(&load_flat(0x0000_0000, &img));
    m.reset().expect("reset vector is odd and executable");
    assert_eq!(m.core.regs.pc, 0x8);
    m.step().expect("branch to 0xC retires");
    assert_eq!(m.core.regs.pc, 0xC);
    // The ORR.W faults as Undefined on M0; the HardFault vector is odd, so
    // entry succeeds (stacking 8 words) and the step retires the entry.
    m.step().expect("HardFault entry retires");
    assert_eq!(m.core.regs.pc, 0x0301_F042);
    assert_eq!(m.core.regs.sp, 0x2000_1000 - 32);
    assert!(matches!(m.step(), Err(HaltCause::FetchAbort)));
}

/// The `orrw.bin` shape: `ORR.W@0x8` with an even `HardFault` vector at
/// `0xC` (`0x0000BE00`). No frame is pushed; the machine stops immediately
/// with `NonThumbTarget` and retires 0 instructions.
#[test]
fn vector_overlap_with_even_hardfault_vector_stops_at_once() {
    let mem = FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]);
    let mut m = Machine::new(mem);
    let img: [u8; 14] = [
        0x00, 0x10, 0x00, 0x20, // 0x00: SP = 0x20001000
        0x09, 0x00, 0x00, 0x00, // 0x04: reset vector -> 0x9 (pc 0x8)
        0x43, 0xF0, 0x01, 0x03, // 0x08: ORR.W (Undefined on M0)
        0x00, 0xBE, // 0x0C: even HardFault vector 0x0000BE00
    ];
    m.load_image(&load_flat(0x0000_0000, &img));
    m.reset().expect("reset vector is odd and executable");
    assert!(matches!(m.step(), Err(HaltCause::NonThumbTarget)));
    assert_eq!(m.core.regs.pc, 0x8);
    assert_eq!(m.core.regs.sp, 0x2000_1000);
}

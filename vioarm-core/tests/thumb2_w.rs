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
use vioarm_core::mem::Bus;

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

/// T3 `B<cond>.W` reads I1/I2 directly (no J inversion — unlike T4).
/// Gas proof: `beq.w .+8` is `F000 8004`, `beq.w back` is `F43F AFFE`;
/// live QEMU execution takes the branch. The old inversion read `F000
/// 8002` as +0xC0004 and jumped to unmapped memory.
#[test]
fn bcc_w_taken_path_executes() {
    // 0x08: movs r0, #0; 0x0A: cmp r0, #0 (Z=1); 0x0C: beq.w taken@0x14;
    // 0x10: movs r1, #0x46 (skipped); 0x12: b done (skipped);
    // 0x14: movs r1, #0x54; 0x16: str r1, [r3] (r3 preset to RAM);
    // 0x18: b . (spin).
    let img: [u8; 26] = [
        0x00, 0x10, 0x00, 0x20, // 0x00: SP
        0x09, 0x00, 0x00, 0x00, // 0x04: reset -> 0x8
        0x00, 0x20, // 0x08: movs r0, #0
        0x00, 0x28, // 0x0A: cmp r0, #0
        0x00, 0xF0, 0x02, 0x80, // 0x0C: beq.w 0x14 (F000 8002)
        0x46, 0x21, // 0x10: movs r1, #0x46 (skipped)
        0x00, 0xE0, // 0x12: b 0x16 (skipped)
        0x54, 0x21, // 0x14: movs r1, #0x54
        0x19, 0x60, // 0x16: str r1, [r3]
        0xFE, 0xE7, // 0x18: b .
    ];
    let mut m = boot_v7m(&[]);
    m.load_image(&load_flat(0x0000_0000, &img));
    m.reset().expect("reset vector is odd and executable");
    m.core.regs.r[3] = RAM;
    let _ = m.run(16);
    assert_eq!(m.core.regs.r[1], 0x54, "taken path stores 0x54, not 0x46");
    let word = m.core.mem().read_word(RAM).expect("RAM readable");
    assert_eq!(word & 0xFF, 0x54, "branch skipped the fallthrough store");
}

/// Far `bl` (+0x1004, bytes `01 F0 02 F8`, gas: `bl 0x1108` from 0x100):
/// the non-uniform `imm10` (= 1) distinguishes the 25-bit layout — the old
/// shift read this encoding as +0x4004 and jumped into unmapped memory
/// (found via a real CoreMark binary whose `bl main` decoded -6638 as
/// -18926). Asserts the landing PC, the link register, and execution at
/// the target.
#[test]
fn bl_far_forward_taken_path_executes() {
    const TARGET: u32 = ENTRY + 4 + 0x1004;
    let mut img = vec![0u8; (TARGET - ENTRY) as usize + 4];
    img[0..4].copy_from_slice(&[0x01, 0xF0, 0x02, 0xF8]); // ENTRY: bl TARGET
    let t = (TARGET - ENTRY) as usize;
    img[t..t + 2].copy_from_slice(&[0x2A, 0x20]); // TARGET: movs r0, #0x2A
    img[t + 2..t + 4].copy_from_slice(&[0xFE, 0xE7]); // b .
    let mut m = boot_v7m(&img);
    m.step().expect("far bl must retire");
    assert_eq!(m.core.regs.pc, TARGET);
    assert_eq!(m.core.regs.lr, ENTRY + 4 + 1);
    m.step().expect("movs at the target retires");
    assert_eq!(m.core.regs.r[0], 0x2A);
}

/// 32-bit shift (register): `Rd = Rn <op> (Rm & 0xFF)` (gas-verified
/// `lsl.w r7, r8, r9` = FA08 F709; live QEMU probe agrees on seats and
/// results). Asymmetric operands prove the seats: `r8 << r9 = 96`, and the
/// `lsls.w` form sets no flags on a clean shift.
#[test]
fn shift_reg_wide_shifts_by_rm_byte() {
    // movs r0, #3 ; movs r1, #5 ; mov r8, r0 ; mov r9, r1 ;
    // lsl.w r7, r8, r9 ; lsls.w r6, r8, r9 ; b .
    // (`movs` cannot target high regs, hence the `mov` bridges.)
    let img: [u8; 26] = [
        0x00, 0x10, 0x00, 0x20, // 0x00: SP
        0x09, 0x00, 0x00, 0x00, // 0x04: reset -> 0x8
        0x03, 0x20, // 0x08: movs r0, #3
        0x05, 0x21, // 0x0A: movs r1, #5
        0x80, 0x46, // 0x0C: mov r8, r0
        0x89, 0x46, // 0x0E: mov r9, r1
        0x08, 0xFA, 0x09, 0xF7, // 0x10: lsl.w r7, r8, r9
        0x18, 0xFA, 0x09, 0xF6, // 0x14: lsls.w r6, r8, r9
        0xFE, 0xE7, // 0x18: b .
    ];
    let mut m = boot_v7m(&[]);
    m.load_image(&load_flat(0x0000_0000, &img));
    m.reset().expect("reset vector is odd and executable");
    let _ = m.run(8);
    // Seats per live QEMU probe: `lsl.w r7, r8, r9` is `r8 << r9`
    // (shifted = first source = hi nibble, amount = second = lo nibble).
    assert_eq!(m.core.regs.r[7], 96, "r8 << r9, not r9 << r8");
    assert_eq!(m.core.regs.r[6], 96);
    assert!(
        !m.core.xpsr.n() && !m.core.xpsr.z(),
        "clean shift sets no N/Z"
    );
}

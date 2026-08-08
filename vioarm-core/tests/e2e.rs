//! End-to-end machine tests: build an image, boot, run, inspect state.
//!
//! All instruction byte sequences are verified against
//! `llvm-mc -triple=thumbv6m-none-eabi`.

use vioarm_core::error::HaltCause;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::flat::FlatMemory;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_8000;

/// Boot `code` at ENTRY, with a reset vector table above it.
fn boot(code: &[u8]) -> Machine {
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

/// `movs r0, #7; subs r0, r0, #3; muls r0, r0, r0; bx lr`
/// (bytes: 07 20 | c0 1e | 40 43 | 70 47).
const MOV_SUB_MUL: &[u8] = &[0x07, 0x20, 0xc0, 0x1e, 0x40, 0x43, 0x70, 0x47];

#[test]
fn mov_sub_mul_arithmetic() {
    let mut m = boot(MOV_SUB_MUL);
    let err = m.run(10);
    // bx went to the unset LR, whose fetch aborts.
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, xpsr, cycles) = m.core.snapshot();
    assert_eq!(cycles, 4);
    // (7 - 3) * (7 - 3) == 16, positive, non-zero.
    assert_eq!(regs.r[0], 16);
    assert!(!xpsr.n());
    assert!(!xpsr.z());
    assert!(!xpsr.c());
    assert!(!xpsr.v());
    assert!(xpsr.is_thumb());
}

/// `adds r0, r1, r2; bx lr` with r1 = 0xFFFF_FFFF, r2 = 1.
const ADD_CARRY: &[u8] = &[0x88, 0x18, 0x70, 0x47];

#[test]
fn add_sets_carry_but_not_overflow() {
    let mut m = boot(ADD_CARRY);
    m.core.regs.r[1] = u32::MAX;
    m.core.regs.r[2] = 1;
    let err = m.run(4);
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, xpsr, _) = m.core.snapshot();
    assert_eq!(regs.r[0], 0);
    assert!(xpsr.c());
    assert!(!xpsr.v());
    assert!(xpsr.z());
}

#[test]
fn add_overflow_from_max_int() {
    let mut m = boot(ADD_CARRY);
    m.core.regs.r[1] = i32::MAX as u32;
    m.core.regs.r[2] = 1;
    m.run(4).unwrap_err();
    let (regs, xpsr, _) = m.core.snapshot();
    assert_eq!(regs.r[0], i32::MIN as u32);
    assert!(!xpsr.c());
    assert!(xpsr.v());
    assert!(xpsr.n());
    assert!(!xpsr.z());
}

/// `sub sp, #8; add sp, #8; bx lr` (82 b0 | 02 b0 | 70 47).
#[test]
fn sp_changes_roundtrip() {
    let mut m = boot(&[0x82, 0xb0, 0x02, 0xb0, 0x70, 0x47]);
    m.run(5).unwrap_err();
    let (regs, _, _) = m.core.snapshot();
    assert_eq!(regs.sp, INIT_SP);
}

/// `ldr r0, [pc, #8]; bx lr` with a literal word at the aligned slot.
#[test]
fn pc_relative_literal_load() {
    let mut m = boot(&[0x02, 0x48, 0x70, 0x47]);
    let lit_addr = ((ENTRY + 4) & !3) + 4 * 2;
    m.core.mem().write_word(lit_addr, 0xabcd_1234).unwrap();
    // ldr (1), bx lr (2), then fetch of the target aborts.
    let err = m.run(4);
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, _, _) = m.core.snapshot();
    assert_eq!(regs.r[0], 0xabcd_1234);
}

/// A `run(budget)` that never halts reports the budget as exhausted and the
/// cycle count reflects exactly `budget` steps.
#[test]
fn cycle_budget_is_reported() {
    // movs r0, #0; zero bytes then decode as endless lsls #0 (a "nop").
    let mut m = boot(&[0x00, 0x20]);
    let err = m.run(100);
    assert_eq!(err, Err(HaltCause::CycleBudgetExhausted));
    let (_, _, cycles) = m.core.snapshot();
    assert_eq!(cycles, 100);
}

/// Real firmware exercising conditional branches, `bl`/`bx lr`, `push`/`pop`
/// and `stm`/`ldm`, assembled with `llvm-mc -triple=thumbv6m-none-eabi`:
///
/// ```text
///     movs r0, #3        ; loop counter
///     movs r1, #0        ; accumulator
/// loop:
///     adds r1, r1, r0    ; r1 = 3 + 2 + 1 = 6
///     subs r0, r0, #1
///     bne loop
///     bl sub             ; lr = 0x1f, calls sub
///     movs r4, #0xaa
///     movs r5, #0xbb
///     push {r4, r5}      ; spill
///     movs r4, #0
///     movs r5, #0
///     pop {r4, r5}       ; restore
///     movs r0, #0x20
///     lsls r0, r0, #24   ; r0 = 0x2000_0000 (RAM base)
///     stm r0!, {r4, r5}  ; store 0xaa, 0xbb; r0 = +8
///     movs r0, #0x20
///     lsls r0, r0, #24
///     ldm r0!, {r6, r7}  ; reload into r6, r7
///     bkpt #0
/// sub:
///     adds r2, r1, #1
///     bx lr
/// ```
#[test]
fn loop_bl_push_pop_ldm_stm() {
    let mut m = boot(&[
        0x03, 0x20, 0x00, 0x21, 0x09, 0x18, 0x40, 0x1e, 0xfc, 0xd1, 0x00, 0xf0, 0x0d, 0xf8, 0xaa,
        0x24, 0xbb, 0x25, 0x30, 0xb4, 0x00, 0x24, 0x00, 0x25, 0x30, 0xbc, 0x20, 0x20, 0x00, 0x06,
        0x30, 0xc0, 0x20, 0x20, 0x00, 0x06, 0xc0, 0xc8, 0x00, 0xbe, 0x4a, 0x1c, 0x70, 0x47,
    ]);
    // `bl` lands at the `bkpt`, so the machine halts on the breakpoint.
    let err = m.run(64);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // 26 instructions retire; the bkpt halts before it is counted.
    assert_eq!(cycles, 26);
    // Loop: 3 + 2 + 1, then sub() returns r2 = 6 + 1.
    assert_eq!(regs.r[1], 6);
    assert_eq!(regs.r[2], 7);
    // push/pop round trip.
    assert_eq!(regs.r[4], 0xaa);
    assert_eq!(regs.r[5], 0xbb);
    // stm/ldm round trip via RAM.
    assert_eq!(regs.r[6], 0xaa);
    assert_eq!(regs.r[7], 0xbb);
    assert_eq!(regs.r[0], RAM + 8);
    assert_eq!(regs.sp, INIT_SP);
    // The two stored words are in memory.
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0xaa);
    assert_eq!(m.core.mem().read_word(RAM + 4).unwrap(), 0xbb);
}

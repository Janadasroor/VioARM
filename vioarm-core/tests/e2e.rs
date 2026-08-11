//! End-to-end machine tests: build an image, boot, run, inspect state.
//!
//! All instruction byte sequences are verified against
//! `llvm-mc -triple=thumbv6m-none-eabi`.

use std::fmt::Write as _;

use vioarm_core::error::HaltCause;
use vioarm_core::loader::{load_flat, load_hex};
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::system::SystemMemory;

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

/// Boot `code` at ENTRY on an ARMv7-M (Cortex-M3/M4) core.
fn boot_v7m(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, ENTRY | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    m
}

/// Boot with a full vector table: SP at 0, reset at +4, NMI at +8, and a
/// `HardFault` vector at +12 pointing at `hardfault`.
fn boot_with_vectors(
    reset: &[u8],
    hardfault: &[u8],
    code_addr: u32,
    hf_addr: u32,
) -> Machine<FlatMemory> {
    let mut m = Machine::new(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(code_addr, reset));
    m.load_image(&load_flat(hf_addr, hardfault));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, code_addr | 1).unwrap();
        mem.write_word(FLASH + 8, 0).unwrap();
        mem.write_word(FLASH + 12, hf_addr | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    m
}

/// `movs r0, #7; subs r0, r0, #3; muls r0, r0, r0; bx lr`
/// (bytes: 07 20 | c0 1e | 40 43 | 70 47).
const MOV_SUB_MUL: &[u8] = &[0x07, 0x20, 0xc0, 0x1e, 0x40, 0x43, 0x70, 0x47];

/// Encode a chunk as an Intel HEX data record with a correct checksum.
#[allow(clippy::cast_possible_truncation)]
fn hex_record(addr: u16, data: &[u8]) -> String {
    let mut line = format!(":{:02X}{:04X}00", data.len(), addr);
    let mut sum = data.len() as u8 + (addr >> 8) as u8 + addr as u8;
    for &b in data {
        let _ = write!(line, "{b:02X}");
        sum = sum.wrapping_add(b);
    }
    let _ = write!(line, "{:02X}", 0u8.wrapping_sub(sum));
    line.push('\n');
    line
}

/// A complete Intel HEX image: vector table + code, plus an EOF record.
#[allow(clippy::cast_possible_truncation)]
fn hex_image(code: &[u8]) -> Vec<u8> {
    let mut vt = Vec::new();
    vt.extend_from_slice(&INIT_SP.to_le_bytes());
    vt.extend_from_slice(&(ENTRY | 1).to_le_bytes());
    vt.extend_from_slice(&[0u8; 8]);
    let mut out = hex_record(0, &vt);
    let mut off = 0;
    while off < code.len() {
        let chunk = &code[off..(off + 16).min(code.len())];
        out.push_str(&hex_record(ENTRY as u16 + off as u16, chunk));
        off += chunk.len();
    }
    out.push_str(&hex_record(0, &[])); // EOF record (type 00, length 0)
    out.into_bytes()
}

/// A firmware image delivered as Intel HEX boots, runs, and computes the same
/// result as the raw-binary path.
#[test]
fn hex_loaded_image_boots_and_runs() {
    let img = load_hex(&hex_image(MOV_SUB_MUL)).unwrap();
    assert_eq!(img.segments.len(), 2, "vector table and code stay separate");
    assert!(img.entry.is_none());

    let mut m = Machine::new(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&img);
    m.reset().expect("reset from vector table");
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, xpsr, cycles) = m.core.snapshot();
    assert_eq!(cycles, 4);
    assert_eq!(regs.r[0], 16);
    assert!(xpsr.is_thumb());
}

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

/// Thread code shared by the exception tests: two setup moves, an undefined
/// `0xBFFF` (reserved HINT), then `movs r2, #0x22; bkpt`.
const EXC_THREAD: &[u8] = &[0x20, 0x00, 0x01, 0x21, 0xff, 0xbf, 0x22, 0x22, 0x00, 0xbe];
/// A `HardFault` handler that sets RAM to `marker` and returns with `bx lr`.
fn marker_handler(marker: u8) -> Vec<u8> {
    vec![
        0x20, 0x20, 0x00, 0x06, // movs r0, #0x20; lsls r0, r0, #24 -> r0 = RAM
        marker, 0x23, 0x03, 0x60, // movs r3, #marker; str r3, [r0]
        0x70, 0x47, // bx lr
    ]
}

/// An undefined instruction raises a `HardFault`: the frame is pushed, the
/// handler runs, and `bx lr` (`EXC_RETURN`) restores the thread which resumes
/// one instruction after the faulting opcode.
#[test]
fn hardfault_handler_runs_and_returns() {
    const CODE: u32 = 0x40;
    const HF: u32 = 0x100;
    let mut m = boot_with_vectors(EXC_THREAD, &marker_handler(0x7a), CODE, HF);
    let err = m.run(16);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // Handler ran, visible through its RAM store...
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0x7a);
    // ...and the thread's registers are restored from the frame, clobbered
    // caller-saved registers (r0, r3) and all.
    assert_eq!(regs.r[0], 0);
    assert_eq!(regs.r[1], 1);
    assert_eq!(regs.r[2], 0x22);
    assert_eq!(regs.r[3], 0);
    assert_eq!(regs.sp, INIT_SP);
    assert_eq!(cycles, 9);
}

/// A misaligned thread SP is 8-byte-aligned on exception entry; the stacked
/// xPSR records the adjustment (ALIGN bit) and the original SP is restored on
/// return.
#[test]
fn hard_fault_aligns_misaligned_stack() {
    const CODE: u32 = 0x40;
    const HF: u32 = 0x100;
    // `sub sp, #4` (26 b1); restore checks the stacked xPSR.
    let thread = &[
        0x81, 0xb0, // sub sp, #4
        0xff, 0xbf, 0x22, 0x22, 0x00, 0xbe,
    ];
    let handler = &[
        0x20, 0x21, // movs r1, #0x20
        0x09, 0x06, // lsls r1, r1, #24 -> r1 = RAM
        0x07, 0x98, // ldr r0, [sp, #28] (stacked xPSR)
        0x08, 0x60, // str r0, [r1]
        0x70, 0x47, // bx lr
    ];
    let mut m = boot_with_vectors(thread, handler, CODE, HF);
    let err = m.run(12);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // The stacked xPSR carries Thumb plus the ALIGN bit.
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), (1 << 24) | (1 << 9));
    // The thread's SP is restored to its pre-excerpt value.
    assert_eq!(regs.sp, INIT_SP - 4);
    assert_eq!(regs.r[2], 0x22);
    assert_eq!(cycles, 8);
}

/// An exception taken from a process-stack thread pushes its frame on the
/// process SP, runs on the main SP, and returns through the PSP `EXC_RETURN`.
#[test]
fn exception_from_process_stack_returns_via_psp() {
    const CODE: u32 = 0x40;
    const HF: u32 = 0x100;
    const PSP: u32 = 0x2000_6000;
    const MSP: u32 = 0x2000_7000;
    let mut m = boot_with_vectors(EXC_THREAD, &marker_handler(0x5a), CODE, HF);
    // Run the thread on the process stack: active SP becomes the PSP, and the
    // main stack is banked for the handler.
    m.core.control = 0x2;
    m.core.regs.sp = PSP;
    m.core.regs.psp = PSP;
    m.core.regs.msp = MSP;
    let err = m.run(16);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0x5a);
    // The handler ran on, and preserved, the banked main SP.
    assert_eq!(regs.msp, MSP);
    assert_eq!(regs.sp, PSP);
    assert_eq!(regs.r[2], 0x22);
    assert_eq!(cycles, 9);
}

/// An undefined instruction with no `HardFault` vector configured halts with a
/// typed cause rather than silently continuing.
#[test]
fn undefined_instruction_without_handler_halts() {
    let mut m = boot(&[0xff, 0xbf]);
    let err = m.run(4);
    assert_eq!(err, Err(HaltCause::NonThumbTarget));
}

/// Exception return through `pop {pc}` as well as `bx lr`.
#[test]
fn handler_returns_via_pop_pc() {
    const CODE: u32 = 0x40;
    const HF: u32 = 0x100;
    let mut handler = marker_handler(0x5a);
    handler.extend_from_slice(&[0x00, 0xb5, 0x00, 0xbd]); // push {lr}; pop {pc}
    let mut m = boot_with_vectors(EXC_THREAD, &handler, CODE, HF);
    let err = m.run(16);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, _) = m.core.snapshot();
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0x5a);
    assert_eq!(regs.r[2], 0x22);
    assert_eq!(regs.sp, INIT_SP);
}

/// A 16-bit `svc` takes the `SVCall` exception (#11, vector at +0x2c): the
/// handler runs from its own vector, and `bx lr` resumes the thread one
/// halfword after the SVC.
#[test]
fn svc_takes_svcall_and_returns() {
    const CODE: u32 = 0x40;
    const SC: u32 = 0x100;
    // movs r0, #0x55 ; svc #3 ; movs r3, #0x22 ; bkpt
    let thread = &[
        0x55, 0x20, // movs r0, #0x55
        0x03, 0xdf, // svc #3
        0x22, 0x23, // movs r3, #0x22
        0x00, 0xbe, // bkpt
    ];
    let mut m = Machine::new(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(CODE, thread));
    m.load_image(&load_flat(SC, &marker_handler(0x77)));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, CODE | 1).unwrap();
        mem.write_word(FLASH + 8, 0).unwrap();
        mem.write_word(FLASH + 0x2c, SC | 1).unwrap(); // SVCall vector (#11)
    }
    m.reset().expect("reset from vector table");
    let err = m.run(16);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, _) = m.core.snapshot();
    // The handler ran and stored its marker; the exception return undid its
    // register clobbers, and the thread resumed after the SVC.
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0x77);
    assert_eq!(regs.r[0], 0x55);
    assert_eq!(regs.r[3], 0x22);
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: `movw`/`movt` build a 32-bit constant, `movs rn, #ThumbExpandImm`
/// materializes a rotated immediate, and the register `orrs`/`eors` fold it in
/// (`orrs r1, r1, r0; eors r2, r2, r0`).
#[test]
fn movt_and_dp_immediate_on_armv7m() {
    // movw r0, #0xABCD ; movt r0, #0x1234 ; movs r1, #0x0F000000
    // orrs r1, r1, r0  ; movs r2, #0x0F000000 ; eors r2, r2, r0 ; bx lr
    let mut m = boot_v7m(&[
        0x4a, 0xf6, 0xcd, 0x30, 0xc1, 0xf2, 0x34, 0x20, 0x5f, 0xf0, 0x70, 0x61, 0x51, 0xea, 0x00,
        0x01, 0x5f, 0xf0, 0x70, 0x62, 0x92, 0xea, 0x00, 0x02, 0x70, 0x47,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, xpsr, cycles) = m.core.snapshot();
    assert_eq!(cycles, 7);
    assert_eq!(regs.r[0], 0x1234_abcd);
    // 0x0F00_0000 OR 0x1234_ABCD, then XOR the same pair.
    assert_eq!(regs.r[1], 0x1f34_abcd);
    assert_eq!(regs.r[2], 0x1d34_abcd);
    assert!(!xpsr.n() && !xpsr.z() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: a shifted-register data-processing folds the shift carry into
/// `C`, and the plain `addw`/`subw` immediates work on `sp` without touching
/// the flags.
#[test]
fn shift_carry_and_addw_subw_sp_on_armv7m() {
    // movw r0, #0xFFE0 ; movt r0, #0xF000 ; lsls r1, r0, #4
    // subw sp, sp, #0x20 ; addw sp, sp, #0x20 ; bx lr
    let mut m = boot_v7m(&[
        0x4f, 0xf6, 0xe0, 0x70, 0xcf, 0xf2, 0x00, 0x00, 0x5f, 0xea, 0x00, 0x11, 0xad, 0xf2, 0x20,
        0x0d, 0x0d, 0xf2, 0x20, 0x0d, 0x70, 0x47,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::FetchAbort));
    let (regs, xpsr, cycles) = m.core.snapshot();
    assert_eq!(cycles, 6);
    assert_eq!(regs.r[0], 0xf000_ffe0);
    // LSL #4 drops the four set high nibbles; the top shifted-out bit is 1.
    assert_eq!(regs.r[1], 0x000f_fe00);
    assert!(xpsr.c());
    assert!(!xpsr.n() && !xpsr.z() && !xpsr.v());
    // The raw-immediate subw/addw cancel and leave sp and flags untouched.
    assert_eq!(regs.sp, INIT_SP);
    assert!(!xpsr.n() && !xpsr.z());
}

/// ARMv7-M: writing `ICSR.PENDSVSET` latches `PendSV` in the `SCB`; the core
/// stacks an 8-word frame, runs the `PendSV` handler, and `bx lr` restores the
/// thread, which resumes at its entry point.
#[test]
fn pendsv_handler_runs_and_returns() {
    const CODE: u32 = 0x40;
    const PS: u32 = 0x2000_0100;
    let mut m = Machine::armv7m(SystemMemory::new(FlatMemory::with_layout(
        &[FLASH, RAM],
        &[0x10000, 0x10000],
    )));
    m.load_image(&load_flat(CODE, &[0x22, 0x23, 0x00, 0xbe])); // movs r3,#0x22; bkpt
    m.load_image(&load_flat(PS, &marker_handler(0x66)));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, CODE | 1).unwrap();
        mem.write_word(FLASH + 8, 0).unwrap();
        mem.write_word(FLASH + 0x38, PS | 1).unwrap(); // PendSV vector (#14)
    }
    m.reset().expect("reset from vector table");
    // Pend PendSV after reset, since reset clears the SCB's latches.
    m.core.mem().write_word(0xE000_ED04, 0x1000_0000).unwrap();
    let err = m.run(16);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, _) = m.core.snapshot();
    // The handler ran and stored its marker; the exception return undid its
    // register clobbers, and the thread resumed and advanced r3.
    assert_eq!(m.core.mem().read_word(RAM).unwrap(), 0x66);
    assert_eq!(regs.r[3], 0x22);
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: `udiv` returns the unsigned quotient, and a zero divisor yields
/// zero (the reset `DIV_0_TRP` state raises no fault). No flags change.
#[test]
fn divide_on_armv7m() {
    // movs r0, #100 ; movs r2, #4 ; udiv r1, r0, r2
    // movs r3, #0 ; udiv r4, r0, r3 ; bkpt
    let mut m = boot_v7m(&[
        0x64, 0x20, 0x04, 0x22, 0x90, 0xfb, 0xf2, 0xf1, 0x00, 0x23, 0x90, 0xfb, 0xf3, 0xf4, 0x00,
        0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // Two movs, two udiv, then the stopping bkpt (which costs no cycle).
    assert_eq!(cycles, 5);
    // 100 / 4 = 25; the divide-by-zero `udiv` wrote zero and left r0 intact.
    assert_eq!(regs.r[1], 25);
    assert_eq!(regs.r[4], 0);
    assert_eq!(regs.r[0], 100);
}

/// ARMv7-M: the multiply family. `mul`/`mla`/`mls` write the low 32 bits,
/// and the long forms split the 64-bit product across `RdHi:RdLo`; at these
/// magnitudes the high words stay zero and no flags move.
#[test]
fn multiply_on_armv7m() {
    // movs r0, #7 ; movs r1, #6 ; movs r2, #0
    // mul r3, r0, r1 ; mls r4, r0, r1, r3 ; mla r5, r0, r1, r2
    // smull r6, r7, r0, r1 ; umull r8, r9, r0, r1 ; bkpt
    let mut m = boot_v7m(&[
        0x07, 0x20, 0x06, 0x21, 0x00, 0x22, 0x00, 0xfb, 0x01, 0xf3, 0x00, 0xfb, 0x11, 0x34, 0x00,
        0xfb, 0x01, 0x25, 0x80, 0xfb, 0x01, 0x76, 0xa0, 0xfb, 0x01, 0x98, 0x00, 0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, xpsr, cycles) = m.core.snapshot();
    // Eight instructions retire before the stopping bkpt.
    assert_eq!(cycles, 8);
    // mul/mla/mls agree on 7 * 6 = 42.
    assert_eq!(regs.r[3], 42);
    assert_eq!(regs.r[5], 42);
    assert_eq!(regs.r[4], 0); // 42 - 42
                              // The long products split into all-zero high words at this magnitude.
    assert_eq!(regs.r[6], 42);
    assert_eq!(regs.r[7], 0);
    assert_eq!(regs.r[8], 42);
    assert_eq!(regs.r[9], 0);
    // The Z flag remains set from `movs r2, #0`: no multiply touched the
    // flags, not even the 42 results below.
    assert!(xpsr.z() && !xpsr.n() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: the unary byte/bits data-processing. `rev`/`rev16`/`revsh`/`rbit`
/// reorder, `clz` counts, and none of them touch the flags left by `movs`.
#[test]
fn unary_data_processing_on_armv7m() {
    // movs r0, #4 ; lsls r0, r0, #24  -> 0x0400_0000
    // rev r1, r0 ; rbit r2, r0 ; clz r3, r0 ; bkpt
    let mut m = boot_v7m(&[
        0x04, 0x20, 0x00, 0x06, 0x80, 0xfa, 0x80, 0xf1, 0x90, 0xfa, 0xa0, 0xf2, 0xb0, 0xfa, 0x80,
        0xf3, 0x00, 0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, xpsr, cycles) = m.core.snapshot();
    // Five instructions retire before the stopping bkpt.
    assert_eq!(cycles, 5);
    assert_eq!(regs.r[0], 0x0400_0000);
    // rev: byte order flipped.
    assert_eq!(regs.r[1], 0x0000_0004);
    // rbit: the one set bit moves to the mirrored position (bit 26 -> 5).
    assert_eq!(regs.r[2], 0x0000_0020);
    // clz: five leading zeros above bit 26.
    assert_eq!(regs.r[3], 5);
    // No unary instruction touched the flags from movs/lsls.
    assert!(!xpsr.n() && !xpsr.z() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: the bitfield family. `bfi` deposits a field, `ubfx` extracts it
/// back out, and `bfc` clears a window in place; none of them move the flags
/// left by the `movs` setup.
#[test]
fn bitfield_on_armv7m() {
    // movs r0, #0xab ; movs r2, #0
    // bfi r2, r0, #8, #8 ; ubfx r3, r2, #12, #4 ; bfc r0, #4, #4 ; bkpt
    let mut m = boot_v7m(&[
        0xab, 0x20, 0x00, 0x22, 0x60, 0xf3, 0x0f, 0x22, 0xc2, 0xf3, 0x03, 0x33, 0x6f, 0xf3, 0x07,
        0x10, 0x00, 0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, xpsr, cycles) = m.core.snapshot();
    // Five instructions retire before the stopping bkpt.
    assert_eq!(cycles, 5);
    // bfi parked r0's low byte in r2's [15:8]; ubfx then read its top nibble.
    assert_eq!(regs.r[2], 0xAB00);
    assert_eq!(regs.r[3], 0xA);
    // bfc cleared r0's [7:4], turning 0xab into 0x0b.
    assert_eq!(regs.r[0], 0x0B);
    // Z remains set from `movs r2, #0`: no bitfield op touched the flags.
    assert!(xpsr.z() && !xpsr.n() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: the dual word `ldrd`/`strd`. `strd` sinks a register pair into a
/// pair of words, `ldrd` brings them back, and the `!`/post-indexed variants
/// step the SP base down and back up so the pair of words written at
/// `SP - 8` round-trips intact.
#[test]
fn ldrd_strd_on_armv7m() {
    // movs r0, #0xab ; movs r1, #0xcd
    // strd r0, r1, [sp, #-8] ; ldrd r2, r3, [sp, #-8]
    // strd r0, r1, [sp, #-8]! ; ldrd r4, r5, [sp], #8 ; bkpt
    let mut m = boot_v7m(&[
        0xab, 0x20, 0xcd, 0x21, 0x4d, 0xe9, 0x02, 0x01, 0x5d, 0xe9, 0x02, 0x23, 0x6d, 0xe9, 0x02,
        0x01, 0xfd, 0xe8, 0x02, 0x45, 0x00, 0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // Six instructions retire before the stopping bkpt.
    assert_eq!(cycles, 6);
    // The offset `strd` parked the pair at SP - 8 and the offset `ldrd`
    // fetched it back; the `!` and post-indexed forms then stepped SP down
    // and up by 8, reading the same two words on the way back.
    assert_eq!(regs.r[2], 0xAB);
    assert_eq!(regs.r[3], 0xCD);
    assert_eq!(regs.r[4], 0xAB);
    assert_eq!(regs.r[5], 0xCD);
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv6-M: the 16-bit extend (`sxth`/`sxtb`/`uxth`/`uxtb`) and reverse
/// (`rev`/`rev16`/`revsh`) family. Given `r0 = 0x80`, the signed forms
/// sign-extend the field, `uxtb` keeps the byte, and the reverses flip byte
/// order per their operand width; none of them touch the flags from `movs`.
#[test]
fn extend_and_reverse_on_v6m() {
    // movs r0, #0x80
    // sxtb r1, r0 ; sxth r2, r0 ; uxtb r6, r0
    // rev r3, r0 ; rev16 r4, r0 ; revsh r5, r0 ; bkpt
    let mut m = boot(&[
        0x80, 0x20, 0x41, 0xb2, 0x02, 0xb2, 0xc6, 0xb2, 0x03, 0xba, 0x44, 0xba, 0xc5, 0xba, 0x00,
        0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, xpsr, cycles) = m.core.snapshot();
    // Seven instructions retire before the stopping bkpt.
    assert_eq!(cycles, 7);
    // 0x80 as a byte and halfword sign-extends identically...
    assert_eq!(regs.r[1], 0xFFFF_FF80);
    assert_eq!(regs.r[2], 0x0000_0080);
    assert_eq!(regs.r[6], 0x0000_0080);
    // ...and byte-reverses to 0x8000_0000, 0x0000_8000, and 0xFFFF_8000.
    assert_eq!(regs.r[3], 0x8000_0000);
    assert_eq!(regs.r[4], 0x0000_8000);
    assert_eq!(regs.r[5], 0xFFFF_8000);
    // `movs r0, #0x80` zero-extends the byte, so the flags are all clear
    // (N=0, Z=0) and no extend/reverse touched them.
    assert!(!xpsr.n() && !xpsr.z() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: the `IT` (If-Then) block. `movs r0, #5` leaves Z clear, so the
/// `eq` arm of the `ite eq` pair is suppressed while its `ne` arm runs; a
/// following three-instruction `ittt ne` runs all three arms. The 16-bit ADD
/// forms inside the blocks must not clobber the flags from `movs`.
#[test]
fn it_blocks_on_armv7m() {
    // movs r0, #5 ; ite eq ; adds r1, r1, #1 (skipped) ; adds r1, r1, #7
    // ittt ne ; adds r2, r2, #1 ; adds r3, r3, #2 ; adds r4, r4, #3 ; bkpt
    let mut m = boot_v7m(&[
        0x05, 0x20, 0x0c, 0xbf, 0x01, 0x31, 0x07, 0x31, 0x1e, 0xbf, 0x01, 0x32, 0x02, 0x33, 0x03,
        0x34, 0x00, 0xbe,
    ]);
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, xpsr, cycles) = m.core.snapshot();
    // The stopping bkpt does not count toward cycles.
    assert_eq!(cycles, 8);
    assert_eq!(regs.r[0], 5);
    assert_eq!(regs.r[1], 7);
    assert_eq!(regs.r[2], 1);
    assert_eq!(regs.r[3], 2);
    assert_eq!(regs.r[4], 3);
    // The in-block 16-bit ADDS forms left the flags from `movs` untouched.
    assert!(!xpsr.n() && !xpsr.z() && !xpsr.c() && !xpsr.v());
    assert_eq!(regs.sp, INIT_SP);
}

/// ARMv7-M: ITSTATE is part of xPSR, so an exception taken inside an IT block
/// saves the block in the stacked frame and returns into it. The `svc` is the
/// second (last) instruction of an `itt ne` block; the handler's `bx lr`
/// restores ITSTATE and the instruction after the `svc` still runs under the
/// block's condition.
#[test]
fn itstate_round_trips_exception_on_armv7m() {
    // movs r0, #5 ; itt ne ; adds r1, r1, #1 ; svc 0
    // adds r2, r2, #2 ; bkpt
    let mut m = boot_v7m(&[
        0x05, 0x20, 0x1c, 0xbf, 0x01, 0x31, 0x00, 0xdf, 0x02, 0x32, 0x00, 0xbe,
    ]);
    {
        let mem = m.core.mem();
        // SVC vector (exception 11) -> handler at 0x50: `bx lr`.
        mem.write_word(11 * 4, 0x51).unwrap();
        mem.write(0x50, 0x70).unwrap();
        mem.write(0x51, 0x47).unwrap();
    }
    let err = m.run(10);
    assert_eq!(err, Err(HaltCause::Breakpoint));
    let (regs, _, cycles) = m.core.snapshot();
    // The stopping bkpt does not count toward cycles.
    assert_eq!(cycles, 6);
    assert_eq!(regs.r[1], 1);
    assert_eq!(regs.r[2], 2);
    assert_eq!(regs.sp, INIT_SP);
}

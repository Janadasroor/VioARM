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

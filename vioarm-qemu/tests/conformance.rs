//! Differential conformance tests: identical Thumb binaries run in `vioarm`
//! and in `qemu-system-arm` (Cortex-M3), and the per-instruction register
//! traces are diffed. Each test builds its program with the crate's tiny
//! assembler, then the harness stops both sides at the program's landing loop
//! and requires byte-for-byte agreement on real registers and the modeled xPSR
//! bits for every single executed instruction.
//!
//! The tests are skipped (with a note) when no `qemu-system-arm` is found.

use std::time::Duration;
use vioarm_qemu::asm::{Asm, Image, SP};
use vioarm_qemu::qemu::{run_until_pc as qemu_run, QemuMachine};
use vioarm_qemu::vioarm::run_until_pc as vio_run;
use vioarm_qemu::{compare_traces, CpuState};

/// A test harness that assembles `a`, picks the halt address, runs both
/// simulators, and diffs.
fn conformance(name: &str, a: &mut Asm, halt: &str, mt: &QemuMachine) {
    let img: Image = a.clone().finish(SP).expect("assemble");
    let stop = img
        .addr(halt)
        .unwrap_or_else(|| panic!("{name}: no label {halt:?}"));
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "{name}: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run(mt, &img.bytes, stop, Duration::from_secs(15));
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "{name}: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces(name, &vio, &qemu).expect("traces agree");
}

fn snapshot_like(states: &[CpuState]) -> String {
    states.last().map_or_else(|| "no states".to_string(), std::string::ToString::to_string)
}

/// A flag-sync prologue: `ADDS r0, #0` forces `N=0 C=0 V=0 Z=1` identically in
/// both simulators. The Cortex-M3 resets with `Z` set but VioARM resets with
/// all flags clear, so without this the difference would leak through every
/// flag-transparent instruction in the prologue of the program.
fn sync_flags(a: &mut Asm) {
    a.adds_imm8(0, 0);
}

fn qemu() -> Option<QemuMachine> {
    let mt = QemuMachine::detect();
    if mt.is_none() {
        eprintln!("skipping conformance tests: `qemu-system-arm` not found on PATH");
    }
    mt
}

/// An ARMv7E-M (Cortex-M4) QEMU reference, for the DSP zone that faults on the
/// M3 board. Returns `None` when `qemu-system-arm` is missing.
fn qemu_m4() -> Option<QemuMachine> {
    let mt = QemuMachine::detect_m4();
    if mt.is_none() {
        eprintln!("skipping ARMv7E-M conformance tests: `qemu-system-arm` not found on PATH");
    }
    mt
}

/// Re-encode a `MOVW Rd, #imm16` into the 4 bytes at `addr` of `bytes`. Used
/// to patch a register form's base address into a place-holder `movw` after
/// assembly, when the address is only known once labels are resolved.
fn patch_movw(bytes: &mut [u8], addr: usize, rd: u8, imm: u16) {
    let base: u16 = 0xF240;
    let hi = base | (u16::from((imm >> 11) & 1)) << 10 | (imm >> 12);
    let lo = (u16::from((imm >> 8) & 7)) << 12 | (u16::from(rd) << 8) | (imm & 0xFF);
    let raw = hi as u32 | (lo as u32) << 16;
    bytes[addr..addr + 4].copy_from_slice(&raw.to_le_bytes());
}

// ---------------------------------------------------------------------------
// ARMv6-M / Thumb-1 data processing: moves, adds/subs/cmp, the whole 0x4000
// ALU family, shifts (register and immediate), and the high-register specials.
// ---------------------------------------------------------------------------

#[test]
fn alu_flags() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.movw(0, 0x4567);
    a.movt(0, 0x8123);
    a.movw(1, 0x00FF);
    a.movw(2, 0x8000);
    a.mov_imm32(3, 0x1020_3040);
    a.movs_imm8(4, 0);
    a.adds_imm8(4, 0x40);
    a.adds_imm8(4, 0x40);
    a.adds_reg(4, 4, 4);
    a.adcs(4, 3);
    a.cmp_imm8(0, 0xFF);
    a.subs_imm8(2, 1);
    a.subs_imm3(2, 2, 1);
    a.sbcs(2, 3);
    a.adds_imm3(5, 2, 3);
    a.lsls_imm(5, 5, 7);
    a.lsrs_imm(5, 5, 20);
    a.asrs_imm(5, 5, 2);
    a.rors_reg(5, 5);
    a.ands(0, 1);
    a.eors(0, 1);
    a.orrs(1, 2);
    a.bics(3, 3);
    a.mvns(0, 2);
    a.tst(0, 1);
    a.cmn(0, 1);
    a.cmp_reg(0, 1);
    a.negs(0, 3);
    a.muls(0, 3);
    a.lsls_reg(2, 1);
    a.lsrs_reg(2, 1);
    a.asrs_reg(2, 4);
    a.mov_reg(8, 0);
    a.add_high(8, 0);
    a.add_high(8, 8);
    a.cmp_high(8, 0);
    a.mov_reg(10, 0);
    a.mov_reg(11, 1);
    a.add_high(10, 11);
    a.nop();
    a.bc(0, "halt_eq"); // EQ should be false here (flags unknown) -> not taken
    a.movs_imm8(5, 0xEE); // executed when the beq falls through
    a.label("halt_eq");
    a.movs_imm8(6, 0xFF); // executed only on the not-taken path
    a.b("halt");
    a.nop();
    a.label("halt");
    a.b("halt");
    conformance("alu_flags", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Memory: word/byte/halfword loads and stores (immediate and register
// offsets), SP-relative, PC-literal, address computation, push/pop, and the
// 16-bit LDM/STM. Touches only SRAM.
// ---------------------------------------------------------------------------

#[test]
fn loadstore_index() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Base plus distinct bytes at each offset so every load is observable.
    a.mov_imm32(4, 0x2000_0000); // r4 = SRAM base
    a.mov_imm32(0, 0xAA01_2345);
    a.mov_imm32(1, 0xBB11_3456);
    a.mov_imm32(2, 0xCC22_4567);
    a.mov_imm32(3, 0xDD33_5678);
    a.str_wide(0, 4, 0);
    a.str_wide(1, 4, 4);
    a.str_wide(2, 4, 8);
    a.str_wide(3, 4, 12);

    // Pre-indexed forms (P=1 U=1 W=1): load/store then write back the address.
    a.ldr_pre(5, 4, 4); // r5 = [r4+4]; r4 = r4+4
    a.str_pre(0, 4, 8); // [r4+8] = r0; r4 = r4+8
    a.ldr_pre(6, 4, 4); // r6 = [r4+4]; r4 = r4+4
    a.str_pre(1, 4, 0); // [r4] = r1; r4 = r4

    // Post-indexed forms (P=0 U=1 W=1): load/store then add the offset.
    a.ldr_post(7, 4, 4); // r7 = [r4]; r4 = r4+4
    a.str_post(2, 4, 8); // [r4] = r2; r4 = r4+8
    a.ldr_post(8, 4, 4); // r8 = [r4]; r4 = r4+4
    a.str_post(3, 4, 0); // [r4] = r3; r4 = r4

    // Unprivileged forms (P=1 U=1 W=0): no write back.
    a.ldrt(9, 4, 4); // r9 = [r4+4]
    a.strt(3, 4, 8); // [r4+8] = r3
    a.ldrt(0, 4, 8); // r0 = [r4+8]

    // Wide register-offset form: LDR.W Rt, [Rn, Rm, LSL #shift].
    a.mov_imm32(1, 2); // r1 = 2 (index)
    a.ldr_reg_wide(2, 4, 1, 2); // r2 = [r4 + 2<<2] = [r4+8]
    a.ldr_reg_wide(3, 4, 1, 1); // r3 = [r4 + 2<<1] = [r4+4]
    a.mov_imm32(4, 0x2000_0000); // back to base
    a.ldr_wide(1, 4, 4); // pre-indexed str_pre target
    a.ldr_wide(2, 4, 8); // strt target
    a.ldr_wide(3, 4, 12); // str_post target
    a.ldr_wide(0, 4, 8);

    a.label("halt");
    a.b("halt");
    conformance("loadstore_index", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------

#[test]
fn memory() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // base + values
    a.mov_imm32(4, 0x2000_0000); // r4 = SRAM base
    a.mov_imm32(0, 0x3344_1122);
    a.mov_imm32(1, 0xCDAB_EEFF);
    a.movw(2, 0x55AA);
    a.movw(3, 0x00FF);
    // stores
    a.str_imm(0, 4, 0);
    a.strb_imm(1, 4, 4);
    a.strh_imm(0, 4, 6);
    a.str_wide(1, 4, 12);
    a.ls_reg(0, 0, 4, 3); // str r0, [r4, r3]
    // loads
    a.ldr_imm(5, 4, 0); // word
    a.ldrb_imm(6, 4, 4); // byte
    a.ldrh_imm(7, 4, 6); // halfword
    a.ls_reg(4, 6, 4, 3); // ldrsb r6, [r4, r3] -> signed byte from +0xFF/3
    a.ls_reg(7, 7, 4, 3); // ldrsh r7, [r4, r3]
    a.ldr_wide(8, 4, 12);
    a.ldrb_wide(8, 4, 4);
    a.ldrh_wide(9, 4, 6);
    a.ldrsb_wide(9, 4, 4);
    a.ldrsh_wide(9, 4, 6);
    // PC-relative literal (pool must lie after the load: offsets are unsigned)
    a.ldr_lit(0, "pool");
    a.b("past_pool");
    a.align4();
    a.label("pool");
    a.dc_u32(0xDEAD_BEEF);
    a.label("past_pool");
    // SP-relative
    a.sub_sp(16);
    a.mov_reg(6, 13);
    a.add_high(7, 13);
    a.str_sp(0, 8);
    a.ldr_sp(6, 8);
    a.add_sp(16);
    // push/pop + LDM/STM (registers only in SRAM)
    a.push(0b0000_0001, false); // {r0}
    a.push(0b0000_0010 | 0b1000_0000, true); // {r1, r7, lr}
    a.pop(0b0000_0010 | 0b0100_0000, false); // {r1, r6}
    a.mov_imm32(5, 0x2000_0020);
    a.stmia(5, 0b0000_0011); // stmia r5!, {r0, r1}
    a.ldmia(5, 0b0000_0011); // ldmia r5!, {r0, r1}
    a.stmia(5, 0b0001_0001); // stmia r5!, {r0, r4}
    a.ldmia(5, 0b0001_0001);
    a.label("halt");
    a.b("halt");
    conformance("memory", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Exclusive access: LDREX/STREX word, byte and halfword forms, success and
// (cross-address) failure paths of the local exclusive monitor. Every STREX
// is preceded by a fresh LDREX so the post-failure monitor state, which
// implementations may model differently, never feeds a later STREX.
// ---------------------------------------------------------------------------

#[test]
fn exclusive() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(4, 0x2000_1000); // r4 = exclusive region
    a.mov_imm32(0, 0x1122_3344);
    a.mov_imm32(1, 0xAABB_CCDD);
    a.mov_imm32(2, 0x5566_7788);
    a.str_imm(0, 4, 0); // seed [r4] = 0x11223344

    // word: success at [r4]
    a.ldrex(3, 4, 0);
    a.strex(5, 1, 4, 0); // r5 = 0, [r4] = 0xAABBCCDD
    a.ldr_imm(3, 4, 0); // verify

    // word: STREX to a *different* address fails and does not store
    a.ldrex(3, 4, 0);
    a.strex(6, 2, 4, 8); // r6 = 1
    a.ldr_imm(7, 4, 0); // [r4] still 0xAABBCCDD

    // word: success at [r4+8] after re-tagging
    a.ldrex(3, 4, 8);
    a.strex(7, 2, 4, 8); // r7 = 0, [r4+8] = 0x55667788

    // byte forms at [r4+12]
    a.addw(8, 4, 12);
    a.ldrexb(3, 8);
    a.strexb(5, 1, 8); // success r5 = 0
    a.ldrb_wide(3, 8, 0); // r3 = 0xDD

    // halfword forms at [r4+16]
    a.addw(9, 4, 16);
    a.ldrexh(3, 9);
    a.strexh(5, 1, 9); // success r5 = 0
    a.ldrh_wide(3, 9, 0); // r3 = 0xCCDD

    a.label("halt");
    a.b("halt");
    conformance("exclusive", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// System registers: PRIMASK / BASEPRI / FAULTMASK / CONTROL round-trips and
// an MRS MSP read. APSR reads are excluded: QEMU's `-d cpu` xPSR dump and
// VioARM's modeled xPSR already cover the flags, and an MRS APSR would fold
// those same bits into a general register.
// ---------------------------------------------------------------------------

#[test]
fn sysreg() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // MSP/PSP bank probe (privileged)
    a.mov_imm32(5, 0x4000_0000);
    a.msr(0x08, 5); // MSR MSP, r5
    a.mrs(2, 0x08); // r2 = MSP
    a.mrs(6, 0x09); // r6 = PSP
    a.mov_imm32(5, 0x5000_0000);
    a.msr(0x09, 5); // MSR PSP, r5
    a.mrs(7, 0x09); // r7 = PSP
    a.mrs(3, 0x08); // r3 = MSP

    // PRIMASK 0 -> 0, then 1 -> 1
    a.movs_imm8(0, 0);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);
    a.movs_imm8(0, 1);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);
    // BASEPRI 0x80
    a.movs_imm8(0, 0x80);
    a.msr(0x11, 0);
    a.mrs(1, 0x11);
    // FAULTMASK 1 then 0
    a.movs_imm8(0, 1);
    a.msr(0x13, 0);
    a.mrs(1, 0x13);
    a.movs_imm8(0, 0);
    a.msr(0x13, 0);
    a.mrs(1, 0x13);
    // CONTROL: nPRIV then back to privileged
    a.movs_imm8(0, 1);
    a.msr(0x14, 0);
    a.mrs(1, 0x14);
    a.movs_imm8(0, 0);
    a.msr(0x14, 0);
    a.mrs(1, 0x14);
    // MSP read (Thread mode, main stack)
    a.mrs(2, 0x08);

    a.label("halt");
    a.b("halt");
    conformance("sysreg", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Exceptions: an SVC with a real handler in the vector table. Entry pushes
// the eight-word frame, IPSR becomes 11, and `bx lr` (EXC_RETURN THREAD_MSP)
// pops it and resumes after the SVC with the interrupted context intact.
// ---------------------------------------------------------------------------

#[test]
fn exceptions() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.vec(11, "svc_handler"); // SVC exception 11
    a.movs_imm8(0, 1);
    a.svc(0);
    a.movs_imm8(1, 0x22); // after first return
    a.movs_imm8(0, 2);
    a.svc(0);
    a.movs_imm8(2, 0x33); // after second return
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(9, 0x99); // marker
                          // Expose the stacked frame in the trace: r6 = frame base (the handler's
                          // SP), r7 = the stacked PC. For an `SVC` the interrupt pushes the address
                          // of the *following* instruction as a plain address (no Thumb bit; the
                          // T bit lives in the stacked xPSR) -- the same convention as fault frames.
    a.mov_reg(6, 13);
    a.ldr_wide(7, 6, 24); // r7 = stacked PC (instruction after the SVC)
    a.bx(14);
    a.label("halt");
    a.b("halt");
    conformance("exceptions", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Faults: an undefined instruction (0xDE00, permanently-undefined) escalates
// through UsageFault to HardFault (vector 3). The handler reads the eight-word
// frame (stacked PC / LR / r0 / xPSR), rewrites the stacked PC to skip the bad
// opcode, and returns via `bx lr`. The stacked PC must be the address of the
// *faulting* instruction, not the following one.
// ---------------------------------------------------------------------------

#[test]
fn faults() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.vec(3, "hf"); // HardFault vector
                    // Non-trivial register values so the stacked frame is distinguishable.
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.mov_imm32(2, 0x5555_6666);
    a.mov_imm32(3, 0x7777_8888);
    a.mov_imm32(12, 0x9999_AAAA);
    a.raw(0xDE00); // UDEF -> HardFault
    a.label("after_udef");
    a.movs_imm8(0, 0xAA); // reached only once the handler skips the bad opcode
    a.b("halt");

    a.label("hf");
    a.mov_reg(6, 13); // r6 = frame base (SP in the handler)
    a.ldr_wide(7, 6, 24); // r7 = stacked PC (faulting UDEF address | 1)
    a.ldr_wide(8, 6, 0); // r8 = stacked r0
    a.ldr_wide(9, 6, 20); // r9 = stacked LR
    a.ldr_wide(5, 6, 28); // r5 = stacked xPSR
    a.adds_imm8(7, 2); // resume address = faulting PC + 2 (skip UDEF)
    a.str_imm(7, 6, 24); // patch the stacked PC
    a.bx(14); // exception return to the patched PC

    a.label("halt");
    a.b("halt");
    conformance("faults", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Control flow: conditional branches, CBZ/CBNZ, IT blocks, BL+BX LR,
// POP{PC}, and a wide B.W over a large gap.
// ---------------------------------------------------------------------------

#[test]
fn control_flow() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.movs_imm8(0, 1);
    a.cmp_imm8(0, 1);
    a.bc(0, "taken_eq"); // beq taken
    a.movs_imm8(1, 0xEE); // skipped
    a.label("taken_eq");
    a.movs_imm8(2, 0x11); // executed
    a.bc(1, "taken_ne"); // bne (Z=0) not taken
    a.movs_imm8(3, 0x22); // executed
    a.label("taken_ne");
// CBZ/CBNZ (targets must stay within ±126 bytes; the minimum reachable
    // offset is +2 past `pc+4`, so the skipped instruction alone is not enough)
    a.movs_imm8(2, 0); // r2 = 0
    a.cbz(2, "cbz_local"); // taken
    a.movs_imm8(4, 0xEE); // skipped
    a.nop(); // padding so the target lands at offset +2
    a.label("cbz_local");
    a.movs_imm8(2, 1); // r2 != 0
    a.cbnz(2, "cbnz_local"); // taken
    a.movs_imm8(4, 0xEE); // skipped
    a.nop(); // padding so the target lands at offset +2
    a.label("cbnz_local");
    // IT blocks
    a.movs_imm8(0, 0x7F);
    a.cmp_imm8(0, 0x80);
    a.it(0, 0b1000); // it eq: one conditional instruction
    a.movs_imm8(5, 0x55); // EQ true -> executed
    a.it(1, 0b0100); // itt ne
    a.movs_imm8(6, 0x66); // NE false -> skipped
    a.adds_imm8(6, 1); // skipped
    a.it(0xB, 0b1100); // ite lt
    a.movs_imm8(7, 0x77); // LT false (0x7F >= 0x80) -> else
    a.adds_imm8(7, 1); // LT true -> executed
    // BL / BX LR subroutine
    a.bl("sub");
    a.movs_imm8(8, 0x88); // after return
    // pop {pc} return path
    a.bl("sub2");
    a.b("wide_gap_src");
    a.label("sub");
    a.movs_imm8(9, 0x99);
    a.bx(14);
a.label("sub2");
    a.push(0b1000_0000, true); // {r7, lr} so {r7, pc} POP reads LR two words into the list
    a.movs_imm8(10, 0xAA);
    a.pop(0b1000_0000, true); // {r7, pc}
    a.label("wide_gap_src");
    a.pad_nops(1100); // > 2046 bytes, forcing the wide T4 form
    a.label("wide_gap_dst");
    a.movs_imm8(11, 0xBB);
    a.b("halt");
    a.label("cbz_skip");
    a.b("halt");
    a.label("halt");
    a.b("halt");
    conformance("control_flow", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Thumb-2 data processing: MOVW/MOVT, wide DP (register and immediate),
// divide, multiply family, bitfield, saturate, unary, and extend-add.
//
// Restricted to instructions that are valid on the ARMv7-M (Cortex-M3)
// baseline: the DSP zone (QADD family, SADD8/SEL, SXTAB16/UXTAB16, ...) is
// ARMv7E-M only and QEMU's mps2-an385 (cortex-m3) faults on it â€” a divergence
// to revisit when VioARM grows an ARMv7E-M ISA flag.
// ---------------------------------------------------------------------------

#[test]
fn thumb2_dp() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(0, 0x8123_4567);
    a.mov_imm32(1, 0x0000_00FF);
    a.mov_imm32(2, 0x8000_0000);
    a.mov_imm32(3, 0x7FFF_FFFF);
    a.mov_imm32(4, 0x0001_0001);
    // wide register DP
    a.adds_wide(5, 0, 1);
    a.subs_wide(5, 0, 1);
    a.ands_wide(5, 0, 1);
    a.orr_wide(5, 0, 1);
    a.eor_wide(5, 0, 1);
    a.bic_wide(5, 0, 1);
    a.lsls_wide(5, 5, 3, 5); // ANDS r5, r5, r3, LSL #5
    a.mov_wide(5, 0);
    a.mvn_wide(5, 0);
    a.asr_wide(5, 0, 0, 7); // ANDS r5, r0, r0, ASR #7
    // addw / subw
    a.addw(5, 0, 0x1234);
    a.subw(5, 0, 0x0500);
    // divide
    a.sdiv(5, 0, 1); // negative divisor path
    a.udiv(6, 2, 3);
    // multiply family
    a.mul_wide(5, 0, 1);
    a.mla(5, 0, 1, 2);
    a.mls(6, 0, 2, 3);
    a.smull(6, 7, 0, 1);
    a.umull(6, 7, 0, 2);
    a.smlal(6, 7, 0, 1);
    a.umlal(6, 7, 0, 2);
    // bitfield
    a.bfi(5, 0, 4, 8);
    a.bfc(6, 8, 8);
    a.sbfx(5, 0, 4, 8);
    a.ubfx(5, 0, 8, 16);
    // saturate
    a.ssat(5, 0, 12, 0);
    a.usat(6, 2, 12, 0);
    a.ssat(6, 0, 8, 2); // with shift
    // unary
    a.rev_wide(5, 0);
    a.rev16_wide(5, 0);
    a.revsh_wide(5, 0);
    a.rbit(5, 0);
    a.clz(5, 0);
    a.clz(6, 3);
    // extend-add
    a.sxtah(5, 0, 1, 0);
    a.uxtah(5, 0, 1, 1);
    a.sxtab(5, 0, 1, 2);
    a.uxtab(6, 0, 1, 3);
    a.label("halt");
    a.b("halt");
    conformance("thumb2_dp", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Table branches: TBB/TBH in both addressing flavors.
//
// The PC-relative form reads its table from `Align(pc+4, 4) + Rm` with `Rm`
// holding the index, so the table must sit immediately after the instruction
// and its entries must be filled in from the resolved case-label addresses
// *after* assembly. The register form points a base register at a table in
// flash; its entries are relative to the TBB/TBH instruction itself (case
// address minus instruction+4), which is what both simulators implement.
//
// The whole program is assembled with place-holder tables, then `finish()`
// resolves every label and the entries are patched into the image bytes
// before either side runs, so a change in instruction widths can't silently
// corrupt the table.
// ---------------------------------------------------------------------------

#[test]
fn tables() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // ---- PC-relative `TBB` (byte table): index in r0 ----
    // Four branches, one per index; each lands on its case which sets r1 to a
    // distinct value and branches on to the next section's *entry* label, so
    // the next branch gets its index re-loaded.
    for i in 0..4u8 {
        a.label(&format!("tbb{i}_in")); // entry: re-load the index
        a.movs_imm8(0, i);
        a.align4();
        a.label(&format!("tbb{i}"));
        a.tb(15, 0);
        a.dc_u32(0); // patched below
        let next = if i == 3 { "tbh0_in" } else { &format!("tbb{}_in", i + 1) };
        for (k, v) in [(0u8, 0x10u8), (1, 0x20), (2, 0x30), (3, 0x40)] {
            a.label(&format!("tbb{i}_c{k}"));
            a.movs_imm8(1, v);
            a.b(next);
        }
    }

    // ---- PC-relative `TBH` (halfword table): index in r0 ----
    for i in 0..4u8 {
        a.label(&format!("tbh{i}_in"));
        a.movs_imm8(0, i);
        a.align4();
        a.label(&format!("tbh{i}"));
        a.tbh(15, 0);
        a.dc_u32(0); // patched below (four halfword entries)
        a.dc_u32(0);
        let next = if i == 3 { "rtbb_in" } else { &format!("tbh{}_in", i + 1) };
        for (k, v) in [(0u8, 0xA0u8), (1, 0xB0), (2, 0xC0), (3, 0xD0)] {
            a.label(&format!("tbh{i}_c{k}"));
            a.movs_imm8(1, v);
            a.b(next);
        }
    }

    // ---- Register `TBB` (byte table in flash at `rtbb_tab`) ----
    a.label("rtbb_in"); // entry: load the base and index
    a.label("rtbb_base"); // address of the `mov_imm32` immediates
    a.mov_imm32(5, 0); // patched to the table address below
    a.movs_imm8(0, 2); // index 2
    a.align4();
    a.label("rtbb");
    a.tb(5, 0);
    for (k, v) in [(0u8, 0x51u8), (1, 0x52), (2, 0x53), (3, 0x54)] {
        a.label(&format!("rtbb_c{k}"));
        a.movs_imm8(3, v);
        a.b("rtbh_in");
    }
    a.align4();
    a.label("rtbb_tab"); // byte table (4 entries)
    a.dc_u32(0);

    // ---- Register `TBH` (halfword table in flash at `rtbh_tab`) ----
    a.label("rtbh_in");
    a.label("rtbh_base");
    a.mov_imm32(5, 0); // patched below
    a.movs_imm8(0, 3); // index 3
    a.align4();
    a.label("rtbh");
    a.tbh(5, 0);
    for (k, v) in [(0u8, 0x61u8), (1, 0x62), (2, 0x63), (3, 0x64)] {
        a.label(&format!("rtbh_c{k}"));
        a.movs_imm8(4, v);
        a.b("halt");
    }
    a.align4();
    a.label("rtbh_tab"); // halfword table (two words)
    a.dc_u32(0);
    a.dc_u32(0);

    a.label("halt");
    a.b("halt");

    // ---- finish, then patch the table entries from resolved addresses ----
    let mut img: Image = a.clone().finish(SP).expect("assemble");
    fn pc4(img: &Image, name: &str) -> u32 {
        let t = img.addr(name).expect("label");
        (t + 4) & !3
    }
    for i in 0..4u8 {
        let base = pc4(&img, &format!("tbb{i}"));
        for (k, c) in [(0u8, '0'), (1, '1'), (2, '2'), (3, '3')] {
            let target = img.addr(&format!("tbb{i}_c{c}")).expect("case");
            let entry = (target - base) / 2;
            img.bytes[(base + u32::from(k)) as usize] = entry as u8;
        }
    }
    for i in 0..4u8 {
        let base = pc4(&img, &format!("tbh{i}"));
        for (k, c) in [(0u8, '0'), (1, '1'), (2, '2'), (3, '3')] {
            let target = img.addr(&format!("tbh{i}_c{c}")).expect("case");
            let entry = (target - base) / 2;
            let off = (base + u32::from(2 * k)) as usize;
            img.bytes[off] = entry as u8;
            img.bytes[off + 1] = (entry >> 8) as u8;
        }
    }
    // register form: entries are relative to the *instruction*, not the base.
    // The flash-resident table is loaded into r5 at runtime.
    {
        let tab = img.addr("rtbb_tab").expect("table");
        let bs = img.addr("rtbb").expect("label") + 4;
        let mut word = 0u32;
        for (k, c) in [(0u8, '0'), (1, '1'), (2, '2'), (3, '3')] {
            let target = img.addr(&format!("rtbb_c{c}")).expect("case");
            let entry = (target - bs) / 2;
            word |= u32::from(entry) << (8 * u32::from(k));
        }
        let mvw = img.addr("rtbb_base").expect("base") as usize;
        patch_movw(&mut img.bytes[..], mvw, 5, tab as u16);
        img.bytes[tab as usize..tab as usize + 4].copy_from_slice(&word.to_le_bytes());
    }
    {
        let tab = img.addr("rtbh_tab").expect("table");
        let bs = img.addr("rtbh").expect("label") + 4;
        let mut word = 0u64;
        for (k, c) in [(0u8, '0'), (1, '1'), (2, '2'), (3, '3')] {
            let target = img.addr(&format!("rtbh_c{c}")).expect("case");
            let entry = (target - bs) / 2;
            word |= u64::from(entry) << (16 * u64::from(k));
        }
        let mvw = img.addr("rtbh_base").expect("base") as usize;
        patch_movw(&mut img.bytes[..], mvw, 5, tab as u16);
        img.bytes[tab as usize..tab as usize + 8].copy_from_slice(&word.to_le_bytes());
    }

    // ---- run both sides on the patched image and diff ----
    let stop = img.addr("halt").expect("halt");
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "tables: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );

    let qemu = qemu_run(&mt, &img.bytes, stop, Duration::from_secs(15));
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "tables: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("tables", &vio, &qemu).expect("traces agree");
}

// ---------------------------------------------------------------------------
// Load/store double: LDRD/STRD in the offset, pre-indexed (writeback) and
// post-indexed forms, plus the PC-relative `LDRD` literal. Register pairs must
// avoid the base when writeback is used, and neither end of a pair may touch
// the PC or equal the other.
// ---------------------------------------------------------------------------

#[test]
fn ldrd_strd() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(4, 0x2000_2000); // r4 = SRAM base
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.mov_imm32(2, 0x5555_6666);
    a.mov_imm32(3, 0x7777_8888);

    // offset form (no writeback): [r4] = r0:r1, read back into r5:r6
    a.strd(0, 1, 4, 0);
    a.ldrd(5, 6, 4, 0);
    // offset with a non-zero displacement
    a.strd(2, 3, 4, 16);
    a.ldrd(9, 10, 4, 16);
    // pre-indexed with writeback: [r4+32] and r4 += 32
    a.strd_pre(0, 1, 4, 32);
    a.ldrd_pre(5, 6, 4, 32);
    // post-indexed: [r4], then r4 += 8
    a.strd_post(2, 3, 4, 8);
    a.ldrd_post(9, 10, 4, 8);
    // writeback base must not overlap the pair: r0:r1 vs r4 is fine; verify
    // r4 advanced by the writebacks while the pair registers kept their data.
    a.mov_imm32(4, 0x2000_2400); // fresh base
    a.strd(0, 1, 4, 0); // offset store, no writeback
    a.strd_pre(2, 3, 4, 64); // pre-indexed: [r4+64], r4 += 64
    a.ldrd_pre(5, 6, 4, 64); // reads the pair just stored, r4 += 64
    // PC-relative `LDRD` literal from a 4-aligned pool
    a.ldrd_lit(5, 6, "pair");
    a.b("past_pair");
    a.align4();
    a.label("pair");
    a.dc_u32(0xAABB_CCDD);
    a.dc_u32(0xEEFF_0011);
    a.label("past_pair");

    a.label("halt");
    a.b("halt");
    conformance("ldrd_strd", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// ARMv7E-M DSP zone, diffed against a Cortex-M4 (mps2-an386): the dual
// multiply-accumulate family (SMLAD/X, SMLSD/X, SMLALD/X, SMLSLD/X, SMMLA/R,
// SMMLS/R), the halfword-picking multiplies (SMULxx, SMLAx, SMLAWB/WT),
// SMMUL/R, USAD8/USADA8, and the saturating QADD/QDADD/QSUB/QDSUB. These all
// fault on the M3 board, so this test runs only when a QEMU is available and
// uses the M4 reference machine.
// ---------------------------------------------------------------------------

#[test]
fn dsp_m4() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // operand registers with interesting halfwords / signs
    a.mov_imm32(0, 0x8000_0001); // lo hi halves: 0x0001 0x8000
    a.mov_imm32(1, 0xFFFF_7FFF); // lo hi halves: 0x7FFF 0xFFFF (-1)
    a.mov_imm32(2, 0x0001_0002);
    a.mov_imm32(3, 0xFFFF_FFFE);
    a.mov_imm32(4, 0x7FFF_8000);

    // halfword-picking multiplies
    a.smulbb(5, 0, 1); // 0x0001 * 0x7FFF
    a.smulbt(6, 0, 1); // 0x0001 * 0xFFFF
    a.smultb(7, 0, 1); // 0x8000 * 0x7FFF
    a.smultt(8, 0, 1); // 0x8000 * 0xFFFF
    a.smulwb(5, 0, 2); // full r0 * low half r2 >> 16
    a.smulwt(6, 0, 2); // full r0 * high half r2 >> 16

    // dual multiply-accumulate
    a.smlabb(7, 0, 1, 4);
    a.smlabt(8, 0, 1, 4);
    a.smlatb(9, 0, 1, 4);
    a.smlatt(10, 0, 1, 4);
    a.smlawb(11, 0, 2, 4);
    a.smlawt(12, 0, 2, 4);
    a.smlad(5, 0, 1, 4);
    a.smladx(6, 0, 1, 4);
    a.smlsd(7, 0, 1, 4);
    a.smlsdx(8, 0, 1, 4);

    // signed most-significant-word multiplies
    a.smmul(9, 0, 1);
    a.smmulr(10, 0, 1);
    a.smmla(11, 0, 1, 4);
    a.smmlar(12, 0, 1, 4);
    a.smmls(5, 0, 1, 4);
    a.smmlsr(6, 0, 1, 4);

    // sum-of-absolute-differences
    a.usad8(7, 0, 1);
    a.usada8(8, 0, 1, 4);

    // long dual multiply-accumulate
    a.smlalbb(5, 6, 0, 1);
    a.smlalbt(7, 8, 0, 1);
    a.smlaltb(9, 10, 0, 1);
    a.smlaltt(11, 12, 0, 1);
    a.smlald(5, 6, 0, 1);
    a.smlaldx(7, 8, 0, 1);
    a.smlsld(9, 10, 0, 1);
    a.smlsldx(11, 12, 0, 1);
    a.umaal(5, 6, 0, 1);

    // saturating adds/subs (may set Q)
    a.qadd(7, 3, 4); // huge + huge -> saturate
    a.qdadd(8, 0, 1);
    a.qsub(9, 3, 4);
    a.qdsub(10, 0, 1);

    a.label("halt");
    a.b("halt");
    conformance("dsp_m4", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Parallel add/subtract family (ARMv7E-M): SADD8/SSUB8/SADD16/SSUB16/SASX/SSAX
// in every packing (S/U/Q/SH/UQ/UH) plus the GE-driven byte `SEL`. The signed
// `S` and unsigned `U` forms write the per-lane GE bits of APSR (which the
// snapshot compares via xPSR[19:16]); the saturating and halving forms must
// leave GE alone. Runs on the M4 reference machine.
// ---------------------------------------------------------------------------

#[test]
fn par_m4() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // byte operands with mixed signs (+128..-128 extremes), halfword operands
    // with mixed signs, and all-identical magnitudes to force out-of-range
    // sums/subtracts.
    a.mov_imm32(0, 0x8081_7F80); // bytes: -128 -127 +127 -128
    a.mov_imm32(1, 0x0102_0001); // bytes: +1 +2 0 +1
    a.mov_imm32(2, 0x0002_0001); // halfwords: +2 +1
    a.mov_imm32(3, 0x7FFF_FFFF); // halfwords: +32767 -1
    a.mov_imm32(4, 0x7F7F_7F7F); // all +127 / +32639
    a.mov_imm32(5, 0x8080_8080); // all -128 / -32512
    a.mov_imm32(6, 0xFFFF_FFFF); // all -1 / max unsigned
    a.mov_imm32(7, 0x0000_0001);

    // GE-setting byte adds/subs (S and U modes)
    a.sadd8(8, 0, 1);
    a.ssub8(9, 0, 1);
    a.uadd8(10, 0, 6);
    a.usub8(11, 0, 1);
    a.sel(8, 0, 1); // byte-select on GE left by usub8

    // GE-setting halfword add/sub/swap families
    a.sadd16(9, 2, 3);
    a.ssub16(10, 2, 3);
    a.sasx(11, 2, 3);
    a.ssax(8, 2, 3);
    a.uadd16(9, 2, 6);
    a.usub16(10, 2, 3);
    a.uasx(11, 2, 3);
    a.usax(9, 2, 3);
    a.sel(10, 0, 1);

    // saturating add/sub (may light Q, must leave GE alone)
    a.qadd8(11, 4, 4); // 127+127 lanes saturate
    a.qadd16(8, 4, 6);
    a.qsub8(9, 5, 5); // -128 - -128 == 0
    a.qsub16(10, 4, 4);
    a.qasx(11, 3, 5);
    a.qsax(8, 2, 6);
    a.sel(9, 0, 1); // GE untouched by the Q ops

    // signed halving add/sub
    a.shadd8(10, 4, 4);
    a.shadd16(11, 2, 3);
    a.shsub8(8, 4, 4);
    a.shsub16(9, 2, 3);
    a.shasx(10, 2, 3);
    a.shsax(11, 2, 3);

    // unsigned saturating add/sub
    a.uqadd8(8, 6, 1);
    a.uqadd16(9, 2, 6);
    a.uqsub8(10, 6, 1);
    a.uqsub16(11, 2, 3);
    a.uqasx(8, 2, 6);
    a.uqsax(9, 2, 6);

    // unsigned halving add/sub
    a.uhadd8(10, 4, 4);
    a.uhadd16(11, 2, 3);
    a.uhsub8(8, 4, 4);
    a.uhsub16(9, 2, 3);
    a.uhasx(10, 2, 3);
    a.uhsax(11, 2, 3);

    a.label("halt");
    a.b("halt");
    conformance("par_m4", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Extend family: the Thumb-1 `SXTH/SXTB/UXTH/UXTB` and `REV/REV16/REVSH` narrow
// unary forms, exercised against rotation `ROR #rot*8` of the 32-bit
// `SXTAH/UXTAH/SXTAB16/UXTAB16/SXTAB/UXTAB` (extend + add) forms across all
// four rotations. Rotation 0 also re-checks the alias-free versions already
// sampled by `thumb2_dp`, but with different operands. The `SXTAB16/UXTAB16`
// pair are DSP-extension instructions that fault on the M3 board, so the whole
// family runs on the M4 reference machine.
// ---------------------------------------------------------------------------

#[test]
fn extend() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // byte/halfword patterns with mixed signs, plus an addend with distinct
    // top/bottom halves to expose per-halfword (b16) behavior.
    a.mov_imm32(0, 0x8012_7FFE); // bytes -128 +18 +127 -2
    a.mov_imm32(1, 0x0123_4567); // bytes +1 +35 +69 +103
    a.mov_imm32(2, 0xFFFF_8000); // halfwords -1 -32768

    // narrow unary reverse
    a.rev(5, 0);
    a.rev16(6, 0);
    a.revsh(7, 0);
    a.rev(8, 1);
    a.rev16(9, 1);
    a.revsh(10, 1);

    // narrow unary sign/zero extend
    a.sxth(5, 0);
    a.sxtb(6, 0);
    a.uxth(7, 0);
    a.uxtb(8, 0);
    a.sxth(5, 1);
    a.sxtb(6, 1);
    a.uxth(7, 1);
    a.uxtb(8, 1);

    // extend + add across all four rotations
    a.sxtah(5, 2, 0, 0);
    a.sxtah(6, 2, 0, 1);
    a.sxtah(7, 2, 0, 2);
    a.sxtah(8, 2, 0, 3);
    a.uxtah(5, 2, 0, 0);
    a.uxtah(6, 2, 0, 1);
    a.uxtah(7, 2, 0, 2);
    a.uxtah(8, 2, 0, 3);
    a.sxtab16(9, 2, 0, 0);
    a.sxtab16(10, 2, 0, 1);
    a.sxtab16(11, 2, 0, 2);
    a.sxtab16(5, 2, 0, 3);
    a.uxtab16(6, 2, 0, 0);
    a.uxtab16(7, 2, 0, 1);
    a.uxtab16(8, 2, 0, 2);
    a.uxtab16(9, 2, 0, 3);
    a.sxtab(10, 2, 0, 0);
    a.sxtab(11, 2, 0, 1);
    a.sxtab(5, 2, 0, 2);
    a.sxtab(6, 2, 0, 3);
    a.uxtab(7, 2, 0, 0);
    a.uxtab(8, 2, 0, 1);
    a.uxtab(9, 2, 0, 2);
    a.uxtab(10, 2, 0, 3);

    a.label("halt");
    a.b("halt");
    conformance("extend", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Saturate and bitfield family: `SSAT`/`USAT` (LSL and ASR shifts), the
// per-halfword `SSAT16`/`USAT16` (the `sh==1, shift==0` alias), and
// `SBFX`/`UBFX`/`BFI`/`BFC` at edge-adjacent lsb/width combinations. The
// saturating ops exercise the sticky Q flag, which the snapshot compares via
// xPSR[27]. Runs on the M4 reference machine (the `SSAT16`/`USAT16` alias
// encodings are faulted by the M3 board).
// ---------------------------------------------------------------------------

#[test]
fn sat_bitfield() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // operands at the signed/unsigned extremes and with distinct halfwords
    a.mov_imm32(0, 0x8000_0000); // min signed
    a.mov_imm32(1, 0x7FFF_FFFF); // max signed
    a.mov_imm32(2, 0x0000_FFFF);
    a.mov_imm32(3, 0xFFFF_8000); // halfwords: -1 -32768
    a.mov_imm32(4, 0x7FFF_0000); // halfwords: +32767 0

    // signed/unsigned saturate, LSL shifts
    a.ssat(5, 0, 1, 0);
    a.ssat(6, 1, 32, 0);
    a.ssat(7, 2, 8, 4);
    a.ssat(8, 3, 16, 0);
    a.ssat(5, 1, 12, 16);
    a.usat(6, 0, 1, 0);
    a.usat(7, 1, 31, 0);
    a.usat(8, 2, 16, 4);
    a.usat(5, 3, 8, 0);
    a.usat(6, 0, 24, 8);

    // signed/unsigned saturate, ASR shifts
    a.ssat_asr(7, 0, 8, 4);
    a.ssat_asr(8, 1, 8, 16);
    a.ssat_asr(9, 3, 1, 4);
    a.usat_asr(5, 0, 8, 4);
    a.usat_asr(6, 3, 16, 16);
    a.usat_asr(7, 1, 12, 8);

    // per-halfword saturate
    a.ssat16(8, 3, 1);
    a.ssat16(9, 4, 16);
    a.ssat16(5, 0, 8);
    a.ssat16(6, 1, 4);
    a.usat16(7, 3, 1);
    a.usat16(8, 4, 15);
    a.usat16(9, 2, 12);

    // bitfield extracts
    a.sbfx(5, 0, 0, 1);
    a.sbfx(6, 1, 0, 32);
    a.sbfx(7, 2, 4, 8);
    a.sbfx(8, 3, 16, 8);
    a.sbfx(9, 0, 31, 1);
    a.sbfx(10, 1, 24, 8);
    a.sbfx(11, 2, 8, 16);
    a.ubfx(5, 0, 0, 1);
    a.ubfx(6, 1, 0, 32);
    a.ubfx(7, 2, 4, 8);
    a.ubfx(8, 3, 16, 8);
    a.ubfx(9, 0, 31, 1);
    a.ubfx(10, 2, 8, 16);
    a.ubfx(11, 1, 1, 16);

    // bitfield inserts / clears
    a.bfi(5, 0, 0, 32);
    a.bfi(6, 1, 4, 8);
    a.bfi(7, 2, 16, 16);
    a.bfi(8, 3, 24, 8);
    a.bfi(9, 0, 1, 31);
    a.bfc(5, 0, 32);
    a.bfc(6, 4, 8);
    a.bfc(7, 16, 16);
    a.bfc(8, 24, 8);
    a.bfc(9, 31, 1);

    a.label("halt");
    a.b("halt");
    conformance("sat_bitfield", &mut a, "halt", &mt);
}

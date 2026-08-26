//! Differential conformance tests: identical Thumb binaries run in `vioarm`
//! and in `qemu-system-arm` (Cortex-M3), and the per-instruction register
//! traces are diffed. Each test builds its program with the crate's tiny
//! assembler, then the harness stops both sides at the program's landing loop
//! and requires byte-for-byte agreement on real registers and the modeled xPSR
//! bits for every single executed instruction.
//!
//! The tests are skipped (with a note) when no `qemu-system-arm` is found.
//!
//! # Known reference-model limitation
//!
//! QEMU 10.2 (the Cortex-M3 `armv7m` model used as the oracle here) cannot
//! execute a *taken* conditional branch that is the conditional instruction of
//! an IT block: it escalates to a HardFault and locks up, even for clang's
//! canonical `it ne; bne <label>` sequence, which real Cortex-M silicon and
//! VioARM both execute correctly (the taken branch clears ITSTATE). The
//! differential tests therefore only exercise branches whose IT condition is
//! false (branch suppressed, ITSTATE advances), while VioARM's taken-branch
//! semantics are covered by unit tests in `vioarm-core`
//! (`it_block_branch_clears_state`).

use std::time::Duration;
use vioarm_qemu::asm::{Asm, Image, CODE_BASE, SP};
use vioarm_qemu::qemu::{run_until_pc as qemu_run, QemuMachine};
use vioarm_qemu::vioarm::run_until_pc as vio_run;
use vioarm_qemu::vioarm::run_until_pc_v6m as vio_run_v6m;
use vioarm_qemu::vioarm::run_until_pc_v7em as vio_run_v7em;
use vioarm_qemu::vioarm::run_until_pc_v8m as vio_run_v8m;
use vioarm_qemu::{compare_traces, CpuState};

/// Run `bin` in QEMU up to `stop`, retrying up to three times when the spawn
/// produces no state at all. Under parallel test load that many QEMU processes
/// boot simultaneously and a straggler can be starved; an empty trace is a
/// spawn failure, never a legitimate result.
fn qemu_run_retry(mt: &QemuMachine, bin: &[u8], stop: u32) -> Vec<CpuState> {
    for attempt in 0..3 {
        let states = qemu_run(mt, bin, stop, Duration::from_secs(20));
        if !states.is_empty() {
            return states;
        }
        if attempt == 2 {
            return states;
        }
    }
    unreachable!("loop always returns")
}

/// A test harness that assembles `a`, picks the halt address, runs both
/// simulators, and diffs.
///
/// A divergence is re-checked once with a fresh QEMU run before failing:
/// under parallel test load the oracle's `-d cpu` log can lose a mid-file
/// state block (the process is starved, then killed past a quiet window,
/// leaving a truncated block the parser drops), which shifts its trace and
/// reads as a false divergence. VioARM is deterministic (locked by
/// `determinism_twice_same_trace`), so a *real* model defect diverges on
/// both attempts and still fails; only the flaky-oracle path recovers.
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
    let qemu = qemu_run_retry(mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "{name}: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    if let Err(first) = compare_traces(name, &vio, &qemu) {
        // Fresh run of both sides; VioARM first so the second QEMU trace is
        // compared against an identically-computed expectation.
        let (vio2, reached2) = vio_run(&img.bytes, stop);
        assert!(reached2, "{name}: retry never reached halt");
        let qemu2 = qemu_run_retry(mt, &img.bytes, stop);
        assert!(
            !qemu2.is_empty() && qemu2.last().expect("empty").r[15] & !1 == stop,
            "{name}: retry never reached halt on the oracle side"
        );
        if let Err(second) = compare_traces(name, &vio2, &qemu2) {
            panic!("{name}: traces diverge (retried): first: {first}\nsecond: {second}");
        }
    }
}

fn snapshot_like(states: &[CpuState]) -> String {
    states
        .last()
        .map_or_else(|| "no states".to_string(), std::string::ToString::to_string)
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

/// An ARMv6-M (Cortex-M0, BBC micro:bit) QEMU reference. Returns `None` when
/// `qemu-system-arm` is missing.
fn qemu_m0() -> Option<QemuMachine> {
    let mt = QemuMachine::detect_m0();
    if mt.is_none() {
        eprintln!("skipping ARMv6-M conformance tests: `qemu-system-arm` not found on PATH");
    }
    mt
}

/// An ARMv8-M mainline (Cortex-M33, mps2-an505) QEMU reference, the oracle for
/// the ARMv8-M baseline subset. Returns `None` when `qemu-system-arm` is
/// missing.
fn qemu_m33() -> Option<QemuMachine> {
    let mt = QemuMachine::detect_m33();
    if mt.is_none() {
        eprintln!(
            "skipping ARMv8-M baseline conformance tests: `qemu-system-arm` not found on PATH"
        );
    }
    mt
}

/// An ARMv7E-M (Cortex-M7) QEMU reference with FPv5-D16, for double tests.
fn qemu_m7() -> Option<QemuMachine> {
    let mt = QemuMachine::detect_m7();
    if mt.is_none() {
        eprintln!("skipping ARMv7E-M conformance tests: `qemu-system-arm` not found on PATH");
    }
    mt
}

/// The ARMv6-M variant of [`conformance`]: runs `a` through the v6m VioARM
/// machine (no FPU) against the Cortex-M0 reference.
fn conformance_m0(name: &str, a: &mut Asm, halt: &str, mt: &QemuMachine) {
    let img: Image = a.clone().finish(SP).expect("assemble");
    let stop = img
        .addr(halt)
        .unwrap_or_else(|| panic!("{name}: no label {halt:?}"));
    let (vio, reached) = vio_run_v6m(&img.bytes, stop);
    assert!(
        reached,
        "{name}: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "{name}: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces(name, &vio, &qemu).expect("traces agree");
}

/// The ARMv8-M baseline variant of [`conformance`]: runs `a` through the
/// ARMv8-M baseline VioARM machine (Cortex-M23 semantics, no FPU) against the
/// Cortex-M33 mainline reference. The image is laid out for flash at
/// `0x1000_0000` so both simulators execute at identical addresses.
fn conformance_v8m(name: &str, a: &mut Asm, halt: &str, mt: &QemuMachine) {
    conformance_v8m_at(name, a, halt, SP, mt)
}

/// The ARMv8-M baseline variant of [`conformance`]: runs `a` through the
/// ARMv8-M baseline VioARM machine (Cortex-M23 semantics, no FPU) against the
/// Cortex-M33 mainline reference. The image is laid out for flash at
/// `0x1000_0000` so both simulators execute at identical addresses.
fn conformance_v8m_at(name: &str, a: &mut Asm, halt: &str, sp: u32, mt: &QemuMachine) {
    let img: Image = a.clone().finish_at(sp, 0x1000_0000).expect("assemble");
    let stop = img
        .addr(halt)
        .unwrap_or_else(|| panic!("{name}: no label {halt:?}"));
    let (vio, reached) = vio_run_v8m(&img.bytes, stop);
    assert!(
        reached,
        "{name}: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "{name}: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces(name, &vio, &qemu).expect("traces agree");
}

fn conformance_v7em(name: &str, a: &mut Asm, halt: &str, mt: &QemuMachine) {
    let img: Image = a.clone().finish(SP).expect("assemble");
    let stop = img
        .addr(halt)
        .unwrap_or_else(|| panic!("{name}: no label {halt:?}"));
    let (vio, reached) = vio_run_v7em(&img.bytes, stop);
    assert!(
        reached,
        "{name}: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "{name}: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces(name, &vio, &qemu).expect("traces agree");
}

/// Re-encode a `MOVW Rd, #imm16` into the 4 bytes at `addr` of `bytes`. Used
/// to patch a register form's base address into a place-holder `movw` after
/// assembly, when the address is only known once labels are resolved.
fn patch_movw(bytes: &mut [u8], addr: usize, rd: u8, imm: u16) {
    let base: u16 = 0xF240;
    let hi = base | ((imm >> 11) & 1) << 10 | (imm >> 12);
    let lo = ((imm >> 8) & 7) << 12 | (u16::from(rd) << 8) | (imm & 0xFF);
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
fn loadstore_small() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Seed known byte/halfword patterns, including sign bits.
    a.mov_imm32(4, 0x2000_0000);
    a.mov_imm32(0, 0x4080_0001);
    a.mov_imm32(1, 0x1234_5678);
    a.strb_wide(0, 4, 0); // [0] = 0x01
    a.strb_wide(0, 4, 1); // [1] = 0x80 (LDRSB -> -128)
    a.strb_wide(0, 4, 2); // [2] = 0x40
    a.strh_wide(1, 4, 4); // [4] = 0x5678
    a.strh_wide(1, 4, 6); // [6] = 0x1234
    a.strh_wide(1, 4, 8); // [8] = 0x1234

    a.mov_imm32(4, 0x2000_0000);

    // Byte pre/post-indexed loads.
    a.ldrb_pre(3, 4, 1); // r3 = [1] = 0x80 ; r4 = 1
    a.mov_imm32(4, 0x2000_0000);
    a.ldrb_post(5, 4, 2); // r5 = [0] = 0x01 ; r4 = 2
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsb_pre(3, 4, 1); // r3 = sext([1]) = 0xFFFFFF80 ; r4 = 1
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsb_post(5, 4, 2); // r5 = sext([0]) = 1 ; r4 = 2
    a.mov_imm32(4, 0x2000_0000);
    a.ldrbt(6, 4, 1); // r6 = [1] = 0x80 ; no write-back
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsbt(7, 4, 1); // r7 = sext([1]) = 0xFFFFFF80 ; no write-back

    // Byte pre/post-indexed stores, then read them back.
    a.mov_imm32(4, 0x2000_0000);
    a.strb_pre(1, 4, 5); // [5] = r1 & 0xFF ; r4 = 5
    a.mov_imm32(4, 0x2000_0000);
    a.strb_post(1, 4, 3); // [0] = r1 & 0xFF ; r4 = 3
    a.mov_imm32(4, 0x2000_0000);
    a.strbt(1, 4, 7); // [7] = r1 & 0xFF ; no write-back
    a.mov_imm32(4, 0x2000_0000);
    a.ldrb_wide(2, 4, 5); // r2 = [5]
    a.ldrb_wide(8, 4, 0); // r8 = [0]
    a.ldrb_wide(2, 4, 7); // r2 = [7]

    a.mov_imm32(4, 0x2000_0000);

    // Halfword pre/post-indexed loads (aligned offsets).
    a.ldrh_pre(3, 4, 4); // r3 = [4] = 0x5678 ; r4 = 4
    a.mov_imm32(4, 0x2000_0000);
    a.ldrh_post(5, 4, 4); // r5 = [0] = 0x0001 (byte leftovers) ; r4 = 4
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsh_pre(3, 4, 4); // r3 = sext([4]) = 0x5678 ; r4 = 4
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsh_post(5, 4, 6); // r5 = sext([0]) = 1 ; r4 = 6
    a.mov_imm32(4, 0x2000_0000);
    a.ldrht(6, 4, 6); // r6 = [6] = 0x1234 ; no write-back
    a.mov_imm32(4, 0x2000_0000);
    a.ldrsht(7, 4, 6); // r7 = sext([6]) = 0x1234 ; no write-back

    // Halfword pre/post-indexed stores.
    a.mov_imm32(4, 0x2000_0000);
    a.strh_pre(1, 4, 10); // [10] = r1 & 0xFFFF ; r4 = 10
    a.mov_imm32(4, 0x2000_0000);
    a.strh_post(1, 4, 12); // [0] = r1 & 0xFFFF ; r4 = 12
    a.mov_imm32(4, 0x2000_0000);
    a.strht(1, 4, 14); // [14] = r1 & 0xFFFF ; no write-back

    a.label("halt");
    a.b("halt");
    conformance("loadstore_small", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------

#[test]
fn loadstore_neg() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Seed distinct values at addresses we will reach by stepping down.
    a.mov_imm32(4, 0x2000_0008);
    a.mov_imm32(0, 0xDEAD_BEEF);
    a.mov_imm32(1, 0x1122_3344);
    a.str_wide(0, 4, 0); // [8] = BEEF
    a.str_wide(1, 4, -4i32 as u16); // [4] = 3344
    a.str_wide(0, 4, -8i32 as u16); // [0] = BEEF

    // Negative pre-indexed: load at Rn - imm, then Rn -= imm.
    a.mov_imm32(4, 0x2000_0008);
    a.ldr_pre_neg(5, 4, 4); // r5 = [4] = 0x1122_3344 ; r4 = 0x2000_0004
    a.str_pre_neg(1, 4, 4); // [0] = 0x1122_3344 ; r4 = 0x2000_0000
    a.mov_imm32(4, 0x2000_0008);
    a.ldr_post_neg(6, 4, 4); // r6 = [8] = 0xDEAD_BEEF ; r4 = 0x2000_0004
    a.str_post_neg(0, 4, 4); // [4] = 0xDEAD_BEEF ; r4 = 0x2000_0000

    // Byte/halfword/signed negative forms.
    a.mov_imm32(0, 0x0000_0061); // ASCII 'a' at byte offset
    a.mov_imm32(1, 0x0000_00FF);
    a.mov_imm32(4, 0x2000_0004);
    a.strb_wide(1, 4, 0); // [4] = 0xFF
    a.strh_wide(0, 4, 0); // [4] = 0x0061

    a.mov_imm32(4, 0x2000_0008);
    a.ldrb_pre_neg(7, 4, 4); // r7 = byte[4] = 0x61 ; r4 = 0x2000_0004
    a.strb_pre_neg(1, 4, 4); // byte[0] = 0xFF ; r4 = 0x2000_0000
    a.mov_imm32(4, 0x2000_0008);
    a.ldrh_post_neg(8, 4, 4); // r8 = hword[8] = 0xDEAD ; r4 = 0x2000_0004
    a.strh_post_neg(0, 4, 4); // hword[4] = 0x0061 ; r4 = 0x2000_0000
    a.mov_imm32(4, 0x2000_0008);
    a.ldrsb_pre_neg(9, 4, 4); // r9 = sext(byte[4]) = 0x61 ; r4 = 0x2000_0004
    a.ldrsh_post_neg(5, 4, 4); // r5 = sext(hword[4]) = 0x0061 ; r4 = 0x2000_0000
    a.mov_imm32(4, 0x2000_0008);
    a.ldrsb_post_neg(6, 4, 4); // r6 = sext(byte[8]) = sext(0xEF0F) ... ; r4 = 0x2000_0004
    a.ldrsh_pre_neg(7, 4, 4); // r7 = sext(hword[4]) = 0x0061 ; r4 = 0x2000_0000

    // T32 PC-relative literals with negative offsets (pool placed before the
    // loads, so `U == 0`). The imm12 offset is absolute bytes from the
    // 4-aligned literal base, so every slot is 4-aligned even for narrow loads.
    a.b("past_pool");
    a.align4();
    a.label("litpool"); // +0: word slot
    a.dc_u32(0x3344_5566);
    a.label("litpool4"); // +4: byte slot (LE low byte 0x99)
    a.dc_u32(0x7700_8899);
    a.label("litpool8"); // +8: hword slot (LE value 0xCCDD)
    a.dc_u32(0xAABB_CCDD);
    a.label("litpool12"); // +12: dword slot
    a.dc_u32(0xEEFF_0011);
    a.label("litpool16"); // +16: another word
    a.dc_u32(0x8899_AABB);
    a.label("past_pool");
    a.ldr_lit_neg(0, "litpool"); // r0 = word = 0x33445566
    a.ldrb_lit_neg(1, "litpool4"); // r1 = byte[+4] = 0x99
    a.ldrsb_lit_neg(2, "litpool4"); // r2 = sext(byte[+4]) = 0x99
    a.ldrh_lit_neg(3, "litpool8"); // r3 = hword[+8] = 0xCCDD
    a.ldrsh_lit_neg(4, "litpool8"); // r4 = sext(hword[+8]) = 0xFFFFCCDD
    a.ldr_lit_neg(5, "litpool12"); // r5 = word = 0xEEFF0011
    a.ldrsh_lit_neg(6, "litpool16"); // r6 = sext(hword[+16]) = 0xFFFFAABB
                                     // Positive literal with the pool after the loads (`U == 1`).
    a.ldr_lit_wide(7, "pospool");
    a.ldrb_lit_wide(8, "pospool4");
    a.ldrsb_lit_wide(9, "pospool8");
    a.ldrh_lit_wide(8, "pospool12");
    a.ldrsh_lit_wide(9, "pospool16");
    a.b("past_pospool");
    a.align4();
    a.label("pospool"); // +0: word slot
    a.dc_u32(0x0102_0304);
    a.label("pospool4"); // +4: byte slot (LE low byte 0x00)
    a.dc_u32(0x80FF_FF00);
    a.label("pospool8"); // +8: hword slot (LE value 0x00FF)
    a.dc_u32(0x7FF0_00FF);
    a.label("pospool12"); // +12: dword slot
    a.dc_u32(0x1122_3344);
    a.label("pospool16"); // +16: another word
    a.dc_u32(0x7765_5443);
    a.label("past_pospool");

    a.label("halt");
    a.b("halt");
    conformance("loadstore_neg", &mut a, "halt", &mt);
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

    // CLREX clears the local monitor: a STREX immediately after must fail
    // and must not store, even at the same address that was just tagged.
    a.ldrex(3, 4, 0); // tag [r4]
    a.clrex(); // clear the monitor
    a.strex(5, 1, 4, 0); // r5 = 1, [r4] unchanged
    a.ldr_imm(7, 4, 0); // [r4] still 0xAABBCCDD
    a.dmb();
    // A fresh LDREX re-tags and the store succeeds again.
    a.ldrex(3, 4, 0);
    a.strex(5, 1, 4, 0); // r5 = 0, [r4] = 0xAABBCCDD
    a.dsb();

    // Barriers are architectural no-ops: the register file must be untouched
    // across a DMB/DSB/ISB run (verify by summing r1+r7 into r0, which must
    // round-trip unchanged through the barriers).
    a.adds_reg(0, 1, 7); // r0 = r1 + r7
    a.dmb();
    a.dsb();
    a.isb();
    a.adds_wide(6, 0, 0); // r6 = r0 (same value through barriers)
    a.dmb();

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
// An exception taken while the thread runs on the process stack: CONTROL.SPSEL
// switches the active SP to PSP, so the SVC frame is pushed there; the handler
// runs on the main stack; `bx lr` returns via EXC_RETURN_THREAD_PSP
// (0xfffffffd), handing the thread back its PSP with both banks intact.
// ---------------------------------------------------------------------------

#[test]
fn svc_on_process_stack() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler"); // SVC vector
    a.mov_imm32(0, 0x1111_2222);
    // CONTROL.SPSEL = 1, then give PSP a known value.
    a.movs_imm8(0, 2); // 0b10: SPSEL
    a.msr(0x14, 0); // MSR CONTROL, r0
    a.mov_imm32(0, 0x2000_3ff0); // PSP value
    a.msr(0x09, 0); // MSR PSP, r0
    a.mrs(2, 0x09); // r2 = PSP
    a.mrs(3, 0x08); // r3 = MSP
    a.svc(0); // taken from Thread mode on the process stack
    a.mrs(4, 0x09); // r4 = PSP after return
    a.mrs(5, 0x08); // r5 = MSP after return
    a.movs_imm8(6, 0xAA);
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(7, 0x99); // marker
    a.bx(14); // return via EXC_RETURN_THREAD_PSP
    a.label("halt");
    a.b("halt");
    conformance("svc_on_process_stack", &mut a, "halt", &mt);
}

#[test]
fn hardfault_on_process_stack() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf"); // HardFault vector
    a.mov_imm32(0, 0x1111_2222);
    a.movs_imm8(0, 2); // CONTROL.SPSEL = 1
    a.msr(0x14, 0);
    a.mov_imm32(0, 0x2000_3ff0); // PSP value
    a.msr(0x09, 0);
    a.mrs(2, 0x09); // r2 = PSP
    a.raw(0xDE00); // UDEF -> HardFault, frame pushed on the process stack
    a.mrs(3, 0x08); // r3 = MSP after return
    a.mrs(4, 0x09); // r4 = PSP after return
    a.movs_imm8(5, 0xAA);
    a.b("halt");
    a.label("hf");
    // Handler runs on the main stack; the frame lives at PSP - 32.
    a.mov_imm32(6, 0x2000_3fd0); // frame base
    a.ldr_wide(7, 6, 24); // stacked PC (the UDEF)
    a.adds_imm8(7, 2); // skip it
    a.str_imm(7, 6, 24);
    a.bx(14); // return to process-thread via EXC_RETURN_THREAD_PSP
    a.label("halt");
    a.b("halt");
    conformance("hardfault_on_process_stack", &mut a, "halt", &mt);
}

#[test]
fn exception_msp_align() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler");
    a.mov_imm32(0, 0x1111_2222);
    // A non-word-aligned MSP write is silently word-aligned (low 2 bits
    // dropped), so MSR MSP, #0x20002002 yields MSP == 0x20002000.
    a.mov_imm32(0, 0x2000_2002);
    a.msr(0x08, 0); // MSR MSP, r0
    a.mrs(1, 0x08); // r1 = 0x20002000
                    // A word-aligned but 8-misaligned MSP (0x20002004): exception entry must
                    // decrement SP by 4 (setting xPSR.ALIGN), push the frame at 0x20001fe0,
                    // and the handler's THREAD_MSP return must re-align SP back to 0x20002004.
    a.mov_imm32(0, 0x2000_2004);
    a.msr(0x08, 0); // MSR MSP, r0
    a.svc(0); // taken with SP%8 == 4 -> ALIGN adjustment
    a.mrs(2, 0x08); // r2 = MSP after return
    a.movs_imm8(3, 0x55);
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(4, 0x99); // marker
    a.bx(14); // EXC_RETURN_THREAD_MSP: un-align back to 0x20002004
    a.label("halt");
    a.b("halt");
    conformance("exception_msp_align", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// A SVC executed while a SVC handler is already active has the same priority
// as the handler it runs inside, so it cannot preempt it: real ARMv7-M
// escalates the offending SVC to a UsageFault (folded to HardFault here). The
// stacked return address keeps the normal SVC convention (the instruction
// after the SVC), so the HardFault handler returns straight into the SVC
// handler, which then returns to Thread mode.
// ---------------------------------------------------------------------------

#[test]
fn svc_does_not_preempt_svc() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler"); // SVC vector
    a.vec(3, "hf"); // HardFault vector
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.svc(0); // enter the SVC handler
    a.movs_imm8(2, 0xAA); // reached once SVC returns
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(3, 0x33); // marker in the SVC handler
    a.svc(0); // SVC inside SVC: cannot preempt -> HardFault
    a.movs_imm8(4, 0x44); // reached once the fault is skipped
    a.bx(14); // SVC return to Thread mode
    a.label("hf");
    a.bx(14); // stacked PC already points past the SVC; just return
    a.label("halt");
    a.b("halt");
    conformance("svc_does_not_preempt_svc", &mut a, "halt", &mt);
}

#[test]
fn svc_from_unprivileged_thread() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler");
    a.mov_imm32(0, 0x1111_2222);
    a.movs_imm8(0, 1); // CONTROL.nPRIV = 1 (drop to unprivileged)
    a.msr(0x14, 0); // MSR CONTROL, r0
    a.mrs(1, 0x14); // r1 = CONTROL (should be 1)
    a.svc(0); // SVC from unprivileged thread
    a.mrs(2, 0x14); // r2 = CONTROL after return (back to 1)
    a.movs_imm8(3, 0xAA);
    a.b("halt");
    a.label("svc_handler");
    a.mrs(4, 0x14); // r4 = CONTROL in handler
    a.movs_imm8(5, 0x99);
    a.bx(14); // EXC_RETURN_THREAD
    a.label("halt");
    a.b("halt");
    conformance("svc_from_unprivileged_thread", &mut a, "halt", &mt);
}

#[test]
fn svc_masked_by_primask_escalates() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler"); // SVC (priority 0)
    a.vec(3, "hf"); // HardFault (priority -1)
    a.mov_imm32(0, 0x1111_2222);
    // PRIMASK = 1 masks every configurable-priority exception, including SVC.
    // A synchronous exception that cannot be taken is escalated to HardFault
    // (the stacked return address is the instruction after the SVC), so the
    // thread reaches the HardFault handler, not the SVC handler.
    a.movs_imm8(0, 1);
    a.msr(0x10, 0); // MSR PRIMASK, r0
    a.svc(1); // masked -> escalates to HardFault
    a.movs_imm8(2, 0xAA); // not reached (fault entry happens first)
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(5, 0x99); // marker; must NOT be reached
    a.bx(14);
    a.label("hf");
    a.movs_imm8(6, 0x66); // marker in the HardFault handler
    a.b("halt"); // stop here; both sides converge on the halt loop
    a.label("halt");
    a.b("halt");
    conformance("svc_masked_by_primask_escalates", &mut a, "halt", &mt);
}

#[test]
fn svc_locked_up_by_faultmask() {
    // FAULTMASK is stronger than PRIMASK: it raises the current priority to
    // -1, so when a masked SVC escalates to HardFault (also priority -1) the
    // escalation itself cannot activate. Both simulators lock the core up
    // (QEMU: "Lockup: can't escalate 3 to HardFault (current priority -1)").
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler"); // SVC vector (must not be reached)
    a.vec(3, "hf"); // HardFault vector (must not be reached)
    a.mov_imm32(0, 0x1111_2222);
    a.movs_imm8(0, 1);
    a.msr(0x13, 0); // MSR FAULTMASK, r0
    a.svc(1); // masked -> escalates to HardFault -> HardFault masked -> lockup
    a.movs_imm8(2, 0xAA); // never reached
    a.b("halt");
    a.label("svc_handler");
    a.movs_imm8(5, 0x99); // marker; must NOT be reached
    a.bx(14);
    a.label("hf");
    a.movs_imm8(6, 0x66); // marker; must NOT be reached
    a.b("halt");
    a.label("halt");
    a.b("halt");
    conformance_halt("svc_locked_up_by_faultmask", &mut a, "halt", &mt);
}

#[test]
fn unaligned() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // ARMv7-M requires LDM/STM/LDRD/STRD to be word-aligned. An unaligned
    // base on any of them is an alignment UsageFault (escalated to HardFault
    // on this machine). Each case sets r12 = skip-bytes before faulting; the
    // shared handler reads r12 and the stacked PC off the frame (r12 is one
    // of the eight stacked registers), patches the PC past the faulting
    // instruction, and returns.
    a.vec(3, "hf");
    a.mov_imm32(4, 0x2000_0002); // misaligned base
    a.mov_imm32(0, 0x1122_3344);
    a.mov_imm32(1, 0xAABB_CCDD);

    // 16-bit STMIA r4! at a misaligned base.
    a.mov_imm32(12, 2);
    a.stmia(4, 0b0000_0011); // {r0,r1} @ 0x20000002 -> fault
    a.mov_imm32(12, 4);
    a.stm_wide(4, false, false, 0b0000_0011); // T32 STM @ misaligned -> fault
    a.mov_imm32(12, 4);
    a.strd(0, 1, 4, 4); // STRD r0,r1, [r4] @ misaligned -> fault
    a.mov_imm32(12, 2);
    a.ldmia(4, 0b0000_0011); // LDMIA r4!, {r0,r1} @ misaligned -> fault
    a.mov_imm32(12, 4);
    a.ldm_wide(4, false, false, 0b0000_0011); // T32 LDM @ misaligned -> fault
    a.mov_imm32(12, 4);
    a.ldrd(0, 1, 4, 4); // LDRD r0,r1, [r4] @ misaligned -> fault

    // Aligned controls must NOT fault.
    a.mov_imm32(5, 0x2000_0020);
    a.stmia(5, 0b0000_0011); // ok
    a.ldmia(5, 0b0000_0011); // ok
    a.strd(0, 1, 5, 4); // ok
    a.ldrd(0, 1, 5, 4); // ok
    a.addw(12, 5, 8); // r12 = past the aligned transfers, a non-trivial marker

    a.label("halt");
    a.b("halt");

    // Shared handler: patch the stacked PC past the faulting instruction.
    a.label("hf");
    a.mov_reg(6, 13); // r6 = frame base
    a.ldr_wide(7, 6, 24); // r7 = stacked PC (faulting instruction)
    a.ldr_wide(8, 6, 16); // r8 = stacked r12 (skip bytes)
    a.adds_wide(7, 7, 8); // r7 += r12
    a.str_imm(7, 6, 24); // patch stacked PC
    a.bx(14);
    conformance("unaligned", &mut a, "halt", &mt);
}

#[test]
fn data_faults() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Reads and writes to an unmapped address raise a (Hard)Fault; reads and
    // writes inside the mapped flash/SRAM regions must not. Each faulting
    // case runs into the shared handler, which skips the 2-byte access using
    // the stacked r12 skip hint (r12 is one of the eight stacked registers).
    a.vec(3, "hf");
    a.mov_imm32(0, 0x1122_3344);

    // Reserved read -> fault.
    a.mov_imm32(5, 0x9000_0000);
    a.mov_imm32(12, 2);
    a.ldr_imm(1, 5, 0);
    // Reserved write -> fault.
    a.mov_imm32(12, 2);
    a.str_imm(0, 5, 0);
    // Flash write is ignored, not a fault (matching mps2-an385).
    a.mov_imm32(5, 0x0000_0400);
    a.mov_imm32(12, 2);
    a.str_imm(0, 5, 0);
    // Mapping-covered accesses must not fault: SRAM write+read.
    a.mov_imm32(5, 0x2000_0400);
    a.mov_imm32(12, 2);
    a.str_imm(0, 5, 0);
    a.ldr_imm(1, 5, 0); // r1 = 0x11223344
    a.movs_imm8(2, 0x99);

    a.label("halt");
    a.b("halt");

    a.label("hf");
    a.mov_reg(6, 13); // r6 = frame base
    a.ldr_wide(7, 6, 24); // r7 = stacked PC (faulting access)
    a.ldr_wide(8, 6, 16); // r8 = stacked r12 (skip bytes)
    a.adds_wide(7, 7, 8); // r7 += r12
    a.str_imm(7, 6, 24); // patch stacked PC
    a.bx(14);
    conformance("data_faults", &mut a, "halt", &mt);
}

/// Like [`conformance`], but for programs that are *supposed* to halt before
/// ever reaching the landing loop: a double fault (HardFault taken while a
/// HardFault handler is active) locks the core up. Both implementations must
/// stop executing at the same instruction with identical state, and neither
/// may reach `halt`.
fn conformance_halt(name: &str, a: &mut Asm, halt: &str, mt: &QemuMachine) {
    let img: Image = a.clone().finish(SP).expect("assemble");
    let stop = img
        .addr(halt)
        .unwrap_or_else(|| panic!("{name}: no label {halt:?}"));
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        !reached,
        "{name}: vioarm reached halt @ {stop:#x} instead of locking up (last {})",
        snapshot_like(&vio)
    );
    // The lockup ends QEMU's instruction stream, so the target PC never shows
    // up in the log; the run returns once the trace goes quiet past the boot
    // preamble. It must at least have produced the reset state and reached
    // the (failed) handler entry.
    let qemu = qemu_run_retry(mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty(),
        "{name}: qemu produced no state (failed to boot under load)"
    );
    assert!(
        !qemu.last().map(|s| s.r[15] & !1 == stop).unwrap_or(false),
        "{name}: qemu reached halt @ {stop:#x} instead of locking up (got {} states)",
        qemu.len()
    );
    compare_traces(name, &vio, &qemu).expect("traces agree");
}

// ---------------------------------------------------------------------------
// A fault taken while a HardFault handler is already active is a double fault:
// ARMv7-M locks the core up (no exception can preempt HardFault). The first
// UDEF escalates to the handler; the second UDEF inside the handler must stop
// both simulators at the identical stacked state, never reaching the landing
// loop.
// ---------------------------------------------------------------------------

#[test]
fn double_fault() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf"); // HardFault vector
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.raw(0xDE00); // UDEF -> HardFault
    a.movs_imm8(2, 0xAA); // never reached
    a.b("halt");
    a.label("hf");
    a.raw(0xDE00); // UDEF inside the handler -> double fault -> lockup
    a.bx(14); // would return, but never executes
    a.label("halt");
    a.b("halt");
    conformance_halt("double_fault", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Unlike a double fault, a fault taken inside a *non-fault* handler
// legitimately preempts it: HardFault has fixed priority -1, strictly
// outranking SVC (and PendSV/SysTick/IRQ). The fault handler runs, patches the
// SVC frame, and the two stacked frames unwind in order.
// ---------------------------------------------------------------------------

#[test]
fn hardfault_preempts_svc() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc"); // SVC vector
    a.vec(3, "hf"); // HardFault vector
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.svc(0); // enter the SVC handler
    a.movs_imm8(2, 0xAA); // reached once SVC returns
    a.b("halt");
    a.label("svc");
    a.movs_imm8(3, 0x33); // marker in the SVC handler
    a.raw(0xDE00); // UDEF inside SVC -> HardFault preempts
    a.bx(14); // SVC return to Thread mode
    a.label("hf");
    a.mov_reg(6, 13); // frame base
    a.ldr_wide(7, 6, 24); // stacked SVC-handler PC (faulting UDEF)
    a.adds_imm8(7, 2); // skip the UDEF
    a.str_imm(7, 6, 24);
    a.bx(14); // return to the SVC handler (EXC_RETURN_HANDLER)
    a.label("halt");
    a.b("halt");
    conformance("hardfault_preempts_svc", &mut a, "halt", &mt);
}
// ---------------------------------------------------------------------------
// Faults: an undefined instruction (0xDE00, permanently-undefined) escalates
// through UsageFault to HardFault. The handler reads the stacked frame, skips
// the bad opcode, and returns via `bx lr`; the stacked xPSR, LR, and data
// registers must match QEMU's frame layout exactly.
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
// PRIMASK and BASEPRI are not part of the stacked frame: ARMv7-M preserves
// them across exception entry and return (the handler inherits the mask state
// of the preempted context and it is not restored from the 8-word frame, so
// it survives both the UDEF escalation to HardFault and the bx lr return).
// FAULTMASK must stay clear here (UDEF is escalated, not a nested fault).
// ---------------------------------------------------------------------------

#[test]
fn exception_masks_persist() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf"); // HardFault vector

    a.movs_imm8(0, 1);
    a.msr(0x10, 0); // PRIMASK = 1
    a.movs_imm8(0, 0x80);
    a.msr(0x11, 0); // BASEPRI = 0x80
    a.raw(0xDE00); // UDEF -> HardFault (fixed priority, masks don't block it)
                   // resumed here after the handler patches PAST the UDEF:
    a.mrs(4, 0x10); // r4 = PRIMASK after return (still 1)
    a.mrs(5, 0x11); // r5 = BASEPRI after return (still 0x80)
    a.b("halt");

    a.label("hf");
    // Handler: read the masks we set, then patch the stacked PC past the
    // UDEF and return to Thread mode.
    a.mrs(1, 0x10); // r1 = PRIMASK in handler (still 1)
    a.mrs(2, 0x11); // r2 = BASEPRI in handler (still 0x80)
    a.mrs(3, 0x13); // r3 = FAULTMASK in handler (0)
    a.mov_reg(6, 13); // frame base
    a.ldr_wide(7, 6, 24); // stacked PC (the UDEF itself)
    a.adds_imm8(7, 2); // skip the UDEF
    a.str_imm(7, 6, 24);
    a.bx(14); // return to Thread mode, past the UDEF

    a.label("halt");
    a.b("halt");
    conformance("exception_masks_persist", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// IT blocks (ARMv7-M conditional execution). The conformance harness now
// folds ITSTATE into the compared xPSR bits [26:25]/[15:10], so every
// executed and every suppressed instruction must leave the same ITSTATE in
// both simulators, and the skipped instructions must not touch any register.
//
// The `mask` field of `IT<c>` is a count/shape register, not a plain count:
// each mask bit, read from bit 3 down, encodes one following instruction as
// "then" (bit set) or "else" (bit clear), and the count of conditional
// instructions is the number of bits from the first set bit to the lowest
// (the trailing-zeros ITSTATE advance drops one bit per instruction). Masks
// below follow that encoding with the T/E shapes noted per block.
// ---------------------------------------------------------------------------

#[test]
fn it_blocks() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a); // N=0 C=0 V=0 Z=1

    // Flags: Z=0 (0x5F cmp 0xA0 is negative) so EQ fails, NE/GE/GT pass.
    a.movs_imm8(0, 0x5F);
    a.cmp_imm8(0, 0xA0);

    // IT NE, one instruction (mask 0b1000), condition true -> executes.
    a.it(1, 0b1000);
    a.movs_imm8(1, 0x11);
    // IT EQ, one instruction, condition false -> suppressed.
    a.it(0, 0b1000);
    a.movs_imm8(2, 0x22);
    // mask 0b0100: two instructions with shape T,E (then NE, else EQ).
    // NE true -> r3 executes; EQ false -> r4 suppressed.
    a.it(1, 0b0100);
    a.movs_imm8(3, 0x33);
    a.movs_imm8(4, 0x44);
    // mask 0b1001: four instructions, shape T,T,E,E. Only r5, r6 write.
    a.it(1, 0b1001);
    a.movs_imm8(5, 0x55);
    a.movs_imm8(6, 0x66);
    a.movs_imm8(7, 0x77);
    a.movs_imm8(8, 0x88);
    // mask 0b0001: four instructions, shape T,E,E,E. Only r9 writes.
    a.it(1, 0b0001);
    a.movs_imm8(9, 0x99);
    a.movs_imm8(10, 0xAA);
    a.movs_imm8(11, 0xBB);
    a.movs_imm8(12, 0xCC);
    // A single wide (32-bit) instruction inside IT, condition true.
    a.it(1, 0b1000);
    a.movw(10, 0x2222); // executes (NE true), one slot
                        // A suppressed (not-taken) conditional branch inside IT. clang emits the
                        // canonical `it ne; bne` for conditional branches, and with the condition
                        // false (Z=1 -> NE fails) the branch is suppressed and ITSTATE advances
                        // normally. NOTE: a *taken* branch inside an IT block is intentionally
                        // absent here -- QEMU 10.2 (reference model) HardFault-lockups on it even
                        // for clang's canonical sequence, so it is not differentially testable.
    a.movs_imm8(0, 0); // Z=1 -> NE false
    a.it(1, 0b1000); // [T], single, cond NE
    a.bc(1, "it_nt"); // bne: suppressed (NE false), falls through
    a.movs_imm8(2, 0x22); // unconditional fall-through
    a.label("it_nt");
    a.movs_imm8(3, 0x33); // reached via fall-through only
    a.b("halt");
    a.label("halt");
    a.b("halt");
    conformance("it_blocks", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// A BKPT with debug disabled is a fault, not a debug halt: it escalates to
// HardFault, stacking the BKPT's own address (so a handler can skip it) and
// the pre-fault context. Both the frame layout and the retry-through-the-
// handler path must match QEMU's.
// ---------------------------------------------------------------------------

#[test]
fn bkpt() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.vec(13, "dbg"); // DebugMon vector (never taken: debug is disabled)
    a.vec(3, "hf"); // HardFault vector
    a.mov_imm32(0, 0x1111_2222);
    a.mov_imm32(1, 0x3333_4444);
    a.raw(0xBE00); // bkpt #0 -> HardFault
    a.movs_imm8(2, 0xAA); // reached once the handler skips the BKPT
    a.b("halt");
    a.label("dbg");
    a.movs_imm8(5, 0x55); // marker; must NOT be reached
    a.bx(14);
    a.label("hf");
    // The stacked PC is the BKPT instruction itself: read it, skip past the
    // BKPT, patch the frame, and return like the `faults` handler does.
    a.mov_reg(6, 13); // frame base
    a.ldr_wide(7, 6, 24); // stacked PC (the BKPT)
    a.adds_imm8(7, 2); // skip the BKPT
    a.str_imm(7, 6, 24);
    a.bx(14); // return to Thread mode, past the BKPT
    a.label("halt");
    a.b("halt");
    conformance("bkpt", &mut a, "halt", &mt);
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
// ARMv7E-M only and QEMU's mps2-an385 (cortex-m3) faults on it — a divergence
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
        a.label(format!("tbb{i}_in")); // entry: re-load the index
        a.movs_imm8(0, i);
        a.align4();
        a.label(format!("tbb{i}"));
        a.tb(15, 0);
        a.dc_u32(0); // patched below
        let next = if i == 3 {
            "tbh0_in"
        } else {
            &format!("tbb{}_in", i + 1)
        };
        for (k, v) in [(0u8, 0x10u8), (1, 0x20), (2, 0x30), (3, 0x40)] {
            a.label(format!("tbb{i}_c{k}"));
            a.movs_imm8(1, v);
            a.b(next);
        }
    }

    // ---- PC-relative `TBH` (halfword table): index in r0 ----
    for i in 0..4u8 {
        a.label(format!("tbh{i}_in"));
        a.movs_imm8(0, i);
        a.align4();
        a.label(format!("tbh{i}"));
        a.tbh(15, 0);
        a.dc_u32(0); // patched below (four halfword entries)
        a.dc_u32(0);
        let next = if i == 3 {
            "rtbb_in"
        } else {
            &format!("tbh{}_in", i + 1)
        };
        for (k, v) in [(0u8, 0xA0u8), (1, 0xB0), (2, 0xC0), (3, 0xD0)] {
            a.label(format!("tbh{i}_c{k}"));
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
        a.label(format!("rtbb_c{k}"));
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
        a.label(format!("rtbh_c{k}"));
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
            word |= entry << (8 * u32::from(k));
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
    // negative pre/post-indexed: same forms, but the offset is subtracted.
    // r4 is 0x2000_2020 here after the writebacks above (32+32+8+8).
    a.mov_imm32(4, 0x2000_2400); // fresh base
    a.strd_pre_neg(0, 1, 4, 32); // [r4-32], r4 -= 32
    a.mov_imm32(4, 0x2000_2400);
    a.ldrd_pre_neg(5, 6, 4, 32); // [r4-32], r4 -= 32
    a.mov_imm32(4, 0x2000_2420);
    a.strd_post_neg(2, 3, 4, 16); // [r4], r4 -= 16
    a.mov_imm32(4, 0x2000_2420);
    a.ldrd_post_neg(9, 10, 4, 16); // [r4], r4 -= 16
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

// ---------------------------------------------------------------------------
// VFP (FPv4-SP): the full single-precision data-processing and load/store
// repertoire, diffed against the Cortex-M4's FPU. The program first enables
// the FPU via `CPACR` (QEMU gates on it; VioARM records the write), loads the
// operand register bank, and then runs the arithmetic, conversion, comparison,
// immediate, and memory-move instructions. The harness now diffs `S0`-`S31`
// and the modeled FPSCR bits after every step.
// ---------------------------------------------------------------------------

#[test]
fn vfp_m4() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    // operand bit patterns in GPRs (r0/r1 are clobbered by enable_fpu)
    a.mov_imm32(2, 0x3f80_0000); //  1.0
    a.mov_imm32(3, 0x4000_0000); //  2.0
    a.mov_imm32(4, 0x4040_0000); //  3.0
    a.mov_imm32(5, 0x4120_0000); // 10.0
    a.mov_imm32(6, 0x4020_0000); //  2.5
    a.mov_imm32(7, 0xC040_0000); // -3.0
    a.mov_imm32(8, 0x4080_0000); //  4.0
    a.mov_imm32(9, 0xBF80_0000); // -1.0
    a.mov_imm32(10, 0x3FE0_0000); //  1.75
    a.mov_imm32(11, 0x3FC0_0000); //  1.5
    a.mov_imm32(12, 0xFFFF_FFFD); // -3 (as an int)

    // load the S bank
    a.vmov_s_gpr(0, 2); // s0  =  1.0
    a.vmov_s_gpr(1, 3); // s1  =  2.0
    a.vmov_s_gpr(2, 4); // s2  =  3.0
    a.vmov_s_gpr(3, 5); // s3  = 10.0
    a.vmov_s_gpr(4, 6); // s4  =  2.5
    a.vmov_s_gpr(5, 7); // s5  = -3.0
    a.vmov_s_gpr(6, 8); // s6  =  4.0
    a.vmov_s_gpr(7, 9); // s7  = -1.0
    a.vmov_s_gpr(8, 10); // s8  =  1.75
    a.vmov_s_gpr(9, 11); // s9  =  1.5

    // three-operand arithmetic
    a.vadd_f32(10, 0, 1); // s10 = 1.0 + 2.0
    a.vsub_f32(11, 3, 1); // s11 = 10.0 - 2.0
    a.vmul_f32(12, 0, 1); // s12 = 1.0 * 2.0
    a.vdiv_f32(13, 3, 1); // s13 = 10.0 / 2.0
    a.vmla_f32(14, 0, 1); // s14 = 4.0 + 1.0*2.0
    a.vmls_f32(15, 0, 1); // s15 = 10.0 - 1.0*2.0
    a.vfma_f32(16, 0, 1); // s16 = 3.0 + 1.0*2.0
    a.vfms_f32(17, 0, 1); // s17 = 10.0 - 1.0*2.0
    a.vfnma_f32(18, 0, 1); // s18 = -(4.0) + 1.0*2.0
    a.vfnms_f32(19, 0, 1); // s19 = -(4.0) - 1.0*2.0

    // two-operand
    a.vabs_f32(20, 5); // s20 = |−3.0|
    a.vneg_f32(21, 0); // s21 = −1.0
    a.vsqrt_f32(22, 6); // s22 = √4.0

    // immediate, register, and 64-bit moves
    a.vmov_f32_imm(23, 0x70); // s23 = 1.0
    a.vmov_f32_imm(24, 0xF0); // s24 = -1.0
    a.vmov_reg(25, 5); // s25 = s5 (VMOV register)
    a.vmov_s_pair_from_gpr(26, 10, 11); // s26 = 1.75, s27 = 1.5

    // conversions
    a.vcvt_f32_s32(28, 0); // s28 = (float)-3   (s0 holds int -3)
    a.vcvt_s32_f32(29, 8); // s29 = (int)1.75
    a.vcvt_u32_f32(30, 9); // s30 = (uint)1.5
    a.vcvtr_s32_f32(31, 8); // s31 = (int)1.75, toward zero

    // comparisons write FPSCR NZCV, which the harness diffs
    a.vcmp_f32(0, 1); // 1.0 vs 2.0 -> less
    a.vcmpz_f32(0); // 1.0 vs 0 -> greater
    a.vcmpe_f32(0, 0); // equal

    // load/store and stack moves
    a.mov_imm32(0, 0x2000_0000);
    a.vstr_s(10, 0, 0);
    a.vldr_s(11, 0, 0);
    a.vpush(0, 3);
    a.vpop(0, 3);

    // NaN compare (quiet) and NaN conversion: unordered flags / IOC
    a.mov_imm32(0, 0x7FC0_0000);
    a.vmov_s_gpr(1, 0);
    a.vcmp_f32(1, 1); // QNaN vs QNaN -> unordered, no IOC
    a.vcvt_s32_f32(2, 1); // QNaN -> 0, sets IOC

    // read FPSCR into a GPR to make the full register diffable
    a.vmrs(0, 1); // vmrs r0, fpscr
    a.movs_imm8(0, 0); // clear the low 8 flags for a stable end state

    a.label("halt");
    a.b("halt");
    conformance("vfp_m4", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// FP context switching / lazy stacking (FPCCR ASPEN/LSPEN, CONTROL.FPCA).
//
// These tests drive the exception machinery against the Cortex-M4 oracle:
// thread-mode code uses the FPU (setting CONTROL.FPCA), then takes an SVC, and
// the handler either does or does not touch the FPU. Depending on LSPEN and on
// whether the handler uses the FPU, the extended exception frame is either
// lazily reserved (FPCAR/LSPACT), eagerly stored, or abandoned.
// ---------------------------------------------------------------------------

/// Minimal probe: an `SVC` through the non-FP exception path. Both simulators
/// must agree on the stacked frame, `EXC_RETURN`, and the return.
#[test]
fn fp_ctx_svc_basic() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.svc(1);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.adds_imm8(0, 1);
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("fp_ctx_svc_basic", &mut a, "halt", &mt);
}

/// Scenario A: lazy stacking (LSPEN=1, the reset default). The handler uses
/// the FPU, so on entry the FP frame is lazily reserved (FPCAR/LSPACT) and the
/// thread's S0-S15/FPSCR are written out on the handler's first FP instruction;
/// on return they are restored from the frame. S16-S31 are not in the frame and
/// the handler's writes persist.
#[test]
fn fp_ctx_lazy_handler_uses_fp() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    // Thread-mode FP context: FPCA set by the first FP instruction; FPSCR gets
    // an inexact flag from 2/3 so the frame stores a nonzero FPSCR.
    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (first FP insn -> FPCA, FPSCR reset)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (inexact -> IXC set)
    a.mov_imm32(0, 0x4120_0000); // 10.0
    a.vmov_s_gpr(4, 0);
    a.vmov_f32_imm(16, 0x70); // s16 = 1.0 (not stacked)
    a.vmov_f32_imm(17, 0x70); // s17 = 1.0 (not stacked)
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1);
    a.label("halt");
    a.b("halt");

    // Handler uses the FPU: lazy preserve writes the thread's S0-S15 + FPSCR
    // to the reserved frame, then FPCA is set and FPSCR reset for the handler.
    a.label("svc_handler");
    a.vadd_f32(6, 0, 1); // first handler FP insn: preserve + context create
    a.vmul_f32(7, 2, 3);
    a.vsub_f32(8, 4, 6);
    a.vmov_f32_imm(16, 0xF0); // s16 = -1.0, persists after return
    a.vmov_f32_imm(17, 0xF0); // s17 = -1.0, persists after return
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("fp_ctx_lazy_handler_uses_fp", &mut a, "halt", &mt);
}

/// Scenario B: non-lazy stacking (LSPEN=0). On entry the thread's S0-S15 +
/// FPSCR are eagerly stored and the registers zeroed; the handler's first FP
/// instruction creates a fresh context from the zeroed bank. The values are
/// restored on return.
#[test]
fn fp_ctx_nolazy_handler_uses_fp() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmov_f32_imm(4, 0x70); // s4 = 1.0
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    // Turn lazy stacking off (ASPEN stays on).
    a.mov_imm32(0, 0xE000_EF34);
    a.mov_imm32(1, 0x8780_0004); // LSPEN=0
    a.str_imm(1, 0, 0);

    a.svc(1);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.vadd_f32(6, 0, 1); // first handler FP insn from the zeroed bank
    a.vmul_f32(7, 2, 3);
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("fp_ctx_nolazy_handler_uses_fp", &mut a, "halt", &mt);
}

/// Scenario C: lazy stacking, but the handler never touches the FPU. The frame
/// is reserved lazily (FPCAR/LSPACT) and abandoned on return without any FPU
/// save/restore, so the thread's S0-S15/FPSCR survive untouched.
#[test]
fn fp_ctx_lazy_handler_no_fp() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmov_f32_imm(4, 0x70); // s4 = 1.0
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1);
    a.label("halt");
    a.b("halt");

    // Handler does NOT use the FPU: LSPACT is cleared on return and nothing
    // else happens to the FP registers.
    a.label("svc_handler");
    a.adds_imm8(0, 1);
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("fp_ctx_lazy_handler_no_fp", &mut a, "halt", &mt);
}

/// Scenario D: thread mode on the process stack. `CONTROL.SPSEL` switches the
/// active SP to PSP, so the extended frame is pushed there and the return uses
/// `EXC_RETURN_THREAD_PSP_FP` (0xffffffed), handing the thread back its PSP
/// with both stack banks intact.
#[test]
fn fp_ctx_svc_on_process_stack() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.movs_imm8(0, 2); // CONTROL.SPSEL = 1
    a.msr(0x14, 0);
    a.mov_imm32(0, 0x2000_3ff0); // PSP value
    a.msr(0x09, 0);
    a.mrs(2, 0x09); // r2 = PSP
    a.mrs(3, 0x08); // r3 = MSP

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1); // extended frame pushed on PSP
    a.mrs(4, 0x09); // r4 = PSP after return (back to 0x2000_3ff0)
    a.mrs(5, 0x08); // r5 = MSP after return
    a.vmov_gpr_s(1, 0); // r1 = s0 (restored thread value)
    a.movs_imm8(6, 0xAA);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.vadd_f32(6, 0, 1); // first handler FP insn: lazy preserve + context create
    a.vmul_f32(7, 2, 3);
    a.bx(14); // EXC_RETURN_THREAD_PSP_FP

    a.vec(11, "svc_handler");
    conformance("fp_ctx_svc_on_process_stack", &mut a, "halt", &mt);
}

/// Scenario E: a nested exception while the handler owns the FP context. The
/// SVC handler uses the FPU (setting `CONTROL.FPCA`), then takes a second SVC
/// which cannot preempt and escalates to `HardFault`; the `HardFault` handler
/// also uses the FPU, so both frames are extended and the return chain
/// exercises `EXC_RETURN_HANDLER_FP` (0xffffffe1) as well as a double lazy
/// preserve/restore. `S16` is not in the extended frame, so the last handler's
/// write to it persists all the way back to the thread.
#[test]
fn fp_ctx_nested_handler_extended_return() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmov_f32_imm(16, 0x70); // s16 = 1.0 (not stacked)
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1); // -> SVC handler
    a.vmov_gpr_s(1, 0); // r1 = s0 (restored thread 1.0)
    a.vmov_gpr_s(1, 16); // r1 = s16 (1.0: the HardFault handler's write won)
    a.movs_imm8(2, 0xAA);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.vadd_f32(6, 0, 1); // first handler FP insn: preserve + context create
    a.vmul_f32(7, 2, 3);
    a.vmov_f32_imm(16, 0xF0); // s16 = -1.0 (SVC handler's value)
    a.svc(2); // SVC inside the handler: cannot preempt -> HardFault
    a.movs_imm8(4, 0x44); // reached once the fault returns
    a.bx(14); // EXC_RETURN_THREAD_MSP_FP back to the thread

    a.label("hf");
    a.vadd_f32(8, 0, 1); // first HF FP insn: preserve + context create
    a.vmul_f32(9, 6, 7);
    a.vmov_f32_imm(16, 0x70); // s16 = 1.0 (persists past both returns)
    a.bx(14); // EXC_RETURN_HANDLER_FP back into the SVC handler

    a.vec(11, "svc_handler");
    a.vec(3, "hf");
    conformance("fp_ctx_nested_handler_extended_return", &mut a, "halt", &mt);
}

/// Scenario F: the thread's SP is 8-misaligned (`SP % 8 == 4`) when the FP
/// exception is taken. Entry decrements SP by 4 and sets `xPSR.ALIGN`; the
/// extended frame is pushed below the align slot, and the return re-aligns SP
/// to the thread's original value.
#[test]
fn fp_ctx_sprealign_extended_frame() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0x2000_2004); // SP % 8 == 4
    a.msr(0x08, 0); // MSR MSP, r0

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1); // taken with SP%8 == 4 -> ALIGN adjustment, extended frame
    a.mrs(2, 0x08); // r2 = MSP after return (0x20002004 again)
    a.vmov_gpr_s(1, 0); // r1 = s0 (restored thread 1.0)
    a.movs_imm8(3, 0x55);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.vadd_f32(6, 0, 1); // first handler FP insn: lazy preserve + context create
    a.vmul_f32(7, 2, 3);
    a.bx(14); // EXC_RETURN_THREAD_MSP_FP: re-align SP to 0x20002004

    a.vec(11, "svc_handler");
    conformance("fp_ctx_sprealign_extended_frame", &mut a, "halt", &mt);
}

/// Scenario G: the handler's first FP instruction targets `S16` (outside the
/// stacked `S0`-`S15` block). The lazy preserve and context creation still run
/// on the first FP instruction regardless of its destination; `S16` is never
/// stacked, so the handler's writes persist after return while `S0`-`S15` and
/// FPSCR come back from the frame.
#[test]
fn fp_ctx_handler_uses_high_registers() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0x3F80_0000); //  1.0
    a.vmov_s_gpr(0, 0);
    a.mov_imm32(0, 0x4000_0000); //  2.0
    a.vmov_s_gpr(1, 0);
    a.vadd_f32(2, 0, 1); // s2 = 3.0   (FPCA)
    a.vdiv_f32(3, 1, 2); // s3 = 2/3   (IXC)
    a.vmov_f32_imm(16, 0x70); // s16 = 1.0 (not stacked)
    a.vmrs(0, 1); // r0 = FPSCR (IXC)

    a.svc(1);
    a.vmov_gpr_s(1, 0); // r1 = s0 (restored thread 1.0)
    a.vmov_gpr_s(1, 16); // r1 = s16 (5.0: the handler's write persists)
    a.movs_imm8(2, 0xAA);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.vmov_f32_imm(16, 0x78); // s16 = 5.0 -- first FP insn targets a high reg
    a.vmov_f32_imm(17, 0x40); // s17 = 3.0
    a.vadd_f32(18, 16, 17); // s18 = 8.0
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("fp_ctx_handler_uses_high_registers", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// FPDSCR supplies the control bits a fresh FP context's FPSCR is loaded from.
// Firmware writes round-towards-zero (`RMODE = 0b11`) to `0xE000_EF3C`; the
// first FP instruction then creates a context whose FPSCR carries that mode,
// and a `VCVT.S32.F32` rounds 1.75 to 1 instead of the round-to-nearest 2.
// ---------------------------------------------------------------------------

#[test]
fn fp_ctx_fpdscr_supplies_new_context() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();

    a.mov_imm32(0, 0xE000_EF3C);
    a.mov_imm32(1, 0x00C0_0000); // FPDSCR: RMODE = 0b11 (round towards zero)
    a.str_imm(1, 0, 0);
    a.mov_imm32(1, 0x00C0_0000); // r1 = the written FPDSCR bits

    a.mov_imm32(0, 0x3FE0_0000); // 1.75
    a.vmov_s_gpr(1, 0); // first FP insn: create the context, FPSCR = FPDSCR
    a.vcvt_s32_f32(2, 1); // 1.75 towards zero -> 1 (not 2)
    a.vmrs(0, 1); // r0 = FPSCR (RMODE = 0b11)
    a.movs_imm8(3, 0xAA);
    a.label("halt");
    a.b("halt");
    conformance("fp_ctx_fpdscr_supplies_new_context", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// SCB system-control windows: AIRCR (VECTKEY-protected PRIGROUP store and
// VECTKEYSTAT read), SCR, and CCR are diffed against the M4 oracle through
// register reads.
// ---------------------------------------------------------------------------

#[test]
fn scb_aircr_scr_ccr_roundtrip() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // AIRCR: an unkeyed write is ignored (read stays VECTKEYSTAT with no
    // PRIGROUP); a keyed write stores PRIGROUP and the read reports it.
    a.mov_imm32(0, 0xE000_ED0C);
    a.mov_imm32(1, 0x0000_0700); // no VECTKEY: ignored
    a.str_imm(1, 0, 0);
    a.ldr_imm(2, 0, 0); // r2 = 0xFA05_0000
    a.mov_imm32(1, 0x05FA_0700); // VECTKEY | PRIGROUP = 7
    a.str_imm(1, 0, 0);
    a.ldr_imm(3, 0, 0); // r3 = 0xFA05_0700

    // SCR: SEVONPEND|SLEEPONEXIT store and read back; the deep-sleep bits
    // are RAZ/WI in both models.
    a.mov_imm32(0, 0xE000_ED10);
    a.movs_imm8(1, 0x16);
    a.str_imm(1, 0, 0);
    a.ldr_imm(1, 0, 0); // r1 = 0x12

    // CCR: the reset value carries STKALIGN; a STKALIGN|DIV_0_TRP write reads
    // back exactly what was written.
    a.mov_imm32(0, 0xE000_ED14);
    a.ldr_imm(2, 0, 0); // r2 = 0x200 (reset)
    a.mov_imm32(1, 0x0000_0210);
    a.str_imm(1, 0, 0);
    a.ldr_imm(1, 0, 0); // r1 = 0x210

    a.movs_imm8(4, 0xAA);
    a.label("halt");
    a.b("halt");
    conformance("scb_aircr_scr_ccr_roundtrip", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// CCR.STKALIGN: with the bit cleared, exception entry skips the 8-byte
// stack alignment. The main SP is parked on a 4-aligned-but-not-8 address, an
// SVC is taken, and the handler reads MSP, so any divergence in the frame
// base lands directly in the diffed registers.
// ---------------------------------------------------------------------------

#[test]
fn ccr_stkalign_cleared_skips_align() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Clear STKALIGN (CCR bit 9); DIV_0_TRP is irrelevant here.
    a.mov_imm32(0, 0xE000_ED14);
    a.mov_imm32(1, 0x0000_0010);
    a.str_imm(1, 0, 0);

    // Park MSP on a 4-aligned-but-not-8 address, then take SVC.
    a.mov_imm32(0, 0x2000_1004);
    a.msr(0x08, 0); // MSR MSP, r0
    a.svc(1);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.mrs(0, 0x08); // r0 = MSP after entry
    a.movs_imm8(1, 0xAA);
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("ccr_stkalign_cleared_skips_align", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// CCR.DIV_0_TRP: a zero divisor traps to a UsageFault (folded into HardFault
// here) instead of yielding zero. Both simulators must take the HardFault
// vector, which branches to the same halt the untrapped path would also
// reach, so whichever way the oracle behaves the trace either agrees or the
// exception entry/return state diverges.
// ---------------------------------------------------------------------------

#[test]
fn ccr_div0_trp_hardfaults_zero_divide() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(0, 0xE000_ED14);
    a.mov_imm32(1, 0x0000_0010); // DIV_0_TRP
    a.str_imm(1, 0, 0);

    a.mov_imm32(0, 100);
    a.movs_imm8(1, 0); // divisor zero
    a.udiv(2, 0, 1);
    a.label("halt");
    a.b("halt");

    a.label("hf_handler");
    a.b("halt");

    a.vec(3, "hf_handler");
    conformance("ccr_div0_trp_hardfaults_zero_divide", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// SHCSR: the fault-enable bits store and read back, and a written
// `SVCALLPENDED` makes the machine take the SVC vector like a pended SVC.
// (The read-only `*ACT` bits are not exercised here: QEMU lets firmware set
// them through its NVIC state, while VioARM reads them as zero; the model's
// behavior is covered by the SCB unit tests.)
// ---------------------------------------------------------------------------

#[test]
fn scb_shcsr_svcallpend_takes_svc() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // SHCSR = USGFAULTENA|BUSFAULTENA|MEMFAULTENA (0x0007_0000): reads back.
    a.mov_imm32(0, 0xE000_ED24);
    a.mov_imm32(1, 0x0007_0000);
    a.str_imm(1, 0, 0);
    a.ldr_imm(2, 0, 0); // r2 = 0x0007_0000
                        // Pend an SVC: the machine must take the SVC vector before the next step.
    a.mov_imm32(1, 0x0007_8000);
    a.str_imm(1, 0, 0);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.movs_imm8(4, 0x77);
    a.bx(14);

    a.vec(11, "svc_handler");
    conformance("scb_shcsr_svcallpend_takes_svc", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// VTOR: the vector table base relocates. The firmware points `VTOR` at a
// 128-byte-aligned RAM address, writes the SVC entry into the relocated table
// (patched with the handler's runtime address), and takes SVC. The base table
// at address zero deliberately points the SVC entry at an infinite loop, so
// the test only reaches `halt` if the relocated table is actually used.
// ---------------------------------------------------------------------------

#[test]
fn scb_vtor_relocates_vector_table() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // VTOR = 0x2000_0080 (128-byte aligned RAM).
    a.mov_imm32(0, 0xE000_ED08);
    a.mov_imm32(1, 0x2000_0080);
    a.str_imm(1, 0, 0);
    a.ldr_imm(2, 0, 0); // r2 = 0x2000_0080

    // Store the SVC handler pointer at the relocated table's SVC entry
    // (0x2000_0080 + 11*4 = 0x2000_00AC). `svc_w` is patched below with the
    // handler's resolved address once labels exist.
    a.label("svc_w");
    a.movw(1, 0);
    a.mov_imm32(0, 0x2000_00AC);
    a.str_imm(1, 0, 0);

    // Take SVC: the vector comes from the relocated table.
    a.svc(1);
    a.label("halt");
    a.b("halt");

    a.label("svc_handler");
    a.movs_imm8(3, 0x99);
    a.bx(14);
    a.label("not_handler");
    a.b("not_handler");

    a.vec(11, "not_handler");

    let mut img: Image = a.clone().finish(SP).expect("assemble");
    let mvw = img.addr("svc_w").expect("svc_w") as usize;
    let handler = img.addr("svc_handler").expect("svc_handler");
    patch_movw(&mut img.bytes[..], mvw, 1, (handler | 1) as u16);

    let stop = img.addr("halt").expect("halt");
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "scb_vtor_relocates_vector_table: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "scb_vtor_relocates_vector_table: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("scb_vtor_relocates_vector_table", &vio, &qemu).expect("traces agree");
}

// ---------------------------------------------------------------------------
// CFSR/HFSR: a divide-by-zero with DIV_0_TRP records a UsageFault DIVBYZERO
// in CFSR and the FORCED escalation bit in HFSR; the HardFault handler reads
// both through the memory windows, then clears them write-1-to-clear.
// ---------------------------------------------------------------------------

#[test]
fn scb_div0_records_cfsr_hfsr() {
    let Some(mt) = qemu_m4() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(0, 0xE000_ED14);
    a.mov_imm32(1, 0x0000_0010); // DIV_0_TRP
    a.str_imm(1, 0, 0);

    a.mov_imm32(0, 100);
    a.movs_imm8(1, 0); // divisor zero
    a.udiv(2, 0, 1);
    a.label("halt");
    a.b("halt");

    a.label("hf_handler");
    // CFSR: read DIVBYZERO (combined bit 25), then clear it the canonical
    // way firmware does, by writing the read value back (W1C).
    a.mov_imm32(0, 0xE000_ED28);
    a.ldr_imm(4, 0, 0); // r4 = 0x0200_0000
    a.str_imm(4, 0, 0); // clear DIVBYZERO
    a.ldr_imm(6, 0, 0); // r6 = 0
                        // HFSR: read FORCED (bit 30, where the QEMU oracle reports it), then
                        // clear it the same way.
    a.mov_imm32(0, 0xE000_ED2C);
    a.ldr_imm(5, 0, 0); // r5 = 0x4000_0000
    a.str_imm(5, 0, 0); // clear FORCED
    a.ldr_imm(7, 0, 0); // r7 = 0
    a.b("halt");

    a.vec(3, "hf_handler");
    conformance("scb_div0_records_cfsr_hfsr", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// ARMv6-M (Cortex-M0) conformance: the 16-bit Thumb-1 subset, the system
// registers that exist on M0, and the RAZ/WI treatment of the ARMv7-M-only
// BASEPRI/FAULTMASK (QEMU's M0 model reads them as 0 and ignores writes,
// matching VioARM's ARMv6-M gating). All these programs use only instructions
// legal on ARMv6-M — the same encodings the shared M3 suite exercises, minus
// the 32-bit Thumb-2 forms — and the `b .` landing loop is the 16-bit form.
// ---------------------------------------------------------------------------

/// The 16-bit `B .` self-loop, legal on ARMv6-M (`a.b()` emits the 32-bit `B.W`
/// which Cortex-M0 does not decode).
fn halt16(a: &mut Asm) {
    a.label("halt");
    a.raw(0xE7FE);
}

#[test]
fn m0_sysreg() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // MSP read (Thread mode, main stack), then a write with a 16-bit value.
    a.mrs(2, 0x08); // r2 = MSP
    a.movs_imm8(5, 0x40);
    a.msr(0x08, 5); // MSR MSP, r5 (low byte)
    a.mrs(3, 0x08); // r3 = MSP after write

    // PRIMASK 0 -> 0, then 1 -> 1 (M0 implements PRIMASK).
    a.movs_imm8(0, 0);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);
    a.movs_imm8(0, 1);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);

    // BASEPRI: ARMv7-M only. On M0 QEMU reads it as 0 and ignores writes —
    // exactly VioARM's gating, so the diff pins the RAZ/WI behavior.
    a.movs_imm8(0, 0x80);
    a.msr(0x11, 0); // MSR BASEPRI, r0 (ignored)
    a.mrs(1, 0x11); // r1 = 0
    a.msr(0x12, 0); // MSR BASEPRI_MAX, r0 (ignored)
    a.mrs(1, 0x12); // r1 = 0

    // FAULTMASK: also ARMv7-M only, same RAZ/WI on M0.
    a.movs_imm8(0, 1);
    a.msr(0x13, 0); // MSR FAULTMASK, r0 (ignored)
    a.mrs(1, 0x13); // r1 = 0
    a.movs_imm8(0, 0);
    a.msr(0x13, 0); // still ignored
    a.mrs(1, 0x13); // r1 = 0

    // CONTROL: nPRIV is RAZ/WI on M0 (no unprivileged state); SPSEL is the
    // only writable bit, and only in Thread mode.
    a.movs_imm8(0, 1);
    a.msr(0x14, 0); // CONTROL = 1 (nPRIV ignored)
    a.mrs(1, 0x14); // r1 = 0
    a.movs_imm8(0, 2);
    a.msr(0x14, 0); // CONTROL = 2 (SPSEL=1 -> PSP active)
    a.mrs(1, 0x14); // r1 = 2
    a.mrs(2, 0x09); // r2 = PSP (0 at reset)
    a.movs_imm8(0, 0);
    a.msr(0x14, 0); // CONTROL = 0 (back to MSP)
    a.mrs(1, 0x14); // r1 = 0

    // MSP read again.
    a.mrs(2, 0x08);

    halt16(&mut a);
    conformance_m0("m0_sysreg", &mut a, "halt", &mt);
}

#[test]
fn m0_alu_thumb1() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Build operand values with 16-bit instructions only.
    a.movs_imm8(0, 0x45);
    a.lsls_imm(0, 0, 8); // r0 = 0x4500
    a.movs_imm8(1, 0xFF);
    a.movs_imm8(2, 0x80);
    a.movs_imm8(3, 0x03);
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

    halt16(&mut a);
    conformance_m0("m0_alu_thumb1", &mut a, "halt", &mt);
}

#[test]
fn m0_loadstore_thumb1() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Base into low RAM via 16-bit construction.
    a.movs_imm8(0, 0x20);
    a.lsls_imm(0, 0, 24); // r0 = 0x2000_0000
    a.movs_imm8(1, 0x40);

    // Word, byte, and halfword immediate-offset forms.
    a.str_imm(1, 0, 0); // [r0] = 0x40
    a.ldr_imm(2, 0, 0); // r2 = 0x40
    a.movs_imm8(1, 0xAB);
    a.strb_imm(1, 0, 4); // [r0+4] = 0xAB
    a.ldrb_imm(3, 0, 4); // r3 = 0xAB
    a.movs_imm8(1, 0x7F);
    a.strh_imm(1, 0, 8); // [r0+8] = 0x7F
    a.ldrh_imm(3, 0, 8); // r3 = 0x7F

    // Register-offset form.
    a.movs_imm8(1, 0x33);
    a.movs_imm8(6, 4); // r6 = 4 (offset)
    a.ls_reg(4, 2, 0, 6); // ldr r2, [r0, r6] = [0x2000_0004] = 0xAB
    a.ls_reg(0, 1, 0, 6); // str r1, [r0, r6] = [0x2000_0004] = r1

    // SP-relative and literal loads.
    a.movs_imm8(1, 0x55);
    a.str_sp(1, 0); // [sp] = 0x55
    a.ldr_sp(3, 0); // r3 = 0x55
    a.ldr_lit(4, "lit"); // r4 = 0x11223344
    a.addr_sp(5, 8); // r5 = sp + 8

    // Push/pop.
    a.push(0x3F, true); // push {r0-r5, lr}
    a.pop(0x3F, false); // pop {r0-r5}
    a.movs_imm8(6, 0x66);

    a.b16("past_pool");
    a.align4();
    a.label("lit");
    a.dc_u32(0x1122_3344);
    a.label("past_pool");

    halt16(&mut a);
    conformance_m0("m0_loadstore_thumb1", &mut a, "halt", &mt);
}

#[test]
fn m0_branch_bl() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // 16-bit conditional branch (not taken).
    a.movs_imm8(0, 0);
    a.cmp_imm8(0, 1);
    a.bc(1, "not_taken"); // bne -> taken (flags differ)
    a.movs_imm8(1, 0xAA); // skipped
    a.label("not_taken");
    a.movs_imm8(1, 0xBB);

    // BL to a forward label and back via bx (M0-valid).
    a.movs_imm8(2, 0x11);
    a.bl("sub");
    a.movs_imm8(3, 0x22);
    a.b16("past_sub");
    a.label("sub");
    a.movs_imm8(4, 0x33);
    a.bx(14);
    a.label("past_sub");

    // CBZ / CBNZ (16-bit). Targets must be >= pc+6 (the offset is from pc+4).
    a.movs_imm8(0, 0);
    a.cbz(0, "zero_taken");
    a.movs_imm8(5, 0xCC); // skipped
    a.movs_imm8(6, 0xEE); // padding (skipped)
    a.label("zero_taken");
    a.movs_imm8(0, 1);
    a.cbnz(0, "nz_taken");
    a.movs_imm8(6, 0xDD); // skipped
    a.movs_imm8(7, 0xEE); // padding (skipped)
    a.label("nz_taken");

    halt16(&mut a);
    conformance_m0("m0_branch_bl", &mut a, "halt", &mt);
}

#[test]
fn m0_exception_svc() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(11, "svc_handler"); // SVC exception 11

    a.movs_imm8(0, 1);
    a.svc(0);
    a.movs_imm8(1, 0x22); // after first return
    a.movs_imm8(0, 2);
    a.svc(0);
    a.movs_imm8(2, 0x33); // after second return
    a.b16("halt");

    a.label("svc_handler");
    a.movs_imm8(9, 0x99); // marker
    a.bx(14); // EXC_RETURN THREAD_MSP

    halt16(&mut a);
    conformance_m0("m0_exception_svc", &mut a, "halt", &mt);
}

#[test]
fn m0_hardfault_undef() {
    let Some(mt) = qemu_m0() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf_handler"); // HardFault vector

    // An undefined 16-bit encoding: on ARMv6-M the only exceptions are Reset,
    // NMI, HardFault, SVCall, PendSV, and SysTick, so this escalates to
    // HardFault directly (no UsageFault on M0). The handler must not `bx lr`
    // back to the faulting instruction (that would re-fault and lock up), so
    // it lands on the shared halt loop instead.
    a.raw(0xDEFF); // undefined
    a.movs_imm8(1, 0xAA); // never reached

    a.label("hf_handler");
    a.movs_imm8(9, 0x77); // marker
    a.b16("halt");

    halt16(&mut a);
    conformance_m0("m0_hardfault_undef", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// ARMv8-M baseline conformance against the Cortex-M33 mainline oracle: every
// instruction the baseline (Cortex-M23) implements behaves identically on the
// M33, so a baseline-valid binary's trace must agree exactly. The programs use
// only baseline encodings — the ARMv6-M 16-bit set plus the baseline Thumb-2
// additions (B.W, MOVW/MOVT, SDIV/UDIV, LDREX/STREX family, CLREX, MRS/MSR,
// IT blocks, CBZ/CBNZ). BASEPRI/FAULTMASK are deliberately excluded: the M33
// *has* them (they are mainline), so their baseline RAZ/WI behavior cannot be
// diffed and is pinned by unit tests in `vioarm-core` instead.
//
// Like the M3 reference, QEMU's M33 model cannot execute a *taken*
// conditional branch when it is the single conditional instruction of an IT
// block (verified empirically: the M33 lockups exactly like the M3), so IT
// conformance is restricted to suppressed branches, mirroring the shared M3
// suite.
// ---------------------------------------------------------------------------

/// The system registers that exist on both the baseline and mainline behave
/// identically on the M33: MSP/PSP/PRIMASK round-trips, and — unlike ARMv6-M —
/// CONTROL.nPRIV and SPSEL are both writable.
#[test]
fn m33_baseline_sysreg() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // MSP/PSP bank probe (privileged).
    a.mov_imm32(5, 0x4000_0000);
    a.msr(0x08, 5); // MSR MSP, r5
    a.mrs(2, 0x08); // r2 = MSP
    a.mrs(6, 0x09); // r6 = PSP
    a.mov_imm32(5, 0x5000_0000);
    a.msr(0x09, 5); // MSR PSP, r5
    a.mrs(7, 0x09); // r7 = PSP
    a.mrs(3, 0x08); // r3 = MSP

    // PRIMASK 0 -> 0, then 1 -> 1.
    a.movs_imm8(0, 0);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);
    a.movs_imm8(0, 1);
    a.msr(0x10, 0);
    a.mrs(1, 0x10);

    // CONTROL: nPRIV then SPSEL, both writable on the baseline.
    a.movs_imm8(0, 1);
    a.msr(0x14, 0); // CONTROL.nPRIV = 1
    a.mrs(1, 0x14); // r1 = 1
    a.movs_imm8(0, 2);
    a.msr(0x14, 0); // CONTROL.SPSEL = 1 (thread switches to PSP)
    a.mrs(1, 0x14); // r1 = 3
    a.mrs(4, 0x09); // r4 = PSP (0x5000_0000)
    a.movs_imm8(0, 0);
    a.msr(0x14, 0); // back to privileged, MSP
    a.mrs(1, 0x14); // r1 = 0

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_sysreg", &mut a, "halt", &mt);
}

/// The baseline Thumb-2 additions: MOVW/MOVT, SDIV/UDIV, the exclusive family,
/// CLREX, CBZ/CBNZ, IT blocks, and the wide B.W/BL branches.
#[test]
fn m33_baseline_additions() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // MOVW/MOVT compose any 32-bit constant.
    a.movw(0, 0x4567);
    a.movt(0, 0x8123);
    a.mov_imm32(1, 0x1020_3040);

    // SDIV/UDIV: positive, negative, MIN/-1 (saturates to MIN), zero.
    a.mov_imm32(2, 0x8000_0000);
    a.mov_imm32(3, 0xFFFF_FFFF); // -1
    a.mov_imm32(4, 0x0000_0007);
    a.sdiv(5, 1, 4); // 0x10203040 / 7
    a.sdiv(6, 2, 3); // MIN / -1 -> MIN
    a.udiv(7, 2, 4); // 0x80000000 / 7
    a.udiv(8, 3, 4); // 0xFFFFFFFF / 7
    a.udiv(8, 1, 4);

    // Exclusive word/byte/halfword on SRAM, plus CLREX clearing the monitor.
    // The 16-bit STR/LDR* helpers only address R0-R7, so the base stays in a
    // low register even though the 32-bit exclusive instructions accept any.
    a.mov_imm32(3, 0x2000_1000);
    a.mov_imm32(4, 8);
    a.mov_imm32(0, 0x1122_3344);
    a.mov_imm32(1, 0xAABB_CCDD);
    a.str_imm(0, 3, 0); // seed
    a.ldrex(1, 3, 0);
    a.strex(5, 1, 3, 0); // success
    a.ldr_imm(1, 3, 0); // verify
    a.adds_reg(4, 3, 4); // r4 = base + 8
    a.ldrexb(1, 4);
    a.strexb(5, 1, 4); // byte success
    a.ldrb_imm(1, 4, 0);
    a.ldrexh(1, 4);
    a.strexh(5, 1, 4); // halfword success
    a.ldrh_imm(1, 4, 0);
    a.ldrex(1, 3, 0); // tag [r3]
    a.clrex(); // clear the monitor
    a.strex(5, 1, 3, 0); // fails, no store
    a.ldr_imm(2, 3, 0); // still 0xAABBCCDD
    a.dmb();
    a.ldrex(1, 3, 0); // re-tag
    a.strex(5, 1, 3, 0); // succeeds
    a.dsb();

    // CBZ/CBNZ (16-bit, forward-only, within +2..126 bytes).
    a.movs_imm8(2, 0);
    a.cbz(2, "cbz_local"); // taken
    a.movs_imm8(4, 0xEE); // skipped
    a.nop(); // padding so the target lands at offset +2
    a.label("cbz_local");
    a.movs_imm8(2, 1);
    a.cbnz(2, "cbnz_local"); // taken
    a.movs_imm8(4, 0xEE); // skipped
    a.nop(); // padding so the target lands at offset +2
    a.label("cbnz_local");

    // IT blocks: executed, suppressed, and a suppressed conditional branch
    // inside a single-instruction IT (the taken-branch-inside-IT case is
    // intentionally absent, mirroring the shared M3 suite's limitation).
    a.movs_imm8(0, 1);
    a.cmp_imm8(0, 1); // Z = 1
    a.it(0, 0b1000); // it eq
    a.movs_imm8(3, 0x33); // EQ true -> executes
    a.it(1, 0b1000); // it ne
    a.movs_imm8(4, 0x44); // NE false -> suppressed
    a.it(1, 0b1000); // it ne
    a.bc(1, "it_nt"); // bne suppressed (Z=1), falls through
    a.movs_imm8(6, 0x66);
    a.label("it_nt");
    a.movs_imm8(7, 0x77);

    // BL + BX LR subroutine; wide B.W into the landing loop.
    a.bl("sub");
    a.movs_imm8(8, 0x88); // after return
    a.b("halt");
    a.label("sub");
    a.movs_imm8(9, 0x99);
    a.bx(14);
    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_additions", &mut a, "halt", &mt);
}

/// The load-acquire / store-release family: `LDA`/`LDAB`/`LDAH`,
/// `STL`/`STLB`/`STLH`, and the exclusive `LDAEX*`/`STLEX*` forms with
/// `CLREX` interplay. All register targets stay in R0-R7 so the 16-bit
/// STR/LDR verifiers (which address low registers only) work.
#[test]
fn m33_baseline_lda_stl() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.mov_imm32(3, 0x2000_1000);
    a.mov_imm32(0, 0x1122_3344);
    a.str_imm(0, 3, 0); // seed [base]
    a.lda(1, 3); // r1 = 0x11223344
    a.ldab(2, 3); // r2 = 0x44
    a.ldah(4, 3); // r4 = 0x3344

    // Plain store-release updates the memory the same way a store does.
    a.mov_imm32(1, 0xAABB_CCDD);
    a.stl(1, 3);
    a.ldr_imm(5, 3, 0); // r5 = 0xAABBCCDD
    a.mov_imm32(2, 0xEE);
    a.stlb(2, 3); // low byte = 0xEE
    a.mov_imm32(4, 0x9988);
    a.stlh(4, 3); // low halfword = 0x9988
    a.ldr_imm(5, 3, 0); // r5 = 0xAABB9988

    // Exclusive acquire/release: LDAEX tags, STLEX succeeds, CLREX fails it.
    a.mov_imm32(0, 0x5566_7788);
    a.ldaex(6, 3); // r6 = 0xAABB9988, tags [base]
    a.stlex(7, 0, 3); // success: r7 = 0, [base] = 0x55667788
    a.ldr_imm(5, 3, 0); // r5 = 0x55667788
    a.clrex();
    a.stlex(7, 0, 3); // fails: r7 = 1, no store
    a.ldr_imm(5, 3, 0); // r5 = 0x55667788 (unchanged)

    // Byte/halfword exclusive forms round-trip.
    a.ldaexb(6, 3); // tags byte
    a.stlexb(7, 2, 3); // success: r7 = 0, low byte 0xEE
    a.ldaexh(6, 3); // tags halfword
    a.stlexh(7, 4, 3); // success: low halfword 0x9988
    a.ldr_imm(5, 3, 0); // r5 = 0x55669988

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_lda_stl", &mut a, "halt", &mt);
}

/// MSPLIM/PSPLIM are readable/writable on the ARMv8-M baseline and on the M33
/// mainline oracle. Only register round-trips are diffed: a stack-limit
/// *violation* raises a StackFault (IPSR=4) on the M33 but folds to HardFault
/// on the baseline, so that behavior is pinned by VioARM unit tests instead.
#[test]
fn m33_baseline_splim() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Reset state: both limits read as zero.
    a.mrs(1, 0x0A); // r1 = MSPLIM = 0
    a.mrs(2, 0x0B); // r2 = PSPLIM = 0

    // MSPLIM round-trip.
    a.mov_imm32(5, 0x2000_1000);
    a.msr(0x0A, 5);
    a.mrs(1, 0x0A); // r1 = 0x2000_1000

    // PSPLIM round-trip.
    a.mov_imm32(5, 0x2000_2000);
    a.msr(0x0B, 5);
    a.mrs(2, 0x0B); // r2 = 0x2000_2000

    // Writing the *non-current* stack's limit below the active SP never
    // faults; the value round-trips. The active SP here sits in flash, well
    // below 0x2000_1000.
    a.mov_imm32(5, 0x1000_0000);
    a.msr(0x0B, 5); // PSPLIM = 0x1000_0000
    a.mrs(2, 0x0B); // r2 = 0x1000_0000

    // Restore both limits to their reset value so nothing downstream is
    // constrained, then confirm.
    a.mov_imm32(5, 0);
    a.msr(0x0A, 5);
    a.msr(0x0B, 5);
    a.mrs(1, 0x0A); // r1 = 0
    a.mrs(2, 0x0B); // r2 = 0

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_splim", &mut a, "halt", &mt);
}

/// The ARMv8-M `TT` family reads the security state of an address without
/// performing an access. On the mps2-an505 the ARMSSE IDAU attributes by
/// address bit [31:28] (even regions Non-secure, odd Secure) with the system
/// aliases exempt, and with the SAU disabled every non-exempt address reads
/// Secure. VioARM runs the same TrustZone model, so `TT`/`TTTT`/`TTA`/`TTAT`
/// over representative addresses must diff identically against the M33.
#[test]
fn m33_baseline_tt() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // TT: region attribution by Addr[31:28], Secure with the SAU off.
    a.mov_imm32(0, 0x0000_0000);
    a.tt(4, 0); // region 0
    a.mov_imm32(0, 0x1000_0000);
    a.tt(4, 0); // region 1
    a.mov_imm32(0, 0x2000_0000);
    a.tt(4, 0); // region 2
    a.mov_imm32(0, 0xE000_0000);
    a.tt(4, 0); // system alias, exempt (no irvalid/iregion)
    a.mov_imm32(0, 0xE100_0000);
    a.tt(4, 0); // region 14 but not exempt
    a.mov_imm32(0, 0xF000_0000);
    a.tt(4, 0); // F alias, exempt
    a.mov_imm32(0, 0xFFFF_FFFF);
    a.tt(4, 0); // region 15

    // TTTT forces an unprivileged target; without an MPU the permission bits
    // are unchanged.
    a.mov_imm32(0, 0x2000_0000);
    a.tttt(5, 0);
    a.mov_imm32(0, 0xE000_0000);
    a.tttt(5, 0);

    // TTA/TTAT probe the opposite (Non-secure) state: the exempt aliases flip
    // to Non-secure (S=0, NSR/NSRW set), non-exempt addresses stay Secure
    // because the SAU is disabled.
    a.mov_imm32(0, 0x2000_0000);
    a.tta(6, 0);
    a.mov_imm32(0, 0xE000_0000);
    a.tta(6, 0);
    a.mov_imm32(0, 0x2000_0000);
    a.ttat(7, 0);
    a.mov_imm32(0, 0xE000_0000);
    a.ttat(7, 0);

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_tt", &mut a, "halt", &mt);
}

/// TT with the SAU programmed, diffed against the M33: two regions (region 0
/// = `0x8000_0000`..`0x8000_001F` Non-secure, region 1 = `0x0000_0000`..
/// `0x0000_001F` Non-secure-callable), `SAU_CTRL.ENABLE = 1`. The probes
/// cover a region hit, a no-hit (which QEMU leaves Secure — not the ARM
/// default-NS), an NSC region, an IDAU-Secure no-hit, the exempt aliases
/// through `TTA`, and the `ALLNS = 1` mode where every non-IDAU-Secure
/// address defaults Non-secure.
#[test]
fn m33_baseline_sau_tt() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Region 0: 0x8000_0000..0x8000_001F, Non-secure.
    a.mov_imm32(0, 0xE000_EDD8); // SAU_RNR
    a.movs_imm8(1, 0);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDDC); // SAU_RBAR
    a.mov_imm32(1, 0x8000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDE0); // SAU_RLAR
    a.mov_imm32(1, 0x8000_0001); // NS, ENABLE
    a.str_imm(1, 0, 0);
    // Region 1: 0x0000_0000..0x0000_001F, Non-secure-callable.
    a.mov_imm32(0, 0xE000_EDD8); // SAU_RNR
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDDC); // SAU_RBAR
    a.movs_imm8(1, 0);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDE0); // SAU_RLAR
    a.mov_imm32(1, 0x0000_0003); // NSC, ENABLE
    a.str_imm(1, 0, 0);
    // Enable the SAU.
    a.mov_imm32(0, 0xE000_EDD0); // SAU_CTRL
    a.movs_imm8(1, 1); // ENABLE
    a.str_imm(1, 0, 0);

    // TT probes (results land in r4..r11).
    a.mov_imm32(0, 0x8000_0000);
    a.tt(4, 0); // NS region hit
    a.mov_imm32(0, 0x8000_0020);
    a.tt(5, 0); // no hit: QEMU keeps it Secure
    a.mov_imm32(0, 0x0000_0000);
    a.tt(6, 0); // NSC region hit (Secure, SRVALID, SREGION=1)
    a.mov_imm32(0, 0x1000_0000);
    a.tt(7, 0); // IDAU-Secure no hit
    a.mov_imm32(0, 0xE000_0000);
    a.tta(8, 0); // exempt alias, probing Non-secure
    a.mov_imm32(0, 0x8000_0000);
    a.tta(9, 0); // NS region hit, probing Non-secure

    // ALLNS mode: non-IDAU-Secure addresses default Non-secure.
    a.mov_imm32(0, 0xE000_EDD0); // SAU_CTRL
    a.movs_imm8(1, 2); // ALLNS
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0x8000_0020);
    a.tt(10, 0); // no region, no IDAU upgrade -> Non-secure
    a.mov_imm32(0, 0x1000_0000);
    a.tt(11, 0); // IDAU-Secure upgrade -> Secure

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_sau_tt", &mut a, "halt", &mt);
}

/// TT with the PMSAv8 MPU programmed in the Secure bank, diffed against the
/// M33: regions 0/1 cover the flash/SRAM with AP=1 (read/write/execute),
/// region 2 is read-only (AP=3), region 3 is privileged read-only (AP=2),
/// then `MPU_CTRL.ENABLE = 1`. The probes cover a hit's `r`/`rw`/`mrvalid`/
/// `mregion`, the unprivileged (`TTTT`) user column of the `AP` matrix, and a
/// no-hit without `PRIVDEFENA` (a background fault).
#[test]
fn m33_baseline_mpu_tt() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Region 0: 0x1000_0000..0x1000_FFFF (flash), AP=1, no XN.
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x1000_0002); // base, AP=1
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RLAR
    a.mov_imm32(1, 0x1000_FFFF); // limit, ENABLE
    a.str_imm(1, 0, 0);
    // Region 1: 0x2000_0000..0x2000_FFFF (SRAM), AP=1.
    a.mov_imm32(0, 0xE000_ED98); // MPU_RNR
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x2000_0002);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RLAR
    a.mov_imm32(1, 0x2000_FFFF);
    a.str_imm(1, 0, 0);
    // Region 2: 0x8000_0000..0x8000_001F, AP=3 (read-only for everyone).
    a.mov_imm32(0, 0xE000_ED98); // MPU_RNR
    a.movs_imm8(1, 2);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x8000_0006); // base, AP=3
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RLAR
    a.mov_imm32(1, 0x8000_001F);
    a.str_imm(1, 0, 0);
    // Region 3: 0x3000_0000..0x3000_001F, AP=2 (privileged read-only).
    a.mov_imm32(0, 0xE000_ED98); // MPU_RNR
    a.movs_imm8(1, 3);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x3000_0004); // base, AP=2
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RLAR
    a.mov_imm32(1, 0x3000_001F);
    a.str_imm(1, 0, 0);
    // Enable the MPU.
    a.mov_imm32(0, 0xE000_ED94); // MPU_CTRL
    a.movs_imm8(1, 1); // ENABLE
    a.str_imm(1, 0, 0);

    // TT probes (results land in r4..r11).
    a.mov_imm32(0, 0x1000_0000);
    a.tt(4, 0); // region 0 hit: R/W, MRVALID, MREGION=0
    a.mov_imm32(0, 0x2000_0000);
    a.tt(5, 0); // region 1 hit: MREGION=1
    a.mov_imm32(0, 0x8000_0000);
    a.tt(6, 0); // region 2: read-only, MREGION=2
    a.mov_imm32(0, 0x8000_0000);
    a.tttt(7, 0); // AP=3: still read-only unprivileged
    a.mov_imm32(0, 0x3000_0000);
    a.tt(8, 0); // region 3 privileged: read-only, MREGION=3
    a.mov_imm32(0, 0x3000_0000);
    a.tttt(9, 0); // region 3 unprivileged: AP=2 user column -> no permission
    a.mov_imm32(0, 0x4000_0000);
    a.tt(10, 0); // no region, no PRIVDEFENA: background fault
    a.mov_imm32(0, 0x2000_0020);
    a.tttt(11, 0); // region 1 unprivileged: AP=1 still R/W, MREGION=1

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_mpu_tt", &mut a, "halt", &mt);
}

/// The full ARMv6-M 16-bit subset is baseline-valid, so the Thumb-1 ALU suite
/// must diff identically against the M33.
#[test]
fn m33_baseline_alu_thumb1() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    a.movs_imm8(0, 0x45);
    a.lsls_imm(0, 0, 8); // r0 = 0x4500
    a.movs_imm8(1, 0xFF);
    a.movs_imm8(2, 0x80);
    a.movs_imm8(3, 0x03);
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

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_alu_thumb1", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// ARMv8-M Security Extension: SG, BXNS/BLXNS and the FNC_RETURN secure
// function return, diffed against the M33 (mainline) oracle. The Non-secure
// routines are stored at runtime into `0x8000_0000` — region 8 on
// mps2-an505, a 16 MiB "system RAM" with no MPC block, and the only RAM the
// reset SP value `0x2000_2000` does *not* cover. At reset the M33 treats
// *every* address as Secure (`SAU_CTRL` resets with `ENABLE == 0` and
// `ALLNS == 0`, and the ARMSSE IDAU can only upgrade a Non-secure
// attribution to Secure), so the Secure code first programs
// `SAU_CTRL.ALLNS = 1`: that makes the IDAU Non-secure regions effective,
// while `0x1000_0000` (the code/vector table) stays Secure through the
// IDAU's odd-region upgrade. Both simulators execute the same store. The
// trace diff then exercises the whole state-switch path (banked SPs,
// CONTROL, stack limits) through R13/R15 and the MRS/MSR register accesses.
//
// `BLXNS` pushes its return frame on the *Secure* stack, so the tests that
// return from the Non-secure side run with `SP = 0x8000_0100` — inside the
// shared RAM, clear of the runtime-stored routine. (The VioARM v8m machine
// also maps the conventional `0x2000_0000` RAM, but the AN505 oracle has no
// memory there, so a stack in that window would diverge.)
// ---------------------------------------------------------------------------

/// The stack for the `BLXNS`-return tests: inside the `0x8000_0000` RAM that
/// both simulators back, above the runtime-stored Non-secure routine.
const V8M_NS_SP: u32 = 0x8000_0100;

/// Assemble a function and return the raw instruction bytes of its code (the
/// vector table occupies the first `CODE_BASE` bytes of the image).
fn code_bytes(a: &Asm) -> Vec<u8> {
    let img: Image = a.clone().finish(SP).expect("assemble");
    img.bytes[CODE_BASE as usize..].to_vec()
}

/// Emit a runtime loop storing `bytes` (zero-padded to a word multiple) as
/// little-endian words at `addr`, using r0 as the address and r1 as the word.
fn store_bytes(a: &mut Asm, addr: u32, bytes: &[u8]) {
    let mut buf = bytes.to_vec();
    while !buf.len().is_multiple_of(4) {
        buf.push(0);
    }
    for (i, chunk) in buf.chunks(4).enumerate() {
        let mut w = [0u8; 4];
        w.copy_from_slice(chunk);
        a.mov_imm32(1, u32::from_le_bytes(w));
        a.mov_imm32(0, addr.wrapping_add(u32::try_from(i).expect("word") * 4));
        a.str_imm(1, 0, 0);
    }
}

/// Assemble the Non-secure routine: `SG` (a NOP in the Non-secure state),
/// reads of the NS-banked registers from the Non-secure state (all RAZ) and
/// of the current-state SPs and CONTROL, a `CONTROL` write that clears NS
/// SPSEL and moves the active NS SP back to the main stack, then `BX LR`.
/// The return is a plain `BX` to the `FNC_RETURN` magic: `BXNS` is UNDEF from
/// the Non-secure state, and QEMU routes the `FNC_RETURN` magic on the `BX`
/// path (its `gen_bx_excret`), which the `VioARM` `branch()` mirrors. With
/// `ret == true` the routine returns; otherwise it halts at a loop whose byte
/// offset within the routine is returned (the stop address for the
/// direct-`BXNS` test). Returns `(bytes, stop_offset)`.
fn ns_routine_bytes(ret: bool) -> (Vec<u8>, u32) {
    let mut a = Asm::new();
    a.sg();
    a.mrs(6, 0x94); // r6 = CONTROL_NS (0 from NS)
    a.mrs(7, 0x88); // r7 = MSP_NS (0 from NS)
    a.mrs(8, 0x09); // r8 = PSP (the active NS SP while SPSEL is set)
    a.mrs(9, 0x14); // r9 = CONTROL (the current-state NS CONTROL)
    a.mrs(10, 0x08); // r10 = MSP (the inactive NS SP while SPSEL is set)
    a.movs_imm8(2, 0);
    a.msr(0x14, 2); // NS CONTROL = 0 -> SPSEL cleared, active SP back to MSP
    a.mrs(8, 0x09); // r8 = PSP (now the inactive NS SP)
    a.mrs(10, 0x08); // r10 = MSP (now the active NS SP)
    if ret {
        a.bx(14); // LR == 0xFEFFFFFF: FNC_RETURN
    } else {
        a.label("ns_halt");
        a.b("ns_halt");
    }
    let img: Image = a.clone().finish(SP).expect("assemble ns routine");
    let bytes = code_bytes(&a);
    let stop = if ret {
        0
    } else {
        img.addr("ns_halt").expect("ns halt") - CODE_BASE
    };
    (bytes, stop)
}

/// ARMv8-M `SG` (`0xE97FE97F`): unconditional even inside an IT block whose
/// condition fails — it executes and clears ITSTATE, so the following `beq` is
/// no longer in the IT block — and a 4-byte NOP outside an IT block.
#[test]
fn m33_baseline_sg() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // SG executes (clearing ITSTATE) despite the pending EQ condition failing,
    // so the `beq` after it is a plain conditional branch whose EQ test fails,
    // falling through to `movs r3,#0`.
    a.movs_imm8(2, 0xFF); // Z = 0
    a.it(0, 1); // it eq
    a.sg();
    a.bc(0, "taken");
    a.movs_imm8(3, 0);
    a.label("taken");
    a.sg(); // 4-byte NOP outside an IT block
    a.nop();
    a.nop();

    a.label("halt");
    a.b("halt");
    conformance_v8m("m33_baseline_sg", &mut a, "halt", &mt);
}

/// `BLXNS` (Non-secure function call) and the `FNC_RETURN` secure function
/// return. The Secure side primes the NS-banked SPs and CONTROL, reads them
/// back through `CONTROL_NS`/`PSP_NS`/`SP_NS`, then calls into the NS routine
/// and reads the banked state again after it returns.
#[test]
fn m33_baseline_blxns() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Make the IDAU Non-secure regions effective: `SAU_CTRL.ALLNS = 1`
    // attributes every IDAU Non-secure address as Non-secure, while the odd
    // IDAU regions (0x1000_0000, ...) stay Secure. Without this the M33
    // treats all of memory as Secure and the NS fetch below faults.
    a.mov_imm32(0, 0x2);
    a.mov_imm32(1, 0xE000_EDD0);
    a.str_imm(0, 1, 0);

    let (ns_bytes, _) = ns_routine_bytes(true);
    store_bytes(&mut a, 0x8000_0000, &ns_bytes);

    // Prime the Non-secure banked registers from Secure.
    a.mov_imm32(2, 0x2);
    a.msr(0x94, 2); // CONTROL_NS = SPSEL -> NS process stack
    a.mov_imm32(2, 0x2000_8000);
    a.msr(0x89, 2); // PSP_NS = 0x2000_8000
    a.mov_imm32(2, 0x2000_4000);
    a.msr(0x88, 2); // MSP_NS = 0x2000_4000
    a.mrs(3, 0x94); // r3 = CONTROL_NS = 0x2
    a.mrs(4, 0x89); // r4 = PSP_NS = 0x2000_8000
    a.mrs(5, 0x98); // r5 = SP_NS = 0x2000_8000 (thread mode + SPSEL)

    // BLXNS to the Non-secure routine and back via FNC_RETURN.
    a.mov_imm32(1, 0x8000_0000);
    a.blxns(1);
    a.mrs(3, 0x94); // CONTROL_NS: the NS code cleared its SPSEL -> 0
    a.mrs(4, 0x88); // MSP_NS = 0x2000_4000 (saved at the state switch)
    a.mrs(5, 0x89); // PSP_NS = 0x2000_8000

    a.label("halt");
    a.b("halt");
    conformance_v8m_at("m33_baseline_blxns", &mut a, "halt", V8M_NS_SP, &mt);
}

/// A direct `BXNS` to an even (Non-secure) target: the state switch without a
/// pushed return frame. The NS routine runs and halts in the Non-secure
/// state; both simulators stop at the loop inside the runtime-stored routine.
#[test]
fn m33_baseline_bxns() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // See `m33_baseline_blxns`: program `SAU_CTRL.ALLNS = 1` so the M33
    // attributes the IDAU Non-secure `0x8000_0000` region as Non-secure.
    a.mov_imm32(0, 0x2);
    a.mov_imm32(1, 0xE000_EDD0);
    a.str_imm(0, 1, 0);

    let (ns_bytes, ns_stop) = ns_routine_bytes(false);
    let ns_halt = 0x8000_0000 + ns_stop;
    store_bytes(&mut a, 0x8000_0000, &ns_bytes);

    a.mov_imm32(1, 0x8000_0000);
    a.bxns(1);

    let img: Image = a.clone().finish_at(SP, 0x1000_0000).expect("assemble");
    let (vio, reached) = vio_run_v8m(&img.bytes, ns_halt);
    assert!(
        reached,
        "m33_baseline_bxns: vioarm never reached NS halt @ {ns_halt:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, ns_halt);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == ns_halt,
        "m33_baseline_bxns: qemu never reached NS halt @ {ns_halt:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("m33_baseline_bxns", &vio, &qemu).expect("traces agree");
}

/// A Secure-state `udf` escalates to the Secure HardFault. `AIRCR.BFHFNMINS`
/// resets clear, so every escalated fault targets the Secure bank, whose vector
/// table `S_VTOR` the program re-homes at the flash base (the image's own
/// table, with the HardFault entry at `0x1000_000C`). The handler runs in
/// Handler mode with the Secure-thread `EXC_RETURN` (`0xFFFF_FFF9`) in `LR`
/// and the pushed frame's PC pointing back at the `udf`, and halts.
#[test]
fn m33_baseline_secure_undef_hardfault() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // The core resets into Secure state, so the `S_VTOR` write is a Secure
    // access; `BFHFNMINS` stays clear (reset) so escalated faults target
    // Secure and are taken from the `S_VTOR` table.
    a.mov_imm32(0, 0x1000_0000);
    a.mov_imm32(1, 0xE002_ED08);
    a.str_imm(0, 1, 0);

    a.raw(0xDE00); // udf #0: UsageFault; USGFAULTENA reset clear -> HardFault

    a.label("h_hardfault");
    a.b("halt");

    a.label("halt");
    a.b("halt");
    a.vec(3, "h_hardfault");

    conformance_v8m_at(
        "m33_baseline_secure_undef_hardfault",
        &mut a,
        "halt",
        SP,
        &mt,
    );
}

/// A Secure-thread `udf` escalates to the Secure `HardFault` (Secure to Secure
/// entry). The
/// handler clobbers `r0-r3`/`r12`, fixes the stacked PC past the 16-bit `udf`,
/// and returns via `bx lr`. Both simulators must restore `r0-r3`/`r12` from
/// the pushed basic frame on the v8-M exception return, so the trace diff after
/// the return is a direct check of the return-path restore.
#[test]
fn m33_baseline_secure_udf_return_restores_r0r3() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Re-home the Secure vector table at the flash base, as in
    // `m33_baseline_secure_undef_hardfault`.
    a.mov_imm32(0, 0x1000_0000);
    a.mov_imm32(1, 0xE002_ED08);
    a.str_imm(0, 1, 0);

    // Prime the registers the exception return must restore.
    a.mov_imm32(0, 0x1111_1111);
    a.mov_imm32(1, 0x2222_2222);
    a.mov_imm32(2, 0x3333_3333);
    a.mov_imm32(3, 0x4444_4444);
    a.mov_imm32(12, 0xCCCC_CCCC);
    a.raw(0xDE00); // udf #0: UsageFault; USGFAULTENA reset clear -> HardFault
    a.b("halt"); // the return resumes here (udf + 2)

    a.label("h_hardfault");
    a.mov_reg(4, 13); // r4 = frame base (the handler's SP)
    a.ldr_imm(5, 4, 24); // r5 = stacked PC (points at the `udf`)
    a.adds_imm8(5, 2); // skip the 16-bit `udf`
    a.str_imm(5, 4, 24); // write the resume PC back into the frame
    a.movs_imm8(0, 0xEE); // clobber what the return must restore
    a.movs_imm8(1, 0xEE);
    a.movs_imm8(2, 0xEE);
    a.movs_imm8(3, 0xEE);
    a.mov_imm32(12, 0xEEEE_EEEE);
    a.bx(14); // return to Secure thread via EXC_RETURN (0xFFFF_FFF9)

    a.label("halt");
    a.b("halt");
    a.vec(3, "h_hardfault");

    conformance_v8m_at(
        "m33_baseline_secure_udf_return_restores_r0r3",
        &mut a,
        "halt",
        SP,
        &mt,
    );
}

/// The Non-secure setup routine executed via a `BLXNS` round-trip before the
/// interrupt is armed: it writes the Non-secure `VTOR` (`0xE000_ED08`) from the
/// Non-secure state — the only way to address the NS bank, a Secure write hits
/// the Secure `VTOR` on the M33 — then returns through the `FNC_RETURN` magic
/// the `BLXNS` left in `LR`.
fn ns_vtor_setup_bytes() -> Vec<u8> {
    let mut a = Asm::new();
    a.mov_imm32(0, 0x8000_0000); // the NS vector table base
    a.mov_imm32(1, 0xE000_ED08); // VTOR
    a.str_imm(0, 1, 0);
    a.bx(14); // LR == 0xFEFFFFFF: FNC_RETURN
    code_bytes(&a)
}

/// The Non-secure `IRQ0` handler: clobbers every general register so the diff
/// can only pass if the Secure exception return restores both the caller-saved
/// registers (from the basic frame) and the callee-saved `r4-r11` (from the
/// `DCRS` integrity block) on the Secure stack, then returns via the
/// `EXC_RETURN` in `LR`.
fn ns_irq_handler_bytes() -> Vec<u8> {
    let mut a = Asm::new();
    for r in 0u8..=12 {
        a.mov_imm32(r, 0xEE);
    }
    a.bx(14);
    code_bytes(&a)
}

/// A Secure-thread external interrupt whose ARMSSE-default `ITNS` targets the
/// Non-secure state is a Secure-to-Non-secure v8-M exception entry, diffed
/// against the M33: both simulators must stack the callee-saves block
/// (integrity signature + `r4-r11`) on the Secure stack and run the handler in
/// Non-secure state with `r0-r12` cleared; the handler clobbers every register
/// and returns through `EXC_RETURN`, and the diff after the return is the
/// direct check that the whole `DCRS` block and the basic frame are restored.
///
/// The Non-secure vector table and handler live in the shared system RAM at
/// `0x8000_0000` (the only Non-secure-executable memory on the AN505 oracle),
/// and the Non-secure `VTOR` is pointed there by a `BLXNS` round-trip before
/// the interrupt is software-pended.
///
/// This scenario cannot be diffed against current `qemu-system-arm`
/// (10.2.x, `mps2-an505`): there the board delivers external interrupts
/// Secure-targeted regardless of `NVIC_ITNS` programming — the entry builds a
/// basic frame (`EXC_RETURN` `0xFFFFFFF9`) and vectors through `VTOR_S`, so
/// the Secure-to-Non-secure entry this test exercises never happens on the
/// oracle side. The full round trip is therefore locked VioARM-only below,
/// asserting every register the cross-state return must restore: caller-saved
/// from the basic frame, callee-saved `r4-r11` from the integrity block.
#[test]
fn m33_baseline_secure_irq_returns_nonsecure() {
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Make the IDAU Non-secure regions effective, as in the `BXNS` tests.
    a.mov_imm32(0, 0x2);
    a.mov_imm32(1, 0xE000_EDD0); // SAU_CTRL
    a.str_imm(0, 1, 0); // ALLNS

    // Give the Non-secure bank a known main stack in shared RAM.
    a.mov_imm32(2, 0x8000_0800);
    a.msr(0x88, 2); // MSP_NS = 0x8000_0800

    // Store the Non-secure vector table and routines at `0x8000_0000`: with
    // the NS `VTOR` at `0x8000_0000`, `IRQ0`'s entry is word 16 (`0x8000_0040`).
    store_bytes(&mut a, 0x8000_0040, &(0x8000_0140u32 | 1).to_le_bytes());
    store_bytes(&mut a, 0x8000_0200, &ns_vtor_setup_bytes());
    store_bytes(&mut a, 0x8000_0140, &ns_irq_handler_bytes());

    // Point the Non-secure `VTOR` at the stored table, from the Non-secure
    // state, and return.
    a.mov_imm32(1, 0x8000_0200);
    a.blxns(1);

    // Re-home the Secure vector table at the flash base, as in the other
    // baseline tests.
    a.mov_imm32(0, 0x1000_0000);
    a.mov_imm32(1, 0xE002_ED08); // S_VTOR
    a.str_imm(0, 1, 0);

    // Prime the registers the exception return must restore.
    a.mov_imm32(0, 0x1111_1111);
    a.mov_imm32(1, 0x2222_2222);
    a.mov_imm32(2, 0x3333_3333);
    a.mov_imm32(3, 0x4444_4444);
    a.mov_imm32(12, 0xCCCC_CCCC);
    a.mov_imm32(4, 0x4000_4004);
    a.mov_imm32(5, 0x4000_4005);
    a.mov_imm32(6, 0x4000_4006);
    a.mov_imm32(7, 0x4000_4007);
    a.mov_imm32(8, 0x4000_4008);
    a.mov_imm32(9, 0x4000_4009);
    a.mov_imm32(10, 0x4000_400A);
    a.mov_imm32(11, 0x4000_400B);

    // Enable and software-pend IRQ0, pinning its targeting Non-secure first
    // (`NVIC_ITNS0` bit set; the reset value is implementation defined).
    // Taking it from this Secure thread is then a Secure-to-Non-secure entry:
    // the callee-saves block is stacked on the Secure stack and the handler
    // runs Non-secure with `r0-r12` cleared.
    a.mov_imm32(1, 0xE000_E380); // NVIC_ITNS0
    a.movs_imm8(0, 1);
    a.str_imm(0, 1, 0);
    a.mov_imm32(1, 0xE000_E100); // ISER0
    a.movs_imm8(0, 1);
    a.str_imm(0, 1, 0);
    a.mov_imm32(1, 0xE000_E200); // ISPR0
    a.movs_imm8(0, 1);
    a.str_imm(0, 1, 0);

    // The Non-secure handler ran and returned; execution resumes here (the
    // instruction after the pend) in the Secure state with `r0-r3`/`r12` and
    // `r4-r11` restored from the two stack blocks -- checked register-for-
    // register below.
    a.b("halt");

    a.label("h_hardfault");
    a.b("halt");

    a.label("halt");
    a.b("halt");
    a.vec(3, "h_hardfault");

    let img: Image = a.finish_at(V8M_NS_SP, 0x1000_0000).expect("assemble");
    let stop = img.addr("halt").expect("halt label");
    let (vio, reached) = vio_run_v8m(&img.bytes, stop);
    assert!(
        reached,
        "vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let s = vio.last().expect("trace states");
    assert_eq!(s.r[15] & !1, stop);
    assert_eq!(s.r[13], V8M_NS_SP, "thread SP restored");
    assert_eq!(s.r[14], 0xFEFF_FFFF, "thread LR restored");
    // r0/r1 were re-used by the ITNS/ISER/ISPR stores after priming; the
    // values at pend time are what the return must restore.
    for (reg, want) in [
        (0, 0x0000_0001),
        (1, 0xE000_E200),
        (2, 0x3333_3333),
        (3, 0x4444_4444),
        (12, 0xCCCC_CCCC),
    ] {
        assert_eq!(s.r[reg], want, "basic-frame r{reg} restored");
    }
    for (i, reg) in (4..=11).enumerate() {
        let want = 0x4000_4004 + i as u32;
        assert_eq!(s.r[reg], want, "callee-block r{reg} restored");
    }
}

/// The Non-secure routine executed via a `BXNS` jump: an `udf`. With
/// `AIRCR.BFHFNMINS` reset clear, the escalated `HardFault` targets the
/// **Secure** state (`S_VTOR`), so the entry is Non-secure back-ground to
/// Secure handler — the no-callee-block counterpart of
/// [`m33_baseline_secure_irq_returns_nonsecure`]. The Secure handler patches
/// the stacked PC past the 16-bit `udf` and returns; the thread then loops at
/// the returned offset within the routine. Returns `(bytes, after_offset)`.
fn ns_udef_bytes() -> (Vec<u8>, u32) {
    let mut a = Asm::new();
    a.raw(0xDE00); // udf #0: UsageFault; USGFAULTENA reset clear -> HardFault
    a.label("after");
    a.b("after"); // the patched return resumes here (udf + 2)
    let img: Image = a.clone().finish(SP).expect("assemble ns routine");
    let bytes = code_bytes(&a);
    let stop = img.addr("after").expect("after") - CODE_BASE;
    (bytes, stop)
}

/// A Non-secure `udf` with `BFHFNMINS` clear escalates to the **Secure**
/// `HardFault`: a Non-secure-to-Secure v8-M entry, diffed against the M33. The
/// frame is pushed on the Non-secure stack and the handler runs in Secure
/// state with the `S`-clear/`ES`-set `EXC_RETURN` (`0xFFFF_FFB9`); it patches
/// the stacked PC past the `udf`, clobbers the caller-saved registers, and
/// returns through that `EXC_RETURN`, so the trace diff after the return is
/// the direct check of the whole Non-secure exception-return path.
///
/// The Non-secure routine runs in the shared system RAM at `0x8000_0000`, and
/// the Secure handler finds its frame base by reading `MSP_NS` (the only way
/// to locate the frame the interrupt state change left in the other bank).
#[test]
fn m33_baseline_nonsecure_udf_returns_through_secure_handler() {
    let Some(mt) = qemu_m33() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);

    // Make the IDAU Non-secure regions effective, as in the `BXNS` tests.
    a.mov_imm32(0, 0x2);
    a.mov_imm32(1, 0xE000_EDD0); // SAU_CTRL
    a.str_imm(0, 1, 0); // ALLNS

    // Re-home the Secure vector table at the flash base.
    a.mov_imm32(0, 0x1000_0000);
    a.mov_imm32(1, 0xE002_ED08); // S_VTOR
    a.str_imm(0, 1, 0);

    // A known Non-secure main stack, so the fault frame base is deterministic.
    a.mov_imm32(2, 0x8000_0800);
    a.msr(0x88, 2); // MSP_NS = 0x8000_0800

    // Store the Non-secure `udf` routine and jump to it.
    let (ns_bytes, ns_after) = ns_udef_bytes();
    let stop = 0x8000_0000 + ns_after;
    store_bytes(&mut a, 0x8000_0000, &ns_bytes);
    a.mov_imm32(1, 0x8000_0000);
    a.bxns(1);

    // Never returned here (the fault pushed straight into the handler).
    a.label("never");
    a.b("never");

    // Secure HardFault handler: the frame lives on the Non-secure stack, so
    // resolve the base through `MSP_NS`, skip the `udf`, clobber the
    // caller-saved registers (which the return must bring back from the frame),
    // and return via the `S`-clear/`ES`-set `EXC_RETURN`.
    a.label("h_hardfault");
    a.mrs(4, 0x88); // r4 = MSP_NS = frame base
    a.ldr_imm(5, 4, 24); // r5 = stacked PC (the `udf`)
    a.adds_imm8(5, 2); // skip the 16-bit `udf`
    a.str_imm(5, 4, 24); // patch the resume PC back into the frame
    a.mov_imm32(0, 0xAAAA_AAAA);
    a.mov_imm32(1, 0xBBBB_BBBB);
    a.mov_imm32(2, 0xCCCC_CCCC);
    a.mov_imm32(3, 0xDDDD_DDDD);
    a.mov_imm32(12, 0xEEEE_EEEE);
    a.bx(14); // return to the Non-secure thread via EXC_RETURN (0xFFFF_FFB9)
    a.vec(3, "h_hardfault");

    let img: Image = a.clone().finish_at(SP, 0x1000_0000).expect("assemble");
    let (vio, reached) = vio_run_v8m(&img.bytes, stop);
    assert!(
        reached,
        "m33_baseline_nonsecure_udf_returns_through_secure_handler: \
         vioarm never reached NS halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "m33_baseline_nonsecure_udf_returns_through_secure_handler: \
         qemu never reached NS halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces(
        "m33_baseline_nonsecure_udf_returns_through_secure_handler",
        &vio,
        &qemu,
    )
    .expect("traces agree");
}

// ---------------------------------------------------------------------------
// PMSAv7 (ARMv7-M, Cortex-M3) MPU *fault* behavior. The register model is
// already locked by the conformance suite; this diff pins the recorded fault
// status too: a data access outside every enabled region (no `PRIVDEFENA`) is
// a `DACCVIOL` with `MMARVALID` and the address in `MMFAR`, escalated to
// HardFault with `HFSR.FORCED`. The handler captures `MMFSR`/`HFSR`/`MMFAR`
// into the trace so the per-instruction diff checks VioARM's fault recording
// against the M3 oracle. The SP and code stay inside the mapped region so the
// error-stacking itself does not fault (which would diverge from the oracle's
// `STKERR` synthesis).
// ---------------------------------------------------------------------------

/// A denied data access under the PMSAv7 MPU: region 1 covers the flash base
/// (where this image runs), region 0 covers `0x2000_0000` for 2 KiB with
/// read/write access, and `MPU_CTRL.ENABLE` is set without `PRIVDEFENA`, so a
/// load from outside both regions is a background `DACCVIOL`. Both simulators
/// must escalate it to HardFault with identical `MMFSR`/`HFSR`/`MMFAR`
/// contents and resume through the handler's patched frame. The SP runs inside
/// region 0 so the error-stacking itself never faults (or it would diverge
/// through the oracle's `STKERR` synthesis, which VioARM does not model).
const MPU_SP: u32 = 0x2000_0600;

#[test]
fn mpu_data_violation_escalates_to_hardfault() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf"); // HardFault vector

    // Region 1: the flash base, 1 MiB, rw/rw (this image runs there).
    a.mov_imm32(0, 0xE000_ED98); // MPU_RNR
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x0000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RASR
    a.mov_imm32(1, 0x0300_0027); // EN, SIZE=1 MiB, AP=rw/rw
    a.str_imm(1, 0, 0);
    // Region 0: 0x2000_0000..=0x2000_07FF, rw/rw (the SP lives here).
    a.mov_imm32(0, 0xE000_ED98); // MPU_RNR
    a.movs_imm8(1, 0);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C); // MPU_RBAR
    a.mov_imm32(1, 0x2000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0); // MPU_RASR
    a.mov_imm32(1, 0x0300_0015); // EN, SIZE=2 KiB, AP=rw/rw
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED94); // MPU_CTRL
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0); // ENABLE (no PRIVDEFENA)

    // The denied load: 0x2000_2000 is in RAM but outside region 0.
    a.mov_imm32(1, 0x2000_2000);
    a.ldr_imm(0, 1, 0); // -> DACCVIOL, escalated to HardFault

    a.label("halt");
    a.b("halt");

    a.label("hf");
    // Capture the fault status into the trace (the PPB bypasses the MPU, so
    // these status-register reads are always permitted).
    a.mov_imm32(4, 0xE000_ED20); // CFSR
    a.ldr_imm(5, 4, 8); // r5 = MMFSR
    a.ldr_imm(6, 4, 12); // r6 = HFSR
    a.ldr_imm(7, 4, 20); // r7 = MMFAR
                         // Patch the stacked PC past the faulting load and return.
    a.mov_reg(1, 13); // frame base
    a.ldr_imm(2, 1, 24); // stacked PC (the faulting `ldr`)
    a.adds_imm8(2, 2); // skip it
    a.str_imm(2, 1, 24);
    a.bx(14);

    let img: Image = a.clone().finish_at(MPU_SP, 0x0000_0000).expect("assemble");
    let stop = img.addr("halt").expect("halt");
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "mpu_data_violation_escalates_to_hardfault: vioarm never reached halt \
         @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "mpu_data_violation_escalates_to_hardfault: qemu never reached halt \
         @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("mpu_data_violation_escalates_to_hardfault", &vio, &qemu).expect("traces agree");
}

/// A denied data access with `SHCSR.MEMFAULTENA` set: the same MPU
/// violation is taken as a distinct `MemManage` fault (vector 4) instead of
/// being escalated to `HardFault`. The handler captures `MMFSR`/`MMFAR` (no
/// `HFSR.FORCED`) and the `MemManage` frame, patches the stacked PC, and
/// returns; the diff locks ARMv7-M's `MEMFAULTENA` gating and the
/// `MemManage` priority/frame against the M3.
#[test]
fn mpu_data_violation_taken_as_memmanage() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(4, "mm"); // MemManage vector
    a.vec(3, "hf"); // HardFault fallback (should not be taken)

    // Region 1: flash base, 1 MiB, rw/rw (this image runs there).
    a.mov_imm32(0, 0xE000_ED98);
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C);
    a.mov_imm32(1, 0x0000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0);
    a.mov_imm32(1, 0x0300_0027);
    a.str_imm(1, 0, 0);
    // Region 0: 0x2000_0000..=0x2000_07FF, rw/rw (the SP lives here).
    a.mov_imm32(0, 0xE000_ED98);
    a.movs_imm8(1, 0);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C);
    a.mov_imm32(1, 0x2000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0);
    a.mov_imm32(1, 0x0300_0015);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED94);
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0); // MPU_CTRL ENABLE
                        // Enable the distinct MemManage exception.
    a.mov_imm32(0, 0xE000_ED24); // SHCSR
    a.mov_imm32(1, 0x0001_0000); // MEMFAULTENA
    a.str_imm(1, 0, 0);

    // The denied load: 0x2000_2000 is outside region 0 -> MemManage.
    a.mov_imm32(1, 0x2000_2000);
    a.ldr_imm(0, 1, 0);

    a.label("halt");
    a.b("halt");

    a.label("mm");
    a.mov_imm32(4, 0xE000_ED20);
    a.ldr_imm(5, 4, 8); // r5 = MMFSR (DACCVIOL|MMARVALID, no FORCED)
    a.ldr_imm(6, 4, 12); // r6 = HFSR (must be 0)
    a.ldr_imm(7, 4, 20); // r7 = MMFAR
    a.mov_reg(1, 13);
    a.ldr_imm(2, 1, 24);
    a.adds_imm8(2, 2);
    a.str_imm(2, 1, 24);
    a.bx(14);

    a.label("hf");
    a.b("halt");

    let img: Image = a.clone().finish_at(MPU_SP, 0x0000_0000).expect("assemble");
    let stop = img.addr("halt").expect("halt");
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "mpu_data_violation_taken_as_memmanage: vioarm never reached halt \
         @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "mpu_data_violation_taken_as_memmanage: qemu never reached halt \
         @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("mpu_data_violation_taken_as_memmanage", &vio, &qemu).expect("traces agree");
}

/// A burst that straddles an MPU region boundary: `STMIA r0!, {r1-r4}`
/// from `0x2000_07FC` (the last word inside region 0) writes four words
/// `0x07FC/0x0800/0x0804/0x0808`; the second word is already outside the
/// region and must fault with `MMFAR` equal to that word, not the base.
/// Both simulators check every word of a burst, not just the first address.
#[test]
fn mpu_burst_straddles_region() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(3, "hf");

    // Region 1: flash base, 1 MiB, rw/rw (this image runs there).
    a.mov_imm32(0, 0xE000_ED98);
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C);
    a.mov_imm32(1, 0x0000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0);
    a.mov_imm32(1, 0x0300_0027);
    a.str_imm(1, 0, 0);
    // Region 0: 0x2000_0000..=0x2000_07FF, rw/rw (the SP lives here).
    a.mov_imm32(0, 0xE000_ED98);
    a.movs_imm8(1, 0);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED9C);
    a.mov_imm32(1, 0x2000_0000);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_EDA0);
    a.mov_imm32(1, 0x0300_0015);
    a.str_imm(1, 0, 0);
    a.mov_imm32(0, 0xE000_ED94);
    a.movs_imm8(1, 1);
    a.str_imm(1, 0, 0); // ENABLE

    // Four-word burst straddling the boundary: 0x07FC (in), 0x0800 (out).
    a.mov_imm32(0, 0x2000_07FC);
    a.mov_imm32(1, 0x1111_1111);
    a.mov_imm32(2, 0x2222_2222);
    a.mov_imm32(3, 0x3333_3333);
    a.mov_imm32(4, 0x4444_4444);
    a.stmia(0, 0b0001_1110); // stmia r0!, {r1-r4}

    a.label("halt");
    a.b("halt");

    a.label("hf");
    a.mov_imm32(4, 0xE000_ED20);
    a.ldr_imm(5, 4, 8); // MMFSR
    a.ldr_imm(6, 4, 12); // HFSR
    a.ldr_imm(7, 4, 20); // MMFAR (must be 0x2000_0800, the first faulting word)
    a.mov_reg(1, 13);
    a.ldr_imm(2, 1, 24);
    a.adds_imm8(2, 2);
    a.str_imm(2, 1, 24);
    a.bx(14);

    let img: Image = a.clone().finish_at(MPU_SP, 0x0000_0000).expect("assemble");
    let stop = img.addr("halt").expect("halt");
    let (vio, reached) = vio_run(&img.bytes, stop);
    assert!(
        reached,
        "mpu_burst_straddles_region: vioarm never reached halt @ {stop:#x} (last {})",
        snapshot_like(&vio)
    );
    let qemu = qemu_run_retry(&mt, &img.bytes, stop);
    assert!(
        !qemu.is_empty() && qemu.last().expect("empty").r[15] & !1 == stop,
        "mpu_burst_straddles_region: qemu never reached halt @ {stop:#x} (got {} states)",
        qemu.len()
    );
    compare_traces("mpu_burst_straddles_region", &vio, &qemu).expect("traces agree");
}

/// A precise bus error with `SHCSR.BUSFAULTENA` set: a load from an
/// unmapped address (`0x5000_0000`, in the peripheral window but not backed
/// by any device on this machine) is taken as a distinct `BusFault` (vector
/// 5) with `BFSR.PRECISERR` and no `HFSR.FORCED`. The handler captures
/// `BFSR`/`HFSR` into the trace, patches the stacked PC past the faulting
/// load, and returns; the diff locks ARMv7-M's `BUSFAULTENA` gating and the
/// `BusFault` priority/frame against the M3.
#[test]
fn busfault_precise_taken_as_busfault() {
    let Some(mt) = qemu() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.vec(5, "bf"); // BusFault vector
    a.vec(3, "hf"); // HardFault fallback

    // Enable the distinct BusFault.
    a.mov_imm32(0, 0xE000_ED24); // SHCSR
    a.mov_imm32(1, 0x0002_0000); // BUSFAULTENA
    a.str_imm(1, 0, 0);

    // The faulting load: an address in the peripheral window that no device
    // claims on this machine (reserved -> precise BusFault).
    a.mov_imm32(1, 0x5000_0000);
    a.ldr_imm(0, 1, 0);

    a.label("halt");
    a.b("halt");

    a.label("bf");
    a.mov_imm32(4, 0xE000_ED20);
    a.ldr_imm(5, 4, 8); // r5 = CFSR (BFSR byte must have PRECISERR)
    a.ldr_imm(6, 4, 12); // r6 = HFSR (must be 0, no FORCED)
    a.mov_reg(1, 13);
    a.ldr_imm(2, 1, 24);
    a.adds_imm8(2, 2);
    a.str_imm(2, 1, 24);
    a.bx(14);

    a.label("hf");
    a.b("halt");

    conformance("busfault_precise_taken_as_busfault", &mut a, "halt", &mt);
}

// ---------------------------------------------------------------------------
// Cortex-M7 FPv5-D16 double-precision: vmov.f64 #imm, vadd.f64, vcvt
// between single and double, and vrint. Diffed against cortex-m7
// (mps2-an500) with fpu log. Skipped if qemu-system-arm lacks the board.
// ---------------------------------------------------------------------------

#[test]
fn m7_f64() {
    let Some(mt) = qemu_m7() else { return };
    let mut a = Asm::new();
    sync_flags(&mut a);
    a.enable_fpu();
    // Seed s0=1.5f via GPR->S
    // `raw32` takes the half-swapped word (first halfword in the low bits),
    // exactly like the crate's own encoders (`w.rotate_left(16)`).
    a.mov_imm32(0, 0x3FC00000);
    a.raw32(0xEE00_0A10_u32.rotate_left(16)); // vmov s0, r0
    a.mov_imm32(0, 0x40200000);
    a.raw32(0xEE01_0A10_u32.rotate_left(16)); // vmov s1, r0
                                              // Seed doubles d1=1.5, d2=2.5 via GPR pair -> D
    a.mov_imm32(0, 0x00000000);
    a.mov_imm32(1, 0x3FF80000);
    a.raw32(0xEC41_0B11_u32.rotate_left(16)); // vmov d1, r0, r1
    a.mov_imm32(0, 0x00000000);
    a.mov_imm32(1, 0x40040000);
    a.raw32(0xEC41_0B12_u32.rotate_left(16)); // vmov d2, r0, r1
    a.raw32(0xEE31_0B02_u32.rotate_left(16)); // vadd.f64 d0, d1, d2
    a.raw32(0xEEB7_0AC0_u32.rotate_left(16)); // vcvt.f64.f32 d0, s0
    a.raw32(0xEEB7_0BC0_u32.rotate_left(16)); // vcvt.f32.f64 s0, d0
    a.raw32(0xEEB7_1B00_u32.rotate_left(16)); // vmov.f64 d1, #1.0
    a.raw32(0xEEB6_2B40_u32.rotate_left(16)); // vrintr.f64 d2, d0
    a.label("halt");
    a.b("halt");
    conformance_v7em("m7_f64", &mut a, "halt", &mt);
}

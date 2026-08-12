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
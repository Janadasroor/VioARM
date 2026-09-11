//! Phase-1 differential suite (P1-6): TB-quanta execution must match the
//! accurate stepper instruction-for-instruction on timing-insensitive
//! firmware: same registers, same flags, same RAM/code bytes, same
//! `cycles`/`cyccnt`. Workloads spin (like `fast_mode.rs`) so no
//! halt-alignment is needed; every block boundary the runner takes, the
//! stepper takes one step at a time, and both must converge cycle-exact.
//!
//! Coverage: ALU + flags + conditional branches, an IT block inside a hot
//! loop (heavy fallback-path exercise), loads/stores/multiple/push/pop,
//! BL calls with PUSH/POP-pc returns, an SVC round-trip through a vector
//! table (exception entry + `just_returned` return path), and firmware
//! self-modifying code plus host (DMA-analog) pokes between quanta.

#![cfg(feature = "jit")]

use vioarm_core::cpu::registers::{Registers, Xpsr};
use vioarm_core::error::HaltCause;
use vioarm_core::jit::{QuantumEnd, TbRunner};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_8000;
const STORE: u32 = 0x2000_0800;

/// 16-bit `B from -> to`.
fn b(from: u32, to: u32) -> u16 {
    let off = to as i32 - (from as i32 + 4);
    assert_eq!(off & 1, 0, "misaligned branch");
    assert!((-2048..=2046).contains(&off), "branch out of range");
    0xE000 | ((off >> 1) as u16 & 0x07FF)
}

/// 16-bit `B<cond> from -> to` (`cond` raw, e.g. `1` for NE).
fn bcond(cond: u8, from: u32, to: u32) -> u16 {
    let off = to as i32 - (from as i32 + 4);
    assert_eq!(off & 1, 0, "misaligned branch");
    assert!((-256..=254).contains(&off), "cond branch out of range");
    0xD000 | (u16::from(cond) << 8) | (off >> 1) as u16 & 0x00FF
}

/// 32-bit `BL from -> to` (small forward only, matching the decoder's
/// `S:imm10:I1:I2:imm11` split; `bl(x, x + 4) == (0xF000, 0xF800)`).
fn bl(from: u32, to: u32) -> (u16, u16) {
    let off = to as i32 - (from as i32 + 4);
    assert!(off >= 0 && off & 1 == 0, "backward/odd BL");
    let h0 = 0xF000 | ((off >> 12) as u16 & 0x03FF);
    let h1 = 0xF800 | ((off >> 1) as u16 & 0x07FF);
    (h0, h1)
}

/// 32-bit `B.W from -> to` (inverted from `branch_off25`; self-checked
/// against the decoder's vectors below).
fn bw(from: u32, to: u32) -> (u16, u16) {
    let off = to.wrapping_sub(from.wrapping_add(4)) as i32;
    assert_eq!(off & 1, 0, "misaligned B.W");
    let f = off as u32 & 0x1FF_FFFF;
    let (s, i10, i1, i2, i11) = (
        (f >> 24) & 1,
        (f >> 14) & 0x3FF,
        (f >> 13) & 1,
        (f >> 12) & 1,
        (f >> 1) & 0x7FF,
    );
    let (j1, j2) = (i1 ^ s ^ 1, i2 ^ s ^ 1);
    (
        0xF000 | ((s << 10) as u16) | i10 as u16,
        0x9000 | ((j1 << 13) as u16) | ((j2 << 11) as u16) | i11 as u16,
    )
}

/// 32-bit `B<cond>.W from -> to` (T3; same inversion with the `imm6`
/// field and the condition in the first halfword).
fn bcond_w(cond: u8, from: u32, to: u32) -> (u16, u16) {
    assert!(cond <= 13, "B<c>.W has no AL/NV form");
    let off = to.wrapping_sub(from.wrapping_add(4)) as i32;
    assert_eq!(off & 1, 0, "misaligned B<c>.W");
    let f = off as u32 & 0x1F_FFFF;
    let (s, i1, i2, i6, i11) = (
        (f >> 20) & 1,
        (f >> 19) & 1,
        (f >> 18) & 1,
        (f >> 12) & 0x3F,
        (f >> 1) & 0x7FF,
    );
    let (j1, j2) = (i1 ^ s ^ 1, i2 ^ s ^ 1);
    (
        0xF000 | ((s << 10) as u16) | ((u16::from(cond)) << 6) | i6 as u16,
        0x8000 | ((j1 << 13) as u16) | ((j2 << 11) as u16) | i11 as u16,
    )
}

/// `TBB [rn, rm]` halves (verified against the decoder below).
fn tbb(rn: u8, rm: u8) -> (u16, u16) {
    (0xE8D0 | u16::from(rn), 0xF000 | u16::from(rm))
}

/// `MOVW rd, #imm16` halves (`imm = imm4:i:imm3:imm8` per the decoder:
/// `imm4 = `h0[3:0]``, `i = `h0[10]``, `imm3 = `h1[14:12]``; verified below).
fn movw(rd: u8, imm: u16) -> (u16, u16) {
    let i = (imm >> 11) & 1;
    let imm4 = (imm >> 12) & 0xF;
    let imm3 = (imm >> 8) & 0x7;
    let imm8 = imm & 0xFF;
    (
        0xF240 | (i << 10) | imm4,
        (u16::from(rd) << 8) | (imm3 << 12) | imm8,
    )
}

#[test]
fn encoding_helpers_match_decoder_vectors() {
    use vioarm_core::cpu::decode32::{decode32_v7m, Inst32};
    // TBB/TBH + MOVW helpers reproduce the decoder's own vectors (proves
    // the helpers, not the emitter).
    assert_eq!(tbb(2, 5), (0xE8D2, 0xF005));
    assert_eq!(
        decode32_v7m(tbb(2, 5).0, tbb(2, 5).1),
        Ok(Inst32::TblBranch {
            rn: 2,
            rm: 5,
            half: false
        })
    );
    assert_eq!(movw(9, 0x1234), (0xF241, 0x2934));
    assert_eq!(
        decode32_v7m(movw(9, 0x1234).0, movw(9, 0x1234).1),
        Ok(Inst32::MovWide {
            rd: 9,
            imm: 0x1234,
            high: false
        })
    );
    assert_eq!(movw(0, 0), (0xF240, 0x0000));
}

#[test]
fn wide_branch_helpers_match_decoder_vectors() {
    // B.W +10 / -12 / +16384 / +16380 and BEQ.W +8, straight from the
    // decoder's own tests (proves the helpers above, not the emitter).
    assert_eq!(bw(0x6, 0x6 + 4 + 10), (0xF000, 0xB805));
    assert_eq!(bw(0xA, 0xA + 4 - 12), (0xF7FF, 0xBFFA));
    assert_eq!(bw(0x0, 0x4004), (0xF001, 0xB800));
    assert_eq!(bw(0x0, 0x4000), (0xF000, 0x97FE));
    assert_eq!(bcond_w(0, 0x6, 0x6 + 4 + 8), (0xF000, 0xA804));
}

fn halves(code: &[u16]) -> Vec<u8> {
    code.iter().flat_map(|h| h.to_le_bytes()).collect()
}

/// Full comparable architectural state plus RAM/code windows.
#[derive(Debug, PartialEq, Eq)]
struct Snap {
    regs: Registers,
    xpsr: Xpsr,
    ipsr: u8,
    primask: bool,
    basepri: u8,
    faultmask: bool,
    control: u32,
    itstate: u8,
    sleeping: bool,
    cycles: u64,
    cyccnt: u64,
    ram: Vec<u8>,
    code: Vec<u8>,
}

fn snap(m: &mut Machine<FlatMemory>, code_len: u32) -> Snap {
    let mem = m.core.mem();
    let mut ram = Vec::new();
    for i in 0..0x20 {
        ram.push(mem.read(STORE + i).unwrap_or(0xAA));
    }
    let mut code = Vec::new();
    for i in 0..code_len {
        code.push(mem.read(ENTRY + i).unwrap_or(0xAA));
    }
    Snap {
        regs: m.core.regs,
        xpsr: m.core.xpsr,
        ipsr: m.core.ipsr,
        primask: m.core.primask,
        basepri: m.core.basepri,
        faultmask: m.core.faultmask,
        control: m.core.control,
        itstate: m.core.itstate,
        sleeping: m.core.sleeping,
        cycles: m.core.cycles,
        cyccnt: m.core.cyccnt,
        ram,
        code,
    }
}

fn boot(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    m.core.mem().write_word(FLASH, INIT_SP).unwrap();
    m.core.mem().write_word(FLASH + 4, ENTRY | 1).unwrap();
    m.reset().expect("reset from vector table");
    m.core.regs.r[1] = STORE;
    m
}

/// Drive the stepper exactly `steps` steps, then drive quanta until at
/// least as many ops retired; step the stepper to identical `cycles`
/// (reachable: the same deterministic op sequence with identical per-op
/// accounting), and compare full snapshots.
fn check(code: &[u8], code_len: u32, steps: u64) {
    check_with(code, code_len, steps, |_| {});
}

fn check_with(code: &[u8], code_len: u32, steps: u64, setup: impl Fn(&mut Machine<FlatMemory>)) {
    check_mode(code, code_len, steps, &setup, false);
    check_mode(code, code_len, steps, &setup, true);
}

/// Drive stepper vs quanta and compare. Interpreted mode (`native =
/// false`) is cycle-exact including `cyccnt`; native mode (`true`) keeps
/// retired-op `cycles` and all architectural state exact while `cyccnt`
/// takes block estimates (D7) — so `cyccnt` is compared only in
/// interpreted mode.
fn check_mode(
    code: &[u8],
    code_len: u32,
    steps: u64,
    setup: &dyn Fn(&mut Machine<FlatMemory>),
    native: bool,
) {
    let mut a = boot(code);
    setup(&mut a);
    for _ in 0..steps {
        a.step().unwrap();
    }
    let mut b = boot(code);
    setup(&mut b);
    let mut runner = TbRunner::new();
    runner.set_native(native);
    let mut ops = 0u64;
    while ops < steps {
        ops += b.run_tb_quantum(&mut runner, 16).unwrap().ops;
    }
    while a.core.cycles < b.core.cycles {
        a.step().unwrap();
    }
    assert_eq!(a.core.cycles, b.core.cycles, "cycle convergence");
    let sa = snap(&mut a, code_len);
    let sb = snap(&mut b, code_len);
    assert_eq!(sa.regs, sb.regs, "regs");
    assert_eq!(sa.xpsr, sb.xpsr, "xpsr");
    assert_eq!(sa.ipsr, sb.ipsr, "ipsr");
    assert_eq!(sa.primask, sb.primask, "primask");
    assert_eq!(sa.basepri, sb.basepri, "basepri");
    assert_eq!(sa.faultmask, sb.faultmask, "faultmask");
    assert_eq!(sa.control, sb.control, "control");
    assert_eq!(sa.itstate, sb.itstate, "itstate");
    assert_eq!(sa.sleeping, sb.sleeping, "sleeping");
    assert_eq!(sa.cycles, sb.cycles, "cycles");
    assert_eq!(sa.ram, sb.ram, "ram");
    assert_eq!(sa.code, sb.code, "code");
    if !native {
        assert_eq!(sa.cyccnt, sb.cyccnt, "cyccnt convergence");
    }
}

/// ALU + flags + conditional branches + an IT block in a hot loop (the IT
/// body runs interpreted via fallback every iteration).
fn workload_alu() -> Vec<u8> {
    // Layout (ENTRY-relative): 3xMOVS, loop: ADDS/SUBS/MULS/SUBS-r3/IT/MOVEQ/BNE, STR, B.
    let e = ENTRY;
    let mut c = vec![
        0x2000, // MOVS r0, #0
        0x2203, // MOVS r2, #3
        0x23C8, // MOVS r3, #200
        // loop:
        0x3023, // ADDS r0, #0x23
        0x3811, // SUBS r0, #0x11
        0x4350, // MULS r0, r2
        0x3B01, // SUBS r3, #1
        0xBF08, // IT EQ
        0x245A, // MOVEQ r4, #0x5A (last iteration only, Z set)
                // BNE loop, STR r0, [r1], B .
    ];
    let bne_at = e + c.len() as u32 * 2;
    c.push(bcond(1, bne_at, e + 6));
    c.push(0x6008); // STR r0, [r1]
    let b_at = e + c.len() as u32 * 2;
    c.push(b(b_at, b_at)); // B .
    halves(&c)
}

#[test]
fn alu_loop_with_it_matches() {
    let code = workload_alu();
    let code_len = code.len() as u32;
    // Functional self-check on the stepper side first (proves the workload
    // does what the encodings claim): 200 trips, last sets r4.
    let mut m = boot(&code);
    for _ in 0..1500 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[4], 0x5A, "MOVEQ fired on final trip");
    drop(m);
    check(&code, code_len, 2000);
}

/// Loads/stores/multiple/push/pop with a MOVW literal (functionally
/// asserted: RAM must read back `BEEF BEEF`).
fn workload_mem() -> Vec<u8> {
    let c = vec![
        0xF64B, 0x62EF, // MOVW r2, #0xBEEF
        0x600A, // STR r2, [r1] (Rt = bits[2:0] = r2)
        0x780B, // LDRB r3, [r1]
        0x704B, // STRB r3, [r1, #1]
        0x880C, // LDRH r4, [r1]
        0x804C, // STRH r4, [r1, #2] (imm5 = 1, halfword offsets scale x2)
        0xC90C, // LDMIA r1!, {r2, r3}
        0xC00C, // STMIA r0!, {r2, r3} (r0 = STORE + 4, set by harness)
        0xB430, // PUSH {r4, r5}
        0xBC30, // POP {r4, r5}
        0xE7FE, // B .
    ];
    halves(&c)
}

#[test]
fn memory_ops_match() {
    let code = workload_mem();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    m.core.regs.r[0] = STORE + 4;
    m.step().unwrap();
    assert_eq!(m.core.regs.r[2], 0xBEEF, "MOVW literal");
    for _ in 0..10 {
        m.step().unwrap();
    }
    // STRB/LDRH/STRH fold the slot to all-EF before LDMIA reloads it.
    let mem = m.core.mem();
    assert_eq!(
        (0..4)
            .map(|i| mem.read(STORE + i).unwrap())
            .collect::<Vec<_>>(),
        vec![0xEF, 0xEF, 0xEF, 0xEF],
        "RAM self-consistency"
    );
    assert_eq!(m.core.regs.r[2], 0xEFEF_EFEF, "LDMIA reload");
    drop(m);
    // r0 (STMIA base) must be set identically on both sides.
    check_with(&code, code_len, 60, |m| {
        m.core.regs.r[0] = STORE + 4;
    });
}

/// BL calls with PUSH {lr} / POP {pc} returns.
fn workload_calls() -> Vec<u8> {
    let e = ENTRY;
    // main: MOVS r0,#0; BL inc; BL inc; STR r0,[r1]; B .
    // inc: PUSH {lr}; ADDS r0,#1; POP {pc}.
    let bl1_at = e + 2;
    let bl2_at = e + 6;
    let inc_at = e + 14;
    let (b1h, b1l) = bl(bl1_at, inc_at);
    let (b2h, b2l) = bl(bl2_at, inc_at);
    let b_at = e + 12;
    halves(&[
        0x2000, // MOVS r0, #0
        b1h,
        b1l, // BL inc
        b2h,
        b2l,           // BL inc
        0x6008,        // STR r0, [r1]
        b(b_at, b_at), // B .
        0xB500,        // inc: PUSH {lr}
        0x3001,        // ADDS r0, #1
        0xBD00,        // POP {pc}
    ])
}

#[test]
fn calls_match() {
    let code = workload_calls();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..20 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 2, "both calls retired");
    drop(m);
    check(&code, code_len, 60);
}

/// BL calls with `BX lr` returns (P2-4): the call AND the return execute
/// natively, exercising the dynamic link edge in both directions.
fn workload_bx_calls() -> Vec<u8> {
    let e = ENTRY;
    // main: MOVS r0,#0; BL inc; BL inc; STR r0,[r1]; B .
    // inc: ADDS r0,#1; BX lr.
    let bl1_at = e + 2;
    let bl2_at = e + 6;
    let inc_at = e + 14;
    let (b1h, b1l) = bl(bl1_at, inc_at);
    let (b2h, b2l) = bl(bl2_at, inc_at);
    let b_at = e + 12;
    halves(&[
        0x2000, // MOVS r0, #0
        b1h,
        b1l, // BL inc
        b2h,
        b2l,           // BL inc
        0x6008,        // STR r0, [r1]
        b(b_at, b_at), // B .
        0x3001,        // inc: ADDS r0, #1
        0x4770,        // BX lr
    ])
}

#[test]
fn bx_calls_match() {
    let code = workload_bx_calls();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..20 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 2, "both BX-returns retired");
    drop(m);
    check(&code, code_len, 60);
}

/// `BNE.W` countdown loop plus a `B.W` forward skip (P2-4 wide forms).
fn workload_wide() -> Vec<u8> {
    let e = ENTRY;
    // MOVS r2,#8; loop: SUBS r2,#1; BNE.W loop; MOVS r0,#1; B.W skip;
    // MOVS r0,#2 (skipped); skip: STR r0,[r1]; B .
    let loop_at = e + 2;
    let bne_at = e + 4;
    let (bneh, bnel) = bcond_w(1, bne_at, loop_at);
    let bw_at = e + 10;
    let skip_at = e + 16;
    let (bwh, bwl) = bw(bw_at, skip_at);
    halves(&[
        0x2208, // MOVS r2, #8
        0x3A01, // loop: SUBS r2, #1
        bneh,
        bnel,   // BNE.W loop
        0x2001, // MOVS r0, #1
        bwh,
        bwl,               // B.W skip
        0x2002,            // MOVS r0, #2 (skipped)
        0x6008,            // skip: STR r0, [r1]
        b(e + 18, e + 18), // B .
    ])
}

#[test]
fn wide_branches_match() {
    let code = workload_wide();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..40 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 1, "B.W skipped the MOVS #2");
    assert_eq!(m.core.regs.r[2], 0, "countdown drained");
    drop(m);
    check(&code, code_len, 120);
}

/// IT-block loop (P2-5, S4): predicated ADDS fires only on EQ, then the
/// countdown exits. The IT setter + predicated body go native together.
fn workload_it() -> Vec<u8> {
    let e = ENTRY;
    // MOVS r0,#0; MOVS r2,#3; loop: ADDS r0,#1; SUBS r2,#1; IT EQ;
    // ADDEQ r0,r0; BNE loop; STR r0,[r1]; B .
    let loop_at = e + 4;
    let bne_at = e + 12;
    halves(&[
        0x2000, // MOVS r0, #0
        0x2203, // MOVS r2, #3
        0x3001, // loop: ADDS r0, #1
        0x3A01, // SUBS r2, #1
        0xBF08, // IT EQ
        0x1800, // ADDEQ r0, r0
        bcond(1, bne_at, loop_at),
        0x6008, // STR r0, [r1]
        b(e + 16, e + 16),
    ])
}

#[test]
fn it_blocks_match() {
    let code = workload_it();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..24 {
        m.step().unwrap();
    }
    // 3 iterations (+1 each) plus the EQ-fired double on drain: 3*2 = 6.
    assert_eq!(m.core.regs.r[0], 6, "predicated double fired once");
    drop(m);
    check(&code, code_len, 80);
}

/// TBB dispatch (P2-5): table-driven jump to a MOVS + store tail.
fn workload_tbb() -> Vec<u8> {
    let e = ENTRY;
    // MOVS r0,#0; TBB [pc, r2] (table at e+8); table byte; target at
    // e+8+2*idx: MOVS r0,#7; STR r0,[r1]; B .
    let (t0, t1) = tbb(15, 2);
    halves(&[
        0x2000, // MOVS r0, #0
        t0,
        t1,     // TBB [pc, r2] (table base pc+4 = e+6)
        0xBF02, // table index 2 (low) + padding (high)
        0xBF00, // padding (skipped by dispatch)
        0x2007, // target (e+6+2*2 = e+10): MOVS r0, #7
        0x6008, // STR r0, [r1]
        b(e + 14, e + 14),
    ])
}

#[test]
fn tbb_dispatch_matches() {
    let code = workload_tbb();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[2] = 0;
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..8 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 7, "dispatched to the MOVS #7 arm");
    drop(m);
    check_with(&code, code_len, 40, setup);
}

/// MOVW immediates + native PUSH/POP calls in one loop (S5 + S3).
fn workload_movw_calls() -> Vec<u8> {
    let e = ENTRY;
    // main: MOVW r0,#0x1234 (top half zeroed); BL inc; STR r0,[r1]; B .
    // inc: PUSH {lr}; ADDS r0,#1; POP {pc}.
    let bl_at = e + 4;
    let inc_at = e + 12;
    let (b1h, b1l) = bl(bl_at, inc_at);
    let (w0h, w0l) = movw(0, 0x1234);
    let b_at = e + 10;
    halves(&[
        w0h,
        w0l, // MOVW r0, #0x1234
        b1h,
        b1l,           // BL inc
        0x6008,        // STR r0, [r1]
        b(b_at, b_at), // B .
        0xB500,        // inc: PUSH {lr}
        0x3001,        // ADDS r0, #1
        0xBD00,        // POP {pc}
    ])
}

#[test]
fn movw_push_pop_calls_match() {
    let code = workload_movw_calls();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..16 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 0x1235, "MOVW materialized + inc returned");
    drop(m);
    check(&code, code_len, 60);
}

/// Chained side exit (P2-6 `cur_pc`): a page-crossing word load
/// side-exits on EVERY execution (the guard refuses cross-page host
/// adjacency even warm) while its block is chained into — the driver must
/// commit the faulting block's prefix and attribute the exit to it, not
/// to the entered block. Both pages are genuinely mapped (the
/// interpreter performs the access: no fault, no writes, stable
/// generation, so the edges really do chain).
fn workload_crossloop() -> Vec<u8> {
    let e = ENTRY;
    // A: B loop; loop: LDR r3,[r1,#0]; SUBS r2,#1; BNE A; STR r0,[r1];
    // B . — r1 aims at 0x20000FFD (word straddles into 0x20001xxx).
    halves(&[
        b(e, e + 2), // A: B loop
        0x680B,      // loop: LDR r3, [r1, #0]
        0x3A01,      // SUBS r2, #1
        bcond(1, e + 6, e),
        0x6008, // STR r0, [r1]
        b(e + 10, e + 10),
    ])
}

#[test]
fn chained_cross_page_side_exit_matches() {
    let code = workload_crossloop();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.regs.r[1] = 0x2000_0FFD;
        m.core.regs.r[2] = 4;
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..24 {
        m.step().unwrap();
    }
    drop(m);
    check_with(&code, code_len, 80, setup);
}

/// Sleep/wake latches (P2-6): `SEV` arms the event, the first `WFE`
/// consumes it, the second sleeps; `WFI` always sleeps. Native stores
/// drive the same latches the stepper does.
#[test]
fn wfi_wfe_sev_match() {
    // SEV; WFE (consumes); WFE (sleeps); B .
    let e = ENTRY;
    let code = halves(&[0xBF40, 0xBF20, 0xBF20, b(e + 6, e + 6)]);
    let code_len = code.len() as u32;
    let mut a = boot(&code);
    for _ in 0..3 {
        a.step().unwrap();
    }
    assert!(a.core.sleeping, "second WFE sleeps (stepper)");
    assert!(!a.core.event, "event consumed");
    let mut b = boot(&code);
    let mut runner = TbRunner::new();
    let mut ops = 0u64;
    let end = loop {
        let q = b.run_tb_quantum(&mut runner, 4).unwrap();
        ops += q.ops;
        if q.end != QuantumEnd::Budget || ops > 16 {
            break q.end;
        }
    };
    assert_eq!(end, QuantumEnd::Sleep);
    assert!(b.core.sleeping);
    assert!(!b.core.event);
    assert_eq!(b.core.regs.pc, a.core.regs.pc);
    assert!(
        runner.stats().native > 0,
        "the WFI block ran natively, not via fallback"
    );
    let _ = code_len;
}

/// Lock/unlock dance (P2-6): `CPSID i`, `MSR PRIMASK`, `MRS` reads, and
/// `CPSIE i` around a counter, all converging stepper-vs-quanta.
fn workload_locks() -> Vec<u8> {
    let e = ENTRY;
    // loop: CPSID i; ADDS r0,#1; MRS r3, PRIMASK; CPSIE i; SUBS r2,#1;
    // BNE loop; STR r0,[r1]; B .
    // CPSID i = B672; CPSIE i = B662; MRS r3, PRIMASK = F3EF 8310.
    let loop_at = e;
    let bne_at = e + 12;
    halves(&[
        0xB672, // loop: CPSID i
        0x3001, // ADDS r0, #1
        0xF3EF,
        0x8310, // MRS r3, PRIMASK
        0xB662, // CPSIE i
        0x3A01, // SUBS r2, #1
        bcond(1, bne_at, loop_at),
        0x6008, // STR r0, [r1]
        b(e + 16, e + 16),
    ])
}

#[test]
fn locks_match() {
    let code = workload_locks();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.regs.r[2] = 4;
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..40 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 4, "counter retired under locks");
    assert!(!m.core.primask, "unlocked at the end");
    drop(m);
    check_with(&code, code_len, 120, setup);
}

/// `MRS` under handler mode and unprivileged thread (P2-6): IPSR reads
/// live, unprivileged reads RAZ.
#[test]
fn mrs_modes_match() {
    // MRS r0, IPSR (sys 5); MRS r1, CONTROL (sys 0x14); MRS r2, MSP
    // (sys 8); B . — sys numbers per the decoder's `SysReg::from`.
    let code = halves(&[0xF3EF, 0x8005, 0xF3EF, 0x8114, 0xF3EF, 0x8208]);
    let code_len = code.len() as u32;
    // Handler mode (ipsr set): live values, no RAZ.
    let setup_handler = |m: &mut Machine<FlatMemory>| {
        m.core.ipsr = 8;
    };
    check_with(&code, code_len, 12, setup_handler);
    // Unprivileged thread (nPRIV, no handler): IPSR/MSP RAZ, CONTROL live.
    let setup_unpriv = |m: &mut Machine<FlatMemory>| {
        m.core.control = 1;
    };
    let mut m = boot(&code);
    setup_unpriv(&mut m);
    for _ in 0..4 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 0, "IPSR RAZ unprivileged");
    assert_eq!(m.core.regs.r[2], 0, "MSP RAZ unprivileged");
    drop(m);
    check_with(&code, code_len, 12, setup_unpriv);
}

/// Exclusive spinlock shape (P2-6): `LDREX`/`STREX` retry loop, side-exit
/// tails with the monitor owned by the interpreter. (`CBNZ` cannot loop
/// backward, so the retry tests the STREX status with `CMP`+`BNE`.)
fn workload_excl() -> Vec<u8> {
    let e = ENTRY;
    // loop: LDREX r0,[r1]; ADDS r0,#1; STREX r3,r0,[r1]; CMP r3,#0;
    // BNE loop; STR r0,[r1]; B .
    // LDREX r0,[r1] = E851 0F00; STREX r3,r0,[r1] = E841 0300
    // (decoder vectors: `strex r1, r0, [r2]` = E842 0100).
    let loop_at = e;
    let bne_at = e + 12;
    halves(&[
        0xE851,
        0x0F00, // loop: LDREX r0, [r1]
        0x3001, // ADDS r0, #1
        0xE841,
        0x0300, // STREX r3, r0, [r1]
        0x2C00, // CMP r3, #0
        bcond(1, bne_at, loop_at),
        0x6008, // STR r0, [r1]
        b(e + 16, e + 16),
    ])
}

#[test]
fn exclusives_match() {
    let code = workload_excl();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.mem().write_word(STORE, 41).unwrap();
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..16 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 42, "exclusive increment retired");
    drop(m);
    check_with(&code, code_len, 60, setup);
}

/// FP around native code (P2-6): `VADD` side-exits every touch (the
/// interpreter owns the FP context), but the surrounding integer blocks
/// stay native — values round-trip through `VMOV` and must match bit-exactly.
fn workload_fp() -> Vec<u8> {
    use vioarm_core::cpu::decode32::{decode32_v7m, Inst32, Vfp3Op};
    // `vadd.f32 s0, s0, s1` + `vmov s1, r0` (derived in-test,
    // decoder-verified: fail-loud if the derivation slips).
    assert_eq!(
        decode32_v7m(0xEE30, 0x0A20),
        Ok(Inst32::Vfp3 {
            op: Vfp3Op::Vadd,
            vd: 0,
            vn: 0,
            vm: 1,
            is_dp: false,
        })
    );
    assert_eq!(
        decode32_v7m(0xEE00, 0x0A90),
        Ok(Inst32::VfpMov {
            to_gp: false,
            vd: 1,
            rt: 0,
        })
    );
    let e = ENTRY;
    // MOVS r0,#1; VMOV S0,r0; VMOV S1,r0; VADD S0,S0,S1; VMOV r0,S0;
    // STR r0,[r1]; B .
    halves(&[
        0x2001, // MOVS r0, #1
        0xEE00,
        0x0A10, // VMOV S0, r0
        0xEE00,
        0x0A90, // VMOV S1, r0
        0xEE30,
        0x0A20, // VADD.F32 S0, S0, S1
        0xEE10,
        0x0A10, // VMOV r0, S0
        0x6008, // STR r0, [r1] (e+18)
        b(e + 20, e + 20),
    ])
}

#[test]
fn fp_around_native_matches() {
    let code = workload_fp();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..16 {
        m.step().unwrap();
    }
    // 0x00000001 + 0x00000001 as f32 denormals doubles the mantissa.
    assert_eq!(m.core.regs.r[0], 2, "fp round-trips bit-exactly");
    drop(m);
    check(&code, code_len, 60);
}

/// `BX` to an even target halts identically (stepper `Err` vs runner
/// `Err` — the native `HALT` exit reports `NonThumbTarget`).
#[test]
fn bx_even_target_halts_identically() {
    // Single BX r2 at ENTRY with r2 even.
    let code = halves(&[0x4710]); // BX r2
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[2] = 0x2000_0000;
    };
    let mut a = boot(&code);
    setup(&mut a);
    let ea = a.step().unwrap_err();
    assert_eq!(ea, HaltCause::NonThumbTarget);
    let mut b = boot(&code);
    setup(&mut b);
    let mut runner = TbRunner::new();
    let eb = b.run_tb_quantum(&mut runner, 16).unwrap_err();
    assert_eq!(eb, HaltCause::NonThumbTarget);
    // Untouched architectural state on both sides (prefix-free block).
    let sa = snap(&mut a, code_len);
    let sb = snap(&mut b, code_len);
    assert_eq!(sa.regs, sb.regs, "regs");
    assert_eq!(sa.cycles, sb.cycles, "no cycles charged for the trap");
}

/// Hot loop must actually chain (P2-4): driver-bypassed executions show
/// up in `stats().chained`, with state identical to the stepper.
#[test]
fn hot_loop_chains() {
    let code = workload_memloop();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.regs.r[2] = 8;
        let mem = m.core.mem();
        for i in 0..8u32 {
            mem.write_word(0x2000_0800 + 4 * i, i + 1).unwrap();
        }
    };
    let mut b = boot(&code);
    setup(&mut b);
    let mut runner = TbRunner::new();
    let mut ops = 0u64;
    while ops < 200 {
        ops += b.run_tb_quantum(&mut runner, 64).unwrap().ops;
    }
    assert!(
        runner.stats().chained > 0,
        "hot loop must bypass the driver"
    );
    assert_eq!(b.core.regs.r[0], 36, "1+..+8 accumulated");
    drop(b);
    check_with(&code, code_len, 200, setup);
}

/// Patching a linked target mid-run executes the new bytes (P2-4): links
/// break on the write generation, the target rebuilds, state converges
/// with a patched-from-the-start stepper run.
#[test]
fn patch_breaks_links() {
    // Loop: MOVS r0,#1; SUBS r2,#1; BNE loop; STR; B . — patch the MOVS
    // to #9 after links form, then re-run from the top with the cache
    // (and stale links) intact: only fresh bytes may execute.
    let e = ENTRY;
    let loop_at = e;
    let bne_at = e + 4;
    let code = halves(&[
        0x2001, // loop: MOVS r0, #1
        0x3A01, // SUBS r2, #1
        bcond(1, bne_at, loop_at),
        0x6008, // STR r0, [r1]
        b(e + 8, e + 8),
    ]);
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.regs.r[2] = 3;
    };
    // Phase 1: converge links on the unpatched image.
    let mut b = boot(&code);
    setup(&mut b);
    let mut runner = TbRunner::new();
    let mut ops = 0u64;
    while ops < 64 {
        ops += b.run_tb_quantum(&mut runner, 16).unwrap().ops;
    }
    assert!(runner.stats().chained > 0, "links formed first");
    // Phase 2: patch the body, reset registers (NOT the cache), re-run.
    b.core.mem().write(ENTRY, 0x09).unwrap();
    b.core.regs.pc = ENTRY;
    b.core.regs.r[0] = 0;
    b.core.regs.r[2] = 3;
    let mut ops = 0u64;
    while ops < 64 {
        ops += b.run_tb_quantum(&mut runner, 16).unwrap().ops;
    }
    assert_eq!(
        b.core.regs.r[0], 9,
        "patched bytes executed, no stale links"
    );
    // Reference: patched image from the start, stepped + native alike.
    let mut patched = code.clone();
    patched[0] = 0x09;
    let plen = patched.len() as u32;
    check_with(&patched, plen, 64, setup);
    // And the re-run state itself converges with the patched reference.
    let mut r = boot(&patched);
    setup(&mut r);
    for _ in 0..64 {
        r.step().unwrap();
    }
    assert_eq!(b.core.regs.r[0], r.core.regs.r[0]);
    assert_eq!(b.core.regs.r[2], r.core.regs.r[2]);
}

#[test]
fn svc_round_trip_matches() {
    let e = ENTRY;
    let handler = e + 0x100;
    let b_at = e + 4;
    let code = halves(&[0xDF00, 0xBF00, b(b_at, b_at)]);
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.mem().write_word(0x2C, handler | 1).unwrap(); // SVC vector
        let mem = m.core.mem();
        mem.write(handler, 0x70).unwrap(); // BX LR
        mem.write(handler + 1, 0x47).unwrap();
        mem.write(handler + 2, 0x00).unwrap(); // NOP
        mem.write(handler + 3, 0xBF).unwrap();
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..12 {
        m.step().unwrap();
    }
    assert_eq!(m.core.ipsr, 0, "returned to thread");
    assert_eq!(m.core.regs.pc, b_at, "spinning after return");
    drop(m);
    // Both modes (native uses estimates for `cyccnt`, excluded there).
    // Small quanta would land boundaries inside the entry/return window;
    // `check_with` uses 16-op quanta, still crossing several boundaries
    // over the 12-step run including the just_returned skip path.
    check_with(&code, code_len + 0x104, 12, setup);
}

/// Firmware self-modifying code: a STRH patches the loop's first slot,
/// then execution re-enters it (checksum invalidation + rebuild).
fn workload_smc() -> Vec<u8> {
    let e = ENTRY;
    assert_eq!(e & 3, 0);
    halves(&[
        0x2000,         // slot: MOVS r0, #0 (rewritten to MOVS r0, #2)
        0x2204,         // MOVS r2, #4
        0x0092,         // LSLS r2, r2, #2 (r2 = 0x10 = slot)
        0x4902,         // LDR r1, [pc, #8] (-> E + 0x10)
        0x8011,         // STRH r1, [r2, #0]
        b(e + 0x0A, e), // B slot
        0xBF00,         // padding
        0xBF00,         // padding
        0x2002,         // literal halfword: new slot content (low half of .word)
        0x0000,         // literal high halfword
    ])
}

/// Fixed-trip array-sum loop: native loads/stores/branches end to end
/// (r1 = array base from the harness, r2 = trip count, r0 accumulates).
fn workload_memloop() -> Vec<u8> {
    let e = ENTRY;
    // loop: LDR r3,[r1,#0]; ADDS r0,r0,r3; ADDS r1,#4; SUBS r2,#1;
    // BNE loop; STR r0,[r1]; B .
    let loop_at = e;
    let bne_at = e + 8;
    halves(&[
        0x680B, // LDR r3, [r1, #0]
        0x18C0, // ADDS r0, r0, r3
        0x3104, // ADDS r1, #4
        0x3A01, // SUBS r2, #1 (Rd = r2, not r3!)
        bcond(1, bne_at, loop_at),
        0x6008, // STR r0, [r1]
        b(e + 12, e + 12),
    ])
}

#[test]
fn firmware_memloop_matches() {
    let code = workload_memloop();
    let code_len = code.len() as u32;
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.regs.r[0] = 0;
        m.core.regs.r[2] = 8;
        let mem = m.core.mem();
        for i in 0..8u32 {
            mem.write_word(0x2000_0800 + 4 * i, i + 1).unwrap();
        }
    };
    let mut m = boot(&code);
    setup(&mut m);
    for _ in 0..60 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 36, "1+..+8 accumulated");
    drop(m);
    check_with(&code, code_len, 200, setup);
}

#[test]
fn firmware_smc_matches() {
    let code = workload_smc();
    let code_len = code.len() as u32;
    let mut m = boot(&code);
    for _ in 0..18 {
        m.step().unwrap();
    }
    assert_eq!(m.core.regs.r[0], 2, "patched slot executed");
    drop(m);
    check(&code, code_len, 60);
}

/// Host (DMA-analog) writes between quanta, cycle-matched on both sides:
/// the full M5 source mix — firmware STRH plus host pokes — must converge.
#[test]
fn host_pokes_between_quanta_match() {
    let code = workload_smc();
    let code_len = code.len() as u32;
    let mut a = boot(&code);
    let mut b = boot(&code);
    let mut runner = TbRunner::new();
    // Interpreted mode: this test proves invalidation interleaving with
    // full `cyccnt` equality (native state parity rides on `check_mode`).
    runner.set_native(false);
    // Advance the burst side first (it overshoots atomically), converge the
    // stepper onto its exact cycle count (reachable: B's cycle trajectory
    // is a subsequence of A's), poke the slot identically, repeat. The
    // firmware's own STRH rewrites the slot every trip, so pokes only stick
    // until the next rewrite — the snapshots prove both sides agree at
    // every sync point, including on the slot bytes themselves.
    for k in 1..=4u32 {
        let target = b.core.cycles.max(a.core.cycles) + 12;
        while b.core.cycles < target {
            b.run_tb_quantum(&mut runner, 4).unwrap();
        }
        while a.core.cycles < b.core.cycles {
            a.step().unwrap();
        }
        assert_eq!(a.core.cycles, b.core.cycles);
        assert_eq!(snap(&mut a, code_len), snap(&mut b, code_len));
        let patch = 0x2000 | (10 + k);
        for m in [&mut a, &mut b] {
            let mem = m.core.mem();
            mem.write(ENTRY, (patch & 0xFF) as u8).unwrap();
            mem.write(ENTRY + 1, ((patch >> 8) & 0xFF) as u8).unwrap();
        }
    }
    // Settle without pokes: the firmware STRH re-establishes MOVS r0, #2 on
    // both sides alike.
    let target = b.core.cycles.max(a.core.cycles) + 12;
    while b.core.cycles < target {
        b.run_tb_quantum(&mut runner, 4).unwrap();
    }
    while a.core.cycles < b.core.cycles {
        a.step().unwrap();
    }
    assert_eq!(a.core.cycles, b.core.cycles);
    assert_eq!(snap(&mut a, code_len), snap(&mut b, code_len));
    assert_eq!(a.core.regs.r[0], 2);
    assert_eq!(b.core.regs.r[0], 2);
}

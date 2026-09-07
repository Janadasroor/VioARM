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
use vioarm_core::jit::TbRunner;
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

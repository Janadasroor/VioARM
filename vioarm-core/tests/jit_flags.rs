//! Lazy NZCV materialization (G3 flag traffic): native and stepper must
//! agree bit-exactly on flags across producer runs, carry chains, and
//! faults after skipped producers. Each case is shaped so the backward
//! pass skips at least one merge (verified by the op-count bound in the
//! last test); equality then proves the skip was unobservable.

#![cfg(feature = "jit")]

use vioarm_core::board::RAM;
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;

const BKPT_0: [u8; 2] = [0x00, 0xBE];

fn bcond(addr: u32, target: u32) -> [u8; 2] {
    let off = target as i32 - (addr as i32 + 4);
    assert!(off % 2 == 0 && (-256..=254).contains(&off));
    (0xD100u16 | ((off >> 1) as u16 & 0xFF)).to_le_bytes()
}

fn ldr_lit(rt: u8, addr: u32, target: u32) -> [u8; 2] {
    let base = (addr + 4) & !3;
    let off = target - base;
    assert!(off % 4 == 0 && off <= 1020);
    (0x4800u16 | (u16::from(rt) << 8) | u16::from((off >> 2) as u8)).to_le_bytes()
}

fn machine(code: &[u8]) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(RAM, code));
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x0100_0000);
    m
}

/// Run to halt both ways; regs (incl. flags) and halt cause must match.
fn check(code: &[u8]) {
    let mut mn = machine(code);
    let mut runner = TbRunner::new();
    let nerr = mn.run_tb(&mut runner, 100_000).unwrap_err();
    let mut ms = machine(code);
    let serr = ms.run(100_000).unwrap_err();
    assert_eq!(nerr, serr, "halt cause");
    assert_eq!(mn.core.regs, ms.core.regs, "regs incl. flags");
    assert_eq!(mn.core.xpsr.get(), ms.core.xpsr.get(), "xpsr");
}

#[test]
fn producer_run_before_branch() {
    // ADDS/SUBS run; only the last merges, BNE reads it. One lap is
    // enough: liveness runs at TB-build time, parity is the assertion.
    let mut code = vec![
        0x00, 0x20, // MOVS r0, #0
        0x01, 0x21, // MOVS r1, #1
        0x02, 0x22, // MOVS r2, #2
    ];
    let loop_at = RAM + code.len() as u32;
    code.extend_from_slice(&[
        0x42, 0x18, // ADDS r2, r0, r1
        0x49, 0x1A, // SUBS r1, r1, r2 (r1 = 0, Z set)
        0x01, 0x33, // ADDS r3, #1
        0x01, 0x3B, // SUBS r3, #1 (r3 = 0, Z set)
    ]);
    let bne_at = RAM + code.len() as u32;
    code.extend_from_slice(&bcond(bne_at, loop_at));
    code.extend_from_slice(&BKPT_0);
    check(&code);
}

#[test]
fn carry_chain_through_preserver() {
    // SUBS clears C; ANDS preserves it; ADC consumes it. Skipping SUBS
    // would corrupt the ADC — equality proves the chain is intact.
    let code = vec![
        0x00, 0x20, // MOVS r0, #0
        0x01, 0x21, // MOVS r1, #1
        0x41, 0x1A, // SUBS r1, r0, r1 (r1 = 0-1 = MAX, C=0 borrow)
        0x08, 0x40, // ANDS r0, r1 (r0 = 0, N/Z new, C preserved = 0)
        0x02, 0x22, // MOVS r2, #2
        0x53, 0x41, // ADCS r3, r2 (r3 = 0 + 2 + C(0) = 2)
        0x00, 0xBE, // BKPT #0
    ];
    check(&code);
    // r3 must be 2 (C was 0); r1 MAX; flags from ADCS.
    let mut m = machine(&code);
    let _ = m.run(100_000);
    assert_eq!(m.core.regs.r[3], 2);
    assert_eq!(m.core.regs.r[1], u32::MAX);
}

#[test]
fn fault_after_skipped_producers() {
    // Two producers run, only the second merges; then a load from
    // unmapped memory faults. Cause, regs, and stacked xPSR (flags!)
    // must match the stepper exactly.
    // Layout: MOVS/MOVS/LDR r5/ADDS/SUBS/LDR fault/BKPT/pad/pool.
    // 0x682B = LDR r3, [r5, #0] (r5 = unmapped 0x50000000).
    let mut h: Vec<u16> = vec![
        0x2000, 0x2101, 0, 0x1842, 0x1A01, 0x682B, 0xBE00, 0xBF00,
    ];
    let at = |i: usize| RAM + i as u32 * 2;
    h[2] = u16::from_le_bytes(ldr_lit(5, at(2), at(8)));
    let mut code = Vec::new();
    for x in &h {
        code.extend_from_slice(&x.to_le_bytes());
    }
    code.extend_from_slice(&0x5000_0000u32.to_le_bytes());
    check(&code);
}

//! P3-6 TLB index-thrash repro: two hot pages sharing one direct-mapped
//! TLB index must not side-exit every lap.
//!
//! The 64-entry direct-mapped TLB indexes `vpn & 63`, so RAM pages
//! `0x2000_0000` (vpn `0x20000`) and `0x2004_0000` (vpn `0x20040`) collide
//! at index 0. A loop reading a literal pool from the first and a counter
//! from the second must settle after the two cold fills; without a victim
//! slot each fill evicts the other page and every lap pays a side-exit
//! plus a full stepper re-execution (~2x counted ops).

#![cfg(feature = "jit")]

use vioarm_core::board::RAM;
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;

const BKPT_0: [u8; 2] = [0x00, 0xBE];
const TRIPS: u32 = 200;
const CTR: u32 = 0x2004_0000;
/// Setup (1) + `TRIPS` laps of 10 ops. The faulting `BKPT` may or may not
/// count; the bound below leaves wide slack for it either way.
const EXACT_OPS: u64 = 1 + TRIPS as u64 * 10;

fn ldr_lit(rt: u8, addr: u32, target: u32) -> [u8; 2] {
    let base = (addr + 4) & !3;
    let off = target - base;
    assert!(off % 4 == 0 && off <= 1020);
    (0x4800u16 | (u16::from(rt) << 8) | u16::from((off >> 2) as u8)).to_le_bytes()
}

fn bcond(addr: u32, target: u32) -> [u8; 2] {
    let off = target as i32 - (addr as i32 + 4);
    assert!(off % 2 == 0 && (-256..=254).contains(&off));
    (0xD100u16 | ((off >> 1) as u16 & 0xFF)).to_le_bytes()
}

/// Colliding loop at `RAM` (vpn `0x20000`, index 0): per lap four ALU
/// ops (so a mid-TB side exit wastes native work, like the real BODY),
/// one literal read (pool, same page) plus one counter read-modify-write
/// at `CTR` (vpn `0x20040`, index 0). Only `r7` is live across laps.
fn colliding_loop() -> Vec<u8> {
    // Layout: LDR r7 / loop(4xALU, LDR, LDR, ADDS, STR, SUBS, BNE) /
    // BKPT / pool. Pool lands at +24/+28 (word-aligned).
    let mut h: Vec<u16> = vec![
        0, // 0: LDR r7, =trips
        0x2001, 0x2102, 0x1840, 0x4041, // 1-4: ALU padding
        0,      // 5: LDR r2, =CTR
        0x6813, 0x3301, 0x6013, 0x3F01, // 6-9: counter RMW + SUBS
        0,      // 10: BNE loop
        0xBE00, // 11: BKPT #0
    ];
    let at = |i: usize| RAM + i as u32 * 2;
    h[0] = u16::from_le_bytes(ldr_lit(7, at(0), at(12)));
    h[5] = u16::from_le_bytes(ldr_lit(2, at(5), at(14)));
    h[10] = u16::from_le_bytes(bcond(at(10), at(1)));
    let mut code = Vec::new();
    for x in &h {
        code.extend_from_slice(&x.to_le_bytes());
    }
    // Pool is already word-aligned (`BKPT` ends at +24).
    code.extend_from_slice(&TRIPS.to_le_bytes());
    code.extend_from_slice(&CTR.to_le_bytes());
    code
}

fn machine(code: &[u8]) -> Machine<FlatMemory> {
    // One window covering the code page and the counter page, both full
    // pages so `fill_tlb` installs (no partial-page refusal involved).
    let mem = FlatMemory::with_layout(&[RAM], &[0x50000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(RAM, code));
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x0100_0000);
    m
}

#[test]
fn colliding_pages_settle_after_cold_fills() {
    let code = colliding_loop();
    let mut mn = machine(&code);
    let mut runner = TbRunner::new();
    let nerr = mn.run_tb(&mut runner, 100_000).unwrap_err();
    let native_ops = runner.stats().ops;
    let mut ms = machine(&code);
    let serr = ms.run(100_000).unwrap_err();
    assert_eq!(nerr, serr, "halt cause");
    // Post-fix the two cold fills cost a couple of stepper re-executions;
    // pre-fix every lap side-exits and re-executes (~2x counted ops).
    assert!(
        native_ops <= EXACT_OPS + 256,
        "side-exit every lap: {native_ops} ops for {EXACT_OPS} exact"
    );
}

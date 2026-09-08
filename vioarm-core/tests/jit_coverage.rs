//! X4 coverage under JIT: `Coverage` fed by instruction hooks sees the
//! same retired PCs natively as accurately (hooked blocks fall back per
//! X1, so coverage never misses translated code).

#![cfg(feature = "jit")]

use std::cell::RefCell;
use std::rc::Rc;
use vioarm_core::hooks::{Coverage, HookEvent};
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const RAM: u32 = 0x2000_0000;

/// CBNZ r0, taken; MOVS r1,#1; loop: ADDS r1,#1; SUBS r0,#1; BNE loop;
/// B . — exercises covered + skipped + looped blocks under translation.
fn prog() -> Vec<u16> {
    vec![
        0xB904, // CBNZ r0, +4 -> MOVS r1 (r0 != 0)
        0x2000, // MOVS r0, #0 (skipped)
        0x2101, // MOVS r1, #1
        0x3101, // ADDS r1, #1
        0x3801, // SUBS r0, #1
        0xD1FC, // BNE loop (0x06)
        0xE7FE, // B .
    ]
}

fn machine(r0: u32) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x1000]);
    let mut m = Machine::armv7m(mem);
    let bytes: Vec<u8> = prog().iter().flat_map(|h| h.to_le_bytes()).collect();
    m.load_image(&load_flat(RAM, &bytes));
    m.core.regs.pc = RAM;
    m.core.xpsr.set(0x01000000);
    m.core.regs.r[0] = r0;
    m
}

fn cover(m: &mut Machine<FlatMemory>, pcs: &[u32]) -> (Vec<u32>, Vec<bool>) {
    let mut cov = Coverage::new();
    for pc in pcs {
        cov.record(*pc);
    }
    let mem = m.core.mem();
    let mut fetch = |addr: u32| {
        let lo = mem.read(addr).ok()?;
        let hi = mem.read(addr.wrapping_add(1)).ok()?;
        Some(u16::from_le_bytes([lo, hi]))
    };
    cov.analyze(&mut fetch)
}

fn run_jit(r0: u32, ops: u64) -> (Vec<u32>, Machine<FlatMemory>, u64) {
    let mut m = machine(r0);
    let cov = Rc::new(RefCell::new(Coverage::new()));
    let sink = cov.clone();
    assert!(m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            sink.borrow_mut().record(pc);
        }
    }));
    let mut runner = TbRunner::new();
    let mut done = 0u64;
    while done < ops {
        done += m.run_tb_quantum(&mut runner, 32).unwrap().ops;
    }
    // The registry keeps its own clone; read through ours.
    let out = cov.borrow().pcs();
    (out, m, done)
}

fn run_acc(r0: u32, ops: u64) -> (Vec<u32>, Machine<FlatMemory>) {
    let mut m = machine(r0);
    let mut cov = Coverage::new();
    let pcs = Rc::new(RefCell::new(Vec::new()));
    let sink = pcs.clone();
    assert!(m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            sink.borrow_mut().push(pc);
        }
    }));
    for _ in 0..ops {
        m.step().unwrap();
    }
    for pc in pcs.borrow().iter() {
        cov.record(*pc);
    }
    (cov.pcs(), m)
}

#[test]
fn coverage_matches_in_jit_mode() {
    // r0 = 3: CBNZ taken, else skipped, 3 loop laps, then spin.
    // Identical retired-op counts on both engines (native overshoot
    // included: the stepper walks the native total).
    let (cj, mut mj, done) = run_jit(3, 200);
    let (ca, mut ma) = run_acc(3, done);
    assert_eq!(cj, ca, "covered pcs");
    assert_eq!(cover(&mut mj, &cj), cover(&mut ma, &ca), "analysis");
    assert_eq!(mj.core.regs, ma.core.regs, "regs");
}

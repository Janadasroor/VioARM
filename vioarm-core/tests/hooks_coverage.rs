//! Hook registry e2e on a real machine: pre/post-instruction hooks,
//! watchpoints, peripheral events, and block coverage over a hand-assembled
//! branchy program (all encodings verified with `arm-none-eabi-as`).

use std::cell::RefCell;
use std::rc::Rc;
use vioarm_core::hooks::{Coverage, HookEvent};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const RAM: u32 = 0x2000_0000;

// 0x00: CBNZ r0, 0x06 ; 0x02: B 0x08 ; 0x04: MOVS r0, #1 (never runs) ;
// 0x06: MOVS r0, #2   ; 0x08: STR r0, [r1] ; 0x0A: BKPT #0.
fn program() -> Vec<u8> {
    [
        0xB908u16, // CBNZ r0, 0x06 (assembler-verified)
        0xE001,    // B 0x08
        0x2001,    // MOVS r0, #1 (uncovered else)
        0x2002,    // MOVS r0, #2
        0x6008,    // STR r0, [r1]
        0xBE00,    // BKPT #0
    ]
    .iter()
    .flat_map(|h| (*h).to_le_bytes())
    .collect()
}

fn machine(r0: u32) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x1000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(RAM, &program()));
    m.core.regs.pc = RAM;
    m.core.xpsr.set(0x0100_0000);
    m.core.regs.r[0] = r0;
    m.core.regs.r[1] = RAM + 0x100;
    m
}

#[test]
fn pre_and_post_hooks_see_every_retired_pc() {
    let mut m = machine(0);
    let pre = Rc::new(RefCell::new(Vec::new()));
    let post = Rc::new(RefCell::new(Vec::new()));
    let s1 = pre.clone();
    let s2 = post.clone();
    assert!(m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            s1.borrow_mut().push(pc);
        }
    }));
    assert!(m.on_instruction(false, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            s2.borrow_mut().push(pc);
        }
    }));
    loop {
        if m.step().is_err() {
            break;
        }
    }
    // r0 == 0: fall through to B, skip the else, store, halt. The BKPT
    // fires pre (about to retire) but never post (it halts, retires nothing).
    assert_eq!(*pre.borrow(), [RAM, RAM + 2, RAM + 8, RAM + 0xA]);
    assert_eq!(*post.borrow(), [RAM, RAM + 2, RAM + 8]);
}

#[test]
fn hooks_honor_pc_ranges() {
    let mut m = machine(1);
    let hits = Rc::new(RefCell::new(Vec::new()));
    let sink = hits.clone();
    assert!(m.on_instruction(true, RAM + 4, RAM + 8, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            sink.borrow_mut().push(pc);
        }
    }));
    loop {
        if m.step().is_err() {
            break;
        }
    }
    // r0 == 1: CBNZ jumps to 0x06; only that PC is in range.
    assert_eq!(*hits.borrow(), [RAM + 6]);
}

#[test]
fn write_watchpoint_sees_the_store() {
    let mut m = machine(0);
    let writes = Rc::new(RefCell::new(Vec::new()));
    let sink = writes.clone();
    assert!(m.on_write(RAM + 0x100, RAM + 0x104, move |e| {
        if let HookEvent::MemWrite { addr, value } = *e {
            sink.borrow_mut().push((addr, value));
        }
    }));
    loop {
        if m.step().is_err() {
            break;
        }
    }
    // r0 == 0 skips both MOVS, so STR stores r0 == 0, one event per byte.
    assert_eq!(
        *writes.borrow(),
        [
            (RAM + 0x100, 0),
            (RAM + 0x101, 0),
            (RAM + 0x102, 0),
            (RAM + 0x103, 0)
        ],
        "one event per byte, little-endian"
    );
}

#[test]
fn block_coverage_marks_the_else_uncovered() {
    let mut m = machine(0);
    let mut cov = Coverage::new();
    let pcs = Rc::new(RefCell::new(Vec::new()));
    let sink = pcs.clone();
    assert!(m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            sink.borrow_mut().push(pc);
        }
    }));
    loop {
        if m.step().is_err() {
            break;
        }
    }
    for pc in pcs.borrow().iter() {
        cov.record(*pc);
    }
    let mem = m.core.mem();
    let mut fetch = |addr: u32| {
        let lo = mem.read(addr).ok()?;
        let hi = mem.read(addr.wrapping_add(1)).ok()?;
        Some(u16::from_le_bytes([lo, hi]))
    };
    let (leaders, covered) = cov.analyze(&mut fetch);
    assert_eq!(
        leaders,
        [RAM, RAM + 2, RAM + 4, RAM + 6, RAM + 8],
        "entry, B fall-through, else, CBNZ target, post-B"
    );
    assert_eq!(covered, [true, true, false, false, true]);
}

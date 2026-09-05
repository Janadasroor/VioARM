//! Block coverage over a branchy firmware, driven by hooks.
//!
//! Host-side example: runs a hand-assembled if/else program on a bare
//! machine, records every retired PC with a pre-instruction hook, and
//! reconstructs basic blocks with [`Coverage`](vioarm_core::hooks::Coverage).
//! Uncovered blocks (here: the untaken `else`) are listed at the end.
//!
//! ```sh
//! cargo run -p vioarm-core --example coverage
//! ```

use std::cell::RefCell;
use std::rc::Rc;
use vioarm_core::hooks::{Coverage, HookEvent};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const RAM: u32 = 0x2000_0000;

fn main() {
    // 0x00: CBNZ r0, 0x06 ; 0x02: B 0x08 ; 0x04: MOVS r0, #1 (else) ;
    // 0x06: MOVS r0, #2   ; 0x08: STR r0, [r1] ; 0x0A: BKPT #0.
    // (Encodings verified with `arm-none-eabi-as`.)
    let prog: Vec<u8> = [0xB908u16, 0xE001, 0x2001, 0x2002, 0x6008, 0xBE00]
        .iter()
        .flat_map(|h| (*h).to_le_bytes())
        .collect();
    let mem = FlatMemory::with_layout(&[RAM], &[0x1000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(RAM, &prog));
    m.core.regs.pc = RAM;
    m.core.xpsr.set(0x0100_0000);
    m.core.regs.r[0] = 0; // take the fall-through: the else never runs
    m.core.regs.r[1] = RAM + 0x100;

    let pcs = Rc::new(RefCell::new(Vec::new()));
    let sink = pcs.clone();
    m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, .. } = *e {
            sink.borrow_mut().push(pc);
        }
    });
    loop {
        if m.step().is_err() {
            break;
        }
    }

    let mut cov = Coverage::new();
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
    let total = leaders.len();
    let hit = covered.iter().filter(|c| **c).count();
    println!("covered {hit}/{total} blocks");
    for (leader, yes) in leaders.iter().zip(covered.iter()) {
        println!("  {:#010x} {}", leader, if *yes { "hit" } else { "MISSED" });
    }
}

//! Fast-mode differential suite (M11): timing-insensitive workloads must
//! finish byte-identical (RAM, `cycles`, `cyccnt`) in accurate and fast
//! modes, fast reruns must match, and interrupt delivery must never be
//! early with bounded lateness.

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const RAM: u32 = 0x2000_0000;

/// Run `steps` steps from `pc`, ignoring halts (workloads spin, never halt).
fn run(m: &mut Machine<FlatMemory>, steps: u64) {
    for _ in 0..steps {
        let _ = m.step();
    }
}

/// Snapshot RAM bytes plus both counters.
fn snap_flat(m: &mut Machine<FlatMemory>, len: u32) -> (Vec<u8>, u64, u64) {
    let mem = m.core.mem();
    let mut bytes = Vec::new();
    for i in 0..len {
        bytes.push(mem.read(RAM + 0x800 + i).unwrap_or(0xAA));
    }
    (bytes, m.cycles(), m.core.cyccnt)
}

fn flat_machine(code: &[u8]) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x1000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(RAM, code));
    m.core.regs.pc = RAM;
    m.core.xpsr.set(0x0100_0000);
    m
}

/// NOP sled with a final store (control workload).
fn workload_nop() -> (Vec<u8>, u32) {
    let mut code = Vec::new();
    for _ in 0..200 {
        code.extend_from_slice(&[0x00, 0xBF]);
    }
    // MOVS r0, #0x5A; STR r0, [r1]; B . (r1 set by the harness)
    code.extend_from_slice(&[0x5A, 0x20, 0x08, 0x60, 0xFE, 0xE7]);
    (code, 0x2000_0800)
}

/// ALU torture: running checksum over immediates into RAM.
fn workload_alu() -> (Vec<u8>, u32) {
    // MOVS r0,#0; loop: ADDS r0,#0x11; EORS r0,r1... kept simple:
    // 8x (ADDS r0, #0x23; SUBS r0, #0x11; MULS r0, r2) with r2 = 3,
    // then STR r0, [r1]; B .
    let mut code = vec![0x00, 0x20, 0x03, 0x22]; // MOVS r0,#0; MOVS r2,#3
    for _ in 0..8 {
        code.extend_from_slice(&[0x23, 0x30]); // ADDS r0, #0x23
        code.extend_from_slice(&[0x11, 0x38]); // SUBS r0, #0x11
        code.extend_from_slice(&[0x50, 0x43]); // MULS r0, r2
    }
    code.extend_from_slice(&[0x08, 0x60, 0xFE, 0xE7]); // STR; B .
    (code, 0x2000_0800)
}

/// Exercise one workload under several quanta; all must agree exactly.
fn check_differential(code: &[u8], ram_base: u32, ram_len: u32) {
    let mut reference = None;
    for quantum in [0, 1, 8, 32, 128] {
        for _ in 0..2 {
            let mut m = flat_machine(code);
            m.core.regs.r[1] = ram_base;
            m.set_fast_quantum(quantum);
            run(&mut m, 2_000);
            let got = snap_flat(&mut m, ram_len);
            if let Some(prev) = reference.clone() {
                assert_eq!(got, prev, "quantum {quantum} diverged");
            } else {
                reference = Some(got);
            }
        }
    }
}

#[test]
fn fast_modes_match_accurate_exactly() {
    let (nop, _) = workload_nop();
    check_differential(&nop, 0x2000_0800, 8);
    let (alu, _) = workload_alu();
    check_differential(&alu, 0x2000_0800, 8);
}

#[test]
fn board_timers_converge_but_events_shift() {
    // TIM2 free-runs while the core spins NOPs: final CNT converges
    // (debt preserves totals), but the UIF latch appears on boundaries.
    const TIM2: u32 = 0x4000_0000;
    let setup = |m: &mut Machine<BoardMemory>| {
        // Park the core on a NOP sled in RAM (fault-spinning would tangle
        // exception paths into the measurement).
        for (i, b) in [0x00u8, 0xBF, 0xFE, 0xE7].iter().enumerate() {
            m.core.mem().write(RAM + i as u32, *b).unwrap();
        }
        m.core.regs.pc = RAM;
        m.core.xpsr.set(0x0100_0000);
        let mem = m.core.mem();
        mem.write_word(TIM2 + 0x2C, 0xFF).unwrap(); // ARR
        mem.write_word(TIM2 + 0x28, 0).unwrap(); // PSC
        mem.write_word(TIM2 + 0x14, 1).unwrap(); // UG
        mem.write_word(TIM2 + 0x10, 0).unwrap(); // SR
        mem.write_word(TIM2, 1).unwrap(); // CEN
    };
    // Converged totals after 5k steps (settled: the tail quantum lands).
    let mut cnts = Vec::new();
    for quantum in [0, 1, 8, 32] {
        let mut m = Machine::armv7m(board::stm32f103());
        setup(&mut m);
        m.set_fast_quantum(quantum);
        for _ in 0..5_000 {
            let _ = m.step();
        }
        m.settle_fast();
        let mem = m.core.mem();
        cnts.push(mem.read_word(TIM2 + 0x24).unwrap());
    }
    assert_eq!(
        cnts,
        [cnts[0], cnts[0], cnts[0], cnts[0]],
        "all quanta converge"
    );
}

#[test]
fn interrupt_delivery_never_early_bounded_late() {
    // First step index with the TIM2 update line latched, accurate vs
    // fast: fast may lag up to one quantum, never lead.
    const TIM2: u32 = 0x4000_0000;
    fn first_pending(quantum: u64) -> u64 {
        let mut m = Machine::armv7m(board::stm32f103());
        {
            for (i, b) in [0x00u8, 0xBF, 0xFE, 0xE7].iter().enumerate() {
                m.core.mem().write(RAM + i as u32, *b).unwrap();
            }
            m.core.regs.pc = RAM;
            m.core.xpsr.set(0x0100_0000);
            let mem = m.core.mem();
            mem.write_word(TIM2 + 0x2C, 0x3F).unwrap();
            mem.write_word(TIM2 + 0x28, 0).unwrap();
            mem.write_word(TIM2 + 0x14, 1).unwrap();
            mem.write_word(TIM2 + 0x10, 0).unwrap();
            mem.write_word(TIM2 + 0x0C, 1).unwrap(); // DIER.UIE gates the line
            mem.write_word(TIM2, 1).unwrap();
        }
        m.set_fast_quantum(quantum);
        for step in 0..5_000u64 {
            let _ = m.step();
            let pending = m
                .core
                .mem()
                .devices()
                .iter()
                .any(|d| d.name() == "tim2" && d.pending() != 0);
            if pending {
                return step;
            }
        }
        panic!("UIF never latched");
    }
    let accurate = first_pending(0);
    for quantum in [8, 32] {
        let fast = first_pending(quantum);
        assert!(
            fast >= accurate,
            "fast delivery must never precede accurate (K={quantum})"
        );
        assert!(
            fast - accurate <= quantum,
            "lateness bounded by the quantum (K={quantum})"
        );
    }
}

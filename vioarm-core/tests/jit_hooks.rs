//! X1 per-range hook bypass: traces in JIT mode equal traces in accurate
//! mode event-for-event (pre/post-instruction PCs *and* `cycles`, plus
//! watchpoint bytes). Unhooked blocks stay native (asserted via stats);
//! overlapping ones fall back; arming mid-run fences stale linked edges.

#![cfg(feature = "jit")]

use std::cell::RefCell;
use std::rc::Rc;
use vioarm_core::hooks::HookEvent;
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const BASE: u32 = 0x2000_0000;
const ARRAY: u32 = 0x2000_1000;
const OUT: u32 = 0x2000_0200;
const FUNC: u32 = 0x2000_001C;

/// top: LDR r3,[r4]; MOVS r0,#0; MOVS r1,#1; MOVS r2,#10;
/// loop: ADDS r0,r0,r3; LDR r3,[r4]; ADDS r4,#4; SUBS r2,#1; BNE loop;
/// STR r0,[r5]; BL func; B top; func: ADDS r0,r0,r1; BX lr.
/// Layout rule (load-use hazards are a documented D7 gap: the stepper
/// charges a cross-block stall the block estimates don't): no block ends
/// in a single-register load — the LDRs sit mid-block (intra-block
/// stalls are exact on both engines) and every tail clears the tracker.
fn prog() -> Vec<u16> {
    vec![
        0x6823, 0x2000, 0x2101, 0x220A, 0x18C0, 0x6823, 0x3404, 0x3A01, 0xD1FA, 0x6028, 0xF000,
        0xF802, 0xE7F2, 0x0000, 0x1840, 0x4770,
    ]
}

fn machine() -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[BASE], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    let bytes: Vec<u8> = prog().iter().flat_map(|h| h.to_le_bytes()).collect();
    m.load_image(&load_flat(BASE, &bytes));
    m.core.regs.pc = BASE;
    m.core.xpsr.set(0x01000000);
    m.core.regs.r[4] = ARRAY;
    m.core.regs.r[5] = OUT;
    for i in 0..64u32 {
        m.core
            .mem()
            .write_word(ARRAY + 4 * i, 0x0102_0300 + i)
            .unwrap();
    }
    m
}

type Sink = Rc<RefCell<Vec<HookEvent>>>;

fn arm_all(m: &mut Machine<FlatMemory>, pre: &Sink, post: &Sink, rd: &Sink, wr: &Sink) {
    let (a, b, c, d) = (pre.clone(), post.clone(), rd.clone(), wr.clone());
    assert!(m.on_instruction(true, 0, u32::MAX, move |e| a.borrow_mut().push(*e)));
    assert!(m.on_instruction(false, 0, u32::MAX, move |e| b.borrow_mut().push(*e)));
    assert!(m.on_read(ARRAY, ARRAY + 0x100, move |e| c.borrow_mut().push(*e)));
    assert!(m.on_write(OUT, OUT + 4, move |e| d.borrow_mut().push(*e)));
}

fn sinks() -> (Sink, Sink, Sink, Sink) {
    (
        Rc::new(RefCell::new(Vec::new())),
        Rc::new(RefCell::new(Vec::new())),
        Rc::new(RefCell::new(Vec::new())),
        Rc::new(RefCell::new(Vec::new())),
    )
}

/// Drive native quanta and the stepper the same op count; compare event
/// streams plus end state. Returns native blocks executed (bypass check).
fn check(
    native_ops: u64,
    arm: impl Fn(&mut Machine<FlatMemory>, &Sink, &Sink, &Sink, &Sink),
) -> u64 {
    let (pa, qa, ra, wa) = sinks();
    let mut mn = machine();
    arm(&mut mn, &pa, &qa, &ra, &wa);
    let mut runner = TbRunner::new();
    let mut ops = 0u64;
    while ops < native_ops {
        ops += mn.run_tb_quantum(&mut runner, 64).unwrap().ops;
    }
    let (pb, qb, rb, wb) = sinks();
    let mut ms = machine();
    arm(&mut ms, &pb, &qb, &rb, &wb);
    for _ in 0..ops {
        ms.step().unwrap();
    }
    assert_eq!(*pa.borrow(), *pb.borrow(), "pre-instruction trace");
    assert_eq!(*qa.borrow(), *qb.borrow(), "post-instruction trace");
    assert_eq!(*ra.borrow(), *rb.borrow(), "read watch trace");
    assert_eq!(*wa.borrow(), *wb.borrow(), "write watch trace");
    assert_eq!(mn.core.regs, ms.core.regs, "regs");
    assert_eq!(mn.core.xpsr, ms.core.xpsr, "xpsr");
    assert_eq!(mn.core.cycles, ms.core.cycles, "cycles");
    runner.stats().native
}

#[test]
fn full_hook_coverage_matches_and_falls_back() {
    // Everything hooked: every block overlaps, nothing goes native —
    // but the traces are still exact.
    let native = check(400, arm_all);
    assert_eq!(native, 0, "full overlap bypasses everything");
}

#[test]
fn partial_range_keeps_unhooked_blocks_native() {
    // Only the funclet is hooked: the hot loop stays native.
    let native = check(400, |m, pa, qa, ra, wa| {
        let a = pa.clone();
        let b = qa.clone();
        assert!(m.on_instruction(true, FUNC, FUNC + 4, move |e| a.borrow_mut().push(*e)));
        assert!(m.on_instruction(false, FUNC, FUNC + 4, move |e| b.borrow_mut().push(*e)));
        let _ = (ra, wa);
    });
    assert!(native > 0, "unhooked loop stays native");
}

#[test]
fn data_watch_only_keeps_alu_blocks_native() {
    // Watchpoints but no instruction hooks: load/store blocks fall back,
    // ALU/branch/call blocks stay native.
    let native = check(400, |m, _pa, _qa, ra, wa| {
        let c = ra.clone();
        let d = wa.clone();
        assert!(m.on_read(ARRAY, ARRAY + 0x100, move |e| c.borrow_mut().push(*e)));
        assert!(m.on_write(OUT, OUT + 4, move |e| d.borrow_mut().push(*e)));
    });
    assert!(native > 0, "alu blocks stay native under data watch");
}

#[test]
fn arming_mid_run_fences_stale_links() {
    // Warm up unhooked (links form and chain), then arm over everything
    // at the exact retired-op count on both machines: the native side
    // must stop executing covered blocks natively (link fence + overlap
    // gate), and the post-arm traces must still match exactly.
    let warm = 120u64;
    let post = 200u64;
    let (pa, qa, ra, wa) = sinks();
    let mut m = machine();
    let mut runner = TbRunner::new();
    let mut ops_warm = 0u64;
    while ops_warm < warm {
        ops_warm += m.run_tb_quantum(&mut runner, 64).unwrap().ops;
    }
    let native_before = runner.stats().native;
    assert!(native_before > 0, "warmup went native");
    arm_all(&mut m, &pa, &qa, &ra, &wa);
    let mut ops = ops_warm;
    while ops < ops_warm + post {
        ops += m.run_tb_quantum(&mut runner, 64).unwrap().ops;
    }
    assert_eq!(
        runner.stats().native,
        native_before,
        "no covered block runs natively after arming"
    );
    // Stepper reference armed at the same retired-op counts.
    let (pb, qb, rb, wb) = sinks();
    let mut ms = machine();
    for _ in 0..ops_warm {
        ms.step().unwrap();
    }
    arm_all(&mut ms, &pb, &qb, &rb, &wb);
    for _ in ops_warm..ops {
        ms.step().unwrap();
    }
    assert_eq!(*pa.borrow(), *pb.borrow(), "pre trace after late arm");
    assert_eq!(*qa.borrow(), *qb.borrow(), "post trace after late arm");
    assert_eq!(*ra.borrow(), *rb.borrow(), "read trace after late arm");
    assert_eq!(*wa.borrow(), *wb.borrow(), "write trace after late arm");
    assert_eq!(m.core.regs, ms.core.regs, "regs after late arm");
    assert_eq!(m.core.cycles, ms.core.cycles, "cycles after late arm");
}

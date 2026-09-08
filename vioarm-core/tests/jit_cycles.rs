//! P2-7 cycle-accounting tests: native `cyccnt` must match the stepper on
//! every statically-modelable shape (TRM base + intra-block stalls +
//! taken-branch deltas), and never step backward anywhere (E6
//! monotonicity — including `DWT_CYCCNT` reads from inside native code).
//!
//! Deliberate non-goals (documented D7 gaps, asserted nowhere): `DIV`
//! with a nonzero dividend (2c minimum baked), reg-relative data-region
//! latency, cross-block stalls, IT-suppressed overcounts.

#![cfg(feature = "jit")]

use vioarm_core::cpu::decode16::{decode16_v7m, Inst16};
use vioarm_core::cpu::decode32::{decode32_v7m, Inst32, MulOp};
use vioarm_core::cpu::Core;
use vioarm_core::device::dwt::Dwt;
use vioarm_core::device::Peripheral;
use vioarm_core::error::BusError;
use vioarm_core::jit::TbRunner;
use vioarm_core::machine::Machine;
use vioarm_core::mem::{Bus, DataTranslate, MpuAccess, PeripheralBus};

const CODE: u32 = 0x1000;
const RAM: u32 = 0x2000_0000;
const INIT_SP: u32 = 0x2000_8000;

/// Flat code+RAM bus with a stable config generation (so edges link) and
/// a tracked write generation (so content validates without re-hashing).
struct TestBus {
    code: Vec<u8>,
    ram: Vec<u8>,
    writes: u64,
}

impl TestBus {
    fn new(code_halves: &[u16]) -> Self {
        let mut bus = Self {
            code: vec![0; 0x1000],
            ram: vec![0; 0x10000],
            writes: 0,
        };
        for (i, half) in code_halves.iter().enumerate() {
            let [lo, hi] = half.to_le_bytes();
            bus.code[i * 2] = lo;
            bus.code[i * 2 + 1] = hi;
        }
        bus
    }
}

impl Bus for TestBus {
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        match addr {
            a if (CODE..CODE + 0x1000).contains(&a) => Ok(self.code[(a - CODE) as usize]),
            a if (RAM..RAM + 0x10000).contains(&a) => Ok(self.ram[(a - RAM) as usize]),
            _ => Err(BusError::Reserved),
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        match addr {
            a if (CODE..CODE + 0x1000).contains(&a) => self.code[(a - CODE) as usize] = val,
            a if (RAM..RAM + 0x10000).contains(&a) => self.ram[(a - RAM) as usize] = val,
            _ => return Err(BusError::Reserved),
        }
        self.writes = self.writes.wrapping_add(1);
        Ok(())
    }

    fn write_epoch_ptr(&mut self) -> Option<*mut u64> {
        Some(&mut self.writes)
    }

    fn translate(&mut self, addr: u32, _access: MpuAccess) -> DataTranslate {
        // Identity map, mirroring FlatMemory: whole pages only (the TLB
        // tags full pages, so partial coverage must refuse).
        let base: u32;
        let mem: &mut Vec<u8>;
        match addr {
            a if (CODE..CODE + 0x1000).contains(&a) => {
                base = CODE;
                mem = &mut self.code;
            }
            a if (RAM..RAM + 0x10000).contains(&a) => {
                base = RAM;
                mem = &mut self.ram;
            }
            _ => return DataTranslate::Indirect,
        }
        let page_start = addr & !0xFFF;
        if page_start < base {
            return DataTranslate::Indirect;
        }
        let off = page_start - base;
        let len = (mem.len() as u32).saturating_sub(off);
        if len < 0x1000 {
            return DataTranslate::Indirect;
        }
        DataTranslate::Direct {
            ptr: mem.as_mut_ptr().wrapping_add(off as usize),
            len,
        }
    }
}

impl PeripheralBus for TestBus {
    fn next_exception(
        &mut self,
        _primask: bool,
        _faultmask: bool,
        _basepri: u8,
        _current_ipsr: u8,
    ) -> Option<u8> {
        None
    }

    fn acknowledge_exception(&mut self, _n: u8) {}

    fn tick_cycle(&mut self) {}

    fn tick_by(&mut self, _delta: u64) {}

    fn mpu_check(
        &mut self,
        _addr: u32,
        _access: MpuAccess,
        _privileged: bool,
        _exception: u8,
    ) -> bool {
        true
    }

    fn config_epoch(&self) -> Option<u64> {
        Some(0)
    }

    fn write_epoch(&self) -> Option<u64> {
        Some(self.writes)
    }
}

fn core_at(code: &[u16]) -> Core<TestBus> {
    let mut core = Core::armv7m(TestBus::new(code));
    core.regs.pc = CODE;
    core.regs.sp = INIT_SP;
    core
}

/// One native block (after a warmup lap that builds it, fills the TLB,
/// and links its edges) must retire exactly what stepping its ops does:
/// same state, same `cycles`, same `cyccnt`.
fn check_case(name: &str, code: &[u16], setup: impl Fn(&mut Core<TestBus>), steps: usize) {
    let mut runner = TbRunner::new();
    let mut warm = core_at(code);
    setup(&mut warm);
    let _ = runner.run_block(&mut warm).unwrap().unwrap();

    let mut n = core_at(code);
    setup(&mut n);
    let outcome = runner.run_block(&mut n).unwrap().unwrap();
    assert_eq!(outcome.ops, steps, "native op count ({name})");

    let mut s = core_at(code);
    setup(&mut s);
    for _ in 0..steps {
        s.step().unwrap();
    }
    assert_eq!(n.regs, s.regs, "registers ({name})");
    assert_eq!(n.xpsr, s.xpsr, "flags ({name})");
    assert_eq!(n.cycles, s.cycles, "retired cycles ({name})");
    assert_eq!(n.cyccnt, s.cyccnt, "cyccnt ({name})");
}

#[test]
fn estimates_match_stepper_battery() {
    // ALU + NOP + always-taken `B .`: 1 + 1 + 3.
    check_case("alu", &[0x2001, 0xBF00, 0xE7FE], |c| {}, 3);
    // `MULS r0, r1` (0x4348): 1c data.
    check_case(
        "muls",
        &[0x4348, 0xE7FE],
        |c| {
            c.regs.r[0] = 7;
            c.regs.r[1] = 6;
        },
        2,
    );
    // `LDR r0, [pc, #0]` feeding `ADDS r1, r0, r0` (0x1841): 2 + (1+1
    // stall); the pool halfword decodes as a continuing `LSL`.
    check_case("stall", &[0x4800, 0x1841, 0x00FF, 0xE7FE], |_| {}, 4);
    // Same load, consumer reading elsewhere: no stall.
    check_case("no-stall", &[0x4800, 0x1849, 0x00FF, 0xE7FE], |_| {}, 4);
    // `POP {r0, r1}` (0xBC01): 1 + 2, no refill.
    check_case(
        "pop",
        &[0xBC01, 0xE7FE],
        |c| {
            c.mem().write_word(0x2000_7FF8, 0x1111_2222).unwrap();
            c.mem().write_word(0x2000_7FFC, 0x3333_4444).unwrap();
            c.regs.sp = 0x2000_7FF8;
        },
        2,
    );
    // `PUSH {r0, r1}` (0xB401): 1 + 2 store.
    check_case(
        "push",
        &[0xB401, 0xE7FE],
        |c| {
            c.regs.r[0] = 0xAAAA_BBBB;
            c.regs.r[1] = 0xCCCC_DDDD;
        },
        1,
    );
    // `LDM r0!, {r1, r2}` (0xC806): 1 + 2.
    check_case(
        "ldm",
        &[0xC806, 0xE7FE],
        |c| {
            c.mem().write_word(0x2000_0100, 0xDEAD_BEEF).unwrap();
            c.mem().write_word(0x2000_0104, 0xCAFE_F00D).unwrap();
            c.regs.r[0] = 0x2000_0100;
        },
        2,
    );
    // `CBZ r0, +6` (0xB118) taken: 1 + 2 delta.
    check_case(
        "cbz-taken",
        &[0xB118, 0xBF00, 0xBF00, 0xBF00, 0xBF00, 0xE7FE],
        |c| {
            c.regs.r[0] = 0;
        },
        1,
    );
    // Same `CBZ`, falling through: 1.
    check_case(
        "cbz-fall",
        &[0xB118, 0xBF00, 0xBF00, 0xBF00, 0xBF00, 0xBF00, 0xE7FE],
        |c| {
            c.regs.r[0] = 1;
        },
        1,
    );
    // `BNE .` (0xD1FE) taken lives in `bne_taken_chains_exactly` below
    // (a self-loop never terminates under default budgets).
    // Same `BNE`, falling (Z == 1): 1.
    check_case(
        "bne-fall",
        &[0xD1FE],
        |c| c.xpsr.set_nzcv(false, true, false, false),
        1,
    );
    // `BL` (4c) to a `NOP; BX LR` tail.
    check_case("bl", &[0xF000, 0xF802, 0xE7FE, 0xBF00, 0x4770], |_| {}, 1);
    // `SDIV` by zero side-exits natively but accounts exactly (2c).
    assert_eq!(
        decode32_v7m(0xFB91, 0xF0F2),
        Ok(Inst32::Div {
            signed: true,
            rd: 0,
            rn: 1,
            rm: 2,
        })
    );
    check_case(
        "sdiv-zero",
        &[0xFB91, 0xF0F2, 0xE7FE],
        |c| {
            c.regs.r[1] = 7;
            c.regs.r[2] = 0;
        },
        1,
    );
}

/// 32-bit multiply costs ride the estimate exactly (hand encodings pinned
/// against the decoder first).
#[test]
fn mul_costs_match() {
    assert_eq!(
        decode32_v7m(0xFB01, 0x3002),
        Ok(Inst32::Mul {
            op: MulOp::Mla,
            rd: 0,
            rn: 1,
            rm: 2,
            ra: 3,
        })
    );
    // `MLA r0, r1, r2, r3`: 2c.
    check_case(
        "mla",
        &[0xFB01, 0x3002, 0xE7FE],
        |c| {
            c.regs.r[1] = 3;
            c.regs.r[2] = 4;
            c.regs.r[3] = 5;
        },
        2,
    );
    assert_eq!(
        decode32_v7m(0xFB82, 0x0103),
        Ok(Inst32::Mul {
            op: MulOp::Smull,
            rd: 1,
            rn: 2,
            rm: 3,
            ra: 0,
        })
    );
    // `SMULL r1, r0, r2, r3` (RdHi=r1, RdLo=r0): 3c.
    check_case(
        "smull",
        &[0xFB82, 0x0103, 0xE7FE],
        |c| {
            c.regs.r[2] = 0x1_0000;
            c.regs.r[3] = 0x1_0000;
        },
        2,
    );
    // 16-bit CBZ encoding sanity (used above).
    assert_eq!(
        decode16_v7m(0xB118),
        Ok(Inst16::CbzCbnz {
            rn: 0,
            off: 6,
            nonzero: false,
        })
    );
}

/// A taken self-loop under tight budgets: two laps retire exactly 2 × 3c.
/// Both taken deltas arrive via edge traversals (the budget break itself
/// adds none — it falls through to the block's own PC).
#[test]
fn bne_taken_chains_exactly() {
    let code = [0xD1FE];
    let mut runner = TbRunner::new();
    let mut warm = core_at(&code);
    runner.set_chain_budgets(3, 2);
    let _ = runner.run_block(&mut warm).unwrap().unwrap();
    let mut n = core_at(&code);
    runner.set_chain_budgets(3, 2);
    let outcome = runner.run_block(&mut n).unwrap().unwrap();
    assert_eq!(outcome.ops, 2, "two laps before the budget break");
    // Entered estimate (1) + lap-1 edge +2 + lap-2 entry (1) + lap-2 edge +2.
    assert_eq!(n.cyccnt, 6);
    assert_eq!(n.cycles, 2);
    assert_eq!(n.regs.pc, CODE);
    let mut s = core_at(&code);
    for _ in 0..2 {
        s.step().unwrap();
    }
    assert_eq!(s.cyccnt, 6);
    assert_eq!(s.cycles, 2);
}

/// A nonzero `DIV` documents the 2c-minimum gap (interp pays 3 here).
#[test]
fn div_nonzero_documents_minimum() {
    let code = [0xFB91, 0xF0F2, 0xE7FE];
    let mut runner = TbRunner::new();
    let mut warm = core_at(&code);
    warm.regs.r[1] = 7;
    warm.regs.r[2] = 2;
    let _ = runner.run_block(&mut warm).unwrap().unwrap();
    let mut n = core_at(&code);
    n.regs.r[1] = 7;
    n.regs.r[2] = 2;
    let outcome = runner.run_block(&mut n).unwrap().unwrap();
    assert_eq!(outcome.ops, 2);
    let mut s = core_at(&code);
    s.regs.r[1] = 7;
    s.regs.r[2] = 2;
    for _ in 0..2 {
        s.step().unwrap();
    }
    assert_eq!(n.regs.r[0], 3, "quotient still exact");
    assert_eq!(n.cyccnt, 2 + 3, "baked minimum + taken B");
    assert_eq!(s.cyccnt, 3 + 3, "interp pays the data cost");
}

/// A taken loop through linked edges counts exactly: per lap the estimate
/// (fall-assumed) plus one edge +2; the final fall lap adds nothing.
/// `cyccnt` is sampled after every driver return and must never step back.
#[test]
fn taken_loop_counts_exactly_and_monotonically() {
    // `SUBS r0, #1; BNE back`.
    let code = [0x3801, 0xD1FD];
    let laps = 10u64;
    let mut runner = TbRunner::new();
    let mut warm = core_at(&code);
    warm.regs.r[0] = laps as u32;
    while runner.run_block(&mut warm).unwrap().is_some() {}

    let mut n = core_at(&code);
    n.regs.r[0] = laps as u32;
    let (mut prev_cyccnt, mut prev_cycles) = (n.cyccnt, n.cycles);
    let mut ops = 0usize;
    // Tight per-call chaining budgets (P2-4): each driver return retires
    // ~one lap, so the loop provably lands on r0 == 0 instead of chaining
    // hundreds of laps past it within a single call (correct either way —
    // but then unobservable here). This also exercises budget-break
    // exits and re-entry accounting every lap or two.
    for _ in 0..64 {
        runner.set_chain_budgets(4, 2);
        let Some(outcome) = runner.run_block(&mut n).unwrap() else {
            break;
        };
        ops += outcome.ops;
        assert!(n.cyccnt >= prev_cyccnt, "cyccnt stepped back");
        assert!(n.cycles >= prev_cycles, "cycles stepped back");
        (prev_cyccnt, prev_cycles) = (n.cyccnt, n.cycles);
        if n.regs.pc == CODE + 4 {
            break;
        }
    }
    assert_eq!(n.regs.r[0], 0);
    assert_eq!(n.regs.pc, CODE + 4);
    assert_eq!(ops as u64, 2 * laps, "two ops per lap");
    assert_eq!(n.cycles, 2 * laps, "retired cycles exact");
    // (laps - 1) taken laps at 4c + the final fall lap at 2c.
    assert_eq!(n.cyccnt, 4 * laps - 2, "branch costs exact");

    let mut s = core_at(&code);
    s.regs.r[0] = laps as u32;
    for _ in 0..2 * laps {
        s.step().unwrap();
    }
    assert_eq!((s.regs.r[0], s.regs.pc), (0, CODE + 4));
    assert_eq!(s.cyccnt, n.cyccnt);
    assert_eq!(s.cycles, n.cycles);
}

/// E6 monotonicity through a real device: firmware enables `CYCCNT`, then
/// samples it around native work across quanta. Reads inside TBs return
/// the block-estimated counter — stale within a quantum, but never
/// backward, and advancing once quanta flush.
#[test]
fn dwt_cyccnt_reads_advance_monotonically() {
    struct DwtBus {
        code: Vec<u8>,
        ram: Vec<u8>,
        dwt: Dwt,
        writes: u64,
        ticks: u64,
    }

    impl DwtBus {
        fn new(code_halves: &[u16]) -> Self {
            let mut bus = Self {
                code: vec![0; 0x1000],
                ram: vec![0; 0x10000],
                dwt: Dwt::new(),
                writes: 0,
                ticks: 0,
            };
            for (i, half) in code_halves.iter().enumerate() {
                let [lo, hi] = half.to_le_bytes();
                bus.code[i * 2] = lo;
                bus.code[i * 2 + 1] = hi;
            }
            bus
        }
    }

    impl Bus for DwtBus {
        fn read(&mut self, addr: u32) -> Result<u8, BusError> {
            match addr {
                a if (CODE..CODE + 0x1000).contains(&a) => Ok(self.code[(a - CODE) as usize]),
                a if (RAM..RAM + 0x10000).contains(&a) => Ok(self.ram[(a - RAM) as usize]),
                a if (0xE000_1000..0xE000_1020).contains(&a) => self.dwt.read(a),
                _ => Err(BusError::Reserved),
            }
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            match addr {
                a if (CODE..CODE + 0x1000).contains(&a) => self.code[(a - CODE) as usize] = val,
                a if (RAM..RAM + 0x10000).contains(&a) => self.ram[(a - RAM) as usize] = val,
                a if (0xE000_1000..0xE000_1020).contains(&a) => self.dwt.write(a, val)?,
                _ => return Err(BusError::Reserved),
            }
            self.writes = self.writes.wrapping_add(1);
            Ok(())
        }

        fn write_epoch_ptr(&mut self) -> Option<*mut u64> {
            Some(&mut self.writes)
        }

        fn translate(&mut self, addr: u32, _access: MpuAccess) -> DataTranslate {
            let base: u32;
            let mem: &mut Vec<u8>;
            match addr {
                a if (CODE..CODE + 0x1000).contains(&a) => {
                    base = CODE;
                    mem = &mut self.code;
                }
                a if (RAM..RAM + 0x10000).contains(&a) => {
                    base = RAM;
                    mem = &mut self.ram;
                }
                _ => return DataTranslate::Indirect,
            }
            let page_start = addr & !0xFFF;
            if page_start < base {
                return DataTranslate::Indirect;
            }
            let off = page_start - base;
            let len = (mem.len() as u32).saturating_sub(off);
            if len < 0x1000 {
                return DataTranslate::Indirect;
            }
            DataTranslate::Direct {
                ptr: mem.as_mut_ptr().wrapping_add(off as usize),
                len,
            }
        }
    }

    impl PeripheralBus for DwtBus {
        fn next_exception(
            &mut self,
            _primask: bool,
            _faultmask: bool,
            _basepri: u8,
            _current_ipsr: u8,
        ) -> Option<u8> {
            None
        }

        fn acknowledge_exception(&mut self, _n: u8) {}

        fn tick_by(&mut self, delta: u64) {
            self.dwt.tick(delta);
            self.ticks += delta;
        }

        fn mpu_check(
            &mut self,
            _addr: u32,
            _access: MpuAccess,
            _privileged: bool,
            _exception: u8,
        ) -> bool {
            true
        }

        fn config_epoch(&self) -> Option<u64> {
            Some(0)
        }

        fn write_epoch(&self) -> Option<u64> {
            Some(self.writes)
        }
    }

    // Firmware: an infinite sampling loop (a one-shot prefix plus a spin
    // would sample once, pre-flush, and never again). Each lap enables
    // CYCCNT (idempotent), samples it around two 4-NOP native stretches,
    // and stashes s0/s1/s2 to RAM; every quantum runs several laps, and
    // the flush between quanta is what advances the device.
    let code = [
        0x4809, // 00 LDR r0, =DWT (0x1028)
        0x4C0A, // 02 LDR r4, =SAMPLES (0x102C)
        0x2101, // 04 loop: MOVS r1, #1
        0x6001, // 06 STR r1, [r0]       (CTRL.CYCCNTENA)
        0x6842, // 08 LDR r2, [r0, #4]   (s0)
        0x6022, // 0A STR r2, [r4]
        0xBF00, 0xBF00, 0xBF00, 0xBF00, // 0C..12 work
        0x6843, // 14 LDR r3, [r0, #4]   (s1)
        0x6063, // 16 STR r3, [r4, #4]
        0xBF00, 0xBF00, 0xBF00, 0xBF00, // 18..1E work
        0x6842, // 20 LDR r2, [r0, #4]   (s2)
        0x60A2, // 22 STR r2, [r4, #8]
        0xE7EE, // 24 B loop (0x1004)
        0xBF00, // 26 pad (keeps the pools word-aligned)
        0x1000, 0xE000, // 28 =DWT (0xE0001000)
        0x0100, 0x2000, // 2C =SAMPLES (0x20000100)
    ];
    let mut m = Machine::new(DwtBus::new(&code));
    m.core.regs.pc = CODE;
    m.core.regs.sp = INIT_SP;
    let word = |m: &mut Machine<DwtBus>, a: u32| {
        u32::from_le_bytes([
            m.core.mem().read(a).unwrap(),
            m.core.mem().read(a + 1).unwrap(),
            m.core.mem().read(a + 2).unwrap(),
            m.core.mem().read(a + 3).unwrap(),
        ])
    };
    // One quantum flushes the previous quantum's block estimates into the
    // device; sampling the latest `s2` after each quantum must observe a
    // non-decreasing counter that actually advances.
    let mut runner = TbRunner::new();
    let mut last = Vec::new();
    for _ in 0..4 {
        let _ = m.run_tb_quantum(&mut runner, 64).unwrap();
        last.push(word(&mut m, 0x2000_0108));
    }
    for w in last.windows(2) {
        assert!(
            w[1] >= w[0],
            "CYCCNT never steps back ({} < {})",
            w[1],
            w[0]
        );
    }
    assert!(
        last[3] > last[0],
        "native estimates reach the device ({}..{})",
        last[0],
        last[3]
    );
    // Same-lap consistency: the final lap's three samples came from one
    // unflushed quantum, so they agree with each other and the device.
    let (s0, s1, s2) = (
        word(&mut m, 0x2000_0100),
        word(&mut m, 0x2000_0104),
        word(&mut m, 0x2000_0108),
    );
    assert_eq!((s0, s1, s2), (last[3], last[3], last[3]));
}

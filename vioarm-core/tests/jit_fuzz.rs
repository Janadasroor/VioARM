//! GATE G2 template fuzz: randomized Thumb programs, TB-quanta execution
//! vs the accurate stepper, full architectural state compare (everything
//! except `cyccnt`, which is approximate by design per D7 — `cycles` is
//! exact and compared).
//!
//! Method: rejection-sample real decodes (`decode16_v7m`/`decode32_v7m` +
//! `is_templated`) so every trial executes genuine template paths,
//! including seals, chaining, linking, TLB fills, and side-exit tails.
//! Memories are fully mapped, so faults only come from wild branch
//! targets — those compare as identical `Err` plus identical state.
//!
//! Deterministic: fixed xorshift seed; `FUZZ_TRIALS` overrides the trial
//! count, `FUZZ_ONLY` replays a single trial for debugging.
//!
//! Deliberate boundaries (covered by directed suites, not here):
//! `SVC`/`BKPT` (need exception infrastructure), `WFI`/`WFE` (sleep
//! breaks op-count matching), `IT` blocks (predicated continuation is a
//! narrow exact-match path), devices/MMIO (P2-3 soak + DWT test).

#![cfg(feature = "jit")]

use vioarm_core::cpu::decode16::{decode16_v7m, Hint, Inst16};
use vioarm_core::cpu::decode32::{decode32_v7m, is_thumb2_v7m, Inst32};
use vioarm_core::cpu::registers::{Registers, Xpsr};
use vioarm_core::jit::{is_templated, TbOp, TbRunner};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const ENTRY: u32 = 0x0000_0010;
const RAM_TOP: u32 = 0x2001_0000;
const SP_INIT: u32 = 0x2000_F000;
/// Data window for constrained pointer regs (all offsets stay mapped).
const WIN: u32 = 0x2000_0000;
const WIN_WORDS: u32 = 0x600;

fn seed() -> u64 {
    0xF2A1_1E55_9E37_79B9
}

struct Fz {
    st: u64,
}

impl Fz {
    fn new() -> Self {
        Self { st: seed() }
    }

    fn next(&mut self) -> u32 {
        let mut x = self.st;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.st = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }

    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }

    fn edge(&mut self) -> u32 {
        const E: &[u32] = &[
            0x0000_0000,
            0x0000_0001,
            0x0000_0004,
            0x0000_007C,
            0x0000_0FFF,
            0x7FFF_FFFF,
            0x8000_0000,
            0xFFFF_FFFE,
            0xFFFF_FFFF,
            0x00FF_00FF,
            0xFF00_FF00,
            0x5555_5555,
            0xAAAA_AAAA,
        ];
        if self.below(3) == 0 {
            E[self.below(E.len() as u32) as usize]
        } else {
            self.next()
        }
    }
}

/// A drawn instruction: the exact halves for the image plus the decoded
/// op for classification (real decoder output, never hand-built).
struct Draw {
    halves: [u16; 2],
    len: usize,
    op: TbOp,
}

/// Class-uniform draw (the G2 "each template" obligation by
/// construction): pick a template family first, then rejection-sample
/// halves inside its encoding subspace. Returns real decoder output —
/// seals, tails, and all — or `None` when the draw is undecodable,
/// untemplated, or excluded (SVC/BKPT need exception infra; WFI/WFE break
/// op-count matching; IT continuations stay directed).
fn draw_class(fz: &mut Fz, class: u32) -> Option<Draw> {
    macro_rules! r {
        () => {
            fz.next() as u16
        };
    }
    let (h0, h1, is32) = match class {
        // 0: 16-bit ALU (S0 + extends/reversals).
        0 => match fz.below(5) {
            0 => (r!() & 0x1FFF, 0, false),
            1 => (0x2000 | (r!() & 0x0FFF), 0, false),
            2 => (0x4000 | (r!() & 0x03FF), 0, false),
            3 => (0x4400 | (r!() & 0x00FF), 0, false),
            _ => (0xB200 | (r!() & 0x00FF), 0, false),
        },
        // 1: 16-bit memory.
        1 => match fz.below(4) {
            0 => (0x4800 | (r!() & 0x07FF), 0, false),
            1 => (0x6000 | (r!() & 0x1FFF), 0, false),
            2 => (0x8000 | (r!() & 0x0FFF), 0, false),
            _ => (0x9000 | (r!() & 0x0FFF), 0, false),
        },
        // 2: 16-bit multi/stack.
        2 => match fz.below(3) {
            0 => (0xB400 | (r!() & 0x01FF), 0, false),
            1 => (0xBC00 | (r!() & 0x01FF), 0, false),
            _ => (0xC800 | (r!() & 0x07FF), 0, false),
        },
        // 3: 16-bit branches.
        3 => match fz.below(4) {
            0 => (0xD000 | (r!() & 0x0FFF), 0, false),
            1 => (0xE000 | (r!() & 0x07FF), 0, false),
            2 => (0xB100 | (r!() & 0x00FF), 0, false),
            _ => (0x4700 | (r!() & 0x00FF), 0, false),
        },
        // 4: 16-bit system (hints/cps).
        4 => match fz.below(2) {
            0 => (0xBF00 | (r!() & 0x00FF), 0, false),
            _ => (0xB660 | (r!() & 0x000F), 0, false),
        },
        // 5: 32-bit DP/mul/div/movw.
        5 => match fz.below(5) {
            0 => (0xEA00 | (r!() & 0x00FF), r!(), true),
            1 => (0xF200 | (r!() & 0x00FF), r!(), true),
            2 => (0xFB00 | (r!() & 0x00FF), r!(), true),
            3 => (0xFB90 | (r!() & 0x000F), 0xF0F0 | (r!() & 0x000F), true),
            _ => (0xE800 | (r!() & 0x07FF), r!(), true),
        },
        // 6: 32-bit memory/exclusive.
        6 => match fz.below(3) {
            0 => (0xF840 | (r!() & 0x00FF), r!(), true),
            1 => (0xE850 | (r!() & 0x000F), r!(), true),
            _ => (0xE800 | (r!() & 0x07FF), r!(), true),
        },
        // 7: 32-bit branches/calls.
        7 => match fz.below(3) {
            0 => (0xF000 | (r!() & 0x0FFF), r!(), true),
            1 => (0xE8D0 | (r!() & 0x001F), 0xF000 | (r!() & 0x001F), true),
            _ => (0xF800 | (r!() & 0x03FF), r!(), true),
        },
        // 8: 32-bit system (exact patterns per the decoder: MRS is
        // `F3EF 8Rss`, MSR is `F380|n 80ss`, barriers are `F3BF 8ko`
        // with k in 4/5/6, CLREX is `F3BF 8F2F`).
        _ => match fz.below(5) {
            0 => (
                0xF3EF,
                0x8000 | (fz.below(16) << 8) as u16 | fz.next() as u16 & 0xFF,
                true,
            ),
            1 => (
                0xF380 | fz.below(16) as u16,
                0x8000 | fz.next() as u16 & 0xFF,
                true,
            ),
            2 => (
                0xF3BF,
                0x8000
                    | (([0x4, 0x5, 0x6][fz.below(3) as usize] << 4) as u16)
                    | fz.below(16) as u16,
                true,
            ),
            3 => (0xF3BF, 0x8F2F, true),
            // VFP: EC/ED/EE sub-arms per decode_vfp (most draws still
            // miss the strict sub-patterns — rejection handles it).
            _ => (
                0xEC00 | ([0xC, 0xD, 0xE][fz.below(3) as usize] << 8) | fz.below(256) as u16,
                r!(),
                true,
            ),
        },
    };
    if !is32 {
        let inst = decode16_v7m(h0).ok()?;
        if matches!(
            inst,
            Inst16::Svc { .. }
                | Inst16::Bkpt { .. }
                | Inst16::It { .. }
                | Inst16::Hint {
                    kind: Hint::Wfi | Hint::Wfe,
                    ..
                }
        ) {
            return None;
        }
        let op = TbOp::V16(inst);
        if !is_templated(&op) {
            return None;
        }
        Some(Draw {
            halves: [h0, 0],
            len: 1,
            op,
        })
    } else {
        let inst = decode32_v7m(h0, h1).ok()?;
        let op = TbOp::V32(inst);
        if !is_templated(&op) {
            return None;
        }
        Some(Draw {
            halves: [h0, h1],
            len: 2,
            op,
        })
    }
}

/// Draw any class (salads).
fn draw_any(fz: &mut Fz) -> Option<Draw> {
    let c = fz.below(9);
    draw_class(fz, c)
}

/// Draw a continuing (non-sealing) op from one of `classes` (spins,
/// funclets: the shape owns control flow).
fn draw_cont(fz: &mut Fz, classes: &[u32]) -> Option<Draw> {
    for _ in 0..40 {
        let c = classes[fz.below(classes.len() as u32) as usize];
        let d = draw_class(fz, c)?;
        if !is_branchy(d.op) {
            return Some(d);
        }
    }
    None
}

fn bucket(op: TbOp) -> usize {
    match op {
        TbOp::V16(i) => match i {
            Inst16::LdrLiteral { .. }
            | Inst16::LsReg { .. }
            | Inst16::LsImm { .. }
            | Inst16::LsSp { .. } => 1,
            Inst16::Push { .. } | Inst16::Pop { .. } | Inst16::LdmStm { .. } => 2,
            Inst16::Branch { .. }
            | Inst16::BranchCond { .. }
            | Inst16::CbzCbnz { .. }
            | Inst16::Bx { .. }
            | Inst16::Blx { .. } => 3,
            Inst16::Svc { .. } | Inst16::Bkpt { .. } | Inst16::Cps { .. } | Inst16::Hint { .. } => {
                4
            }
            _ => 0,
        },
        TbOp::V32(i) => match i {
            Inst32::Mem { .. }
            | Inst32::LdrdStrd { .. }
            | Inst32::LdmStm { .. }
            | Inst32::Ldrex { .. }
            | Inst32::Strex { .. }
            | Inst32::Lda { .. }
            | Inst32::Stl { .. }
            | Inst32::VfpLdSt { .. }
            | Inst32::VfpLdStD { .. }
            | Inst32::VfpLdStMult { .. }
            | Inst32::VfpLdStMultD { .. } => 6,
            Inst32::Bl { .. }
            | Inst32::BlxImm { .. }
            | Inst32::BranchWide { .. }
            | Inst32::BranchCondWide { .. }
            | Inst32::TblBranch { .. } => 7,
            Inst32::Mrs { .. } | Inst32::Msr { .. } | Inst32::Barrier { .. } | Inst32::Clrex => 8,
            Inst32::Vfp3 { .. }
            | Inst32::Vfp2 { .. }
            | Inst32::VfpCmp { .. }
            | Inst32::VfpMovImm { .. }
            | Inst32::VfpCvt { .. }
            | Inst32::VfpCvtInt { .. }
            | Inst32::VfpVrint { .. }
            | Inst32::VfpMrsVmsr { .. } => 9,
            _ => 5,
        },
    }
}

fn is_branchy(op: TbOp) -> bool {
    match op {
        TbOp::V16(
            Inst16::Branch { .. }
            | Inst16::BranchCond { .. }
            | Inst16::CbzCbnz { .. }
            | Inst16::Bx { .. }
            | Inst16::Blx { .. }
            | Inst16::Pop { load_pc: true, .. }
            | Inst16::SpecialMov { rd: 15, .. }
            | Inst16::SpecialAdd { rd: 15, .. }
            | Inst16::LdrLiteral { rt: 15, .. },
        ) => true,
        TbOp::V32(
            Inst32::Bl { .. }
            | Inst32::BlxImm { .. }
            | Inst32::BranchWide { .. }
            | Inst32::BranchCondWide { .. }
            | Inst32::TblBranch { .. },
        ) => true,
        TbOp::V32(Inst32::LdmStm { is_load, regs, .. }) => is_load && regs & 0x8000 != 0,
        _ => false,
    }
}

struct Prog {
    halves: Vec<u16>,
    buckets: [u64; 10],
}

fn push_draw(p: &mut Prog, d: &Draw) -> bool {
    if p.halves.len() + d.len > 120 {
        return false;
    }
    p.halves.extend_from_slice(&d.halves[..d.len]);
    p.buckets[bucket(d.op)] += 1;
    true
}

/// 16-bit `BL from -> to` (small forward; copied semantics from the
/// differential suite's helper).
fn bl(from: u32, to: u32) -> (u16, u16) {
    let off = to as i32 - (from as i32 + 4);
    assert!(off >= 0 && off & 1 == 0 && off < 0x400000);
    let h0 = 0xF000 | ((off >> 12) as u16 & 0x03FF);
    let h1 = 0xF800 | ((off >> 1) as u16 & 0x07FF);
    (h0, h1)
}

const B_SELF: u16 = 0xE7FE;
const BX_LR: u16 = 0x4770;

/// Known-good VFP vectors (from the decoder's own tests): random draws
/// almost never satisfy the strict coprocessor sub-patterns, so salads
/// sprinkle these deterministically — still through the full native path
/// (side-exit tails) with random regs/state.
const VFP_POOL: &[(u16, u16)] = &[
    (0xEE00, 0x0A10), // VMOV S0, r0
    (0xEE00, 0x0A90), // VMOV S1, r0
    (0xEE30, 0x0A20), // VADD.F32 S0, S0, S1
    (0xEE1F, 0x6A90), // VMOV r6, S31
    (0xEEF7, 0x0A00), // VLDR-ish (decoder decides; templated either way)
    (0xEEF1, 0xFA10), // VFP data-processing (same)
];

/// Spin shape: continuing ops only (no mid-way seals) + self-loop.
/// Exercises hot chaining and linking.
fn shape_spin(fz: &mut Fz, p: &mut Prog) {
    const CLS: &[u32] = &[0, 0, 1, 2, 5, 6];
    let n = 5 + fz.below(6);
    while p.halves.len() < n as usize {
        let Some(d) = draw_cont(fz, CLS) else { break };
        if !push_draw(p, &d) {
            break;
        }
    }
    p.halves.push(B_SELF);
}

/// Salad shape: anything goes (multi-block graphs, wild targets, seals).
fn shape_salad(fz: &mut Fz, p: &mut Prog) {
    let n = 5 + fz.below(8);
    let mut guard = 0u32;
    let mut vfp = 0usize;
    while p.halves.len() < n as usize && guard < 200 {
        guard += 1;
        // Every ~5th slot: a deterministic VFP vector (see VFP_POOL).
        if guard % 5 == 0 {
            let (h0, h1) = VFP_POOL[vfp % VFP_POOL.len()];
            vfp += 1;
            if let Ok(inst) = decode32_v7m(h0, h1) {
                let op = TbOp::V32(inst);
                if is_templated(&op) && p.halves.len() + 2 <= 120 && bucket(op) == 9 {
                    p.halves.extend_from_slice(&[h0, h1]);
                    p.buckets[9] += 1;
                    continue;
                }
            }
        }
        let Some(d) = draw_any(fz) else { continue };
        if !push_draw(p, &d) {
            break;
        }
    }
    p.halves.push(B_SELF);
}

/// Call shape: `BL funclet; (pad); B .` + funclet body + `BX lr`, with an
/// optional nested second funclet. Exercises call chaining + returns.
fn shape_calls(fz: &mut Fz, p: &mut Prog) {
    // BL at image+0 -> funclet at image+8 (room for B . pad at +4).
    let f1 = ENTRY + 8;
    let (b0, b1) = bl(ENTRY, f1);
    p.halves.extend_from_slice(&[b0, b1, B_SELF, 0xBF00]);
    // Funclet 1: 1-3 continuing draws + BX lr.
    const CLS: &[u32] = &[0, 1, 5];
    let mut n = 0u32;
    while n < 1 + fz.below(3) {
        let Some(d) = draw_cont(fz, CLS) else { break };
        if push_draw(p, &d) {
            n += 1;
        }
    }
    // Optional nested call to funclet 2 (emitted right after).
    if fz.below(2) == 0 {
        let cur = ENTRY + (p.halves.len() as u32) * 2;
        let f2 = cur + 4;
        let (c0, c1) = bl(cur, f2);
        p.halves.extend_from_slice(&[c0, c1]);
        p.halves.push(BX_LR);
        // Funclet 2: one draw + return.
        if let Some(d) = draw_cont(fz, &[0]) {
            push_draw(p, &d);
        }
        p.halves.push(BX_LR);
    } else {
        p.halves.push(BX_LR);
    }
    // Re-bucket the whole image from its decodes (covers the hand-placed
    // BL/BX halves exactly as the builder sees them).
    rebucket(p);
}

/// Memory-heavy spin: prefer mem/multi draws, constrained pointers keep
/// every access mapped.
fn shape_mem(fz: &mut Fz, p: &mut Prog) {
    const CLS: &[u32] = &[1, 1, 2, 6, 6, 0];
    let n = 5 + fz.below(6);
    while p.halves.len() < n as usize {
        let Some(d) = draw_cont(fz, CLS) else { break };
        if !push_draw(p, &d) {
            break;
        }
    }
    p.halves.push(B_SELF);
}

/// Re-derive bucket counts from the final image (covers hand-placed
/// halves like `BL`/`BX lr` exactly as the builder sees them).
fn rebucket(p: &mut Prog) {
    p.buckets = [0; 10];
    let mut i = 0usize;
    while i < p.halves.len() {
        let h0 = p.halves[i];
        if is_thumb2_v7m(h0) && i + 1 < p.halves.len() {
            if let Ok(inst) = decode32_v7m(h0, p.halves[i + 1]) {
                let op = TbOp::V32(inst);
                if is_templated(&op) {
                    p.buckets[bucket(op)] += 1;
                }
                i += 2;
                continue;
            }
        }
        if let Ok(inst) = decode16_v7m(h0) {
            let op = TbOp::V16(inst);
            if is_templated(&op) {
                p.buckets[bucket(op)] += 1;
            }
        }
        i += 1;
    }
}

fn halves_bytes(halves: &[u16]) -> Vec<u8> {
    halves.iter().flat_map(|h| h.to_le_bytes()).collect()
}

fn boot(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    m.core.mem().write_word(FLASH, SP_INIT).unwrap();
    m.core.mem().write_word(FLASH + 4, ENTRY | 1).unwrap();
    m.reset().expect("reset from vector table");
    m
}

/// Constrained random state: pointer regs into the data window
/// (word-aligned), small offset regs, edge-sprinkled rest.
fn setup_state(fz: &mut Fz, m: &mut Machine<FlatMemory>) {
    for r in 0..3 {
        m.core.regs.r[r] = WIN + 4 * fz.below(WIN_WORDS);
    }
    m.core.regs.r[3] = fz.below(128);
    m.core.regs.r[4] = fz.below(64);
    for r in 5..13 {
        m.core.regs.r[r] = fz.edge();
    }
    m.core.regs.sp = SP_INIT;
    m.core.regs.lr = if fz.below(4) == 0 {
        ENTRY | 1
    } else {
        fz.edge() | 1
    };
    m.core.xpsr.set(0x0100_0000 | (fz.next() & 0xF89F_0000));
    // Patterned RAM window (deterministic per seed stream).
    for i in 0..0x800u32 {
        let v = (i.wrapping_mul(0x9E37_79B9).wrapping_add(fz.next())) & 0xFF_FFFF;
        let addr = WIN + 4 * i;
        m.core.mem().write_word(addr, v).unwrap();
    }
}

/// Full comparable state minus `cyccnt` (D7-approximate by design).
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
    ram: Vec<u8>,
}

fn snap(m: &mut Machine<FlatMemory>) -> Snap {
    let mut ram = Vec::with_capacity(0x2000);
    for i in 0..0x2000u32 {
        ram.push(m.core.mem().read(WIN + i).unwrap_or(0xAA));
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
        ram,
    }
}

fn trials() -> u64 {
    std::env::var("FUZZ_TRIALS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(600)
}

fn only() -> Option<u64> {
    std::env::var("FUZZ_ONLY").ok().and_then(|v| v.parse().ok())
}

#[test]
fn template_fuzz_matches_stepper() {
    // `BX lr` sanity (hand-placed in call shapes).
    assert!(matches!(decode16_v7m(BX_LR), Ok(Inst16::Bx { rm: 14 })));
    // VFP pool sanity (every vector must be a templated VFP form).
    for (h0, h1) in VFP_POOL {
        let inst = decode32_v7m(*h0, *h1);
        assert!(inst.is_ok(), "VFP pool decodes ({h0:#X},{h1:#X})");
        assert!(
            is_templated(&TbOp::V32(inst.unwrap())),
            "VFP pool templated ({h0:#X},{h1:#X})"
        );
    }
    let total = trials();
    let range: Vec<u64> = match only() {
        Some(t) => vec![t],
        None => (0..total).collect(),
    };
    let mut cover = [0u64; 10];
    for trial in range {
        // Independent stream per trial (replayable via FUZZ_ONLY).
        let mut fz = Fz::new();
        for _ in 0..trial {
            fz.next();
        }
        let mut p = Prog {
            halves: Vec::new(),
            buckets: [0; 10],
        };
        match trial % 4 {
            0 => shape_spin(&mut fz, &mut p),
            1 => shape_salad(&mut fz, &mut p),
            2 => shape_calls(&mut fz, &mut p),
            _ => shape_mem(&mut fz, &mut p),
        }
        if trial % 4 == 2 {
            // calls shape already rebucketed.
        } else {
            rebucket(&mut p);
        }
        for (b, c) in cover.iter_mut().zip(p.buckets.iter()) {
            *b += *c;
        }
        let code = halves_bytes(&p.halves);
        let target = 120 + (trial % 3) * 40;
        // Native side.
        let mut mn = boot(&code);
        // State setup consumes stream identically on both sides: snapshot
        // the stream position by cloning setup draws — simplest is to run
        // setup twice from the same stream state.
        let st = fz.st;
        setup_state(&mut fz, &mut mn);
        let mut runner = TbRunner::new();
        let mut ops = 0u64;
        let nerr = loop {
            match mn.run_tb_quantum(&mut runner, 32) {
                Ok(q) => {
                    ops += q.ops;
                    if ops >= target {
                        break None;
                    }
                }
                Err(e) => break Some(e),
            }
        };
        // Stepper side: identical setup, identical op count (or Err).
        let mut ms = boot(&code);
        fz.st = st;
        setup_state(&mut fz, &mut ms);
        let serr = if let Some(e) = nerr {
            let mut stepped = 0u64;
            let got = loop {
                if stepped >= ops + 64 {
                    break None;
                }
                match ms.step() {
                    Ok(()) => stepped += 1,
                    Err(ee) => break Some(ee),
                }
            };
            assert_eq!(got, Some(e), "fault parity (trial {trial})");
            Some(e)
        } else {
            for _ in 0..ops {
                ms.step()
                    .expect("stepper must not fault when native did not");
            }
            None
        };
        let (sn, ss) = (snap(&mut mn), snap(&mut ms));
        assert_eq!(sn, ss, "state match (trial {trial}, native err {nerr:?})");
        let _ = serr;
    }
    eprintln!("fuzz coverage buckets: {cover:?}");
    // The G2 obligation: every family fired substantially. (VFP tallies
    // only when the stream draws v7-M FP forms; system-barrier forms are
    // rare by encoding — thresholds reflect draw reality, not importance.)
    for (i, c) in cover.iter().enumerate() {
        let min = match i {
            9 => 5,  // VFP: sparse in Thumb-2 space
            4 => 10, // 16-bit system: small subspace
            8 => 10, // 32-bit sys/barrier: small subspace
            _ => 50,
        };
        assert!(
            *c >= min,
            "bucket {i} under-covered ({c} < {min}): {cover:?}"
        );
    }
}

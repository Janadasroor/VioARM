//! Randomized template-vs-interpreter differential core (GATE G2, P3-2).
//!
//! Class-uniform random Thumb programs run natively (TB quanta) vs the
//! accurate stepper with full architectural state compare (everything
//! except `cyccnt`, approximate by design per D7 — `cycles` is exact).
//! Rejection-sampled real decodes keep every trial on genuine template
//! paths, including seals, chaining, linking, TLB fills, and side-exit
//! tails. Memories are fully mapped, so faults only come from wild
//! branch targets — those compare as identical `Err` plus identical
//! state.
//!
//! Two consumers share this module: `tests/jit_fuzz.rs` (deterministic
//! seeded trials + coverage obligation) and `fuzz/fuzz_jit.rs` (stdin
//! byte inputs for nightly fuzzing, `--trials` bounded runs for CI).
//! Deterministic: fixed xorshift seed; trial `i` always builds the same
//! program and state, so any failure replays standalone.
//!
//! Deliberate boundaries (directed suites own these): `SVC`/`BKPT`
//! (need exception infrastructure), `WFI`/`WFE` (sleep breaks op-count
//! matching), `IT` blocks (predicated continuation is a narrow
//! exact-match path), devices/MMIO (soak + DWT test).

use crate::cpu::decode16::{decode16_v7m, Hint, Inst16};
use crate::cpu::decode32::{decode32_v7m, is_thumb2_v7m, Inst32};
use crate::cpu::registers::{Registers, Xpsr};
use crate::error::HaltCause;
use crate::jit::TbRunner;
use crate::loader::load_flat;
use crate::machine::Machine;
use crate::mem::flat::FlatMemory;
use crate::mem::Bus;

use super::{is_templated, TbOp};

/// Code base (flash bank) for trial images.
pub const FLASH: u32 = 0x0000_0000;
/// Data bank base.
pub const RAM: u32 = 0x2000_0000;
/// Trial entry point.
pub const ENTRY: u32 = 0x0000_0010;
/// Initial stack pointer (stack grows down inside the RAM bank).
pub const SP_INIT: u32 = 0x2000_F000;
/// Data window for constrained pointer regs (all offsets stay mapped).
pub const WIN: u32 = 0x2000_0000;
/// Window size in words.
pub const WIN_WORDS: u32 = 0x600;
/// Patterned RAM image size in words.
pub const RAM_WORDS: u32 = 0x800;

fn seed() -> u64 {
    0xF2A1_1E55_9E37_79B9
}

/// Deterministic xorshift64* stream (fixed seed: every trial replays).
#[derive(Clone, Copy, Debug)]
pub struct Fz {
    st: u64,
}

impl Fz {
    /// Fresh stream from the fixed seed.
    #[must_use]
    pub fn new() -> Self {
        Self { st: seed() }
    }

    /// Next pseudorandom word.
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.st;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.st = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }

    /// Uniform value in `0..n`.
    pub fn below(&mut self, n: u32) -> u32 {
        self.next_u32() % n
    }

    /// Random word, biased toward directed edge values.
    pub fn edge(&mut self) -> u32 {
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
            self.next_u32()
        }
    }
}

impl Default for Fz {
    fn default() -> Self {
        Self::new()
    }
}

/// A drawn instruction: exact halves for the image plus the decoded op.
#[derive(Clone, Copy, Debug)]
pub struct Draw {
    /// Halves (`len` of them valid).
    pub halves: [u16; 2],
    /// 1 (16-bit) or 2 (32-bit).
    pub len: usize,
    /// The decoded op (for classification).
    pub op: TbOp,
}

/// Class-uniform draw (the G2 "each template" obligation by
/// construction): pick a template family first, then rejection-sample
/// halves inside its encoding subspace. Returns real decoder output —
/// seals, tails, and all — or `None` on undecodable/untemplated draws.
#[must_use]
pub fn draw_class(fz: &mut Fz, class: u32) -> Option<Draw> {
    macro_rules! r {
        () => {
            fz.next_u32() as u16
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
                0x8000 | (fz.below(16) << 8) as u16 | fz.next_u32() as u16 & 0xFF,
                true,
            ),
            1 => (
                0xF380 | fz.below(16) as u16,
                0x8000 | fz.next_u32() as u16 & 0xFF,
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

/// Draw any family (salads).
#[must_use]
pub fn draw_any(fz: &mut Fz) -> Option<Draw> {
    let c = fz.below(9);
    draw_class(fz, c)
}

/// Draw a continuing (non-sealing) op from `classes` (spins, funclets:
/// the shape owns control flow).
#[must_use]
pub fn draw_cont(fz: &mut Fz, classes: &[u32]) -> Option<Draw> {
    for _ in 0..40 {
        let c = classes[fz.below(classes.len() as u32) as usize];
        let d = draw_class(fz, c)?;
        if !is_branchy(d.op) {
            return Some(d);
        }
    }
    None
}

/// Template family bucket for coverage accounting (0–9, see the G2
/// obligation in `tests/jit_fuzz.rs`).
#[must_use]
pub fn bucket(op: TbOp) -> usize {
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
            | Inst32::VfpVrint { .. }
            | Inst32::VfpCvt { .. }
            | Inst32::VfpCvtInt { .. }
            | Inst32::VfpMovImm { .. }
            | Inst32::VfpMrsVmsr { .. } => 9,
            _ => 5,
        },
    }
}

/// Whether the op seals a block with control flow (shapes route around
/// these except where they own the edges).
#[must_use]
pub fn is_branchy(op: TbOp) -> bool {
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

/// A trial program: image halves plus static family counts.
#[derive(Clone, Debug, Default)]
pub struct Prog {
    /// Image halves at `ENTRY`.
    pub halves: Vec<u16>,
    /// Static classification counts (coverage input).
    pub buckets: [u64; 10],
}

fn push_draw(p: &mut Prog, d: &Draw) -> bool {
    if p.halves.len() + d.len > 120 {
        return false;
    }
    p.halves.extend_from_slice(&d.halves[..d.len]);
    p.buckets[bucket(d.op)] += 1;
    true
}

/// 16-bit `BL from -> to` (small forward).
fn bl(from: u32, to: u32) -> (u16, u16) {
    let off = to as i32 - (from as i32 + 4);
    assert!(off >= 0 && off & 1 == 0 && off < 0x400000);
    let h0 = 0xF000 | ((off >> 12) as u16 & 0x03FF);
    let h1 = 0xF800 | ((off >> 1) as u16 & 0x07FF);
    (h0, h1)
}

/// Self-loop terminator ending every shape.
pub const B_SELF: u16 = 0xE7FE;
/// Hand-placed return (call shapes; decode-verified by consumers).
pub const BX_LR: u16 = 0x4770;

/// Known-good VFP vectors (from the decoder's own tests): random draws
/// almost never satisfy the strict coprocessor sub-patterns, so salads
/// sprinkle these deterministically — still through the full native path
/// with random regs/state.
pub const VFP_POOL: &[(u16, u16)] = &[
    (0xEE00, 0x0A10), // VMOV S0, r0
    (0xEE00, 0x0A90), // VMOV S1, r0
    (0xEE30, 0x0A20), // VADD.F32 S0, S0, S1
    (0xEE1F, 0x6A90), // VMOV r6, S31
    (0xEEF7, 0x0A00), // VLDR-ish (decoder decides; templated either way)
    (0xEEF1, 0xFA10), // VFP data-processing (same)
];

/// Spin shape: continuing ops only (no mid-way seals) + self-loop.
/// Exercises hot chaining and linking.
pub fn shape_spin(fz: &mut Fz, p: &mut Prog) {
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
pub fn shape_salad(fz: &mut Fz, p: &mut Prog) {
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
pub fn shape_calls(fz: &mut Fz, p: &mut Prog) {
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
pub fn shape_mem(fz: &mut Fz, p: &mut Prog) {
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
pub fn rebucket(p: &mut Prog) {
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

/// Trial program for index `i` (shape round-robins spin/salad/calls/mem).
#[must_use]
pub fn trial_program(i: u64) -> Prog {
    let mut fz = Fz::new();
    for _ in 0..i {
        fz.next_u32();
    }
    let mut p = Prog {
        halves: Vec::new(),
        buckets: [0; 10],
    };
    match i % 4 {
        0 => shape_spin(&mut fz, &mut p),
        1 => shape_salad(&mut fz, &mut p),
        2 => shape_calls(&mut fz, &mut p),
        _ => shape_mem(&mut fz, &mut p),
    }
    if i % 4 != 2 {
        rebucket(&mut p);
    }
    p
}

/// Drawn machine state (no machine needed to build it, so trials
/// serialize exactly).
#[derive(Clone, Debug)]
pub struct FuzzState {
    /// `r0`–`r12` (`r0`–`r2` constrained into the data window).
    pub r: [u32; 13],
    /// Stack pointer (inside the RAM bank).
    pub sp: u32,
    /// Link register.
    pub lr: u32,
    /// Raw XPSR bits (Thumb set, IT zero, random NZCV/Q/GE).
    pub xpsr_bits: u32,
    /// Patterned RAM image (`RAM_WORDS` words at `WIN`).
    pub ram: Vec<u32>,
}

/// Constrained random state: pointer regs into the data window
/// (word-aligned), small offset regs, edge-sprinkled rest.
#[must_use]
pub fn build_state(fz: &mut Fz) -> FuzzState {
    let mut r = [0u32; 13];
    for v in r.iter_mut().take(3) {
        *v = WIN + 4 * fz.below(WIN_WORDS);
    }
    r[3] = fz.below(128);
    r[4] = fz.below(64);
    for v in r.iter_mut().skip(5) {
        *v = fz.edge();
    }
    let sp = SP_INIT;
    let lr = if fz.below(4) == 0 {
        ENTRY | 1
    } else {
        fz.edge() | 1
    };
    let xpsr_bits = 0x0100_0000 | (fz.next_u32() & 0xF89F_0000);
    let mut ram = Vec::with_capacity(RAM_WORDS as usize);
    for i in 0..RAM_WORDS {
        ram.push(i.wrapping_mul(0x9E37_79B9).wrapping_add(fz.next_u32()) & 0xFF_FFFF);
    }
    FuzzState {
        r,
        sp,
        lr,
        xpsr_bits,
        ram,
    }
}

/// Trial state for index `i` (stream continues where the program draws
/// left off — same order as the historical harness).
#[must_use]
pub fn trial_state(i: u64) -> (Prog, FuzzState) {
    let mut fz = Fz::new();
    for _ in 0..i {
        fz.next_u32();
    }
    let mut p = Prog {
        halves: Vec::new(),
        buckets: [0; 10],
    };
    match i % 4 {
        0 => shape_spin(&mut fz, &mut p),
        1 => shape_salad(&mut fz, &mut p),
        2 => shape_calls(&mut fz, &mut p),
        _ => shape_mem(&mut fz, &mut p),
    }
    if i % 4 != 2 {
        rebucket(&mut p);
    }
    (p, build_state(&mut fz))
}

/// Image bytes for a program.
#[must_use]
pub fn halves_bytes(halves: &[u16]) -> Vec<u8> {
    halves.iter().flat_map(|h| h.to_le_bytes()).collect()
}

/// Boot a machine over a trial image.
#[must_use]
pub fn boot(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    m.core.mem().write_word(FLASH, SP_INIT).unwrap();
    m.core.mem().write_word(FLASH + 4, ENTRY | 1).unwrap();
    m.reset().expect("reset from vector table");
    m
}

/// Apply drawn state to a booted machine.
pub fn apply_state(m: &mut Machine<FlatMemory>, st: &FuzzState) {
    m.core.regs.r = st.r;
    m.core.regs.sp = st.sp;
    m.core.regs.lr = st.lr;
    m.core.xpsr.set(st.xpsr_bits);
    for (i, w) in st.ram.iter().enumerate() {
        m.core
            .mem()
            .write_word(WIN + 4 * i as u32, *w)
            .expect("ram image fits the bank");
    }
}

/// Full comparable state minus `cyccnt` (D7-approximate by design).
#[derive(Debug, PartialEq, Eq)]
pub struct Snap {
    /// Register file (includes `pc`/`sp`/`lr`).
    pub regs: Registers,
    /// Flags.
    pub xpsr: Xpsr,
    /// Exception number.
    pub ipsr: u8,
    /// Masks and control.
    pub primask: bool,
    /// Base priority.
    pub basepri: u8,
    /// Fault mask.
    pub faultmask: bool,
    /// CONTROL register.
    pub control: u32,
    /// IT state (always drained to zero here — no IT draws).
    pub itstate: u8,
    /// Sleep latch (never set — no sleep draws).
    pub sleeping: bool,
    /// Retired-op cycles (exact on both engines).
    pub cycles: u64,
    /// RAM window image.
    pub ram: Vec<u8>,
}

/// Snapshot a machine for comparison.
#[must_use]
pub fn snap(m: &mut Machine<FlatMemory>) -> Snap {
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

/// Per-trial outcome (coverage input + retired-op count).
#[derive(Clone, Copy, Debug, Default)]
pub struct TrialStats {
    /// Static family counts of the trial program.
    pub buckets: [u64; 10],
    /// Retired ops on the compared path.
    pub ops: u64,
}

/// A native-vs-stepper divergence (or fault-parity break).
#[derive(Debug)]
pub struct Mismatch {
    /// Trial index or input-byte description.
    pub desc: String,
    /// Image halves.
    pub halves: Vec<u16>,
    /// Native halt (if it faulted).
    pub native_err: Option<HaltCause>,
    /// Stepper halt (if it faulted).
    pub stepper_err: Option<HaltCause>,
    /// End states.
    pub native: Snap,
    /// End states.
    pub stepper: Snap,
}

impl std::fmt::Display for Mismatch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "fuzz mismatch ({})", self.desc)?;
        writeln!(f, "  halves: {:04X?}", self.halves)?;
        writeln!(
            f,
            "  native err: {:?} / stepper err: {:?}",
            self.native_err, self.stepper_err
        )?;
        writeln!(f, "  native regs: {:?}", self.native.regs)?;
        writeln!(f, "  stepper regs: {:?}", self.stepper.regs)?;
        writeln!(
            f,
            "  native xpsr: {:?} / stepper: {:?}",
            self.native.xpsr, self.stepper.xpsr
        )?;
        writeln!(
            f,
            "  native cycles: {} / stepper: {}",
            self.native.cycles, self.stepper.cycles
        )?;
        let diff = self
            .native
            .ram
            .iter()
            .zip(self.stepper.ram.iter())
            .position(|(a, b)| a != b);
        match diff {
            Some(i) => writeln!(
                f,
                "  ram first diff at {:#X}: {:02X?} vs {:02X?}",
                WIN + i as u32,
                &self.native.ram[i..(i + 8).min(self.native.ram.len())],
                &self.stepper.ram[i..(i + 8).min(self.stepper.ram.len())],
            ),
            None => writeln!(f, "  ram: identical"),
        }
    }
}

/// Target retired-op count for trial `i` (small: shapes terminate via
/// self-loops, unmapped targets, or faults).
fn target_ops(i: u64) -> u64 {
    120 + (i % 3) * 40
}

/// Run trial `i` natively vs the stepper; `Ok` stats on full match.
pub fn check_trial(i: u64) -> Result<TrialStats, Box<Mismatch>> {
    let (p, st) = trial_state(i);
    check_prog(&format!("trial {i}"), &p, &st, target_ops(i))
}

/// Run one program/state pair natively vs the stepper.
pub fn check_prog(
    desc: &str,
    p: &Prog,
    st: &FuzzState,
    target: u64,
) -> Result<TrialStats, Box<Mismatch>> {
    let code = halves_bytes(&p.halves);
    let mut mn = boot(&code);
    apply_state(&mut mn, st);
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
    let mut ms = boot(&code);
    apply_state(&mut ms, st);
    // Stepper side: step the identical path (faults break with cause).
    let mut serr = None;
    let mut stepped = 0u64;
    let cap = if nerr.is_some() { ops + 64 } else { ops };
    while stepped < cap {
        match ms.step() {
            Ok(()) => stepped += 1,
            Err(ee) => {
                serr = Some(ee);
                break;
            }
        }
    }
    // Fault parity: identical halt causes (state equality below pins the
    // same fault point on the deterministic path).
    if nerr != serr {
        return Err(Box::new(Mismatch {
            desc: desc.to_string(),
            halves: p.halves.clone(),
            native_err: nerr,
            stepper_err: serr,
            native: snap(&mut mn),
            stepper: snap(&mut ms),
        }));
    }
    let (sn, ss) = (snap(&mut mn), snap(&mut ms));
    if sn != ss {
        return Err(Box::new(Mismatch {
            desc: desc.to_string(),
            halves: p.halves.clone(),
            native_err: nerr,
            stepper_err: serr,
            native: sn,
            stepper: ss,
        }));
    }
    Ok(TrialStats {
        buckets: p.buckets,
        ops,
    })
}

/// Serialize a trial to the `fuzz_jit` stdin format: `u16` half count,
/// halves, 13 regs, sp, lr, xpsr bits, `u32` RAM word count, RAM words
/// (all little-endian).
#[must_use]
pub fn trial_input(i: u64) -> Vec<u8> {
    let (p, st) = trial_state(i);
    let mut out = Vec::new();
    out.extend_from_slice(&(p.halves.len() as u16).to_le_bytes());
    for h in &p.halves {
        out.extend_from_slice(&h.to_le_bytes());
    }
    for r in st.r {
        out.extend_from_slice(&r.to_le_bytes());
    }
    out.extend_from_slice(&st.sp.to_le_bytes());
    out.extend_from_slice(&st.lr.to_le_bytes());
    out.extend_from_slice(&st.xpsr_bits.to_le_bytes());
    out.extend_from_slice(&(st.ram.len() as u32).to_le_bytes());
    for w in &st.ram {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

/// Parse the `fuzz_jit` stdin format back into a program + state
/// (trailing bytes ignored: fuzzer-friendly).
fn parse_input(input: &[u8]) -> Option<(Prog, FuzzState)> {
    let mut cur = input;
    fn take<'a>(cur: &mut &'a [u8], n: usize) -> Option<&'a [u8]> {
        if cur.len() < n {
            return None;
        }
        let (h, t) = cur.split_at(n);
        *cur = t;
        Some(h)
    }
    fn u16le(b: &[u8]) -> u16 {
        u16::from_le_bytes([b[0], b[1]])
    }
    fn u32le(b: &[u8]) -> u32 {
        u32::from_le_bytes([b[0], b[1], b[2], b[3]])
    }
    let n = u16le(take(&mut cur, 2)?) as usize;
    if n == 0 || n > 120 {
        return None;
    }
    let mut halves = Vec::with_capacity(n);
    for _ in 0..n {
        halves.push(u16le(take(&mut cur, 2)?));
    }
    let mut r = [0u32; 13];
    for v in r.iter_mut() {
        *v = u32le(take(&mut cur, 4)?);
    }
    // Keep every access mapped and the entry sane regardless of input:
    // pointers fold into the window, SP/LR stay valid, flags stay bland.
    for v in r.iter_mut().take(3) {
        *v = (WIN + (*v % (WIN_WORDS * 4))) & !3;
    }
    r[3] %= 128;
    r[4] %= 64;
    // SP travels in the format but stays pinned: an unmapped stack
    // would fault before any template runs (noise, not signal).
    let _sp = take(&mut cur, 4)?;
    let sp = SP_INIT;
    let lr = u32le(take(&mut cur, 4)?) | 1;
    let xpsr_bits = 0x0100_0000 | (u32le(take(&mut cur, 4)?) & 0xF89F_0000);
    let nw = u32le(take(&mut cur, 4)?) as usize;
    if nw > RAM_WORDS as usize {
        return None;
    }
    let mut ram = Vec::with_capacity(nw);
    for _ in 0..nw {
        ram.push(u32le(take(&mut cur, 4)?));
    }
    while ram.len() < RAM_WORDS as usize {
        let i = ram.len() as u32;
        ram.push(i.wrapping_mul(0x9E37_79B9) & 0xFF_FFFF);
    }
    let mut p = Prog {
        halves,
        buckets: [0; 10],
    };
    rebucket(&mut p);
    Some((
        p,
        FuzzState {
            r,
            sp,
            lr,
            xpsr_bits,
            ram,
        },
    ))
}

/// Run raw input bytes as one differential (the nightly-fuzz entry).
/// Rejects malformed inputs with `None` (not a mismatch).
#[must_use]
pub fn check_bytes(input: &[u8]) -> Option<Result<TrialStats, Box<Mismatch>>> {
    let (p, st) = parse_input(input)?;
    // Skip degenerate images (no executable content): not interesting.
    if p.halves.iter().all(|&h| h == 0xBF00 || h == 0xE7FE) {
        return None;
    }
    Some(check_prog("stdin", &p, &st, 160))
}

/// Default trial count (`FUZZ_TRIALS` overrides).
pub fn trials() -> u64 {
    std::env::var("FUZZ_TRIALS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(600)
}

/// Single-trial replay (`FUZZ_ONLY`), for debugging.
pub fn only() -> Option<u64> {
    std::env::var("FUZZ_ONLY").ok().and_then(|v| v.parse().ok())
}

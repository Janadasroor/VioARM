//! Translation-block builder (P1-2): linear scan over the existing
//! decoders with T2 stop rules.
//!
//! The builder reuses `decode16*`/`decode32*` with exactly the per-ISA
//! dispatch the accurate stepper uses, fetches through the same
//! [`Bus::read_half`] path (so code-fetch side effects are identical to
//! single-stepping), and seals the block at the first T2 boundary:
//!
//! - any branch/call/return or PC-writing op (inclusive, [`TbEnd::ControlFlow`]);
//! - any S6 callout op — system, exclusive, barrier, FP, DSP, `WFI`/`WFE`,
//!   `IT` — or unpredictable PC-register form (inclusive, [`TbEnd::System`]);
//! - any bus-writing op (inclusive, [`TbEnd::Store`]; P2-3: forces
//!   entry revalidation after every store for both engines);
//! - [`MAX_TB_INSNS`] ops (inclusive, [`TbEnd::MaxInsns`]);
//! - a 4 KiB page-boundary cross (exclusive, [`TbEnd::PageCross`]);
//! - an unfetchable/undecodable instruction (exclusive, [`TbEnd::Halt`];
//!   the interpreter owns whatever happens there, including the fault).
//!
//! The builder refuses to start (`None`) when `itstate != 0` (predicated
//! bodies run interpreted until the IT block drains — Phase-1 IT policy,
//! S4), on odd entry PCs (mirrors the `FetchAbort`), and when even the
//! first instruction cannot be fetched or decoded.
//!
//! What the builder deliberately does NOT do (owned by P1-3 / later phases):
//! MPU fetch-permission gating (the execution loop must gate block entry on
//! fetch permission — a block over a no-execute region must never run),
//! MMIO-touch detection (unknowable statically; the loop side-exits on the
//! first MMIO access), and bit-band exclusion (M4: the loop must refuse
//! blocks covering alias regions).

use crate::cpu::decode16::{decode16, decode16_v7m, Hint, Inst16, LsImm, LsReg};
use crate::cpu::decode32::{
    decode32, decode32_v7em, decode32_v7m, decode32_v8m, decode32_v8m_mainline, is_thumb2,
    is_thumb2_v7m, Inst32, MemOp,
};
use crate::cpu::Isa;
use crate::mem::bus::Bus;

use super::tb::{MpuEpoch, TbMeta, MAX_TB_INSNS, PAGE_SHIFT};

/// One decoded block operation: the existing decoder output, reused as-is
/// (P1-2 introduces no new decode logic).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TbOp {
    /// A 16-bit Thumb instruction.
    V16(Inst16),
    /// A 32-bit Thumb-2 instruction.
    V32(Inst32),
}

/// Why the builder sealed a block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TbEnd {
    /// Hit [`MAX_TB_INSNS`] (last op included).
    MaxInsns,
    /// A branch/call/return or genuine PC load (included).
    ControlFlow,
    /// A predicated-position op the native predicated set cannot cover
    /// (P2-5, S4: excluded — the interpreter owns predicated execution of
    /// branches, memory, and system ops).
    ItBreak,
    /// An S6 callout op or unpredictable PC-register form (included).
    System,
    /// A bus-writing op (included; the block ends here with a fallthrough
    /// exit). Stores seal blocks so a store is never followed by more
    /// translated ops in the same block: the next block entry revalidates
    /// content (self-modifying code) and config (MPU reprogramming) after
    /// every store, for both engines (P2-3). Native templates additionally
    /// bump the write generation on the fast path.
    Store,
    /// The next instruction starts on another 4 KiB page (excluded).
    PageCross,
    /// The next instruction could not be fetched or decoded (excluded; the
    /// interpreter takes over there, faults included).
    Halt,
}

/// A built (not yet cached) translation block: decoded ops plus the
/// footprint and accounting the cache needs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TbBody {
    /// Guest PC of the first op.
    pub pc: u32,
    /// Security state (recorded for the key; fetch uses the stepper path).
    pub secure: bool,
    /// ISA variant decoded under.
    pub isa: Isa,
    /// Decoded ops in order; always non-empty.
    pub ops: Vec<TbOp>,
    /// Fallthrough address past the last op.
    pub end_pc: u32,
    /// Why the block was sealed.
    pub end: TbEnd,
    /// First 4 KiB code page spanned.
    pub start_page: u32,
    /// Contiguous code pages spanned (`>= 1`).
    pub page_count: u32,
    /// Approximate cycle cost (D7): 1 per op plus 1 per memory-touching op.
    pub cycle_estimate: u64,
    /// Bit `i` set when op `i` performs a bus write (P1-6): the runner
    /// revalidates content + config only after write ops, so hot loops
    /// without stores pay no per-op polling. At most [`MAX_TB_INSNS`] ops,
    /// so a `u64` always suffices.
    pub store_mask: u64,
    /// ITSTATE at block entry (P2-5, S4): nonzero for predicated-
    /// continuation blocks built under an active IT block. The runner
    /// enters native blocks only on an exact match (unifying the old
    /// blanket `itstate != 0` refusal: plain blocks record 0).
    pub it_entry: u8,
}

impl TbBody {
    /// Convert to cache metadata (the cache stamps the live MPU epoch).
    #[must_use]
    pub fn to_meta(&self) -> TbMeta {
        TbMeta {
            key: crate::jit::TbKey {
                pc: self.pc,
                secure: self.secure,
                isa: self.isa,
                mpu_epoch: MpuEpoch::initial(),
            },
            insns: self.ops.len() as u16,
            len_bytes: self.end_pc.wrapping_sub(self.pc),
            start_page: self.start_page,
            page_count: self.page_count,
            cycle_estimate: self.cycle_estimate,
            store_mask: self.store_mask,
            it_entry: self.it_entry,
            // The runner stamps the real content hash at insert time.
            check: 0,
            write_gen: 0,
        }
    }
}

/// How one instruction continues or seals a block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Seal {
    Continue,
    ControlFlow,
    System,
}

/// Byte length of one op (shared by the runner and the block emitter).
#[must_use]
pub(crate) const fn op_len(op: &TbOp) -> u32 {
    match op {
        TbOp::V16(_) => 2,
        TbOp::V32(_) => 4,
    }
}

/// Build a translation block starting at `pc`, or `None` when no block can
/// start here (odd PC, first fetch/decode failure, or a predicated-
/// continuation start whose first op the native predicated set cannot
/// cover — the interpreter owns that op). `bus` is read with the same
/// halfword fetches the stepper performs.
///
/// A nonzero `itstate` builds a predicated-continuation block (P2-5, S4):
/// each op must be predicable (ALU-only — branches, memory, and system
/// ops seal the block before them with [`TbEnd::ItBreak`); ITSTATE is
/// simulated statically over the body (draining back to straight-line
/// code when the mask runs out). The runner enters the block natively
/// only on an exact ITSTATE match.
pub fn build_tb<B: Bus>(
    bus: &mut B,
    pc: u32,
    secure: bool,
    isa: Isa,
    itstate: u8,
) -> Option<TbBody> {
    if pc & 1 != 0 {
        return None;
    }
    let start_page = pc >> PAGE_SHIFT;
    let mut ops: Vec<TbOp> = Vec::new();
    let mut cur = pc;
    let mut cycles: u64 = 0;
    let mut store_mask: u64 = 0;
    let mut it = itstate;

    let end = loop {
        if ops.len() >= usize::from(MAX_TB_INSNS) {
            break TbEnd::MaxInsns;
        }
        if (cur >> PAGE_SHIFT) != start_page && !ops.is_empty() {
            break TbEnd::PageCross;
        }
        // Fetch + decode one instruction; on failure seal before it (or
        // refuse when nothing was built yet).
        let Some((op, len)) = fetch_op(bus, cur, isa) else {
            if ops.is_empty() {
                return None;
            }
            break TbEnd::Halt;
        };
        // A 32-bit instruction straddling into the next page joins only an
        // otherwise-empty block (progress guarantee); otherwise the page
        // crossing seals before it.
        let last_byte = cur.wrapping_add(len).wrapping_sub(1);
        if (last_byte >> PAGE_SHIFT) != start_page && !ops.is_empty() {
            break TbEnd::PageCross;
        }
        // Predicated positions only admit the native predicated set (S4);
        // anything else seals before it (or refuses an empty start, so the
        // fallback stepper — which owns predication there — makes progress).
        // `IT` itself always joins, even nested: the fresh value replaces
        // the stale one under both engines, and whatever follows is gated
        // on the fresh mask either way (native simulation or interp).
        let is_it = matches!(op, TbOp::V16(Inst16::It { .. }));
        if it != 0 && !is_it && !super::templates::is_predicable(&op) {
            if ops.is_empty() {
                return None;
            }
            break TbEnd::ItBreak;
        }
        let seal = match op {
            TbOp::V16(inst) => seal16(&inst),
            TbOp::V32(inst) => seal32(&inst),
        };
        cycles += 1 + u64::from(op_touches_memory(&op));
        if op_writes_memory(&op) {
            // `ops.len() < MAX_TB_INSNS <= 64` (checked at loop top).
            store_mask |= 1_u64 << ops.len();
        }
        cur = cur.wrapping_add(len);
        ops.push(op);
        if is_it {
            let TbOp::V16(Inst16::It { cond, mask }) = op else {
                unreachable!("matched IT above")
            };
            it = (cond << 4) | mask;
        } else if it != 0 {
            it = super::templates::it_advance(it);
        }
        match seal {
            Seal::Continue if op_writes_memory(&op) => break TbEnd::Store,
            Seal::Continue => {}
            Seal::ControlFlow => break TbEnd::ControlFlow,
            Seal::System => break TbEnd::System,
        }
    };

    let last_byte = cur.wrapping_sub(1);
    let page_count = (last_byte >> PAGE_SHIFT)
        .wrapping_sub(start_page)
        .saturating_add(1)
        .max(1);
    Some(TbBody {
        pc,
        secure,
        isa,
        ops,
        end_pc: cur,
        end,
        start_page,
        page_count,
        cycle_estimate: cycles,
        store_mask,
        it_entry: itstate,
    })
}

/// Fetch and decode the instruction at `cur` with the stepper's per-ISA
/// dispatch. Returns the op and its byte length. `pub(crate)` for the
/// runner's verify mode (re-decode each stored op from the bus).
pub(crate) fn fetch_op<B: Bus>(bus: &mut B, cur: u32, isa: Isa) -> Option<(TbOp, u32)> {
    let h0 = bus.read_half(cur).ok()?;
    let is32 = match isa {
        Isa::Armv6M => is_thumb2(h0),
        Isa::Armv7M | Isa::Armv7EM | Isa::Armv8MBaseline | Isa::Armv8MMainline => is_thumb2_v7m(h0),
    };
    if !is32 {
        let inst = match isa {
            Isa::Armv6M | Isa::Armv8MBaseline => decode16(h0).ok()?,
            Isa::Armv7M | Isa::Armv7EM | Isa::Armv8MMainline => decode16_v7m(h0).ok()?,
        };
        return Some((TbOp::V16(inst), 2));
    }
    let h1 = bus.read_half(cur.wrapping_add(2)).ok()?;
    let inst = match isa {
        Isa::Armv6M => decode32(h0, h1).ok()?,
        Isa::Armv7M => decode32_v7m(h0, h1).ok()?,
        Isa::Armv7EM => decode32_v7em(h0, h1).ok()?,
        Isa::Armv8MBaseline => decode32_v8m(h0, h1).ok()?,
        Isa::Armv8MMainline => decode32_v8m_mainline(h0, h1).ok()?,
    };
    Some((TbOp::V32(inst), 4))
}

/// Whether the op performs a bus write (P1-6): loads never mutate code or
/// config, so only these ops can invalidate a running block. Reads
/// (including MMIO side-effect reads) execute identically through the
/// shared executor and affect no future decode or permission, so they need
/// no revalidation. Exception entry stacking diverts the PC, which ends the
/// block by construction; DMA/devices never tick mid-burst.
fn op_writes_memory(op: &TbOp) -> bool {
    match op {
        TbOp::V16(inst) => match inst {
            Inst16::LsReg { kind, .. } => {
                matches!(kind, LsReg::Str | LsReg::Strh | LsReg::Strb)
            }
            Inst16::LsImm { kind, .. } => {
                matches!(kind, LsImm::Str | LsImm::Strh | LsImm::Strb)
            }
            Inst16::LsSp { is_load, .. } => !is_load,
            Inst16::Push { .. } => true,
            Inst16::LdmStm { is_load, .. } => !is_load,
            _ => false,
        },
        TbOp::V32(inst) => match inst {
            Inst32::Mem { access, .. } => {
                matches!(access, MemOp::Str | MemOp::Strb | MemOp::Strh)
            }
            Inst32::LdrdStrd { is_load, .. } => !is_load,
            Inst32::LdmStm { is_load, .. } => !is_load,
            // Store-release always writes (exclusive forms seal the block
            // anyway); load-acquire never does.
            Inst32::Stl { .. } => true,
            Inst32::VfpLdSt { is_load, .. }
            | Inst32::VfpLdStD { is_load, .. }
            | Inst32::VfpLdStMult { is_load, .. }
            | Inst32::VfpLdStMultD { is_load, .. } => !is_load,
            _ => false,
        },
    }
}

/// Whether the op performs a data-memory access (cycle-estimate surcharge).
fn op_touches_memory(op: &TbOp) -> bool {
    match op {
        TbOp::V16(inst) => matches!(
            inst,
            Inst16::LsReg { .. }
                | Inst16::LsImm { .. }
                | Inst16::LsSp { .. }
                | Inst16::LdrLiteral { .. }
                | Inst16::Push { .. }
                | Inst16::Pop { .. }
                | Inst16::LdmStm { .. }
        ),
        TbOp::V32(inst) => matches!(
            inst,
            Inst32::Mem { .. }
                | Inst32::LdrdStrd { .. }
                | Inst32::LdmStm { .. }
                | Inst32::Lda { .. }
                | Inst32::Stl { .. }
                | Inst32::VfpLdSt { .. }
                | Inst32::VfpLdStD { .. }
                | Inst32::VfpLdStMult { .. }
                | Inst32::VfpLdStMultD { .. }
        ),
    }
}

/// T2 classification for 16-bit instructions: only plain ALU, plain
/// loads/stores, stack/multiple (no-PC), and benign hints continue.
fn seal16(inst: &Inst16) -> Seal {
    match inst {
        Inst16::Bx { .. }
        | Inst16::Blx { .. }
        | Inst16::BxNs { .. }
        | Inst16::BlxNs { .. }
        | Inst16::Branch { .. }
        | Inst16::BranchCond { .. }
        | Inst16::CbzCbnz { .. } => Seal::ControlFlow,
        Inst16::Pop { load_pc: true, .. } => Seal::ControlFlow,
        Inst16::SpecialAdd { rd: 15, .. } | Inst16::SpecialMov { rd: 15, .. } => Seal::ControlFlow,
        Inst16::LdrLiteral { rt: 15, .. } => Seal::ControlFlow,
        Inst16::Svc { .. } | Inst16::Bkpt { .. } | Inst16::Cps { .. } | Inst16::It { .. } => {
            Seal::System
        }
        Inst16::Hint { kind } => match kind {
            Hint::Nop | Hint::Yield => Seal::Continue,
            Hint::Wfe | Hint::Wfi | Hint::Sev => Seal::System,
        },
        _ => Seal::Continue,
    }
}

/// T2 classification for 32-bit instructions: the S0–S5 continuer set is
/// plain DP (no PC destination), plain loads/stores (no PC target), and
/// `DIV`/`MUL`; everything else — branches, system/exclusive/barrier,
/// acquire-release-exclusive, DSP, FP, `SG`/`TT`-adjacent state readers —
/// seals the block. (`TT` itself is a pure attribution read: continue.)
fn seal32(inst: &Inst32) -> Seal {
    match inst {
        Inst32::Bl { .. }
        | Inst32::BranchWide { .. }
        | Inst32::BranchCondWide { .. }
        | Inst32::BlxImm { .. }
        | Inst32::TblBranch { .. } => Seal::ControlFlow,
        Inst32::DpReg { rd: 15, .. }
        | Inst32::DpImm { rd: 15, .. }
        | Inst32::MovWide { rd: 15, .. }
        | Inst32::Div { rd: 15, .. }
        | Inst32::Mul { rd: 15, .. } => Seal::ControlFlow,
        Inst32::Mem { rt: 15, .. } => Seal::ControlFlow,
        Inst32::LdrdStrd { rt: 15, .. } | Inst32::LdrdStrd { rt2: 15, .. } => Seal::ControlFlow,
        Inst32::LdmStm { regs, .. } if regs & (1_u16 << 15) != 0 => Seal::ControlFlow,
        Inst32::DpReg { .. }
        | Inst32::DpImm { .. }
        | Inst32::MovWide { .. }
        | Inst32::Div { .. }
        | Inst32::Mul { .. }
        | Inst32::Mem { .. }
        | Inst32::LdrdStrd { .. }
        | Inst32::LdmStm { .. }
        | Inst32::Tt { .. } => Seal::Continue,
        Inst32::Lda {
            exclusive: false, ..
        }
        | Inst32::Stl {
            exclusive: false, ..
        } => Seal::Continue,
        _ => Seal::System,
    }
}

#[cfg(test)]
mod tests {
    use crate::error::BusError;

    use super::*;

    /// Byte-addressable stub bus: unmapped reads fail (fetch abort model).
    struct VecMem {
        base: u32,
        data: Vec<u8>,
    }

    impl VecMem {
        fn new(base: u32, size: usize) -> Self {
            Self {
                base,
                data: vec![0; size],
            }
        }

        fn poke_half(&mut self, addr: u32, half: u16) {
            let [lo, hi] = half.to_le_bytes();
            let i = (addr - self.base) as usize;
            self.data[i] = lo;
            self.data[i + 1] = hi;
        }
    }

    impl Bus for VecMem {
        fn read(&mut self, addr: u32) -> Result<u8, BusError> {
            self.data
                .get((addr.wrapping_sub(self.base)) as usize)
                .copied()
                .ok_or(BusError::Reserved)
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            let i = (addr.wrapping_sub(self.base)) as usize;
            if i < self.data.len() {
                self.data[i] = val;
                Ok(())
            } else {
                Err(BusError::Reserved)
            }
        }
    }

    const NOP: u16 = 0xBF00;
    const WFI: u16 = 0xBF30;
    const SVC0: u16 = 0xDF00;
    const POP_PC: u16 = 0xBD00;
    const PUSH_LR: u16 = 0xB500;
    const MOVS_R0_1: u16 = 0x2001;
    const B_SELF: u16 = 0xE7FE;
    /// `MOV pc, r0` (high-register move onto the PC).
    const MOV_PC_R0: u16 = 0x4687;

    fn mem_with(base: u32, halves: &[u16]) -> VecMem {
        let mut mem = VecMem::new(base, halves.len() * 2 + 8);
        for (i, half) in halves.iter().enumerate() {
            mem.poke_half(base + (i as u32) * 2, *half);
        }
        mem
    }

    #[test]
    fn straight_line_seals_at_branch_inclusive() {
        let mut mem = mem_with(0x1000, &[NOP, NOP, NOP, NOP, NOP, B_SELF]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!(body.ops.len(), 6);
        assert_eq!(body.end, TbEnd::ControlFlow);
        assert_eq!(body.end_pc, 0x100C);
        assert_eq!(body.start_page, 1);
        assert_eq!(body.page_count, 1);
        // 6 plain ops, no memory: 6 cycles.
        assert_eq!(body.cycle_estimate, 6);
    }

    #[test]
    fn svc_seals_system_inclusive() {
        let mut mem = mem_with(0x1000, &[MOVS_R0_1, SVC0, NOP]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!(body.ops.len(), 2);
        assert_eq!(body.end, TbEnd::System);
        assert_eq!(body.end_pc, 0x1004);
    }

    #[test]
    fn wfi_and_pop_pc_seal() {
        let mut mem = mem_with(0x1000, &[NOP, WFI]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (2, TbEnd::System));

        let mut mem = mem_with(0x1000, &[PUSH_LR, POP_PC]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        // PUSH seals as a store (P2-3); POP-pc seals the next block.
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Store));
        assert_eq!(body.cycle_estimate, 2);
        let body = build_tb(&mut mem, 0x1002, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::ControlFlow));
    }

    #[test]
    fn mov_onto_pc_is_control_flow() {
        let mut mem = mem_with(0x1000, &[MOV_PC_R0]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::ControlFlow));
    }

    #[test]
    fn bl_is_one_32bit_control_op() {
        // `BL #0`: F000 F800.
        let mut mem = mem_with(0x1000, &[0xF000, 0xF800, NOP]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!(body.ops.len(), 1);
        assert_eq!(body.end, TbEnd::ControlFlow);
        assert_eq!(body.end_pc, 0x1004);
        assert!(matches!(body.ops[0], TbOp::V32(Inst32::Bl { .. })));
    }

    #[test]
    fn movw_continues_as_32bit_plain_op() {
        // `MOVW r0, #0`: F240 0000. Exactly-sized memory (no zero padding:
        // 0x0000 decodes as `LSLs`, a continuer) so the block halts after it.
        let mut mem = VecMem::new(0x1000, 6);
        mem.poke_half(0x1000, 0xF240);
        mem.poke_half(0x1002, 0x0000);
        mem.poke_half(0x1004, NOP);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (2, TbEnd::Halt));
        assert_eq!(body.end_pc, 0x1006);
        assert!(matches!(body.ops[0], TbOp::V32(Inst32::MovWide { .. })));
    }

    #[test]
    fn max_insns_caps_the_block() {
        let halves = vec![NOP; 70];
        let mut mem = mem_with(0x1000, &halves);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv6M, 0).unwrap();
        assert_eq!(body.ops.len(), usize::from(MAX_TB_INSNS));
        assert_eq!(body.end, TbEnd::MaxInsns);
        assert_eq!(body.end_pc, 0x1000 + u32::from(MAX_TB_INSNS) * 2);
    }

    #[test]
    fn page_cross_seals_before_the_next_page() {
        // NOP ends at 0x2000: next instruction starts a new page.
        let mut mem = mem_with(0x1FFE, &[NOP, NOP]);
        let body = build_tb(&mut mem, 0x1FFE, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::PageCross));
        assert_eq!(body.end_pc, 0x2000);
        assert_eq!(body.page_count, 1);
    }

    #[test]
    fn straddling_first_op_joins_for_progress() {
        // 32-bit BL at 0x1FFE spans pages 1-2; as the first op it is kept.
        let mut mem = mem_with(0x1FFE, &[0xF000, 0xF800, NOP]);
        let body = build_tb(&mut mem, 0x1FFE, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::ControlFlow));
        assert_eq!(body.page_count, 2);
        // ...but a straddler after built ops seals first.
        let mut mem = mem_with(0x1FFC, &[NOP, 0xF000, 0xF800]);
        let body = build_tb(&mut mem, 0x1FFC, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::PageCross));
        assert_eq!(body.page_count, 1);
    }

    #[test]
    fn itstate_and_odd_pc_refuse() {
        let mut mem = mem_with(0x1000, &[NOP]);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 1).is_none());
        assert!(build_tb(&mut mem, 0x1001, false, Isa::Armv7M, 0).is_none());
    }

    #[test]
    fn it_instruction_seals_system() {
        // `IT EQ` (0xBF08) sets ITSTATE: the block ends here, the predicated
        // body runs interpreted (entry refusal above).
        let mut mem = mem_with(0x1000, &[0xBF08, NOP]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::System));
    }

    #[test]
    fn fetch_failure_refuses_or_halts() {
        // Nothing mapped at all: refuse.
        let mut mem = VecMem::new(0x5000, 0);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).is_none());
        // NOP then unmapped: seal before the hole; the interpreter owns it.
        // (Exactly 2 bytes mapped: no zero padding to decode through.)
        let mut mem = VecMem::new(0x1000, 2);
        mem.poke_half(0x1000, NOP);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Halt));
    }

    #[test]
    fn undecodable_refuses_or_halts() {
        // 0xBF50 is outside the HINT space: undefined on every ISA.
        let mut mem = mem_with(0x1000, &[0xBF50, NOP]);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).is_none());
        let mut mem = mem_with(0x1000, &[NOP, 0xBF50]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Halt));
    }

    #[test]
    fn stores_seal_blocks_inclusive() {
        // STR seals (P2-3): the NOP after it starts a new block.
        let mut mem = mem_with(0x1000, &[0x600A, NOP, B_SELF]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Store));
        assert_eq!(body.end_pc, 0x1002);
        // Pure loads do not seal.
        let mut mem = mem_with(0x1000, &[0x4800, NOP, B_SELF]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (3, TbEnd::ControlFlow));
    }

    #[test]
    fn store_mask_marks_write_ops_only() {
        // STR seals (P2-3), so single-op blocks: mask bit 0 set.
        // Exactly-sized memory: no zero padding to decode through.
        let mut mem = VecMem::new(0x1000, 2);
        mem.poke_half(0x1000, 0x600A);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Store));
        assert_eq!(body.store_mask, 0b1);
        assert_eq!(body.to_meta().store_mask, 0b1);
        // PUSH seals the same way (bit 0); a lone LDR has an empty mask.
        let mut mem = mem_with(0x1000, &[PUSH_LR, POP_PC]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (1, TbEnd::Store));
        assert_eq!(body.store_mask, 0b1);
        let mut mem = VecMem::new(0x1000, 2);
        mem.poke_half(0x1000, 0x4800);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!(body.store_mask, 0);
    }

    #[test]
    fn body_converts_to_cache_meta() {
        // LDR r0,[pc,#0] (0x4800) touches memory: 1 + 2 cycles.
        let mut mem = mem_with(0x1000, &[0x4800, NOP, B_SELF]);
        let body = build_tb(&mut mem, 0x1000, true, Isa::Armv6M, 0).unwrap();
        assert_eq!(body.ops.len(), 3);
        assert_eq!(body.cycle_estimate, 2 + 1 + 1);
        let meta = body.to_meta();
        assert_eq!(meta.key.pc, 0x1000);
        assert!(meta.key.secure);
        assert_eq!(meta.key.isa, Isa::Armv6M);
        assert_eq!(meta.key.mpu_epoch, MpuEpoch::initial());
        assert_eq!(meta.insns, 3);
        assert_eq!(meta.len_bytes, 6);
    }

    #[test]
    fn isa_dispatch_mirrors_the_stepper() {
        // `AND.W r0, r0, r0`: EA00 0000. v6-M sees a 16-bit halfword the
        // v6 decoder rejects; the v8-M baseline sees Thumb-2 outside the M23
        // subset; v7-M/mainline decode it as plain continuing DP.
        // (Note: CBZ does NOT discriminate — the 16-bit decoders accept it
        // on every ISA and the executor gates it.)
        let mut mem = mem_with(0x1000, &[0xEA00, 0x0000, B_SELF]);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv6M, 0).is_none());
        let mut mem = mem_with(0x1000, &[0xEA00, 0x0000, B_SELF]);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv8MBaseline, 0).is_none());
        let mut mem = mem_with(0x1000, &[0xEA00, 0x0000, B_SELF]);
        let body = build_tb(&mut mem, 0x1000, false, Isa::Armv7M, 0).unwrap();
        assert_eq!((body.ops.len(), body.end), (2, TbEnd::ControlFlow));
        assert!(matches!(body.ops[0], TbOp::V32(Inst32::DpReg { .. })));
        let mut mem = mem_with(0x1000, &[0xEA00, 0x0000, B_SELF]);
        assert!(build_tb(&mut mem, 0x1000, false, Isa::Armv8MMainline, 0).is_some());
    }
}

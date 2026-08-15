//! Running binaries under the `vioarm-core` backend for the conformance suite.
//!
//! The VioARM side uses a `SystemMemory` over a bare `FlatMemory` (flash at
//! `0x0000_0000`, SRAM at `0x2000_0000`, matching the `mps2-an385` machine)
//! with no peripherals attached, so no interrupt can ever be taken and the
//! instruction stream is deterministic — exactly parallel to QEMU, where
//! everything on the board sits idle. The system bus's `SCB` claims the
//! `CPACR` window so firmware that enables the FPU (as Cortex-M4 images do)
//! does not fault; the VFP register bank and FPSCR are captured in every
//! snapshot for the ARMv7E-M conformance tests.

use crate::{CpuState, VfpState};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::system::SystemMemory;

pub const FLASH_LEN: u32 = 0x0020_0000;
pub const SRAM_LEN: u32 = 0x0020_0000;

/// Build an ARMv7-M machine over a bare memory and load `bin` at address 0.
#[must_use]
pub fn new_machine(bin: &[u8]) -> Machine<SystemMemory<FlatMemory>> {
    let backing = FlatMemory::with_layout(
        &[0x0000_0000, 0x2000_0000],
        &[FLASH_LEN, SRAM_LEN],
    );
    let mut m = Machine::armv7m(SystemMemory::new(backing));
    m.load_image(&load_flat(0x0000_0000, bin));
    m
}

/// Snapshot the machine's architectural state. `pc` is stored Thumb-masked by
/// the core, exactly as QEMU reports `R15`. `xpsr` carries the modeled xPSR
/// bits plus the CPU's live `ITSTATE` folded in at bits [26:25]/[15:10], the
/// same positions QEMU's `XPSR=` dump uses, so IT-block execution is diffed.
#[must_use]
pub fn snapshot(m: &Machine<SystemMemory<FlatMemory>>) -> CpuState {
    let regs = m.core.regs;
    let mut r = [0u32; 16];
    r[..13].copy_from_slice(&regs.r);
    r[13] = regs.sp;
    r[14] = regs.lr;
    r[15] = regs.pc;
    let it = u32::from(m.core.itstate);
    let xpsr = m.core.xpsr.get() | ((it & 0xFC) << 8) | ((it & 0x03) << 25);
    let vfp = VfpState {
        s: m.core.vfp.s,
        fpscr: m.core.vfp.fpscr,
    };
    CpuState {
        r,
        xpsr,
        vfp: Some(vfp),
    }
}

/// Run `bin` under VioARM, collecting the state after every instruction until
/// the PC reaches `stop_pc` (the address of the landing loop). Returns the
/// trace and whether the landing loop was reached.
#[must_use]
pub fn run_until_pc(bin: &[u8], stop_pc: u32) -> (Vec<CpuState>, bool) {
    let mut m = new_machine(bin);
    m.reset().expect("reset should pick the reset vector");
    let mut states = Vec::new();
    states.push(snapshot(&m));
    let budget = 200_000u32;
    for _ in 0..budget {
        // A halt (undefined instruction, fault, ...) is a divergence finding
        // and must surface as a trace that never reaches the landing loop.
        // Record the halted state so it can be diffed against QEMU's lockup
        // frame; a halt that does not advance the PC produces a state identical
        // to the previous one, which is collapsed exactly like QEMU's parser.
        if m.step().is_err() {
            let s = snapshot(&m);
            if states.last() != Some(&s) {
                states.push(s);
            }
            return (states, false);
        }
        states.push(snapshot(&m));
        if m.core.regs.pc & !1 == stop_pc {
            return (states, true);
        }
    }
    (states, false)
}
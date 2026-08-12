//! Running binaries under the `vioarm-core` backend for the conformance suite.
//!
//! The VioARM side uses a bare `FlatMemory` (flash at `0x0000_0000`, SRAM at
//! `0x2000_0000`, matching the `mps2-an385` machine) with no peripherals, so
//! no interrupt can ever be taken and the instruction stream is deterministic
//! — exactly parallel to QEMU, where everything on the board sits idle.

use crate::CpuState;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;

pub const FLASH_LEN: u32 = 0x0020_0000;
pub const SRAM_LEN: u32 = 0x0020_0000;

/// Build an ARMv7-M machine over a bare memory and load `bin` at address 0.
#[must_use]
pub fn new_machine(bin: &[u8]) -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(
        &[0x0000_0000, 0x2000_0000],
        &[FLASH_LEN, SRAM_LEN],
    );
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(0x0000_0000, bin));
    m
}

/// Snapshot the machine's architectural state. `pc` is stored Thumb-masked by
/// the core, exactly as QEMU reports `R15`.
#[must_use]
pub fn snapshot(m: &Machine<FlatMemory>) -> CpuState {
    let regs = m.core.regs;
    let mut r = [0u32; 16];
    r[..13].copy_from_slice(&regs.r);
    r[13] = regs.sp;
    r[14] = regs.lr;
    r[15] = regs.pc;
    CpuState {
        r,
        xpsr: m.core.xpsr.get(),
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
        if m.step().is_err() {
            return (states, false);
        }
        states.push(snapshot(&m));
        if m.core.regs.pc & !1 == stop_pc {
            return (states, true);
        }
    }
    (states, false)
}
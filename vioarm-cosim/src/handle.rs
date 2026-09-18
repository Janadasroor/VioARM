//! A live simulated machine behind an opaque C handle.
//!
//! The struct is deliberately thin: construction policy (board lookup,
//! core selection, image load, reset) lives in
//! `vioarm_core::machine::Machine::for_name`, so the CLI and the
//! co-simulation path cannot diverge.

use std::os::raw::c_void;

use vioarm_core::board::BoardMemory;
use vioarm_core::machine::Machine;

use super::VioArmEventCallback;

/// How the last stepping call (`step_duration` / `run_quantum`) ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HaltOutcome {
    /// The full budget retired (or nothing was ever stepped).
    Completed,
    /// A host breakpoint fired before the step at `pc`.
    Breakpoint(u32),
    /// The core halted (fault, reset request, ... — detail in last error).
    Fault,
}

/// A live simulated machine behind an opaque C handle.
pub struct VioArmHandle {
    machine: Machine<BoardMemory>,
    /// Board short name (for `load`, which rebuilds on the same board).
    board: String,
    /// Last host-driven GPIO word per port (for `pin_set` read-modify-write:
    /// firmware cannot change host levels, only pin modes, so this cache is
    /// exact between steps).
    driven: [u32; 16],
    /// Host execution breakpoints (match `pc` before a step). Empty is the
    /// fast path: stepping calls run unmodified single-call quanta.
    breakpoints: Vec<u32>,
    /// Outcome of the last stepping call (for `halt_reason`).
    last_halt: HaltOutcome,
    /// Host event callback + context (E1.5; `None` disables).
    callback: (VioArmEventCallback, *mut c_void),
    /// Set while a host callback runs: reentrant entries fail closed.
    in_callback: bool,
}

impl VioArmHandle {
    /// Wrap a reset machine (see `Machine::for_name` for the policy).
    #[must_use]
    pub fn new(machine: Machine<BoardMemory>, board: String) -> Self {
        Self {
            machine,
            board,
            driven: [0; 16],
            breakpoints: Vec::new(),
            last_halt: HaltOutcome::Completed,
            callback: (None, std::ptr::null_mut()),
            in_callback: false,
        }
    }

    /// The simulated machine (exclusive access — handles are single-threaded).
    pub fn machine_mut(&mut self) -> &mut Machine<BoardMemory> {
        &mut self.machine
    }

    /// The board short name (what `create` was given).
    #[must_use]
    pub fn board(&self) -> &str {
        &self.board
    }

    /// The cached host-driven word for `port`.
    #[must_use]
    pub fn driven(&self, port: usize) -> u32 {
        self.driven[port]
    }

    /// Store the host-driven word for `port` after driving it.
    pub fn set_driven(&mut self, port: usize, mask: u32) {
        self.driven[port] = mask;
    }

    /// Forget host-driven state (after `reset`/`load`: a fresh boot owns
    /// nothing until the host drives again).
    pub fn clear_driven(&mut self) {
        self.driven = [0; 16];
    }

    /// Breakpoint set (arbitrary length; scans are linear, so hosts should
    /// keep it short — a 256-entry cap keeps worst-case step cost bounded).
    pub fn breakpoints(&self) -> &[u32] {
        &self.breakpoints
    }

    /// Arm a breakpoint (`false` when already present or the cap is hit).
    pub fn break_add(&mut self, pc: u32) -> bool {
        if self.breakpoints.contains(&pc) || self.breakpoints.len() >= 256 {
            return false;
        }
        self.breakpoints.push(pc);
        true
    }

    /// Disarm a breakpoint (idempotent).
    pub fn break_remove(&mut self, pc: u32) {
        self.breakpoints.retain(|armed| *armed != pc);
    }

    /// Outcome of the last stepping call.
    #[must_use]
    pub fn last_halt(&self) -> HaltOutcome {
        self.last_halt
    }

    /// Record the outcome of a stepping call.
    pub fn set_last_halt(&mut self, outcome: HaltOutcome) {
        self.last_halt = outcome;
    }

    /// The host event callback + context.
    #[must_use]
    pub fn callback(&self) -> (VioArmEventCallback, *mut c_void) {
        self.callback
    }

    /// Install (`None` disables) the host event callback + context.
    pub fn set_callback(&mut self, callback: VioArmEventCallback, ctx: *mut c_void) {
        self.callback = (callback, ctx);
    }

    /// Whether a host callback is currently running (reentrancy guard).
    #[must_use]
    pub fn in_callback(&self) -> bool {
        self.in_callback
    }

    /// Mark callback entry/exit (always paired, never observed by hosts).
    pub fn set_in_callback(&mut self, active: bool) {
        self.in_callback = active;
    }
}

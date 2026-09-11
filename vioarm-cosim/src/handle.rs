//! A live simulated machine behind an opaque C handle.
//!
//! The struct is deliberately thin: construction policy (board lookup,
//! core selection, image load, reset) lives in
//! `vioarm_core::machine::Machine::for_name`, so the CLI and the
//! co-simulation path cannot diverge.

use vioarm_core::board::BoardMemory;
use vioarm_core::machine::Machine;

/// A live simulated machine behind an opaque C handle.
pub struct VioArmHandle {
    machine: Machine<BoardMemory>,
    /// Last host-driven GPIO word per port (for `pin_set` read-modify-write:
    /// firmware cannot change host levels, only pin modes, so this cache is
    /// exact between steps).
    driven: [u32; 16],
}

impl VioArmHandle {
    /// Wrap a reset machine (see `Machine::for_name` for the policy).
    #[must_use]
    pub fn new(machine: Machine<BoardMemory>) -> Self {
        Self {
            machine,
            driven: [0; 16],
        }
    }

    /// The simulated machine (exclusive access — handles are single-threaded).
    pub fn machine_mut(&mut self) -> &mut Machine<BoardMemory> {
        &mut self.machine
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
}

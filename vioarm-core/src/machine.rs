//! A `Machine` couples a `Core` with its `FlatMemory` and provides the
//! host-facing API: reset, run until halt, and instant image loading.

use crate::cpu::Core;
use crate::error::HaltCause;
use crate::loader::LoadedImage;
use crate::mem::bus::Bus;
use crate::mem::flat::FlatMemory;

/// A ready-to-run core+memory pair. The core owns the memory; the machine is
/// the host-facing facade.
pub struct Machine {
    pub core: Core<FlatMemory>,
}

impl Machine {
    #[must_use]
    pub fn new(mem: FlatMemory) -> Self {
        Self {
            core: Core::new(mem),
        }
    }

    /// Load an image's segments into the machine's memory.
    pub fn load_image(&mut self, img: &LoadedImage) {
        let mem = self.core.mem();
        for (addr, bytes) in &img.segments {
            for (i, &b) in bytes.iter().enumerate() {
                let _ = mem.write(addr.wrapping_add(i as u32), b);
            }
        }
    }

    /// Boot from the reset vector table (`[0x0000_0000]` SP, `[0x0000_0004]`
    /// PC). Silently ignored if there is nothing there yet.
    pub fn reset(&mut self) -> Result<(), HaltCause> {
        self.core.reset_from_vector_table()
    }

    /// Step one instruction.
    pub fn step(&mut self) -> Result<(), HaltCause> {
        self.core.step()
    }

    /// Run until the machine halts of its own accord or `budget` instructions
    /// have executed.
    pub fn run(&mut self, budget: u64) -> Result<(), HaltCause> {
        self.core.run(budget)
    }

    /// The number of executed instructions so far.
    #[must_use]
    pub const fn cycles(&self) -> u64 {
        self.core.cycles
    }
}

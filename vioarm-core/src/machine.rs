//! A `Machine` couples a `Core` with its memory and provides the host-facing
//! API: reset, run until halt, and instant image loading.
//!
//! The memory is any `PeripheralBus`, so the same facade works for a bare
//! `FlatMemory` and for a `SystemMemory` with peripherals attached. The
//! machine is where peripherals get their time: every step retires an
//! instruction (or takes an exception), then advances the bus by one cycle so
//! timers count and freshly-asserted interrupt lines reach the NVIC.

use crate::cpu::{Core, FpuCtx};
use crate::error::HaltCause;
use crate::loader::LoadedImage;
use crate::mem::PeripheralBus;
use std::sync::{Arc, Mutex};

/// A ready-to-run core+memory pair. The core owns the memory; the machine is
/// the host-facing facade.
pub struct Machine<M: PeripheralBus> {
    pub core: Core<M>,
}

impl<M: PeripheralBus> Machine<M> {
    #[must_use]
    pub fn new(mem: M) -> Self {
        Self {
            core: Core::new(mem),
        }
    }

    /// A machine over an ARMv7-M (Cortex-M3/M4) core.
    #[must_use]
    pub fn armv7m(mem: M) -> Self {
        Self {
            core: Core::armv7m(mem),
        }
    }

    /// An ARMv7-M machine whose FP context-control state is shared between the
    /// core and the `SCB` device. `mem` must be a `SystemMemory` built with the
    /// same `fpu_ctx`.
    #[must_use]
    pub fn armv7m_with_fpu(mem: M, fpu_ctx: Arc<Mutex<FpuCtx>>) -> Self {
        Self {
            core: Core::armv7m_with_fpu(mem, fpu_ctx),
        }
    }

    /// A machine over an ARMv8-M baseline (Cortex-M23) core.
    #[must_use]
    pub fn armv8m(mem: M) -> Self {
        Self {
            core: Core::armv8m(mem),
        }
    }

    /// Load an image's segments into the machine's memory.
    #[allow(clippy::cast_possible_truncation)]
    pub fn load_image(&mut self, img: &LoadedImage) {
        let mem = self.core.mem();
        for (addr, bytes) in &img.segments {
            for (i, &b) in bytes.iter().enumerate() {
                let _ = mem.write(addr.wrapping_add(i as u32), b);
            }
        }
    }

    /// Boot from the reset vector table (SP at the vector base, PC four bytes
    /// later — address 0 for flash-at-0 machines, or the base configured via
    /// [`Core::set_vector_base`]), resetting peripherals to their post-reset
    /// state. Silently ignored if there is nothing there yet.
    ///
    /// # Errors
    ///
    /// Returns [`HaltCause::InvalidVector`] if the reset vector points at a
    /// non-executable or misaligned address.
    pub fn reset(&mut self) -> Result<(), HaltCause> {
        self.core.mem().reset_peripherals();
        self.core.reset_from_vector_table()
    }

    /// Step one machine cycle: either take a pending exception (if one
    /// outranks the running context) or retire one instruction, then advance
    /// peripherals.
    ///
    /// A software-pended exception (an `SHCSR.SVCALLPENDED` or a NVIC
    /// `SETPEND` write) is taken at the same boundary QEMU uses: QEMU's
    /// per-instruction dump after the pending write already shows the entry,
    /// so an instruction that pends an exception is followed by that
    /// exception within the same step, never by an extra thread-mode state.
    ///
    /// # Errors
    ///
    /// Returns the halt cause if the machine halts while stepping.
    pub fn step(&mut self) -> Result<(), HaltCause> {
        // AIRCR.SYSRESETREQ resets the whole machine: peripherals to their
        // post-reset state and the core re-booted from the reset vector table.
        // The SCB's reset clears the request latch, so the reset runs exactly
        // once.
        if self.core.mem().reset_requested() {
            self.reset()?;
        }
        // An exception return (`bx lr` with an EXC_RETURN value) sets the
        // core's `just_returned` flag: the architecture guarantees the
        // restored instruction executes before a new exception is taken, so
        // this step must run that instruction instead of re-entering the
        // handler. Otherwise a level-sensitive line that stays asserted (for
        // example a transmit buffer flag left set under `ITBUFEN`) would
        // re-enter the handler forever and starve the thread of its first
        // instruction back.
        let just_returned = std::mem::take(&mut self.core.just_returned);
        let (primask, faultmask, basepri, ipsr) = (
            self.core.primask,
            self.core.faultmask,
            self.core.basepri,
            self.core.ipsr,
        );
        if just_returned {
            self.run_instruction()?;
        } else if let Some(n) = self
            .core
            .mem()
            .next_exception(primask, faultmask, basepri, ipsr)
        {
            self.core.enter_exception(n)?;
            self.core.mem().acknowledge_exception(n);
            self.core.cycles += 1;
        } else {
            self.run_instruction()?;
        }
        self.core.mem().tick_cycle();
        Ok(())
    }

    /// Retire one instruction, then (unless that instruction was an exception
    /// return) take any exception that now outranks the running context.
    fn run_instruction(&mut self) -> Result<(), HaltCause> {
        // Whether the core was already asleep: if so its `step()` returns
        // immediately without a cycle, so we count the sleeping cycle here.
        let was_sleeping = self.core.sleeping;
        self.core.step()?;
        if self.core.just_returned {
            // The instruction was an exception return: the restored thread
            // instruction must execute next, before any pending exception.
            return Ok(());
        }
        let (primask, faultmask, basepri, ipsr) = (
            self.core.primask,
            self.core.faultmask,
            self.core.basepri,
            self.core.ipsr,
        );
        if let Some(n) = self
            .core
            .mem()
            .next_exception(primask, faultmask, basepri, ipsr)
        {
            self.core.enter_exception(n)?;
            self.core.mem().acknowledge_exception(n);
            self.core.cycles += 1;
        } else if was_sleeping {
            // A sleeping core that no exception wakes still spends the
            // cycle asleep; count it so `cycles()` advances monotonically
            // while firmware sits in `WFI`/`WFE` (peripherals keep ticking).
            self.core.cycles += 1;
        }
        Ok(())
    }

    /// Run until the machine halts of its own accord or `budget` cycles have
    /// elapsed (whichever comes first).
    ///
    /// # Errors
    ///
    /// Returns the halt cause if the machine halts before the budget runs out,
    /// or [`HaltCause::CycleBudgetExhausted`] otherwise.
    pub fn run(&mut self, budget: u64) -> Result<(), HaltCause> {
        for _ in 0..budget {
            self.step()?;
        }
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// The number of executed machine cycles so far.
    #[must_use]
    pub const fn cycles(&self) -> u64 {
        self.core.cycles
    }
}

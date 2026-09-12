//! A `Machine` couples a `Core` with its memory and provides the host-facing
//! API: reset, run until halt, and instant image loading.
//!
//! The memory is any `PeripheralBus`, so the same facade works for a bare
//! `FlatMemory` and for a `SystemMemory` with peripherals attached. The
//! machine is where peripherals get their time: every step retires an
//! instruction (or takes an exception), then advances the bus by one cycle so
//! timers count and freshly-asserted interrupt lines reach the NVIC.

use crate::board::{self, BoardMemory};
use crate::cpu::{Core, FpuCtx};
use crate::device::GpioPinMode;
use crate::error::HaltCause;
use crate::hooks::HookEvent;
use crate::loader::LoadedImage;
use crate::mem::PeripheralBus;
use std::sync::{Arc, Mutex};

/// A ready-to-run core+memory pair. The core owns the memory; the machine is
/// the host-facing facade.
pub struct Machine<M: PeripheralBus> {
    /// The CPU core and its memory.
    pub core: Core<M>,
    /// Fast-mode quantum in steps (`0` = accurate mode, the default): NVIC
    /// scans and device-tick flushes happen on quantum boundaries only.
    /// See [`Machine::set_fast_quantum`].
    fast_quantum: u64,
    /// Steps remaining until the next boundary.
    fast_countdown: u64,
    /// Device cycles owed by retired instructions, flushed on boundaries.
    fast_debt: u64,
}

impl<M: PeripheralBus> Machine<M> {
    /// A machine over an ARMv6-M core.
    #[must_use]
    pub fn new(mem: M) -> Self {
        Self {
            core: Core::new(mem),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// A machine over an ARMv7-M (Cortex-M3/M4) core.
    #[must_use]
    pub fn armv7m(mem: M) -> Self {
        Self {
            core: Core::armv7m(mem),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// A machine over an ARMv7E-M (Cortex-M7) core with FPv5-D16.
    #[must_use]
    pub fn armv7em(mem: M) -> Self {
        Self {
            core: Core::armv7em(mem),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// An ARMv7-M machine whose FP context-control state is shared between the
    /// core and the `SCB` device. `mem` must be a `SystemMemory` built with the
    /// same `fpu_ctx`.
    #[must_use]
    pub fn armv7m_with_fpu(mem: M, fpu_ctx: Arc<Mutex<FpuCtx>>) -> Self {
        Self {
            core: Core::armv7m_with_fpu(mem, fpu_ctx),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// An ARMv7E-M machine whose FP context-control state is shared.
    #[must_use]
    pub fn armv7em_with_fpu(mem: M, fpu_ctx: Arc<Mutex<FpuCtx>>) -> Self {
        Self {
            core: Core::armv7em_with_fpu(mem, fpu_ctx),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// A machine over an ARMv8-M baseline (Cortex-M23) core.
    #[must_use]
    pub fn armv8m(mem: M) -> Self {
        Self {
            core: Core::armv8m(mem),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// A machine over an ARMv8-M mainline (Cortex-M33) core: full Thumb-2
    /// with DSP and single-precision VFP.
    #[must_use]
    pub fn armv8m_mainline(mem: M) -> Self {
        Self {
            core: Core::armv8m_mainline(mem),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
        }
    }

    /// An ARMv8-M mainline machine whose FP context-control state is shared
    /// between the core and the `SCB` device. `mem` must be a `SystemMemory`
    /// built with the same `fpu_ctx`.
    #[must_use]
    pub fn armv8m_mainline_with_fpu(mem: M, fpu_ctx: Arc<Mutex<FpuCtx>>) -> Self {
        Self {
            core: Core::armv8m_mainline_with_fpu(mem, fpu_ctx),
            fast_quantum: 0,
            fast_countdown: 0,
            fast_debt: 0,
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
    /// Returns [`HaltCause::NonThumbTarget`] if the reset vector points at a
    /// non-executable or misaligned address.
    pub fn reset(&mut self) -> Result<(), HaltCause> {
        self.core.mem().reset_peripherals();
        // Host channels are session state: a fresh boot starts with empty
        // semihost/RTT buffers (the enabled flag survives, it is host config).
        self.core.semihost.reset_session();
        // Fast scheduling restarts too (the quantum is host config and
        // survives), so no stale device debt leaks into the fresh boot.
        self.fast_countdown = 0;
        self.fast_debt = 0;
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
    /// Fast mode: run with coarse device time. `quantum` is the number of
    /// steps between NVIC scans and device-tick flushes (`0` disables fast
    /// mode and restores the accurate machine). Instructions, bus traffic,
    /// hooks, and `cycles`/`cyccnt` stay exact; only *device timing* goes
    /// coarse: timers, baud edges, and IRQ delivery can lag up to `quantum`
    /// steps behind the retired instruction (never ahead), and `WFI` wakes
    /// late by the same bound. Device state may also lag up to one quantum
    /// behind at any observation point — call [`Machine::settle_fast`]
    /// (done automatically at the end of [`Machine::run`]) to land owed
    /// time before reading devices. Valid for timing-insensitive workloads
    /// (soak runs, fuzzing, compute kernels, crypto loops); invalid for
    /// timing tests, IRQ-latency tests, and baud-sensitive flows. Reruns
    /// stay byte-identical. Outstanding device debt flushes immediately,
    /// so switching modes never strands device time.
    pub fn set_fast_quantum(&mut self, quantum: u64) {
        let debt = core::mem::take(&mut self.fast_debt);
        if debt != 0 {
            self.core.mem().tick_by(debt);
        }
        self.fast_quantum = quantum;
        self.fast_countdown = 0;
    }

    /// The fast-mode quantum (`0` = accurate mode).
    #[must_use]
    pub const fn fast_quantum(&self) -> u64 {
        self.fast_quantum
    }

    /// Whether this step is a fast-mode boundary (scans + tick flush run).
    /// Accurate mode (`quantum == 0`) treats every step as a boundary, so
    /// the default path below is untouched.
    fn fast_boundary(&mut self) -> bool {
        if self.fast_quantum == 0 {
            return true;
        }
        if self.fast_countdown == 0 {
            self.fast_countdown = self.fast_quantum.saturating_sub(1);
            true
        } else {
            self.fast_countdown -= 1;
            false
        }
    }

    /// Bank device cycles from one step, flushing on boundaries. With
    /// `quantum == 0` this is exactly the old per-step `tick_by`, so the
    /// accurate machine is bit-for-bit unchanged.
    fn tick_fast(&mut self, delta: u64, boundary: bool) {
        self.fast_debt = self.fast_debt.wrapping_add(delta);
        if boundary {
            let debt = core::mem::take(&mut self.fast_debt);
            self.core.mem().tick_by(debt);
        }
    }

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
        let boundary = self.fast_boundary();
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
            let before = self.core.cyccnt;
            self.run_instruction(boundary)?;
            let delta = self.core.cyccnt.wrapping_sub(before);
            self.tick_fast(delta.max(1), boundary);
        } else if boundary {
            if let Some(n) = self
                .core
                .mem()
                .next_exception(primask, faultmask, basepri, ipsr)
            {
                self.core.enter_exception(n)?;
                self.core.mem().acknowledge_exception(n);
                self.core.cycles += 1;
                self.core.cyccnt += 1;
                self.tick_fast(1, true);
            } else {
                let before = self.core.cyccnt;
                self.run_instruction(true)?;
                let delta = self.core.cyccnt.wrapping_sub(before);
                if delta == 0 {
                    // run_instruction already ticked (e.g. just_returned path); avoid double tick
                    self.tick_fast(1, true);
                } else {
                    self.tick_fast(delta, true);
                }
            }
        } else {
            // Fast inter-boundary step: retire only. Scans and device ticks
            // wait for the boundary; the debt keeps every cycle accounted.
            let before = self.core.cyccnt;
            self.run_instruction(false)?;
            let delta = self.core.cyccnt.wrapping_sub(before);
            self.tick_fast(delta.max(1), false);
        }
        Ok(())
    }

    /// Retire one instruction, then (unless that instruction was an exception
    /// return) take any exception that now outranks the running context —
    /// the scan only runs on fast-mode boundaries (`boundary`), which the
    /// caller established in [`Machine::step`].
    fn run_instruction(&mut self, boundary: bool) -> Result<(), HaltCause> {
        // Whether the core was already asleep: if so its `step()` returns
        // immediately without a cycle, so we count the sleeping cycle here.
        let was_sleeping = self.core.sleeping;
        // Instruction hooks bracket the retired instruction (never for an
        // already-sleeping core, which retires nothing).
        let hooked = !was_sleeping
            && self
                .core
                .mem()
                .hooks()
                .is_some_and(|h| h.has_before() || h.has_after());
        let pc = self.core.regs.pc;
        if hooked {
            let cycles = self.core.cyccnt;
            if let Some(h) = self.core.mem().hooks_mut() {
                if h.has_before() {
                    h.fire_before(pc, cycles);
                }
            }
        }
        self.core.step()?;
        if hooked {
            let cycles = self.core.cyccnt;
            if let Some(h) = self.core.mem().hooks_mut() {
                if h.has_after() {
                    h.fire_after(pc, cycles);
                }
            }
        }
        if self.core.just_returned {
            // The instruction was an exception return: the restored thread
            // instruction must execute next, before any pending exception.
            return Ok(());
        }
        if !boundary {
            if was_sleeping {
                // A sleeping core that no exception wakes still spends the
                // cycle asleep; the caller banks it as device debt.
                self.core.cycles += 1;
                self.core.cyccnt += 1;
            }
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
            self.core.cyccnt += 1;
        } else if was_sleeping {
            // A sleeping core that no exception wakes still spends the
            // cycle asleep; count it so `cycles()` advances monotonically
            // while firmware sits in `WFI`/`WFE` (peripherals keep ticking).
            self.core.cycles += 1;
            self.core.cyccnt += 1;
        }
        Ok(())
    }

    /// Run until the machine halts of its own accord or `budget` cycles have
    /// elapsed (whichever comes first). Either way, outstanding fast-mode
    /// device debt flushes first, so post-run observations (register reads,
    /// drains, snapshots) see settled device state — a halted machine never
    /// owes device time.
    ///
    /// # Errors
    ///
    /// Returns the halt cause if the machine halts before the budget runs out,
    /// or [`HaltCause::CycleBudgetExhausted`] otherwise.
    pub fn run(&mut self, budget: u64) -> Result<(), HaltCause> {
        for _ in 0..budget {
            if let Err(cause) = self.step() {
                self.settle_fast();
                return Err(cause);
            }
        }
        self.settle_fast();
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// Advance until `cyccnt` grows by `cycles` (or the machine halts).
    /// Unlike [`Machine::run`], whose budget counts retired instructions,
    /// this counts true core cycles (fetch wait states included) — the
    /// unit SPICE seconds convert into. Overshoot is bounded by one
    /// step's cost and deterministic; device ticks already follow
    /// `cyccnt` (`tick_fast(delta)`), so device time stays exact. The
    /// loop is bounded by the budget (every step retires, so this always
    /// terminates) — a pathological zero-cost step would merely meet the
    /// budget in steps, never hang.
    ///
    /// # Errors
    ///
    /// Returns the halt cause if the machine halts first, or
    /// [`HaltCause::CycleBudgetExhausted`] once the cycle budget is met.
    pub fn run_cycles(&mut self, cycles: u64) -> Result<(), HaltCause> {
        let start = self.core.cyccnt;
        for _ in 0..cycles.saturating_add(1) {
            if self.core.cyccnt.wrapping_sub(start) >= cycles {
                self.settle_fast();
                return Err(HaltCause::CycleBudgetExhausted);
            }
            if let Err(cause) = self.step() {
                self.settle_fast();
                return Err(cause);
            }
        }
        self.settle_fast();
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// Flush outstanding fast-mode device debt without scanning (also what
    /// `set_fast_quantum` and end-of-`run` do). Accurate mode has no debt,
    /// so this is a no-op there.
    pub fn settle_fast(&mut self) {
        let debt = core::mem::take(&mut self.fast_debt);
        if debt != 0 {
            self.core.mem().tick_by(debt);
        }
    }

    /// The number of executed machine cycles so far.
    #[must_use]
    pub const fn cycles(&self) -> u64 {
        self.core.cycles
    }

    /// Arm (`true`) or disarm the semihosting `BKPT 0xAB` trap. Disarmed by
    /// default, preserving QEMU-oracle parity for the conformance suite.
    pub fn semihost_enable(&mut self, on: bool) {
        self.core.semihost.set_enabled(on);
    }

    /// Queue host bytes for semihosting `READC`.
    pub fn semihost_input(&mut self, bytes: &[u8]) {
        self.core.semihost.push_input(bytes);
    }

    /// Take everything firmware emitted through semihosting since the last
    /// drain (`WRITEC`/`WRITE0`/`WRITE` to fd 1/2).
    pub fn semihost_output(&mut self) -> Vec<u8> {
        self.core.semihost.drain_output()
    }

    /// Take a latched semihosting `EXIT` code, if firmware requested one.
    pub fn semihost_exit(&mut self) -> Option<i32> {
        self.core.semihost.take_exit()
    }

    /// Drain new target→host bytes on RTT channel 0, scanning `len` bytes of
    /// guest RAM at `base` for the `SEGGER RTT` control block on first use.
    pub fn rtt_poll(&mut self, ram_base: u32, ram_len: u32) -> Vec<u8> {
        self.core.rtt_poll(ram_base, ram_len)
    }

    /// Move host bytes into RTT down-channel 0. Returns bytes accepted.
    pub fn rtt_inject(&mut self, ram_base: u32, ram_len: u32, bytes: &[u8]) -> usize {
        self.core.rtt_inject(ram_base, ram_len, bytes)
    }

    /// Drain a named device output channel (`itm`, `uart`, ...). Unknown
    /// names drain to empty.
    pub fn drain_channel(&mut self, name: &str) -> Vec<u8> {
        self.core.mem().drain_channel(name)
    }

    /// Drain a named device's host-output channel (USART TX bytes, CAN TX
    /// frames, ...). Unknown names drain to empty.
    pub fn drain_device(&mut self, name: &str) -> Vec<u8> {
        self.core.mem().drain_channel(name)
    }

    /// Inject bytes into a named device's host-input channel. Returns
    /// whether a device by that name exists.
    pub fn drive_device(&mut self, name: &str, bytes: &[u8]) -> bool {
        self.core.mem().drive_device(name, bytes)
    }

    /// A pre- (`before`) or post-instruction hook over `start..end`
    /// (end-exclusive; `0..u32::MAX` covers every PC). Returns whether the
    /// memory installed it (plain test doubles without a registry decline).
    pub fn on_instruction(
        &mut self,
        before: bool,
        start: u32,
        end: u32,
        callback: impl FnMut(&HookEvent) + 'static,
    ) -> bool {
        if let Some(h) = self.core.mem().hooks_mut() {
            h.on_instruction(before, start, end, callback);
            true
        } else {
            false
        }
    }

    /// A read watchpoint over `start..end` (end-exclusive). See
    /// [`Machine::on_instruction`] for the return value.
    pub fn on_read(
        &mut self,
        start: u32,
        end: u32,
        callback: impl FnMut(&HookEvent) + 'static,
    ) -> bool {
        if let Some(h) = self.core.mem().hooks_mut() {
            h.on_read(start, end, callback);
            true
        } else {
            false
        }
    }

    /// A write watchpoint over `start..end` (end-exclusive). See
    /// [`Machine::on_instruction`] for the return value.
    pub fn on_write(
        &mut self,
        start: u32,
        end: u32,
        callback: impl FnMut(&HookEvent) + 'static,
    ) -> bool {
        if let Some(h) = self.core.mem().hooks_mut() {
            h.on_write(start, end, callback);
            true
        } else {
            false
        }
    }

    /// Remove a pre- (`before`) or post-instruction hook over exactly
    /// `start..end` (X2: GDB breakpoint withdrawal un-bypasses the range).
    /// Returns how many entries were removed (0 when the memory has no
    /// registry or nothing matched).
    pub fn remove_instruction(&mut self, before: bool, start: u32, end: u32) -> usize {
        if let Some(h) = self.core.mem().hooks_mut() {
            h.remove_instruction(before, start, end)
        } else {
            0
        }
    }

    /// A peripheral-access hook: `write` selects direction, `name` (if
    /// given) restricts to one device (`Peripheral::name`, e.g. `"usart1"`).
    /// See [`Machine::on_instruction`] for the return value.
    pub fn on_peripheral(
        &mut self,
        write: bool,
        name: Option<&str>,
        callback: impl FnMut(&HookEvent) + 'static,
    ) -> bool {
        if let Some(h) = self.core.mem().hooks_mut() {
            h.on_peripheral(write, name, callback);
            true
        } else {
            false
        }
    }
}

/// Building a [`Machine`] by board name (embedding / co-simulation path).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MachineForError {
    /// No board in [`board::all_boards`] has this name.
    UnknownBoard(String),
    /// The board exists but `reset` failed after loading the image.
    ResetFailed(String),
}

impl std::fmt::Display for MachineForError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownBoard(name) => write!(f, "unknown board '{name}'"),
            Self::ResetFailed(cause) => write!(f, "reset failed: {cause}"),
        }
    }
}

impl Machine<BoardMemory> {
    /// A machine over an already-built board memory, with the core selected
    /// by family from `name` (an [`board::all_boards`] short name).
    /// WP-K1: every board previously ran on the ARMv6-M core, so
    /// Thumb-2-only encodings faulted even on M3/M4 firmware. `virtual`
    /// stays the explicit M0 home; F0/C0/G0/L0 are M0/M0+, F7/H7 are M7,
    /// U5/H5/WBA/L5 are M33, everything else (F1/F2/F3/F4/G4/L4/L1/WB/WL/MP1)
    /// is M3/M4. The CLI's `machine_for` shares this; keep the two in sync.
    #[must_use]
    pub fn for_mem(name: &str, mem: BoardMemory) -> Self {
        if name == "virtual"
            || name.starts_with("stm32f0")
            || name.starts_with("stm32c0")
            || name.starts_with("stm32g0")
            || name.starts_with("stm32l0")
        {
            Self::new(mem)
        } else if name.starts_with("stm32f7") || name.starts_with("stm32h7") {
            Self::armv7em(mem)
        } else if name.starts_with("stm32u5")
            || name.starts_with("stm32h5")
            || name.starts_with("stm32wba")
            || name.starts_with("stm32l5")
        {
            Self::armv8m_mainline(mem)
        } else {
            Self::armv7m(mem)
        }
    }

    /// A machine for the named board with `img` loaded and reset.
    /// Unlike the CLI (which exits the process on failure), failures come
    /// back as [`MachineForError`] so embedding hosts can report them.
    pub fn for_name(name: &str, img: &LoadedImage) -> Result<Self, MachineForError> {
        let mut machine = Self::for_name_bare(name, img)?;
        machine
            .reset()
            .map_err(|cause| MachineForError::ResetFailed(cause.to_string()))?;
        Ok(machine)
    }

    /// A machine for the named board with `img` loaded but NOT reset: the
    /// embedding path for hosts that must configure boot state (e.g. the
    /// vector-table base for raw binaries) before the first reset.
    pub fn for_name_bare(name: &str, img: &LoadedImage) -> Result<Self, MachineForError> {
        // `virtual` is the generic M0 home, not one of the 500 CMSIS chips,
        // so it is not in `all_boards` — accept it explicitly.
        if name == "virtual" {
            let mut machine = Self::for_mem(name, board::virtual_board());
            machine.load_image(img);
            return Ok(machine);
        }
        let (_, build) = board::all_boards()
            .into_iter()
            .find(|(entry, _)| *entry == name)
            .ok_or_else(|| MachineForError::UnknownBoard(name.to_owned()))?;
        let mut machine = Self::for_mem(name, build());
        machine.load_image(img);
        Ok(machine)
    }

    /// Set the reset vector-table base (see `Core::set_vector_base`).
    /// Embedding hosts loading raw binaries outside address 0 need this
    /// before `reset`; ELF/HEX images boot from the table their vectors
    /// live in (address 0 aliases flash on the STM32 boards).
    pub fn set_vector_base(&mut self, base: u32) {
        self.core.set_vector_base(base);
    }

    /// Drive the supply rail to `mv` millivolts (brown-out evaluated
    /// with hysteresis; see `SystemMemory::set_operating_millivolts`).
    /// Takes `&mut` only because the bus accessor does; the rail write
    /// itself is the mutation.
    pub fn set_operating_millivolts(&mut self, mv: u32) {
        self.core.mem().set_operating_millivolts(mv);
    }

    /// The current CPU clock in Hz (see `SystemMemory::cpu_hz`): the rate
    /// the co-simulation shim converts SPICE seconds into core cycles with.
    /// Takes `&mut` only because the bus accessor does; nothing is mutated.
    pub fn cpu_hz(&mut self) -> u32 {
        self.core.mem().cpu_hz()
    }

    /// Co-simulation: find GPIO port `port` (0 = A) among the attached
    /// devices: `gpioa`..`gpiok`, falling back to the virtual board's
    /// single `gpio` for port 0. Returns the device index, if present.
    fn gpio_index(devices: &[Box<dyn crate::device::InterruptSource>], port: u8) -> Option<usize> {
        const LETTERS: [&str; 11] = [
            "gpioa", "gpiob", "gpioc", "gpiod", "gpioe", "gpiof", "gpiog", "gpioh", "gpioi",
            "gpioj", "gpiok",
        ];
        devices.iter().position(|device| {
            let name = device.name();
            (usize::from(port) < LETTERS.len() && name == LETTERS[usize::from(port)])
                || (port == 0 && name == "gpio")
        })
    }

    /// Co-simulation: the `(mode, level)` of `pin` on GPIO port `port`
    /// (0 = A). `None` when the port is absent or the pin out of range.
    pub fn gpio_pin(&mut self, port: u8, pin: u8) -> Option<(GpioPinMode, bool)> {
        let devices = self.core.mem().devices_mut();
        let index = Self::gpio_index(devices, port)?;
        let mode = devices[index].gpio_mode(pin)?;
        let level = devices[index].gpio_level(pin)?;
        Some((mode, level))
    }

    /// Co-simulation: drive the whole host-input word of GPIO port `port`
    /// (0 = A) — input pins follow it, driven pins ignore it. Returns
    /// whether the port exists. Word (not byte) granularity is why this
    /// exists alongside `drive_device`: the byte-stream channel would
    /// deliver one `drive_inputs` call per byte.
    pub fn gpio_drive(&mut self, port: u8, mask: u32) -> bool {
        let devices = self.core.mem().devices_mut();
        let Some(index) = Self::gpio_index(devices, port) else {
            return false;
        };
        devices[index].drive_inputs(mask);
        true
    }

    /// Co-simulation: inject `mv` millivolts into the `apin` window's
    /// receiver (one little-endian word through the byte channel, exactly
    /// as the fabric delivers it). Honors the window's `EN` bit — firmware
    /// must enable the endpoint first, like enabling an ADC. Returns
    /// whether an `apin` device is attached. Channel numbering is future
    /// work (P2): the N3 seam is a single window.
    pub fn apin_inject_mv(&mut self, mv: u32) -> bool {
        let devices = self.core.mem().devices_mut();
        let Some(device) = devices.iter_mut().find(|device| device.name() == "apin") else {
            return false;
        };
        for byte in mv.to_le_bytes() {
            device.drive_inputs(u32::from(byte));
        }
        true
    }

    /// Co-simulation: the most recently transmitted whole millivolt word
    /// from the `apin` window, if firmware staged one (`TXMV` + `TXGO`).
    /// Most-recent-word semantics: the shim polls once per quantum, so at
    /// most one word is ever pending; older words are superseded, like a
    /// DAC output register.
    pub fn apin_drain_mv(&mut self) -> Option<u32> {
        let devices = self.core.mem().devices_mut();
        let device = devices.iter_mut().find(|device| device.name() == "apin")?;
        let bytes = device.drain_output();
        let words = bytes.len() / 4;
        if words == 0 {
            return None;
        }
        let start = (words - 1) * 4;
        Some(u32::from_le_bytes([
            bytes[start],
            bytes[start + 1],
            bytes[start + 2],
            bytes[start + 3],
        ]))
    }
}

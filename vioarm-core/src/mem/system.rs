//! A `Bus` that hangs `Peripheral` devices over a backing memory.
//!
//! The dispatch rule is simple and deterministic: an address hits the first
//! device whose `ranges()` contain it; only if no device claims it does the
//! request fall through to the backing memory, and if that has no bank either
//! the access is `Reserved`. Device windows therefore always win over backing
//! memory they overlap, which matches how an `SoC` overlays peripherals on its
//! bus.

use crate::device::nvic::Nvic;
use crate::device::scb::Scb;
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::{Bus, PeripheralBus};
use crate::cpu::FpuCtx;
use std::sync::{Arc, Mutex};

/// A `Bus` over flat backing memory plus a set of MMIO devices and the NVIC.
///
/// Devices are stored as `dyn InterruptSource` — most peripherals can raise
/// at least one line, and the default no-op methods keep the purely
/// memory-mapped ones cheap — so the interrupt controller can peek and
/// acknowledge their state without a second registry. Lookup is a linear scan
/// over a small vector of windows; the M1 roadmap allows a two-level lookup
/// later if real runs ever make it hot.
pub struct SystemMemory<M: Bus> {
    backing: M,
    devices: Vec<Box<dyn InterruptSource>>,
    nvic: Nvic,
    /// The ARMv7-M system control block: interrupt control/state, priority
    /// registers for the fixed exceptions, and the AIRCR reset bits.
    scb: Scb,
}

impl<M: Bus + core::fmt::Debug> core::fmt::Debug for SystemMemory<M> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SystemMemory")
            .field("backing", &self.backing)
            .field("nvic", &self.nvic)
            .field("scb", &self.scb)
            .field(
                "devices",
                &self.devices.iter().map(|d| d.name()).collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl<M: Bus> SystemMemory<M> {
    /// Wrap `backing` with no peripherals attached.
    #[must_use]
    pub fn new(backing: M) -> Self {
        Self {
            backing,
            devices: Vec::new(),
            nvic: Nvic::new(),
            scb: Scb::new(),
        }
    }

    /// Wrap `backing`, sharing the FP context-control state with the core.
    /// Both this bus's `SCB` and the core must see the same `FPCCR`/`FPCAR`;
    /// the caller creates the `Rc` once and passes clones to both.
    #[must_use]
    pub fn new_with_fpu(backing: M, fpu_ctx: Arc<Mutex<FpuCtx>>) -> Self {
        let mut mem = Self::new(backing);
        mem.scb.set_fpu_ctx(fpu_ctx);
        mem
    }

    /// Attach a device. Its windows immediately take priority over the
    /// backing memory and over devices that were attached earlier.
    pub fn attach(&mut self, device: Box<dyn InterruptSource>) {
        self.devices.push(device);
    }

    /// The attached devices, in attach order.
    pub fn devices(&self) -> &[Box<dyn InterruptSource>] {
        &self.devices
    }

    /// Mutable view of the attached devices, for the NVIC's peek/acknowledge
    /// pass and for host-side inspection.
    pub fn devices_mut(&mut self) -> &mut [Box<dyn InterruptSource>] {
        &mut self.devices
    }

    /// Restore every attached device, the NVIC, and the SCB to its post-reset
    /// state. The processor's reset (AIRCR.SYSRESETREQ) folds into this too —
    /// the SCB clears its own latches, which is the architectural reset
    /// behaviour for pending SYSTICK and PENDSV.
    pub fn reset_devices(&mut self) {
        self.nvic.reset();
        self.scb.reset();
        for device in &mut self.devices {
            device.reset();
        }
    }

    /// Whether `AIRCR.SYSRESETREQ` has been latched since the last reset; the
    /// `Machine` performs a full reset when set.
    #[must_use]
    pub const fn reset_requested(&self) -> bool {
        self.scb.reset_requested()
    }

    /// Advance every un-gated device by exactly `delta` cycles, then latch any
    /// freshly-asserted interrupt lines into the NVIC's pending register.
    /// Called at a synchronization point with the true elapsed delta; gated
    /// devices skip time entirely, mirroring a powered-down clock domain whose
    /// counters froze.
    pub fn tick_devices(&mut self, delta: u64) {
        for device in &mut self.devices {
            if !device.gated() {
                device.tick(delta);
            }
        }
        self.latch_device_asserts();
    }

    /// Split a priority byte into its group and subpriority parts under the
    /// current `AIRCR.PRIGROUP` (the number of subpriority bits). A zero
    /// `PRIGROUP` gives the whole byte to the group and leaves no subpriority
    /// bits. Preemption is decided on the group part alone; subpriority only
    /// orders which pending exception is activated within the same group.
    fn split_priority(&self, p: u8) -> (u8, u8) {
        let sub = self.scb.prig() & 0x7;
        (
            p >> sub,
            if sub == 0 {
                0
            } else {
                p & ((1u8 << sub) - 1)
            },
        )
    }

    /// Fold candidate exception `n` with priority parts `group`/`sub` and raw
    /// priority byte `raw` into `best`. Higher priority wins; among equal
    /// groups the higher subpriority wins; ties fall to the lower exception
    /// number. Lower numeric value means higher priority everywhere.
    fn consider(best: &mut Option<(u8, u8, u8, u8)>, n: u8, group: u8, sub: u8, raw: u8) {
        let replace = match *best {
            None => true,
            Some((bn, bg, bs, _)) => {
                group < bg || (group == bg && (sub < bs || (sub == bs && n < bn)))
            }
        };
        if replace {
            *best = Some((n, group, sub, raw));
        }
    }

    /// The current execution priority: the priority of the running exception,
    /// or `u8::MAX` in Thread mode.
    #[allow(clippy::cast_possible_truncation)]
    fn current_priority(&self, ipsr: u8) -> u8 {
        match ipsr {
            0 => u8::MAX,
            // SVCall (11), PendSV (14), and SysTick (15) are the
            // configurable-priority system exceptions with SHPR bytes.
            11 | 14 | 15 => self.scb.priority(ipsr),
            n => self.nvic.priority(u32::from(n - 16)),
        }
    }

    /// The two-phase "peek": the highest-priority exception that should be
    /// taken right now, or `None`. Side-effect free; the NVIC pending register
    /// was already freshened by `tick_devices`, and its priority bytes plus the
    /// SCB's fixed-exception priorities are the arbiter.
    #[allow(clippy::cast_possible_truncation)]
    fn peek_exception(
        &self,
        primask: bool,
        faultmask: bool,
        basepri: u8,
        current_ipsr: u8,
    ) -> Option<u8> {
        // PRIMASK and FAULTMASK block every configurable-priority exception we
        // model (only faults escape them).
        if primask || faultmask {
            return None;
        }
        // Best pending candidate by group/subpriority; ties go to the lowest
        // exception number, which is exactly the M-profile preemption rule.
        let mut best: Option<(u8, u8, u8, u8)> = None;
        for device in &self.devices {
            if device.exception_number() == Some(15) && device.pending() != 0 {
                let p = self.scb.priority(15);
                let (g, s) = self.split_priority(p);
                Self::consider(&mut best, 15, g, s, p);
            }
        }
        if self.scb.pendsv_pending() {
            let p = self.scb.priority(14);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, 14, g, s, p);
        }
        if self.scb.svcall_pending() {
            let p = self.scb.priority(11);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, 11, g, s, p);
        }
        let mut ready = self.nvic.ready();
        while ready != 0 {
            let line = ready.trailing_zeros();
            let n = 16 + line as u8;
            let p = self.nvic.priority(line);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, n, g, s, p);
            ready &= ready - 1;
        }
        let (n, group, _sub, priority) = best?;
        // BASEPRI masks the configurable-priority exceptions (SVCall, PendSV,
        // SysTick, and external IRQs) whose priority number is `>=` its value.
        // A zero BASEPRI never masks. Faults (3) are not configurable.
        if basepri != 0 && (n == 11 || n >= 14) && priority >= basepri {
            return None;
        }
        // Only a strictly higher-priority (numerically lower) group preempts
        // the one running; an equal group never preempts, even with a higher
        // subpriority. HardFault (3) blocks everything we model.
        match self.current_priority(current_ipsr) {
            3 => None,
            current => {
                let (cur_group, _) = self.split_priority(current);
                if group < cur_group {
                    Some(n)
                } else {
                    None
                }
            }
        }
    }

    /// The two-phase "consume": the core is taking exception `n`; clear the
    /// NVIC pending latch and tell the source device to drop its line.
    fn consume_exception(&mut self, n: u8) {
        match n {
            11 => self.scb.acknowledge_svcall(),
            14 => self.scb.acknowledge(1),
            15 => {
                for device in &mut self.devices {
                    if device.exception_number() == Some(15) {
                        device.acknowledge(1);
                    }
                }
            }
            n if n >= 16 => {
                let line = u32::from(n - 16);
                self.nvic.take(line);
                for device in &mut self.devices {
                    if device.exception_number() == Some(n) {
                        device.acknowledge(1);
                    }
                }
            }
            _ => {}
        }
    }

    /// Copy each asserting device line into the NVIC's pending register.
    fn latch_device_asserts(&mut self) {
        for device in &self.devices {
            let Some(n) = device.exception_number() else {
                continue;
            };
            if n >= 16 && device.pending() != 0 {
                self.nvic.latch(1u64 << u32::from(n - 16));
            }
        }
    }
}

impl<M: Bus> Bus for SystemMemory<M> {
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if let Some(dev) = self
            .devices
            .iter_mut()
            .find(|d| d.ranges().iter().any(|r| r.contains(addr)))
        {
            dev.read(addr)
        } else if self.scb.ranges().iter().any(|r| r.contains(addr)) {
            self.scb.read(addr)
        } else if self.nvic.ranges().iter().any(|r| r.contains(addr)) {
            self.nvic.read(addr)
        } else {
            self.backing.read(addr)
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if let Some(dev) = self
            .devices
            .iter_mut()
            .find(|d| d.ranges().iter().any(|r| r.contains(addr)))
        {
            dev.write(addr, val)
        } else if self.scb.ranges().iter().any(|r| r.contains(addr)) {
            self.scb.write(addr, val)
        } else if self.nvic.ranges().iter().any(|r| r.contains(addr)) {
            self.nvic.write(addr, val)
        } else {
            self.backing.write(addr, val)
        }
    }
}

impl<M: Bus> PeripheralBus for SystemMemory<M> {
    fn next_exception(
        &mut self,
        primask: bool,
        faultmask: bool,
        basepri: u8,
        current_ipsr: u8,
    ) -> Option<u8> {
        self.peek_exception(primask, faultmask, basepri, current_ipsr)
    }

    fn acknowledge_exception(&mut self, n: u8) {
        self.consume_exception(n);
    }

    fn tick_cycle(&mut self) {
        self.tick_devices(1);
    }

    fn reset_peripherals(&mut self) {
        self.reset_devices();
    }

    fn record_fault(&mut self, cfsr_bits: u32, hfsr_bits: u32) {
        self.scb.record_fault(cfsr_bits, hfsr_bits);
    }
}

#[cfg(test)]
mod tests {
    use super::SystemMemory;
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use crate::mem::bus::{Bus, PeripheralBus, Region};
    use crate::mem::flat::FlatMemory;

    const RAM: u32 = 0x2000_0000;
    const RAM_LEN: u32 = 0x1000;

    fn ram() -> FlatMemory {
        FlatMemory::with_layout(&[RAM], &[RAM_LEN])
    }

    /// A 4-byte register at `base`, a 4-byte tick counter at `base + 4`, and
    /// a latched interrupt line at `base + 8` (written bits 0..1). Exposing
    /// its state as registers keeps the tests honest through the bus.
    struct Beacon {
        ranges: [Region; 1],
        reg: u32,
        ticks: u32,
        pending: u32,
        gated: bool,
    }

    impl Beacon {
        fn at(base: u32) -> Self {
            Self {
                ranges: [Region::new(base, 16)],
                reg: 0,
                ticks: 0,
                pending: 0,
                gated: false,
            }
        }
    }

    impl Peripheral for Beacon {
        fn ranges(&self) -> &[Region] {
            &self.ranges
        }

        fn gated(&self) -> bool {
            self.gated
        }

        #[allow(clippy::cast_possible_truncation)]
        fn read(&mut self, addr: u32) -> Result<u8, BusError> {
            let off = addr - self.ranges[0].base;
            match off {
                0..=3 => Ok((self.reg >> (8 * off)) as u8),
                4..=7 => Ok((self.ticks >> (8 * (off - 4))) as u8),
                8 => Ok(self.pending as u8),
                _ => Ok(0),
            }
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            let off = addr - self.ranges[0].base;
            match off {
                0..=3 => {
                    let shift = 8 * off;
                    self.reg = (self.reg & !(0xFF << shift)) | (u32::from(val) << shift);
                }
                8 => self.pending = u32::from(val & 1),
                _ => {}
            }
            Ok(())
        }

        fn reset(&mut self) {
            self.reg = 0;
            self.ticks = 0;
            self.pending = 0;
            self.gated = false;
        }

        #[allow(clippy::cast_possible_truncation)]
        fn tick(&mut self, delta: u64) {
            self.ticks = self.ticks.wrapping_add(delta as u32);
        }

        fn name(&self) -> &'static str {
            "beacon"
        }
    }

    impl InterruptSource for Beacon {
        fn pending(&self) -> u32 {
            self.pending
        }

        fn acknowledge(&mut self, lines: u32) {
            self.pending &= !lines;
        }
    }

    #[test]
    fn word_routes_through_device_window_as_bytes() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000)));
        sys.write_word(0x4200_0000, 0xDEAD_BEEF).unwrap();
        assert_eq!(sys.read_word(0x4200_0000).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn unclaimed_addresses_fall_through_to_ram() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000)));
        sys.write(RAM, 0x77).unwrap();
        sys.write_word(RAM + 4, 0x1234_5678).unwrap();
        assert_eq!(sys.read(RAM).unwrap(), 0x77);
        assert_eq!(sys.read_word(RAM + 4).unwrap(), 0x1234_5678);
    }

    #[test]
    fn address_with_no_bank_or_device_is_reserved() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000)));
        assert!(matches!(sys.read(0x9000_0000), Err(BusError::Reserved)));
    }

    #[test]
    fn device_window_wins_over_backing_memory() {
        // Overlay a device right on top of the RAM bank.
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(RAM)));
        // A RAM byte just past the device window is plain backing memory.
        sys.write(RAM + 0x20, 0xAA).unwrap();
        // A device-register write shares a street address with the RAM.
        sys.write_word(RAM, 0x00C0_FFEE).unwrap();
        assert_eq!(sys.read_word(RAM).unwrap(), 0x00C0_FFEE);
        // The backing RAM is still visible outside the window.
        assert_eq!(sys.read(RAM + 0x20).unwrap(), 0xAA);
    }

    #[test]
    fn reset_restores_device_state() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000)));
        sys.write_word(0x4200_0000, 0xFFFF_FFFF).unwrap();
        sys.write_word(0x4200_0000 + 8, 1).unwrap();
        sys.reset_devices();
        assert_eq!(sys.read_word(0x4200_0000).unwrap(), 0);
        assert_eq!(sys.read_word(0x4200_0000 + 8).unwrap(), 0);
    }

    #[test]
    fn tick_advances_live_and_skips_gated() {
        let live = Beacon::at(0x4200_0000);
        let mut frozen = Beacon::at(0x4300_0000);
        frozen.gated = true;
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(live));
        sys.attach(Box::new(frozen));
        sys.tick_devices(30);
        sys.tick_devices(70);
        // Live device advanced by both deltas; the gated one advanced by
        // nothing even though the same exact delta was presented.
        assert_eq!(sys.read_word(0x4200_0000 + 4).unwrap(), 100);
        assert_eq!(sys.read_word(0x4300_0000 + 4).unwrap(), 0);
    }

    #[test]
    fn interrupt_handshake_is_peek_then_acknowledge() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000)));
        sys.write(0x4200_0000 + 8, 1).unwrap();

        // Phase one: peek. Calling twice must observe the same picture.
        let first = sys.devices()[0].pending();
        let again = sys.devices()[0].pending();
        assert_eq!(first, again);
        assert_ne!(first | again, 0);

        // Phase two: the NVIC takes line 0; the source clears its latch.
        sys.devices_mut()[0].acknowledge(1);
        assert_eq!(sys.devices()[0].pending(), 0);
    }

    /// A level-raised IRQ on line 0 (exception 16): a write to `base + 1`
    /// raises it, and consuming the interrupt clears it.
    struct Raiser {
        ranges: [Region; 1],
        asserted: bool,
    }

    impl Raiser {
        const BASE: u32 = 0x4400_0000;

        fn new() -> Self {
            Self {
                ranges: [Region::new(Self::BASE, 4)],
                asserted: false,
            }
        }
    }

    impl Peripheral for Raiser {
        fn ranges(&self) -> &[Region] {
            &self.ranges
        }

        fn read(&mut self, _addr: u32) -> Result<u8, BusError> {
            Ok(u8::from(self.asserted))
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            if addr - Self::BASE == 1 {
                self.asserted = val & 1 != 0;
            }
            Ok(())
        }

        fn name(&self) -> &'static str {
            "raiser"
        }
    }

    impl InterruptSource for Raiser {
        fn exception_number(&self) -> Option<u8> {
            Some(16)
        }

        fn pending(&self) -> u32 {
            u32::from(self.asserted)
        }

        fn acknowledge(&mut self, _lines: u32) {
            self.asserted = false;
        }
    }

    #[test]
    fn device_assert_latches_into_nvic_and_consumes() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Raiser::new()));
        sys.write_word(0xE000_E100, 1).unwrap(); // ISER: enable line 0
        sys.write(Raiser::BASE + 1, 1).unwrap(); // device asserts
        sys.tick_devices(1); // a sync point latches it into the NVIC
        let n = sys
            .next_exception(false, false, 0, 0)
            .expect("IRQ0 pending");
        assert_eq!(n, 16);
        sys.acknowledge_exception(n);
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
        assert_eq!(sys.devices()[0].pending(), 0);
    }

    #[test]
    fn software_pending_needs_enable() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E200, 1).unwrap(); // ISPR: pend line 0
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
        sys.write_word(0xE000_E100, 1).unwrap(); // ISER: enable line 0
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
        sys.acknowledge_exception(16);
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
    }

    #[test]
    fn lowest_irq_line_wins() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E100, 0b11).unwrap();
        sys.write_word(0xE000_E200, 0b11).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
    }

    #[test]
    fn primask_blocks_all() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E100, 1).unwrap();
        sys.write_word(0xE000_E200, 1).unwrap();
        assert_eq!(sys.next_exception(true, false, 0, 0), None);
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
    }

    #[test]
    fn faultmask_blocks_all() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E100, 1).unwrap();
        sys.write_word(0xE000_E200, 1).unwrap();
        assert_eq!(sys.next_exception(false, true, 0, 0), None);
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
    }

    #[test]
    fn basepri_masks_by_priority_number() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E100, 1).unwrap();
        sys.write_word(0xE000_E200, 1).unwrap();
        sys.write(0xE000_E400, 0x80).unwrap(); // IRQ0 priority 0x80
                                               // BASEPRI at or below the priority number masks it.
        assert_eq!(sys.next_exception(false, false, 0x40, 0), None);
        assert_eq!(sys.next_exception(false, false, 0x80, 0), None);
        // A larger BASEPRI leaves it unmasked; zero disables the mask.
        assert_eq!(sys.next_exception(false, false, 0x90, 0), Some(16));
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
    }

    #[test]
    fn basepri_masks_pendsv() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_ED04, 0x1000_0000).unwrap(); // pend PendSV
        sys.write(0xE000_ED22, 0x40).unwrap(); // PendSV priority 0x40
        assert_eq!(sys.next_exception(false, false, 0x20, 0), None);
        assert_eq!(sys.next_exception(false, false, 0x40, 0), None);
        assert_eq!(sys.next_exception(false, false, 0x50, 0), Some(14));
    }

    #[test]
    fn handler_only_preempted_by_higher_priority() {
        let mut sys = SystemMemory::new(ram());
        // IRQ 0 (exc 16) and IRQ 7 (exc 23) both pending, enabled.
        sys.write_word(0xE000_E100, (1 << 0) | (1 << 7)).unwrap();
        sys.write_word(0xE000_E200, (1 << 0) | (1 << 7)).unwrap();
        // IRQ7 keeps a low priority (0xC0); IRQ0 stays default 0.
        sys.write(0xE000_E407, 0xC0).unwrap();
        // Running in the IRQ7 handler (23, prio 0xC0): IRQ0 (prio 0) preempts.
        assert_eq!(sys.next_exception(false, false, 0, 23), Some(16));
        // Running in the IRQ0 handler (16, prio 0): IRQ7 (prio 0xC0) cannot.
        assert_eq!(sys.next_exception(false, false, 0, 16), None);
    }

    #[test]
    fn equal_priority_never_preempts() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_E100, (1 << 0) | (1 << 7)).unwrap();
        sys.write_word(0xE000_E200, (1 << 0) | (1 << 7)).unwrap();
        // Same priority in thread mode: the lower line wins the take...
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
        // ...but a same-priority handler blocks the other.
        assert_eq!(sys.next_exception(false, false, 0, 16), None);
    }

    #[test]
    fn systick_beats_external_irq() {
        use crate::device::systick::SysTick;
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(SysTick::new()));
        sys.write_word(0xE000_E100, 1).unwrap();
        sys.write_word(0xE000_E200, 1).unwrap();
        // IRQ0 tries to lower itself below SysTick's default priority 0.
        sys.write(0xE000_E400, 0x40).unwrap();
        // Arm SysTick to wrap after two cycles with TICKINT set.
        sys.write_word(0xE000_E014, 1).unwrap();
        sys.write_word(0xE000_E010, 0b111).unwrap();
        sys.tick_devices(2);
        // Exception 15 (prio 0) still outranks IRQ0 (now prio 0x40).
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(15));
    }

    #[test]
    fn pendsv_pends_via_icsr_and_is_taken_in_thread_mode() {
        let mut sys = SystemMemory::new(ram());
        // Pend PendSV through ICSR.PENDSVSET.
        sys.write_word(0xE000_ED04, 0x1000_0000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(14));
        sys.acknowledge_exception(14);
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
        // PRIMASK still masks it.
        sys.write_word(0xE000_ED04, 0x1000_0000).unwrap();
        assert_eq!(sys.next_exception(true, false, 0, 0), None);
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(14));
    }

    #[test]
    fn pendsv_preempts_lower_priority_handler() {
        let mut sys = SystemMemory::new(ram());
        // Pended PendSV at SHPR3 pris; IRQ7 (exc 23) running at prio 0xC0.
        sys.write_word(0xE000_E200, 1 << 7).unwrap();
        sys.write_word(0xE000_E100, 1 << 7).unwrap();
        sys.write(0xE000_E407, 0xC0).unwrap();
        sys.write(0xE000_ED22, 0x40).unwrap(); // PendSV prio 0x40
        sys.write_word(0xE000_ED04, 0x1000_0000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), Some(14));
    }

    #[test]
    fn pendsv_waits_at_same_priority() {
        let mut sys = SystemMemory::new(ram());
        // PendSV pended at priority equal to the running IRQ7 handler.
        sys.write_word(0xE000_E200, 1 << 7).unwrap();
        sys.write_word(0xE000_E100, 1 << 7).unwrap();
        sys.write(0xE000_E407, 0xC0).unwrap();
        sys.write(0xE000_ED22, 0xC0).unwrap();
        sys.write_word(0xE000_ED04, 0x1000_0000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), None);
    }

    #[test]
    fn shcsr_svcallpend_takes_svc_in_thread_mode() {
        let mut sys = SystemMemory::new(ram());
        // Pend an SVC through SHCSR.SVCALLPENDED.
        sys.write_word(0xE000_ED24, 0x0000_8000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(11));
        // Acknowledge clears the pend; the second peek sees nothing.
        sys.acknowledge_exception(11);
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
        // The pend is restored and re-taken; PRIMASK still masks it.
        sys.write_word(0xE000_ED24, 0x0000_8000).unwrap();
        assert_eq!(sys.next_exception(true, false, 0, 0), None);
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(11));
    }

    #[test]
    fn svcall_pend_preempts_lower_priority_handler() {
        let mut sys = SystemMemory::new(ram());
        // IRQ7 (exc 23) running at priority 0xC0; SVC pended at 0x40.
        sys.write_word(0xE000_E200, 1 << 7).unwrap();
        sys.write_word(0xE000_E100, 1 << 7).unwrap();
        sys.write(0xE000_E407, 0xC0).unwrap();
        sys.write(0xE000_ED1F, 0x40).unwrap(); // SHPR2 SVCall prio 0x40
        sys.write_word(0xE000_ED24, 0x0000_8000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), Some(11));
        // At the same priority the running handler blocks the pend.
        sys.write(0xE000_ED1F, 0xC0).unwrap();
        sys.write_word(0xE000_ED24, 0x0000_8000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), None);
    }

    #[test]
    fn svcall_pend_masked_by_basepri() {
        let mut sys = SystemMemory::new(ram());
        sys.write(0xE000_ED1F, 0x40).unwrap(); // SVC prio 0x40
        sys.write_word(0xE000_ED24, 0x0000_8000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0x20, 0), None);
        assert_eq!(sys.next_exception(false, false, 0x40, 0), None);
        assert_eq!(sys.next_exception(false, false, 0x50, 0), Some(11));
    }

    #[test]
    fn prigroup_blocks_same_group_preemption() {
        let mut sys = SystemMemory::new(ram());
        // PRIGROUP = 2: two subpriority bits.
        sys.write_word(0xE000_ED0C, 0x05FA_0200).unwrap();
        // IRQ0 (exc 16) priority 0x3C, IRQ7 (exc 23) priority 0x3F. Under
        // grouping both are group 0x0F, so IRQ0 may not preempt the running
        // IRQ7 even though its raw priority byte is numerically higher.
        sys.write(0xE000_E400, 0x3C).unwrap();
        sys.write(0xE000_E407, 0x3F).unwrap();
        sys.write_word(0xE000_E100, (1 << 0) | (1 << 7)).unwrap();
        sys.write_word(0xE000_E200, (1 << 0) | (1 << 7)).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), None);
        // In thread mode the higher subpriority in the group is served first.
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(16));
    }

    #[test]
    fn prigroup_subpriority_orders_same_group() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_ED0C, 0x05FA_0200).unwrap();
        // IRQ0 has subpriority 3, IRQ7 subpriority 0 within group 0x0F.
        sys.write(0xE000_E400, 0x3F).unwrap();
        sys.write(0xE000_E407, 0x3C).unwrap();
        sys.write_word(0xE000_E100, (1 << 0) | (1 << 7)).unwrap();
        sys.write_word(0xE000_E200, (1 << 0) | (1 << 7)).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(23));
    }

    #[test]
    fn without_prigroup_full_byte_preempts() {
        let mut sys = SystemMemory::new(ram());
        // Default PRIGROUP 0: the raw byte arbitrates, so IRQ0 (0x3C)
        // preempts the running IRQ7 (0x3F).
        sys.write(0xE000_E400, 0x3C).unwrap();
        sys.write(0xE000_E407, 0x3F).unwrap();
        sys.write_word(0xE000_E100, (1 << 0) | (1 << 7)).unwrap();
        sys.write_word(0xE000_E200, (1 << 0) | (1 << 7)).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), Some(16));
    }

    #[test]
    fn aircr_sysresetreq_latches_until_reset() {
        let mut sys = SystemMemory::new(ram());
        assert!(!sys.reset_requested());
        sys.write_word(0xE000_ED0C, 0x05FA_0004).unwrap();
        assert!(sys.reset_requested());
        // The machine's reset path clears the latch.
        sys.reset_devices();
        assert!(!sys.reset_requested());
        // Without the VECTKEY the request never latches.
        sys.write_word(0xE000_ED0C, 0x0000_0004).unwrap();
        assert!(!sys.reset_requested());
    }
}

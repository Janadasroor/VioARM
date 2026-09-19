//! A `Bus` that hangs `Peripheral` devices over a backing memory.
//!
//! The dispatch rule is simple and deterministic: an address hits the first
//! device whose `ranges()` contain it; only if no device claims it does the
//! request fall through to the backing memory, and if that has no bank either
//! the access is `Reserved`. Device windows therefore always win over backing
//! memory they overlap, which matches how an `SoC` overlays peripherals on its
//! bus.

use crate::cpu::FpuCtx;
use crate::device::analog::AnalogBusState;
use crate::device::itm::Itm;
use crate::device::nvic::Nvic;
use crate::device::scb::Scb;
use crate::device::stm32::dma::DmaUnit;
use crate::device::stm32::eth::{EthMailbox, EthMemOp};
use crate::device::stm32::fdcan::{FdcanMailbox, FdcanResult};
use crate::device::stm32::flash::{FlashErase, FlashGate};
use crate::device::stm32::fmc::Dma2dMailbox;
use crate::device::stm32::fmc::QspiGate;
use crate::device::stm32::rcc::ClockState;
use crate::device::vdd::VddState;
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::hooks::Hooks;
use crate::mem::bus::{
    Bus, DataTranslate, IdauAttribution, MpuAccess, MpuLookup, PeripheralBus, Region, SauHit,
};
use std::sync::{Arc, Mutex};

/// One attached-device window in the route index below.
#[derive(Clone, Copy, Debug)]
struct DevWindow {
    base: u32,
    end: u32,
    dev: usize,
}

/// A `Bus` over flat backing memory plus a set of MMIO devices and the NVIC.
///
/// Devices are stored as `dyn InterruptSource` — most peripherals can raise
/// at least one line, and the default no-op methods keep the purely
/// memory-mapped ones cheap — so the interrupt controller can peek and
/// acknowledge their state without a second registry. Lookup is a sorted
/// window index over the attach-order windows (see `route_device`); the M7
/// profile made the old linear scan untenable (~5 ns per device per byte,
/// ~150 windows on big boards).
pub struct SystemMemory<M: Bus> {
    backing: M,
    devices: Vec<Box<dyn InterruptSource>>,
    /// Device windows sorted by `(base, dev)`; rebuilt lazily whenever
    /// `devices` grows. `dev` is the attach index, so precedence never
    /// depends on sort stability. Overlaps are real (controllers sharing
    /// an SRAM window), so lookup checks every window that can contain the
    /// address and keeps attach-order-first.
    dev_index: Vec<DevWindow>,
    /// `devices.len()` the last time `dev_index` was rebuilt.
    dev_index_len: usize,
    /// See `set_route_force_linear`.
    route_force_linear: bool,
    /// The longest indexed window. Lookup only inspects windows with
    /// `base` in `(addr - max, addr]`: anything earlier ends at or before
    /// `addr` and cannot contain it.
    dev_index_max_len: u32,
    /// Per-device APB timer domain (`tim_apb` of the device name), built
    /// with the window index. Saves a string match per device per tick.
    dev_apb: Vec<Option<u8>>,
    /// NVIC lines currently inside their handler (consumed, not yet
    /// returned). While a line is active its source flag may legitimately
    /// stay set — firmware clears it inside the handler, which has not run
    /// yet — so `latch_device_asserts` must not re-pend it: re-pending on
    /// entry/prologue ticks is what used to fire every IRQ twice. On return
    /// the core clears the line via `note_exception_return`; a line that is
    /// still asserted then re-latches on the next pass, exactly like silicon
    /// refiring on a held line. Transient across a handler flight, so
    /// snapshots (which already skip NVIC pending) need not persist it;
    /// resets clear it.
    active: u128,
    nvic: Nvic,
    /// The ARMv7-M system control block: interrupt control/state, priority
    /// registers for the fixed exceptions, and the AIRCR reset bits.
    scb: Scb,
    /// The STM32 flash controller's write gate, shared with the `FLASH`
    /// device. When present, writes to `gate.region` are only honored while
    /// the controller is unlocked and programming (`CR.PG`); otherwise they
    /// are consumed and `WRPRTERR` latches.
    flash: Option<Arc<Mutex<FlashGate>>>,
    /// The QSPI memory-mapped gate `0x9000_0000`, shared with the `QUADSPI`
    /// device. While `QUADSPI->CR.EN == 0` accesses to the gate's region are
    /// consumed as `Ok(())` without touching backing, mirroring silicon where
    /// the window is not `BUSY` until `EN`.
    qspi: Option<Arc<Mutex<QspiGate>>>,
    /// Second QSPI/OCTOSPI2 gate at `0x70000000` (H723 OCTOSPI2).
    qspi2: Option<Arc<Mutex<QspiGate>>>,
    /// The shared DMA transfer queue, drained after every device tick. DMA
    /// controllers push `(src, dst, len)` units here because they have no view
    /// of the bus; this bus performs the actual copy.
    dma_mailbox: Option<Arc<Mutex<Vec<DmaUnit>>>>,
    /// The shared Ethernet mailbox, drained after every device tick. The
    /// controller pushes read/write requests here because it has no view of the
    /// bus; this bus performs them, appending read results to the mailbox.
    eth_mailbox: Option<Arc<Mutex<EthMailbox>>>,
    /// The shared FDCAN mailbox, drained after every device tick. The
    /// controllers push message-RAM reads and received-frame writes here;
    /// this bus performs them, appending correlated read results.
    fdcan_mailbox: Option<Arc<Mutex<FdcanMailbox>>>,
    /// The shared DMA2D mailbox, drained after every device tick. The
    /// controller pushes source-row reads and destination writes here;
    /// this bus performs them, appending correlated read results.
    dma2d_mailbox: Option<Arc<Mutex<Dma2dMailbox>>>,
    /// The shared watchdog reset latch. The IWDG/WWDG devices set it when their
    /// counter underflows; `reset_requested()` folds it into the SCB's latch so
    /// the `Machine` performs a full reset, and `reset_devices()` clears it.
    watchdog: Option<Arc<Mutex<bool>>>,
    /// The supply rail shared by power-aware models (`Pwr` PVD/BOR; ADC
    /// reference in phase 2). Present on every bus, defaulting to nominal
    /// voltage, so hosts can drive it on any board; see [`VddState`].
    /// Brown-out needs no latch: `reset_requested()` reads the rail's
    /// asserted level directly, so a held-low rail holds the machine in
    /// reset like silicon (and recovery resumes immediately).
    vdd: Arc<Mutex<VddState>>,
    /// The shared RCC prescaler state, written by the `RCC` device on
    /// `CFGR`/`D1CFGR`/`D2CFGR` writes. When present, `tick_devices` scales
    /// general-timer deltas by the APB timer-kernel ratio (including the ×2
    /// rule); without it every timer runs at core rate, the old behavior.
    clock: Option<Arc<Mutex<ClockState>>>,
    /// The shared analog bus (WP-R4): one host-driven millivolt node every
    /// `Apin`-fed consumer samples coherently. Pre-installed unset, like
    /// the supply rail is pre-installed at nominal voltage — hosts drive
    /// it on any board via [`Self::set_analog_millivolts`]; consumers fall
    /// back to legacy digital paths while it is unset.
    analog: Arc<Mutex<AnalogBusState>>,
    /// Fractional-tick accumulators, one per APB domain (index 0 = APB1,
    /// 1 = APB2), for exact rational timer scaling across sync points.
    tim_phase: [u64; 2],
    /// The ARM ITM stimulus sink (`0xE000_0000`): firmware `printf`
    /// retargeting lands here when the host enables the unit. Present on
    /// every board without touching any constructor.
    itm: Itm,
    /// In-process hook registry (instruction/watchpoint/peripheral hooks).
    /// Empty by default: one predicted branch per access then.
    hooks: Hooks,
    /// Generation of fetch-affecting configuration, bumped on every write
    /// to the system-control window (`0xE000_ED00..0xE000_EE00`: SCB, MPU,
    /// SAU, FPU). The JIT runner polls it (see `config_epoch`): a missed
    /// bump would be silent wrong execution, so the window is deliberately
    /// wider than just the MPU registers — over-invalidation (e.g. on
    /// `SHCSR` fault-routing writes, which never change decoded meaning)
    /// is harmless, a missed bump is not.
    config_epoch: u64,
    /// Bus-write generation for JIT content validation (see `write_epoch`):
    /// bumped on every successful `write`, whatever the target.
    write_epoch: u64,
}

impl<M: Bus + core::fmt::Debug> core::fmt::Debug for SystemMemory<M> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SystemMemory")
            .field("backing", &self.backing)
            .field("nvic", &self.nvic)
            .field("active", &self.active)
            .field("scb", &self.scb)
            .field("flash", &self.flash)
            .field("qspi", &self.qspi)
            .field("qspi2", &self.qspi2)
            .field("dma_mailbox", &self.dma_mailbox)
            .field("eth_mailbox", &self.eth_mailbox)
            .field("fdcan_mailbox", &self.fdcan_mailbox)
            .field("dma2d_mailbox", &self.dma2d_mailbox)
            .field("watchdog", &self.watchdog)
            .field("clock", &self.clock)
            .field("tim_phase", &self.tim_phase)
            .field("itm", &self.itm)
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
            dev_index: Vec::new(),
            dev_index_len: 0,
            route_force_linear: false,
            dev_index_max_len: 0,
            dev_apb: Vec::new(),
            active: 0,
            nvic: Nvic::new(),
            scb: Scb::new(),
            flash: None,
            qspi: None,
            qspi2: None,
            dma_mailbox: None,
            eth_mailbox: None,
            fdcan_mailbox: None,
            dma2d_mailbox: None,
            watchdog: None,
            vdd: VddState::shared(),
            clock: None,
            analog: AnalogBusState::shared(),
            tim_phase: [0, 0],
            itm: Itm::new(),
            hooks: Hooks::default(),
            config_epoch: 0,
            write_epoch: 0,
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

    /// Switch the SCB's MPU to the `PMSAv8` (v8-M) model, banked per security
    /// state. The machine builder calls this for an `ARMv8-M` core.
    pub fn enable_pmsav8(&mut self) {
        self.scb.set_pmsav8();
    }

    /// Attach a device. Its windows immediately take priority over the
    /// backing memory and over devices that were attached earlier.
    pub fn attach(&mut self, device: Box<dyn InterruptSource>) {
        // No explicit invalidation: `route_device` rebuilds whenever
        // `devices.len()` outgrows `dev_index_len`.
        self.devices.push(device);
    }

    /// Rebuild the sorted window index from the attached devices.
    fn rebuild_dev_index(&mut self) {
        let mut index = Vec::new();
        let mut max_len = 0;
        for (dev, d) in self.devices.iter().enumerate() {
            for r in d.ranges() {
                max_len = max_len.max(r.len);
                index.push(DevWindow {
                    base: r.base,
                    end: r.base.saturating_add(r.len),
                    dev,
                });
            }
        }
        index.sort_by_key(|a| (a.base, a.dev));
        self.dev_index = index;
        self.dev_index_len = self.devices.len();
        self.dev_index_max_len = max_len;
        self.dev_apb = self
            .devices
            .iter()
            .map(|d| Self::tim_apb(d.name()))
            .collect();
    }

    /// Route `addr` to the attached device index, attach-order-first.
    ///
    /// Only windows with `base` in `(addr - max_len, addr]` can contain
    /// `addr` (anything earlier ends at or before it); the survivors are
    /// usually one window, and the minimum attach index wins ties, exactly
    /// matching the pre-index linear scan.
    ///
    /// Measurement hook: when set, routing reverts to the linear scan
    /// inside one process so benches can A/B both paths back-to-back.
    /// Firmware has no reason to touch this; it exists for M7.
    pub fn set_route_force_linear(&mut self, on: bool) {
        self.route_force_linear = on;
    }

    fn route_device(&mut self, addr: u32) -> Option<usize> {
        if self.route_force_linear {
            return self
                .devices
                .iter()
                .position(|d| d.ranges().iter().any(|r| r.contains(addr)));
        }
        if self.dev_index_len != self.devices.len() {
            self.rebuild_dev_index();
        }
        let from = if addr < self.dev_index_max_len {
            0
        } else {
            let floor = addr - self.dev_index_max_len;
            self.dev_index.partition_point(|w| w.base <= floor)
        };
        let to = self.dev_index.partition_point(|w| w.base <= addr);
        self.dev_index[from..to]
            .iter()
            .filter(|w| addr < w.end)
            .map(|w| w.dev)
            .min()
    }

    /// Install the shared flash-controller gate. Writes to `gate.region` are
    /// thereafter gated by the controller's unlock/program state; the `FLASH`
    /// device must be attached with a clone of the same `Arc`. `None` (the
    /// default) leaves every backing write ungated.
    pub fn set_flash_gate(&mut self, gate: Arc<Mutex<FlashGate>>) {
        self.flash = Some(gate);
    }

    /// Install the QSPI gate shared with the `QUADSPI` device. While
    /// `QUADSPI->CR.EN == 0` the `0x9000_0000` window is `Ok(())` without
    /// touching backing, mirroring silicon.
    pub fn set_qspi_gate(&mut self, gate: Arc<Mutex<QspiGate>>) {
        self.qspi = Some(gate);
    }

    /// Install the second QSPI/OCTOSPI2 gate at `0x70000000`.
    pub fn set_qspi_gate2(&mut self, gate: Arc<Mutex<QspiGate>>) {
        self.qspi2 = Some(gate);
    }

    /// Install the shared DMA transfer queue. Every attached DMA controller
    /// must push through the same `Arc`; this bus performs the copies it
    /// accumulates after each device tick. `None` (the default) leaves DMA
    /// units undelivered (a controller with no bus).
    pub fn set_dma_mailbox(&mut self, mailbox: Arc<Mutex<Vec<DmaUnit>>>) {
        self.dma_mailbox = Some(mailbox);
    }

    /// Install the shared Ethernet mailbox. The attached Ethernet controller
    /// must push through the same `Arc`; this bus performs its read/write
    /// requests after each device tick and appends read results to the
    /// mailbox. `None` (the default) leaves the Ethernet controller unable to
    /// reach memory.
    pub fn set_eth_mailbox(&mut self, mailbox: Arc<Mutex<EthMailbox>>) {
        self.eth_mailbox = Some(mailbox);
    }

    /// Install the shared FDCAN mailbox. The attached FDCAN controllers must
    /// push through the same `Arc`; this bus performs their message-RAM reads
    /// and received-frame writes after each device tick. `None` (the default)
    /// leaves FDCAN payloads unable to move (flags still update).
    pub fn set_fdcan_mailbox(&mut self, mailbox: Arc<Mutex<FdcanMailbox>>) {
        self.fdcan_mailbox = Some(mailbox);
    }

    /// Install the shared DMA2D mailbox. The attached DMA2D controller must
    /// push through the same `Arc`; this bus performs its source-row reads
    /// and destination writes after each device tick. `None` (the default)
    /// leaves pixel traffic unable to move (flags still update).
    pub fn set_dma2d_mailbox(&mut self, mailbox: Arc<Mutex<Dma2dMailbox>>) {
        self.dma2d_mailbox = Some(mailbox);
    }

    /// Install the shared watchdog reset latch. The `IWDG`/`WWDG` devices must
    /// be attached with clones of the same `Arc`; when one underflows it sets
    /// the latch and the `Machine` performs a full reset. `None` (the default)
    /// leaves watchdog underflows unable to reset the machine.
    pub fn set_watchdog_reset(&mut self, latch: Arc<Mutex<bool>>) {
        self.watchdog = Some(latch);
    }

    /// The shared supply rail (pre-installed at nominal voltage; see
    /// [`Self::set_vdd`] to replace it).
    #[must_use]
    pub fn vdd_handle(&self) -> Arc<Mutex<VddState>> {
        self.vdd.clone()
    }

    /// Replace the shared supply-rail state.
    pub fn set_vdd(&mut self, vdd: Arc<Mutex<VddState>>) {
        self.vdd = vdd;
    }

    /// Drive the rail to `mv` millivolts, evaluating brown-out with
    /// hysteresis (see [`VddState::set_millivolts`]). This is the
    /// host/SPICE entry point (firmware cannot move the rail); a rail
    /// held under [`BOR_FALL_MV`](crate::device::vdd::BOR_FALL_MV) holds
    /// the machine in reset via [`Self::reset_requested`].
    pub fn set_operating_millivolts(&mut self, mv: u32) {
        self.vdd
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .set_millivolts(mv);
    }

    /// Install the shared RCC prescaler state. The attached `RCC` device must
    /// be built with a clone of the same `Arc` (see `Rcc::with_clock`); this
    /// bus then scales general-timer tick deltas by the APB timer-kernel
    /// ratio. `None` (the default) runs every timer at core rate.
    pub fn set_clock(&mut self, clock: Arc<Mutex<ClockState>>) {
        self.clock = Some(clock);
    }

    /// The current CPU clock in Hz from the shared [`ClockState`]
    /// (through any H7 `D1CPRE` divider, so this is the rate core cycles
    /// actually run at — the rate the co-simulation shim converts SPICE
    /// seconds into). Falls back to [`ClockState::default`] when no board
    /// wired a clock; every board constructor calls [`Self::set_clock`],
    /// so the fallback only serves hand-built buses.
    #[must_use]
    pub fn cpu_hz(&self) -> u32 {
        self.clock
            .as_ref()
            .and_then(|clock| clock.lock().ok().map(|guard| guard.cpu_hz()))
            .unwrap_or_else(|| ClockState::default().cpu_hz())
    }

    /// The shared analog bus (pre-installed unset; see the `analog` field).
    /// Boards wire `Apin` + `Apin`-fed consumers with clones of this handle
    /// (see `AnalogBusState::shared`); hand-built buses can do the same.
    #[must_use]
    pub fn analog_handle(&self) -> Arc<Mutex<AnalogBusState>> {
        self.analog.clone()
    }

    /// Replace the shared analog-bus state.
    pub fn set_analog(&mut self, analog: Arc<Mutex<AnalogBusState>>) {
        self.analog = analog;
    }

    /// Drive the shared analog bus to `mv` millivolts. This is the
    /// host/SPICE entry point (firmware observes through ADC/COMP/GPIO/EXTI/
    /// timer); a single driver per net by convention.
    pub fn set_analog_millivolts(&mut self, mv: u32) {
        AnalogBusState::set_millivolts(&self.analog, mv);
    }

    /// The driven bus value, if the host has driven yet (`None` means
    /// consumers use legacy digital paths).
    #[must_use]
    pub fn analog_millivolts(&self) -> Option<u32> {
        AnalogBusState::millivolts(&self.analog)
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
    ///
    /// # Panics
    ///
    /// Panics if the flash gate mutex is poisoned.
    pub fn reset_devices(&mut self) {
        self.nvic.reset();
        self.active = 0;
        self.scb.reset();
        self.itm.reset();
        // Fractional timer phases are simulation bookkeeping, not silicon
        // state, but they restart with the clocks (the `RCC` device resets
        // its shared handle in the loop below).
        self.tim_phase = [0, 0];
        for device in &mut self.devices {
            device.reset();
        }
        if let Some(gate) = &self.flash {
            gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        }
        if let Some(gate) = &self.qspi {
            gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        }
        if let Some(gate) = &self.qspi2 {
            gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        }
        if let Some(latch) = &self.watchdog {
            *latch.lock().unwrap_or_else(|e| e.into_inner()) = false;
        }
    }

    /// Whether a reset has been latched since the last one — by
    /// `AIRCR.SYSRESETREQ` through the SCB, by an IWDG/WWDG underflow, or
    /// by brown-out (the rail reads asserted while held low, so recovery
    /// resumes immediately with no stale latch). The `Machine` performs a
    /// full reset when set.
    #[must_use]
    pub fn reset_requested(&self) -> bool {
        self.scb.reset_requested()
            || self
                .watchdog
                .as_ref()
                .is_some_and(|latch| *latch.lock().unwrap_or_else(|e| e.into_inner()))
            || self
                .vdd
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .bor_asserted()
    }

    /// The APB domain of a general timer by device name (`TIM1`/`tim1` and
    /// friends live on APB2, every other `TIMx` on APB1), or `None` for
    /// non-timer devices. Matches F1/F4 and H7 (whose APB1 timers are
    /// `TIM2-7/12-14` and APB2 ones `TIM1/8/15-17`); families with a single
    /// APB read the APB2 prescaler as /1, which is their undivided bus.
    fn tim_apb(name: &str) -> Option<u8> {
        let rest = name
            .strip_prefix("tim")
            .or_else(|| name.strip_prefix("TIM"))?;
        match rest {
            "1" | "8" | "9" | "10" | "11" | "15" | "16" | "17" | "20" => Some(2),
            _ if rest.bytes().all(|b| b.is_ascii_digit()) => Some(1),
            _ => None,
        }
    }

    /// Advance every un-gated device by exactly `delta` cycles, then latch any
    /// freshly-asserted interrupt lines into the NVIC's pending register.
    /// Called at a synchronization point with the true elapsed delta; gated
    /// devices skip time entirely, mirroring a powered-down clock domain whose
    /// counters froze. General timers (`tim*`) advance by the APB
    /// timer-kernel ratio instead (see `tim_apb`): with prescalers at /1 this
    /// is exactly `delta`, so unwired boards behave as before.
    pub fn tick_devices(&mut self, delta: u64) {
        // Sample the clock once per sync point; every timer in one APB domain
        // shares the ratio and the fractional accumulator, which keeps the
        // scaling exact across arbitrarily chunked deltas.
        let (ratio1, phase1) = self.tim_scaled(delta, 1);
        let (ratio2, phase2) = self.tim_scaled(delta, 2);
        self.tim_phase = [phase1, phase2];
        if self.dev_apb.len() != self.devices.len() {
            self.rebuild_dev_index();
        }
        for i in 0..self.devices.len() {
            let device = &mut self.devices[i];
            if device.gated() {
                continue;
            }
            let delta = match self.dev_apb[i] {
                Some(1) => ratio1,
                Some(2) => ratio2,
                _ => delta,
            };
            device.tick(delta);
        }
        // DMA units queued by the controllers above land on this bus now.
        self.drain_dma_units();
        // So do the Ethernet controller's descriptor reads and frame writes.
        self.drain_eth_ops();
        // And the FDCAN controllers' message-RAM traffic.
        self.drain_fdcan_ops();
        // And the DMA2D controller's row reads and frame writes.
        self.drain_dma2d_ops();
        self.latch_device_asserts();
    }

    /// Scale `delta` core cycles into APB-domain timer ticks: `delta * num /
    /// den` with the fractional remainder carried in this bus's accumulator
    /// (returned alongside the scaled delta). Without a clock handle or with
    /// /1 prescalers this is the identity.
    fn tim_scaled(&self, delta: u64, apb: u8) -> (u64, u64) {
        let Some(clock) = &self.clock else {
            return (delta, self.tim_phase[usize::from(apb - 1)]);
        };
        let clock = clock.lock().unwrap_or_else(|e| e.into_inner());
        let (num, den) = clock.tim_factor(apb);
        drop(clock);
        let acc = self.tim_phase[usize::from(apb - 1)] as u128 + delta as u128 * num as u128;
        #[allow(clippy::cast_possible_truncation)]
        let scaled = (acc / den as u128) as u64;
        #[allow(clippy::cast_possible_truncation)]
        let phase = (acc % den as u128) as u64;
        (scaled, phase)
    }

    /// Perform the copies queued by the DMA controllers. Reads and writes go
    /// through the full bus path, so a unit can touch peripheral windows and
    /// the flash gate; a unit whose addresses are reserved is dropped.
    fn drain_dma_units(&mut self) {
        let Some(mailbox) = &self.dma_mailbox else {
            return;
        };
        let units = core::mem::take(&mut *mailbox.lock().unwrap_or_else(|e| e.into_inner()));
        for (src, dst, len) in units {
            for i in 0..len {
                let Some(byte) = self.read(src.wrapping_add(u32::from(i))).ok() else {
                    continue;
                };
                let _ = self.write(dst.wrapping_add(u32::from(i)), byte);
            }
        }
    }

    /// Perform the Ethernet controller's queued memory requests. Reads go
    /// through the full bus path and their bytes are appended to the mailbox's
    /// results FIFO, in request order, for the controller to consume on a later
    /// tick; writes are performed directly. A request whose address is reserved
    /// is dropped.
    fn drain_eth_ops(&mut self) {
        let Some(mailbox) = self.eth_mailbox.clone() else {
            return;
        };
        let ops = core::mem::take(&mut mailbox.lock().unwrap_or_else(|e| e.into_inner()).ops);
        for op in ops {
            match op {
                EthMemOp::Read { addr, len } => {
                    let mut bytes = Vec::with_capacity(usize::from(len));
                    for i in 0..len {
                        let Some(byte) = self.read(addr.wrapping_add(u32::from(i))).ok() else {
                            break;
                        };
                        bytes.push(byte);
                    }
                    mailbox
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .results
                        .push_back(bytes);
                }
                EthMemOp::Write { addr, bytes } => {
                    for (i, byte) in bytes.iter().enumerate() {
                        let _ = self.write(
                            addr.wrapping_add(u32::try_from(i).unwrap_or(u32::MAX)),
                            *byte,
                        );
                    }
                }
            }
        }
    }

    /// Perform the FDCAN controllers' queued message-RAM traffic. Reads go
    /// through the full bus path and their bytes are appended as correlated
    /// results, in request order; writes are performed directly. A request
    /// whose addresses are reserved is dropped (short result).
    fn drain_fdcan_ops(&mut self) {
        let Some(mailbox) = self.fdcan_mailbox.clone() else {
            return;
        };
        let reads = core::mem::take(&mut mailbox.lock().unwrap_or_else(|e| e.into_inner()).reads);
        for r in reads {
            let mut bytes = Vec::with_capacity(usize::from(r.len));
            for i in 0..r.len {
                let Some(byte) = self.read(r.addr.wrapping_add(u32::from(i))).ok() else {
                    break;
                };
                bytes.push(byte);
            }
            mailbox
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .results
                .push_back(FdcanResult {
                    kind: r.kind,
                    index: r.index,
                    bytes,
                });
        }
        let writes = core::mem::take(&mut mailbox.lock().unwrap_or_else(|e| e.into_inner()).writes);
        for (addr, bytes) in writes {
            for (i, byte) in bytes.iter().enumerate() {
                let _ = self.write(
                    addr.wrapping_add(u32::try_from(i).unwrap_or(u32::MAX)),
                    *byte,
                );
            }
        }
    }

    /// Perform the DMA2D controller's queued pixel traffic. Source-row
    /// reads go through the full bus path and their bytes are appended as
    /// correlated results, in request order; destination writes are
    /// performed directly. A request whose addresses are reserved is
    /// dropped (short result).
    fn drain_dma2d_ops(&mut self) {
        use crate::device::stm32::fmc::Dma2dResult;
        let Some(mailbox) = self.dma2d_mailbox.clone() else {
            return;
        };
        let reads = core::mem::take(&mut mailbox.lock().unwrap_or_else(|e| e.into_inner()).reads);
        for r in reads {
            let mut bytes = Vec::with_capacity(r.len as usize);
            for i in 0..r.len {
                let Some(byte) = self.read(r.addr.wrapping_add(i)).ok() else {
                    break;
                };
                bytes.push(byte);
            }
            mailbox
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .results
                .push_back(Dma2dResult { row: r.row, bytes });
        }
        let writes = core::mem::take(&mut mailbox.lock().unwrap_or_else(|e| e.into_inner()).writes);
        for (addr, bytes) in writes {
            for (i, byte) in bytes.iter().enumerate() {
                let _ = self.write(
                    addr.wrapping_add(u32::try_from(i).unwrap_or(u32::MAX)),
                    *byte,
                );
            }
        }
    }

    /// Split a priority byte into its group and subpriority parts under the
    /// current `AIRCR.PRIGROUP` (the number of subpriority bits). A zero
    /// `PRIGROUP` gives the whole byte to the group and leaves no subpriority
    /// bits. Preemption is decided on the group part alone; subpriority only
    /// orders which pending exception is activated within the same group.
    fn split_priority(&self, p: u8) -> (u8, u8) {
        let sub = self.scb.prig() & 0x7;
        (p >> sub, if sub == 0 { 0 } else { p & ((1u8 << sub) - 1) })
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
    /// or `u8::MAX` in Thread mode. `HardFault` (3) and `UsageFault` (6) get their
    /// fixed/configurable priorities; `SVCall` (11), `PendSV` (14), and `SysTick`
    /// (15) read the SHPR bytes.
    #[allow(clippy::cast_possible_truncation)]
    fn current_priority(&self, ipsr: u8) -> u8 {
        match ipsr {
            0 => u8::MAX,
            // HardFault is priority -1: encoded as the highest possible
            // (lowest numeric) priority, so nothing preempts it.
            3 => 0,
            4..=6 => self.scb.priority(ipsr),
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
        // A software-pended UsageFault / MemManage / BusFault is taken only
        // while the exception is enabled; disabled, the pend stays latched (it
        // escalates only when a fault is raised synchronously).
        if self.scb.memfault_pending() && self.scb.memfault_enabled() {
            let p = self.scb.priority(4);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, 4, g, s, p);
        }
        if self.scb.busfault_pending() && self.scb.busfault_enabled() {
            let p = self.scb.priority(5);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, 5, g, s, p);
        }
        if self.scb.usage_fault_pending() && self.scb.usage_fault_enabled() {
            let p = self.scb.priority(6);
            let (g, s) = self.split_priority(p);
            Self::consider(&mut best, 6, g, s, p);
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
        // A zero BASEPRI never masks. Faults (3, 6) are not configurable.
        if basepri != 0 && (n == 11 || n >= 14) && priority >= basepri {
            return None;
        }
        // Only a strictly higher-priority (numerically lower) group preempts
        // the one running; an equal group never preempts, even with a higher
        // subpriority. HardFault runs at priority -1 (group 0), so nothing
        // preempts it. Thread mode (IPSR 0) has no priority at all — it sits
        // below every group, including 255 — so anything that survived the
        // BASEPRI mask above preempts it. (Comparing groups numerically
        // alone would read thread mode as group 255 via `current_priority`
        // and wrongly refuse priority-255 SysTick/PendSV, the level every
        // RTOS parks its kernel interrupts at: found via FreeRTOS, whose
        // tick never fired.)
        if current_ipsr == 0 {
            return Some(n);
        }
        let (cur_group, _) = self.split_priority(self.current_priority(current_ipsr));
        if group < cur_group {
            Some(n)
        } else {
            None
        }
    }

    /// The two-phase "consume": the core is taking exception `n`; clear the
    /// NVIC pending latch and tell the source device to drop its line.
    fn consume_exception(&mut self, n: u8) {
        match n {
            4 => self.scb.acknowledge_memfault(),
            5 => self.scb.acknowledge_busfault(),
            6 => self.scb.acknowledge_usage_fault(),
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
                // The source flag is still set: firmware clears it inside the
                // handler, which has not run yet. Mark the line active so the
                // entry and prologue ticks do not re-pend it (the core clears
                // the mark on exception return; a line still asserted then
                // re-latches, like silicon refiring on a held line).
                self.active |= 1u128 << line;
                for device in &mut self.devices {
                    // Acknowledge whichever device line maps to exception `n`:
                    // a single-line device keeps line 0, a controller with
                    // consecutive lines (the F1 DMA) services `n - base`, and
                    // one whose lines scatter (the F4 DMA) matches per line.
                    let lines = device.lines();
                    for l in 0..32 {
                        if lines & (1 << l) != 0 && device.exception_for_line(l) == Some(n) {
                            device.acknowledge(1 << l);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    /// Copy each asserting device line into the NVIC's pending register. A
    /// device's `pending()` is a bitmask of its device-local lines; each set
    /// line maps to an NVIC line through `exception_for_line`, so a controller
    /// whose lines are consecutive (the F1 DMA) lands a contiguous band and one
    /// whose lines scatter (the F4 DMA) still reaches the right vector.
    fn latch_device_asserts(&mut self) {
        for device in &self.devices {
            let mask = device.pending();
            if mask == 0 {
                continue;
            }
            for line in 0..32 {
                if mask & (1 << line) == 0 {
                    continue;
                }
                if let Some(n) = device.exception_for_line(line) {
                    if n >= 16 {
                        let bit = u32::from(n - 16);
                        // Never re-pend a line inside its own handler: the
                        // source flag is legitimately still set until firmware
                        // clears it (see `active`). Held lines refire after
                        // the return clears the mark, like silicon.
                        if self.active & (1u128 << bit) == 0 {
                            self.nvic.latch(1u128 << bit);
                        }
                    }
                }
            }
        }
    }

    /// A backing write, routed through the flash gate when one guards `addr`.
    /// Writes to the gated main-flash region land only while the controller is
    /// unlocked and programming; rejected writes are consumed (no fault) and
    /// latch `WRPRTERR`.
    fn write_backing(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if let Some(g) = &self.qspi {
            let guard = g.lock().unwrap_or_else(|e| e.into_inner());
            if guard.region.contains(addr) && !guard.enabled {
                return Ok(());
            }
        }
        if let Some(g) = &self.qspi2 {
            let guard = g.lock().unwrap_or_else(|e| e.into_inner());
            if guard.region.contains(addr) && !guard.enabled {
                return Ok(());
            }
        }
        let gate = match &self.flash {
            Some(g)
                if g.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .region
                    .contains(addr) =>
            {
                g.clone()
            }
            _ => return self.backing.write(addr, val),
        };
        self.flash_region_write(&gate, addr, val)
    }

    /// A backing read, executing any pending erase first so firmware reading
    /// a just-erased page sees zeros.
    fn read_backing(&mut self, addr: u32) -> Result<u8, BusError> {
        if let Some(g) = &self.qspi {
            let guard = g.lock().unwrap_or_else(|e| e.into_inner());
            if guard.region.contains(addr) && !guard.enabled {
                return Ok(0);
            }
        }
        if let Some(g) = &self.qspi2 {
            let guard = g.lock().unwrap_or_else(|e| e.into_inner());
            if guard.region.contains(addr) && !guard.enabled {
                return Ok(0);
            }
        }
        let gate = match &self.flash {
            Some(g)
                if g.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .region
                    .contains(addr) =>
            {
                g.clone()
            }
            _ => return self.backing.read(addr),
        };
        self.flash_region_read(&gate, addr)
    }

    /// Program (or reject) a write into the gated flash region.
    fn flash_region_write(
        &mut self,
        gate: &Arc<Mutex<FlashGate>>,
        addr: u32,
        val: u8,
    ) -> Result<(), BusError> {
        let mut guard = gate.lock().unwrap_or_else(|e| e.into_inner());
        // A pending erase (`CR.STRT`) lands before the access that triggered it.
        if let Some(erase) = guard.erase.take() {
            let (region, page) = (guard.region, guard.page_size);
            drop(guard);
            self.perform_erase(erase, region, page);
            guard = gate.lock().unwrap_or_else(|e| e.into_inner());
        }
        // Write protection (RM0008 §9.1.5 `WRPR`): a cleared bit protects its
        // page; a protected page rejects programming and latches the error
        // even while unlocked and programming.
        let page_index = (addr - guard.region.base) / guard.page_size.max(1);
        if guard.wrpr & (1 << page_index.min(31)) == 0 {
            guard.wrprterr = true;
            drop(guard);
            return Ok(());
        }
        if guard.unlocked && guard.program {
            drop(guard);
            let result = self.backing.write(addr, val);
            gate.lock().unwrap_or_else(|e| e.into_inner()).eop = true;
            // Direct backing write (bypasses `write`): bump the generation
            // by hand so JIT content validation sees the new bytes (a
            // missed bump here is silent stale-code execution).
            if result.is_ok() {
                self.write_epoch = self.write_epoch.wrapping_add(1);
            }
            result
        } else {
            guard.wrprterr = true;
            drop(guard);
            Ok(())
        }
    }

    /// Read from the gated flash region, executing a pending erase first.
    fn flash_region_read(
        &mut self,
        gate: &Arc<Mutex<FlashGate>>,
        addr: u32,
    ) -> Result<u8, BusError> {
        let mut guard = gate.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(erase) = guard.erase.take() {
            let (region, page) = (guard.region, guard.page_size);
            drop(guard);
            self.perform_erase(erase, region, page);
        }
        self.backing.read(addr)
    }

    /// Zero the backing flash bank for a page, sector, or mass erase.
    /// Direct backing writes (bypassing `write`, and executing lazily on a
    /// later read): bump the generation afterwards so JIT content
    /// validation sees the erased bytes.
    fn perform_erase(&mut self, erase: FlashErase, region: Region, page_size: u32) {
        match erase {
            FlashErase::Mass => {
                for off in 0..region.len {
                    let _ = self.backing.write(region.base + off, 0);
                }
            }
            FlashErase::Page(addr) => {
                let base = (addr / page_size) * page_size;
                for off in 0..page_size {
                    let _ = self.backing.write(base + off, 0);
                }
            }
            FlashErase::Range(addr, len) => {
                for off in 0..len {
                    let _ = self.backing.write(addr + off, 0);
                }
            }
        }
        self.write_epoch = self.write_epoch.wrapping_add(1);
    }
}

impl<M: Bus> Bus for SystemMemory<M> {
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // Resolve first so hooks fire after (and only after) a successful
        // access, with the device name for peripheral events.
        enum Target {
            Device(usize),
            Scb,
            Nvic,
            Itm,
            Backing,
        }
        let target = if let Some(i) = self.route_device(addr) {
            Target::Device(i)
        } else if self.scb.ranges().iter().any(|r| r.contains(addr)) {
            Target::Scb
        } else if self.nvic.ranges().iter().any(|r| r.contains(addr)) {
            Target::Nvic
        } else if self.itm.ranges().iter().any(|r| r.contains(addr)) {
            Target::Itm
        } else {
            Target::Backing
        };
        let value = match target {
            Target::Device(i) => self.devices[i].read(addr)?,
            Target::Scb => self.scb.read(addr)?,
            Target::Nvic => self.nvic.read(addr)?,
            Target::Itm => self.itm.read(addr)?,
            Target::Backing => self.read_backing(addr)?,
        };
        if self.hooks.has_read() {
            self.hooks.fire_read(addr, value);
        }
        let name = match target {
            Target::Device(i) => Some(self.devices[i].name()),
            Target::Scb => Some("scb"),
            Target::Nvic => Some("nvic"),
            Target::Itm => Some("itm"),
            Target::Backing => None,
        };
        if let Some(name) = name {
            if self.hooks.has_prd() {
                self.hooks.fire_peripheral(name, addr, value, false);
            }
        }
        Ok(value)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        enum Target {
            Device(usize),
            Scb,
            Nvic,
            Itm,
            Backing,
        }
        let target = if let Some(i) = self.route_device(addr) {
            Target::Device(i)
        } else if self.scb.ranges().iter().any(|r| r.contains(addr)) {
            Target::Scb
        } else if self.nvic.ranges().iter().any(|r| r.contains(addr)) {
            Target::Nvic
        } else if self.itm.ranges().iter().any(|r| r.contains(addr)) {
            Target::Itm
        } else {
            Target::Backing
        };
        match target {
            Target::Device(i) => self.devices[i].write(addr, val)?,
            Target::Scb => self.scb.write(addr, val)?,
            Target::Nvic => self.nvic.write(addr, val)?,
            Target::Itm => self.itm.write(addr, val)?,
            Target::Backing => self.write_backing(addr, val)?,
        }
        // Every writer funnels through here — CPU stores, DMA/mailbox
        // drains, debugger writes — so one bump site covers all M5 sources.
        self.write_epoch = self.write_epoch.wrapping_add(1);
        if (0xE000_ED00..0xE000_EE00).contains(&addr) {
            self.config_epoch = self.config_epoch.wrapping_add(1);
        }
        if self.hooks.has_write() {
            self.hooks.fire_write(addr, val);
        }
        let name = match target {
            Target::Device(i) => Some(self.devices[i].name()),
            Target::Scb => Some("scb"),
            Target::Nvic => Some("nvic"),
            Target::Itm => Some("itm"),
            Target::Backing => None,
        };
        if let Some(name) = name {
            if self.hooks.has_pwr() {
                self.hooks.fire_peripheral(name, addr, val, true);
            }
        }
        Ok(())
    }

    fn write_epoch_ptr(&mut self) -> Option<*mut u64> {
        Some(&mut self.write_epoch as *mut u64)
    }

    /// Translate a data access for the JIT fast path, mirroring the
    /// read/write dispatch exactly: anything that is not plain backing
    /// bytes (devices, SCB/NVIC/ITM, disabled QSPI windows, the gated
    /// flash region with its erase-on-read and program semantics) is
    /// `Indirect`, forcing a side exit to the interpreter.
    fn translate(&mut self, addr: u32, _access: MpuAccess) -> DataTranslate {
        if self.route_device(addr).is_some() {
            return DataTranslate::Indirect;
        }
        if self.scb.ranges().iter().any(|r| r.contains(addr))
            || self.nvic.ranges().iter().any(|r| r.contains(addr))
            || self.itm.ranges().iter().any(|r| r.contains(addr))
        {
            return DataTranslate::Indirect;
        }
        // Disabled QSPI windows consume accesses (reads see zeros); an
        // enabled window is plain backing flash (mirrors `read_backing`).
        for gate in [&self.qspi, &self.qspi2].into_iter().flatten() {
            let guard = gate.lock().unwrap_or_else(|e| e.into_inner());
            if guard.region.contains(addr) {
                if !guard.enabled {
                    return DataTranslate::Indirect;
                }
                break;
            }
        }
        // The gated flash region reads through erase handling and writes
        // through program gating: never directly mapped.
        if let Some(g) = &self.flash {
            if g.lock()
                .unwrap_or_else(|e| e.into_inner())
                .region
                .contains(addr)
            {
                return DataTranslate::Indirect;
            }
        }
        self.backing.translate(addr, _access)
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

    fn note_exception_return(&mut self, n: u8) {
        if n >= 16 {
            self.active &= !(1u128 << u32::from(n - 16));
        }
    }

    fn tick_cycle(&mut self) {
        self.tick_devices(1);
    }

    fn tick_by(&mut self, delta: u64) {
        self.tick_devices(delta);
    }

    fn reset_peripherals(&mut self) {
        self.reset_devices();
    }

    /// Drain a named host-output channel: `itm` first, then any attached
    /// device with a matching [`Peripheral::name`].
    fn drain_channel(&mut self, name: &str) -> Vec<u8> {
        if name == self.itm.name() {
            return self.itm.drain_output();
        }
        for device in &mut self.devices {
            if device.name() == name {
                return device.drain_output();
            }
        }
        Vec::new()
    }

    /// Whether the SCB or a watchdog has latched a reset request. The generic
    /// `Machine` sees this through the trait, so it must be overridden here —
    /// the default (false) would never reboot the machine.
    fn reset_requested(&self) -> bool {
        SystemMemory::reset_requested(self)
    }

    fn hooks(&self) -> Option<&Hooks> {
        Some(&self.hooks)
    }

    fn hooks_mut(&mut self) -> Option<&mut Hooks> {
        Some(&mut self.hooks)
    }

    fn drive_device(&mut self, name: &str, bytes: &[u8]) -> bool {
        let Some(device) = self.devices.iter_mut().find(|d| d.name() == name) else {
            return false;
        };
        for &b in bytes {
            device.drive_inputs(u32::from(b));
        }
        true
    }

    fn record_fault(&mut self, cfsr_bits: u32, hfsr_bits: u32) {
        self.scb.record_fault(cfsr_bits, hfsr_bits);
    }

    fn record_bus_fault(&mut self, addr: u32, hfsr_bits: u32) {
        self.scb.record_bus_fault(addr, hfsr_bits);
    }

    fn record_secure_fault(&mut self, bits: u32) {
        self.scb.record_secure_fault(bits);
    }

    fn mpu_check(&mut self, addr: u32, access: MpuAccess, privileged: bool, exception: u8) -> bool {
        self.scb.mpu_check(addr, access, privileged, exception)
    }

    fn config_epoch(&self) -> Option<u64> {
        Some(self.config_epoch)
    }

    fn write_epoch(&self) -> Option<u64> {
        Some(self.write_epoch)
    }

    fn mpu_fault(&mut self, addr: u32, fetch: bool) {
        self.scb.record_mpu_fault(addr, fetch);
    }

    fn mpu_violation(&mut self, addr: u32, fetch: bool) {
        self.scb.record_mpu_violation(addr, fetch);
    }

    /// The ARMSSE (mps2-an505-style) IDAU, which attributes by address bit
    /// `[31:28]`: even regions are Non-secure, the odd `NSC`-capable regions
    /// `0x1xxxxxxx`/`0x3xxxxxxx` are Non-secure-callable only while the
    /// corresponding `NSCFG` bit is set (reset: not NSC), and the system
    /// aliases `0xE0000000..0xE00FFFFF`/`0xF0000000..0xF00FFFFF` are exempt.
    fn idau_attribution(&mut self, addr: u32) -> Option<IdauAttribution> {
        let region = ((addr >> 28) & 0xF) as u8;
        let ns = region & 1 == 0;
        let exempt = addr & 0xEFF0_0000 == 0xE000_0000;
        Some(IdauAttribution {
            region,
            ns,
            nsc: false,
            exempt,
        })
    }

    fn sau_hit(&mut self, addr: u32) -> SauHit {
        self.scb.sau_hit(addr)
    }

    fn mpu_lookup_v8(&mut self, addr: u32, secure: bool, privileged: bool) -> MpuLookup {
        self.scb.mpu_lookup_v8(addr, secure, privileged)
    }

    fn set_secure(&mut self, secure: bool) {
        self.scb.set_secure(secure);
    }

    fn nvic_irq_targets_secure(&mut self, line: u32) -> bool {
        self.nvic.targets_secure(line)
    }

    fn vector_table_base(&mut self, secure: bool) -> u32 {
        self.scb.vector_table_base(secure)
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

    /// Pre-index routing semantics: first attached device whose window
    /// contains the address. The index must agree with this everywhere.
    fn linear_route(sys: &SystemMemory<FlatMemory>, addr: u32) -> Option<usize> {
        sys.devices
            .iter()
            .position(|d| d.ranges().iter().any(|r| r.contains(addr)))
    }

    #[test]
    fn route_index_matches_linear_scan_on_synthetic_overlap() {
        let mut sys = SystemMemory::new(ram());
        sys.attach(Box::new(Beacon::at(0x4200_0000))); // [..+16)
        sys.attach(Box::new(Beacon::at(0x4200_000F))); // 1-byte overlap
        sys.attach(Box::new(Beacon::at(0x4300_0000))); // disjoint
                                                       // The overlapped byte routes to the first-attached device (whose
                                                       // offset 15 is an ignored lane, while B's offset 0 is its low
                                                       // data byte): a B win would read back 0xAB.
        sys.write(0x4200_000F, 0xAB).unwrap();
        assert_eq!(sys.read(0x4200_000F).unwrap(), 0x00);
        // Both neighbors still work: A-only data area and B-only window.
        sys.write_word(0x4200_0000, 0x1111_2222).unwrap();
        assert_eq!(sys.read_word(0x4200_0000).unwrap(), 0x1111_2222);
        assert_eq!(sys.read(0x4200_0010).unwrap(), 0x00); // B.reg byte 1
                                                          // ...and the router agrees with the scan on every sampled address.
        for base in [0x4200_0000, 0x4200_000F, 0x4300_0000, RAM, 0x5000_0000] {
            for delta in 0..32u32 {
                let addr = base.wrapping_add(delta);
                assert_eq!(
                    sys.route_device(addr),
                    linear_route(&sys, addr),
                    "route {addr:#x}"
                );
            }
        }
    }

    #[test]
    fn route_index_matches_linear_scan_on_a_real_board() {
        let mut sys = crate::board::stm32f103();
        let mut addrs = vec![
            0x0000_0000,
            0x0800_0000,
            0x2000_0000,
            0x4000_0000,
            0xE000_0000,
            0xFFFF_FFFF,
        ];
        for d in sys.devices.iter() {
            for r in d.ranges() {
                let end = r.base.wrapping_add(r.len);
                addrs.extend([
                    r.base.wrapping_sub(1),
                    r.base,
                    r.base.wrapping_add(1),
                    end.wrapping_sub(1),
                    end,
                ]);
            }
        }
        for addr in addrs {
            assert_eq!(
                sys.route_device(addr),
                linear_route(&sys, addr),
                "route {addr:#x}"
            );
        }
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
    fn shcsr_usagefaultpend_takes_when_enabled() {
        let mut sys = SystemMemory::new(ram());
        // Pend a UsageFault while it is disabled: the pend stays latched.
        sys.write_word(0xE000_ED24, 0x0000_1000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
        // Enable USGFAULTENA: the pended UsageFault is then taken (exc 6).
        sys.write_word(0xE000_ED24, 0x0004_1000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(6));
        // Acknowledge clears the pend; the second peek sees nothing.
        sys.acknowledge_exception(6);
        assert_eq!(sys.next_exception(false, false, 0, 0), None);
    }

    #[test]
    fn usagefault_pend_blocked_inside_hardfault() {
        let mut sys = SystemMemory::new(ram());
        sys.write_word(0xE000_ED24, 0x0004_1000).unwrap();
        // Inside the HardFault handler (ipsr 3, priority -1) nothing preempts.
        assert_eq!(sys.next_exception(false, false, 0, 3), None);
        // In Thread mode it is taken.
        assert_eq!(sys.next_exception(false, false, 0, 0), Some(6));
    }

    #[test]
    fn usagefault_pend_preempts_lower_priority_handler() {
        let mut sys = SystemMemory::new(ram());
        // IRQ7 (exc 23) running at priority 0xC0; UsageFault pended at 0x40.
        sys.write_word(0xE000_E200, 1 << 7).unwrap();
        sys.write_word(0xE000_E100, 1 << 7).unwrap();
        sys.write(0xE000_E407, 0xC0).unwrap();
        sys.write(0xE000_ED1A, 0x40).unwrap(); // SHPR1 UsageFault prio 0x40
        sys.write_word(0xE000_ED24, 0x0004_1000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), Some(6));
        // At the same priority the running handler blocks the pend.
        sys.write(0xE000_ED1A, 0xC0).unwrap();
        sys.write_word(0xE000_ED24, 0x0004_1000).unwrap();
        assert_eq!(sys.next_exception(false, false, 0, 23), None);
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

    const MAIN_FLASH: u32 = 0x0800_0000;
    const MAIN_FLASH_LEN: u32 = 0x1000;

    /// A `SystemMemory` over RAM plus a gated main-flash bank, sharing the
    /// gate with the (unattached) controller state the tests drive directly.
    fn gated_system() -> (
        SystemMemory<FlatMemory>,
        std::sync::Arc<std::sync::Mutex<crate::device::stm32::flash::FlashGate>>,
    ) {
        let backing = FlatMemory::with_layout(&[RAM, MAIN_FLASH], &[RAM_LEN, MAIN_FLASH_LEN]);
        let gate = std::sync::Arc::new(std::sync::Mutex::new(
            crate::device::stm32::flash::FlashGate::new(
                Region::new(MAIN_FLASH, MAIN_FLASH_LEN),
                1024,
            ),
        ));
        let mut sys = SystemMemory::new(backing);
        sys.set_flash_gate(gate.clone());
        (sys, gate)
    }

    #[test]
    fn flash_writes_rejected_while_locked() {
        let (mut sys, gate) = gated_system();
        sys.write_word(MAIN_FLASH, 0xDEAD_BEEF).unwrap();
        assert_eq!(
            sys.read_word(MAIN_FLASH).unwrap(),
            0,
            "locked flash writes are consumed, not stored"
        );
        assert!(
            gate.lock().unwrap_or_else(|e| e.into_inner()).wrprterr,
            "WRPRTERR latches"
        );
    }

    #[test]
    fn flash_writes_land_when_unlocked_and_programming() {
        let (mut sys, gate) = gated_system();
        {
            let mut g = gate.lock().unwrap_or_else(|e| e.into_inner());
            g.unlocked = true;
            g.program = true;
        }
        sys.write_word(MAIN_FLASH, 0xDEAD_BEEF).unwrap();
        assert_eq!(sys.read_word(MAIN_FLASH).unwrap(), 0xDEAD_BEEF);
        assert!(
            gate.lock().unwrap_or_else(|e| e.into_inner()).eop,
            "EOP latches after a program"
        );
    }

    /// `WRPR` write protection (RM0008 §9.1.5): a cleared bit protects its
    /// page — writes there are consumed with `WRPRTERR` even while unlocked
    /// and programming, while unprotected pages in the same bank still land.
    #[test]
    fn wrpr_protects_pages_even_when_unlocked() {
        use crate::device::stm32::flash::FlashGate as G;
        const FLASH: u32 = MAIN_FLASH;
        let backing = FlatMemory::with_layout(&[FLASH], &[0x1000]);
        // 4 pages of 1024 bytes.
        let gate = std::sync::Arc::new(std::sync::Mutex::new(G::new(
            Region::new(FLASH, 0x1000),
            1024,
        )));
        let mut sys = SystemMemory::new(backing);
        sys.set_flash_gate(gate.clone());
        {
            let mut g = gate.lock().unwrap_or_else(|e| e.into_inner());
            g.unlocked = true;
            g.program = true;
            g.wrpr = !(1u32 << 1); // protect page 1 only
        }
        // Unprotected page 0 lands; protected page 1 rejects + errors.
        sys.write_word(FLASH, 0x1111_1111).unwrap();
        sys.write_word(FLASH + 1024, 0x2222_2222).unwrap();
        assert_eq!(sys.read_word(FLASH).unwrap(), 0x1111_1111);
        assert_eq!(sys.read_word(FLASH + 1024).unwrap(), 0, "page 1 protected");
        let g = gate.lock().unwrap_or_else(|e| e.into_inner());
        assert!(g.wrprterr, "WRPRTERR latched");
        drop(g);
        // Clearing the flag restores normal reporting.
        let mut g = gate.lock().unwrap_or_else(|e| e.into_inner());
        g.wrprterr = false;
        drop(g);
        sys.write_word(FLASH + 2048, 0x3333_3333).unwrap();
        assert_eq!(sys.read_word(FLASH + 2048).unwrap(), 0x3333_3333);
    }

    #[test]
    fn page_erase_zeroes_only_the_target_page() {
        use crate::device::stm32::flash::FlashErase;
        let (mut sys, gate) = gated_system();
        {
            let mut g = gate.lock().unwrap_or_else(|e| e.into_inner());
            g.unlocked = true;
            g.program = true;
        }
        sys.write_word(MAIN_FLASH, 0xAAAA_AAAA).unwrap();
        sys.write_word(MAIN_FLASH + 0x400, 0xBBBB_BBBB).unwrap();
        gate.lock().unwrap_or_else(|e| e.into_inner()).erase = Some(FlashErase::Page(MAIN_FLASH));
        // The next access to the region runs the pending erase first.
        assert_eq!(sys.read_word(MAIN_FLASH).unwrap(), 0);
        assert_eq!(sys.read_word(MAIN_FLASH + 0x400).unwrap(), 0xBBBB_BBBB);
    }

    #[test]
    fn mass_erase_zeroes_the_whole_region() {
        use crate::device::stm32::flash::FlashErase;
        let (mut sys, gate) = gated_system();
        {
            let mut g = gate.lock().unwrap_or_else(|e| e.into_inner());
            g.unlocked = true;
            g.program = true;
        }
        sys.write_word(MAIN_FLASH, 0x1111_1111).unwrap();
        sys.write_word(MAIN_FLASH + MAIN_FLASH_LEN - 4, 0x2222_2222)
            .unwrap();
        gate.lock().unwrap_or_else(|e| e.into_inner()).erase = Some(FlashErase::Mass);
        assert_eq!(sys.read_word(MAIN_FLASH).unwrap(), 0);
        assert_eq!(sys.read_word(MAIN_FLASH + MAIN_FLASH_LEN - 4).unwrap(), 0);
    }

    #[test]
    fn flash_gate_leaves_ram_and_other_banks_alone() {
        let (mut sys, _gate) = gated_system();
        sys.write_word(RAM, 0xCAFE_F00D).unwrap();
        assert_eq!(sys.read_word(RAM).unwrap(), 0xCAFE_F00D);
    }

    #[test]
    fn dma_mailbox_units_land_after_device_ticks() {
        use crate::device::stm32::dma::{DmaF1, DmaRequests};
        use std::sync::{Arc, Mutex};

        const DMA: u32 = 0x4002_0000;

        // Source bytes in RAM at `0x2000_0100`; DMA1 channel 1 copies them to
        // `0x2000_0200` through the mailbox the controller shares with this bus.
        let mut backing = ram();
        for i in 0u8..8 {
            backing.write(RAM + u32::from(i) + 0x100, i).unwrap();
        }
        let mailbox = Arc::new(Mutex::new(Vec::new()));
        let mut sys = SystemMemory::new(backing);
        sys.set_dma_mailbox(mailbox.clone());
        sys.attach(Box::new(DmaF1::dma1(
            mailbox,
            Arc::new(Mutex::new(DmaRequests::new())),
        )));
        sys.write_word(DMA + 0x10, RAM + 0x100).unwrap(); // CPAR1
        sys.write_word(DMA + 0x14, RAM + 0x200).unwrap(); // CMAR1
        sys.write_word(DMA + 0x0C, 8).unwrap(); // CNDTR1
        sys.write_word(DMA + 0x08, 0x4000 | 0x80 | 0x40 | 0x1)
            .unwrap(); // CCR1: MEM2MEM|MINC|PINC|EN
        sys.tick_devices(1);
        assert_eq!(
            sys.read_word(RAM + 0x200).unwrap(),
            0x0302_0100,
            "the first four bytes landed"
        );
        assert_eq!(
            sys.read_word(RAM + 0x204).unwrap(),
            0x0706_0504,
            "the next four bytes landed"
        );
    }

    /// A tick-counting device with a caller-chosen name, so `tick_devices`
    /// clock scaling can be observed per APB domain through one shared total.
    struct Ticker {
        ranges: [Region; 1],
        name: &'static str,
        seen: std::sync::Arc<std::sync::Mutex<u64>>,
    }

    impl Ticker {
        fn named(base: u32, name: &'static str) -> (Self, std::sync::Arc<std::sync::Mutex<u64>>) {
            let seen = std::sync::Arc::new(std::sync::Mutex::new(0));
            (
                Self {
                    ranges: [Region::new(base, 4)],
                    name,
                    seen: seen.clone(),
                },
                seen,
            )
        }
    }

    impl Peripheral for Ticker {
        fn ranges(&self) -> &[Region] {
            &self.ranges
        }

        fn read(&mut self, _addr: u32) -> Result<u8, BusError> {
            Err(BusError::Reserved)
        }

        fn write(&mut self, _addr: u32, _val: u8) -> Result<(), BusError> {
            Err(BusError::Reserved)
        }

        fn tick(&mut self, delta: u64) {
            // NOTE: bind one guard — two `lock()`s in one statement deadlocks.
            let mut seen = self.seen.lock().unwrap_or_else(|e| e.into_inner());
            *seen = seen.wrapping_add(delta);
        }

        fn name(&self) -> &'static str {
            self.name
        }
    }

    impl InterruptSource for Ticker {}

    /// A clocked test bus plus its shared clock and tick counters.
    type TimedSystem = (
        SystemMemory<FlatMemory>,
        std::sync::Arc<std::sync::Mutex<crate::device::stm32::rcc::ClockState>>,
        std::sync::Arc<std::sync::Mutex<u64>>,
        std::sync::Arc<std::sync::Mutex<u64>>,
        std::sync::Arc<std::sync::Mutex<u64>>,
    );

    fn timed_system() -> TimedSystem {
        use crate::device::stm32::rcc::ClockState;
        let clock = ClockState::shared();
        let mut sys = SystemMemory::new(ram());
        sys.set_clock(clock.clone());
        let (apb1, seen1) = Ticker::named(0x4000_0000, "tim2");
        let (apb2, seen2) = Ticker::named(0x4001_0000, "tim1");
        let (plain, seen3) = Ticker::named(0x4002_0000, "beacon");
        sys.attach(Box::new(apb1));
        sys.attach(Box::new(apb2));
        sys.attach(Box::new(plain));
        (sys, clock, seen1, seen2, seen3)
    }

    fn seen_total(seen: &std::sync::Arc<std::sync::Mutex<u64>>) -> u64 {
        *seen.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[test]
    fn tim_apb_routes_advanced_timers_to_apb2() {
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("tim1"), Some(2));
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("TIM8"), Some(2));
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("tim17"), Some(2));
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("tim2"), Some(1));
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("tim14"), Some(1));
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("gpioa"), None);
        assert_eq!(SystemMemory::<FlatMemory>::tim_apb("usart1"), None);
    }

    #[test]
    fn timers_scale_with_apb_ratios_and_carry_fractions() {
        let (mut sys, clock, seen1, seen2, seen3) = timed_system();
        // PPRE1=/4 (×2 rule → 1/2 rate), PPRE2=/1 (full rate).
        clock.lock().unwrap_or_else(|e| e.into_inner()).ppre1 = 4;
        sys.tick_devices(10);
        assert_eq!(seen_total(&seen1), 5, "APB1 timer at half rate");
        assert_eq!(seen_total(&seen2), 10, "APB2 timer at full rate");
        assert_eq!(seen_total(&seen3), 10, "non-timer unscaled");
        // Fractional carry: 1+1 core cycles at half rate make exactly 1 tick.
        sys.tick_devices(1);
        assert_eq!(seen_total(&seen1), 5);
        sys.tick_devices(1);
        assert_eq!(seen_total(&seen1), 6);
        // Reset clears the fractional phase.
        sys.reset_devices();
        sys.tick_devices(1);
        assert_eq!(seen_total(&seen1), 6, "phase restarted, no carry yet");
    }

    #[test]
    fn timers_run_at_core_rate_without_a_clock() {
        let mut sys = SystemMemory::new(ram());
        let (tim, seen) = Ticker::named(0x4000_0000, "tim2");
        sys.attach(Box::new(tim));
        sys.tick_devices(7);
        assert_eq!(seen_total(&seen), 7);
    }
}

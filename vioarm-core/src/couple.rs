//! Machine-driven live coupling: bounded quanta with timestamped events.
//!
//! The SPICE-master path (`vioarm_cosim_step_duration` over
//! [`Machine::run_cycles`](crate::machine::Machine::run_cycles)) advances
//! the machine a caller-chosen number of true core cycles and leaves event
//! observation to the caller (poll pins, drain `apin`). This module is the
//! reverse direction: the machine runs ahead at most one quantum and
//! publishes what changed, so a sparse-analog consumer steps to stamps
//! instead of polling every quantum.
//!
//! Single-producer / single-consumer by construction: the machine produces
//! events into the returned [`QuantumOutcome`] (or a caller-drained
//! [`CoupledQueue`]); the bridge consumes them. The machine stays
//! single-threaded — no threads, ever.

use crate::board::BoardMemory;
use crate::error::HaltCause;
use crate::machine::Machine;
use std::collections::VecDeque;

/// Default quantum in core cycles: 1 µs at the 8 MHz virtual-board rate.
/// Real callers derive the quantum from live time
/// ([`seconds_to_cycles`] at [`Machine::cpu_hz`](crate::machine::Machine::cpu_hz));
/// this constant only anchors tests and docs.
pub const DEFAULT_QUANTUM_CYCLES: u64 = 8;

/// Default event-queue capacity: one quantum of dense pin toggles plus
/// analog words, with headroom. [`CoupledQueue::push`] refuses past it so
/// an unconsumed bridge surfaces instead of growing without bound.
pub const DEFAULT_QUEUE_CAPACITY: usize = 4096;

/// Convert SPICE seconds to true core cycles at `hz`.
///
/// Saturates instead of wrapping; non-finite or negative inputs map to
/// `0` (the C ABI rejects those before calling — see
/// `vioarm_cosim_run_quantum`).
#[must_use]
pub fn seconds_to_cycles(seconds: f64, hz: u32) -> u64 {
    if !seconds.is_finite() || seconds <= 0.0 || hz == 0 {
        return 0;
    }
    let cycles = seconds * f64::from(hz);
    if !cycles.is_finite() {
        return u64::MAX;
    }
    cycles.min(u64::MAX as f64) as u64
}

/// Convert a `cyccnt` stamp to SPICE seconds at `hz`.
///
/// `hz` is read live (see `ClockState::cpu_hz`): a PLL reconfiguration
/// mid-run changes the rate future stamps convert at, never past stamps —
/// stamps stay `cyccnt`, the consumer converts with the rate in force.
#[must_use]
pub fn cyccnt_to_seconds(cyccnt: u64, hz: u32) -> f64 {
    if hz == 0 {
        return 0.0;
    }
    // `u64` fits `f64` with rounding only past 2^53 — documented, and the
    // stamp itself stays exact.
    #[allow(clippy::cast_precision_loss)]
    let cycles = cyccnt as f64;
    cycles / f64::from(hz)
}

/// What changed inside a quantum.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoupledEventKind {
    /// A watched pin changed level or drive state.
    Pin {
        /// GPIO port (0 = A).
        port: u8,
        /// Pin index.
        pin: u8,
        /// Sampled level after the retiring step.
        level: bool,
        /// Whether firmware drives the pin (output/alternate).
        driving: bool,
    },
    /// Firmware staged one `apin` word (`TXMV` + `TXGO`).
    Analog {
        /// Millivolts of the staged word.
        mv: u32,
    },
}

/// One change with the exact `cyccnt` after its retiring step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoupledEvent {
    /// `DWT CYCCNT` immediately after the step that produced the change.
    pub cyccnt: u64,
    /// What changed.
    pub kind: CoupledEventKind,
}

impl CoupledEvent {
    /// A pin-change event.
    #[must_use]
    pub const fn pin(cyccnt: u64, port: u8, pin: u8, level: bool, driving: bool) -> Self {
        Self {
            cyccnt,
            kind: CoupledEventKind::Pin {
                port,
                pin,
                level,
                driving,
            },
        }
    }

    /// An analog-word event.
    #[must_use]
    pub const fn analog(cyccnt: u64, mv: u32) -> Self {
        Self {
            cyccnt,
            kind: CoupledEventKind::Analog { mv },
        }
    }

    /// Whether this is a pin event.
    #[must_use]
    pub const fn is_pin(&self) -> bool {
        matches!(self.kind, CoupledEventKind::Pin { .. })
    }

    /// Whether this is an analog event.
    #[must_use]
    pub const fn is_analog(&self) -> bool {
        matches!(self.kind, CoupledEventKind::Analog { .. })
    }
}

/// Bounded single-producer / single-consumer event queue.
///
/// The machine pushes, the bridge drains — one thread, no locking beyond
/// the caller's own handle discipline. [`Self::push`] returns `false`
/// past capacity instead of growing: the bridge must drain each quantum.
#[derive(Clone, Debug, Default)]
pub struct CoupledQueue {
    events: VecDeque<CoupledEvent>,
    capacity: usize,
}

impl CoupledQueue {
    /// An empty queue with `capacity` slots (`0` means unbounded, for tests).
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            events: VecDeque::new(),
            capacity,
        }
    }

    /// Push one event; `false` when a bounded queue is full (event dropped).
    pub fn push(&mut self, event: CoupledEvent) -> bool {
        if self.capacity != 0 && self.events.len() >= self.capacity {
            return false;
        }
        self.events.push_back(event);
        true
    }

    /// Drain every buffered event in retire order.
    pub fn drain(&mut self) -> Vec<CoupledEvent> {
        self.events.drain(..).collect()
    }

    /// Buffered event count.
    #[must_use]
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// Whether the queue holds no events.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Forget buffered events without consuming them.
    pub fn clear(&mut self) {
        self.events.clear();
    }
}

/// One bounded machine-driven run: the events plus how it stopped.
#[derive(Clone, Debug)]
pub struct QuantumOutcome {
    /// Change-only events in retire order (pin changes in `watch` order,
    /// then at most one analog word per step — never reordered).
    pub events: Vec<CoupledEvent>,
    /// `None` when the full quantum retired; `Some` on an early halt.
    pub halt: Option<HaltCause>,
    /// `cyccnt` before the first step.
    pub start_cyccnt: u64,
    /// `cyccnt` after the last step (or the faulting step).
    pub end_cyccnt: u64,
    /// Retired steps (`0` when `max_cycles` was `0`).
    pub steps: u64,
}

impl QuantumOutcome {
    /// Whether the quantum retired fully without halting.
    #[must_use]
    pub const fn completed(&self) -> bool {
        self.halt.is_none()
    }

    /// Consumed core cycles (`end_cyccnt - start_cyccnt`, wrapping).
    #[must_use]
    pub fn consumed_cycles(&self) -> u64 {
        self.end_cyccnt.wrapping_sub(self.start_cyccnt)
    }
}

impl<M: crate::mem::PeripheralBus> Machine<M> {
    /// `DWT CYCCNT` now (the unit every [`CoupledEvent`] stamps).
    #[must_use]
    pub fn cyccnt(&self) -> u64 {
        self.core.cyccnt
    }
}

impl Machine<BoardMemory> {
    /// Retire at most `max_cycles` of true core cycles, buffering
    /// change-only pin events for `watch` plus every staged `apin` word.
    ///
    /// The quantum is a forced sync point: this never runs past
    /// `start + max_cycles` except by one step's overshoot (same bound as
    /// [`Machine::run_cycles`](crate::machine::Machine::run_cycles)), and
    /// never past a halt. Each event carries the `cyccnt` after its
    /// retiring step; events push in retire order, so stamps are monotone
    /// non-decreasing. Steady pins emit nothing — the bridge holds its
    /// last value, like the held-DAC bridge.
    ///
    /// `watch` order is the observation order per step (pin changes sort
    /// before the step's analog word). Absent ports/pins never emit.
    /// The SPICE-master path is untouched: identical firmware with no
    /// host traffic ends in the same state through repeated quanta as
    /// through one [`Machine::run_cycles`](crate::machine::Machine::run_cycles)
    /// call, and reruns are byte-identical.
    pub fn run_quantum(&mut self, max_cycles: u64, watch: &[(u8, u8)]) -> QuantumOutcome {
        let start_cyccnt = self.core.cyccnt;
        let mut last: Vec<Option<(bool, bool)>> = watch
            .iter()
            .map(|(port, pin)| {
                self.gpio_pin(*port, *pin)
                    .map(|(mode, level)| (level, mode.is_driven()))
            })
            .collect();
        let mut events = Vec::new();
        let mut steps = 0u64;
        let mut halt = None;
        while self.core.cyccnt.wrapping_sub(start_cyccnt) < max_cycles {
            if let Err(cause) = self.step() {
                self.settle_fast();
                halt = Some(cause);
                break;
            }
            steps += 1;
            let stamp = self.core.cyccnt;
            for (i, (port, pin)) in watch.iter().enumerate() {
                if let Some((mode, level)) = self.gpio_pin(*port, *pin) {
                    let state = (level, mode.is_driven());
                    if last[i] != Some(state) {
                        last[i] = Some(state);
                        events.push(CoupledEvent::pin(stamp, *port, *pin, level, state.1));
                    }
                }
            }
            if let Some(mv) = self.apin_drain_mv() {
                events.push(CoupledEvent::analog(stamp, mv));
            }
        }
        if halt.is_none() {
            self.settle_fast();
        }
        let end_cyccnt = self.core.cyccnt;
        QuantumOutcome {
            events,
            halt,
            start_cyccnt,
            end_cyccnt,
            steps,
        }
    }

    /// [`Self::run_quantum`] with a SPICE-seconds budget at the live CPU
    /// clock. `None` on a bad budget (non-finite or negative — the C ABI
    /// maps this to `VIOARM_ERR_BAD_ARG`); `Some` with zero steps on
    /// `0.0`, matching [`Self::run_quantum`] with `0` cycles.
    pub fn run_quantum_seconds(
        &mut self,
        seconds: f64,
        watch: &[(u8, u8)],
    ) -> Option<QuantumOutcome> {
        if !seconds.is_finite() || seconds < 0.0 {
            return None;
        }
        let hz = self.cpu_hz();
        Some(self.run_quantum(seconds_to_cycles(seconds, hz), watch))
    }
}

#[cfg(test)]
mod tests {
    use super::{cyccnt_to_seconds, seconds_to_cycles, CoupledQueue};

    #[test]
    fn time_mapping_saturates_and_converts() {
        assert_eq!(seconds_to_cycles(0.001, 8_000_000), 8000);
        assert_eq!(seconds_to_cycles(0.0, 8_000_000), 0);
        assert_eq!(seconds_to_cycles(-1.0, 8_000_000), 0);
        assert_eq!(seconds_to_cycles(f64::NAN, 8_000_000), 0);
        assert_eq!(seconds_to_cycles(0.001, 0), 0);
        assert!((cyccnt_to_seconds(8000, 8_000_000) - 0.001).abs() < 1e-12);
        assert_eq!(cyccnt_to_seconds(8000, 0), 0.0);
    }

    #[test]
    fn queue_refuses_past_capacity_in_order() {
        let mut q = CoupledQueue::with_capacity(1);
        assert!(q.is_empty());
        assert!(q.push(super::CoupledEvent::analog(1, 100)));
        assert!(!q.push(super::CoupledEvent::analog(2, 200)));
        assert_eq!(q.len(), 1);
        let drained = q.drain();
        assert_eq!(drained.len(), 1);
        assert_eq!(drained[0].cyccnt, 1);
        assert!(q.is_empty());
    }
}

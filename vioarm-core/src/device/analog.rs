//! Virtual analog pin endpoint: the millivolt transport behind N3's analog
//! lane, and the seam a future SPICE-class engine plugs into.
//!
//! Explicitly NOT analog physics: no noise, settling, impedance, or
//! temperature — values cross the lane as exact millivolt `u32`s (no FPU
//! needed on either side), single-driver-per-net by convention. The
//! register layout is documented-virtual (no silicon equivalent):
//! `TXMV@0x00` stages millivolts, any lane-0 write to `TXGO@0x04`
//! transmits the staged value to the host channel (drained by the
//! [`Fabric`](crate::fabric::Fabric) analog bus into peer `RXMV`
//! registers), `RXMV@0x08` reads the last received value and clears
//! `READY`, `SR@0x0C` carries `READY@0`, `CR@0x10` carries `EN@0` and
//! `LOOPBACK@1`. With `LOOPBACK` set, `TXGO` copies into the own `RXMV`
//! instead (also how a lone machine self-tests with no peer attached).
//! Traffic requires `EN`; while disabled, strobes are dropped, driven
//! bytes are dropped, and `READY` reads clear.
//!
//! Quanta discipline (shared with every fabric wire): delivery is
//! immediate within the pumping round, in machine-index order; a net
//! holds one value, so concurrent drivers resolve lowest-index-wins
//! (documented simplification, same spirit as the radio bus). Partial
//! trailing bytes (not a whole `u32`) are held for the next round.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Staged millivolts.
const TXMV: u32 = 0x00;
/// Transmit strobe (command, lane 0 only).
const TXGO: u32 = 0x04;
/// Last received millivolts (read clears `READY`).
const RXMV: u32 = 0x08;
/// Status register.
const SR: u32 = 0x0C;
/// Control register.
const CR: u32 = 0x10;
/// Window length.
const WINDOW_LEN: u32 = 0x14;

/// `SR` new-sample flag.
const READY: u8 = 0x01;
/// `CR` enable bit.
const EN: u32 = 1;
/// `CR` loopback bit.
const LOOPBACK: u32 = 1 << 1;

/// ADC reference in millivolts (documented-virtual): the full-scale rail
/// every consumer converts against. Matches the nominal supply rail.
pub const VDDA_MV: u32 = 3300;
/// ADC full-scale code (12-bit).
pub const ADC_MAX: u32 = 4095;

/// GPIO/EXTI Schmitt thresholds in millivolts (documented-virtual CMOS):
/// above `GPIO_VIH_MV` reads high, below `GPIO_VIL_MV` reads low, between
/// holds the previous level.
pub const GPIO_VIH_MV: u32 = 2000;
/// GPIO/EXTI Schmitt low threshold (see [`GPIO_VIH_MV`]).
pub const GPIO_VIL_MV: u32 = 800;

/// Comparator trip point in millivolts (documented-virtual `VDDA/2`).
pub const COMP_THRESH_MV: u32 = 1650;
/// Comparator hysteresis band in millivolts (output flips at
/// `THRESH ± HYST/2`, holds between).
pub const COMP_HYST_MV: u32 = 100;

/// Convert bus millivolts to a 12-bit ADC code against [`VDDA_MV`]:
/// saturates at full scale, truncates otherwise. The `u64` intermediate
/// keeps `u32::MAX` inputs overflow-safe.
#[must_use]
pub const fn mv_to_adc_code(mv: u32) -> u32 {
    if mv >= VDDA_MV {
        return ADC_MAX;
    }
    ((mv as u64 * ADC_MAX as u64) / VDDA_MV as u64) as u32
}

/// The digital level of `mv` millivolts through the GPIO/EXTI Schmitt
/// path: high above [`GPIO_VIH_MV`], low below [`GPIO_VIL_MV`], `prev`
/// between (hysteresis).
#[must_use]
pub const fn schmitt_level(mv: u32, prev: bool) -> bool {
    if mv >= GPIO_VIH_MV {
        true
    } else if mv <= GPIO_VIL_MV {
        false
    } else {
        prev
    }
}

/// The comparator output for `plus_mv` millivolts against the fixed
/// [`COMP_THRESH_MV`] with [`COMP_HYST_MV`] hysteresis: high above
/// `THRESH + HYST/2`, low below `THRESH - HYST/2`, `prev` between.
#[must_use]
pub const fn comp_level(plus_mv: u32, prev: bool) -> bool {
    let hi = COMP_THRESH_MV + COMP_HYST_MV / 2;
    let lo = COMP_THRESH_MV - COMP_HYST_MV / 2;
    if plus_mv >= hi {
        true
    } else if plus_mv <= lo {
        false
    } else {
        prev
    }
}

/// The shared analog bus (WP-R4): one host-driven voltage every consumer
/// on a machine reads coherently.
///
/// Explicitly NOT analog physics (same stance as [`Apin`]): one exact
/// millivolt `u32`, no noise/settling/impedance. Single-driver per net by
/// convention — the host (or the `apin` window it feeds) drives, firmware
/// observes — matching the fabric's lowest-index-wins discipline across
/// machines. [`Apin`] stays the transport endpoint; this is what
/// `Apin`-fed consumers (ADC, comparator, GPIO/EXTI, timer ext-clock)
/// sample.
///
/// The value starts unset (`None`): consumers fall back to their legacy
/// host-digital paths until the host first drives the bus, so existing
/// tests and boards without a bus wired behave exactly as before.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AnalogBusState {
    mv: Option<u32>,
}

impl AnalogBusState {
    /// An unset bus (no driver yet).
    #[must_use]
    pub const fn new() -> Self {
        Self { mv: None }
    }

    /// A shareable bus (the constructor pattern: boards/tests wire one,
    /// devices clone it — same as `VddState`/`ClockState`).
    #[must_use]
    pub fn shared() -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self::new()))
    }

    /// Drive the bus (host side). Never fails; poison is recovered.
    pub fn set_millivolts(bus: &Arc<Mutex<Self>>, mv: u32) {
        bus.lock().unwrap_or_else(|poison| poison.into_inner()).mv = Some(mv);
    }

    /// The driven value, if the host has driven yet.
    #[must_use]
    pub fn millivolts(bus: &Arc<Mutex<Self>>) -> Option<u32> {
        bus.lock().unwrap_or_else(|poison| poison.into_inner()).mv
    }

    /// Forget the driver (tests only): consumers revert to legacy paths.
    pub fn reset(bus: &Arc<Mutex<Self>>) {
        bus.lock().unwrap_or_else(|poison| poison.into_inner()).mv = None;
    }
}

/// A virtual analog pin: millivolt staging/transmit on one side, last-
/// value receive on the other, over the fabric's analog bus.
#[derive(Debug, Clone, Default)]
pub struct Apin {
    ranges: [Region; 1],
    enabled: bool,
    loopback: bool,
    tx_mv: u32,
    rx_mv: u32,
    rx_ready: bool,
    tx_out: Vec<u8>,
    pending: Vec<u8>,
    bus: Option<Arc<Mutex<AnalogBusState>>>,
}

impl Apin {
    /// A new pin window mapped at `base`.
    #[must_use]
    pub const fn at(base: u32) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            enabled: false,
            loopback: false,
            tx_mv: 0,
            rx_mv: 0,
            rx_ready: false,
            tx_out: Vec::new(),
            pending: Vec::new(),
            bus: None,
        }
    }

    /// Share the analog bus: received millivolts publish into it, so
    /// `Apin`-fed consumers (ADC/COMP/GPIO/EXTI/timer) read one node.
    /// The bus is host-owned and survives [`Peripheral::reset`] (like the
    /// supply rail survives a PWR reset).
    #[must_use]
    pub fn with_bus(mut self, bus: Arc<Mutex<AnalogBusState>>) -> Self {
        self.bus = Some(bus);
        self
    }

    /// Publish `mv` into the shared bus, if one is wired.
    fn publish(&self, mv: u32) {
        if let Some(bus) = self.bus.as_ref() {
            AnalogBusState::set_millivolts(bus, mv);
        }
    }

    /// Transmit the staged value: into the own receiver under loopback,
    /// otherwise out to the host channel (the fabric's analog bus).
    fn transmit(&mut self) {
        if !self.enabled {
            return;
        }
        if self.loopback {
            self.rx_mv = self.tx_mv;
            self.rx_ready = true;
            self.publish(self.tx_mv);
        } else {
            self.tx_out.extend_from_slice(&self.tx_mv.to_le_bytes());
        }
    }

    /// A stored word with byte `val` merged into lane `shift`.
    fn merged_word(current: u32, val: u8, shift: u32) -> u32 {
        let mask = 0xFFu32 << shift;
        (current & !mask) | (u32::from(val) << shift)
    }
}

impl Peripheral for Apin {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let lane = off & 3;
        let word = off & !3;
        let value = match word {
            TXMV => self.tx_mv,
            TXGO => 0,
            RXMV => {
                let value = if self.enabled { self.rx_mv } else { 0 };
                self.rx_ready = false;
                value
            }
            SR => u32::from(self.enabled && self.rx_ready),
            CR => u32::from(self.enabled) | if self.loopback { LOOPBACK } else { 0 },
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((value >> (8 * lane)) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let lane = off & 3;
        let shift = 8 * lane;
        match off & !3 {
            TXMV => self.tx_mv = Self::merged_word(self.tx_mv, val, shift),
            // Lane 0 strobes (a word store strobes once, like `UartTx` DR).
            TXGO => {
                if lane == 0 {
                    self.transmit();
                }
            }
            // `RXMV`/`SR` are read-only; writes are consumed silently.
            RXMV | SR => {}
            CR => {
                let word = Self::merged_word(
                    u32::from(self.enabled) | if self.loopback { LOOPBACK } else { 0 },
                    val,
                    shift,
                );
                self.enabled = word & EN != 0;
                self.loopback = word & LOOPBACK != 0;
            }
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.enabled = false;
        self.loopback = false;
        self.tx_mv = 0;
        self.rx_mv = 0;
        self.rx_ready = false;
        self.tx_out.clear();
        self.pending.clear();
        // The shared bus survives: it is host-owned, like the supply rail.
    }

    fn name(&self) -> &'static str {
        "apin"
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }

    fn drive_inputs(&mut self, value: u32) {
        if !self.enabled {
            return;
        }
        // Reassemble little-endian millivolt words; a trailing partial
        // word waits for the next round.
        self.pending.push((value & 0xFF) as u8);
        while self.pending.len() >= 4 {
            let word = u32::from_le_bytes([
                self.pending[0],
                self.pending[1],
                self.pending[2],
                self.pending[3],
            ]);
            self.pending.drain(..4);
            self.rx_mv = word;
            self.rx_ready = true;
            self.publish(word);
        }
    }
}

impl InterruptSource for Apin {}

#[cfg(test)]
mod tests {
    use super::{
        comp_level, mv_to_adc_code, schmitt_level, AnalogBusState, Apin, CR, EN, LOOPBACK, READY,
        RXMV, SR, TXGO, TXMV,
    };
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4000_2000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(p: &mut Apin, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(p: &mut Apin, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = p.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn loopback_copies_staged_mv_and_sets_ready() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN | LOOPBACK);
        write_word(&mut p, TXMV, 1200);
        write_word(&mut p, TXGO, 1);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), u32::from(READY));
        assert_eq!(read_word(&mut p, RXMV), 1200);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
    }

    #[test]
    fn disabled_window_drops_traffic() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, TXMV, 1200);
        write_word(&mut p, TXGO, 1);
        assert!(p.drain_output().is_empty());
        p.drive_inputs(0xD2);
        p.drive_inputs(0x04);
        p.drive_inputs(0x00);
        p.drive_inputs(0x00);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
    }

    #[test]
    fn host_channel_round_trips_one_word() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN);
        write_word(&mut p, TXMV, 2400);
        write_word(&mut p, TXGO, 1);
        assert_eq!(p.drain_output(), vec![0x60, 0x09, 0x00, 0x00]);
        for b in [0x10u8, 0x27, 0x00, 0x00] {
            p.drive_inputs(u32::from(b));
        }
        assert_eq!(read_word(&mut p, RXMV), 10000);
    }

    #[test]
    fn partial_words_wait_for_the_next_round() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN);
        p.drive_inputs(0x01);
        p.drive_inputs(0x02);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
        p.drive_inputs(0x03);
        p.drive_inputs(0x04);
        assert_eq!(read_word(&mut p, RXMV), 0x0403_0201);
    }

    #[test]
    fn outside_the_window_is_reserved() {
        let mut p = Apin::at(BASE);
        assert!(matches!(p.read(BASE + 0x14), Err(BusError::Reserved)));
        assert!(matches!(p.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn mv_to_code_saturates_and_scales() {
        assert_eq!(mv_to_adc_code(0), 0);
        assert_eq!(mv_to_adc_code(1650), 2047);
        assert_eq!(mv_to_adc_code(3300), 4095);
        assert_eq!(mv_to_adc_code(5000), 4095);
        assert_eq!(mv_to_adc_code(u32::MAX), 4095);
    }

    #[test]
    fn schmitt_holds_between_thresholds() {
        assert!(schmitt_level(2500, false));
        assert!(!schmitt_level(500, true));
        assert!(schmitt_level(1200, true));
        assert!(!schmitt_level(1200, false));
    }

    #[test]
    fn comp_holds_between_thresholds() {
        assert!(comp_level(1800, false));
        assert!(!comp_level(1500, true));
        assert!(comp_level(1650, true));
        assert!(!comp_level(1650, false));
    }

    #[test]
    fn apin_receipt_publishes_into_the_bus() {
        let bus = AnalogBusState::shared();
        let mut p = Apin::at(BASE).with_bus(bus.clone());
        write_word(&mut p, CR, EN);
        assert_eq!(AnalogBusState::millivolts(&bus), None);
        for b in 1200u32.to_le_bytes() {
            p.drive_inputs(u32::from(b));
        }
        assert_eq!(AnalogBusState::millivolts(&bus), Some(1200));
        // Device reset keeps the host-owned bus (like the supply rail).
        p.reset();
        assert_eq!(AnalogBusState::millivolts(&bus), Some(1200));
        AnalogBusState::reset(&bus);
        assert_eq!(AnalogBusState::millivolts(&bus), None);
    }
}

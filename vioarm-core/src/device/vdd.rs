//! The supply-rail state shared by every device on a machine.
//!
//! Explicitly NOT analog physics: no noise, ramp rates, decoupling, or
//! temperature — the rail is one exact millivolt `u32` (no FPU needed on
//! either side), driven by the host or the SPICE `vcc_bridge`, observed
//! by power-aware models. This mirrors [`ClockState`](crate::device::stm32::rcc::ClockState):
//! a single shared source of truth every consumer clones.
//!
//! Brown-out (BOR) is evaluated here with hysteresis: the rail asserts
//! below [`BOR_FALL_MV`] and releases above [`BOR_RISE_MV`]. The
//! thresholds are documented-virtual (silicon parks BOR in option bytes
//! with no register image); the PVD ladder in `Pwr` instead uses the
//! real `PLS` datasheet values.

use std::sync::{Arc, Mutex};

/// Power-on default rail: 3.3 V nominal.
pub const VDD_DEFAULT_MV: u32 = 3300;
/// BOR assert level (documented-virtual): the rail is failing below this.
pub const BOR_FALL_MV: u32 = 2000;
/// BOR release level (documented-virtual): hysteresis against `BOR_FALL_MV`.
pub const BOR_RISE_MV: u32 = 2100;

/// The supply rail: one millivolt word plus brown-out state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VddState {
    vdd_mv: u32,
    bor_asserted: bool,
}

impl VddState {
    /// A rail at nominal voltage, brown-out released.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vdd_mv: VDD_DEFAULT_MV,
            bor_asserted: false,
        }
    }

    /// A shareable rail (the constructor pattern: every board wires one,
    /// devices clone it).
    #[must_use]
    pub fn shared() -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self::new()))
    }

    /// The rail in millivolts.
    #[must_use]
    pub const fn millivolts(&self) -> u32 {
        self.vdd_mv
    }

    /// Whether brown-out is currently asserted (sticky across the
    /// hysteresis band).
    #[must_use]
    pub const fn bor_asserted(&self) -> bool {
        self.bor_asserted
    }

    /// Drive the rail; returns the brown-out state after applying
    /// hysteresis (asserts below [`BOR_FALL_MV`], releases above
    /// [`BOR_RISE_MV`], holds between).
    pub fn set_millivolts(&mut self, mv: u32) -> bool {
        self.vdd_mv = mv;
        if mv < BOR_FALL_MV {
            self.bor_asserted = true;
        } else if mv > BOR_RISE_MV {
            self.bor_asserted = false;
        }
        self.bor_asserted
    }
}

impl Default for VddState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::{VddState, BOR_FALL_MV, BOR_RISE_MV};

    #[test]
    fn hysteresis_holds_between_thresholds() {
        let mut rail = VddState::new();
        assert!(!rail.bor_asserted());
        assert!(rail.set_millivolts(BOR_FALL_MV - 1));
        assert!(rail.bor_asserted());
        // Inside the band: still asserted.
        assert!(rail.set_millivolts((BOR_FALL_MV + BOR_RISE_MV) / 2));
        // Above release: clear, and stays clear back inside the band.
        assert!(!rail.set_millivolts(BOR_RISE_MV + 1));
        assert!(!rail.set_millivolts((BOR_FALL_MV + BOR_RISE_MV) / 2));
        assert_eq!(rail.millivolts(), (BOR_FALL_MV + BOR_RISE_MV) / 2);
    }
}

//! Comparator (`COMP1`/`COMP2`) at `0x4001001C` (F0) and `0x58003800` (H743 COMP1/2).
//! The SVD describes `CSR` with `COMPxEN` and status `COMPxOUT`; the tables cannot express the
//! analog core, so `CompBehavior` synthesizes `COMPxOUT` one tick after `COMPxEN` is set.
//! `COMPxOUT` is read-only and reflects the comparator output (driven high when enabled for determinism).
//! H7 `COMP1` uses `CFGR1@0xC EN@0 / CFGR2@0x10 EN@0` with `SR C1VAL@0/C2VAL@1 C1IF@16/C2IF@17` and `ICFR CCxIF@16`.
//!
//! With a shared analog bus wired (WP-R4), the output instead derives from
//! the bus millivolts against the fixed `VDDA/2` threshold with hysteresis
//! (`analog::COMP_THRESH_MV/HYST_MV`): the bus pair is (signal, threshold),
//! and the band holds the previous level. Unset bus keeps the legacy
//! enabled-means-high path.

use crate::device::analog::{comp_level, AnalogBusState};
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const CSR_OUT1: u32 = 1 << 14;
const CSR_OUT2: u32 = 1 << 30; // F0 uses 14, H7 uses 30; we set both for coverage
const CSR_EN1: u32 = 1 << 0;
const CSR_EN2: u32 = 1 << 16;
const H7_SR_C1VAL: u32 = 1 << 0;
const H7_SR_C2VAL: u32 = 1 << 1;
const H7_SR_C1IF: u32 = 1 << 16;
const H7_SR_C2IF: u32 = 1 << 17;
const H7_CFGR_EN: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct CompBehavior {
    pending_out1: bool,
    pending_out2: bool,
    is_h7: bool,
    /// Shared analog bus (WP-R4): when wired and driven, `tick` derives
    /// the outputs from it instead of latching high on enable.
    analog_bus: Option<Arc<Mutex<AnalogBusState>>>,
    /// Previous bus-derived outputs (hysteresis holds between thresholds).
    out1: bool,
    out2: bool,
}

impl CompBehavior {
    /// Derive both outputs from `mv` millivolts (same node feeds both
    /// comparators by convention; the pair is signal-vs-threshold).
    fn tick_bus(&mut self, view: &mut Registers<&mut [u32]>, mv: u32) {
        if self.is_h7 {
            let c1en = view.get(0x0C) & H7_CFGR_EN != 0;
            let c2en = view.get(0x10) & H7_CFGR_EN != 0;
            let n1 = if c1en {
                comp_level(mv, self.out1)
            } else {
                false
            };
            let n2 = if c2en {
                comp_level(mv, self.out2)
            } else {
                false
            };
            let mut sr = view.get(0x00);
            // VAL follows the comparison; IF latches on a rising output.
            if n1 {
                sr |= H7_SR_C1VAL;
                if !self.out1 && c1en {
                    sr |= H7_SR_C1IF;
                }
            } else {
                sr &= !H7_SR_C1VAL;
            }
            if n2 {
                sr |= H7_SR_C2VAL;
                if !self.out2 && c2en {
                    sr |= H7_SR_C2IF;
                }
            } else {
                sr &= !H7_SR_C2VAL;
            }
            if !c1en {
                sr &= !(H7_SR_C1VAL | H7_SR_C1IF);
            }
            if !c2en {
                sr &= !(H7_SR_C2VAL | H7_SR_C2IF);
            }
            view.set(0x00, sr);
            self.out1 = n1;
            self.out2 = n2;
            return;
        }
        let en1 = view.get(0x00) & CSR_EN1 != 0;
        let en2 = view.get(0x00) & CSR_EN2 != 0;
        let n1 = if en1 {
            comp_level(mv, self.out1)
        } else {
            false
        };
        let n2 = if en2 {
            comp_level(mv, self.out2)
        } else {
            false
        };
        let mut csr = view.get(0x00) & !(CSR_OUT1 | CSR_OUT2);
        if n1 {
            csr |= CSR_OUT1 | CSR_OUT2;
        }
        if n2 {
            csr |= CSR_OUT2;
        }
        view.set(0x00, csr);
        self.out1 = n1;
        self.out2 = n2;
    }
}

impl Behavior for CompBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // F0 CSR at 0x00: latch EN, schedule OUT to be set next tick
        if offset == 0x00 {
            if self.is_h7 {
                // H7 SR is read-only (C1VAL/C1IF) — ignore writes
                return view.get(0x00);
            }
            let en1 = word & CSR_EN1 != 0;
            let en2 = word & CSR_EN2 != 0;
            // Clear OUT immediately on disable, schedule set on enable
            let cur = word & !(CSR_OUT1 | CSR_OUT2);
            if !en1 {
                self.pending_out1 = false;
            } else if view.get(0x00) & CSR_EN1 == 0 {
                // rising EN1
                self.pending_out1 = true;
            }
            if !en2 {
                self.pending_out2 = false;
            } else if view.get(0x00) & CSR_EN2 == 0 {
                self.pending_out2 = true;
            }
            // Keep OUT cleared until tick
            return cur;
        }
        if offset == 0x0C || offset == 0x10 {
            // H7 CFGR1/2 EN@0 → schedule SR C1VAL/C1IF or C2VAL/C2IF
            let is_c1 = offset == 0x0C;
            let en = word & H7_CFGR_EN != 0;
            let prev_en = view.get(offset) & H7_CFGR_EN != 0;
            if en && !prev_en {
                if is_c1 {
                    self.pending_out1 = true;
                } else {
                    self.pending_out2 = true;
                }
            }
            if !en {
                // Clear SR VAL/IF on disable
                let sr = view.get(0x00);
                if is_c1 {
                    view.set(0x00, sr & !(H7_SR_C1VAL | H7_SR_C1IF));
                    self.pending_out1 = false;
                } else {
                    view.set(0x00, sr & !(H7_SR_C2VAL | H7_SR_C2IF));
                    self.pending_out2 = false;
                }
            }
            return word;
        }
        if offset == 0x04 {
            // ICFR CC1IF@16 / CC2IF@17 W1C clears SR C1IF/C2IF
            if word & (1 << 16) != 0 {
                view.set(0x00, view.get(0x00) & !H7_SR_C1IF);
            }
            if word & (1 << 17) != 0 {
                view.set(0x00, view.get(0x00) & !H7_SR_C2IF);
            }
            return 0;
        }
        word
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, _offset: u32, value: u32) -> u32 {
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Shared-bus path: derive the outputs from the bus millivolts
        // (signal vs fixed threshold, hysteresis holds between).
        if let Some(bus) = self.analog_bus.clone() {
            if let Some(mv) = AnalogBusState::millivolts(&bus) {
                self.tick_bus(view, mv);
                // The bus owns the outputs now; legacy pendings stay clear.
                self.pending_out1 = false;
                self.pending_out2 = false;
                return;
            }
        }
        if self.pending_out1 {
            let cur = view.get(0x00);
            if self.is_h7 {
                view.set(0x00, cur | H7_SR_C1VAL | H7_SR_C1IF);
            } else {
                view.set(0x00, cur | CSR_OUT1 | CSR_OUT2);
            }
            self.pending_out1 = false;
        }
        if self.pending_out2 {
            let cur = view.get(0x00);
            if self.is_h7 {
                view.set(0x00, cur | H7_SR_C2VAL | H7_SR_C2IF);
            } else {
                view.set(0x00, cur | CSR_OUT2);
            }
            self.pending_out2 = false;
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        if !self.is_h7 {
            return 0;
        }
        let sr = view.get(0x00);
        if sr & (H7_SR_C1IF | H7_SR_C2IF) != 0 {
            return 1;
        }
        0
    }

    fn reset(&mut self) {
        self.pending_out1 = false;
        self.pending_out2 = false;
        self.out1 = false;
        self.out2 = false;
        // The shared bus survives: it is host-owned.
    }
}

/// Comparator.
#[derive(Debug)]
pub struct Comp {
    inner: RegisterFile<CompBehavior>,
    name: &'static str,
}

impl Comp {
    /// A new `Comp` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("COMP")
            .or_else(|| D::peripheral("COMP1"))
            .expect("COMP/comp peripheral");
        let is_h7 = def.name == "COMP1";
        let beh = CompBehavior {
            is_h7,
            ..CompBehavior::default()
        };
        Self {
            inner: RegisterFile::new(def, beh),
            name: "comp",
        }
    }
    /// A new `COMP` on another chip's device tables.
    #[must_use]
    pub fn comp_for<D: Device>() -> Self {
        Self::new_for::<D>()
    }

    /// Share the analog bus: `tick` derives the outputs from its
    /// millivolts (WP-R4). Unset bus keeps the legacy enable-latches-high
    /// path.
    #[must_use]
    pub fn with_analog_bus(mut self, bus: Arc<Mutex<AnalogBusState>>) -> Self {
        self.inner.behavior_mut().analog_bus = Some(bus);
        self
    }
}

impl Peripheral for Comp {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
    fn name(&self) -> &'static str {
        self.name
    }
}
impl InterruptSource for Comp {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f042::Stm32f042;
    fn write_word(c: &mut Comp, off: u32, val: u32) {
        let base = c.ranges()[0].base;
        for i in 0..4 {
            c.write(base + off + i, ((val >> (8 * i)) & 0xFF) as u8)
                .unwrap();
        }
    }
    fn read_word(c: &mut Comp, off: u32) -> u32 {
        let base = c.ranges()[0].base;
        let mut w = 0;
        for i in 0..4 {
            w |= u32::from(c.read(base + off + i).unwrap()) << (8 * i);
        }
        w
    }
    #[test]
    fn comp_enable_sets_out_after_one_tick() {
        let mut c = Comp::new_for::<Stm32f042>();
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0);
        write_word(&mut c, 0x00, CSR_EN1);
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0, "OUT not yet");
        c.tick(1);
        assert_ne!(
            read_word(&mut c, 0x00) & CSR_OUT1,
            0,
            "OUT latches after 1c"
        );
        // Disable clears
        write_word(&mut c, 0x00, 0);
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0);
    }

    #[test]
    fn bus_derives_output_with_hysteresis() {
        let bus = AnalogBusState::shared();
        let mut c = Comp::new_for::<Stm32f042>().with_analog_bus(bus.clone());
        write_word(&mut c, 0x00, CSR_EN1);
        AnalogBusState::set_millivolts(&bus, 1800);
        c.tick(1);
        assert_ne!(read_word(&mut c, 0x00) & CSR_OUT1, 0, "above threshold");
        // Mid-band holds the previous high ...
        AnalogBusState::set_millivolts(&bus, 1650);
        c.tick(1);
        assert_ne!(
            read_word(&mut c, 0x00) & CSR_OUT1,
            0,
            "hysteresis holds high"
        );
        AnalogBusState::set_millivolts(&bus, 1500);
        c.tick(1);
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0, "below threshold");
        // ... and the previous low.
        AnalogBusState::set_millivolts(&bus, 1650);
        c.tick(1);
        assert_eq!(
            read_word(&mut c, 0x00) & CSR_OUT1,
            0,
            "hysteresis holds low"
        );
    }
}

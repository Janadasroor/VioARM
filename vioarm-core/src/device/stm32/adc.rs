//! The STM32 12-bit successive-approximation ADC (`ADC1`/`ADC2`/`ADC3`),
//! driven by the SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! sampling core, so [`AdcBehavior`] runs it: `CR2.ADON` powers the converter
//! on, a software start (`CR2.SWSTART`, or — on the F1 — a second `ADON`
//! write) begins a conversion of the first channel in the `SQR` sequence, and
//! after a fixed [`CONVERSION_CYCLES`] the sample lands in `DR` with
//! `SR.EOC`/`SR.STRT` set. `CR1.SCAN` walks the whole sequence and `CR2.CONT`
//! re-arms continuously; `CR1.EOCIE` (and the F4's `OVR`/`OVRIE` pair) raises
//! the ADC interrupt line. Reading `DR` clears `EOC`. The analog level of
//! every channel comes from the host through [`Peripheral::drive_inputs`],
//! truncated to the 12-bit result.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, PeripheralDef, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the ADC, per the SVD.
const SR: u32 = 0x00;
const CR1: u32 = 0x04;
const CR2: u32 = 0x08;
const SQR1: u32 = 0x2C;
const SQR2: u32 = 0x30;
const SQR3: u32 = 0x34;
const JSQR: u32 = 0x38;
const JDR1: u32 = 0x3C;
/// `HTR`/`LTR` — the analog-watchdog high/low thresholds.
const HTR: u32 = 0x24;
const LTR: u32 = 0x28;
const DR: u32 = 0x4C;

/// Test-visible aliases for the injected-group offsets.
#[cfg(test)]
pub(crate) const JSQR_OFFSET: u32 = JSQR;
#[cfg(test)]
pub(crate) const JDR1_OFFSET: u32 = JDR1;
#[cfg(test)]
pub(crate) const HTR_OFFSET: u32 = HTR;
#[cfg(test)]
pub(crate) const LTR_OFFSET: u32 = LTR;
/// `SQR3` for the tests (first six regular sequence slots).
const SQR3_OFF: u32 = SQR3;
#[cfg(test)]
pub(crate) const SR_TEST: u32 = SR;

/// `SR` flag bits.
const EOC: u32 = 1 << 1;
const JEOC: u32 = 1 << 2;
const JSTRT: u32 = 1 << 3;
const STRT: u32 = 1 << 4;
const OVR: u32 = 1 << 5; // F4 only
/// `SR.AWD` (bit 0): an analog-watchdog event latched.
const AWD: u32 = 1 << 0;

/// `CR1` bits.
const EOCIE: u32 = 1 << 5;
const JEOCIE: u32 = 1 << 7;
const SCAN: u32 = 1 << 8;
const JAUTO: u32 = 1 << 10;
const OVRIE: u32 = 1 << 26; // F4 only
/// `CR1.AWDEN` (bit 23): the analog watchdog is enabled.
const AWDEN: u32 = 1 << 23;
/// `CR1.AWDSGL` (bit 9): watch only the channel selected by `AWDCH`.
const AWDSGL: u32 = 1 << 9;
/// `CR1.AWDIE` (bit 6): the watchdog interrupt enable.
const AWDIE: u32 = 1 << 6;
/// `CR1.AWDCH[4:0]`: the watched channel when `AWDSGL` is set.
const AWDCH_MASK: u32 = 0x1F;

/// `CR2` bits.
const ADON: u32 = 1 << 0;
const CONT: u32 = 1 << 1;
const ALIGN: u32 = 1 << 11;

/// The number of machine cycles a conversion takes. Real conversion time is a
/// function of the ADC clock and the per-channel sampling time; the model
/// fixes it so the result is deterministic while the EOC handshake stays
/// faithful.
const CONVERSION_CYCLES: u32 = 14;

/// An in-progress conversion.
#[derive(Clone, Copy, Debug)]
struct Conversion {
    channel: u8,
    remaining: u64,
}

/// Chip differences the SVD tables express as field layouts.
#[derive(Clone, Copy, Debug)]
struct AdcConfig {
    /// The `CR2.SWSTART` / `CR.ADSTART` bit position (22 on the F1, 30 on the F4, 2 on H7/F0).
    swstart: u32,
    /// Whether `SR.OVR`/`CR1.OVRIE` exist (the F4 overrun pair).
    has_ovr: bool,
    /// Whether the status register declares `ADRDY` (H7/F0 ISR) — bit 0 there
    /// is not the F1 `AWD`.
    has_adrdy: bool,
    /// Whether a second `ADON` write restarts a conversion (F1 legacy).
    adon_restarts: bool,
    /// Whether the sequence is `CHSELR` (F0) instead of `SQRx` (F1/F4/H7).
    has_chselr: bool,
    cr_off: u32,
    sr_off: u32,
    sqr1_off: u32,
    chselr_off: u32,
    dr_off: u32,
    eoc_bit: u32,
    adon_bit: u32,
}

/// The sampling core the SVD cannot express.
#[derive(Clone, Debug)]
struct AdcBehavior {
    analog: [u16; 18],
    converting: Option<Conversion>,
    sq_index: u32,
    /// In-progress injected conversion and its index into `JSQR`.
    jconverting: Option<Conversion>,
    jindex: u32,
    config: AdcConfig,
    dma: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
    common: Option<Arc<Mutex<AdcCommonState>>>,
    adc_index: u8,
}

/// Shared `C_ADC` / `ADC12_Common` state for dual-mode (R2).
#[derive(Clone, Debug, Default)]
pub struct AdcCommonState {
    /// F4 `CCR.MULT` / H7 `CCR.DUAL` (bits 0..4).
    pub dual: u32,
    /// `CCR.DMA` (F4 bits 14..15) / `CCR.DAMDF` (H7).
    pub ccr_dma: u32,
    pub data1: u16,
    pub data2: u16,
    pub eoc1: bool,
    pub eoc2: bool,
    /// Master fired SWSTART while `dual != 0`; slave should start next tick.
    pub dual_pending: bool,
}

impl AdcBehavior {
    fn new(config: AdcConfig) -> Self {
        Self {
            analog: [0; 18],
            converting: None,
            sq_index: 0,
            jconverting: None,
            jindex: 0,
            config,
            dma: None,
            common: None,
            adc_index: 1,
        }
    }

    /// The number of conversions in the regular sequence (`SQR1.L + 1` or `CHSELR` popcount).
    fn sequence_len(&self, view: &Registers<&mut [u32]>) -> u32 {
        if self.config.has_chselr {
            view.get(self.config.chselr_off).count_ones().max(1)
        } else {
            ((view.get(self.config.sqr1_off) >> 20) & 0x0F) + 1
        }
    }

    /// The channel at `index` in the regular sequence (`SQR3` SQ1..SQ6,
    /// `SQR2` SQ7..SQ12, `SQR1` SQ13..SQ16, or `CHSELR` bitset on F0).
    fn channel_at(&self, view: &Registers<&mut [u32]>, index: u32) -> u8 {
        if self.config.has_chselr {
            let bits = view.get(self.config.chselr_off);
            let mut seen = 0;
            for ch in 0..32 {
                if bits & (1 << ch) != 0 {
                    if seen == index {
                        return ch as u8;
                    }
                    seen += 1;
                }
            }
            return 0;
        }
        let sqr1 = self.config.sqr1_off;
        let (reg, shift) = match index {
            0..=5 => (sqr1 + 8, index * 5),        // SQR3 = SQR1+8
            6..=11 => (sqr1 + 4, (index - 6) * 5), // SQR2 = SQR1+4
            _ => (sqr1, (index - 12) * 5),
        };
        ((view.get(reg) >> shift) & 0x1F) as u8
    }

    /// The length of the injected sequence (`JSQR.JL`, 0 → 1 conversion).
    fn injected_len(&self, view: &Registers<&mut [u32]>) -> u32 {
        ((view.get(JSQR) >> 20) & 0x3) + 1
    }

    /// The channel at `index` in the injected sequence (`JSQR` JSQ1..JSQ4
    /// starting at bit 15 downward).
    fn injected_channel_at(&self, view: &Registers<&mut [u32]>, index: u32) -> u8 {
        let shift = 15 - index * 5;
        ((view.get(JSQR) >> shift) & 0x1F) as u8
    }

    /// Begin a conversion of the first channel in the sequence.
    fn start_sequence(&mut self, view: &Registers<&mut [u32]>) {
        if view.get(self.config.cr_off) & self.config.adon_bit == 0 {
            return;
        }
        self.sq_index = 0;
        self.converting = Some(Conversion {
            channel: self.channel_at(view, 0),
            remaining: u64::from(CONVERSION_CYCLES),
        });
    }

    /// Begin the injected sequence (`CR2.JSWSTART` strobe or `JAUTO`).
    fn start_injected(&mut self, view: &Registers<&mut [u32]>) {
        if view.get(self.config.cr_off) & self.config.adon_bit == 0 {
            return;
        }
        self.jindex = 0;
        self.jconverting = Some(Conversion {
            channel: self.injected_channel_at(view, 0),
            remaining: u64::from(CONVERSION_CYCLES),
        });
    }

    /// Land one regular result in `DR` (shared by the tick paths).
    fn land_regular(&mut self, view: &mut Registers<&mut [u32]>, channel: u8) {
        let value = self.sample(channel);
        let aligned = if view.get(self.config.cr_off) & ALIGN != 0 {
            value << 4
        } else {
            value
        };
        let sr = view.get(self.config.sr_off);
        let eoc = self.config.eoc_bit;
        if self.config.has_ovr && sr & eoc != 0 {
            view.set(self.config.sr_off, sr | eoc | STRT | OVR);
        } else {
            view.set(self.config.sr_off, sr | eoc | STRT);
        }
        view.set(self.config.dr_off, aligned);
        // Dual-mode bookkeeping: mirror result into common CDR staging (R2).
        if let Some(common) = self.common.as_ref() {
            if let Ok(mut c) = common.lock() {
                match self.adc_index {
                    1 => {
                        c.data1 = value as u16;
                        c.eoc1 = true;
                    }
                    2 => {
                        c.data2 = value as u16;
                        c.eoc2 = true;
                    }
                    _ => {}
                }
                // Clear dual_pending once slave has landed.
                if self.adc_index == 2 && c.dual_pending {
                    c.dual_pending = false;
                }
            }
        }
        // The analog watchdog guards the regular group (RM0008 §11.7.4):
        // enabled channels outside [LTR, HTR] latch SR.AWD.
        self.run_watchdog(view, channel, value);
        let dma_enabled = if self.config.has_chselr {
            // F0 `CFGR.DMAEN` at bit0 (0x0C), not `CR` bit8
            view.get(0x0C) & 1 != 0
        } else {
            view.get(self.config.cr_off) & (1 << 8) != 0
        };
        if dma_enabled {
            if let Some((reqs, ctrl, ch)) = self.dma.as_ref() {
                let reqs = Arc::clone(reqs);
                reqs.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .request(*ctrl, *ch, false);
            }
        }
    }

    fn sample(&self, channel: u8) -> u32 {
        u32::from(self.analog.get(usize::from(channel)).copied().unwrap_or(0) & 0x0FFF)
    }

    /// Evaluate the analog watchdog for a landed regular result: with
    /// `AWDEN` set, the watched channel (all channels when `AWDSGL` is
    /// clear) outside `[LTR, HTR]` latches `SR.AWD`.
    fn run_watchdog(&self, view: &mut Registers<&mut [u32]>, channel: u8, value: u32) {
        let cr1 = view.get(CR1);
        if cr1 & AWDEN == 0 {
            return;
        }
        if cr1 & AWDSGL != 0 && (cr1 & AWDCH_MASK) as u8 != channel {
            return;
        }
        let ht = view.get(HTR) & 0x0FFF;
        let lt = view.get(LTR) & 0x0FFF;
        if value > ht || value < lt {
            view.set(self.config.sr_off, view.get(self.config.sr_off) | AWD);
        }
    }
}

impl Behavior for AdcBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == self.config.cr_off {
            let before = view.get(self.config.cr_off);
            let word = view.merged(self.config.cr_off, val, shift);
            let powered = word & self.config.adon_bit != 0;
            let was_powered = before & self.config.adon_bit != 0;
            let swstart = word & self.config.swstart != 0;
            // F1/F4: JSWSTART (CR2 bit 21) strobes the injected sequence.
            const JSWSTART: u32 = 1 << 21;
            let jswstart = word & JSWSTART != 0;
            // SWSTART/ADSTART is a self-clearing strobe: it reads back as zero.
            let stored = word & !self.config.swstart;
            // H7: ADEN rising sets ISR.ADRDY (bit 0 of ISR). On F1/F4 SR
            // bit 0 is AWD, so only tables that declare ADRDY get the latch.
            if powered && !was_powered && self.config.has_adrdy {
                let isr = view.get(self.config.sr_off);
                view.set(self.config.sr_off, isr | 1);
            }
            if powered {
                if swstart {
                    self.start_sequence(view);
                    // R2 dual simultaneous: master SWSTART also arms slave.
                    if self.adc_index == 1 {
                        if let Some(common) = self.common.as_ref() {
                            if let Ok(mut c) = common.lock() {
                                if c.dual != 0 {
                                    c.dual_pending = true;
                                    c.eoc1 = false;
                                    c.eoc2 = false;
                                }
                            }
                        }
                    } else if self.adc_index == 2 {
                        // Slave direct SWSTART clears pending.
                        if let Some(common) = self.common.as_ref() {
                            if let Ok(mut c) = common.lock() {
                                c.dual_pending = false;
                            }
                        }
                    }
                } else if jswstart {
                    self.start_injected(view);
                } else if self.config.adon_restarts
                    && was_powered
                    && word & CONT == 0
                    && self.converting.is_none()
                {
                    // F1 legacy: a second ADON write (while already powered,
                    // outside continuous mode) starts a conversion.
                    self.start_sequence(view);
                }
            } else {
                // Powering off aborts any in-flight conversion and clears ADRDY.
                view.set(self.config.sr_off, view.get(self.config.sr_off) & !1);
                self.converting = None;
            }
            return stored;
        }
        view.merged(offset, val, shift)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == self.config.dr_off {
            // Reading the data register clears EOC and STRT.
            view.set(
                self.config.sr_off,
                view.get(self.config.sr_off) & !(self.config.eoc_bit | STRT),
            );
        }
        if (JDR1..JDR1 + 16).contains(&offset) {
            // Reading an injected data register clears JEOC/JSTRT
            // (RM0008 §11.7.5).
            view.set(
                self.config.sr_off,
                view.get(self.config.sr_off) & !(JEOC | JSTRT),
            );
        }
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        // R2 dual simultaneous: slave auto-starts when master flagged pending.
        if self.adc_index == 2 && self.converting.is_none() {
            if let Some(common) = self.common.as_ref() {
                let pending = common
                    .lock()
                    .map(|c| c.dual_pending && c.dual != 0)
                    .unwrap_or(false);
                if pending && view.get(self.config.cr_off) & self.config.adon_bit != 0 {
                    self.start_sequence(view);
                }
            }
        }
        // The injected group outranks the regular one: it converts first.
        if let Some(mut jconv) = self.jconverting {
            if jconv.remaining > delta {
                jconv.remaining -= delta;
                self.jconverting = Some(jconv);
                return;
            }
            self.jconverting = None;
            let value = self.sample(jconv.channel);
            let aligned = if view.get(self.config.cr_off) & ALIGN != 0 {
                value << 4
            } else {
                value
            };
            view.set(JDR1 + self.jindex * 4, aligned);
            view.set(
                self.config.sr_off,
                view.get(self.config.sr_off) | JEOC | JSTRT,
            );
            let len = self.injected_len(view);
            let next = self.jindex + 1;
            if next < len {
                // Walk the rest of the injected sequence.
                self.jindex = next;
                self.jconverting = Some(Conversion {
                    channel: self.injected_channel_at(view, next),
                    remaining: u64::from(CONVERSION_CYCLES),
                });
            }
            return;
        }
        let Some(mut conv) = self.converting else {
            return;
        };
        if conv.remaining > delta {
            conv.remaining -= delta;
            self.converting = Some(conv);
            return;
        }
        // The sample completes: land it in DR and latch EOC/STRT.
        self.converting = None;
        self.land_regular(view, conv.channel);
        // F0 CHSELR: one ADSTART walks all selected channels sequentially.
        if self.config.has_chselr {
            let len = self.sequence_len(view);
            if self.sq_index + 1 < len {
                self.sq_index += 1;
                self.converting = Some(Conversion {
                    channel: self.channel_at(view, self.sq_index),
                    remaining: u64::from(CONVERSION_CYCLES),
                });
            } else {
                // End of sequence: set EOS (bit 3) for F0 ISR.
                view.set(self.config.sr_off, view.get(self.config.sr_off) | (1 << 3));
            }
            return;
        }
        // JAUTO: the regular sequence completing auto-starts an injected one.
        let continuous = view.get(self.config.cr_off) & CONT != 0;
        let scan = view.get(CR1) & SCAN != 0;
        let len = self.sequence_len(view);
        let at_end = !continuous && (!scan || self.sq_index + 1 >= len);
        if at_end && view.get(CR1) & JAUTO != 0 {
            self.start_injected(view);
            return;
        }
        // Continuous or scan mode re-arms the next conversion immediately.
        let next = if continuous && scan {
            (self.sq_index + 1) % len
        } else {
            self.sq_index
        };
        if continuous {
            self.sq_index = next;
            self.converting = Some(Conversion {
                channel: self.channel_at(view, next),
                remaining: u64::from(CONVERSION_CYCLES),
            });
        } else if scan && next + 1 < len {
            self.sq_index = next + 1;
            self.converting = Some(Conversion {
                channel: self.channel_at(view, self.sq_index),
                remaining: u64::from(CONVERSION_CYCLES),
            });
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(self.config.sr_off);
        let cr1 = view.get(CR1);
        // H7 uses IER at sr_off+4 with EOCIE at bit 2, while F1/F4 use CR1 at 0x04 with EOCIE at bit 5
        let ier = view.get(self.config.sr_off + 4);
        let eocie = (cr1 & EOCIE != 0) || (ier & (1 << 2) != 0);
        u32::from(
            (sr & self.config.eoc_bit != 0 && eocie)
                || (sr & JEOC != 0 && cr1 & JEOCIE != 0)
                || (sr & AWD != 0 && cr1 & AWDIE != 0)
                || (self.config.has_ovr && sr & OVR != 0 && cr1 & OVRIE != 0),
        )
    }

    fn reset(&mut self) {
        self.converting = None;
        self.sq_index = 0;
        self.jconverting = None;
        self.jindex = 0;
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        let level = (value & 0x0FFF) as u16;
        for a in &mut self.analog {
            *a = level;
        }
    }
}

/// The STM32 analog-to-digital converter.
#[derive(Debug)]
pub struct Adc {
    inner: RegisterFile<AdcBehavior>,
    name: &'static str,
}

impl Adc {
    /// The F103 `ADC1` at `0x4001_2400`: IRQ 18.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `ADC1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn adc1() -> Self {
        Self::adc1_for::<stm32f103::Stm32f103>()
    }

    /// An `ADC1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `ADC1` peripheral.
    #[must_use]
    pub fn adc1_for<D: Device>() -> Self {
        Self::for_device::<D>("ADC1", "adc1")
    }

    /// The F103 `ADC2` at `0x4001_2800`: shares IRQ 18 with `ADC1`.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `ADC2`
    /// peripheral.
    #[must_use]
    pub fn adc2() -> Self {
        Self::adc2_for::<stm32f103::Stm32f103>()
    }

    /// An `ADC2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `ADC2` peripheral.
    #[must_use]
    pub fn adc2_for<D: Device>() -> Self {
        Self::for_device::<D>("ADC2", "adc2")
    }

    /// The F103 `ADC3` at `0x4001_3C00`: IRQ 47.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `ADC3`
    /// peripheral.
    #[must_use]
    pub fn adc3() -> Self {
        Self::adc3_for::<stm32f103::Stm32f103>()
    }

    /// An `ADC3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `ADC3` peripheral.
    #[must_use]
    pub fn adc3_for<D: Device>() -> Self {
        Self::for_device::<D>("ADC3", "adc3")
    }
    /// Generic `ADC` (F0 `ADC@0x40012400`) — picks `ADC1` if present else `ADC`.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let per = if D::peripheral("ADC1").is_some() {
            "ADC1"
        } else if D::peripheral("ADC").is_some() {
            "ADC"
        } else {
            panic!("generated tables contain ADC/ADC1")
        };
        Self::for_device::<D>(per, "adc")
    }
    #[must_use]
    pub fn adc4_for<D: Device>() -> Self {
        Self::for_device::<D>("ADC4", "adc4")
    }
    fn for_device<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        let idx = match peripheral {
            "ADC1" => 1,
            "ADC2" => 2,
            "ADC3" => 3,
            _ => 1,
        };
        let mut beh = AdcBehavior::new(config_from(def));
        beh.adc_index = idx;
        Self {
            inner: RegisterFile::new(def, beh),
            name,
        }
    }

    /// Attach the dual-mode common state shared with `C_ADC` / `ADC12_Common` (R2).
    #[must_use]
    pub fn with_common(mut self, common: Arc<Mutex<AdcCommonState>>) -> Self {
        self.inner.behavior_mut().common = Some(common);
        self
    }

    /// Attach DMA peripheral->memory request line.
    #[must_use]
    pub fn with_dma(
        mut self,
        requests: Arc<Mutex<DmaRequests>>,
        controller: u8,
        channel: u8,
    ) -> Self {
        self.inner.behavior_mut().dma = Some((requests, controller, channel));
        self
    }
}

/// Common ADC (`C_ADC` on F4, `ADC12_Common`/`ADC3_Common` on H7) for R2 dual simultaneous.
#[derive(Debug)]
pub struct AdcCommon {
    inner: RegisterFile<AdcCommonBehavior>,
    name: &'static str,
}

#[derive(Clone, Debug)]
struct AdcCommonConfig {
    ccr_off: u32,
    csr_off: u32,
    cdr_off: u32,
    /// F4: `CCR.MULT` at 0, H7: `CCR.DUAL` at 0 — same encoding.
    dual_shift: u32,
    dual_width: u32,
}

#[derive(Clone, Debug)]
struct AdcCommonBehavior {
    state: Arc<Mutex<AdcCommonState>>,
    cfg: AdcCommonConfig,
}

impl Behavior for AdcCommonBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == self.cfg.ccr_off {
            let word = view.merged(self.cfg.ccr_off, val, shift);
            if let Ok(mut c) = self.state.lock() {
                let mask = if self.cfg.dual_width >= 32 {
                    u32::MAX
                } else {
                    ((1u32 << self.cfg.dual_width) - 1) << self.cfg.dual_shift
                };
                c.dual = (word & mask) >> self.cfg.dual_shift;
                // DMA bits are also latched for the CDR read path (not yet modeled).
                c.ccr_dma = (word >> 14) & 0x3;
            }
            return word;
        }
        view.merged(offset, val, shift)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == self.cfg.cdr_off {
            if let Ok(c) = self.state.lock() {
                let combined = (u32::from(c.data2) << 16) | u32::from(c.data1);
                view.set(self.cfg.cdr_off, combined);
                return combined;
            }
        }
        if offset == self.cfg.csr_off {
            // Synthesize CSR EOC bits from shared eoc1/eoc2 for F4's CSR layout
            // (EOC1@1, EOC2@9, EOC3@17). H7 uses a different CSR but the
            // behavioral bit positions still surface in the same register
            // offsets for the doctest harness.
            if let Ok(c) = self.state.lock() {
                let mut syn = value;
                if c.eoc1 {
                    syn |= 1 << 1;
                    syn |= 1 << 4; // STRT1
                }
                if c.eoc2 {
                    syn |= 1 << 9;
                    syn |= 1 << 12;
                }
                view.set(self.cfg.csr_off, syn);
                return syn;
            }
        }
        value
    }

    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {}
    fn reset(&mut self) {
        if let Ok(mut c) = self.state.lock() {
            c.dual = 0;
            c.ccr_dma = 0;
            c.data1 = 0;
            c.data2 = 0;
            c.eoc1 = false;
            c.eoc2 = false;
            c.dual_pending = false;
        }
    }
}

impl AdcCommon {
    /// F4 `C_ADC` shared by `ADC1`/`ADC2`/`ADC3`.
    #[must_use]
    pub fn c_adc_for<D: Device>(state: Arc<Mutex<AdcCommonState>>) -> Self {
        Self::for_device::<D>("C_ADC", "c_adc", state)
    }
    #[must_use]
    pub fn c_adc(state: Arc<Mutex<AdcCommonState>>) -> Self {
        Self::c_adc_for::<stm32f103::Stm32f103>(state)
    }
    /// F2/F767 `ADC_Common` shared by their ADCs.
    #[must_use]
    pub fn adc_common_for<D: Device>(state: Arc<Mutex<AdcCommonState>>) -> Self {
        Self::for_device::<D>("ADC_Common", "adc_common", state)
    }
    /// H7 `ADC12_Common` (also covers `ADC3_Common` layout).
    #[must_use]
    pub fn adc12_common_for<D: Device>(state: Arc<Mutex<AdcCommonState>>) -> Self {
        let name = if D::peripheral("ADC12_Common").is_some() {
            "ADC12_Common"
        } else {
            "ADC3_Common"
        };
        Self::for_device::<D>(name, "adc12_common", state)
    }
    #[must_use]
    pub fn adc3_common_for<D: Device>(state: Arc<Mutex<AdcCommonState>>) -> Self {
        Self::for_device::<D>("ADC3_Common", "adc3_common", state)
    }
    #[must_use]
    pub fn adc1_2_common_for<D: Device>(state: Arc<Mutex<AdcCommonState>>) -> Self {
        Self::for_device::<D>("ADC1_2", "adc1_2_common", state)
    }

    fn for_device<D: Device>(
        peripheral: &str,
        name: &'static str,
        state: Arc<Mutex<AdcCommonState>>,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        let ccr = def
            .registers
            .iter()
            .find(|r| r.name == "CCR")
            .expect("common has CCR");
        let csr = def
            .registers
            .iter()
            .find(|r| r.name == "CSR")
            .expect("common has CSR");
        let cdr = def
            .registers
            .iter()
            .find(|r| r.name == "CDR")
            .expect("common has CDR");
        let dual_field = ccr
            .fields
            .iter()
            .find(|f| f.name == "MULT" || f.name == "MULTI" || f.name == "DUAL")
            .expect("CCR has MULT/DUAL");
        let cfg = AdcCommonConfig {
            ccr_off: ccr.offset,
            csr_off: csr.offset,
            cdr_off: cdr.offset,
            dual_shift: u32::from(dual_field.offset),
            dual_width: u32::from(dual_field.width),
        };
        Self {
            inner: RegisterFile::new(def, AdcCommonBehavior { state, cfg }),
            name,
        }
    }
}

impl Peripheral for AdcCommon {
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

impl InterruptSource for AdcCommon {
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

/// Derive the chip's [`AdcConfig`] from the SVD table's field layout, so the
/// model follows whichever silicon the tables describe.
fn config_from(def: &'static PeripheralDef) -> AdcConfig {
    let cr = def
        .registers
        .iter()
        .find(|r| r.name == "CR2" || r.name == "CR")
        .expect("ADC table contains CR2/CR");
    let swstart = cr
        .fields
        .iter()
        .find(|f| f.name == "SWSTART" || f.name == "ADSTART")
        .map_or(1 << 22, |f| 1u32 << f.offset);
    let sr = def
        .registers
        .iter()
        .find(|r| r.name == "SR" || r.name == "ISR")
        .expect("ADC table contains SR/ISR");
    let has_ovr = sr.fields.iter().any(|f| f.name == "OVR");
    let has_adrdy = sr.fields.iter().any(|f| f.name == "ADRDY");
    let eoc_bit = sr
        .fields
        .iter()
        .find(|f| f.name == "EOC")
        .map(|f| 1u32 << f.offset)
        .unwrap_or(1 << 1);
    let adon_field = cr
        .fields
        .iter()
        .find(|f| f.name == "ADON" || f.name == "ADEN");
    let adon_bit = adon_field.map(|f| 1u32 << f.offset).unwrap_or(1 << 0);
    // The F1 still routes the external trigger through `CR2.EXTTRIG`; the F4
    // replaced it with `EXTEN`. Where `EXTTRIG` exists, a second `ADON` write
    // restarts the converter.
    let adon_restarts = cr.fields.iter().any(|f| f.name == "EXTTRIG");
    let has_chselr = def.registers.iter().any(|r| r.name == "CHSELR");
    let chselr_off = def
        .registers
        .iter()
        .find(|r| r.name == "CHSELR")
        .map(|r| r.offset)
        .unwrap_or(0x28);
    let sqr1_off = if has_chselr {
        chselr_off // unused, but keep valid
    } else {
        def.registers
            .iter()
            .find(|r| r.name == "SQR1")
            .expect("ADC table contains SQR1")
            .offset
    };
    let dr = def
        .registers
        .iter()
        .find(|r| r.name == "DR")
        .expect("ADC table contains DR");
    AdcConfig {
        swstart,
        has_ovr,
        has_adrdy,
        adon_restarts,
        has_chselr,
        cr_off: cr.offset,
        sr_off: sr.offset,
        sqr1_off,
        chselr_off,
        dr_off: dr.offset,
        eoc_bit,
        adon_bit,
    }
}

impl Peripheral for Adc {
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

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for Adc {
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
    use crate::device::svd::stm32f407::Stm32f407;

    fn read_word(adc: &mut Adc, off: u32) -> u32 {
        let base = adc.ranges()[0].base;
        let mut word = 0;
        for i in 0..4 {
            let byte = adc.read(base + off + i).unwrap();
            word |= u32::from(byte) << (8 * i);
        }
        word
    }

    fn write_word(adc: &mut Adc, off: u32, val: u32) {
        let base = adc.ranges()[0].base;
        for i in 0..4 {
            adc.write(base + off + i, ((val >> (8 * i)) & 0xFF) as u8)
                .unwrap();
        }
    }

    const SWSTART_F1: u32 = 1 << 22;
    const SWSTART_F4: u32 = 1 << 30;

    #[test]
    fn swstart_samples_the_sequence() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x0ABC);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        assert_eq!(read_word(&mut adc, SR) & EOC, 0); // not done yet
        adc.tick(14);
        let sr = read_word(&mut adc, SR);
        assert!(sr & EOC != 0 && sr & STRT != 0);
        assert_eq!(read_word(&mut adc, DR), 0x0ABC);
        // Reading the result clears EOC/STRT.
        assert_eq!(read_word(&mut adc, SR) & (EOC | STRT), 0);
    }

    #[test]
    fn eoc_raises_the_irq_line() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x0ABC);
        write_word(&mut adc, CR1, EOCIE);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        assert_eq!(adc.pending(), 0);
        adc.tick(14);
        assert_eq!(adc.pending(), 1);
        // Level-triggered: acknowledging does not release the line.
        adc.acknowledge(1);
        assert_eq!(adc.pending(), 1);
        // Reading the result does.
        assert_eq!(read_word(&mut adc, DR), 0x0ABC);
        assert_eq!(adc.pending(), 0);
    }

    #[test]
    fn f4_overruns_when_the_result_is_stale() {
        let mut adc = Adc::adc1_for::<Stm32f407>();
        adc.drive_inputs(0x0ABC);
        write_word(&mut adc, CR1, OVRIE);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F4);
        adc.tick(14);
        assert_eq!(read_word(&mut adc, SR) & OVR, 0);
        // A second conversion while the first result is unread overruns.
        write_word(&mut adc, CR2, ADON | SWSTART_F4);
        adc.tick(14);
        let sr = read_word(&mut adc, SR);
        assert!(sr & OVR != 0 && sr & EOC != 0);
        assert_eq!(adc.pending(), 1); // OVRIE raises the line
    }

    #[test]
    fn f1_has_no_overrun_flag() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x0ABC);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        write_word(&mut adc, CR2, ADON | SWSTART_F1); // stale result, start again
        adc.tick(14);
        let sr = read_word(&mut adc, SR);
        assert_eq!(sr & OVR, 0);
        assert!(sr & EOC != 0);
    }

    #[test]
    fn cont_rearms_without_a_new_start() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x123);
        write_word(&mut adc, CR2, ADON | CONT);
        write_word(&mut adc, CR2, ADON | CONT | SWSTART_F1);
        adc.tick(14);
        assert!(read_word(&mut adc, SR) & EOC != 0);
        assert_eq!(read_word(&mut adc, DR), 0x123); // reads clear EOC
        assert_eq!(read_word(&mut adc, SR) & EOC, 0);
        // The converter re-arms and finishes the next sample by itself.
        adc.tick(14);
        assert!(read_word(&mut adc, SR) & EOC != 0);
        assert_eq!(read_word(&mut adc, DR), 0x123);
    }

    #[test]
    fn scan_walks_the_sequence_then_stops() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0xABC);
        // SQ1 = ch3, SQ2 = ch5, L = 1 (two conversions).
        write_word(&mut adc, SQR3, 3 | (5 << 5));
        write_word(&mut adc, SQR1, 1 << 20);
        write_word(&mut adc, CR1, SCAN);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        assert!(read_word(&mut adc, SR) & EOC != 0);
        assert_eq!(read_word(&mut adc, DR), 0xABC);
        // The scan re-arms the second channel without another SWSTART.
        assert_eq!(read_word(&mut adc, SR) & EOC, 0);
        adc.tick(14);
        assert!(read_word(&mut adc, SR) & EOC != 0);
        assert_eq!(read_word(&mut adc, DR), 0xABC);
        // The two-channel sequence is finished.
        adc.tick(14);
        assert_eq!(read_word(&mut adc, SR) & EOC, 0);
    }

    #[test]
    fn align_shifts_the_result_left() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0xABC);
        write_word(&mut adc, CR2, ADON | ALIGN);
        write_word(&mut adc, CR2, ADON | ALIGN | SWSTART_F1);
        adc.tick(14);
        assert_eq!(read_word(&mut adc, DR), 0xABC << 4);
    }

    #[test]
    fn f1_second_adon_restarts_a_conversion() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x5);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON); // second ADON while powered
        adc.tick(14);
        assert!(read_word(&mut adc, SR) & EOC != 0);
        assert_eq!(read_word(&mut adc, DR), 0x5);
    }

    #[test]
    fn f4_second_adon_does_not_restart() {
        let mut adc = Adc::adc1_for::<Stm32f407>();
        adc.drive_inputs(0x5);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON);
        adc.tick(14);
        assert_eq!(read_word(&mut adc, SR) & EOC, 0);
    }

    #[test]
    fn powering_off_aborts_the_conversion() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x5);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(5);
        write_word(&mut adc, CR2, 0); // power off mid-conversion
        adc.tick(20);
        assert_eq!(read_word(&mut adc, SR) & EOC, 0);
        assert_eq!(read_word(&mut adc, DR), 0);
    }

    #[test]
    fn dma_request_on_eoc() {
        use crate::device::stm32::dma::DmaRequests;
        use std::sync::{Arc, Mutex};
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut adc = Adc::adc1().with_dma(reqs.clone(), 0, 1);
        adc.drive_inputs(0x123);
        write_word(&mut adc, CR2, ADON | (1 << 8)); // DMA
        write_word(&mut adc, CR2, ADON | (1 << 8) | SWSTART_F1);
        adc.tick(14);
        assert_ne!(reqs.lock().unwrap().from_periph(), 0, "DMA requested");
    }

    const JSWSTART_F1: u32 = 1 << 21;

    /// `CR2.JSWSTART` runs the injected sequence from `JSQR`: each result
    /// lands in its own `JDRn`, `SR.JEOC` latches at the end, and the line
    /// asserts only with `CR1.JEOCIE`.
    #[test]
    fn injected_sequence_lands_jdrs_and_jeoc() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0xABC);
        // JSQR: JL=1 (2 conversions), JSQ1=ch3 @bit15, JSQ2=ch8 @bit10.
        write_word(
            &mut adc,
            super::JSQR_OFFSET,
            (1 << 21) | (3 << 15) | (8 << 10),
        );
        write_word(&mut adc, CR2, ADON | JSWSTART_F1);
        assert_eq!(adc.pending(), 0, "JEOCIE off");
        adc.tick(14); // first injected conversion
        assert_eq!(read_word(&mut adc, super::JDR1_OFFSET), 0xABC);
        adc.tick(14); // second
        assert_eq!(read_word(&mut adc, super::JDR1_OFFSET + 4), 0xABC);
        // Third pass to re-latch flags (the JDR reads above cleared them).
        write_word(&mut adc, CR2, ADON | JSWSTART_F1);
        adc.tick(14);
        adc.tick(14);
        let sr = read_word(&mut adc, SR);
        assert_ne!(sr & (1 << 2), 0, "JEOC");
        assert_ne!(sr & (1 << 3), 0, "JSTRT");
        // Enable JEOCIE: line asserts; reading JDR1 clears JEOC/JSTRT.
        write_word(&mut adc, CR1, 1 << 7);
        assert_eq!(adc.pending(), 1);
        let _ = read_word(&mut adc, super::JDR1_OFFSET);
        assert_eq!(adc.pending(), 0);
    }

    /// The injected group preempts an in-flight regular conversion and the
    /// regular one resumes afterwards.
    #[test]
    fn injected_preempts_regular_and_resumes() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x111);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1); // regular ch0
        adc.tick(5); // partially through
                     // Inject now.
        write_word(&mut adc, super::JSQR_OFFSET, 3 << 15); // JL=0: ch3 only
        write_word(&mut adc, CR2, ADON | JSWSTART_F1);
        adc.tick(14);
        assert_eq!(read_word(&mut adc, super::JDR1_OFFSET), 0x111);
        assert_eq!(read_word(&mut adc, DR) & 0xFFF, 0, "regular not done yet");
        // Regular conversion still completes after the injected one.
        adc.tick(14);
        assert_eq!(read_word(&mut adc, DR) & 0xFFF, 0x111, "regular resumed");
    }

    /// `CR1.JAUTO` starts the injected sequence automatically when the
    /// regular sequence completes.
    #[test]
    fn jauto_starts_injected_after_regular() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x222);
        write_word(&mut adc, super::JSQR_OFFSET, 6 << 15); // injected ch6
                                                           // JAUTO bit 10 + scan-less single regular conversion.
        write_word(&mut adc, CR1, 1 << 10);
        write_word(&mut adc, CR2, ADON);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14); // regular completes...
        adc.tick(14); // ...injected auto-started and finishes
        assert_ne!(read_word(&mut adc, SR) & (1 << 2), 0, "JEOC");
        assert_eq!(read_word(&mut adc, super::JDR1_OFFSET), 0x222);
    }

    /// The analog watchdog latches `SR.AWD` when a watched regular result
    /// leaves `[LTR, HTR]`; with `AWDIE` the line asserts and software
    /// clears the flag by writing zeros to `SR`.
    #[test]
    fn analog_watchdog_latches_out_of_band_samples() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x800); // mid-scale: inside the band
        write_word(&mut adc, super::HTR_OFFSET, 0x900);
        write_word(&mut adc, super::LTR_OFFSET, 0x100);
        // Watch all channels, interrupt on.
        write_word(&mut adc, CR1, 1 << 23 | 1 << 6);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        assert_eq!(read_word(&mut adc, SR) & (1 << 0), 0, "inside band");
        assert_eq!(adc.pending(), 0);

        // Drive above the high threshold.
        adc.drive_inputs(0xA00);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        assert_ne!(read_word(&mut adc, SR) & (1 << 0), 0, "above HT");
        assert_eq!(adc.pending(), 1, "AWDIE raises the line");

        // Below the low threshold also trips.
        write_word(&mut adc, SR, !(1 << 0)); // clear AWD
        assert_eq!(adc.pending(), 0, "flag cleared by software");
        adc.drive_inputs(0x50);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        assert_ne!(read_word(&mut adc, SR) & (1 << 0), 0, "below LT");
    }

    /// With `AWDSGL` set only the selected channel is guarded.
    #[test]
    fn watchdog_single_channel_ignores_others() {
        let mut adc = Adc::adc1();
        adc.drive_inputs(0x900);
        write_word(&mut adc, super::HTR_OFFSET, 0x700);
        write_word(&mut adc, super::LTR_OFFSET, 0x000);
        // SGL watching channel 2; sequence samples channel 5.
        write_word(&mut adc, CR1, 1 << 23 | 1 << 9 | 2);
        write_word(&mut adc, SQR3_OFF, 5);
        if std::env::var("ADC_DBG").is_ok() {
            println!(
                "pre-cr2 sr={:#x} cr1={:#x}",
                read_word(&mut adc, super::SR_TEST),
                read_word(&mut adc, CR1)
            );
        }
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        if std::env::var("ADC_DBG").is_ok() {
            println!("post-cr2 sr={:#x}", read_word(&mut adc, super::SR_TEST));
        }
        adc.tick(14);
        if std::env::var("ADC_DBG").is_ok() {
            println!(
                "sr={:#x} dr={:#x}",
                read_word(&mut adc, super::SR_TEST),
                read_word(&mut adc, DR)
            );
        }
        assert_eq!(
            read_word(&mut adc, SR) & (1 << 0),
            0,
            "channel 5 not watched"
        );
        // Now sample the watched channel: it trips.
        write_word(&mut adc, SQR3_OFF, 2);
        write_word(&mut adc, CR2, ADON | SWSTART_F1);
        adc.tick(14);
        assert_ne!(read_word(&mut adc, SR) & (1 << 0), 0, "channel 2 watched");
    }

    #[test]
    fn f4_dual_simultaneous_cdr_combines_both_adcs() {
        use std::sync::{Arc, Mutex};
        let common = Arc::new(Mutex::new(AdcCommonState::default()));
        // C_ADC CCR at 0x04 for F407: MULT=6 regular simultaneous
        let mut c_adc = AdcCommon::c_adc_for::<Stm32f407>(common.clone());
        let mut adc1 = Adc::adc1_for::<Stm32f407>().with_common(common.clone());
        let mut adc2 = Adc::adc2_for::<Stm32f407>().with_common(common.clone());
        // Power both ADCs
        write_word(&mut adc1, CR2, ADON);
        write_word(&mut adc2, CR2, ADON);
        // Distinct analog levels per channel; drive separately then start
        adc1.drive_inputs(0x111);
        adc2.drive_inputs(0x222);
        // Program DUAL=6
        let base_c = c_adc.ranges()[0].base;
        for i in 0..4 {
            c_adc
                .write(base_c + 0x04 + i, ((6u32 >> (8 * i)) & 0xFF) as u8)
                .unwrap();
        }
        // Master SWSTART
        write_word(&mut adc1, CR2, ADON | SWSTART_F4);
        // Slave should auto-start next tick
        adc1.tick(14);
        adc2.tick(14);
        assert_ne!(read_word(&mut adc1, SR) & EOC, 0, "adc1 EOC");
        assert_ne!(read_word(&mut adc2, SR) & EOC, 0, "adc2 slave EOC");
        // CDR combines
        let mut cdr = 0u32;
        for i in 0..4 {
            cdr |= u32::from(c_adc.read(base_c + 0x08 + i).unwrap()) << (8 * i);
        }
        assert_eq!(cdr & 0xFFFF, 0x111);
        assert_eq!((cdr >> 16) & 0xFFFF, 0x222);
        // CSR mirrors
        let mut csr = 0u32;
        for i in 0..4 {
            csr |= u32::from(c_adc.read(base_c + i).unwrap()) << (8 * i);
        }
        assert_ne!(csr & (1 << 1), 0, "CSR EOC1");
        assert_ne!(csr & (1 << 9), 0, "CSR EOC2");
    }
}

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
const DR: u32 = 0x4C;

/// `SR` flag bits.
const EOC: u32 = 1 << 1;
const STRT: u32 = 1 << 4;
const OVR: u32 = 1 << 5; // F4 only

/// `CR1` bits.
const EOCIE: u32 = 1 << 5;
const SCAN: u32 = 1 << 8;
const OVRIE: u32 = 1 << 26; // F4 only

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
    /// The `CR2.SWSTART` / `CR.ADSTART` bit position (22 on the F1, 30 on the F4, 2 on H7).
    swstart: u32,
    /// Whether `SR.OVR`/`CR1.OVRIE` exist (the F4 overrun pair).
    has_ovr: bool,
    /// Whether a second `ADON` write restarts a conversion (F1 legacy).
    adon_restarts: bool,
    cr_off: u32,
    sr_off: u32,
    sqr1_off: u32,
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
    config: AdcConfig,
    dma: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
}

impl AdcBehavior {
    fn new(config: AdcConfig) -> Self {
        Self {
            analog: [0; 18],
            converting: None,
            sq_index: 0,
            config,
            dma: None,
        }
    }

    /// The number of conversions in the regular sequence (`SQR1.L + 1`).
    fn sequence_len(&self, view: &Registers<&mut [u32]>) -> u32 {
        ((view.get(self.config.sqr1_off) >> 20) & 0x0F) + 1
    }

    /// The channel at `index` in the regular sequence (`SQR3` SQ1..SQ6,
    /// `SQR2` SQ7..SQ12, `SQR1` SQ13..SQ16).
    fn channel_at(&self, view: &Registers<&mut [u32]>, index: u32) -> u8 {
        let sqr1 = self.config.sqr1_off;
        let (reg, shift) = match index {
            0..=5 => (sqr1 + 8, index * 5),        // SQR3 = SQR1+8
            6..=11 => (sqr1 + 4, (index - 6) * 5), // SQR2 = SQR1+4
            _ => (sqr1, (index - 12) * 5),
        };
        ((view.get(reg) >> shift) & 0x1F) as u8
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
            // SWSTART/ADSTART is a self-clearing strobe: it reads back as zero.
            let stored = word & !self.config.swstart;
            // H7: ADEN rising sets ISR.ADRDY
            if powered && !was_powered {
                let isr = view.get(self.config.sr_off);
                view.set(self.config.sr_off, isr | 1);
            }
            if powered {
                if swstart {
                    self.start_sequence(view);
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
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
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
        let value = u32::from(
            self.analog
                .get(usize::from(conv.channel))
                .copied()
                .unwrap_or(0)
                & 0x0FFF,
        );
        let aligned = if view.get(self.config.cr_off) & ALIGN != 0 {
            value << 4
        } else {
            value
        };
        let sr = view.get(self.config.sr_off);
        let eoc = self.config.eoc_bit;
        if self.config.has_ovr && sr & eoc != 0 {
            // A second result while the previous one is unread overruns.
            view.set(self.config.sr_off, sr | eoc | STRT | OVR);
        } else {
            view.set(self.config.sr_off, sr | eoc | STRT);
        }
        view.set(self.config.dr_off, aligned);
        // DMA request on EOC: CR2 DMA (bit8) enabled.
        if view.get(self.config.cr_off) & (1 << 8) != 0 {
            if let Some((reqs, ctrl, ch)) = self.dma.as_ref() {
                let reqs = Arc::clone(reqs);
                reqs.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .request(*ctrl, *ch, false);
            }
        }
        // Continuous or scan mode re-arms the next conversion immediately.
        let continuous = view.get(self.config.cr_off) & CONT != 0;
        let scan = view.get(CR1) & SCAN != 0;
        let len = self.sequence_len(view);
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
                || (self.config.has_ovr && sr & OVR != 0 && cr1 & OVRIE != 0),
        )
    }

    fn reset(&mut self) {
        self.converting = None;
        self.sq_index = 0;
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

    fn for_device<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, AdcBehavior::new(config_from(def))),
            name,
        }
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
    let sqr1 = def
        .registers
        .iter()
        .find(|r| r.name == "SQR1")
        .expect("ADC table contains SQR1");
    let dr = def
        .registers
        .iter()
        .find(|r| r.name == "DR")
        .expect("ADC table contains DR");
    AdcConfig {
        swstart,
        has_ovr,
        adon_restarts,
        cr_off: cr.offset,
        sr_off: sr.offset,
        sqr1_off: sqr1.offset,
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
}

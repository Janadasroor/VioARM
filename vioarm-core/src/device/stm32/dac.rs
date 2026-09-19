//! STM32 DAC (`DAC`, `0x4000_7400`) — accurate DHR→DOR with trigger.
//! `CR` (`EN1`/`TEN1`/`TSEL1`/`EN2`…), `SWTRIGR` software trigger, `DHR12R1`/
//! `DHR12L1`/`DHR8R1` and channel-2, `DHR12RD` dual, `DOR1`/`DOR2` read-only
//! reflect the last converted value. When `TENx==0` a `DHR` write loads `DORx`
//! if `ENx`; when `TENx==1` and `TSELx==0` (software) `SWTRIGR` latches.
//!
//! Waveform generation (`CR.WAVEx`, `MAMPx`): a software trigger steps the
//! wave instead of latching `DHR` — triangle ramps the counter up and down
//! within `(1 << (MAMP+1)) - 1`, noise steps a 12-bit LFSR (taps 12/6/4/1,
//! a documented approximation: the RM names the LFSR but not its
//! polynomial) masked to the same amplitude. Both add to the `DHR` base,
//! clipped at full scale. Converted outputs publish to the analog bus when
//! wired (`with_analog_bus`), feeding ADC/COMP chains. Deliberately
//! unmodeled: timer-triggered waves (no stimulus edge exists yet) and
//! wave+DMA combinations.

use crate::device::analog::AnalogBusState;
use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const CR: u32 = 0x00;
const SWTRIGR: u32 = 0x04;
const DHR12R1: u32 = 0x08;
const DHR12L1: u32 = 0x0C;
const DHR8R1: u32 = 0x10;
const DHR12R2: u32 = 0x14;
const DHR12L2: u32 = 0x18;
const DHR8R2: u32 = 0x1C;
const DHR12RD: u32 = 0x20;
const DHR12LD: u32 = 0x24;
const DHR8RD: u32 = 0x28;
const DOR1: u32 = 0x2C;
const DOR2: u32 = 0x30;

const CR_EN1: u32 = 1 << 0;
const CR_TEN1: u32 = 1 << 2;
const CR_TSEL1_SHIFT: u32 = 3;
const CR_DMAEN1: u32 = 1 << 12;
const CR_EN2: u32 = 1 << 16;
const CR_TEN2: u32 = 1 << 18;
const CR_TSEL2_SHIFT: u32 = 19;
const CR_DMAEN2: u32 = 1 << 28;
/// Waveform generation (`WAVE[1:0]`: 0 off, 1 noise, 2/3 triangle) and
/// amplitude mask (`MAMP[3:0]`: mask `(1 << (MAMP+1)) - 1`).
const CR_WAVE1_SHIFT: u32 = 6;
const CR_MAMP1_SHIFT: u32 = 8;
const CR_WAVE2_SHIFT: u32 = 22;
const CR_MAMP2_SHIFT: u32 = 24;

#[derive(Debug)]
struct DacBehavior {
    requests: Option<Arc<Mutex<DmaRequests>>>,
    /// Waveform state per channel: triangle counter + direction, and the
    /// noise LFSR (reset seed `0xAAA`).
    tri1: u16,
    tri1_up: bool,
    lfsr1: u16,
    tri2: u16,
    tri2_up: bool,
    lfsr2: u16,
    /// Shared analog bus (WP-R4): converted outputs publish their
    /// millivolts, feeding ADC/COMP chains on the same board.
    analog_bus: Option<Arc<Mutex<AnalogBusState>>>,
}

/// Noise LFSR seed (silicon reset value).
const LFSR_SEED: u16 = 0xAAA;

impl Default for DacBehavior {
    fn default() -> Self {
        Self {
            requests: None,
            tri1: 0,
            tri1_up: true,
            lfsr1: LFSR_SEED,
            tri2: 0,
            tri2_up: true,
            lfsr2: LFSR_SEED,
            analog_bus: None,
        }
    }
}

impl DacBehavior {
    /// One 12-bit LFSR step (taps 12/6/4/1, maximal — a documented
    /// approximation: the RM names the LFSR but not its polynomial).
    fn lfsr_step(lfsr: u16) -> u16 {
        let bit = ((lfsr >> 11) ^ (lfsr >> 5) ^ (lfsr >> 3) ^ lfsr) & 1;
        ((lfsr << 1) | bit) & 0xFFF
    }

    /// Publish a converted code to the analog bus, if wired.
    fn publish(&self, code: u32) {
        if let Some(bus) = self.analog_bus.as_ref() {
            AnalogBusState::set_millivolts(bus, code * 3300 / 4095);
        }
    }
    fn transfer_ch1(&mut self, view: &mut Registers<&mut [u32]>, val12: u32) {
        let cr = view.get(CR);
        if cr & CR_EN1 == 0 {
            return;
        }
        // In DMAEN triggered mode the DMA write still latches.
        if cr & CR_TEN1 != 0 && cr & CR_DMAEN1 == 0 {
            return;
        }
        view.set(DOR1, val12 & 0xFFF);
        self.publish(val12 & 0xFFF);
    }
    fn transfer_ch2(&mut self, view: &mut Registers<&mut [u32]>, val12: u32) {
        let cr = match view.try_get(CR) {
            Some(v) => v,
            None => return,
        };
        if cr & CR_EN2 == 0 {
            return;
        }
        if cr & CR_TEN2 != 0 && cr & CR_DMAEN2 == 0 {
            return;
        }
        view.try_set(DOR2, val12 & 0xFFF);
        self.publish(val12 & 0xFFF);
    }
    /// Amplitude mask from `MAMP[3:0]`: `(1 << (MAMP+1)) - 1`.
    fn wave_mask(mamp: u32) -> u32 {
        if mamp >= 12 {
            0xFFF
        } else {
            (1 << (mamp + 1)) - 1
        }
    }

    /// Step channel 1's waveform on a software trigger: the wave value
    /// adds to the `DHR` base (clipped at full scale), like silicon.
    fn step_wave1(&mut self, view: &mut Registers<&mut [u32]>) {
        let cr = view.get(CR);
        let base = view.get(DHR12R1) & 0xFFF;
        let mask = Self::wave_mask((cr >> CR_MAMP1_SHIFT) & 0xF);
        let out = if (cr >> CR_WAVE1_SHIFT) & 0x3 == 1 {
            self.lfsr1 = Self::lfsr_step(self.lfsr1);
            base.saturating_add(u32::from(self.lfsr1) & mask).min(0xFFF)
        } else {
            if self.tri1_up {
                if self.tri1 >= mask as u16 {
                    self.tri1_up = false;
                    self.tri1 -= 1;
                } else {
                    self.tri1 += 1;
                }
            } else if self.tri1 == 0 {
                self.tri1_up = true;
                self.tri1 += 1;
            } else {
                self.tri1 -= 1;
            }
            base.saturating_add(u32::from(self.tri1)).min(0xFFF)
        };
        view.set(DOR1, out);
        self.publish(out);
    }

    /// Step channel 2's waveform (mirror of channel 1).
    fn step_wave2(&mut self, view: &mut Registers<&mut [u32]>) {
        let cr = view.get(CR);
        let base = match view.try_get(DHR12R2) {
            Some(v) => v & 0xFFF,
            None => return,
        };
        let mask = Self::wave_mask((cr >> CR_MAMP2_SHIFT) & 0xF);
        let out = if (cr >> CR_WAVE2_SHIFT) & 0x3 == 1 {
            self.lfsr2 = Self::lfsr_step(self.lfsr2);
            base.saturating_add(u32::from(self.lfsr2) & mask).min(0xFFF)
        } else {
            if self.tri2_up {
                if self.tri2 >= mask as u16 {
                    self.tri2_up = false;
                    self.tri2 -= 1;
                } else {
                    self.tri2 += 1;
                }
            } else if self.tri2 == 0 {
                self.tri2_up = true;
                self.tri2 += 1;
            } else {
                self.tri2 -= 1;
            }
            base.saturating_add(u32::from(self.tri2)).min(0xFFF)
        };
        view.try_set(DOR2, out);
        self.publish(out);
    }

    fn assert_dma(&self, ch: u8) {
        if let Some(req) = &self.requests {
            // DAC1 -> DMA1 Ch3, DAC2 -> DMA1 Ch4, memory→periph.
            req.lock()
                .unwrap_or_else(|e| e.into_inner())
                .request(0, ch, true);
        }
    }
}

impl Behavior for DacBehavior {
    // Triggers convert synchronously (SWTRIGR/TIM edge); nothing free-runs.
    fn needs_tick() -> bool {
        false
    }
    fn reset(&mut self) {
        self.tri1 = 0;
        self.tri1_up = true;
        self.lfsr1 = LFSR_SEED;
        self.tri2 = 0;
        self.tri2_up = true;
        self.lfsr2 = LFSR_SEED;
    }
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let cur = view.get(offset);
        let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);

        match offset {
            CR => word,
            SWTRIGR => {
                if word & (1 << 0) != 0 {
                    let cr = view.get(CR);
                    if cr & CR_EN1 != 0 && cr & CR_TEN1 != 0 && ((cr >> CR_TSEL1_SHIFT) & 0x7) == 0
                    {
                        if cr & CR_DMAEN1 != 0 {
                            self.assert_dma(3);
                        } else if (cr >> CR_WAVE1_SHIFT) & 0x3 != 0 {
                            self.step_wave1(view);
                        } else {
                            let v = view.get(DHR12R1) & 0xFFF;
                            view.set(DOR1, v);
                            self.publish(v);
                        }
                    }
                }
                if word & (1 << 1) != 0 {
                    let cr = view.get(CR);
                    if cr & CR_EN2 != 0 && cr & CR_TEN2 != 0 && ((cr >> CR_TSEL2_SHIFT) & 0x7) == 0
                    {
                        if cr & CR_DMAEN2 != 0 {
                            self.assert_dma(4);
                        } else if (cr >> CR_WAVE2_SHIFT) & 0x3 != 0 {
                            self.step_wave2(view);
                        } else if let Some(v) = view.try_get(DHR12R2) {
                            let v = v & 0xFFF;
                            view.try_set(DOR2, v);
                            self.publish(v);
                        }
                    }
                }
                // SWTRIGR is W1, self-clears.
                0
            }
            DHR12R1 => {
                let v = word & 0xFFF;
                self.transfer_ch1(view, v);
                word
            }
            DHR12L1 => {
                let v = (word >> 4) & 0xFFF;
                self.transfer_ch1(view, v);
                word
            }
            DHR8R1 => {
                let v = word & 0xFF;
                self.transfer_ch1(view, v);
                word
            }
            DHR12R2 => {
                let v = word & 0xFFF;
                self.transfer_ch2(view, v);
                word
            }
            DHR12L2 => {
                let v = (word >> 4) & 0xFFF;
                self.transfer_ch2(view, v);
                word
            }
            DHR8R2 => {
                let v = word & 0xFF;
                self.transfer_ch2(view, v);
                word
            }
            DHR12RD => {
                let v1 = word & 0xFFF;
                let v2 = (word >> 16) & 0xFFF;
                self.transfer_ch1(view, v1);
                self.transfer_ch2(view, v2);
                word
            }
            DHR12LD => {
                let v1 = (word >> 4) & 0xFFF;
                let v2 = (word >> 20) & 0xFFF;
                self.transfer_ch1(view, v1);
                self.transfer_ch2(view, v2);
                word
            }
            DHR8RD => {
                let v1 = word & 0xFF;
                let v2 = (word >> 8) & 0xFF;
                self.transfer_ch1(view, v1);
                self.transfer_ch2(view, v2);
                word
            }
            DOR1 | DOR2 => cur, // read-only
            _ => word,
        }
    }
}

/// Digital-to-analog converter.
#[derive(Debug)]
pub struct Dac {
    inner: RegisterFile<DacBehavior>,
}
impl Dac {
    /// A new `Dac` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DAC").expect("generated tables contain DAC");
        Self {
            inner: RegisterFile::new(def, DacBehavior::default()),
        }
    }

    /// A new peripheral with DMA request lines.
    #[must_use]
    pub fn new_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        let def = D::peripheral("DAC").expect("generated tables contain DAC");
        Self {
            inner: RegisterFile::new(
                def,
                DacBehavior {
                    requests: Some(requests),
                    ..DacBehavior::default()
                },
            ),
        }
    }

    /// Publish converted outputs to the shared analog bus (feeds ADC/COMP
    /// chains on the same board).
    #[must_use]
    pub fn with_analog_bus(mut self, bus: Arc<Mutex<AnalogBusState>>) -> Self {
        self.inner.behavior_mut().analog_bus = Some(bus);
        self
    }
}
impl Peripheral for Dac {
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
    fn name(&self) -> &'static str {
        "dac"
    }
}
impl InterruptSource for Dac {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::Dac;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;
    const BASE: u32 = 0x4000_7400;
    fn write_word(d: &mut Dac, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(BASE + off + i as u32, *b).unwrap();
        }
    }
    fn read_word(d: &mut Dac, off: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = d.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn dac_immediate_transfers_when_enabled_no_trigger() {
        let mut d = Dac::new_for::<Stm32f103>();
        write_word(&mut d, 0x00, 0x0000_0001); // EN1
        write_word(&mut d, 0x08, 0x0ABC);
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 0x0ABC);
    }
    #[test]
    fn dac_triggered_waits_for_swtrig() {
        let mut d = Dac::new_for::<Stm32f103>();
        write_word(&mut d, 0x00, 0x0000_0005); // EN1 | TEN1 (TSEL 0 = SW)
        write_word(&mut d, 0x08, 0x0555);
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 0x000, "not yet latched");
        write_word(&mut d, 0x04, 0x01); // SWTRIGR SWTRIG1
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 0x555);
    }
    #[test]
    fn dac_disabled_does_not_latch() {
        let mut d = Dac::new_for::<Stm32f103>();
        write_word(&mut d, 0x08, 0x0FFF);
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 0x000);
    }

    /// Triangle wave: `SWTRIGR` steps the counter up within the `MAMP`
    /// mask and back down, added to the `DHR` base.
    #[test]
    fn triangle_ramps_up_and_down_within_amplitude() {
        let mut d = Dac::new_for::<Stm32f103>();
        // EN1 | TEN1 | WAVE1 = triangle (2) | MAMP1 = 7 (mask 0xFF).
        write_word(&mut d, 0x00, 0x1 | 0x4 | (2 << 6) | (7 << 8));
        write_word(&mut d, 0x08, 0x000);
        for _ in 0..10 {
            write_word(&mut d, 0x04, 0x01);
        }
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 10);
        for _ in 0..245 {
            write_word(&mut d, 0x04, 0x01);
        }
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 255);
        write_word(&mut d, 0x04, 0x01);
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 254);
        write_word(&mut d, 0x04, 0x01);
        assert_eq!(read_word(&mut d, 0x2C) & 0xFFF, 253);
    }

    /// Noise wave: outputs vary inside the mask and rerun identically.
    #[test]
    fn noise_varies_within_mask_and_reruns_identically() {
        fn run8() -> Vec<u32> {
            let mut d = Dac::new_for::<Stm32f103>();
            write_word(&mut d, 0x00, 0x1 | 0x4 | (1 << 6) | (3 << 8));
            write_word(&mut d, 0x08, 0x000);
            (0..8)
                .map(|_| {
                    write_word(&mut d, 0x04, 0x01);
                    read_word(&mut d, 0x2C) & 0xFFF
                })
                .collect()
        }
        let first = run8();
        assert!(first.iter().all(|&v| v <= 0xF), "masked: {first:?}");
        assert!(first.iter().any(|&v| v != first[0]), "varies: {first:?}");
        assert_eq!(run8(), first, "deterministic rerun");
    }

    /// Converted outputs publish to the wired analog bus.
    #[test]
    fn converted_output_publishes_to_the_bus() {
        use crate::device::analog::AnalogBusState;
        let bus = AnalogBusState::shared();
        let mut d = Dac::new_for::<Stm32f103>().with_analog_bus(bus.clone());
        write_word(&mut d, 0x00, 0x0000_0001); // EN1, direct
        write_word(&mut d, 0x08, 2048);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(2048 * 3300 / 4095));
    }
}

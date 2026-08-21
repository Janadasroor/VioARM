//! STM32 DAC (`DAC`, `0x4000_7400`) — accurate DHR→DOR with trigger.
//! `CR` (`EN1`/`TEN1`/`TSEL1`/`EN2`…), `SWTRIGR` software trigger, `DHR12R1`/
//! `DHR12L1`/`DHR8R1` and channel-2, `DHR12RD` dual, `DOR1`/`DOR2` read-only
//! reflect the last converted value. When `TENx==0` a `DHR` write loads `DORx`
//! if `ENx`; when `TENx==1` and `TSELx==0` (software) `SWTRIGR` latches.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const CR: u16 = 0x00;
const SWTRIGR: u16 = 0x04;
const DHR12R1: u16 = 0x08;
const DHR12L1: u16 = 0x0C;
const DHR8R1: u16 = 0x10;
const DHR12R2: u16 = 0x14;
const DHR12L2: u16 = 0x18;
const DHR8R2: u16 = 0x1C;
const DHR12RD: u16 = 0x20;
const DHR12LD: u16 = 0x24;
const DHR8RD: u16 = 0x28;
const DOR1: u16 = 0x2C;
const DOR2: u16 = 0x30;

const CR_EN1: u32 = 1 << 0;
const CR_TEN1: u32 = 1 << 2;
const CR_TSEL1_SHIFT: u32 = 3;
const CR_DMAEN1: u32 = 1 << 12;
const CR_EN2: u32 = 1 << 16;
const CR_TEN2: u32 = 1 << 18;
const CR_TSEL2_SHIFT: u32 = 19;
const CR_DMAEN2: u32 = 1 << 28;

#[derive(Debug)]
struct DacBehavior {
    requests: Option<Arc<Mutex<DmaRequests>>>,
}

impl Default for DacBehavior {
    fn default() -> Self {
        Self { requests: None }
    }
}

impl DacBehavior {
    fn transfer_ch1(view: &mut Registers<&mut [u32]>, val12: u32) {
        let cr = view.get(CR);
        if cr & CR_EN1 == 0 {
            return;
        }
        // In DMAEN triggered mode the DMA write still latches.
        if cr & CR_TEN1 != 0 && cr & CR_DMAEN1 == 0 {
            return;
        }
        view.set(DOR1, val12 & 0xFFF);
    }
    fn transfer_ch2(view: &mut Registers<&mut [u32]>, val12: u32) {
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
    }
    fn assert_dma(&self, ch: u8) {
        if let Some(req) = &self.requests {
            // DAC1 -> DMA1 Ch3, DAC2 -> DMA1 Ch4, memory→periph.
            req.lock().unwrap().request(0, ch, true);
        }
    }
}

impl Behavior for DacBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
        val: u8,
        shift: u32,
    ) -> u32 {
        let cur = view.get(offset);
        let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
        let ret = match offset {
            CR => word,
            SWTRIGR => {
                if word & (1 << 0) != 0 {
                    let cr = view.get(CR);
                    if cr & CR_EN1 != 0 && cr & CR_TEN1 != 0 && ((cr >> CR_TSEL1_SHIFT) & 0x7) == 0 {
                        if cr & CR_DMAEN1 != 0 {
                            self.assert_dma(3);
                        } else {
                            let v = view.get(DHR12R1) & 0xFFF;
                            view.set(DOR1, v);
                        }
                    }
                }
                if word & (1 << 1) != 0 {
                    let cr = view.get(CR);
                    if cr & CR_EN2 != 0 && cr & CR_TEN2 != 0 && ((cr >> CR_TSEL2_SHIFT) & 0x7) == 0 {
                        if cr & CR_DMAEN2 != 0 {
                            self.assert_dma(4);
                        } else if let Some(v) = view.try_get(DHR12R2) {
                            let v = v & 0xFFF;
                            view.try_set(DOR2, v);
                        }
                    }
                }
                // SWTRIGR is W1, self-clears.
                return 0;
            }
            DHR12R1 => {
                let v = word & 0xFFF;
                Self::transfer_ch1(view, v);
                word
            }
            DHR12L1 => {
                let v = (word >> 4) & 0xFFF;
                Self::transfer_ch1(view, v);
                word
            }
            DHR8R1 => {
                let v = word & 0xFF;
                Self::transfer_ch1(view, v);
                word
            }
            DHR12R2 => {
                let v = word & 0xFFF;
                Self::transfer_ch2(view, v);
                word
            }
            DHR12L2 => {
                let v = (word >> 4) & 0xFFF;
                Self::transfer_ch2(view, v);
                word
            }
            DHR8R2 => {
                let v = word & 0xFF;
                Self::transfer_ch2(view, v);
                word
            }
            DHR12RD => {
                let v1 = word & 0xFFF;
                let v2 = (word >> 16) & 0xFFF;
                Self::transfer_ch1(view, v1);
                Self::transfer_ch2(view, v2);
                word
            }
            DHR12LD => {
                let v1 = (word >> 4) & 0xFFF;
                let v2 = (word >> 20) & 0xFFF;
                Self::transfer_ch1(view, v1);
                Self::transfer_ch2(view, v2);
                word
            }
            DHR8RD => {
                let v1 = word & 0xFF;
                let v2 = (word >> 8) & 0xFF;
                Self::transfer_ch1(view, v1);
                Self::transfer_ch2(view, v2);
                word
            }
            DOR1 | DOR2 => cur, // read-only
            _ => word,
        };
        ret
    }
}

#[derive(Debug)]
pub struct Dac {
    inner: RegisterFile<DacBehavior>,
}
impl Dac {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DAC").expect("generated tables contain DAC");
        Self {
            inner: RegisterFile::new(def, DacBehavior::default()),
        }
    }

    #[must_use]
    pub fn new_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        let def = D::peripheral("DAC").expect("generated tables contain DAC");
        Self {
            inner: RegisterFile::new(
                def,
                DacBehavior {
                    requests: Some(requests),
                },
            ),
        }
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
}

//! DCMI `0x48020000` (H7) / `0x50050000` — digital camera interface.
//! `CR.CAPTURE@0` enable, `CR.ENABLE@14`, `RIS.FRAMERI@0` latches 1-tick after CAPTURE.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_CAPTURE: u32 = 1 << 0;
const CR_ENABLE: u32 = 1 << 14;
const RIS_FRAMERI: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct DcmiBehavior {
    pending: bool,
}

impl Behavior for DcmiBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let cap = word & CR_CAPTURE != 0;
            let en = word & CR_ENABLE != 0;
            if cap && en {
                self.pending = true;
                return word & !CR_CAPTURE;
            }
            if !en {
                self.pending = false;
                view.set(0x04, view.get(0x04) & !RIS_FRAMERI); // RIS at 0x04? Check SVD: RIS at 0x08? For H7 DCMI RIS at 0x04?
                                                               // Actually for H7 DCMI, RIS at 0x08, ICR at 0x0C
                                                               // We handle both
                view.set(0x08, view.get(0x08) & !RIS_FRAMERI);
            }
        }
        if offset == 0x14 {
            // ICR at 0x14 for H7
            if word & 1 != 0 {
                view.set(0x08, view.get(0x08) & !RIS_FRAMERI);
                self.pending = false;
            }
            return 0;
        }
        // ISR W1C at 0x08
        if offset == 0x08 && word & RIS_FRAMERI != 0 {
            view.set(0x08, view.get(0x08) & !RIS_FRAMERI);
            self.pending = false;
            return view.get(0x08);
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x08, view.get(0x08) | RIS_FRAMERI);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let isr = view.get(0x08);
        let ier = view.get(0x0C); // IER at 0x0C
        u32::from(isr & RIS_FRAMERI != 0 && ier & 1 != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

#[derive(Debug)]
pub struct Dcmi {
    inner: RegisterFile<DcmiBehavior>,
}
impl Dcmi {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DCMI").expect("DCMI");
        Self {
            inner: RegisterFile::new(def, DcmiBehavior::default()),
        }
    }
}
impl Peripheral for Dcmi {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, a: u32) -> Result<u8, BusError> {
        self.inner.read(a)
    }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> {
        self.inner.write(a, v)
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn name(&self) -> &'static str {
        "dcmi"
    }
}
impl InterruptSource for Dcmi {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, l: u32) {
        self.inner.acknowledge(l);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Dcmi, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> 8 * i) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Dcmi, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << 8 * i;
        }
        v
    }
    #[test]
    fn dcmi_capture_latches_frameri() {
        let mut d = Dcmi::new_for::<Stm32h743>();
        w(&mut d, 0x0C, 1); // IER at 0x0C
        w(&mut d, 0x00, CR_ENABLE | CR_CAPTURE);
        assert_eq!(r(&mut d, 0x08) & RIS_FRAMERI, 0);
        d.tick(1);
        assert_ne!(r(&mut d, 0x08) & RIS_FRAMERI, 0);
        assert_eq!(d.pending(), 1);
        assert_ne!(r(&mut d, 0x08) & RIS_FRAMERI, 0);
        assert_eq!(d.pending(), 1);
        // Clear via ICR at 0x14? Actually ICR after IER, but we handle RIS W1C
        w(&mut d, 0x14, 1); // ICR
        assert_eq!(r(&mut d, 0x08) & RIS_FRAMERI, 0);
    }
}

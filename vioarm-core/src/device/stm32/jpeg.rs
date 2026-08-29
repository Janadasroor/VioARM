//! JPEG `0x52003000` (H7) — codec.
//! `CONFR0.START@0` write starts, `SR.EOCF@5` latches 1-tick, `CR` enables IRQ.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const SR_EOCF: u32 = 1 << 5;
const CR_JCEN: u32 = 1 << 0; // CR at 0x30, JCEN

#[derive(Clone, Debug, Default)]
struct JpegBehavior { pending: bool }

impl Behavior for JpegBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 && word & 1 != 0 {
            // CONFR0 START
            if view.get(0x30) & CR_JCEN != 0 {
                self.pending = true;
            }
            return 0; // W1, reads 0
        }
        if offset == 0x30 {
            if word & CR_JCEN == 0 {
                self.pending = false;
                view.set(0x34, view.get(0x34) & !SR_EOCF);
            }
        }
        if offset == 0x38 {
            // CFR at 0x38: CEOCF
            if word & (1<<5) != 0 {
                view.set(0x34, view.get(0x34) & !SR_EOCF);
                self.pending = false;
            }
            return 0;
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x34, view.get(0x34) | SR_EOCF);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(0x34);
        let cr = view.get(0x30);
        u32::from(sr & SR_EOCF != 0 && cr & 1 != 0) // JCEN + EOCF, assume JCIE at 0
    }
    fn reset(&mut self) { self.pending = false; }
}

#[derive(Debug)]
pub struct Jpeg { inner: RegisterFile<JpegBehavior> }
impl Jpeg {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("JPEG").expect("JPEG");
        Self { inner: RegisterFile::new(def, JpegBehavior::default()) }
    }
}
impl Peripheral for Jpeg {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "jpeg" }
}
impl InterruptSource for Jpeg {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Jpeg, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF)as u8).unwrap(); } }
    fn r(c: &mut Jpeg, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn jpeg_start_latches_eocf() {
        let mut j = Jpeg::new_for::<Stm32h743>();
        w(&mut j, 0x30, 1); // CR JCEN
        w(&mut j, 0x00, 1); // CONFR0 START
        assert_eq!(r(&mut j, 0x34) & SR_EOCF,0);
        j.tick(1);
        assert_ne!(r(&mut j, 0x34) & SR_EOCF,0);
        w(&mut j, 0x38, 1<<5);
        assert_eq!(r(&mut j, 0x34) & SR_EOCF,0);
    }
}

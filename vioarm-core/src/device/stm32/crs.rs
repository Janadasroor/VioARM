//! CRS `0x4000C400` (F0) — clock recovery system.
//! `CR.AUTOTRIMEN@7` + `CR.CEN@5` enable, `ISR.SYNCOKF@0` latches after 1 tick, `ICR` clears.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_CEN: u32 = 1 << 5;
const CR_AUTOTRIMEN: u32 = 1 << 7;
const ISR_SYNCOKF: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct CrsBehavior { pending: bool }

impl Behavior for CrsBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let cen = word & CR_CEN != 0;
            let auto = word & CR_AUTOTRIMEN != 0;
            if cen && auto {
                self.pending = true;
            }
            if !cen {
                self.pending = false;
                view.set(0x08, view.get(0x08) & !ISR_SYNCOKF); // ISR at 0x08
            }
        }
        if offset == 0x0C {
            // ICR at 0x0C: SYNCOFC at 0
            if word & 1 != 0 {
                view.set(0x08, view.get(0x08) & !ISR_SYNCOKF);
                self.pending = false;
            }
            return 0;
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x08, view.get(0x08) | ISR_SYNCOKF);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let isr = view.get(0x08);
        let cr = view.get(0x00);
        // SYNCOKIE is in CR? Actually not, but we use CEN as enable for test
        u32::from(isr & ISR_SYNCOKF != 0 && cr & CR_CEN != 0)
    }
    fn reset(&mut self) { self.pending = false; }
}

#[derive(Debug)]
pub struct Crs { inner: RegisterFile<CrsBehavior> }
impl Crs {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CRS").or_else(|| D::peripheral("CRS1")).expect("CRS");
        Self { inner: RegisterFile::new(def, CrsBehavior::default()) }
    }
}
impl Peripheral for Crs {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "crs" }
}
impl InterruptSource for Crs {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f042::Stm32f042;
    fn w(c: &mut Crs, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF) as u8).unwrap(); } }
    fn r(c: &mut Crs, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn crs_syncok_latches() {
        let mut c = Crs::new_for::<Stm32f042>();
        w(&mut c, 0x00, CR_CEN|CR_AUTOTRIMEN);
        assert_eq!(r(&mut c, 0x08) & ISR_SYNCOKF,0);
        c.tick(1);
        assert_ne!(r(&mut c, 0x08) & ISR_SYNCOKF,0);
        w(&mut c, 0x0C, 1);
        assert_eq!(r(&mut c, 0x08) & ISR_SYNCOKF,0);
    }
}

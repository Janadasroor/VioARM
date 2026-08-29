//! VREFBUF `0x58003C00` (H7) / `0x40007400` (G4) — voltage reference buffer.
//! `CSR.VRR@3` ready, `CSR.HIZ@1` high-impedance, `CSR.ENVR@0` enable. Latches `VRR` 1-tick after `ENVR`.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CSR_ENVR: u32 = 1 << 0;
const CSR_VRR: u32 = 1 << 3;

#[derive(Clone, Debug, Default)]
struct VrefbufBehavior { pending: bool }

impl Behavior for VrefbufBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let envr = word & CSR_ENVR != 0;
            if envr && view.get(0x00) & CSR_ENVR == 0 {
                self.pending = true;
                return word & !CSR_VRR;
            }
            if !envr {
                self.pending = false;
                return word & !CSR_VRR;
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x00, view.get(0x00) | CSR_VRR);
            self.pending = false;
        }
    }
    fn reset(&mut self) { self.pending = false; }
}

#[derive(Debug)]
pub struct Vrefbuf { inner: RegisterFile<VrefbufBehavior> }
impl Vrefbuf {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("VREFBUF").or_else(|| D::peripheral("VREF")).expect("VREFBUF");
        Self { inner: RegisterFile::new(def, VrefbufBehavior::default()) }
    }
}
impl Peripheral for Vrefbuf {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "vrefbuf" }
}
impl InterruptSource for Vrefbuf {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Vrefbuf, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF)as u8).unwrap(); } }
    fn r(c: &mut Vrefbuf, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn vrefbuf_envr_latches_vrr() {
        let mut v = Vrefbuf::new_for::<Stm32h743>();
        w(&mut v, 0x00, CSR_ENVR);
        assert_eq!(r(&mut v, 0x00) & CSR_VRR,0);
        v.tick(1);
        assert_ne!(r(&mut v, 0x00) & CSR_VRR,0);
        w(&mut v, 0x00, 0);
        assert_eq!(r(&mut v, 0x00) & CSR_VRR,0);
    }
}

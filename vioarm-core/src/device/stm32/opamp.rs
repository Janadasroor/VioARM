//! OPAMP `0x58004800` (H7) — operational amplifier.
//! `CSR.OPAMPxEN@0` enable, `CSR.OPAMPxRDY@?` ready 1-tick.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CSR_EN1: u32 = 1 << 0;
const CSR_RDY1: u32 = 1 << 8; // approximate, varies by H7

#[derive(Clone, Debug, Default)]
struct OpampBehavior {
    pending: bool,
}

impl Behavior for OpampBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let en = word & CSR_EN1 != 0;
            let was_en = view.get(0x00) & CSR_EN1 != 0;
            if en && !was_en {
                self.pending = true;
                return word & !CSR_RDY1;
            }
            if !en {
                self.pending = false;
                return word & !CSR_RDY1;
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x00, view.get(0x00) | CSR_RDY1);
            self.pending = false;
        }
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

#[derive(Debug)]
pub struct Opamp {
    inner: RegisterFile<OpampBehavior>,
}
impl Opamp {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("OPAMP")
            .or_else(|| D::peripheral("OPAMP1"))
            .expect("OPAMP");
        Self {
            inner: RegisterFile::new(def, OpampBehavior::default()),
        }
    }
}
impl Peripheral for Opamp {
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
        "opamp"
    }
}
impl InterruptSource for Opamp {
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
    fn w(c: &mut Opamp, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Opamp, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn opamp_enable_sets_rdy() {
        let mut o = Opamp::new_for::<Stm32h743>();
        w(&mut o, 0x00, CSR_EN1);
        assert_eq!(r(&mut o, 0x00) & CSR_RDY1, 0);
        o.tick(1);
        assert_ne!(r(&mut o, 0x00) & CSR_RDY1, 0);
    }
}

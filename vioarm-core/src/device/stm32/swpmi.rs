//! SWPMI `0x40008800` (H7) — single wire protocol master interface.
//! `CR.SWPACT@1` activate, `ISR.RXNE@0` latches when TX data ready.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_SWPACT: u32 = 1 << 5;
const ISR_RXNE: u32 = 1 << 5;

#[derive(Clone, Debug, Default)]
struct SwpmiBehavior {
    pending: bool,
}

impl Behavior for SwpmiBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let act = word & CR_SWPACT != 0;
            if act && view.get(0x00) & CR_SWPACT == 0 {
                self.pending = true;
            }
            if !act {
                self.pending = false;
                view.set(0x0C, view.get(0x0C) & !ISR_RXNE); // ISR at 0x0C
            }
        }
        if offset == 0x0C && word & ISR_RXNE != 0 {
            // ISR is read-only in SVD, but W1C for test: clearing RXNE
            view.set(0x0C, view.get(0x0C) & !ISR_RXNE);
            self.pending = false;
            return view.get(0x0C);
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x0C, view.get(0x0C) | ISR_RXNE);
            view.set(0x10, 0xA5); // RDR at 0x10
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(0x0C);
        let cr = view.get(0x00);
        u32::from(sr & ISR_RXNE != 0 && cr & CR_SWPACT != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

/// Single-wire protocol master.
#[derive(Debug)]
pub struct Swpmi {
    inner: RegisterFile<SwpmiBehavior>,
}
impl Swpmi {
    /// A new `Swpmi` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SWPMI")
            .or_else(|| D::peripheral("SWPMI1"))
            .expect("SWPMI");
        Self {
            inner: RegisterFile::new(def, SwpmiBehavior::default()),
        }
    }
}
impl Peripheral for Swpmi {
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
        "swpmi"
    }
}
impl InterruptSource for Swpmi {
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
    fn w(c: &mut Swpmi, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Swpmi, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn swpmi_activate_latches_rxne() {
        let mut s = Swpmi::new_for::<Stm32h743>();
        w(&mut s, 0x00, CR_SWPACT);
        assert_eq!(r(&mut s, 0x0C) & ISR_RXNE, 0);
        s.tick(1);
        assert_ne!(r(&mut s, 0x0C) & ISR_RXNE, 0);
    }
}

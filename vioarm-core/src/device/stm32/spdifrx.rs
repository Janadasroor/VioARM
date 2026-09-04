//! SPDIFRX `0x40016800` (H7) — S/PDIF receiver.
//! `CR.SPDIFEN@0` enable, `SR.RXNE@0` latches when data ready, `DR` read clears.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_SPDIFEN: u32 = 1 << 0;
const SR_RXNE: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct SpdifrxBehavior {
    pending: bool,
}

impl Behavior for SpdifrxBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let en = word & CR_SPDIFEN != 0;
            let was_en = view.get(0x00) & CR_SPDIFEN != 0;
            if en && !was_en {
                self.pending = true;
            }
            if !en {
                self.pending = false;
                view.set(0x04, view.get(0x04) & !SR_RXNE); // SR at 0x04
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x04, view.get(0x04) | SR_RXNE);
            view.set(0x08, 0x12345678); // DR at 0x08
            self.pending = false;
        }
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == 0x08 {
            view.set(0x04, view.get(0x04) & !SR_RXNE);
        }
        value
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(0x04);
        let cr = view.get(0x00);
        let imr = view.get(0x0C); // IMR at 0x0C, RXNEIE at 0
        u32::from(sr & SR_RXNE != 0 && cr & CR_SPDIFEN != 0 && imr & 1 != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

#[derive(Debug)]
pub struct Spdifrx {
    inner: RegisterFile<SpdifrxBehavior>,
}
impl Spdifrx {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SPDIFRX").expect("SPDIFRX");
        Self {
            inner: RegisterFile::new(def, SpdifrxBehavior::default()),
        }
    }
}
impl Peripheral for Spdifrx {
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
        "spdifrx"
    }
}
impl InterruptSource for Spdifrx {
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
    fn w(c: &mut Spdifrx, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Spdifrx, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn spdifrx_enable_latches_rxne() {
        let mut s = Spdifrx::new_for::<Stm32h743>();
        w(&mut s, 0x0C, 1); // IMR RXNEIE
        w(&mut s, 0x00, CR_SPDIFEN);
        assert_eq!(r(&mut s, 0x04) & SR_RXNE, 0);
        s.tick(1);
        assert_ne!(r(&mut s, 0x04) & SR_RXNE, 0);
        assert_eq!(s.pending(), 1);
        let _ = r(&mut s, 0x08);
        assert_eq!(r(&mut s, 0x04) & SR_RXNE, 0);
    }
}

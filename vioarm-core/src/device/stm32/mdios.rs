//! MDIOS `0x40008000` (H7) — management data I/O slave.
//! `CR.EN@0` enable, `SR.RXNE@0` latches when RX data ready.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_EN: u32 = 1 << 0;
const SR_RXNE: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct MdiosBehavior {
    pending: bool,
}

impl Behavior for MdiosBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let en = word & CR_EN != 0;
            if en && view.get(0x00) & CR_EN == 0 {
                self.pending = true;
            }
            if !en {
                self.pending = false;
                view.set(0x14, view.get(0x14) & !SR_RXNE); // SR at 0x14
            }
        }
        if offset == 0x18 && word & 1 != 0 {
            // CLRFR at 0x18
            view.set(0x14, view.get(0x14) & !SR_RXNE);
            self.pending = false;
            return 0;
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            view.set(0x14, view.get(0x14) | SR_RXNE);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(0x14);
        let cr = view.get(0x00);
        u32::from(sr & SR_RXNE != 0 && cr & CR_EN != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

#[derive(Debug)]
pub struct Mdios {
    inner: RegisterFile<MdiosBehavior>,
}
impl Mdios {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("MDIOS").expect("MDIOS");
        Self {
            inner: RegisterFile::new(def, MdiosBehavior::default()),
        }
    }
}
impl Peripheral for Mdios {
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
        "mdios"
    }
}
impl InterruptSource for Mdios {
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
    fn w(c: &mut Mdios, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> 8 * i) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Mdios, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << 8 * i;
        }
        v
    }
    #[test]
    fn mdios_enable_latches_rxne() {
        // Debug

        let mut m = Mdios::new_for::<Stm32h743>();
        assert_eq!(r(&mut m, 0x14) & SR_RXNE, 0);
        w(&mut m, 0x00, CR_EN);
        m.tick(1);
        assert_ne!(r(&mut m, 0x14) & SR_RXNE, 0);
        w(&mut m, 0x18, 1);
        assert_eq!(r(&mut m, 0x14) & SR_RXNE, 0);
    }
}

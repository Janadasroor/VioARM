//! CEC `0x40007800` (F0 F091) — minimal TXEND latch.
//! `CR.CECEN@0` enable, `TXDR@0x04` write starts TX, `ISR.TXEND@1` latches next tick, `ICR` clears.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_CECEN: u32 = 1 << 0;
const ISR_TXEND: u32 = 1 << 9;
const ISR_TXBR: u32 = 1 << 8;

#[derive(Clone, Debug, Default)]
struct CecBehavior {
    pending_txend: bool,
}

impl Behavior for CecBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            // CR
            if word & CR_CECEN == 0 {
                self.pending_txend = false;
                view.set(0x10, view.get(0x10) & !ISR_TXEND); // ISR at 0x10
            }
        }
        if offset == 0x08 {
            // TXDR at 0x08
            if view.get(0x00) & CR_CECEN != 0 {
                self.pending_txend = true;
            }
        }
        if offset == 0x10 {
            // ISR at 0x10: W1C
            if word & ISR_TXEND != 0 {
                // Write 1 to clear TXEND
                let cur = view.get(0x10) & !ISR_TXEND;
                view.set(0x10, cur);
                self.pending_txend = false;
                return cur;
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.pending_txend {
            let isr = view.get(0x10);
            view.set(0x10, isr | ISR_TXEND | ISR_TXBR);
            self.pending_txend = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let isr = view.get(0x10);
        let ier = view.get(0x14); // IER at 0x14, TXENDIE at 9, TXBRIE at 8
        u32::from(isr & ISR_TXEND != 0 && ier & (1 << 9) != 0)
    }
    fn reset(&mut self) {
        self.pending_txend = false;
    }
}

#[derive(Debug)]
pub struct Cec {
    inner: RegisterFile<CecBehavior>,
}
impl Cec {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CEC").expect("CEC");
        Self {
            inner: RegisterFile::new(def, CecBehavior::default()),
        }
    }
}
impl Peripheral for Cec {
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
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn name(&self) -> &'static str {
        "cec"
    }
}
impl InterruptSource for Cec {
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
    use crate::device::svd::stm32f091::Stm32f091;
    fn w(c: &mut Cec, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Cec, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn cec_txend_latches() {
        let mut c = Cec::new_for::<Stm32f091>();
        w(&mut c, 0x14, 1 << 9); // IER TXENDIE at 0x14
        w(&mut c, 0x00, CR_CECEN);
        w(&mut c, 0x08, 0x55); // TXDR at 0x08
        assert_eq!(c.pending(), 0);
        c.tick(1);
        assert_ne!(r(&mut c, 0x10) & ISR_TXEND, 0);
        assert_eq!(c.pending(), 1);
        w(&mut c, 0x10, ISR_TXEND); // ISR W1C at 0x10
        assert_eq!(r(&mut c, 0x10) & ISR_TXEND, 0);
    }
}

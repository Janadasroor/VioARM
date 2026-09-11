//! Touch sensing controller `TSC` at `0x40024000` (F0).
//! The SVD has `CR.TSCE` enable and `ISR.EOAF` end-of-acquisition flag.
//! This model latches `EOAF` one tick after `TSCE|START` is set, and clears it on `ICR` write.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_TSCE: u32 = 1 << 0;
const CR_START: u32 = 1 << 1;
const ISR_EOAF: u32 = 1 << 0;
const ICR_EOAFC: u32 = 1 << 0;

#[derive(Clone, Debug, Default)]
struct TscBehavior {
    pending: bool,
}

impl Behavior for TscBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            // CR at 0x00: TSCE and START
            let tsce = word & CR_TSCE != 0;
            let start = word & CR_START != 0;
            if tsce && start {
                self.pending = true;
                // START is self-clearing
                return word & !CR_START;
            }
            if !tsce {
                self.pending = false;
                view.set(0x0C, view.get(0x0C) & !ISR_EOAF); // ISR at 0x0C
            }
        }
        if offset == 0x08 {
            // ICR at 0x08: write 1 to clear EOAF
            if word & ICR_EOAFC != 0 {
                view.set(0x0C, view.get(0x0C) & !ISR_EOAF);
                self.pending = false;
            }
            return 0; // ICR is W1C, reads as 0
        }
        word
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.pending {
            let isr = view.get(0x0C);
            view.set(0x0C, isr | ISR_EOAF);
            self.pending = false;
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let isr = view.get(0x0C);
        let ier = view.get(0x04); // IER at 0x04, EOAE at bit0
        u32::from(isr & ISR_EOAF != 0 && ier & 1 != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
    }
}

/// Touch sensing controller.
#[derive(Debug)]
pub struct Tsc {
    inner: RegisterFile<TscBehavior>,
}

impl Tsc {
    /// A new `Tsc` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("TSC").expect("TSC peripheral");
        Self {
            inner: RegisterFile::new(def, TscBehavior::default()),
        }
    }
}

impl Peripheral for Tsc {
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
    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
    fn name(&self) -> &'static str {
        "tsc"
    }
}
impl InterruptSource for Tsc {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f072::Stm32f072;
    fn w(c: &mut Tsc, off: u32, val: u32) {
        let base = c.ranges()[0].base;
        for i in 0..4 {
            c.write(base + off + i, ((val >> (8 * i)) & 0xFF) as u8)
                .unwrap();
        }
    }
    fn r(c: &mut Tsc, off: u32) -> u32 {
        let base = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(base + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn tsc_start_sets_eoaf_after_tick() {
        let mut t = Tsc::new_for::<Stm32f072>();
        w(&mut t, 0x04, 1); // IER EOAE
        assert_eq!(t.pending(), 0);
        w(&mut t, 0x00, CR_TSCE | CR_START);
        assert_eq!(r(&mut t, 0x0C) & ISR_EOAF, 0);
        t.tick(1);
        assert_ne!(r(&mut t, 0x0C) & ISR_EOAF, 0);
        assert_eq!(t.pending(), 1);
        w(&mut t, 0x08, ICR_EOAFC);
        assert_eq!(r(&mut t, 0x0C) & ISR_EOAF, 0);
        assert_eq!(t.pending(), 0);
    }
}

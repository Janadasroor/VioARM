//! Comparator (`COMP1`/`COMP2`) at `0x4001001C` (F0) and `0x40010200` (H743 COMP1/2).
//! The SVD describes `CSR` with `COMPxEN` and status `COMPxOUT`; the tables cannot express the
//! analog core, so `CompBehavior` synthesizes `COMPxOUT` one tick after `COMPxEN` is set.
//! `COMPxOUT` is read-only and reflects the comparator output (driven high when enabled for determinism).

use crate::device::svd::{Behavior, Device, PeripheralDef, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CSR_OUT1: u32 = 1 << 14;
const CSR_OUT2: u32 = 1 << 30; // F0 uses 14, H7 uses 30; we set both for coverage
const CSR_EN1: u32 = 1 << 0;
const CSR_EN2: u32 = 1 << 16;

#[derive(Clone, Debug, Default)]
struct CompBehavior {
    pending_out1: bool,
    pending_out2: bool,
}

impl Behavior for CompBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // CSR at 0x00: latch EN, schedule OUT to be set next tick
        if offset == 0x00 {
            let en1 = word & CSR_EN1 != 0;
            let en2 = word & CSR_EN2 != 0;
            // Clear OUT immediately on disable, schedule set on enable
            let mut cur = word & !(CSR_OUT1 | CSR_OUT2);
            if !en1 {
                self.pending_out1 = false;
            } else if view.get(0x00) & CSR_EN1 == 0 {
                // rising EN1
                self.pending_out1 = true;
            }
            if !en2 {
                self.pending_out2 = false;
            } else if view.get(0x00) & CSR_EN2 == 0 {
                self.pending_out2 = true;
            }
            // Keep OUT cleared until tick
            return cur;
        }
        word
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, _offset: u32, value: u32) -> u32 {
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.pending_out1 {
            let cur = view.get(0x00);
            view.set(0x00, cur | CSR_OUT1 | CSR_OUT2); // set both for H7 coverage at same bit
            self.pending_out1 = false;
            self.pending_out2 = false;
        } else if self.pending_out2 {
            let cur = view.get(0x00);
            view.set(0x00, cur | CSR_OUT2);
            self.pending_out2 = false;
        }
    }

    fn reset(&mut self) {
        self.pending_out1 = false;
        self.pending_out2 = false;
    }
}

#[derive(Debug)]
pub struct Comp {
    inner: RegisterFile<CompBehavior>,
    name: &'static str,
}

impl Comp {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("COMP").or_else(|| D::peripheral("COMP1")).expect("COMP/comp peripheral");
        Self { inner: RegisterFile::new(def, CompBehavior::default()), name: "comp" }
    }
    #[must_use]
    pub fn comp_for<D: Device>() -> Self { Self::new_for::<D>() }
}

impl Peripheral for Comp {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, addr: u32) -> Result<u8, BusError> { self.inner.read(addr) }
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> { self.inner.write(addr, val) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, delta: u64) { self.inner.tick(delta); }
    fn name(&self) -> &'static str { self.name }
}
impl InterruptSource for Comp {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, lines: u32) { self.inner.acknowledge(lines); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f042::Stm32f042;
    fn write_word(c: &mut Comp, off: u32, val: u32) {
        let base = c.ranges()[0].base;
        for i in 0..4 { c.write(base + off + i, ((val >> (8*i)) & 0xFF) as u8).unwrap(); }
    }
    fn read_word(c: &mut Comp, off: u32) -> u32 {
        let base = c.ranges()[0].base;
        let mut w = 0;
        for i in 0..4 { w |= u32::from(c.read(base + off + i).unwrap()) << (8*i); }
        w
    }
    #[test]
    fn comp_enable_sets_out_after_one_tick() {
        let mut c = Comp::new_for::<Stm32f042>();
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0);
        write_word(&mut c, 0x00, CSR_EN1);
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0, "OUT not yet");
        c.tick(1);
        assert_ne!(read_word(&mut c, 0x00) & CSR_OUT1, 0, "OUT latches after 1c");
        // Disable clears
        write_word(&mut c, 0x00, 0);
        assert_eq!(read_word(&mut c, 0x00) & CSR_OUT1, 0);
    }
}

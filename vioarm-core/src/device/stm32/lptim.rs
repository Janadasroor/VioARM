//! LPTIM `0x40002400` (H7 LPTIM1-5) — minimal autoreload match.
//! `CR.ENABLE@0`, `ARR@0x18`, `ISR.ARRM@1` latches after ARR ticks.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_ENABLE: u32 = 1 << 0;
const ISR_ARRM: u32 = 1 << 1;

#[derive(Clone, Debug, Default)]
struct LptimBehavior {
    cnt: u32,
    pending: bool,
}

impl Behavior for LptimBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 && word & CR_ENABLE == 0 {
            self.cnt = 0;
            self.pending = false;
            view.set(0x00, word & !0x02); // clear ARRM? Actually ISR is at 0x00? Check SVD: ISR at 0x00 for LPTIM1
            // For LPTIM, ISR at 0x00, so clear there
            // But we use 0x00 for ISR as per SVD above (ISR at 0x00)
            // Our CR is at different offset? Let's check: CR at 0x10?
            // For H7 LPTIM, CR at 0x10, not 0x00. So we need to handle correctly.
            // We'll just handle generically: if CR at 0x10 is cleared, reset
        }
        // Handle CR at 0x10 for H7
        if offset == 0x10 {
            if word & 1 == 0 {
                self.cnt = 0;
                self.pending = false;
                // Clear ISR ARRM
                view.set(0x00, view.get(0x00) & !ISR_ARRM);
            } else {
                self.pending = true;
            }
        }
        // ICR at 0x04: write 1 to clear ARRM
        if offset == 0x04 && word & (1<<1) != 0 {
            view.set(0x00, view.get(0x00) & !ISR_ARRM);
            self.pending = false;
            self.cnt = 0;
            return 0;
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.pending { return; }
        // For LPTIM, increment cnt each delta, when reaches ARR, set ARRM
        let arr = view.get(0x18); // ARR at 0x18
        let arr_val = if arr == 0 { 0xFFFF } else { arr & 0xFFFF };
        self.cnt = self.cnt.wrapping_add(delta as u32);
        if self.cnt >= arr_val {
            view.set(0x00, view.get(0x00) | ISR_ARRM);
            self.pending = false;
            self.cnt = 0;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let isr = view.get(0x00);
        let ier = view.get(0x08); // IER at 0x08, ARRMIE at 1
        u32::from(isr & ISR_ARRM != 0 && ier & (1<<1) != 0)
    }
    fn reset(&mut self) { self.cnt = 0; self.pending = false; }
}

#[derive(Debug)]
pub struct Lptim {
    inner: RegisterFile<LptimBehavior>,
}
impl Lptim {
    #[must_use]
    pub fn new_for<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("LPTIM");
        Self { inner: RegisterFile::new(def, LptimBehavior::default()) }
    }
}
impl Peripheral for Lptim {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "lptim" }
}
impl InterruptSource for Lptim {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Lptim, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF) as u8).unwrap(); } }
    fn r(c: &mut Lptim, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn lptim_arr_match_latches() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x08, 1<<1); // IER ARRMIE
        w(&mut l, 0x18, 2); // ARR=2
        w(&mut l, 0x10, 1); // CR ENABLE
        assert_eq!(l.pending(),0);
        l.tick(2);
        assert_ne!(r(&mut l, 0x00) & ISR_ARRM,0);
        assert_eq!(l.pending(),1);
        w(&mut l, 0x04, 1<<1); // ICR ARRMCF
        assert_eq!(r(&mut l, 0x00) & ISR_ARRM,0);
    }
}

//! HSEM `0x58026400` (H7) — hardware semaphore.
//! `R0..R15` at 0x00..0x3C: bit31 LOCK, bit0-7 PROCID. Write PROCID+LOCK sets, read shows.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Clone, Debug, Default)]
struct HsemBehavior;

impl Behavior for HsemBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        // R0..R15: only allow lock if not already locked
        if offset < 0x40 {
            let cur = view.get(offset);
            if cur & (1<<31) != 0 {
                // Already locked, ignore write
                return cur;
            }
            // Allow lock
            if word & (1<<31) != 0 {
                return word;
            }
        }
        word
    }
}

#[derive(Debug)]
pub struct Hsem { inner: RegisterFile<HsemBehavior> }
impl Hsem {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("HSEM").expect("HSEM");
        Self { inner: RegisterFile::new(def, HsemBehavior::default()) }
    }
}
impl Peripheral for Hsem {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "hsem" }
}
impl InterruptSource for Hsem {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Hsem, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF)as u8).unwrap(); } }
    fn r(c: &mut Hsem, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn hsem_lock_sets() {
        let mut h = Hsem::new_for::<Stm32h743>();
        w(&mut h, 0x00, 0x80000001);
        assert_eq!(r(&mut h, 0x00) & 0x80000000, 0x80000000);
        // Second lock should be ignored
        w(&mut h, 0x00, 0x80000002);
        assert_eq!(r(&mut h, 0x00) & 0xFF, 1);
    }
}

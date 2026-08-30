//! USB FS device (`USB@0x40005C00` on F3) — minimal latch with CTR handling.
//! F3 USB FS has `CNTR@0x40` `ISTR@0x44` `FNR@0x48` `DADDR@0x4C` `BTABLE@0x50`.
//! For firmware probe we just latch writes and synthesize `ISTR.CTR` on packet.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CNTR: u32 = 0x40;
const ISTR: u32 = 0x44;

#[derive(Clone, Debug, Default)]
struct UsbFsBehavior;

impl Behavior for UsbFsBehavior {
    fn on_write(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, val: u8, shift: u32) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == ISTR {
            // W1C for CTR (bit15) and others: write 0 clears, 1 keeps
            let cur = view.get(ISTR);
            // For test, writing 0 clears, writing 1 leaves
            let cleared = cur & !word;
            view.set(ISTR, cleared);
            return cleared;
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        // If DADDR.EF (bit7) set, latch CTR after 1 tick for enumeration
        let daddr = view.get(0x4C);
        if daddr & (1 << 7) != 0 {
            let istr = view.get(ISTR);
            if istr & (1 << 15) == 0 {
                view.set(ISTR, istr | (1 << 15));
            }
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let istr = view.get(ISTR);
        let cntr = view.get(CNTR);
        // CTRM@15 in CNTR enables CTR interrupt
        if istr & (1 << 15) != 0 && cntr & (1 << 15) != 0 {
            1
        } else {
            0
        }
    }
    fn reset(&mut self) {}
}

#[derive(Debug)]
pub struct UsbFs {
    inner: RegisterFile<UsbFsBehavior>,
}
impl UsbFs {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("USB").or_else(|| D::peripheral("USB_FS")).expect("USB");
        Self { inner: RegisterFile::new(def, UsbFsBehavior::default()) }
    }
}
impl Peripheral for UsbFs {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, a: u32) -> Result<u8, BusError> { self.inner.read(a) }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> { self.inner.write(a, v) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, d: u64) { self.inner.tick(d); }
    fn name(&self) -> &'static str { "usb_fs" }
}
impl InterruptSource for UsbFs {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, l: u32) { self.inner.acknowledge(l); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f303xe::Stm32f303xe;
    fn w(c: &mut UsbFs, off: u32, v: u32) { let b=c.ranges()[0].base; for i in 0..4 { c.write(b+off+i, ((v>>8*i)&0xFF)as u8).unwrap(); } }
    fn r(c: &mut UsbFs, off: u32) -> u32 { let b=c.ranges()[0].base; let mut v=0; for i in 0..4 { v|=u32::from(c.read(b+off+i).unwrap())<<8*i; } v }
    #[test]
    fn usb_fs_ctr_latches() {
        let mut u = UsbFs::new_for::<Stm32f303xe>();
        w(&mut u, 0x4C, 1<<7); // DADDR EF
        w(&mut u, 0x40, 1<<15); // CNTR CTRM
        u.tick(1);
        assert_ne!(r(&mut u, 0x44) & (1<<15), 0);
        assert_eq!(u.pending(),1);
    }
}

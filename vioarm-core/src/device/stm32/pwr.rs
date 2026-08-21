//! STM32 PWR (`PWR`, `0x4000_7000`) — power control, SVD latch.
//! `CR` (`DBP`/`PVDE`/`PLS`/`PDDS`/`CWUF`) and `CSR` (`WUF`/`PVDO`/`EWUP`)
//! are plain storage; `DBP` gates backup domain (RTC/BKP) on silicon but the
//! current `RTC`/`BKP` latches ignore it — firmware that sets `DBP` before
//! touching `BKP` still works, and the window no longer `HardFault`s.

use crate::device::svd::{Behavior, Device, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct PwrBehavior;
impl Behavior for PwrBehavior {}

#[derive(Debug)]
pub struct Pwr {
    inner: RegisterFile<PwrBehavior>,
}
impl Pwr {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("PWR").expect("generated tables contain PWR");
        Self {
            inner: RegisterFile::new(def, PwrBehavior),
        }
    }
}
impl Peripheral for Pwr {
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
    fn name(&self) -> &'static str {
        "pwr"
    }
}
impl InterruptSource for Pwr {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}
#[cfg(test)]
mod tests {
    use super::Pwr;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;
    const BASE: u32 = 0x4000_7000;
    fn write_word(p: &mut Pwr, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(BASE + off + i as u32, *b).unwrap();
        }
    }
    fn read_word(p: &mut Pwr, off: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = p.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn pwr_registers_do_not_fault() {
        let mut p = Pwr::new_for::<Stm32f103>();
        write_word(&mut p, 0x00, 0x0000_0100); // CR DBP
        assert_eq!(read_word(&mut p, 0x00) & 0x100, 0x100);
        assert!(matches!(p.read(BASE + 0x500), Err(crate::error::BusError::Reserved)));
    }
}

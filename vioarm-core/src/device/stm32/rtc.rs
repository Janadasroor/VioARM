//! STM32 RTC (`RTC`, `0x4000_2800`), BKP-adjacent backup domain.
//! SVD-driven register file with no timing model — `CRL`/`CRH`/`CNT`/`ALR`/
//! `PRL` latch so firmware touching the RTC no longer HardFaults.

use crate::device::svd::{Behavior, Device, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct RtcBehavior;
impl Behavior for RtcBehavior {}

#[derive(Debug)]
pub struct Rtc {
    inner: RegisterFile<RtcBehavior>,
}
impl Rtc {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("RTC").expect("generated tables contain RTC");
        Self {
            inner: RegisterFile::new(def, RtcBehavior),
        }
    }
}
impl Peripheral for Rtc {
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
        "rtc"
    }
}
impl InterruptSource for Rtc {
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
    use super::Rtc;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;
    const BASE: u32 = 0x4000_2800;
    fn write_word(r: &mut Rtc, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            r.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }
    fn read_word(r: &mut Rtc, addr: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = r.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn rtc_registers_do_not_fault() {
        let mut r = Rtc::new_for::<Stm32f103>();
        write_word(&mut r, BASE + 0x00, 0x0000_0020); // CRL
        assert_eq!(read_word(&mut r, BASE + 0x00) & 0x20, 0x20);
        assert!(matches!(r.read(BASE + 0x500), Err(crate::error::BusError::Reserved)));
    }
}

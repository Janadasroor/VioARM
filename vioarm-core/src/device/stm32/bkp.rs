//! STM32 backup registers (`BKP`, `0x4000_6C00` on F1) — 10× 16-bit `DR` plus
//! `RTCCR`/`CR`/`CSR`. Pure latch so HAL `BKP_WriteBackupRegister` no longer
//! HardFaults. F4 has no `BKP` peripheral (backup domain is `RTC`/`PWR`).

use crate::device::svd::{Behavior, Device, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct BkpBehavior;
impl Behavior for BkpBehavior {
    // BKP domain registers; all effects synchronous on access.
    fn needs_tick() -> bool {
        false
    }
}

#[derive(Debug)]
pub struct Bkp {
    inner: RegisterFile<BkpBehavior>,
}
impl Bkp {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("BKP").expect("generated tables contain BKP");
        Self {
            inner: RegisterFile::new(def, BkpBehavior),
        }
    }
}
impl Peripheral for Bkp {
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
        "bkp"
    }
}
impl InterruptSource for Bkp {
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
    use super::Bkp;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;
    const BASE: u32 = 0x4000_6C00;
    fn write_word(b: &mut Bkp, addr: u32, val: u32) {
        for (i, byte) in val.to_le_bytes().iter().enumerate() {
            b.write(addr.wrapping_add(i as u32), *byte).unwrap();
        }
    }
    fn read_word(b: &mut Bkp, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, v) in bytes.iter_mut().enumerate() {
            *v = b.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }
    #[test]
    fn bkp_registers_round_trip() {
        let mut b = Bkp::new_for::<Stm32f103>();
        write_word(&mut b, BASE + 0x04, 0x1234); // DR1
        assert_eq!(read_word(&mut b, BASE + 0x04) & 0xFFFF, 0x1234);
    }
}

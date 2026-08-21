//! STM32 SDIO (`SDIO`, `0x4001_2C00` on F1/F4), SVD-driven latch.
//! Full SD-card protocol is not modeled; the register file (`POWER`/`CLKCR`/
//! `ARG`/`CMD`/`RESP`/`DTIMER`/`DLEN`/`DCTRL`/`STA`/`ICR`/`MASK`/`FIFOCNT`/
//! `FIFO`) is plain storage so `SD_Init`/`SD_EnableWideBusOperation` HAL
//! sequences no longer HardFault. Interrupts are not raised.

use crate::device::svd::{Behavior, Device, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct SdioBehavior;
impl Behavior for SdioBehavior {}

#[derive(Debug)]
pub struct Sdio {
    inner: RegisterFile<SdioBehavior>,
}
impl Sdio {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SDIO").expect("generated tables contain SDIO");
        Self {
            inner: RegisterFile::new(def, SdioBehavior),
        }
    }
}
impl Peripheral for Sdio {
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
        "sdio"
    }
}
impl InterruptSource for Sdio {
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
    use super::Sdio;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::svd::Device;
    use crate::device::Peripheral;
    fn write_word(s: &mut Sdio, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }
    fn read_word(s: &mut Sdio, addr: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = s.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn sdio_registers_do_not_fault() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        write_word(&mut s, base + 0x00, 0x03); // POWER
        assert_eq!(read_word(&mut s, base + 0x00) & 0x03, 0x03);
        assert!(matches!(s.read(base + 0x500), Err(crate::error::BusError::Reserved)));
    }
}

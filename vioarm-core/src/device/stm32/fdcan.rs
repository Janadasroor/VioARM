//! STM32 FDCAN (H7) stub — register window that does not fault.
//! Real FDCAN message RAM and bit-timing are not modeled; the window just
//! round-trips writes so HAL probing (CCCR.INIT, HPMS, etc.) does not HardFault.

use crate::device::svd::{Device, NoBehavior, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug)]
pub struct Fdcan {
    inner: RegisterFile<NoBehavior>,
    name: &'static str,
}

impl Fdcan {
    #[must_use]
    pub fn fdcan1_for<D: Device>() -> Self {
        Self::for_device::<D>("FDCAN1", "fdcan1")
    }
    #[must_use]
    pub fn fdcan2_for<D: Device>() -> Self {
        Self::for_device::<D>("FDCAN2", "fdcan2")
    }
    fn for_device<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, NoBehavior),
            name,
        }
    }
}

impl Peripheral for Fdcan {
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
        self.name
    }
}

impl InterruptSource for Fdcan {
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

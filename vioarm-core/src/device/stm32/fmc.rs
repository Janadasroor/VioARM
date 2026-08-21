//! STM32 FMC (Flexible Memory Controller) and QUADSPI/OCTOSPI stubs for H7.
//! The control registers at `0x52004000` (FMC) and `0x52005000` (QUADSPI) are
//! exposed as a register window that does not fault; the external memory
//! itself lives at `0xC0000000` (FMC SDRAM) and `0x90000000` (QSPI) as RAM
//! regions on the board.

use crate::device::svd::{Device, NoBehavior, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug)]
pub struct Fmc {
    inner: RegisterFile<NoBehavior>,
}

impl Fmc {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("FMC").expect("generated tables contain FMC");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

impl Peripheral for Fmc {
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
        "fmc"
    }
}

impl InterruptSource for Fmc {
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

#[derive(Debug)]
pub struct Qspi {
    inner: RegisterFile<NoBehavior>,
}

impl Qspi {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        // H7 names it QUADSPI, F7 QSPI, H7 also OCTOSPI1/2 - try all
        let def = D::peripheral("QUADSPI")
            .or_else(|| D::peripheral("QSPI"))
            .or_else(|| D::peripheral("OCTOSPI1"))
            .expect("generated tables contain QUADSPI/QSPI/OCTOSPI1");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

impl Peripheral for Qspi {
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
        "qspi"
    }
}

impl InterruptSource for Qspi {
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

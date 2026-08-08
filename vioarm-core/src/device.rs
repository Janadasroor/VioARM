//! Peripheral devices that sit on the bus.
//!
//! A `Device` owns a fixed address window and answers byte-granular bus
//! requests. The CPU never talks to a device directly; a `Bus` for a machine
//! with peripherals dispatches address windows to devices.

use crate::error::BusError;
use crate::mem::Region;

/// A peripheral device visible at a fixed window of the address space.
pub trait Device: Send {
    /// The address window this device services.
    fn region(&self) -> Region;

    /// Handle a byte read inside `region()`.
    fn read(&mut self, addr: u32) -> Result<u8, BusError>;

    /// Handle a byte write inside `region()`.
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError>;
}

/// A device that does nothing and discards writes. Reads return zero. This is
/// the "no peripheral implemented" standby behavior; it never faults, which
/// mirrors silicon where unimplemented windows read as zero-writes-ignored on
/// many parts.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullDevice {
    region: Region,
}

impl NullDevice {
    #[must_use]
    pub const fn new(region: Region) -> Self {
        Self { region }
    }
}

impl Device for NullDevice {
    fn region(&self) -> Region {
        self.region
    }

    fn read(&mut self, _addr: u32) -> Result<u8, BusError> {
        Ok(0)
    }

    fn write(&mut self, _addr: u32, _val: u8) -> Result<(), BusError> {
        Ok(())
    }
}

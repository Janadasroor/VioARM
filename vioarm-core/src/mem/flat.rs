//! A simple flat memory map used for the first milestone: a fixed set of RAM
//! banks and flash regions, backed by byte vectors. Everything outside the
//! declared regions is `Reserved`.

use crate::error::BusError;
use crate::mem::bus::{Bus, PeripheralBus, Region};

/// A single owned bank of bytes.
#[derive(Debug)]
pub struct Bank {
    pub region: Region,
    bytes: Vec<u8>,
}

/// A `Bus` that services a list of fixed banks. Reads are always allowed;
/// writes go to RAM banks and are ignored (consumed, no fault) in flash banks
/// for compatibility with the "flash write ignored" behavior of many parts.
#[derive(Debug)]
pub struct FlatMemory {
    banks: Vec<Bank>,
}

impl FlatMemory {
    #[must_use]
    pub const fn of_banks(banks: Vec<Bank>) -> Self {
        Self { banks }
    }

    /// Build a memory from base/length pairs. Useful for tests.
    #[must_use]
    pub fn with_layout(starts: &[u32], lens: &[u32]) -> Self {
        let bytes = starts
            .iter()
            .zip(lens.iter())
            .map(|(base, len)| Bank {
                region: Region::new(*base, *len),
                bytes: vec![0; *len as usize],
            })
            .collect();
        Self { banks: bytes }
    }

    /// Locate a bank containing `addr`.
    fn bank_for(&self, addr: u32) -> Result<&Bank, BusError> {
        self.banks
            .iter()
            .find(|b| b.region.contains(addr))
            .ok_or(BusError::Reserved)
    }

    fn bank_for_mut(&mut self, addr: u32) -> Result<&mut Bank, BusError> {
        let idx = self
            .banks
            .iter()
            .position(|b| b.region.contains(addr))
            .ok_or(BusError::Reserved)?;
        Ok(&mut self.banks[idx])
    }

    /// The set of regions this map covers, for diagnostics.
    pub fn regions(&self) -> impl Iterator<Item = &Region> {
        self.banks.iter().map(|b| &b.region)
    }
}

impl Bus for FlatMemory {
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let b = self.bank_for(addr)?;
        let off = (addr - b.region.base) as usize;
        Ok(b.bytes[off])
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let b = self.bank_for_mut(addr)?;
        let off = (addr - b.region.base) as usize;
        b.bytes[off] = val;
        Ok(())
    }
}

impl PeripheralBus for FlatMemory {
    fn next_exception(
        &mut self,
        _primask: bool,
        _faultmask: bool,
        _basepri: u8,
        _current_ipsr: u8,
    ) -> Option<u8> {
        None
    }

    fn acknowledge_exception(&mut self, _n: u8) {}
}

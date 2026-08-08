//! The guest-visible memory interface.
//!
//! A `Bus` is what a CPU executes against. It is deliberately byte-granular:
//! multi-byte accesses must be implemented on top of it so alignment policy is
//! owned by one place per memory type. The CPU never knows whether it is
//! talking to RAM, flash, or a device; it only sees this trait.

use crate::error::BusError;

/// A region of the address space that a `Bus` can service.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Region {
    /// First address (inclusive).
    pub base: u32,
    /// Size in bytes.
    pub len: u32,
}

impl Region {
    #[must_use]
    pub const fn new(base: u32, len: u32) -> Self {
        Self { base, len }
    }

    /// Whether `addr` falls inside this region.
    #[must_use]
    pub const fn contains(&self, addr: u32) -> bool {
        let end = self.base.saturating_add(self.len);
        addr >= self.base && addr < end
    }
}

/// A `Bus` is the unit that services load/store requests from the CPU.
pub trait Bus {
    /// Read one byte.
    fn read(&mut self, addr: u32) -> Result<u8, BusError>;

    /// Write one byte.
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError>;

    /// Read a little-endian halfword. Implemented as two byte reads so
    /// alignment is a property of the underlying memory, not this default.
    fn read_half(&mut self, addr: u32) -> Result<u16, BusError> {
        let lo = self.read(addr)?;
        let hi = self.read(addr.wrapping_add(1))?;
        Ok(u16::from_le_bytes([lo, hi]))
    }

    /// Write a little-endian halfword.
    fn write_half(&mut self, addr: u32, val: u16) -> Result<(), BusError> {
        let [lo, hi] = val.to_le_bytes();
        self.write(addr, lo)?;
        self.write(addr.wrapping_add(1), hi)
    }

    /// Read a little-endian word.
    fn read_word(&mut self, addr: u32) -> Result<u32, BusError> {
        let lo = self.read(addr)?;
        let hi = self.read(addr.wrapping_add(1))?;
        let b2 = self.read(addr.wrapping_add(2))?;
        let b3 = self.read(addr.wrapping_add(3))?;
        Ok(u32::from_le_bytes([lo, hi, b2, b3]))
    }

    /// Write a little-endian word.
    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), BusError> {
        let [b0, b1, b2, b3] = val.to_le_bytes();
        self.write(addr, b0)?;
        self.write(addr.wrapping_add(1), b1)?;
        self.write(addr.wrapping_add(2), b2)?;
        self.write(addr.wrapping_add(3), b3)
    }
}

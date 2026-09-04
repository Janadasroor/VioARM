//! DWT cycle counter (`DWT`, `0xE000_1000`).
//! Only `CTRL.CYCCNTENA` (bit 0) and `CYCCNT` (`0x04`, R/W) are modeled.
//! When enabled, `CYCCNT` mirrors `Core::cyccnt` low 32 bits.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0xE000_1000;

#[derive(Debug)]
pub struct Dwt {
    ranges: [Region; 1],
    ctrl: u32,
    count: u64,
}

impl Default for Dwt {
    fn default() -> Self {
        Self::new()
    }
}

impl Dwt {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, 0x20)],
            ctrl: 0,
            count: 0,
        }
    }
}

impl Peripheral for Dwt {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = addr.wrapping_sub(BASE);
        match off {
            0x00..=0x03 => {
                let shift = (off & 0x3) * 8;
                Ok(((self.ctrl >> shift) & 0xFF) as u8)
            }
            0x04..=0x07 => {
                // CYCCNT low 32 bits, little endian
                let v = (self.count & 0xFFFF_FFFF) as u32;
                let shift = (off & 0x3) * 8;
                Ok(((v >> shift) & 0xFF) as u8)
            }
            _ => Err(BusError::Reserved),
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = addr.wrapping_sub(BASE);
        match off {
            0x00..=0x03 => {
                let shift = (off & 0x3) * 8;
                // Only bit0 writable, other bits RES0
                if shift == 0 {
                    if val & 0x1 != 0 {
                        self.ctrl |= 0x1;
                    } else {
                        self.ctrl &= !0x1;
                    }
                }
                Ok(())
            }
            0x04..=0x07 => {
                let shift = (off & 0x3) * 8;
                let mut v = (self.count & 0xFFFF_FFFF) as u32;
                v = (v & !(0xFF << shift)) | (u32::from(val) << shift);
                self.count = (self.count & 0xFFFF_FFFF_0000_0000) | u64::from(v);
                Ok(())
            }
            _ => Err(BusError::Reserved),
        }
    }

    fn tick(&mut self, delta: u64) {
        if self.ctrl & 0x1 != 0 {
            self.count = self.count.wrapping_add(delta);
        }
    }

    fn reset(&mut self) {
        self.ctrl = 0;
        self.count = 0;
    }

    fn name(&self) -> &'static str {
        "dwt"
    }
}

impl InterruptSource for Dwt {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Peripheral;

    #[test]
    fn cyccnt_counts_when_enabled() {
        let mut d = Dwt::new();
        d.write(BASE, 1).unwrap();
        assert_eq!(d.read(BASE).unwrap(), 1);
        d.tick(5);
        let mut w = [0u8; 4];
        for (i, b) in w.iter_mut().enumerate() {
            *b = d.read(BASE + 4 + i as u32).unwrap();
        }
        assert_eq!(u32::from_le_bytes(w), 5);
        // Write CYCCNT
        for (i, b) in 0xAABBCCDDu32.to_le_bytes().iter().enumerate() {
            d.write(BASE + 4 + i as u32, *b).unwrap();
        }
        let mut w2 = [0u8; 4];
        for (i, b) in w2.iter_mut().enumerate() {
            *b = d.read(BASE + 4 + i as u32).unwrap();
        }
        assert_eq!(u32::from_le_bytes(w2), 0xAABBCCDD);
        d.tick(1);
        let mut w3 = [0u8; 4];
        for (i, b) in w3.iter_mut().enumerate() {
            *b = d.read(BASE + 4 + i as u32).unwrap();
        }
        assert_eq!(u32::from_le_bytes(w3), 0xAABBCCDE);
        // Disabled does not count
        d.write(BASE, 0).unwrap();
        d.tick(10);
        let mut w4 = [0u8; 4];
        for (i, b) in w4.iter_mut().enumerate() {
            *b = d.read(BASE + 4 + i as u32).unwrap();
        }
        assert_eq!(u32::from_le_bytes(w4), 0xAABBCCDE);
    }
}

//! The STM32F1 alternate-function I/O unit (`AFIO`, `0x4001_0000`).
//!
//! Latches the real register file (`EVCR`, `MAPR`, `EXTICR1..4`, `MAPR2`) so
//! firmware that configures pin remapping or EXTI routing does not fault.
//! Remapping is accepted but not acted on — the board's fixed pin functions
//! are correct for the default mapping the e2e firmware uses.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0x4001_0000;
const WINDOW_LEN: u32 = 0x20;

#[cfg(test)]
const MAPR: u32 = 0x04;

const REGS: usize = 7;

const fn reg_of(off: u32) -> Option<usize> {
    match off >> 2 {
        0 => Some(0),
        1 => Some(1),
        2 => Some(2),
        3 => Some(3),
        4 => Some(4),
        5 => Some(5),
        7 => Some(6), // MAPR2 sits at 0x1C; 0x18 is reserved.
        _ => None,
    }
}

/// The `AFIO` register file.
#[derive(Debug, Clone)]
pub struct Afio {
    ranges: [Region; 1],
    regs: [u32; REGS],
}

impl Afio {
    /// A new `AFIO` at its STM32F1 base address.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, WINDOW_LEN)],
            regs: [0; REGS],
        }
    }
}

impl Default for Afio {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Afio {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - BASE;
        let Some(idx) = reg_of(off) else {
            return Err(BusError::Reserved);
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((self.regs[idx] >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - BASE;
        let Some(idx) = reg_of(off) else {
            return Err(BusError::Reserved);
        };
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        self.regs[idx] = (self.regs[idx] & !mask) | (u32::from(val) << shift);
        Ok(())
    }

    fn reset(&mut self) {
        self.regs = [0; REGS];
    }

    fn name(&self) -> &'static str {
        "afio"
    }
}

impl crate::device::InterruptSource for Afio {}

#[cfg(test)]
mod tests {
    use super::{Afio, BASE, MAPR};
    use crate::device::Peripheral;
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(a: &mut Afio, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            a.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[test]
    fn registers_roundtrip() {
        let mut a = Afio::new();
        write_word(&mut a, BASE + MAPR, 0xABCD_EF01);
        assert_eq!(a.read(BASE + MAPR).unwrap(), 0x01);
        assert_eq!(a.read(BASE + MAPR + 3).unwrap(), 0xAB);
    }

    #[test]
    fn gap_between_exticr4_and_mapr2_is_reserved() {
        let mut a = Afio::new();
        assert!(matches!(a.read(BASE + 0x18), Err(BusError::Reserved)));
        assert!(matches!(a.write(BASE + 0x18, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut a = Afio::new();
        assert!(matches!(a.read(BASE + 0x20), Err(BusError::Reserved)));
    }
}

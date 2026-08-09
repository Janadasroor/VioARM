//! The STM32F1 reset and clock control unit (`RCC`, `0x4002_1000`).
//!
//! A faithful register file: every clock enable and status bit from the SVD is
//! latched and readable, the internal HSI oscillator reads as always on and
//! ready (so `while (!(RCC->CR & RCC_CR_HSIRDY))` terminates), and the PLL
//! locks instantly once `PLLON` is set. Clock gating of the downstream
//! peripherals is not enforced here — see the `stm32` module docs.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0x4002_1000;

const CR: u32 = 0x00;

#[cfg(test)]
mod test_offsets {
    #![allow(dead_code)]
    pub const CFGR: u32 = 0x04;
    pub const CIR: u32 = 0x08;
    pub const APB2RSTR: u32 = 0x0C;
    pub const APB1RSTR: u32 = 0x10;
    pub const AHBENR: u32 = 0x14;
    pub const APB2ENR: u32 = 0x18;
    pub const APB1ENR: u32 = 0x1C;
    pub const BDCR: u32 = 0x20;
    pub const CSR: u32 = 0x24;
}

const WINDOW_LEN: u32 = 0x28;

const REGS: usize = 10;

/// HSION bit in `CR`.
const HSION: u32 = 1 << 0;
/// HSIRDY bit in `CR`.
const HSIRDY: u32 = 1 << 1;
/// PLLON bit in `CR`.
const PLLON: u32 = 1 << 24;
/// PLLRDY bit in `CR`.
const PLLRDY: u32 = 1 << 25;

const fn reg_of(off: u32) -> usize {
    (off >> 2) as usize
}

/// The `RCC` register file.
#[derive(Debug, Clone)]
pub struct Rcc {
    ranges: [Region; 1],
    regs: [u32; REGS],
}

impl Rcc {
    /// A new `RCC` at the STM32F1 base address.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, WINDOW_LEN)],
            regs: [0; REGS],
        }
    }

    const fn cr(&self) -> u32 {
        // HSI is always on and ready; PLL locks as soon as it is requested.
        let pll = if self.regs[reg_of(CR)] & PLLON != 0 {
            PLLRDY
        } else {
            0
        };
        self.regs[reg_of(CR)] | HSION | HSIRDY | pll
    }
}

impl Default for Rcc {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Rcc {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - BASE;
        if off < CR + 4 {
            #[allow(clippy::cast_possible_truncation)]
            return Ok((self.cr() >> (8 * (off & 3))) as u8);
        }
        let word = self.regs[reg_of(off)];
        #[allow(clippy::cast_possible_truncation)]
        Ok((word >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - BASE;
        let idx = reg_of(off);
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        self.regs[idx] = (self.regs[idx] & !mask) | (u32::from(val) << shift);
        Ok(())
    }

    fn reset(&mut self) {
        self.regs = [0; REGS];
        self.regs[reg_of(CR)] = HSION | HSIRDY;
    }

    fn name(&self) -> &'static str {
        "rcc"
    }
}

impl crate::device::InterruptSource for Rcc {}

#[cfg(test)]
mod tests {
    use super::test_offsets::{APB1ENR, APB2ENR};
    use super::{Rcc, BASE, CR, HSION, HSIRDY, PLLON, PLLRDY};
    use crate::device::Peripheral;
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(r: &mut Rcc, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            r.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(r: &mut Rcc, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = r.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn hsi_is_always_on_and_ready() {
        let mut r = Rcc::new();
        assert_eq!(
            read_word(&mut r, BASE + CR) & (HSION | HSIRDY),
            HSION | HSIRDY
        );
    }

    #[test]
    fn pll_locks_instantly_when_enabled() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + CR, PLLON);
        assert_eq!(read_word(&mut r, BASE + CR) & PLLRDY, PLLRDY);
    }

    #[test]
    fn enable_bits_roundtrip() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + APB2ENR, 0x0000_4014);
        write_word(&mut r, BASE + APB1ENR, 0x0000_0001);
        assert_eq!(read_word(&mut r, BASE + APB2ENR), 0x0000_4014);
        assert_eq!(read_word(&mut r, BASE + APB1ENR), 0x0000_0001);
    }

    #[test]
    fn reset_restores_hsi_only() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + CR, PLLON);
        write_word(&mut r, BASE + APB2ENR, 0xFFFF_FFFF);
        r.reset();
        assert_eq!(read_word(&mut r, BASE + CR) & 0xFF, HSION | HSIRDY);
        assert_eq!(read_word(&mut r, BASE + APB2ENR), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut r = Rcc::new();
        assert!(matches!(r.read(BASE + 0x28), Err(BusError::Reserved)));
        assert!(matches!(r.write(BASE - 4, 0), Err(BusError::Reserved)));
    }
}

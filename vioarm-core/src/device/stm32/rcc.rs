//! The STM32F1 reset and clock control unit (`RCC`, `0x4002_1000`), driven by
//! the SVD-generated register tables.
//!
//! Every clock enable and status bit from the SVD is latched and readable; the
//! internal HSI oscillator reads as always on and ready (so
//! `while (!(RCC->CR & RCC_CR_HSIRDY))` terminates), and the PLL locks
//! instantly once `PLLON` is set — both expressed as a [`Behavior`] hook on
//! the otherwise table-driven [`RegisterFile`]. Clock gating of the downstream
//! peripherals is not enforced here — see the `stm32` module docs.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// HSION bit in `CR`.
const HSION: u32 = 1 << 0;
/// HSIRDY bit in `CR`.
const HSIRDY: u32 = 1 << 1;
/// PLLON bit in `CR`.
const PLLON: u32 = 1 << 24;
/// PLLRDY bit in `CR`.
const PLLRDY: u32 = 1 << 25;

/// `CR` lives at register offset `0x00`.
const CR_OFFSET: u16 = 0x00;

/// RCC's only synthesis: the clock-ready bits in `CR`.
#[derive(Clone, Copy, Debug, Default)]
struct RccBehavior;

impl Behavior for RccBehavior {
    fn on_read(&mut self, offset: u16, value: u32) -> u32 {
        if offset == CR_OFFSET {
            // HSI is always on and ready; PLL locks as soon as it is requested.
            let pll = if value & PLLON != 0 { PLLRDY } else { 0 };
            value | HSION | HSIRDY | pll
        } else {
            value
        }
    }
}

/// The `RCC` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct Rcc {
    inner: RegisterFile<RccBehavior>,
}

impl Rcc {
    /// A new `RCC` at the STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `RCC`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        let def = stm32f103::peripheral("RCC").expect("generated tables contain RCC");
        Self {
            inner: RegisterFile::new(def),
        }
    }
}

impl Default for Rcc {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Rcc {
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
        "rcc"
    }
}

impl InterruptSource for Rcc {
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

#[cfg(test)]
mod tests {
    use super::{Rcc, HSION, HSIRDY, PLLON, PLLRDY};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4002_1000;
    const CR: u32 = 0x00;
    const APB2ENR: u32 = 0x18;
    const APB1ENR: u32 = 0x1C;

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
    fn reset_restores_svd_values() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + CR, PLLON);
        write_word(&mut r, BASE + APB2ENR, 0xFFFF_FFFF);
        r.reset();
        assert_eq!(read_word(&mut r, BASE + CR) & 0xFF, 0x83);
        assert_eq!(read_word(&mut r, BASE + APB2ENR), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut r = Rcc::new();
        assert!(matches!(r.read(BASE + 0x28), Err(BusError::Reserved)));
        assert!(matches!(r.write(BASE - 4, 0), Err(BusError::Reserved)));
    }
}

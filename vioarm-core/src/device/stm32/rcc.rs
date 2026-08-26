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
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// HSION bit in `CR`.
const HSION: u32 = 1 << 0;
/// HSIRDY bit in `CR` — F1 is bit 1, H7 is bit 2.
const HSIRDY_F1: u32 = 1 << 1;
const HSIRDY_H7: u32 = 1 << 2;
/// HSEON/HSERDY bits (H7 `CR` 16/17).
const HSEON: u32 = 1 << 16;
const HSERDY: u32 = 1 << 17;
/// PLLON / PLL1ON bit in `CR`.
const PLLON: u32 = 1 << 24;
/// PLLRDY / PLL1RDY bit in `CR`.
const PLLRDY: u32 = 1 << 25;
/// H7 PLL2/PLL3.
const PLL2ON: u32 = 1 << 26;
const PLL2RDY: u32 = 1 << 27;
const PLL3ON: u32 = 1 << 28;
const PLL3RDY: u32 = 1 << 29;
/// CSI comparable (H7 CSION 7 / CSIRDY 8).
const CSION: u32 = 1 << 7;
const CSIRDY: u32 = 1 << 8;

/// `CR` lives at register offset `0x00`.
const CR_OFFSET: u32 = 0x00;
/// `CFGR` lives at register offset `0x04`.
const CFGR_OFFSET: u32 = 0x04;

/// RCC's synthesis: clock-ready bits in `CR` + CFGR SW/SWS.
#[derive(Clone, Copy, Debug)]
struct RccBehavior {
    hsi_rdy: u32,
}

impl Default for RccBehavior {
    fn default() -> Self {
        Self { hsi_rdy: HSIRDY_F1 }
    }
}

impl Behavior for RccBehavior {
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CR_OFFSET {
            let hse = if value & HSEON != 0 { HSERDY } else { 0 };
            let pll = if value & PLLON != 0 { PLLRDY } else { 0 };
            let pll2 = if value & PLL2ON != 0 { PLL2RDY } else { 0 };
            let pll3 = if value & PLL3ON != 0 { PLL3RDY } else { 0 };
            let csi = if value & CSION != 0 { CSIRDY } else { 0 };
            value | HSION | self.hsi_rdy | hse | pll | pll2 | pll3 | csi
        } else if offset == CFGR_OFFSET {
            // SW (1:0) selects SYSCLK; SWS (3:2) reports it.
            // HSI always ready, so SWS follows SW instantly.
            let sw = value & 0x3;
            (value & !0xC) | (sw << 2)
        } else {
            value
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == CFGR_OFFSET {
            // CFGR is fully writable; SW/SWS handling is in on_read.
            return view.merged(offset, val, shift);
        }
        view.merged(offset, val, shift)
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
        Self::for_device::<stm32f103::Stm32f103>()
    }

    /// A new `RCC` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `RCC` peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("RCC").expect("generated tables contain RCC");
        // H7 RCC lives at 0x58024400 with HSIRDY at bit2; F1/F4/etc at
        // 0x4002xxxx use bit1. Derive the synthesized bit from the base so
        // we don't OR both shims on every read.
        let hsi_rdy = if def.base == 0x5802_4400 {
            HSIRDY_H7
        } else {
            HSIRDY_F1
        };
        Self {
            inner: RegisterFile::new(def, RccBehavior { hsi_rdy }),
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
    use super::{Rcc, HSION, HSIRDY_F1 as HSIRDY, PLLON, PLLRDY};
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
        // SVD CR reset is 0x83 (HSION|HSIRDY_F1| bit7); H7 would synthesize
        // HSIRDY_H7 (0x84) instead, so F1 reads 0x83.
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

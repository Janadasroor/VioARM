//! The STM32F1 alternate-function I/O unit (`AFIO`, `0x4001_0000`), driven by
//! the SVD-generated register tables.
//!
//! Every register is a pure latch, so [`NoBehavior`] needs no hooks: the whole
//! device is the register file the table describes. `EVCR`, `MAPR`,
//! `EXTICR1..4`, and `MAPR2` (with the reserved `0x18` gap) all roundtrip so
//! firmware that configures pin remapping or EXTI routing does not fault.
//! Remapping is accepted but not acted on — the board's fixed pin functions
//! are correct for the default mapping the e2e firmware uses.

use crate::device::svd::stm32f103;
use crate::device::svd::{Device, NoBehavior, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// The `AFIO` register file.
#[derive(Debug)]
pub struct Afio {
    inner: RegisterFile<NoBehavior>,
}

impl Afio {
    /// A new `AFIO` at its STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `AFIO`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f103::Stm32f103>()
    }

    /// A new `AFIO` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `AFIO`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("AFIO").expect("generated tables contain AFIO");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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
        "afio"
    }
}

impl InterruptSource for Afio {
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
    use super::{Afio, NoBehavior, RegisterFile};
    use crate::device::svd::stm32f103;
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4001_0000;
    const MAPR: u32 = 0x04;

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

    #[test]
    fn table_matches_silicon_layout() {
        let def = stm32f103::peripheral("AFIO").unwrap();
        let expect = [
            ("EVCR", 0x00),
            ("MAPR", 0x04),
            ("EXTICR1", 0x08),
            ("EXTICR2", 0x0C),
            ("EXTICR3", 0x10),
            ("EXTICR4", 0x14),
            ("MAPR2", 0x1C),
        ];
        for (name, offset) in expect {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("AFIO:{name} present"));
            assert_eq!(r.offset, offset, "AFIO:{name} offset");
        }
        let rf = RegisterFile::<NoBehavior>::new(def, NoBehavior);
        assert_eq!(rf.irq_exception("AFIO_IRQn"), None);
    }
}

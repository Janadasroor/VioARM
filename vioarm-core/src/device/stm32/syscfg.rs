//! The STM32F4 system configuration controller (`SYSCFG`, `0x4001_3800`),
//! driven by the SVD-generated register tables.
//!
//! Every register is a pure latch, so [`NoBehavior`] needs no hooks: `MEMRM`,
//! `PMC`, `EXTICR1..4`, and `CMPCR` all roundtrip so firmware that routes EXTI
//! lines through `EXTICR` or remaps the memory map does not fault. Routing is
//! accepted but not acted on — the EXTI model wires line 0 directly. `SYSCFG`
//! is the Cortex-M4 successor to the F1 `AFIO`.

use crate::device::svd::stm32f407;
use crate::device::svd::{Device, NoBehavior, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// The `SYSCFG` register file.
#[derive(Debug)]
pub struct Syscfg {
    inner: RegisterFile<NoBehavior>,
}

impl Syscfg {
    /// A new `SYSCFG` at its STM32F4 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `SYSCFG`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f407::Stm32f407>()
    }

    /// A new `SYSCFG` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SYSCFG`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        // Generic F0 donor tables merge SYSCFG (+COMP CSR) into one
        // `SYSCFG_COMP` window; same latch contract either way.
        let def = D::peripheral("SYSCFG")
            .or_else(|| D::peripheral("SYSCFG_COMP"))
            .expect("generated tables contain SYSCFG");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

impl Default for Syscfg {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Syscfg {
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
        "syscfg"
    }
}

impl InterruptSource for Syscfg {
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
    use super::{NoBehavior, RegisterFile, Syscfg};
    use crate::device::svd::stm32f407;
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4001_3800;
    const EXTICR1: u32 = 0x08;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(s: &mut Syscfg, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[test]
    fn registers_roundtrip() {
        let mut s = Syscfg::new();
        write_word(&mut s, BASE + EXTICR1, 0xABCD_EF01);
        assert_eq!(s.read(BASE + EXTICR1).unwrap(), 0x01);
        assert_eq!(s.read(BASE + EXTICR1 + 3).unwrap(), 0xAB);
    }

    #[test]
    fn gap_between_exticr4_and_cmpcr_is_reserved() {
        let mut s = Syscfg::new();
        assert!(matches!(s.read(BASE + 0x18), Err(BusError::Reserved)));
        assert!(matches!(s.write(BASE + 0x18, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut s = Syscfg::new();
        assert!(matches!(s.read(BASE + 0x400), Err(BusError::Reserved)));
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = stm32f407::peripheral("SYSCFG").unwrap();
        let expect = [
            ("MEMRM", 0x00),
            ("PMC", 0x04),
            ("EXTICR1", 0x08),
            ("EXTICR2", 0x0C),
            ("EXTICR3", 0x10),
            ("EXTICR4", 0x14),
            ("CMPCR", 0x20),
        ];
        for (name, offset) in expect {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("SYSCFG:{name} present"));
            assert_eq!(r.offset, offset, "SYSCFG:{name} offset");
        }
        let rf = RegisterFile::<NoBehavior>::new(def, NoBehavior);
        assert_eq!(rf.irq_exception("SYSCFG_IRQn"), None);
    }
}

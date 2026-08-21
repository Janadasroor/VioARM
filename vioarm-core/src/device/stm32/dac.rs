//! The STM32 DAC (`DAC`, `0x4000_7400` on F1/F4), driven by the
//! SVD-generated register tables.
//!
//! The DAC is modeled as a pure register latch: `CR`, `SWTRIGR`, `DHR12R1`/
//! `DHR12L1`/`DHR8R1` (and channel 2), `DOR1`/`DOR2` reflect the last holding
//! register written. No analog conversion is modeled; the SVD `reset` values
//! and `read/write` access are honored so firmware touching the DAC does not
//! fault, which is the current `real simulator` gap.

use crate::device::svd::{Behavior, Device, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct DacBehavior;

impl Behavior for DacBehavior {}

/// The `DAC` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct Dac {
    inner: RegisterFile<DacBehavior>,
}

impl Dac {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DAC").expect("generated tables contain DAC");
        Self {
            inner: RegisterFile::new(def, DacBehavior),
        }
    }
}

impl Peripheral for Dac {
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
        "dac"
    }
}

impl InterruptSource for Dac {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::Dac;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;

    const BASE: u32 = 0x4000_7400;

    fn write_word(d: &mut Dac, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }
    fn read_word(d: &mut Dac, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = d.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn dac_registers_round_trip_and_do_not_fault() {
        let mut d = Dac::new_for::<Stm32f103>();
        // `CR` and `DHR12R1` are read/write; a write round-trips.
        write_word(&mut d, BASE + 0x00, 0x0000_0001); // CR EN1
        assert_eq!(read_word(&mut d, BASE + 0x00) & 0x1, 0x1);
        write_word(&mut d, BASE + 0x08, 0x0ABC); // DHR12R1
        assert_eq!(read_word(&mut d, BASE + 0x08) & 0xFFF, 0x0ABC);
        // Out-of-window faults.
        assert!(matches!(d.read(BASE + 0x500), Err(crate::error::BusError::Reserved)));
    }
}

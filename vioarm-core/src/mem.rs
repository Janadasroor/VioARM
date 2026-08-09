//! The guest memory system.
//!
//! A `Bus` is the interface the CPU executes against. The first milestone
//! ships a `flat` map; `Bus` is a trait so device/wrapper models can be added
//! without touching the executor.

pub mod bus;
pub mod flat;
pub mod system;

pub use bus::{Bus, PeripheralBus, Region};

#[cfg(test)]
mod tests {
    use crate::mem::bus::Bus;
    use crate::mem::flat::FlatMemory;

    #[test]
    fn flat_memory_read_write() {
        let mut m = FlatMemory::with_layout(&[0x0000_0000, 0x2000_0000], &[0x1000, 0x4000]);
        m.write(0, 0x12).unwrap();
        assert_eq!(0x12, m.read(0).unwrap());
        m.write_word(0x2000_0000, 0xDEAD_BEEF).unwrap();
        assert_eq!(0xDEAD_BEEF, m.read_word(0x2000_0000).unwrap());
    }

    #[test]
    fn out_of_range_is_reserved() {
        let mut m = FlatMemory::with_layout(&[0x2000_0000], &[0x100]);
        assert!(matches!(m.read(0x2123_0000), Err(BusError::Reserved)));
    }

    use crate::error::BusError;
}

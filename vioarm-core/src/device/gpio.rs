//! A generic 32-bit GPIO port for the virtual board.
//!
//! Three 32-bit registers model the classic `DIR` / `OUT` / `IN` trio: a
//! direction latch (1 = output), an output latch, and a host-driven input
//! level register. The firmware can read input pins and write/read the output
//! latch; the host injects input levels through the `Peripheral` stimulus
//! channel. Pins with `DIR` cleared read the host-driven level, matching a
//! tri-state input.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Direction latch: bit set means the pin is an output.
const DIR: u32 = 0x00;
/// Output latch: written by the firmware, readable back.
const OUT: u32 = 0x04;
/// Input level register: host-driven, read-only to the firmware.
const IN: u32 = 0x08;
/// Last byte offset of the `DIR` word.
const DIR_END: u32 = DIR + 3;
/// Last byte offset of the `OUT` word.
const OUT_END: u32 = OUT + 3;
/// Last byte offset of the `IN` word.
const IN_END: u32 = IN + 3;
/// The register window covers `DIR..=IN_END`.
const WINDOW_LEN: u32 = 0x0C;

/// A 32-pin GPIO port.
#[derive(Debug, Clone)]
pub struct Gpio {
    ranges: [Region; 1],
    dir: u32,
    out: u32,
    input: u32,
}

impl Gpio {
    /// A new port mapped at `base`.
    #[must_use]
    pub const fn at(base: u32) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            dir: 0,
            out: 0,
            input: 0,
        }
    }

    /// The current output latch, for host inspection.
    #[must_use]
    pub const fn output(&self) -> u32 {
        self.out
    }
}

impl Peripheral for Gpio {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let word = match off {
            DIR..=DIR_END => self.dir,
            OUT..=OUT_END => self.out,
            IN..=IN_END => self.input,
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((word >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        let reg = match off {
            DIR..=DIR_END => &mut self.dir,
            OUT..=OUT_END => &mut self.out,
            // IN is host-driven; firmware writes are consumed without fault.
            IN..=IN_END => return Ok(()),
            _ => return Err(BusError::Reserved),
        };
        *reg = (*reg & !mask) | (u32::from(val) << shift);
        Ok(())
    }

    fn reset(&mut self) {
        self.dir = 0;
        self.out = 0;
        self.input = 0;
    }

    fn name(&self) -> &'static str {
        "gpio"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.input = value;
    }
}

impl InterruptSource for Gpio {}

#[cfg(test)]
mod tests {
    use super::{Gpio, DIR, IN, OUT};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4000_0000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(g: &mut Gpio, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            g.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(g: &mut Gpio, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = g.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn output_latch_writes_and_reads_back() {
        let mut g = Gpio::at(BASE);
        write_word(&mut g, BASE + DIR, 0xFFFF_FFFF);
        write_word(&mut g, BASE + OUT, 0x00C0_FFEE);
        assert_eq!(read_word(&mut g, BASE + OUT), 0x00C0_FFEE);
        assert_eq!(g.output(), 0x00C0_FFEE);
    }

    #[test]
    fn partial_writes_merge_byte_lanes() {
        let mut g = Gpio::at(BASE);
        write_word(&mut g, BASE + OUT, 0xFFFF_FFFF);
        // Zero the upper three bytes of the output latch.
        write_word(&mut g, BASE + OUT + 1, 0);
        assert_eq!(read_word(&mut g, BASE + OUT), 0x0000_00FF);
    }

    #[test]
    fn input_levels_are_host_driven() {
        let mut g = Gpio::at(BASE);
        assert_eq!(read_word(&mut g, BASE + IN), 0);
        g.drive_inputs(0x0000_5A00);
        assert_eq!(read_word(&mut g, BASE + IN), 0x0000_5A00);
        // Firmware writes to IN are ignored.
        write_word(&mut g, BASE + IN, 0xDEAD_BEEF);
        assert_eq!(read_word(&mut g, BASE + IN), 0x0000_5A00);
    }

    #[test]
    fn reset_clears_all_registers() {
        let mut g = Gpio::at(BASE);
        write_word(&mut g, BASE + DIR, 0xFFFF_FFFF);
        write_word(&mut g, BASE + OUT, 0xFFFF_FFFF);
        g.drive_inputs(0xFFFF_FFFF);
        g.reset();
        assert_eq!(read_word(&mut g, BASE + DIR), 0);
        assert_eq!(read_word(&mut g, BASE + OUT), 0);
        assert_eq!(read_word(&mut g, BASE + IN), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut g = Gpio::at(BASE);
        assert!(matches!(g.read(BASE + 0x0C), Err(BusError::Reserved)));
        assert!(matches!(g.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}

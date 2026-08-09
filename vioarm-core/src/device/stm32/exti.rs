//! The STM32F1 external interrupt controller (`EXTI`, `0x4001_0400`).
//!
//! A full 16-line register file (`IMR`/`EMR`/`RTSR`/`FTSR`/`SWIER`/`PR`) with
//! the complete edge-detection logic, but only line 0 is wired to an exception
//! (IRQ 6). Hosts raise it through the stimulus channel to simulate a button
//! or GPIO pin; software can also self-trigger via `SWIER`. Pending bits are
//! cleared the silicon way: writing 1 to `PR`.

use crate::device::InterruptSource;
use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0x4001_0400;
const WINDOW_LEN: u32 = 0x18;

// Word indices (all registers are 4-byte aligned slots).
const W_IMR: u32 = 0;
const W_EMR: u32 = 1;
const W_RTSR: u32 = 2;
const W_FTSR: u32 = 3;
const W_SWIER: u32 = 4;
const W_PR: u32 = 5;

/// Register offsets, used by tests to address the file directly.
#[cfg(test)]
const IMR: u32 = 0x00;
#[cfg(test)]
const FTSR: u32 = 0x0C;
#[cfg(test)]
const SWIER: u32 = 0x10;
#[cfg(test)]
const PR: u32 = 0x14;
#[cfg(test)]
const RTSR: u32 = 0x08;

/// Exception number for the EXTI0 line: IRQ 6.
const EXTI0_EXCEPTION: u8 = 16 + 6;

/// The `EXTI` controller. Line 0 drives the interrupt line.
#[derive(Debug, Clone)]
pub struct Exti {
    ranges: [Region; 1],
    imr: u32,
    emr: u32,
    rtsr: u32,
    ftsr: u32,
    pr: u32,
    level: u32,
    level_valid: bool,
}

impl Exti {
    /// A new `EXTI` at its STM32F1 base address.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, WINDOW_LEN)],
            imr: 0,
            emr: 0,
            rtsr: 0,
            ftsr: 0,
            pr: 0,
            level: 0,
            level_valid: false,
        }
    }

    const fn reg_read(&self, off: u32) -> u32 {
        match off >> 2 {
            W_IMR => self.imr,
            W_EMR => self.emr,
            W_RTSR => self.rtsr,
            W_FTSR => self.ftsr,
            // SWIER is write-only on silicon: reads return 0.
            W_PR => self.pr,
            _ => 0,
        }
    }

    fn reg_write(&mut self, off: u32, val: u8) {
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        let bits = u32::from(val) << shift;
        match off >> 2 {
            W_IMR => self.imr = (self.imr & !mask) | bits,
            W_EMR => self.emr = (self.emr & !mask) | bits,
            W_RTSR => self.rtsr = (self.rtsr & !mask) | bits,
            W_FTSR => self.ftsr = (self.ftsr & !mask) | bits,
            W_SWIER => self.pr |= bits,
            // PR is write-1-to-clear.
            W_PR => self.pr &= !bits,
            _ => {}
        }
    }
}

impl Default for Exti {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Exti {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - BASE;
        #[allow(clippy::cast_possible_truncation)]
        Ok((self.reg_read(off) >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        self.reg_write(addr - BASE, val);
        Ok(())
    }

    fn reset(&mut self) {
        self.imr = 0;
        self.emr = 0;
        self.rtsr = 0;
        self.ftsr = 0;
        self.pr = 0;
        self.level = 0;
        self.level_valid = false;
    }

    fn name(&self) -> &'static str {
        "exti"
    }

    fn drive_inputs(&mut self, value: u32) {
        if self.level_valid {
            let old = self.level;
            let rising = value & !old;
            let falling = old & !value;
            self.pr |= (rising & self.rtsr) | (falling & self.ftsr);
        }
        // The first sample after reset only establishes the reference level;
        // there is no prior level to edge against.
        self.level = value;
        self.level_valid = true;
    }
}

impl InterruptSource for Exti {
    fn exception_number(&self) -> Option<u8> {
        Some(EXTI0_EXCEPTION)
    }

    fn pending(&self) -> u32 {
        // Only EXTI line 0 is wired; it asserts while masked and pending.
        u32::from(self.pr & self.imr & 1 != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::{Exti, BASE, FTSR, IMR, PR, RTSR, SWIER};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(e: &mut Exti, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            e.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[test]
    fn rising_and_falling_edges() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + RTSR, 0b101); // rising on 0, 2
        write_word(&mut e, BASE + FTSR, 0b010); // falling on 1
                                                // The first sample only establishes the reference level.
        e.drive_inputs(0);
        assert_eq!(e.pr & 0b111, 0);
        // Rising on lines 0 and 2 (both configured) sets pending.
        e.drive_inputs(0b101);
        assert_eq!(e.pr & 0b111, 0b101);
        // Falling on 0,2 and rising on 1 are not configured: no change.
        e.drive_inputs(0b010);
        assert_eq!(e.pr & 0b111, 0b101);
        // Falling on line 1 (configured) now sets its bit.
        e.drive_inputs(0);
        assert_eq!(e.pr & 0b111, 0b111);
    }

    #[test]
    fn pending_requires_imr_and_line_zero() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + RTSR, 1);
        write_word(&mut e, BASE + SWIER, 1); // software self-trigger line 0
        assert_eq!(e.pending(), 0); // not masked
        write_word(&mut e, BASE + IMR, 1);
        assert_eq!(e.pending(), 1);
        // Clearing PR (write 1) drops the request.
        write_word(&mut e, BASE + PR, 1);
        assert_eq!(e.pending(), 0);
    }

    #[test]
    fn pr_is_write_one_to_clear() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + RTSR, 1);
        e.drive_inputs(0); // establish reference level
        e.drive_inputs(1);
        assert_eq!(e.pr & 1, 1);
        write_word(&mut e, BASE + PR, 0xFFFF_FFFF);
        assert_eq!(e.pr, 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut e = Exti::new();
        assert!(matches!(e.read(BASE + 0x18), Err(BusError::Reserved)));
        assert!(matches!(e.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}

//! The ARMv6-M `SysTick` timer.
//!
//! A 24-bit down-counter that runs on the processor clock when enabled. On
//! reaching zero it reloads from `SYST_RVR`, sets the `COUNTFLAG`, and — if
//! the `TICKINT` bit is set — asserts exception 15 (edge-triggered). The
//! counter is byte-addressable like every other device; the four registers
//! occupy `0xE000_E010..0xE000_E020`.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0xE000_E010;

/// The ARMv6-M system tick timer.
///
/// The one-bit CSR fields (ENABLE, TICKINT, CLKSOURCE, COUNTFLAG) map 1:1 to
/// bools; keeping them as such mirrors the register more closely than a bitfield.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone)]
pub struct SysTick {
    ranges: [Region; 1],
    enable: bool,
    tickint: bool,
    clksource: bool,
    countflag: bool,
    reload: u32,
    current: u32,
    int_pending: bool,
}

impl SysTick {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, 0x10)],
            enable: false,
            tickint: false,
            clksource: false,
            countflag: false,
            reload: 0,
            current: 0,
            int_pending: false,
        }
    }

    fn csr(&self) -> u32 {
        let mut v = 0;
        v |= u32::from(self.enable);
        v |= u32::from(self.tickint) << 1;
        v |= u32::from(self.clksource) << 2;
        v |= u32::from(self.countflag) << 16;
        v
    }

    fn read_reg(&self, offset: u32) -> u8 {
        match offset {
            0..=3 => ((self.csr() >> (8 * offset)) & 0xFF) as u8,
            4..=7 => ((self.reload >> (8 * (offset - 4))) & 0xFF) as u8,
            8..=11 => ((self.current >> (8 * (offset - 8))) & 0xFF) as u8,
            _ => 0,
        }
    }

    fn write_reg(&mut self, offset: u32, val: u8) {
        match offset {
            0..=3 => {
                self.countflag = false;
                if offset == 0 {
                    self.enable = val & 1 != 0;
                    self.tickint = val & 0x2 != 0;
                    self.clksource = val & 0x4 != 0;
                }
            }
            4..=7 => {
                let shift = 8 * (offset - 4);
                let mask = 0x00FF_FFFFu32 & (0xFF << shift);
                self.reload = (self.reload & !mask) | (u32::from(val) << shift) & mask;
            }
            8..=11 => {
                // Any write to CVR resets the counter and clears COUNTFLAG.
                self.current = 0;
                self.countflag = false;
            }
            _ => {}
        }
    }
}

impl Default for SysTick {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for SysTick {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        // On v6-M, reading CVR clears the COUNTFLAG.
        let offset = addr - BASE;
        if (8..=11).contains(&offset) {
            self.countflag = false;
        }
        Ok(self.read_reg(offset))
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        self.write_reg(addr - BASE, val);
        Ok(())
    }

    fn reset(&mut self) {
        self.enable = false;
        self.tickint = false;
        self.clksource = false;
        self.countflag = false;
        self.reload = 0;
        self.current = 0;
        self.int_pending = false;
    }

    fn tick(&mut self, delta: u64) {
        if !self.enable {
            return;
        }
        let mut n = delta;
        while n > 0 {
            n -= 1;
            if self.current == 0 {
                self.current = self.reload;
            } else {
                self.current -= 1;
            }
            if self.current == 0 {
                self.countflag = true;
                if self.tickint {
                    self.int_pending = true;
                }
            }
        }
    }

    fn name(&self) -> &'static str {
        "systick"
    }
}

impl InterruptSource for SysTick {
    fn exception_number(&self) -> Option<u8> {
        Some(15)
    }

    fn pending(&self) -> u32 {
        u32::from(self.int_pending)
    }

    fn acknowledge(&mut self, _lines: u32) {
        self.int_pending = false;
    }
}

#[cfg(test)]
mod tests {
    use super::SysTick;
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const CSR: u32 = 0xE000_E010;
    const RVR: u32 = 0xE000_E014;
    const CVR: u32 = 0xE000_E018;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(t: &mut SysTick, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            t.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(t: &mut SysTick, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = t.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn counts_down_and_wraps() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 9);
        write_word(&mut t, CSR, 0b111); // ENABLE | TICKINT | CLKSOURCE
        for _ in 0..9 {
            t.tick(1);
            assert_eq!(t.pending(), 0);
        }
        // 9 ticks: 0 reloads to 9, then 8..1, no wrap yet. The 10th wraps.
        t.tick(1);
        assert_eq!(t.pending(), 1);
        assert_eq!(read_word(&mut t, CVR), 0);
    }

    #[test]
    fn tickint_off_means_no_interrupt() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 2);
        write_word(&mut t, CSR, 0b101); // ENABLE | CLKSOURCE, no TICKINT
        for _ in 0..4 {
            t.tick(1);
        }
        assert_eq!(t.pending(), 0);
        // COUNTFLAG is still set even without an interrupt request.
        assert_eq!(read_word(&mut t, CSR) >> 16 & 1, 1);
    }

    #[test]
    fn reading_cvr_clears_countflag() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 1);
        write_word(&mut t, CSR, 0b111);
        t.tick(1);
        t.tick(1); // wrap
        assert_eq!(read_word(&mut t, CSR) >> 16 & 1, 1);
        let _ = read_word(&mut t, CVR);
        assert_eq!(read_word(&mut t, CSR) >> 16 & 1, 0);
    }

    #[test]
    fn disabled_timer_does_not_advance() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 9);
        t.tick(100);
        assert_eq!(read_word(&mut t, CVR), 0);
        assert_eq!(t.pending(), 0);
    }

    #[test]
    fn reload_masks_to_24_bits() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 0xFFFF_FFFF);
        assert_eq!(t.reload, 0x00FF_FFFF);
    }

    #[test]
    fn writing_cvr_resets_counter() {
        let mut t = SysTick::new();
        write_word(&mut t, RVR, 0x10);
        write_word(&mut t, CSR, 0b111);
        // First tick primes the fresh counter (0 -> RELOAD), then it counts
        // down: after 4 ticks it has decremented 3 times from 0x10.
        t.tick(4);
        assert_eq!(read_word(&mut t, CVR), 0x10 - 3);
        write_word(&mut t, CVR, 0);
        assert_eq!(read_word(&mut t, CVR), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut t = SysTick::new();
        assert!(matches!(t.read(0xE000_E02C), Err(BusError::Reserved)));
        assert!(matches!(t.write(0xE000_E030, 0), Err(BusError::Reserved)));
    }
}

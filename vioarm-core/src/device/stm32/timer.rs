//! The STM32F1 general-purpose timer (`TIM2`/`TIM3`).
//!
//! Models the counting core of the real peripheral: `CR1.CEN` gates the
//! counter, the prescaler `PSC` divides the clock by `PSC + 1`, and `CNT`
//! wraps past `ARR` into an update event that sets `SR.UIF` and — with
//! `DIER.UIE` set — raises a level-triggered IRQ. `EGR.UG` forces an update
//! from software. Channel logic (`CCR`/`CCER`) is latched state only.

use crate::device::InterruptSource;
use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const WINDOW_LEN: u32 = 0x50;

const CEN: u32 = 1 << 0;
const UIF: u32 = 1 << 0;
const UIE: u32 = 1 << 0;
const UG: u32 = 1 << 0;

/// STM32F1 IRQ numbers (SVD): `TIM2 = 28`, `TIM3 = 29`.
const TIM2_IRQ: u8 = 28;
const TIM3_IRQ: u8 = 29;

/// A general-purpose timer: `TIM2` (32-bit) or `TIM3` (16-bit).
#[derive(Debug, Clone)]
pub struct Timer {
    ranges: [Region; 1],
    name: &'static str,
    /// Register width mask (`u32::MAX` for TIM2, `0xFFFF` for TIM3).
    width_mask: u32,
    irq: u8,
    cr1: u32,
    dier: u32,
    sr: u32,
    cnt: u32,
    psc: u32,
    arr: u32,
    /// `CR2`, `SMCR`, `CCMR1/2`, `CCER`, `CCR1..4`, `DCR`, `DMAR`.
    other: [u32; 10],
    psc_phase: u64,
}

impl Timer {
    /// A `TIM2` at `0x4000_0000`: 32-bit counter, IRQ 28.
    #[must_use]
    pub const fn tim2() -> Self {
        Self::new(crate::device::stm32::TIM2, "tim2", TIM2_IRQ, u32::MAX)
    }

    /// A `TIM3` at `0x4000_0400`: 16-bit counter, IRQ 29.
    #[must_use]
    pub const fn tim3() -> Self {
        Self::new(crate::device::stm32::TIM3, "tim3", TIM3_IRQ, 0xFFFF)
    }

    const fn new(base: u32, name: &'static str, irq: u8, width_mask: u32) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            name,
            width_mask,
            irq,
            cr1: 0,
            dier: 0,
            sr: 0,
            cnt: 0,
            psc: 0,
            arr: 0,
            other: [0; 10],
            psc_phase: 0,
        }
    }

    /// Index into `other` for the latched register at `off`.
    const fn other_idx(off: u32) -> Option<usize> {
        match off {
            0x04 => Some(0),        // CR2
            0x08 => Some(1),        // SMCR
            0x18 => Some(2),        // CCMR1
            0x1C => Some(3),        // CCMR2
            0x20 => Some(4),        // CCER
            0x34 => Some(5),        // CCR1
            0x38 => Some(6),        // CCR2
            0x3C => Some(7),        // CCR3
            0x40 => Some(8),        // CCR4
            0x48 | 0x4C => Some(9), // DCR / DMAR
            _ => None,
        }
    }

    const fn pending_now(&self) -> bool {
        self.sr & UIF != 0 && self.dier & UIE != 0
    }
}

impl Peripheral for Timer {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let word = match off >> 2 {
            0 => self.cr1,
            3 => self.dier, // DIER
            4 => self.sr,   // SR
            9 => self.cnt,  // CNT
            10 => self.psc, // PSC
            11 => self.arr, // ARR
            _ => match Self::other_idx(off) {
                Some(i) => self.other[i],
                None => return Err(BusError::Reserved),
            },
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
        let bits = u32::from(val) << shift;
        let mask_reg = |reg: &mut u32| *reg = (*reg & !mask) | bits;
        match off >> 2 {
            0 => mask_reg(&mut self.cr1),
            3 => mask_reg(&mut self.dier), // DIER
            4 => mask_reg(&mut self.sr),   // SR
            9 => {
                let m = self.width_mask;
                self.cnt = ((self.cnt & !mask) | bits) & m;
            }
            10 => mask_reg(&mut self.psc), // PSC
            11 => {
                let m = self.width_mask;
                self.arr = ((self.arr & !mask) | bits) & m;
            }
            5 => {
                // EGR: any byte write with UG set forces an update event.
                if u32::from(val) & UG != 0 {
                    // Software update event: reload the counter and prescaler,
                    // and set the update flag like a real overflow would.
                    self.cnt = 0;
                    self.psc_phase = 0;
                    self.sr |= UIF;
                }
            }
            _ => {
                if let Some(i) = Self::other_idx(off) {
                    self.other[i] = (self.other[i] & !mask) | bits;
                } else {
                    return Err(BusError::Reserved);
                }
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.cr1 = 0;
        self.dier = 0;
        self.sr = 0;
        self.cnt = 0;
        self.psc = 0;
        self.arr = 0;
        self.other = [0; 10];
        self.psc_phase = 0;
    }

    fn tick(&mut self, delta: u64) {
        if self.cr1 & CEN == 0 {
            return;
        }
        let div = u64::from(self.psc.wrapping_add(1));
        let arr = self.arr;
        for _ in 0..delta {
            self.psc_phase += 1;
            // The counter advances once every `PSC + 1` clock edges.
            if self.psc_phase >= div {
                self.psc_phase = 0;
                self.cnt = self.cnt.wrapping_add(1) & self.width_mask;
                if self.cnt > arr {
                    self.cnt = 0;
                    self.sr |= UIF;
                }
            }
        }
    }

    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for Timer {
    fn exception_number(&self) -> Option<u8> {
        Some(16 + self.irq)
    }

    fn pending(&self) -> u32 {
        u32::from(self.pending_now())
    }
}

#[cfg(test)]
mod tests {
    use super::{Timer, CEN, UIE, UIF};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4000_0000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(t: &mut Timer, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            t.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(t: &mut Timer, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = t.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn armed() -> Timer {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 99); // ARR
        write_word(&mut t, BASE + 0x0C, UIE); // DIER
        write_word(&mut t, BASE + 0x14, 1); // EGR: UG
        write_word(&mut t, BASE + 0x10, 0); // SR: clear UIF
        write_word(&mut t, BASE, CEN); // CR1
        t
    }

    #[test]
    fn counts_up_and_wraps_at_arr() {
        let mut t = armed();
        for _ in 0..98 {
            t.tick(1);
            assert_eq!(t.pending(), 0);
        }
        assert_eq!(read_word(&mut t, BASE + 0x24), 98);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x24), 99);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn prescaler_divides_the_count() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x28, 9); // PSC -> divide by 10
        write_word(&mut t, BASE + 0x2C, 0); // ARR 0 -> wrap every count
        write_word(&mut t, BASE + 0x0C, UIE);
        write_word(&mut t, BASE, CEN);
        t.tick(9);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn update_interrupt_needs_uie() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 0);
        write_word(&mut t, BASE, CEN);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x10) & UIF, UIF);
        assert_eq!(t.pending(), 0);
    }

    #[test]
    fn disabled_timer_does_not_advance() {
        let mut t = armed();
        write_word(&mut t, BASE, 0); // CEN off
        t.tick(1000);
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
    }

    #[test]
    fn eg_ug_generates_an_update() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 50);
        write_word(&mut t, BASE, CEN);
        t.tick(25);
        assert_eq!(read_word(&mut t, BASE + 0x24), 25);
        write_word(&mut t, BASE + 0x14, 1); // UG
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
        assert_eq!(read_word(&mut t, BASE + 0x10) & UIF, UIF);
    }

    #[test]
    fn tim3_counter_is_sixteen_bit() {
        let mut t = Timer::tim3();
        let tim3_base = crate::device::stm32::TIM3;
        write_word(&mut t, tim3_base + 0x24, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, tim3_base + 0x24), 0xFFFF);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut t = Timer::tim2();
        assert!(matches!(t.read(BASE + 0x50), Err(BusError::Reserved)));
        assert!(matches!(t.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}

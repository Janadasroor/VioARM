//! The STM32F1 general-purpose timer (`TIM2`/`TIM3`), driven by the
//! SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! counting core, so [`TimerBehavior`] runs it: `CR1.CEN` gates the counter,
//! the prescaler `PSC` divides the clock by `PSC + 1`, and `CNT` wraps past
//! `ARR` into an update event that sets `SR.UIF` and — with `DIER.UIE` set —
//! raises a level-triggered IRQ. `EGR.UG` forces an update from software.
//! Channel logic (`CCR`/`CCER`) is latched state only.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Register offsets within the timer, per the SVD.
const CR1: u16 = 0x00;
const DIER: u16 = 0x0C;
const SR: u16 = 0x10;
const EGR: u16 = 0x14;
const CNT: u16 = 0x24;
const PSC: u16 = 0x28;
const ARR: u16 = 0x2C;

/// `CEN` bit in `CR1`.
const CEN: u32 = 1 << 0;
/// `UIF` bit in `SR` / `UG` bit in `EGR` / `UIE` bit in `DIER`.
const UIF: u32 = 1 << 0;
const UIE: u32 = 1 << 0;
const UG: u32 = 1 << 0;

/// The counting core the SVD cannot express.
#[derive(Clone, Copy, Debug)]
struct TimerBehavior {
    /// Register width mask: `u32::MAX` for the 32-bit `TIM2`, `0xFFFF` for the
    /// 16-bit `TIM3` counter and auto-reload registers.
    width_mask: u32,
    /// Prescaler phase within the current `PSC + 1` division.
    psc_phase: u64,
}

impl TimerBehavior {
    const fn new(width_mask: u32) -> Self {
        Self {
            width_mask,
            psc_phase: 0,
        }
    }
}

impl Behavior for TimerBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == EGR && (u32::from(val) << shift) & UG != 0 {
            // Any write with UG set forces a software update event: reload the
            // counter and prescaler, and set the update flag like a real
            // overflow would.
            view.set(CNT, 0);
            self.psc_phase = 0;
            view.set(SR, view.get(SR) | UIF);
        }
        let word = view.merged(offset, val, shift);
        match offset {
            // TIM3's counter and auto-reload are 16-bit on silicon even
            // though the SVD describes 32-bit register words.
            CNT | ARR => word & self.width_mask,
            _ => word,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if view.get(CR1) & CEN == 0 {
            return;
        }
        let div = u64::from(view.get(PSC).wrapping_add(1));
        let arr = view.get(ARR);
        for _ in 0..delta {
            self.psc_phase += 1;
            // The counter advances once every `PSC + 1` clock edges.
            if self.psc_phase >= div {
                self.psc_phase = 0;
                let cnt = view.get(CNT).wrapping_add(1) & self.width_mask;
                if cnt > arr {
                    view.set(CNT, 0);
                    view.set(SR, view.get(SR) | UIF);
                } else {
                    view.set(CNT, cnt);
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(SR) & UIF != 0 && view.get(DIER) & UIE != 0)
    }
}

/// A general-purpose timer: `TIM2` (32-bit) or `TIM3` (16-bit).
#[derive(Debug)]
pub struct Timer {
    inner: RegisterFile<TimerBehavior>,
    name: &'static str,
}

impl Timer {
    /// A `TIM2` at `0x4000_0000`: 32-bit counter, IRQ 28.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM2`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim2() -> Self {
        Self::tim2_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM2` peripheral.
    #[must_use]
    pub fn tim2_for<D: Device>() -> Self {
        let def = D::peripheral("TIM2").expect("generated tables contain TIM2");
        Self {
            inner: RegisterFile::new(def, TimerBehavior::new(u32::MAX)),
            name: "tim2",
        }
    }

    /// A `TIM3` at `0x4000_0400`: 16-bit counter, IRQ 29.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM3`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim3() -> Self {
        Self::tim3_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM3` peripheral.
    #[must_use]
    pub fn tim3_for<D: Device>() -> Self {
        let def = D::peripheral("TIM3").expect("generated tables contain TIM3");
        Self {
            inner: RegisterFile::new(def, TimerBehavior::new(0xFFFF)),
            name: "tim3",
        }
    }
}

impl Peripheral for Timer {
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

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for Timer {
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

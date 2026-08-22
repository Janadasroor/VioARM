//! The STM32 general-purpose and basic timers (`TIM1`..`TIM7`), driven by the
//! SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! counting core, so [`TimerBehavior`] runs it: `CR1.CEN` gates the counter,
//! the prescaler `PSC` divides the clock by `PSC + 1`, and `CNT` wraps past
//! `ARR` into an update event that sets `SR.UIF` and — with `DIER.UIE` set —
//! raises a level-triggered IRQ. `EGR.UG` forces an update from software.
//! Channel logic (`CCR`/`CCER`) is latched state only.
//!
//! The counter width varies by instance: `TIM2`/`TIM5` are 32-bit, the rest
//! 16-bit. The advanced `TIM1` raises four lines (break, update, trigger/comm,
//! capture/compare); the update interrupt is the second line in the table, so
//! the model routes its single pending line to `TIM1_UP` (IRQ 25) rather than
//! the break line the table happens to list first.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Register offsets within the timer, per the SVD.
const CR1: u32 = 0x00;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;
const CCR2: u32 = 0x38;
const CCR3: u32 = 0x3C;
const CCR4: u32 = 0x40;

/// `CEN` bit in `CR1`.
const CEN: u32 = 1 << 0;
/// `UIF` bit in `SR` / `UG` bit in `EGR` / `UIE` bit in `DIER`.
const UIF: u32 = 1 << 0;
const UIE: u32 = 1 << 0;
const UG: u32 = 1 << 0;
const CC1IE: u32 = 1 << 1;
const CC2IE: u32 = 1 << 2;
const CC3IE: u32 = 1 << 3;
const CC4IE: u32 = 1 << 4;
const CC1IF: u32 = 1 << 1;
const CC2IF: u32 = 1 << 2;
const CC3IF: u32 = 1 << 3;
const CC4IF: u32 = 1 << 4;

/// The counting core the SVD cannot express.
#[derive(Clone, Copy, Debug)]
struct TimerBehavior {
    /// Register width mask: `u32::MAX` for the 32-bit `TIM2`/`TIM5`,
    /// `0xFFFF` for the 16-bit counters.
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
        offset: u32,
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
            // The 16-bit counters and auto-reloads are 16-bit on silicon even
            // though the SVD describes 32-bit register words.
            CNT | ARR => word & self.width_mask,
            _ => word,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if view.get(CR1) & CEN == 0 {
            return;
        }
        let div = u64::from(view.get(PSC).wrapping_add(1));
        let arr = u64::from(view.get(ARR));
        for _ in 0..delta {
            self.psc_phase += 1;
            if self.psc_phase >= div {
                self.psc_phase = 0;
                let cnt = u64::from(view.get(CNT)) + 1;
                let next = if cnt > arr { 0 } else { cnt as u32 };
                if cnt > arr {
                    view.set(SR, view.get(SR) | UIF);
                }
                view.set(CNT, next);
                if let (Some(ccr), Some(ccmr1), Some(ccer)) =
                    (view.try_get(CCR1), view.try_get(CCMR1), view.try_get(CCER))
                {
                    let ocm = (ccmr1 >> 4) & 0x7;
                    if ocm != 0 && ccer & 1 != 0 && next == ccr & self.width_mask {
                        view.set(SR, view.get(SR) | CC1IF);
                    }
                }
                if let (Some(ccr), Some(ccmr1), Some(ccer)) =
                    (view.try_get(CCR2), view.try_get(CCMR1), view.try_get(CCER))
                {
                    let ocm = (ccmr1 >> 12) & 0x7;
                    if ocm != 0 && ccer & (1 << 4) != 0 && next == ccr & self.width_mask {
                        view.set(SR, view.get(SR) | CC2IF);
                    }
                }
                if let (Some(ccr), Some(ccmr2), Some(ccer)) =
                    (view.try_get(CCR3), view.try_get(CCMR2), view.try_get(CCER))
                {
                    let ocm = (ccmr2 >> 4) & 0x7;
                    if ocm != 0 && ccer & (1 << 8) != 0 && next == ccr & self.width_mask {
                        view.set(SR, view.get(SR) | CC3IF);
                    }
                }
                if let (Some(ccr), Some(ccmr2), Some(ccer)) =
                    (view.try_get(CCR4), view.try_get(CCMR2), view.try_get(CCER))
                {
                    let ocm = (ccmr2 >> 12) & 0x7;
                    if ocm != 0 && ccer & (1 << 12) != 0 && next == ccr & self.width_mask {
                        view.set(SR, view.get(SR) | CC4IF);
                    }
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(SR);
        let dier = view.get(DIER);
        let uie = sr & UIF != 0 && dier & UIE != 0;
        let cc1 = sr & CC1IF != 0 && dier & CC1IE != 0;
        let cc2 = sr & CC2IF != 0 && dier & CC2IE != 0;
        let cc3 = sr & CC3IF != 0 && dier & CC3IE != 0;
        let cc4 = sr & CC4IF != 0 && dier & CC4IE != 0;
        u32::from(uie || cc1 || cc2 || cc3 || cc4)
    }
}

/// A timer: `TIM2`/`TIM5` (32-bit), the rest 16-bit. Base address, register
/// set, and IRQ come from `D`'s generated tables.
#[derive(Debug)]
pub struct Timer {
    inner: RegisterFile<TimerBehavior>,
    name: &'static str,
    /// The table index of the update interrupt line. The advanced `TIM1`
    /// lists break first, so its update event routes to line 1 (`TIM1_UP`);
    /// the general/basic timers have a single line at index 0.
    irq_line: u32,
}

impl Timer {
    /// A `TIM1` at `0x4001_2C00` (F1): 16-bit counter, update IRQ 25.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim1() -> Self {
        Self::tim1_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM1` peripheral.
    #[must_use]
    pub fn tim1_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM1", "tim1", 0xFFFF, 1)
    }

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
        Self::for_device::<D>("TIM2", "tim2", u32::MAX, 0)
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
        Self::for_device::<D>("TIM3", "tim3", 0xFFFF, 0)
    }

    /// A `TIM4` at `0x4000_0800`: 16-bit counter, IRQ 30.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM4`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim4() -> Self {
        Self::tim4_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM4` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM4` peripheral.
    #[must_use]
    pub fn tim4_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM4", "tim4", 0xFFFF, 0)
    }

    /// A `TIM5` at `0x4000_0C00`: 32-bit counter, IRQ 50.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM5`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim5() -> Self {
        Self::tim5_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM5` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM5` peripheral.
    #[must_use]
    pub fn tim5_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM5", "tim5", u32::MAX, 0)
    }

    /// A `TIM6` at `0x4000_1000`: 16-bit basic counter, IRQ 54.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM6`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim6() -> Self {
        Self::tim6_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM6` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM6` peripheral.
    #[must_use]
    pub fn tim6_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM6", "tim6", 0xFFFF, 0)
    }

    /// A `TIM7` at `0x4000_1400`: 16-bit basic counter, IRQ 55.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM7`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim7() -> Self {
        Self::tim7_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM7` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM7` peripheral.
    #[must_use]
    pub fn tim7_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM7", "tim7", 0xFFFF, 0)
    }

    fn for_device<D: Device>(
        peripheral: &str,
        name: &'static str,
        width_mask: u32,
        irq_line: u32,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, TimerBehavior::new(width_mask)),
            name,
            irq_line,
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
        // The advanced TIM1's first table line is the break interrupt; the
        // update event belongs to the line the constructor stored (1 = UP).
        self.inner.exception_for_line(self.irq_line)
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
    fn tim4_and_tim7_counters_are_sixteen_bit() {
        for (timer, base) in [
            (Timer::tim4(), 0x4000_0800u32),
            (Timer::tim7(), 0x4000_1400),
        ] {
            let mut t = timer;
            write_word(&mut t, base + 0x24, 0xFFFF_FFFF);
            assert_eq!(read_word(&mut t, base + 0x24), 0xFFFF);
        }
    }

    #[test]
    fn tim5_counter_is_thirty_two_bit() {
        let mut t = Timer::tim5();
        write_word(&mut t, 0x4000_0C00 + 0x24, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, 0x4000_0C00 + 0x24), 0xFFFF_FFFF);
    }

    #[test]
    fn tim6_basic_counter_wraps_and_irqs() {
        let mut t = Timer::tim6();
        let base = 0x4000_1000u32;
        write_word(&mut t, base + 0x2C, 3); // ARR 3
        write_word(&mut t, base + 0x0C, UIE);
        write_word(&mut t, base, CEN);
        for _ in 0..4 {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, base + 0x24), 0);
        assert_eq!(t.pending(), 1);
        assert_eq!(t.exception_number(), Some(16 + 54));
    }

    #[test]
    fn full_auto_reload_wrap_sets_the_update_flag() {
        // A 16-bit counter with ARR = 0xFFFF wraps from 0xFFFF back to zero and
        // must raise UIF — the masked counter must not hide the overflow.
        let mut t = Timer::tim4();
        let base = 0x4000_0800u32;
        write_word(&mut t, base + 0x2C, 0xFFFF); // ARR
        write_word(&mut t, base + 0x0C, UIE);
        write_word(&mut t, base, CEN);
        for _ in 0..0xFFFF {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, base + 0x24), 0xFFFF);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(read_word(&mut t, base + 0x24), 0);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn tim1_update_routes_to_the_up_line() {
        // The advanced TIM1's table lists break (24) first; the model must
        // surface the update interrupt (25), not the break line.
        let mut t = Timer::tim1();
        assert_eq!(t.ranges()[0].base, 0x4001_2C00);
        assert_eq!(t.exception_number(), Some(16 + 25));
        write_word(&mut t, 0x4001_2C00 + 0x2C, 1); // ARR 1
        write_word(&mut t, 0x4001_2C00 + 0x0C, UIE);
        write_word(&mut t, 0x4001_2C00, CEN);
        t.tick(2);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut t = Timer::tim2();
        assert!(matches!(t.read(BASE + 0x50), Err(BusError::Reserved)));
        assert!(matches!(t.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}

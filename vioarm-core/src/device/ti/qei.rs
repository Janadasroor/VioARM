//! TI Tiva C quadrature encoder interfaces (`QEI0`-`QEI1`).
//!
//! Position/velocity capture follows the TM4C123GH6PM datasheet
//! (SPMS376E §21): `POS` integrates driven phase edges (quadrature
//! gray-code with forward/reverse/error decode, or `SIGMODE` clock/dir),
//! wrapping at `MAXPOS` (forward→0, reverse 0→`MAXPOS`), index resets in
//! `RESMODE=1`, `VELEN` velocity capture with `VELDIV` predivide over the
//! `LOAD` timer into `SPEED`, and `INTINDEX`/`INTTIMER`/`INTDIR`/
//! `INTERROR` latches behind `INTEN` with `ISC` W1C clear. `STAT` reports
//! last direction and sticky phase error; `ENABLE` is sticky-once-set
//! (factory reset only, like silicon's `SRQEI` requirement).
//!
//! Phase inputs arrive through [`Qei::drive_phase`] (host/test stimulus;
//! no pin coupling in T2): `CAPMODE=0` counts PhA edges, `CAPMODE=1`
//! counts both, `INVA`/`INVB`/`SWAP` transform first (documented order).
//! The velocity timer runs in device ticks (no `sysclk_hz` coupling:
//! firmware observes counts, exact in tick units). Filter (`FILTEN`),
//! stall, and index-pulse electrical details are unmodeled (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CTL_OFF: u32 = 0x000;
const STAT_OFF: u32 = 0x004;
const POS_OFF: u32 = 0x008;
const MAXPOS_OFF: u32 = 0x00C;
const LOAD_OFF: u32 = 0x010;
const TIME_OFF: u32 = 0x014;
const COUNT_OFF: u32 = 0x018;
const SPEED_OFF: u32 = 0x01C;
const INTEN_OFF: u32 = 0x020;
const RIS_OFF: u32 = 0x024;
const ISC_OFF: u32 = 0x028;

/// `CTL` bits.
const CTL_ENABLE: u32 = 1 << 0;
const CTL_SWAP: u32 = 1 << 1;
const CTL_SIGMODE: u32 = 1 << 2;
const CTL_CAPMODE: u32 = 1 << 3;
const CTL_RESMODE: u32 = 1 << 4;
const CTL_VELEN: u32 = 1 << 5;
const CTL_VELDIV_SHIFT: u32 = 6;
const CTL_VELDIV_MASK: u32 = 0x07;
const CTL_INVA: u32 = 1 << 9;
const CTL_INVB: u32 = 1 << 10;
/// `STAT` bits: `DIRECTION` 1 (1 = reverse), `ERROR` 0.
const STAT_DIRECTION: u32 = 1 << 1;
const STAT_ERROR: u32 = 1 << 0;
/// Interrupt bits: index 0, timer 1, direction 2, error 3.
const INT_INDEX: u32 = 1 << 0;
const INT_TIMER: u32 = 1 << 1;
const INT_DIR: u32 = 1 << 2;
const INT_ERROR: u32 = 1 << 3;

/// Behavior behind one QEI window.
#[derive(Debug, Default)]
pub struct QeiBehavior {
    /// Last sampled phase levels.
    pa: bool,
    pb: bool,
    /// Last step direction (`false` forward, `true` reverse).
    reverse: bool,
    /// Sticky phase error (cleared with `INTERROR`).
    error: bool,
    /// Interrupt latches `(index, timer, dir, error)`.
    ris: u32,
    /// Velocity accumulator (pre-divided pulses) and prescaler remainder.
    vel_acc: u32,
    vel_pre: u32,
    /// Velocity-timer countdown remainder.
    vel_timer: u32,
    /// Last completed velocity count.
    speed: u32,
    /// Cached `RIS & INTEN` bitmask (`InterruptSource::pending` is `&self`).
    pending: u32,
}

impl QeiBehavior {
    fn enabled(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(CTL_OFF) & CTL_ENABLE != 0
    }

    /// Transform driven levels through `INVA`/`INVB`/`SWAP`.
    fn xform(&self, view: &Registers<&mut [u32]>, a: bool, b: bool) -> (bool, bool) {
        let ctl = view.get(CTL_OFF);
        let (mut a, mut b) = (a, b);
        if ctl & CTL_INVA != 0 {
            a = !a;
        }
        if ctl & CTL_INVB != 0 {
            b = !b;
        }
        if ctl & CTL_SWAP != 0 {
            std::mem::swap(&mut a, &mut b);
        }
        (a, b)
    }

    /// Apply one forward (`+1`) or reverse (`-1`) step to the position.
    fn step_position(&mut self, view: &mut Registers<&mut [u32]>, reverse: bool) {
        let maxpos = view.get(MAXPOS_OFF);
        let mut pos = view.get(POS_OFF);
        if reverse {
            pos = if pos == 0 { maxpos } else { pos - 1 };
        } else {
            pos = if pos >= maxpos { 0 } else { pos + 1 };
        }
        view.set(POS_OFF, pos);
        if reverse != self.reverse {
            self.reverse = reverse;
            self.ris |= INT_DIR;
        }
        // Velocity pulse through the `VELDIV` predivider.
        let veldiv = (view.get(CTL_OFF) >> CTL_VELDIV_SHIFT) & CTL_VELDIV_MASK;
        self.vel_pre += 1;
        if self.vel_pre >= (1 << veldiv) {
            self.vel_pre = 0;
            self.vel_acc = self.vel_acc.wrapping_add(1);
        }
    }

    /// Evaluate a driven `(a, b)` sample against the previous one.
    fn sample(&mut self, view: &mut Registers<&mut [u32]>, a: bool, b: bool) {
        let (a, b) = self.xform(view, a, b);
        if !self.enabled(view) {
            self.pa = a;
            self.pb = b;
            return;
        }
        let ctl = view.get(CTL_OFF);
        if ctl & CTL_SIGMODE != 0 {
            // Clock/dir: rising `A` clocks one step in the `B` direction.
            if !self.pa && a {
                self.step_position(view, !b);
            }
            self.pa = a;
            self.pb = b;
            self.refresh(view);
            return;
        }
        let (pa, pb) = (self.pa, self.pb);
        self.pa = a;
        self.pb = b;
        if (pa, pb) == (a, b) {
            return;
        }
        let a_changed = pa != a;
        if ctl & CTL_CAPMODE == 0 && !a_changed {
            return; // `CAPMODE=0` counts PhA edges only.
        }
        // Gray-code step: exactly one line changed.
        let forward = matches!(
            (pa, pb, a, b),
            (false, false, true, false)
                | (true, false, true, true)
                | (true, true, false, true)
                | (false, true, false, false)
        );
        let reverse = matches!(
            (pa, pb, a, b),
            (false, false, false, true)
                | (false, true, true, true)
                | (true, true, true, false)
                | (true, false, false, false)
        );
        if forward {
            self.step_position(view, false);
        } else if reverse {
            self.step_position(view, true);
        } else {
            // Both lines changed: gray-code error.
            self.error = true;
            self.ris |= INT_ERROR;
        }
        self.refresh(view);
    }

    fn ris_word(&self) -> u32 {
        self.ris & 0x0F
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris_word() & view.get(INTEN_OFF) & 0x0F;
    }
}

impl Behavior for QeiBehavior {
    fn needs_tick() -> bool {
        true
    }

    /// Run the velocity timer down; on expiry, latch `SPEED`, reset the
    /// accumulator, and raise `INTTIMER`.
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.enabled(view) || view.get(CTL_OFF) & CTL_VELEN == 0 {
            return;
        }
        let period = view.get(LOAD_OFF);
        if period == 0 {
            return;
        }
        let mut dt = delta;
        while dt > 0 {
            dt -= 1;
            if self.vel_timer == 0 {
                self.vel_timer = period;
            }
            self.vel_timer -= 1;
            if self.vel_timer == 0 {
                self.speed = self.vel_acc;
                self.vel_acc = 0;
                self.vel_pre = 0;
                self.ris |= INT_TIMER;
            }
        }
        self.refresh(view);
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            STAT_OFF => (u32::from(self.reverse) << 1) | u32::from(self.error),
            TIME_OFF => self.vel_timer,
            COUNT_OFF => self.vel_acc,
            SPEED_OFF => self.speed,
            RIS_OFF => self.ris_word(),
            ISC_OFF => self.ris_word() & view.get(INTEN_OFF) & 0x0F,
            _ => value,
        };
        self.refresh(view);
        out
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == STAT_OFF
            || offset == TIME_OFF
            || offset == COUNT_OFF
            || offset == SPEED_OFF
            || offset == RIS_OFF
        {
            return view.get(offset);
        }
        if offset == ISC_OFF {
            let word = view.merged(offset, val, shift);
            if word & INT_INDEX != 0 {
                self.ris &= !INT_INDEX;
            }
            if word & INT_TIMER != 0 {
                self.ris &= !INT_TIMER;
            }
            if word & INT_DIR != 0 {
                self.ris &= !INT_DIR;
            }
            if word & INT_ERROR != 0 {
                self.ris &= !INT_ERROR;
                self.error = false;
            }
            self.refresh(view);
            return self.ris_word() & view.get(INTEN_OFF) & 0x0F;
        }
        if offset == CTL_OFF {
            // `ENABLE` is sticky-once-set (silicon needs `SRQEI`).
            let cur = view.get(offset);
            let mut word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
            if cur & CTL_ENABLE != 0 {
                word |= CTL_ENABLE;
            }
            view.set(offset, word);
            self.refresh(view);
            return word;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if offset == POS_OFF {
            // Firmware re-seeds the integrator; keep direction history.
        }
        self.refresh(view);
        word
    }
}

/// A Tiva C quadrature encoder interface.
pub struct Qei {
    inner: RegisterFile<QeiBehavior>,
}

impl Qei {
    /// A new QEI on `D`'s tables (`"QEI0"`/`"QEI1"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain QEI");
        Self {
            inner: RegisterFile::new(def, QeiBehavior::default()),
        }
    }

    /// Drive the phase inputs (host/test stimulus). With `index`, an
    /// index pulse arrives with this sample.
    pub fn drive_phase(&mut self, a: bool, b: bool, index: bool) {
        self.inner.with_view(|view, beh| {
            beh.sample(view, a, b);
            if index {
                if view.get(CTL_OFF) & CTL_RESMODE != 0 {
                    view.set(POS_OFF, 0);
                }
                beh.ris |= INT_INDEX;
                beh.refresh(view);
            }
        });
    }
}

impl Peripheral for Qei {
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
}

impl InterruptSource for Qei {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.behavior().pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const QEI0_BASE: u32 = 0x4002_C000;

    fn qei0() -> Qei {
        Qei::for_device::<Tm4c123gh6pm>("QEI0")
    }

    fn read_word(qei: &mut Qei, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(qei.read(QEI0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(qei: &mut Qei, offset: u32, value: u32) {
        for lane in 0..4 {
            qei.write(
                QEI0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Enable with a wide-open position range.
    fn enable(qei: &mut Qei) {
        write_word(qei, MAXPOS_OFF, 0xFFFF_FFFF);
        write_word(qei, CTL_OFF, CTL_ENABLE);
    }

    /// One forward gray step from (0,0): (1,0). Drive full cycles.
    fn step_forward(qei: &mut Qei) {
        qei.drive_phase(true, false, false);
        qei.drive_phase(true, true, false);
        qei.drive_phase(false, true, false);
        qei.drive_phase(false, false, false);
    }

    /// Quadrature forward counts up; reverse counts down with direction
    /// status and change interrupt.
    #[test]
    fn quadrature_counts_both_ways() {
        let mut qei = qei0();
        enable(&mut qei);
        write_word(&mut qei, INTEN_OFF, INT_DIR);
        step_forward(&mut qei);
        // `CAPMODE=0`: the two PhA edges of the cycle count.
        assert_eq!(read_word(&mut qei, POS_OFF), 2);
        assert_eq!(read_word(&mut qei, STAT_OFF) & STAT_DIRECTION, 0);
        // Reverse one cycle.
        qei.drive_phase(false, true, false);
        qei.drive_phase(true, true, false);
        qei.drive_phase(true, false, false);
        qei.drive_phase(false, false, false);
        assert_eq!(read_word(&mut qei, POS_OFF), 0);
        assert_eq!(
            read_word(&mut qei, STAT_OFF) & STAT_DIRECTION,
            STAT_DIRECTION
        );
        assert_eq!(read_word(&mut qei, RIS_OFF) & INT_DIR, INT_DIR);
        assert_eq!(qei.pending(), INT_DIR);
        assert_eq!(qei.exception_number(), Some(29));
        write_word(&mut qei, ISC_OFF, INT_DIR);
        assert_eq!(qei.pending(), 0);
    }

    /// Forward past `MAXPOS` wraps to zero.
    #[test]
    fn wraps_at_maxpos() {
        let mut qei = qei0();
        write_word(&mut qei, MAXPOS_OFF, 1);
        write_word(&mut qei, CTL_OFF, CTL_ENABLE);
        step_forward(&mut qei); // two PhA counts: 0 → 1 → 0.
        assert_eq!(read_word(&mut qei, POS_OFF), 0);
    }

    /// `CAPMODE=1` counts both edges: four per cycle.
    #[test]
    fn capmode_counts_both_edges() {
        let mut qei = qei0();
        write_word(&mut qei, MAXPOS_OFF, 0xFFFF_FFFF);
        write_word(&mut qei, CTL_OFF, CTL_ENABLE | CTL_CAPMODE);
        step_forward(&mut qei);
        assert_eq!(read_word(&mut qei, POS_OFF), 4);
    }

    /// Both-lines-changed is a gray-code error: sticky `ERROR` +
    /// `INTERROR`, cleared together.
    #[test]
    fn gray_error_latches() {
        let mut qei = qei0();
        enable(&mut qei);
        write_word(&mut qei, INTEN_OFF, INT_ERROR);
        qei.drive_phase(true, true, false); // (0,0) → (1,1): illegal.
        assert_eq!(read_word(&mut qei, STAT_OFF) & STAT_ERROR, STAT_ERROR);
        assert_eq!(read_word(&mut qei, POS_OFF), 0);
        assert_eq!(qei.pending(), INT_ERROR);
        write_word(&mut qei, ISC_OFF, INT_ERROR);
        assert_eq!(read_word(&mut qei, STAT_OFF) & STAT_ERROR, 0);
        assert_eq!(qei.pending(), 0);
    }

    /// Index resets the position in `RESMODE=1` and always raises `INTINDEX`.
    #[test]
    fn index_resets_and_fires() {
        let mut qei = qei0();
        write_word(&mut qei, MAXPOS_OFF, 0xFFFF_FFFF);
        write_word(&mut qei, CTL_OFF, CTL_ENABLE | CTL_RESMODE);
        write_word(&mut qei, INTEN_OFF, INT_INDEX);
        step_forward(&mut qei);
        assert_eq!(read_word(&mut qei, POS_OFF), 2);
        qei.drive_phase(false, false, true);
        assert_eq!(read_word(&mut qei, POS_OFF), 0);
        assert_eq!(qei.pending(), INT_INDEX);
    }

    /// Velocity capture: `LOAD` ticks latch `SPEED` and raise `INTTIMER`.
    #[test]
    fn velocity_capture_flow() {
        let mut qei = qei0();
        enable(&mut qei);
        write_word(&mut qei, CTL_OFF, CTL_ENABLE | CTL_VELEN);
        write_word(&mut qei, LOAD_OFF, 100);
        write_word(&mut qei, INTEN_OFF, INT_TIMER);
        step_forward(&mut qei); // 2 pulses.
        step_forward(&mut qei); // 4 pulses.
        qei.tick(100);
        assert_eq!(read_word(&mut qei, SPEED_OFF), 4);
        assert_eq!(read_word(&mut qei, COUNT_OFF), 0);
        assert_eq!(qei.pending(), INT_TIMER);
        write_word(&mut qei, ISC_OFF, INT_TIMER);
        assert_eq!(qei.pending(), 0);
    }

    /// `ENABLE` is sticky: clear-writes ignored until reset.
    #[test]
    fn enable_is_sticky() {
        let mut qei = qei0();
        write_word(&mut qei, CTL_OFF, CTL_ENABLE);
        write_word(&mut qei, CTL_OFF, 0x00);
        assert_eq!(read_word(&mut qei, CTL_OFF) & CTL_ENABLE, CTL_ENABLE);
    }
}

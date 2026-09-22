//! TI Tiva C general-purpose timers (`TIMER0`-`TIMER5`, `WTIMER0`-`WTIMER5`).
//!
//! Concatenated Timer A operation follows the TM4C123GH6PM datasheet
//! (SPMS376E §11): `CFG=0` selects 32-bit (`TIMERn`) or 64-bit (`WTIMERn`,
//! `TAILR` low + `TBILR` high) down/up counting, one-shot (`TAMR=1`) or
//! periodic (`TAMR=2`), with `TAPR` as a true prescaler (clock divided by
//! `TAPSR+1`; 16-bit prescaler on wide timers), `TAMATCHR` + `TAMIE` match,
//! and timeout/match latches behind `IMR`/`RIS`/`MIS` with `ICR` W1C clear.
//! `TAEN` 0→1 reloads from `TAILR` (immediate when `TAILD=0`, on timeout
//! when `TAILD=1`); `TAR`/`TBR` read the running count, `TAV`/`TBV` write
//! it, `TAPV` reads the prescaler remainder.
//!
//! Deliberate scope: Timer B, split modes (`CFG=4`), RTC (`CFG=1`),
//! capture/PWM (`TAMR=3`, `TAAMS`), `WOT`, daisy-chain `SYNC`, prescale
//! match (`TAPMR`), and DMA/ADC event registers stay latched words —
//! Timer A concatenated one-shot/periodic is the TivaWare workhorse and
//! the only mode advanced here. Device ticks drive the counter 1:1 with
//! the prescaler divide (no `sysclk_hz` coupling: firmware observes poll
//! loops and interrupt counts, both exact in tick units).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CFG_OFF: u32 = 0x000;
const TAMR_OFF: u32 = 0x004;
const CTL_OFF: u32 = 0x00C;
const IMR_OFF: u32 = 0x018;
const RIS_OFF: u32 = 0x01C;
const MIS_OFF: u32 = 0x020;
const ICR_OFF: u32 = 0x024;
const TAILR_OFF: u32 = 0x028;
const TBILR_OFF: u32 = 0x02C;
const TAMATCHR_OFF: u32 = 0x030;
const TBMATCHR_OFF: u32 = 0x034;
const TAPR_OFF: u32 = 0x038;
const TAR_OFF: u32 = 0x048;
const TBR_OFF: u32 = 0x04C;
const TAV_OFF: u32 = 0x050;
const TBV_OFF: u32 = 0x054;
const TAPV_OFF: u32 = 0x064;

/// `CTL.TAEN` bit 0.
const CTL_TAEN: u32 = 1 << 0;
/// `TAMR` fields: `TAMIE` 5, `TACDIR` 4, `TAMR` 1:0 (`1` one-shot, `2` periodic).
const TAMR_TAMIE: u32 = 1 << 5;
const TAMR_TACDIR: u32 = 1 << 4;
const TAMR_MODE_MASK: u32 = 0x03;
const TAMR_ONE_SHOT: u32 = 0x01;
const TAMR_PERIODIC: u32 = 0x02;
/// `TAMR.TAILD` bit 8 (reload on timeout vs immediately).
const TAMR_TAILD: u32 = 1 << 8;
/// `RIS`/`IMR`/`ICR` Timer A bits: timeout 0, match 4.
const RIS_TATO: u32 = 1 << 0;
const RIS_TAM: u32 = 1 << 4;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[
    (TAILR_OFF, 0xFFFF_FFFF),
    (TAMATCHR_OFF, 0xFFFF_FFFF),
    (TAR_OFF, 0xFFFF_FFFF),
    (TAV_OFF, 0xFFFF_FFFF),
];

/// Behavior behind one GPTM window (Timer A concatenated only).
#[derive(Debug)]
pub struct TimerBehavior {
    wide: bool,
    count: u64,
    /// Prescaler countdown remainder (counts down from `TAPSR`).
    pre: u32,
    /// `true` while a one-shot waits for re-enable after timing out.
    stopped: bool,
    timeout_latch: bool,
    match_latch: bool,
    /// Cached `(RIS & IMR) != 0` (`InterruptSource::pending` is `&self`).
    pending: bool,
}

impl TimerBehavior {
    fn mask(&self) -> u64 {
        if self.wide {
            u64::MAX
        } else {
            0xFFFF_FFFF
        }
    }

    fn prescale(&self, view: &Registers<&mut [u32]>) -> u32 {
        let w = view.get(TAPR_OFF);
        if self.wide {
            w & 0xFFFF
        } else {
            w & 0xFF
        }
    }

    fn reload_value(&self, view: &Registers<&mut [u32]>) -> u64 {
        let lo = u64::from(view.get(TAILR_OFF));
        if self.wide {
            (u64::from(view.get(TBILR_OFF)) << 32) | lo
        } else {
            lo & 0xFFFF_FFFF
        }
    }

    fn match_value(&self, view: &Registers<&mut [u32]>) -> u64 {
        let lo = u64::from(view.get(TAMATCHR_OFF));
        if self.wide {
            (u64::from(view.get(TBMATCHR_OFF)) << 32) | lo
        } else {
            lo & 0xFFFF_FFFF
        }
    }

    /// Whether the counter advances: concatenated, one-shot/periodic.
    fn runnable(&self, view: &Registers<&mut [u32]>) -> bool {
        if view.get(CFG_OFF) & 0x07 != 0 {
            return false;
        }
        if view.get(CTL_OFF) & CTL_TAEN == 0 {
            return false;
        }
        matches!(
            view.get(TAMR_OFF) & TAMR_MODE_MASK,
            TAMR_ONE_SHOT | TAMR_PERIODIC
        )
    }

    fn up(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(TAMR_OFF) & TAMR_TACDIR != 0
    }

    /// Advance the counter by `delta` device ticks through the prescaler.
    /// `pre` accumulates fractional ticks; `TAPV` reads it back.
    fn step(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.runnable(view) || self.stopped {
            return;
        }
        let div = u64::from(self.prescale(view)) + 1;
        let total = u64::from(self.pre) + delta;
        let mut steps = total / div;
        self.pre = u32::try_from(total % div).unwrap_or(u32::MAX);
        let up = self.up(view);
        let reload = self.reload_value(view);
        let one_shot = view.get(TAMR_OFF) & TAMR_MODE_MASK == TAMR_ONE_SHOT;
        while steps > 0 {
            steps -= 1;
            if up {
                self.count += 1;
                if self.count >= reload {
                    self.timeout_latch = true;
                    self.count = if one_shot { reload } else { 0 };
                    if one_shot {
                        self.stopped = true;
                        break;
                    }
                }
            } else if self.count == 0 {
                self.timeout_latch = true;
                if one_shot {
                    self.stopped = true;
                    break;
                }
                self.count = reload;
            } else {
                self.count -= 1;
                if self.count == 0 {
                    self.timeout_latch = true;
                    if one_shot {
                        self.stopped = true;
                        break;
                    }
                }
            }
            self.check_match(view);
        }
        self.refresh(view);
    }

    fn check_match(&mut self, view: &Registers<&mut [u32]>) {
        if view.get(TAMR_OFF) & TAMR_TAMIE != 0
            && self.count & self.mask() == self.match_value(view)
        {
            self.match_latch = true;
        }
    }

    fn ris(&self, view: &Registers<&mut [u32]>) -> u32 {
        let mut ris = 0u32;
        if self.timeout_latch {
            ris |= RIS_TATO;
        }
        if self.match_latch {
            ris |= RIS_TAM;
        }
        ris
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris(view) & view.get(IMR_OFF) != 0;
    }
}

impl Behavior for TimerBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        self.step(view, delta);
    }

    fn reset(&mut self) {
        let wide = self.wide;
        *self = Self {
            wide,
            count: 0xFFFF_FFFF,
            pre: 0,
            stopped: false,
            timeout_latch: false,
            match_latch: false,
            pending: false,
        };
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            TAR_OFF => (self.count & 0xFFFF_FFFF) as u32,
            TBR_OFF => (self.count >> 32) as u32,
            TAV_OFF => (self.count & 0xFFFF_FFFF) as u32,
            TBV_OFF => (self.count >> 32) as u32,
            TAPV_OFF => self.pre,
            RIS_OFF => self.ris(view),
            MIS_OFF => self.ris(view) & view.get(IMR_OFF),
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
        // `TAR`/`TBR`/`TAPS`/`TBPS`/`TAPV`/`TBPV`/`RIS`/`MIS` are RO.
        if matches!(offset, TAR_OFF | TBR_OFF | TAPV_OFF | RIS_OFF | MIS_OFF) {
            return view.get(offset);
        }
        if offset == ICR_OFF {
            let word = view.merged(offset, val, shift);
            if word & RIS_TATO != 0 {
                self.timeout_latch = false;
            }
            if word & RIS_TAM != 0 {
                self.match_latch = false;
            }
            self.refresh(view);
            return view.get(offset);
        }
        if offset == TAV_OFF || offset == TBV_OFF {
            // Counter preload: assemble the lane write, then load.
            let cur = if offset == TAV_OFF {
                (self.count & 0xFFFF_FFFF) as u32
            } else {
                (self.count >> 32) as u32
            };
            let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
            if offset == TAV_OFF {
                self.count = (self.count & !0xFFFF_FFFF) | u64::from(word);
            } else {
                self.count = (u64::from(word) << 32) | (self.count & 0xFFFF_FFFF);
            }
            self.check_match(view);
            self.refresh(view);
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        if offset == CTL_OFF {
            let was = view.get(CTL_OFF);
            let now_enabled = word & CTL_TAEN != 0;
            let was_enabled = was & CTL_TAEN != 0;
            if now_enabled && !was_enabled {
                // 0→1: reload, clear the one-shot halt. Up-counters
                // start from 0 (the interval is the upper bound).
                let up = self.up(view);
                self.count = if up {
                    0
                } else {
                    self.reload_value(view) & self.mask()
                };
                self.pre = 0;
                self.stopped = false;
                self.check_match(view);
            }
            view.set(offset, word);
            self.refresh(view);
            return word;
        }
        if offset == TAILR_OFF || offset == TBILR_OFF || offset == TAPR_OFF {
            view.set(offset, word);
            // `TAILD=0` reloads the counter on the next cycle (≈ now);
            // with `TAILD=1` the new interval waits for the next timeout.
            if view.get(TAMR_OFF) & TAMR_TAILD == 0 {
                self.count = self.reload_value(view) & self.mask();
                self.check_match(view);
            }
            self.refresh(view);
            return word;
        }
        word
    }
}

/// A Tiva C GPTM (Timer A concatenated only; Timer B latched).
pub struct Gptm {
    inner: RegisterFile<TimerBehavior>,
}

impl Gptm {
    /// A new timer on `D`'s tables (`"TIMER0"`…`"WTIMER5"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain timer");
        let wide = name.starts_with("WTIMER");
        let mut timer = Self {
            inner: RegisterFile::new(
                def,
                TimerBehavior {
                    wide,
                    count: 0xFFFF_FFFF,
                    pre: 0,
                    stopped: false,
                    timeout_latch: false,
                    match_latch: false,
                    pending: false,
                },
            ),
        };
        timer.stamp_resets();
        timer
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }

    /// Current counter value (test hook).
    #[must_use]
    pub fn count(&mut self) -> u64 {
        self.inner.with_view(|_, b| b.count)
    }
}

impl Peripheral for Gptm {
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
        self.stamp_resets();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Gptm {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        u32::from(self.inner.behavior().pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const TIMER0_BASE: u32 = 0x4003_0000;
    const WTIMER0_BASE: u32 = 0x4003_6000;

    fn timer0() -> Gptm {
        Gptm::for_device::<Tm4c123gh6pm>("TIMER0")
    }

    fn read_word(timer: &mut Gptm, base: u32, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(timer.read(base + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(timer: &mut Gptm, base: u32, offset: u32, value: u32) {
        for lane in 0..4 {
            timer
                .write(base + offset + lane, ((value >> (8 * lane)) & 0xFF) as u8)
                .unwrap();
        }
    }

    /// Datasheet resets: interval/match/count all `0xFFFF_FFFF`.
    #[test]
    fn datasheet_resets() {
        let mut t = timer0();
        assert_eq!(read_word(&mut t, TIMER0_BASE, TAILR_OFF), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, TIMER0_BASE, TAMATCHR_OFF), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, TIMER0_BASE, TAR_OFF), 0xFFFF_FFFF);
    }

    /// Periodic down-count: enable reloads, timeout fires every wrap,
    /// `ICR` clears, counter re-arms.
    #[test]
    fn periodic_timeout_flow() {
        let mut t = timer0();
        write_word(&mut t, TIMER0_BASE, TAMR_OFF, TAMR_PERIODIC);
        write_word(&mut t, TIMER0_BASE, TAILR_OFF, 100);
        write_word(&mut t, TIMER0_BASE, IMR_OFF, RIS_TATO);
        assert_eq!(t.pending(), 0);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        assert_eq!(t.count(), 100);
        t.tick(99);
        assert_eq!(t.count(), 1);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(t.count(), 0);
        assert_eq!(read_word(&mut t, TIMER0_BASE, RIS_OFF) & RIS_TATO, RIS_TATO);
        assert_eq!(t.pending(), 1);
        assert_eq!(t.exception_number(), Some(35));
        // Clear and re-arm: next wrap fires again.
        write_word(&mut t, TIMER0_BASE, ICR_OFF, RIS_TATO);
        assert_eq!(t.pending(), 0);
        t.tick(101);
        assert_eq!(t.pending(), 1);
    }

    /// One-shot stops at zero until re-enabled.
    #[test]
    fn one_shot_stops() {
        let mut t = timer0();
        write_word(&mut t, TIMER0_BASE, TAMR_OFF, TAMR_ONE_SHOT);
        write_word(&mut t, TIMER0_BASE, TAILR_OFF, 10);
        write_word(&mut t, TIMER0_BASE, IMR_OFF, RIS_TATO);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        t.tick(10);
        assert_eq!(t.count(), 0);
        assert_eq!(t.pending(), 1);
        t.tick(1000);
        assert_eq!(t.count(), 0);
        // Re-enable restarts from the interval.
        write_word(&mut t, TIMER0_BASE, CTL_OFF, 0);
        write_word(&mut t, TIMER0_BASE, ICR_OFF, RIS_TATO);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        assert_eq!(t.count(), 10);
        assert_eq!(t.pending(), 0);
    }

    /// Match fires when the counter passes through `TAMATCHR`.
    #[test]
    fn match_flow() {
        let mut t = timer0();
        write_word(&mut t, TIMER0_BASE, TAMR_OFF, TAMR_PERIODIC | TAMR_TAMIE);
        write_word(&mut t, TIMER0_BASE, TAILR_OFF, 100);
        write_word(&mut t, TIMER0_BASE, TAMATCHR_OFF, 90);
        write_word(&mut t, TIMER0_BASE, IMR_OFF, RIS_TAM);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        t.tick(10);
        assert_eq!(read_word(&mut t, TIMER0_BASE, RIS_OFF) & RIS_TAM, RIS_TAM);
        assert_eq!(t.pending(), 1);
        write_word(&mut t, TIMER0_BASE, ICR_OFF, RIS_TAM);
        assert_eq!(t.pending(), 0);
    }

    /// Prescaler divides the tick rate: `TAPR=3` needs 4 ticks per count.
    #[test]
    fn prescaler_divides() {
        let mut t = timer0();
        write_word(&mut t, TIMER0_BASE, TAMR_OFF, TAMR_PERIODIC);
        write_word(&mut t, TIMER0_BASE, TAILR_OFF, 10);
        write_word(&mut t, TIMER0_BASE, TAPR_OFF, 3);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        t.tick(3);
        assert_eq!(t.count(), 10);
        t.tick(1);
        assert_eq!(t.count(), 9);
    }

    /// Up-counting wraps at the interval back to zero.
    #[test]
    fn up_count_wraps() {
        let mut t = timer0();
        write_word(&mut t, TIMER0_BASE, TAMR_OFF, TAMR_PERIODIC | TAMR_TACDIR);
        write_word(&mut t, TIMER0_BASE, TAILR_OFF, 4);
        write_word(&mut t, TIMER0_BASE, CTL_OFF, CTL_TAEN);
        // Up-counting starts from 0 regardless of the interval load.
        t.tick(4);
        assert_eq!(t.count(), 0);
        assert_eq!(read_word(&mut t, TIMER0_BASE, RIS_OFF) & RIS_TATO, RIS_TATO);
    }

    /// Wide timers assemble 64-bit intervals across `TAILR`/`TBILR`.
    #[test]
    fn wide_timer_is_64bit() {
        let mut t = Gptm::for_device::<Tm4c123gh6pm>("WTIMER0");
        write_word(&mut t, WTIMER0_BASE, TAMR_OFF, TAMR_PERIODIC);
        write_word(&mut t, WTIMER0_BASE, TAILR_OFF, 0xFFFF_FFFF);
        write_word(&mut t, WTIMER0_BASE, TBILR_OFF, 0x0000_0001);
        write_word(&mut t, WTIMER0_BASE, CTL_OFF, CTL_TAEN);
        assert_eq!(t.count(), 0x1_FFFF_FFFF);
        assert_eq!(read_word(&mut t, WTIMER0_BASE, TAR_OFF), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, WTIMER0_BASE, TBR_OFF), 0x0000_0001);
    }
}

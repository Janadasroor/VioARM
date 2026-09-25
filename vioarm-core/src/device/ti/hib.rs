//! TI Tiva C hibernation module (`HIB`, RTC subset).
//!
//! RTC operation follows the TM4C123GH6PM datasheet (SPMS376E §7):
//! `RTCC` counts seconds once `RTCEN` sets, loaded from `RTCLD`, matching
//! `RTCM0` into the `RTCALT0` latch only when `RTCSSC` also matches
//! `RTCSSM` in `HIBRTCSS` (p516), behind `IM` with `IC` W1C clear;
//! `RTCT` trim and `HIBDATA` latch; `HIBCTL` reset `0x8000_2000` (`WRC`
//! reads 1 — instant write completion, documented — plus the `CLK32EN`
//! default); `HIBREQ`/power-domain bits sink (no power domains in
//! simulation; documented).
//!
//! The RTC runs on the 32.768 kHz oscillator, which has no coupling to
//! the device tick domain: it advances one second per 488 device ticks
//! (16 MHz nominal `PIOSC` ÷ 32768 — the reset clock — documented
//! approximation), with `RTCSS` tracking the fractional remainder.
//! `LOWBAT`/`EXTW`/`WC` never fire, `RTCALT1` has no match register on
//! this die (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const RTCC_OFF: u32 = 0x000;
const RTCM0_OFF: u32 = 0x004;
const RTCLD_OFF: u32 = 0x00C;
const CTL_OFF: u32 = 0x010;
const IM_OFF: u32 = 0x014;
const RIS_OFF: u32 = 0x018;
const MIS_OFF: u32 = 0x01C;
const IC_OFF: u32 = 0x020;
const RTCSS_OFF: u32 = 0x028;
/// `CTL` bits: `WRC` 31, `RTCEN` 0.
const CTL_WRC: u32 = 1 << 31;
const CTL_RTCEN: u32 = 1 << 0;
/// `RIS`/`IM`/`IC` bit: `RTCALT0` 0.
const RIS_RTCALT0: u32 = 1 << 0;
/// Device ticks per RTC second at the nominal 16 MHz reset clock.
const TICKS_PER_SECOND: u64 = 488;
/// Sub-second ticks per `RTCSS` unit (32768ths of a second).
const SUBSECOND_DIV: u64 = 32768;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[
    (RTCM0_OFF, 0xFFFF_FFFF),
    (CTL_OFF, 0x8000_2000),
    (0x024, 0x0000_7FFF), // RTCT
];

/// Behavior behind the HIB window (RTC subset).
#[derive(Debug, Default)]
pub struct HibBehavior {
    rtc: u32,
    /// Fractional-tick remainder toward the next second.
    remainder: u64,
    rtcmatch: bool,
    pending: bool,
}

impl HibBehavior {
    fn running(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(CTL_OFF) & CTL_RTCEN != 0
    }

    fn ris(&self) -> u32 {
        u32::from(self.rtcmatch)
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris() & view.get(IM_OFF) & RIS_RTCALT0 != 0;
    }
}

impl Behavior for HibBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.running(view) {
            return;
        }
        let mut total = self.remainder + delta;
        while total >= TICKS_PER_SECOND {
            total -= TICKS_PER_SECOND;
            self.rtc = self.rtc.wrapping_add(1);
        }
        self.remainder = total;
        // `RTCALT0` latches only when the seconds match `RTCM0` and the
        // sub-seconds match `RTCSSM` (HIBRTCSS p516).
        let rtcssc = ((self.remainder * SUBSECOND_DIV / TICKS_PER_SECOND) & 0x7FFF) as u32;
        let rtcssm = (view.get(RTCSS_OFF) >> 16) & 0x7FFF;
        if self.rtc == view.get(RTCM0_OFF) && rtcssc == rtcssm {
            self.rtcmatch = true;
        }
        self.refresh(view);
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            RTCC_OFF => self.rtc,
            RTCSS_OFF => {
                // `RTCSSC` (14:0) is the live sub-second count; `RTCSSM`
                // (30:16) is the latched match value.
                let rtcssc = ((self.remainder * SUBSECOND_DIV / TICKS_PER_SECOND) & 0x7FFF) as u32;
                (view.get(RTCSS_OFF) & 0x7FFF_0000) | rtcssc
            }
            CTL_OFF => value | CTL_WRC,
            RIS_OFF => self.ris(),
            MIS_OFF => self.ris() & view.get(IM_OFF) & RIS_RTCALT0,
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
        if offset == RTCC_OFF || offset == RIS_OFF || offset == MIS_OFF {
            return view.get(offset);
        }
        if offset == RTCSS_OFF {
            // `RTCSSM` (30:16) is RW; `RTCSSC` (14:0) and reserved bits sink.
            let cur = view.get(offset);
            let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
            let new = (cur & !0x7FFF_0000) | (word & 0x7FFF_0000);
            view.set(offset, new);
            self.refresh(view);
            return new;
        }
        if offset == IC_OFF {
            let word = view.merged(offset, val, shift);
            if word & RIS_RTCALT0 != 0 {
                self.rtcmatch = false;
            }
            self.refresh(view);
            return view.get(offset);
        }
        if offset == RTCLD_OFF {
            // Loading the counter takes effect immediately.
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            self.rtc = word;
            self.remainder = 0;
            self.refresh(view);
            return word;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        self.refresh(view);
        word
    }
}

/// A Tiva C hibernation module (RTC subset).
pub struct Hib {
    inner: RegisterFile<HibBehavior>,
}

impl Hib {
    /// A new HIB on `D`'s tables (`"HIB"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"HIB"`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain HIB");
        let mut hib = Self {
            inner: RegisterFile::new(def, HibBehavior::default()),
        };
        hib.stamp_resets();
        hib
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

    /// Current RTC seconds (test hook).
    #[must_use]
    pub fn rtc(&mut self) -> u32 {
        self.inner.with_view(|_, b| b.rtc)
    }
}

impl Peripheral for Hib {
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

impl InterruptSource for Hib {
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

    const HIB_BASE: u32 = 0x400F_C000;

    fn hib() -> Hib {
        Hib::for_device::<Tm4c123gh6pm>("HIB")
    }

    fn read_word(hib: &mut Hib, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(hib.read(HIB_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(hib: &mut Hib, offset: u32, value: u32) {
        for lane in 0..4 {
            hib.write(
                HIB_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Datasheet resets: `WRC` reads 1, match register all-ones.
    #[test]
    fn datasheet_resets() {
        let mut hib = hib();
        assert_eq!(read_word(&mut hib, CTL_OFF) & CTL_WRC, CTL_WRC);
        assert_eq!(read_word(&mut hib, RTCM0_OFF), 0xFFFF_FFFF);
        assert_eq!(hib.exception_number(), Some(59));
    }

    /// `RTCLD` seeds the counter; `RTCEN` runs seconds; match fires.
    #[test]
    fn rtc_counts_and_matches() {
        let mut hib = hib();
        write_word(&mut hib, RTCLD_OFF, 1000);
        assert_eq!(hib.rtc(), 1000);
        write_word(&mut hib, RTCM0_OFF, 1002);
        write_word(&mut hib, IM_OFF, RIS_RTCALT0);
        write_word(&mut hib, CTL_OFF, CTL_RTCEN);
        assert_eq!(hib.pending(), 0);
        hib.tick(2 * TICKS_PER_SECOND);
        assert_eq!(hib.rtc(), 1002);
        assert_eq!(read_word(&mut hib, RIS_OFF) & RIS_RTCALT0, RIS_RTCALT0);
        assert_eq!(hib.pending(), RIS_RTCALT0);
        write_word(&mut hib, IC_OFF, RIS_RTCALT0);
        assert_eq!(hib.pending(), 0);
    }

    /// `RTCALT0` stays quiet when the sub-seconds mismatch (p516), even
    /// with `RTCC==RTCM0`.
    #[test]
    fn rtcalt0_needs_subsecond_match() {
        let mut hib = hib();
        write_word(&mut hib, RTCLD_OFF, 1000);
        write_word(&mut hib, RTCM0_OFF, 1002);
        write_word(&mut hib, RTCSS_OFF, 0x0100_0000); // `RTCSSM` = 0x100
        assert_eq!(read_word(&mut hib, RTCSS_OFF) & 0x7FFF_0000, 0x0100_0000);
        write_word(&mut hib, IM_OFF, RIS_RTCALT0);
        write_word(&mut hib, CTL_OFF, CTL_RTCEN);
        hib.tick(2 * TICKS_PER_SECOND); // `RTCC` reaches `RTCM0`, sub-seconds 0
        assert_eq!(hib.rtc(), 1002);
        assert_eq!(read_word(&mut hib, RIS_OFF) & RIS_RTCALT0, 0);
        assert_eq!(hib.pending(), 0);
    }

    /// Mid-second match: seconds already equal, sub-seconds reach `RTCSSM`.
    #[test]
    fn rtcalt0_midsecond_match() {
        let mut hib = hib();
        write_word(&mut hib, RTCLD_OFF, 1000);
        write_word(&mut hib, RTCM0_OFF, 1000);
        write_word(&mut hib, RTCSS_OFF, 0x4000_0000); // `RTCSSM` = 0x4000
        write_word(&mut hib, IM_OFF, RIS_RTCALT0);
        write_word(&mut hib, CTL_OFF, CTL_RTCEN);
        hib.tick(TICKS_PER_SECOND / 2); // remainder 244 -> `RTCSSC` 0x4000
        assert_eq!(read_word(&mut hib, RIS_OFF) & RIS_RTCALT0, RIS_RTCALT0);
        assert_eq!(hib.pending(), RIS_RTCALT0);
        write_word(&mut hib, IC_OFF, RIS_RTCALT0);
        assert_eq!(hib.pending(), 0);
    }

    /// Held in reset (`RTCEN=0`), the counter does not advance.
    #[test]
    fn disabled_holds() {
        let mut hib = hib();
        hib.tick(10 * TICKS_PER_SECOND);
        assert_eq!(hib.rtc(), 0);
    }
}

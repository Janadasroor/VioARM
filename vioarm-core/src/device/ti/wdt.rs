//! TI Tiva C watchdog timers (`WATCHDOG0`-`WATCHDOG1`).
//!
//! Down-count operation follows the TM4C123GH6PM datasheet (SPMS376E
//! §12): `LOAD` interval with `VALUE` counting down from reset at one
//! device tick per cycle, first time-out raising the interrupt latch when
//! `INTEN` sets, a second unserviced time-out driving the shared
//! watchdog-reset flag when `RESEN` sets (the counter reloads from `LOAD`
//! on every time-out, like silicon). Any `ICR` write clears the latch and reloads.
//! `INTEN` is sticky-once-set (further writes to the bit ignored until
//! reset), `WRC` reads 1 on `WATCHDOG1` (instant write completion,
//! documented — real WDT1 needs inter-access gaps) and 0 on `WATCHDOG0`
//! (reserved), and `LOCK` (`0x1ACC_E551` unlocks; reads return lock status)
//! gates every register but `ICR` and `TEST` (datasheet `WDTLOCK` page:
//! a non-key lock value reapplies the lock "except for the Watchdog Test
//! (`WDTTEST`) register", and locking "does not affect the `WDTICR`
//! register"). Both modules share vector 34
//! (datasheet Table 2-9), matching the SVD's inherited IRQ 18.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const LOAD_OFF: u32 = 0x000;
const VALUE_OFF: u32 = 0x004;
const CTL_OFF: u32 = 0x008;
const ICR_OFF: u32 = 0x00C;
const RIS_OFF: u32 = 0x010;
const MIS_OFF: u32 = 0x014;
const TEST_OFF: u32 = 0x418;
const LOCK_OFF: u32 = 0xC00;

/// `CTL` bits: `WRC` 31 (WDT1 only), `INTTYPE` 2, `RESEN` 1, `INTEN` 0.
const CTL_WRC: u32 = 1 << 31;
const CTL_RESEN: u32 = 1 << 1;
const CTL_INTEN: u32 = 1 << 0;
/// `WDTLOCK` unlock key.
const LOCK_KEY: u32 = 0x1ACC_E551;
/// `WDTLOCK` read: locked.
const LOCK_LOCKED: u32 = 0x0000_0001;

/// Nonzero datasheet resets stamped over the SVD zeros (`WDTLOCK` resets
/// `0x0000_0000`, unlocked, so it needs no stamp).
const RESETS: &[(u32, u32)] = &[(LOAD_OFF, 0xFFFF_FFFF), (VALUE_OFF, 0xFFFF_FFFF)];

/// Behavior behind one watchdog window.
#[derive(Debug)]
pub struct WdtBehavior {
    /// `true` for `WATCHDOG1` (`WRC` reads 1; WDT0 reserves the bit).
    wdt1: bool,
    count: u32,
    /// Interrupt latch (first time-out awaits service).
    latched: bool,
    unlocked: bool,
    reset_out: Arc<Mutex<bool>>,
    /// Cached `(RIS & INTEN) != 0` (`InterruptSource::pending` is `&self`).
    pending: bool,
}

impl WdtBehavior {
    fn reload(&self, view: &Registers<&mut [u32]>) -> u32 {
        view.get(LOAD_OFF)
    }

    /// Time-out effects: with `INTEN`, latch the interrupt and — on a
    /// second unserviced time-out with `RESEN` — drive reset. Without
    /// `INTEN` the counter wraps silently (no interrupt, no reset).
    fn fire(&mut self, view: &Registers<&mut [u32]>) {
        if view.get(CTL_OFF) & CTL_INTEN == 0 {
            return;
        }
        if self.latched && view.get(CTL_OFF) & CTL_RESEN != 0 {
            if let Ok(mut flag) = self.reset_out.lock() {
                *flag = true;
            }
        }
        self.latched = true;
    }

    fn ris(&self) -> u32 {
        u32::from(self.latched)
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.latched && view.get(CTL_OFF) & CTL_INTEN != 0;
    }
}

impl Behavior for WdtBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        // The counter runs from reset; `INTEN` gates interrupt + reset.
        let mut steps = delta;
        while steps > 0 {
            steps -= 1;
            if self.count == 0 {
                self.fire(view);
                self.count = self.reload(view);
            } else {
                self.count -= 1;
                if self.count == 0 {
                    self.fire(view);
                    self.count = self.reload(view);
                }
            }
        }
        self.refresh(view);
    }

    fn reset(&mut self) {
        let (wdt1, reset_out) = (self.wdt1, self.reset_out.clone());
        *self = Self {
            wdt1,
            count: 0xFFFF_FFFF,
            latched: false,
            // `WDTLOCK` resets `0x0000_0000`: unlocked (p786).
            unlocked: true,
            reset_out,
            pending: false,
        };
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            VALUE_OFF => self.count,
            CTL_OFF => {
                if self.wdt1 {
                    value | CTL_WRC
                } else {
                    value & !CTL_WRC
                }
            }
            RIS_OFF => self.ris(),
            MIS_OFF => u32::from(self.latched && view.get(CTL_OFF) & CTL_INTEN != 0),
            LOCK_OFF => {
                if self.unlocked {
                    0
                } else {
                    LOCK_LOCKED
                }
            }
            ICR_OFF => 0, // WO: reads indeterminate, model 0.
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
        // `ICR` always services, even locked: clear + reload.
        if offset == ICR_OFF {
            self.latched = false;
            self.count = self.reload(view);
            self.refresh(view);
            return view.get(offset);
        }
        if offset == LOCK_OFF {
            let cur = view.get(offset);
            let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
            view.set(offset, word);
            self.unlocked = word == LOCK_KEY;
            return word;
        }
        if !self.unlocked && offset != TEST_OFF {
            return view.get(offset);
        }
        if offset == CTL_OFF {
            // `INTEN` is sticky-once-set; `WRC` is RO.
            let cur = view.get(offset);
            let mut word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
            word &= !CTL_WRC;
            if cur & CTL_INTEN != 0 {
                word |= CTL_INTEN;
            }
            view.set(offset, word);
            self.refresh(view);
            return word;
        }
        if offset == RIS_OFF || offset == MIS_OFF || offset == VALUE_OFF {
            return view.get(offset);
        }
        if offset == LOAD_OFF {
            // A `LOAD` write immediately reloads and restarts the counter.
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            self.count = word;
            self.refresh(view);
            return word;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        self.refresh(view);
        word
    }
}

/// A Tiva C watchdog timer.
pub struct Wdt {
    inner: RegisterFile<WdtBehavior>,
}

impl Wdt {
    /// A new watchdog on `D`'s tables (`"WATCHDOG0"`/`"WATCHDOG1"`),
    /// driving `reset_out` on a `RESEN` second time-out.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str, reset_out: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral(name).expect("generated tables contain watchdog");
        let wdt1 = name == "WATCHDOG1";
        let mut wdt = Self {
            inner: RegisterFile::new(
                def,
                WdtBehavior {
                    wdt1,
                    count: 0xFFFF_FFFF,
                    latched: false,
                    // `WDTLOCK` resets `0x0000_0000`: unlocked (p786).
                    unlocked: true,
                    reset_out,
                    pending: false,
                },
            ),
        };
        wdt.stamp_resets();
        wdt
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
    pub fn count(&mut self) -> u32 {
        self.inner.with_view(|_, b| b.count)
    }
}

impl Peripheral for Wdt {
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

impl InterruptSource for Wdt {
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

    const WDT0_BASE: u32 = 0x4000_0000;

    fn wdt0() -> (Wdt, Arc<Mutex<bool>>) {
        let flag = Arc::new(Mutex::new(false));
        (
            Wdt::for_device::<Tm4c123gh6pm>("WATCHDOG0", flag.clone()),
            flag,
        )
    }

    fn read_word(wdt: &mut Wdt, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(wdt.read(WDT0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(wdt: &mut Wdt, offset: u32, value: u32) {
        for lane in 0..4 {
            wdt.write(
                WDT0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Unlock helper: `LOCK_KEY` word-store.
    fn unlock(wdt: &mut Wdt) {
        write_word(wdt, LOCK_OFF, LOCK_KEY);
        assert_eq!(read_word(wdt, LOCK_OFF), 0);
    }

    /// Datasheet resets land on the bus (`WDTLOCK` resets unlocked, p786).
    #[test]
    fn datasheet_resets_and_lock() {
        let (mut wdt, _) = wdt0();
        assert_eq!(read_word(&mut wdt, LOAD_OFF), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut wdt, VALUE_OFF), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut wdt, LOCK_OFF), 0);
        // Unlocked out of reset: `LOAD` writes land.
        write_word(&mut wdt, LOAD_OFF, 100);
        assert_eq!(read_word(&mut wdt, LOAD_OFF), 100);
        // A non-key `LOCK` write relocks: `LOAD` writes sink again.
        write_word(&mut wdt, LOCK_OFF, 0x0000_0000);
        assert_eq!(read_word(&mut wdt, LOCK_OFF), LOCK_LOCKED);
        write_word(&mut wdt, LOAD_OFF, 200);
        assert_eq!(read_word(&mut wdt, LOAD_OFF), 100);
    }

    /// Locked state excepts `WDTTEST`, and `WDTICR` always services (p786).
    #[test]
    fn lock_excepts_test_and_icr_services() {
        let (mut wdt, _) = wdt0();
        write_word(&mut wdt, LOCK_OFF, 0x0000_0000);
        assert_eq!(read_word(&mut wdt, LOCK_OFF), LOCK_LOCKED);
        // `TEST` writes land while locked; other registers sink.
        write_word(&mut wdt, TEST_OFF, 0x0000_0100);
        assert_eq!(read_word(&mut wdt, TEST_OFF), 0x0000_0100);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN);
        assert_eq!(read_word(&mut wdt, CTL_OFF) & CTL_INTEN, 0);
        // Latch an interrupt, relock, then service it via `ICR` while locked.
        unlock(&mut wdt);
        write_word(&mut wdt, LOAD_OFF, 4);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN);
        wdt.tick(4);
        assert_eq!(wdt.pending(), 1);
        write_word(&mut wdt, LOCK_OFF, 0x0);
        write_word(&mut wdt, ICR_OFF, 0x01);
        assert_eq!(wdt.pending(), 0);
        assert_eq!(wdt.count(), 4);
    }

    /// First time-out latches the interrupt; `ICR` clears and reloads.
    #[test]
    fn timeout_latches_and_services() {
        let (mut wdt, _) = wdt0();
        unlock(&mut wdt);
        write_word(&mut wdt, LOAD_OFF, 10);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN);
        assert_eq!(wdt.pending(), 0);
        wdt.tick(10);
        assert_eq!(wdt.count(), 10); // reloaded from LOAD
        assert_eq!(read_word(&mut wdt, RIS_OFF), 1);
        assert_eq!(read_word(&mut wdt, MIS_OFF), 1);
        assert_eq!(wdt.pending(), 1);
        assert_eq!(wdt.exception_number(), Some(34));
        write_word(&mut wdt, ICR_OFF, 0xDEAD_BEEF); // any value, even locked
        assert_eq!(wdt.pending(), 0);
        assert_eq!(wdt.count(), 10);
    }

    /// `INTEN` is sticky: once set, clear-writes are ignored.
    #[test]
    fn inten_is_sticky() {
        let (mut wdt, _) = wdt0();
        unlock(&mut wdt);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN);
        write_word(&mut wdt, CTL_OFF, 0x00);
        assert_eq!(read_word(&mut wdt, CTL_OFF) & CTL_INTEN, CTL_INTEN);
    }

    /// Second unserviced time-out with `RESEN` drives the reset output.
    #[test]
    fn resen_second_timeout_resets() {
        let (mut wdt, flag) = wdt0();
        unlock(&mut wdt);
        write_word(&mut wdt, LOAD_OFF, 4);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN | CTL_RESEN);
        wdt.tick(4);
        assert!(!*flag.lock().unwrap()); // first: interrupt only
        wdt.tick(4);
        assert!(*flag.lock().unwrap()); // second: reset
    }

    /// Without `RESEN`, repeated time-outs only re-latch the interrupt.
    #[test]
    fn no_resen_no_reset() {
        let (mut wdt, flag) = wdt0();
        unlock(&mut wdt);
        write_word(&mut wdt, LOAD_OFF, 4);
        write_word(&mut wdt, CTL_OFF, CTL_INTEN);
        wdt.tick(12);
        assert!(!*flag.lock().unwrap());
        assert_eq!(wdt.pending(), 1);
    }

    /// `WDT1` reports `WRC=1`; `WDT0` reserves the bit at 0.
    #[test]
    fn wrc_differs_per_instance() {
        let (mut wdt, _) = wdt0();
        assert_eq!(read_word(&mut wdt, CTL_OFF) & CTL_WRC, 0);
        let flag = Arc::new(Mutex::new(false));
        let mut wdt1 = Wdt::for_device::<Tm4c123gh6pm>("WATCHDOG1", flag);
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(wdt1.read(0x4000_1000 + CTL_OFF + lane).unwrap()) << (8 * lane);
        }
        assert_eq!(v & CTL_WRC, CTL_WRC);
    }
}

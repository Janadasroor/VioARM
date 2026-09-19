//! The STM32 independent and window watchdogs (`IWDG`, `WWDG`), driven by the
//! SVD-generated register tables.
//!
//! The `IWDG` is a 12-bit down-counter on the (unmodeled) LSI. Writing the
//! key `0x5555` to `KR` unlocks `PR`/`RLR`; `0xAAAA` reloads the counter and
//! `0xCCCC` starts it. The counter decrements once every `2^PR` machine cycles
//! and, on reaching zero, latches the shared watchdog reset so the `Machine`
//! reboots the whole part — exactly what an un-refreshed watchdog does on
//! silicon. There is no way to stop it once started.
//!
//! The `WWDG` is a 7-bit down-counter gated by `WDGA` in `CR`, clocked on
//! `PCLK/4096` divided by `2^WDGTB`. Each refresh (a write to `CR`) must
//! happen inside the window set by `CFR.W`: refreshing above it, or
//! letting the counter underflow past `0x40`, also latches the watchdog
//! reset. When the counter reaches `0x40` the `SR.EWIF` flag is set
//! and — with `CFR.EWI` enabled — the WWDG interrupt (IRQ 0) raises, giving
//! the handler thousands of steps to refresh before the reset at `0x3F`.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the IWDG, per the SVD.
const KR: u32 = 0x00;
const PR: u32 = 0x04;
const RLR: u32 = 0x08;

/// IWDG key values (`KR.`KEY[15:0]``).
const KEY_ACCESS: u32 = 0x5555;
const KEY_RELOAD: u32 = 0xAAAA;
const KEY_START: u32 = 0xCCCC;

/// The independent-watchdog counting core.
#[derive(Clone, Debug)]
struct IwdgBehavior {
    reset: Arc<Mutex<bool>>,
    /// Whether `0x5555` has unlocked `PR`/`RLR` writes.
    access: bool,
    /// Whether `0xCCCC` has started the watchdog.
    running: bool,
    /// The 12-bit down-counter value.
    counter: u32,
    /// Prescaler phase within the current `2^PR` division.
    psc_phase: u64,
}

impl IwdgBehavior {
    const fn new(reset: Arc<Mutex<bool>>) -> Self {
        Self {
            reset,
            access: false,
            running: false,
            counter: 0,
            psc_phase: 0,
        }
    }
}

impl Behavior for IwdgBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            KR => {
                // KR is write-only; the key is the low 16 bits of the stored
                // word, which a halfword/word write assembles lane by lane.
                let word = view.merged(KR, val, shift);
                match word & 0xFFFF {
                    KEY_ACCESS => self.access = true,
                    KEY_RELOAD => {
                        if self.running {
                            self.counter = view.get(RLR) & 0x0FFF;
                        }
                        self.psc_phase = 0;
                    }
                    KEY_START => {
                        self.running = true;
                        self.access = false;
                        self.counter = view.get(RLR) & 0x0FFF;
                        self.psc_phase = 0;
                    }
                    _ => self.access = false,
                }
                word
            }
            // PR and RLR are only writable after the 0x5555 key.
            PR | RLR if self.access => view.merged(offset, val, shift),
            PR | RLR => view.get(offset),
            _ => view.merged(offset, val, shift),
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.running {
            return;
        }
        let div = 1u64 << (view.get(PR) & 0x7);
        self.psc_phase += delta;
        let mut steps = self.psc_phase / div;
        self.psc_phase %= div;
        let mut cnt = self.counter;
        while steps > 0 && cnt > 0 {
            cnt -= 1;
            steps -= 1;
        }
        if steps > 0 {
            // The counter hit zero: the watchdog underflows and resets the MCU.
            *self.reset.lock().unwrap_or_else(|e| e.into_inner()) = true;
            cnt = view.get(RLR) & 0x0FFF;
        }
        self.counter = cnt;
    }

    fn reset(&mut self) {
        self.access = false;
        self.running = false;
        self.counter = 0;
        self.psc_phase = 0;
    }
}

/// Register offsets within the WWDG, per the SVD.
const WCR: u32 = 0x00;
const CFR: u32 = 0x04;
const WSR: u32 = 0x08;

/// `WDGA` bit in `CR` — write-once: once set it cannot be cleared by software.
const WDGA: u32 = 1 << 7;
/// `EWI` bit in `CFR` (the early-wakeup interrupt enable).
const EWI_CFR: u32 = 1 << 9;
/// `EWIF` bit in `SR`.
const EWIF: u32 = 1 << 0;

/// The window-watchdog counting core.
#[derive(Clone, Debug)]
struct WwdgBehavior {
    reset: Arc<Mutex<bool>>,
    /// The 7-bit down-counter value.
    counter: u8,
    /// Prescaler phase within the current `2^WDGTB` division.
    psc_phase: u64,
}

impl WwdgBehavior {
    const fn new(reset: Arc<Mutex<bool>>) -> Self {
        Self {
            reset,
            counter: 0x7F,
            psc_phase: 0,
        }
    }
}

impl Behavior for WwdgBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            WSR => {
                // Any write clears EWIF (and with it the WWDG interrupt).
                0
            }
            WCR if shift == 0 => {
                let was_running = view.get(WCR) & WDGA != 0;
                // WDGA latches: once set, a refresh write cannot clear it.
                let word = if was_running {
                    view.merged(WCR, val, shift) | WDGA
                } else {
                    view.merged(WCR, val, shift)
                };
                if word & WDGA != 0 {
                    if was_running {
                        // A refresh above the window is an immediate reset.
                        let window = view.get(CFR) & 0x7F;
                        if u32::from(self.counter) & 0x7F > window {
                            *self.reset.lock().unwrap_or_else(|e| e.into_inner()) = true;
                        }
                    } else if word & 0x7F < 0x40 {
                        // Enabling with the counter already below the edge
                        // resets immediately, like silicon.
                        *self.reset.lock().unwrap_or_else(|e| e.into_inner()) = true;
                    }
                    // Both enable and refresh load the written T bits.
                    self.counter = (word & 0x7F) as u8;
                    self.psc_phase = 0;
                }
                word
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if view.get(WCR) & WDGA == 0 {
            return;
        }
        // The counter runs on PCLK/4096 divided by 2^WDGTB (RM0008
        // §20.3.2). The base prescaler matters beyond timing: it keeps
        // thousands of steps between the 0x40 early wakeup and the 0x3F
        // underflow, so an armed EWI always gets its interrupt before a
        // reset can preempt it.
        let div = 4096u64 * (1u64 << ((view.get(CFR) >> 7) & 0x3));
        self.psc_phase += delta;
        let mut steps = self.psc_phase / div;
        self.psc_phase %= div;
        while steps > 0 {
            steps -= 1;
            self.counter = self.counter.wrapping_sub(1) & 0x7F;
            if self.counter == 0x40 {
                // The counter reached the window edge: set EWIF, which raises
                // the WWDG interrupt when CFR.EWI is enabled.
                view.set(WSR, view.get(WSR) | EWIF);
            } else if self.counter == 0x3F {
                // Underflow past the window: reset the MCU.
                *self.reset.lock().unwrap_or_else(|e| e.into_inner()) = true;
                self.counter = 0x7F;
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(WSR) & EWIF != 0 && view.get(CFR) & EWI_CFR != 0)
    }

    fn reset(&mut self) {
        self.counter = 0x7F;
        self.psc_phase = 0;
    }
}

/// The independent watchdog, sharing a reset latch with the bus.
#[derive(Debug)]
pub struct Iwdg {
    inner: RegisterFile<IwdgBehavior>,
}

impl Iwdg {
    /// An `IWDG` at `0x4000_3000`, latching `reset` on underflow.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `IWDG`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new(reset: Arc<Mutex<bool>>) -> Self {
        Self::for_device::<stm32f103::Stm32f103>(reset)
    }

    /// An `IWDG` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `IWDG` peripheral.
    #[must_use]
    pub fn for_device<D: Device>(reset: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral("IWDG").expect("generated tables contain IWDG");
        Self {
            inner: RegisterFile::new(def, IwdgBehavior::new(reset)),
        }
    }
}

/// The window watchdog, sharing a reset latch with the bus.
#[derive(Debug)]
pub struct Wwdg {
    inner: RegisterFile<WwdgBehavior>,
}

impl Wwdg {
    /// A `WWDG` at `0x4000_2C00`, latching `reset` on underflow.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `WWDG`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new(reset: Arc<Mutex<bool>>) -> Self {
        Self::for_device::<stm32f103::Stm32f103>(reset)
    }

    /// A `WWDG` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `WWDG` peripheral.
    #[must_use]
    pub fn for_device<D: Device>(reset: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral("WWDG").expect("generated tables contain WWDG");
        Self {
            inner: RegisterFile::new(def, WwdgBehavior::new(reset)),
        }
    }
}

macro_rules! delegate {
    ($ty:ty) => {
        impl Peripheral for $ty {
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
                self.inner.name()
            }
        }

        impl InterruptSource for $ty {
            fn exception_number(&self) -> Option<u8> {
                self.inner.exception_number()
            }

            fn pending(&self) -> u32 {
                self.inner.pending()
            }

            fn acknowledge(&mut self, lines: u32) {
                self.inner.acknowledge(lines);
            }

            fn lines(&self) -> u32 {
                self.inner.lines()
            }

            fn exception_for_line(&self, line: u32) -> Option<u8> {
                self.inner.exception_for_line(line)
            }
        }
    };
}

delegate!(Iwdg);
delegate!(Wwdg);

#[cfg(test)]
mod tests {
    use super::{Iwdg, Wwdg};
    use crate::device::{InterruptSource, Peripheral};
    use std::sync::{Arc, Mutex};

    const IWDG_BASE: u32 = 0x4000_3000;
    const WWDG_BASE: u32 = 0x4000_2C00;
    /// One WWDG count at WDGTB=0 (PCLK/4096 base prescaler).
    const WTICK: u64 = 4096;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word<P: Peripheral>(p: &mut P, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word<P: Peripheral>(p: &mut P, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = p.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn latch() -> Arc<Mutex<bool>> {
        Arc::new(Mutex::new(false))
    }

    fn armed() -> (Iwdg, Arc<Mutex<bool>>) {
        let reset = latch();
        let mut i = Iwdg::new(reset.clone());
        write_word(&mut i, IWDG_BASE, 0x5555); // unlock
        write_word(&mut i, IWDG_BASE + 0x08, 0x0100); // RLR = 0x100
        write_word(&mut i, IWDG_BASE, 0xCCCC); // start
        (i, reset)
    }

    #[test]
    fn counts_down_and_underflows_into_a_reset() {
        let (mut i, reset) = armed();
        // RLR ticks bring the counter to zero; the next tick underflows.
        for _ in 0..0x100 {
            i.tick(1);
            assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
        }
        i.tick(1);
        assert!(
            *reset.lock().unwrap_or_else(|e| e.into_inner()),
            "underflow latches the reset"
        );
    }

    #[test]
    fn reload_key_refreshes_before_the_watchdog_fires() {
        let (mut i, reset) = armed();
        // Halfway down, the 0xAAAA key reloads the counter from RLR.
        for _ in 0..0x80 {
            i.tick(1);
        }
        write_word(&mut i, IWDG_BASE, 0xAAAA);
        // A full RLR period (0x100 ticks) brings the counter to zero; the
        // next tick underflows.
        for _ in 0..0x100 {
            i.tick(1);
            assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
        }
        i.tick(1);
        assert!(
            *reset.lock().unwrap_or_else(|e| e.into_inner()),
            "resets after the reloaded period"
        );
    }

    #[test]
    fn pr_and_rlr_need_the_access_key() {
        let mut i = Iwdg::new(latch());
        // Without the 0x5555 key, writes to PR and RLR are ignored.
        write_word(&mut i, IWDG_BASE + 0x04, 0x7);
        write_word(&mut i, IWDG_BASE + 0x08, 0x0FFF);
        assert_eq!(read_word(&mut i, IWDG_BASE + 0x04), 0);
        assert_eq!(read_word(&mut i, IWDG_BASE + 0x08), 0x0FFF); // reset value
        write_word(&mut i, IWDG_BASE, 0x5555);
        write_word(&mut i, IWDG_BASE + 0x04, 0x7);
        assert_eq!(read_word(&mut i, IWDG_BASE + 0x04), 0x7);
        // The access key is one-shot: after a start key it closes again.
        write_word(&mut i, IWDG_BASE, 0xCCCC);
        write_word(&mut i, IWDG_BASE + 0x04, 0x0);
        assert_eq!(read_word(&mut i, IWDG_BASE + 0x04), 0x7);
    }

    #[test]
    fn prescaler_divides_the_count() {
        let reset = latch();
        let mut i = Iwdg::new(reset.clone());
        write_word(&mut i, IWDG_BASE, 0x5555);
        write_word(&mut i, IWDG_BASE + 0x04, 0x1); // PR: divide by 2
        write_word(&mut i, IWDG_BASE + 0x08, 0x0010); // RLR = 0x10
        write_word(&mut i, IWDG_BASE, 0xCCCC);
        // 2 * RLR cycles brings the counter to zero without underflowing.
        for _ in 0..0x20 {
            i.tick(1);
            assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
        }
        i.tick(2);
        assert!(
            *reset.lock().unwrap_or_else(|e| e.into_inner()),
            "underflows at 2 cycles per count"
        );
    }

    #[test]
    fn wwdg_underflow_latches_reset_and_interrupt() {
        let reset = latch();
        let mut w = Wwdg::new(reset.clone());
        // CR = WDGA | T=0x7F starts it; the counter runs down from 0x7F.
        write_word(&mut w, WWDG_BASE, 0x80 | 0x7F);
        // Counter 0x41 -> 0x40: EWIF + interrupt; 0x40 -> 0x3F: reset.
        for _ in 0..0x3E {
            w.tick(WTICK);
            assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
            assert_eq!(w.pending(), 0);
        }
        w.tick(WTICK); // reaches 0x40
        assert_eq!(w.pending(), 0, "EWI is not yet enabled");
        w.tick(WTICK); // 0x40 -> 0x3F: reset
        assert!(*reset.lock().unwrap_or_else(|e| e.into_inner()));
    }

    #[test]
    fn wwdg_ewi_enabled_raises_an_interrupt_before_reset() {
        let reset = latch();
        let mut w = Wwdg::new(reset.clone());
        write_word(&mut w, WWDG_BASE + 0x04, 0x0200 | 0x7F); // CFR: EWI | W=0x7F
        write_word(&mut w, WWDG_BASE, 0x80 | 0x7F); // WDGA | T
        for _ in 0..0x3E {
            w.tick(WTICK);
        }
        w.tick(WTICK); // reaches 0x40
        assert_eq!(w.pending(), 1, "EWIF set and EWI enabled");
        assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
        // Writing SR clears EWIF and drops the interrupt.
        w.write(WWDG_BASE + 0x08, 0).unwrap();
        assert_eq!(w.pending(), 0);
        // The next step still resets.
        w.tick(WTICK);
        assert!(*reset.lock().unwrap_or_else(|e| e.into_inner()));
    }

    #[test]
    fn wwdg_refresh_inside_window_keeps_the_machine_alive() {
        let reset = latch();
        let mut w = Wwdg::new(reset.clone());
        write_word(&mut w, WWDG_BASE + 0x04, 0x7F); // CFR: W = 0x7F (whole window)
        write_word(&mut w, WWDG_BASE, 0x80 | 0x7F);
        for _ in 0..0x20 {
            w.tick(WTICK);
            assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
            // Refresh on the same counter value each iteration.
            write_word(&mut w, WWDG_BASE, 0x7F);
        }
        assert_eq!(read_word(&mut w, WWDG_BASE) & 0x80, 0x80, "WDGA latches");
    }

    #[test]
    fn wwdg_refresh_above_the_window_resets() {
        let reset = latch();
        let mut w = Wwdg::new(reset.clone());
        write_word(&mut w, WWDG_BASE + 0x04, 0x10); // CFR: W = 0x10
        write_word(&mut w, WWDG_BASE, 0x80 | 0x7F);
        // Counter is 0x7F, well above the window 0x10: a refresh resets.
        write_word(&mut w, WWDG_BASE, 0x7F);
        assert!(*reset.lock().unwrap_or_else(|e| e.into_inner()));
    }

    #[test]
    fn wwdg_register_layout() {
        assert_eq!(Wwdg::new(latch()).exception_number(), Some(16), "IRQ 0");
        assert_eq!(Iwdg::new(latch()).exception_number(), None, "no IWDG IRQ");
    }

    /// Enabling loads the written counter: below the edge resets at once,
    /// a mid-window value counts down from there.
    #[test]
    fn wwdg_enable_loads_counter_and_checks_the_edge() {
        let reset = latch();
        let mut w = Wwdg::new(reset.clone());
        write_word(&mut w, WWDG_BASE + 0x04, 0x7F); // CFR: whole window
        write_word(&mut w, WWDG_BASE, 0x80 | 0x50); // enable at T=0x50
        assert!(!*reset.lock().unwrap_or_else(|e| e.into_inner()));
        // 0x50 -> 0x40 is 16 ticks: EWIF appears without EWI.
        for _ in 0..16 {
            w.tick(WTICK);
        }
        assert_eq!(read_word(&mut w, WWDG_BASE + 0x08) & 1, 1, "EWIF at 0x40");
        // Enabling straight below the edge resets immediately.
        let reset2 = latch();
        let mut w2 = Wwdg::new(reset2.clone());
        write_word(&mut w2, WWDG_BASE + 0x04, 0x7F);
        write_word(&mut w2, WWDG_BASE, 0x80 | 0x30);
        assert!(*reset2.lock().unwrap_or_else(|e| e.into_inner()));
    }
}

//! TAMP tamper monitor + backup registers (G4 `TAMP@0x40002400`).
//!
//! The 32 `BKPxR` registers latch and survive system `reset()` like the
//! backup domain on silicon (only a tamper wipes them, unless that line's
//! `NOERASE` is set). Tamper lines 1–3 and internal tampers 3–6 follow
//! `CR1` enables: a host-driven event latches `SR` (`TAMPxF`/`ITAMPxF`),
//! wipes the backup registers unless masked by `CR2` `NOERASE`, and
//! raises the single line while the matching `IER` bit is set (`MISR`
//! reads the masked combination). `SCR` clears flags write-1-to-clear;
//! `CR2` mask/trigger and `FLTCR` latch (filter timings unmodeled).
//!
//! Host injection arrives through [`Peripheral::drive_inputs`] (one bit
//! per tamper line), the same channel debuggers use for pin stimulus —
//! real tampers come from pins and the RTC, which have no other model
//! here. Timestamp, monotonic counters, and secret erase-on-tamper stay
//! unmodeled (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const IER: u32 = 0x2C;
const SR: u32 = 0x30;
const MISR: u32 = 0x34;
const SCR: u32 = 0x3C;
const BKP0R: u32 = 0x100;

/// Tamper lines the model tracks: external 1–3 plus internal 3–6.
const LINES: [u32; 7] = [0, 1, 2, 18, 19, 20, 21];

#[derive(Clone, Debug, Default)]
struct TampBehavior {
    pending_tampers: u32,
}

impl TampBehavior {
    fn line_enabled(view: &Registers<&mut [u32]>, line: u32) -> bool {
        view.get(CR1) & (1 << line) != 0
    }

    fn noerase(view: &Registers<&mut [u32]>, line: u32) -> bool {
        // External lines 0–2 carry `TAMPxNOER`; internal tampers always
        // wipe (documented).
        line < 3 && view.get(CR2) & (1 << line) != 0
    }

    fn fire(&mut self, view: &mut Registers<&mut [u32]>, line: u32) {
        view.set(SR, view.get(SR) | (1 << line));
        if !Self::noerase(view, line) {
            for off in (0..32).map(|i| BKP0R + 4 * i) {
                view.set(off, 0);
            }
        }
    }
}

impl Behavior for TampBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            SCR => {
                // Write-1-to-clear into SR.
                let new = view.merged(SCR, val, shift);
                view.set(SR, view.get(SR) & !new);
                new
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            MISR => view.get(SR) & view.get(IER),
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        let fire = self.pending_tampers;
        self.pending_tampers = 0;
        for line in LINES {
            if fire & (1 << line) != 0 && Self::line_enabled(view, line) {
                self.fire(view, line);
            }
        }
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // One bit per tamper line; the enable gate applies at tick time
        // so configuration order does not matter.
        self.pending_tampers |= value;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(SR) & view.get(IER) != 0)
    }

    fn reset(&mut self) {
        // Backup registers survive system reset (backup domain); only
        // the live tamper state clears.
        self.pending_tampers = 0;
    }
}

/// TAMP tamper monitor.
#[derive(Debug)]
pub struct Tamp {
    inner: RegisterFile<TampBehavior>,
}

impl Tamp {
    /// A new `TAMP` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TAMP`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("TAMP").expect("generated tables contain TAMP");
        Self {
            inner: RegisterFile::new(def, TampBehavior::default()),
        }
    }
}

impl Peripheral for Tamp {
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
        // Backup domain: BKPxR survive system reset while everything
        // else returns to table resets.
        let saved: Vec<u32> = self
            .inner
            .with_view(|view, _| (0..32).map(|i| view.get(BKP0R + 4 * i)).collect());
        self.inner.reset();
        self.inner.with_view(|view, _| {
            for (i, w) in saved.iter().enumerate() {
                view.set(BKP0R + 4 * i as u32, *w);
            }
        });
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        "tamp"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
}

impl InterruptSource for Tamp {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32g474xx::Stm32g474;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4000_2400;
    const CR1_O: u32 = 0x00;
    const CR2_O: u32 = 0x04;
    const IER_O: u32 = 0x2C;
    const SR_O: u32 = 0x30;
    const MISR_O: u32 = 0x34;
    const SCR_O: u32 = 0x3C;

    #[allow(clippy::cast_possible_truncation)]
    fn w(t: &mut Tamp, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            t.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(t: &mut Tamp, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = t.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn tamp() -> Tamp {
        let mut t = Tamp::new_for::<Stm32g474>();
        t.reset();
        t
    }

    #[test]
    fn backup_persists_across_reset() {
        let mut t = tamp();
        w(&mut t, 0x100, 0x1234_5678);
        w(&mut t, 0x17C, 0xDEAD_BEEF);
        t.reset();
        assert_eq!(r(&mut t, 0x100), 0x1234_5678, "BKP0R survives");
        assert_eq!(r(&mut t, 0x17C), 0xDEAD_BEEF, "BKP31R survives");
    }

    #[test]
    fn tamper_flags_wipes_and_irqs() {
        let mut t = tamp();
        w(&mut t, 0x100, 0x1234_5678);
        w(&mut t, CR1_O, 1); // TAMP1E
        w(&mut t, IER_O, 1); // TAMP1IE
        t.drive_inputs(1);
        t.tick(1);
        assert_ne!(r(&mut t, SR_O) & 1, 0, "TAMP1F latches");
        assert_eq!(r(&mut t, 0x100), 0, "backup wiped");
        assert_ne!(r(&mut t, MISR_O) & 1, 0, "masked flag shows");
        assert_eq!(t.pending(), 1, "line raises");
        w(&mut t, SCR_O, 1);
        assert_eq!(r(&mut t, SR_O) & 1, 0, "SCR clears");
        assert_eq!(t.pending(), 0, "line drops");
    }

    #[test]
    fn disabled_tamper_is_ignored() {
        let mut t = tamp();
        w(&mut t, 0x100, 0x1234_5678);
        // TAMP1E clear: the event changes nothing.
        t.drive_inputs(1);
        t.tick(1);
        assert_eq!(r(&mut t, SR_O) & 1, 0, "no flag without enable");
        assert_eq!(r(&mut t, 0x100), 0x1234_5678, "no wipe without enable");
    }

    #[test]
    fn noerase_keeps_backup() {
        let mut t = tamp();
        w(&mut t, 0x100, 0x1234_5678);
        w(&mut t, CR1_O, 1);
        w(&mut t, CR2_O, 1); // TAMP1NOER
        t.drive_inputs(1);
        t.tick(1);
        assert_ne!(r(&mut t, SR_O) & 1, 0, "flag still latches");
        assert_eq!(r(&mut t, 0x100), 0x1234_5678, "backup kept");
    }

    #[test]
    fn internal_tamper_always_wipes() {
        let mut t = tamp();
        w(&mut t, 0x100, 0x1234_5678);
        w(&mut t, CR1_O, 1 << 18); // ITAMP3E
        w(&mut t, CR2_O, 0x7); // NOERASE on externals only
        t.drive_inputs(1 << 18);
        t.tick(1);
        assert_ne!(r(&mut t, SR_O) & (1 << 18), 0, "ITAMP3F latches");
        assert_eq!(r(&mut t, 0x100), 0, "backup wiped anyway");
    }
}

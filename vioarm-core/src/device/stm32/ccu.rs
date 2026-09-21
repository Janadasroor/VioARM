//! H7 FDCAN clock-calibration unit (`CAN_CCU@0x4000a800`).
//!
//! The CCU trims the FDCAN time-quanta clock: firmware programs `CCFG`
//! (`TQBT`/`CFL`/`OCPM`, or `BCC` to bypass calibration for the `CDIV`
//! divider), and polls `CSTAT.CALS` — `0` not calibrated, `0x40000000`
//! basic, `0x80000000` precision (the `FDCAN_CLOCK_*_CALIBRATED` states
//! from the HAL). With calibration on (`BCC == 0`) the model walks those
//! states over [`CAL_TICKS`] ticks each and raises `IR.CSC` on every
//! transition; `CCFG.SWR` is a self-clearing reset back to uncalibrated.
//! The calibration watchdog (`CWD.WDC` start value, `CWD.WDV` reload)
//! counts bus ticks while calibration is on and raises `IR.CWE` on
//! expiry, reloading from `WDV`; `CWD` reads back the live counter.
//! `IR` clears write-1-to-clear (the HAL clears CCU flags by writing them
//! back in `__HAL_FDCAN_CLEAR_FLAG`); the single line follows `IE`
//! (`CWEE`/`CSCE`). `CREL` latches; `CSTAT.OCPC`/`TQC` counters latch zero
//! (calibration traffic unmodeled, documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CCFG: u32 = 0x04;
const CSTAT: u32 = 0x08;
const CWD: u32 = 0x0C;
const IR: u32 = 0x10;
const IE: u32 = 0x14;

const BCC: u32 = 1 << 6;
const SWR: u32 = 1 << 31;
const CALS_MASK: u32 = 0x3 << 30;
const CALS_BASIC: u32 = 0x1 << 30;
const CALS_PRECISION: u32 = 0x2 << 30;
const IR_CWE: u32 = 1 << 0;
const IR_CSC: u32 = 1 << 1;
const IE_CWEE: u32 = 1 << 0;
const IE_CSCE: u32 = 1 << 1;

/// Ticks per calibration step (uncalibrated → basic → precision).
const CAL_TICKS: u64 = 8;

#[derive(Clone, Debug, Default)]
struct CcuBehavior {
    cal_at: Option<u64>,
    wd: u32,
    wd_reload: u32,
}

impl CcuBehavior {
    fn set_ir(view: &mut Registers<&mut [u32]>, bits: u32) {
        view.set(IR, view.get(IR) | bits);
    }

    fn calibrating(view: &mut Registers<&mut [u32]>) -> bool {
        view.get(CCFG) & BCC == 0
    }

    fn set_cals(view: &mut Registers<&mut [u32]>, cals: u32) {
        view.set(CSTAT, view.get(CSTAT) & !CALS_MASK | cals);
    }

    fn arm_cal(&mut self, view: &mut Registers<&mut [u32]>) {
        if Self::calibrating(view) && view.get(CSTAT) & CALS_MASK != CALS_PRECISION {
            self.cal_at = Some(CAL_TICKS);
        } else {
            self.cal_at = None;
        }
    }
}

impl Behavior for CcuBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            IR => {
                // Write-1-to-clear, per byte lane (like the M_CAN IR).
                view.get(IR) & !(u32::from(val) << shift)
            }
            CWD => {
                let new = view.merged(CWD, val, shift);
                self.wd = new & 0xFFFF;
                self.wd_reload = new >> 16;
                new
            }
            CCFG => {
                let mut new = view.merged(CCFG, val, shift);
                if new & SWR != 0 {
                    // Software reset: back to uncalibrated, watchdog
                    // reloaded. Self-clearing.
                    new &= !SWR;
                    Self::set_cals(view, 0);
                }
                self.arm_cal(view);
                new
            }
            CSTAT => {
                // CALS is engine-owned; firmware writes only reach the
                // counter fields.
                let new = view.merged(CSTAT, val, shift);
                new & !CALS_MASK | (view.get(CSTAT) & CALS_MASK)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CWD {
            return self.wd_reload << 16 | self.wd.min(0xFFFF);
        }
        let _ = view;
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !Self::calibrating(view) {
            self.cal_at = None;
            return;
        }
        let mut budget = delta;
        while let Some(left) = self.cal_at {
            if budget < left {
                self.cal_at = Some(left - budget);
                break;
            }
            budget -= left;
            let next = match view.get(CSTAT) & CALS_MASK {
                0 => CALS_BASIC,
                CALS_BASIC => CALS_PRECISION,
                done => done,
            };
            Self::set_cals(view, next);
            Self::set_ir(view, IR_CSC);
            self.arm_cal(view);
        }
        if self.wd > 0 {
            let step = u32::try_from(delta).unwrap_or(u32::MAX);
            self.wd = self.wd.saturating_sub(step);
            if self.wd == 0 {
                Self::set_ir(view, IR_CWE);
                self.wd = self.wd_reload;
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let ir = view.get(IR);
        let ie = view.get(IE);
        u32::from(ir & IR_CWE != 0 && ie & IE_CWEE != 0 || ir & IR_CSC != 0 && ie & IE_CSCE != 0)
    }

    fn reset(&mut self) {
        self.cal_at = None;
        self.wd = 0;
        self.wd_reload = 0;
    }
}

/// H7 FDCAN clock-calibration unit.
#[derive(Debug)]
pub struct Ccu {
    inner: RegisterFile<CcuBehavior>,
}

impl Ccu {
    /// A new `CAN_CCU` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `CAN_CCU`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CAN_CCU").expect("generated tables contain CAN_CCU");
        Self {
            inner: RegisterFile::new(def, CcuBehavior::default()),
        }
    }
}

impl Peripheral for Ccu {
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
        "ccu"
    }
}

impl InterruptSource for Ccu {
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
    use crate::device::svd::stm32h743::Stm32h743;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4000_A800;
    const CCFG_O: u32 = 0x04;
    const CSTAT_O: u32 = 0x08;
    const CWD_O: u32 = 0x0C;
    const IR_O: u32 = 0x10;
    const IE_O: u32 = 0x14;

    #[allow(clippy::cast_possible_truncation)]
    fn w(c: &mut Ccu, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            c.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(c: &mut Ccu, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = c.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn ccu() -> Ccu {
        let mut c = Ccu::new_for::<Stm32h743>();
        c.reset();
        c
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = Stm32h743::peripheral("CAN_CCU").unwrap();
        assert_eq!(def.base, BASE, "CAN_CCU base");
        for (name, offset) in [
            ("CREL", 0x00),
            ("CCFG", 0x04),
            ("CSTAT", 0x08),
            ("CWD", 0x0C),
            ("IR", 0x10),
            ("IE", 0x14),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("CAN_CCU:{name} present"));
            assert_eq!(r.offset, offset, "CAN_CCU:{name} offset");
        }
    }

    #[test]
    fn bypass_stays_uncalibrated() {
        let mut c = ccu();
        w(&mut c, CCFG_O, BCC | (3 << 16));
        c.tick(100);
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, 0, "bypass: no walk");
        assert_eq!(r(&mut c, IR_O), 0, "no events");
    }

    #[test]
    fn calibration_walks_and_irqs() {
        let mut c = ccu();
        w(&mut c, IE_O, IE_CSCE);
        w(&mut c, CCFG_O, 32 | (2 << 16)); // TQBT + CDIV, BCC clear
        assert_eq!(c.pending(), 0, "quiet before walk");
        c.tick(8);
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, CALS_BASIC, "basic");
        assert_ne!(r(&mut c, IR_O) & IR_CSC, 0, "CSC on transition");
        assert_eq!(c.pending(), 1, "line raises");
        w(&mut c, IR_O, IR_CSC);
        assert_eq!(c.pending(), 0, "W1C drops the line");
        c.tick(8);
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, CALS_PRECISION, "precision");
        c.tick(100);
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, CALS_PRECISION, "terminal");
    }

    #[test]
    fn swr_resets_calibration() {
        let mut c = ccu();
        w(&mut c, CCFG_O, 0);
        c.tick(20);
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, CALS_PRECISION);
        w(&mut c, CCFG_O, SWR);
        assert_eq!(r(&mut c, CCFG_O) & SWR, 0, "SWR self-clears");
        assert_eq!(r(&mut c, CSTAT_O) & CALS_MASK, 0, "back to zero");
    }

    #[test]
    fn watchdog_barks_and_reloads() {
        let mut c = ccu();
        w(&mut c, IE_O, IE_CWEE);
        w(&mut c, CCFG_O, 0); // calibration on (watchdog runs)
        w(&mut c, CWD_O, (10 << 16) | 5); // reload 10, start 5
        assert_eq!(r(&mut c, CWD_O) & 0xFFFF, 5, "live counter reads");
        c.tick(5);
        assert_ne!(r(&mut c, IR_O) & IR_CWE, 0, "CWE on expiry");
        assert_eq!(c.pending(), 1, "line raises");
        assert_eq!(r(&mut c, CWD_O) & 0xFFFF, 10, "reloaded");
        w(&mut c, IR_O, IR_CWE);
        c.tick(10);
        assert_ne!(r(&mut c, IR_O) & IR_CWE, 0, "barks again");
    }

    #[test]
    fn watchdog_gated_by_bypass() {
        let mut c = ccu();
        w(&mut c, CCFG_O, BCC);
        w(&mut c, CWD_O, 1);
        c.tick(100);
        assert_eq!(r(&mut c, IR_O) & IR_CWE, 0, "quiet on bypass");
    }
}

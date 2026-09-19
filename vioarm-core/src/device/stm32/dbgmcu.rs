//! Debug MCU (`DBGMCU@0x40015800` on F0) — identification plus freeze latches.
//!
//! `IDCODE` reads the documented device identifier (board-supplied:
//! `REV_ID` in the high half, `DEV_ID` in `0xFFF`; e.g. `0x1000_0444`
//! for `STM32F030x6` — RM0360 §26.4.1, cross-checked against OpenOCD's
//! `stm32f1x.c` device table and the `stm32flash`/`AN2606` list).
//! `CR` and the `APBxFZ` freeze registers latch; freezing has no effect
//! in simulation (time is virtual), which the latch makes observable.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const IDCODE: u32 = 0x00;

#[derive(Clone, Debug)]
struct DbgmcuBehavior {
    idcode: u32,
}

impl Behavior for DbgmcuBehavior {
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, _value: u32) -> u32 {
        if offset == IDCODE {
            return self.idcode;
        }
        _value
    }

    fn needs_tick() -> bool {
        false
    }
}

/// A `DBGMCU`: identification plus freeze latches.
#[derive(Debug)]
pub struct Dbgmcu {
    inner: RegisterFile<DbgmcuBehavior>,
}

impl Dbgmcu {
    /// A new `DBGMCU` on another chip's device tables, reporting `idcode`
    /// from `IDCODE` (document the authority at the call site).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `DBGMCU`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>(idcode: u32) -> Self {
        let def = D::peripheral("DBGMCU").expect("generated tables contain DBGMCU");
        Self {
            inner: RegisterFile::new(def, DbgmcuBehavior { idcode }),
        }
    }
}

impl Peripheral for Dbgmcu {
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

    fn name(&self) -> &'static str {
        "dbgmcu"
    }
}

impl InterruptSource for Dbgmcu {
    fn exception_number(&self) -> Option<u8> {
        None
    }

    fn pending(&self) -> u32 {
        0
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f030::Stm32f030;

    const BASE: u32 = 0x4001_5800;
    const CR: u32 = 0x04;
    const APBLFZ: u32 = 0x08;

    #[allow(clippy::cast_possible_truncation)]
    fn w(d: &mut Dbgmcu, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(d: &mut Dbgmcu, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = d.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn idcode_reports_and_freezes_latch() {
        let mut d = Dbgmcu::new_for::<Stm32f030>(0x1000_0444);
        assert_eq!(r(&mut d, IDCODE) & 0xFFF, 0x444, "DEV_ID");
        assert_eq!(r(&mut d, IDCODE) >> 16, 0x1000, "REV_ID");
        w(&mut d, IDCODE, 0xFFFF_FFFF);
        assert_eq!(r(&mut d, IDCODE), 0x1000_0444, "IDCODE read-only");
        w(&mut d, CR, 0x0000_0003);
        assert_eq!(r(&mut d, CR), 0x0000_0003, "CR latches");
        w(&mut d, APBLFZ, 0x0000_0001);
        assert_eq!(r(&mut d, APBLFZ), 0x0000_0001, "freeze latches");
    }
}

//! Flexible static memory controller (`FSMC@0xA0000000` on F1) — bank
//! configuration with gated external windows.
//!
//! The control registers (`BCRx`/`BTRx`/`BWTRx` NOR/PSRAM timing,
//! `PCRx`/`PMEMx`/`PATTx` NAND timing, `SRx` status) latch and read back
//! from the generated tables. The bank windows themselves (`0x60000000`
//! NOR banks 1–4, `0x70000000`/`0x80000000` NAND banks 2–3, `0x90000000`
//! PC card) have no memory attached — the documented-virtual stance:
//! an enabled bank (`BCR.MBKEN` / `PCR.PBKEN`) reads `0xFF` (floating
//! bus) and drops writes, a disabled bank answers `Reserved` like a
//! gated device. `SRx.FEMPT` reads set: with no transfer engine the
//! FIFO is always empty.
//!
//! Deliberately unmodeled: NAND/PC-card transfers and ECC computation
//! (`ECCR` reads zero), remap, wait-signal timing, the table's `FSMC`
//! IRQ (no edge source exists without a transfer engine).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const MBKEN: u32 = 1 << 0;
const PBKEN: u32 = 1 << 2;
const FEMPT: u32 = 1 << 6;

const SR2: u32 = 0x64;
const SR3: u32 = 0x84;
const SR4: u32 = 0xA4;

const NOR_BASE: u32 = 0x6000_0000;
const NOR_STRIDE: u32 = 0x0400_0000;
const NOR_LEN: u32 = 0x0400_0000;
const NAND2: u32 = 0x7000_0000;
const NAND_LEN: u32 = 0x1000_0000;

#[derive(Clone, Debug, Default)]
struct FsmcBehavior;

impl FsmcBehavior {
    /// NOR bank `n` (1–4) enable from its `BCR` (`BCR1@0x00`, stride 8).
    fn nor_enabled(view: &Registers<&mut [u32]>, bank: u8) -> bool {
        view.get(8 * u32::from(bank - 1)) & MBKEN != 0
    }

    /// NAND/PC bank enable from its `PCR` (`PCR2@0x60`, stride `0x20`).
    /// `pcr` selects the register: bank 2 → `PCR2`, 3 → `PCR3`,
    /// PC card → `PCR4`.
    fn nand_enabled(view: &Registers<&mut [u32]>, pcr: u32) -> bool {
        view.get(pcr) & PBKEN != 0
    }
}

impl Behavior for FsmcBehavior {
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        // No transfer engine ever fills the FIFO: empty always.
        if offset == SR2 || offset == SR3 || offset == SR4 {
            return value | FEMPT;
        }
        value
    }

    fn needs_tick() -> bool {
        false
    }
}

/// Which external window owns `addr`: NOR bank (1–4), NAND `PCR` offset,
/// or nothing.
fn bank_at(addr: u32) -> Option<Bank> {
    if (NOR_BASE..NOR_BASE + 4 * NOR_STRIDE).contains(&addr) {
        let bank = ((addr - NOR_BASE) / NOR_STRIDE) as u8 + 1;
        return Some(Bank::Nor(bank));
    }
    // NAND bank 2 @0x70000000, bank 3 @0x80000000, PC card @0x90000000.
    for (i, base) in [NAND2, NAND2 + NAND_LEN, NAND2 + 2 * NAND_LEN]
        .iter()
        .enumerate()
    {
        if (*base..*base + NAND_LEN).contains(&addr) {
            return Some(Bank::Nand(0x60 + (i as u32) * 0x20));
        }
    }
    None
}

#[derive(Clone, Copy, Debug)]
enum Bank {
    Nor(u8),
    /// NAND/PC bank, selected by its `PCR` offset (`PCR2@0x60` …).
    Nand(u32),
}

/// An FSMC: control registers plus gated bank windows.
#[derive(Debug)]
pub struct Fsmc {
    inner: RegisterFile<FsmcBehavior>,
    windows: [Region; 8],
}

impl Fsmc {
    /// A new `FSMC` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FSMC`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("FSMC").expect("generated tables contain FSMC");
        // Tight control span: some tables grant a nonsense window
        // (`0xfffff400` on the F1 value line), which would swallow the
        // whole upper address space including the core block.
        let mut lo = u32::MAX;
        let mut hi = 0u32;
        for r in def.registers {
            lo = lo.min(r.offset);
            hi = hi.max(r.offset + u32::from(r.width) / 8);
        }
        let ctrl = if lo <= hi {
            Region::new(def.base + lo, hi - lo)
        } else {
            Region::new(def.base, def.window)
        };
        let mut windows = [Region::default(); 8];
        windows[0] = ctrl;
        for (i, w) in windows.iter_mut().enumerate().skip(1).take(4) {
            let base = NOR_BASE + ((i - 1) as u32) * NOR_STRIDE;
            *w = Region::new(base, NOR_LEN);
        }
        for (i, w) in windows.iter_mut().enumerate().skip(5) {
            let base = NAND2 + ((i - 5) as u32) * NAND_LEN;
            *w = Region::new(base, NAND_LEN);
        }
        Self {
            inner: RegisterFile::new(def, FsmcBehavior),
            windows,
        }
    }

    fn bank_enabled(&mut self, bank: Bank) -> bool {
        self.inner.with_view(|view, _| match bank {
            Bank::Nor(n) => FsmcBehavior::nor_enabled(view, n),
            Bank::Nand(pcr) => FsmcBehavior::nand_enabled(view, pcr),
        })
    }
}

impl Peripheral for Fsmc {
    fn ranges(&self) -> &[Region] {
        &self.windows
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if addr >= self.windows[0].base && addr < self.windows[0].base + self.windows[0].len {
            return self.inner.read(addr);
        }
        match bank_at(addr) {
            Some(bank) if self.bank_enabled(bank) => Ok(0xFF),
            _ => Err(BusError::Reserved),
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if addr >= self.windows[0].base && addr < self.windows[0].base + self.windows[0].len {
            return self.inner.write(addr, val);
        }
        match bank_at(addr) {
            Some(bank) if self.bank_enabled(bank) => Ok(()),
            _ => Err(BusError::Reserved),
        }
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn name(&self) -> &'static str {
        "fsmc"
    }
}

impl InterruptSource for Fsmc {
    // No edge source exists without a transfer engine, so the line never
    // pends (the table's `FSMC` IRQ stays unwired by design).
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
    use crate::device::svd::stm32f103::Stm32f103;

    const BASE: u32 = 0xA000_0000;
    const BCR1: u32 = 0x00;
    const BCR2: u32 = 0x08;
    const BTR3: u32 = 0x14;
    const PCR2: u32 = 0x60;
    const PMEM2: u32 = 0x68;
    const PATT2: u32 = 0x6C;
    const BWTR1: u32 = 0x104;

    #[allow(clippy::cast_possible_truncation)]
    fn w(f: &mut Fsmc, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(f: &mut Fsmc, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn fsmc() -> Fsmc {
        let mut f = Fsmc::new_for::<Stm32f103>();
        f.reset();
        f
    }

    #[test]
    fn reset_values_match_tables() {
        let mut f = fsmc();
        assert_eq!(r(&mut f, BCR1), 0x0000_30D0, "BCR1 reset");
        assert_eq!(r(&mut f, SR2), 0x0000_0040, "SR2 reset (FEMPT)");
        assert_eq!(r(&mut f, 0x04), 0xFFFF_FFFF, "BTR1 reset");
        assert_eq!(r(&mut f, PMEM2), 0xFCFC_FCFC, "PMEM2 reset");
    }

    #[test]
    fn config_latches() {
        let mut f = fsmc();
        w(&mut f, BCR2, 0x0000_30D1);
        w(&mut f, BTR3, 0x0FFF_FFFF);
        w(&mut f, PCR2, 0x0004_001E);
        w(&mut f, PMEM2, 0x1111_1111);
        w(&mut f, PATT2, 0x2222_2222);
        w(&mut f, BWTR1, 0x0EEE_EEEE);
        assert_eq!(r(&mut f, BCR2), 0x0000_30D1);
        assert_eq!(r(&mut f, BTR3), 0x0FFF_FFFF);
        assert_eq!(r(&mut f, PCR2), 0x0004_001E);
        assert_eq!(r(&mut f, PMEM2), 0x1111_1111);
        assert_eq!(r(&mut f, PATT2), 0x2222_2222);
        assert_eq!(r(&mut f, BWTR1), 0x0EEE_EEEE);
    }

    #[test]
    fn femt_always_reads_set() {
        let mut f = fsmc();
        w(&mut f, SR2, 0x0000_0000);
        assert_ne!(r(&mut f, SR2) & FEMPT, 0, "FEMPT stuck");
        assert_ne!(r(&mut f, SR3) & FEMPT, 0, "SR3 too");
    }

    #[test]
    fn banks_gated_at_reset() {
        let mut f = fsmc();
        for base in [0x6000_0000, 0x6400_0000, 0x7000_0000, 0x9000_0000] {
            assert!(
                matches!(f.read(base), Err(BusError::Reserved)),
                "disabled bank faults"
            );
            assert!(
                matches!(f.write(base, 0xAA), Err(BusError::Reserved)),
                "disabled write faults"
            );
        }
    }

    #[test]
    fn nor_bank1_enable_floats() {
        let mut f = fsmc();
        let bcr1 = r(&mut f, BCR1);
        w(&mut f, BCR1, bcr1 | MBKEN);
        assert_eq!(f.read(0x6000_0000).unwrap(), 0xFF, "bank 1 floats");
        assert_eq!(f.read(0x63FF_FFFF).unwrap(), 0xFF, "bank 1 top");
        f.write(0x6000_0000, 0xAA).unwrap();
        assert_eq!(f.read(0x6000_0000).unwrap(), 0xFF, "writes dropped");
        assert!(
            matches!(f.read(0x6400_0000), Err(BusError::Reserved)),
            "bank 2 still gated"
        );
        w(&mut f, BCR1, bcr1 & !MBKEN);
        assert!(
            matches!(f.read(0x6000_0000), Err(BusError::Reserved)),
            "re-gated"
        );
    }

    #[test]
    fn nand_bank2_and_pccard_enable() {
        let mut f = fsmc();
        let pcr2 = r(&mut f, PCR2);
        w(&mut f, PCR2, pcr2 | PBKEN);
        assert_eq!(f.read(0x7000_0000).unwrap(), 0xFF, "NAND bank 2");
        assert!(
            matches!(f.read(0x8000_0000), Err(BusError::Reserved)),
            "bank 3 still gated"
        );
        let pcr4 = r(&mut f, 0xA0);
        w(&mut f, 0xA0, pcr4 | PBKEN); // PCR4
        assert_eq!(f.read(0x9000_0000).unwrap(), 0xFF, "PC card");
    }

    #[test]
    fn never_interrupts() {
        let f = fsmc();
        assert_eq!(f.exception_number(), None);
        assert_eq!(f.pending(), 0);
    }
}

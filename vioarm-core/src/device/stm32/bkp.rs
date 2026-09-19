//! STM32 backup registers (`BKP`, `0x4000_6C00` on F1) — 10× 16-bit `DR` plus
//! `RTCCR`/`CR`/`CSR`. F4 has no `BKP` peripheral (backup domain is `RTC`/`PWR`).
//!
//! The backup domain survives system reset: `reset()` keeps every register
//! (only a backup-domain reset via `RCC.BDCR`, unmodeled, or a tamper event
//! clears the data registers). A tamper event arrives through the stimulus
//! channel (bit 8): with `CR.TPE` set it latches `CSR.TEF`, wipes the `DRx`
//! registers, and — with `CSR.TPIE` — raises the `TAMPER` line (F1 IRQ 2,
//! hardcoded: the SVD tables carry no `BKP` IRQ entry). `CSR.CTEF` clears
//! `TEF`. Deliberately unmodeled: write protection via `PWR.DB` (writes are
//! accepted, keeping host-driven testing order-free) and `RCC.BDCR` resets.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// F1 `TAMPER` exception number (IRQ 2): hardcoded, the SVD has no entry.
const TAMPER_EXCEPTION: u8 = 16 + 2;

#[derive(Debug)]
struct BkpBehavior {
    /// Offsets of the `DRx` data registers (wiped by tamper).
    drs: Vec<u32>,
    /// `CR` offset and `TPE` bit.
    cr_off: u32,
    tpe_bit: u32,
    /// `CSR` offset with `TEF`/`TPIE`/`CTEF` bits.
    csr_off: u32,
    tef_bit: u32,
    tpie_bit: u32,
    ctef_bit: u32,
}

impl Default for BkpBehavior {
    fn default() -> Self {
        Self {
            drs: Vec::new(),
            cr_off: 0x30,
            tpe_bit: 1,
            csr_off: 0x34,
            tef_bit: 1 << 8,
            tpie_bit: 1 << 2,
            ctef_bit: 1,
        }
    }
}
impl Behavior for BkpBehavior {
    // BKP domain registers; all effects synchronous on access.
    fn needs_tick() -> bool {
        false
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // `CSR.CTEF` clears the tamper flag (only its own lane can carry it).
        if offset == self.csr_off && u32::from(val) & (self.ctef_bit >> shift) != 0 {
            if let Some(csr) = view.try_get(self.csr_off) {
                view.try_set(self.csr_off, csr & !self.tef_bit);
            }
        }
        view.merged(offset, val, shift)
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // Bit 8 is a tamper-pin event: with `CR.TPE` set it latches `TEF`
        // and wipes the data registers.
        if value & (1 << 8) == 0 {
            return;
        }
        if view.try_get(self.cr_off).unwrap_or(0) & self.tpe_bit == 0 {
            return;
        }
        for off in self.drs.clone() {
            view.try_set(off, 0);
        }
        if let Some(csr) = view.try_get(self.csr_off) {
            view.try_set(self.csr_off, csr | self.tef_bit);
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let csr = view.try_get(self.csr_off).unwrap_or(0);
        u32::from(csr & self.tef_bit != 0 && csr & self.tpie_bit != 0)
    }
}

/// Backup registers.
#[derive(Debug)]
pub struct Bkp {
    inner: RegisterFile<BkpBehavior>,
}
impl Bkp {
    /// A new `Bkp` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("BKP").expect("generated tables contain BKP");
        let off = |name: &str, fb: u32| {
            def.registers
                .iter()
                .find(|r| r.name == name)
                .map(|r| r.offset)
                .unwrap_or(fb)
        };
        let bit = |reg: &str, field: &str, fb: u32| {
            def.registers
                .iter()
                .find(|r| r.name == reg)
                .and_then(|r| r.fields.iter().find(|f| f.name == field))
                .map(|f| 1u32 << f.offset)
                .unwrap_or(fb)
        };
        let drs = def
            .registers
            .iter()
            .filter(|r| r.name.starts_with("DR"))
            .map(|r| r.offset)
            .collect();
        let behavior = BkpBehavior {
            drs,
            cr_off: off("CR", 0x30),
            tpe_bit: bit("CR", "TPE", 1),
            csr_off: off("CSR", 0x34),
            tef_bit: bit("CSR", "TEF", 1 << 8),
            tpie_bit: bit("CSR", "TPIE", 1 << 2),
            ctef_bit: bit("CSR", "CTEF", 1),
        };
        Self {
            inner: RegisterFile::new(def, behavior),
        }
    }
}
impl Peripheral for Bkp {
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
        // The backup domain survives system reset: registers keep their
        // values (only backup-domain reset or tamper clears them).
    }
    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
    fn name(&self) -> &'static str {
        "bkp"
    }
}
impl InterruptSource for Bkp {
    fn exception_number(&self) -> Option<u8> {
        Some(TAMPER_EXCEPTION)
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
    use super::Bkp;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::{InterruptSource, Peripheral};
    const BASE: u32 = 0x4000_6C00;
    fn write_word(b: &mut Bkp, addr: u32, val: u32) {
        for (i, byte) in val.to_le_bytes().iter().enumerate() {
            b.write(addr.wrapping_add(i as u32), *byte).unwrap();
        }
    }
    fn read_word(b: &mut Bkp, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, v) in bytes.iter_mut().enumerate() {
            *v = b.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }
    #[test]
    fn bkp_registers_round_trip() {
        let mut b = Bkp::new_for::<Stm32f103>();
        write_word(&mut b, BASE + 0x04, 0x1234); // DR1
        assert_eq!(read_word(&mut b, BASE + 0x04) & 0xFFFF, 0x1234);
    }

    /// The backup domain survives system reset: data registers keep
    /// their values through `reset()`.
    #[test]
    fn backup_registers_persist_across_reset() {
        let mut b = Bkp::new_for::<Stm32f103>();
        write_word(&mut b, BASE + 0x04, 0xBEEF); // DR1
        write_word(&mut b, BASE + 0x28, 0xCAFE); // DR10
        b.reset();
        assert_eq!(read_word(&mut b, BASE + 0x04) & 0xFFFF, 0xBEEF);
        assert_eq!(read_word(&mut b, BASE + 0x28) & 0xFFFF, 0xCAFE);
    }

    /// Tamper needs `CR.TPE`: without it the event is ignored.
    #[test]
    fn tamper_without_tpe_is_ignored() {
        let mut b = Bkp::new_for::<Stm32f103>();
        write_word(&mut b, BASE + 0x04, 0x1234);
        b.drive_inputs(1 << 8);
        assert_eq!(read_word(&mut b, BASE + 0x04) & 0xFFFF, 0x1234);
        assert_eq!(read_word(&mut b, BASE + 0x34) & (1 << 8), 0, "no TEF");
        assert_eq!(b.pending(), 0);
    }

    /// With `TPE`, a tamper event latches `TEF`, wipes the data registers,
    /// and asserts the line when `TPIE` is set; `CTEF` clears it.
    #[test]
    fn tamper_latches_tef_wipes_data_and_raises_irq() {
        let mut b = Bkp::new_for::<Stm32f103>();
        write_word(&mut b, BASE + 0x30, 0x1); // CR.TPE
        write_word(&mut b, BASE + 0x34, 0x4); // CSR.TPIE
        write_word(&mut b, BASE + 0x04, 0x1234);
        write_word(&mut b, BASE + 0x28, 0x5678);
        b.drive_inputs(1 << 8);
        assert_ne!(read_word(&mut b, BASE + 0x34) & (1 << 8), 0, "TEF");
        assert_eq!(read_word(&mut b, BASE + 0x04) & 0xFFFF, 0, "wiped");
        assert_eq!(read_word(&mut b, BASE + 0x28) & 0xFFFF, 0, "wiped");
        assert_eq!(b.pending(), 1, "TAMPER line via TPIE");
        assert_eq!(b.exception_number(), Some(18));
        write_word(&mut b, BASE + 0x34, 0x1); // CSR.CTEF
        assert_eq!(read_word(&mut b, BASE + 0x34) & (1 << 8), 0, "cleared");
        assert_eq!(b.pending(), 0);
    }
}

//! The ARMv7-M System Control Block (subset).
//!
//! Models the three fields the core actually uses: the `ICSR` `PendSV`
//! set/clear bits, and the `SHPR2`/`SHPR3` configurable exception priorities
//! (`SVCall`, `PendSV`, `SysTick`). The remaining `SCB` registers (`AIRCR`,
//! `SCR`, `CCR`, ...) are not modeled; the device only claims its three
//! windows so writes elsewhere fall through to the NVIC or backing memory.
//!
//! Priority encoding follows the architecture: lower value == higher
//! priority, with the `SVCall` byte in `SHPR2` and the `PendSV`/`SysTick`
//! bytes in `SHPR3` (CMSIS `SCB->SHP[7]`, `SHP[10]`, `SHP[11]`).

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const ICSR: u32 = 0xE000_ED04;
const SHPR2: u32 = 0xE000_ED1C;
const SHPR3: u32 = 0xE000_ED20;

const PENDSVSET: u8 = 0x10; // ICSR byte 3, bit 28: write 1 to pend.
const PENDSVCLR: u8 = 0x08; // ICSR byte 3, bit 27: write 1 to clear.

/// The System Control Block: interrupt control state and exception priorities.
#[derive(Debug, Clone)]
pub struct Scb {
    ranges: [Region; 3],
    pendsv: bool,
    shpr2: u32,
    shpr3: u32,
}

impl Scb {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [
                Region::new(ICSR, 4),
                Region::new(SHPR2, 4),
                Region::new(SHPR3, 4),
            ],
            pendsv: false,
            shpr2: 0,
            shpr3: 0,
        }
    }

    /// Whether the `PendSV` line is latched pending.
    #[must_use]
    pub const fn pendsv_pending(&self) -> bool {
        self.pendsv
    }

    /// The configurable priority byte for system exception `n` (`11`, `14`,
    /// `15`), lower meaning higher priority. Unrelated numbers read zero.
    #[must_use]
    pub const fn priority(&self, n: u8) -> u8 {
        match n {
            11 => ((self.shpr2 >> 24) & 0xFF) as u8,
            14 => ((self.shpr3 >> 16) & 0xFF) as u8,
            15 => ((self.shpr3 >> 24) & 0xFF) as u8,
            _ => 0,
        }
    }

    fn read_reg(&self, base: u32, offset: u32) -> u8 {
        let reg = match base {
            ICSR => u32::from(self.pendsv) << 28,
            SHPR2 => self.shpr2,
            SHPR3 => self.shpr3,
            _ => 0,
        };
        ((reg >> (8 * offset)) & 0xFF) as u8
    }

    fn write_reg(&mut self, base: u32, offset: u32, val: u8) {
        match base {
            ICSR => {
                if offset == 3 {
                    // Only the PendSV set/clear bits are modeled.
                    if val & PENDSVSET != 0 {
                        self.pendsv = true;
                    } else if val & PENDSVCLR != 0 {
                        self.pendsv = false;
                    }
                }
            }
            SHPR2 => {
                let shift = 8 * offset;
                self.shpr2 = (self.shpr2 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            SHPR3 => {
                let shift = 8 * offset;
                self.shpr3 = (self.shpr3 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            _ => {}
        }
    }
}

impl Default for Scb {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Scb {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                return Ok(self.read_reg(r.base, addr - r.base));
            }
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                self.write_reg(r.base, addr - r.base, val);
                return Ok(());
            }
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        self.pendsv = false;
        self.shpr2 = 0;
        self.shpr3 = 0;
    }

    fn name(&self) -> &'static str {
        "scb"
    }
}

impl InterruptSource for Scb {
    fn exception_number(&self) -> Option<u8> {
        Some(14)
    }

    fn pending(&self) -> u32 {
        u32::from(self.pendsv)
    }

    fn acknowledge(&mut self, _lines: u32) {
        self.pendsv = false;
    }
}

#[cfg(test)]
mod tests {
    use super::Scb;
    use crate::device::Peripheral;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(s: &mut Scb, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(s: &mut Scb, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = s.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn icsr_pendsv_set_clear_roundtrip() {
        let mut s = Scb::new();
        assert!(!s.pendsv_pending());
        // ICSR byte 3, bit 4 (PENDSVSET).
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        assert!(s.pendsv_pending());
        assert_eq!(read_word(&mut s, 0xE000_ED04), 0x1000_0000);
        // ICSR byte 3, bit 3 (PENDSVCLR).
        write_word(&mut s, 0xE000_ED04, 0x0800_0000);
        assert!(!s.pendsv_pending());
    }

    #[test]
    fn shpr_priorities_stored_per_exception() {
        let mut s = Scb::new();
        // SHPR2 byte 3 -> SVCall; SHPR3 byte 3 -> SysTick; SHPR3 byte 2 -> PendSV.
        s.write(0xE000_ED1F, 0x40).unwrap();
        s.write(0xE000_ED23, 0x20).unwrap();
        s.write(0xE000_ED22, 0x80).unwrap();
        assert_eq!(s.priority(11), 0x40);
        assert_eq!(s.priority(15), 0x20);
        assert_eq!(s.priority(14), 0x80);
        assert_eq!(read_word(&mut s, 0xE000_ED1C), 0x4000_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED20), 0x2080_0000);
    }

    #[test]
    fn reset_clears_pend_and_priorities() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        write_word(&mut s, 0xE000_ED1F, 0x10);
        s.reset();
        assert!(!s.pendsv_pending());
        assert_eq!(s.priority(11), 0);
    }
}

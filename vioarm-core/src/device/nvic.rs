//! The ARMv6-M nested vectored interrupt controller.
//!
//! A fixed-priority controller for up to 64 external interrupts (exception
//! `16..=79`): the lowest-numbered pending line wins, exactly as the
//! architecture requires. The register file is small — set/clear enable and
//! set/clear pending (two 32-bit words each for lines 0..63), plus ignored
//! priority bytes — and the "ready to take" decision lives in the
//! `SystemMemory` that wires device lines into the `pending` register.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const ISER: u32 = 0xE000_E100;
const ICER: u32 = 0xE000_E180;
const ISPR: u32 = 0xE000_E200;
const ICPR: u32 = 0xE000_E280;
const IPR: u32 = 0xE000_E400;

/// Two 32-bit words cover lines 0..63 (the range the STM32F1 needs).
const WORDS: u32 = 2;
const BANK_LEN: u32 = WORDS * 4;
const IPR_LEN: u32 = 64;

const ISER_END: u32 = ISER + BANK_LEN;
const ICER_END: u32 = ICER + BANK_LEN;
const ISPR_END: u32 = ISPR + BANK_LEN;
const ICPR_END: u32 = ICPR + BANK_LEN;
const IPR_END: u32 = IPR + IPR_LEN;

/// The NVIC register file: enabled and latched-pending masks for lines 0..63.
#[derive(Debug, Clone)]
pub struct Nvic {
    ranges: [Region; 5],
    enabled: u64,
    pending: u64,
}

impl Nvic {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [
                Region::new(ISER, BANK_LEN),
                Region::new(ICER, BANK_LEN),
                Region::new(ISPR, BANK_LEN),
                Region::new(ICPR, BANK_LEN),
                Region::new(IPR, IPR_LEN),
            ],
            enabled: 0,
            pending: 0,
        }
    }

    /// Which of `lines` are both enabled and pending.
    #[must_use]
    pub const fn ready(&self) -> u64 {
        self.enabled & self.pending
    }

    /// Latch `lines` as pending (device asserts, or software `ISPR`).
    pub fn latch(&mut self, lines: u64) {
        self.pending |= lines;
    }

    /// The interrupt currently being taken (exception `16 + line`) has been
    /// acknowledged; drop its pending latch.
    pub fn take(&mut self, line: u32) {
        self.pending &= !(1u64 << line);
    }
}

impl Default for Nvic {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Nvic {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // Word index (0..=1) and byte lane within the word.
        #[allow(clippy::cast_possible_truncation)]
        let pick = |mask: u64, base: u32| -> u8 {
            let word = if (addr - base) / 4 == 0 {
                mask as u32
            } else {
                (mask >> 32) as u32
            };
            (word >> (8 * ((addr - base) % 4))) as u8
        };
        if (ISER..ISER_END).contains(&addr) {
            Ok(pick(self.enabled, ISER))
        } else if (ICER..ICER_END).contains(&addr) {
            Ok(pick(self.enabled, ICER))
        } else if (ISPR..ISPR_END).contains(&addr) {
            Ok(pick(self.pending, ISPR))
        } else if (ICPR..ICPR_END).contains(&addr) {
            Ok(pick(self.pending, ICPR))
        } else if (IPR..IPR_END).contains(&addr) {
            Ok(0)
        } else {
            Err(BusError::Reserved)
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let bank = |base: u32| (addr - base) / 4;
        let byte = |base: u32| (addr - base) % 4;
        let shift = |base: u32| 32 * bank(base) + 8 * byte(base);
        match addr {
            ISER..ISER_END => self.enabled |= u64::from(val) << shift(ISER),
            ICER..ICER_END => self.enabled &= !(u64::from(val) << shift(ICER)),
            ISPR..ISPR_END => self.pending |= u64::from(val) << shift(ISPR),
            ICPR..ICPR_END => self.pending &= !(u64::from(val) << shift(ICPR)),
            // IPR: priority is fixed by vector number on v6-M; writes ignored.
            IPR..IPR_END => {}
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.enabled = 0;
        self.pending = 0;
    }

    fn name(&self) -> &'static str {
        "nvic"
    }
}

#[cfg(test)]
mod tests {
    use super::Nvic;
    use crate::device::Peripheral;
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(n: &mut Nvic, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            n.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(n: &mut Nvic, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = n.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn set_and_clear_enable() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E100, 0x0000_0005);
        assert_eq!(read_word(&mut n, 0xE000_E100), 0x0000_0005);
        write_word(&mut n, 0xE000_E180, 0x0000_0001);
        assert_eq!(read_word(&mut n, 0xE000_E100), 0x0000_0004);
    }

    #[test]
    fn set_and_clear_pending() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E200, 0x0000_0003);
        assert_eq!(read_word(&mut n, 0xE000_E200), 0x0000_0003);
        write_word(&mut n, 0xE000_E280, 0x0000_0002);
        assert_eq!(read_word(&mut n, 0xE000_E200), 0x0000_0001);
    }

    #[test]
    fn ready_is_enabled_and_pending() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E200, 0x0000_000A);
        assert_eq!(n.ready(), 0);
        write_word(&mut n, 0xE000_E100, 0x0000_0008);
        assert_eq!(n.ready(), 0x0000_0008);
    }

    #[test]
    fn priority_bytes_read_zero_and_accept_writes() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E400, 0xC0C0_C0C0);
        assert_eq!(read_word(&mut n, 0xE000_E400), 0);
    }

    #[test]
    fn out_of_window_is_reserved() {
        let mut n = Nvic::new();
        assert!(matches!(n.read(0xE000_E500), Err(BusError::Reserved)));
    }
}

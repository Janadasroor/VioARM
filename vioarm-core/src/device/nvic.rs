//! The ARMv6-M nested vectored interrupt controller.
//!
//! A fixed-priority controller for 32 external interrupts (exception
//! `16..=47`): the lowest-numbered pending line wins, exactly as the
//! architecture requires. The register file is small — set/clear enable and
//! set/clear pending, plus ignored priority bytes — and the "ready to take"
//! decision lives in the `SystemMemory` that wires device lines into the
//! `pending` register.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const ISER: u32 = 0xE000_E100;
const ICER: u32 = 0xE000_E180;
const ISPR: u32 = 0xE000_E200;
const ICPR: u32 = 0xE000_E280;
const IPR: u32 = 0xE000_E400;

const REGION_LEN: u32 = 0x10;
const IPR_LEN: u32 = 0x20;
const ISER_END: u32 = ISER + REGION_LEN;
const ICER_END: u32 = ICER + REGION_LEN;
const ISPR_END: u32 = ISPR + REGION_LEN;
const ICPR_END: u32 = ICPR + REGION_LEN;
const IPR_END: u32 = IPR + IPR_LEN;

/// The NVIC register file: enabled and latched-pending masks for lines 0..31.
#[derive(Debug, Clone)]
pub struct Nvic {
    ranges: [Region; 5],
    enabled: u32,
    pending: u32,
}

impl Nvic {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [
                Region::new(ISER, REGION_LEN),
                Region::new(ICER, REGION_LEN),
                Region::new(ISPR, REGION_LEN),
                Region::new(ICPR, REGION_LEN),
                Region::new(IPR, IPR_LEN),
            ],
            enabled: 0,
            pending: 0,
        }
    }

    /// Which of `lines` are both enabled and pending.
    #[must_use]
    pub const fn ready(&self) -> u32 {
        self.enabled & self.pending
    }

    /// Latch `lines` as pending (device asserts, or software `ISPR`).
    pub fn latch(&mut self, lines: u32) {
        self.pending |= lines;
    }

    /// The interrupt currently being taken (exception `16 + line`) has been
    /// acknowledged; drop its pending latch.
    pub fn take(&mut self, line: u32) {
        self.pending &= !(1 << line);
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
        let reg = if (ISER..ISER_END).contains(&addr) || (ICER..ICER_END).contains(&addr) {
            &self.enabled
        } else if (ISPR..ISPR_END).contains(&addr) || (ICPR..ICPR_END).contains(&addr) {
            &self.pending
        } else if (IPR..IPR_END).contains(&addr) {
            return Ok(0);
        } else {
            return Err(BusError::Reserved);
        };
        // Byte lane pick: the 4-bit offset selects which byte of the 32-bit reg.
        #[allow(clippy::cast_possible_truncation)]
        Ok((*reg >> (8 * (addr & 0xF))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let shift = 8 * (addr & 0xF);
        let bits = u32::from(val) << shift;
        match addr {
            ISER..ISER_END => self.enabled |= bits,
            ICER..ICER_END => self.enabled &= !bits,
            ISPR..ISPR_END => self.pending |= bits,
            ICPR..ICPR_END => self.pending &= !bits,
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

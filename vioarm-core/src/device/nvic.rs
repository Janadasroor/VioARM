//! The ARMv7-M nested vectored interrupt controller.
//!
//! Up to 96 external interrupts (exception `16..=111`) with set/clear enable,
//! set/clear pending (three 32-bit words each, covering the F4 USB OTG lines
//! at IRQ 67 and 74..=77), and one configurable priority byte per line at
//! `IPR`. The "ready to take" decision lives in the `SystemMemory`, which
//! wires device lines into the `pending` register and arbitrates by priority.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const ISER: u32 = 0xE000_E100;
const ICER: u32 = 0xE000_E180;
const ISPR: u32 = 0xE000_E200;
const ICPR: u32 = 0xE000_E280;
/// `NVIC_ITNS<n>` (v8-M): the per-line security target. A set bit makes the
/// line target Non-secure; a cleared bit targets Secure. Resets all-set (every
/// external interrupt targets Non-secure, the ARMSSE default).
const ITNS: u32 = 0xE000_E380;
const IPR: u32 = 0xE000_E400;

/// Implemented priority bits: ARMv7-M implements priority bytes
/// most-significant-bit first, and every STM32 in the board list wires four
/// (`0xF0`; a `0xFF` write reads back `0xF0`). Cortex-M0 parts only wire two
/// (`0xC0`) — this shared model keeps four, so an M0 read-back probe still
/// sees four bits; real M0 firmware writes top-aligned values (`0x00`,
/// `0x40`, `0x80`, `0xC0`), which survive either mask, so arbitration is
/// unaffected.
pub(crate) const PRIO_MASK: u8 = 0xF0;

/// Three 32-bit words cover lines 0..95 (the range the F1/F4 lines need).
const WORDS: u32 = 3;
const BANK_LEN: u32 = WORDS * 4;
const IPR_LEN: u32 = 96;

const ISER_END: u32 = ISER + BANK_LEN;
const ICER_END: u32 = ICER + BANK_LEN;
const ISPR_END: u32 = ISPR + BANK_LEN;
const ICPR_END: u32 = ICPR + BANK_LEN;
const ITNS_END: u32 = ITNS + BANK_LEN;
const IPR_END: u32 = IPR + IPR_LEN;

/// The NVIC register file: enabled and latched-pending masks for lines 0..95,
/// the per-line security target (`ITNS`, set = Non-secure, v8-M), plus one
/// configurable 8-bit priority byte per line.
///
/// Lower priority value means higher priority, exactly as the architecture
/// defines it.
#[derive(Debug, Clone)]
pub struct Nvic {
    ranges: [Region; 6],
    enabled: u128,
    pending: u128,
    itns: u128,
    priority: [u8; 96],
}

impl Nvic {
    /// A new `Nvic`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [
                Region::new(ISER, BANK_LEN),
                Region::new(ICER, BANK_LEN),
                Region::new(ISPR, BANK_LEN),
                Region::new(ICPR, BANK_LEN),
                Region::new(ITNS, BANK_LEN),
                Region::new(IPR, IPR_LEN),
            ],
            enabled: 0,
            pending: 0,
            // Reset: all external interrupts target the Non-secure state (the
            // ARMSSE default), so firmware must clear an `ITNS` bit to claim a
            // line for the Secure state.
            itns: !0,
            priority: [0; 96],
        }
    }

    /// Which of `lines` are both enabled and pending.
    #[must_use]
    pub const fn ready(&self) -> u128 {
        self.enabled & self.pending
    }

    /// The configured priority byte for external interrupt `line` (lower is
    /// higher). Defaults to zero (the highest configurable priority).
    #[must_use]
    pub const fn priority(&self, line: u32) -> u8 {
        self.priority[line as usize]
    }

    /// Whether external interrupt `line` targets the Secure state: `ITNS`
    /// clear. A set `ITNS` bit (the reset value) targets Non-secure.
    #[must_use]
    pub const fn targets_secure(&self, line: u32) -> bool {
        self.itns & (1u128 << line) == 0
    }

    /// Latch `lines` as pending (device asserts, or software `ISPR`).
    pub fn latch(&mut self, lines: u128) {
        self.pending |= lines;
    }

    /// The interrupt currently being taken (exception `16 + line`) has been
    /// acknowledged; drop its pending latch.
    pub fn take(&mut self, line: u32) {
        self.pending &= !(1u128 << line);
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
        // Word index (0..=2) and byte lane within the word.
        #[allow(clippy::cast_possible_truncation)]
        let pick = |mask: u128, base: u32| -> u8 {
            let word = match (addr - base) / 4 {
                0 => mask as u32,
                1 => (mask >> 32) as u32,
                _ => (mask >> 64) as u32,
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
        } else if (ITNS..ITNS_END).contains(&addr) {
            Ok(pick(self.itns, ITNS))
        } else if (IPR..IPR_END).contains(&addr) {
            Ok(self.priority[(addr - IPR) as usize])
        } else {
            Err(BusError::Reserved)
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let bank = |base: u32| (addr - base) / 4;
        let byte = |base: u32| (addr - base) % 4;
        let shift = |base: u32| 32 * bank(base) + 8 * byte(base);
        match addr {
            ISER..ISER_END => self.enabled |= u128::from(val) << shift(ISER),
            ICER..ICER_END => self.enabled &= !(u128::from(val) << shift(ICER)),
            ISPR..ISPR_END => self.pending |= u128::from(val) << shift(ISPR),
            ICPR..ICPR_END => self.pending &= !(u128::from(val) << shift(ICPR)),
            // `ITNS`: direct per-bit assignment (QEMU's `nvic_writel`), each
            // written bit sets or clears one line's target.
            ITNS..ITNS_END => {
                let shift = shift(ITNS);
                self.itns = (self.itns & !(0xFFu128 << shift)) | (u128::from(val) << shift);
            }
            // IPR: one priority byte per line, implemented bits kept
            // (top-aligned `PRIO_MASK`; unimplemented low bits read zero).
            IPR..IPR_END => self.priority[(addr - IPR) as usize] = val & PRIO_MASK,
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.enabled = 0;
        self.pending = 0;
        self.itns = !0;
        self.priority.fill(0);
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
    fn priority_bytes_store_and_read() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E400, 0xC0C0_C0C0);
        assert_eq!(read_word(&mut n, 0xE000_E400), 0xC0C0_C0C0);
        assert_eq!(n.priority(0), 0xC0);
        assert_eq!(n.priority(3), 0xC0);
    }

    #[test]
    fn priority_bytes_keep_only_implemented_top_bits() {
        let mut n = Nvic::new();
        write_word(&mut n, 0xE000_E400, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut n, 0xE000_E400), 0xF0F0_F0F0);
        assert_eq!(n.priority(0), 0xF0);
        n.write(0xE000_E400, 0xBF).unwrap();
        assert_eq!(n.priority(0), 0xB0);
        n.write(0xE000_E401, 0x0F).unwrap();
        assert_eq!(n.priority(1), 0x00);
    }

    #[test]
    fn out_of_window_is_reserved() {
        let mut n = Nvic::new();
        assert!(matches!(n.read(0xE000_E500), Err(BusError::Reserved)));
    }

    #[test]
    fn itns_resets_all_nonsecure_and_reads_back() {
        let mut n = Nvic::new();
        assert!(!n.targets_secure(0), "reset ITNS targets Non-secure");
        assert!(!n.targets_secure(95));
        // Claim line 0 for the Secure state: clear its ITNS bit.
        write_word(&mut n, 0xE000_E380, 0xFFFF_FFFE);
        assert!(n.targets_secure(0));
        assert!(!n.targets_secure(1));
        assert_eq!(read_word(&mut n, 0xE000_E380), 0xFFFF_FFFE);
        // Setting the bit back routes it to Non-secure again.
        write_word(&mut n, 0xE000_E380, 0xFFFF_FFFF);
        assert!(!n.targets_secure(0));
    }
}

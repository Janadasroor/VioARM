//! The STM32F1 `USART1` (`0x4001_3800`, IRQ 37).
//!
//! A byte-oriented model of the real register file: `SR`/`DR`/`BRR`/`CR1..3`
//! with the `TXE`/`TC`/`RXNE` status bits. Transmitted bytes are captured into
//! a host-readable buffer (like the generic `UartTx`); received bytes are
//! injected by the host through the stimulus channel. `RXNE` is set on
//! receive, cleared by reading `DR`, and — together with `CR1.RXNEIE` and
//! `CR1.UE` — drives a level-triggered IRQ that keeps asserting until the
//! handler drains the data register, exactly as on silicon.

use crate::device::InterruptSource;
use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

const BASE: u32 = 0x4001_3800;
const WINDOW_LEN: u32 = 0x1C;

const SR: u32 = 0x00;
const DR: u32 = 0x04;
const BRR: u32 = 0x08;
const CR1: u32 = 0x0C;
const CR2: u32 = 0x10;
const CR3: u32 = 0x14;
const GTPR: u32 = 0x18;

const SR_END: u32 = SR + 4;
const DR_END: u32 = DR + 4;
const BRR_END: u32 = BRR + 4;
const CR1_END: u32 = CR1 + 4;
const CR2_END: u32 = CR2 + 4;
const CR3_END: u32 = CR3 + 4;
const GTPR_END: u32 = GTPR + 4;

/// `SR` bit positions.
const RXNE: u32 = 1 << 5;
const TC: u32 = 1 << 6;
const TXE: u32 = 1 << 7;

/// `CR1` bit positions.
const UE: u32 = 1 << 13;
const TE: u32 = 1 << 3;
const RXNEIE: u32 = 1 << 5;

/// Exception number for USART1: IRQ 37.
const IRQ_EXCEPTION: u8 = 16 + 37;

/// The STM32F1 `USART1` peripheral.
#[derive(Debug, Clone)]
pub struct Usart1 {
    ranges: [Region; 1],
    sr: u32,
    rdr: u32,
    brr: u32,
    cr1: u32,
    cr2: u32,
    cr3: u32,
    gtpr: u32,
    output: Vec<u8>,
}

impl Usart1 {
    /// A new `USART1` at its fixed STM32F1 base address.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ranges: [Region::new(BASE, WINDOW_LEN)],
            sr: 0,
            rdr: 0,
            brr: 0,
            cr1: 0,
            cr2: 0,
            cr3: 0,
            gtpr: 0,
            output: Vec::new(),
        }
    }

    /// `SR` as firmware sees it: `TXE`/`TC` track the enable bits, the rest
    /// is latched status.
    const fn status(&self) -> u32 {
        let mut v = self.sr & !(TXE | TC);
        if self.cr1 & UE != 0 {
            v |= TC;
            if self.cr1 & TE != 0 {
                v |= TXE;
            }
        }
        v
    }

    fn reg_read(&mut self, off: u32) -> Result<u8, BusError> {
        let word = match off {
            SR..SR_END => self.status(),
            DR..DR_END => {
                let v = self.rdr;
                self.sr &= !RXNE;
                v
            }
            BRR..BRR_END => self.brr,
            CR1..CR1_END => self.cr1,
            CR2..CR2_END => self.cr2,
            CR3..CR3_END => self.cr3,
            GTPR..GTPR_END => self.gtpr,
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((word >> (8 * (off & 3))) as u8)
    }

    fn reg_write(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        let bits = u32::from(val) << shift;
        match off {
            SR..SR_END => self.sr = (self.sr & !mask) | bits,
            DR..DR_END => {
                // Byte 0 of the data register is the transmitted character;
                // the higher lanes are the extension of the same store on
                // silicon, so only it queues a byte for output.
                if off == DR {
                    if self.cr1 & TE != 0 {
                        self.output.push(val);
                    }
                    // A zero write to the data register acknowledges RXNE,
                    // matching the silicon rule that zero is not data.
                    if val == 0 {
                        self.sr &= !RXNE;
                    }
                }
            }
            BRR..BRR_END => self.brr = (self.brr & !mask) | bits,
            CR1..CR1_END => self.cr1 = (self.cr1 & !mask) | bits,
            CR2..CR2_END => self.cr2 = (self.cr2 & !mask) | bits,
            CR3..CR3_END => self.cr3 = (self.cr3 & !mask) | bits,
            GTPR..GTPR_END => self.gtpr = (self.gtpr & !mask) | bits,
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }
}

impl Default for Usart1 {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Usart1 {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        self.reg_read(addr - BASE)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        self.reg_write(addr - BASE, val)
    }

    fn reset(&mut self) {
        self.sr = 0;
        self.rdr = 0;
        self.brr = 0;
        self.cr1 = 0;
        self.cr2 = 0;
        self.cr3 = 0;
        self.gtpr = 0;
        self.output.clear();
    }

    fn name(&self) -> &'static str {
        "usart1"
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }

    fn drive_inputs(&mut self, value: u32) {
        // The low byte is one received character.
        self.rdr = value & 0xFF;
        self.sr |= RXNE;
    }
}

impl InterruptSource for Usart1 {
    fn exception_number(&self) -> Option<u8> {
        Some(IRQ_EXCEPTION)
    }

    fn pending(&self) -> u32 {
        // Level-triggered: asserts while a received byte is both unread and
        // unmasked. The handler clears it by reading `DR`.
        u32::from(self.sr & RXNE != 0 && self.cr1 & (RXNEIE | UE) == (RXNEIE | UE))
    }
}

#[cfg(test)]
mod tests {
    use super::{Usart1, BASE, CR1, DR, RXNE, RXNEIE, SR, TC, TE, TXE, UE};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(u: &mut Usart1, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(u: &mut Usart1, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn enabled() -> Usart1 {
        let mut u = Usart1::new();
        write_word(&mut u, BASE + CR1, UE | TE | (1 << 2));
        u
    }

    #[test]
    fn tx_queues_bytes_and_drains() {
        let mut u = enabled();
        u.write(BASE + DR, b'H').unwrap();
        assert_eq!(u.drain_output(), b"H");
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn status_tracks_enable_and_data_ready() {
        let mut u = enabled();
        assert_eq!(read_word(&mut u, BASE + SR) & (TXE | TC), TXE | TC);
        u.drive_inputs(0x5A);
        assert_eq!(read_word(&mut u, BASE + SR) & RXNE, RXNE);
    }

    #[test]
    fn rxne_is_cleared_by_reading_dr() {
        let mut u = enabled();
        u.drive_inputs(0x77);
        assert_eq!(read_word(&mut u, BASE + DR) & 0xFF, 0x77);
        assert_eq!(read_word(&mut u, BASE + SR) & RXNE, 0);
    }

    #[test]
    fn rx_interrupt_is_level_triggered_and_masked() {
        let mut u = Usart1::new();
        // Byte arrives before the peripheral is enabled: no interrupt yet.
        u.drive_inputs(0x41);
        assert_eq!(u.pending(), 0);
        // Enabling UE + RXNEIE (firmware init) surfaces the waiting byte.
        write_word(&mut u, BASE + CR1, UE | RXNEIE);
        assert_eq!(u.pending(), 1);
        // The handler drains DR; the level drops.
        let _ = read_word(&mut u, BASE + DR);
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn rxneie_off_means_no_interrupt() {
        let mut u = enabled();
        u.drive_inputs(0x42);
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut u = Usart1::new();
        assert!(matches!(u.read(BASE + 0x1C), Err(BusError::Reserved)));
        assert!(matches!(u.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn reset_clears_status_and_output() {
        let mut u = enabled();
        u.drive_inputs(0x11);
        write_word(&mut u, BASE + DR, 0x22);
        u.reset();
        assert_eq!(read_word(&mut u, BASE + SR) & (TXE | TC | RXNE), 0);
        assert_eq!(u.drain_output(), b"");
    }
}

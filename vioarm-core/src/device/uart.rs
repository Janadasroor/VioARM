//! A minimal output-only UART for the generic virtual board.
//!
//! Writes to the data register are captured by the simulator into a
//! host-readable buffer, so a CLI or test can observe exactly what the
//! firmware transmitted. There is no receive side yet, and the transmit side
//! never overruns: the output buffer is unbounded, so a firmware that polls
//! the TX-ready flag always sees it set.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Transmit data register: a byte written here is queued for output.
const DR: u32 = 0x00;
/// Flag register: bit 0 is `TX_READY` (a byte is accepted right now).
const FR: u32 = 0x04;
/// Last byte offset of the `DR` word.
const DR_END: u32 = DR + 3;
/// Last byte offset of the `FR` word.
const FR_END: u32 = FR + 3;
/// The register window covers `DR..=FR_END`.
const WINDOW_LEN: u32 = 0x08;

/// The TX-ready bit in the flag register.
const TX_READY: u8 = 0x01;

/// A transmit-only UART port.
#[derive(Debug, Clone)]
pub struct UartTx {
    ranges: [Region; 1],
    output: Vec<u8>,
}

impl UartTx {
    /// A new port mapped at `base`.
    #[must_use]
    pub const fn at(base: u32) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            output: Vec::new(),
        }
    }
}

impl Peripheral for UartTx {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let word = match off {
            DR..=DR_END => 0,
            FR..=FR_END => u32::from(TX_READY),
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((word >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        match addr - self.ranges[0].base {
            // Only lane 0 of the data word carries the byte; a word store's
            // upper bytes are padding.
            DR..=DR_END => {
                if addr - self.ranges[0].base == DR {
                    self.output.push(val);
                }
            }
            // FR is read-only; writes are consumed without a fault.
            FR..=FR_END => {}
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.output.clear();
    }

    fn name(&self) -> &'static str {
        "uart"
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }
}

impl InterruptSource for UartTx {}

#[cfg(test)]
mod tests {
    use super::{UartTx, DR, FR, TX_READY};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4000_1000;

    #[test]
    fn writes_queue_bytes_and_drain_is_idempotent() {
        let mut u = UartTx::at(BASE);
        u.write(BASE, b'H').unwrap();
        u.write(BASE, b'i').unwrap();
        assert_eq!(u.drain_output(), b"Hi");
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn tx_ready_flag_is_always_set() {
        let mut u = UartTx::at(BASE);
        assert_eq!(u.read(BASE + FR).unwrap(), TX_READY);
        for _ in 0..1024 {
            u.write(BASE, 0x41).unwrap();
        }
        assert_eq!(u.read(BASE + FR).unwrap(), TX_READY);
    }

    #[test]
    fn data_register_reads_zero() {
        let mut u = UartTx::at(BASE);
        u.write(BASE, 0x77).unwrap();
        assert_eq!(u.read(BASE + DR).unwrap(), 0);
    }

    #[test]
    fn flag_writes_are_ignored() {
        let mut u = UartTx::at(BASE);
        u.write(BASE + FR, 0).unwrap();
        assert_eq!(u.read(BASE + FR).unwrap(), TX_READY);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut u = UartTx::at(BASE);
        assert!(matches!(u.read(BASE + 8), Err(BusError::Reserved)));
        assert!(matches!(u.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn reset_clears_the_output_buffer() {
        let mut u = UartTx::at(BASE);
        u.write(BASE, b'x').unwrap();
        u.reset();
        assert_eq!(u.drain_output(), b"");
    }
}

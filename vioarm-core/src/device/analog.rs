//! Virtual analog pin endpoint: the millivolt transport behind N3's analog
//! lane, and the seam a future SPICE-class engine plugs into.
//!
//! Explicitly NOT analog physics: no noise, settling, impedance, or
//! temperature — values cross the lane as exact millivolt `u32`s (no FPU
//! needed on either side), single-driver-per-net by convention. The
//! register layout is documented-virtual (no silicon equivalent):
//! `TXMV@0x00` stages millivolts, any lane-0 write to `TXGO@0x04`
//! transmits the staged value to the host channel (drained by the
//! [`Fabric`](crate::fabric::Fabric) analog bus into peer `RXMV`
//! registers), `RXMV@0x08` reads the last received value and clears
//! `READY`, `SR@0x0C` carries `READY@0`, `CR@0x10` carries `EN@0` and
//! `LOOPBACK@1`. With `LOOPBACK` set, `TXGO` copies into the own `RXMV`
//! instead (also how a lone machine self-tests with no peer attached).
//! Traffic requires `EN`; while disabled, strobes are dropped, driven
//! bytes are dropped, and `READY` reads clear.
//!
//! Quanta discipline (shared with every fabric wire): delivery is
//! immediate within the pumping round, in machine-index order; a net
//! holds one value, so concurrent drivers resolve lowest-index-wins
//! (documented simplification, same spirit as the radio bus). Partial
//! trailing bytes (not a whole `u32`) are held for the next round.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Staged millivolts.
const TXMV: u32 = 0x00;
/// Transmit strobe (command, lane 0 only).
const TXGO: u32 = 0x04;
/// Last received millivolts (read clears `READY`).
const RXMV: u32 = 0x08;
/// Status register.
const SR: u32 = 0x0C;
/// Control register.
const CR: u32 = 0x10;
/// Window length.
const WINDOW_LEN: u32 = 0x14;

/// `SR` new-sample flag.
const READY: u8 = 0x01;
/// `CR` enable bit.
const EN: u32 = 1;
/// `CR` loopback bit.
const LOOPBACK: u32 = 1 << 1;

/// A virtual analog pin: millivolt staging/transmit on one side, last-
/// value receive on the other, over the fabric's analog bus.
#[derive(Debug, Clone, Default)]
pub struct Apin {
    ranges: [Region; 1],
    enabled: bool,
    loopback: bool,
    tx_mv: u32,
    rx_mv: u32,
    rx_ready: bool,
    tx_out: Vec<u8>,
    pending: Vec<u8>,
}

impl Apin {
    /// A new pin window mapped at `base`.
    #[must_use]
    pub const fn at(base: u32) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            enabled: false,
            loopback: false,
            tx_mv: 0,
            rx_mv: 0,
            rx_ready: false,
            tx_out: Vec::new(),
            pending: Vec::new(),
        }
    }

    /// Transmit the staged value: into the own receiver under loopback,
    /// otherwise out to the host channel (the fabric's analog bus).
    fn transmit(&mut self) {
        if !self.enabled {
            return;
        }
        if self.loopback {
            self.rx_mv = self.tx_mv;
            self.rx_ready = true;
        } else {
            self.tx_out.extend_from_slice(&self.tx_mv.to_le_bytes());
        }
    }

    /// A stored word with byte `val` merged into lane `shift`.
    fn merged_word(current: u32, val: u8, shift: u32) -> u32 {
        let mask = 0xFFu32 << shift;
        (current & !mask) | (u32::from(val) << shift)
    }
}

impl Peripheral for Apin {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let lane = off & 3;
        let word = off & !3;
        let value = match word {
            TXMV => self.tx_mv,
            TXGO => 0,
            RXMV => {
                let value = if self.enabled { self.rx_mv } else { 0 };
                self.rx_ready = false;
                value
            }
            SR => u32::from(self.enabled && self.rx_ready),
            CR => u32::from(self.enabled) | if self.loopback { LOOPBACK } else { 0 },
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((value >> (8 * lane)) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let lane = off & 3;
        let shift = 8 * lane;
        match off & !3 {
            TXMV => self.tx_mv = Self::merged_word(self.tx_mv, val, shift),
            // Lane 0 strobes (a word store strobes once, like `UartTx` DR).
            TXGO => {
                if lane == 0 {
                    self.transmit();
                }
            }
            // `RXMV`/`SR` are read-only; writes are consumed silently.
            RXMV | SR => {}
            CR => {
                let word = Self::merged_word(
                    u32::from(self.enabled) | if self.loopback { LOOPBACK } else { 0 },
                    val,
                    shift,
                );
                self.enabled = word & EN != 0;
                self.loopback = word & LOOPBACK != 0;
            }
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.enabled = false;
        self.loopback = false;
        self.tx_mv = 0;
        self.rx_mv = 0;
        self.rx_ready = false;
        self.tx_out.clear();
        self.pending.clear();
    }

    fn name(&self) -> &'static str {
        "apin"
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }

    fn drive_inputs(&mut self, value: u32) {
        if !self.enabled {
            return;
        }
        // Reassemble little-endian millivolt words; a trailing partial
        // word waits for the next round.
        self.pending.push((value & 0xFF) as u8);
        while self.pending.len() >= 4 {
            let word = u32::from_le_bytes([
                self.pending[0],
                self.pending[1],
                self.pending[2],
                self.pending[3],
            ]);
            self.pending.drain(..4);
            self.rx_mv = word;
            self.rx_ready = true;
        }
    }
}

impl InterruptSource for Apin {}

#[cfg(test)]
mod tests {
    use super::{Apin, CR, EN, LOOPBACK, READY, RXMV, SR, TXGO, TXMV};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4000_2000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(p: &mut Apin, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(p: &mut Apin, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = p.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn loopback_copies_staged_mv_and_sets_ready() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN | LOOPBACK);
        write_word(&mut p, TXMV, 1200);
        write_word(&mut p, TXGO, 1);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), u32::from(READY));
        assert_eq!(read_word(&mut p, RXMV), 1200);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
    }

    #[test]
    fn disabled_window_drops_traffic() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, TXMV, 1200);
        write_word(&mut p, TXGO, 1);
        assert!(p.drain_output().is_empty());
        p.drive_inputs(0xD2);
        p.drive_inputs(0x04);
        p.drive_inputs(0x00);
        p.drive_inputs(0x00);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
    }

    #[test]
    fn host_channel_round_trips_one_word() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN);
        write_word(&mut p, TXMV, 2400);
        write_word(&mut p, TXGO, 1);
        assert_eq!(p.drain_output(), vec![0x60, 0x09, 0x00, 0x00]);
        for b in [0x10u8, 0x27, 0x00, 0x00] {
            p.drive_inputs(u32::from(b));
        }
        assert_eq!(read_word(&mut p, RXMV), 10000);
    }

    #[test]
    fn partial_words_wait_for_the_next_round() {
        let mut p = Apin::at(BASE);
        write_word(&mut p, CR, EN);
        p.drive_inputs(0x01);
        p.drive_inputs(0x02);
        assert_eq!(read_word(&mut p, SR) & u32::from(READY), 0);
        p.drive_inputs(0x03);
        p.drive_inputs(0x04);
        assert_eq!(read_word(&mut p, RXMV), 0x0403_0201);
    }

    #[test]
    fn outside_the_window_is_reserved() {
        let mut p = Apin::at(BASE);
        assert!(matches!(p.read(BASE + 0x14), Err(BusError::Reserved)));
        assert!(matches!(p.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}

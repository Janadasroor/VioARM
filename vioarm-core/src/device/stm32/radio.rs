//! Virtual radio link-layer endpoint: the packet engine behind `SUBGHZ`
//! packet mode and the `BLE` advertising window.
//!
//! Explicitly NOT RF physics/propagation/timing: bytes staged through
//! `TXDATA` leave via `TXGO` for the host channel (drained by the
//! [`Fabric`](crate::fabric::Fabric) radio bus into peer RX FIFOs), and
//! inbound bytes arrive through the host channel into the RX FIFO. With
//! `CR.LOOPBACK` set, `TXGO` copies into the own RX FIFO instead (also how
//! a lone machine self-tests with no peer attached).
//!
//! The register layout is virtual where silicon has no equivalent: real WL
//! SUBGHZ packets move through `SUBGHZSPI` commands (a whole protocol, out
//! of scope) and real WBA BLE runs on CPU2 via IPCC (no AP registers at
//! all). Only `SUBGHZ CR/SR` keep silicon meaning (`EN` → `READY` +
//! interrupt, preserved from the stub); everything from `TXLEN` on is the
//! documented-virtual endpoint.
//!
//! Link layer (this item): every frame on the virtual air is
//! `LEN | ADDR? | PAYLOAD | CRC?` — one length byte (payload length,
//! 0–255; a `TXGO` takes at most 255 payload bytes, the rest stays
//! staged), an optional node-address byte, the payload, and an optional
//! big-endian CRC-16/CCITT-FALSE (`poly 0x1021`, init `0xFFFF`) over the
//! address + payload. Receivers parse this framing from the byte stream
//! (split deliveries reassemble); address mismatches drop silently while
//! CRC failures latch `SR.CRCERR` — both count in `DROPSTAT`. Link
//! parameters must match on both ends (like real radios); the CRC is NOT
//! BLE CRC-24 nor the LoRa header (protocol, out of scope like
//! `SUBGHZSPI` commands).
//!
//! Map (`CR` at `0x00`; `SR` beside it per device): `TXLEN@0x20` (payload
//! bytes to take on `TXGO`, `0` = all staged up to 255), `TXDATA@0x24`
//! (payload bytes, lane order; staged past 256 drop silently),
//! `TXGO@0x28` (command, any write transmits), `RXSTAT@0x2C` (pending RX
//! payload bytes), `RXDATA@0x30` (FIFO pop: each access consumes what it
//! reads — a word read pops four payload bytes, a byte read one; empty
//! pops read `0`), `ADDRF@0x34` (node address `[7:0]` + `ADDR_EN@8`,
//! reset promiscuous), `CRCF@0x38` (`CRC_EN@0`, reset off),
//! `DROPSTAT@0x3C` (dropped frames, saturating, cleared on any write).
//! `CR`: `EN@0`, `LOOPBACK@1`, `RXIE@2`, `TXIE@3`. `SR`: `READY@0`
//! (follows `EN`), `RXNE@1` (RX pending), `TXDONE@2` (write-1-to-clear,
//! set by `TXGO`), `CRCERR@3` (write-1-to-clear, set by a bad CRC).

use crate::device::svd::{Behavior, Registers};
use std::collections::VecDeque;

// Control bits.
const EN: u32 = 1 << 0;
const LOOPBACK: u32 = 1 << 1;
const RXIE: u32 = 1 << 2;
const TXIE: u32 = 1 << 3;
// Status bits.
const READY: u32 = 1 << 0;
const RXNE: u32 = 1 << 1;
const TXDONE: u32 = 1 << 2;
/// CRC failure latch (write-1-to-clear, set by a bad frame).
const CRCERR: u32 = 1 << 3;

/// Virtual packet registers (offsets from the window base).
pub const TXLEN: u32 = 0x20;
/// Payload staging register.
pub const TXDATA: u32 = 0x24;
/// Transmit strobe (command).
pub const TXGO: u32 = 0x28;
/// Pending RX bytes (read-only count).
pub const RXSTAT: u32 = 0x2c;
/// Pop one RX byte (`0` when empty).
pub const RXDATA: u32 = 0x30;
/// Node address + filter enable (`ADDR[7:0]`, `ADDR_EN@8`).
pub const ADDRF: u32 = 0x34;
/// Address-filter enable bit.
pub const ADDR_EN: u32 = 1 << 8;
/// CRC control (`CRC_EN@0`).
pub const CRCF: u32 = 0x38;
/// CRC enable bit.
pub const CRC_EN: u32 = 1 << 0;
/// Dropped-frame counter (address miss + CRC error, saturating).
pub const DROPSTAT: u32 = 0x3c;

/// Staged TX payload cap in bytes (SUBGHZ packets top out at 255, BLE
/// advertising at 37 — beyond the cap drops silently, documented).
pub const TX_CAP: usize = 256;

/// CRC-16/CCITT-FALSE (`poly 0x1021`, init `0xFFFF`, no reflection):
/// `"123456789"` checks `0x29B1`.
fn crc16_ccitt(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in bytes {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// The packet engine: TX staging/transmit, RX stream parsing, done latch.
/// One instance serves a whole window; `sr_off` places the status register
/// (`0x10` keeps the WL `SUBGHZ` layout, `0x04` the conventional spot).
#[derive(Clone, Debug)]
pub struct PacketBehavior {
    sr_off: u32,
    enabled: bool,
    tx_stage: Vec<u8>,
    tx_out: Vec<u8>,
    rx_fifo: VecDeque<u8>,
    rx_stream: Vec<u8>,
    tx_len: u32,
    tx_done: bool,
    crc_err: bool,
    drops: u32,
}

impl PacketBehavior {
    /// A packet engine with the status register at `sr_off`.
    #[must_use]
    pub const fn new(sr_off: u32) -> Self {
        Self {
            sr_off,
            enabled: false,
            tx_stage: Vec::new(),
            tx_out: Vec::new(),
            rx_fifo: VecDeque::new(),
            rx_stream: Vec::new(),
            tx_len: 0,
            tx_done: false,
            crc_err: false,
            drops: 0,
        }
    }

    fn addr_on(view: &Registers<&mut [u32]>) -> bool {
        view.get(ADDRF) & ADDR_EN != 0
    }

    fn crc_on(view: &Registers<&mut [u32]>) -> bool {
        view.get(CRCF) & CRC_EN != 0
    }

    /// Build one wire frame (`LEN | ADDR? | PAYLOAD | CRC?`) from up to
    /// 255 staged payload bytes; the rest stays staged for the next `TXGO`.
    fn build_frame(&mut self, view: &Registers<&mut [u32]>) -> Vec<u8> {
        let want = usize::try_from(self.tx_len).unwrap_or(usize::MAX);
        let n = if want == 0 {
            self.tx_stage.len()
        } else {
            want.min(self.tx_stage.len())
        }
        .min(255);
        let payload: Vec<u8> = self.tx_stage.drain(..n).collect();
        let mut frame = Vec::with_capacity(1 + n + 3);
        frame.push(n as u8);
        if Self::addr_on(view) {
            frame.push((view.get(ADDRF) & 0xFF) as u8);
        }
        frame.extend_from_slice(&payload);
        if Self::crc_on(view) {
            let crc = crc16_ccitt(&frame[1..]);
            frame.extend(crc.to_be_bytes());
        }
        frame
    }

    /// Transmit the staged payload: parsed back into the own RX FIFO under
    /// loopback (so self-filtering applies), otherwise out to the host
    /// channel (the fabric's radio bus).
    fn transmit(&mut self, view: &mut Registers<&mut [u32]>) {
        let frame = self.build_frame(view);
        if view.get(0x00) & LOOPBACK != 0 {
            self.rx_stream.extend_from_slice(&frame);
            self.parse_stream(view);
        } else {
            self.tx_out.extend(frame);
        }
        self.tx_done = true;
    }

    /// Parse complete wire frames from the receive stream: address filter
    /// drops silently, CRC failures latch `CRCERR` — both count drops.
    /// Short reads wait (split deliveries reassemble).
    fn parse_stream(&mut self, view: &mut Registers<&mut [u32]>) {
        loop {
            if self.rx_stream.is_empty() {
                break;
            }
            let len = self.rx_stream[0] as usize;
            let addr_on = Self::addr_on(view);
            let crc_on = Self::crc_on(view);
            let need = 1 + usize::from(addr_on) + len + if crc_on { 2 } else { 0 };
            if self.rx_stream.len() < need {
                break;
            }
            let frame: Vec<u8> = self.rx_stream.drain(..need).collect();
            let mut body = &frame[1..];
            if addr_on {
                if body[0] != (view.get(ADDRF) & 0xFF) as u8 {
                    self.drops = self.drops.saturating_add(1);
                    continue;
                }
                body = &body[1..];
            }
            let payload = &body[..len];
            if crc_on {
                let at = 1 + usize::from(addr_on) + len;
                let got = u16::from_be_bytes([frame[at], frame[at + 1]]);
                if crc16_ccitt(&frame[1..at]) != got {
                    self.drops = self.drops.saturating_add(1);
                    self.crc_err = true;
                    continue;
                }
            }
            self.rx_fifo.extend(payload.iter());
        }
    }
}

impl Behavior for PacketBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == self.sr_off {
            // Status is composed on read; `TXDONE`/`CRCERR` clear here.
            let clear = u32::from(val) << shift & (TXDONE | CRCERR);
            if clear & TXDONE != 0 {
                self.tx_done = false;
            }
            if clear & CRCERR != 0 {
                self.crc_err = false;
            }
            return view.get(offset);
        }
        if offset == TXGO {
            // Command strobe: lane 0 transmits (word-write convention —
            // the bus splits words into lane writes, and only the first
            // may fire the command). Nothing is stored.
            if shift == 0 && self.enabled {
                self.transmit(view);
            }
            return view.get(offset);
        }
        if offset == TXDATA {
            // Payload bytes append in lane order (word and byte writes
            // both work); past the cap drops silently (documented).
            if self.tx_stage.len() < TX_CAP {
                self.tx_stage.push(val);
            }
            return view.merged(offset, val, shift);
        }
        if offset == TXLEN {
            let word = view.merged(offset, val, shift);
            self.tx_len = word;
            return word;
        }
        if offset == DROPSTAT {
            // Any write clears the drop counter (the latch reads back the
            // count through `on_read`, so nothing is stored).
            self.drops = 0;
            return 0;
        }
        if offset == 0x00 {
            // Control register: `EN` latches the session (fresh sessions
            // clear a stale done flag), the rest stores through.
            let word = view.merged(offset, val, shift);
            let was = self.enabled;
            self.enabled = word & EN != 0;
            if self.enabled && !was {
                self.tx_done = false;
            }
            return word;
        }
        view.merged(offset, val, shift)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == self.sr_off {
            let mut word = value & !(READY | RXNE | TXDONE | CRCERR);
            if self.enabled {
                word |= READY;
            }
            if !self.rx_fifo.is_empty() {
                word |= RXNE;
            }
            if self.tx_done {
                word |= TXDONE;
            }
            if self.crc_err {
                word |= CRCERR;
            }
            return word;
        }
        if offset == RXSTAT {
            return u32::try_from(self.rx_fifo.len()).unwrap_or(u32::MAX);
        }
        if offset == DROPSTAT {
            return self.drops;
        }
        if offset == RXDATA {
            // FIFO semantics: each access consumes what it reads (a word
            // read pops four payload bytes, a byte read one).
            return u32::from(self.rx_fifo.pop_front().unwrap_or(0));
        }
        value
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(0x00);
        // Legacy latch: enabled implies READY, which raises the line (kept
        // for the WL stub's behavior); packet interrupts gate on their bits.
        let legacy = self.enabled;
        let rx = !self.rx_fifo.is_empty() && cr & RXIE != 0;
        let tx = self.tx_done && cr & TXIE != 0;
        u32::from(legacy || rx || tx)
    }

    fn reset(&mut self) {
        let sr_off = self.sr_off;
        *self = Self::new(sr_off);
    }

    fn needs_tick() -> bool {
        false
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // Wire bytes accumulate in the receive stream; complete frames
        // parse out (split deliveries reassemble). The debugger speaks
        // the same wire format (documented).
        self.rx_stream.push((value & 0xFF) as u8);
        self.parse_stream(view);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx_out)
    }
}

#[cfg(test)]
mod tests {
    use super::{PacketBehavior, RXDATA, RXSTAT, TXDATA, TXGO, TXLEN};
    use crate::device::svd::{Access, Behavior, PeripheralDef, RegisterDef, RegisterFile};
    use crate::device::{InterruptSource, Peripheral};

    const DEF: PeripheralDef = PeripheralDef {
        name: "RADIO",
        base: 0x5800_0000,
        window: 0x400,
        irqs: &[],
        registers: &[
            RegisterDef {
                name: "CR",
                offset: 0x00,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "SR",
                offset: 0x04,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "TXLEN",
                offset: 0x20,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "TXDATA",
                offset: 0x24,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "TXGO",
                offset: 0x28,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "RXSTAT",
                offset: 0x2c,
                width: 32,
                access: Access::ReadOnly,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "RXDATA",
                offset: 0x30,
                width: 32,
                access: Access::ReadOnly,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "ADDRF",
                offset: 0x34,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "CRCF",
                offset: 0x38,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
            RegisterDef {
                name: "DROPSTAT",
                offset: 0x3c,
                width: 32,
                access: Access::ReadWrite,
                reset: 0,
                fields: &[],
            },
        ],
    };

    fn radio() -> RegisterFile<PacketBehavior> {
        RegisterFile::new(&DEF, PacketBehavior::new(0x04))
    }

    #[allow(clippy::cast_possible_truncation)]
    fn wr(r: &mut RegisterFile<PacketBehavior>, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            r.write(0x5800_0000 + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn rd(r: &mut RegisterFile<PacketBehavior>, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = r.read(0x5800_0000 + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn txgo_drains_and_latches_txdone() {
        let mut r = radio();
        wr(&mut r, 0x00, 1 | (1 << 3)); // EN + TXIE
        assert_ne!(rd(&mut r, 0x04) & 1, 0, "READY");
        wr(&mut r, TXDATA, 0x4443_4241); // "ABCD" lane order
        wr(&mut r, TXLEN, 4);
        wr(&mut r, TXGO, 1);
        assert_ne!(rd(&mut r, 0x04) & (1 << 2), 0, "TXDONE");
        assert_ne!(r.pending(), 0, "TXIE raises the line");
        wr(&mut r, 0x04, 1 << 2); // W1C TXDONE
        assert_eq!(rd(&mut r, 0x04) & (1 << 2), 0, "cleared");
        let mut out = Vec::new();
        r.with_view(|_, b| out.extend(b.drain_output()));
        assert_eq!(out, b"\x04ABCD", "length-prefixed frame");
    }

    #[test]
    fn loopback_copies_into_own_rx() {
        let mut r = radio();
        wr(&mut r, 0x00, 1 | (1 << 1) | (1 << 2)); // EN + LOOPBACK + RXIE
        wr(&mut r, TXDATA, 0x42);
        wr(&mut r, TXLEN, 1);
        wr(&mut r, TXGO, 1);
        assert_eq!(rd(&mut r, RXSTAT), 1);
        assert_ne!(r.pending(), 0, "RXIE raises the line");
        assert_eq!(rd(&mut r, RXDATA), 0x42);
        assert_eq!(rd(&mut r, RXSTAT), 0);
        let mut out = Vec::new();
        r.with_view(|_, b| out.extend(b.drain_output()));
        assert!(out.is_empty(), "nothing reaches the air");
    }

    #[test]
    fn host_channel_feeds_rx() {
        let mut r = radio();
        wr(&mut r, 0x00, 1);
        // The debugger speaks the wire format: length-prefixed frames.
        r.with_view(|view, b| {
            b.drive_inputs(view, 1);
            b.drive_inputs(view, 0x7E);
        });
        assert_eq!(rd(&mut r, RXSTAT), 1);
        assert_eq!(rd(&mut r, RXDATA), 0x7E);
        assert_eq!(rd(&mut r, RXDATA), 0, "empty pops zero");
        // A short read waits: length 2 with one byte present parses nothing.
        r.with_view(|view, b| {
            b.drive_inputs(view, 2);
            b.drive_inputs(view, 0xAA);
        });
        assert_eq!(rd(&mut r, RXSTAT), 0, "partial frame waits");
        r.with_view(|view, b| b.drive_inputs(view, 0xBB));
        assert_eq!(rd(&mut r, RXSTAT), 2, "split delivery reassembles");
    }

    #[test]
    fn txgo_while_disabled_does_nothing() {
        let mut r = radio();
        wr(&mut r, TXDATA, 0x41);
        wr(&mut r, TXGO, 1);
        assert_eq!(rd(&mut r, 0x04) & (1 << 2), 0, "no TXDONE");
        let mut out = Vec::new();
        r.with_view(|_, b| out.extend(b.drain_output()));
        assert!(out.is_empty());
    }

    #[test]
    fn crc16_checks_123456789() {
        assert_eq!(super::crc16_ccitt(b"123456789"), 0x29B1);
    }

    /// Enable address + CRC with node address `addr`.
    fn link_up(r: &mut RegisterFile<PacketBehavior>, addr: u8) {
        wr(r, super::ADDRF, (1 << 8) | u32::from(addr));
        wr(r, super::CRCF, 1);
    }

    #[test]
    fn tx_appends_addr_and_crc() {
        let mut r = radio();
        wr(&mut r, 0x00, 1);
        link_up(&mut r, 0x42);
        wr(&mut r, TXDATA, 0x4443_4241);
        wr(&mut r, TXLEN, 4);
        wr(&mut r, TXGO, 1);
        let mut out = Vec::new();
        r.with_view(|_, b| out.extend(b.drain_output()));
        // LEN + ADDR + "ABCD" + CRC-16 over ADDR+payload.
        assert_eq!(&out[..6], &[4, 0x42, b'A', b'B', b'C', b'D']);
        assert_eq!(
            u16::from_be_bytes([out[6], out[7]]),
            super::crc16_ccitt(&out[1..6])
        );
    }

    #[test]
    fn rx_accepts_addressed_crc_frame() {
        let mut r = radio();
        wr(&mut r, 0x00, 1 | (1 << 2)); // EN + RXIE
        link_up(&mut r, 0x42);
        let payload = b"hi";
        let mut frame = vec![2, 0x42, payload[0], payload[1]];
        let crc = super::crc16_ccitt(&frame[1..]);
        frame.extend(crc.to_be_bytes());
        r.with_view(|view, b| {
            for byte in frame {
                b.drive_inputs(view, u32::from(byte));
            }
        });
        assert_eq!(rd(&mut r, RXSTAT), 2);
        // Single-byte pops (a word read would consume four payload bytes).
        let base = 0x5800_0000;
        let mut b0 = [0u8; 1];
        let mut b1 = [0u8; 1];
        b0[0] = r.read(base + RXDATA).unwrap();
        b1[0] = r.read(base + RXDATA).unwrap();
        assert_eq!([b0[0], b1[0]], *payload);
        assert_eq!(rd(&mut r, 0x04) & (1 << 3), 0, "no CRCERR");
        assert_eq!(rd(&mut r, super::DROPSTAT), 0, "no drops");
    }

    #[test]
    fn rx_drops_bad_crc_with_flag_and_count() {
        let mut r = radio();
        wr(&mut r, 0x00, 1);
        link_up(&mut r, 0x42);
        r.with_view(|view, b| {
            for byte in [2u8, 0x42, b'a', b'b', 0xDE, 0xAD] {
                b.drive_inputs(view, u32::from(byte));
            }
        });
        assert_eq!(rd(&mut r, RXSTAT), 0, "frame dropped");
        assert_ne!(rd(&mut r, 0x04) & (1 << 3), 0, "CRCERR latches");
        assert_eq!(rd(&mut r, super::DROPSTAT), 1, "counted");
        wr(&mut r, 0x04, 1 << 3); // W1C CRCERR
        assert_eq!(rd(&mut r, 0x04) & (1 << 3), 0, "cleared");
        wr(&mut r, super::DROPSTAT, 0xFFFF_FFFF); // any write clears
        assert_eq!(rd(&mut r, super::DROPSTAT), 0, "count cleared");
    }

    #[test]
    fn rx_drops_wrong_address_silently() {
        let mut r = radio();
        wr(&mut r, 0x00, 1);
        link_up(&mut r, 0x42);
        // Addressed to 0x43, CRC over its own bytes so only the
        // address check fires.
        let mut frame = vec![1, 0x43, 0x55];
        let crc = super::crc16_ccitt(&frame[1..]);
        frame.extend(crc.to_be_bytes());
        r.with_view(|view, b| {
            for byte in frame {
                b.drive_inputs(view, u32::from(byte));
            }
        });
        assert_eq!(rd(&mut r, RXSTAT), 0, "frame dropped");
        assert_eq!(rd(&mut r, 0x04) & (1 << 3), 0, "no CRCERR on miss");
        assert_eq!(rd(&mut r, super::DROPSTAT), 1, "miss counted");
    }

    #[test]
    fn loopback_applies_own_filter() {
        let mut r = radio();
        wr(&mut r, 0x00, 1 | (1 << 1)); // EN + LOOPBACK
        link_up(&mut r, 0x42);
        wr(&mut r, TXDATA, 0x41);
        wr(&mut r, TXLEN, 1);
        wr(&mut r, TXGO, 1);
        assert_eq!(rd(&mut r, RXSTAT), 1, "own address loops back");
        assert_eq!(rd(&mut r, RXDATA), 0x41);
    }
}

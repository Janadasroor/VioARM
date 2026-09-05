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
//! documented-virtual endpoint. The byte stream carries no framing:
//! payload bytes in, payload bytes out, in order.
//!
//! Map (`CR` at `0x00`; `SR` beside it per device): `TXLEN@0x20` (bytes to
//! take on `TXGO`, `0` = all staged), `TXDATA@0x24` (payload bytes,
//! lane order; staged past 256 drop silently), `TXGO@0x28` (command, any
//! write transmits), `RXSTAT@0x2C` (pending RX bytes), `RXDATA@0x30`
//! (FIFO pop: each access consumes what it reads — a word read pops four
//! payload bytes, a byte read one; empty pops read `0`). `CR`: `EN@0`,
//! `LOOPBACK@1`, `RXIE@2`, `TXIE@3`. `SR`: `READY@0` (follows `EN`),
//! `RXNE@1` (RX pending), `TXDONE@2` (write-1-to-clear, set by `TXGO`).

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

/// Staged TX payload cap in bytes (SUBGHZ packets top out at 255, BLE
/// advertising at 37 — beyond the cap drops silently, documented).
pub const TX_CAP: usize = 256;

/// The packet engine: TX staging/transmit, RX FIFO, done latch. One
/// instance serves a whole window; `sr_off` places the status register
/// (`0x10` keeps the WL `SUBGHZ` layout, `0x04` the conventional spot).
#[derive(Clone, Debug)]
pub struct PacketBehavior {
    sr_off: u32,
    enabled: bool,
    tx_stage: Vec<u8>,
    tx_out: Vec<u8>,
    rx_fifo: VecDeque<u8>,
    tx_len: u32,
    tx_done: bool,
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
            tx_len: 0,
            tx_done: false,
        }
    }

    /// Transmit the staged payload: into the own RX FIFO under loopback,
    /// otherwise out to the host channel (the fabric's radio bus).
    fn transmit(&mut self, view: &Registers<&mut [u32]>) {
        let cr = view.get(0x00);
        let want = usize::try_from(self.tx_len).unwrap_or(usize::MAX);
        let n = if want == 0 {
            self.tx_stage.len()
        } else {
            want.min(self.tx_stage.len())
        };
        let payload: Vec<u8> = self.tx_stage.drain(..n).collect();
        if cr & LOOPBACK != 0 {
            self.rx_fifo.extend(payload);
        } else {
            self.tx_out.extend(payload);
        }
        self.tx_done = true;
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
            // Status is composed on read; only `TXDONE` is clearable here.
            if u32::from(val) << shift & TXDONE != 0 {
                self.tx_done = false;
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
            let mut word = value & !(READY | RXNE | TXDONE);
            if self.enabled {
                word |= READY;
            }
            if !self.rx_fifo.is_empty() {
                word |= RXNE;
            }
            if self.tx_done {
                word |= TXDONE;
            }
            return word;
        }
        if offset == RXSTAT {
            return u32::try_from(self.rx_fifo.len()).unwrap_or(u32::MAX);
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

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        self.rx_fifo.push_back((value & 0xFF) as u8);
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
        assert_eq!(out, b"ABCD");
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
        r.with_view(|view, b| b.drive_inputs(view, 0x7E));
        assert_eq!(rd(&mut r, RXSTAT), 1);
        assert_eq!(rd(&mut r, RXDATA), 0x7E);
        assert_eq!(rd(&mut r, RXDATA), 0, "empty pops zero");
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
}

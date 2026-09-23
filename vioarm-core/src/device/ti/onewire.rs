//! TI Tiva C 1-Wire master (`ONEWIRE0`, `0x400B_6000`).
//!
//! Register map and transaction semantics follow the TI `hw_onewire.h` +
//! `onewire.c` oracle (BSD): `CS` carries `RST` (bit 0), `OP` (`RD`/`WR`/
//! `WRRD`, bits 2:1), `SZ` (bytes-1, bits 4:3), `BSIZE` (last-byte bits,
//! 0 = 8; bits 18:16), status `BUSY`/`NOATR`/`STUCK`; `DATW`/`DATR`
//! 32-bit data; `TIM` timing overrides; `IM`/`RIS`/`MIS`/`ICR`
//! (`RESET_DONE`/`OP_DONE`/`NO_SLAVE`/`STUCK`/`DMA_DONE`).
//!
//! Modeled: reset with presence detection (`NOATR` + `NO_SLAVE` when no
//! device is attached), instant `WR`/`RD`/`WRRD` transactions over
//! `DATW`/`DATR` (first byte in the low bits), unattached read bytes
//! return `0xFF` (the bus idles high — physically honest; untouched
//! `DATR` bytes keep prior content), completion interrupts + `ICR` W1C
//! + `IM` gating, an attachable byte-source device.
//!
//! Deliberate scope: `BUSY` never reads set (instant completion, same
//! convention as the other TI models); search-ROM/triplet flows are
//! firmware bit patterns over `RD`/`WR` (no engine assist needed);
//! `TIM`/`DMA`/`PP` latch; overdrive/two-wire-alternate modes latch.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

/// `ONEWIRE0` base address.
pub const ONEWIRE_BASE: u32 = 0x400B_6000;

const CS_OFF: u32 = 0x00;
const DATW_OFF: u32 = 0x08;
const DATR_OFF: u32 = 0x0C;
const IM_OFF: u32 = 0x100;
const RIS_OFF: u32 = 0x104;
const MIS_OFF: u32 = 0x108;
const ICR_OFF: u32 = 0x10C;

/// `CS` bits.
const CS_RST: u32 = 1 << 0;
const CS_OP_MASK: u32 = 0x06;
const CS_OP_RD: u32 = 0x02;
const CS_OP_WR: u32 = 0x04;
const CS_OP_WRRD: u32 = 0x06;
const CS_SZ_SHIFT: u32 = 3;
const CS_SZ_MASK: u32 = 0x03;
const CS_BUSY: u32 = 1 << 8;
const CS_NOATR: u32 = 1 << 9;
const CS_STUCK: u32 = 1 << 10;
const CS_BSIZE_SHIFT: u32 = 16;
const CS_BSIZE_MASK: u32 = 0x07;
/// Interrupt bits.
const INT_RESET_DONE: u32 = 1 << 0;
const INT_OP_DONE: u32 = 1 << 1;
const INT_NO_SLAVE: u32 = 1 << 2;

/// Behavior behind the 1-Wire window.
#[derive(Debug, Default)]
pub struct OneWireBehavior {
    /// Attached device response bytes (presence = attached).
    device: VecDeque<u8>,
    /// Last transaction's read bytes (first byte in low bits of `DATR`).
    read_back: u32,
    /// Bytes of the last `WR` transaction (for tests/introspection).
    written: Vec<u8>,
    /// Latched interrupt flags.
    ris: u32,
}

impl OneWireBehavior {
    /// Attach a byte-source device (presence + read responses).
    pub fn attach_device(&mut self, bytes: &[u8]) {
        self.device = bytes.iter().copied().collect();
    }

    /// Run one `WR`/`RD`/`WRRD` transaction instantly.
    fn transact(&mut self, view: &mut Registers<&mut [u32]>, op: u32) {
        let cs = view.get(CS_OFF);
        let nbytes = (((cs >> CS_SZ_SHIFT) & CS_SZ_MASK) + 1) as usize;
        let last_bits = ((cs >> CS_BSIZE_SHIFT) & CS_BSIZE_MASK) as usize;
        let last_bits = if last_bits == 0 { 8 } else { last_bits };
        if op & CS_OP_WR != 0 {
            let w = view.get(DATW_OFF);
            self.written.clear();
            for i in 0..nbytes {
                let mut b = ((w >> (8 * i)) & 0xFF) as u8;
                if i + 1 == nbytes && last_bits < 8 {
                    b &= (1 << last_bits) - 1;
                }
                self.written.push(b);
            }
            // Writes go out to the device; they never consume its
            // response bytes (reads do that below).
        }
        if op & CS_OP_RD != 0 {
            let mut word = 0u32;
            for i in 0..nbytes {
                let mut b = self.device.pop_front().unwrap_or(0xFF);
                if i + 1 == nbytes && last_bits < 8 {
                    // Partial last byte: kept bits from the device, the
                    // rest idle high.
                    let mask = (1u8 << last_bits) - 1;
                    b = (b & mask) | !mask;
                }
                word |= u32::from(b) << (8 * i);
            }
            self.read_back = word;
        }
        self.ris |= INT_OP_DONE;
    }
}

impl Behavior for OneWireBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.read_back = 0;
        self.written.clear();
        self.ris = 0;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.ris & view.get(IM_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            // `BUSY`/`STUCK` never latch (instant completion, no faults);
            // `NOATR` is live state firmware polls.
            CS_OFF => value & !(CS_BUSY | CS_STUCK),
            DATR_OFF => self.read_back,
            RIS_OFF => self.ris,
            MIS_OFF => self.ris & view.get(IM_OFF),
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == RIS_OFF || offset == MIS_OFF {
            return view.get(offset);
        }
        if offset == ICR_OFF {
            let word = view.merged(offset, val, shift);
            self.ris &= !word;
            view.set(offset, 0);
            return 0;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if offset != CS_OFF || shift != 24 {
            // Word stores land lane by lane; the transaction decodes
            // once whole (TivaWare writes full `CS` words).
            return word;
        }
        if offset == CS_OFF {
            // `BUSY` never latches (instant completion); status reflects
            // the just-finished transaction.
            if word & CS_RST != 0 {
                let present = !self.device.is_empty();
                let mut cs = word & !(CS_RST | CS_NOATR);
                if !present {
                    cs |= CS_NOATR;
                    self.ris |= INT_NO_SLAVE;
                }
                view.set(offset, cs);
                self.ris |= INT_RESET_DONE;
                return cs;
            }
            match word & CS_OP_MASK {
                CS_OP_RD | CS_OP_WR | CS_OP_WRRD => self.transact(view, word & CS_OP_MASK),
                _ => {}
            }
        }
        word
    }
}

/// A Tiva C 1-Wire master (instant transactions; attachable device).
pub struct OneWire {
    inner: RegisterFile<OneWireBehavior>,
}

impl OneWire {
    /// A new 1-Wire master on `D`'s tables (`"ONEWIRE0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"ONEWIRE0"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("ONEWIRE0").expect("generated tables contain ONEWIRE0");
        Self {
            inner: RegisterFile::new(def, OneWireBehavior::default()),
        }
    }

    /// Attach a byte-source device (presence + read responses).
    pub fn attach_device(&mut self, bytes: &[u8]) {
        self.inner.with_view(|_, b| b.attach_device(bytes));
    }
}

impl Peripheral for OneWire {
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
        self.inner.reset();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for OneWire {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const ONEWIRE0_BASE: u32 = 0x400B_6000;

    fn onewire() -> OneWire {
        OneWire::for_device::<Tm4c129xnczad>()
    }

    fn read_word(ow: &mut OneWire, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(ow.read(ONEWIRE0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(ow: &mut OneWire, offset: u32, value: u32) {
        for lane in 0..4 {
            ow.write(
                ONEWIRE0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Reset with no device: `NOATR` + `NO_SLAVE`; with device: clean.
    #[test]
    fn reset_presence() {
        let mut ow = onewire();
        write_word(&mut ow, CS_OFF, CS_RST);
        assert_eq!(read_word(&mut ow, CS_OFF) & CS_NOATR, CS_NOATR);
        assert_eq!(read_word(&mut ow, RIS_OFF) & INT_NO_SLAVE, INT_NO_SLAVE);
        assert_eq!(read_word(&mut ow, RIS_OFF) & INT_RESET_DONE, INT_RESET_DONE);
        let mut ow = onewire();
        ow.attach_device(&[0x28]);
        write_word(&mut ow, CS_OFF, CS_RST);
        assert_eq!(read_word(&mut ow, CS_OFF) & CS_NOATR, 0);
        assert_eq!(read_word(&mut ow, RIS_OFF) & INT_NO_SLAVE, 0);
    }

    /// `WR` latches `DATW` and raises `OP_DONE`.
    #[test]
    fn write_flow() {
        let mut ow = onewire();
        ow.attach_device(&[0x00; 8]);
        write_word(&mut ow, DATW_OFF, 0xCC000033);
        write_word(&mut ow, CS_OFF, CS_OP_WR); // SZ=0 → 1 byte.
        assert_eq!(read_word(&mut ow, RIS_OFF) & INT_OP_DONE, INT_OP_DONE);
        assert_eq!(read_word(&mut ow, CS_OFF) & CS_BUSY, 0);
    }

    /// `RD` returns attached bytes; unattached reads idle high.
    #[test]
    fn read_flow() {
        let mut ow = onewire();
        ow.attach_device(&[0x28, 0xFF, 0x4B]);
        write_word(&mut ow, CS_OFF, CS_OP_RD | (2 << CS_SZ_SHIFT)); // 3 bytes.
        assert_eq!(read_word(&mut ow, DATR_OFF), 0x4BFF28);
        let mut ow = onewire();
        write_word(&mut ow, CS_OFF, CS_OP_RD);
        assert_eq!(read_word(&mut ow, DATR_OFF), 0x0000_00FF);
    }

    /// `WRRD` writes then reads; `ICR` W1C clears; `IM` gates.
    #[test]
    fn wrrd_and_gating() {
        let mut ow = onewire();
        ow.attach_device(&[0xBE, 0xEF]);
        write_word(&mut ow, IM_OFF, INT_OP_DONE);
        write_word(&mut ow, DATW_OFF, 0xCC);
        write_word(&mut ow, CS_OFF, CS_OP_WRRD);
        assert_eq!(read_word(&mut ow, DATR_OFF) & 0xFF, 0xBE);
        assert_eq!(ow.pending(), 1);
        write_word(&mut ow, ICR_OFF, INT_OP_DONE);
        assert_eq!(ow.pending(), 0);
    }
}

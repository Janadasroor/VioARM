//! SPDIFRX S/PDIF receiver: sync handshake, staged data, parity errors.
//!
//! `CR.SPDIFRXEN` out of idle arms synchronization (`SR.SYNCD` two ticks
//! later); staged bytes (see [`Spdifrx::inject_frame`]) pop out of `DR`
//! in words with `SR.RXNE` while any remain. `IFCR` clears flags
//! write-1-to-clear; `IMR` gates the single line. Parity errors arrive by
//! [`Spdifrx::inject_error`] (next word latches `PERR`). Biphase-mark
//! decoding, channel/user data (`CSR`/`DIR`), overruns, and DMA stay
//! latches (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

// CR bits.
const EN_MASK: u32 = 0x3;
// SR bits.
const SR_RXNE: u32 = 1 << 0;
const SR_PERR: u32 = 1 << 2;
const SR_SYNCD: u32 = 1 << 5;
// DR / register offsets.
const DR: u32 = 0x10;

#[derive(Clone, Debug, Default)]
struct SpdifrxBehavior {
    sync_ticks: u64,
    /// Receiver locked (flag `SYNCD` reports transitions of this).
    synced: bool,
    staged: std::collections::VecDeque<u8>,
    dr_word: u32,
    dr_lane: u8,
    perr_armed: bool,
}

impl Behavior for SpdifrxBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            // Leaving idle starts the sync countdown; entering idle
            // unlocks the receiver (documented).
            if word & EN_MASK == 0 {
                self.sync_ticks = 0;
                self.synced = false;
                let sr = view.get(0x08) & !(SR_SYNCD | SR_RXNE);
                view.set(0x08, sr);
            } else if view.get(0x00) & EN_MASK == 0 {
                self.sync_ticks = 2;
            }
        }
        if offset == 0x0c {
            // `IFCR`: write-1-to-clear `SR` bits 0-8.
            let sr = view.get(0x08) & !(word & 0x1FF);
            view.set(0x08, sr);
            return view.get(offset);
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.sync_ticks > 0 {
            self.sync_ticks = self.sync_ticks.saturating_sub(delta);
            if self.sync_ticks == 0 {
                self.synced = true;
                view.set(0x08, view.get(0x08) | SR_SYNCD);
            }
        }
        let rxne = self.synced && !self.staged.is_empty();
        let mut sr = view.get(0x08);
        if rxne {
            sr |= SR_RXNE;
        } else {
            sr &= !SR_RXNE;
        }
        view.set(0x08, sr);
    }
    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // Stage one captured byte (tests, debugger inject); capped so a
        // runaway stimulator cannot grow the queue without bound.
        if self.staged.len() < 1 << 20 {
            self.staged.push_back((value & 0xFF) as u8);
        }
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        // `DR` pops one staged word per word-read (lanes share a counter
        // — firmware always reads `DR` as words). An armed parity error
        // latches `PERR` on the popped word instead.
        if offset == DR {
            if self.dr_lane == 0 {
                let mut w = [0u8; 4];
                for b in w.iter_mut() {
                    *b = self.staged.pop_front().unwrap_or(0);
                }
                self.dr_word = u32::from_le_bytes(w);
                if self.perr_armed && (!self.staged.is_empty() || w != [0u8; 4]) {
                    view.set(0x08, view.get(0x08) | SR_PERR);
                    self.perr_armed = false;
                } else if self.perr_armed {
                    self.perr_armed = false;
                }
            }
            let w = self.dr_word;
            self.dr_lane += 1;
            if self.dr_lane == 4 {
                self.dr_lane = 0;
            }
            return w;
        }
        value
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(0x08) & view.get(0x04) & 0x1FF != 0)
    }
    fn reset(&mut self) {
        self.sync_ticks = 0;
        self.perr_armed = false;
    }
}

/// SPDIF receiver.
#[derive(Debug)]
pub struct Spdifrx {
    inner: RegisterFile<SpdifrxBehavior>,
}
impl Spdifrx {
    /// A new `Spdifrx` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SPDIFRX").expect("SPDIFRX");
        Self {
            inner: RegisterFile::new(def, SpdifrxBehavior::default()),
        }
    }

    /// Stage captured bytes: they pop out of `DR` words once synced
    /// (tests, debugger inject).
    pub fn inject_frame(&mut self, bytes: &[u8]) {
        self.inner.behavior_mut().staged.extend(bytes.iter());
    }

    /// Arm a parity error on the next popped word (tests, inject).
    pub fn inject_error(&mut self) {
        self.inner.behavior_mut().perr_armed = true;
    }
}
impl Peripheral for Spdifrx {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, a: u32) -> Result<u8, BusError> {
        self.inner.read(a)
    }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> {
        self.inner.write(a, v)
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
    fn name(&self) -> &'static str {
        "spdifrx"
    }
}
impl InterruptSource for Spdifrx {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, l: u32) {
        self.inner.acknowledge(l);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Spdifrx, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Spdifrx, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn spdifrx_sync_data_and_parity() {
        let mut s = Spdifrx::new_for::<Stm32h743>();
        w(&mut s, 0x04, (1 << 5) | 1); // IMR SYNCDIE RXNEIE
        w(&mut s, 0x00, 1); // CR out of idle
        assert_eq!(r(&mut s, 0x08) & (1 << 5), 0, "not synced yet");
        s.tick(1);
        assert_eq!(r(&mut s, 0x08) & (1 << 5), 0, "one more");
        s.tick(1);
        assert_ne!(r(&mut s, 0x08) & (1 << 5), 0, "SYNCD");
        assert_ne!(s.pending(), 0, "sync line");
        w(&mut s, 0x0c, 1 << 5); // IFCR SYNCDCF
        assert_eq!(s.pending(), 0);
        // Staged bytes pop out of DR with RXNE while any remain.
        s.inject_frame(&[0x11, 0x22, 0x33, 0x44, 0x55]);
        s.tick(1);
        assert_ne!(r(&mut s, 0x08) & 1, 0, "RXNE");
        assert_eq!(r(&mut s, 0x10), 0x4433_2211);
        assert_eq!(r(&mut s, 0x10) & 0xFF, 0x55);
        // Parity error on the next word.
        s.inject_frame(&[0xAA, 0xBB, 0xCC, 0xDD]);
        s.inject_error();
        assert_eq!(r(&mut s, 0x10), 0xDDCC_BBAA);
        assert_ne!(r(&mut s, 0x08) & (1 << 2), 0, "PERR");
    }
}

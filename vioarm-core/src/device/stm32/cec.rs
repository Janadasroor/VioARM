//! CEC — minimal TX handshake, both generations.
//!
//! F0 (`CR`/`TXDR`/`ISR`): `CR.CECEN@0` enable, `TXDR@0x08` write starts TX,
//! `ISR.TXEND` (bit 9) latches next tick, `ISR` write-1-to-clear.
//!
//! F1 (`CFGR`/`TXD`/`CSR`, e.g. value line): `CFGR.PE@0` enable, `TXD@0x14`
//! write stages a byte, `CSR.TSOM@0` starts the transfer, `CSR.TEOM@1`
//! latches next tick with `TSOM` self-clearing; flags clear by plain
//! write (table `ReadWrite`). `CSR` RX-side flags (`RSOM`/`REOM`/`RERR`/
//! `RBTF`), `ESR` errors, and the RX path stay unmodeled on both paths
//! (registers latch); only TX bytes reach the host drain.

use crate::device::svd::{Behavior, Device, PeripheralDef, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_CECEN: u32 = 1 << 0;
const ISR_TXEND: u32 = 1 << 9;
const ISR_TXBR: u32 = 1 << 8;

/// F1 register offsets and `CSR`/`CFGR` bits.
const F1_CFGR: u32 = 0x00;
const F1_TXD: u32 = 0x14;
const F1_CSR: u32 = 0x10;
const F1_PE: u32 = 1 << 0;
const F1_IE: u32 = 1 << 1;
const F1_TSOM: u32 = 1 << 0;
const F1_TEOM: u32 = 1 << 1;
const F1_REOM: u32 = 1 << 5;
const F1_ERRS: u32 = (1 << 2) | (1 << 6);

#[derive(Clone, Debug, Default)]
struct CecBehavior {
    pending_txend: bool,
    /// F1 layout (`TXD` register present).
    f1: bool,
    /// Staged F1 transmit byte + scheduled completion.
    f1_staged: Option<u8>,
    f1_tx_pending: bool,
    /// Transmitted bytes, for host draining.
    tx: Vec<u8>,
}

impl CecBehavior {
    /// Whether `def` carries the F1 register set.
    fn is_f1(def: &PeripheralDef) -> bool {
        def.registers.iter().any(|r| r.name == "TXD")
    }
}

impl Behavior for CecBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if self.f1 {
            if offset == F1_TXD {
                // Only the low lane is the wire byte.
                if shift == 0 {
                    self.f1_staged = Some(val);
                }
            }
            if offset == F1_CSR
                && word & F1_TSOM != 0
                && view.get(F1_CFGR) & F1_PE != 0
                && self.f1_staged.is_some()
            {
                // Fresh transfer: drop any stale end flag, schedule.
                view.set(F1_CSR, view.get(F1_CSR) & !F1_TEOM);
                self.f1_tx_pending = true;
            }
            return word;
        }
        if offset == 0x00 {
            // CR
            if word & CR_CECEN == 0 {
                self.pending_txend = false;
                view.set(0x10, view.get(0x10) & !ISR_TXEND); // ISR at 0x10
            }
        }
        if offset == 0x08 {
            // TXDR at 0x08
            if view.get(0x00) & CR_CECEN != 0 {
                self.pending_txend = true;
            }
        }
        if offset == 0x10 {
            // ISR at 0x10: W1C
            if word & ISR_TXEND != 0 {
                // Write 1 to clear TXEND
                let cur = view.get(0x10) & !ISR_TXEND;
                view.set(0x10, cur);
                self.pending_txend = false;
                return cur;
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.f1 {
            if self.f1_tx_pending {
                if let Some(byte) = self.f1_staged.take() {
                    self.tx.push(byte);
                }
                let csr = view.get(F1_CSR);
                view.set(F1_CSR, (csr | F1_TEOM) & !F1_TSOM);
                self.f1_tx_pending = false;
            }
            return;
        }
        if self.pending_txend {
            let isr = view.get(0x10);
            view.set(0x10, isr | ISR_TXEND | ISR_TXBR);
            self.pending_txend = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        if self.f1 {
            let csr = view.get(F1_CSR);
            let cfgr = view.get(F1_CFGR);
            return u32::from(cfgr & F1_IE != 0 && csr & (F1_TEOM | F1_REOM | F1_ERRS) != 0);
        }
        let isr = view.get(0x10);
        let ier = view.get(0x14); // IER at 0x14, TXENDIE at 9, TXBRIE at 8
        u32::from(isr & ISR_TXEND != 0 && ier & (1 << 9) != 0)
    }
    fn reset(&mut self) {
        self.pending_txend = false;
        self.f1_staged = None;
        self.f1_tx_pending = false;
        self.tx.clear();
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx)
    }
}

/// Consumer Electronics Control.
#[derive(Debug)]
pub struct Cec {
    inner: RegisterFile<CecBehavior>,
}
impl Cec {
    /// A new `Cec` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CEC").expect("CEC");
        let f1 = CecBehavior::is_f1(def);
        Self {
            inner: RegisterFile::new(
                def,
                CecBehavior {
                    f1,
                    ..Default::default()
                },
            ),
        }
    }
}
impl Peripheral for Cec {
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
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn name(&self) -> &'static str {
        "cec"
    }
    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}
impl InterruptSource for Cec {
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
    use crate::device::svd::stm32f091::Stm32f091;
    fn w(c: &mut Cec, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Cec, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    #[test]
    fn cec_txend_latches() {
        let mut c = Cec::new_for::<Stm32f091>();
        w(&mut c, 0x14, 1 << 9); // IER TXENDIE at 0x14
        w(&mut c, 0x00, CR_CECEN);
        w(&mut c, 0x08, 0x55); // TXDR at 0x08
        assert_eq!(c.pending(), 0);
        c.tick(1);
        assert_ne!(r(&mut c, 0x10) & ISR_TXEND, 0);
        assert_eq!(c.pending(), 1);
        w(&mut c, 0x10, ISR_TXEND); // ISR W1C at 0x10
        assert_eq!(r(&mut c, 0x10) & ISR_TXEND, 0);
    }

    // --- F1 layout (value line) ---

    use crate::device::svd::stm32f100::Stm32f100;

    #[test]
    fn f1_tx_handshake() {
        let mut c = Cec::new_for::<Stm32f100>();
        assert!(c.inner.behavior().f1, "F1 tables select F1");
        w(&mut c, 0x00, F1_PE | F1_IE); // CFGR
        w(&mut c, 0x14, 0x55); // TXD stages
        assert_eq!(r(&mut c, 0x10) & F1_TEOM, 0, "nothing yet");
        w(&mut c, 0x10, F1_TSOM); // CSR starts the transfer
        assert_eq!(c.pending(), 0, "no flag yet");
        c.tick(1);
        let csr = r(&mut c, 0x10);
        assert_ne!(csr & F1_TEOM, 0, "TEOM latches");
        assert_eq!(csr & F1_TSOM, 0, "TSOM self-clears");
        assert_eq!(c.pending(), 1, "IE-gated line");
        assert_eq!(c.drain_output(), [0x55], "byte drains");
        // Plain write clears the end flag.
        w(&mut c, 0x10, 0);
        assert_eq!(r(&mut c, 0x10) & F1_TEOM, 0);
        assert_eq!(c.pending(), 0);
    }

    #[test]
    fn f1_needs_enable_and_byte() {
        let mut c = Cec::new_for::<Stm32f100>();
        // TSOM with no staged byte: nothing scheduled.
        w(&mut c, 0x10, F1_TSOM);
        c.tick(1);
        assert_eq!(r(&mut c, 0x10) & F1_TEOM, 0);
        // Staged byte but PE clear: still nothing.
        w(&mut c, 0x14, 0x55);
        w(&mut c, 0x10, F1_TSOM);
        c.tick(1);
        assert_eq!(r(&mut c, 0x10) & F1_TEOM, 0);
    }
}

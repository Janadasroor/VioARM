//! UCPD USB Type-C / Power-Delivery controller (G4 `UCPD1@0x4000a000`).
//!
//! Models the PD message engine: firmware stages `TX_ORDSET`, `TX_PAYSZ`
//! and payload bytes in `TXDR`, then sets `CR.TXSEND` (self-clearing) and
//! the frame transmits after [`SEND_TICKS`] ticks, latching `SR.TXMSGSENT`;
//! the transmitted image (ordered-set word plus payload, little-endian)
//! waits in [`Peripheral::drain_output`]. `CR.TXHRST` (also self-clearing)
//! sends a Hard Reset and latches `SR.HRSTSENT`. `ICR` clears every flag
//! write-1-to-clear.
//!
//! The transmitter mirrors silicon flow control: when fewer than `TXPAYSZ`
//! bytes are staged at `TXSEND`, `SR.TXIS` asserts and the engine waits up
//! to [`TOPUP_DEADLINE`] ticks for firmware to top up `TXDR`; each `TXDR`
//! byte clears `TXIS` once the declared count is reached, while starving
//! the deadline latches `SR.TXUND` + `SR.TXMSGDISC`. `TXSEND` with the CC
//! block disabled (`CR.CCENABLE == 0`) or while a frame is in flight
//! latches `SR.TXMSGDISC` instead of transmitting.
//!
//! Reception arrives through [`Peripheral::drive_inputs`], one 32-bit word
//! per call, in frame order: ordered set, payload size in bytes, then
//! payload words. The engine latches `RX_ORDSET`/`RX_PAYSZ`, raises
//! `SR.RXORDDET`, streams payload bytes out of `RXDR` (`SR.RXNE` while
//! bytes remain), and latches `SR.RXMSGEND` once the declared count is
//! received. A new frame while the previous one is unread or incomplete
//! latches `SR.RXOVR`. Reception is gated on `CR.PHYRXEN` with the CC
//! block enabled; anything else is ignored.
//!
//! The Type-C analog state machine stays out of scope: `CFG1`/`CFG2`, the
//! `CR` mode bits (`TXMODE`, `RXMODE`, `PHYCCSEL`, `ANASUBMODE`,
//! `ANAMODE`, `CCENABLE`, `FRSRXEN`, `FRSTX`, `RDCH`), `RX_ORDEXT1/2`,
//! `RXERR`, `RXHRSTDET`, and the `TYPEC_*`/`FRSEVT` flags latch (their
//! interrupt-clear bits work) but no line-state engine runs behind them.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR: u32 = 0x0C;
const IMR: u32 = 0x10;
const SR: u32 = 0x14;
const ICR: u32 = 0x18;
const TX_ORDSET: u32 = 0x1C;
const TX_PAYSZ: u32 = 0x20;
const TXDR: u32 = 0x24;
const RX_ORDSET: u32 = 0x28;
const RX_PAYSZ: u32 = 0x2C;
const RXDR: u32 = 0x30;

const TXSEND: u32 = 1 << 2;
const TXHRST: u32 = 1 << 3;
const PHYRXEN: u32 = 1 << 5;
const CCENABLE: u32 = 0b11 << 10;

const TXIS: u32 = 1 << 0;
const TXMSGDISC: u32 = 1 << 1;
const TXMSGSENT: u32 = 1 << 2;
const HRSTDISC: u32 = 1 << 4;
const HRSTSENT: u32 = 1 << 5;
const TXUND: u32 = 1 << 6;
const RXNE: u32 = 1 << 8;
const RXORDDET: u32 = 1 << 9;
const RXOVR: u32 = 1 << 11;
const RXMSGEND: u32 = 1 << 12;

/// Ticks from `TXSEND`/top-up-complete to `TXMSGSENT`.
const SEND_TICKS: u64 = 4;
/// Ticks `TXIS` waits for firmware to top up `TXDR` before underrun.
const TOPUP_DEADLINE: u64 = 16;
/// Ticks from `TXHRST` to `HRSTSENT`.
const HRST_TICKS: u64 = 4;
/// Staging cap: longest PD frame is far shorter; excess bytes drop.
const STAGE_CAP: usize = 512;

#[derive(Clone, Debug, Default)]
struct UcpdBehavior {
    tx_stage: Vec<u8>,
    send_at: Option<u64>,
    topup_at: Option<u64>,
    hrst_at: Option<u64>,
    rx_state: RxState,
    rx_fifo: Vec<u8>,
    rx_word: u32,
    rx_lane: u8,
    output: Vec<u8>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RxState {
    #[default]
    Idle,
    OrdsetSeen,
    InPayload {
        remaining: u32,
    },
}

impl UcpdBehavior {
    fn set_sr(view: &mut Registers<&mut [u32]>, bits: u32) {
        view.set(SR, view.get(SR) | bits);
    }

    fn clear_sr(view: &mut Registers<&mut [u32]>, bits: u32) {
        view.set(SR, view.get(SR) & !bits);
    }

    fn cc_on(view: &mut Registers<&mut [u32]>) -> bool {
        view.get(CR) & CCENABLE != 0
    }

    fn paysz(view: &mut Registers<&mut [u32]>) -> usize {
        (view.get(TX_PAYSZ) & 0x3FF) as usize
    }

    fn start_send(&mut self, view: &mut Registers<&mut [u32]>) {
        if self.tx_stage.len() >= Self::paysz(view) {
            Self::clear_sr(view, TXIS);
            self.topup_at = None;
            self.send_at = Some(SEND_TICKS);
        } else {
            Self::set_sr(view, TXIS);
            self.topup_at = Some(TOPUP_DEADLINE);
        }
    }

    fn start_tx(&mut self, view: &mut Registers<&mut [u32]>) {
        if !Self::cc_on(view) || self.send_at.is_some() || self.topup_at.is_some() {
            // Disabled CC block, or a frame already in flight: the
            // newcomer is discarded.
            Self::set_sr(view, TXMSGDISC);
            self.send_at = None;
            self.topup_at = None;
            Self::clear_sr(view, TXIS);
            return;
        }
        Self::clear_sr(view, TXMSGSENT | TXMSGDISC);
        self.start_send(view);
    }

    fn feed_txdr(&mut self, view: &mut Registers<&mut [u32]>, byte: u8) {
        if self.tx_stage.len() < STAGE_CAP {
            self.tx_stage.push(byte);
        }
        if self.topup_at.is_some() && self.tx_stage.len() >= Self::paysz(view) {
            self.start_send(view);
        }
    }

    fn complete_send(&mut self, view: &mut Registers<&mut [u32]>) {
        let n = Self::paysz(view).min(self.tx_stage.len());
        let payload: Vec<u8> = self.tx_stage.drain(..n).collect();
        self.tx_stage.clear();
        self.output
            .extend_from_slice(&view.get(TX_ORDSET).to_le_bytes());
        self.output.extend_from_slice(&payload);
        Self::clear_sr(view, TXIS);
        Self::set_sr(view, TXMSGSENT);
    }

    fn inject_rx_word(&mut self, view: &mut Registers<&mut [u32]>, word: u32) {
        // `CCENABLE` is a 2-bit field (01 = CC1, 10 = CC2, 11 = both):
        // any nonzero selection with the PHY receiver on gates RX.
        if view.get(CR) & PHYRXEN == 0 || view.get(CR) & CCENABLE == 0 {
            return;
        }
        match self.rx_state {
            RxState::Idle => {
                if view.get(SR) & RXMSGEND != 0 {
                    Self::set_sr(view, RXOVR);
                }
                view.set(RX_ORDSET, word);
                Self::set_sr(view, RXORDDET);
                self.rx_state = RxState::OrdsetSeen;
            }
            RxState::OrdsetSeen => {
                let paysz = word & 0x3FF;
                view.set(RX_PAYSZ, paysz);
                self.rx_fifo.clear();
                if paysz == 0 {
                    Self::set_sr(view, RXMSGEND);
                    self.rx_state = RxState::Idle;
                } else {
                    self.rx_state = RxState::InPayload { remaining: paysz };
                }
            }
            RxState::InPayload { mut remaining } => {
                for byte in word.to_le_bytes() {
                    if remaining == 0 {
                        break;
                    }
                    if self.rx_fifo.len() < STAGE_CAP {
                        self.rx_fifo.push(byte);
                    }
                    remaining -= 1;
                }
                if remaining == 0 {
                    Self::set_sr(view, RXNE | RXMSGEND);
                    self.rx_state = RxState::Idle;
                } else {
                    Self::set_sr(view, RXNE);
                    self.rx_state = RxState::InPayload { remaining };
                }
            }
        }
    }
}

impl Behavior for UcpdBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            ICR => {
                // Write-1-to-clear into SR.
                let new = view.merged(ICR, val, shift);
                Self::clear_sr(view, new);
                new
            }
            TXDR => {
                let new = view.merged(TXDR, val, shift);
                self.feed_txdr(view, val);
                new
            }
            CR => {
                let new = view.merged(CR, val, shift);
                if new & TXSEND != 0 {
                    self.start_tx(view);
                }
                if new & TXHRST != 0 {
                    if Self::cc_on(view) {
                        Self::clear_sr(view, HRSTSENT | HRSTDISC);
                        self.hrst_at = Some(HRST_TICKS);
                    } else {
                        Self::set_sr(view, HRSTDISC);
                    }
                }
                // Command bits are self-clearing.
                new & !(TXSEND | TXHRST)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == RXDR {
            // One pop per word read (lanes share a counter, like DCMI
            // `DR` — firmware always reads data registers as words).
            if self.rx_lane == 0 {
                let mut w = [0u8; 4];
                for b in w.iter_mut() {
                    if self.rx_fifo.is_empty() {
                        break;
                    }
                    *b = self.rx_fifo.remove(0);
                }
                self.rx_word = u32::from_le_bytes(w);
                if self.rx_fifo.is_empty() {
                    Self::clear_sr(view, RXNE);
                }
            }
            let w = self.rx_word;
            self.rx_lane = (self.rx_lane + 1) % 4;
            return w;
        }
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if let Some(left) = self.send_at {
            let left = left.saturating_sub(delta);
            self.send_at = Some(left);
            if left == 0 {
                self.send_at = None;
                self.complete_send(view);
            }
        }
        if let Some(left) = self.topup_at {
            let left = left.saturating_sub(delta);
            self.topup_at = Some(left);
            if left == 0 {
                self.topup_at = None;
                Self::clear_sr(view, TXIS);
                Self::set_sr(view, TXUND | TXMSGDISC);
            }
        }
        if let Some(left) = self.hrst_at {
            let left = left.saturating_sub(delta);
            self.hrst_at = Some(left);
            if left == 0 {
                self.hrst_at = None;
                Self::set_sr(view, HRSTSENT);
            }
        }
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        self.inject_rx_word(view, value);
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(SR) & view.get(IMR) != 0)
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }

    fn reset(&mut self) {
        self.tx_stage.clear();
        self.send_at = None;
        self.topup_at = None;
        self.hrst_at = None;
        self.rx_state = RxState::Idle;
        self.rx_fifo.clear();
        self.rx_word = 0;
        self.rx_lane = 0;
        self.output.clear();
    }
}

/// UCPD USB Type-C / Power-Delivery controller.
#[derive(Debug)]
pub struct Ucpd {
    inner: RegisterFile<UcpdBehavior>,
}

impl Ucpd {
    /// A new `UCPD` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `UCPD1`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("UCPD1").expect("generated tables contain UCPD1");
        Self {
            inner: RegisterFile::new(def, UcpdBehavior::default()),
        }
    }
}

impl Peripheral for Ucpd {
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

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        "ucpd"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Ucpd {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32g474xx::Stm32g474;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4000_A000;
    const CR_O: u32 = 0x0C;
    const IMR_O: u32 = 0x10;
    const SR_O: u32 = 0x14;
    const ICR_O: u32 = 0x18;
    const TX_ORDSET_O: u32 = 0x1C;
    const TX_PAYSZ_O: u32 = 0x20;
    const TXDR_O: u32 = 0x24;
    const RX_ORDSET_O: u32 = 0x28;
    const RX_PAYSZ_O: u32 = 0x2C;
    const RXDR_O: u32 = 0x30;

    const CC1: u32 = 1 << 10;
    const PHYRXEN_V: u32 = 1 << 5;
    const TXSEND_V: u32 = 1 << 2;
    const TXHRST_V: u32 = 1 << 3;
    const TXISIE: u32 = 1 << 0;
    const TXMSGSENTIE: u32 = 1 << 2;
    const RXMSGENDIE: u32 = 1 << 12;
    const RXORDDETIE: u32 = 1 << 9;

    #[allow(clippy::cast_possible_truncation)]
    fn w(u: &mut Ucpd, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(u: &mut Ucpd, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn ucpd() -> Ucpd {
        let mut u = Ucpd::new_for::<Stm32g474>();
        u.reset();
        u
    }

    #[test]
    fn tx_control_message_sends_and_drains() {
        let mut u = ucpd();
        w(&mut u, IMR_O, TXMSGSENTIE);
        w(&mut u, CR_O, CC1);
        w(&mut u, TX_ORDSET_O, 0x0_1111);
        w(&mut u, TX_PAYSZ_O, 2);
        w(&mut u, TXDR_O, 0xC0FF_EE00);
        w(&mut u, CR_O, CC1 | TXSEND_V);
        assert_eq!(r(&mut u, CR_O) & TXSEND_V, 0, "TXSEND self-clears");
        u.tick(10);
        assert_ne!(r(&mut u, SR_O) & TXMSGSENT, 0, "TXMSGSENT latches");
        assert_eq!(u.pending(), 1, "line raises");
        let out = u.drain_output();
        assert_eq!(
            out,
            [0x11, 0x11, 0x00, 0x00, 0x00, 0xEE],
            "ordset LE + first paysz bytes"
        );
        w(&mut u, ICR_O, TXMSGSENT);
        assert_eq!(r(&mut u, SR_O) & TXMSGSENT, 0, "ICR clears");
        assert_eq!(u.pending(), 0, "line drops");
    }

    #[test]
    fn tx_topup_through_txis() {
        let mut u = ucpd();
        w(&mut u, IMR_O, TXISIE | TXMSGSENTIE);
        w(&mut u, CR_O, CC1);
        w(&mut u, TX_PAYSZ_O, 6);
        w(&mut u, TXDR_O, 0x0000_BEEF);
        w(&mut u, CR_O, CC1 | TXSEND_V);
        assert_ne!(r(&mut u, SR_O) & TXIS, 0, "TXIS asks for more");
        // Four bytes staged so far; feed the rest of the declared six.
        w(&mut u, TXDR_O, 0x0000_AAAA);
        assert_eq!(r(&mut u, SR_O) & TXIS, 0, "TXIS clears once fed");
        u.tick(10);
        assert_ne!(r(&mut u, SR_O) & TXMSGSENT, 0, "frame sends");
    }

    #[test]
    fn tx_starved_deadline_underruns() {
        let mut u = ucpd();
        w(&mut u, CR_O, CC1);
        w(&mut u, TX_PAYSZ_O, 6);
        w(&mut u, TXDR_O, 0x0000_BEEF);
        w(&mut u, CR_O, CC1 | TXSEND_V);
        u.tick(100);
        let sr = r(&mut u, SR_O);
        assert_ne!(sr & TXUND, 0, "TXUND latches");
        assert_ne!(sr & TXMSGDISC, 0, "frame discarded");
        assert_eq!(sr & TXMSGSENT, 0, "nothing sent");
    }

    #[test]
    fn tx_gated_without_cc_block() {
        let mut u = ucpd();
        w(&mut u, TX_PAYSZ_O, 2);
        w(&mut u, TXDR_O, 0xC0FF_EE00);
        w(&mut u, CR_O, TXSEND_V);
        u.tick(10);
        let sr = r(&mut u, SR_O);
        assert_ne!(sr & TXMSGDISC, 0, "discarded with CC off");
        assert_eq!(sr & TXMSGSENT, 0, "nothing sent");
        assert!(u.drain_output().is_empty(), "no bytes out");
    }

    #[test]
    fn tx_hrst_completes() {
        let mut u = ucpd();
        w(&mut u, CR_O, CC1);
        w(&mut u, CR_O, CC1 | TXHRST_V);
        assert_eq!(r(&mut u, CR_O) & TXHRST_V, 0, "TXHRST self-clears");
        u.tick(10);
        assert_ne!(r(&mut u, SR_O) & HRSTSENT, 0, "HRSTSENT latches");
    }

    #[test]
    fn rx_frame_stages_and_reads() {
        let mut u = ucpd();
        w(&mut u, IMR_O, RXORDDETIE | RXMSGENDIE);
        w(&mut u, CR_O, CC1 | PHYRXEN_V);
        u.drive_inputs(0x0_2222);
        assert_ne!(r(&mut u, SR_O) & RXORDDET, 0, "RXORDDET latches");
        assert_eq!(r(&mut u, RX_ORDSET_O), 0x2222, "ordset latched");
        u.drive_inputs(4);
        assert_eq!(r(&mut u, RX_PAYSZ_O), 4, "paysz latched");
        u.drive_inputs(0xDEAD_BEEF);
        let sr = r(&mut u, SR_O);
        assert_ne!(sr & RXMSGEND, 0, "RXMSGEND latches");
        assert_ne!(sr & RXNE, 0, "RXNE set");
        assert_eq!(u.pending(), 1, "line raises");
        let mut payload = [0u8; 4];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = u.read(BASE + RXDR_O + i as u32).unwrap();
        }
        assert_eq!(payload, [0xEF, 0xBE, 0xAD, 0xDE], "payload LE");
        assert_eq!(r(&mut u, SR_O) & RXNE, 0, "RXNE drains");
        w(&mut u, ICR_O, RXORDDET | RXMSGEND);
        assert_eq!(u.pending(), 0, "line drops");
    }

    #[test]
    fn rx_overrun_on_unread_frame() {
        let mut u = ucpd();
        w(&mut u, CR_O, CC1 | PHYRXEN_V);
        u.drive_inputs(0x1111);
        u.drive_inputs(0); // zero-length frame: RXMSGEND immediately
        assert_ne!(r(&mut u, SR_O) & RXMSGEND, 0, "first frame ends");
        u.drive_inputs(0x2222); // newcomer before firmware clears
        assert_ne!(r(&mut u, SR_O) & RXOVR, 0, "RXOVR latches");
        assert_eq!(r(&mut u, RX_ORDSET_O), 0x2222, "new ordset wins");
    }

    #[test]
    fn rx_gated_without_phy() {
        let mut u = ucpd();
        w(&mut u, CR_O, CC1); // PHYRXEN clear
        u.drive_inputs(0x1111);
        assert_eq!(r(&mut u, SR_O) & RXORDDET, 0, "ignored with PHY off");
    }
}

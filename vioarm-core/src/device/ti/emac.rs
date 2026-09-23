//! TI Tiva C Ethernet MAC (`EMAC0`, `0x400E_C000`).
//!
//! Register map and MII/descriptor semantics follow the TI `hw_emac.h` +
//! `emac.c`/`lwiplib` oracle (Synopsys DWC core, BSD): `MIIADDR` `MIIB`/
//! `MIIW`/`CR`/`MII`/`PLA`, `MIIDATA`, `CFG` `RE`/`TE`, enhanced
//! descriptors (`TDES0`: `OWN` 31/`IC` 30/`LS` 29/`FS` 28/`TCH` 20/
//! `TER` 21; `TDES1`: `TBS1` 12:0/`TBS2` 28:16; `TDES2/3` buffer/next;
//! `RDES0`: `OWN`/`ES` 15/`FL` 29:16/`FS` 9/`LS` 8; `RDES1`: `RBS1`
//! 12:0/`RCH` 14/`RER` 15/`DIC` 31), `DMAOPMODE` `ST`/`SR`,
//! `DMARIS` `TI`/`RI`/`NIS`. The SVD carries no `<resetValue>`, so all
//! words reset 0.
//!
//! Modeled: instant `MIIB` completion (no PHY — `MIIDATA` reads
//! all-ones), config/address latches, and descriptor-ring DMA through
//! the shared [`EthMailbox`] (same bus coupling as the H7 model):
//! TX reads descriptors + buffers on `TXPOLLD`/store-forward, clears
//! `OWN`, latches `TI`; RX stores injected frames into owned
//! descriptors with `FS`/`LS`/`FL` (length as delivered — CRC bytes are
//! not appended), latches `RI`; `DMARIS` W1C, `NIS` synthesis.
//! Deliberate scope: checksum offload, MMC counting, PTP time,
//! magic-packet wake, second-buffer (`TBS2`/`RBS2`) scatter, DMA burst
//! arbitration (instant, unpaced).

use crate::device::stm32::eth::{EthMailbox, EthMemOp};
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `EMAC0` base address.
pub const EMAC_BASE: u32 = 0x400E_C000;

const MIIADDR_OFF: u32 = 0x10;
const MIIDATA_OFF: u32 = 0x14;
const DMAOPMODE_OFF: u32 = 0xC18;
const TXPOLLD_OFF: u32 = 0xC04;
const RXPOLLD_OFF: u32 = 0xC08;
const RXDLADDR_OFF: u32 = 0xC0C;
const TXDLADDR_OFF: u32 = 0xC10;
const DMARIS_OFF: u32 = 0xC14;

/// `MIIADDR.MIIB` (busy) — self-clearing on instant completion.
const MII_MIIB: u32 = 1 << 0;
/// `DMAOPMODE` engine enables.
const OP_ST: u32 = 1 << 13;
const OP_SR: u32 = 1 << 1;
/// Descriptor bits.
const OWN: u32 = 1 << 31;
const TDES0_FS: u32 = 1 << 28;
const TDES0_LS: u32 = 1 << 29;
const TDES0_TCH: u32 = 1 << 20;
const TDES0_TER: u32 = 1 << 21;
const TBS1_MASK: u32 = 0x1FFF;
const RDES1_RCH: u32 = 1 << 14;
const RDES1_RER: u32 = 1 << 15;
const RBS1_MASK: u32 = 0x1FFF;
const RDES0_FS: u32 = 1 << 9;
const RDES0_LS: u32 = 1 << 8;
/// `DMARIS` bits.
const DMA_TI: u32 = 1 << 0;
const DMA_RI: u32 = 1 << 6;
const DMA_NIS: u32 = 1 << 16;

/// DMA phase: one mailbox round-trip per tick.
#[derive(Debug, Default, PartialEq, Eq)]
enum Phase {
    #[default]
    Idle,
    AwaitTxDesc,
    AwaitTxData {
        desc: u32,
        t0: u32,
        next: u32,
    },
    AwaitRxDesc,
}

/// Behavior behind the MAC window: shell registers plus the
/// descriptor-ring DMA engine.
#[derive(Debug)]
pub struct EmacBehavior {
    mailbox: Arc<Mutex<EthMailbox>>,
    tx_desc: u32,
    rx_desc: u32,
    tx_init: bool,
    rx_init: bool,
    tx_poll: bool,
    tx_accum: Vec<u8>,
    /// Frames firmware transmitted (test/closeout introspection).
    tx_frames: Vec<Vec<u8>>,
    /// Host-injected receive frame.
    rx_pending: Option<Vec<u8>>,
    dmaris: u32,
    phase: Phase,
}

impl EmacBehavior {
    /// Queue a descriptor/buffer read; bytes land in `results`.
    fn push_read(&self, addr: u32, len: u16) {
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.push(EthMemOp::Read { addr, len });
        }
    }

    /// Queue a memory write.
    fn push_write(&self, addr: u32, bytes: Vec<u8>) {
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.push(EthMemOp::Write { addr, bytes });
        }
    }

    /// Consume one landed mailbox result, if any.
    fn pop_result(&self) -> Option<Vec<u8>> {
        self.mailbox
            .lock()
            .ok()
            .and_then(|mut mb| mb.results.pop_front())
    }

    fn tx_enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(DMAOPMODE_OFF) & OP_ST != 0
    }

    fn rx_enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(DMAOPMODE_OFF) & OP_SR != 0
    }

    /// Consume a fetched TX descriptor: read its buffer next.
    fn consume_tx_desc(&mut self, view: &mut Registers<&mut [u32]>, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (t0, t1, buf, t3) = (w(0), w(1), w(2), w(3));
        if t0 & OWN == 0 {
            // Nothing owned by the DMA; the poll was spurious.
            self.phase = Phase::Idle;
            return;
        }
        let tbs1 = (t1 & TBS1_MASK) as u16;
        let desc = self.tx_desc;
        let mut next = if t0 & TDES0_TCH != 0 {
            t3
        } else {
            desc.wrapping_add(16)
        };
        if t0 & TDES0_TER != 0 {
            next = view.get(TXDLADDR_OFF);
        }
        self.push_read(buf, tbs1);
        self.phase = Phase::AwaitTxData { desc, t0, next };
    }

    /// Complete a TX buffer read: frame bytes out, ownership home.
    fn complete_tx(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        data: &[u8],
        desc: u32,
        t0: u32,
        next: u32,
    ) {
        self.push_write(desc, (t0 & !OWN).to_le_bytes().to_vec());
        self.tx_desc = next;
        self.tx_accum.extend_from_slice(data);
        if t0 & TDES0_LS != 0 {
            self.tx_frames.push(core::mem::take(&mut self.tx_accum));
            self.dmaris |= DMA_TI;
        }
        let _ = view;
        self.phase = Phase::Idle;
    }

    /// Advance the RX ring: store the pending frame, write status back.
    fn rx_advance(&mut self, view: &mut Registers<&mut [u32]>, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (r0, r1, buf, r3) = (w(0), w(1), w(2), w(3));
        if r0 & OWN == 0 {
            // Not owned: drop rather than overwrite firmware's data
            // (the ISR resumes the ring with a poll).
            self.rx_pending = None;
            self.phase = Phase::Idle;
            return;
        }
        if let Some(frame) = self.rx_pending.take() {
            let rbs1 = (r1 & RBS1_MASK) as usize;
            let n = frame.len().min(rbs1);
            if n > 0 {
                self.push_write(buf, frame[..n].to_vec());
            }
            // Status: ownership home, first+last, length as delivered
            // (CRC bytes are not appended by this model).
            let fl = ((frame.len() as u32) & 0x3FFF) << 16;
            self.push_write(
                self.rx_desc,
                (fl | RDES0_FS | RDES0_LS).to_le_bytes().to_vec(),
            );
            let mut next = if r1 & RDES1_RCH != 0 {
                r3
            } else {
                self.rx_desc.wrapping_add(16)
            };
            if r1 & RDES1_RER != 0 {
                next = view.get(RXDLADDR_OFF);
            }
            self.rx_desc = next;
            self.dmaris |= DMA_RI;
        }
        self.phase = Phase::Idle;
    }
}

impl Behavior for EmacBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn reset(&mut self) {
        self.tx_desc = 0;
        self.rx_desc = 0;
        self.tx_init = false;
        self.rx_init = false;
        self.tx_poll = false;
        self.tx_accum.clear();
        self.rx_pending = None;
        self.dmaris = 0;
        self.phase = Phase::Idle;
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.clear();
            mb.results.clear();
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let _ = view;
        u32::from(self.dmaris & (DMA_TI | DMA_RI) != 0)
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Consume one landed result per tick (bus drains after ticks).
        if self.phase != Phase::Idle {
            if let Some(result) = self.pop_result() {
                match core::mem::replace(&mut self.phase, Phase::Idle) {
                    Phase::AwaitRxDesc => self.rx_advance(view, &result),
                    Phase::AwaitTxDesc => self.consume_tx_desc(view, &result),
                    Phase::AwaitTxData { desc, t0, next } => {
                        self.complete_tx(view, &result, desc, t0, next)
                    }
                    Phase::Idle => {}
                }
            }
            return;
        }
        // Nothing in flight: start new work.
        if Self::rx_enabled(view) && self.rx_pending.is_some() {
            if !self.rx_init {
                self.rx_desc = view.get(RXDLADDR_OFF);
                self.rx_init = true;
            }
            self.push_read(self.rx_desc, 16);
            self.phase = Phase::AwaitRxDesc;
            return;
        }
        if Self::tx_enabled(view) && self.tx_poll {
            self.tx_poll = false;
            if !self.tx_init {
                self.tx_desc = view.get(TXDLADDR_OFF);
                self.tx_init = true;
            }
            self.push_read(self.tx_desc, 16);
            self.phase = Phase::AwaitTxDesc;
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            MIIADDR_OFF => value & !MII_MIIB,
            MIIDATA_OFF => 0xFFFF, // No PHY: MDIO floats high.
            0x24 => 0,             // Engines idle between bursts.
            0x38 => 0,             // MAC `RIS`: no events without traffic.
            DMARIS_OFF => {
                let mut d = self.dmaris;
                if d & (DMA_TI | DMA_RI) != 0 {
                    d |= DMA_NIS;
                }
                d
            }
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
        if offset == 0x38 {
            return view.get(offset);
        }
        if offset == DMARIS_OFF {
            // W1C status (only whole-word stores clear, like `FCMISC`).
            if shift == 24 {
                let word = view.merged(offset, val, shift);
                self.dmaris &= !word;
            }
            return self.dmaris;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if offset == TXPOLLD_OFF && shift == 24 {
            self.tx_poll = true; // Resume-poll: any word store wakes TX.
        }
        word
    }
}

/// A Tiva C Ethernet MAC (shell + descriptor-ring DMA).
pub struct Emac {
    inner: RegisterFile<EmacBehavior>,
}

impl Emac {
    /// A new MAC on `D`'s tables (`"EMAC0"`), sharing the Ethernet
    /// mailbox with the memory bus for descriptor/frame DMA.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"EMAC0"`.
    #[must_use]
    pub fn for_device<D: Device>(mailbox: Arc<Mutex<EthMailbox>>) -> Self {
        let def = D::peripheral("EMAC0").expect("generated tables contain EMAC0");
        Self {
            inner: RegisterFile::new(
                def,
                EmacBehavior {
                    mailbox,
                    tx_desc: 0,
                    rx_desc: 0,
                    tx_init: false,
                    rx_init: false,
                    tx_poll: false,
                    tx_accum: Vec::new(),
                    tx_frames: Vec::new(),
                    rx_pending: None,
                    dmaris: 0,
                    phase: Phase::Idle,
                },
            ),
        }
    }

    /// Queue a received frame (host stimulus).
    pub fn inject_frame(&mut self, bytes: &[u8]) {
        self.inner.with_view(|_, b| {
            b.rx_pending = Some(bytes.to_vec());
        });
    }

    /// Frames firmware transmitted since the last drain.
    #[must_use]
    pub fn drain_frames(&mut self) -> Vec<Vec<u8>> {
        self.inner
            .with_view(|_, b| core::mem::take(&mut b.tx_frames))
    }
}

impl Peripheral for Emac {
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
        // Clear the shared mailbox in place (the board holds the twin).
        self.inner.with_view(|_, b| {
            if let Ok(mut mb) = b.mailbox.lock() {
                mb.ops.clear();
                mb.results.clear();
            }
        });
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Emac {
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

    const EMAC0_BASE: u32 = 0x400E_C000;
    const CFG_OFF: u32 = 0x00;
    const ADDR0L_OFF: u32 = 0x44;

    fn emac() -> (Emac, Arc<Mutex<EthMailbox>>) {
        let mb = Arc::new(Mutex::new(EthMailbox::default()));
        (Emac::for_device::<Tm4c129xnczad>(mb.clone()), mb)
    }

    fn read_word(emac: &mut Emac, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(emac.read(EMAC0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(emac: &mut Emac, offset: u32, value: u32) {
        for lane in 0..4 {
            emac.write(
                EMAC0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Seed one landed mailbox result (simulates the bus drain).
    fn seed_result(mb: &Arc<Mutex<EthMailbox>>, bytes: Vec<u8>) {
        mb.lock().unwrap().results.push_back(bytes);
    }

    fn word_le(w: u32) -> Vec<u8> {
        w.to_le_bytes().to_vec()
    }

    /// `CFG` + MAC address latches; engines report idle and quiet.
    #[test]
    fn config_latches_and_idle() {
        let (mut emac, _) = emac();
        write_word(&mut emac, CFG_OFF, (1 << 2) | (1 << 3)); // RE + TE.
        assert_eq!(read_word(&mut emac, CFG_OFF) & 0x0C, 0x0C);
        write_word(&mut emac, ADDR0L_OFF, 0x1234_5678);
        assert_eq!(read_word(&mut emac, ADDR0L_OFF), 0x1234_5678);
        assert_eq!(read_word(&mut emac, 0x24), 0);
        assert_eq!(read_word(&mut emac, 0x38), 0);
        assert_eq!(emac.pending(), 0);
    }

    /// MII completes instantly; no PHY reads all-ones.
    #[test]
    fn mii_completes() {
        let (mut emac, _) = emac();
        write_word(&mut emac, MIIADDR_OFF, 0x01); // MIIB + PHY 0/reg 0.
        assert_eq!(read_word(&mut emac, MIIADDR_OFF) & MII_MIIB, 0);
        assert_eq!(read_word(&mut emac, MIIDATA_OFF), 0xFFFF);
    }

    /// TX: descriptor → buffer → ownership home + `TI`, frame observed.
    #[test]
    fn tx_single_desc_flow() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1); // Queues the descriptor read.
                      // Descriptor: OWN|LS|FS, TBS1=4, buffer @0x20000100.
        let desc = [OWN | TDES0_LS | TDES0_FS, 4, 0x2000_0100, 0x2000_0010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1); // Consumes desc, queues the buffer read.
        seed_result(&mb, vec![0xDE, 0xAD, 0xBE, 0xEF]);
        emac.tick(1); // Completes: frame out, OWN home, TI.
        assert_eq!(emac.drain_frames(), [vec![0xDE, 0xAD, 0xBE, 0xEF]]);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_TI, DMA_TI);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_NIS, DMA_NIS);
        assert_eq!(emac.pending(), 1);
        // Ownership write-back queued for the bus.
        let ops = &mb.lock().unwrap().ops;
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_0000, bytes } if bytes == &word_le(TDES0_LS | TDES0_FS))));
        // W1C clears.
        write_word(&mut emac, DMARIS_OFF, DMA_TI | DMA_NIS);
        assert_eq!(emac.pending(), 0);
    }

    /// TX with `OWN` clear: spurious poll, nothing moves.
    #[test]
    fn tx_spurious_poll() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        seed_result(&mb, vec![0u8; 16]);
        emac.tick(1);
        assert_eq!(emac.drain_frames(), Vec::<Vec<u8>>::new());
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_TI, 0);
    }

    /// RX: injected frame lands in the owned buffer with `FS`/`LS`/`FL`.
    #[test]
    fn rx_flow() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_SR);
        write_word(&mut emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(&[0x01, 0x02, 0x03]);
        emac.tick(1); // Queues the descriptor read.
                      // Descriptor: OWN, RBS1=1536, buffer @0x20000200.
        let desc = [OWN, 1536, 0x2000_0200, 0x2000_1010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1); // Stores the frame, writes status, RI.
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_RI, DMA_RI);
        let ops = &mb.lock().unwrap().ops;
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_0200, bytes } if bytes == &vec![0x01, 0x02, 0x03])));
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_1000, bytes } if bytes == &word_le((3 << 16) | RDES0_FS | RDES0_LS))));
    }

    /// RX with no owned descriptor drops the frame.
    #[test]
    fn rx_drop_unowned() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_SR);
        write_word(&mut emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(&[0x01]);
        emac.tick(1);
        seed_result(&mb, vec![0u8; 16]); // OWN clear.
        emac.tick(1);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_RI, 0);
    }
}

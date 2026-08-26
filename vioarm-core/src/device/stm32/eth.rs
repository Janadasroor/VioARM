//! The STM32F1/F4 Ethernet controller (`ETHERNET_MAC`/`Ethernet_MAC`), driven
//! by the SVD-generated register tables.
//!
//! The SVD splits one controller across four peripherals — MAC at
//! `0x4002_8000`, MMC at `0x4002_8100`, PTP at `0x4002_8700`, DMA at
//! `0x4002_9000` — so [`Eth`] reconnects them behind a single address space and
//! runs the parts the tables cannot: the descriptor-ring DMA engines and their
//! write-one-to-clear status, and the host channel that injects received
//! frames and drains transmitted ones.
//!
//! The controller has no view of the memory bus, so each descriptor fetch and
//! each frame store is queued into an [`EthMailbox`] shared with
//! [`SystemMemory`], which drains it after every device tick. At most one read
//! is outstanding at a time: a phased state machine walks the RX ring (read
//! the descriptor, write the frame into its buffer, write back the status word)
//! and the TX ring (read the descriptor, read the frame data, write back the
//! cleared ownership word). The engines run only while the MAC is enabled for
//! that direction (`MACCR.RE` + `DMAOMR.SR` for RX, `MACCR.TE` + `DMAOMR.ST`
//! for TX).
//!
//! The controller raises two NVIC lines: `ETH` and `ETH_WKUP`, found by name
//! in the MAC table (61/62 on both the F107 and the F407). Line 0 is the DMA
//! normal-interrupt summary gated by `DMAIER.NISE`; line 1 is a wakeup latch
//! from the magic-packet filter (`MACPMTCSR.MPE`), gated by `MACIMR.PMTIM`,
//! cleared by reading `MACPMTCSR`. `pending()` is level-based on the latches,
//! so the ISR must clear them with its write-back (no `acknowledge` latch).
//!
//! The host channel frames every frame with an opcode and a two-byte little-
//! endian length (Ethernet frames run to 1518 bytes, beyond one byte). Input:
//! `[CMD_RX, len_lo, len_hi, payload...]`; output: `[EVT_TX, len_lo, len_hi,
//! payload...]`.

use crate::device::svd::{Access, Device, NoBehavior, PeripheralDef, RegisterDef, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

// MAC window register offsets.
const MACCR: u32 = 0x00;
#[allow(dead_code)] // frame filter, written through the generic path
const MACFFR: u32 = 0x04;
#[allow(dead_code)] // hash filters, written through the generic path
const MACHTHR: u32 = 0x08;
#[allow(dead_code)] // hash filters, written through the generic path
const MACHTLR: u32 = 0x0c;
const MACMIIAR: u32 = 0x10;
#[allow(dead_code)] // MII data register, no PHY behind the model
const MACMIIDR: u32 = 0x14;
const MACPMTCSR: u32 = 0x2c;
#[allow(dead_code)] // MAC wakeup source address, read through the generic path
const MACSR: u32 = 0x38;
const MACIMR: u32 = 0x3c;
const MACA0HR: u32 = 0x40;
const MACA0LR: u32 = 0x44;

// DMA window register offsets.
const DMABMR: u32 = 0x00;
const DMATPDR: u32 = 0x04;
const DMARPDR: u32 = 0x08;
const DMARDLAR: u32 = 0x0c;
const DMATDLAR: u32 = 0x10;
const DMASR: u32 = 0x14;
const DMAOMR: u32 = 0x18;
const DMAIER: u32 = 0x1c;
const DMACHTDR: u32 = 0x48;
const DMACHRDR: u32 = 0x4c;

// MACCR bits.
const RE: u32 = 1 << 2; // receiver enable
const TE: u32 = 1 << 3; // transmitter enable

// MACMIIAR bits: the MII busy bit is cleared as soon as the transaction
// "completes" (there is no PHY behind the model), so firmware's poll loop
// `while (ETH->MACMIIAR & MB)` can exit.
const MIIAR_MB: u32 = 1 << 0;
const MIIAR_MW: u32 = 1 << 1;
const PHY_BMCR: usize = 0;
const PHY_BMSR: usize = 1;
const PHY_ID1: usize = 2;
const PHY_ID2: usize = 3;

// MACPMTCSR / MACSR / MACIMR bits for the magic-packet wakeup line.
const PMTCSR_MPE: u32 = 1 << 1; // magic packet enable
const PMTCSR_MPR: u32 = 1 << 5; // magic packet received
const MACSR_PMTS: u32 = 1 << 3; // magic packet sent
const MACIMR_PMTIM: u32 = 1 << 3; // magic packet interrupt mask

// DMABMR bits.
const DMABMR_SR: u32 = 1 << 0; // software reset

// DMAOMR bits.
const DMAOMR_SR: u32 = 1 << 1; // receive start
const DMAOMR_ST: u32 = 1 << 13; // transmit start

// DMAIER bits.
const DMAIER_NISE: u32 = 1 << 16;
#[allow(dead_code)] // receive interrupt, enabled through the generic path
const DMAIER_RIE: u32 = 1 << 6;
#[allow(dead_code)] // transmit interrupt, enabled through the generic path
const DMAIER_TIE: u32 = 1 << 0;

// DMASR bits.
const DMASR_TS: u32 = 1 << 0; // transmit status
const DMASR_RS: u32 = 1 << 6; // receive status
const DMASR_NIS: u32 = 1 << 16; // normal interrupt summary
/// `RPS` read-only "running" encoding, shifted into place.
const DMASR_RPS_RUN: u32 = 0b01 << 17;
/// `TPS` read-only "running" encoding, shifted into place.
const DMASR_TPS_RUN: u32 = 0b001 << 20;

// Descriptor ring words.
const OWN: u32 = 0x8000_0000; // RDES0/TDES0 ownership bit
const RBS1_MASK: u32 = 0x1FFF; // RDES1 receive buffer 1 size
const TBS1_MASK: u32 = 0x1FFF; // TDES1 transmit buffer 1 size
const RX_FS: u32 = 0x0000_0200; // RDES0 first segment
const RX_LS: u32 = 0x0000_0100; // RDES0 last segment
#[allow(dead_code)] // frame length field, built inline in `rx_advance`
const RX_FL_MASK: u32 = 0x3FFF_0000; // RDES0 frame length (includes CRC)

// Host channel framing.
const CMD_RX: u8 = 0x01;
/// Output frame opcode for a transmitted frame.
const EVT_TX: u8 = 0x80;

fn eth_hash(mac: &[u8]) -> u32 {
    let mut crc = 0xFFFFFFFFu32;
    for &b in mac {
        crc ^= u32::from(b);
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    (!crc >> 26) & 0x3F
}

/// One memory request queued by the Ethernet controller for `SystemMemory` to
/// perform. Reads return their bytes through the mailbox's results FIFO.
#[derive(Debug)]
pub enum EthMemOp {
    /// Read `len` bytes at `addr`.
    Read { addr: u32, len: u16 },
    /// Write `bytes` at `addr`.
    Write { addr: u32, bytes: Vec<u8> },
}

/// The shared Ethernet mailbox: the controller pushes memory requests here and
/// `SystemMemory` performs them after every device tick.
#[derive(Debug, Default)]
pub struct EthMailbox {
    /// Pending memory requests, drained by `SystemMemory`.
    pub ops: Vec<EthMemOp>,
    /// Bytes read back by `SystemMemory`, consumed by the controller.
    pub results: VecDeque<Vec<u8>>,
}

/// One window of the controller: the SVD register words under `def`, reloaded
/// from the table's reset values by a soft reset.
#[derive(Debug)]
struct Win {
    def: &'static PeripheralDef,
    words: Box<[u32]>,
}

impl Win {
    fn new(def: &'static PeripheralDef) -> Self {
        let words = def.registers.iter().map(|r| r.reset).collect();
        Self { def, words }
    }

    fn index(&self, offset: u32) -> usize {
        self.def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .unwrap_or_else(|| panic!("no register at offset 0x{offset:x}"))
    }

    fn get(&self, offset: u32) -> u32 {
        self.words[self.index(offset)]
    }

    fn set(&mut self, offset: u32, value: u32) {
        let i = self.index(offset);
        self.words[i] = value;
    }

    fn merged(&self, offset: u32, val: u8, shift: u32) -> u32 {
        let current = self.get(offset);
        let mask = 0xFFu32 << shift;
        (current & !mask) | (u32::from(val) << shift)
    }

    /// The register covering byte offset `off` and its byte lane within it.
    fn reg_at(&self, off: u32) -> Result<(&'static RegisterDef, u32), BusError> {
        self.def
            .registers
            .iter()
            .find(|r| {
                let len = u32::from(r.width) / 8;
                off >= r.offset && off < r.offset + len
            })
            .map(|r| (r, 8 * (off - r.offset)))
            .ok_or(BusError::Reserved)
    }

    fn reload_reset(&mut self) {
        for (word, reg) in self.words.iter_mut().zip(self.def.registers) {
            *word = reg.reset;
        }
    }
}

/// Which descriptor fetch or frame move the DMA engine is waiting on.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum Phase {
    #[default]
    Idle,
    /// An RX descriptor read is in flight.
    AwaitRxDesc,
    /// A TX descriptor read is in flight.
    AwaitTxDesc,
    /// A TX frame-data read is in flight; the descriptor address to write back
    /// and its original status word are stashed until the data arrives.
    AwaitTxData { desc: u32, ctrlstat: u32, next: u32 },
}

/// Everything the SVD cannot describe about the controller.
#[derive(Debug, Default)]
struct EthState {
    /// The DMA engine's place in its descriptor walk.
    phase: Phase,
    /// The current RX descriptor address (from `DMARDLAR`, advanced by `Next`).
    rx_desc: u32,
    /// The current TX descriptor address (from `DMATDLAR`, advanced by `Next`).
    tx_desc: u32,
    /// A frame injected by the host, waiting for the RX engine to store it.
    rx_frame: Option<Vec<u8>>,
    /// Bytes of one host frame being accumulated.
    host_rx: Vec<u8>,
    /// A `DMATPDR` poll demand was written and the TX engine is enabled.
    tx_poll: bool,
    /// Completed TX frames, framed for the host channel.
    output: Vec<u8>,
}

/// A full Ethernet controller, reconnecting the SVD's four windows behind one
/// address space and the memory bus through a shared mailbox.
#[derive(Debug)]
pub struct Eth {
    ranges: [Region; 4],
    mac: Win,
    mmc: Win,
    ptp: Win,
    dma: Win,
    state: EthState,
    mailbox: Arc<Mutex<EthMailbox>>,
    exception: Option<u8>,
    exception_wkup: Option<u8>,
    name: &'static str,
    phy: [u16; 32],
}

impl Eth {
    /// An Ethernet controller on another chip's device tables (base
    /// `0x4002_8000`, IRQ 61/62 on the connectivity and F4 lines), sharing
    /// `mailbox` with the memory bus.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the four Ethernet
    /// peripherals.
    #[must_use]
    pub fn for_device<D: Device>(mailbox: Arc<Mutex<EthMailbox>>) -> Self {
        let mac = Win::new(eth_def::<D>("MAC"));
        let exception = irq_number(&mac, "ETH");
        let exception_wkup = irq_number(&mac, "ETH_WKUP");
        let mut phy = [0u16; 32];
        phy[PHY_BMCR] = 0x1000; // autoneg enable
        phy[PHY_BMSR] = 0x7809; // link up, autoneg complete, capable
        phy[PHY_ID1] = 0x0007;
        phy[PHY_ID2] = 0xC0F1;
        Self {
            ranges: [
                Region::new(mac.def.base, mac.def.window),
                Region::new(eth_def::<D>("MMC").base, eth_def::<D>("MMC").window),
                Region::new(eth_def::<D>("PTP").base, eth_def::<D>("PTP").window),
                Region::new(eth_def::<D>("DMA").base, eth_def::<D>("DMA").window),
            ],
            mac,
            mmc: Win::new(eth_def::<D>("MMC")),
            ptp: Win::new(eth_def::<D>("PTP")),
            dma: Win::new(eth_def::<D>("DMA")),
            state: EthState::default(),
            mailbox,
            exception,
            exception_wkup,
            name: "eth",
            phy,
        }
    }
}

/// The generated name of the Ethernet controller's `which` window: the F1
/// tables use `ETHERNET_`, the F4 tables `Ethernet_`. Each chip has exactly
/// one controller, so the name lookup is unambiguous. H7 uses `Ethernet_MAC`
/// at 0x40028000 with `Ethernet_MTL`/`Ethernet_DMA` (no `MMC`/`PTP`), so fall
/// back to a dummy window when a table is missing.
fn eth_def<D: Device>(which: &str) -> &'static PeripheralDef {
    for prefix in ["ETHERNET", "Ethernet"] {
        if let Some(def) = D::peripheral(&format!("{prefix}_{which}")) {
            return def;
        }
    }
    // H7: MMC/PTP are part of MAC or absent; use a dummy 0-window so the
    // controller still constructs (reads/writes there will be Reserved, which
    // the generic latch path tolerates).
    static DUMMY: PeripheralDef = PeripheralDef {
        name: "Ethernet_DUMMY",
        base: 0,
        window: 0,
        irqs: &[],
        registers: &[],
    };
    &DUMMY
}

/// The exception number of a named interrupt in a peripheral's table.
fn irq_number(win: &Win, name: &str) -> Option<u8> {
    win.def
        .irqs
        .iter()
        .find(|irq| irq.name == name)
        .map(|irq| 16 + irq.number)
}

/// The byte lane of `off` inside a four-byte register starting at `reg`.
fn lane(off: u32, reg: u32) -> Option<u32> {
    if off >= reg && off < reg + 4 {
        Some(off - reg)
    } else {
        None
    }
}

/// A plain read: the stored word's byte, or zero for a write-only register.
fn read_latch(win: &Win, off: u32) -> Result<u8, BusError> {
    let (reg, shift) = win.reg_at(off)?;
    if reg.access == Access::WriteOnly {
        return Ok(0);
    }
    let word = win.get(reg.offset);
    Ok(((word >> shift) & 0xFF) as u8)
}

/// A plain byte-lane read-modify-write, ignoring writes to read-only registers.
fn write_latch(win: &mut Win, off: u32, val: u8) -> Result<(), BusError> {
    let (reg, shift) = win.reg_at(off)?;
    if reg.access == Access::ReadOnly {
        return Ok(());
    }
    win.set(reg.offset, win.merged(reg.offset, val, shift));
    Ok(())
}

impl Eth {
    /// The MAC window's byte-offset dispatch. The generic latch path covers
    /// every register except the wakeup pair, which the ISR reads to clear.
    fn read_mac(&mut self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, MACPMTCSR) {
            let word = self.mac.get(MACPMTCSR);
            // Reading the power-management register clears the wakeup flags
            // the ISR just handled, releasing the `ETH_WKUP` line.
            if l == 0 {
                self.clear_macsr(MACSR_PMTS);
                self.mac.set(MACPMTCSR, word & !PMTCSR_MPR);
            }
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        read_latch(&self.mac, off)
    }

    fn write_mac(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        if let Some(l) = lane(off, MACMIIAR) {
            let merged = self.mac.merged(MACMIIAR, val, 8 * l);
            // MII transaction: if MB set, emulate PHY access via MACMIIDR.
            if merged & MIIAR_MB != 0 {
                let mw = merged & MIIAR_MW != 0;
                let mr = ((merged >> 6) & 0x1F) as usize;
                // PA is bits 11:15, ignored (single PHY at 0).
                if mr < 32 {
                    if mw {
                        // Write: latch low 16 bits from MACMIIDR into PHY.
                        let data = (self.mac.get(MACMIIDR) & 0xFFFF) as u16;
                        self.phy[mr] = data;
                    } else {
                        // Read: load PHY register into MACMIIDR.
                        let data = self.phy[mr];
                        self.mac.set(MACMIIDR, u32::from(data));
                    }
                }
            }
            self.mac.set(MACMIIAR, merged & !MIIAR_MB);
            return Ok(());
        }
        if let Some(l) = lane(off, MACPMTCSR) {
            let merged = self.mac.merged(MACPMTCSR, val, 8 * l);
            // Software enabling the wakeup filter clears the received latch.
            self.mac.set(MACPMTCSR, merged & !PMTCSR_MPR);
            return Ok(());
        }
        write_latch(&mut self.mac, off, val)
    }

    fn read_dma(&self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, DMASR) {
            // The status word is the latched bits plus the read-only running
            // state of whichever engine is enabled.
            let mut word = self.dma.get(DMASR);
            let omr = self.dma.get(DMAOMR);
            if omr & DMAOMR_SR != 0 {
                word |= DMASR_RPS_RUN;
            }
            if omr & DMAOMR_ST != 0 {
                word |= DMASR_TPS_RUN;
            }
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, DMACHTDR) {
            return Ok(((self.state.tx_desc >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, DMACHRDR) {
            return Ok(((self.state.rx_desc >> (8 * l)) & 0xFF) as u8);
        }
        read_latch(&self.dma, off)
    }

    fn write_dma(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        if let Some(l) = lane(off, DMABMR) {
            let merged = self.dma.merged(DMABMR, val, 8 * l);
            if l == 0 && merged & DMABMR_SR != 0 {
                self.soft_reset();
                return Ok(());
            }
            self.dma.set(DMABMR, merged);
            return Ok(());
        }
        if let Some(l) = lane(off, DMATPDR) {
            // A transmit poll demand; the engine picks it up while enabled.
            let _ = l;
            self.state.tx_poll = true;
            return Ok(());
        }
        if lane(off, DMARPDR).is_some() {
            // A receive poll demand: RX is host-driven, nothing to resume.
            return Ok(());
        }
        if let Some(l) = lane(off, DMARDLAR) {
            self.dma
                .set(DMARDLAR, self.dma.merged(DMARDLAR, val, 8 * l));
            self.state.rx_desc = self.dma.get(DMARDLAR);
            return Ok(());
        }
        if let Some(l) = lane(off, DMATDLAR) {
            self.dma
                .set(DMATDLAR, self.dma.merged(DMATDLAR, val, 8 * l));
            self.state.tx_desc = self.dma.get(DMATDLAR);
            return Ok(());
        }
        if let Some(l) = lane(off, DMASR) {
            // Write-one-to-clear of the latched status bits.
            let bits = u32::from(val) << (8 * l);
            self.dma.set(DMASR, self.dma.get(DMASR) & !bits);
            return Ok(());
        }
        write_latch(&mut self.dma, off, val)
    }
}

impl Eth {
    /// The `MACSR` wakeup latch, stored in the MAC window's register word.
    fn macsr(&self) -> u32 {
        self.mac.get(MACSR)
    }

    fn set_macsr(&mut self, bits: u32) {
        let word = self.mac.get(MACSR) | bits;
        self.mac.set(MACSR, word);
    }

    fn clear_macsr(&mut self, bits: u32) {
        let word = self.mac.get(MACSR) & !bits;
        self.mac.set(MACSR, word);
    }

    /// Whether the MAC is receiving and its DMA engine is started.
    fn rx_enabled(&self) -> bool {
        self.mac.get(MACCR) & RE != 0 && self.dma.get(DMAOMR) & DMAOMR_SR != 0
    }

    /// Whether the MAC is transmitting and its DMA engine is started.
    fn tx_enabled(&self) -> bool {
        self.mac.get(MACCR) & TE != 0 && self.dma.get(DMAOMR) & DMAOMR_ST != 0
    }

    /// The next pending host frame, if any.
    fn take_frame(&mut self) -> Option<Vec<u8>> {
        self.state.rx_frame.take()
    }

    fn push_read(&self, addr: u32, len: u16) {
        self.mailbox
            .lock()
            .unwrap()
            .ops
            .push(EthMemOp::Read { addr, len });
    }

    fn push_write(&self, addr: u32, bytes: Vec<u8>) {
        self.mailbox
            .lock()
            .unwrap()
            .ops
            .push(EthMemOp::Write { addr, bytes });
    }

    /// Pop the one result expected by the phase machine, if it has arrived.
    fn pop_result(&self) -> Option<Vec<u8>> {
        self.mailbox
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .results
            .pop_front()
    }

    /// Advance the RX ring one frame: store the pending host frame into the
    /// descriptor's buffer and write back its status word.
    fn rx_advance(&mut self, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (stat, ctrl, buf, next) = (w(0), w(1), w(2), w(3));
        if stat & OWN == 0 {
            // The buffer is not owned by the DMA; drop the frame rather than
            // overwrite firmware's data. (The ISR resumes the ring with a poll.)
            self.state.rx_frame = None;
            self.state.phase = Phase::Idle;
            return;
        }
        if let Some(frame) = self.take_frame() {
            let _ffr = self.mac.get(MACFFR);
            let _accept = true;
            let rbs1 = usize::try_from(ctrl & RBS1_MASK).unwrap_or(0);
            let n = frame.len().min(rbs1);
            if n > 0 {
                self.push_write(buf, frame[..n].to_vec());
            }
            // Status word: ownership returned, first+last segment set, frame
            // length (with CRC) in the upper field — exactly what the driver's
            // `GetRxFrameSize` decodes back to `len`.
            let fl = (u32::try_from(frame.len()).unwrap_or(0).wrapping_add(4)) & 0x3FFF;
            self.push_write(
                self.state.rx_desc,
                ((fl << 16) | RX_FS | RX_LS).to_le_bytes().to_vec(),
            );
            // A magic packet wakes the `ETH_WKUP` line when the filter is on.
            if self.mac.get(MACPMTCSR) & PMTCSR_MPE != 0 && self.is_magic_packet(&frame) {
                self.set_macsr(MACSR_PMTS);
                let word = self.mac.get(MACPMTCSR) | PMTCSR_MPR;
                self.mac.set(MACPMTCSR, word);
            }
        }
        self.latch_rx_status();
        self.state.rx_desc = next;
        self.state.phase = Phase::Idle;
    }

    /// Latch the receive-interrupt bits into `DMASR`.
    fn latch_rx_status(&mut self) {
        let word = self.dma.get(DMASR) | DMASR_RS | DMASR_NIS;
        self.dma.set(DMASR, word);
    }

    /// Latch the transmit-interrupt bits into `DMASR`.
    fn latch_tx_status(&mut self) {
        let word = self.dma.get(DMASR) | DMASR_TS | DMASR_NIS;
        self.dma.set(DMASR, word);
    }

    /// Whether `frame` is a magic packet: six bytes of `0xFF` followed by
    /// sixteen copies of the station address from `MACA0`.
    fn is_magic_packet(&self, frame: &[u8]) -> bool {
        if frame.len() < 6 + 96 {
            return false;
        }
        let hr = self.mac.get(MACA0HR);
        let lr = self.mac.get(MACA0LR);
        let mac = [
            ((hr >> 8) & 0xFF) as u8,
            (hr & 0xFF) as u8,
            ((lr >> 24) & 0xFF) as u8,
            ((lr >> 16) & 0xFF) as u8,
            ((lr >> 8) & 0xFF) as u8,
            (lr & 0xFF) as u8,
        ];
        frame[..6].iter().all(|&b| b == 0xFF) && frame[6..6 + 96].chunks_exact(6).all(|c| c == mac)
    }

    /// Complete a TX frame: drain the data to the host channel and return the
    /// descriptor's ownership to firmware.
    fn tx_complete(&mut self, data: &[u8]) {
        let len = u16::try_from(data.len()).unwrap_or(u16::MAX);
        self.state.output.push(EVT_TX);
        self.state.output.push((len & 0xFF) as u8);
        self.state.output.push((len >> 8) as u8);
        self.state.output.extend_from_slice(data);
        self.latch_tx_status();
        self.state.phase = Phase::Idle;
    }
}

impl Peripheral for Eth {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = addr.wrapping_sub(self.mac.def.base);
        if off < self.mac.def.window {
            return self.read_mac(off);
        }
        let off = addr.wrapping_sub(self.mmc.def.base);
        if off < self.mmc.def.window {
            return read_latch(&self.mmc, off);
        }
        let off = addr.wrapping_sub(self.ptp.def.base);
        if off < self.ptp.def.window {
            return read_latch(&self.ptp, off);
        }
        let off = addr.wrapping_sub(self.dma.def.base);
        if off < self.dma.def.window {
            return self.read_dma(off);
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = addr.wrapping_sub(self.mac.def.base);
        if off < self.mac.def.window {
            return self.write_mac(off, val);
        }
        let off = addr.wrapping_sub(self.mmc.def.base);
        if off < self.mmc.def.window {
            return write_latch(&mut self.mmc, off, val);
        }
        let off = addr.wrapping_sub(self.ptp.def.base);
        if off < self.ptp.def.window {
            return write_latch(&mut self.ptp, off, val);
        }
        let off = addr.wrapping_sub(self.dma.def.base);
        if off < self.dma.def.window {
            return self.write_dma(off, val);
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        self.soft_reset();
    }

    fn tick(&mut self, _delta: u64) {
        // Consume the result of the in-flight read, if it has landed.
        if let Some(result) = self.pop_result() {
            match self.state.phase {
                Phase::AwaitRxDesc => self.rx_advance(&result),
                Phase::AwaitTxDesc => self.consume_tx_desc(&result),
                Phase::AwaitTxData {
                    desc,
                    ctrlstat,
                    next,
                } => {
                    let own_clear = ctrlstat & !OWN;
                    self.push_write(desc, own_clear.to_le_bytes().to_vec());
                    self.state.tx_desc = next;
                    self.tx_complete(&result);
                }
                Phase::Idle => {}
            }
            return;
        }
        // Nothing in flight: start new work.
        if self.state.phase == Phase::Idle {
            if self.rx_enabled() && self.state.rx_frame.is_some() {
                self.push_read(self.state.rx_desc, 16);
                self.state.phase = Phase::AwaitRxDesc;
            } else if self.tx_enabled() && self.state.tx_poll {
                self.state.tx_poll = false;
                self.push_read(self.state.tx_desc, 16);
                self.state.phase = Phase::AwaitTxDesc;
            }
        }
    }

    fn name(&self) -> &'static str {
        self.name
    }

    fn drive_inputs(&mut self, value: u32) {
        self.state.host_rx.push((value & 0xFF) as u8);
        let frame = core::mem::take(&mut self.state.host_rx);
        let opcode = frame.first().copied().unwrap_or(0);
        let len = usize::from(*frame.get(1).unwrap_or(&0))
            | (usize::from(*frame.get(2).unwrap_or(&0)) << 8);
        if opcode != CMD_RX {
            // Unknown opcode: the stream is out of sync, so drop the whole
            // frame rather than misparse a later byte.
            self.state.host_rx.clear();
            return;
        }
        let expected = 3 + len;
        if frame.len() < expected {
            self.state.host_rx = frame;
            return;
        }
        self.state.rx_frame = Some(frame[3..3 + len].to_vec());
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.state.output)
    }
}

impl Eth {
    fn consume_tx_desc(&mut self, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (ctrlstat, size, buf, next) = (w(0), w(1), w(2), w(3));
        if ctrlstat & OWN == 0 {
            // Nothing owned by the DMA; the poll was spurious.
            self.state.phase = Phase::Idle;
            return;
        }
        let tbs1 = size & TBS1_MASK;
        let len = u16::try_from(tbs1).unwrap_or(u16::MAX);
        let desc = self.state.tx_desc;
        self.push_read(buf, len);
        self.state.phase = Phase::AwaitTxData {
            desc,
            ctrlstat,
            next,
        };
    }
}

impl InterruptSource for Eth {
    fn exception_number(&self) -> Option<u8> {
        self.exception
    }

    fn pending(&self) -> u32 {
        let mut lines = 0u32;
        let dmasr = self.dma.get(DMASR);
        let ier = self.dma.get(DMAIER);
        if dmasr & DMASR_NIS != 0 && ier & DMAIER_NISE != 0 {
            lines |= 1 << 0;
        }
        let pmts = self.macsr() & MACSR_PMTS;
        if pmts != 0 && self.mac.get(MACIMR) & MACIMR_PMTIM == 0 {
            lines |= 1 << 1;
        }
        lines
    }

    fn lines(&self) -> u32 {
        0b11
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        match line {
            0 => self.exception,
            1 => self.exception_wkup,
            _ => None,
        }
    }
}

impl Eth {
    fn soft_reset(&mut self) {
        self.mac.reload_reset();
        self.mmc.reload_reset();
        self.ptp.reload_reset();
        self.dma.reload_reset();
        self.state = EthState::default();
        let mut mailbox = self.mailbox.lock().unwrap_or_else(|e| e.into_inner());
        mailbox.ops.clear();
        mailbox.results.clear();
    }
}

/// H7 single-window Ethernet (MAC at 0x40028000, window 0x0BDF) — stub that
/// round-trips writes so HAL probing does not HardFault. Real H7 DMA/MTL
/// are not modeled; use this for H743 board bringup.
#[derive(Debug)]
pub struct EthH7 {
    inner: RegisterFile<NoBehavior>,
}

impl EthH7 {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("Ethernet_MAC")
            .or_else(|| D::peripheral("ETH"))
            .expect("generated tables contain Ethernet_MAC");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

impl Peripheral for EthH7 {
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
    fn name(&self) -> &'static str {
        "eth"
    }
}

impl InterruptSource for EthH7 {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::{Eth, EthMailbox, EVT_TX, MACA0HR, MACA0LR, MACCR, MACPMTCSR, OWN, RE, TE};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use crate::mem::bus::Bus;
    use crate::mem::flat::FlatMemory;
    use crate::mem::system::SystemMemory;
    use std::sync::{Arc, Mutex};

    const MAC: u32 = 0x4002_8000;
    const DMA: u32 = 0x4002_9000;
    const RAM: u32 = 0x2000_0000;

    /// A `SystemMemory` over RAM with an attached Ethernet controller sharing
    /// the mailbox, plus the mailbox `Arc` for direct inspection.
    fn sys() -> (SystemMemory<FlatMemory>, Arc<Mutex<EthMailbox>>) {
        let backing = FlatMemory::with_layout(&[RAM], &[0x1000]);
        let mailbox = Arc::new(Mutex::new(EthMailbox::default()));
        let mut mem = SystemMemory::new(backing);
        mem.set_eth_mailbox(mailbox.clone());
        mem.attach(Box::new(Eth::for_device::<
            crate::device::svd::stm32f107::Stm32f107,
        >(mailbox.clone())));
        (mem, mailbox)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(sys: &mut SystemMemory<FlatMemory>, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            sys.write(addr.wrapping_add(u32::try_from(i).unwrap_or(u32::MAX)), *b)
                .unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(sys: &mut SystemMemory<FlatMemory>, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = sys
                .read(addr.wrapping_add(u32::try_from(i).unwrap_or(u32::MAX)))
                .unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    /// One full cycle: every device tick runs and the mailbox is drained.
    fn cycle(sys: &mut SystemMemory<FlatMemory>) {
        sys.tick_devices(1);
    }

    fn enable_rx(sys: &mut SystemMemory<FlatMemory>) {
        write_word(sys, MAC + MACCR, RE);
        write_word(sys, DMA + 0x18, 1 << 1); // DMAOMR SR
        write_word(sys, DMA + 0x1c, (1 << 16) | (1 << 6)); // DMAIER NISE|RIE
    }

    fn rx_ring(sys: &mut SystemMemory<FlatMemory>, buf: u32) {
        write_word(sys, RAM, 0x8000_0000); // OWN
        write_word(sys, RAM + 4, 0x4000 | 0x0600); // RCH | RBS1 (1536)
        write_word(sys, RAM + 8, buf);
        write_word(sys, RAM + 0x0c, RAM); // self-chained
        write_word(sys, DMA + 0x0c, RAM); // DMARDLAR
    }

    fn inject_frame(sys: &mut SystemMemory<FlatMemory>, frame: &[u8]) {
        let len = u32::try_from(frame.len()).unwrap_or(u32::MAX);
        sys.devices_mut()[0].drive_inputs(u32::from(super::CMD_RX));
        sys.devices_mut()[0].drive_inputs(len & 0xFF);
        sys.devices_mut()[0].drive_inputs((len >> 8) & 0xFF);
        for b in frame {
            sys.devices_mut()[0].drive_inputs(u32::from(*b));
        }
    }

    #[test]
    fn eth_f107_and_f407_share_layout_and_irqs() {
        let (mem, _mailbox) = sys();
        let e = mem.devices()[0].as_ref() as &dyn InterruptSource;
        assert_eq!(e.exception_number(), Some(16 + 61), "ETH");
        assert_eq!(e.exception_for_line(1), Some(16 + 62), "ETH_WKUP");
        assert_eq!(e.ranges()[0].base, MAC);
        let f407 = Eth::for_device::<crate::device::svd::stm32f407::Stm32f407>(Arc::new(
            Mutex::new(EthMailbox::default()),
        ));
        assert_eq!(f407.exception_number(), Some(16 + 61));
        assert_eq!(f407.exception_for_line(1), Some(16 + 62));
        assert_eq!(f407.ranges()[0].len, 0x61, "F407 MAC window");
        assert_eq!(f407.ranges()[1].base, 0x4002_8100, "MMC");
        assert_eq!(f407.ranges()[2].base, 0x4002_8700, "PTP");
        assert_eq!(f407.ranges()[3].base, 0x4002_9000, "DMA");
    }

    #[test]
    fn soft_reset_reloads_dma_bmr() {
        let (mut mem, _mailbox) = sys();
        // Writing the SWR bit reloads every window from the SVD reset values.
        mem.write(DMA, 0x01).unwrap();
        assert_eq!(read_word(&mut mem, DMA), 0x0002_0101);
    }

    #[test]
    fn rx_frame_walks_descriptor_and_latches_status() {
        let (mut mem, _mailbox) = sys();
        enable_rx(&mut mem);
        rx_ring(&mut mem, RAM + 0x400);
        inject_frame(&mut mem, &[1, 2, 3, 4, 5, 6]);
        cycle(&mut mem);
        // Second cycle consumes the descriptor read and lands the frame.
        cycle(&mut mem);
        assert_eq!(mem.devices()[0].pending() & 1, 1, "ETH line");
        assert_eq!(
            read_word(&mut mem, RAM + 0x400),
            0x0403_0201,
            "frame stored little-endian"
        );
        let stat = read_word(&mut mem, RAM);
        assert_eq!((stat >> 16) & 0x3FFF, 10, "FL = len + 4");
        assert_ne!(stat & 0x0000_0200, 0, "FS");
        assert_ne!(stat & 0x0000_0100, 0, "LS");
        assert_eq!(stat & OWN, 0, "ownership returned to CPU");
        // The ISR's write-back clears the latches.
        write_word(&mut mem, DMA + 0x14, 0xFFFF_FFFF);
        assert_eq!(mem.devices()[0].pending() & 1, 0);
    }

    #[test]
    fn tx_frame_walks_descriptor_and_drains() {
        let (mut mem, _mailbox) = sys();
        write_word(&mut mem, MAC + MACCR, TE);
        write_word(&mut mem, DMA + 0x18, 1 << 13); // DMAOMR ST
        write_word(&mut mem, DMA + 0x1c, (1 << 16) | 1); // DMAIER NISE|TIE
        write_word(&mut mem, RAM + 0x100, OWN | 0x3000_0000); // TDES0 OWN|LS|FS
        write_word(&mut mem, RAM + 0x104, 4); // TDES1 TBS1
        write_word(&mut mem, RAM + 0x108, RAM + 0x800); // TDES2 buffer
        write_word(&mut mem, RAM + 0x10c, RAM + 0x100); // TDES3 self
        write_word(&mut mem, DMA + 0x10, RAM + 0x100); // DMATDLAR
        write_word(&mut mem, RAM + 0x800, 0x4443_4241); // frame bytes
        write_word(&mut mem, DMA + 0x04, 0); // DMATPDR poll demand
        cycle(&mut mem);
        cycle(&mut mem);
        cycle(&mut mem);
        let out = mem.devices_mut()[0].drain_output();
        assert_eq!(out[0], EVT_TX);
        assert_eq!(out[1], 4);
        assert_eq!(out[2], 0);
        assert_eq!(&out[3..], &[0x41, 0x42, 0x43, 0x44]);
        assert_eq!(read_word(&mut mem, RAM + 0x100) & OWN, 0, "OWN cleared");
        assert_eq!(mem.devices()[0].pending() & 1, 1, "ETH line");
    }

    #[test]
    fn miir_busy_bit_clears_immediately() {
        let (mut mem, _mailbox) = sys();
        write_word(&mut mem, MAC + 0x10, 1 | (2 << 2)); // MACMIIAR MB | CR
        assert_eq!(read_word(&mut mem, MAC + 0x10) & 1, 0, "MB auto-clear");
    }

    #[test]
    fn magic_packet_wakes_the_wkup_line() {
        let (mut mem, _mailbox) = sys();
        enable_rx(&mut mem);
        write_word(&mut mem, MAC + MACA0HR, 0x0102);
        write_word(&mut mem, MAC + MACA0LR, 0x0304_0506);
        write_word(&mut mem, MAC + MACPMTCSR, 1 << 1); // MPE
        rx_ring(&mut mem, RAM + 0x400);
        let mut frame = Vec::new();
        frame.extend([0xFF; 6]);
        for _ in 0..16 {
            frame.extend([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        }
        inject_frame(&mut mem, &frame);
        cycle(&mut mem);
        cycle(&mut mem);
        assert_eq!(mem.devices()[0].pending() & 0b10, 0b10, "ETH_WKUP line");
        // The ISR reads MACPMTCSR to clear the wakeup.
        assert_eq!(
            read_word(&mut mem, MAC + MACPMTCSR) & (1 << 5),
            1 << 5,
            "MPR"
        );
        assert_eq!(mem.devices()[0].pending() & 0b10, 0, "wakeup cleared");
        // A non-magic frame never wakes.
        let (mut mem2, _mailbox2) = sys();
        enable_rx(&mut mem2);
        write_word(&mut mem2, MAC + MACPMTCSR, 1 << 1);
        rx_ring(&mut mem2, RAM + 0x400);
        inject_frame(&mut mem2, &[0xAA, 0xBB, 0xCC]);
        cycle(&mut mem2);
        cycle(&mut mem2);
        assert_eq!(mem2.devices()[0].pending() & 0b10, 0);
    }

    #[test]
    fn interrupts_gated_by_masks() {
        let (mut mem, _mailbox) = sys();
        write_word(&mut mem, MAC + MACCR, RE);
        write_word(&mut mem, DMA + 0x18, 1 << 1);
        // No NISE: the RX latch must not assert the line.
        write_word(&mut mem, DMA + 0x1c, 0);
        rx_ring(&mut mem, RAM + 0x400);
        inject_frame(&mut mem, &[1, 2, 3]);
        cycle(&mut mem);
        cycle(&mut mem);
        assert_eq!(mem.devices()[0].pending() & 1, 0, "NIS masked");
        write_word(&mut mem, DMA + 0x1c, 1 << 16);
        assert_eq!(mem.devices()[0].pending() & 1, 1, "NISE now enables");
    }

    #[test]
    fn addresses_outside_the_windows_are_reserved() {
        let (mut mem, _mailbox) = sys();
        assert!(matches!(mem.read(DMA + 0x400), Err(BusError::Reserved)));
        assert!(matches!(mem.write(MAC - 1, 0), Err(BusError::Reserved)));
        // Inside a window but past the last register is still reserved.
        assert!(matches!(mem.read(MAC + 0x60), Err(BusError::Reserved)));
        assert!(mem.read(MAC).is_ok(), "MAC window");
        assert!(mem.read(DMA).is_ok(), "DMA window");
    }
}

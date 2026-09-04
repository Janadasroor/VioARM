//! Bosch M_CAN (FDCAN) controller: transmit/receive with message-RAM
//! elements, standard-ID filtering, and loopback.
//!
//! The register file is table-driven; the transfer engine is not. Message-RAM
//! elements (filters, TX buffers, RX FIFO storage) live wherever firmware
//! points `SIDFC`/`TXBC`/`RXF0C`/`RXF1C` — dedicated RAM windows or plain
//! SRAM, the controller cannot tell — and reach the controller through the
//! [`FdcanMailbox`] the bus drains after every tick (request latency of a
//! tick or two, like DMA/Ethernet). Transmit completes once its element
//! arrives; a received frame lands in the configured RX FIFO after that, and
//! its payload bytes are written back to message RAM through the mailbox.
//!
//! Modeled: `CCCR` (`INIT`/`CCE` config gate, `FDO`/`BRSE`/`DAR`/loopback
//! latches), `NBTP`/`DBTP` latches, TX request/pending/complete flags with a
//! free-level counter, RX FIFO0/1 with watermark/full/lost flags,
//! blocking-or-overwrite choice and acknowledge index, standard-ID filters
//! (range/dual-ID/exact; extended IDs follow `GFC`), `IR`/`IE` for
//! receive/transmit-complete, single-line interrupts. `IR` bit positions are
//! Bosch-fixed (`RF0N` 0, `RF0W` 1, `RF0F` 2, `RF0L` 3, `RF1N` 4, `TC` 9 —
//! several SVDs leave the fields anonymous). Loopback (`TEST.LBCK`) feeds
//! transmitted frames into the RX path so the whole chain is testable
//! without a bus. Configuration registers honor the `INIT`+`CCE` gate.
//!
//! Deliberately unmodeled (documented): bit-timing enforcement and error
//! frames (`ECR`/`PSR` latch), TX event FIFO (frames with `EFC` complete
//! normally), cancellation (`TXBCR` consumed), timestamping, clock-stop and
//! power-down. Without an installed mailbox (unit tests) TX cannot fetch
//! elements and RX payloads are not written back; flags still update.

use crate::device::svd::{Behavior, Device, NoBehavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// `TEST` lives at register offset `0x10`.
const TEST: u32 = 0x10;
/// `CCCR` lives at register offset `0x18`.
const CCCR: u32 = 0x18;
/// `GFC` lives at register offset `0x80`.
const GFC: u32 = 0x80;
/// `SIDFC` lives at register offset `0x84`.
const SIDFC: u32 = 0x84;
/// `RXF0C`/`RXF0S`/`RXF0A` live at `0xA0`/`0xA4`/`0xA8`.
const RXF0C: u32 = 0xA0;
const RXF0S: u32 = 0xA4;
const RXF0A: u32 = 0xA8;
/// `RXF1C`/`RXF1S`/`RXF1A` live at `0xB0`/`0xB4`/`0xB8`.
const RXF1C: u32 = 0xB0;
const RXF1S: u32 = 0xB4;
const RXF1A: u32 = 0xB8;
/// `RXESC` lives at register offset `0xBC`.
const RXESC: u32 = 0xBC;
/// `TXBC`/`TXFQS`/`TXESC` live at `0xC0`/`0xC4`/`0xC8`.
const TXBC: u32 = 0xC0;
const TXFQS: u32 = 0xC4;
const TXESC: u32 = 0xC8;
/// `TXBRP`/`TXBAR`/`TXBTO` live at `0xCC`/`0xD0`/`0xD8`.
const TXBRP: u32 = 0xCC;
const TXBAR: u32 = 0xD0;
const TXBTO: u32 = 0xD8;
/// `IR`/`IE` live at `0x50`/`0x54`.
const IR: u32 = 0x50;
const IE: u32 = 0x54;

const CCCR_INIT: u32 = 1;
const CCCR_CCE: u32 = 1 << 1;
const TEST_LBCK: u32 = 1 << 4;

/// Bosch-fixed `IR` bit numbers.
const IR_RF0N: u32 = 0;
const IR_RF0W: u32 = 1;
const IR_RF0F: u32 = 2;
const IR_RF0L: u32 = 3;
const IR_RF1N: u32 = 4;
const IR_RF1W: u32 = 5;
const IR_RF1F: u32 = 6;
const IR_RF1L: u32 = 7;
const IR_TC: u32 = 9;

/// `RxFnS` flag bits.
const F0F: u32 = 1 << 24;
const F0LOST: u32 = 1 << 25;

/// A message-RAM read the controller needs the bus to perform.
#[derive(Clone, Copy, Debug)]
pub struct FdcanRead {
    /// First byte address.
    pub addr: u32,
    /// Byte count.
    pub len: u8,
    /// What the bytes are: `0` = TX buffer element, `1` = standard filters.
    pub kind: u8,
    /// Element index (TX buffer number; filters always start at 0).
    pub index: u32,
}

/// A completed message-RAM read, correlated by kind and index.
#[derive(Clone, Debug)]
pub struct FdcanResult {
    /// Echo of [`FdcanRead::kind`].
    pub kind: u8,
    /// Echo of [`FdcanRead::index`].
    pub index: u32,
    /// The bytes read.
    pub bytes: Vec<u8>,
}

/// The controller/bus mailbox, shared like the DMA/Ethernet ones: the
/// controller queues reads and message-RAM writes, the bus performs them
/// after the tick and appends correlated results for the controller.
#[derive(Debug, Default)]
pub struct FdcanMailbox {
    /// Pending reads for the bus.
    pub reads: Vec<FdcanRead>,
    /// Pending `(address, bytes)` writes for the bus.
    pub writes: Vec<(u32, Vec<u8>)>,
    /// Read results, for the controller to consume.
    pub results: VecDeque<FdcanResult>,
}

/// A CAN frame on the wire (classical or FD): the unit the FIFOs move.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    /// 11-bit standard or 29-bit extended identifier.
    pub id: u32,
    /// Extended identifier.
    pub extended: bool,
    /// Remote request (classical RTR / FD RRS).
    pub remote: bool,
    /// FD format (else classical).
    pub fd: bool,
    /// Bit-rate switch (FD only).
    pub brs: bool,
    /// Payload bytes.
    pub data: Vec<u8>,
}

/// CAN-FD data length code to byte count.
#[must_use]
pub const fn dlc_bytes(dlc: u32) -> usize {
    match dlc & 0xF {
        0..=8 => (dlc & 0xF) as usize,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// Payload byte count from a data-byte-size field value.
fn tbds(v: u32) -> usize {
    dlc_bytes(8 + (v & 0x7))
}

/// Payload length to data length code (exact for valid sizes only).
fn bytes_dlc(n: usize) -> u32 {
    match n {
        0..=8 => n as u32,
        12 => 9,
        16 => 10,
        20 => 11,
        24 => 12,
        32 => 13,
        48 => 14,
        _ => 15,
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RxTarget {
    #[default]
    Fifo0,
    Fifo1,
    Reject,
}

/// Standard-ID filter element decision for one identifier: range, dual-ID,
/// classic mask (`(id ^ filter) & mask == 0`), or exact match route per the
/// element's FIFO assignment; anything else misses.
fn std_filter_match(word: u32, id: u32) -> Option<RxTarget> {
    let sft = (word >> 30) & 0x3;
    let sfec = (word >> 27) & 0x7;
    let sfid2 = (word >> 16) & 0x7FF;
    let sfid1 = word & 0x7FF;
    let hit = match sft {
        0 | 3 => sfid1 <= id && id <= sfid2,
        1 => id == sfid1 || id == sfid2,
        2 => (id ^ sfid1) & sfid2 == 0,
        _ => false,
    };
    if !hit {
        return None;
    }
    match sfec {
        1 | 5 | 6 => Some(RxTarget::Fifo0),
        2 | 7 => Some(RxTarget::Fifo1),
        _ => Some(RxTarget::Reject),
    }
}

/// TEST/IR/IE handling shared by both layouts: the SVDs mark them read-only,
/// but silicon implements TEST gated on `CCCR.TEST`, `IR` write-1-to-clear,
/// and plain `IE`. Returns the outcome when `addr` hits one of them.
fn test_ir_ie_write<B: Behavior>(
    inner: &mut RegisterFile<B>,
    addr: u32,
    val: u8,
) -> Option<Result<(), BusError>> {
    let base = inner.ranges()[0].base;
    let off = addr.wrapping_sub(base);
    if off == TEST || off == TEST + 1 || off == TEST + 2 || off == TEST + 3 {
        let cccr = inner.word(CCCR);
        if cccr & (1 << 7) == 0 {
            return Some(Ok(()));
        }
        return Some(inner.write_raw(addr, val));
    }
    if off == IR || off == IR + 1 || off == IR + 2 || off == IR + 3 {
        let cur = inner.read(addr).unwrap_or(0);
        return Some(inner.write_raw(addr, cur & !val));
    }
    if off == IE || off == IE + 1 || off == IE + 2 || off == IE + 3 {
        return Some(inner.write_raw(addr, val));
    }
    None
}

#[derive(Debug, Default)]
struct FdcanBehavior {
    /// Message-RAM base: register words hold 16-bit offsets into it
    /// (board-wired from the SVD RAM window; unit tests pass their own).
    msg_ram_base: u32,
    /// Requested TX buffer bits without an outstanding element read.
    tx_pending: u32,
    /// TX buffer bits with an outstanding element read.
    tx_outstanding: u32,
    /// Cached standard-filter words, refreshed from message RAM.
    filters: Vec<u32>,
    /// A filter read is in flight (don't queue another).
    filters_outstanding: bool,
    /// Received frames awaiting FIFO insertion.
    rx_queue: VecDeque<CanFrame>,
    /// RX FIFO0/1 contents.
    fifo0: VecDeque<CanFrame>,
    fifo1: VecDeque<CanFrame>,
    mailbox: Option<Arc<Mutex<FdcanMailbox>>>,
}

impl FdcanBehavior {
    fn raise_ir(view: &mut Registers<&mut [u32]>, bit: u32) {
        view.set(IR, view.get(IR) | (1 << bit));
    }

    /// Effective message-RAM address: base plus the 16-bit word offset.
    fn ram_addr(&self, reg_word: u32) -> u32 {
        self.msg_ram_base.wrapping_add(reg_word & 0x0000_FFFC)
    }

    /// Configuration mode: `CCCR.INIT` and `CCE` both set.
    fn config_mode(view: &Registers<&mut [u32]>) -> bool {
        view.get(CCCR) & (CCCR_INIT | CCCR_CCE) == (CCCR_INIT | CCCR_CCE)
    }

    /// FIFO configuration `(size, overwrite, watermark, base)` from an `RxFnC`
    /// word; the base is a message-RAM offset resolved against `msg_ram_base`.
    fn fifo_cfg(&self, word: u32) -> (usize, bool, usize, u32) {
        (
            ((word >> 16) & 0x7F) as usize,
            word & (1 << 31) != 0,
            ((word >> 24) & 0x7F) as usize,
            self.ram_addr(word),
        )
    }

    /// Data-byte size for TX buffers (`TXESC`) or RX FIFO `n` (`RXESC`).
    fn data_bytes(view: &Registers<&mut [u32]>, tx: bool, fifo: u32) -> usize {
        if tx {
            tbds(view.get(TXESC) & 0x7)
        } else if fifo == 0 {
            tbds(view.get(RXESC) & 0x7)
        } else {
            tbds((view.get(RXESC) >> 4) & 0x7)
        }
    }

    /// Queue a standard-filter refresh when `SIDFC` names any and none is
    /// cached or in flight.
    fn refresh_filters(&mut self, view: &mut Registers<&mut [u32]>) {
        let sidfc = view.get(SIDFC);
        let lss = ((sidfc >> 16) & 0xFF) as usize;
        if lss == 0 || !self.filters.is_empty() || self.filters_outstanding {
            return;
        }
        if let Some(mb) = &self.mailbox {
            let addr = self.ram_addr(sidfc);
            mb.lock()
                .unwrap_or_else(|e| e.into_inner())
                .reads
                .push(FdcanRead {
                    addr,
                    len: (lss * 4).min(255) as u8,
                    kind: 1,
                    index: 0,
                });
            self.filters_outstanding = true;
        }
    }

    /// Consume arrived filter words.
    fn drain_filter_results(&mut self, mb: &mut FdcanMailbox) {
        let mut i = 0;
        while i < mb.results.len() {
            if mb.results[i].kind != 1 {
                i += 1;
                continue;
            }
            let r = mb.results.remove(i).expect("indexed");
            self.filters = r
                .bytes
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect();
            self.filters_outstanding = false;
        }
    }

    /// Classify one frame: standard filters, else extended/global rules.
    /// Returns `None` while filter words are still in flight.
    fn classify(&self, view: &mut Registers<&mut [u32]>, frame: &CanFrame) -> Option<RxTarget> {
        let gfc = view.get(GFC);
        let anfs = (gfc >> 2) & 0x3;
        let anfe = (gfc >> 4) & 0x3;
        let global = |fifo01: u32| match fifo01 {
            0 | 1 => RxTarget::Fifo0,
            2 => RxTarget::Fifo1,
            _ => RxTarget::Reject,
        };
        if frame.remote {
            let rrfe = gfc & 1 != 0;
            let rrfs = gfc & (1 << 1) != 0;
            if (frame.extended && rrfe) || (!frame.extended && rrfs) {
                return Some(RxTarget::Fifo0);
            }
            return Some(RxTarget::Reject);
        }
        if !frame.extended {
            let sidfc = view.get(SIDFC);
            if ((sidfc >> 16) & 0xFF) > 0 {
                if self.filters.is_empty() {
                    return None;
                }
                for f in &self.filters {
                    if let Some(target) = std_filter_match(*f, frame.id) {
                        return Some(target);
                    }
                }
            }
            return Some(global(anfs));
        }
        Some(global(anfe))
    }

    /// Insert one classified frame into its FIFO: payload bytes go to message
    /// RAM through the mailbox, flags and `IR` update immediately.
    fn insert(&mut self, view: &mut Registers<&mut [u32]>, fifo_no: u32, frame: CanFrame) {
        let (c_off, s_off) = if fifo_no == 0 {
            (RXF0C, RXF0S)
        } else {
            (RXF1C, RXF1S)
        };
        let (size, overwrite, watermark, base) = self.fifo_cfg(view.get(c_off));
        if size == 0 {
            return;
        }
        let fifo = if fifo_no == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        if fifo.len() >= size {
            view.set(s_off, view.get(s_off) | F0F);
            Self::raise_ir(view, if fifo_no == 0 { IR_RF0F } else { IR_RF1F });
            if !overwrite {
                view.set(s_off, view.get(s_off) | F0LOST);
                Self::raise_ir(view, if fifo_no == 0 { IR_RF0L } else { IR_RF1L });
                return;
            }
            fifo.pop_front();
        }
        let elen = 8 + Self::data_bytes(view, false, fifo_no);
        let mut bytes = Vec::with_capacity(elen);
        bytes.extend_from_slice(
            &(frame.id | (u32::from(frame.remote) << 30) | (u32::from(frame.extended) << 31))
                .to_le_bytes(),
        );
        bytes.extend_from_slice(
            &((bytes_dlc(frame.data.len()) << 16)
                | (u32::from(frame.brs) << 20)
                | (u32::from(frame.fd) << 21))
                .to_le_bytes(),
        );
        bytes.extend_from_slice(&frame.data);
        bytes.resize(elen, 0);
        if let Some(mb) = &self.mailbox {
            mb.lock()
                .unwrap_or_else(|e| e.into_inner())
                .writes
                .push((base.wrapping_add(fifo.len() as u32 * elen as u32), bytes));
        }
        fifo.push_back(frame);
        Self::write_fifo_status(view, s_off, fifo.len(), size);
        Self::raise_ir(view, if fifo_no == 0 { IR_RF0N } else { IR_RF1N });
        if watermark > 0 && fifo.len() >= watermark {
            Self::raise_ir(view, if fifo_no == 0 { IR_RF0W } else { IR_RF1W });
        }
    }

    /// Rewrite an `RxFnS` word from FIFO state: fill count and put index;
    /// the full flag recomputes, the lost flag sticks until acknowledged.
    fn write_fifo_status(view: &mut Registers<&mut [u32]>, s_off: u32, fill: usize, size: usize) {
        let mut s = view.get(s_off) & !(0x007F_FFFF | F0F);
        s |= (fill.min(0x7F)) as u32;
        s |= ((fill.min(size).min(0x7F)) as u32) << 16;
        if fill >= size && size > 0 {
            s |= F0F;
        }
        view.set(s_off, s);
    }

    /// Drain one RXFIFO acknowledge: drop `n` frames from the front and clear
    /// the lost flag.
    fn acknowledge_fifo(&mut self, view: &mut Registers<&mut [u32]>, fifo_no: u32, n: usize) {
        let (c_off, s_off) = if fifo_no == 0 {
            (RXF0C, RXF0S)
        } else {
            (RXF1C, RXF1S)
        };
        let (size, _, _, _) = self.fifo_cfg(view.get(c_off));
        let fifo = if fifo_no == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        for _ in 0..n {
            if fifo.pop_front().is_none() {
                break;
            }
        }
        view.set(s_off, view.get(s_off) & !F0LOST);
        Self::write_fifo_status(view, s_off, fifo.len(), size);
    }

    /// Update `TXFQS`/`TXBRP` from request/complete state.
    fn update_tx_status(&self, view: &mut Registers<&mut [u32]>) {
        let txbc = view.get(TXBC);
        let total = (((txbc >> 16) & 0x3F) + ((txbc >> 24) & 0x7F)) as u32;
        let used = (self.tx_pending | self.tx_outstanding).count_ones();
        view.set(TXFQS, (view.get(TXFQS) & !0x3F) | free_count(total, used));
        view.set(TXBRP, self.tx_pending | self.tx_outstanding);
    }
}

/// Free TX slots, capped at the 6-bit field.
fn free_count(total: u32, used: u32) -> u32 {
    total.saturating_sub(used).min(0x3F)
}

impl Behavior for FdcanBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        match offset {
            IR => {
                // Write-1-to-clear the pending interrupt flags.
                view.set(IR, view.get(IR) & !word);
                return view.get(IR);
            }
            TXBAR => {
                // Request bits live in `tx_pending`/`TXBRP`; the register
                // itself reads back zero so a later request cannot re-fire
                // stale bits through the byte-lane merge.
                self.tx_pending |= word;
                return 0;
            }
            RXF0A => {
                self.acknowledge_fifo(view, 0, (word & 0x7) as usize);
                return 0;
            }
            RXF1A => {
                self.acknowledge_fifo(view, 1, (word & 0x7) as usize);
                return 0;
            }
            SIDFC | TXBC | RXF0C | RXF1C | TXESC | RXESC | GFC => {
                if !Self::config_mode(view) {
                    // Locked: configuration ignores writes outside INIT+CCE.
                    return view.get(offset);
                }
                if offset == SIDFC {
                    self.filters.clear();
                }
                word
            }
            _ => word,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Admit queued frames whose filters are ready; defer the rest.
        self.refresh_filters(view);
        let mut deferred = VecDeque::new();
        while let Some(frame) = self.rx_queue.pop_front() {
            match self.classify(view, &frame) {
                Some(RxTarget::Reject) => {}
                Some(target) => self.insert(view, u32::from(target == RxTarget::Fifo1), frame),
                None => deferred.push_back(frame),
            }
        }
        self.rx_queue = deferred;
        // Request elements for newly pending TX buffers.
        let mut fresh = self.tx_pending & !self.tx_outstanding;
        if fresh != 0 {
            if self.mailbox.is_some() {
                let txbc = view.get(TXBC);
                let tbsa = self.ram_addr(txbc);
                let elen = 8 + Self::data_bytes(view, true, 0);
                let mb = self.mailbox.clone().expect("checked");
                let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
                let mut bit = 0;
                while fresh != 0 {
                    if fresh & 1 != 0 {
                        mb.reads.push(FdcanRead {
                            addr: tbsa.wrapping_add(bit * elen as u32),
                            len: elen.min(255) as u8,
                            kind: 0,
                            index: bit,
                        });
                        self.tx_outstanding |= 1 << bit.min(31);
                    }
                    bit += 1;
                    fresh >>= 1;
                }
            }
        }
        // Complete TX buffers whose elements arrived.
        if let Some(mb) = self.mailbox.clone() {
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            self.drain_filter_results(&mut mb);
            let mut done = Vec::new();
            mb.results.retain(|r| {
                if r.kind != 0 || r.bytes.len() < 8 {
                    return true;
                }
                let t0 = u32::from_le_bytes([r.bytes[0], r.bytes[1], r.bytes[2], r.bytes[3]]);
                let t1 = u32::from_le_bytes([r.bytes[4], r.bytes[5], r.bytes[6], r.bytes[7]]);
                let n = dlc_bytes((t1 >> 16) & 0xF).min(r.bytes.len().saturating_sub(8));
                let extended = t0 & (1 << 31) != 0;
                // Standard IDs live at T0[28:18]; extended use all 29 bits.
                let id = if extended {
                    t0 & 0x1FFF_FFFF
                } else {
                    (t0 >> 18) & 0x7FF
                };
                done.push((
                    r.index,
                    CanFrame {
                        id,
                        extended,
                        remote: t0 & (1 << 30) != 0,
                        fd: t1 & (1 << 21) != 0,
                        brs: t1 & (1 << 20) != 0,
                        data: r.bytes[8..8 + n].to_vec(),
                    },
                ));
                false
            });
            drop(mb);
            for (idx, frame) in done {
                if idx < 32 {
                    self.tx_pending &= !(1 << idx);
                    self.tx_outstanding &= !(1 << idx);
                    view.set(TXBTO, view.get(TXBTO) | (1 << idx));
                }
                Self::raise_ir(view, IR_TC);
                if view.get(TEST) & TEST_LBCK != 0 {
                    self.rx_queue.push_back(frame);
                }
            }
        }
        self.update_tx_status(view);
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(IR) & view.get(IE) != 0)
    }

    fn reset(&mut self) {
        self.tx_pending = 0;
        self.tx_outstanding = 0;
        self.rx_queue.clear();
        self.fifo0.clear();
        self.fifo1.clear();
        self.filters.clear();
        self.filters_outstanding = false;
    }
}

/// Cut-down FDCAN (G0/G4/H5/U5/L5): register window that does not fault.
/// Full TX/RX needs RM-verified fixed message-RAM counts the SVDs do not
/// carry (filter/FIFO depths, element strides), so these windows stay
/// round-trip latches until a written customer scenario funds the
/// per-family tables — the WP-B6 pattern.
#[derive(Debug)]
pub struct FdcanStub {
    inner: RegisterFile<NoBehavior>,
    name: &'static str,
}

impl FdcanStub {
    fn for_device<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, NoBehavior),
            name,
        }
    }

    /// A latch-only `FDCAN1` for cut-down tables.
    #[must_use]
    pub fn fdcan1_stub_for<D: Device>() -> Self {
        Self::for_device::<D>("FDCAN1", "fdcan1")
    }

    /// A latch-only `FDCAN2` for cut-down tables.
    #[must_use]
    pub fn fdcan2_stub_for<D: Device>() -> Self {
        Self::for_device::<D>("FDCAN2", "fdcan2")
    }
}

impl Peripheral for FdcanStub {
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
        self.name
    }
}

impl InterruptSource for FdcanStub {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

/// FDCAN message RAM: flat byte storage over an `FDCANx_RAM` window. The
/// SVD describes controller registers inside RAM windows on some families,
/// which would fault element traffic as reserved; this device owns the bytes
/// directly. Secure/nonsecure view pairs share one backing vector.
#[derive(Debug, Clone)]
pub struct FdcanRam {
    range: Region,
    bytes: Arc<Mutex<Vec<u8>>>,
}

impl FdcanRam {
    /// Storage for `len` bytes at `base`.
    #[must_use]
    pub fn at(base: u32, len: u32) -> Self {
        Self {
            range: Region::new(base, len),
            bytes: Arc::new(Mutex::new(vec![0; len as usize])),
        }
    }

    /// Another view of the same cells (secure/nonsecure pairs share RAM).
    #[must_use]
    pub fn alias(&self, base: u32, len: u32) -> Self {
        Self {
            range: Region::new(base, len),
            bytes: self.bytes.clone(),
        }
    }
}

impl Peripheral for FdcanRam {
    fn ranges(&self) -> &[Region] {
        std::slice::from_ref(&self.range)
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let idx = addr.wrapping_sub(self.range.base) as usize;
        self.bytes
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(idx)
            .copied()
            .ok_or(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let idx = addr.wrapping_sub(self.range.base) as usize;
        let mut bytes = self.bytes.lock().unwrap_or_else(|e| e.into_inner());
        if let Some(slot) = bytes.get_mut(idx) {
            *slot = val;
            Ok(())
        } else {
            Err(BusError::Reserved)
        }
    }

    fn reset(&mut self) {
        for b in self
            .bytes
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter_mut()
        {
            *b = 0;
        }
    }

    fn name(&self) -> &'static str {
        "fdcan_ram"
    }
}

impl InterruptSource for FdcanRam {}

#[derive(Debug)]
pub struct Fdcan {
    inner: RegisterFile<FdcanBehavior>,
    name: &'static str,
}

impl Fdcan {
    fn for_device<D: Device>(
        peripheral: &str,
        name: &'static str,
        mailbox: Option<Arc<Mutex<FdcanMailbox>>>,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(
                def,
                FdcanBehavior {
                    mailbox,
                    ..FdcanBehavior::default()
                },
            ),
            name,
        }
    }

    /// An `FDCAN1` sharing the bus mailbox (boards); without it TX/RX
    /// payloads cannot move.
    #[must_use]
    pub fn fdcan1_for<D: Device>(mailbox: Arc<Mutex<FdcanMailbox>>) -> Self {
        Self::for_device::<D>("FDCAN1", "fdcan1", Some(mailbox))
    }

    /// An `FDCAN2` sharing the bus mailbox (boards).
    #[must_use]
    pub fn fdcan2_for<D: Device>(mailbox: Arc<Mutex<FdcanMailbox>>) -> Self {
        Self::for_device::<D>("FDCAN2", "fdcan2", Some(mailbox))
    }

    /// Point the controller at its message RAM: configuration words hold
    /// 16-bit offsets into it. Boards pass the SVD RAM window base.
    #[must_use]
    pub fn with_msg_ram(mut self, base: u32) -> Self {
        self.inner.behavior_mut().msg_ram_base = base;
        self
    }

    /// Queue a host-side received frame (tests, debugger inject).
    pub fn inject_frame(&mut self, frame: CanFrame) {
        self.inner.behavior_mut().rx_queue.push_back(frame);
    }
}

impl Peripheral for Fdcan {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if let Some(outcome) = test_ir_ie_write(&mut self.inner, addr, val) {
            return outcome;
        }
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        self.name
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Fdcan {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }

    fn lines(&self) -> u32 {
        self.inner.lines()
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.inner.exception_for_line(line)
    }
}

/// Cut-down M_CAN (G0/G4/H5/U5/L5): fixed message-RAM layout, no
/// pointer registers. Verified against the STM32G0/U5/H5/L5 HAL drivers,
/// which share one layout: 28 standard filters at +0 (4B), 8 extended at
/// +112 (8B), RX FIFO0 at +176 (3 x 72B), RX FIFO1 at +392 (3 x 72B), TX
/// event FIFO skipped, TX FIFO/queue at +632 (3 x 72B). Elements are 8
/// header bytes + up to 64 payload bytes, header words compatible with the
/// full layout above.
///
/// Register offsets: `CCCR` 0x18, `TEST` 0x10, `IR` 0x50, `IE` 0x54,
/// `RXGFC` 0x80 (`RRFE`0/`RRFS`1/`ANFE`[3:2]/`ANFS`[5:4]/`F1OM`8/`F0OM`9/
/// `LSS`[20:16]/`LSE`[27:24]), `RXF0S` 0x90 (`F0FL`[3:0]/`F0GI`[9:8]/
/// `F0PI`[17:16]/`F0F`24/`RF0L`25), `RXF0A` 0x94, `RXF1S`/`RXF1A`
/// 0x98/0x9C, `TXBC` 0xC0 (`TFQM`24), `TXFQS` 0xC4 (`TFFL`[2:0]/`TFGI`8/
/// `TFQPI`16), `TXBRP` 0xC8, `TXBAR` 0xCC, `TXBCR` 0xD0, `TXBTO` 0xD4.
const CUT_RXGFC: u32 = 0x80;
const CUT_RXF0S: u32 = 0x90;
const CUT_RXF0A: u32 = 0x94;
const CUT_RXF1S: u32 = 0x98;
const CUT_RXF1A: u32 = 0x9C;
const CUT_TXBC: u32 = 0xC0;
const CUT_TXFQS: u32 = 0xC4;
const CUT_TXBRP: u32 = 0xC8;
const CUT_TXBAR: u32 = 0xCC;
const CUT_TXBTO: u32 = 0xD4;

/// Fixed message-RAM geometry (bytes), shared by all cut-down parts.
const CUT_STD_N: usize = 28;
const CUT_EXT_N: usize = 8;
const CUT_FIFO_N: usize = 3;
const CUT_TX_N: usize = 3;
const CUT_ELEN: usize = 72;
const CUT_FLSSA: u32 = 0;
const CUT_FLESA: u32 = 112;
const CUT_RF0SA: u32 = 176;
const CUT_RF1SA: u32 = 392;
const CUT_TFQSA: u32 = 632;

/// Extended-ID filter element decision (words as the HAL builds them:
/// `W1 = config<<29 | id1`, `W2 = type<<30 | id2`).
fn ext_filter_match(w1: u32, w2: u32, id: u32) -> Option<RxTarget> {
    let eft = (w2 >> 30) & 0x3;
    let efec = (w1 >> 29) & 0x7;
    let efid1 = w1 & 0x1FFF_FFFF;
    let efid2 = w2 & 0x1FFF_FFFF;
    let hit = match eft {
        0 | 3 => efid1 <= id && id <= efid2,
        1 => id == efid1 || id == efid2,
        2 => (id ^ efid1) & efid2 == 0,
        _ => false,
    };
    if !hit {
        return None;
    }
    match efec {
        1 | 5 | 6 => Some(RxTarget::Fifo0),
        2 | 7 => Some(RxTarget::Fifo1),
        _ => Some(RxTarget::Reject),
    }
}

#[derive(Debug, Default)]
struct FdcanCutBehavior {
    /// Message-RAM base (board-wired silicon address).
    msg_ram_base: u32,
    /// Requested TX buffer bits without an outstanding element read.
    tx_pending: u32,
    /// TX buffer bits with an outstanding element read.
    tx_outstanding: u32,
    /// TX put/get indices for `TFQPI`/`TFGI`.
    tx_put: u32,
    tx_get: u32,
    /// Cached filter words.
    std_filters: Vec<u32>,
    ext_filters: Vec<(u32, u32)>,
    /// A filter read is in flight.
    filters_outstanding: bool,
    /// Received frames awaiting FIFO insertion.
    rx_queue: VecDeque<CanFrame>,
    /// RX FIFO0/1 contents and put counters (for get/put indices).
    fifo0: VecDeque<CanFrame>,
    fifo1: VecDeque<CanFrame>,
    put0: u32,
    put1: u32,
    mailbox: Option<Arc<Mutex<FdcanMailbox>>>,
}

impl FdcanCutBehavior {
    fn raise_ir(view: &mut Registers<&mut [u32]>, bit: u32) {
        view.set(IR, view.get(IR) | (1 << bit));
    }

    /// `(LSS, LSE, ANFS, ANFE, F0OM, F1OM, RRFE, RRFS)` from `RXGFC`.
    fn rxgfc(view: &Registers<&mut [u32]>) -> (usize, usize, u32, u32, bool, bool, bool, bool) {
        let w = view.get(CUT_RXGFC);
        (
            ((w >> 16) & 0x1F) as usize,
            ((w >> 24) & 0xF) as usize,
            (w >> 4) & 0x3,
            (w >> 2) & 0x3,
            w & (1 << 9) != 0,
            w & (1 << 8) != 0,
            w & 1 != 0,
            w & (1 << 1) != 0,
        )
    }

    /// Queue a filter refresh when counts are nonzero and nothing cached.
    fn refresh_filters(&mut self, view: &mut Registers<&mut [u32]>) {
        let (lss, lse, _, _, _, _, _, _) = Self::rxgfc(view);
        if (lss == 0 && lse == 0)
            || (!self.std_filters.is_empty() || !self.ext_filters.is_empty())
            || self.filters_outstanding
        {
            return;
        }
        if let Some(mb) = &self.mailbox {
            mb.lock()
                .unwrap_or_else(|e| e.into_inner())
                .reads
                .push(FdcanRead {
                    addr: self.msg_ram_base.wrapping_add(CUT_FLSSA),
                    len: (lss * 4 + lse * 8).min(255) as u8,
                    kind: 2,
                    index: ((lss as u32) << 16) | lse as u32,
                });
            self.filters_outstanding = true;
        }
    }

    /// Consume arrived cut-down filter words.
    fn drain_filter_results(&mut self, mb: &mut FdcanMailbox) {
        let mut i = 0;
        while i < mb.results.len() {
            if mb.results[i].kind != 2 {
                i += 1;
                continue;
            }
            let r = mb.results.remove(i).expect("indexed");
            let lss = ((r.index >> 16) & 0xFFFF) as usize;
            let words: Vec<u32> = r
                .bytes
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect();
            self.std_filters = words.iter().take(lss).copied().collect();
            self.ext_filters = words
                .iter()
                .skip(lss)
                .copied()
                .collect::<Vec<u32>>()
                .chunks_exact(2)
                .map(|c| (c[0], c[1]))
                .collect();
            self.filters_outstanding = false;
        }
    }

    /// Classify one frame; `None` while filter words are in flight.
    fn classify(&self, view: &mut Registers<&mut [u32]>, frame: &CanFrame) -> Option<RxTarget> {
        let (lss, lse, anfs, anfe, _, _, rrfe, rrfs) = Self::rxgfc(view);
        let global = |fifo01: u32| match fifo01 {
            0 | 1 => RxTarget::Fifo0,
            2 => RxTarget::Fifo1,
            _ => RxTarget::Reject,
        };
        if frame.remote {
            if (frame.extended && rrfe) || (!frame.extended && rrfs) {
                return Some(RxTarget::Fifo0);
            }
            return Some(RxTarget::Reject);
        }
        if !frame.extended {
            if lss > 0 {
                if self.std_filters.is_empty() {
                    return None;
                }
                for f in self.std_filters.iter().take(lss) {
                    if let Some(target) = std_filter_match(*f, frame.id) {
                        return Some(target);
                    }
                }
            }
            return Some(global(anfs));
        }
        if lse > 0 {
            if self.ext_filters.is_empty() {
                return None;
            }
            for (w1, w2) in self.ext_filters.iter().take(lse) {
                if let Some(target) = ext_filter_match(*w1, *w2, frame.id) {
                    return Some(target);
                }
            }
        }
        Some(global(anfe))
    }

    /// Insert one classified frame: payload to message RAM, flags live.
    fn insert(&mut self, view: &mut Registers<&mut [u32]>, fifo_no: u32, frame: CanFrame) {
        let (s_off, a_off, base_off) = if fifo_no == 0 {
            (CUT_RXF0S, CUT_RXF0A, CUT_RF0SA)
        } else {
            (CUT_RXF1S, CUT_RXF1A, CUT_RF1SA)
        };
        let _ = a_off;
        let overwrite = if fifo_no == 0 {
            view.get(CUT_RXGFC) & (1 << 9) != 0
        } else {
            view.get(CUT_RXGFC) & (1 << 8) != 0
        };
        let fifo = if fifo_no == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        if fifo.len() >= CUT_FIFO_N {
            view.set(s_off, view.get(s_off) | F0F);
            Self::raise_ir(view, if fifo_no == 0 { IR_RF0F } else { IR_RF1F });
            if !overwrite {
                view.set(s_off, view.get(s_off) | F0LOST);
                Self::raise_ir(view, if fifo_no == 0 { IR_RF0L } else { IR_RF1L });
                return;
            }
            fifo.pop_front();
        }
        let put = if fifo_no == 0 {
            &mut self.put0
        } else {
            &mut self.put1
        };
        let idx = *put % CUT_FIFO_N as u32;
        *put = put.wrapping_add(1);
        let mut bytes = vec![0u8; CUT_ELEN];
        bytes[0..4].copy_from_slice(
            &(frame.id | (u32::from(frame.remote) << 30) | (u32::from(frame.extended) << 31))
                .to_le_bytes(),
        );
        bytes[4..8].copy_from_slice(
            &((bytes_dlc(frame.data.len()) << 16)
                | (u32::from(frame.brs) << 20)
                | (u32::from(frame.fd) << 21))
                .to_le_bytes(),
        );
        let n = frame.data.len().min(64);
        bytes[8..8 + n].copy_from_slice(&frame.data[..n]);
        if let Some(mb) = &self.mailbox {
            mb.lock().unwrap_or_else(|e| e.into_inner()).writes.push((
                self.msg_ram_base
                    .wrapping_add(base_off + idx * CUT_ELEN as u32),
                bytes,
            ));
        }
        fifo.push_back(frame);
        Self::write_fifo_status(view, s_off, fifo.len(), *put);
        Self::raise_ir(view, if fifo_no == 0 { IR_RF0N } else { IR_RF1N });
    }

    /// Rewrite an `RxFnS` word: fill count, get index (oldest = put - fill),
    /// put index; the full flag recomputes, the lost flag sticks.
    fn write_fifo_status(view: &mut Registers<&mut [u32]>, s_off: u32, fill: usize, put: u32) {
        let mut s = view.get(s_off) & !(0x0003_030F | F0F);
        s |= (fill.min(3)) as u32;
        s |= (((put.wrapping_sub(fill as u32)) % 3) & 0x3) << 8;
        s |= ((put % 3) & 0x3) << 16;
        if fill >= CUT_FIFO_N {
            s |= F0F;
        }
        view.set(s_off, s);
    }

    fn acknowledge_fifo(&mut self, view: &mut Registers<&mut [u32]>, fifo_no: u32, n: usize) {
        let s_off = if fifo_no == 0 { CUT_RXF0S } else { CUT_RXF1S };
        let put = if fifo_no == 0 { self.put0 } else { self.put1 };
        let fifo = if fifo_no == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        for _ in 0..n.min(3) {
            if fifo.pop_front().is_none() {
                break;
            }
        }
        view.set(s_off, view.get(s_off) & !F0LOST);
        Self::write_fifo_status(view, s_off, fifo.len(), put);
    }

    /// Update `TXFQS`/`TXBRP` from request/complete state.
    fn update_tx_status(&self, view: &mut Registers<&mut [u32]>) {
        let used = (self.tx_pending | self.tx_outstanding).count_ones();
        let mut fqs = view.get(CUT_TXFQS) & !0x0003_0307;
        fqs |= (CUT_TX_N as u32).saturating_sub(used).min(3);
        fqs |= (self.tx_get % 3) << 8;
        fqs |= (self.tx_put % 3) << 16;
        view.set(CUT_TXFQS, fqs);
        view.set(CUT_TXBRP, self.tx_pending | self.tx_outstanding);
    }
}

impl Behavior for FdcanCutBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        match offset {
            CUT_TXBAR => {
                self.tx_pending |= word & 0x7;
                self.tx_put = self.tx_put.wrapping_add((word & 0x7).count_ones());
                return word;
            }
            CUT_RXF0A => {
                self.acknowledge_fifo(view, 0, (word & 0x7) as usize);
                return 0;
            }
            CUT_RXF1A => {
                self.acknowledge_fifo(view, 1, (word & 0x7) as usize);
                return 0;
            }
            CUT_RXGFC => {
                self.std_filters.clear();
                self.ext_filters.clear();
                word
            }
            _ => word,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        self.refresh_filters(view);
        let mut deferred = VecDeque::new();
        while let Some(frame) = self.rx_queue.pop_front() {
            match self.classify(view, &frame) {
                Some(RxTarget::Reject) => {}
                Some(target) => self.insert(view, u32::from(target == RxTarget::Fifo1), frame),
                None => deferred.push_back(frame),
            }
        }
        self.rx_queue = deferred;
        let mut fresh = self.tx_pending & !self.tx_outstanding;
        if fresh != 0 {
            if let Some(mb) = self.mailbox.clone() {
                let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
                let mut bit = 0;
                while fresh != 0 {
                    if fresh & 1 != 0 {
                        mb.reads.push(FdcanRead {
                            addr: self
                                .msg_ram_base
                                .wrapping_add(CUT_TFQSA + bit * CUT_ELEN as u32),
                            len: CUT_ELEN.min(255) as u8,
                            kind: 0,
                            index: bit,
                        });
                        self.tx_outstanding |= 1 << bit.min(31);
                    }
                    bit += 1;
                    fresh >>= 1;
                }
            }
        }
        if let Some(mb) = self.mailbox.clone() {
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            self.drain_filter_results(&mut mb);
            let mut done = Vec::new();
            mb.results.retain(|r| {
                if r.kind != 0 || r.bytes.len() < 8 {
                    return true;
                }
                let t0 = u32::from_le_bytes([r.bytes[0], r.bytes[1], r.bytes[2], r.bytes[3]]);
                let t1 = u32::from_le_bytes([r.bytes[4], r.bytes[5], r.bytes[6], r.bytes[7]]);
                let n = dlc_bytes((t1 >> 16) & 0xF).min(r.bytes.len().saturating_sub(8));
                let extended = t0 & (1 << 31) != 0;
                // Standard IDs live at T0[28:18]; extended use all 29 bits.
                let id = if extended {
                    t0 & 0x1FFF_FFFF
                } else {
                    (t0 >> 18) & 0x7FF
                };
                done.push((
                    r.index,
                    CanFrame {
                        id,
                        extended,
                        remote: t0 & (1 << 30) != 0,
                        fd: t1 & (1 << 21) != 0,
                        brs: t1 & (1 << 20) != 0,
                        data: r.bytes[8..8 + n].to_vec(),
                    },
                ));
                false
            });
            drop(mb);
            for (idx, frame) in done {
                if idx < 32 {
                    self.tx_pending &= !(1 << idx);
                    self.tx_outstanding &= !(1 << idx);
                    self.tx_get = self.tx_get.wrapping_add(1);
                    view.set(CUT_TXBTO, view.get(CUT_TXBTO) | (1 << idx));
                }
                Self::raise_ir(view, IR_TC);
                if view.get(TEST) & TEST_LBCK != 0 {
                    self.rx_queue.push_back(frame);
                }
            }
        }
        self.update_tx_status(view);
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(IR) & view.get(IE) != 0)
    }

    fn reset(&mut self) {
        self.tx_pending = 0;
        self.tx_outstanding = 0;
        self.tx_put = 0;
        self.tx_get = 0;
        self.std_filters.clear();
        self.ext_filters.clear();
        self.filters_outstanding = false;
        self.rx_queue.clear();
        self.fifo0.clear();
        self.fifo1.clear();
        self.put0 = 0;
        self.put1 = 0;
    }
}

/// Cut-down FDCAN (G0/G4/H5/U5/L5): fixed message-RAM layout, `TEST`/`IR`/`IE`
/// handled like the full variant.
#[derive(Debug)]
pub struct FdcanCut {
    inner: RegisterFile<FdcanCutBehavior>,
    name: &'static str,
}

impl FdcanCut {
    fn for_device<D: Device>(
        peripheral: &str,
        name: &'static str,
        mailbox: Option<Arc<Mutex<FdcanMailbox>>>,
        msg_ram_base: u32,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(
                def,
                FdcanCutBehavior {
                    msg_ram_base,
                    mailbox,
                    ..FdcanCutBehavior::default()
                },
            ),
            name,
        }
    }

    /// A cut-down `FDCAN1` with message RAM at `msg_ram_base`.
    #[must_use]
    pub fn fdcan1_for<D: Device>(mailbox: Arc<Mutex<FdcanMailbox>>, msg_ram_base: u32) -> Self {
        Self::for_device::<D>("FDCAN1", "fdcan1", Some(mailbox), msg_ram_base)
    }

    /// A cut-down `FDCAN2` with message RAM at `msg_ram_base`.
    #[must_use]
    pub fn fdcan2_for<D: Device>(mailbox: Arc<Mutex<FdcanMailbox>>, msg_ram_base: u32) -> Self {
        Self::for_device::<D>("FDCAN2", "fdcan2", Some(mailbox), msg_ram_base)
    }

    /// Queue a host-side received frame (tests, debugger inject).
    pub fn inject_frame(&mut self, frame: CanFrame) {
        self.inner.behavior_mut().rx_queue.push_back(frame);
    }
}

impl Peripheral for FdcanCut {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if let Some(outcome) = test_ir_ie_write(&mut self.inner, addr, val) {
            return outcome;
        }
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        self.name
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for FdcanCut {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }

    fn lines(&self) -> u32 {
        self.inner.lines()
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.inner.exception_for_line(line)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;

    fn base(dev: &mut Fdcan) -> u32 {
        dev.ranges()[0].base
    }

    fn word(dev: &mut Fdcan, off: u32) -> u32 {
        let base = base(dev);
        let mut v = 0u32;
        for i in 0..4 {
            v |= u32::from(dev.read(base + off + i).unwrap()) << (8 * i);
        }
        v
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(dev: &mut Fdcan, off: u32, v: u32) {
        let base = base(dev);
        for (i, b) in v.to_le_bytes().iter().enumerate() {
            dev.write(base + off + i as u32, *b).unwrap();
        }
    }

    fn with_mailbox() -> (Fdcan, Arc<Mutex<FdcanMailbox>>) {
        let mb = Arc::new(Mutex::new(FdcanMailbox::default()));
        (
            Fdcan::fdcan1_for::<Stm32h743>(mb.clone()).with_msg_ram(0x2000_0000),
            mb,
        )
    }

    /// Serve one bus drain by hand: reads come from `ram`, writes land there.
    fn drain(mb: &Arc<Mutex<FdcanMailbox>>, ram: &mut Vec<u8>, base: u32) {
        let mut mb = mb.lock().unwrap();
        let reads: Vec<FdcanRead> = mb.reads.drain(..).collect();
        for r in reads {
            let off = r.addr.wrapping_sub(base) as usize;
            mb.results.push_back(FdcanResult {
                kind: r.kind,
                index: r.index,
                bytes: ram[off..off + r.len as usize].to_vec(),
            });
        }
        let writes: Vec<(u32, Vec<u8>)> = mb.writes.drain(..).collect();
        for (addr, bytes) in writes {
            let off = addr.wrapping_sub(base) as usize;
            ram[off..off + bytes.len()].copy_from_slice(&bytes);
        }
    }

    #[test]
    fn config_gate_blocks_writes_outside_init() {
        let (mut f, _) = with_mailbox();
        let reset = word(&mut f, SIDFC);
        write_word(&mut f, SIDFC, 0x1234_5678);
        assert_eq!(word(&mut f, SIDFC), reset);
        write_word(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        write_word(&mut f, SIDFC, 0x1234_5678);
        assert_eq!(word(&mut f, SIDFC), 0x1234_5678);
    }

    #[test]
    fn tx_completes_loopback_into_fifo0() {
        let (mut f, mb) = with_mailbox();
        // Fake message RAM at 0x20000000.
        let mut ram = vec![0u8; 0x1000];
        write_word(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        write_word(&mut f, TXBC, 0x0100 | (2 << 24));
        write_word(&mut f, TXESC, 0);
        write_word(&mut f, RXF0C, (4 << 16) | 0x0200);
        // No standard filters: classify by the global rule immediately.
        write_word(&mut f, SIDFC, 0x0300);
        write_word(&mut f, TEST, TEST_LBCK);
        // TX element 0 at TBSA (offset 0x100): std ID 0x123 at T0[28:18].
        let mut el = vec![0x00, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x08, 0x00];
        el.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]);
        ram[0x100..0x110].copy_from_slice(&el);
        write_word(&mut f, TXBAR, 1);
        f.tick(1);
        drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        assert_ne!(word(&mut f, TXBTO) & 1, 0);
        assert_eq!(word(&mut f, TXBRP) & 1, 0);
        assert_ne!(word(&mut f, IR) & (1 << IR_TC), 0);
        f.tick(1);
        drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        // Loopback frame classified (no filters: GFC reset accepts FIFO0).
        assert_eq!(word(&mut f, RXF0S) & 0x7F, 1);
        assert_ne!(word(&mut f, IR) & (1 << IR_RF0N), 0);
        // Payload landed in message RAM at F0SA (header first, then data).
        assert_eq!(
            &ram[0x208..0x210],
            &[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]
        );
    }

    #[test]
    fn rx_reject_without_matching_filter() {
        let (mut f, mb) = with_mailbox();
        let mut ram = vec![0u8; 0x1000];
        write_word(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        // One exact filter for 0x100 -> FIFO0, at FLSSA 0x20000300.
        // SFT=01 (dual), SFEC=1 (FIFO0), SFID1=0x100.
        write_word(&mut f, SIDFC, (1 << 16) | 0x0300);
        write_word(&mut f, RXF0C, (4 << 16) | 0x0200);
        // GFC: reject non-matching standard and extended.
        write_word(&mut f, GFC, (3 << 2) | (3 << 4));
        f.inject_frame(CanFrame {
            id: 0x200,
            extended: false,
            remote: false,
            fd: false,
            brs: false,
            data: vec![1, 2, 3],
        });
        f.tick(1);
        // Serve the filter words: one exact-match element for 0x100.
        {
            let mut mb = mb.lock().unwrap();
            assert_eq!(mb.reads.len(), 1);
            let r = mb.reads.pop().unwrap();
            assert_eq!((r.kind, r.index), (1, 0));
            mb.results.push_back(FdcanResult {
                kind: 1,
                index: 0,
                bytes: vec![0x00, 0x01, 0x88, 0x48],
            });
        }
        f.tick(1);
        // 0x200 misses the 0x100 filter and ANFS=3 rejects: FIFO stays empty.
        assert_eq!(word(&mut f, RXF0S) & 0x7F, 0);
        assert_eq!(word(&mut f, IR) & (1 << IR_RF0N), 0);
    }

    use crate::device::svd::stm32g0c1::Stm32g0c1;

    /// Cut-down device with message RAM at `base` (test double memory).
    fn cut(base: u32) -> (FdcanCut, Arc<Mutex<FdcanMailbox>>) {
        let mb = Arc::new(Mutex::new(FdcanMailbox::default()));
        (FdcanCut::fdcan1_for::<Stm32g0c1>(mb.clone(), base), mb)
    }

    fn cut_word(dev: &mut FdcanCut, off: u32) -> u32 {
        let base = dev.ranges()[0].base;
        let mut v = 0u32;
        for i in 0..4 {
            v |= u32::from(dev.read(base + off + i).unwrap()) << (8 * i);
        }
        v
    }

    #[allow(clippy::cast_possible_truncation)]
    fn cut_write(dev: &mut FdcanCut, off: u32, v: u32) {
        let base = dev.ranges()[0].base;
        for (i, b) in v.to_le_bytes().iter().enumerate() {
            dev.write(base + off + i as u32, *b).unwrap();
        }
    }

    /// Serve one bus drain against `ram` at `base`.
    fn cut_drain(mb: &Arc<Mutex<FdcanMailbox>>, ram: &mut Vec<u8>, base: u32) {
        let mut mb = mb.lock().unwrap();
        let reads: Vec<FdcanRead> = mb.reads.drain(..).collect();
        for r in reads {
            let off = r.addr.wrapping_sub(base) as usize;
            mb.results.push_back(FdcanResult {
                kind: r.kind,
                index: r.index,
                bytes: ram[off..off + r.len as usize].to_vec(),
            });
        }
        let writes: Vec<(u32, Vec<u8>)> = mb.writes.drain(..).collect();
        for (addr, bytes) in writes {
            let off = addr.wrapping_sub(base) as usize;
            ram[off..off + bytes.len()].copy_from_slice(&bytes);
        }
    }

    #[test]
    fn cut_tx_loopback_with_mask_filter() {
        let (mut f, mb) = cut(0x2000_0000);
        let mut ram = vec![0u8; 0x400];
        cut_write(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        // RXGFC: one standard filter; accept non-matching into FIFO0.
        cut_write(&mut f, CUT_RXGFC, (1 << 16) | (1 << 4));
        cut_write(&mut f, TEST, TEST_LBCK);
        // Mask filter 0: pass IDs matching 0x120 & 0x7F0 (i.e. 0x120-0x12F).
        // SFT=10 (mask), SFEC=1 (FIFO0), ID1=0x120 filter, ID2=0x7F0 mask.
        let filt: u32 = (2 << 30) | (1 << 27) | (0x120 << 16) | 0x7F0;
        ram[0..4].copy_from_slice(&filt.to_le_bytes());
        // TX buffer 0 at TFQSA=632: std ID 0x123 (at T0[28:18]), DLC=2.
        let mut el = vec![0x00, 0x00, 0x8C, 0x04, 0x00, 0x00, 0x02, 0x00];
        el.extend_from_slice(&[0xDE, 0xAD]);
        ram[632..632 + 10].copy_from_slice(&el);
        cut_write(&mut f, CUT_TXBAR, 1);
        f.tick(1);
        cut_drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        assert_ne!(cut_word(&mut f, CUT_TXBTO) & 1, 0);
        f.tick(1);
        cut_drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        // 0x123 matches the mask filter: FIFO0 holds one, payload in RAM.
        assert_eq!(cut_word(&mut f, CUT_RXF0S) & 0xF, 1);
        assert_eq!(&ram[176 + 8..176 + 10], &[0xDE, 0xAD]);
        // A non-matching ID is rejected by the global rule... ANFS=1 here
        // accepts FIFO0, so reprogram to reject and retry via inject.
        cut_write(&mut f, CUT_RXGFC, (1 << 16) | (3 << 4));
        f.inject_frame(CanFrame {
            id: 0x200,
            extended: false,
            remote: false,
            fd: false,
            brs: false,
            data: vec![9],
        });
        f.tick(1);
        cut_drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        assert_eq!(cut_word(&mut f, CUT_RXF0S) & 0xF, 1);
    }

    #[test]
    fn cut_overwrite_and_blocking_modes() {
        let (mut f, mb) = cut(0x2000_0000);
        let mut ram = vec![0u8; 0x400];
        cut_write(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        cut_write(&mut f, CUT_RXGFC, 0);
        for id in [1u32, 2, 3, 4] {
            f.inject_frame(CanFrame {
                id,
                extended: false,
                remote: false,
                fd: false,
                brs: false,
                data: vec![],
            });
        }
        for _ in 0..6 {
            f.tick(1);
            cut_drain(&mb, &mut ram, 0x2000_0000);
        }
        // Blocking (F0OM=0): 3 kept, full + lost flagged.
        assert_eq!(cut_word(&mut f, CUT_RXF0S) & 0xF, 3);
        assert_ne!(cut_word(&mut f, CUT_RXF0S) & (1 << 24), 0);
        assert_ne!(cut_word(&mut f, CUT_RXF0S) & (1 << 25), 0);
        // Acknowledge two, room for one more.
        cut_write(&mut f, CUT_RXF0A, 2);
        assert_eq!(cut_word(&mut f, CUT_RXF0S) & 0xF, 1);
        // Overwrite mode keeps the newest three.
        cut_write(&mut f, CUT_RXGFC, 1 << 9);
        for id in [5u32, 6, 7, 8] {
            f.inject_frame(CanFrame {
                id,
                extended: false,
                remote: false,
                fd: false,
                brs: false,
                data: vec![],
            });
        }
        for _ in 0..8 {
            f.tick(1);
            cut_drain(&mb, &mut ram, 0x2000_0000);
        }
        assert_eq!(cut_word(&mut f, CUT_RXF0S) & 0xF, 3);
    }

    #[test]
    fn ir_is_write_one_to_clear_and_gates_pending() {
        let (mut f, mb) = with_mailbox();
        let mut ram = vec![0u8; 0x1000];
        write_word(&mut f, CCCR, CCCR_INIT | CCCR_CCE | (1 << 7));
        write_word(&mut f, RXF0C, (4 << 16) | 0x0200);
        write_word(&mut f, SIDFC, 0x0300);
        f.inject_frame(CanFrame {
            id: 0x123,
            extended: false,
            remote: false,
            fd: false,
            brs: false,
            data: vec![],
        });
        f.tick(1);
        drain(&mb, &mut ram, 0x2000_0000);
        f.tick(1);
        // GFC reset accepts into FIFO0: RF0N raised.
        assert_ne!(word(&mut f, IR) & (1 << IR_RF0N), 0);
        write_word(&mut f, IE, 0xFFFF_FFFF);
        assert_eq!(f.pending(), 1);
        write_word(&mut f, IR, 0xFFFF_FFFF);
        assert_eq!(word(&mut f, IR), 0);
        assert_eq!(f.pending(), 0);
    }
}

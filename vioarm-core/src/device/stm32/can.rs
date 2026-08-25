//! The STM32 `bxCAN` controller (`CAN1`/`CAN2`), driven by the SVD-generated
//! register tables.
//!
//! The SVD describes the register words; this behavior runs the parts it
//! cannot: the init handshake (`MCR.INRQ` ↔ `MSR.INAK`), the transmit path (a
//! mailbox loaded with an identifier and data, then `TXRQ`, is sent after
//! `BTR`-derived bit timing and latches `RQCP`/`TXOK`/`TME`), the receive
//! FIFOs (`FMP`/`FULL`/`FOVR`, released by `RFOM`), and the four interrupt
//! lines (TX, RX0, RX1, SCE). Filter banks at `CAN1+0x200` (`0x40006600`,
//! `0x200` window) implement `FMR`/`FM1R`/`FS1R`/`FFA1R`/`FA1R` control and
//! `FR1`/`FR2` per bank with `CAN2SB` sharing; received frames are matched
//! against active banks and routed to `FIFO0`/`FIFO1` per `FFA1R` (drop on
//! no match, accept-all when no filter active for backward compat).
//!
//! The controller raises four NVIC lines. The SVD lists them in order `TX`,
//! `RX0`, `RX1`, `SCE`, so the device-local lines 0..3 map to exceptions
//! `16 + irqs[line].number` through the register file.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const MCR: u32 = 0x00;
const MSR: u32 = 0x04;
const TSR: u32 = 0x08;
const RF0R: u32 = 0x0C;
const RF1R: u32 = 0x10;
const IER: u32 = 0x14;
const ESR: u32 = 0x18;
const BTR: u32 = 0x1C;
const TI0R: u32 = 0x180;
const TDT0R: u32 = 0x184;
const TDL0R: u32 = 0x188;
const TDH0R: u32 = 0x18C;
const TI1R: u32 = 0x190;
const TI2R: u32 = 0x1A0;
const RI0R: u32 = 0x1B0;
const RDT0R: u32 = 0x1B4;
const RDL0R: u32 = 0x1B8;
const RDH0R: u32 = 0x1BC;
const RI1R: u32 = 0x1C0;
const RDT1R: u32 = 0x1C4;
const RDL1R: u32 = 0x1C8;
const RDH1R: u32 = 0x1CC;

const INRQ: u32 = 1 << 0;
const RFLM: u32 = 1 << 3;
const RESET: u32 = 1 << 15;
const INAK: u32 = 1 << 0;
const RQCP0: u32 = 1 << 0;
const TXOK0: u32 = 1 << 1;
const ALST0: u32 = 1 << 2;
const TERR0: u32 = 1 << 3;
const ABRQ0: u32 = 1 << 7;
const RQCP1: u32 = 1 << 8;
const TXOK1: u32 = 1 << 9;
const ALST1: u32 = 1 << 10;
const TERR1: u32 = 1 << 11;
const ABRQ1: u32 = 1 << 15;
const RQCP2: u32 = 1 << 16;
const TXOK2: u32 = 1 << 17;
const ALST2: u32 = 1 << 18;
const TERR2: u32 = 1 << 19;
const ABRQ2: u32 = 1 << 23;
const FMP_MASK: u32 = 0b11;
const FULL0: u32 = 1 << 3;
const FOVR0: u32 = 1 << 4;
const RFOM0: u32 = 1 << 5;
const FULL1: u32 = 1 << 3;
const FOVR1: u32 = 1 << 4;
const RFOM1: u32 = 1 << 5;
const TMEIE: u32 = 1 << 0;
const FMPIE0: u32 = 1 << 1;
const FFIE0: u32 = 1 << 2;
const FOVIE0: u32 = 1 << 3;
const FMPIE1: u32 = 1 << 4;
const FFIE1: u32 = 1 << 5;
const FOVIE1: u32 = 1 << 6;
const LECIE: u32 = 1 << 11;
const ERRIE: u32 = 1 << 15;
const EWGF: u32 = 1 << 0;
const EPVF: u32 = 1 << 1;
const BOFF: u32 = 1 << 2;
const LEC: u32 = 0b111 << 4;
const TXRQ: u32 = 1 << 0;
const IDE: u32 = 1 << 2;
const DLC: u32 = 0b1111;

// Filter window layout (RM0008 §24.7, RM0090 §41.7). Offsets relative to
// filter base 0x200: FMR 0x00, FM1R 0x04, FS1R 0x0C, FFA1R 0x14, FA1R 0x1C,
// banks at 0x40 + 8*n (FR1, FR2). Indexes in the 0x80-word window.
const FMR_IDX: usize = 0x00 / 4;
const FM1R_IDX: usize = 0x04 / 4;
const FS1R_IDX: usize = 0x0C / 4;
const FFA1R_IDX: usize = 0x14 / 4;
const FA1R_IDX: usize = 0x1C / 4;
const BANK_BASE_IDX: usize = 0x40 / 4;

/// One captured frame: the four mailbox words exactly as firmware wrote them
/// (or as they will be read from the FIFO). Transmissions and injected frames
/// share this encoding, so `drain_output` and `drive_inputs` round-trip.
#[derive(Clone, Copy, Debug, Default)]
struct Frame {
    ti: u32,
    tdt: u32,
    tdl: u32,
    tdh: u32,
}

impl Frame {
    fn to_bytes(self) -> Vec<u8> {
        let mut out = Vec::with_capacity(16);
        out.extend_from_slice(&self.ti.to_le_bytes());
        out.extend_from_slice(&self.tdt.to_le_bytes());
        out.extend_from_slice(&self.tdl.to_le_bytes());
        out.extend_from_slice(&self.tdh.to_le_bytes());
        out
    }

    fn from_bytes(bytes: &[u8]) -> Self {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        Self {
            ti: w(0),
            tdt: w(1),
            tdl: w(2),
            tdh: w(3),
        }
    }
}

fn ti_reg(mb: u8) -> u32 {
    TI0R + 0x10u32 * u32::from(mb)
}

fn tdt_reg(mb: u8) -> u32 {
    TDT0R + 0x10u32 * u32::from(mb)
}

fn tdl_reg(mb: u8) -> u32 {
    TDL0R + 0x10u32 * u32::from(mb)
}

fn tdh_reg(mb: u8) -> u32 {
    TDH0R + 0x10u32 * u32::from(mb)
}

const fn rqcp_bit(mb: u8) -> u32 {
    match mb {
        0 => RQCP0,
        1 => RQCP1,
        _ => RQCP2,
    }
}

const fn txok_bit(mb: u8) -> u32 {
    match mb {
        0 => TXOK0,
        1 => TXOK1,
        _ => TXOK2,
    }
}

const fn abrq_bit(mb: u8) -> u32 {
    match mb {
        0 => ABRQ0,
        1 => ABRQ1,
        _ => ABRQ2,
    }
}

const fn fifo_reg(fifo: u8) -> u32 {
    if fifo == 0 {
        RF0R
    } else {
        RF1R
    }
}

const fn full_bit(fifo: u8) -> u32 {
    if fifo == 0 {
        FULL0
    } else {
        FULL1
    }
}

const fn fovr_bit(fifo: u8) -> u32 {
    if fifo == 0 {
        FOVR0
    } else {
        FOVR1
    }
}

/// The CAN core the SVD cannot express: init handshake, TX mailboxes, and the
/// two receive FIFOs, all with their level-triggered interrupt lines.
#[derive(Debug)]
struct CanBehavior {
    fifo0: Vec<Frame>,
    fifo1: Vec<Frame>,
    tx_queue: Vec<(u8, Frame)>,
    busy: [bool; 3],
    in_flight: Option<(u8, Frame)>,
    tx_timer: u64,
    rx_buffer: Vec<u8>,
    output: Vec<u8>,
    filter: Box<[u32; 0x80]>,
    is_can2: bool,
}

impl Default for CanBehavior {
    fn default() -> Self {
        Self {
            fifo0: Vec::new(),
            fifo1: Vec::new(),
            tx_queue: Vec::new(),
            busy: [false; 3],
            in_flight: None,
            tx_timer: 0,
            rx_buffer: Vec::new(),
            output: Vec::new(),
            filter: Box::new([0u32; 0x80]),
            is_can2: false,
        }
    }
}

impl CanBehavior {
    fn enter_init(&mut self, view: &mut Registers<&mut [u32]>) {
        self.fifo0.clear();
        self.fifo1.clear();
        self.tx_queue.clear();
        self.busy = [false; 3];
        self.in_flight = None;
        view.set(MSR, view.get(MSR) | INAK);
    }

    fn leave_init(view: &mut Registers<&mut [u32]>) {
        view.set(MSR, view.get(MSR) & !INAK);
    }

    fn request_tx(&mut self, view: &Registers<&mut [u32]>, mb: u8) {
        if view.get(MCR) & INRQ != 0 {
            return;
        }
        if self.busy[usize::from(mb)] {
            return;
        }
        let frame = Frame {
            ti: view.get(ti_reg(mb)),
            tdt: view.get(tdt_reg(mb)),
            tdl: view.get(tdl_reg(mb)),
            tdh: view.get(tdh_reg(mb)),
        };
        self.busy[usize::from(mb)] = true;
        if self.in_flight.is_none() {
            self.start_tx(view, mb, frame);
        } else {
            self.tx_queue.push((mb, frame));
        }
    }

    fn start_tx(&mut self, view: &Registers<&mut [u32]>, mb: u8, frame: Frame) {
        self.tx_timer = Self::frame_latency(view, &frame);
        self.in_flight = Some((mb, frame));
    }

    /// Wire time for a frame from `BTR`'s bit timing and its length (standard
    /// or extended, `DLC` data bytes).
    fn frame_latency(view: &Registers<&mut [u32]>, frame: &Frame) -> u64 {
        let btr = view.get(BTR);
        let brp = (btr & 0x3FF) + 1;
        let ts1 = ((btr >> 16) & 0xF) + 1;
        let ts2 = (btr >> 20) & 0x7;
        let bit_time = brp * (1 + ts1 + ts2);
        let dlc = frame.tdt & DLC;
        let bits = if frame.ti & IDE != 0 {
            64 + 8 * dlc
        } else {
            44 + 8 * dlc
        };
        u64::from(bits * bit_time)
    }

    fn complete_tx(&mut self, view: &mut Registers<&mut [u32]>, mb: u8, frame: Frame, ok: bool) {
        self.busy[usize::from(mb)] = false;
        let status = rqcp_bit(mb) | if ok { txok_bit(mb) } else { 0 };
        view.set(TSR, view.get(TSR) | status);
        if ok {
            self.output.extend(frame.to_bytes());
        }
    }

    fn abort_mailbox(&mut self, view: &mut Registers<&mut [u32]>, mb: u8) {
        self.tx_queue.retain(|(m, _)| *m != mb);
        if self.in_flight.as_ref().is_some_and(|(m, _)| *m == mb) {
            let (_, frame) = self.in_flight.take().unwrap();
            self.complete_tx(view, mb, frame, false);
        } else {
            self.busy[usize::from(mb)] = false;
            view.set(TSR, view.get(TSR) | rqcp_bit(mb));
        }
        if self.in_flight.is_none() {
            if let Some((mb, frame)) = self.tx_queue.first().copied() {
                self.tx_queue.remove(0);
                self.start_tx(view, mb, frame);
            }
        }
    }

    fn enqueue(&mut self, view: &mut Registers<&mut [u32]>, frame: Frame, fifo: u8) {
        let queue = if fifo == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        let locked = view.get(MCR) & RFLM != 0;
        if queue.len() < 2 {
            queue.push(frame);
        } else if !locked {
            // Full: overwrite the newest message and latch the overflow flag.
            queue[1] = frame;
            view.set(fifo_reg(fifo), view.get(fifo_reg(fifo)) | fovr_bit(fifo));
        }
        self.sync_fifo_flags(view, fifo);
    }

    fn release_front(&mut self, view: &mut Registers<&mut [u32]>, fifo: u8) {
        let queue = if fifo == 0 {
            &mut self.fifo0
        } else {
            &mut self.fifo1
        };
        if !queue.is_empty() {
            queue.remove(0);
        }
        self.sync_fifo_flags(view, fifo);
    }

    fn sync_fifo_flags(&self, view: &mut Registers<&mut [u32]>, fifo: u8) {
        let len = self.queue_len(fifo);
        // Preserve the latched overflow bit; only FMP and FULL are recomputed.
        let clean = view.get(fifo_reg(fifo)) & !(FMP_MASK | full_bit(fifo));
        let fmp = u32::try_from(len).unwrap_or(u32::MAX) & FMP_MASK;
        let mut flags = clean | fmp;
        if len == 2 {
            flags |= full_bit(fifo);
        }
        view.set(fifo_reg(fifo), flags);
    }

    fn queue_len(&self, fifo: u8) -> usize {
        if fifo == 0 {
            self.fifo0.len()
        } else {
            self.fifo1.len()
        }
    }

    fn front(&self, fifo: u8) -> Option<&Frame> {
        if fifo == 0 {
            self.fifo0.first()
        } else {
            self.fifo1.first()
        }
    }

    /// Determine the target FIFO for `frame` via the filter banks, or `None`
    /// to drop (filter mismatch). When no filter is active, accept to FIFO0
    /// for backward compat (existing tests inject without configuring filters).
    fn filter_fifo(&self, frame: &Frame) -> Option<u8> {
        let fmr = self.filter[FMR_IDX];
        let can2sb = ((fmr >> 8) & 0x3F) as usize;
        let fa1r = self.filter[FA1R_IDX];
        if fa1r == 0 {
            return Some(0);
        }
        let fm1r = self.filter[FM1R_IDX];
        let fs1r = self.filter[FS1R_IDX];
        let ffa1r = self.filter[FFA1R_IDX];

        // Number of banks: 28 for F4, 14 for F1; use 28 max and active mask decides.
        let num_banks = 28usize;
        let (start, end) = if self.is_can2 {
            (can2sb.min(num_banks), num_banks)
        } else if can2sb == 0 {
            (0, num_banks)
        } else {
            (0, can2sb.min(num_banks))
        };

        for bank in start..end {
            if (fa1r >> bank) & 1 == 0 {
                continue;
            }
            let fifo = if (ffa1r >> bank) & 1 != 0 { 1 } else { 0 };
            let scale_32 = (fs1r >> bank) & 1 != 0;
            let mode_list = (fm1r >> bank) & 1 != 0;
            let fr1 = self.filter[BANK_BASE_IDX + 2 * bank];
            let fr2 = self.filter[BANK_BASE_IDX + 2 * bank + 1];

            if scale_32 {
                if !mode_list {
                    // Mask mode: (ID & mask) == (filter & mask) on whole TI word.
                    if (frame.ti & fr2) == (fr1 & fr2) {
                        return Some(fifo);
                    }
                } else {
                    // List mode: exact match against either FR.
                    if frame.ti == fr1 || frame.ti == fr2 {
                        return Some(fifo);
                    }
                }
            } else {
                // 16-bit scale: each FR holds two 16-bit entries.
                let id_low = frame.ti & 0xFFFF;
                let fr1_low = fr1 & 0xFFFF;
                let fr1_high = (fr1 >> 16) & 0xFFFF;
                let fr2_low = fr2 & 0xFFFF;
                let fr2_high = (fr2 >> 16) & 0xFFFF;
                if !mode_list {
                    // Two mask pairs: (ID & mask) == (filter & mask) per half.
                    if (id_low & fr2_low) == (fr1_low & fr2_low) {
                        return Some(fifo);
                    }
                    if (id_low & fr2_high) == (fr1_high & fr2_high) {
                        return Some(fifo);
                    }
                } else {
                    // List mode: 4 IDs.
                    if id_low == fr1_low
                        || id_low == fr1_high
                        || id_low == fr2_low
                        || id_low == fr2_high
                    {
                        return Some(fifo);
                    }
                }
            }
        }
        None
    }
}

impl Behavior for CanBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            MCR => {
                let w = view.merged(offset, val, shift);
                if w & RESET != 0 {
                    self.enter_init(view);
                    view.set(MSR, 0);
                }
                let in_init = view.get(MSR) & INAK != 0;
                if w & INRQ != 0 && !in_init {
                    self.enter_init(view);
                } else if w & INRQ == 0 && in_init {
                    Self::leave_init(view);
                }
                w
            }
            TSR => {
                // Each byte lane covers one mailbox (TME/status live in the
                // low 24 bits). Only the written byte's status bits clear.
                let lane = u32::from(val) << shift;
                let mb = u8::try_from(shift >> 3).unwrap_or(0);
                let w1c = match mb {
                    0 => RQCP0 | TXOK0 | ALST0 | TERR0,
                    1 => RQCP1 | TXOK1 | ALST1 | TERR1,
                    _ => RQCP2 | TXOK2 | ALST2 | TERR2,
                };
                let mut word = view.get(TSR) & !(lane & w1c);
                if mb < 3 {
                    view.set(TSR, word);
                    if lane & abrq_bit(mb) != 0 {
                        self.abort_mailbox(view, mb);
                        word = view.get(TSR);
                    }
                }
                view.set(TSR, word);
                word
            }
            RF0R => {
                let w = view.merged(offset, val, shift);
                if w & RFOM0 != 0 {
                    self.release_front(view, 0);
                }
                view.get(RF0R)
            }
            RF1R => {
                let w = view.merged(offset, val, shift);
                if w & RFOM1 != 0 {
                    self.release_front(view, 1);
                }
                view.get(RF1R)
            }
            TI0R | TI1R | TI2R => {
                let mb = u8::try_from((offset - TI0R) / 0x10).unwrap_or(0);
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                // A mailbox request lands when the top byte of the identifier
                // register arrives, i.e. on the last byte of a 32-bit store.
                if shift == 24 && word & TXRQ != 0 {
                    self.request_tx(view, mb);
                }
                word
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            TSR => {
                // The transmit-empty bits are read-only; recompute them live
                // from which mailboxes hold a frame.
                let empty = u32::from(!self.busy[0]) << 26
                    | u32::from(!self.busy[1]) << 27
                    | u32::from(!self.busy[2]) << 28;
                value | empty
            }
            // The FIFO read registers expose the front frame, which lives in
            // behavior state rather than a stored word.
            RI0R | RDT0R | RDL0R | RDH0R => self.front(0).map_or(value, |f| match offset {
                RI0R => f.ti,
                RDT0R => f.tdt,
                RDL0R => f.tdl,
                _ => f.tdh,
            }),
            RI1R | RDT1R | RDL1R | RDH1R => self.front(1).map_or(value, |f| match offset {
                RI1R => f.ti,
                RDT1R => f.tdt,
                RDL1R => f.tdl,
                _ => f.tdh,
            }),
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if let Some((mb, frame)) = self.in_flight {
            self.tx_timer = self.tx_timer.saturating_sub(delta);
            if self.tx_timer == 0 {
                self.in_flight = None;
                self.complete_tx(view, mb, frame, true);
                if self.in_flight.is_none() {
                    if let Some((mb, frame)) = self.tx_queue.first().copied() {
                        self.tx_queue.remove(0);
                        self.start_tx(view, mb, frame);
                    }
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let ier = view.get(IER);
        let mut lines = 0u32;
        let tsr = view.get(TSR);
        if tsr & (RQCP0 | RQCP1 | RQCP2) != 0 && ier & TMEIE != 0 {
            lines |= 1 << 0;
        }
        let rf0 = view.get(RF0R);
        if (rf0 & FMP_MASK != 0 && ier & FMPIE0 != 0)
            || (rf0 & FULL0 != 0 && ier & FFIE0 != 0)
            || (rf0 & FOVR0 != 0 && ier & FOVIE0 != 0)
        {
            lines |= 1 << 1;
        }
        let rf1 = view.get(RF1R);
        if (rf1 & FMP_MASK != 0 && ier & FMPIE1 != 0)
            || (rf1 & FULL1 != 0 && ier & FFIE1 != 0)
            || (rf1 & FOVR1 != 0 && ier & FOVIE1 != 0)
        {
            lines |= 1 << 2;
        }
        let esr = view.get(ESR);
        if (esr & LEC != 0 && ier & LECIE != 0)
            || (esr & (BOFF | EPVF | EWGF) != 0 && ier & ERRIE != 0)
        {
            lines |= 1 << 3;
        }
        lines
    }

    fn lines(&self) -> u32 {
        0b1111
    }

    fn reset(&mut self) {
        *self = Self {
            filter: Box::new([0u32; 0x80]),
            is_can2: self.is_can2,
            ..Self::default()
        };
        // Preserve is_can2 across reset, but clear filter.
        self.is_can2 = self.is_can2;
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        self.rx_buffer.push((value & 0xFF) as u8);
        if self.rx_buffer.len() == 16 {
            let frame = Frame::from_bytes(&self.rx_buffer);
            self.rx_buffer.clear();
            if let Some(fifo) = self.filter_fifo(&frame) {
                self.enqueue(view, frame, fifo);
            }
        }
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }
}

/// A `bxCAN` controller, table-driven from the SVD: base address, register set,
/// and the four interrupt lines (`TX`, `RX0`, `RX1`, `SCE`) all come from `D`'s
/// generated tables. Filter banks at `0x40006600` (`CAN1+0x200`) are handled
/// via the latch in `CanBehavior` so `HAL_CAN_ConfigFilter` does not
/// `HardFault`; frames are filtered per `FMR`/`FM1R`/`FS1R`/`FFA1R`/`FA1R`.
#[derive(Debug)]
pub struct Can {
    inner: RegisterFile<CanBehavior>,
    name: &'static str,
}

impl Can {
    /// A `CAN1` on another chip's device tables (base `0x4000_6400`, IRQ
    /// 19/20/21/22 on the connectivity and F4 lines).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `CAN1` peripheral.
    #[must_use]
    pub fn can1_for<D: Device>() -> Self {
        Self::for_device::<D>("CAN1", "can1", false)
    }

    /// A `CAN2` on another chip's device tables (base `0x4000_6800`, IRQ
    /// 63/64/65/66 on the connectivity and F4 lines).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `CAN2` peripheral.
    #[must_use]
    pub fn can2_for<D: Device>() -> Self {
        Self::for_device::<D>("CAN2", "can2", true)
    }

    fn for_device<D: Device>(peripheral: &str, name: &'static str, is_can2: bool) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        let mut beh = CanBehavior::default();
        beh.is_can2 = is_can2;
        Self {
            inner: RegisterFile::new(def, beh),
            name,
        }
    }
}

impl Peripheral for Can {
    fn ranges(&self) -> &[Region] {
        // Filter window 0x40006600 lies inside CAN1's 0x40006400/0x400
        // window; CAN2's shared banks are at the same physical address via
        // CAN1's window, so no extra region is advertised. Handling is via
        // early return in read/write.
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if (0x4000_6600..0x4000_6800).contains(&addr) {
            let off = (addr - 0x4000_6600) as usize;
            let word = self.inner.behavior().filter[off / 4];
            return Ok(((word >> (8 * (off % 4))) & 0xFF) as u8);
        }
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if (0x4000_6600..0x4000_6800).contains(&addr) {
            let off = (addr - 0x4000_6600) as usize;
            let idx = off / 4;
            let shift = 8 * (off % 4);
            let beh = self.inner.behavior_mut();
            beh.filter[idx] = (beh.filter[idx] & !(0xFF << shift)) | (u32::from(val) << shift);
            return Ok(());
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

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Can {
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

/// CAN filter bank window `0x4000_6600` `0x200` — latch-only so
/// `HAL_CAN_ConfigFilter` (`FMR/FM1R/FS1R/FFA1R/FA1R/FxR1/2`) not
/// `HardFault`; filtering still always `FIFO0` (see `can.rs:9`).
#[derive(Debug)]
pub struct CanFilter {
    mem: Box<[u32; 0x80]>,
}

impl CanFilter {
    #[must_use]
    pub fn new() -> Self {
        Self {
            mem: Box::new([0u32; 0x80]),
        }
    }
}

impl Default for CanFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for CanFilter {
    fn ranges(&self) -> &[Region] {
        const R: [Region; 1] = [Region::new(0x4000_6600, 0x200)];
        &R
    }
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = (addr - 0x4000_6600) as usize;
        if off >= 0x200 {
            return Err(BusError::Reserved);
        }
        let word = self.mem[off / 4];
        Ok(((word >> (8 * (off % 4))) & 0xFF) as u8)
    }
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = (addr - 0x4000_6600) as usize;
        if off >= 0x200 {
            return Err(BusError::Reserved);
        }
        let idx = off / 4;
        let shift = 8 * (off % 4);
        self.mem[idx] = (self.mem[idx] & !(0xFF << shift)) | (u32::from(val) << shift);
        Ok(())
    }
    fn reset(&mut self) {
        *self.mem = [0u32; 0x80];
    }
    fn name(&self) -> &'static str {
        "canfilter"
    }
}

impl InterruptSource for CanFilter {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::{Can, DLC, FMPIE0, FMPIE1, INAK, INRQ, RQCP0, TMEIE, TXOK0, TXRQ};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4000_6400;
    const FILTER_BASE: u32 = 0x4000_6600;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(c: &mut Can, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            c.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(c: &mut Can, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = c.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn can() -> Can {
        let mut c = Can::can1_for::<crate::device::svd::stm32f107::Stm32f107>();
        c.reset();
        c
    }

    /// Inject one frame through the host byte channel: identifier `0x123`,
    /// DLC 3, bytes `0xAA 0xBB 0xCC`.
    #[allow(clippy::cast_possible_truncation)]
    fn inject(c: &mut Can, ti: u32, dlc: u32, data: &[u8]) {
        let mut frame = [0u8; 16];
        frame[0..4].copy_from_slice(&ti.to_le_bytes());
        frame[4] = dlc as u8;
        for (i, byte) in data.iter().enumerate() {
            frame[8 + i] = *byte;
        }
        for byte in frame {
            c.drive_inputs(u32::from(byte));
        }
    }

    fn load_mailbox(c: &mut Can, mb: u32, ti: u32, dlc: u32, data: &[u8]) {
        let base = BASE + 0x180u32 + 0x10u32 * mb;
        write_word(c, base + 0x04, dlc);
        let mut dl = 0u32;
        let mut dh = 0u32;
        for (i, byte) in data.iter().enumerate() {
            let w = if i < 4 { &mut dl } else { &mut dh };
            *w |= u32::from(*byte) << (8 * (i % 4));
        }
        write_word(c, base + 0x08, dl);
        write_word(c, base + 0x0C, dh);
        write_word(c, base, ti | TXRQ);
    }

    #[test]
    fn init_handshake_tracks_inrq() {
        let mut c = can();
        assert_eq!(read_word(&mut c, BASE + 0x04) & INAK, 0);
        write_word(&mut c, BASE, INRQ);
        assert_eq!(read_word(&mut c, BASE + 0x04) & INAK, INAK);
        write_word(&mut c, BASE, 0);
        assert_eq!(read_word(&mut c, BASE + 0x04) & INAK, 0);
    }

    #[test]
    fn received_frame_fills_fifo_and_raises_rx0() {
        let mut c = can();
        write_word(&mut c, BASE + 0x14, FMPIE0);
        assert_eq!(c.pending() & 0b10, 0);
        inject(&mut c, 0x123, 3, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 1, "FMP0");
        assert_eq!(c.pending() & 0b10, 0b10, "RX0 line");
        assert_eq!(read_word(&mut c, BASE + 0x1B0), 0x123, "RI0R");
        assert_eq!(read_word(&mut c, BASE + 0x1B4) & DLC, 3, "RDT0R DLC");
        assert_eq!(read_word(&mut c, BASE + 0x1B8) & 0xFFFF, 0xBBAA, "RDL0R");
        assert_eq!(
            (read_word(&mut c, BASE + 0x1B8) >> 16) & 0xFF,
            0xCC,
            "RDL0R byte 2"
        );
        // Release the FIFO slot.
        write_word(&mut c, BASE + 0x0C, 1 << 5);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 0, "FMP0 empty");
        assert_eq!(c.pending() & 0b10, 0, "RX0 line dropped");
    }

    #[test]
    fn transmitted_frame_latches_status_and_drains() {
        let mut c = can();
        write_word(&mut c, BASE + 0x1C, 0); // BTR: default bit timing
        load_mailbox(&mut c, 0, 0x321, 2, &[0xDE, 0xAD]);
        // Mailbox 0 is busy: its TME bit reads 0.
        assert_eq!(read_word(&mut c, BASE + 0x08) & (1 << 26), 0);
        for _ in 0..1000 {
            c.tick(1);
        }
        let tsr = read_word(&mut c, BASE + 0x08);
        assert_ne!(tsr & RQCP0, 0, "RQCP0");
        assert_ne!(tsr & TXOK0, 0, "TXOK0");
        assert_ne!(tsr & (1 << 26), 0, "TME0 back");
        let out = c.drain_output();
        assert_eq!(out.len(), 16);
        assert_eq!(out[0..4], 0x321u32.to_le_bytes());
        assert_eq!(out[4], 2);
        assert_eq!(out[8], 0xDE);
        assert_eq!(out[9], 0xAD);
    }

    #[test]
    fn tx_line_needs_tmeie() {
        let mut c = can();
        load_mailbox(&mut c, 0, 1, 1, &[0x11]);
        for _ in 0..1000 {
            c.tick(1);
        }
        assert_eq!(c.pending() & 1, 0, "no TMEIE, no TX line");
        write_word(&mut c, BASE + 0x14, TMEIE);
        assert_eq!(c.pending() & 1, 1, "TX line with TMEIE");
        // Clearing RQCP drops the line.
        write_word(&mut c, BASE + 0x08, RQCP0);
        assert_eq!(c.pending() & 1, 0);
    }

    #[test]
    fn abort_mailbox_latches_rqcp_without_txok() {
        let mut c = can();
        load_mailbox(&mut c, 0, 1, 1, &[0x11]);
        write_word(&mut c, BASE + 0x08, 1 << 7); // ABRQ0
        let tsr = read_word(&mut c, BASE + 0x08);
        assert_ne!(tsr & RQCP0, 0, "RQCP0 latched");
        assert_eq!(tsr & TXOK0, 0, "no TXOK on abort");
        assert_eq!(c.drain_output().len(), 0, "no frame sent");
    }

    #[test]
    fn fifo_overflows_and_keeps_the_oldest() {
        let mut c = can();
        inject(&mut c, 0x1, 1, &[1]);
        inject(&mut c, 0x2, 1, &[2]);
        assert_eq!(
            read_word(&mut c, BASE + 0x0C) & (0b11 | (1 << 3)),
            0b10 | (1 << 3),
            "FMP=2 FULL"
        );
        inject(&mut c, 0x3, 1, &[3]);
        assert_ne!(read_word(&mut c, BASE + 0x0C) & (1 << 4), 0, "FOVR0");
        assert_eq!(read_word(&mut c, BASE + 0x1B0), 0x1, "oldest preserved");
        assert_eq!(read_word(&mut c, BASE + 0x1B8) & 0xFF, 1);
    }

    #[test]
    fn rflm_drops_the_new_frame_when_full() {
        let mut c = can();
        write_word(&mut c, BASE, 1 << 3); // RFLM
        inject(&mut c, 0x1, 1, &[1]);
        inject(&mut c, 0x2, 1, &[2]);
        inject(&mut c, 0x3, 1, &[3]);
        assert_eq!(
            read_word(&mut c, BASE + 0x0C) & (1 << 4),
            0,
            "no overflow latch"
        );
        assert_eq!(read_word(&mut c, BASE + 0x1B0), 0x1, "oldest still there");
        assert_eq!(read_word(&mut c, BASE + 0x1B4) & DLC, 1);
    }

    #[test]
    fn init_mode_blocks_transmission() {
        let mut c = can();
        write_word(&mut c, BASE, INRQ);
        load_mailbox(&mut c, 0, 1, 1, &[0x11]);
        for _ in 0..1000 {
            c.tick(1);
        }
        assert_eq!(
            read_word(&mut c, BASE + 0x08) & RQCP0,
            0,
            "no transmission in init"
        );
        assert_eq!(c.drain_output().len(), 0);
    }

    #[test]
    fn can1_and_can2_have_distinct_lines() {
        let c1 = can();
        let c2 = Can::can2_for::<crate::device::svd::stm32f107::Stm32f107>();
        assert_eq!(c1.exception_for_line(0), Some(16 + 19), "CAN1 TX");
        assert_eq!(c1.exception_for_line(1), Some(16 + 20), "CAN1 RX0");
        assert_eq!(c1.exception_for_line(2), Some(16 + 21), "CAN1 RX1");
        assert_eq!(c1.exception_for_line(3), Some(16 + 22), "CAN1 SCE");
        assert_eq!(c2.exception_for_line(0), Some(16 + 63), "CAN2 TX");
        assert_eq!(c2.exception_for_line(1), Some(16 + 64), "CAN2 RX0");
        assert_eq!(c2.ranges()[0].base, 0x4000_6800);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut c = can();
        assert!(matches!(c.read(BASE + 0x400), Err(BusError::Reserved)));
        assert!(matches!(c.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    // --- Filter tests (WP-A1) ---

    fn configure_mask_filter(c: &mut Can, bank: usize, id: u32, mask: u32, fifo: u8) {
        // FINIT=1 to allow config, then set bank.
        write_word(c, FILTER_BASE + 0x00, 1); // FMR FINIT
        write_word(c, FILTER_BASE + 0x1C, 1 << bank); // FA1R active
                                                      // Keep default 32-bit scale and mask mode (FS1R=1, FM1R=0)
        let mut fs1r = read_word(c, FILTER_BASE + 0x0C);
        fs1r |= 1 << bank;
        write_word(c, FILTER_BASE + 0x0C, fs1r);
        let mut fm1r = read_word(c, FILTER_BASE + 0x04);
        fm1r &= !(1 << bank);
        write_word(c, FILTER_BASE + 0x04, fm1r);
        if fifo == 1 {
            let mut ffa = read_word(c, FILTER_BASE + 0x14);
            ffa |= 1 << bank;
            write_word(c, FILTER_BASE + 0x14, ffa);
        }
        let fr1_addr = FILTER_BASE + 0x40 + (bank as u32) * 8;
        write_word(c, fr1_addr, id);
        write_word(c, fr1_addr + 4, mask);
        write_word(c, FILTER_BASE + 0x00, 0); // FINIT=0
    }

    fn configure_list_filter(c: &mut Can, bank: usize, id1: u32, id2: u32, fifo: u8) {
        write_word(c, FILTER_BASE + 0x00, 1);
        write_word(c, FILTER_BASE + 0x1C, 1 << bank);
        let mut fs1r = read_word(c, FILTER_BASE + 0x0C);
        fs1r |= 1 << bank;
        write_word(c, FILTER_BASE + 0x0C, fs1r);
        let mut fm1r = read_word(c, FILTER_BASE + 0x04);
        fm1r |= 1 << bank;
        write_word(c, FILTER_BASE + 0x04, fm1r);
        if fifo == 1 {
            let mut ffa = read_word(c, FILTER_BASE + 0x14);
            ffa |= 1 << bank;
            write_word(c, FILTER_BASE + 0x14, ffa);
        }
        let fr1_addr = FILTER_BASE + 0x40 + (bank as u32) * 8;
        write_word(c, fr1_addr, id1);
        write_word(c, fr1_addr + 4, id2);
        write_word(c, FILTER_BASE + 0x00, 0);
    }

    #[test]
    fn filter_mask_hit_routes_to_fifo0() {
        let mut c = can();
        configure_mask_filter(&mut c, 0, 0x123, 0xFFFF_FFFF, 0);
        write_word(&mut c, BASE + 0x14, FMPIE0);
        inject(&mut c, 0x123, 1, &[0xAA]);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 1, "FMP0 hit");
        assert_eq!(read_word(&mut c, BASE + 0x1B0), 0x123);
    }

    #[test]
    fn filter_mask_miss_drops_frame() {
        let mut c = can();
        configure_mask_filter(&mut c, 0, 0x123, 0xFFFF_FFFF, 0);
        write_word(&mut c, BASE + 0x14, FMPIE0);
        inject(&mut c, 0x456, 1, &[0xBB]);
        assert_eq!(
            read_word(&mut c, BASE + 0x0C) & 0b11,
            0,
            "FMP0 miss -> drop"
        );
        assert_eq!(c.pending() & 0b10, 0);
    }

    #[test]
    fn filter_list_hit() {
        let mut c = can();
        configure_list_filter(&mut c, 1, 0x111, 0x222, 0);
        write_word(&mut c, BASE + 0x14, FMPIE0);
        inject(&mut c, 0x222, 1, &[0xCC]);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 1, "list hit 0x222");
        inject(&mut c, 0x333, 1, &[0xDD]);
        assert_eq!(
            read_word(&mut c, BASE + 0x0C) & 0b11,
            1,
            "still 1, second not added"
        );
    }

    #[test]
    fn filter_fifo_assignment() {
        let mut c = can();
        configure_mask_filter(&mut c, 0, 0x123, 0xFFFF_FFFF, 1);
        write_word(&mut c, BASE + 0x14, FMPIE1);
        inject(&mut c, 0x123, 1, &[0x11]);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 0, "FIFO0 empty");
        assert_eq!(read_word(&mut c, BASE + 0x10) & 0b11, 1, "FIFO1 has frame");
        assert_eq!(c.pending() & 0b100, 0b100, "RX1 line");
        assert_eq!(read_word(&mut c, BASE + 0x1C0), 0x123, "RI1R");
    }

    #[test]
    fn filter_drop_on_no_match() {
        let mut c = can();
        configure_mask_filter(&mut c, 0, 0x123, 0xFFFF_FFFF, 0);
        // Also bank1 with different ID
        configure_mask_filter(&mut c, 1, 0x456, 0xFFFF_FFFF, 0);
        inject(&mut c, 0x999, 1, &[0xEE]);
        assert_eq!(read_word(&mut c, BASE + 0x0C) & 0b11, 0);
        assert_eq!(read_word(&mut c, BASE + 0x10) & 0b11, 0);
    }

    #[test]
    fn filter_can2sb_split() {
        // CAN2 should only see banks >= CAN2SB
        let mut c2 = Can::can2_for::<crate::device::svd::stm32f107::Stm32f107>();
        c2.reset();
        // Configure bank0 for ID 0x111 (belongs to CAN1) and bank7 for 0x222 (belongs to CAN2)
        // Set CAN2SB=7
        write_word(&mut c2, FILTER_BASE + 0x00, (7 << 8) | 1); // FMR CAN2SB=7 FINIT
                                                               // Bank0
        write_word(&mut c2, FILTER_BASE + 0x1C, 1 << 0); // FA1R bank0 active initially
        let mut fs1r = read_word(&mut c2, FILTER_BASE + 0x0C);
        fs1r |= 1 << 0 | 1 << 7;
        write_word(&mut c2, FILTER_BASE + 0x0C, fs1r);
        let mut fa1r = read_word(&mut c2, FILTER_BASE + 0x1C);
        fa1r |= 1 << 7;
        write_word(&mut c2, FILTER_BASE + 0x1C, fa1r);
        write_word(&mut c2, FILTER_BASE + 0x40 + 0 * 8, 0x111);
        write_word(&mut c2, FILTER_BASE + 0x40 + 0 * 8 + 4, 0xFFFF_FFFF);
        write_word(&mut c2, FILTER_BASE + 0x40 + 7 * 8, 0x222);
        write_word(&mut c2, FILTER_BASE + 0x40 + 7 * 8 + 4, 0xFFFF_FFFF);
        write_word(&mut c2, FILTER_BASE + 0x00, 7 << 8); // FINIT=0 keep CAN2SB

        write_word(&mut c2, 0x4000_6800 + 0x14, FMPIE0);
        // Frame matching bank0 should be dropped (belongs to CAN1)
        inject(&mut c2, 0x111, 1, &[0x11]);
        let base2 = 0x4000_6800;
        assert_eq!(
            read_word(&mut c2, base2 + 0x0C) & 0b11,
            0,
            "bank0 belongs to CAN1, CAN2 drops"
        );
        inject(&mut c2, 0x222, 1, &[0x22]);
        assert_eq!(
            read_word(&mut c2, base2 + 0x0C) & 0b11,
            1,
            "bank7 belongs to CAN2"
        );
    }
}

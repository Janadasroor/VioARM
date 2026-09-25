//! TI Tiva C USB controller (`USB0`, device mode, Mentor MUSBD core).
//!
//! Device operation follows the TM4C123GH6PM datasheet (SPMS376E §18).
//!
//! Latches for `FADDR`/`POWER`/`DEVCTL`; endpoint FIFOs with separate
//! `TXMAXP`/`RXMAXP` sizing (`MAXLOAD[10:0]`, reset `0x0000`, p1157/1175;
//! EP0 keeps the 64-byte reserved size); TX `TXRDY` sending up to `TXMAXP`
//! bytes into the host mailbox (auto-clear, `TXIS[ep]` set); RX injection
//! raising `RXRDY` + `RXCOUNT` + `RXIS[ep]` with `RXMAXP`-gated `FULL`;
//! `FLUSH` clearing FIFOs and ready bits; `STALL`/`STALLED` latches
//! (device `RX STALL=5, STALLED=6, CLRDT=7 W1C`, p1178; `TX STALL=4,
//! `STALLED=5`, p1169; EP0 `STALL=5` auto-cleared, `STALLED=2` SW-cleared,
//! `DATAEND=3` auto-cleared, p1160-1161); read-sensitive `TXIS`/`RXIS`/
//! `IS`; EP0 `CSR0` semantics; `RXRDY` auto-clear when `AUTOCLR` set and
//! `MAXP` bytes unloaded (p1180); bus `RESET` clearing `FADDR`/`EPIDX`/
//! FIFOs/all `CSR` and enabling EP ints with `IS.RESET` (§18.3.1.8 p1106);
//! general `IS` bits behind `IE`; one NVIC line (IRQ 44). `FSEOF=0x77`
//! and `LSEOF=0x72` stamp per Table 18-5 p1115.
//!
//! Like the repo's other serials, packets complete synchronously — no
//! bit timing, no host, no SOF ticking (`FRAME` reads 0; documented).
//! Host-side events arrive through hooks: [`Usb::inject_rx`] (OUT data),
//! [`Usb::inject_setup`] (EP0 SETUP), [`Usb::signal_reset`] (bus reset),
//! [`Usb::signal_suspend`]/[`Usb::signal_resume`]. Data toggles,
//! double-buffering, DMA, hub, OTG/host mode, and remaining timing
//! registers (`CONTIM`/`VPLEN`) stay latched (documented). FIFO access is
//! 32-bit words (like TivaWare); byte writes push single bytes
//! (documented).

use crate::device::svd::{Device, PeripheralDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

// Core offsets.
const FADDR_OFF: u32 = 0x000;
const POWER_OFF: u32 = 0x001;
const TXIS_OFF: u32 = 0x002;
const RXIS_OFF: u32 = 0x004;
const TXIE_OFF: u32 = 0x006;
const RXIE_OFF: u32 = 0x008;
const IS_OFF: u32 = 0x00A;
const IE_OFF: u32 = 0x00B;
const FRAME_OFF: u32 = 0x00C;
const EPIDX_OFF: u32 = 0x00E;
const DEVCTL_OFF: u32 = 0x060;
// FIFO offsets per endpoint.
const FIFO_BASE: u32 = 0x020;
const FIFO_STRIDE: u32 = 0x004;
// EP0 CSR offsets.
const CSRL0_OFF: u32 = 0x102;
const CSRH0_OFF: u32 = 0x103;
const COUNT0_OFF: u32 = 0x108;
// EP1-7 CSR bases (`TXMAXPn` at `0x110+0x10(n-1)`).
const EP_CSR_BASE: u32 = 0x110;
const EP_CSR_STRIDE: u32 = 0x010;
const TXMAXP_OFF: u32 = 0x00;
const TXCSRL_OFF: u32 = 0x02;
const TXCSRH_OFF: u32 = 0x03;
const RXMAXP_OFF: u32 = 0x04;
const RXCSRL_OFF: u32 = 0x06;
const RXCSRH_OFF: u32 = 0x07;
const RXCOUNT_OFF: u32 = 0x08;

/// `POWER.SOFTCONN` bit 6 (soft connect/disconnect).
const POWER_SOFTCONN: u32 = 1 << 6;
/// Device `IS` bits: `DISCON` 5, `SOF` 3, `RESET` 2, `RESUME` 1, `SUSPEND` 0.
const IS_DISCON: u32 = 1 << 5;
const IS_RESET: u32 = 1 << 2;
const IS_RESUME: u32 = 1 << 1;
const IS_SUSPEND: u32 = 1 << 0;
/// `CSRL0` bits.
const CSR0_RXRDY: u32 = 1 << 0;
const CSR0_TXRDY: u32 = 1 << 1;
const CSR0_STALLED: u32 = 1 << 2;
const CSR0_DATAEND: u32 = 1 << 3;
const CSR0_SETEND: u32 = 1 << 4;
const CSR0_STALL: u32 = 1 << 5;
const CSR0_RXRDYC: u32 = 1 << 6;
const CSR0_SETENDC: u32 = 1 << 7;
/// `TXCSRL` bits: `TXRDY` 0, `FIFONE` 1, `ERROR` 2, `FLUSH` 3, `STALL` 4,
/// `STALLED` 5, `CLRDT` 6, `NAKTO` 7.
const TX_TXRDY: u32 = 1 << 0;
const TX_FIFONE: u32 = 1 << 1;
const TX_ERROR: u32 = 1 << 2;
const TX_FLUSH: u32 = 1 << 3;
const TX_STALL: u32 = 1 << 4;
const TX_STALLED: u32 = 1 << 5;
const TX_CLRDT: u32 = 1 << 6;
/// `RXCSRL` bits (device p1178): `RXRDY` 0, `FULL` 1 `RO`, `OVER` 2,
/// `DATAERR` 3 `RO`, `FLUSH` 4, `STALL` 5, `STALLED` 6, `CLRDT` 7 `W1C`.
const RX_RXRDY: u32 = 1 << 0;
const RX_FULL: u32 = 1 << 1;
const RX_OVER: u32 = 1 << 2;
const RX_DATAERR: u32 = 1 << 3;
const RX_FLUSH: u32 = 1 << 4;
const RX_STALL: u32 = 1 << 5;
const RX_STALLED: u32 = 1 << 6;
const RX_CLRDT: u32 = 1 << 7;
/// `RXCSRH` `AUTOCLR` bit 7 (device p1183).
const RXCSRH_AUTOCLR: u8 = 1 << 7;
/// Full-speed packet cap for EP0 (64-byte reserved FIFO).
const EP0_MAXP: usize = 64;

/// One endpoint's live state.
#[derive(Clone, Debug, Default)]
struct Endpoint {
    tx_fifo: VecDeque<u8>,
    rx_fifo: VecDeque<u8>,
    tx_rdy: bool,
    rx_rdy: bool,
    tx_stall: bool,
    rx_stall: bool,
    /// `TX STALLED` (p1169) / `RX STALLED` (p1178): set when the `STALL`
    /// handshake is transmitted, cleared by software (write 0).
    tx_stalled: bool,
    rx_stalled: bool,
    tx_error: bool,
    /// `OVER` overrun (device p1179, `RW`, SW-clear).
    rx_over: bool,
    rx_dataerr: bool,
    // EP0 only.
    dataend: bool,
    setend: bool,
    stalled0: bool,
    /// Bytes unloaded from the RX FIFO in the current `RXRDY` session
    /// (for `AUTOCLR`: auto-clear when `RXMAXP` bytes unloaded, p1180).
    rx_unloaded: usize,
    /// Latched FIFO pop word (upper `FIFO` lanes read its tail).
    popped: u32,
}

/// Shared controller state behind the window.
#[derive(Debug)]
pub struct UsbCore {
    regs: HashMap<u32, u8>,
    eps: [Endpoint; 8],
    txis: u16,
    rxis: u16,
    is: u8,
    /// `(ep, packet)` transmit mailbox, for the host to drain.
    tx_out: Vec<(u8, Vec<u8>)>,
    pending: bool,
}

/// Behavior behind the USB window.
pub struct UsbBehavior {
    def: &'static PeripheralDef,
    shared: Arc<Mutex<UsbCore>>,
}

impl UsbBehavior {
    /// Datasheet resets for the byte-latch registers.
    /// Table 18-5 p1115: `FSEOF=0x77`, `LSEOF=0x72`.
    fn stamp(core: &mut UsbCore) {
        let resets: &[(u32, u8)] = &[
            (FADDR_OFF, 0x00),
            (POWER_OFF, 0x20),
            (TXIE_OFF, 0xFF),
            (TXIE_OFF + 1, 0xFF),
            (RXIE_OFF, 0xFE),
            (RXIE_OFF + 1, 0xFF),
            (IE_OFF, 0x06),
            (EPIDX_OFF, 0x00),
            (DEVCTL_OFF, 0x80),
            (0x062, 0x00), // TXFIFOSZ
            (0x063, 0x00), // RXFIFOSZ
            (0x07A, 0x5C), // CONTIM
            (0x07B, 0x3C), // VPLEN
            (0x07D, 0x77), // FSEOF
            (0x07E, 0x72), // LSEOF
        ];
        for &(off, val) in resets {
            core.regs.insert(off, val);
        }
    }

    fn get(core: &UsbCore, off: u32) -> u8 {
        core.regs.get(&off).copied().unwrap_or(0)
    }

    /// Programmed `MAXLOAD[10:0]` for a TX endpoint (p1157, reset `0x0000`).
    fn tx_maxload(core: &UsbCore, ep: usize) -> usize {
        if ep == 0 {
            return EP0_MAXP;
        }
        let base = EP_CSR_BASE + (ep as u32 - 1) * EP_CSR_STRIDE + TXMAXP_OFF;
        let lo = Self::get(core, base) as usize;
        let hi = Self::get(core, base + 1) as usize;
        (lo | (hi << 8)) & 0x07FF
    }

    /// Programmed `MAXLOAD[10:0]` for an RX endpoint (p1175, reset `0x0000`).
    /// Unprogrammed (`0`) stays `0` for EP1-7 (cosmetic); EP0 keeps 64.
    fn rx_maxload(core: &UsbCore, ep: usize) -> usize {
        if ep == 0 {
            return EP0_MAXP;
        }
        let base = EP_CSR_BASE + (ep as u32 - 1) * EP_CSR_STRIDE + RXMAXP_OFF;
        let lo = Self::get(core, base) as usize;
        let hi = Self::get(core, base + 1) as usize;
        (lo | (hi << 8)) & 0x07FF
    }

    /// Move up to `TXMAXP` TX bytes into the mailbox; clear `TXRDY`.
    /// EP0 `DATAEND` auto-clears on completion (p1160).
    fn send_packet(core: &mut UsbCore, ep: usize) {
        let cap = Self::tx_maxload(core, ep);
        // Unprogrammed `MAXP==0` for EP1-7 sends nothing (firmware must
        // program `TXMAXP`; cosmetic per audit).
        let n = core.eps[ep].tx_fifo.len().min(cap);
        let packet: Vec<u8> = core.eps[ep].tx_fifo.drain(..n).collect();
        core.tx_out.push((ep as u8, packet));
        core.eps[ep].tx_rdy = false;
        if ep == 0 {
            core.eps[0].dataend = false;
        }
        core.txis |= 1 << ep;
        Self::refresh(core);
    }

    fn refresh(core: &mut UsbCore) {
        let txie =
            u16::from(Self::get(core, TXIE_OFF)) | u16::from(Self::get(core, TXIE_OFF + 1)) << 8;
        let rxie =
            u16::from(Self::get(core, RXIE_OFF)) | u16::from(Self::get(core, RXIE_OFF + 1)) << 8;
        let ie = Self::get(core, IE_OFF);
        core.pending = (core.txis & txie) != 0 || (core.rxis & rxie) != 0 || (core.is & ie) != 0;
    }

    /// `TXCSRLn` value for EP1-7 (computed + latched mix, p1169).
    fn txcsrl(core: &UsbCore, ep: usize) -> u8 {
        let e = &core.eps[ep];
        let mut v = 0u8;
        if e.tx_rdy {
            v |= TX_TXRDY as u8;
        }
        if !e.tx_fifo.is_empty() {
            v |= TX_FIFONE as u8;
        }
        if e.tx_error {
            v |= TX_ERROR as u8;
        }
        if e.tx_stall {
            v |= TX_STALL as u8;
        }
        if e.tx_stalled {
            v |= TX_STALLED as u8;
        }
        v
    }

    /// `RXCSRLn` value for EP1-7 (device p1178).
    fn rxcsrl(core: &UsbCore, ep: usize) -> u8 {
        let e = &core.eps[ep];
        let mut v = 0u8;
        if e.rx_rdy {
            v |= RX_RXRDY as u8;
        }
        if !e.rx_fifo.is_empty() {
            v |= RX_FULL as u8;
        }
        if e.rx_over {
            v |= RX_OVER as u8;
        }
        if e.rx_dataerr {
            v |= RX_DATAERR as u8;
        }
        if e.rx_stall {
            v |= RX_STALL as u8;
        }
        if e.rx_stalled {
            v |= RX_STALLED as u8;
        }
        v
    }

    /// `CSRL0` value.
    fn csrl0(core: &UsbCore) -> u8 {
        let e = &core.eps[0];
        let mut v = 0u8;
        if e.rx_rdy {
            v |= CSR0_RXRDY as u8;
        }
        if e.tx_rdy {
            v |= CSR0_TXRDY as u8;
        }
        if e.stalled0 {
            v |= CSR0_STALLED as u8;
        }
        if e.dataend {
            v |= CSR0_DATAEND as u8;
        }
        if e.setend {
            v |= CSR0_SETEND as u8;
        }
        if e.tx_stall {
            v |= CSR0_STALL as u8;
        }
        v
    }
}

/// A Tiva C USB controller (device mode).
pub struct Usb {
    ranges: [Region; 1],
    exception: Option<u8>,
    inner: Mutex<UsbBehavior>,
}

impl Usb {
    /// A new USB controller on `D`'s tables (`"USB0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain USB");
        let core = UsbCore {
            regs: HashMap::new(),
            eps: Default::default(),
            txis: 0,
            rxis: 0,
            is: 0,
            tx_out: Vec::new(),
            pending: false,
        };
        let shared = Arc::new(Mutex::new(core));
        UsbBehavior::stamp(&mut shared.lock().unwrap_or_else(|e| e.into_inner()));
        Self {
            ranges: [Region::new(def.base, def.window)],
            exception: def.irqs.first().map(|irq| 16 + irq.number),
            inner: Mutex::new(UsbBehavior { def, shared }),
        }
    }

    fn offset_in_window(&self, addr: u32) -> Result<u32, BusError> {
        let region = self.ranges[0];
        addr.checked_sub(region.base)
            .filter(|o| *o < region.len)
            .ok_or(BusError::Reserved)
    }

    /// Transmit packets `(ep, bytes)` firmware sent, for the host to drain.
    pub fn drain_output(&mut self) -> Vec<(u8, Vec<u8>)> {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        std::mem::take(&mut core.tx_out)
    }

    /// Feed an OUT packet to endpoint `ep` (host stimulus).
    pub fn inject_rx(&mut self, ep: usize, data: &[u8]) {
        if ep > 7 {
            return;
        }
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.eps[ep].rx_fifo.extend(data.iter().copied());
        core.eps[ep].rx_unloaded = 0;
        if ep == 0 {
            core.eps[0].rx_rdy = true;
            core.txis |= 1;
        } else {
            // `OVER` when the packet cannot fit (`RXMAXP`-gated, p1179);
            // unprogrammed `MAXP==0` never overruns (firmware must program
            // `RXMAXP` before use).
            let maxp = UsbBehavior::rx_maxload(&core, ep);
            core.eps[ep].rx_rdy = true;
            if maxp > 0 && core.eps[ep].rx_fifo.len() > maxp {
                core.eps[ep].rx_over = true;
            }
            core.rxis |= 1 << ep;
        }
        UsbBehavior::refresh(&mut core);
    }

    /// Feed an EP0 SETUP packet (host stimulus, always 8 bytes).
    pub fn inject_setup(&mut self, data: [u8; 8]) {
        self.inject_rx(0, &data);
    }

    /// Signal a USB bus reset (host stimulus, §18.3.1.8 p1106): clears
    /// `FADDR`/`EPIDX`, flushes all FIFOs, clears all `CSR`, enables all
    /// endpoint interrupts, and raises `IS.RESET`.
    pub fn signal_reset(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.regs.insert(FADDR_OFF, 0);
        core.regs.insert(EPIDX_OFF, 0);
        for e in core.eps.iter_mut() {
            e.tx_fifo.clear();
            e.rx_fifo.clear();
            e.tx_rdy = false;
            e.rx_rdy = false;
            e.tx_stall = false;
            e.rx_stall = false;
            e.tx_stalled = false;
            e.rx_stalled = false;
            e.tx_error = false;
            e.rx_over = false;
            e.rx_dataerr = false;
            e.dataend = false;
            e.setend = false;
            e.rx_unloaded = 0;
            e.popped = 0;
        }
        core.txis = 0;
        core.rxis = 0;
        // Enable all endpoint interrupts (reset defaults).
        core.regs.insert(TXIE_OFF, 0xFF);
        core.regs.insert(TXIE_OFF + 1, 0xFF);
        core.regs.insert(RXIE_OFF, 0xFE);
        core.regs.insert(RXIE_OFF + 1, 0xFF);
        core.is |= IS_RESET as u8;
        UsbBehavior::refresh(&mut core);
    }

    /// Signal suspend/resume (host stimulus).
    pub fn signal_suspend(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.is |= IS_SUSPEND as u8;
        UsbBehavior::refresh(&mut core);
    }

    /// Signal resume (host stimulus).
    pub fn signal_resume(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.is |= IS_RESUME as u8;
        UsbBehavior::refresh(&mut core);
    }
}

impl Peripheral for Usb {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        // FIFOs: word reads pop four RX bytes (latched tail for lanes 1-3).
        // p1180: if `AUTOCLR` is set and a full `RXMAXP`-byte packet has
        // been unloaded, `RXRDY` auto-clears; short packets need manual
        // clear (write 0).
        if (FIFO_BASE..FIFO_BASE + 8 * FIFO_STRIDE).contains(&off) {
            let ep = ((off - FIFO_BASE) / FIFO_STRIDE) as usize;
            let lane = (off % 4) as usize;
            if lane == 0 {
                let before = core.eps[ep].rx_fifo.len();
                let mut word = 0u32;
                for i in 0..4 {
                    word |= u32::from(core.eps[ep].rx_fifo.pop_front().unwrap_or(0)) << (8 * i);
                }
                core.eps[ep].popped = word;
                let popped = before.saturating_sub(core.eps[ep].rx_fifo.len());
                if ep != 0 {
                    core.eps[ep].rx_unloaded += popped;
                    let base = EP_CSR_BASE + (ep as u32 - 1) * EP_CSR_STRIDE + RXCSRH_OFF;
                    let autoclr = UsbBehavior::get(&core, base) & RXCSRH_AUTOCLR != 0;
                    let maxp = UsbBehavior::rx_maxload(&core, ep);
                    if autoclr
                        && maxp > 0
                        && core.eps[ep].rx_unloaded >= maxp
                        && core.eps[ep].rx_rdy
                    {
                        core.eps[ep].rx_rdy = false;
                        core.eps[ep].rx_unloaded = 0;
                    }
                }
                return Ok((word & 0xFF) as u8);
            }
            return Ok(((core.eps[ep].popped >> (8 * lane)) & 0xFF) as u8);
        }
        // Read-sensitive interrupt registers clear their lane's bits.
        if off == TXIS_OFF || off == TXIS_OFF + 1 {
            let bit = (off - TXIS_OFF) as usize * 8;
            let mut bits = 0u16;
            for i in 0..8 {
                if core.txis & (1 << (bit + i)) != 0 {
                    bits |= 1 << i;
                }
            }
            core.txis &= !(0xFF << bit);
            UsbBehavior::refresh(&mut core);
            return Ok(bits as u8);
        }
        if off == RXIS_OFF || off == RXIS_OFF + 1 {
            let bit = (off - RXIS_OFF) as usize * 8;
            let mut bits = 0u16;
            for i in 0..8 {
                if core.rxis & (1 << (bit + i)) != 0 {
                    bits |= 1 << i;
                }
            }
            core.rxis &= !(0xFF << bit);
            UsbBehavior::refresh(&mut core);
            return Ok(bits as u8);
        }
        if off == IS_OFF {
            let bits = core.is;
            core.is = 0;
            UsbBehavior::refresh(&mut core);
            return Ok(bits);
        }
        // Computed CSRs and counts.
        if off == CSRL0_OFF {
            return Ok(UsbBehavior::csrl0(&core));
        }
        if off == CSRH0_OFF || off == COUNT0_OFF {
            if off == COUNT0_OFF {
                return Ok(core.eps[0].rx_fifo.len().min(255) as u8);
            }
            return Ok(UsbBehavior::get(&core, off));
        }
        for ep in 1..8 {
            let base = EP_CSR_BASE + (ep as u32 - 1) * EP_CSR_STRIDE;
            if off == base + TXCSRL_OFF {
                return Ok(UsbBehavior::txcsrl(&core, ep));
            }
            if off == base + RXCSRL_OFF {
                return Ok(UsbBehavior::rxcsrl(&core, ep));
            }
            if off == base + RXCOUNT_OFF {
                let n = core.eps[ep].rx_fifo.len().min(8191);
                return Ok(((n >> (8 * (off % 2))) & 0xFF) as u8);
            }
        }
        if off == FRAME_OFF || off == FRAME_OFF + 1 {
            return Ok(0); // No SOF ticking in simulation.
        }
        // In-table words latch (SVD widths respected by the bus gaps).
        let known = b.def.registers.iter().any(|r| {
            let start = r.offset;
            off >= start && off < start + 4
        });
        if known {
            return Ok(UsbBehavior::get(&core, off));
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        if (FIFO_BASE..FIFO_BASE + 8 * FIFO_STRIDE).contains(&off) {
            let ep = ((off - FIFO_BASE) / FIFO_STRIDE) as usize;
            core.eps[ep].tx_fifo.push_back(val);
            return Ok(());
        }
        if off == TXIS_OFF
            || off == TXIS_OFF + 1
            || off == RXIS_OFF
            || off == RXIS_OFF + 1
            || off == IS_OFF
        {
            return Ok(()); // Read-sensitive: writes sink.
        }
        if off == CSRL0_OFF {
            // Device p1160-1161: `STALL` auto-clears after the handshake,
            // `STALLED` is set + SW-cleared, `DATAEND` auto-clears on
            // completion, `RXRDYC`/`SETENDC` are `W1C`.
            let rxrdyc = val & CSR0_RXRDYC as u8 != 0;
            let setendc = val & CSR0_SETENDC as u8 != 0;
            let stall = val & CSR0_STALL as u8 != 0;
            let stalled_in = val & CSR0_STALLED as u8 != 0;
            let dataend = val & CSR0_DATAEND as u8 != 0;
            let txrdy = val & CSR0_TXRDY as u8 != 0;
            {
                let e = &mut core.eps[0];
                if rxrdyc {
                    e.rx_rdy = false;
                }
                if setendc {
                    e.setend = false;
                }
                if stall {
                    // Instant handshake: set `STALLED`, auto-clear `STALL`.
                    e.stalled0 = true;
                    e.tx_stall = false;
                } else if !stalled_in {
                    e.stalled0 = false;
                }
                if dataend {
                    e.dataend = true;
                    if e.rx_rdy {
                        e.rx_rdy = false;
                    }
                }
                if txrdy {
                    e.tx_rdy = true;
                }
            }
            if dataend && core.eps[0].tx_rdy || txrdy {
                UsbBehavior::send_packet(&mut core, 0);
            }
            UsbBehavior::refresh(&mut core);
            return Ok(());
        }
        if off == CSRH0_OFF {
            if val & 0x08 != 0 {
                // FLUSH: clear FIFOs and ready bits.
                core.eps[0].tx_fifo.clear();
                core.eps[0].rx_fifo.clear();
                core.eps[0].tx_rdy = false;
                core.eps[0].rx_rdy = false;
            }
            let cur = UsbBehavior::get(&core, off);
            core.regs.insert(off, cur | (val & 0x07));
            return Ok(());
        }
        for ep in 1..8 {
            let base = EP_CSR_BASE + (ep as u32 - 1) * EP_CSR_STRIDE;
            if off == base + TXCSRL_OFF {
                // Device p1169: `STALL` issues the handshake (SW-cleared to
                // terminate), `STALLED` is set + SW-cleared, `FLUSH` clears
                // the FIFO/`TXRDY`, `CLRDT` sinks (toggle unmodeled).
                let stall = val & TX_STALL as u8 != 0;
                let stalled_in = val & TX_STALLED as u8 != 0;
                if val & TX_FLUSH as u8 != 0 {
                    core.eps[ep].tx_fifo.clear();
                    core.eps[ep].tx_rdy = false;
                    core.eps[ep].tx_error = false;
                }
                if stall {
                    core.eps[ep].tx_stall = true;
                    // Instant handshake: flush, clear `TXRDY`, set `STALLED`.
                    core.eps[ep].tx_fifo.clear();
                    core.eps[ep].tx_rdy = false;
                    core.eps[ep].tx_stalled = true;
                } else {
                    core.eps[ep].tx_stall = false;
                    if !stalled_in {
                        core.eps[ep].tx_stalled = false;
                    }
                }
                if val & TX_CLRDT as u8 != 0 {
                    // Data toggle unmodeled; the write sinks.
                }
                if val & TX_TXRDY as u8 != 0 {
                    core.eps[ep].tx_rdy = true;
                    UsbBehavior::send_packet(&mut core, ep);
                }
                UsbBehavior::refresh(&mut core);
                return Ok(());
            }
            if off == base + RXCSRL_OFF {
                // Device p1178-1180: `STALL=5` issues the handshake
                // (SW-cleared), `STALLED=6` is set + SW-cleared, `CLRDT=7`
                // is `W1C` (toggle unmodeled), `FLUSH` clears the FIFO/
                // `RXRDY`, `OVER` is SW-cleared, `RXRDY` clears on write 0
                // (or `AUTOCLR`, see the FIFO path).
                let stall = val & RX_STALL as u8 != 0;
                let stalled_in = val & RX_STALLED as u8 != 0;
                if val & RX_RXRDY as u8 == 0 {
                    core.eps[ep].rx_rdy = false;
                    core.eps[ep].rx_unloaded = 0;
                }
                if val & RX_FLUSH as u8 != 0 {
                    core.eps[ep].rx_fifo.clear();
                    core.eps[ep].rx_rdy = false;
                    core.eps[ep].rx_unloaded = 0;
                    core.eps[ep].rx_over = false;
                    core.eps[ep].rx_dataerr = false;
                }
                if val & RX_OVER as u8 == 0 {
                    core.eps[ep].rx_over = false;
                }
                if stall {
                    core.eps[ep].rx_stall = true;
                    core.eps[ep].rx_stalled = true;
                } else {
                    core.eps[ep].rx_stall = false;
                    if !stalled_in {
                        core.eps[ep].rx_stalled = false;
                    }
                }
                // `CLRDT` `W1C` sinks; `FULL`/`DATAERR` are `RO`.
                UsbBehavior::refresh(&mut core);
                return Ok(());
            }
        }
        // Plain latch words (SVD-known offsets only).
        let known = b.def.registers.iter().any(|r| {
            let start = r.offset;
            off >= start && off < start + 4
        });
        if known {
            core.regs.insert(off, val);
            UsbBehavior::refresh(&mut core);
            return Ok(());
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.regs.clear();
        core.eps = Default::default();
        core.txis = 0;
        core.rxis = 0;
        core.is = 0;
        UsbBehavior::stamp(&mut core);
        UsbBehavior::refresh(&mut core);
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Usb {
    fn exception_number(&self) -> Option<u8> {
        self.exception
    }

    fn pending(&self) -> u32 {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        u32::from(core.pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const USB0_BASE: u32 = 0x4005_0000;
    const EP1_TXMAXP: u32 = EP_CSR_BASE + TXMAXP_OFF;
    const EP1_RXMAXP: u32 = EP_CSR_BASE + RXMAXP_OFF;
    const EP1_RXCSRH: u32 = EP_CSR_BASE + RXCSRH_OFF;
    const EP1_TXCSRL: u32 = EP_CSR_BASE + TXCSRL_OFF;
    const EP1_RXCSRL: u32 = EP_CSR_BASE + RXCSRL_OFF;
    const EP1_RXCOUNT: u32 = EP_CSR_BASE + RXCOUNT_OFF;
    const EP1_FIFO: u32 = FIFO_BASE + FIFO_STRIDE;

    fn usb0() -> Usb {
        Usb::for_device::<crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm>("USB0")
    }

    fn read_word(usb: &mut Usb, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(usb.read(USB0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(usb: &mut Usb, offset: u32, value: u32) {
        for lane in 0..4 {
            usb.write(
                USB0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    fn write_maxp(usb: &mut Usb, base: u32, maxp: u16) {
        usb.write(USB0_BASE + base, (maxp & 0xFF) as u8).unwrap();
        usb.write(USB0_BASE + base + 1, (maxp >> 8) as u8).unwrap();
    }

    /// Datasheet resets land on the bus (Table 18-5 p1115: `FSEOF=0x77`,
    /// `LSEOF=0x72`; `TXMAXP`/`RXMAXP` reset `0x0000`, p1157/1175).
    #[test]
    fn datasheet_resets() {
        let mut usb = usb0();
        assert_eq!(usb.read(USB0_BASE + POWER_OFF).unwrap(), 0x20);
        assert_eq!(read_word(&mut usb, TXIE_OFF) & 0xFFFF, 0xFFFF);
        assert_eq!(read_word(&mut usb, RXIE_OFF) & 0xFFFF, 0xFFFE);
        assert_eq!(usb.read(USB0_BASE + IE_OFF).unwrap(), 0x06);
        assert_eq!(usb.read(USB0_BASE + DEVCTL_OFF).unwrap(), 0x80);
        assert_eq!(usb.read(USB0_BASE + 0x07D).unwrap(), 0x77);
        assert_eq!(usb.read(USB0_BASE + 0x07E).unwrap(), 0x72);
        assert_eq!(read_word(&mut usb, EP1_TXMAXP) & 0xFFFF, 0);
        assert_eq!(read_word(&mut usb, EP1_RXMAXP) & 0xFFFF, 0);
        assert_eq!(usb.exception_number(), Some(60));
    }

    /// EP1 IN: FIFO load + `TXRDY` sends up to `TXMAXP` bytes, sets `TXIS`.
    #[test]
    fn ep1_in_flow() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_TXMAXP, 64);
        write_word(&mut usb, EP1_FIFO, 0x4433_2211);
        usb.write(USB0_BASE + EP1_TXCSRL, TX_TXRDY as u8).unwrap();
        assert_eq!(usb.drain_output(), [(1, vec![0x11, 0x22, 0x33, 0x44])]);
        assert_eq!(usb.pending(), 1);
        assert_eq!(read_word(&mut usb, TXIS_OFF) & 0x02, 0x02);
        // Read-sensitive: the lane read clears EP1's bit.
        assert_eq!(read_word(&mut usb, TXIS_OFF) & 0x02, 0);
        assert_eq!(usb.pending(), 0);
    }

    /// `TXMAXP` caps the packet: 4 queued bytes with `TXMAXP=2` send 2.
    #[test]
    fn txmaxp_caps_packet() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_TXMAXP, 2);
        write_word(&mut usb, EP1_FIFO, 0x4433_2211);
        usb.write(USB0_BASE + EP1_TXCSRL, TX_TXRDY as u8).unwrap();
        assert_eq!(usb.drain_output(), [(1, vec![0x11, 0x22])]);
    }

    /// EP1 OUT: injected packets raise `RXRDY` + `RXCOUNT` + `RXIS`;
    /// unloading + clearing `RXRDY` services them.
    #[test]
    fn ep1_out_flow() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_RXMAXP, 64);
        usb.inject_rx(1, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            RX_RXRDY as u8
        );
        assert_eq!(read_word(&mut usb, EP1_RXCOUNT) & 0xFFFF, 3);
        assert_eq!(read_word(&mut usb, RXIS_OFF) & 0x02, 0x02);
        assert_eq!(read_word(&mut usb, EP1_FIFO), 0x00CC_BBAA);
        // `RXRDY` stays until firmware clears it (write-0).
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            RX_RXRDY as u8
        );
        usb.write(USB0_BASE + EP1_RXCSRL, 0x00).unwrap();
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            0
        );
    }

    /// `RXMAXP` is separate from `TXMAXP` (p1157/1175): `OVER` fires only
    /// when the RX packet exceeds `RXMAXP`.
    #[test]
    fn rxmaxp_separate_from_txmaxp() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_TXMAXP, 2);
        write_maxp(&mut usb, EP1_RXMAXP, 8);
        usb.inject_rx(1, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_OVER as u8, 0);
        usb.write(USB0_BASE + EP1_RXCSRL, RX_FLUSH as u8).unwrap();
        write_maxp(&mut usb, EP1_RXMAXP, 2);
        usb.inject_rx(1, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_OVER as u8,
            RX_OVER as u8
        );
    }

    /// Device `RX STALL=5, STALLED=6, CLRDT=7 W1C` (p1178): firmware
    /// `STALL(5)` sets `STALL`+`STALLED`; `STALLED(6)` alone does not set
    /// `STALL`; `CLRDT(7)` sinks.
    #[test]
    fn rx_stall_is_bit5() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_RXMAXP, 64);
        usb.write(USB0_BASE + EP1_RXCSRL, RX_STALL as u8).unwrap();
        let v = usb.read(USB0_BASE + EP1_RXCSRL).unwrap();
        assert_eq!(v & RX_STALL as u8, RX_STALL as u8);
        assert_eq!(v & RX_STALLED as u8, RX_STALLED as u8);
        // SW-clear both.
        usb.write(USB0_BASE + EP1_RXCSRL, 0x00).unwrap();
        let v = usb.read(USB0_BASE + EP1_RXCSRL).unwrap();
        assert_eq!(v & (RX_STALL | RX_STALLED) as u8, 0);
        // Writing `STALLED(6)` alone does not set `STALL(5)`.
        usb.write(USB0_BASE + EP1_RXCSRL, RX_STALLED as u8).unwrap();
        let v = usb.read(USB0_BASE + EP1_RXCSRL).unwrap();
        assert_eq!(v & RX_STALL as u8, 0);
        usb.write(USB0_BASE + EP1_RXCSRL, 0x00).unwrap();
        // `CLRDT(7)` `W1C` sinks without setting stall latches.
        usb.write(USB0_BASE + EP1_RXCSRL, RX_CLRDT as u8).unwrap();
        let v = usb.read(USB0_BASE + EP1_RXCSRL).unwrap();
        assert_eq!(v & (RX_STALL | RX_STALLED) as u8, 0);
    }

    /// TX `STALL` sets `STALLED` (p1169); both are SW-cleared.
    #[test]
    fn tx_stall_sets_stalled() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_TXMAXP, 64);
        usb.write(USB0_BASE + EP1_TXCSRL, TX_STALL as u8).unwrap();
        let v = usb.read(USB0_BASE + EP1_TXCSRL).unwrap();
        assert_eq!(v & TX_STALL as u8, TX_STALL as u8);
        assert_eq!(v & TX_STALLED as u8, TX_STALLED as u8);
        usb.write(USB0_BASE + EP1_TXCSRL, 0x00).unwrap();
        let v = usb.read(USB0_BASE + EP1_TXCSRL).unwrap();
        assert_eq!(v & (TX_STALL | TX_STALLED) as u8, 0);
    }

    /// EP0 `STALL` auto-clears after the handshake, `STALLED` sets +
    /// SW-clears, `DATAEND` auto-clears on completion (p1160-1161).
    #[test]
    fn ep0_stall_and_dataend_auto_clear() {
        let mut usb = usb0();
        usb.write(USB0_BASE + CSRL0_OFF, CSR0_STALL as u8).unwrap();
        let v = usb.read(USB0_BASE + CSRL0_OFF).unwrap();
        assert_eq!(v & CSR0_STALL as u8, 0);
        assert_eq!(v & CSR0_STALLED as u8, CSR0_STALLED as u8);
        usb.write(USB0_BASE + CSRL0_OFF, 0x00).unwrap();
        assert_eq!(
            usb.read(USB0_BASE + CSRL0_OFF).unwrap() & CSR0_STALLED as u8,
            0
        );
        // `DATAEND` auto-clears when the TX packet completes.
        usb.write(USB0_BASE + FIFO_BASE, 0xAA).unwrap();
        usb.write(USB0_BASE + CSRL0_OFF, (CSR0_TXRDY | CSR0_DATAEND) as u8)
            .unwrap();
        assert_eq!(usb.drain_output(), [(0, vec![0xAA])]);
        assert_eq!(
            usb.read(USB0_BASE + CSRL0_OFF).unwrap() & CSR0_DATAEND as u8,
            0
        );
    }

    /// Bus reset (§18.3.1.8 p1106): clears `FADDR`/`EPIDX`/FIFOs/all `CSR`,
    /// enables EP ints, raises `IS.RESET`.
    #[test]
    fn reset_flow() {
        let mut usb = usb0();
        usb.write(USB0_BASE + FADDR_OFF, 0x2A).unwrap();
        usb.write(USB0_BASE + EPIDX_OFF, 0x01).unwrap();
        write_maxp(&mut usb, EP1_TXMAXP, 64);
        write_word(&mut usb, EP1_FIFO, 0xAABB_CCDD);
        usb.write(USB0_BASE + EP1_TXCSRL, TX_STALL as u8).unwrap();
        // Disable an EP interrupt, then reset must re-enable it.
        usb.write(USB0_BASE + TXIE_OFF, 0x00).unwrap();
        usb.signal_reset();
        assert_eq!(usb.read(USB0_BASE + FADDR_OFF).unwrap(), 0);
        assert_eq!(usb.read(USB0_BASE + EPIDX_OFF).unwrap(), 0);
        assert_eq!(
            usb.read(USB0_BASE + EP1_TXCSRL).unwrap() & (TX_STALL | TX_STALLED) as u8,
            0
        );
        // `IS.RESET` first: `RXIE` word-reads would clear the read-sensitive
        // `IS` lane as a side effect.
        assert_eq!(
            usb.read(USB0_BASE + IS_OFF).unwrap() & IS_RESET as u8,
            IS_RESET as u8
        );
        assert_eq!(read_word(&mut usb, TXIE_OFF) & 0xFFFF, 0xFFFF);
        assert_eq!(read_word(&mut usb, RXIE_OFF) & 0xFFFF, 0xFFFE);
        // Read-sensitive: second read is clear, line drops.
        assert_eq!(usb.read(USB0_BASE + IS_OFF).unwrap(), 0);
        assert_eq!(usb.pending(), 0);
    }

    /// `RXRDY` auto-clears when `AUTOCLR` is set and `MAXP` bytes are
    /// unloaded (p1180); short packets still need manual clear.
    #[test]
    fn rxrdy_autoclr() {
        let mut usb = usb0();
        write_maxp(&mut usb, EP1_RXMAXP, 4);
        usb.write(USB0_BASE + EP1_RXCSRH, RXCSRH_AUTOCLR).unwrap();
        usb.inject_rx(1, &[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            RX_RXRDY as u8
        );
        assert_eq!(read_word(&mut usb, EP1_FIFO), 0x4433_2211);
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            0
        );
        // Short packet (`3 < MAXP=4`): manual clear still required.
        usb.inject_rx(1, &[0xAA, 0xBB, 0xCC]);
        assert_eq!(read_word(&mut usb, EP1_FIFO), 0x00CC_BBAA);
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            RX_RXRDY as u8
        );
        usb.write(USB0_BASE + EP1_RXCSRL, 0x00).unwrap();
        assert_eq!(
            usb.read(USB0_BASE + EP1_RXCSRL).unwrap() & RX_RXRDY as u8,
            0
        );
    }

    /// EP0 control OUT: SETUP injects, `RXRDY` + `TXIS[0]` raise, `DATAEND`
    /// completes the transfer.
    #[test]
    fn ep0_setup_flow() {
        let mut usb = usb0();
        usb.inject_setup([0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00]);
        assert_eq!(
            usb.read(USB0_BASE + CSRL0_OFF).unwrap() & CSR0_RXRDY as u8,
            CSR0_RXRDY as u8
        );
        assert_eq!(usb.read(USB0_BASE + COUNT0_OFF).unwrap(), 8);
        assert_eq!(read_word(&mut usb, TXIS_OFF) & 0x01, 0x01);
        assert_eq!(read_word(&mut usb, FIFO_BASE), 0x0100_0680);
        // Complete with RXRDYC + DATAEND.
        usb.write(USB0_BASE + CSRL0_OFF, (CSR0_RXRDYC | CSR0_DATAEND) as u8)
            .unwrap();
        assert_eq!(
            usb.read(USB0_BASE + CSRL0_OFF).unwrap() & CSR0_RXRDY as u8,
            0
        );
    }
}

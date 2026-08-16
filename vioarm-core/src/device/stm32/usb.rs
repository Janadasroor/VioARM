//! The STM32 OTG USB controller (`USB_OTG_FS`/`OTG_HS`), driven by the
//! SVD-generated register tables.
//!
//! The SVD splits one controller across four peripherals — GLOBAL at
//! `base + 0x000`, HOST at `base + 0x400`, DEVICE at `base + 0x800`, PWRCLK at
//! `base + 0xe00` — and the endpoint FIFO RAM at `base + 0x1000` is in no
//! table at all. [`Usb`] reconnects them behind a single address space and
//! runs the parts the tables cannot: the global interrupt latches and their
//! write-one-to-clear, the soft reset, the pop-on-read receive status, the
//! endpoint-0 control-transfer handshake (`EPENA`-armed IN/OUT transfers that
//! complete immediately and latch `XFRC`/`STUP`), and the host channel that
//! injects bus events and drains transmitted packets.
//!
//! Only device-mode control traffic is modeled: bus reset, SOF, SETUP/OUT
//! packets into the RX FIFO, and IN packets out of the TX FIFO. The controller
//! raises one NVIC line (`OTG_FS`/`OTG_HS`), found by name in the GLOBAL
//! table, and `pending()` is level-based on `GINTSTS & GINTMSK`, so the ISR
//! must clear the latches with its write-back (no `acknowledge` latch).

use crate::device::svd::{Access, Device, PeripheralDef, RegisterDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

// GLOBAL window register offsets.
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const GOTGCTL: u16 = 0x00;
const GAHBCFG: u16 = 0x08;
#[allow(dead_code)] // written through the generic `write_latch` path
const GUSBCFG: u16 = 0x0c;
const GRSTCTL: u16 = 0x10;
const GINTSTS: u16 = 0x14;
const GINTMSK: u16 = 0x18;
const GRXSTSR: u16 = 0x1c;
const GRXSTSP: u16 = 0x20;
#[allow(dead_code)] // FIFO sizing registers (fixed-size FIFO is modeled)
const GRXFSIZ: u16 = 0x24;
#[allow(dead_code)] // FIFO sizing registers (fixed-size FIFO is modeled)
const GNPTXFSIZ: u16 = 0x28;
#[allow(dead_code)] // FIFO space is reported as always-available
const GNPTXSTS: u16 = 0x2c;
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const GCCFG: u16 = 0x38;
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const CID: u16 = 0x3c;

// DEVICE window register offsets.
#[allow(dead_code)] // reference map for the DEVICE window (partially modeled)
const DCFG: u16 = 0x00;
#[allow(dead_code)] // reference map for the DEVICE window (partially modeled)
const DCTL: u16 = 0x04;
const DSTS: u16 = 0x08;
const DIEPMSK: u16 = 0x10;
const DOEPMSK: u16 = 0x14;
const DAINT: u16 = 0x18;
#[allow(dead_code)] // only the endpoint-0 INT mask bits are needed
const DAINTMSK: u16 = 0x1c;
const DIEPCTL0: u16 = 0x100;
#[allow(dead_code)] // only the XFRC/STUP latch bits drive the INT lines
const DIEPINT0: u16 = 0x108;
const DIEPTSIZ0: u16 = 0x110;
#[allow(dead_code)] // the TX FIFO is drained in a single burst
const DTXFSTS0: u16 = 0x118;
const DOEPCTL0: u16 = 0x300;
#[allow(dead_code)] // only the XFRC/STUP latch bits drive the INT lines
const DOEPINT0: u16 = 0x308;
const DOEPTSIZ0: u16 = 0x310;
const EP_STRIDE: u16 = 0x20;

// GINTSTS bits.
const SOF: u32 = 1 << 3;
const RXFLVL: u32 = 1 << 4;
const NPTXFE: u32 = 1 << 5;
const USBRST: u32 = 1 << 12;
const ENUMDNE: u32 = 1 << 13;
const IEPINT: u32 = 1 << 18;
const OEPINT: u32 = 1 << 19;

// Register field bits.
const GINT_EN: u32 = 1 << 0; // GAHBCFG
#[allow(dead_code)] // device mode is forced by the board's setup sequence
const FDMOD: u32 = 1 << 30; // GUSBCFG
const CSRST: u32 = 1 << 0; // GRSTCTL
const RXFFLSH: u32 = 1 << 4; // GRSTCTL
const TXFFLSH: u32 = 1 << 5; // GRSTCTL
const AHBIDL: u32 = 1 << 31; // GRSTCTL, read-only
const EPENA: u32 = 1 << 31; // DIEPCTL/DOEPCTL
const SNAK: u32 = 1 << 27;
const CNAK: u32 = 1 << 26;
const USBAEP: u32 = 1 << 15;
const NAKSTS: u32 = 1 << 17;
const XFRC: u32 = 1 << 0; // DIEPINT/DOEPINT
const STUP: u32 = 1 << 3; // DOEPINT
const TXFE: u32 = 1 << 7; // DIEPINT bit 7, always set; DOEPINT bit 7 too
const STUPCNT: u32 = 0b11 << 29; // DOEPTSIZ
#[allow(dead_code)] // the model always transfers a single packet
const PKTCNT: u32 = 0b11 << 19;
const XFRSIZ_EP0: u32 = 0x7F;
const XFRSIZ_NON_EP0: u32 = 0x7_FFFF;

// RX status word `PKTSTS` encodings, as the ST libraries decode them.
const PKTSTS_OUT: u32 = 1;
const PKTSTS_SETUP: u32 = 2;
const PKTSTS_IN_COMPLETE: u32 = 4;
const PKTSTS_OUT_COMPLETE: u32 = 6;

// The controller's windows are fixed strides from the GLOBAL base; the EP0
// FIFO RAM sits just past the PWRCLK window.
const HOST_OFFSET: u32 = 0x400;
const DEVICE_OFFSET: u32 = 0x800;
const PWRCLK_OFFSET: u32 = 0xe00;
const FIFO_OFFSET: u32 = 0x1000;
const FIFO_LEN: u32 = 0x400;
/// Free words `DTXFSTS` reports once the TX FIFO is empty.
const TX_CAPACITY: u32 = 0x40;

// Host channel framing. Every frame starts with an opcode and a length; SETUP
// carries `len` payload bytes, OUT carries an endpoint byte plus `len` bytes.
const CMD_RESET: u8 = 0x00;
const CMD_SOF: u8 = 0x01;
const CMD_SETUP: u8 = 0x02;
const CMD_OUT: u8 = 0x03;
/// Output frame opcode for a completed IN transfer.
const EVT_IN: u8 = 0x80;

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

    fn index(&self, offset: u16) -> usize {
        self.def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .unwrap_or_else(|| panic!("no register at offset 0x{offset:x}"))
    }

    fn get(&self, offset: u16) -> u32 {
        self.words[self.index(offset)]
    }

    fn set(&mut self, offset: u16, value: u32) {
        let i = self.index(offset);
        self.words[i] = value;
    }

    fn merged(&self, offset: u16, val: u8, shift: u32) -> u32 {
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
                off >= u32::from(r.offset) && off < u32::from(r.offset) + len
            })
            .map(|r| (r, 8 * (off - u32::from(r.offset))))
            .ok_or(BusError::Reserved)
    }

    fn reload_reset(&mut self) {
        for (word, reg) in self.words.iter_mut().zip(self.def.registers) {
            *word = reg.reset;
        }
    }
}

/// Per-endpoint device-side state: whether a transfer is armed and the amount
/// of data still expected. Endpoints start NAK'd until firmware clears them.
#[derive(Clone, Copy, Debug)]
struct Ep {
    armed: bool,
    nak: bool,
    xfersiz: u32,
}

impl Default for Ep {
    fn default() -> Self {
        Self {
            armed: false,
            nak: true,
            xfersiz: 0,
        }
    }
}

/// Everything the SVD cannot describe about the controller.
#[derive(Debug, Default)]
struct UsbState {
    /// Latched global events (`GINTSTS` writable bits), cleared by the ISR's
    /// write-one-to-clear.
    events: u32,
    /// Pending RX status words, popped by `GRXSTSR` reads.
    rx_status: Vec<u32>,
    /// The packet payload behind the next status word.
    rx_data: Vec<u8>,
    /// Per-endpoint TX FIFO data written by firmware at `base + 0x1000`.
    tx_fifo: [Vec<u8>; 4],
    in_ep: [Ep; 4],
    out_ep: [Ep; 4],
    /// Latched per-endpoint IN interrupt flags (bit 7 synthesized on read).
    diepint: [u32; 4],
    /// Latched per-endpoint OUT interrupt flags (bit 7 synthesized on read).
    doepint: [u32; 4],
    /// A SETUP packet is in the FIFO but the data stage has not been armed.
    setup_pending: bool,
    /// The bus reset was cleared by software: device is enumerated.
    enum_done: bool,
    /// SOF frame number.
    frame: u32,
    /// The status word popped from `GRXSTSR`'s first byte lane.
    rx_pop: u32,
    /// Bytes of one host frame being accumulated.
    host_rx: Vec<u8>,
    /// Completed IN transfers, framed for the host channel.
    output: Vec<u8>,
}

impl UsbState {
    /// The full `GINTSTS` word: latched events plus the synthesized flags
    /// (`NPTXFE`, and `IEPINT`/`OEPINT` gated by the endpoint masks).
    fn gintsts(&self, d: &Win) -> u32 {
        let in_mask = d.get(DIEPMSK);
        let out_mask = d.get(DOEPMSK);
        let iep = (0..4).any(|i| self.diepint[i] & !TXFE & in_mask != 0);
        let oep = (0..4).any(|i| self.doepint[i] & !TXFE & out_mask != 0);
        self.events | NPTXFE | if iep { IEPINT } else { 0 } | if oep { OEPINT } else { 0 }
    }

    /// The single NVIC line: asserted once the global interrupt enable is set
    /// and any `GINTSTS` bit is unmasked.
    fn pending(&self, g: &Win, d: &Win) -> u32 {
        if g.get(GAHBCFG) & GINT_EN == 0 {
            return 0;
        }
        u32::from(self.gintsts(d) & g.get(GINTMSK) != 0)
    }

    /// Queue one receive entry: the packet payload plus its status word.
    fn enqueue(&mut self, ep: u8, pktsts: u32, dpid: u32, data: &[u8]) {
        let bcnt = u32::try_from(data.len()).unwrap_or(u32::MAX);
        self.rx_data.extend_from_slice(data);
        self.rx_status
            .push(u32::from(ep) | (bcnt << 4) | (dpid << 15) | (pktsts << 17));
        self.events |= RXFLVL;
    }

    /// Pop the next `GRXSTSR` word, releasing `RXFLVL` with the queue.
    fn pop_status(&mut self) -> u32 {
        if self.rx_status.is_empty() {
            return 0;
        }
        let word = self.rx_status.remove(0);
        if self.rx_status.is_empty() {
            self.events &= !RXFLVL;
        }
        word
    }

    /// `EPENA` in `DIEPCTLn`: drain the TX FIFO into the host channel and
    /// latch `XFRC` plus the IN-complete status, all immediately.
    fn start_in(&mut self, d: &Win, ep: usize) {
        let tsz = d.get(DIEPTSIZ0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE);
        let xfersiz = if ep == 0 {
            tsz & XFRSIZ_EP0
        } else {
            tsz & XFRSIZ_NON_EP0
        };
        let n = usize::try_from(xfersiz)
            .unwrap_or(usize::MAX)
            .min(self.tx_fifo[ep].len());
        let data: Vec<u8> = self.tx_fifo[ep].drain(..n).collect();
        let ep8 = u8::try_from(ep).unwrap_or(u8::MAX);
        let n8 = u8::try_from(n).unwrap_or(u8::MAX);
        self.output.push(EVT_IN);
        self.output.push(n8);
        self.output.push(ep8);
        self.output.extend(&data);
        self.diepint[ep] |= XFRC;
        self.rx_status
            .push(u32::from(ep8) | (PKTSTS_IN_COMPLETE << 17));
        self.events |= RXFLVL;
    }

    /// `EPENA` in `DOEPCTLn`: arm the OUT endpoint. On endpoint 0 with a
    /// pending SETUP and `STUPCNT` armed, the setup stage completes instead —
    /// `STUP` latches, `EPENA` drops, and `false` tells the caller not to
    /// keep the bit.
    fn arm_out(&mut self, d: &Win, ep: usize) -> bool {
        if ep == 0 && d.get(DOEPTSIZ0) & STUPCNT != 0 && self.setup_pending {
            self.setup_pending = false;
            self.doepint[0] |= STUP;
            return false;
        }
        let tsz = d.get(DOEPTSIZ0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE);
        let xfersiz = if ep == 0 {
            tsz & XFRSIZ_EP0
        } else {
            tsz & XFRSIZ_NON_EP0
        };
        self.out_ep[ep].armed = true;
        self.out_ep[ep].xfersiz = xfersiz;
        true
    }

    /// An OUT packet arrived from the host: queue the data, and if the
    /// endpoint was armed for enough bytes, complete the transfer.
    fn out_packet(&mut self, d: &mut Win, ep: usize, data: &[u8]) {
        let ep8 = u8::try_from(ep).unwrap_or(u8::MAX);
        self.enqueue(ep8, PKTSTS_OUT, 2, data);
        let expected = usize::try_from(self.out_ep[ep].xfersiz).unwrap_or(0);
        if self.out_ep[ep].armed && expected >= data.len() {
            let reg = DOEPCTL0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
            d.set(reg, d.get(reg) & !EPENA);
            self.out_ep[ep].armed = false;
            self.doepint[ep] |= XFRC;
            self.rx_status
                .push(u32::from(ep8) | (PKTSTS_OUT_COMPLETE << 17));
            self.events |= RXFLVL;
        }
    }

    /// `DSTS`: full-speed `ENUMSPD` once enumerated, plus the SOF frame count.
    const fn dsts(&self) -> u32 {
        let mut word = 0u32;
        if self.enum_done {
            word |= 1 << 1;
        }
        word | (self.frame & 0x3FFF) << 8
    }

    /// `DAINT`: the OR of every latched, non-synthesized endpoint flag.
    fn daint(&self) -> u32 {
        let mut word = 0u32;
        for (i, &int) in self.diepint.iter().enumerate() {
            if int & !TXFE != 0 {
                word |= 1 << i;
            }
        }
        for (i, &int) in self.doepint.iter().enumerate() {
            if int & !TXFE != 0 {
                word |= 1 << (16 + i);
            }
        }
        word
    }

    /// `DIEPCTLn` read: the stored word plus the always-set, read-only bits.
    fn diepctl(&self, d: &Win, ep: usize) -> u32 {
        let off = DIEPCTL0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
        let mut word = d.get(off);
        if ep == 0 {
            word |= USBAEP;
        }
        if self.in_ep[ep].nak {
            word |= NAKSTS;
        }
        word
    }

    /// `DOEPCTLn` read: `EPENA` reflects an armed OUT endpoint.
    fn doepctl(&self, d: &Win, ep: usize) -> u32 {
        let off = DOEPCTL0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
        let mut word = d.get(off);
        if self.out_ep[ep].armed {
            word |= EPENA;
        }
        if self.out_ep[ep].nak {
            word |= NAKSTS;
        }
        word
    }

    /// `DIEPINTn` read: latched flags plus the read-only TXFIFO-empty bit.
    const fn diepint(&self, ep: usize) -> u32 {
        (self.diepint[ep] & !TXFE) | TXFE
    }

    /// `DOEPINTn` read: bit 7 mirrors the post-reset `OTEPDIS` state.
    const fn doepint(&self, ep: usize) -> u32 {
        (self.doepint[ep] & !TXFE) | TXFE
    }

    /// `DTXFSTS0`: free words in the endpoint's TX FIFO.
    fn dtxfsts(&self, ep: usize) -> u32 {
        let used = u32::try_from(self.tx_fifo[ep].len() >> 2).unwrap_or(u32::MAX);
        TX_CAPACITY.saturating_sub(used)
    }

    /// The host asserted a bus reset: flush the FIFOs and endpoint state but
    /// keep the programmed registers, as real silicon does.
    fn usb_reset(&mut self) {
        self.events |= USBRST;
        self.events &= !RXFLVL;
        self.rx_status.clear();
        self.rx_data.clear();
        self.tx_fifo = Default::default();
        self.in_ep = Default::default();
        self.out_ep = Default::default();
        self.diepint = [0; 4];
        self.doepint = [0; 4];
        self.setup_pending = false;
        self.enum_done = false;
        self.frame = 0;
    }
}

/// A full OTG controller, reconnecting the SVD's four windows and the FIFO
/// RAM behind one peripheral and one interrupt line.
#[derive(Debug)]
pub struct Usb {
    ranges: [Region; 5],
    g: Win,
    h: Win,
    d: Win,
    p: Win,
    state: UsbState,
    exception: Option<u8>,
    name: &'static str,
}

impl Usb {
    /// The FS controller at `0x5000_0000`, on the connectivity or F4 lines.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the FS OTG controller.
    #[must_use]
    pub fn otg_fs_for<D: Device>() -> Self {
        Self::for_device::<D>("OTG_FS")
    }

    /// The HS controller at `0x4004_0000` on the F4 lines.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the HS OTG controller.
    #[must_use]
    pub fn otg_hs_for<D: Device>() -> Self {
        Self::for_device::<D>("OTG_HS")
    }

    fn for_device<D: Device>(irq: &'static str) -> Self {
        let gdef = win_def::<D>(irq, "GLOBAL");
        let base = gdef.base;
        let exception = gdef
            .irqs
            .iter()
            .find(|i| i.name == irq)
            .or_else(|| gdef.irqs.first())
            .map(|irq| 16 + irq.number);
        Self {
            ranges: [
                Region::new(base, gdef.window),
                Region::new(base + HOST_OFFSET, 0x400),
                Region::new(base + DEVICE_OFFSET, 0x400),
                Region::new(base + PWRCLK_OFFSET, FIFO_OFFSET - PWRCLK_OFFSET),
                Region::new(base + FIFO_OFFSET, FIFO_LEN),
            ],
            g: Win::new(gdef),
            h: Win::new(win_def::<D>(irq, "HOST")),
            d: Win::new(win_def::<D>(irq, "DEVICE")),
            p: Win::new(win_def::<D>(irq, "PWRCLK")),
            state: UsbState::default(),
            exception,
            name: irq,
        }
    }
}

/// The generated name of the OTG controller's `which` window: the F1 tables
/// use `USB_OTG_`, the F4 tables `OTG_FS_`/`OTG_HS_`. Each chip has exactly
/// one controller per variant, so the name lookup is unambiguous.
fn win_def<D: Device>(irq: &str, which: &str) -> &'static PeripheralDef {
    for prefix in usb_prefixes(irq) {
        if let Some(def) = D::peripheral(&format!("{prefix}_{which}")) {
            return def;
        }
    }
    panic!("generated tables contain {irq} {which}");
}

fn usb_prefixes(irq: &str) -> &'static [&'static str] {
    match irq {
        "OTG_HS" => &["OTG_HS"],
        _ => &["USB_OTG", "OTG_FS"],
    }
}

/// The byte lane of `off` inside a four-byte register starting at `reg`.
fn lane(off: u32, reg: u16) -> Option<u32> {
    if off >= u32::from(reg) && off < u32::from(reg) + 4 {
        Some(off - u32::from(reg))
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

/// A plain byte-lane read-modify-write, ignoring reads to read-only registers.
fn write_latch(win: &mut Win, off: u32, val: u8) -> Result<(), BusError> {
    let (reg, shift) = win.reg_at(off)?;
    if reg.access == Access::ReadOnly {
        return Ok(());
    }
    win.set(reg.offset, win.merged(reg.offset, val, shift));
    Ok(())
}

impl Peripheral for Usb {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let base = self.g.def.base;
        if addr >= base + FIFO_OFFSET && addr < base + FIFO_OFFSET + FIFO_LEN {
            return if self.state.rx_data.is_empty() {
                Ok(0)
            } else {
                Ok(self.state.rx_data.remove(0))
            };
        }
        let off = addr.wrapping_sub(base);
        if off < HOST_OFFSET {
            return self.read_global(off);
        }
        if addr >= base + HOST_OFFSET && addr < base + DEVICE_OFFSET {
            return read_latch(&self.h, addr - (base + HOST_OFFSET));
        }
        if addr >= base + DEVICE_OFFSET && addr < base + PWRCLK_OFFSET {
            return self.read_device(addr - (base + DEVICE_OFFSET));
        }
        if addr >= base + PWRCLK_OFFSET && addr < base + FIFO_OFFSET {
            return read_latch(&self.p, addr - (base + PWRCLK_OFFSET));
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let base = self.g.def.base;
        if addr >= base + FIFO_OFFSET && addr < base + FIFO_OFFSET + FIFO_LEN {
            let ep = usize::try_from((addr - (base + FIFO_OFFSET)) / 0x1000).unwrap_or(0);
            self.state.tx_fifo[ep].push(val);
            return Ok(());
        }
        let off = addr.wrapping_sub(base);
        if off < HOST_OFFSET {
            return self.write_global(off, val);
        }
        if addr >= base + HOST_OFFSET && addr < base + DEVICE_OFFSET {
            return write_latch(&mut self.h, addr - (base + HOST_OFFSET), val);
        }
        if addr >= base + DEVICE_OFFSET && addr < base + PWRCLK_OFFSET {
            return self.write_device(addr - (base + DEVICE_OFFSET), val);
        }
        if addr >= base + PWRCLK_OFFSET && addr < base + FIFO_OFFSET {
            return write_latch(&mut self.p, addr - (base + PWRCLK_OFFSET), val);
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        self.soft_reset();
    }

    fn name(&self) -> &'static str {
        self.name
    }

    fn drive_inputs(&mut self, value: u32) {
        self.state.host_rx.push((value & 0xFF) as u8);
        let frame = core::mem::take(&mut self.state.host_rx);
        let opcode = frame.first().copied().unwrap_or(0);
        let len = usize::from(*frame.get(1).unwrap_or(&0));
        let expected = match opcode {
            CMD_RESET | CMD_SOF => 2,
            CMD_SETUP => 2 + len,
            CMD_OUT => 3 + len,
            _ => {
                // Unknown opcode: the stream is out of sync, so drop the
                // whole frame rather than misparse a later byte.
                self.state.host_rx.clear();
                return;
            }
        };
        if frame.len() < expected {
            self.state.host_rx = frame;
            return;
        }
        match opcode {
            CMD_RESET => self.state.usb_reset(),
            CMD_SOF => {
                self.state.frame = (self.state.frame + 1) & 0x3FFF;
                self.state.events |= SOF;
            }
            CMD_SETUP => {
                let data = frame[2..].to_vec();
                self.state.enqueue(0, PKTSTS_SETUP, 0, &data);
                self.state.setup_pending = true;
            }
            CMD_OUT => {
                let ep = usize::from(frame[2]);
                let data = frame[3..].to_vec();
                self.state.out_packet(&mut self.d, ep, &data);
            }
            _ => {}
        }
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.state.output)
    }
}

impl InterruptSource for Usb {
    fn exception_number(&self) -> Option<u8> {
        self.exception
    }

    fn pending(&self) -> u32 {
        self.state.pending(&self.g, &self.d)
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

impl Usb {
    fn read_global(&mut self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, GINTSTS) {
            let word = self.state.gintsts(&self.d);
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, GRXSTSR) {
            // The status word pops on its first byte lane; later lanes keep
            // returning the same word so a 32-bit read sees it whole.
            let word = if l == 0 {
                let word = self.state.pop_status();
                self.state.rx_pop = word;
                word
            } else {
                self.state.rx_pop
            };
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, GRXSTSP) {
            // The non-destructive status port (not in the SVD tables, but on
            // silicon) peeks at the queue head.
            let word = self.state.rx_status.first().copied().unwrap_or(0);
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        read_latch(&self.g, off)
    }

    fn read_device(&self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, DSTS) {
            let word = self.state.dsts();
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, DAINT) {
            let word = self.state.daint();
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if off >= u32::from(DIEPCTL0) && off < u32::from(DIEPCTL0) + 4 * 0x20 {
            let (ep, sub, l) = ep_block(off, DIEPCTL0);
            let word = if sub < 8 {
                self.state.diepctl(&self.d, ep)
            } else if sub < 16 {
                self.state.diepint(ep)
            } else if sub < 24 {
                let reg = DIEPTSIZ0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
                self.d.get(reg)
            } else {
                self.state.dtxfsts(ep)
            };
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if off >= u32::from(DOEPCTL0) && off < u32::from(DOEPCTL0) + 4 * 0x20 {
            let (ep, sub, l) = ep_block(off, DOEPCTL0);
            let word = if sub < 8 {
                self.state.doepctl(&self.d, ep)
            } else if sub < 16 {
                self.state.doepint(ep)
            } else if sub < 24 {
                let reg = DOEPTSIZ0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
                self.d.get(reg)
            } else {
                return Err(BusError::Reserved);
            };
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        read_latch(&self.d, off)
    }

    fn write_global(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        if let Some(l) = lane(off, GRSTCTL) {
            self.write_grstctl(l, val);
            return Ok(());
        }
        if let Some(l) = lane(off, GINTSTS) {
            // Write-one-to-clear of the latched events. Software clearing the
            // bus-reset latch is what makes the core latch ENUMDNE.
            let bits = u32::from(val) << (8 * l);
            if bits & USBRST != 0 && self.state.events & USBRST != 0 {
                self.state.events |= ENUMDNE;
                self.state.enum_done = true;
            }
            self.state.events &= !bits;
            return Ok(());
        }
        write_latch(&mut self.g, off, val)
    }

    fn write_grstctl(&mut self, l: u32, val: u8) {
        let merged = self.g.merged(GRSTCTL, val, 8 * l);
        if l == 0 {
            if merged & CSRST != 0 {
                // A full soft reset: reload every register and drop behavior
                // state. `AHBIDL` reports the reset completed so the firmware
                // poll `while (GRSTCTL & CSRST)` can exit.
                self.soft_reset();
                self.g.set(GRSTCTL, AHBIDL);
                return;
            }
            if merged & RXFFLSH != 0 {
                self.state.rx_status.clear();
                self.state.rx_data.clear();
                self.g.set(GRSTCTL, merged & !RXFFLSH);
                return;
            }
            if merged & TXFFLSH != 0 {
                for fifo in &mut self.state.tx_fifo {
                    fifo.clear();
                }
                self.g.set(GRSTCTL, merged & !TXFFLSH);
                return;
            }
        }
        // `AHBIDL` is read-only status on silicon; a byte write to its lane
        // must not clear it.
        let merged = (merged & !AHBIDL) | (self.g.get(GRSTCTL) & AHBIDL);
        self.g.set(GRSTCTL, merged);
    }

    fn write_device(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        if off >= u32::from(DIEPCTL0) && off < u32::from(DIEPCTL0) + 4 * 0x20 {
            let (ep, sub, l) = ep_block(off, DIEPCTL0);
            if sub < 8 {
                self.write_diepctl(ep, l, val);
                return Ok(());
            }
            if sub < 16 {
                let bits = u32::from(val) << (8 * l);
                self.state.diepint[ep] &= !bits;
                return Ok(());
            }
            return write_latch(&mut self.d, off, val);
        }
        if off >= u32::from(DOEPCTL0) && off < u32::from(DOEPCTL0) + 4 * 0x20 {
            let (ep, sub, l) = ep_block(off, DOEPCTL0);
            if sub < 8 {
                self.write_doepctl(ep, l, val);
                return Ok(());
            }
            if sub < 16 {
                let bits = u32::from(val) << (8 * l);
                self.state.doepint[ep] &= !bits;
                return Ok(());
            }
            return write_latch(&mut self.d, off, val);
        }
        write_latch(&mut self.d, off, val)
    }

    fn write_diepctl(&mut self, ep: usize, l: u32, val: u8) {
        let reg = DIEPCTL0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
        let mut merged = self.d.merged(reg, val, 8 * l);
        if l == 3 {
            if val & 0x80 != 0 {
                self.state.start_in(&self.d, ep);
                merged &= !EPENA;
            }
            if val & 0x08 != 0 {
                self.state.in_ep[ep].nak = true;
                merged &= !SNAK;
            }
            if val & 0x04 != 0 {
                self.state.in_ep[ep].nak = false;
                merged &= !CNAK;
            }
        }
        self.d.set(reg, merged);
    }

    fn write_doepctl(&mut self, ep: usize, l: u32, val: u8) {
        let reg = DOEPCTL0 + u16::try_from(ep).unwrap_or(0) * EP_STRIDE;
        let mut merged = self.d.merged(reg, val, 8 * l);
        if l == 3 {
            if val & 0x80 != 0 && !self.state.arm_out(&self.d, ep) {
                merged &= !EPENA;
            }
            if val & 0x08 != 0 {
                self.state.out_ep[ep].nak = true;
                merged &= !SNAK;
            }
            if val & 0x04 != 0 {
                self.state.out_ep[ep].nak = false;
                merged &= !CNAK;
            }
        }
        self.d.set(reg, merged);
    }

    fn soft_reset(&mut self) {
        self.g.reload_reset();
        self.h.reload_reset();
        self.d.reload_reset();
        self.p.reload_reset();
        self.state = UsbState::default();
    }
}

/// Decompose an endpoint-block byte offset: the endpoint index, the byte
/// within its 0x20 block, and the byte lane within the register.
fn ep_block(off: u32, base: u16) -> (usize, u32, u32) {
    let rel = off - u32::from(base);
    let ep = usize::try_from(rel / 0x20).unwrap_or(0);
    let sub = rel % 0x20;
    let l = rel % 4;
    (ep, sub, l)
}

#[cfg(test)]
mod tests {
    use super::{Usb, CMD_OUT, CMD_RESET, CMD_SETUP, CMD_SOF, ENUMDNE, EVT_IN, RXFLVL, USBRST};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const FS: u32 = 0x5000_0000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(u: &mut Usb, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(u: &mut Usb, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn usb_fs() -> Usb {
        let mut u = Usb::otg_fs_for::<crate::device::svd::stm32f107::Stm32f107>();
        u.reset();
        u
    }

    fn inject(u: &mut Usb, frame: &[u8]) {
        for b in frame {
            u.drive_inputs(u32::from(*b));
        }
    }

    /// The device-mode init a minimal stack performs: force device mode, soft
    /// reset, re-assert device mode, enable the global interrupt and the
    /// reset/enumeration/RX/ep flags, and arm the endpoint 0 masks.
    fn init_device(u: &mut Usb) {
        write_word(u, FS + 0x0c, 1 << 30); // GUSBCFG FDMOD
        write_word(u, FS + 0x10, 1); // GRSTCTL CSRST
        write_word(u, FS + 0x0c, 1 << 30); // re-assert FDMOD after the soft reset
        write_word(u, FS + 0x08, 1); // GAHBCFG GINT
        write_word(
            u,
            FS + 0x18,
            (1 << 12) | (1 << 13) | (1 << 4) | (1 << 18) | (1 << 19),
        );
        write_word(u, FS + 0x800 + 0x10, 1); // DIEPMSK XFRCM
        write_word(u, FS + 0x800 + 0x14, 1 | (1 << 3)); // DOEPMSK XFRCM | STUPM
        write_word(u, FS + 0x800 + 0x1c, (1 << 0) | (1 << 16)); // DAINTMSK
    }

    #[test]
    fn soft_reset_reloads_reset_values() {
        let mut u = usb_fs();
        write_word(&mut u, FS + 0x0c, 0xFFFF_FFFF); // GUSBCFG
        write_word(&mut u, FS + 0x18, 0xFFFF_FFFF); // GINTMSK
        write_word(&mut u, FS + 0x800, 0xDEAD_BEEF); // DCFG
        inject(&mut u, &[CMD_SOF, 0]);
        assert_eq!(u.pending(), 0);
        write_word(&mut u, FS + 0x10, 1); // CSRST
        assert_eq!(read_word(&mut u, FS + 0x10), 0x8000_0000, "AHBIDL");
        assert_eq!(read_word(&mut u, FS + 0x0c), 0x0000_0A00);
        assert_eq!(read_word(&mut u, FS + 0x18), 0);
        assert_eq!(read_word(&mut u, FS + 0x800), 0x0220_0000);
        assert_eq!(u.pending(), 0, "events dropped by the soft reset");
    }

    #[test]
    fn bus_reset_latches_and_clears_by_w1c() {
        let mut u = usb_fs();
        init_device(&mut u);
        assert_eq!(u.pending(), 0);
        inject(&mut u, &[CMD_RESET, 0]);
        assert_eq!(u.pending(), 1);
        assert_ne!(read_word(&mut u, FS + 0x14) & USBRST, 0, "USBRST");
        // The ISR's write-back clears USBRST and makes the core latch ENUMDNE.
        write_word(&mut u, FS + 0x14, USBRST);
        assert_eq!(read_word(&mut u, FS + 0x14) & USBRST, 0);
        assert_ne!(read_word(&mut u, FS + 0x14) & ENUMDNE, 0, "ENUMDNE");
        // Enumeration completed with the latch: the speed is now reported.
        assert_eq!(
            read_word(&mut u, FS + 0x800 + 0x08) & (0b11 << 1),
            1 << 1,
            "ENUMSPD"
        );
        write_word(&mut u, FS + 0x14, ENUMDNE);
        assert_eq!(u.pending(), 0, "line dropped once events clear");
    }

    #[test]
    fn soft_reset_clears_fifo_and_interrupt_state() {
        let mut u = usb_fs();
        init_device(&mut u);
        inject(
            &mut u,
            &[CMD_SETUP, 8, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
        );
        assert_eq!(
            read_word(&mut u, FS + 0x1c) & 0xFFFF,
            0x0080,
            "setup status queued"
        );
        assert_eq!(u.read(FS + 0x1000).unwrap(), 0x80, "setup byte readable");
        write_word(&mut u, FS + 0x10, 1); // CSRST
        assert_eq!(u.read(FS + 0x1000).unwrap(), 0, "FIFO flushed");
        assert_eq!(read_word(&mut u, FS + 0x1c), 0, "status queue flushed");
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn setup_stage_latches_stup_and_reports_enum_speed() {
        let mut u = usb_fs();
        init_device(&mut u);
        inject(&mut u, &[CMD_RESET, 0]);
        inject(
            &mut u,
            &[CMD_SETUP, 8, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
        );
        // ISR: handle the reset and the setup packet.
        let g = read_word(&mut u, FS + 0x14);
        assert_ne!(g & USBRST, 0, "USBRST");
        assert_ne!(g & RXFLVL, 0, "RXFLVL");
        write_word(&mut u, FS + 0x800 + 0x108, 0xFFFF_FFFF);
        write_word(&mut u, FS + 0x800 + 0x308, 0xFFFF_FFFF);
        let st = read_word(&mut u, FS + 0x1c);
        assert_eq!((st >> 17) & 0xF, 2, "PKTSTS SETUP");
        assert_eq!((st >> 4) & 0x7FF, 8, "BCNT 8");
        let mut setup = [0u8; 8];
        for b in &mut setup {
            *b = u.read(FS + 0x1000).unwrap();
        }
        assert_eq!(&setup, &[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
        // Arm the control endpoint: the pending setup completes immediately.
        write_word(&mut u, FS + 0x800 + 0x310, (1 << 29) | (1 << 19) | 8);
        write_word(&mut u, FS + 0x800 + 0x300, 0x8000_0000);
        assert_eq!(
            (read_word(&mut u, FS + 0x800 + 0x300) >> 31) & 1,
            0,
            "EPENA dropped"
        );
        write_word(&mut u, FS + 0x14, g); // write back: USBRST -> ENUMDNE
        assert_ne!(read_word(&mut u, FS + 0x14) & ENUMDNE, 0, "ENUMDNE");
        // ISR entry 2: the STUP interrupt wakes the device.
        let g2 = read_word(&mut u, FS + 0x14);
        assert_ne!(g2 & (1 << 19), 0, "OEPINT");
        assert_ne!(read_word(&mut u, FS + 0x800 + 0x308) & (1 << 3), 0, "STUP");
        write_word(&mut u, FS + 0x800 + 0x308, 0xFFFF_FFFF);
        write_word(&mut u, FS + 0x14, g2); // clears ENUMDNE
        assert_eq!(u.pending(), 0, "all events serviced");
        assert_eq!(
            (read_word(&mut u, FS + 0x800 + 0x08) >> 1) & 0b11,
            1,
            "ENUMSPD"
        );
    }

    #[test]
    fn in_transfer_drains_the_descriptor_to_the_host() {
        let mut u = usb_fs();
        init_device(&mut u);
        let desc = [
            0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x83, 0x04, 0x10, 0x57, 0x00, 0x01,
            0x01, 0x02, 0x00, 0x01,
        ];
        for (i, b) in desc.iter().enumerate() {
            u.write(FS + 0x1000 + u32::try_from(i).unwrap(), *b)
                .unwrap();
        }
        write_word(&mut u, FS + 0x800 + 0x110, (1 << 19) | 0x12); // DIEPTSIZ0 XFRSIZ 18
        write_word(&mut u, FS + 0x800 + 0x100, 0x8000_0000); // DIEPCTL0 EPENA
        let diepint = read_word(&mut u, FS + 0x800 + 0x108);
        assert_ne!(diepint & 1, 0, "XFRC");
        assert_ne!(u.pending(), 0, "IEPINT pending");
        let out = u.drain_output();
        assert_eq!(out.len(), 3 + 18, "framed IN packet");
        assert_eq!(out[0], EVT_IN);
        assert_eq!(out[1], 18);
        assert_eq!(out[2], 0, "endpoint 0");
        assert_eq!(&out[3..], &desc);
        let st = read_word(&mut u, FS + 0x1c);
        assert_eq!((st >> 17) & 0xF, 4, "IN transfer complete status");
        assert_ne!(
            read_word(&mut u, FS + 0x800 + 0x118),
            0,
            "TX FIFO empty again"
        );
    }

    #[test]
    fn armed_out_transfer_completes_on_data() {
        let mut u = usb_fs();
        write_word(&mut u, FS + 0x800 + 0x310, (1 << 19) | 8); // DOEPTSIZ0 XFRSIZ 8
        write_word(&mut u, FS + 0x800 + 0x300, 0x8000_0000); // EPENA
        assert_ne!(
            (read_word(&mut u, FS + 0x800 + 0x300) >> 31) & 1,
            0,
            "EPENA set while armed"
        );
        inject(&mut u, &[CMD_OUT, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8]);
        let st1 = read_word(&mut u, FS + 0x1c);
        assert_eq!((st1 >> 17) & 0xF, 1, "OUT data");
        assert_eq!((st1 >> 4) & 0x7FF, 8, "BCNT 8");
        let st2 = read_word(&mut u, FS + 0x1c);
        assert_eq!((st2 >> 17) & 0xF, 6, "OUT transfer complete");
        assert_ne!(read_word(&mut u, FS + 0x800 + 0x308) & 1, 0, "XFRC");
        assert_eq!(
            (read_word(&mut u, FS + 0x800 + 0x300) >> 31) & 1,
            0,
            "EPENA dropped"
        );
    }

    #[test]
    fn synthesized_flags_read_through() {
        let mut u = usb_fs();
        assert_ne!(read_word(&mut u, FS + 0x14) & (1 << 5), 0, "NPTXFE");
        assert_ne!(read_word(&mut u, FS + 0x800 + 0x108) & 0x80, 0, "TXFE");
        assert_eq!(read_word(&mut u, FS + 0x800 + 0x18), 0, "DAINT clean");
        assert!(read_word(&mut u, FS + 0x800 + 0x118) > 0, "DTXFSTS free");
    }

    #[test]
    fn host_channel_ignores_unknown_frames() {
        let mut u = usb_fs();
        init_device(&mut u);
        inject(&mut u, &[0x7F, 0xFF, 0x7F, 0xFF]);
        assert_eq!(u.pending(), 0);
        assert_eq!(read_word(&mut u, FS + 0x14) & (USBRST | RXFLVL), 0);
    }

    #[test]
    fn addresses_outside_the_windows_are_reserved() {
        let mut u = usb_fs();
        assert!(matches!(u.read(FS + 0x2000), Err(BusError::Reserved)));
        assert!(matches!(u.write(FS - 1, 0), Err(BusError::Reserved)));
        assert!(u.read(FS + 0x400).is_ok(), "HOST window");
        assert!(u.read(FS + 0x1000).is_ok(), "FIFO window");
    }

    #[test]
    fn otg_fs_and_hs_have_distinct_exceptions() {
        let u = usb_fs();
        assert_eq!(u.exception_number(), Some(16 + 67), "OTG_FS on F107");
        assert_eq!(u.ranges()[0].base, 0x5000_0000);
        let f407 = Usb::otg_fs_for::<crate::device::svd::stm32f407::Stm32f407>();
        assert_eq!(f407.exception_number(), Some(16 + 67), "OTG_FS on F407");
        let hs = Usb::otg_hs_for::<crate::device::svd::stm32f407::Stm32f407>();
        assert_eq!(hs.exception_number(), Some(16 + 77), "OTG_HS on F407");
        assert_eq!(hs.ranges()[0].base, 0x4004_0000);
    }
}

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
//!
//! Host mode (`GUSBCFG.FDMOD` clear) drives a [`VirtualUsbDevice`] plugged
//! with [`Usb::plug_virtual_device`]: the HPRT port state machine (power,
//! connect detect, SOF-counted reset, enable), the SOF scheduler (frame
//! counter, `HFNUM`, periodic interrupt transfers deferred to the frame
//! boundary), and host-channel execution (control SETUP/data/status,
//! bulk/interrupt IN/OUT against the virtual device, with ACK/NAK/STALL and
//! `HCTSIZ` remaining-byte write-back). `CMD_SOF` on the host channel is the
//! controller emitting one frame; everything completes deterministically, so
//! a host stack's register sequence runs identically on every run.

use crate::device::svd::{Access, Device, PeripheralDef, RegisterDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

// GLOBAL window register offsets.
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const GOTGCTL: u32 = 0x00;
const GAHBCFG: u32 = 0x08;
#[allow(dead_code)] // written through the generic `write_latch` path
const GUSBCFG: u32 = 0x0c;
const GRSTCTL: u32 = 0x10;
const GINTSTS: u32 = 0x14;
const GINTMSK: u32 = 0x18;
const GRXSTSR: u32 = 0x1c;
const GRXSTSP: u32 = 0x20;
#[allow(dead_code)] // FIFO sizing registers (fixed-size FIFO is modeled)
const GRXFSIZ: u32 = 0x24;
#[allow(dead_code)] // FIFO sizing registers (fixed-size FIFO is modeled)
const GNPTXFSIZ: u32 = 0x28;
#[allow(dead_code)] // FIFO space is reported as always-available
const GNPTXSTS: u32 = 0x2c;
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const GCCFG: u32 = 0x38;
#[allow(dead_code)] // reference map for the GLOBAL window (partially modeled)
const CID: u32 = 0x3c;

// DEVICE window register offsets.
#[allow(dead_code)] // reference map for the DEVICE window (partially modeled)
const DCFG: u32 = 0x00;
#[allow(dead_code)] // reference map for the DEVICE window (partially modeled)
const DCTL: u32 = 0x04;
const DSTS: u32 = 0x08;
const DIEPMSK: u32 = 0x10;
const DOEPMSK: u32 = 0x14;
const DAINT: u32 = 0x18;
#[allow(dead_code)] // only the endpoint-0 INT mask bits are needed
const DAINTMSK: u32 = 0x1c;
const DIEPCTL0: u32 = 0x100;
#[allow(dead_code)] // only the XFRC/STUP latch bits drive the INT lines
const DIEPINT0: u32 = 0x108;
const DIEPTSIZ0: u32 = 0x110;
#[allow(dead_code)] // the TX FIFO is drained in a single burst
const DTXFSTS0: u32 = 0x118;
const DOEPCTL0: u32 = 0x300;
#[allow(dead_code)] // only the XFRC/STUP latch bits drive the INT lines
const DOEPINT0: u32 = 0x308;
const DOEPTSIZ0: u32 = 0x310;
const EP_STRIDE: u32 = 0x20;

// GINTSTS bits.
const SOF: u32 = 1 << 3;
const RXFLVL: u32 = 1 << 4;
const NPTXFE: u32 = 1 << 5;
const USBRST: u32 = 1 << 12;
const ENUMDNE: u32 = 1 << 13;
const IEPINT: u32 = 1 << 18;
const OEPINT: u32 = 1 << 19;
const HPRTINT: u32 = 1 << 24;
const HCINT_BIT: u32 = 1 << 25;

// HOST window register offsets (from `base + 0x400`).
const HCFG: u32 = 0x00;
const HFIR: u32 = 0x04;
const HFNUM: u32 = 0x08;
const HPTXSTS: u32 = 0x10;
const HAINT: u32 = 0x14;
const HAINTMSK: u32 = 0x18;
const HPRT: u32 = 0x40;
const HCCHAR0: u32 = 0x100;
const HCINT0: u32 = 0x108;
const HCINTMSK0: u32 = 0x10c;
const HCTSIZ0: u32 = 0x110;
const HC_STRIDE: u32 = 0x20;
/// FS cores have 8 host channels, HS cores up to 16; the real count comes
/// from the SVD tables (see `Usb::for_device`).
const MAX_HOST_CHANNELS: usize = 16;

// HPRT bits.
const PCSTS: u32 = 1 << 0;
const PCDET: u32 = 1 << 1;
const PENA: u32 = 1 << 2;
const PENCHNG: u32 = 1 << 3;
const POCCHNG: u32 = 1 << 4;
const PRST: u32 = 1 << 8;
const PPWR: u32 = 1 << 12;
const PRTSPD_SHIFT: u32 = 17;
const PRTSPD_FS: u32 = 0b01;
/// HPRT change bits that raise `GINTSTS.HPRTINT` while set.
const HPRT_CHNG: u32 = PCDET | PENCHNG | POCCHNG;

// HCCHAR bits.
const HC_MPSIZ: u32 = 0x7FF;
const HC_EPNUM_SHIFT: u32 = 11;
const HC_EPNUM: u32 = 0xF;
const HC_EPDIR: u32 = 1 << 15;
const HC_EPTYP_SHIFT: u32 = 18;
const HC_CHDIS: u32 = 1 << 30;
const HC_CHENA: u32 = 1 << 31;
const EPTYP_CTRL: u32 = 0;
/// Isochronous host/device transfers: one packet per (micro)frame on the
/// periodic schedule, no handshake, no retry.
const EPTYP_ISO: u32 = 1;
const EPTYP_BULK: u32 = 2;
const EPTYP_INTR: u32 = 3;

// HCTSIZ fields.
const HCTSIZ_XFRSIZ: u32 = 0x7_FFFF;
const HCTSIZ_PKTCNT_SHIFT: u32 = 19;
const HCTSIZ_DPID_SHIFT: u32 = 29;

// HCINT bits.
const HC_XFRC: u32 = 1 << 0;
const HC_CHH: u32 = 1 << 1;
const HC_STALL: u32 = 1 << 3;
const HC_NAK: u32 = 1 << 4;
const HC_ACK: u32 = 1 << 5;
const HC_TXERR: u32 = 1 << 7;
/// Host GRXSTSR `PKTSTS` for a received IN packet (shares the device layout:
// `CHNUM[3:0] BCNT[14:4] DPID[16:15] PKTSTS[20:17]`).
const PKTSTS_HOST_IN: u32 = 2;
/// Port-reset duration in emitted SOFs (silicon needs ~10 ms; the model
/// counts frames so firmware SOF delays still observe a real reset).
const PORT_RESET_SOFS: u8 = 2;

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
/// `DIEPCTLn.EPTYPE` bits [12:11]: `01` selects an isochronous endpoint,
/// whose IN transfers complete on the SOF boundary instead of immediately.
const EPTYPE_SHIFT: u32 = 11;
const EPTYPE_ISO: u32 = 0b01;
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
const FIFO_LEN: u32 = 0x4000;
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

/// Host port state: power and reset are firmware-driven through HPRT, the
/// rest follows the device presence and the SOF scheduler.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum HostPort {
    /// `HPRT.PPWR` clear: the port is dark.
    #[default]
    NoPower,
    /// Powered, no device attached.
    Powered,
    /// Powered with a device attached, not yet reset.
    Connected,
    /// `PRST` set: counting SOFs down to the enable.
    Resetting,
    /// Reset done: transactions run.
    Enabled,
}

/// The outcome of one host-channel transaction against the virtual device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HostXfer {
    /// Bytes moved (IN data is already queued to the RX FIFO); latch
    /// `ACK` and `XFRC`.
    Done,
    /// The device had nothing to send; latch `NAK` (+`CHH`, the channel
    /// halts and firmware re-arms, as on silicon).
    Nak,
    /// The endpoint is halted or the request unsupported; `STALL` + `CHH`.
    Stall,
    /// No device, or the port is not enabled; `TXERR` + `CHH`.
    NoDevice,
    /// The FIFO did not hold the programmed OUT bytes; `TXERR` + `CHH`.
    TxErr,
}

/// One control transfer on the virtual device's endpoint 0.
#[derive(Clone, Debug, Default)]
struct CtrlXfer {
    setup: [u8; 8],
    /// Pending IN data stage bytes and how many the host took so far.
    data_in: Vec<u8>,
    data_in_off: usize,
    /// OUT data stage still expected (e.g. `SET_LINE_CODING`).
    data_out_expected: usize,
    data_out: Vec<u8>,
    /// Address to apply once the status stage completes.
    pending_addr: Option<u8>,
    /// The data stage ran host-to-device, so the status stage is IN.
    status_in: bool,
}

/// A virtual full-speed USB device behind the host port: a CDC-ACM function
/// with a control endpoint and bulk IN/OUT plus interrupt IN endpoints.
/// Tests preload IN data with [`VirtualUsbDevice::push_in`] and collect OUT
/// data with [`VirtualUsbDevice::take_out`]; the standard enumeration
/// requests (descriptors, address, configuration, line coding) are answered
/// from built-in descriptors.
#[derive(Clone, Debug)]
pub struct VirtualUsbDevice {
    address: u8,
    configuration: u8,
    ctrl: Option<CtrlXfer>,
    in_data: [Vec<u8>; 4],
    out_data: [Vec<u8>; 4],
    halted_in: [bool; 4],
    halted_out: [bool; 4],
    line_coding: [u8; 7],
}

impl Default for VirtualUsbDevice {
    fn default() -> Self {
        Self::new()
    }
}

/// Device descriptor: CDC class, 64-byte EP0, open VID/PID, three strings.
const VDEV_DEVICE_DESC: [u8; 18] = [
    0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x83, 0x04, 0x40, 0x57, 0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,
];

/// CDC-ACM configuration: COMM interface with header/call-mgmt/ACM/union
/// function descriptors and an interrupt IN endpoint, plus a DATA interface
/// with bulk OUT/IN endpoints (`wTotalLength` 67).
const VDEV_CONFIG_DESC: [u8; 67] = [
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32, 0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02,
    0x01, 0x00, 0x05, 0x24, 0x00, 0x10, 0x01, 0x05, 0x24, 0x01, 0x00, 0x01, 0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01, 0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x0A, 0x09, 0x04, 0x01, 0x00,
    0x02, 0x0A, 0x00, 0x00, 0x00, 0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00, 0x07, 0x05, 0x81, 0x02,
    0x40, 0x00, 0x00,
];

/// String descriptors: language, manufacturer, product, serial.
const VDEV_STR_LANG: [u8; 4] = [0x04, 0x03, 0x09, 0x04];
const VDEV_STR_MAN: &[u8] = b"VioARM";
const VDEV_STR_PROD: &[u8] = b"VioARM Virtual CDC";
const VDEV_STR_SER: &[u8] = b"VIOARM0001";

/// Default line coding reported by `GET_LINE_CODING`: 115200 8-N-1.
const VDEV_LINE_CODING: [u8; 7] = [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08];

fn utf16le(s: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(2 + 2 * s.len());
    out.push(u8::try_from(2 + 2 * s.len()).unwrap_or(u8::MAX));
    out.push(0x03);
    for b in s {
        out.push(*b);
        out.push(0);
    }
    out
}

impl VirtualUsbDevice {
    /// A fresh CDC-ACM device at address 0, unconfigured.
    #[must_use]
    pub fn new() -> Self {
        Self {
            address: 0,
            configuration: 0,
            ctrl: None,
            in_data: Default::default(),
            out_data: Default::default(),
            halted_in: [false; 4],
            halted_out: [false; 4],
            line_coding: VDEV_LINE_CODING,
        }
    }

    /// The assigned device address (`SET_ADDRESS`).
    #[must_use]
    pub const fn address(&self) -> u8 {
        self.address
    }

    /// The active configuration (`SET_CONFIGURATION`).
    #[must_use]
    pub const fn configuration(&self) -> u8 {
        self.configuration
    }

    /// The line coding stored by `SET_LINE_CODING`.
    #[must_use]
    pub const fn line_coding(&self) -> [u8; 7] {
        self.line_coding
    }

    /// Stage bytes for the device to send on an IN endpoint (1-3).
    pub fn push_in(&mut self, ep: usize, data: &[u8]) {
        if let Some(buf) = self.in_data.get_mut(ep) {
            buf.extend_from_slice(data);
        }
    }

    /// Drain what the host sent to an OUT endpoint (1-3).
    #[must_use]
    pub fn take_out(&mut self, ep: usize) -> Vec<u8> {
        self.out_data
            .get_mut(ep)
            .map(core::mem::take)
            .unwrap_or_default()
    }

    /// A SETUP packet arrived: decode the standard request and stage the
    /// data phase. Returns `false` for an unsupported request (STALL).
    fn setup(&mut self, pkt: &[u8]) -> bool {
        if pkt.len() < 8 {
            return false;
        }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&pkt[..8]);
        let req_type = raw[0];
        let req = raw[1];
        let value = u16::from_le_bytes([raw[2], raw[3]]);
        let _index = u16::from_le_bytes([raw[4], raw[5]]);
        let length = u16::from_le_bytes([raw[6], raw[7]]);
        let mut xfer = CtrlXfer {
            setup: raw,
            ..CtrlXfer::default()
        };
        let dir_in = req_type & 0x80 != 0;
        let std = req_type & 0x60 == 0;
        let ok = if std {
            match req {
                // GET_STATUS: two zero bytes for device/interface/endpoints.
                0 if dir_in => {
                    xfer.data_in = vec![0, 0];
                    true
                }
                // CLEAR_FEATURE: un-halt an endpoint, else ignore.
                1 if !dir_in && req_type & 0x0F == 2 && value == 0 => {
                    let ep = usize::from(raw[4] & 0x0F);
                    if raw[4] & 0x80 != 0 {
                        self.halted_in[ep.min(3)] = false;
                    } else {
                        self.halted_out[ep.min(3)] = false;
                    }
                    true
                }
                // SET_FEATURE: halt an endpoint (only ENDPOINT_HALT is real).
                3 if !dir_in && req_type & 0x0F == 2 && value == 0 => {
                    let ep = usize::from(raw[4] & 0x0F);
                    if raw[4] & 0x80 != 0 {
                        self.halted_in[ep.min(3)] = true;
                    } else {
                        self.halted_out[ep.min(3)] = true;
                    }
                    true
                }
                // SET_ADDRESS: applies after the status stage.
                5 if !dir_in => {
                    xfer.pending_addr = Some((value & 0x7F) as u8);
                    true
                }
                // GET_DESCRIPTOR: device, configuration, or string.
                6 if dir_in => {
                    let bytes: Vec<u8> = match (value >> 8) as u8 {
                        1 => VDEV_DEVICE_DESC.to_vec(),
                        2 => VDEV_CONFIG_DESC.to_vec(),
                        3 => match value as u8 {
                            0 => VDEV_STR_LANG.to_vec(),
                            1 => utf16le(VDEV_STR_MAN),
                            2 => utf16le(VDEV_STR_PROD),
                            3 => utf16le(VDEV_STR_SER),
                            _ => return false,
                        },
                        _ => return false,
                    };
                    let n = usize::from(length).min(bytes.len());
                    xfer.data_in = bytes[..n].to_vec();
                    true
                }
                // GET_CONFIGURATION.
                8 if dir_in => {
                    xfer.data_in = vec![self.configuration];
                    true
                }
                // SET_CONFIGURATION.
                9 if !dir_in => {
                    self.configuration = (value & 0xFF) as u8;
                    true
                }
                // GET_INTERFACE: alternate setting 0 for a known interface.
                10 if dir_in && (raw[4] == 0 || raw[4] == 1) => {
                    xfer.data_in = vec![0];
                    true
                }
                // SET_INTERFACE: accept alternate 0.
                11 if !dir_in && raw[3] == 0 => true,
                _ => false,
            }
        } else if req_type & 0x60 == 0x20 {
            // CDC class requests on the COMM interface.
            match req {
                // GET_LINE_CODING.
                0x21 if dir_in => {
                    xfer.data_in = self.line_coding.to_vec();
                    true
                }
                // SET_LINE_CODING: 7 OUT data bytes follow.
                0x20 if !dir_in => {
                    xfer.data_out_expected = 7;
                    true
                }
                // SET_CONTROL_LINE_STATE: no data stage.
                0x22 if !dir_in => true,
                _ => false,
            }
        } else {
            false
        };
        if ok {
            xfer.status_in = !dir_in || xfer.data_out_expected > 0;
            self.ctrl = Some(xfer);
        }
        ok
    }

    /// An IN token on endpoint 0: the next data chunk; an empty packet while
    /// the data phase drained is the ZLP terminator (state kept); an empty
    /// packet with nothing programmed is the status stage (the transfer
    /// completes and a pending address latches).
    fn control_in(&mut self, mpsiz: usize, xfersiz: usize) -> HostXferData {
        let Some(xfer) = self.ctrl.as_mut() else {
            return HostXferData::Stall;
        };
        if xfer.data_in_off < xfer.data_in.len() {
            let n = mpsiz
                .max(1)
                .min(xfersiz)
                .min(xfer.data_in.len() - xfer.data_in_off);
            let chunk = xfer.data_in[xfer.data_in_off..xfer.data_in_off + n].to_vec();
            xfer.data_in_off += n;
            HostXferData::Data(chunk)
        } else if xfer.data_out_expected > 0 {
            // The OUT data stage is still pending: wrong direction.
            HostXferData::Stall
        } else if xfersiz == 0 {
            // Status stage (IN): the transfer is done.
            if let Some(addr) = xfer.pending_addr {
                self.address = addr;
            }
            self.ctrl = None;
            HostXferData::Data(Vec::new())
        } else {
            // Zero-length packet terminating an exactly-sized read.
            HostXferData::Data(Vec::new())
        }
    }

    /// An OUT packet on endpoint 0: data-stage bytes, or the status-stage
    /// completion (empty payload) for transfers whose status is OUT.
    fn control_out(&mut self, payload: &[u8]) -> HostXferData {
        let Some(xfer) = self.ctrl.as_mut() else {
            return HostXferData::Stall;
        };
        if xfer.data_out_expected > 0 {
            xfer.data_out.extend_from_slice(payload);
            if xfer.data_out.len() >= xfer.data_out_expected {
                // SET_LINE_CODING stores its 7 bytes; the status IN follows.
                if xfer.setup[1] == 0x20 && xfer.data_out.len() >= 7 {
                    self.line_coding.copy_from_slice(&xfer.data_out[..7]);
                }
                xfer.data_out_expected = 0;
            }
            HostXferData::Data(Vec::new())
        } else if payload.is_empty() && !xfer.status_in && xfer.data_in_off >= xfer.data_in.len() {
            // Status stage (OUT): the transfer is done.
            if let Some(addr) = xfer.pending_addr {
                self.address = addr;
            }
            self.ctrl = None;
            HostXferData::Data(Vec::new())
        } else {
            HostXferData::Stall
        }
    }

    /// An IN token on a bulk/interrupt endpoint: staged bytes, `None` for
    /// NAK (nothing to send), STALL when halted or unknown.
    fn ep_in(&mut self, ep: usize, max: usize) -> HostXferData {
        if ep == 0 || ep > 3 || *self.halted_in.get(ep).unwrap_or(&true) {
            return HostXferData::Stall;
        }
        let buf = &mut self.in_data[ep];
        if buf.is_empty() {
            return HostXferData::Nak;
        }
        let n = max.max(1).min(buf.len());
        HostXferData::Data(buf.drain(..n).collect())
    }

    /// An OUT packet on a bulk/interrupt endpoint: buffered for the test.
    fn ep_out(&mut self, ep: usize, payload: &[u8]) -> HostXferData {
        if ep == 0 || ep > 3 || *self.halted_out.get(ep).unwrap_or(&true) {
            return HostXferData::Stall;
        }
        self.out_data[ep].extend_from_slice(payload);
        HostXferData::Data(Vec::new())
    }
}

/// What the virtual device hands back for one token: data bytes (possibly
/// empty for a status handshake), a NAK, or a STALL.
#[derive(Clone, Debug, PartialEq, Eq)]
enum HostXferData {
    Data(Vec<u8>),
    Nak,
    Stall,
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
    /// An isochronous IN endpoint armed but not yet completed; its transfer
    /// lands at the next SOF (`start_in` defers, `sof` completes).
    iso_pending_in: Option<usize>,
    /// Host port state machine (power/connect/reset/enable).
    port: HostPort,
    /// SOFs left in a port reset; the enable lands at zero.
    reset_sofs: u8,
    /// Latched host-channel interrupts (`HCINT` per channel).
    hcint: [u32; MAX_HOST_CHANNELS],
    /// DATA0/DATA1 toggle per host channel (`HCTSIZ.DPID` write-back).
    ch_toggle: [bool; MAX_HOST_CHANNELS],
    /// OUT payload staged through the FIFO window while in host mode,
    /// consumed front-to-back by channel OUT transactions.
    host_out: Vec<u8>,
    /// The virtual device behind the port (`None` = nothing attached).
    device: Option<VirtualUsbDevice>,
    /// Double-buffer DATA PID per IN/OUT endpoint (R3): SD0PID/SD1PID latch.
    in_data_pid: [bool; 4],
    out_data_pid: [bool; 4],
    /// Odd/even frame for isochronous double-buffer (`SODDFRM`/`SEVNFRM`).
    in_odd_frame: [bool; 4],
    out_odd_frame: [bool; 4],
}

impl UsbState {
    /// The full `GINTSTS` word: latched events plus the synthesized flags
    /// (`NPTXFE`, and `IEPINT`/`OEPINT` gated by the endpoint masks). In
    /// host mode the device bits are replaced by `HPRTINT` (any HPRT change
    /// bit set) and `HCINT` (any unmasked channel interrupt in `HAINT`).
    fn gintsts(&self, g: &Win, h: &Win, d: &Win) -> u32 {
        if g.get(GUSBCFG) & FDMOD == 0 {
            let hprt = h.get(HPRT);
            let mut word = self.events | NPTXFE;
            if hprt & HPRT_CHNG != 0 {
                word |= HPRTINT;
            }
            if self.haint(h) & h.get(HAINTMSK) != 0 {
                word |= HCINT_BIT;
            }
            return word;
        }
        let in_mask = d.get(DIEPMSK);
        let out_mask = d.get(DOEPMSK);
        let iep = (0..4).any(|i| self.diepint[i] & !TXFE & in_mask != 0);
        let oep = (0..4).any(|i| self.doepint[i] & !TXFE & out_mask != 0);
        self.events | NPTXFE | if iep { IEPINT } else { 0 } | if oep { OEPINT } else { 0 }
    }

    /// The single NVIC line: asserted once the global interrupt enable is set
    /// and any `GINTSTS` bit is unmasked.
    fn pending(&self, g: &Win, h: &Win, d: &Win) -> u32 {
        if g.get(GAHBCFG) & GINT_EN == 0 {
            return 0;
        }
        u32::from(self.gintsts(g, h, d) & g.get(GINTMSK) != 0)
    }

    /// `HAINT`: one bit per channel with a latched, unmasked interrupt.
    fn haint(&self, h: &Win) -> u32 {
        let mut word = 0u32;
        for ch in 0..MAX_HOST_CHANNELS {
            let mask = self.hcint_mask(h, ch);
            if mask != 0 && self.hcint[ch] & mask != 0 {
                word |= 1 << ch;
            }
        }
        word
    }

    /// The `HCINTMSKn` word, or zero when the channel is not in the tables.
    fn hcint_mask(&self, h: &Win, ch: usize) -> u32 {
        let off = HCINTMSK0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
        if h.def.registers.iter().any(|r| r.offset == off) {
            h.get(off)
        } else {
            0
        }
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
        let tsz = d.get(DIEPTSIZ0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE);
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
        let tsz = d.get(DOEPTSIZ0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE);
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
            let reg = DOEPCTL0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
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
        let off = DIEPCTL0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
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
        let off = DOEPCTL0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
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
        self.iso_pending_in = None;
    }

    // ---- host mode ----

    /// The port gained or lost power through `HPRT.PPWR`. Powering up with
    /// a device attached latches the connect-detected change bit.
    fn port_power(&mut self, h: &mut Win, on: bool) {
        if on {
            if self.device.is_some() {
                self.port = HostPort::Connected;
                h.set(HPRT, h.get(HPRT) | PCDET);
            } else {
                self.port = HostPort::Powered;
            }
        } else {
            self.port = HostPort::NoPower;
            self.reset_sofs = 0;
            h.set(HPRT, h.get(HPRT) & !(PENA | PRST));
        }
    }

    /// Attach a virtual device; with the port powered this is the connect
    /// event real firmware enumerates from.
    fn host_plug(&mut self, h: &mut Win, dev: VirtualUsbDevice) {
        self.device = Some(dev);
        if self.port == HostPort::Powered {
            self.port = HostPort::Connected;
            h.set(HPRT, h.get(HPRT) | PCDET);
        }
    }

    /// Detach the device: the enable drops and the change bit latches, as
    /// on a physical disconnect.
    fn host_unplug(&mut self, h: &mut Win) -> Option<VirtualUsbDevice> {
        let dev = self.device.take();
        match self.port {
            HostPort::Connected | HostPort::Resetting | HostPort::Enabled => {
                self.port = HostPort::Powered;
                let mut word = h.get(HPRT);
                word &= !(PENA | PRST);
                word |= PENCHNG;
                h.set(HPRT, word);
            }
            HostPort::Powered | HostPort::NoPower => {}
        }
        self.reset_sofs = 0;
        dev
    }

    /// `HPRT` read: the stored word with the core-owned bits (connect,
    /// enable, reset-in-progress, speed) forced from the port state.
    fn hprt_read(&self, h: &Win) -> u32 {
        let mut word = h.get(HPRT);
        word &= !(PCSTS | PENA | PRST | (0b11 << PRTSPD_SHIFT));
        match self.port {
            HostPort::NoPower | HostPort::Powered => {}
            HostPort::Connected => word |= PCSTS,
            HostPort::Resetting => word |= PCSTS | PRST,
            HostPort::Enabled => word |= PCSTS | PENA | (PRTSPD_FS << PRTSPD_SHIFT),
        }
        word
    }

    /// `HPRT` write: `PPWR` powers the port, a rising `PRST` starts the
    /// SOF-counted reset, change bits are write-one-to-clear, `PENA` only
    /// clears (the core sets it), and `PRES`/`PSUSP` store through without
    /// behavior (no suspend/resume signaling is modeled).
    fn write_hprt(&mut self, h: &mut Win, l: u32, val: u8) {
        let cur = h.get(HPRT);
        let written = u32::from(val) << (8 * l);
        // Only the written lane can set, clear, or edge-trigger a bit: the
        // other lanes of a multi-byte write must not disturb their bits.
        let in_lane = |bit: u32| (bit >> (8 * l)) & 0xFF != 0;
        // Power and reset mutate the stored word directly (connect-detect
        // latches), so the working copy is re-read after each side effect.
        if written & PPWR != 0 && cur & PPWR == 0 {
            self.port_power(h, true);
        } else if in_lane(PPWR) && written & PPWR == 0 && cur & PPWR != 0 {
            self.port_power(h, false);
        }
        let mut word = h.get(HPRT);
        if in_lane(PPWR) {
            word = (word & !PPWR) | (written & PPWR);
        }
        if written & PRST != 0 && cur & PRST == 0 {
            if self.port == HostPort::Connected {
                self.port = HostPort::Resetting;
                self.reset_sofs = PORT_RESET_SOFS;
                word |= PRST;
            }
        } else if in_lane(PRST) && written & PRST == 0 && cur & PRST != 0 {
            if self.port == HostPort::Resetting {
                self.port = HostPort::Connected;
            }
            word &= !PRST;
        }
        word &= !(written & HPRT_CHNG);
        if written & PENA != 0 && cur & PENA == 0 && self.port == HostPort::Connected {
            // A direct firmware enable (the reset path normally does this).
            self.port = HostPort::Enabled;
            word |= PENA | PENCHNG;
        } else if in_lane(PENA) && written & PENA == 0 && cur & PENA != 0 {
            word &= !PENA;
            if self.port == HostPort::Enabled {
                self.port = HostPort::Connected;
            }
        }
        const RESUME_SUSPEND: u32 = (1 << 5) | (1 << 6);
        word = (word & !RESUME_SUSPEND) | (written & RESUME_SUSPEND);
        h.set(HPRT, word);
    }

    /// One SOF tick of the port reset: the enable lands at zero, with the
    /// change bit set for the stack to find.
    fn port_reset_tick(&mut self, h: &mut Win) {
        if self.port != HostPort::Resetting {
            return;
        }
        if self.reset_sofs > 0 {
            self.reset_sofs -= 1;
        }
        if self.reset_sofs == 0 {
            self.port = HostPort::Enabled;
            let mut word = h.get(HPRT);
            word |= PENA | PENCHNG;
            word &= !PRST;
            h.set(HPRT, word);
        }
    }

    /// `HFNUM`: the frame counter plus the reload value as the remaining
    /// time. Without a cycle clock in the controller the remainder is
    /// constant within a frame; firmware SOF delays still observe it tick
    /// down frame to frame only through `FRNUM`.
    fn hfnum(&self, h: &Win) -> u32 {
        (self.frame & 0xFFFF) | ((h.get(HFIR) & 0xFFFF) << 16)
    }

    /// `HCCHARn` write: `CHDIS` halts an armed channel, a rising `CHENA`
    /// runs the programmed transfer. Periodic transfers (interrupt-type and
    /// isochronous) defer to the next SOF (the periodic schedule, one packet
    /// per frame); everything else completes synchronously, exactly like the
    /// device-side `EPENA` path.
    fn write_hcchar(&mut self, h: &mut Win, ch: usize, l: u32, val: u8) {
        let reg = HCCHAR0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
        let cur = h.get(reg);
        let attempted = h.merged(reg, val, 8 * l);
        // `CHDIS` is a command bit (never stored); `CHENA` latches on a
        // rising edge and the core drops it on completion or halt.
        let word = attempted & !HC_CHDIS;
        h.set(reg, word);
        if attempted & HC_CHDIS != 0 && cur & HC_CHENA != 0 {
            h.set(reg, word & !HC_CHENA);
            self.hcint[ch] |= HC_CHH;
        } else if attempted & HC_CHENA != 0 && cur & HC_CHENA == 0 {
            let eptyp = (attempted >> HC_EPTYP_SHIFT) & 0x3;
            if eptyp != EPTYP_INTR && eptyp != EPTYP_ISO {
                self.execute_channel(h, ch);
            }
        }
    }

    /// Run one channel program against the virtual device.
    fn execute_channel(&mut self, h: &mut Win, ch: usize) {
        let char_word = h.get(HCCHAR0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE);
        let tsiz = h.get(HCTSIZ0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE);
        let ep = usize::try_from((char_word >> HC_EPNUM_SHIFT) & HC_EPNUM).unwrap_or(usize::MAX);
        let dir_in = char_word & HC_EPDIR != 0;
        let eptyp = (char_word >> HC_EPTYP_SHIFT) & 0x3;
        let mpsiz = usize::try_from(char_word & HC_MPSIZ).unwrap_or(0);
        let xfersiz = usize::try_from(tsiz & HCTSIZ_XFRSIZ).unwrap_or(0);
        let pktcnt = tsiz >> HCTSIZ_PKTCNT_SHIFT;
        let (res, sent, packets) = self.run_transaction(ch, eptyp, ep, dir_in, mpsiz, xfersiz);
        self.finish_channel(h, ch, res, sent, packets, xfersiz, pktcnt);
    }

    /// One transaction: talk to the virtual device, queue IN data to the RX
    /// FIFO, and report what happened plus bytes moved and packets used.
    fn run_transaction(
        &mut self,
        ch: usize,
        eptyp: u32,
        ep: usize,
        dir_in: bool,
        mpsiz: usize,
        xfersiz: usize,
    ) -> (HostXfer, usize, u32) {
        if self.port != HostPort::Enabled || self.device.is_none() {
            return (HostXfer::NoDevice, 0, 0);
        }
        // Each arm below talks to the device in its own statement so the
        // borrow ends before the completion is queued: staging drains and
        // FIFO enqueues never alias the device borrow.
        if eptyp == EPTYP_CTRL && ep != 0 {
            return (HostXfer::Stall, 0, 0);
        }
        if eptyp == EPTYP_CTRL && dir_in {
            let out = self
                .device
                .as_mut()
                .map_or(HostXferData::Stall, |dev| dev.control_in(mpsiz, xfersiz));
            return match out {
                HostXferData::Data(chunk) => {
                    let sent = chunk.len();
                    if !chunk.is_empty() {
                        let toggle = u32::from(self.ch_toggle[ch]);
                        self.enqueue(0, PKTSTS_HOST_IN, toggle, &chunk);
                    }
                    (HostXfer::Done, sent, 1)
                }
                HostXferData::Nak => (HostXfer::Nak, 0, 0),
                HostXferData::Stall => (HostXfer::Stall, 0, 0),
            };
        }
        if eptyp == EPTYP_CTRL {
            // Control OUT: with no transfer open the staged bytes are the
            // 8-byte SETUP packet; otherwise data/status stage bytes.
            let open = self.device.as_ref().is_some_and(|dev| dev.ctrl.is_some());
            if !open {
                if self.host_out.len() < 8 {
                    return (HostXfer::TxErr, 0, 0);
                }
                let setup: Vec<u8> = self.host_out.drain(..8).collect();
                self.ch_toggle[ch] = false;
                let ok = self.device.as_mut().is_some_and(|dev| dev.setup(&setup));
                return if ok {
                    (HostXfer::Done, 8, 1)
                } else {
                    (HostXfer::Stall, 0, 0)
                };
            }
            let take = self.host_out.len().min(xfersiz);
            let payload: Vec<u8> = self.host_out.drain(..take).collect();
            let out = self
                .device
                .as_mut()
                .map_or(HostXferData::Stall, |dev| dev.control_out(&payload));
            return match out {
                HostXferData::Data(_) => (HostXfer::Done, take, 1),
                HostXferData::Nak => (HostXfer::Nak, 0, 0),
                HostXferData::Stall => (HostXfer::Stall, 0, 0),
            };
        }
        if eptyp == EPTYP_ISO {
            // Isochronous: one packet per SOF, no handshake, no retry. The
            // channel runs on the periodic schedule (`host_sof`); short or
            // empty frames are legal (variable audio samples, silence), so a
            // short FIFO sends what it holds instead of erroring, and an
            // empty IN stage completes as a zero-length packet rather than
            // NAKing. `packets` stays 1 so the DATA0/DATA1 toggle advances
            // every frame.
            if dir_in {
                let max = mpsiz.min(xfersiz.max(1));
                let out = self
                    .device
                    .as_mut()
                    .map_or(HostXferData::Stall, |dev| dev.ep_in(ep, max));
                return match out {
                    HostXferData::Data(chunk) => {
                        let sent = chunk.len();
                        if !chunk.is_empty() {
                            let toggle = u32::from(self.ch_toggle[ch]);
                            self.enqueue(
                                u8::try_from(ep).unwrap_or(u8::MAX),
                                PKTSTS_HOST_IN,
                                toggle,
                                &chunk,
                            );
                        }
                        (HostXfer::Done, sent, 1)
                    }
                    HostXferData::Nak => (HostXfer::Done, 0, 1),
                    HostXferData::Stall => (HostXfer::Stall, 0, 0),
                };
            }
            let take = self.host_out.len().min(xfersiz);
            let payload: Vec<u8> = self.host_out.drain(..take).collect();
            let out = self
                .device
                .as_mut()
                .map_or(HostXferData::Stall, |dev| dev.ep_out(ep, &payload));
            return match out {
                HostXferData::Data(_) => (HostXfer::Done, take, 1),
                HostXferData::Nak => (HostXfer::Done, take, 1),
                HostXferData::Stall => (HostXfer::Stall, 0, 0),
            };
        }
        if eptyp == EPTYP_BULK || eptyp == EPTYP_INTR {
            if dir_in {
                let max = mpsiz.min(xfersiz.max(1));
                let out = self
                    .device
                    .as_mut()
                    .map_or(HostXferData::Stall, |dev| dev.ep_in(ep, max));
                return match out {
                    HostXferData::Data(chunk) => {
                        let sent = chunk.len();
                        let packets = packet_count(sent, mpsiz);
                        if !chunk.is_empty() {
                            let toggle = u32::from(self.ch_toggle[ch]);
                            self.enqueue(
                                u8::try_from(ep).unwrap_or(u8::MAX),
                                PKTSTS_HOST_IN,
                                toggle,
                                &chunk,
                            );
                        }
                        (HostXfer::Done, sent, packets)
                    }
                    HostXferData::Nak => (HostXfer::Nak, 0, 0),
                    HostXferData::Stall => (HostXfer::Stall, 0, 0),
                };
            }
            let take = self.host_out.len().min(xfersiz);
            if take < xfersiz {
                // The FIFO did not hold the programmed bytes: underrun.
                return (HostXfer::TxErr, 0, 0);
            }
            let payload: Vec<u8> = self.host_out.drain(..take).collect();
            let out = self
                .device
                .as_mut()
                .map_or(HostXferData::Stall, |dev| dev.ep_out(ep, &payload));
            match out {
                HostXferData::Data(_) => (HostXfer::Done, take, packet_count(take, mpsiz)),
                HostXferData::Nak => (HostXfer::Nak, 0, 0),
                HostXferData::Stall => (HostXfer::Stall, 0, 0),
            }
        } else {
            // Unreachable: `eptyp` is 2 bits and all four types arm above.
            // STALL defensively rather than mis-transferring.
            (HostXfer::Stall, 0, 0)
        }
    }

    /// Latch the channel outcome, drop `CHENA`, and write the `HCTSIZ`
    /// remainder back so the stack sees what moved.
    #[allow(clippy::too_many_arguments)]
    fn finish_channel(
        &mut self,
        h: &mut Win,
        ch: usize,
        res: HostXfer,
        sent: usize,
        packets: u32,
        xfersiz: usize,
        pktcnt: u32,
    ) {
        match res {
            HostXfer::Done => {
                self.hcint[ch] |= HC_ACK | HC_XFRC;
                if packets & 1 == 1 {
                    self.ch_toggle[ch] = !self.ch_toggle[ch];
                }
            }
            HostXfer::Nak => self.hcint[ch] |= HC_NAK | HC_CHH,
            HostXfer::Stall => self.hcint[ch] |= HC_STALL | HC_CHH,
            HostXfer::NoDevice | HostXfer::TxErr => self.hcint[ch] |= HC_TXERR | HC_CHH,
        }
        let ch_reg = HCCHAR0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
        h.set(ch_reg, h.get(ch_reg) & !HC_CHENA);
        let tsz_reg = HCTSIZ0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
        let rem = u32::try_from(xfersiz.saturating_sub(sent)).unwrap_or(u32::MAX);
        let rpc = pktcnt.saturating_sub(packets);
        h.set(
            tsz_reg,
            (u32::from(self.ch_toggle[ch]) << HCTSIZ_DPID_SHIFT)
                | (rpc << HCTSIZ_PKTCNT_SHIFT)
                | (rem & HCTSIZ_XFRSIZ),
        );
    }

    /// The SOF scheduler: frame event, port-reset countdown, and the
    /// deferred periodic channels (interrupt-type and isochronous, one
    /// packet per frame each).
    fn host_sof(&mut self, h: &mut Win, nchan: usize) {
        self.events |= SOF;
        self.port_reset_tick(h);
        for ch in 0..nchan {
            let char_word = h.get(HCCHAR0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE);
            let eptyp = (char_word >> HC_EPTYP_SHIFT) & 0x3;
            if char_word & HC_CHENA != 0 && (eptyp == EPTYP_INTR || eptyp == EPTYP_ISO) {
                self.execute_channel(h, ch);
            }
        }
    }
}

/// Packets a transfer of `sent` bytes takes at `mpsiz` bytes per packet.
fn packet_count(sent: usize, mpsiz: usize) -> u32 {
    if sent == 0 {
        return 1;
    }
    u32::try_from(sent.saturating_add(mpsiz.max(1) - 1) / mpsiz.max(1)).unwrap_or(u32::MAX)
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
    /// Host channels in the SVD tables (8 on FS cores, up to 16 on HS).
    nchan: usize,
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

    /// The second HS controller at `0x4008_0000` on H7 (`OTG2_HS`).
    #[must_use]
    pub fn otg2_hs_for<D: Device>() -> Self {
        Self::for_device::<D>("OTG2_HS")
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
        let h = Win::new(win_def::<D>(irq, "HOST"));
        let nchan = (0..MAX_HOST_CHANNELS)
            .take_while(|ch| {
                let off = HCCHAR0 + u32::try_from(*ch).unwrap_or(u32::MAX) * HC_STRIDE;
                h.def.registers.iter().any(|r| r.offset == off)
            })
            .count()
            .max(1);
        Self {
            ranges: [
                Region::new(base, gdef.window),
                Region::new(base + HOST_OFFSET, 0x400),
                Region::new(base + DEVICE_OFFSET, 0x400),
                Region::new(base + PWRCLK_OFFSET, FIFO_OFFSET - PWRCLK_OFFSET),
                Region::new(base + FIFO_OFFSET, FIFO_LEN),
            ],
            g: Win::new(gdef),
            h,
            d: Win::new(win_def::<D>(irq, "DEVICE")),
            p: Win::new(win_def::<D>(irq, "PWRCLK")),
            state: UsbState::default(),
            exception,
            name: irq,
            nchan,
        }
    }

    /// Attach a virtual USB device to the host port. With the port powered
    /// this raises the connect-detected change bit, exactly like plugging
    /// a cable into a powered host.
    pub fn plug_virtual_device(&mut self, dev: VirtualUsbDevice) {
        self.state.host_plug(&mut self.h, dev);
    }

    /// Detach the virtual device, dropping the port enable with the change
    /// bit set, as on a physical disconnect. Returns the device, if any.
    pub fn unplug_virtual_device(&mut self) -> Option<VirtualUsbDevice> {
        self.state.host_unplug(&mut self.h)
    }

    /// The attached virtual device, if any.
    #[must_use]
    pub const fn virtual_device(&self) -> Option<&VirtualUsbDevice> {
        self.state.device.as_ref()
    }

    /// The attached virtual device, for staging IN data or collecting OUT
    /// data around a host-stack run.
    pub fn virtual_device_mut(&mut self) -> Option<&mut VirtualUsbDevice> {
        self.state.device.as_mut()
    }

    /// Host channels in this controller's tables.
    #[must_use]
    pub const fn host_channels(&self) -> usize {
        self.nchan
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
        "OTG_HS" => &["OTG_HS", "OTG1_HS"],
        "OTG2_HS" => &["OTG2_HS"],
        "OTG_FS" => &["USB_OTG", "OTG_FS", "OTG1_HS", "OTG2_HS"],
        _ => &["USB_OTG", "OTG_FS", "OTG_HS", "OTG1_HS", "OTG2_HS"],
    }
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
            return self.read_host(addr - (base + HOST_OFFSET));
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
            if self.g.get(GUSBCFG) & FDMOD == 0 {
                // Host mode: OUT payload stages here, consumed front to
                // back by the next channel OUT transaction.
                self.state.host_out.push(val);
                return Ok(());
            }
            let ep = usize::try_from((addr - (base + FIFO_OFFSET)) / 0x1000).unwrap_or(0);
            self.state.tx_fifo[ep].push(val);
            return Ok(());
        }
        let off = addr.wrapping_sub(base);
        if off < HOST_OFFSET {
            return self.write_global(off, val);
        }
        if addr >= base + HOST_OFFSET && addr < base + DEVICE_OFFSET {
            return self.write_host(addr - (base + HOST_OFFSET), val);
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
                if self.g.get(GUSBCFG) & FDMOD == 0 {
                    // Host mode: this frame is emitted by the controller.
                    let nchan = self.nchan;
                    self.state.host_sof(&mut self.h, nchan);
                } else {
                    self.state.events |= SOF;
                    // An armed isochronous IN transfer lands on this boundary.
                    if let Some(ep) = self.state.iso_pending_in.take() {
                        self.state.start_in(&self.d, ep);
                    }
                }
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
        self.state.pending(&self.g, &self.h, &self.d)
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

impl Usb {
    fn read_global(&mut self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, GINTSTS) {
            let word = self.state.gintsts(&self.g, &self.h, &self.d);
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
        if (DIEPCTL0..DIEPCTL0 + 4 * 0x20).contains(&off) {
            let (ep, sub, l) = ep_block(off, DIEPCTL0);
            let word = if sub < 8 {
                self.state.diepctl(&self.d, ep)
            } else if sub < 16 {
                self.state.diepint(ep)
            } else if sub < 24 {
                let reg = DIEPTSIZ0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
                self.d.get(reg)
            } else {
                self.state.dtxfsts(ep)
            };
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if (DOEPCTL0..DOEPCTL0 + 4 * 0x20).contains(&off) {
            let (ep, sub, l) = ep_block(off, DOEPCTL0);
            let word = if sub < 8 {
                self.state.doepctl(&self.d, ep)
            } else if sub < 16 {
                self.state.doepint(ep)
            } else if sub < 24 {
                let reg = DOEPTSIZ0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
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
        if (DIEPCTL0..DIEPCTL0 + 4 * 0x20).contains(&off) {
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
        if (DOEPCTL0..DOEPCTL0 + 4 * 0x20).contains(&off) {
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
        let reg = DIEPCTL0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
        let mut merged = self.d.merged(reg, val, 8 * l);
        if l == 3 {
            if val & 0x80 != 0 {
                // Isochronous IN transfers defer to the SOF boundary
                // (RM0008 §22.4.2: one packet per frame).
                let eptype = (self.d.get(reg) >> EPTYPE_SHIFT) & 0x3;
                if eptype == EPTYPE_ISO {
                    self.state.iso_pending_in = Some(ep);
                    merged &= !EPENA;
                } else {
                    self.state.start_in(&self.d, ep);
                    merged &= !EPENA;
                }
            }
            if val & 0x08 != 0 {
                self.state.in_ep[ep].nak = true;
                merged &= !SNAK;
            }
            if val & 0x04 != 0 {
                self.state.in_ep[ep].nak = false;
                merged &= !CNAK;
            }
            // R3 double-buffer: SD0PID/SEVNFRM (bit 28) → DATA0/even, SD1PID/SODDFRM (bit 29) → DATA1/odd
            if val & 0x10 != 0 {
                self.state.in_data_pid[ep] = false;
                self.state.in_odd_frame[ep] = false;
                merged &= !(1 << 28);
            }
            if val & 0x20 != 0 {
                self.state.in_data_pid[ep] = true;
                self.state.in_odd_frame[ep] = true;
                merged &= !(1 << 29);
            }
        }
        self.d.set(reg, merged);
    }

    fn write_doepctl(&mut self, ep: usize, l: u32, val: u8) {
        let reg = DOEPCTL0 + u32::try_from(ep).unwrap_or(0) * EP_STRIDE;
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
            // R3 double-buffer for OUT: SD0PID/SEVNFRM → DATA0/even, SD1PID/SODDFRM → DATA1/odd
            if val & 0x10 != 0 {
                self.state.out_data_pid[ep] = false;
                self.state.out_odd_frame[ep] = false;
                merged &= !(1 << 28);
            }
            if val & 0x20 != 0 {
                self.state.out_data_pid[ep] = true;
                self.state.out_odd_frame[ep] = true;
                merged &= !(1 << 29);
            }
        }
        self.d.set(reg, merged);
    }

    fn soft_reset(&mut self) {
        self.g.reload_reset();
        self.h.reload_reset();
        self.d.reload_reset();
        self.p.reload_reset();
        // A core soft reset reloads the registers (HPRT loses PPWR, so the
        // port goes dark) but the cable stays plugged in.
        let device = self.state.device.take();
        self.state = UsbState::default();
        self.state.device = device;
    }

    /// HOST window read: `HFNUM`/`HAINT`/`HPRT` synthesize core-owned bits,
    /// `HCINTn` reads the latched channel flags, everything else latches.
    fn read_host(&self, off: u32) -> Result<u8, BusError> {
        if let Some(l) = lane(off, HFNUM) {
            let word = self.state.hfnum(&self.h);
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, HAINT) {
            let word = self.state.haint(&self.h);
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, HPRT) {
            let word = self.state.hprt_read(&self.h);
            return Ok(((word >> (8 * l)) & 0xFF) as u8);
        }
        if let Some(l) = lane(off, HPTXSTS) {
            // Periodic TX room always available: queue space 8, FIFO 0x100.
            const AVAILABLE: u32 = 0x0008_0100;
            return Ok(((AVAILABLE >> (8 * l)) & 0xFF) as u8);
        }
        if off >= HCCHAR0 {
            let rel = off - HCCHAR0;
            let ch = usize::try_from(rel / HC_STRIDE).unwrap_or(usize::MAX);
            let sub = rel % HC_STRIDE;
            if ch < MAX_HOST_CHANNELS && (8..12).contains(&sub) {
                let reg = HCINT0 + u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
                if self.h.def.registers.iter().any(|r| r.offset == reg) {
                    let word = self.state.hcint[ch];
                    let shift = 8 * (off - (reg & !3));
                    return Ok(((word >> shift) & 0xFF) as u8);
                }
            }
        }
        read_latch(&self.h, off)
    }

    /// HOST window write: `HPRT` runs the port machine, `HCCHARn` arms and
    /// halts channels, `HCINTn` is write-one-to-clear, the rest latches.
    fn write_host(&mut self, off: u32, val: u8) -> Result<(), BusError> {
        if lane(off, HPRT).is_some() {
            let l = off - HPRT;
            self.state.write_hprt(&mut self.h, l, val);
            return Ok(());
        }
        if off >= HCCHAR0 {
            let rel = off - HCCHAR0;
            let ch = usize::try_from(rel / HC_STRIDE).unwrap_or(usize::MAX);
            let sub = rel % HC_STRIDE;
            if ch < MAX_HOST_CHANNELS {
                let base = u32::try_from(ch).unwrap_or(u32::MAX) * HC_STRIDE;
                if sub < 4
                    && self
                        .h
                        .def
                        .registers
                        .iter()
                        .any(|r| r.offset == HCCHAR0 + base)
                {
                    let l = off - (HCCHAR0 + base);
                    self.state.write_hcchar(&mut self.h, ch, l, val);
                    return Ok(());
                }
                if (8..12).contains(&sub) {
                    let l = off - (HCINT0 + base);
                    let bits = u32::from(val) << (8 * l);
                    self.state.hcint[ch] &= !bits;
                    return Ok(());
                }
            }
        }
        write_latch(&mut self.h, off, val)
    }
}

/// Decompose an endpoint-block byte offset: the endpoint index, the byte
/// within its 0x20 block, and the byte lane within the register.
fn ep_block(off: u32, base: u32) -> (usize, u32, u32) {
    let rel = off - base;
    let ep = usize::try_from(rel / 0x20).unwrap_or(0);
    let sub = rel % 0x20;
    let l = rel % 4;
    (ep, sub, l)
}

#[cfg(test)]
mod tests {
    use super::{
        Usb, VirtualUsbDevice, CMD_OUT, CMD_RESET, CMD_SETUP, CMD_SOF, ENUMDNE, EVT_IN, RXFLVL,
        USBRST,
    };
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
        assert!(matches!(u.read(FS + 0x5000), Err(BusError::Reserved)));
        assert!(matches!(u.write(FS - 1, 0), Err(BusError::Reserved)));
        assert!(u.read(FS + 0x400).is_ok(), "HOST window");
        assert!(u.read(FS + 0x1000).is_ok(), "FIFO window");
        assert!(u.read(FS + 0x2000).is_ok(), "EP1 FIFO");
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

    #[test]
    fn ep1_bulk_in_and_out() {
        let mut u = usb_fs();
        init_device(&mut u);
        // Enable EP1 IN/OUT interrupts.
        write_word(&mut u, FS + 0x800 + 0x10, 1 | (1 << 1)); // DIEPMSK XFRC for EP0+EP1
        write_word(&mut u, FS + 0x800 + 0x14, 1 | (1 << 1)); // DOEPMSK
                                                             // IN bulk on EP1
        let payload = [0xAA, 0xBB, 0xCC, 0xDD];
        for (i, b) in payload.iter().enumerate() {
            u.write(FS + 0x1000 + 0x1000 + u32::try_from(i).unwrap(), *b)
                .unwrap();
        }
        // DIEPCTL1 EPENA etc is via 0x900? Actually EP1 is at 0x120
        write_word(&mut u, FS + 0x800 + 0x110 + 0x20, (1 << 19) | 4); // DIEPTSIZ1
        write_word(&mut u, FS + 0x800 + 0x100 + 0x20, 0x8000_0000); // DIEPCTL1 EPENA
        assert_ne!(
            read_word(&mut u, FS + 0x800 + 0x108 + 0x20) & 1,
            0,
            "EP1 XFRC"
        );
        let out = u.drain_output();
        assert_eq!(out[0], EVT_IN);
        assert_eq!(out[1], 4);
        assert_eq!(out[2], 1, "EP1 IN framed");
        // OUT bulk on EP1
        write_word(&mut u, FS + 0x800 + 0x310 + 0x20, (1 << 19) | 4); // DOEPTSIZ1
        write_word(&mut u, FS + 0x800 + 0x300 + 0x20, 0x8000_0000); // DOEPCTL1 EPENA
        inject(&mut u, &[CMD_OUT, 4, 1, 0x11, 0x22, 0x33, 0x44]);
        assert_ne!(
            read_word(&mut u, FS + 0x800 + 0x308 + 0x20) & 1,
            0,
            "EP1 OUT XFRC"
        );
    }

    /// Minimal host-mode init: global interrupt on, SOF/RX/port/channel
    /// unmasked, per-channel interrupts unmasked.
    fn init_host(u: &mut Usb) {
        write_word(u, FS + 0x08, 1); // GAHBCFG GINT
        write_word(u, FS + 0x18, (1 << 3) | (1 << 4) | (1 << 24) | (1 << 25));
        write_word(u, FS + 0x400 + 0x18, 0xFF); // HAINTMSK ch0-7
        for ch in 0..8u32 {
            write_word(u, FS + 0x500 + 0x0c + ch * 0x20, 0x7FF); // HCINTMSKn
        }
    }

    /// Read-modify-write OR on HPRT, as a real stack preserves PENA.
    fn set_hprt(u: &mut Usb, bits: u32) {
        let cur = read_word(u, FS + 0x440);
        write_word(u, FS + 0x440, cur | bits);
    }

    /// Power the port, reset it, and run the SOFs that complete the reset.
    fn enable_port(u: &mut Usb) {
        u.plug_virtual_device(VirtualUsbDevice::new());
        set_hprt(u, 1 << 12); // PPWR
        set_hprt(u, 1 << 8); // PRST
        inject(u, &[CMD_SOF, 0]);
        inject(u, &[CMD_SOF, 0]);
    }

    #[test]
    fn host_hprt_ppwr_raises_interrupt_in_host_mode() {
        let mut u = usb_fs();
        init_host(&mut u);
        assert_eq!(u.pending(), 0);
        // Plugging into the powered port latches connect-detected.
        u.plug_virtual_device(VirtualUsbDevice::new());
        assert_eq!(u.pending(), 0, "no power yet: no connect event");
        set_hprt(&mut u, 1 << 12); // PPWR
        assert_eq!(read_word(&mut u, FS + 0x440) & (1 << 12), 1 << 12);
        assert_eq!(
            u.pending(),
            1,
            "connect-detected raises HPRTINT in host mode"
        );
        assert_ne!(read_word(&mut u, FS + 0x14) & (1 << 24), 0, "HPRTINT");
        // The ISR's write-one-to-clear drops the line.
        write_word(&mut u, FS + 0x440, (1 << 12) | (1 << 1));
        assert_eq!(u.pending(), 0, "line dropped once PCDET clears");
        // switch to device mode — HPRT changes stay silent there.
        write_word(&mut u, FS + 0x0c, 1 << 30); // GUSBCFG FDMOD
        set_hprt(&mut u, 1 << 8);
        assert_eq!(u.pending(), 0, "HPRT in device mode does not pend");
    }

    #[test]
    fn host_hcchar_chen_raises_interrupt_in_host_mode() {
        let mut u = usb_fs();
        init_host(&mut u);
        assert_eq!(u.pending(), 0);
        // No device and no enable: the transaction times out.
        write_word(&mut u, FS + 0x500, 1u32 << 31); // HCCHAR0 CHENA
        assert_eq!(
            u.pending(),
            1,
            "HCCHAR CHENA raises host interrupt in host mode"
        );
        assert_eq!(
            read_word(&mut u, FS + 0x508) & ((1 << 7) | (1 << 1)),
            (1 << 7) | (1 << 1),
            "HCINT TXERR+CHH latched"
        );
        assert_eq!(
            (read_word(&mut u, FS + 0x500) >> 31) & 1,
            0,
            "CHENA dropped"
        );
        // The ISR's write-one-to-clear drops the line.
        write_word(&mut u, FS + 0x508, 0xFFFF_FFFF);
        write_word(&mut u, FS + 0x440, (1 << 12) | (1 << 1) | (1 << 3));
        assert_eq!(u.pending(), 0);
        // device mode — HCCHAR should not pend
        write_word(&mut u, FS + 0x0c, 1 << 30); // GUSBCFG FDMOD
        write_word(&mut u, FS + 0x500, 1u32 << 31);
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn host_port_reset_enables_at_two_sofs() {
        let mut u = usb_fs();
        init_host(&mut u);
        u.plug_virtual_device(VirtualUsbDevice::new());
        set_hprt(&mut u, 1 << 12); // PPWR
        write_word(&mut u, FS + 0x440, (1 << 12) | (1 << 1)); // clear PCDET
        set_hprt(&mut u, 1 << 8); // PRST
        assert_ne!(read_word(&mut u, FS + 0x440) & (1 << 8), 0, "PRST");
        inject(&mut u, &[CMD_SOF, 0]);
        assert_eq!(
            read_word(&mut u, FS + 0x440) & ((1 << 8) | (1 << 2)),
            1 << 8,
            "still resetting after one SOF"
        );
        inject(&mut u, &[CMD_SOF, 0]);
        let hprt = read_word(&mut u, FS + 0x440);
        assert_eq!(hprt & (1 << 8), 0, "PRST cleared by the core");
        assert_ne!(hprt & (1 << 2), 0, "PENA set");
        assert_ne!(hprt & (1 << 3), 0, "PENCHNG");
        assert_eq!((hprt >> 17) & 0b11, 0b01, "PRTSPD full-speed");
        assert_ne!(read_word(&mut u, FS + 0x14) & (1 << 24), 0, "HPRTINT");
        // Frame counting runs: two SOFs emitted.
        assert_eq!(read_word(&mut u, FS + 0x408) & 0xFFFF, 2, "HFNUM FRNUM");
    }

    #[test]
    fn host_chdis_halts_armed_interrupt_channel() {
        let mut u = usb_fs();
        init_host(&mut u);
        enable_port(&mut u);
        // Arm an interrupt IN: it defers to the SOF scheduler, so CHENA
        // stays set until the frame boundary.
        write_word(&mut u, FS + 0x510, (1 << 19) | 8); // HCTSIZ0 PKTCNT1 XFRSIZ8
        write_word(
            &mut u,
            FS + 0x500,
            (1u32 << 31) | (3 << 18) | (1 << 15) | (1 << 11) | 8,
        ); // CHENA INTR IN EP1 MPSIZ8
        assert_eq!(
            (read_word(&mut u, FS + 0x500) >> 31) & 1,
            1,
            "CHENA held for the frame"
        );
        write_word(&mut u, FS + 0x500, (1u32 << 30) | (3 << 18)); // CHDIS
        assert_eq!(
            (read_word(&mut u, FS + 0x500) >> 31) & 1,
            0,
            "CHENA dropped by halt"
        );
        assert_ne!(read_word(&mut u, FS + 0x508) & (1 << 1), 0, "CHH");
    }

    #[test]
    fn host_unplug_drops_enable_with_change_bit() {
        let mut u = usb_fs();
        init_host(&mut u);
        enable_port(&mut u);
        assert_ne!(read_word(&mut u, FS + 0x440) & (1 << 2), 0, "enabled");
        assert!(u.unplug_virtual_device().is_some());
        let hprt = read_word(&mut u, FS + 0x440);
        assert_eq!(hprt & (1 << 2), 0, "PENA dropped");
        assert_eq!(hprt & (1 << 0), 0, "PCSTS dropped");
        assert_ne!(hprt & (1 << 3), 0, "PENCHNG");
        assert!(u.virtual_device().is_none());
    }

    #[test]
    fn host_soft_reset_keeps_the_cable_plugged() {
        let mut u = usb_fs();
        init_host(&mut u);
        enable_port(&mut u);
        write_word(&mut u, FS + 0x10, 1); // CSRST
        assert!(u.virtual_device().is_some(), "device stays attached");
        assert_eq!(
            read_word(&mut u, FS + 0x440) & (1 << 2),
            0,
            "HPRT reloaded: port dark"
        );
    }

    #[test]
    fn device_double_buffer_sd1pid_toggles_pid() {
        let mut u = usb_fs();
        // DIEPCTL0 at 0x900, SD1PID bit 29
        write_word(&mut u, FS + 0x900, 1 << 29);
        assert!(u.state.in_data_pid[0], "SD1PID → DATA1");
        assert!(u.state.in_odd_frame[0], "SODDFRM → odd");
        write_word(&mut u, FS + 0x900, 1 << 28);
        assert!(!u.state.in_data_pid[0], "SD0PID → DATA0");
        // DOEPCTL0 at 0xB00 likewise
        write_word(&mut u, FS + 0xB00, 1 << 29);
        assert!(u.state.out_data_pid[0]);
        write_word(&mut u, FS + 0xB00, 1 << 28);
        assert!(!u.state.out_data_pid[0]);
    }
}

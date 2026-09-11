//! USB host-mode enumeration e2e over the modeled OTG FS controller.
//!
//! Drives a host stack's register sequence through the public `Peripheral`/
//! `InterruptSource` surface exactly as the ST USB-host library does: power
//! the port, reset it, then run control transfers (GET_DESCRIPTOR,
//! SET_ADDRESS, SET_CONFIGURATION, line coding) and bulk/interrupt traffic
//! against the virtual CDC-ACM device. No internal fields touched.

use vioarm_core::device::stm32::usb::{Usb, VirtualUsbDevice};
use vioarm_core::device::{InterruptSource, Peripheral};

const FS: u32 = 0x5000_0000;
// GLOBAL
const GAHBCFG: u32 = FS + 0x08;
const GINTSTS: u32 = FS + 0x14;
const GINTMSK: u32 = FS + 0x18;
const GRXSTSR: u32 = FS + 0x1c;
const FIFO: u32 = FS + 0x1000;
// HOST window
const H: u32 = FS + 0x400;
const HPRT: u32 = H + 0x40;
const HAINTMSK: u32 = H + 0x18;
const HCCHAR0: u32 = H + 0x100;
const HCINT0: u32 = H + 0x108;
const HCINTMSK0: u32 = H + 0x10c;
const HCTSIZ0: u32 = H + 0x110;
const CH_STRIDE: u32 = 0x20;

// HCINT bits the stack polls.
const XFRC: u32 = 1 << 0;
const CHH: u32 = 1 << 1;
const STALL: u32 = 1 << 3;
const NAK: u32 = 1 << 4;
const ACK: u32 = 1 << 5;

fn wr(u: &mut Usb, addr: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        u.write(addr + i as u32, *b).unwrap();
    }
}

fn rd(u: &mut Usb, addr: u32) -> u32 {
    let mut bytes = [0u8; 4];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = u.read(addr + i as u32).unwrap();
    }
    u32::from_le_bytes(bytes)
}

fn sof(u: &mut Usb) {
    for b in [0x01u8, 0x00] {
        u.drive_inputs(u32::from(b));
    }
}

/// HPRT read-modify-write OR, as a stack preserves the other port bits.
fn set_hprt(u: &mut Usb, bits: u32) {
    let cur = rd(u, HPRT);
    wr(u, HPRT, cur | bits);
}

/// HPRT change-bit clear: writes 1s to the serviced change bits while
/// preserving the port control bits, as an ISR write-back does.
fn clear_hprt(u: &mut Usb, bits: u32) {
    const KEEP: u32 = (1 << 12) | (1 << 8) | (1 << 2); // PPWR PRST PENA
    const CHNG: u32 = (1 << 1) | (1 << 3) | (1 << 4); // PCDET PENCHNG POCCHNG
    let cur = rd(u, HPRT);
    wr(u, HPRT, (cur & KEEP) | (bits & CHNG));
}

fn hcchar(ch: u32) -> u32 {
    HCCHAR0 + ch * CH_STRIDE
}
fn hcint(ch: u32) -> u32 {
    HCINT0 + ch * CH_STRIDE
}
fn hctsiz(ch: u32) -> u32 {
    HCTSIZ0 + ch * CH_STRIDE
}

/// Host-mode init: global interrupt on, SOF/RX/port/channel unmasked,
/// every channel interrupt unmasked.
fn init_host(u: &mut Usb) {
    wr(u, GAHBCFG, 1);
    wr(u, GINTMSK, (1 << 3) | (1 << 4) | (1 << 24) | (1 << 25));
    wr(u, HAINTMSK, 0xFF);
    for ch in 0..8 {
        wr(u, HCINTMSK0 + ch * CH_STRIDE, 0x7FF);
    }
}

/// Power + reset the port; the stack's SOF delay runs the reset out.
fn port_up(u: &mut Usb) {
    set_hprt(u, 1 << 12); // PPWR
    assert_ne!(rd(u, GINTSTS) & (1 << 24), 0, "connect HPRTINT");
    clear_hprt(u, 1 << 1); // clear PCDET, keep PPWR
    set_hprt(u, 1 << 8); // PRST
    for _ in 0..12 {
        sof(u); // the stack's 10ms+ reset delay
    }
    let hprt = rd(u, HPRT);
    assert_eq!(hprt & (1 << 8), 0, "PRST done");
    assert_ne!(hprt & (1 << 2), 0, "PENA");
    assert_eq!((hprt >> 17) & 0b11, 0b01, "full-speed");
    clear_hprt(u, (1 << 1) | (1 << 3)); // clear PCDET+PENCHNG
}

/// A channel descriptor the way the ST host library programs one.
struct Ch {
    ch: u32,
    ep: u32,
    dir_in: bool,
    eptyp: u32, // 0 ctrl, 1 iso, 2 bulk, 3 intr
    mpsiz: u32,
}

fn setup_ch(u: &mut Usb, c: &Ch, xfersiz: u32) {
    wr(u, hctsiz(c.ch), (1 << 19) | xfersiz); // PKTCNT1
    wr(
        u,
        hcchar(c.ch),
        (c.mpsiz & 0x7FF)
            | ((c.ep & 0xF) << 11)
            | (u32::from(c.dir_in) << 15)
            | ((c.eptyp & 0x3) << 18),
    );
}

/// Set CHENA on a channel.
fn chena(u: &mut Usb, ch: u32) {
    let cur = rd(u, hcchar(ch));
    wr(u, hcchar(ch), cur | (1 << 31));
}

/// Set CHENA and wait for the completion the stack polls on: XFRC for
/// success, or the halted outcome. Returns the HCINT word.
fn run_ch(u: &mut Usb, c: &Ch) -> u32 {
    chena(u, c.ch);
    let periodic = c.eptyp == 3 || c.eptyp == 1;
    assert_eq!(
        (rd(u, hcchar(c.ch)) >> 31) & 1,
        u32::from(periodic),
        "CHENA drops on completion (held for periodic)"
    );
    if periodic {
        sof(u); // the periodic schedule runs it at the frame boundary
        assert_eq!((rd(u, hcchar(c.ch)) >> 31) & 1, 0, "CHENA drops");
    }
    let int = rd(u, hcint(c.ch));
    assert_ne!(int & ACK, 0, "ACK");
    wr(u, hcint(c.ch), 0xFFFF_FFFF); // W1C
    int
}

/// Stage OUT bytes through the FIFO window, as the stack queues packets.
fn stage_out(u: &mut Usb, data: &[u8]) {
    for (i, b) in data.iter().enumerate() {
        u.write(FIFO + i as u32, *b).unwrap();
    }
}

/// Pop one host RX entry: the status word plus its payload bytes.
fn take_in(u: &mut Usb) -> (u32, Vec<u8>) {
    let st = rd(u, GRXSTSR);
    let bcnt = ((st >> 4) & 0x7FF) as usize;
    let mut data = Vec::with_capacity(bcnt);
    for _ in 0..bcnt {
        data.push(u.read(FIFO).unwrap());
    }
    (st, data)
}

const EP0_OUT: Ch = Ch {
    ch: 0,
    ep: 0,
    dir_in: false,
    eptyp: 0,
    mpsiz: 64,
};
const EP0_IN: Ch = Ch {
    ch: 0,
    ep: 0,
    dir_in: true,
    eptyp: 0,
    mpsiz: 64,
};

/// One control transfer: SETUP, optional data stage, status stage. All
/// callers use host-to-device requests here, so a missing data stage means
/// an IN status stage (this is what latches `SET_ADDRESS`).
fn control(u: &mut Usb, setup: &[u8; 8], data_stage_out: Option<&[u8]>) {
    stage_out(u, setup);
    setup_ch(u, &EP0_OUT, 8);
    assert_ne!(run_ch(u, &EP0_OUT) & XFRC, 0, "SETUP XFRC");
    if let Some(payload) = data_stage_out {
        stage_out(u, payload);
        setup_ch(u, &EP0_OUT, payload.len() as u32);
        assert_ne!(run_ch(u, &EP0_OUT) & XFRC, 0, "OUT data XFRC");
    }
    setup_ch(u, &EP0_IN, 0);
    assert_ne!(run_ch(u, &EP0_IN) & XFRC, 0, "IN status XFRC");
}

/// One control-IN transfer returning the data bytes, looping IN tokens the
/// way a stack does until a short packet or `wlen` bytes arrived.
fn control_in(u: &mut Usb, setup: &[u8; 8], wlen: u32) -> Vec<u8> {
    const MPSIZ: usize = 64;
    stage_out(u, setup);
    setup_ch(u, &EP0_OUT, 8);
    assert_ne!(run_ch(u, &EP0_OUT) & XFRC, 0, "SETUP XFRC");
    let mut data = Vec::new();
    loop {
        let want = (wlen as usize).saturating_sub(data.len());
        if want == 0 {
            break;
        }
        setup_ch(u, &EP0_IN, want as u32);
        assert_ne!(run_ch(u, &EP0_IN) & XFRC, 0, "IN data XFRC");
        let (st, chunk) = take_in(u);
        assert_eq!((st >> 17) & 0xF, 2, "host IN status");
        assert_eq!(st & 0xF, 0, "channel 0");
        let short = chunk.len() < MPSIZ;
        data.extend(chunk);
        if short || data.len() >= wlen as usize {
            break;
        }
    }
    setup_ch(u, &EP0_OUT, 0);
    assert_ne!(run_ch(u, &EP0_OUT) & XFRC, 0, "OUT status XFRC");
    data
}

fn usb_host() -> Usb {
    let mut u = Usb::otg_fs_for::<vioarm_core::device::svd::stm32f107::Stm32f107>();
    u.reset();
    u.plug_virtual_device(VirtualUsbDevice::new());
    init_host(&mut u);
    port_up(&mut u);
    u
}

#[test]
fn host_enumerates_virtual_cdc_acm() {
    let mut u = usb_host();

    // ---- GET_DESCRIPTOR(DEVICE), 8 + 18 --------------------------------
    let setup = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x08, 0x00];
    let first8 = control_in(&mut u, &setup, 8);
    assert_eq!(&first8, &[0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40]);
    let setup = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00];
    let desc = control_in(&mut u, &setup, 0x12);
    assert_eq!(desc.len(), 18);
    assert_eq!((desc[8], desc[9]), (0x83, 0x04), "VID");
    assert_eq!(desc[11], 0x57, "PID hi");
    assert_eq!(desc[7], 64, "EP0 max packet");

    // ---- SET_ADDRESS(5): applies at the status stage --------------------
    control(
        &mut u,
        &[0x00, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    assert_eq!(
        u.virtual_device().map(VirtualUsbDevice::address),
        Some(5),
        "address latched after status"
    );

    // ---- GET_DESCRIPTOR(CONFIG), short then full ------------------------
    let setup = [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00];
    let head = control_in(&mut u, &setup, 9);
    let total = u16::from_le_bytes([head[2], head[3]]);
    assert_eq!(total, 67, "wTotalLength");
    let setup = [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x43, 0x00];
    let cfg = control_in(&mut u, &setup, 67);
    assert_eq!(cfg.len(), 67);
    assert_eq!(cfg[1], 0x02, "config type");
    assert_eq!(cfg[4], 0x02, "two interfaces");
    assert_eq!(&cfg[60..67], &[0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00]);

    // ---- String descriptors ---------------------------------------------
    let setup = [0x80, 0x06, 0x02, 0x03, 0x09, 0x04, 0xFF, 0x00];
    let prod = control_in(&mut u, &setup, 0xFF);
    assert_eq!(prod[1], 0x03, "string type");
    assert!(prod.windows(2).any(|w| w == [b'C', 0]), "product name");

    // ---- SET_CONFIGURATION(1) -------------------------------------------
    control(
        &mut u,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    assert_eq!(
        u.virtual_device().map(VirtualUsbDevice::configuration),
        Some(1)
    );

    // ---- CDC class: line coding round-trip ------------------------------
    let setup = [0xA1, 0x21, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00];
    let coding = control_in(&mut u, &setup, 7);
    assert_eq!(&coding, &[0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]);
    let new = [0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00]; // 9600 8-N-1
    control(
        &mut u,
        &[0x21, 0x20, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00],
        Some(&new),
    );
    assert_eq!(
        u.virtual_device().map(VirtualUsbDevice::line_coding),
        Some(new)
    );
    control(
        &mut u,
        &[0x21, 0x22, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );

    // The stack saw channel + port interrupts throughout, all clearable.
    let g = rd(&mut u, GINTSTS);
    wr(&mut u, GINTSTS, g);
    clear_hprt(&mut u, (1 << 1) | (1 << 3));
    assert_eq!(u.pending(), 0, "all events serviced");
}

#[test]
fn host_bulk_echo_and_interrupt_notification() {
    let mut u = usb_host();
    control(
        &mut u,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );

    // ---- Bulk OUT: CDC data reaches the device --------------------------
    let bulk_out = Ch {
        ch: 1,
        ep: 1,
        dir_in: false,
        eptyp: 2,
        mpsiz: 64,
    };
    let payload = [0x48, 0x65, 0x6C, 0x6C, 0x6F]; // "Hello"
    stage_out(&mut u, &payload);
    setup_ch(&mut u, &bulk_out, payload.len() as u32);
    assert_ne!(run_ch(&mut u, &bulk_out) & XFRC, 0, "bulk OUT XFRC");
    assert_eq!(
        rd(&mut u, hctsiz(1)) & 0x7_FFFF,
        0,
        "HCTSIZ remainder drained"
    );
    assert_eq!(
        u.virtual_device_mut().map(|d| d.take_out(1)),
        Some(payload.to_vec())
    );

    // ---- Bulk IN: staged device bytes arrive in the RX FIFO -------------
    if let Some(d) = u.virtual_device_mut() {
        d.push_in(1, &[0x41, 0x43, 0x4B]); // "ACK"
    }
    let bulk_in = Ch {
        ch: 1,
        ep: 1,
        dir_in: true,
        eptyp: 2,
        mpsiz: 64,
    };
    setup_ch(&mut u, &bulk_in, 64);
    assert_ne!(run_ch(&mut u, &bulk_in) & XFRC, 0, "bulk IN XFRC");
    let (st, data) = take_in(&mut u);
    assert_eq!((st >> 17) & 0xF, 2, "host IN status");
    assert_eq!(st & 0xF, 1, "channel 1");
    assert_eq!(((st >> 4) & 0x7FF), 3, "BCNT");
    assert_eq!(&data, &[0x41, 0x43, 0x4B]);

    // ---- Interrupt IN with nothing staged NAKs; SOF delivers later ------
    let intr_in = Ch {
        ch: 2,
        ep: 2,
        dir_in: true,
        eptyp: 3,
        mpsiz: 8,
    };
    setup_ch(&mut u, &intr_in, 8);
    chena(&mut u, 2);
    sof(&mut u); // the scheduler runs the armed periodic channel: NAK
    let int = rd(&mut u, hcint(2));
    assert_ne!(int & NAK, 0, "NAK latched");
    assert_ne!(int & CHH, 0, "channel halted");
    wr(&mut u, hcint(2), 0xFFFF_FFFF);
    // The notification arrives; the stack re-arms and the next SOF moves it.
    if let Some(d) = u.virtual_device_mut() {
        d.push_in(2, &[0xA1, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00]);
    }
    setup_ch(&mut u, &intr_in, 8);
    chena(&mut u, 2);
    sof(&mut u);
    assert_ne!(rd(&mut u, hcint(2)) & XFRC, 0, "periodic XFRC");
    let (st, data) = take_in(&mut u);
    assert_eq!(((st >> 4) & 0x7FF), 8, "notification BCNT");
    assert_eq!(data[0], 0xA1, "CDC notification header");
    wr(&mut u, hcint(2), 0xFFFF_FFFF);
}

#[test]
fn host_stall_and_halt_recovery() {
    let mut u = usb_host();

    // An unsupported descriptor index STALLs the control endpoint.
    stage_out(&mut u, &[0x80, 0x06, 0x00, 0x09, 0x00, 0x00, 0x40, 0x00]);
    setup_ch(&mut u, &EP0_OUT, 8);
    chena(&mut u, 0);
    let int = rd(&mut u, hcint(0));
    assert_ne!(int & STALL, 0, "STALL latched");
    assert_ne!(int & CHH, 0, "channel halted");
    assert_eq!((rd(&mut u, hcchar(0)) >> 31) & 1, 0, "CHENA dropped");
    wr(&mut u, hcint(0), 0xFFFF_FFFF);

    // CLEAR_FEATURE un-halts a halted bulk endpoint; traffic resumes.
    control(
        &mut u,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    control(
        &mut u,
        &[0x02, 0x03, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00],
        None,
    ); // SET_FEATURE EP1-IN HALT
    if let Some(d) = u.virtual_device_mut() {
        d.push_in(1, &[0x55]);
    }
    let bulk_in = Ch {
        ch: 1,
        ep: 1,
        dir_in: true,
        eptyp: 2,
        mpsiz: 64,
    };
    setup_ch(&mut u, &bulk_in, 64);
    chena(&mut u, 1);
    assert_ne!(rd(&mut u, hcint(1)) & STALL, 0, "halted EP STALLs");
    wr(&mut u, hcint(1), 0xFFFF_FFFF);
    control(
        &mut u,
        &[0x02, 0x01, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00],
        None,
    ); // CLEAR_FEATURE
    setup_ch(&mut u, &bulk_in, 64);
    assert_ne!(run_ch(&mut u, &bulk_in) & XFRC, 0, "resumed after clear");
    let (_, data) = take_in(&mut u);
    assert_eq!(&data, &[0x55]);
}

/// Isochronous host traffic, audio-class shaped: one full 48-byte playback
/// frame per SOF, a short 20-byte frame (legal on ISO, an underrun on
/// bulk), and a capture frame back — periodic `XFRC`, never `STALL`.
#[test]
fn host_isochronous_audio_frames() {
    let mut u = usb_host();
    control(
        &mut u,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    ); // SET_CONFIGURATION

    // ---- ISO OUT: full 48-sample frame ----------------------------------
    let iso_out = Ch {
        ch: 3,
        ep: 1,
        dir_in: false,
        eptyp: 1,
        mpsiz: 64,
    };
    let frame: Vec<u8> = (0..48u8).collect();
    stage_out(&mut u, &frame);
    setup_ch(&mut u, &iso_out, frame.len() as u32);
    let int = run_ch(&mut u, &iso_out);
    assert_ne!(int & XFRC, 0, "iso OUT XFRC");
    assert_eq!(int & (STALL | CHH), 0, "no STALL, no halt");
    assert_eq!(
        rd(&mut u, hctsiz(3)) & 0x7_FFFF,
        0,
        "HCTSIZ remainder drained"
    );
    assert_eq!(u.virtual_device_mut().map(|d| d.take_out(1)), Some(frame));

    // ---- ISO OUT: short 20-byte frame (legal, not an underrun) ----------
    let short: Vec<u8> = (0..20u8).collect();
    stage_out(&mut u, &short);
    setup_ch(&mut u, &iso_out, 48);
    let int = run_ch(&mut u, &iso_out);
    assert_ne!(int & XFRC, 0, "short ISO OUT XFRC");
    assert_eq!(int & (STALL | CHH | NAK), 0, "clean completion");
    assert_eq!(u.virtual_device_mut().map(|d| d.take_out(1)), Some(short));

    // ---- ISO IN: staged capture frame arrives in the RX FIFO ------------
    if let Some(d) = u.virtual_device_mut() {
        d.push_in(1, &[0x11, 0x22, 0x33, 0x44]);
    }
    let iso_in = Ch {
        ch: 3,
        ep: 1,
        dir_in: true,
        eptyp: 1,
        mpsiz: 64,
    };
    setup_ch(&mut u, &iso_in, 64);
    let int = run_ch(&mut u, &iso_in);
    assert_ne!(int & XFRC, 0, "iso IN XFRC");
    assert_eq!(int & (STALL | CHH | NAK), 0, "clean completion");
    let (st, data) = take_in(&mut u);
    assert_eq!((st >> 17) & 0xF, 2, "host IN status");
    assert_eq!(st & 0xF, 1, "endpoint 1");
    assert_eq!(&data, &[0x11, 0x22, 0x33, 0x44]);
}

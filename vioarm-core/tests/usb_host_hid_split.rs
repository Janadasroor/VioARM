//! USB host Phase-B proof: HID keyboard traffic plus split-transaction
//! sequencing (`OTG_HS`, whose SVD carries `HCSPLT`).
//!
//! Drives a host stack's register sequence through the public
//! `Peripheral`/`InterruptSource` surface exactly as the ST USB-host
//! library does: enumerate the virtual HID boot keyboard (device,
//! configuration, HID + report descriptors, idle/protocol/report class
//! requests), read an interrupt-IN key report, then run bulk OUT/IN
//! transfers as start-split/complete-split pairs and show the device only
//! sees the complete phase. No internal fields touched.

use vioarm_core::device::stm32::usb::{Usb, VirtualUsbDevice};
use vioarm_core::device::Peripheral;

// `OTG_FS` and `OTG_HS` share the window layout; only the base moves.
const FS: u32 = 0x5000_0000;
const HS: u32 = 0x4004_0000;

// HCSPLT bits the stack programs.
const SPLITENA: u32 = 1 << 31;
const COMPLSPLT: u32 = 1 << 16;

// HCINT bits the stack polls.
const XFRC: u32 = 1 << 0;
const ACK: u32 = 1 << 5;

fn gahbcfg(b: u32) -> u32 {
    b + 0x08
}
fn gintsts(b: u32) -> u32 {
    b + 0x14
}
fn gintmsk(b: u32) -> u32 {
    b + 0x18
}
fn grxstsr(b: u32) -> u32 {
    b + 0x1c
}
fn fifo(b: u32) -> u32 {
    b + 0x1000
}
fn hprt(b: u32) -> u32 {
    b + 0x400 + 0x40
}
fn haintmsk(b: u32) -> u32 {
    b + 0x400 + 0x18
}
fn hcchar(b: u32, ch: u32) -> u32 {
    b + 0x400 + 0x100 + ch * 0x20
}
fn hcsplt(b: u32, ch: u32) -> u32 {
    b + 0x400 + 0x104 + ch * 0x20
}
fn hcint(b: u32, ch: u32) -> u32 {
    b + 0x400 + 0x108 + ch * 0x20
}
fn hcintmsk(b: u32, ch: u32) -> u32 {
    b + 0x400 + 0x10c + ch * 0x20
}
fn hctsiz(b: u32, ch: u32) -> u32 {
    b + 0x400 + 0x110 + ch * 0x20
}

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
fn set_hprt(u: &mut Usb, b: u32, bits: u32) {
    let cur = rd(u, hprt(b));
    wr(u, hprt(b), cur | bits);
}

/// HPRT change-bit clear: writes 1s to the serviced change bits while
/// preserving the port control bits, as an ISR write-back does.
fn clear_hprt(u: &mut Usb, b: u32, bits: u32) {
    const KEEP: u32 = (1 << 12) | (1 << 8) | (1 << 2); // PPWR PRST PENA
    const CHNG: u32 = (1 << 1) | (1 << 3) | (1 << 4); // PCDET PENCHNG POCCHNG
    let cur = rd(u, hprt(b));
    wr(u, hprt(b), (cur & KEEP) | (bits & CHNG));
}

/// Host-mode init: global interrupt on, SOF/RX/port/channel unmasked,
/// every channel interrupt unmasked.
fn init_host(u: &mut Usb, b: u32, nchan: u32) {
    wr(u, gahbcfg(b), 1);
    wr(u, gintmsk(b), (1 << 3) | (1 << 4) | (1 << 24) | (1 << 25));
    wr(u, haintmsk(b), 0xFFFF_FFFF);
    for ch in 0..nchan {
        wr(u, hcintmsk(b, ch), 0x7FF);
    }
}

/// Power + reset the port; the stack's SOF delay runs the reset out.
fn port_up(u: &mut Usb, b: u32) {
    set_hprt(u, b, 1 << 12); // PPWR
    assert_ne!(rd(u, gintsts(b)) & (1 << 24), 0, "connect HPRTINT");
    clear_hprt(u, b, 1 << 1); // clear PCDET, keep PPWR
    set_hprt(u, b, 1 << 8); // PRST
    for _ in 0..12 {
        sof(u); // the stack's 10ms+ reset delay
    }
    let hprt = rd(u, hprt(b));
    assert_eq!(hprt & (1 << 8), 0, "PRST done");
    assert_ne!(hprt & (1 << 2), 0, "PENA");
    assert_eq!((hprt >> 17) & 0b11, 0b01, "full-speed");
    clear_hprt(u, b, (1 << 1) | (1 << 3)); // clear PCDET+PENCHNG
}

/// A channel descriptor the way the ST host library programs one.
struct Ch {
    ch: u32,
    ep: u32,
    dir_in: bool,
    eptyp: u32, // 0 ctrl, 1 iso, 2 bulk, 3 intr
    mpsiz: u32,
}

fn setup_ch(u: &mut Usb, b: u32, c: &Ch, xfersiz: u32) {
    wr(u, hctsiz(b, c.ch), (1 << 19) | xfersiz); // PKTCNT1
    wr(
        u,
        hcchar(b, c.ch),
        (c.mpsiz & 0x7FF)
            | ((c.ep & 0xF) << 11)
            | (u32::from(c.dir_in) << 15)
            | ((c.eptyp & 0x3) << 18),
    );
}

/// Set CHENA on a channel.
fn chena(u: &mut Usb, b: u32, ch: u32) {
    let cur = rd(u, hcchar(b, ch));
    wr(u, hcchar(b, ch), cur | (1 << 31));
}

/// Set CHENA and wait for the completion the stack polls on. Returns the
/// HCINT word; the caller decides which bits must be set.
fn run_ch_raw(u: &mut Usb, b: u32, c: &Ch) -> u32 {
    chena(u, b, c.ch);
    let periodic = c.eptyp == 3 || c.eptyp == 1;
    assert_eq!(
        (rd(u, hcchar(b, c.ch)) >> 31) & 1,
        u32::from(periodic),
        "CHENA drops on completion (held for periodic)"
    );
    if periodic {
        sof(u); // the periodic schedule runs it at the frame boundary
        assert_eq!((rd(u, hcchar(b, c.ch)) >> 31) & 1, 0, "CHENA drops");
    }
    let int = rd(u, hcint(b, c.ch));
    wr(u, hcint(b, c.ch), 0xFFFF_FFFF); // W1C
    int
}

/// Set CHENA and expect the success handshake: ACK plus transfer-complete.
fn run_ch(u: &mut Usb, b: u32, c: &Ch) -> u32 {
    let int = run_ch_raw(u, b, c);
    assert_ne!(int & ACK, 0, "ACK");
    assert_ne!(int & XFRC, 0, "XFRC");
    int
}

/// Stage OUT bytes through the FIFO window, as the stack queues packets.
fn stage_out(u: &mut Usb, b: u32, data: &[u8]) {
    for (i, byte) in data.iter().enumerate() {
        u.write(fifo(b) + i as u32, *byte).unwrap();
    }
}

/// Pop one host RX entry: the status word plus its payload bytes.
fn take_in(u: &mut Usb, b: u32) -> (u32, Vec<u8>) {
    let st = rd(u, grxstsr(b));
    let bcnt = ((st >> 4) & 0x7FF) as usize;
    let mut data = Vec::with_capacity(bcnt);
    for _ in 0..bcnt {
        data.push(u.read(fifo(b)).unwrap());
    }
    (st, data)
}

fn ep0_out() -> Ch {
    Ch {
        ch: 0,
        ep: 0,
        dir_in: false,
        eptyp: 0,
        mpsiz: 64,
    }
}
fn ep0_in() -> Ch {
    Ch {
        ch: 0,
        ep: 0,
        dir_in: true,
        eptyp: 0,
        mpsiz: 64,
    }
}

/// One control transfer: SETUP, optional data stage, status stage. All
/// callers use host-to-device requests here, so a missing data stage means
/// an IN status stage (this is what latches `SET_ADDRESS`).
fn control(u: &mut Usb, b: u32, setup: &[u8; 8], data_stage_out: Option<&[u8]>) {
    stage_out(u, b, setup);
    setup_ch(u, b, &ep0_out(), 8);
    assert_ne!(run_ch(u, b, &ep0_out()) & XFRC, 0, "SETUP XFRC");
    if let Some(payload) = data_stage_out {
        stage_out(u, b, payload);
        setup_ch(u, b, &ep0_out(), payload.len() as u32);
        assert_ne!(run_ch(u, b, &ep0_out()) & XFRC, 0, "OUT data XFRC");
    }
    setup_ch(u, b, &ep0_in(), 0);
    assert_ne!(run_ch(u, b, &ep0_in()) & XFRC, 0, "IN status XFRC");
}

/// One control-IN transfer returning the data bytes, looping IN tokens the
/// way a stack does until a short packet or `wlen` bytes arrived.
fn control_in(u: &mut Usb, b: u32, setup: &[u8; 8], wlen: u32) -> Vec<u8> {
    const MPSIZ: usize = 64;
    stage_out(u, b, setup);
    setup_ch(u, b, &ep0_out(), 8);
    assert_ne!(run_ch(u, b, &ep0_out()) & XFRC, 0, "SETUP XFRC");
    let mut data = Vec::new();
    loop {
        let want = (wlen as usize).saturating_sub(data.len());
        if want == 0 {
            break;
        }
        setup_ch(u, b, &ep0_in(), want as u32);
        assert_ne!(run_ch(u, b, &ep0_in()) & XFRC, 0, "IN data XFRC");
        let (st, chunk) = take_in(u, b);
        assert_eq!((st >> 17) & 0xF, 2, "host IN status");
        assert_eq!(st & 0xF, 0, "channel 0");
        let short = chunk.len() < MPSIZ;
        data.extend(chunk);
        if short || data.len() >= wlen as usize {
            break;
        }
    }
    setup_ch(u, b, &ep0_out(), 0);
    assert_ne!(run_ch(u, b, &ep0_out()) & XFRC, 0, "OUT status XFRC");
    data
}

fn hid_host() -> Usb {
    let mut u = Usb::otg_fs_for::<vioarm_core::device::svd::stm32f407::Stm32f407>();
    u.reset();
    u.plug_virtual_device(VirtualUsbDevice::hid_keyboard());
    init_host(&mut u, FS, 8);
    port_up(&mut u, FS);
    u
}

fn cdc_host_hs() -> Usb {
    let mut u = Usb::otg_hs_for::<vioarm_core::device::svd::stm32f407::Stm32f407>();
    u.reset();
    u.plug_virtual_device(VirtualUsbDevice::new());
    init_host(&mut u, HS, 12);
    port_up(&mut u, HS);
    // SET_CONFIGURATION(1) so the bulk endpoints are live.
    control(
        &mut u,
        HS,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    u
}

#[test]
fn host_hid_keyboard_enumeration_and_reports() {
    let mut u = hid_host();
    let b = FS;

    // ---- GET_DESCRIPTOR(DEVICE): per-interface class, 64-byte EP0 -----
    let desc = control_in(
        &mut u,
        b,
        &[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00],
        0x12,
    );
    assert_eq!(desc.len(), 18);
    assert_eq!(
        (desc[4], desc[5], desc[6]),
        (0, 0, 0),
        "class per interface"
    );
    assert_eq!(desc[7], 64, "EP0 max packet");
    assert_eq!((desc[8], desc[9]), (0x83, 0x04), "VID");

    // ---- GET_DESCRIPTOR(CONFIG): HID interface + interrupt IN EP -------
    let cfg = control_in(
        &mut u,
        b,
        &[0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x00],
        0xFF,
    );
    assert_eq!(u16::from_le_bytes([cfg[2], cfg[3]]), 34, "wTotalLength");
    assert_eq!((cfg[14], cfg[15], cfg[16]), (3, 1, 1), "HID boot keyboard");
    assert_eq!(cfg[29], 0x81, "interrupt IN endpoint 1");
    assert_eq!(cfg[31], 8, "8-byte reports");

    // ---- GET_DESCRIPTOR(HID) + GET_DESCRIPTOR(REPORT) ------------------
    let hid = control_in(
        &mut u,
        b,
        &[0x80, 0x06, 0x00, 0x21, 0x00, 0x00, 0x09, 0x00],
        9,
    );
    assert_eq!(hid.len(), 9);
    assert_eq!(
        (hid[1], hid[7], hid[8]),
        (0x21, 0x3F, 0x00),
        "report length 63"
    );
    let report = control_in(
        &mut u,
        b,
        &[0x80, 0x06, 0x00, 0x22, 0x00, 0x00, 0x3F, 0x00],
        0x3F,
    );
    assert_eq!(report.len(), 63);
    assert_eq!(
        &report[..4],
        &[0x05, 0x01, 0x09, 0x06],
        "generic desktop/keyboard"
    );

    // ---- SET_ADDRESS(7) + SET_CONFIGURATION(1) -------------------------
    control(
        &mut u,
        b,
        &[0x00, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    control(
        &mut u,
        b,
        &[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );

    // ---- SET_IDLE(10) + SET_PROTOCOL(boot) read back -------------------
    control(
        &mut u,
        b,
        &[0x21, 0x0A, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    let idle = control_in(
        &mut u,
        b,
        &[0xA1, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00],
        1,
    );
    assert_eq!(idle, vec![10], "idle rate latched");
    control(
        &mut u,
        b,
        &[0x21, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        None,
    );
    let proto = control_in(
        &mut u,
        b,
        &[0xA1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00],
        1,
    );
    assert_eq!(proto, vec![0], "boot protocol");

    // ---- GET_REPORT with no keys: the idle all-zeros state -------------
    let idle_report = control_in(
        &mut u,
        b,
        &[0xA1, 0x01, 0x00, 0x01, 0x00, 0x00, 0x08, 0x00],
        8,
    );
    assert_eq!(idle_report, vec![0; 8], "no keys pressed");

    // ---- Interrupt IN: the 'a' key report arrives on EP1 ---------------
    u.virtual_device_mut()
        .unwrap()
        .push_in(1, &[0, 0, 0x04, 0, 0, 0, 0, 0]);
    let key = Ch {
        ch: 1,
        ep: 1,
        dir_in: true,
        eptyp: 3,
        mpsiz: 8,
    };
    setup_ch(&mut u, b, &key, 8);
    assert_ne!(run_ch(&mut u, b, &key) & XFRC, 0, "INTR XFRC");
    let (st, data) = take_in(&mut u, b);
    assert_eq!((st >> 17) & 0xF, 2, "host IN status");
    assert_eq!(data, vec![0, 0, 0x04, 0, 0, 0, 0, 0], "key 'a' report");

    // ---- SET_REPORT: the LED byte lands on the device ------------------
    control(
        &mut u,
        b,
        &[0x21, 0x09, 0x00, 0x02, 0x00, 0x00, 0x01, 0x00],
        Some(&[0x02]),
    );
    assert_eq!(u.virtual_device().unwrap().leds(), 0x02, "caps-lock LED");
}

#[test]
fn host_split_start_complete_sequence() {
    let mut u = cdc_host_hs();
    let b = HS;

    // ---- Bulk OUT start-split: ACK only, device sees nothing -----------
    let payload = b"hello split";
    stage_out(&mut u, b, payload);
    let out = Ch {
        ch: 1,
        ep: 1,
        dir_in: false,
        eptyp: 2,
        mpsiz: 64,
    };
    wr(&mut u, hcsplt(b, 1), SPLITENA);
    setup_ch(&mut u, b, &out, payload.len() as u32);
    let int = run_ch_raw(&mut u, b, &out);
    assert_ne!(int & ACK, 0, "start-split accepted");
    assert_eq!(int & XFRC, 0, "no completion yet");
    assert!(
        u.virtual_device_mut().unwrap().take_out(1).is_empty(),
        "hub holds it"
    );

    // ---- Bulk OUT complete-split: the bytes land -----------------------
    wr(&mut u, hcsplt(b, 1), SPLITENA | COMPLSPLT);
    setup_ch(&mut u, b, &out, payload.len() as u32);
    let int = run_ch(&mut u, b, &out);
    assert_ne!(int & XFRC, 0, "complete-split XFRC");
    assert_eq!(
        u.virtual_device_mut().unwrap().take_out(1),
        payload,
        "bytes delivered"
    );

    // ---- Bulk IN start-split: ACK only, no RX entry --------------------
    u.virtual_device_mut().unwrap().push_in(1, b"cs-data");
    let inp = Ch {
        ch: 2,
        ep: 1,
        dir_in: true,
        eptyp: 2,
        mpsiz: 64,
    };
    wr(&mut u, hcsplt(b, 2), SPLITENA);
    setup_ch(&mut u, b, &inp, 64);
    let int = run_ch_raw(&mut u, b, &inp);
    assert_ne!(int & ACK, 0, "start-split accepted");
    assert_eq!(int & XFRC, 0, "no completion yet");

    // ---- Bulk IN complete-split: the data arrives ----------------------
    wr(&mut u, hcsplt(b, 2), SPLITENA | COMPLSPLT);
    setup_ch(&mut u, b, &inp, 64);
    assert_ne!(run_ch(&mut u, b, &inp) & XFRC, 0, "complete-split XFRC");
    let (st, data) = take_in(&mut u, b);
    assert_eq!((st >> 17) & 0xF, 2, "host IN status");
    assert_eq!(data, b"cs-data", "complete-split payload");
}

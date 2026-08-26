//! CDC-ACM-style full enumeration e2e over the modeled OTG FS controller.
//!
//! Drives the real host protocol order through the public `Peripheral`/
//! `InterruptSource` surface exactly as an STM32 USB-device stack sees it:
//! bus reset -> GET_DESCRIPTOR(DEVICE) control-IN -> SET_ADDRESS ->
//! SET_CONFIGURATION -> EP1 bulk IN/OUT echo. No internal fields touched.

use vioarm_core::device::stm32::usb::Usb;
use vioarm_core::device::{InterruptSource, Peripheral};

const FS: u32 = 0x5000_0000;
// GLOBAL
// GLOBAL
const GRSTCTL: u32 = FS + 0x10;
const GINTSTS: u32 = FS + 0x14;
const GINTMSK: u32 = FS + 0x18;
const GAHBCFG: u32 = FS + 0x08;
const GUSBCFG: u32 = FS + 0x0c;
// DEVICE window
const D: u32 = FS + 0x800;
const DSTS: u32 = D + 0x08;
const DIEPMSK: u32 = D + 0x10;
const DOEPMSK: u32 = D + 0x14;
const DAINTMSK: u32 = D + 0x1c;
// Endpoint blocks (stride 0x20)
const fn diepctl(ep: u32) -> u32 {
    D + 0x100 + ep * 0x20
}
const fn diepint(ep: u32) -> u32 {
    D + 0x108 + ep * 0x20
}
const fn dieptsiz(ep: u32) -> u32 {
    D + 0x110 + ep * 0x20
}
const fn doepctl(ep: u32) -> u32 {
    D + 0x300 + ep * 0x20
}
const fn doepint(ep: u32) -> u32 {
    D + 0x308 + ep * 0x20
}
const fn doeptsiz(ep: u32) -> u32 {
    D + 0x310 + ep * 0x20
}
// EP0 TX FIFO
const FIFO0: u32 = FS + 0x1000;

#[allow(clippy::cast_possible_truncation)]
fn wr(u: &mut Usb, addr: u32, val: u32) {
    let mut a = addr;
    for shift in [0u32, 8, 16, 24] {
        u.write(a, ((val >> shift) & 0xFF) as u8).unwrap();
        a += 1;
    }
}

#[allow(clippy::cast_possible_truncation)]
fn rd(u: &mut Usb, addr: u32) -> u32 {
    let mut v = 0u32;
    for i in 0..4u32 {
        v |= u32::from(u.read(addr + i).unwrap()) << (8 * i);
    }
    v
}

/// Host frame injector.
fn inject(u: &mut Usb, frame: &[u8]) {
    for &b in frame {
        u.drive_inputs(u32::from(b));
    }
}

/// Device-mode init identical to the unit-test helper (public-surface only).
fn init_device(u: &mut Usb) {
    wr(u, GUSBCFG, 1 << 30); // FDMOD
    wr(u, GRSTCTL, 1); // CSRST soft reset
    wr(u, GUSBCFG, 1 << 30); // re-assert after reset
    wr(u, GAHBCFG, 1); // GINT
    wr(
        u,
        GINTMSK,
        (1 << 12) | (1 << 13) | (1 << 4) | (1 << 18) | (1 << 19),
    );
    wr(u, DIEPMSK, 1); // XFRCM
    wr(u, DOEPMSK, 1 | (1 << 3)); // XFRCM | STUPM
    wr(u, DAINTMSK, (1 << 0) | (1 << 16));
}

/// One SETUP packet: opcode 2, len 8, then the 8 setup bytes.
fn setup_pkt(pkt: &[u8; 8]) -> Vec<u8> {
    let mut f = vec![2u8, 8];
    f.extend_from_slice(pkt);
    f
}

/// ISR-side handling of a completed SETUP: pop the status word (GRXSTSR
/// pops on its lane-0 read) and read the payload from the RX FIFO.
fn take_setup(u: &mut Usb) -> ([u8; 8], u32) {
    let st = rd(u, FS + 0x1c); // GRXSTSR: destructive pop
    assert_eq!((st >> 17) & 0xF, 2, "PKTSTS == SETUP");
    let mut pkt = [0u8; 8];
    for b in &mut pkt {
        *b = u.read(FIFO0).unwrap();
    }
    (pkt, st)
}

/// Drain one IN-complete status word (what the RXFLVL ISR loop would do).
fn pop_in_complete(u: &mut Usb) {
    let st = rd(u, FS + 0x1c);
    assert_eq!((st >> 17) & 0xF, 4, "PKTSTS IN_COMPLETE");
}

#[test]
fn cdc_acm_full_enumeration_and_bulk_echo() {
    let mut u = Usb::otg_fs_for::<vioarm_core::device::svd::stm32f107::Stm32f107>();
    u.reset();
    init_device(&mut u);
    assert_eq!(u.pending(), 0);

    // ---- 1. Bus reset -----------------------------------------------------
    inject(&mut u, &[0x00, 0]); // CMD_RESET
    assert_ne!(u.pending(), 0, "reset raises the line");
    let g = rd(&mut u, GINTSTS);
    assert_ne!(g & (1 << 12), 0, "USBRST");
    wr(&mut u, GINTSTS, 1 << 12); // ISR w1c -> ENUMDNE latches
    assert_ne!(rd(&mut u, GINTSTS) & (1 << 13), 0, "ENUMDNE");
    assert_eq!(rd(&mut u, DSTS) & (0b11 << 1), 1 << 1, "full-speed ENUMSPD");
    wr(&mut u, GINTSTS, 1 << 13);

    // ---- 2. GET_DESCRIPTOR(DEVICE, 18 bytes) ------------------------------
    inject(
        &mut u,
        &setup_pkt(&[0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]),
    );
    assert_ne!(rd(&mut u, GINTSTS) & (1 << 4), 0, "RXFLVL");
    let (pkt, _st) = take_setup(&mut u);
    assert_eq!(pkt[1], 0x06, "bRequest GET_DESCRIPTOR");

    // Complete the SETUP stage on EP0 (STUPCNT-armed DOEPCTL0 -> STUP).
    wr(&mut u, doeptsiz(0), (1 << 29) | (1 << 19) | 8);
    wr(&mut u, doepctl(0), 1 << 31); // EPENA
    assert_ne!(rd(&mut u, doepint(0)) & (1 << 3), 0, "STUP");
    wr(&mut u, doepint(0), 1 << 3);

    // Data stage: 18-byte device descriptor into TX FIFO EP0, arm, fire IN.
    const DESC: [u8; 18] = [
        0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x83, 0x04, 0x40, 0x57, 0x00, 0x01, 0x01,
        0x02, 0x03, 0x01,
    ];
    for &b in &DESC {
        u.write(FIFO0, b).unwrap();
    }
    wr(&mut u, dieptsiz(0), 18);
    wr(&mut u, diepctl(0), 1 << 31); // EPENA drains the FIFO to the host
    assert_ne!(rd(&mut u, diepint(0)) & 1, 0, "EP0 XFRC");
    let out = u.drain_output();
    assert_eq!(out[0], 0x80, "EVT_IN");
    assert_eq!(out[1], 18);
    assert_eq!(out[2], 0, "EP0");
    assert_eq!(&out[3..21], &DESC, "descriptor bytes reach the host");
    wr(&mut u, diepint(0), 1);
    pop_in_complete(&mut u);

    // Status stage: host IN -> zero-length packet.
    wr(&mut u, dieptsiz(0), 0);
    wr(&mut u, diepctl(0), 1 << 31);
    let out = u.drain_output();
    assert_eq!((out[0], out[1], out[2]), (0x80, 0, 0), "ZLP status");
    pop_in_complete(&mut u);

    // ---- 3. SET_ADDRESS(7) — no data stage --------------------------------
    inject(
        &mut u,
        &setup_pkt(&[0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00]),
    );
    let (pkt, _) = take_setup(&mut u);
    assert_eq!(pkt[1], 0x05, "SET_ADDRESS");
    wr(&mut u, doeptsiz(0), (1 << 29) | (1 << 19) | 8);
    wr(&mut u, doepctl(0), 1 << 31);
    wr(&mut u, doepint(0), 1 << 3);
    wr(&mut u, dieptsiz(0), 0);
    wr(&mut u, diepctl(0), 1 << 31); // status ZLP
    assert_eq!(u.drain_output()[1], 0, "status ZLP");
    pop_in_complete(&mut u);

    // ---- 4. SET_CONFIGURATION(1) — no data stage --------------------------
    inject(
        &mut u,
        &setup_pkt(&[0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]),
    );
    let (pkt, _) = take_setup(&mut u);
    assert_eq!(pkt[1], 0x09, "SET_CONFIGURATION");
    wr(&mut u, doeptsiz(0), (1 << 29) | (1 << 19) | 8);
    wr(&mut u, doepctl(0), 1 << 31);
    wr(&mut u, doepint(0), 1 << 3);
    wr(&mut u, dieptsiz(0), 0);
    wr(&mut u, diepctl(0), 1 << 31);
    assert_eq!(u.drain_output()[1], 0);
    pop_in_complete(&mut u);

    // ---- 5. Configure EP1 bulk (CDC DATA) ---------------------------------
    wr(&mut u, diepctl(1), 1 << 15); // USBAEP on EP1
    wr(&mut u, doepctl(1), 1 << 15); // USBAEP
    wr(&mut u, doepctl(1), 1 << 26); // CNAK clears NAK

    // Bulk IN: CDC-ACM notification-ish payload leaves through EP1.
    let payload = [0xA1, 0x01, 0x00, 0x07]; // SERIAL_STATE header
    for &b in &payload {
        u.write(FS + 0x2000, b).unwrap(); // EP1 TX FIFO @ +0x1000*1
    }
    wr(&mut u, dieptsiz(1), (1 << 19) | 4);
    wr(&mut u, diepctl(1), 1 << 31);
    assert_ne!(rd(&mut u, diepint(1)) & 1, 0, "EP1 IN XFRC");
    let out = u.drain_output();
    assert_eq!(out[0], 0x80);
    assert_eq!(out[1], 4);
    assert_eq!(out[2], 1, "EP1 framed");
    assert_eq!(&out[3..7], &payload);
    wr(&mut u, diepint(1), 1);
    pop_in_complete(&mut u);

    // Bulk OUT: host sends CDC data; firmware reads it from the RX FIFO.
    wr(&mut u, doeptsiz(1), (1 << 19) | 64);
    wr(&mut u, doepctl(1), 1 << 31); // arm OUT EP1
    let rx = [0x68, 0x65, 0x6C, 0x6C, 0x6F]; // "hello"
    let mut frame = vec![3u8, rx.len() as u8, 1];
    frame.extend_from_slice(&rx);
    inject(&mut u, &frame);
    assert_ne!(rd(&mut u, doepint(1)) & 1, 0, "EP1 OUT XFRC");
    // Status word popped via GRXSTSR: PKTSTS_OUT (1), BCNT 5.
    let st = rd(&mut u, FS + 0x1c);
    assert_eq!((st >> 17) & 0xF, 1, "PKTSTS OUT");
    assert_eq!((st >> 4) & 0x7FF, 5);
    let mut got = Vec::new();
    for _ in 0..rx.len() {
        got.push(u.read(FIFO0).unwrap());
    }
    assert_eq!(got, rx, "host bytes land in the RX FIFO");
    // OUT-complete status follows in the queue.
    let st2 = rd(&mut u, FS + 0x1c);
    assert_eq!((st2 >> 17) & 0xF, 6, "PKTSTS OUT_COMPLETE");

    // The interrupt line reflects remaining unmasked events only.
    wr(&mut u, GINTSTS, 0xFFFF_FFFF);
    assert_eq!(rd(&mut u, GINTSTS) & (1 << 4), 0, "RX queue drained");
}

/// An isochronous IN endpoint (EPTYPE=01) defers its transfer completion to
/// the SOF boundary instead of completing on the EPENA write.
#[test]
fn iso_in_completes_on_sof_boundary() {
    let mut u = Usb::otg_fs_for::<vioarm_core::device::svd::stm32f107::Stm32f107>();
    u.reset();
    init_device(&mut u);
    inject(&mut u, &[0x00, 0]); // bus reset
    wr(&mut u, GINTSTS, 1 << 12); // clear USBRST -> ENUMDNE

    // EP1 as isochronous: EPTYPE=01 at bits 12:11, USBAEP.
    let payload = [0x10, 0x20, 0x30];
    for &b in &payload {
        u.write(FS + 0x2000, b).unwrap(); // EP1 TX FIFO
    }
    wr(&mut u, dieptsiz(1), (1 << 19) | payload.len() as u32);
    wr(&mut u, diepctl(1), (1 << 15) | (0b01 << 11)); // USBAEP | ISO
                                                      // Arm the transfer.
    wr(&mut u, diepctl(1), (0b01 << 11) | (1 << 31));
    // Nothing has been sent yet: no XFRC, no output frame.
    assert_eq!(rd(&mut u, diepint(1)) & 1, 0, "ISO IN waits for SOF");
    assert!(u.drain_output().is_empty(), "no bytes before SOF");
    // The next SOF lands exactly one packet.
    inject(&mut u, &[0x01, 0]); // CMD_SOF
    assert_ne!(rd(&mut u, diepint(1)) & 1, 0, "XFRC at SOF");
    let out = u.drain_output();
    assert_eq!(out[0], 0x80, "EVT_IN");
    assert_eq!(out[2], 1, "EP1");
    assert_eq!(&out[3..3 + payload.len()], &payload);
}

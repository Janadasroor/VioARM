//! H7 Ethernet static-IP echo e2e on the `h743` tables.
//!
//! Drives the HAL's register sequence through the public `Peripheral`/
//! `InterruptSource` surface: MAC + DMA enables, RX/TX descriptor rings,
//! then an ARP request → ARP reply and an ICMP echo request → echo reply,
//! with the test playing the firmware's IP stack (it crafts each reply
//! from the received frame's fields). No internal fields touched.

use std::sync::{Arc, Mutex};
use vioarm_core::device::stm32::eth::{EthH7, EthMailbox};
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::system::SystemMemory;
use vioarm_core::mem::Bus;

const MAC: u32 = 0x4002_8000;
const DMA: u32 = 0x4002_9000;
const RAM: u32 = 0x2000_0000;

const OWN: u32 = 1 << 31;
const BUF1V: u32 = 1 << 24;
const RX_FD: u32 = 1 << 29;
const RX_LD: u32 = 1 << 28;
const TX_FD: u32 = 1 << 29;
const TX_LD: u32 = 1 << 28;
const TI: u32 = 1 << 0;
const RI: u32 = 1 << 6;
const NIS: u32 = 1 << 15;

// Our station: locally-administered MAC + static IP.
const US_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
const US_IP: [u8; 4] = [192, 168, 7, 2];
const PEER_MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x02];
const PEER_IP: [u8; 4] = [192, 168, 7, 1];

fn wr(sys: &mut SystemMemory<FlatMemory>, addr: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        sys.write(addr + i as u32, *b).unwrap();
    }
}

fn rd(sys: &mut SystemMemory<FlatMemory>, addr: u32) -> u32 {
    let mut bytes = [0u8; 4];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = sys.read(addr + i as u32).unwrap();
    }
    u32::from_le_bytes(bytes)
}

fn cycle(sys: &mut SystemMemory<FlatMemory>) {
    sys.tick_devices(1);
}

fn sys() -> SystemMemory<FlatMemory> {
    let backing = FlatMemory::with_layout(&[RAM], &[0x4000]);
    let mailbox = Arc::new(Mutex::new(EthMailbox::default()));
    let mut mem = SystemMemory::new(backing);
    mem.set_eth_mailbox(mailbox.clone());
    mem.attach(Box::new(EthH7::new_for::<
        vioarm_core::device::svd::stm32h743::Stm32h743,
    >(mailbox)));
    mem
}

fn inject(sys: &mut SystemMemory<FlatMemory>, frame: &[u8]) {
    let len = frame.len() as u32;
    sys.devices_mut()[0].drive_inputs(0x01);
    sys.devices_mut()[0].drive_inputs(len & 0xFF);
    sys.devices_mut()[0].drive_inputs((len >> 8) & 0xFF);
    for b in frame {
        sys.devices_mut()[0].drive_inputs(u32::from(*b));
    }
}

fn drain_tx(sys: &mut SystemMemory<FlatMemory>) -> Vec<u8> {
    let out = sys.devices_mut()[0].drain_output();
    assert_eq!(out[0], 0x80, "EVT_TX");
    let len = out[1] as usize | ((out[2] as usize) << 8);
    assert_eq!(out.len(), 3 + len);
    out[3..].to_vec()
}

/// HAL-`Start` order init + 4-entry rings with 1536-byte buffers.
fn bring_up(sys: &mut SystemMemory<FlatMemory>) {
    wr(sys, MAC, (1 << 1) | 1 | (1 << 14) | (1 << 13)); // TE RE FES DM
    wr(
        sys,
        MAC + 0x300,
        (1 << 31) | ((US_MAC[5] as u32) << 8) | US_MAC[4] as u32,
    );
    wr(
        sys,
        MAC + 0x304,
        ((US_MAC[3] as u32) << 24)
            | ((US_MAC[2] as u32) << 16)
            | ((US_MAC[1] as u32) << 8)
            | US_MAC[0] as u32,
    );
    for i in 0..4u32 {
        let d = RAM + 0x100 + i * 16;
        wr(sys, d, RAM + 0x800 + i * 0x600);
        wr(sys, d + 12, OWN | BUF1V);
    }
    wr(sys, DMA + 0x11c, RAM + 0x100); // DMACRxDLAR
    wr(sys, DMA + 0x130, 3); // DMACRxRLR
    wr(sys, DMA + 0x128, RAM + 0x130); // DMACRDTPR tail
    wr(sys, DMA + 0x108, 1 | (1536 << 1)); // DMACRxCR SR|RBSZ
    for i in 0..4u32 {
        wr(sys, RAM + 0x200 + i * 16 + 12, 0);
    }
    wr(sys, DMA + 0x114, RAM + 0x200); // DMACTxDLAR
    wr(sys, DMA + 0x12c, 3); // DMACTxRLR
    wr(sys, DMA + 0x104, 1); // DMACTxCR ST
    wr(sys, DMA + 0x134, (1 << 15) | (1 << 6) | 1); // DMACIER NIE|RIE|TIE
}

/// Wait for `desc` to return to the CPU; returns its status word.
fn poll_rx(sys: &mut SystemMemory<FlatMemory>, desc: u32) -> u32 {
    for _ in 0..8 {
        cycle(sys);
        let s = rd(sys, desc + 12);
        if s & OWN == 0 {
            return s;
        }
    }
    panic!("RX descriptor never completed");
}

/// Read the received frame out of the ring buffer.
fn read_rx(sys: &mut SystemMemory<FlatMemory>, idx: u32) -> Vec<u8> {
    let desc = RAM + 0x100 + idx * 16;
    let s = poll_rx(sys, desc);
    assert_ne!(s & RX_FD, 0, "FD");
    assert_ne!(s & RX_LD, 0, "LD");
    let len = (s & 0x7FFF) as usize;
    let buf = RAM + 0x800 + idx * 0x600;
    (0..len as u32)
        .map(|i| sys.read(buf + i).unwrap())
        .collect()
}

/// Re-arm an RX descriptor and hand it back with a tail-pointer write.
fn free_rx(sys: &mut SystemMemory<FlatMemory>, idx: u32) {
    let desc = RAM + 0x100 + idx * 16;
    wr(sys, desc + 12, OWN | BUF1V);
    wr(sys, DMA + 0x128, desc);
}

/// Transmit `frame` through TX descriptor `idx` and return when done.
fn transmit(sys: &mut SystemMemory<FlatMemory>, idx: u32, frame: &[u8]) {
    let desc = RAM + 0x200 + idx * 16;
    let buf = RAM + 0x2000 + idx * 0x600;
    for (i, b) in frame.iter().enumerate() {
        sys.write(buf + i as u32, *b).unwrap();
    }
    let len = frame.len() as u32;
    wr(sys, desc, buf);
    wr(sys, desc + 4, 0);
    wr(sys, desc + 8, len & 0x3FFF);
    wr(sys, desc + 12, OWN | TX_FD | TX_LD | (len & 0x7FFF));
    wr(sys, DMA + 0x120, desc + 16); // DMACTxDTPR kick
    for _ in 0..10 {
        cycle(sys);
        if rd(sys, desc + 12) & OWN == 0 {
            return;
        }
    }
    panic!("TX descriptor never completed");
}

fn arp_request() -> Vec<u8> {
    let mut f = vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    f.extend(PEER_MAC);
    f.extend([0x08, 0x06]);
    f.extend([0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01]);
    f.extend(PEER_MAC);
    f.extend(PEER_IP);
    f.extend([0, 0, 0, 0, 0, 0]);
    f.extend(US_IP);
    f
}

fn arp_reply(req: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(req.len());
    f.extend(PEER_MAC); // dst = requester
    f.extend(US_MAC);
    f.extend([0x08, 0x06]);
    f.extend([0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x02]); // reply
    f.extend(US_MAC);
    f.extend(US_IP);
    f.extend(&req[22..28]); // requester's MAC
    f.extend(&req[28..32]); // requester's IP
    f
}

fn ip_checksum(hdr: &[u8]) -> [u8; 2] {
    let mut sum = 0u32;
    for w in hdr.chunks_exact(2) {
        sum += u16::from_be_bytes([w[0], w[1]]) as u32;
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    (!(sum as u16)).to_be_bytes()
}

fn icmp_echo_request(ident: u16, seq: u16, payload: &[u8]) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(US_MAC);
    f.extend(PEER_MAC);
    f.extend([0x08, 0x00]);
    let total = (20 + 8 + payload.len()) as u16;
    let mut ip = vec![
        0x45,
        0x00,
        (total >> 8) as u8,
        total as u8,
        0x12,
        0x34,
        0x00,
        0x00,
        64,
        0x01,
        0x00,
        0x00,
    ];
    ip.extend(PEER_IP);
    ip.extend(US_IP);
    let c = ip_checksum(&ip);
    ip[10] = c[0];
    ip[11] = c[1];
    f.extend(ip);
    let mut icmp = vec![
        0x08,
        0x00,
        0x00,
        0x00,
        (ident >> 8) as u8,
        ident as u8,
        (seq >> 8) as u8,
        seq as u8,
    ];
    icmp.extend(payload);
    let c = ip_checksum(&icmp);
    icmp[2] = c[0];
    icmp[3] = c[1];
    f.extend(icmp);
    f
}

fn icmp_echo_reply(req: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(req.len());
    f.extend(&req[6..12]); // dst = requester's MAC
    f.extend(US_MAC);
    f.extend([0x08, 0x00]);
    let mut ip = req[14..34].to_vec();
    ip[8] = 64; // TTL
    ip[10] = 0;
    ip[11] = 0;
    ip[12..16].copy_from_slice(&US_IP);
    ip[16..20].copy_from_slice(&PEER_IP);
    let c = ip_checksum(&ip);
    ip[10] = c[0];
    ip[11] = c[1];
    f.extend(&ip);
    let mut icmp = req[34..].to_vec();
    icmp[0] = 0; // echo reply
    icmp[2] = 0;
    icmp[3] = 0;
    let c = ip_checksum(&icmp);
    icmp[2] = c[0];
    icmp[3] = c[1];
    f.extend(icmp);
    f
}

#[test]
fn h7_static_ip_arp_echo() {
    let mut mem = sys();
    bring_up(&mut mem);

    // ---- ARP request (broadcast) → ARP reply ---------------------------
    inject(&mut mem, &arp_request());
    let req = read_rx(&mut mem, 0);
    assert_eq!(req[12..14], [0x08, 0x06], "ethertype ARP");
    assert_eq!(&req[38..42], &US_IP, "asking for us");
    assert_ne!(rd(&mut mem, DMA + 0x160) & (RI | NIS), 0, "RI+NIS");
    assert_eq!(mem.devices()[0].pending(), 1, "ETH line");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF); // ISR clears
    assert_eq!(mem.devices()[0].pending(), 0);
    free_rx(&mut mem, 0);

    let reply = arp_reply(&req);
    transmit(&mut mem, 0, &reply);
    assert_ne!(rd(&mut mem, DMA + 0x160) & (TI | NIS), 0, "TI+NIS");
    let tx = drain_tx(&mut mem);
    assert_eq!(tx, reply, "ARP reply on the wire");
    assert_eq!(&tx[0..6], &PEER_MAC, "to requester");
    assert_eq!(&tx[6..12], &US_MAC, "from us");
    assert_eq!(tx[21], 2, "opcode reply");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
}

#[test]
fn h7_static_ip_icmp_echo() {
    let mut mem = sys();
    bring_up(&mut mem);

    // ---- ICMP echo request (unicast) → echo reply -----------------------
    let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11];
    inject(&mut mem, &icmp_echo_request(0x1234, 0x0007, &payload));
    let req = read_rx(&mut mem, 0);
    assert_eq!(req[12..14], [0x08, 0x00], "ethertype IPv4");
    assert_eq!(req[23], 1, "protocol ICMP");
    assert_eq!(&req[30..34], &US_IP, "for us");
    assert_eq!(req[34], 8, "echo request");
    free_rx(&mut mem, 0);

    let reply = icmp_echo_reply(&req);
    transmit(&mut mem, 0, &reply);
    let tx = drain_tx(&mut mem);
    assert_eq!(tx.len(), req.len(), "same length back");
    assert_eq!(&tx[0..6], &PEER_MAC, "to peer");
    assert_eq!(&tx[6..12], &US_MAC, "from us");
    assert_eq!(&tx[30..34], &PEER_IP, "IP dst peer");
    assert_eq!(&tx[26..30], &US_IP, "IP src us");
    assert_eq!(tx[34], 0, "echo reply");
    assert_eq!(&tx[38..40], &req[38..40], "ident preserved");
    assert_eq!(&tx[42..], &payload, "payload echoed");
    // Checksums the stack recomputed verify cleanly.
    assert_eq!(ip_checksum(&tx[14..34]), [0, 0], "IP checksum valid");
    assert_eq!(ip_checksum(&tx[34..]), [0, 0], "ICMP checksum valid");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
    assert_eq!(mem.devices()[0].pending(), 0);
}

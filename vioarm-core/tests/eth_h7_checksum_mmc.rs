//! H7 Ethernet Phase-B proof: checksum offload, multi-buffer frames, and
//! MMC counters on the `h743` tables.
//!
//! Drives the HAL's register sequence through the public `Peripheral`/
//! `InterruptSource` surface: a 1400-byte UDP frame transmits with zeroed
//! checksums and `TDES3.CIC` set, and the wire bytes carry valid IP + UDP
//! checksums; the same frame splits across two TX descriptors (`FD`/`LD`)
//! and emits once; a 1442-byte RX frame scatters across 512-byte buffers
//! with clean checksum status, while a corrupted IP header latches
//! `IPHE`+`ES` without dropping the frame. MMC packet counters track both
//! directions. No internal fields touched.

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
const RX_ES: u32 = 1 << 15;
const TX_FD: u32 = 1 << 29;
const TX_LD: u32 = 1 << 28;
const CIC_FULL: u32 = 3 << 16;
const CIC_IP_ONLY: u32 = 1 << 16;
const TI: u32 = 1 << 0;
const RI: u32 = 1 << 6;
const RDES1_IPHE: u32 = 1 << 3;
const MMC_TX_GOOD: u32 = MAC + 0x768;
const MMC_RX_UCAST: u32 = MAC + 0x7C4;

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

/// HAL-`Start` order init + rings, with `MACCR.IPC` on so RX checksum
/// status reports.
fn bring_up(sys: &mut SystemMemory<FlatMemory>, rbsz: u32, ndesc: u32) {
    wr(sys, MAC, (1 << 1) | 1 | (1 << 14) | (1 << 13) | (1 << 27)); // TE RE FES DM IPC
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
    for i in 0..ndesc {
        let d = RAM + 0x100 + i * 16;
        wr(sys, d, RAM + 0x800 + i * 0x600);
        wr(sys, d + 12, OWN | BUF1V);
    }
    wr(sys, DMA + 0x11c, RAM + 0x100); // DMACRxDLAR
    wr(sys, DMA + 0x130, ndesc - 1); // DMACRxRLR
    wr(sys, DMA + 0x128, RAM + 0x100 + (ndesc - 1) * 16); // DMACRDTPR tail
    wr(sys, DMA + 0x108, 1 | (rbsz << 1)); // DMACRxCR SR|RBSZ
    for i in 0..ndesc {
        wr(sys, RAM + 0x200 + i * 16 + 12, 0);
    }
    wr(sys, DMA + 0x114, RAM + 0x200); // DMACTxDLAR
    wr(sys, DMA + 0x12c, ndesc - 1); // DMACTxRLR
    wr(sys, DMA + 0x104, 1); // DMACTxCR ST
    wr(sys, DMA + 0x134, (1 << 15) | (1 << 6) | 1); // DMACIER NIE|RIE|TIE
}

/// Wait for `desc` to return to the CPU; returns its status word.
fn poll_rx(sys: &mut SystemMemory<FlatMemory>, desc: u32) -> u32 {
    for _ in 0..12 {
        cycle(sys);
        let s = rd(sys, desc + 12);
        if s & OWN == 0 {
            return s;
        }
    }
    panic!("RX descriptor never completed");
}

/// Transmit `frame` through TX descriptor `idx` with `flags` in `TDES3`
/// and return when done.
fn transmit_flags(sys: &mut SystemMemory<FlatMemory>, idx: u32, frame: &[u8], flags: u32) {
    let desc = RAM + 0x200 + idx * 16;
    let buf = RAM + 0x2000 + idx * 0x600;
    for (i, b) in frame.iter().enumerate() {
        sys.write(buf + i as u32, *b).unwrap();
    }
    let len = frame.len() as u32;
    wr(sys, desc, buf);
    wr(sys, desc + 4, 0);
    wr(sys, desc + 8, len & 0x3FFF);
    wr(sys, desc + 12, OWN | flags | (len & 0x7FFF));
    wr(sys, DMA + 0x120, desc + 16); // DMACTxDTPR kick
    for _ in 0..12 {
        cycle(sys);
        if rd(sys, desc + 12) & OWN == 0 {
            return;
        }
    }
    panic!("TX descriptor never completed");
}

/// A UDP frame with zeroed checksums, ready for offload insertion.
fn udp_frame(payload_len: usize) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend(US_MAC);
    f.extend(PEER_MAC);
    f.extend([0x08, 0x00]);
    let total = (20 + 8 + payload_len) as u16;
    f.extend([
        0x45,
        0x00,
        (total >> 8) as u8,
        total as u8,
        0x56,
        0x78,
        0x00,
        0x00,
        64,
        0x11,
        0x00,
        0x00,
    ]);
    f.extend(PEER_IP);
    f.extend(US_IP);
    let udp_len = (8 + payload_len) as u16;
    f.extend([
        0x04,
        0xD2,
        0x16,
        0x2E,
        (udp_len >> 8) as u8,
        udp_len as u8,
        0x00,
        0x00,
    ]);
    for i in 0..payload_len {
        f.push((i & 0xFF) as u8);
    }
    f
}

/// Folded one's-complement sum; zero means the checksummed region (checksum
/// field included) is valid.
fn csum_ok(bytes: &[u8]) -> bool {
    let mut sum = 0u32;
    let mut it = bytes.chunks_exact(2);
    for w in &mut it {
        sum += u16::from_be_bytes([w[0], w[1]]) as u32;
    }
    if let Some(&last) = it.remainder().first() {
        sum += (last as u32) << 8;
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    sum == 0xFFFF
}

/// UDP checksum with the IPv4 pseudo-header, as the wire must carry it.
fn udp_ok(frame: &[u8]) -> bool {
    let ip = &frame[14..34];
    let seg = &frame[34..];
    let mut pseudo = Vec::new();
    pseudo.extend(&ip[12..20]);
    pseudo.push(0);
    pseudo.push(ip[9]);
    let len = seg.len() as u16;
    pseudo.extend(len.to_be_bytes());
    pseudo.extend(seg);
    csum_ok(&pseudo)
}

#[test]
fn h7_tx_checksum_insertion_large_udp() {
    let mut mem = sys();
    bring_up(&mut mem, 1536, 4);

    // ---- CIC=3: IP header + UDP checksum inserted on the wire ---------
    let frame = udp_frame(1400);
    assert_eq!(frame.len(), 14 + 20 + 8 + 1400);
    transmit_flags(&mut mem, 0, &frame, TX_FD | TX_LD | CIC_FULL);
    assert_ne!(rd(&mut mem, DMA + 0x160) & TI, 0, "TI");
    let tx = drain_tx(&mut mem);
    assert_eq!(tx.len(), frame.len(), "same length back");
    // Checksum fields excluded: verified independently below.
    assert_eq!(&tx[..24], &frame[..24], "headers untouched");
    assert_eq!(&tx[26..40], &frame[26..40], "ports/length untouched");
    assert_eq!(&tx[42..], &frame[42..], "payload untouched");
    assert!(csum_ok(&tx[14..34]), "inserted IP checksum valid");
    assert!(udp_ok(&tx), "inserted UDP checksum valid");
    assert_eq!(rd(&mut mem, MMC_TX_GOOD), 1, "TX packet counted");

    // ---- CIC=1: IP header only, UDP checksum left zero ----------------
    transmit_flags(&mut mem, 1, &frame, TX_FD | TX_LD | CIC_IP_ONLY);
    let tx = drain_tx(&mut mem);
    assert!(csum_ok(&tx[14..34]), "IP checksum valid");
    assert_eq!(&tx[40..42], &[0, 0], "UDP checksum untouched");
    assert_eq!(rd(&mut mem, MMC_TX_GOOD), 2, "second TX counted");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
}

#[test]
fn h7_tx_multi_buffer_chain_emits_once() {
    let mut mem = sys();
    bring_up(&mut mem, 1536, 4);

    // One frame across two descriptors: FD without LD, then LD.
    let frame = udp_frame(900);
    let (head, tail) = frame.split_at(600);
    for (idx, part) in [head, tail].iter().enumerate() {
        let desc = RAM + 0x200 + idx as u32 * 16;
        let buf = RAM + 0x2000 + idx as u32 * 0x600;
        for (i, b) in part.iter().enumerate() {
            mem.write(buf + i as u32, *b).unwrap();
        }
        wr(&mut mem, desc, buf);
        wr(&mut mem, desc + 4, 0);
        wr(&mut mem, desc + 8, (part.len() as u32) & 0x3FFF);
    }
    wr(&mut mem, RAM + 0x200 + 12, OWN | TX_FD | CIC_FULL);
    wr(&mut mem, RAM + 0x210 + 12, OWN | TX_LD);
    wr(&mut mem, DMA + 0x120, RAM + 0x220); // kick
    for _ in 0..16 {
        cycle(&mut mem);
        if rd(&mut mem, RAM + 0x210 + 12) & OWN == 0 {
            break;
        }
    }
    assert_eq!(rd(&mut mem, RAM + 0x200 + 12) & OWN, 0, "desc0 done");
    assert_eq!(rd(&mut mem, RAM + 0x210 + 12) & OWN, 0, "desc1 done");
    let tx = drain_tx(&mut mem);
    assert_eq!(tx.len(), frame.len(), "one frame emitted");
    // Checksum fields excluded: verified independently below.
    assert_eq!(&tx[..24], &frame[..24], "head headers");
    assert_eq!(&tx[26..40], &frame[26..40], "head ports/length");
    assert_eq!(&tx[42..], &frame[42..], "chained payload");
    assert!(csum_ok(&tx[14..34]), "IP checksum over the chain");
    assert!(udp_ok(&tx), "UDP checksum over the chain");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
}

#[test]
fn h7_rx_scatter_and_checksum_status() {
    let mut mem = sys();
    bring_up(&mut mem, 512, 8);

    // ---- Valid checksums, IPC on: clean status across 3 buffers -------
    let mut frame = udp_frame(1400);
    // Fill in correct checksums the way a peer's MAC would.
    {
        let mut sum = 0u32;
        for w in frame[14..34].chunks_exact(2) {
            sum += u16::from_be_bytes([w[0], w[1]]) as u32;
        }
        while sum >> 16 != 0 {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        let c = (!(sum as u16)).to_be_bytes();
        frame[24] = c[0];
        frame[25] = c[1];
        let mut pseudo = Vec::new();
        pseudo.extend(&frame[26..34]);
        pseudo.extend([0, 0x11]);
        pseudo.extend(((frame.len() - 34) as u16).to_be_bytes());
        pseudo.extend(&frame[34..]);
        let mut sum = 0u32;
        for w in pseudo.chunks_exact(2) {
            sum += u16::from_be_bytes([w[0], w[1]]) as u32;
        }
        while sum >> 16 != 0 {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        let c = (!(sum as u16)).to_be_bytes();
        frame[40] = c[0];
        frame[41] = c[1];
    }
    assert_eq!(frame.len(), 1442, "3 buffers at RBSZ 512");
    inject(&mut mem, &frame);
    let s0 = poll_rx(&mut mem, RAM + 0x100);
    let s1 = poll_rx(&mut mem, RAM + 0x110);
    let s2 = poll_rx(&mut mem, RAM + 0x120);
    assert_ne!(s0 & RX_FD, 0, "FD first");
    assert_eq!(s0 & (RX_LD | RX_ES), 0, "no LD/ES first");
    assert_eq!(s0 & 0x7FFF, 512, "PL0");
    assert_eq!(s1 & (RX_FD | RX_LD | RX_ES), 0, "middle clean");
    assert_eq!(s1 & 0x7FFF, 512, "PL1");
    assert_ne!(s2 & RX_LD, 0, "LD last");
    assert_eq!(s2 & RX_ES, 0, "no error on valid frame");
    assert_eq!(s2 & 0x7FFF, 418, "PL2");
    assert_eq!(rd(&mut mem, RAM + 0x120 + 4) & 0xFF, 0, "checksum clean");
    let mut got = Vec::new();
    for (idx, len) in [(0u32, 512usize), (1, 512), (2, 418)] {
        let buf = RAM + 0x800 + idx * 0x600;
        for i in 0..len {
            got.push(mem.read(buf + i as u32).unwrap());
        }
    }
    assert_eq!(got, frame, "frame reassembled");
    assert_ne!(rd(&mut mem, DMA + 0x160) & RI, 0, "RI");
    assert_eq!(rd(&mut mem, MMC_RX_UCAST), 1, "RX unicast counted");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
    for i in 0..3u32 {
        let d = RAM + 0x100 + i * 16;
        wr(&mut mem, d + 12, OWN | BUF1V);
    }
    wr(&mut mem, DMA + 0x128, RAM + 0x120);

    // ---- Corrupted IP header: IPHE + ES, frame still delivered --------
    let mut bad = frame.clone();
    bad[22] ^= 0xFF; // TTL byte, checksum now wrong
    inject(&mut mem, &bad);
    let s3 = poll_rx(&mut mem, RAM + 0x130);
    poll_rx(&mut mem, RAM + 0x140);
    let s5 = poll_rx(&mut mem, RAM + 0x150);
    assert_ne!(s5 & RX_LD, 0, "LD last");
    assert_ne!(s5 & RX_ES, 0, "error summary");
    assert_ne!(rd(&mut mem, RAM + 0x150 + 4) & RDES1_IPHE, 0, "IPHE");
    let _ = s3;
    let mut got = Vec::new();
    for (idx, len) in [(3u32, 512usize), (4, 512), (5, 418)] {
        let buf = RAM + 0x800 + idx * 0x600;
        for i in 0..len {
            got.push(mem.read(buf + i as u32).unwrap());
        }
    }
    assert_eq!(got, bad, "flagged frame still delivered");
    assert_eq!(rd(&mut mem, MMC_RX_UCAST), 2, "flagged frame counted");
    wr(&mut mem, DMA + 0x160, 0xFFFF_FFFF);
}

//! Board-level `EMAC0` TX proof on `tm4c129xnczad`: a real transmit
//! descriptor ring in SRAM walks the bus-coupled DMA path — descriptor
//! read, buffer read, ownership write-back, `TI` — with no test doubles.
//!
//! RX is proven at lib level (`device::ti::emac`, mailbox-seeded);
//! frames have no byte-channel framing for host injection, so there is
//! no board-level RX inject path (same boundary as CAN/USB frame drains).
//!
//! Checksum insertion values live in the DMA-accumulated frame
//! (peripheral-side), so the board observes the status write-back
//! (`IHE`/`ES`) and the MMC counters — the values themselves are
//! proven against Python-oracle vectors at lib level.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const EMAC0: u32 = 0x400E_C000;
const DMAOPMODE: u32 = 0xC18;
const TXPOLLD: u32 = 0xC04;
const TXDLADDR: u32 = 0xC10;
const DMARIS: u32 = 0xC14;
const ST: u32 = 1 << 13;
const TI: u32 = 1 << 0;
const TXCNTGB: u32 = 0x118;
const TXOCTCNTG: u32 = 0x164;
const CIC_SHIFT: u32 = 22;
const IHE: u32 = 1 << 16;
const ES: u32 = 1 << 15;

const SRAM: u32 = 0x2000_0000;
const DESC: u32 = 0x2000_0000;
const BUF: u32 = 0x2000_0100;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

#[test]
fn emac_tx_walks_the_bus() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    // Frame bytes in SRAM.
    ww(mem, BUF, 0xBEAD_DEAD);
    // Descriptor: OWN|LS|FS, TBS1=4, buffer, ring-next.
    ww(mem, DESC, 0xBC00_0000);
    ww(mem, DESC + 4, 4);
    ww(mem, DESC + 8, BUF);
    ww(mem, DESC + 12, DESC);
    // Enable TX, point the ring, poll.
    ww(mem, EMAC0 + DMAOPMODE, ST);
    ww(mem, EMAC0 + TXDLADDR, DESC);
    ww(mem, EMAC0 + TXPOLLD, 0x01);
    for _ in 0..20 {
        mem.tick_devices(1);
    }
    assert_eq!(rw(mem, EMAC0 + DMARIS) & TI, TI, "transmit completes");
    assert_eq!(rw(mem, DESC) & 0x8000_0000, 0, "ownership home");
}

/// UDP/IPv4 frame bytes with zeroed checksums (47 bytes; the engine
/// inserts `b1f6` + `3472` per the lib-level vectors).
fn udp_frame() -> Vec<u8> {
    let mut f = vec![
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1, 0x08, 0x00,
    ];
    f.extend_from_slice(&[
        0x45, 0x00, 0x00, 0x21, 0x1c, 0x46, 0x40, 0x00, 0x40, 0x11, 0x00, 0x00, 0xac, 0x10, 0x0a,
        0x63, 0xac, 0x10, 0x0a, 0x0c, 0x04, 0xd2, 0x16, 0x2e, 0x00, 0x0d, 0x00, 0x00, 0x68, 0x65,
        0x6c, 0x6c, 0x6f,
    ]);
    f
}

/// Transmit `frame` with `CIC`; returns the descriptor status word.
fn tx_with_cic(mem: &mut board::BoardMemory, cic: u32, frame: &[u8]) -> u32 {
    let mut padded = frame.to_vec();
    while padded.len() % 4 != 0 {
        padded.push(0);
    }
    for (i, w) in padded.chunks(4).enumerate() {
        ww(
            mem,
            BUF + 4 * i as u32,
            u32::from_le_bytes([w[0], w[1], w[2], w[3]]),
        );
    }
    ww(mem, DESC, 0xBC00_0000 | (cic << CIC_SHIFT));
    ww(mem, DESC + 4, frame.len() as u32);
    ww(mem, DESC + 8, BUF);
    ww(mem, DESC + 12, DESC);
    ww(mem, EMAC0 + DMAOPMODE, ST);
    ww(mem, EMAC0 + TXDLADDR, DESC);
    ww(mem, EMAC0 + TXPOLLD, 0x01);
    for _ in 0..20 {
        mem.tick_devices(1);
    }
    assert_eq!(rw(mem, EMAC0 + DMARIS) & TI, TI, "transmit completes");
    rw(mem, DESC)
}

#[test]
fn emac_tx_cic3_clean_and_counted() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    let status = tx_with_cic(mem, 3, &udp_frame());
    assert_eq!(status & (IHE | ES), 0, "engine accepted the frame");
    assert_eq!(rw(mem, EMAC0 + TXCNTGB), 1, "frame counted");
    assert_eq!(rw(mem, EMAC0 + TXOCTCNTG), 47, "octets counted");
}

#[test]
fn emac_tx_bad_version_flags_ihe() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    let mut frame = udp_frame();
    frame[14] = 0x60; // EtherType says IPv4, version says otherwise.
    let status = tx_with_cic(mem, 1, &frame);
    assert_eq!(status & (IHE | ES), IHE | ES);
}

/// Default FCS path on real SRAM: `DC=0`/`DP=0` pads the 4-byte buffer
/// to 60 and appends the CRC — 64 octets counted, status clean.
#[test]
fn emac_tx_crc_append_counted() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    ww(mem, BUF, 0xBEAD_DEAD);
    ww(mem, DESC, 0xB000_0000); // OWN|LS|FS, DC=0 appends, DP=0 pads.
    ww(mem, DESC + 4, 4);
    ww(mem, DESC + 8, BUF);
    ww(mem, DESC + 12, DESC);
    ww(mem, EMAC0 + DMAOPMODE, ST);
    ww(mem, EMAC0 + TXDLADDR, DESC);
    ww(mem, EMAC0 + TXPOLLD, 0x01);
    for _ in 0..20 {
        mem.tick_devices(1);
    }
    assert_eq!(rw(mem, EMAC0 + DMARIS) & TI, TI, "transmit completes");
    assert_eq!(rw(mem, DESC) & (IHE | ES), 0, "engine accepted the frame");
    assert_eq!(rw(mem, EMAC0 + TXCNTGB), 1, "frame counted");
    assert_eq!(rw(mem, EMAC0 + TXOCTCNTG), 64, "pad + FCS counted");
}

//! Virtual radio e2e: SUBGHZ packet ping-pong and BLE advertising between
//! two machines over the fabric radio bus, plus single-machine loopback.
//!
//! Driver-style register pokes on real boards (no RF, no toolchain): TX
//! bytes staged through `TXDATA`/`TXGO` appear on the peer's `RXDATA`,
//! loopback copies into the own RX FIFO with no peer attached.

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::fabric::Fabric;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const SUBGHZ: u32 = 0x5800_1400;
const BLE: u32 = 0x5800_0000;
const CR: u32 = 0x00;
const TXLEN: u32 = 0x20;
const TXDATA: u32 = 0x24;
const TXGO: u32 = 0x28;
const RXSTAT: u32 = 0x2c;
const RXDATA: u32 = 0x30;

const EN: u32 = 1;
const LOOPBACK: u32 = 1 << 1;

/// Status register offset: `0x10` keeps the WL layout, `0x04` the BLE one.
const fn sr_off(base: u32) -> u32 {
    if base == SUBGHZ {
        0x10
    } else {
        0x04
    }
}

fn wr(mem: &mut BoardMemory, base: u32, off: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        mem.write(base + off + i as u32, *b).unwrap();
    }
}

fn rd(mem: &mut BoardMemory, base: u32, off: u32) -> u32 {
    let mut bytes = [0u8; 4];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = mem.read(base + off + i as u32).unwrap();
    }
    u32::from_le_bytes(bytes)
}

/// Enable the endpoint (preserving other CR bits, as a driver RMW would)
/// and transmit `payload` (byte-lane writes, as firmware would stage them
/// through either word or byte stores).
fn transmit(mem: &mut BoardMemory, base: u32, payload: &[u8]) {
    let cr = rd(mem, base, CR);
    wr(mem, base, CR, cr | EN);
    for &b in payload {
        mem.write(base + TXDATA, b).unwrap();
    }
    wr(mem, base, TXLEN, payload.len() as u32);
    wr(mem, base, TXGO, 1);
    assert_ne!(rd(mem, base, sr_off(base)) & (1 << 2), 0, "TXDONE");
}

/// Pop all pending RX bytes (single-byte reads: the FIFO consumes exactly
/// what each access reads, so one pop per byte).
fn receive(mem: &mut BoardMemory, base: u32) -> Vec<u8> {
    let n = rd(mem, base, RXSTAT);
    (0..n).map(|_| mem.read(base + RXDATA).unwrap()).collect()
}

#[test]
fn subghz_two_machine_ping_pong() {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let a = f.add(Machine::armv7m(board::stm32wle5()));
    let b = f.add(Machine::armv7m(board::stm32wle5()));
    assert_eq!((a, b), (0, 1));
    f.link_radio(&[(0, "subghz"), (1, "subghz")]);
    // A pings; the frame crosses on the next pump.
    transmit(f.machines_mut()[0].core.mem(), SUBGHZ, b"ping");
    f.run(10);
    let got = receive(f.machines_mut()[1].core.mem(), SUBGHZ);
    assert_eq!(got, b"ping", "peer received the frame");
    // B pongs back.
    transmit(f.machines_mut()[1].core.mem(), SUBGHZ, b"pong");
    f.run(10);
    let back = receive(f.machines_mut()[0].core.mem(), SUBGHZ);
    assert_eq!(back, b"pong", "reply came home");
    // Neither heard its own transmission (no self-delivery).
    assert_eq!(
        receive(f.machines_mut()[0].core.mem(), SUBGHZ),
        Vec::<u8>::new()
    );
}

#[test]
fn subghz_loopback_without_peer() {
    let mut m = Machine::armv7m(board::stm32wle5());
    // Loopback on, no fabric anywhere: TXGO echoes into the own RX FIFO
    // and nothing reaches the host channel.
    wr(m.core.mem(), SUBGHZ, CR, EN | LOOPBACK);
    transmit(m.core.mem(), SUBGHZ, b"self");
    assert_eq!(receive(m.core.mem(), SUBGHZ), b"self", "echoed");
    assert!(m.drain_device("subghz").is_empty(), "nothing aired");
    // Loopback off, still no peer: the frame airs to nobody, RX stays empty.
    {
        let mem = m.core.mem();
        wr(mem, SUBGHZ, CR, EN);
        for &b in b"void" {
            mem.write(SUBGHZ + TXDATA, b).unwrap();
        }
        wr(mem, SUBGHZ, TXLEN, 4);
        wr(mem, SUBGHZ, TXGO, 1);
        assert_eq!(rd(mem, SUBGHZ, RXSTAT), 0, "no echo without loopback");
    }
    assert_eq!(m.drain_device("subghz"), b"void", "aired to the void");
}

#[test]
fn ble_adv_two_peers() {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let a = f.add(Machine::armv7m(board::stm32wba55()));
    let b = f.add(Machine::armv7m(board::stm32wba55()));
    assert_eq!((a, b), (0, 1));
    f.link_radio(&[(0, "ble"), (1, "ble")]);
    // An ADV event: 6-byte AdvA plus advertising data.
    let adv: Vec<u8> = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01]
        .into_iter()
        .chain([0x02, 0x01, 0x06, 0x05, 0x09, b'V', b'i', b'o'])
        .collect();
    transmit(f.machines_mut()[0].core.mem(), BLE, &adv);
    f.run(10);
    let got = receive(f.machines_mut()[1].core.mem(), BLE);
    assert_eq!(got, adv, "peer received the ADV event byte-exact");
}

#[test]
fn ble_loopback() {
    let mut m = Machine::armv7m(board::stm32wba55());
    let mem = m.core.mem();
    wr(mem, BLE, CR, EN | LOOPBACK);
    transmit(mem, BLE, b"adv");
    assert_eq!(receive(mem, BLE), b"adv");
}

#[test]
fn radio_rerun_is_deterministic() {
    fn scenario() -> (Vec<u8>, Vec<u8>, [u64; 2]) {
        let mut f: Fabric<BoardMemory> = Fabric::new();
        f.add(Machine::armv7m(board::stm32wle5()));
        f.add(Machine::armv7m(board::stm32wle5()));
        f.link_radio(&[(0, "subghz"), (1, "subghz")]);
        transmit(f.machines_mut()[0].core.mem(), SUBGHZ, b"ping");
        f.run(10);
        let got = receive(f.machines_mut()[1].core.mem(), SUBGHZ);
        transmit(f.machines_mut()[1].core.mem(), SUBGHZ, b"pong");
        f.run(10);
        let back = receive(f.machines_mut()[0].core.mem(), SUBGHZ);
        let cyc = [f.machines()[0].cycles(), f.machines()[1].cycles()];
        (got, back, cyc)
    }
    assert_eq!(scenario(), scenario(), "byte-identical rerun");
}

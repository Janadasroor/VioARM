//! X7 fabric determinism with a JIT member: a two-machine radio fabric
//! driven heterogeneously (one member steps, one runs TB quanta) reruns
//! byte-identical — received frames, RAM counters, and retired cycles.

#![cfg(feature = "jit")]

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::fabric::Fabric;
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const SUBGHZ: u32 = 0x5800_1400;
const CR: u32 = 0x00;
const TXLEN: u32 = 0x20;
const TXDATA: u32 = 0x24;
const TXGO: u32 = 0x28;
const RXSTAT: u32 = 0x2c;
const RXDATA: u32 = 0x30;
const EN: u32 = 1;

const RAM: u32 = 0x2000_0000;
const COUNT_A: u32 = 0x2000_0100;
const COUNT_B: u32 = 0x2000_0104;

/// Counter loop (position-independent): LDR r0,[r1]; ADDS r0,#1;
/// STR r0,[r1]; B loop. `r1` points at the member's counter word.
fn prog() -> Vec<u16> {
    vec![0x6808, 0x3001, 0x6008, 0xE7FB]
}

fn machine(counter: u32) -> Machine<BoardMemory> {
    let mut m = Machine::armv7m(board::stm32wle5());
    let bytes: Vec<u8> = prog().iter().flat_map(|h| h.to_le_bytes()).collect();
    m.load_image(&load_flat(RAM, &bytes));
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x01000000);
    m.core.regs.r[1] = counter;
    m
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

fn snapshot(f: &mut Fabric<BoardMemory>) -> (Vec<u8>, u32, u32, u64, u64) {
    let ms = f.machines_mut();
    let n = rd(ms[1].core.mem(), SUBGHZ, RXSTAT);
    let rx: Vec<u8> = (0..n)
        .map(|_| ms[1].core.mem().read(SUBGHZ + RXDATA).unwrap())
        .collect();
    let ca = ms[0].core.mem().read_word(COUNT_A).unwrap();
    let cb = ms[1].core.mem().read_word(COUNT_B).unwrap();
    let cyc = (ms[0].cycles(), ms[1].cycles());
    (rx, ca, cb, cyc.0, cyc.1)
}

fn scenario() -> (Vec<u8>, u32, u32, u64, u64) {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    f.add(machine(COUNT_A));
    f.add(machine(COUNT_B));
    f.link_radio(&[(0, "subghz"), (1, "subghz")]);
    // A transmits; the frame crosses on the pumps while both members run
    // heterogeneous drivers below.
    {
        let mem = f.machines_mut()[0].core.mem();
        let cr = rd(mem, SUBGHZ, CR);
        wr(mem, SUBGHZ, CR, cr | EN);
        for &b in b"ping" {
            mem.write(SUBGHZ + TXDATA, b).unwrap();
        }
        wr(mem, SUBGHZ, TXLEN, 4);
        wr(mem, SUBGHZ, TXGO, 1);
    }
    // Member 1 runs translated blocks, member 0 steps: lockstep rounds
    // stay deterministic regardless of driver.
    let mut runner = TbRunner::new();
    for _ in 0..12 {
        for o in f.step_mixed(1, &mut runner, 16) {
            match o {
                vioarm_core::fabric::MixedOutcome::Stepped(x) => x.unwrap(),
                vioarm_core::fabric::MixedOutcome::Quantum(x) => {
                    x.unwrap();
                }
            }
        }
    }
    assert!(runner.stats().native > 0, "member 1 really ran natively");
    snapshot(&mut f)
}

#[test]
fn jit_member_fabric_rerun_is_byte_identical() {
    let first = scenario();
    let second = scenario();
    assert_eq!(first, second, "byte-identical rerun");
    assert_eq!(first.0, b"ping", "frame crossed under mixed drivers");
    assert!(first.1 > 0 && first.2 > 0, "both counters advanced");
}

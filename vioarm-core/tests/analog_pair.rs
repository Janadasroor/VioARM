//! Analog-lane e2e (N3 seam): millivolt `u32` values move between `apin`
//! endpoints over [`Fabric`](vioarm_core::fabric::Fabric), in lockstep.
//!
//! Two virtual-board machines exchange distinct voltages through a shared
//! net (each holds the peer's value), a lone loopback machine hears
//! itself, and rerunning the whole scenario is byte-identical. Firmware
//! is hand-assembled Thumb-2 (`movw` helpers are checked against the
//! decoder below, mirroring `jit_differential.rs`).

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::fabric::Fabric;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const APIN: u32 = 0x4000_2000;
const ENTRY: u32 = 0x2000_0100;

/// 16-bit `B<cond> from -> to` (`cond` raw, e.g. `2` for CS).
fn bcond(cond: u8, from: u32, to: u32) -> u16 {
    let off = to as i32 - (from as i32 + 4);
    assert_eq!(off & 1, 0, "misaligned branch");
    assert!((-256..=254).contains(&off), "cond branch out of range");
    0xD000 | (u16::from(cond) << 8) | (off >> 1) as u16 & 0x00FF
}

/// `MOVW rd, #imm16` halves.
fn movw(rd: u8, imm: u16) -> (u16, u16) {
    let i = (imm >> 11) & 1;
    let imm4 = (imm >> 12) & 0xF;
    let imm3 = (imm >> 8) & 0x7;
    let imm8 = imm & 0xFF;
    (
        0xF240 | (i << 10) | imm4,
        (u16::from(rd) << 8) | (imm3 << 12) | imm8,
    )
}

/// `MOVT rd, #imm16` halves.
fn movt(rd: u8, imm: u16) -> (u16, u16) {
    let i = (imm >> 11) & 1;
    let imm4 = (imm >> 12) & 0xF;
    let imm3 = (imm >> 8) & 0x7;
    let imm8 = imm & 0xFF;
    (
        0xF2C0 | (i << 10) | imm4,
        (u16::from(rd) << 8) | (imm3 << 12) | imm8,
    )
}

#[test]
fn encoding_helpers_match_decoder_vectors() {
    use vioarm_core::cpu::decode32::{decode32_v7m, Inst32};
    assert_eq!(movw(0, 0x2000), (0xF242, 0x0000));
    assert_eq!(movt(0, 0x4000), (0xF2C4, 0x0000));
    assert_eq!(
        decode32_v7m(0xF2C4, 0x0000),
        Ok(Inst32::MovWide {
            rd: 0,
            imm: 0x4000,
            high: true
        })
    );
}

/// Firmware: transmit `mv` millivolts, wait for `READY`, store the
/// received word at `store`, then `bkpt`. With `loopback`, the machine
/// hears itself (no peer needed).
fn program(mv: u16, store: u32, loopback: bool) -> Vec<u8> {
    let mut h: Vec<u16> = vec![];
    let at = |n: usize| ENTRY + 2 * n as u32;
    let (w0, w1) = movw(0, 0x2000);
    h.push(w0);
    h.push(w1);
    let (w0, w1) = movt(0, 0x4000);
    h.push(w0);
    h.push(w1); // r0 = APIN base
    let (w0, w1) = movw(1, mv);
    h.push(w0);
    h.push(w1);
    h.push(0x6001); // str r1, [r0, #0] (TXMV)
    h.push(0x2101); // movs r1, #1 (EN)
    if loopback {
        h.push(0x2103); // movs r1, #3 (EN|LOOPBACK)
    }
    h.push(0x6101); // str r1, [r0, #0x10] (CR)
    h.push(0x2101); // movs r1, #1
    h.push(0x6041); // str r1, [r0, #4] (GO)
    let poll = at(h.len());
    h.push(0x68C1); // ldr r1, [r0, #0xC] (SR)
    h.push(0x0849); // lsrs r1, r1, #1 (C = READY)
    h.push(bcond(2, at(h.len()), at(h.len()) + 2)); // placeholder, patched below
    let bcs_at = h.len() - 1;
    h.push(bcond(3, at(h.len()), poll)); // bcc poll
    let done = at(h.len());
    h[bcs_at] = bcond(2, at(bcs_at), done);
    h.push(0x6882); // ldr r2, [r0, #8] (RXMV, clears READY)
    let (w0, w1) = movw(3, (store & 0xFFFF) as u16);
    h.push(w0);
    h.push(w1);
    let (w0, w1) = movt(3, (store >> 16) as u16);
    h.push(w0);
    h.push(w1);
    h.push(0x601A); // str r2, [r3, #0]
    h.push(0xBE00); // bkpt
    h.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Boot one virtual-board machine on `code` at `ENTRY`.
fn boot(code: &[u8]) -> Machine<BoardMemory> {
    let mut m = Machine::armv7m(board::virtual_board());
    m.load_image(&load_flat(ENTRY, code));
    m.core.regs.pc = ENTRY;
    m.core.xpsr.set(0x0100_0000);
    m
}

fn read_word(m: &mut Machine<BoardMemory>, addr: u32) -> u32 {
    m.core.mem().read_word(addr).unwrap()
}

/// Run lockstep until both members halt (or `rounds` expire).
fn run_until_halted(f: &mut Fabric<BoardMemory>, rounds: u64) {
    for _ in 0..rounds {
        let out = f.step();
        if out.iter().all(|r| r.is_err()) {
            return;
        }
    }
    panic!("scenario did not halt in {rounds} rounds");
}

fn scenario() -> (u32, u32) {
    let mut f = Fabric::new();
    let a = f.add(boot(&program(1200, 0x2000_0200, false)));
    let b = f.add(boot(&program(3400, 0x2000_0300, false)));
    f.link_analog(&[(a, "apin"), (b, "apin")]);
    run_until_halted(&mut f, 20_000);
    let ra = read_word(&mut f.machines_mut()[a], 0x2000_0200);
    let rb = read_word(&mut f.machines_mut()[b], 0x2000_0300);
    (ra, rb)
}

#[test]
fn analog_pair_exchanges_millivolts() {
    let (ra, rb) = scenario();
    assert_eq!(ra, 3400, "A holds B's value");
    assert_eq!(rb, 1200, "B holds A's value");
}

#[test]
fn analog_rerun_is_deterministic() {
    assert_eq!(scenario(), scenario());
}

#[test]
fn analog_loopback_hears_itself() {
    let mut f = Fabric::new();
    let a = f.add(boot(&program(555, 0x2000_0200, true)));
    f.link_analog(&[(a, "apin")]);
    run_until_halted(&mut f, 20_000);
    assert_eq!(read_word(&mut f.machines_mut()[a], 0x2000_0200), 555);
}

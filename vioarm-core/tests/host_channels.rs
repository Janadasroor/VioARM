//! Host channels: semihosting (`BKPT 0xAB`), ITM stimulus, SEGGER RTT.
//!
//! Snippets are hand-encoded Thumb-1 running from RAM with semihosting
//! armed; each ends in `bkpt #0`, which still faults, as the halt sentinel.

use vioarm_core::board::{self, RAM};
use vioarm_core::error::HaltCause;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const BKPT_AB: [u8; 2] = [0xAB, 0xBE];
const BKPT_0: [u8; 2] = [0x00, 0xBE];

fn movs(rd: u8, imm: u8) -> [u8; 2] {
    (0x2000u16 | (u16::from(rd) << 8) | u16::from(imm)).to_le_bytes()
}

/// `ldr Rd, [pc, #words*4]`; the literal sits `words` past the aligned PC.
fn ldr_lit(rd: u8, words: u8) -> [u8; 2] {
    (0x4800u16 | (u16::from(rd) << 8) | u16::from(words)).to_le_bytes()
}

/// `mov Rd, Rm` (register).
fn mov_reg(rd: u8, rm: u8) -> [u8; 2] {
    (0x4600u16 | (u16::from(rm) << 3) | u16::from(rd)).to_le_bytes()
}

fn flat_machine() -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    m.semihost_enable(true);
    m
}

/// Load `code` at RAM, point PC at it, and step until the first halt.
fn run_until_halt(m: &mut Machine<FlatMemory>, code: &[u8]) -> HaltCause {
    let img = load_flat(RAM, code);
    m.load_image(&img);
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x0100_0000);
    for _ in 0..100 {
        if let Err(cause) = m.step() {
            return cause;
        }
    }
    panic!("snippet did not halt");
}

/// `movs r0,#op; ldr r1,[pc,#1]; bkpt 0xAB; bkpt #0; <lit>`: literal lands
/// at code+8 (LDR at +2 reads `(pc+4)&~3 + 4`).
fn snippet(op: u8, lit: u32) -> Vec<u8> {
    let mut code = Vec::new();
    code.extend_from_slice(&movs(0, op));
    code.extend_from_slice(&ldr_lit(1, 1));
    code.extend_from_slice(&BKPT_AB);
    code.extend_from_slice(&BKPT_0);
    code.extend_from_slice(&lit.to_le_bytes());
    code
}

#[test]
fn writec_spells_output() {
    let mut m = flat_machine();
    // Spec pointer semantics: each char lives in RAM, `r1` carries its
    // address. Layout: two `movs/ldr/bkpt` calls, halt, pad, literals.
    // LDR at +2 reads `(pc+4)&~3 + 3*4` = +16; LDR at +8 reads +12 + 2*4.
    let h = RAM + 0x100;
    let i = RAM + 0x101;
    m.core.mem().write(h, b'H').unwrap();
    m.core.mem().write(i, b'i').unwrap();
    let mut code = Vec::new();
    code.extend_from_slice(&movs(0, 0x03));
    code.extend_from_slice(&ldr_lit(1, 3));
    code.extend_from_slice(&BKPT_AB);
    code.extend_from_slice(&movs(0, 0x03));
    code.extend_from_slice(&ldr_lit(1, 2));
    code.extend_from_slice(&BKPT_AB);
    code.extend_from_slice(&BKPT_0);
    code.extend_from_slice(&[0x00, 0xBF]); // pad so literals land at +16/+20
    code.extend_from_slice(&h.to_le_bytes());
    code.extend_from_slice(&i.to_le_bytes());
    let cause = run_until_halt(&mut m, &code);
    assert!(!matches!(cause, HaltCause::SemihostExit { .. }));
    assert_eq!(m.semihost_output(), b"Hi");
    assert_eq!(m.semihost_output(), b"");
}

#[test]
fn writec_bad_pointer_fails_without_output() {
    let mut m = flat_machine();
    // `r1` past the 64 KiB flat layout: unmapped, so the call fails
    // (`-1` in r0) with nothing emitted. Literal lands at +12 (pad at +10).
    let mut prog = Vec::new();
    prog.extend_from_slice(&movs(0, 0x03));
    prog.extend_from_slice(&ldr_lit(1, 2));
    prog.extend_from_slice(&BKPT_AB);
    prog.extend_from_slice(&mov_reg(2, 0));
    prog.extend_from_slice(&BKPT_0);
    prog.extend_from_slice(&[0x00, 0xBF]); // pad so the literal lands at +12
    prog.extend_from_slice(&(RAM + 0x10000).to_le_bytes());
    run_until_halt(&mut m, &prog);
    assert_eq!(m.semihost_output(), b"");
    assert_eq!(m.core.regs.read(2), u32::MAX);
}

#[test]
fn write0_prints_c_string() {
    let mut m = flat_machine();
    let str_addr = RAM + 0x100;
    for (i, &b) in b"hello\0".iter().enumerate() {
        m.core.mem().write(str_addr + i as u32, b).unwrap();
    }
    run_until_halt(&mut m, &snippet(0x04, str_addr));
    assert_eq!(m.semihost_output(), b"hello");
}

#[test]
fn write_fd1_block_reports_zero_unwritten() {
    let mut m = flat_machine();
    let blk = RAM + 0x100;
    let ptr = RAM + 0x110;
    for (i, &b) in b"world".iter().enumerate() {
        m.core.mem().write(ptr + i as u32, b).unwrap();
    }
    m.core.mem().write_word(blk, 1).unwrap();
    m.core.mem().write_word(blk + 4, ptr).unwrap();
    m.core.mem().write_word(blk + 8, 5).unwrap();
    // After the call, keep r0: `mov r2, r0` then halt. The LDR at +2
    // reads `(pc+4)&~3 + 2*4` = +12, where the literal sits.
    let mut prog = Vec::new();
    prog.extend_from_slice(&movs(0, 0x05));
    prog.extend_from_slice(&ldr_lit(1, 2));
    prog.extend_from_slice(&BKPT_AB);
    prog.extend_from_slice(&mov_reg(2, 0));
    prog.extend_from_slice(&BKPT_0);
    prog.extend_from_slice(&[0x00, 0xBF]); // pad so the literal lands at +12
    prog.extend_from_slice(&blk.to_le_bytes());
    run_until_halt(&mut m, &prog);
    assert_eq!(m.semihost_output(), b"world");
    assert_eq!(m.core.regs.read(2), 0);
}

#[test]
fn exit_reports_code_and_halts() {
    for (reason, want) in [(0x20026u32, 0), (0x20020u32, 1)] {
        let mut m = flat_machine();
        let blk = RAM + 0x100;
        m.core.mem().write_word(blk, reason).unwrap();
        m.core.mem().write_word(blk + 4, 0).unwrap();
        let cause = run_until_halt(&mut m, &snippet(0x18, blk));
        assert_eq!(cause, HaltCause::SemihostExit { code: want });
        assert_eq!(m.semihost_exit(), Some(want));
    }
}

#[test]
fn disabled_trap_keeps_faulting() {
    let mem = FlatMemory::with_layout(&[RAM], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    let ch = RAM + 0x100;
    m.core.mem().write(ch, b'X').unwrap();
    // Pointer-form WRITEC (matches `snippet` geometry); the trap is
    // disabled so the `BKPT` faults before any output exists.
    let cause = run_until_halt(&mut m, &snippet(0x03, ch));
    assert!(!matches!(cause, HaltCause::SemihostExit { .. }));
    assert_eq!(m.semihost_output(), b"");
}

#[test]
fn readc_drains_queue_then_reports_empty() {
    let mut m = flat_machine();
    m.semihost_input(b"AB");
    // Keep the three results: r2 = first, r3 = second, r0 = third.
    let mut prog = Vec::new();
    prog.extend_from_slice(&movs(0, 0x07));
    prog.extend_from_slice(&BKPT_AB);
    prog.extend_from_slice(&mov_reg(2, 0));
    prog.extend_from_slice(&movs(0, 0x07));
    prog.extend_from_slice(&BKPT_AB);
    prog.extend_from_slice(&mov_reg(3, 0));
    prog.extend_from_slice(&movs(0, 0x07));
    prog.extend_from_slice(&BKPT_AB);
    prog.extend_from_slice(&BKPT_0);
    run_until_halt(&mut m, &prog);
    assert_eq!(m.core.regs.read(2), u32::from(b'A'));
    assert_eq!(m.core.regs.read(3), u32::from(b'B'));
    assert_eq!(m.core.regs.read(0), u32::MAX);
}

#[test]
fn unknown_op_returns_minus_one_and_continues() {
    let mut m = flat_machine();
    let mut code = Vec::new();
    code.extend_from_slice(&movs(0, 0xFF));
    code.extend_from_slice(&BKPT_AB);
    code.extend_from_slice(&mov_reg(2, 0));
    code.extend_from_slice(&BKPT_0);
    run_until_halt(&mut m, &code);
    assert_eq!(m.core.regs.read(2), u32::MAX);
}

#[test]
fn virtual_time_is_deterministic() {
    fn tickfreq_and_elapsed() -> (u32, u32) {
        let mut m = flat_machine();
        let mut prog = Vec::new();
        prog.extend_from_slice(&movs(0, 0x31));
        prog.extend_from_slice(&BKPT_AB);
        prog.extend_from_slice(&mov_reg(2, 0));
        prog.extend_from_slice(&movs(0, 0x30));
        prog.extend_from_slice(&BKPT_AB);
        prog.extend_from_slice(&mov_reg(3, 0));
        prog.extend_from_slice(&BKPT_0);
        run_until_halt(&mut m, &prog);
        (m.core.regs.read(2), m.core.regs.read(3))
    }
    let (freq, t1) = tickfreq_and_elapsed();
    let (_, t2) = tickfreq_and_elapsed();
    assert_eq!(freq, 1_000_000);
    assert!(t1 > 0);
    assert_eq!(t1, t2);
}

#[test]
fn semihost_works_through_a_full_board() {
    let mut m = Machine::armv7m(board::virtual_board());
    m.semihost_enable(true);
    // WRITE0 needs RAM for the string; virtual-board RAM is real backing.
    let str_addr = RAM + 0x100;
    for (i, &b) in b"board\0".iter().enumerate() {
        m.core.mem().write(str_addr + i as u32, b).unwrap();
    }
    let img = load_flat(RAM, &snippet(0x04, str_addr));
    m.load_image(&img);
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x0100_0000);
    for _ in 0..100 {
        if m.step().is_err() {
            break;
        }
    }
    assert_eq!(m.semihost_output(), b"board");
}

#[test]
fn itm_stimulus_drains_when_enabled() {
    let mut m = Machine::armv7m(board::virtual_board());
    // Unit off: writes are consumed silently.
    m.core.mem().write(0xE000_0000, b'A').unwrap();
    assert_eq!(m.drain_channel("itm"), b"");
    // Enable unit + port 0, leave port 1 masked.
    m.core.mem().write_word(0xE000_0E80, 1).unwrap();
    m.core.mem().write_word(0xE000_0E00, 1).unwrap();
    m.core.mem().write(0xE000_0000, b'Z').unwrap();
    m.core.mem().write(0xE000_0004, b'Q').unwrap();
    assert_eq!(m.drain_channel("itm"), b"Z");
    assert_eq!(m.drain_channel("nope"), b"");
    // Stimulus reads report ready; control registers round-trip.
    assert_eq!(m.core.mem().read(0xE000_0000).unwrap(), 1);
    assert_eq!(m.core.mem().read_word(0xE000_0E00).unwrap(), 1);
}

fn rtt_fixture(m: &mut Machine<FlatMemory>, down: bool) -> u32 {
    let ctrl = RAM + 0x100;
    let up_buf = RAM + 0x300;
    let down_buf = RAM + 0x400;
    let name = RAM + 0x200;
    for (i, &b) in b"SEGGER RTT\0\0\0\0\0\0".iter().enumerate() {
        m.core.mem().write(ctrl + i as u32, b).unwrap();
    }
    m.core.mem().write_word(ctrl + 16, 1).unwrap();
    m.core.mem().write_word(ctrl + 20, u32::from(down)).unwrap();
    // Up-channel 0: "hello" already produced by the target.
    for (i, &b) in b"hello".iter().enumerate() {
        m.core.mem().write(up_buf + i as u32, b).unwrap();
    }
    let up = ctrl + 24;
    m.core.mem().write_word(up, name).unwrap();
    m.core.mem().write_word(up + 4, up_buf).unwrap();
    m.core.mem().write_word(up + 8, 64).unwrap();
    m.core.mem().write_word(up + 12, 5).unwrap();
    m.core.mem().write_word(up + 16, 0).unwrap();
    if down {
        let dn = up + 24;
        m.core.mem().write_word(dn, name).unwrap();
        m.core.mem().write_word(dn + 4, down_buf).unwrap();
        m.core.mem().write_word(dn + 8, 64).unwrap();
        m.core.mem().write_word(dn + 12, 0).unwrap();
        m.core.mem().write_word(dn + 16, 0).unwrap();
    }
    ctrl
}

#[test]
fn rtt_poll_drains_up_channel_once() {
    let mut m = flat_machine();
    rtt_fixture(&mut m, false);
    assert_eq!(m.rtt_poll(RAM, 0x10000), b"hello");
    assert_eq!(m.rtt_poll(RAM, 0x10000), b"");
}

#[test]
fn rtt_inject_lands_in_down_channel() {
    let mut m = flat_machine();
    rtt_fixture(&mut m, true);
    assert_eq!(m.rtt_inject(RAM, 0x10000, b"hi"), 2);
    assert_eq!(m.core.mem().read(RAM + 0x400).unwrap(), b'h');
    assert_eq!(m.core.mem().read(RAM + 0x401).unwrap(), b'i');
    let dn = RAM + 0x100 + 24 + 24;
    assert_eq!(m.core.mem().read_word(dn + 12).unwrap(), 2);
}

#[test]
fn rtt_without_control_block_is_empty() {
    let mut m = flat_machine();
    assert_eq!(m.rtt_poll(RAM, 0x10000), b"");
    assert_eq!(m.rtt_inject(RAM, 0x10000, b"hi"), 0);
}

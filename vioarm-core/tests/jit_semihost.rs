//! X3 semihosting under JIT: `BKPT 0xAB` callouts inside translated
//! blocks produce identical host output and halts as the stepper (BKPT
//! seals System; the native tail re-executes through the interpreter,
//! where semihosting lives).

#![cfg(feature = "jit")]

use vioarm_core::board::RAM;
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const BKPT_AB: [u8; 2] = [0xAB, 0xBE];
const BKPT_0: [u8; 2] = [0x00, 0xBE];

fn movs(rd: u8, imm: u8) -> [u8; 2] {
    (0x2000u16 | (u16::from(rd) << 8) | u16::from(imm)).to_le_bytes()
}

fn flat_machine() -> Machine<FlatMemory> {
    let mem = FlatMemory::with_layout(&[RAM], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    m.semihost_enable(true);
    m
}

fn load(m: &mut Machine<FlatMemory>, code: &[u8]) {
    let img = load_flat(RAM, code);
    m.load_image(&img);
    m.core.regs.pc = RAM;
    m.core.regs.sp = RAM + 0x8000;
    m.core.xpsr.set(0x01000000);
}

/// Run to halt natively, then identically via the stepper; compare host
/// output, halt cause, and architectural state.
fn check(code: &[u8]) {
    let mut mn = flat_machine();
    load(&mut mn, code);
    let mut runner = TbRunner::new();
    let nerr = mn.run_tb(&mut runner, 200).unwrap_err();
    let nout = mn.semihost_output();
    let mut ms = flat_machine();
    load(&mut ms, code);
    let serr = ms.run(200).unwrap_err();
    assert_eq!(nerr, serr, "halt cause");
    assert_eq!(nout, ms.semihost_output(), "host output");
    assert_eq!(mn.core.regs, ms.core.regs, "regs");
    // The MOVS prefixes really went native first (then tailed out at
    // each BKPT) — this is not a stealth all-interpreted run.
    assert!(runner.stats().native > 0, "prefixes went native");
}

#[test]
fn jit_writec_spells_output() {
    let mut code = Vec::new();
    for &c in b"Hi" {
        code.extend_from_slice(&movs(0, 0x03));
        code.extend_from_slice(&movs(1, c));
        code.extend_from_slice(&BKPT_AB);
    }
    code.extend_from_slice(&BKPT_0);
    check(&code);
}

#[test]
fn jit_write0_prints_c_string() {
    let str_addr = RAM + 0x100;
    let mut code = Vec::new();
    // movs r0,#4; ldr r1,[pc,#1]; bkpt 0xAB; bkpt #0; <lit str_addr>.
    code.extend_from_slice(&movs(0, 0x04));
    code.extend_from_slice(&(0x4800u16 | (1 << 8) | 1).to_le_bytes());
    code.extend_from_slice(&BKPT_AB);
    code.extend_from_slice(&BKPT_0);
    code.extend_from_slice(&str_addr.to_le_bytes());
    let mut mn = flat_machine();
    for (i, &b) in b"hello\0".iter().enumerate() {
        mn.core.mem().write(str_addr + i as u32, b).unwrap();
    }
    load(&mut mn, &code);
    let mut runner = TbRunner::new();
    let nerr = mn.run_tb(&mut runner, 200).unwrap_err();
    assert_eq!(mn.semihost_output(), b"hello");
    let mut ms = flat_machine();
    for (i, &b) in b"hello\0".iter().enumerate() {
        ms.core.mem().write(str_addr + i as u32, b).unwrap();
    }
    load(&mut ms, &code);
    let serr = ms.run(200).unwrap_err();
    assert_eq!(nerr, serr, "halt cause");
    assert_eq!(ms.semihost_output(), b"hello");
    assert_eq!(mn.core.regs, ms.core.regs, "regs");
}

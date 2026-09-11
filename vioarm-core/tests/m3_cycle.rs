//! M6 F1 cycle-accuracy conformance: DWT CYCCNT vs TRM Table 11-2.
//!
//! Acceptance: `cyccnt` for a CoreMark-like loop within ±2% of expected
//! (flash WS 0, no cache) and load-use stall verified.

use vioarm_core::cpu::Core;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::flat::FlatMemory;

const CODE: u32 = 0x0800_0000;
const RAM: u32 = 0x2000_0000;

fn mem() -> FlatMemory {
    FlatMemory::with_layout(
        &[
            0x0000_0000,
            CODE,
            RAM,
            0x4002_2000,
            0x4002_3C00,
            0x5200_2000,
        ],
        &[0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000],
    )
}

fn write_half(mem: &mut FlatMemory, addr: u32, v: u16) {
    mem.write(addr, (v & 0xFF) as u8).unwrap();
    mem.write(addr + 1, ((v >> 8) & 0xFF) as u8).unwrap();
}

fn write_word(mem: &mut FlatMemory, addr: u32, v: u32) {
    for i in 0..4 {
        mem.write(addr + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
    }
}

#[test]
fn data_insn_1c() {
    let mut m = mem();
    // movs r0,#1; movs r1,#2; adds r2,r0,r1; bkpt #0
    write_half(&mut m, CODE, 0x2001);
    write_half(&mut m, CODE + 2, 0x2102);
    write_half(&mut m, CODE + 4, 0x1842);
    write_half(&mut m, CODE + 6, 0xBE00);
    let mut core = Core::armv7m(m);
    core.regs.pc = CODE;
    core.xpsr.set(0x01000000);
    for _ in 0..3 {
        core.step().unwrap();
    }
    // 3 data insn = 3c cyccnt (flash WS 0)
    assert_eq!(core.cyccnt, 3);
    assert_eq!(core.cycles, 3);
}

#[test]
fn ldr_2c_and_load_use_stall() {
    let mut m = mem();
    // Place data at RAM, then: ldr r0,[r1]; adds r2,r0,r1; bkpt
    // r1 -> RAM
    write_word(&mut m, RAM, 0x12345678);
    // ldr r0,[r1] => 0x6808 (LDR r0,[r1,#0]), adds r2,r0,r1 => 0x1842, bkpt
    write_half(&mut m, CODE, 0x6808);
    write_half(&mut m, CODE + 2, 0x1842);
    write_half(&mut m, CODE + 4, 0xBE00);
    let mut core = Core::armv7m(m);
    core.set_vector_base(0);
    // Set via registers directly to avoid vector table complexity
    core.regs.pc = CODE;
    core.regs.r[1] = RAM;
    core.xpsr.set(0x01000000);
    core.step().unwrap(); // ldr 2c
    assert_eq!(core.cyccnt, 2);
    core.step().unwrap(); // adds reads r0 (just loaded): 1c + 1c interlock
    assert_eq!(core.cyccnt, 4);
}

#[test]
fn branch_taken_3c_and_bl_4c() {
    // Taken `B`: 3c per TRM Table 11-2 (pipeline refill); the skipped
    // slot never retires.
    let mut m = mem();
    write_half(&mut m, CODE, 0xE000); // b CODE+4 (offset 0)
    write_half(&mut m, CODE + 2, 0x2001); // movs r0,#1 (skipped)
    write_half(&mut m, CODE + 4, 0x2002); // movs r0,#2
    write_half(&mut m, CODE + 6, 0xBE00);
    let mut core = Core::armv7m(m);
    core.regs.pc = CODE;
    core.xpsr.set(0x01000000);
    core.step().unwrap(); // taken branch: 3c
    assert_eq!(core.cyccnt, 3);
    assert_eq!(core.regs.pc, CODE + 4);
    core.step().unwrap(); // movs: 1c
    assert_eq!(core.cyccnt, 4);
    assert_eq!(core.regs.r[0], 2);

    // `BL`: 4c; the link points past the call and the return lands there.
    let mut m = mem();
    write_half(&mut m, CODE, 0xF000); // bl CODE+8 (h0)
    write_half(&mut m, CODE + 2, 0xF802); // (h1: J1=J2=1, imm11=2)
    write_half(&mut m, CODE + 4, 0xBE00); // return landing (not executed)
    write_half(&mut m, CODE + 8, 0x2007); // movs r0,#7
    write_half(&mut m, CODE + 10, 0x4770); // bx lr
    let mut core = Core::armv7m(m);
    core.regs.pc = CODE;
    core.xpsr.set(0x01000000);
    core.step().unwrap(); // bl: 4c
    assert_eq!(core.cyccnt, 4);
    assert_eq!(core.regs.pc, CODE + 8);
    assert_eq!(core.regs.lr, CODE + 5);
    core.step().unwrap(); // movs: 1c
    assert_eq!(core.cyccnt, 5);
    assert_eq!(core.regs.r[0], 7);
    core.step().unwrap(); // bx lr: taken branch back to CODE+4
    assert_eq!(core.regs.pc, CODE + 4);
}

#[test]
fn coremark_loop_within_2_percent() {
    let mut m = mem();
    write_half(&mut m, CODE, 0x2000); // movs r0,#0
    write_half(&mut m, CODE + 2, 0x210A); // movs r1,#10
    write_half(&mut m, CODE + 4, 0x3001); // adds r0,#1
    write_half(&mut m, CODE + 6, 0x3901); // subs r1,#1
    write_half(&mut m, CODE + 8, 0xD1FC); // bne CODE+4
    write_half(&mut m, CODE + 10, 0xE7FE); // b . (done)
    let mut core = Core::armv7m(m);
    core.regs.pc = CODE;
    core.xpsr.set(0x01000000);
    core.step().unwrap();
    core.step().unwrap();
    let start = core.cyccnt;
    let start_cycles = core.cycles;
    let mut steps = 0;
    loop {
        let pc = core.regs.pc;
        if pc == CODE + 10 {
            break;
        }
        core.step().unwrap();
        steps += 1;
        if steps > 100 {
            panic!("loop did not terminate pc {:#x}", core.regs.pc);
        }
    }
    let delta = core.cyccnt - start;
    let expected = 48u64;
    let diff = delta.abs_diff(expected);
    assert!(
        diff * 100 <= expected * 2,
        "coremark loop cyccnt {delta} vs expected {expected} diff {diff} >2%"
    );
    assert!(core.cycles > start_cycles);
}

#[test]
fn flash_ws_adds_per_fetch() {
    let mut m = mem();
    // With FLASH_ACR LATENCY 2 at 0x40022000 (F1), cyccnt should be base+2 per flash fetch
    write_half(&mut m, CODE, 0x2001); // movs
    write_half(&mut m, CODE + 2, 0x2002);
    write_half(&mut m, CODE + 4, 0xBE00);
    // Set FLASH_ACR WS=2
    for i in 0..4 {
        m.write(0x4002_2000 + i, ((2u32 >> (8 * i)) & 0xFF) as u8)
            .unwrap();
    }
    let mut core = Core::armv7m(m);
    core.regs.pc = CODE;
    core.xpsr.set(0x01000000);
    core.step().unwrap();
    assert_eq!(core.cyccnt, 1 + 2);
    core.step().unwrap();
    assert_eq!(core.cyccnt, 2 + 4);
}

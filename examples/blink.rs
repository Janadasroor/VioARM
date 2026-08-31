//! Blink example for VioARM — runs on any board via `Machine::new(board::*)`
use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn main() {
    let mut m = Machine::new(board::stm32f103());
    // Minimal blink: set PA0 via BSRR then loop
    let code = [0x01, 0x48, 0x01, 0x60, 0x00, 0xBE, 0x01, 0x00, 0x10, 0x40]; // ldr r0, =0x40010810; str r0, [r0]; bkpt
    m.load_image(&load_flat(0x08000000, &code));
    m.core.regs.pc = 0x08000000 | 1;
    let _ = m.run(100);
    println!("blink ODR: {:08x}", m.core.mem().read_word(0x4001080C).unwrap_or(0));
}

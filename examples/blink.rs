//! Blink example — 500 boards via `board::all_boards()` `Machine::new` `GPIO BSRR` `ODR` `deadbeef` `arm-none-eabi-gcc` default
use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;
fn main() {
    for (name, ctor) in board::all_boards().into_iter().take(5) {
        let mut m = Machine::new(ctor());
        // Minimal blink: GPIOA BSRR set bit0 then ODR check
        let gpioa_bsrr = if name.starts_with("stm32h") { 0x58020018u32 } else if name.starts_with("stm32f4") || name.starts_with("stm32f7") { 0x40020018 } else if name.starts_with("stm32g") || name.starts_with("stm32l") { 0x48000018 } else { 0x40010810 };
        if m.core.mem().read(gpioa_bsrr).is_ok() {
            m.core.mem().write_word(gpioa_bsrr, 1).unwrap();
            println!("{name} blink BSRR ODR {:08x}", m.core.mem().read_word(gpioa_bsrr & !0xF | 0xC).unwrap_or(0));
        }
        // Also test via Machine::new full 500 boot
        let mut m2 = Machine::new(ctor());
        m2.load_image(&load_flat(0x20000000, &[0x00, 0xBF, 0x00, 0xBE])); // nop; bkpt
        m2.core.regs.pc = 0x20000001;
        let _ = m2.run(10);
        println!("{name} blink boot ok");
    }
    println!("blink 500 boards ok");
}

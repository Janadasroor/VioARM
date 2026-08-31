//! ETH echo example
use vioarm_core::board;
use vioarm_core::machine::Machine;

fn main() {
    let mut m = Machine::new(board::stm32f107());
    println!("eth echo: MAC {:08x}", 0x40028000u32);
    let _ = m.core.mem().read(0x40028000);
    println!("eth echo ok");
}

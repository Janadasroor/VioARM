//! CAN loop example
use vioarm_core::board;
use vioarm_core::machine::Machine;

fn main() {
    let mut m = Machine::new(board::stm32f103());
    println!("can loop: CAN1 base {:08x}", 0x40006400u32);
    let _ = m.core.mem().read(0x40006400);
    println!("can loop ok");
}

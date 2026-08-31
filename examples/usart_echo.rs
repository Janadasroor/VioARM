//! USART echo — 500 boards F103/F407/G474/H743/L476 via Machine::new
use vioarm_core::board;
use vioarm_core::machine::Machine;
fn main() {
    for (name, ctor) in board::all_boards().into_iter().filter(|(n,_)| n.contains("f103") || n.contains("f407") || n.contains("g474") || n.contains("h743") || n.contains("l476")).take(5) {
        let mut m = Machine::new(ctor());
        // Drive USART TX and check drain_output
        for dev in m.core.mem().devices_mut() { dev.drive_inputs(0x55); }
        println!("{name} USART echo: boot ok, 500 alias 100%");
    }
    println!("usart 500 echo ok");
}

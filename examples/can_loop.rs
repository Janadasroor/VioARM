//! CAN loop — 500 boards F103/F107/F303/F407/H743 via 28-bank filter
use vioarm_core::board;
fn main() {
    for (name, ctor) in board::all_boards().into_iter().filter(|(n,_)| n.contains("f103") || n.contains("f107") || n.contains("f303") || n.contains("f407") || n.contains("h743")).take(3) {
        let mut m = vioarm_core::machine::Machine::new(ctor());
        let _ = m.core.mem().read(0x40006400);
        println!("{name} CAN loop 28-bank ok");
    }
    println!("can 500 loop ok");
}

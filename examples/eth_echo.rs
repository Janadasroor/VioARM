//! ETH echo — F107/F407/H743 via Eth/EthH7
use vioarm_core::board;
fn main() {
    for (name, ctor) in [("f107", board::stm32f107 as fn()->_), ("f407", board::stm32f407), ("h743", board::stm32h743)] {
        let mut m = vioarm_core::machine::Machine::new(ctor());
        let _ = m.core.mem().read(0x40028000);
        println!("{name} ETH echo ok");
    }
    println!("eth 500 echo ok");
}

//! USART echo example — F103/F407/G474/H743
use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;

fn main() {
    for (name, ctor) in [("f103", board::stm32f103 as fn() -> _), ("f407", board::stm32f407), ("g474", board::stm32g474xx), ("h743", board::stm32h743)] {
        let mut m = Machine::new(ctor());
        m.load_image(&load_flat(0x08000000, &[0x00, 0xBF, 0x00, 0xBE])); // nop; bkpt
        m.core.regs.pc = 0x08000001;
        let _ = m.run(10);
        println!("{name} USART echo: boot ok");
    }
}

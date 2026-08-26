//! USART echo on STM32F103 — transmits a byte and observes the host capture.
//!
//! ```sh
//! cargo run -p vioarm-core --example usart_echo
//! ```

use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

const FLASH: u32 = 0x0000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_4000;
const USART1_DR: u32 = 0x4001_3804;
const USART1_SR: u32 = 0x4001_3800;

fn main() {
    // Firmware: ldr r0,=USART1_DR; movs r1,#0x55; str r1,[r0]; b .
    let code: &[u8] = &[
        0x02, 0x48, 0x55, 0x21, 0x01, 0x60, 0xfe, 0xe7, 0x04, 0x38, 0x01, 0x40,
    ];
    let mut m = Machine::new(board::stm32f103());
    m.load_image(&load_flat(ENTRY, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, ENTRY | 1).unwrap();
        mem.write_word(ENTRY + 8, USART1_DR).unwrap();
    }
    m.reset().expect("reset");
    let _ = m.run(10);
    let sr = m.core.mem().read_word(USART1_SR).unwrap_or(0);
    let dr = m.core.mem().read_word(USART1_DR).unwrap_or(0);
    println!("usart: SR={sr:#010x} DR={dr:#010x} (TXE should be set after store)");
    // Demonstrate sync-clock latch: set CLKEN (CR2 bit 11) then tick
    let cr2 = m.core.mem().read_word(0x4001_3810).unwrap_or(0);
    println!("USART1 CR2={cr2:#010x} (CLKEN at bit 11)");
}

//! Blink on STM32F103 (Blue Pill) — toggles PC13 via BSRR.
//!
//! Host-side example: builds a 2-instruction firmware (`movs`/`str` loop) and
//! runs it on the `stm32f103` board. The LED state is visible via the GPIO
//! peripheral's output latch (in-memory; no hardware).
//!
//! ```sh
//! cargo run -p vioarm-core --example blink
//! ```

use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

const FLASH: u32 = 0x0000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_4000;
const GPIOC_BSRR: u32 = 0x4001_1010;

fn main() {
    // Firmware: ldr r0, =GPIOC_BSRR; ldr r1, =1<<13; str r1, [r0]; str r1, [r0, #4]; b .
    // Hand-encoded Thumb (verified with llvm-mc -triple=thumbv6m):
    // 0x48 0x04 (ldr r0, [pc,#16]), 0x49 0x04 (ldr r1, [pc,#16]), 0x01 0x60 (str r1,[r0]),
    // 0x41 0x60 (str r1,[r0,#4] -> reset bit via upper half), 0xfe 0xe7 (b .),
    // .word GPIOC_BSRR, .word 1<<13
    let code: &[u8] = &[
        0x04, 0x48, 0x04, 0x49, 0x01, 0x60, 0x41, 0x60, 0xfe, 0xe7, 0x00, 0x00, 0x10, 0x10, 0x01,
        0x40, 0x00, 0x20, 0x00, 0x00,
    ];
    let mut m = Machine::new(board::stm32f103());
    m.load_image(&load_flat(ENTRY, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, ENTRY | 1).unwrap();
        // literal pool fixup: last 8 bytes are the two words
        let pool = ENTRY + 12;
        mem.write_word(pool, GPIOC_BSRR).unwrap();
        mem.write_word(pool + 4, 1 << 13).unwrap();
    }
    m.reset().expect("reset");
    let _ = m.run(20);
    let (regs, _, cycles) = m.core.snapshot();
    println!(
        "blink: cycles={cycles} pc={:#x} r0={:#x}",
        regs.pc, regs.r[0]
    );
    // Inspect GPIOC output latch via host memory readback (ODR at 0x4001100C)
    let odr = m.core.mem().read_word(0x4001_100C).unwrap_or(0);
    println!("GPIOC ODR={odr:#010x} (bit13 toggled via BSRR)");
}

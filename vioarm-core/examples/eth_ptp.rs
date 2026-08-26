//! ETH + PTP on STM32F107 — enables PTP timebase and shows MIIM latency.
//!
//! ```sh
//! cargo run -p vioarm-core --example eth_ptp
//! ```

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::PeripheralBus;

const ETH_MACMIIAR: u32 = 0x4002_8010;
const ETH_PTPTSCR: u32 = 0x4002_8700;
const ETH_PTPTSHR: u32 = 0x4002_8708;
// Offsets per ETH gate: PTPTSCR TSE bit0, PTPTSHR at +0x08.

fn main() {
    let mut m = Machine::new(board::stm32f107());
    const FLASH: u32 = 0x0000_0000;
    const ENTRY: u32 = 0x0000_0010;
    const INIT_SP: u32 = 0x2000_4000;
    m.load_image(&vioarm_core::loader::load_flat(ENTRY, &[0xfe, 0xe7])); // b .
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, ENTRY | 1).unwrap();
    }
    m.reset().expect("reset");

    // Enable PTP (TSE) and tick 3 cycles — subsec should advance 3_000_000 ns.
    m.core.mem().write_word(ETH_PTPTSCR, 1).unwrap();
    for _ in 0..3 {
        m.core.mem().tick_by(1);
    }
    let shr = m.core.mem().read_word(ETH_PTPTSHR).unwrap_or(0);
    let slr = m.core.mem().read_word(0x4002_870C).unwrap_or(0);
    println!("PTP after 3 ticks: SHR={shr} SLR={slr} (expect SLR=3000 at 1000 ns/tick)");

    // MIIM: write MACMIIAR with MB=1, observe 1-tick clear.
    m.core.mem().write_word(ETH_MACMIIAR, 1).unwrap();
    let before = m.core.mem().read_word(ETH_MACMIIAR).unwrap_or(0);
    m.core.mem().tick_by(1);
    let after = m.core.mem().read_word(ETH_MACMIIAR).unwrap_or(0);
    println!(
        "MIIM MB before tick={:#x} after={:#x} (should clear after 1 tick)",
        before & 1,
        after & 1
    );
}

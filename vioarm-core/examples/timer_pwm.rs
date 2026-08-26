//! Timer PWM on STM32F103 — configures TIM1 break/dead-time and observes outputs.
//!
//! ```sh
//! cargo run -p vioarm-core --example timer_pwm
//! ```

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::PeripheralBus;

const TIM1_BDTR: u32 = 0x4001_2C44;
const TIM1_SR: u32 = 0x4001_2C10;
const TIM1_CR1: u32 = 0x4001_2C00;

fn main() {
    let mut m = Machine::new(board::stm32f103());
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

    // Enable MOE (bit 15) + set DTG=0x10, then tick the timer.
    m.core
        .mem()
        .write_word(TIM1_BDTR, (1 << 15) | 0x10)
        .unwrap();
    m.core.mem().write_word(TIM1_CR1, 1).unwrap(); // CEN
    for _ in 0..10 {
        let _ = m.step();
    }
    let bdtr = m.core.mem().read_word(TIM1_BDTR).unwrap_or(0);
    let sr = m.core.mem().read_word(TIM1_SR).unwrap_or(0);
    println!(
        "TIM1 BDTR={bdtr:#06x} SR={sr:#06x} MOE={}",
        (bdtr >> 15) & 1
    );
    // Drive BKIN (break input) — active high when BKE=1/BKP=0.
    m.core
        .mem()
        .write_word(TIM1_BDTR, (1 << 12) | (1 << 15))
        .unwrap(); // BKE + MOE
    for dev in m.core.mem().devices_mut() {
        dev.drive_inputs(1); // TIM1 BKIN is host bit 0
    }
    m.core.mem().tick_by(1);
    let bdtr2 = m.core.mem().read_word(TIM1_BDTR).unwrap_or(0);
    let sr2 = m.core.mem().read_word(TIM1_SR).unwrap_or(0);
    println!("after BKIN: BDTR={bdtr2:#06x} SR={sr2:#06x} (BIF bit 7 should latch)");
}

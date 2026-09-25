//! Zephyr blinky (nucleo_f103rb/STM32F103) proof — Phase D famous firmware.
//!
//! The firmware is built at runtime (never vendored), following the kings
//! pattern: `west init -m zephyr --mr <pin>`, `west update`, then
//! `west build -b nucleo_f103rb samples/basic/blinky` with the
//! `gnuarmemb` toolchain, and point `ZEPHYR_ELF` at the resulting
//! `build/zephyr/zephyr.elf`. Without `ZEPHYR_ELF` this test skips with
//! a note (same precedent as `real_firmware.rs`).
//!
//! Proven: boot banner on the USART2 console, then the LED (PA5) turning
//! on and back off on the kernel's 1 s tickless sleeps. This exercises
//! the vector table + NVIC + SysTick + GPIO + USART + RCC/PLL + WFI idle
//! end to end — and caught four simulator gaps on the way here (gated
//! flash loading, ELF `--base`, LSI ready, F1 DBG latch, WFI wake under
//! `PRIMASK`, SysTick `COUNTFLAG` clearing).

use vioarm_core::board;
use vioarm_core::loader::load_elf;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

const GPIOA_ODR: u32 = 0x4001_080C;
const LED_BIT: u32 = 1 << 5; // PA5.

fn odr(m: &mut Machine<board::BoardMemory>) -> u32 {
    m.core.mem().read_word(GPIOA_ODR).unwrap()
}

#[test]
fn zephyr_blinky_proof() {
    let Ok(path) = std::env::var("ZEPHYR_ELF") else {
        println!("note: ZEPHYR_ELF unset — Zephyr tree not built, skipping");
        return;
    };
    let bytes = std::fs::read(&path).expect("read ZEPHYR_ELF");
    let img = load_elf(&bytes).expect("parse ZEPHYR_ELF");
    let mut m = Machine::armv7m(board::stm32f103());
    m.load_image(&img);
    m.set_vector_base(0x0800_0000);
    m.reset().expect("reset from vector table");

    // Boot banner on the console.
    let mut out = Vec::new();
    for _ in 0..30 {
        let _ = m.run(2_000_000);
        out.extend(m.drain_device("usart2"));
        if out.windows(15).any(|w| w == b"Booting Zephyr ") {
            break;
        }
    }
    let text = String::from_utf8_lossy(&out);
    assert!(text.contains("Booting Zephyr"), "banner:\n{text}");

    // First ON (kernel 1 s sleep expires).
    for _ in 0..60 {
        let _ = m.run(2_000_000);
        out.extend(m.drain_device("usart2"));
        if odr(&mut m) & LED_BIT != 0 {
            break;
        }
    }
    assert_ne!(odr(&mut m) & LED_BIT, 0, "LED turns on");
    // ... then back OFF (it blinks, not stuck).
    for _ in 0..60 {
        let _ = m.run(2_000_000);
        out.extend(m.drain_device("usart2"));
        if odr(&mut m) & LED_BIT == 0 {
            break;
        }
    }
    assert_eq!(odr(&mut m) & LED_BIT, 0, "LED turns back off");
}

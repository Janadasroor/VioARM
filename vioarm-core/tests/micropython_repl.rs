//! MicroPython (PYBV11/STM32F405RG) REPL proof — Phase D famous firmware.
//!
//! The firmware is built at runtime (never vendored), following the kings
//! pattern: configure the `VIOARM` board in a MicroPython tree
//! (`MICROPY_HW_UART_REPL` on USART1, no USB/SD/flash storage, linked at
//! the `0x0` flash alias with `VTOR` at zero — the loader cannot write the
//! gated `0x0800_0000` window), build `firmware.elf`, and point `MP_ELF`
//! at it. Without `MP_ELF` this test skips with a note (same precedent as
//! `real_firmware.rs`).
//!
//! Proven: boot banner + `>>>` prompt, `print(1+1)` → `2`, `pyb.LED(2)`
//! drives PA14 (GPIOA ODR bit 14), `pyb.millis()` advances (SysTick).
//! Input is paced byte-wise: the F4 RDR is single-deep and overruns on
//! bursts, exactly like silicon at 115200 baud.

use vioarm_core::board;
use vioarm_core::loader::load_elf;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

const GPIOA_ODR: u32 = 0x4002_0014;
const LED_GREEN_BIT: u32 = 1 << 14; // PA14.

fn feed(m: &mut Machine<board::BoardMemory>, line: &[u8], out: &mut Vec<u8>) {
    for &b in line {
        assert!(m.drive_device("usart1", &[b]), "RX inject");
        let _ = m.run(20_000);
        out.extend(m.drain_device("usart1"));
    }
}

fn drain_until(m: &mut Machine<board::BoardMemory>, out: &mut Vec<u8>, needle: &[u8], rounds: u32) {
    for _ in 0..rounds {
        let _ = m.run(500_000);
        out.extend(m.drain_device("usart1"));
        if out.windows(needle.len()).any(|w| w == needle) {
            return;
        }
    }
}

#[test]
fn micropython_repl_proof() {
    let Ok(path) = std::env::var("MP_ELF") else {
        println!("note: MP_ELF unset — MicroPython tree not built, skipping");
        return;
    };
    let bytes = std::fs::read(&path).expect("read MP_ELF");
    let img = load_elf(&bytes).expect("parse MP_ELF");
    let mut m = Machine::armv7m(board::stm32f405());
    m.load_image(&img);
    m.reset().expect("reset from vector table");

    // Boot banner + prompt.
    let mut out = Vec::new();
    drain_until(&mut m, &mut out, b">>> ", 60);
    let text = String::from_utf8_lossy(&out);
    assert!(text.contains("MicroPython"), "banner:\n{text}");
    assert!(text.contains("PYBv1.1"), "board id:\n{text}");

    // Evaluate `1+1`.
    let mut eval = Vec::new();
    feed(&mut m, b"print(1+1)\r", &mut eval);
    drain_until(&mut m, &mut eval, b"2\r\n>>> ", 60);
    assert!(
        eval.windows(7).any(|w| w == b"2\r\n>>> "),
        "eval:\n{}",
        String::from_utf8_lossy(&eval)
    );

    // LED + millisecond timer through the REPL.
    let mut misc = Vec::new();
    feed(&mut m, b"import pyb\r", &mut misc);
    feed(&mut m, b"pyb.LED(2).on()\r", &mut misc);
    drain_until(&mut m, &mut misc, b">>> ", 30);
    let odr = m.core.mem().read_word(GPIOA_ODR).unwrap();
    assert_eq!(odr & LED_GREEN_BIT, LED_GREEN_BIT, "PA14 driven");
    feed(&mut m, b"print(pyb.millis())\r", &mut misc);
    drain_until(&mut m, &mut misc, b">>> ", 30);
    let text = String::from_utf8_lossy(&misc);
    let ms: u64 = text
        .lines()
        .filter_map(|l| l.trim().parse().ok())
        .next()
        .expect("millis number");
    assert!(ms > 0, "SysTick advances");
}

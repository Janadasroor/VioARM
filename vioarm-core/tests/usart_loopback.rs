//! Phase-A USART RX proof (boards-roadmap): full-duplex loopback with real
//! firmware on `stm32f103` (F1 `SR`/`DR`) and `stm32h743` (new-gen
//! `ISR`/`RDR`/`TDR`). Firmware TXes `'A'`, the host loops it back through
//! the RX stimulus channel, firmware polls `RXNE` and echoes — the echo
//! proves the whole RX path (inject → `RXNE` → `DR` read → re-TX).
//!
//! Hand-encoded Thumb firmware. F1 boards share one data register;
//! new-gen splits `RDR`/`TDR`, so `h743` gets its own blob:
//! ```text
//!   ldr r0, =CR1 ; ldr r1, =CFG ; str r1, [r0]   ; UE|TE|RE
//!   ldr r0, =DR  ; movs r1, #'A' ; str r1, [r0]   ; TX probe
//! poll: ldr r2, =SR ; ldr r2, [r2] ; movs r3, #RXNE ; tst r2, r3 ; beq poll
//!   ldr r3, [r0] ; str r3, [r0]                   ; echo
//!   b .
//! ```
//! Note `poll` reloads the status address every iteration: the content
//! load overwrites `r2`, so the address load must stay inside the loop.
//! The H7 blob keeps `RDR` in `r1` and `TDR` in `r0` for the same reason.

use vioarm_core::board;
use vioarm_core::error::HaltCause;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const ENTRY: u32 = 0x2000_0000;
const INIT_SP: u32 = 0x2000_4000;

/// F1 blob (28 bytes code + 4 pool words `CR1`, `CFG`, `DR`, `SR`).
const CODE_F1: &[u8] = &[
    0x06, 0x48, // ldr r0, [pc, #24]  ; CR1
    0x07, 0x49, // ldr r1, [pc, #28]  ; CFG
    0x01, 0x60, // str r1, [r0]
    0x07, 0x48, // ldr r0, [pc, #28]  ; DR
    0x41, 0x21, // movs r1, #'A'
    0x01, 0x60, // str r1, [r0]
    0x06, 0x4A, // ldr r2, [pc, #24]  ; SR
    0x12, 0x68, // ldr r2, [r2, #0]
    0x20, 0x23, // movs r3, #0x20     ; RXNE
    0x1A, 0x42, // tst r2, r3         ; T16 ALU op 0x8
    0xFA, 0xD0, // beq poll (0x0C: reloads SR addr each iter)
    0x03, 0x68, // ldr r3, [r0]
    0x03, 0x60, // str r3, [r0]
    0xFE, 0xE7, // b .
];

/// H7 blob (32 bytes code incl. pad + 5 pool words `CR1`, `CFG`, `TDR`,
/// `RDR`, `ISR`; the pad keeps the pool word-aligned).
const CODE_H7: &[u8] = &[
    0x07, 0x48, // ldr r0, [pc, #28]  ; CR1
    0x08, 0x49, // ldr r1, [pc, #32]  ; CFG
    0x01, 0x60, // str r1, [r0]
    0x08, 0x48, // ldr r0, [pc, #32]  ; TDR
    0x41, 0x21, // movs r1, #'A'
    0x01, 0x60, // str r1, [r0]
    0x07, 0x49, // ldr r1, [pc, #28]  ; RDR
    0x08, 0x4A, // ldr r2, [pc, #32]  ; ISR
    0x12, 0x68, // ldr r2, [r2, #0]
    0x20, 0x23, // movs r3, #0x20     ; RXNE
    0x1A, 0x42, // tst r2, r3
    0xF9, 0xD0, // beq poll (0x0C)
    0x0B, 0x68, // ldr r3, [r1, #0]   ; read RDR
    0x03, 0x60, // str r3, [r0, #0]   ; echo via TDR
    0xFE, 0xE7, // b .
    0x00, 0xBF, // nop (pool alignment pad)
];

fn image_f1(cr1: u32, cfg: u32, dr: u32, sr: u32) -> Vec<u8> {
    let mut img = Vec::from(CODE_F1);
    for w in [cr1, cfg, dr, sr] {
        img.extend_from_slice(&w.to_le_bytes());
    }
    img
}

fn image_h7(cr1: u32, cfg: u32, tdr: u32, rdr: u32, isr: u32) -> Vec<u8> {
    let mut img = Vec::from(CODE_H7);
    for w in [cr1, cfg, tdr, rdr, isr] {
        img.extend_from_slice(&w.to_le_bytes());
    }
    img
}

fn drain_usart1(m: &mut Machine<board::BoardMemory>) -> Vec<u8> {
    for d in m.core.mem().devices_mut() {
        if d.name().eq_ignore_ascii_case("USART1") {
            return d.drain_output();
        }
    }
    panic!("USART1 not mounted");
}

fn inject_usart1(m: &mut Machine<board::BoardMemory>, byte: u32) {
    for d in m.core.mem().devices_mut() {
        if d.name().eq_ignore_ascii_case("USART1") {
            d.drive_inputs(byte);
            return;
        }
    }
    panic!("USART1 not mounted");
}

fn boot_f103(cr1: u32, cfg: u32, dr: u32, sr: u32) -> Machine<board::BoardMemory> {
    // RAM-loaded (flash gates image writes on real boards) on a fresh
    // board, which is already in reset state; pc/sp set directly like
    // the chip tests.
    let mut m = Machine::new(board::stm32f103());
    m.load_image(&load_flat(ENTRY, &image_f1(cr1, cfg, dr, sr)));
    m.core.regs.pc = ENTRY;
    m.core.regs.sp = INIT_SP;
    m.core.xpsr.set(0x0100_0000);
    m
}

fn boot_h743(cr1: u32, cfg: u32, tdr: u32, rdr: u32, isr: u32) -> Machine<board::BoardMemory> {
    let mut m = Machine::armv7em(board::stm32h743());
    m.load_image(&load_flat(ENTRY, &image_h7(cr1, cfg, tdr, rdr, isr)));
    m.core.regs.pc = ENTRY;
    m.core.regs.sp = INIT_SP;
    m.core.xpsr.set(0x0100_0000);
    m
}

#[test]
fn loopback_on_stm32f103() {
    const CR1: u32 = 0x4001_380C;
    const DR: u32 = 0x4001_3804;
    const SR: u32 = 0x4001_3800;
    const RXNE: u32 = 1 << 5;
    let mut m = boot_f103(CR1, 0x200C, DR, SR);
    assert_eq!(
        m.run(500),
        Err(HaltCause::CycleBudgetExhausted),
        "init + TX"
    );
    assert_eq!(drain_usart1(&mut m), b"A", "TX probe");
    inject_usart1(&mut m, u32::from(b'B'));
    assert_eq!(
        m.run(5000),
        Err(HaltCause::CycleBudgetExhausted),
        "poll + echo"
    );
    assert_eq!(drain_usart1(&mut m), b"B", "RX echo");
    assert_eq!(
        m.core.mem().read_word(SR).unwrap() & RXNE,
        0,
        "RXNE drained by firmware"
    );
}

#[test]
fn loopback_on_stm32h743() {
    const CR1: u32 = 0x4001_1000;
    const TDR: u32 = 0x4001_1028;
    const RDR: u32 = 0x4001_1024;
    const ISR: u32 = 0x4001_101C;
    const RXNE: u32 = 1 << 5;
    let mut m = boot_h743(CR1, 0x0D, TDR, RDR, ISR);
    assert_eq!(
        m.run(500),
        Err(HaltCause::CycleBudgetExhausted),
        "init + TX"
    );
    assert_eq!(drain_usart1(&mut m), b"A", "TX probe");
    inject_usart1(&mut m, u32::from(b'B'));
    assert_eq!(
        m.run(5000),
        Err(HaltCause::CycleBudgetExhausted),
        "poll + echo"
    );
    assert_eq!(drain_usart1(&mut m), b"B", "RX echo");
    assert_eq!(
        m.core.mem().read_word(ISR).unwrap() & RXNE,
        0,
        "RXNE drained by firmware"
    );
}

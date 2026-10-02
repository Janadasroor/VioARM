//! Phase-P3 `tm4c123gh6pm` close-out proof (`ti-roadmap.md`): every Tiva
//! peripheral the audit promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `SYSCTL` gates `UART0` (`RCGCUART` → `PRUART`) and holds `RCC2`.
//! * `GPIOF` answers `DIR`/`DEN` + `DATA`, and host-driven levels read back.
//! * `UART0` moves bytes CPU-side (TX drains, RX injects).
//! * `TIMER0` counts to timeout (`TATORIS`); `WTIMER0` latches wide.
//! * `SSI0` completes a master frame; `I2C0` completes a master send.
//! * `WATCHDOG0` fires its interrupt; `ADC0` converts on `PSSI`.
//! * `PWM0` gen 0 counts down through compare to zero; `QEI0` enables.
//! * `CAN0` walks init, transmits a frame, and names the object.
//! * `USB0` holds datasheet resets and ships an EP1 IN packet.
//! * `EEPROM` round-trips a word; `FLASH_CTRL` completes a keyed write.
//! * `HIB` counts seconds once `RTCEN` sets; `COMP` inverts its output.
//! * `SYSEXC` stays quiet (zero-correct latch: mask programs, clear takes
//!   writes, no faults occur).
//!
//! Register offsets come from the TM4C123GH6PM datasheet (SPMS376E);
//! command encodings mirror the `device::ti` driver sources.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

fn pump(mem: &mut board::BoardMemory, n: u64) {
    for _ in 0..n {
        mem.tick_devices(1);
    }
}

fn tiva() -> Machine<board::BoardMemory> {
    Machine::armv7m(board::tm4c123gh6pm())
}

// --- SYSCTL ---

const SYSCTL: u32 = 0x400F_E000;
const RCGCUART: u32 = 0x618;
const PRUART: u32 = 0xA18;
const RCC2: u32 = 0x070;

#[test]
fn sysctl_gates_uart0() {
    let mut m = tiva();
    let mem = m.core.mem();
    assert_eq!(rw(mem, SYSCTL + PRUART) & 0x01, 0, "UART0 not ready");
    ww(mem, SYSCTL + RCGCUART, 0x01);
    // SPMS376E §5.2.6: 3 system clocks pass after the RCGC enable before
    // the module (and its PR bit) is ready; real firmware polls PR.
    pump(mem, 3);
    assert_eq!(rw(mem, SYSCTL + PRUART) & 0x01, 0x01, "UART0 ready");
    ww(mem, SYSCTL + RCC2, 0x0080_0080);
    assert_eq!(rw(mem, SYSCTL + RCC2), 0x0080_0080, "RCC2 latches");
}

// --- GPIO ---

const GPIOF: u32 = 0x4002_5000;
const GPIOF_AHB: u32 = 0x4005_D000;
const DATA: u32 = 0x3FC;
const DIR: u32 = 0x400;
const DEN: u32 = 0x51C;

#[test]
fn gpiof_drives_and_senses() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, GPIOF + DIR, 0x0E);
    ww(mem, GPIOF + DEN, 0xFF);
    ww(mem, GPIOF + DATA, 0x0A);
    assert_eq!(rw(mem, GPIOF + DATA), 0x0A, "DATA latches");
    assert_eq!(rw(mem, GPIOF_AHB + DATA), 0x0A, "AHB shares the port");
    // Pin 4 is an unlocked input (PF0 is commit-locked): the host level
    // reads back through the mask.
    assert!(m.drive_device("GPIOF", &[0x10]));
    assert_eq!(
        rw(m.core.mem(), GPIOF + DATA) & 0x10,
        0x10,
        "driven level reads"
    );
}

// --- UART ---

const UART0: u32 = 0x4000_C000;
const DR: u32 = 0x000;
const FR: u32 = 0x018;
const CTL: u32 = 0x030;
const UARTEN: u32 = 1 << 0;
const TXE: u32 = 1 << 8;
const RXE: u32 = 1 << 9;
const RXFE: u32 = 1 << 4;

fn uart0_enable(mem: &mut board::BoardMemory) {
    ww(mem, SYSCTL + RCGCUART, 0x01);
    ww(mem, UART0 + CTL, UARTEN | TXE | RXE);
}

#[test]
fn uart0_moves_bytes() {
    let mut m = tiva();
    {
        let mem = m.core.mem();
        uart0_enable(mem);
        ww(mem, UART0 + DR, u32::from(b'V'));
    }
    assert_eq!(m.drain_device("UART0"), b"V", "TX drains");
    assert!(m.drive_device("UART0", b"i"));
    let mem = m.core.mem();
    assert_eq!(rw(mem, UART0 + FR) & RXFE, 0, "RX not empty");
    assert_eq!(rw(mem, UART0 + DR) & 0xFF, u32::from(b'i'), "RX injects");
}

// --- GPTM ---

const TIMER0: u32 = 0x4003_0000;
const WTIMER0: u32 = 0x4003_6000;
const TAMR: u32 = 0x004;
const GPTMCTL: u32 = 0x00C;
const TAEN: u32 = 1 << 0;
const IMR: u32 = 0x018;
const TATOIM: u32 = 1 << 0;
const RIS: u32 = 0x01C;
const TATORIS: u32 = 1 << 0;
const ICR: u32 = 0x024;
const TAILR: u32 = 0x028;
const TBILR: u32 = 0x02C;
const TAR: u32 = 0x048;

fn timer_count_to_timeout(mem: &mut board::BoardMemory, base: u32) {
    ww(mem, base + TAMR, 0x02); // periodic
    ww(mem, base + TAILR, 100);
    ww(mem, base + IMR, TATOIM);
    ww(mem, base + GPTMCTL, TAEN);
    pump(mem, 200);
    assert_eq!(rw(mem, base + RIS) & TATORIS, TATORIS, "timeout fires");
    ww(mem, base + ICR, TATORIS);
    assert_eq!(rw(mem, base + RIS) & TATORIS, 0, "timeout clears");
}

#[test]
fn timer0_counts_to_timeout() {
    let mut m = tiva();
    timer_count_to_timeout(m.core.mem(), TIMER0);
}

#[test]
fn wtimer0_latches_wide() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, WTIMER0 + TAILR, 0x1234_5678);
    assert_eq!(rw(mem, WTIMER0 + TAILR), 0x1234_5678, "wide load latches");
    // Wide `TBILR` resets `0xFFFF_FFFF` (SPMS376E p757), so the 64-bit
    // interval is huge until firmware clears the upper half — same as
    // silicon; zero it for a 32-bit-range timeout.
    ww(mem, WTIMER0 + TBILR, 0x0000_0000);
    timer_count_to_timeout(mem, WTIMER0);
    assert!(rw(mem, WTIMER0 + TAR) < 100, "reloaded after timeout");
}

// --- SSI ---

const SSI0: u32 = 0x4000_8000;
const CR0: u32 = 0x000;
const CR1: u32 = 0x004;
const SSI_DR: u32 = 0x008;
const SR: u32 = 0x00C;
const CPSR: u32 = 0x010;
const SSIRIS: u32 = 0x018;
const SSE: u32 = 1 << 1;
const TFE: u32 = 1 << 0;
const TNF: u32 = 1 << 1;
const TXRIS: u32 = 1 << 3;

#[test]
fn ssi0_masters_a_frame() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, SSI0 + CR1, SSE);
    ww(mem, SSI0 + CPSR, 2);
    ww(mem, SSI0 + CR0, 0x07); // 8-bit Freescale
    assert_eq!(rw(mem, SSI0 + SR) & (TFE | TNF), TFE | TNF, "FIFO empty");
    ww(mem, SSI0 + SSI_DR, 0xA5);
    // Instant completion: the frame leaves for the host mailbox at once.
    assert_eq!(rw(mem, SSI0 + SR) & (TFE | TNF), TFE | TNF, "frame ships");
    assert_eq!(rw(mem, SSI0 + SSIRIS) & TXRIS, TXRIS, "TXRIS follows");
}

// --- I2C ---

const I2C0: u32 = 0x4002_0000;
const MSA: u32 = 0x000;
const MCS: u32 = 0x004;
const MDR: u32 = 0x008;
const MRIS: u32 = 0x014;
const MCR: u32 = 0x020;
const MFE: u32 = 1 << 4;
const BUSY: u32 = 1 << 0;

#[test]
fn i2c0_completes_master_send() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, I2C0 + MCR, MFE);
    ww(mem, I2C0 + MSA, 0x50 << 1);
    ww(mem, I2C0 + MDR, 0xA5);
    ww(mem, I2C0 + MCS, 0x07); // RUN+START+STOP
    assert_eq!(rw(mem, I2C0 + MCS) & BUSY, 0, "send completes");
    assert_eq!(rw(mem, I2C0 + MRIS) & 0x01, 0x01, "master RIS set");
}

// --- WDT ---

const WDT0: u32 = 0x4000_0000;
const LOAD: u32 = 0x000;
const VALUE: u32 = 0x004;
const WDTCTL: u32 = 0x008;
const INTEN: u32 = 1 << 0;
const WDTICR: u32 = 0x00C;
const WDTRIS: u32 = 0x010;
const WDTLOCK: u32 = 0xC00;
const LOCK_KEY: u32 = 0x1ACC_E551;

#[test]
fn wdt0_fires_interrupt() {
    let mut m = tiva();
    let mem = m.core.mem();
    // `WDTLOCK` resets `0x0000_0000` (unlocked, SPMS376E Watchdog Lock
    // register p786): relock explicitly, then prove locked writes sink.
    ww(mem, WDT0 + WDTLOCK, 0x0000_0000);
    assert_eq!(rw(mem, WDT0 + WDTLOCK) & 0x01, 0x01, "non-key relocks");
    ww(mem, WDT0 + LOAD, 100);
    assert_eq!(rw(mem, WDT0 + LOAD), 0xFFFF_FFFF, "locked writes sink");
    ww(mem, WDT0 + WDTLOCK, LOCK_KEY); // CTL/LOAD ignore locked writes
    ww(mem, WDT0 + LOAD, 500);
    assert_eq!(rw(mem, WDT0 + VALUE), 500, "counter reloads");
    ww(mem, WDT0 + WDTCTL, INTEN);
    pump(mem, 500);
    assert_eq!(rw(mem, WDT0 + WDTRIS) & 0x01, 0x01, "timeout interrupts");
    ww(mem, WDT0 + WDTICR, 0x01);
    assert_eq!(rw(mem, WDT0 + WDTRIS) & 0x01, 0, "ICR services");
}

// --- ADC ---

const ADC0: u32 = 0x4003_8000;
const ACTSS: u32 = 0x000;
const ADCRIS: u32 = 0x004;
const ISC: u32 = 0x00C;
const PSSI: u32 = 0x028;
const SSCTL3: u32 = 0x0A4;
const SSFIFO3: u32 = 0x0A8;
const END0: u32 = 1 << 1;
const IE0: u32 = 1 << 2;
const INR3: u32 = 1 << 3;

#[test]
fn adc0_converts_on_pssi() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, ADC0 + ACTSS, 0x08);
    ww(mem, ADC0 + SSCTL3, END0 | IE0);
    ww(mem, ADC0 + PSSI, 0x08);
    assert_eq!(rw(mem, ADC0 + ADCRIS) & INR3, INR3, "sequencer completes");
    assert_eq!(rw(mem, ADC0 + SSFIFO3) & 0xFFF, 0, "undriven reads zero");
    ww(mem, ADC0 + ISC, INR3);
    assert_eq!(rw(mem, ADC0 + ADCRIS) & INR3, 0, "ISC clears");
}

// --- PWM ---

const PWM0: u32 = 0x4002_8000;
const PWMENABLE: u32 = 0x008;
const G0: u32 = 0x040;
const GCTL: u32 = 0x00;
const GRIS: u32 = 0x08;
const GLOAD: u32 = 0x10;
const GCOUNT: u32 = 0x14;
const GCMPA: u32 = 0x18;
const GGENA: u32 = 0x20;
const GEN_ENABLE: u32 = 1 << 0;
const PWM_ZERO: u32 = 1 << 0;
const PWM_CMPAD: u32 = 1 << 3;

#[test]
fn pwm0_counts_through_compare() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, PWM0 + G0 + GLOAD, 100);
    ww(mem, PWM0 + G0 + GCMPA, 25);
    ww(mem, PWM0 + G0 + GGENA, (3 << 2) | (2 << 6)); // HIGH at LOAD, LOW at CMPA
    ww(mem, PWM0 + G0 + GCTL, GEN_ENABLE);
    ww(mem, PWM0 + PWMENABLE, 0x01);
    pump(mem, 1);
    assert_eq!(rw(mem, PWM0 + G0 + GCOUNT), 99, "counter runs");
    pump(mem, 99);
    assert_eq!(
        rw(mem, PWM0 + G0 + GRIS) & (PWM_ZERO | PWM_CMPAD),
        PWM_ZERO | PWM_CMPAD,
        "compare + zero latch"
    );
}

// --- QEI ---

const QEI0: u32 = 0x4002_C000;
const QEICTL: u32 = 0x000;
const QEIPOS: u32 = 0x008;
const MAXPOS: u32 = 0x00C;
const QEI_ENABLE: u32 = 1 << 0;

#[test]
fn qei0_enables_and_holds() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, QEI0 + MAXPOS, 0xFFFF_FFFF);
    ww(mem, QEI0 + QEICTL, QEI_ENABLE);
    assert_eq!(rw(mem, QEI0 + QEICTL) & QEI_ENABLE, QEI_ENABLE, "QEI runs");
    assert_eq!(rw(mem, QEI0 + QEIPOS), 0, "no edges, no motion");
}

// --- CAN ---

const CAN0: u32 = 0x4004_0000;
const CANCTL: u32 = 0x000;
const CANSTS: u32 = 0x004;
const CANINT: u32 = 0x010;
const IF1CRQ: u32 = 0x020;
const IF1CMSK: u32 = 0x024;
const IF1ARB1: u32 = 0x030;
const IF1ARB2: u32 = 0x034;
const IF1MCTL: u32 = 0x038;
const IF1DA1: u32 = 0x03C;
const TXRQ1: u32 = 0x100;
const CTL_INIT: u32 = 1 << 0;
const CTL_IE: u32 = 1 << 1;
const STS_TXOK: u32 = 1 << 3;
const CMSK_SETUP_TX: u32 = (1 << 7) | (1 << 5) | (1 << 4) | (1 << 2) | (1 << 1) | (1 << 0);
const ARB2_MSGVAL: u32 = 1 << 15;
const ARB2_DIR: u32 = 1 << 13;
const MCTL_TXIE: u32 = 1 << 11;
const MCTL_EOB: u32 = 1 << 7;

#[test]
fn can0_transmits_a_frame() {
    let mut m = tiva();
    let mem = m.core.mem();
    assert_eq!(rw(mem, CAN0 + CANCTL) & CTL_INIT, CTL_INIT, "INIT holds");
    ww(mem, CAN0 + CANCTL, CTL_IE); // leave init
    ww(mem, CAN0 + IF1ARB2, ARB2_MSGVAL | ARB2_DIR | (0x123 << 2));
    ww(mem, CAN0 + IF1ARB1, 0);
    ww(mem, CAN0 + IF1MCTL, MCTL_TXIE | MCTL_EOB | 2);
    ww(mem, CAN0 + IF1DA1, 0xADDE);
    ww(mem, CAN0 + IF1CMSK, CMSK_SETUP_TX);
    ww(mem, CAN0 + IF1CRQ, 1);
    assert_eq!(rw(mem, CAN0 + CANSTS) & STS_TXOK, STS_TXOK, "TX completes");
    assert_eq!(rw(mem, CAN0 + CANINT) & 0x7FFF, 1, "object 1 named");
    assert_eq!(rw(mem, CAN0 + TXRQ1), 0, "TXRQ clears");
}

// --- USB ---

const USB0: u32 = 0x4005_0000;
const POWER: u32 = 0x001;
const TXIS: u32 = 0x002;
const FIFO1: u32 = 0x024;
const TXCSRL1: u32 = 0x112;
const TXRDY: u32 = 1 << 0;

fn usb_byte(mem: &mut board::BoardMemory, addr: u32) -> u8 {
    mem.read(addr).unwrap()
}

fn usb_write_byte(m: &mut Machine<board::BoardMemory>, addr: u32, val: u8) {
    m.core.mem().write(addr, val).unwrap();
}

#[test]
fn usb0_ships_ep1_in() {
    let mut m = tiva();
    let mem = m.core.mem();
    {
        let mem = m.core.mem();
        assert_eq!(usb_byte(mem, USB0 + POWER), 0x20, "datasheet reset");
        ww(mem, USB0 + FIFO1, 0x4433_2211);
    }
    usb_write_byte(&mut m, USB0 + TXCSRL1, TXRDY as u8);
    let mem = m.core.mem();
    assert_eq!(rw(mem, USB0 + TXIS) & 0x02, 0x02, "EP1 TXIS set");
}

// --- EEPROM ---

const EEPROM: u32 = 0x400A_F000;
const EEBLOCK: u32 = 0x004;
const EEOFFSET: u32 = 0x008;
const EERDWR: u32 = 0x010;
const EEDONE: u32 = 0x018;

#[test]
fn eeprom_round_trips_a_word() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, EEPROM + EEBLOCK, 1);
    ww(mem, EEPROM + EEOFFSET, 2);
    ww(mem, EEPROM + EERDWR, 0xDEAD_BEEF);
    assert_eq!(rw(mem, EEPROM + EEDONE), 0, "instant completion");
    assert_eq!(rw(mem, EEPROM + EERDWR), 0xDEAD_BEEF, "word round-trips");
}

// --- FLASH ---

const FLASH: u32 = 0x400F_D000;
const FMA: u32 = 0x000;
const FMD: u32 = 0x004;
const FMC: u32 = 0x008;
const FCRIS: u32 = 0x00C;
const WRITE: u32 = 1 << 0;
const WRKEY: u32 = 0xA442_0000;
const PRIS: u32 = 1 << 1;

#[test]
fn flash_keyed_write_completes() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, FLASH + FMA, 0x1000);
    ww(mem, FLASH + FMD, 0x1234_5678);
    ww(mem, FLASH + FMC, WRKEY | WRITE);
    assert_eq!(rw(mem, FLASH + FMC) & WRITE, 0, "command self-clears");
    assert_eq!(rw(mem, FLASH + FCRIS) & PRIS, PRIS, "program completes");
}

// --- HIB ---

const HIB: u32 = 0x400F_C000;
const RTCC: u32 = 0x000;
const HIBCTL: u32 = 0x010;
const RTCEN: u32 = 1 << 0;

#[test]
fn hib_counts_seconds() {
    let mut m = tiva();
    let mem = m.core.mem();
    ww(mem, HIB + HIBCTL, RTCEN);
    let t0 = rw(mem, HIB + RTCC);
    pump(mem, 488 * 3 + 10);
    assert_eq!(rw(mem, HIB + RTCC), t0.wrapping_add(3), "RTC advances");
}

// --- COMP ---

const COMP: u32 = 0x4003_C000;
const ACSTAT0: u32 = 0x020;
const ACCTL0: u32 = 0x024;
const OVAL: u32 = 1 << 1;
const CINV: u32 = 1 << 1;

#[test]
fn comp_inverts_output() {
    let mut m = tiva();
    let mem = m.core.mem();
    assert_eq!(rw(mem, COMP + ACSTAT0) & OVAL, 0, "undriven reads low");
    ww(mem, COMP + ACCTL0, CINV);
    assert_eq!(rw(mem, COMP + ACSTAT0) & OVAL, OVAL, "inversion flips OVAL");
}

// --- SYSEXC (proven latch) ---

const SYSEXC: u32 = 0x400F_9000;
const SX_RIS: u32 = 0x00;
const SX_IM: u32 = 0x04;
const SX_MIS: u32 = 0x08;
const SX_IC: u32 = 0x0C;

#[test]
fn sysexc_stays_quiet() {
    let mut m = tiva();
    let mem = m.core.mem();
    // Zero-correct: no CPU faults occur in simulation.
    assert_eq!(rw(mem, SYSEXC + SX_RIS), 0, "no raw faults");
    assert_eq!(rw(mem, SYSEXC + SX_MIS), 0, "no masked faults");
    // Mask programs and reads back; the clear register takes writes.
    ww(mem, SYSEXC + SX_IM, 0x3F);
    assert_eq!(rw(mem, SYSEXC + SX_IM), 0x3F, "IM latches");
    ww(mem, SYSEXC + SX_IC, 0x3F);
    assert_eq!(rw(mem, SYSEXC + SX_RIS), 0, "still quiet after clear");
    ww(mem, SYSEXC + SX_IM, 0);
}

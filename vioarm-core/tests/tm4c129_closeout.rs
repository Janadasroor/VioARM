//! Phase-P3 `tm4c1294ncpdt` close-out proof (`ti-roadmap.md` 129 P3):
//! every window the audit promotes off the latch list is exercised
//! through driver-style register pokes on the real board (no toolchain).
//!
//! 129-specific coverage beyond the `tiva_closeout.rs` pattern: the native
//! `SYSCTL` (`DID0`/`DID1`, `RSCLKCFG` 120 MHz PLL path, `RCGC` latch —
//! this family has no `PRn`), the AHB-only GPIO apertures (late-port
//! round-trip, `PD7` commit lock), the new `TIMER6`/`I2C4` instances, and
//! the native flash controller (`FMC2` buffered program, `PP`/`SSIZE`).
//!
//! Register offsets come from the TM4C1294NCPDT datasheet (SPMS433B);
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

fn tiva129() -> Machine<board::BoardMemory> {
    Machine::armv7m(board::tm4c1294ncpdt())
}

// --- SYSCTL (native 129) ---

const SYSCTL: u32 = 0x400F_E000;
const DID0: u32 = 0x000;
const DID1: u32 = 0x004;
const RCGCUART: u32 = 0x618;
const RSCLKCFG: u32 = 0x0B0;
const PLLFREQ0: u32 = 0x160;
const PLLFREQ1: u32 = 0x164;
const PLLSTAT: u32 = 0x168;
const PPUART: u32 = 0x318;

#[test]
fn sysctl_ids_and_clock() {
    let mut m = tiva129();
    let mem = m.core.mem();
    assert_eq!(rw(mem, SYSCTL + DID0), 0x100A_0001, "Snowflake/A1");
    assert_eq!(rw(mem, SYSCTL + DID1), 0x101F_C06E, "1294/128-TQFP");
    assert_eq!(rw(mem, SYSCTL + PPUART), 0xFF, "8 UARTs present");
    // `RCGC` latches (no `PRn` on this family — ready is immediate).
    ww(mem, SYSCTL + RCGCUART, 0x01);
    assert_eq!(rw(mem, SYSCTL + RCGCUART), 0x01, "RCGC latches");
    // LaunchPad 120 MHz path: MOSC PLL, MINT=96, N=4, PSYSDIV=3.
    ww(mem, SYSCTL + PLLFREQ1, 4);
    ww(mem, SYSCTL + PLLFREQ0, (1 << 23) | 96);
    ww(
        mem,
        SYSCTL + RSCLKCFG,
        (1 << 28) | (3 << 24) | 3 | (1 << 30), // USEPLL + MOSC + /4 + NEWFREQ
    );
    pump(mem, 200);
    assert_eq!(rw(mem, SYSCTL + PLLSTAT) & 0x01, 0x01, "PLL locks");
}

// --- GPIO (AHB-only + late ports) ---

const GPIOF_AHB: u32 = 0x4005_D000;
const GPIOQ: u32 = 0x4006_6000;
const GPIOD_AHB: u32 = 0x4005_B000;
const DATA: u32 = 0x3FC;
const DIR: u32 = 0x400;
const DEN: u32 = 0x51C;
const CR: u32 = 0x524;

#[test]
fn gpio_drives_senses_and_locks() {
    let mut m = tiva129();
    let mem = m.core.mem();
    ww(mem, GPIOF_AHB + DIR, 0x0E);
    ww(mem, GPIOF_AHB + DEN, 0xFF);
    ww(mem, GPIOF_AHB + DATA, 0x0A);
    assert_eq!(rw(mem, GPIOF_AHB + DATA), 0x0A, "AHB DATA latches");
    // Late single-aperture port round-trips.
    ww(mem, GPIOQ + DIR, 0xFF);
    ww(mem, GPIOQ + DEN, 0xFF);
    ww(mem, GPIOQ + DATA, 0xA5);
    assert_eq!(rw(mem, GPIOQ + DATA), 0xA5, "GPIOQ drives");
    // `PD7` (NMI) is commit-locked: `DEN` bit 7 ignores the write.
    ww(mem, GPIOD_AHB + DEN, 0xFF);
    assert_eq!(rw(mem, GPIOD_AHB + DEN) & 0x80, 0, "PD7 locked");
    assert_eq!(rw(mem, GPIOD_AHB + CR) & 0x80, 0, "CR bit clear");
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

#[test]
fn uart0_moves_bytes() {
    let mut m = tiva129();
    {
        let mem = m.core.mem();
        ww(mem, SYSCTL + RCGCUART, 0x01);
        ww(mem, UART0 + CTL, UARTEN | TXE | RXE);
        ww(mem, UART0 + DR, u32::from(b'V'));
    }
    assert_eq!(m.drain_device("UART0"), b"V", "TX drains");
    assert!(m.drive_device("UART0", b"i"));
    let mem = m.core.mem();
    assert_eq!(rw(mem, UART0 + FR) & RXFE, 0, "RX not empty");
    assert_eq!(rw(mem, UART0 + DR) & 0xFF, u32::from(b'i'), "RX injects");
}

// --- GPTM (incl. new TIMER6) ---

const TIMER0: u32 = 0x4003_0000;
const TIMER6: u32 = 0x400E_0000;
const TAMR: u32 = 0x004;
const GPTMCTL: u32 = 0x00C;
const TAEN: u32 = 1 << 0;
const IMR: u32 = 0x018;
const TATOIM: u32 = 1 << 0;
const RIS: u32 = 0x01C;
const TATORIS: u32 = 1 << 0;
const ICR: u32 = 0x024;
const TAILR: u32 = 0x028;

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
    let mut m = tiva129();
    timer_count_to_timeout(m.core.mem(), TIMER0);
}

#[test]
fn timer6_new_instance_counts() {
    let mut m = tiva129();
    timer_count_to_timeout(m.core.mem(), TIMER6);
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
    let mut m = tiva129();
    let mem = m.core.mem();
    ww(mem, SSI0 + CR1, SSE);
    ww(mem, SSI0 + CPSR, 2);
    ww(mem, SSI0 + CR0, 0x07); // 8-bit Freescale
    assert_eq!(rw(mem, SSI0 + SR) & (TFE | TNF), TFE | TNF, "FIFO empty");
    ww(mem, SSI0 + SSI_DR, 0xA5);
    assert_eq!(rw(mem, SSI0 + SR) & (TFE | TNF), TFE | TNF, "frame ships");
    assert_eq!(rw(mem, SSI0 + SSIRIS) & TXRIS, TXRIS, "TXRIS follows");
}

// --- I2C (incl. new I2C4) ---

const I2C0: u32 = 0x4002_0000;
const I2C4: u32 = 0x400C_0000;
const MSA: u32 = 0x000;
const MCS: u32 = 0x004;
const MDR: u32 = 0x008;
const MRIS: u32 = 0x014;
const MCR: u32 = 0x020;
const MFE: u32 = 1 << 4;
const BUSY: u32 = 1 << 0;

fn i2c_master_send(mem: &mut board::BoardMemory, base: u32) {
    ww(mem, base + MCR, MFE);
    ww(mem, base + MSA, 0x50 << 1);
    ww(mem, base + MDR, 0xA5);
    ww(mem, base + MCS, 0x07); // RUN+START+STOP
    assert_eq!(rw(mem, base + MCS) & BUSY, 0, "send completes");
    assert_eq!(rw(mem, base + MRIS) & 0x01, 0x01, "master RIS set");
}

#[test]
fn i2c0_completes_master_send() {
    let mut m = tiva129();
    i2c_master_send(m.core.mem(), I2C0);
}

#[test]
fn i2c4_new_instance_sends() {
    let mut m = tiva129();
    i2c_master_send(m.core.mem(), I2C4);
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
    let mut m = tiva129();
    let mem = m.core.mem();
    ww(mem, WDT0 + LOAD, 100);
    assert_eq!(rw(mem, WDT0 + LOAD), 0xFFFF_FFFF, "locked writes sink");
    ww(mem, WDT0 + WDTLOCK, LOCK_KEY);
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
    let mut m = tiva129();
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
    let mut m = tiva129();
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
    let mut m = tiva129();
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
    let mut m = tiva129();
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
    let mut m = tiva129();
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
    let mut m = tiva129();
    let mem = m.core.mem();
    ww(mem, EEPROM + EEBLOCK, 1);
    ww(mem, EEPROM + EEOFFSET, 2);
    ww(mem, EEPROM + EERDWR, 0xDEAD_BEEF);
    assert_eq!(rw(mem, EEPROM + EEDONE), 0, "instant completion");
    assert_eq!(rw(mem, EEPROM + EERDWR), 0xDEAD_BEEF, "word round-trips");
}

// --- FLASH (native 129) ---

const FLASH: u32 = 0x400F_D000;
const FMA: u32 = 0x000;
const FMC: u32 = 0x008;
const FCRIS: u32 = 0x00C;
const FMC2: u32 = 0x020;
const FWBVAL: u32 = 0x030;
const FWBN: u32 = 0x100;
const PP: u32 = 0xFC0;
const SSIZE: u32 = 0xFC4;
const WRITE: u32 = 1 << 0;
const WRKEY: u32 = 0xA442_0000;
const WRBUF: u32 = 1 << 0;
const PRIS: u32 = 1 << 1;

#[test]
fn flash_ids_and_buffers() {
    let mut m = tiva129();
    let mem = m.core.mem();
    assert_eq!(rw(mem, FLASH + PP), 0xF014_01FF, "1 MB properties");
    assert_eq!(rw(mem, FLASH + SSIZE), 0x03FF, "256 KB SRAM");
    // Buffered program: two valid words, one commit.
    ww(mem, FLASH + FMA, 0x2000);
    ww(mem, FLASH + FWBN, 0xDEAD_BEEF);
    ww(mem, FLASH + FWBN + 4, 0xCAFE_0001);
    ww(mem, FLASH + FWBVAL, 0x03);
    ww(mem, FLASH + FMC2, WRKEY | WRBUF);
    assert_eq!(rw(mem, FLASH + FWBVAL), 0, "valid bits clear");
    assert_eq!(rw(mem, FLASH + FCRIS) & PRIS, PRIS, "buffer programs");
    // Single-word path still answers.
    ww(mem, FLASH + FMA, 0x1000);
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
    let mut m = tiva129();
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
    let mut m = tiva129();
    let mem = m.core.mem();
    assert_eq!(rw(mem, COMP + ACSTAT0) & OVAL, 0, "undriven reads low");
    ww(mem, COMP + ACCTL0, CINV);
    assert_eq!(rw(mem, COMP + ACSTAT0) & OVAL, OVAL, "inversion flips OVAL");
}

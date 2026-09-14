//! Shared analog bus e2e (WP-R4): one host-driven millivolt node moves every
//! consumer coherently.
//!
//! A production `stm32f103` board is ramped through the host entry point
//! (`Machine::analog_drive_mv`): the ADC result tracks the bus, the GPIO
//! input word follows it through the Schmitt path, EXTI latches the rising
//! edge, and TIM2 in external-clock mode counts it — all observed in one
//! run, with reruns byte-identical. A second test covers the transport seam:
//! an `Apin` receipt publishes into the same bus an ADC samples.

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::device::analog::{mv_to_adc_code, AnalogBusState, Apin};
use vioarm_core::device::svd::stm32f103::Stm32f103;
use vioarm_core::device::svd::Device;
use vioarm_core::device::Peripheral;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

// F1 ADC register offsets and strobe bits (see `device/stm32/adc.rs`).
const ADC_CR2: u32 = 0x08;
const ADC_DR: u32 = 0x4C;
const ADON: u32 = 1 << 0;
const SWSTART_F1: u32 = 1 << 22;
// F1 GPIO `IDR` offset; EXTI `IMR`/`RTSR`/`PR`; TIM `CR1`/`SMCR`/`CNT`/`ARR`.
const GPIO_IDR: u32 = 0x08;
const EXTI_IMR: u32 = 0x00;
const EXTI_RTSR: u32 = 0x08;
const EXTI_PR: u32 = 0x14;
const TIM_CR1: u32 = 0x00;
const TIM_SMCR: u32 = 0x08;
const TIM_CNT: u32 = 0x24;
const TIM_ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;

fn base(name: &str) -> u32 {
    Stm32f103::peripheral(name)
        .unwrap_or_else(|| panic!("F103 tables contain {name}"))
        .base
}

fn write_word(m: &mut Machine<BoardMemory>, addr: u32, val: u32) {
    m.core.mem().write_word(addr, val).unwrap();
}

fn read_word(m: &mut Machine<BoardMemory>, addr: u32) -> u32 {
    m.core.mem().read_word(addr).unwrap()
}

/// Ramp the shared bus low then high on a production board; returns
/// `(adc_lo, adc_hi, idr_lo, idr_hi, pr_hi, cnt_hi)`.
fn scenario() -> (u32, u32, u32, u32, u32, u32) {
    let mut m = Machine::armv7m(board::stm32f103());
    let adc = base("ADC1");
    let gpioa = base("GPIOA");
    let exti = base("EXTI");
    let tim2 = base("TIM2");
    // Arm every consumer: ADC powered, EXTI rising line 0 masked in, TIM2
    // external-clock mode 1 counting bus-derived TI2 edges.
    write_word(&mut m, adc + ADC_CR2, ADON);
    write_word(&mut m, exti + EXTI_RTSR, 1);
    write_word(&mut m, exti + EXTI_IMR, 1);
    write_word(&mut m, tim2 + TIM_ARR, 99);
    write_word(&mut m, tim2 + TIM_SMCR, 0b111);
    write_word(&mut m, tim2 + TIM_CR1, CEN);
    // Ramp low: settle the edge references, then convert.
    m.analog_drive_mv(500);
    m.core.mem().tick_devices(1);
    write_word(&mut m, adc + ADC_CR2, ADON | SWSTART_F1);
    m.core.mem().tick_devices(14);
    let adc_lo = read_word(&mut m, adc + ADC_DR);
    let idr_lo = read_word(&mut m, gpioa + GPIO_IDR) & 0xFFFF;
    let pr_lo = read_word(&mut m, exti + EXTI_PR) & 1;
    let cnt_lo = read_word(&mut m, tim2 + TIM_CNT);
    assert_eq!(pr_lo, 0, "no edge while low");
    assert_eq!(cnt_lo, 0, "no edge while low");
    // Ramp high: every consumer moves in the same run.
    m.analog_drive_mv(2500);
    write_word(&mut m, adc + ADC_CR2, ADON | SWSTART_F1);
    m.core.mem().tick_devices(14);
    let adc_hi = read_word(&mut m, adc + ADC_DR);
    let idr_hi = read_word(&mut m, gpioa + GPIO_IDR) & 0xFFFF;
    let pr_hi = read_word(&mut m, exti + EXTI_PR) & 1;
    let cnt_hi = read_word(&mut m, tim2 + TIM_CNT);
    (adc_lo, adc_hi, idr_lo, idr_hi, pr_hi, cnt_hi)
}

#[test]
fn ramp_moves_all_consumers_coherently() {
    let (adc_lo, adc_hi, idr_lo, idr_hi, pr_hi, cnt_hi) = scenario();
    assert_eq!(adc_lo, mv_to_adc_code(500), "ADC tracks the low ramp");
    assert_eq!(adc_hi, mv_to_adc_code(2500), "ADC tracks the high ramp");
    assert_eq!(idr_lo, 0, "GPIO follows low");
    assert_eq!(idr_hi, 0xFFFF, "GPIO follows high");
    assert_eq!(pr_hi, 1, "EXTI latches the rising edge");
    assert_eq!(cnt_hi, 1, "TIM2 counts the bus edge");
}

#[test]
fn ramp_rerun_is_deterministic() {
    assert_eq!(scenario(), scenario());
}

#[test]
fn apin_receipt_feeds_bus_consumers() {
    const APIN: u32 = 0x4000_2000;
    const CR: u32 = 0x10;
    const EN: u32 = 1;
    let bus = AnalogBusState::shared();
    let mut apin = Apin::at(APIN).with_bus(bus.clone());
    let mut adc = vioarm_core::device::stm32::adc::Adc::adc1().with_analog_bus(bus.clone());
    // Enable the endpoint, then deliver one word the way the fabric does.
    for (i, b) in EN.to_le_bytes().iter().enumerate() {
        apin.write(APIN + CR + i as u32, *b).unwrap();
    }
    for b in 1200u32.to_le_bytes() {
        apin.drive_inputs(u32::from(b));
    }
    assert_eq!(AnalogBusState::millivolts(&bus), Some(1200));
    // The ADC samples the published node, not its own latch.
    adc.drive_inputs(0x0ABC);
    // Same strobes as the machine test, via the ADC's own window.
    fn adc_write(adc: &mut vioarm_core::device::stm32::adc::Adc, off: u32, val: u32) {
        let base = adc.ranges()[0].base;
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            adc.write(base + off + i as u32, *b).unwrap();
        }
    }
    fn adc_read(adc: &mut vioarm_core::device::stm32::adc::Adc, off: u32) -> u32 {
        let base = adc.ranges()[0].base;
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = adc.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }
    adc_write(&mut adc, ADC_CR2, ADON);
    adc_write(&mut adc, ADC_CR2, ADON | SWSTART_F1);
    adc.tick(14);
    assert_eq!(adc_read(&mut adc, ADC_DR), mv_to_adc_code(1200));
}

use vioarm_core::device::stm32::dma_f4::DmaF4;
use vioarm_core::device::stm32::rcc::Rcc;
use vioarm_core::device::stm32::timer::Timer;
use vioarm_core::device::Peripheral;

fn write_word<P: Peripheral>(p: &mut P, base: u32, off: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        p.write(base + off + i as u32, *b).unwrap();
    }
}
fn read_word<P: Peripheral>(p: &mut P, base: u32, off: u32) -> u32 {
    let mut b = [0u8; 4];
    for (i, v) in b.iter_mut().enumerate() {
        *v = p.read(base + off + i as u32).unwrap();
    }
    u32::from_le_bytes(b)
}

#[test]
fn rcc_cfgr_sws_follows_sw() {
    let mut rcc = Rcc::new();
    let base = 0x4002_1000u32;
    // SW = 10 (PLL), expect SWS = 10
    write_word(&mut rcc, base, 0x04, 0x2);
    assert_eq!(read_word(&mut rcc, base, 0x04) & 0xC, 0x8);
    // SW = 01 (HSE)
    write_word(&mut rcc, base, 0x04, 0x1);
    assert_eq!(read_word(&mut rcc, base, 0x04) & 0xC, 0x4);
}

#[test]
fn rcc_ppre_roundtrips() {
    let mut rcc = Rcc::new();
    let base = 0x4002_1000u32;
    write_word(&mut rcc, base, 0x04, 0x0000_9400); // PPRE1 div4, PPRE2 div2
    let v = read_word(&mut rcc, base, 0x04);
    assert_eq!(v & 0x7C00, 0x0000_1400); // check low bits of PPRE1? Actually just roundtrip
}

#[test]
fn timer_pwm_at_pclk_psc() {
    // TIM2 PWM 50% duty, PSC divide by 2, period 4, CCR 2
    let mut t = Timer::tim2();
    let base = 0x4000_0000u32;
    write_word(&mut t, base, 0x2C, 3); // ARR 3 => period 4
    write_word(&mut t, base, 0x34, 2); // CCR1 2
    write_word(&mut t, base, 0x18, 6 << 4); // OC1M PWM1
    write_word(&mut t, base, 0x20, 1); // CC1E
    write_word(&mut t, base, 0x28, 1); // PSC 1 => div2
    write_word(&mut t, base, 0x00, 1); // CEN
                                       // First tick: psc_phase 1/2 not yet overflow, CNT stays 0, output still false
    t.tick(1);
    assert_eq!(read_word(&mut t, base, 0x24), 0);
    // Second tick: PSC overflow, CNT 1, PWM active (1<2)
    t.tick(1);
    assert!(t.output_level(1));
    // Tick to CNT 2 => inactive
    t.tick(2);
    assert!(!t.output_level(1));
}

#[test]
fn dma_f4_burst_4_beats_per_tick() {
    use std::sync::{Arc, Mutex};
    let m = Arc::new(Mutex::new(Vec::new()));
    let mut d = DmaF4::dma1_for::<vioarm_core::device::svd::stm32f407::Stm32f407>(m.clone());
    let base = 0x4002_6000u32;
    // Stream 0 with MBURST INCR4 (01), NDTR 8, M2M
    let s = 0u32;
    let cr = 0x10 + s * 0x18;
    let ndtr = cr + 0x04;
    let par = cr + 0x08;
    let m0ar = cr + 0x0C;
    // MBURST 01 (INCR4) at bits 23:21 -> 01
    let mburst = 1 << 23;
    write_word(&mut d, base, par, 0x1000);
    write_word(&mut d, base, m0ar, 0x2000);
    write_word(&mut d, base, ndtr, 8);
    write_word(&mut d, base, cr, 0x1 | (2 << 6) | mburst | (1 << 10)); // EN + M2M + MINC + MBURST
    d.tick(1);
    assert_eq!(m.lock().unwrap().len(), 4, "burst 4 first tick");
    assert_eq!(read_word(&mut d, base, ndtr), 4);
    d.tick(1);
    assert_eq!(m.lock().unwrap().len(), 8, "second burst completes");
    assert_eq!(read_word(&mut d, base, ndtr), 0);
    assert_eq!(read_word(&mut d, base, cr) & 0x1, 0, "EN cleared");
}

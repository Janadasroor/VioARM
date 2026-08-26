use vioarm_core::device::stm32::dma_f4::DmaF4;
use vioarm_core::device::stm32::rcc::Rcc;
use vioarm_core::device::stm32::timer::Timer;
use vioarm_core::device::Peripheral;
use vioarm_core::mem::Bus;

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

/// The F103 board wires TIM2's update DMA request to DMA1 channel 2
/// (RM0008 \u{a7}13.3.7 Table 78): arming UDE + a peripheral-request channel
/// moves one unit per counter overflow, end to end through the bus.
#[test]
fn f103_board_tim2_update_dma_moves_units() {
    use std::sync::{Arc, Mutex};
    use vioarm_core::device::stm32::dma::{DmaF1, DmaRequests, DmaUnit};
    use vioarm_core::device::stm32::timer::Timer;
    use vioarm_core::device::svd::stm32f103::Stm32f103;
    use vioarm_core::device::Peripheral;
    use vioarm_core::mem::flat::FlatMemory;
    use vioarm_core::mem::system::SystemMemory;
    use vioarm_core::mem::Bus;

    fn ww(p: &mut SystemMemory<FlatMemory>, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(addr + i as u32, *b).unwrap();
        }
    }
    fn rd(p: &mut SystemMemory<FlatMemory>, addr: u32) -> u32 {
        let mut v = 0u32;
        for i in 0..4u32 {
            v |= u32::from(p.read(addr + i).unwrap()) << (8 * i);
        }
        v
    }

    let requests = Arc::new(Mutex::new(DmaRequests::new()));
    let mailbox = Arc::new(Mutex::new(Vec::<DmaUnit>::new()));

    // A bare F103-style bus: TIM2 wired to the request box plus a DMA1 that
    // drains it.
    let backing = FlatMemory::with_layout(&[0x4000_0000, 0x4002_0000], &[0x4000, 0x400]);
    let mut mem = SystemMemory::new(backing);
    mem.set_dma_mailbox(mailbox.clone());
    let t = Timer::tim2_for::<Stm32f103>().with_dma_update(requests.clone(), 0, 2);
    mem.attach(Box::new(t));
    mem.attach(Box::new(DmaF1::dma_for::<Stm32f103>(
        "DMA1", mailbox, requests,
    )));

    const T2: u32 = 0x4000_0000;
    const D1: u32 = 0x4002_0000;
    // TIM2: UDE + CEN (ARR reset 0 -> overflow every tick).
    ww(&mut mem, T2 + 0x0C, 1 << 8);
    ww(&mut mem, T2 + 0x00, 1);
    // DMA1 ch2 (index 1): MINC, TCIE, NDTR=2.
    ww(&mut mem, D1 + 0x14 + 0x14, 0x2000); // CMAR2
    ww(&mut mem, D1 + 0x10 + 0x14, 0x2000); // CPAR2
    ww(&mut mem, D1 + 0x0C + 0x14, 2); // CNDTR2
    ww(&mut mem, D1 + 0x08 + 0x14, 1 | (1 << 7) | 0x2); // EN|MINC|TCIE

    // Two overflows -> two units land via CMAR2.
    for i in 0..4 {
        mem.tick_devices(1);
    }
    assert_eq!(rd(&mut mem, D1 + 0x0C + 0x14), 0, "CNDTR exhausted");
    assert_ne!(rd(&mut mem, D1 + 0x00) & 0x30, 0, "GIF2|TCIF2 latched");
}

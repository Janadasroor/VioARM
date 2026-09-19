//! Analog-extras e2e on the `h743` board, in firmware-demand order: a
//! DAC→OPAMP→ADC voltage chain with a DFSDM conversion on the same bus,
//! SAI audio streaming, and SPDIFRX sync/data — all through real
//! addresses with interrupt lines observed per device.

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::mem::Bus;

const DAC: u32 = 0x4000_7400;
const ADC1: u32 = 0x4002_2000;
const OPAMP: u32 = 0x4000_9000;
const DFSDM: u32 = 0x4001_7000;
const SAI1: u32 = 0x4001_5800;
const SPDIFRX: u32 = 0x4000_4000;
const RAM: u32 = 0x2000_0000;

fn wr(mem: &mut BoardMemory, addr: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        mem.write(addr + i as u32, *b).unwrap();
    }
}

fn rd(mem: &mut BoardMemory, addr: u32) -> u32 {
    let mut bytes = [0u8; 4];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = mem.read(addr + i as u32).unwrap();
    }
    u32::from_le_bytes(bytes)
}

fn dev_by_base(mem: &BoardMemory, base: u32) -> usize {
    mem.devices()
        .iter()
        .position(|d| d.ranges()[0].base == base)
        .unwrap_or_else(|| panic!("no device at {base:#x}"))
}

fn pending_at(mem: &BoardMemory, base: u32) -> u32 {
    mem.devices()[dev_by_base(mem, base)].pending()
}

#[test]
fn h743_analog_chain_sai_spdifrx() {
    let mut mem = board::stm32h743();

    // ---- DAC channel 1 → 800 mV on the bus ---------------------------
    wr(&mut mem, DAC, 1); // CR EN1
    wr(&mut mem, DAC + 0x08, 993); // DHR12R1 → 993·3300/4095 = 800 mV
    assert_eq!(rd(&mut mem, DAC + 0x2C), 993, "DOR1");

    // ---- OPAMP1 PGA ×2 → 1600 mV --------------------------------------
    wr(&mut mem, OPAMP, 1); // CSR EN, PGA gain 0 (×2)
    mem.tick_devices(1);

    // ---- ADC1 single conversion → ~1985 -------------------------------
    wr(&mut mem, ADC1 + 0x08, 1); // CR ADEN
    for _ in 0..30 {
        mem.tick_devices(1);
        if rd(&mut mem, ADC1) & 1 != 0 {
            break;
        }
    }
    assert_ne!(rd(&mut mem, ADC1) & 1, 0, "ADRDY");
    wr(&mut mem, ADC1 + 0x08, 1 | (1 << 2)); // ADSTART
    for _ in 0..30 {
        mem.tick_devices(1);
        if rd(&mut mem, ADC1) & (1 << 2) != 0 {
            break;
        }
    }
    assert_ne!(rd(&mut mem, ADC1) & (1 << 2), 0, "EOC");
    assert_eq!(rd(&mut mem, ADC1 + 0x40), 1985, "1600 mV code");

    // ---- DFSDM filter 0 regular conversion of the same bus -----------
    // DATA = (1600 − 1650)·2²³/1650 = −254200 → 0xFC1F0800.
    wr(&mut mem, DFSDM, (1 << 7) | (1 << 31)); // CHCFG0R1 CHEN DFSDMEN
    wr(&mut mem, DFSDM + 0xB0, 1 << 1); // CR2 REOCIE
    wr(&mut mem, DFSDM + 0xA0, 1 | (1 << 17)); // CR1 DFEN RSWSTART
    mem.tick_devices(1);
    assert_ne!(rd(&mut mem, DFSDM + 0xC0) & (1 << 1), 0, "REOCF");
    assert_ne!(pending_at(&mem, DFSDM) & 1, 0, "filter 0 line");
    assert_eq!(rd(&mut mem, DFSDM + 0x110), 0xFC1F_0800, "DATA");

    // ---- SAI1_A TX streams, SAI1_B RX underruns ------------------------
    wr(&mut mem, SAI1 + 0x04, 1 << 16); // ACR1 SAIXEN master TX
    wr(&mut mem, SAI1 + 0x14, (1 << 3) | 1); // AIM FREQIE OVRUDRIE
    for w in [0x1111_1111u32, 0x2222_2222, 0x3333_3333, 0x4444_4444] {
        wr(&mut mem, SAI1 + 0x20, w);
    }
    mem.tick_devices(4);
    assert_eq!(rd(&mut mem, SAI1 + 0x18) >> 16 & 0x7, 0, "TX drained");
    assert_eq!(rd(&mut mem, SAI1 + 0x18) & 1, 0, "no overrun");
    wr(&mut mem, SAI1 + 0x24, (1 << 16) | 1); // BCR1 SAIXEN master RX
    wr(&mut mem, SAI1 + 0x34, 1); // BIM OVRUDRIE
    assert_eq!(rd(&mut mem, SAI1 + 0x40), 0, "empty read");
    assert_ne!(rd(&mut mem, SAI1 + 0x38) & 1, 0, "RX OVRUDR");
    assert_ne!(pending_at(&mem, SAI1) & 1, 0, "sai line");

    // ---- SPDIFRX sync + staged data ------------------------------------
    let spd = dev_by_base(&mem, SPDIFRX);
    for b in [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88] {
        mem.devices_mut()[spd].drive_inputs(u32::from(b));
    }
    wr(&mut mem, SPDIFRX, 1); // CR out of idle
    wr(&mut mem, SPDIFRX + 0x04, (1 << 5) | 1); // IMR SYNCDIE RXNEIE
    mem.tick_devices(1);
    mem.tick_devices(1);
    assert_ne!(rd(&mut mem, SPDIFRX + 0x08) & (1 << 5), 0, "SYNCD");
    assert_ne!(rd(&mut mem, SPDIFRX + 0x08) & 1, 0, "RXNE");
    assert_ne!(pending_at(&mem, SPDIFRX), 0, "spdif line");
    assert_eq!(rd(&mut mem, SPDIFRX + 0x10), 0x4433_2211, "frame");
    assert_eq!(rd(&mut mem, SPDIFRX + 0x10), 0x8877_6655, "frame");
}

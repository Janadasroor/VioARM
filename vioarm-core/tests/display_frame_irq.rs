//! Display/camera frame-IRQ e2e on the `h743` board: LTDC layers, reload,
//! and line compare; a DMA2D register fill into RAM; DCMI snapshot bytes;
//! JPEG completion — all through real addresses with interrupt lines
//! observed per device. No internal fields touched.

use vioarm_core::board::{self, BoardMemory};
use vioarm_core::mem::Bus;

const LTDC: u32 = 0x5000_1000;
const DMA2D: u32 = 0x5200_1000;
const DCMI: u32 = 0x4802_0000;
const JPEG: u32 = 0x5200_3000;
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

fn dev_idx(mem: &BoardMemory, name: &str) -> usize {
    mem.devices()
        .iter()
        .position(|d| d.name() == name)
        .unwrap_or_else(|| panic!("no {name} on board"))
}

fn pending(mem: &BoardMemory, name: &str) -> u32 {
    mem.devices()[dev_idx(mem, name)].pending()
}

#[test]
fn h743_display_camera_frame_irqs() {
    let mut mem = board::stm32h743();
    // Scratch RAM is real.
    wr(&mut mem, RAM, 0x1234_5678);
    assert_eq!(rd(&mut mem, RAM), 0x1234_5678);

    // ---- LTDC: layer + reload + line compare -------------------------
    wr(&mut mem, LTDC + 0x18, 1); // GCR LTDCEN
    wr(&mut mem, LTDC + 0x14, 99); // TWCR TOTALH 99 → 100 lines
    wr(&mut mem, LTDC + 0x40, 50); // LIPCR line 50
    wr(&mut mem, LTDC + 0x84, 1); // L1CR LEN
    wr(&mut mem, LTDC + 0xAC, RAM + 0x2000); // L1CFBAR
    wr(&mut mem, LTDC + 0xB0, (320 << 16) | 320); // L1CFBLR latch
    wr(&mut mem, LTDC + 0x34, (1 << 3) | 1); // IER RRIE|LIE
    wr(&mut mem, LTDC + 0x24, 1); // SRCR IMR: reload now
    assert_ne!(rd(&mut mem, LTDC + 0x38) & (1 << 3), 0, "RRIF");
    assert_ne!(pending(&mem, "ltdc"), 0, "reload line");
    wr(&mut mem, LTDC + 0x3c, 1 << 3); // ICR
    assert_eq!(pending(&mem, "ltdc"), 0);
    mem.tick_devices(49);
    assert_eq!(rd(&mut mem, LTDC + 0x38) & 1, 0, "not yet");
    mem.tick_devices(1);
    assert_ne!(rd(&mut mem, LTDC + 0x38) & 1, 0, "LIF at line 50");
    assert_ne!(pending(&mem, "ltdc"), 0, "line IRQ");
    wr(&mut mem, LTDC + 0x3c, 1);
    assert_eq!(pending(&mem, "ltdc"), 0);

    // ---- DMA2D: red fill 8×2 ARGB8888 --------------------------------
    wr(&mut mem, DMA2D + 0x34, 0); // OPFCCR ARGB8888
    wr(&mut mem, DMA2D + 0x38, 0xFFFF_0000); // OCOLR opaque red
    wr(&mut mem, DMA2D + 0x3c, RAM + 0x1000); // OMAR
    wr(&mut mem, DMA2D + 0x44, 2 | (8 << 16)); // NLR 2 lines × 8 px
    wr(&mut mem, DMA2D, (1 << 9) | (3 << 16) | 1); // TCIE R2M START
    for _ in 0..40 {
        mem.tick_devices(1);
        if rd(&mut mem, DMA2D + 0x04) & (1 << 1) != 0 {
            break;
        }
    }
    assert_ne!(rd(&mut mem, DMA2D + 0x04) & (1 << 1), 0, "TCIF");
    assert_ne!(pending(&mem, "dma2d"), 0, "blit line");
    for i in 0..16u32 {
        assert_eq!(
            mem.read(RAM + 0x1000 + i).unwrap(),
            [0, 0, 0xFF, 0xFF][i as usize % 4]
        );
    }
    wr(&mut mem, DMA2D + 0x08, 1 << 1); // IFCR CTCIF
    assert_eq!(pending(&mem, "dma2d"), 0);

    // ---- DCMI: snapshot bytes + frame flag ---------------------------
    let dcmi = dev_idx(&mem, "dcmi");
    for b in [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88] {
        mem.devices_mut()[dcmi].drive_inputs(u32::from(b));
    }
    wr(&mut mem, DCMI + 0x0C, 1); // IER FRAME_IE
    wr(&mut mem, DCMI, (1 << 14) | 1); // ENABLE CAPTURE
    mem.tick_devices(1);
    assert_ne!(rd(&mut mem, DCMI + 0x08) & 1, 0, "FRAMERI");
    assert_ne!(pending(&mem, "dcmi"), 0, "frame line");
    assert_eq!(rd(&mut mem, DCMI + 0x28), 0x4433_2211, "pixels");
    assert_eq!(rd(&mut mem, DCMI + 0x28), 0x8877_6655, "pixels");
    wr(&mut mem, DCMI + 0x14, 1); // ICR
    assert_eq!(pending(&mem, "dcmi"), 0);

    // ---- JPEG: completion flag ---------------------------------------
    wr(&mut mem, JPEG + 0x30, 1); // CR JCEN
    wr(&mut mem, JPEG, 1); // CONFR0 START
    mem.tick_devices(1);
    assert_ne!(rd(&mut mem, JPEG + 0x34) & (1 << 5), 0, "EOCF");
    assert_ne!(pending(&mem, "jpeg"), 0, "codec line");
    wr(&mut mem, JPEG + 0x38, 1 << 5); // CFR CEOCF
    assert_eq!(rd(&mut mem, JPEG + 0x34) & (1 << 5), 0);
}

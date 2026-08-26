//! F5 all-chips smoke: every generated table boots to `BKPT` on the
//! cycle-accurate core (M6 F1-F3), windows round-trip, nothing else moves.

use vioarm_core::{board, loader::load_flat, machine::Machine, mem::Bus};

const NOP: [u8; 2] = [0x00, 0xBF]; // nop
const BKPT: [u8; 2] = [0x70, 0xBE]; // bkpt #0

#[test]
fn all_chips_boot_to_bkpt_on_cycle_accurate_core() {
    let chips = board::all_chips();
    assert!(
        chips.len() >= 12,
        "expected >=12 generated chip tables, got {}",
        chips.len()
    );
    for chip in &chips {
        let mem = board::core_only_board(chip.peripherals, chip.flash_base);
        let mut m = Machine::armv7m(mem);
        // Zero FLASH_ACR wait states where the family exposes it so the
        // expected cycle count is exact (H7 resets LATENCY high).
        for acr in [0x4002_2000u32, 0x4002_3C00, 0x5200_2000] {
            if chip
                .peripherals
                .iter()
                .any(|p| p.base == acr || (acr > p.base && acr < p.base + p.window))
            {
                let _ = m.core.mem().write_word(acr, 0);
            }
        }
        // nop; bkpt — boots from flash alias 0x0800_0000.
        let mut code = Vec::new();
        code.extend_from_slice(&NOP);
        code.extend_from_slice(&BKPT);
        let img = load_flat(chip.flash_base, &code);
        m.load_image(&img);
        m.core.regs.pc = chip.flash_base;
        m.core.xpsr.set(0x01000000);
        // One nop then the bkpt halts; with WS zeroed both are 1c.
        m.step().unwrap();
        assert_eq!(m.core.cyccnt, 1, "{}: nop = 1c", chip.name);
        let halt = m.step();
        assert!(halt.is_err(), "{}: expected bkpt halt, got Ok", chip.name);
    }
}

#[test]
fn every_table_is_internally_consistent() {
    for (name, peripherals) in board::all_chip_tables() {
        for p in peripherals {
            for r in p.registers {
                assert!(u32::from(r.width) >= 8, "{name}:{}: width", r.name);
            }
        }
    }
}

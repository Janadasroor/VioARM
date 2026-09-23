//! `tm4c123gh6zrb` new-window smoke: GPIOL/M/N/P/Q output round-trips —
//! the windows the GH6PZ board lacks. Shared IPs ride the GH6PM
//! closeout (`tiva_closeout.rs`) unchanged.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

#[test]
fn zrb_new_windows_answer() {
    let mut m = Machine::armv7m(board::tm4c123gh6zrb());
    let mem = m.core.mem();
    // Five BGA-only ports, APB DIR/DEN/DATA round-trip each.
    for (name, base) in [
        ("L", 0x4006_2000),
        ("M", 0x4006_3000),
        ("N", 0x4006_4000),
        ("P", 0x4006_5000),
        ("Q", 0x4006_6000),
    ] {
        ww(mem, base + 0x400, 0xFF); // DIR.
        ww(mem, base + 0x51C, 0xFF); // DEN.
        ww(mem, base + 0x3FC, 0xA5); // DATA.
        assert_eq!(rw(mem, base + 0x3FC), 0xA5, "GPIO{name} drives");
    }
}

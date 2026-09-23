//! `tm4c123gh6pz` new-window smoke: GPIOJ, I2C4, and comparator 2 —
//! the windows the GH6PM board lacks. Shared IPs ride the GH6PM
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
fn pz_new_windows_answer() {
    let mut m = Machine::armv7m(board::tm4c123gh6pz());
    let mem = m.core.mem();
    // GPIOJ output round-trip.
    ww(mem, 0x4003_D400, 0xFF); // DIR.
    ww(mem, 0x4003_D51C, 0xFF); // DEN.
    ww(mem, 0x4003_D3FC, 0xA5); // DATA.
    assert_eq!(rw(mem, 0x4003_D3FC), 0xA5, "GPIOJ drives");
    // I2C4 master send completes.
    ww(mem, 0x400C_0020, 0x10); // MCR MFE.
    ww(mem, 0x400C_0000, 0x50 << 1); // MSA.
    ww(mem, 0x400C_0008, 0xA5); // MDR.
    ww(mem, 0x400C_0004, 0x07); // MCS RUN+START+STOP.
    assert_eq!(rw(mem, 0x400C_0004) & 0x01, 0, "send completes");
    assert_eq!(rw(mem, 0x400C_0014) & 0x01, 0x01, "master RIS set");
    // Comparator 2 inverts (driven low + CINV reads high).
    assert_eq!(rw(mem, 0x4003_C060) & 0x02, 0, "comp2 low");
    ww(mem, 0x4003_C064, 0x02); // CINV.
    assert_eq!(rw(mem, 0x4003_C060) & 0x02, 0x02, "comp2 flips");
}

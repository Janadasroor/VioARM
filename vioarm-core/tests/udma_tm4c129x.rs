//! Board-level μDMA proof on `tm4c129xnczad`: a real SRAM control table
//! drives a bus-coupled memory-to-memory copy — table read, data units,
//! write-back, `CHIS` — with no test doubles.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const UDMA0: u32 = 0x400F_F000;
const CFG: u32 = 0x04;
const CTLBASE: u32 = 0x08;
const ENASET: u32 = 0x28;
const SWREQ: u32 = 0x14;
const CHIS: u32 = 0x504;

const TABLE: u32 = 0x2000_0400;
const SRC: u32 = 0x2000_0800;
const DST: u32 = 0x2000_0900;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

#[test]
fn udma_mem_to_mem_copies() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    // Source pattern: 16 words.
    for i in 0..16u32 {
        ww(mem, SRC + 4 * i, 0xA500_0000 + i);
    }
    // Control table ch0: end pointers + `CHCTL` (32-bit, N=16, BASIC).
    ww(mem, TABLE, SRC + 64 - 1);
    ww(mem, TABLE + 4, DST + 64 - 1);
    ww(mem, TABLE + 8, 0xAA00_C0F1);
    // Arm + request.
    ww(mem, UDMA0 + CFG, 1);
    ww(mem, UDMA0 + CTLBASE, TABLE);
    ww(mem, UDMA0 + ENASET, 1);
    ww(mem, UDMA0 + SWREQ, 1);
    for _ in 0..8 {
        mem.tick_devices(1);
    }
    for i in 0..16u32 {
        assert_eq!(rw(mem, DST + 4 * i), 0xA500_0000 + i, "word {i} copied");
    }
    assert_eq!(rw(mem, TABLE + 8) & 7, 0, "mode STOP written back");
    assert_eq!(rw(mem, UDMA0 + CHIS) & 1, 1, "channel flagged");
}

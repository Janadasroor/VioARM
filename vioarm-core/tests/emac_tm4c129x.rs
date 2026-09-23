//! Board-level `EMAC0` TX proof on `tm4c129xnczad`: a real transmit
//! descriptor ring in SRAM walks the bus-coupled DMA path — descriptor
//! read, buffer read, ownership write-back, `TI` — with no test doubles.
//!
//! RX is proven at lib level (`device::ti::emac`, mailbox-seeded);
//! frames have no byte-channel framing for host injection, so there is
//! no board-level RX inject path (same boundary as CAN/USB frame drains).

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const EMAC0: u32 = 0x400E_C000;
const DMAOPMODE: u32 = 0xC18;
const TXPOLLD: u32 = 0xC04;
const TXDLADDR: u32 = 0xC10;
const DMARIS: u32 = 0xC14;
const ST: u32 = 1 << 13;
const TI: u32 = 1 << 0;

const SRAM: u32 = 0x2000_0000;
const DESC: u32 = 0x2000_0000;
const BUF: u32 = 0x2000_0100;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

#[test]
fn emac_tx_walks_the_bus() {
    let mut m = Machine::armv7m(board::tm4c129xnczad());
    let mem = m.core.mem();
    // Frame bytes in SRAM.
    ww(mem, BUF, 0xBEAD_DEAD);
    // Descriptor: OWN|LS|FS, TBS1=4, buffer, ring-next.
    ww(mem, DESC, 0xB000_0000);
    ww(mem, DESC + 4, 4);
    ww(mem, DESC + 8, BUF);
    ww(mem, DESC + 12, DESC);
    // Enable TX, point the ring, poll.
    ww(mem, EMAC0 + DMAOPMODE, ST);
    ww(mem, EMAC0 + TXDLADDR, DESC);
    ww(mem, EMAC0 + TXPOLLD, 0x01);
    for _ in 0..20 {
        mem.tick_devices(1);
    }
    assert_eq!(rw(mem, EMAC0 + DMARIS) & TI, TI, "transmit completes");
    assert_eq!(rw(mem, DESC) & 0x8000_0000, 0, "ownership home");
}

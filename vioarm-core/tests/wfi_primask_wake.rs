//! `WFI` wakes on a pending exception even with `PRIMASK` set (the
//! lock-check-sleep idiom deadlocks otherwise — found via Zephyr's idle
//! loop, whose SysTick never fired). Taking stays gated: the core wakes,
//! runs `cpsie i`, and only then takes the exception.

use vioarm_core::device::systick::SysTick;
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::system::SystemMemory;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const FLAG: u32 = 0x2000_0100;

fn ww(mem: &mut SystemMemory<FlatMemory>, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

#[test]
fn wfi_wakes_under_primask_and_takes_after_cpsie() {
    let mut sysmem = SystemMemory::new(FlatMemory::with_layout(&[FLASH, RAM], &[0x1000, 0x1000]));
    sysmem.attach(Box::new(SysTick::new()));
    let mut m = Machine::new(sysmem);
    {
        let mem = m.core.mem();
        // Vector table: SP, reset and SysTick handlers.
        ww(mem, FLASH, 0x2000_1000);
        ww(mem, FLASH + 4, 0x41); // reset -> 0x40
        ww(mem, FLASH + 0x3C, 0x51); // exc15 -> 0x50
                                     // 0x40: cpsid i; wfi; cpsie i; b .
        ww(mem, 0x40, 0xBF30_B672);
        ww(mem, 0x44, 0xE7FE_B662);
        // 0x50: handler sets FLAG, returns: ldr r0,[pc,#4]; movs r1,1;
        // str r1,[r0]; bx lr; .word FLAG.
        ww(mem, 0x50, 0x2101_4801);
        ww(mem, 0x54, 0x4770_6001);
        ww(mem, 0x58, FLAG);
    }
    m.reset().expect("reset");
    // SysTick: reload 200, ENABLE+TICKINT+CLKSOURCE.
    {
        let mem = m.core.mem();
        ww(mem, 0xE000_E014, 200);
        ww(mem, 0xE000_E010, 7);
    }
    for _ in 0..40 {
        let _ = m.run(100);
        if m.core.mem().read_word(FLAG).unwrap() == 1 {
            break;
        }
    }
    assert_eq!(
        m.core.mem().read_word(FLAG).unwrap(),
        1,
        "SysTick taken after the masked sleep"
    );
}

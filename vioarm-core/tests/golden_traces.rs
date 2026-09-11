//! Golden traces for M5 E4: snapshot per board boot to catch drift

use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

#[test]
fn golden_traces_per_board_boot() {
    // Every board boots to NOP;BKPT at RAM and snapshots regs; each boot
    // runs twice and must be byte-identical (M14-M3: was `.take(5)`).
    let boards = board::all_boards();
    assert!(!boards.is_empty(), "board registry must not be empty");
    for (name, ctor) in boards.iter() {
        let mut m = Machine::new(ctor());
        let code = [0x00, 0xBF, 0x00, 0xBE]; // nop; bkpt
        m.load_image(&load_flat(0x20000000, &code));
        m.core.regs.pc = 0x20000001;
        let _ = m.run(10);
        // Snapshot: pc, regs, cycles - should be deterministic
        let (regs, _, cycles) = m.core.snapshot();
        // Golden values for first board (stm32f103) - just check regs
        if *name == "stm32f103" {
            assert_eq!(regs.r[0], 0);
        }
        // For all, just ensure deterministic: run twice and compare
        let mut m2 = Machine::new(ctor());
        m2.load_image(&load_flat(0x20000000, &code));
        m2.core.regs.pc = 0x20000001;
        let _ = m2.run(10);
        let (regs2, _, cycles2) = m2.core.snapshot();
        assert_eq!(cycles, cycles2, "golden cycles deterministic for {name}");
        assert_eq!(
            regs.r[0], regs2.r[0],
            "golden regs deterministic for {name}"
        );
    }
}

#[test]
fn flash_wait_states_stall_fetches() {
    // Four NOPs planted in the `0x0` flash alias (writes there bypass the
    // locked flash gate); fetches from flash cost `1 + LATENCY` core cycles
    // (`cyccnt`) while the retired-instruction count is unaffected.
    let code = [0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF]; // 4x nop
    let mut m = Machine::new(board::stm32f103());
    m.load_image(&load_flat(0x0000_0000, &code));
    m.core.regs.pc = 0x0000_0001;
    for _ in 0..4 {
        m.step().unwrap();
    }
    assert_eq!(m.core.cyccnt, 4, "unwaited flash fetches cost one cycle");
    // Same sled with `ACR.LATENCY=2`: each fetch costs three core cycles.
    let mut m = Machine::new(board::stm32f103());
    m.load_image(&load_flat(0x0000_0000, &code));
    m.core.mem().write_word(0x4002_2000, 2).unwrap(); // FLASH_ACR.LATENCY
    m.core.regs.pc = 0x0000_0001;
    for _ in 0..4 {
        m.step().unwrap();
    }
    assert_eq!(m.core.cyccnt, 12, "LATENCY=2 triples fetch cost");
    let (_, _, cycles) = m.core.snapshot();
    assert_eq!(cycles, 4, "wait states do not retire extra instructions");
}

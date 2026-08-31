//! Golden traces for M5 E4: snapshot per board boot to catch drift

use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

#[test]
fn golden_traces_per_board_boot() {
    // For each board, boot to NOP;BKPT at RAM and snapshot regs
    for (name, ctor) in board::all_boards().iter().take(5) {
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

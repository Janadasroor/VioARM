//! Phase 0 baseline audit (boards-roadmap): per-board peripheral tier matrix.
//!
//! For each of the 21 SVD family boards: build the board, walk its SVD
//! table, and classify every peripheral window as `behavioral` (a real
//! device is mounted) or `latch` (pure `NoBehavior` register file).
//! Prints a Markdown matrix to stdout; redirect into the audit record.
//!
//! Tier note: `behavioral` here is T2-or-better. The T2/T3 split (proven
//! with vectors/firmware) is tracked separately in `docs/src/scoreboard.md`
//! and per-driver test suites — this audit answers "is there anything
//! behind the door at all".

use std::collections::HashMap;
use vioarm_core::board;

const FAMILIES: [&str; 21] = [
    "stm32f030",
    "stm32f031",
    "stm32f042",
    "stm32f072",
    "stm32f091",
    "stm32f0x0",
    "stm32f0x1",
    "stm32f0x2",
    "stm32f0x8",
    "stm32f0xx",
    "stm32f100",
    "stm32f101",
    "stm32f102",
    "stm32f103",
    "stm32f105",
    "stm32f107",
    "stm32f303xe",
    "stm32f407",
    "stm32g474xx",
    "stm32h743",
    "stm32l476",
];

fn main() {
    let ctors: HashMap<&str, fn() -> board::BoardMemory> =
        board::all_boards().into_iter().collect();
    let tables: HashMap<&str, _> = board::all_chip_tables().into_iter().collect();
    println!("# Phase 0 audit: per-board peripheral tiers");
    println!();
    println!("`behavioral` = real device mounted (T2-or-better); `latch` = pure `NoBehavior` window (T1).");
    println!();
    for fam in FAMILIES {
        audit_one(&ctors, &tables, fam);
    }
    // TI track (`ti-roadmap.md` P1): the Tiva C donor audits the same way.
    audit_one(&ctors, &tables, "tm4c123gh6pm");
    audit_one(&ctors, &tables, "tm4c123gh6pz");
    // TM4C129 donor (`ti-roadmap.md` 129 P1): same treatment.
    audit_one(&ctors, &tables, "tm4c1294ncpdt");
    audit_one(&ctors, &tables, "tm4c129xnczad");
}

fn audit_one(
    ctors: &HashMap<&str, fn() -> board::BoardMemory>,
    tables: &HashMap<&str, &[vioarm_core::device::svd::PeripheralDef]>,
    fam: &str,
) {
    let ctor = ctors[fam];
    let mem = ctor();
    // Mounted behavioral windows: every (base, len) claimed by a real
    // device. Matching by address coverage (not by name) so instances
    // that share one driver name (`I2C1`/`I2C2` both live behind
    // `I2c`) still classify honestly.
    let mut beh_windows: Vec<(u32, u32)> = Vec::new();
    for d in mem.devices() {
        if d.behavior_kind() != "behavioral" {
            continue;
        }
        for r in d.ranges() {
            beh_windows.push((r.base, r.len));
        }
    }
    let table = tables[fam];
    let mut beh_names: Vec<&str> = Vec::new();
    let mut latch_names: Vec<&str> = Vec::new();
    let mut core_names: Vec<&str> = Vec::new();
    for p in table {
        // System block (NVIC/SCB/SysTick/...) is modeled in the Core,
        // not on the bus — neither latch nor missing.
        if p.base >= 0xE000_0000 {
            core_names.push(p.name);
            continue;
        }
        let covered = beh_windows
            .iter()
            .any(|(b, l)| p.base >= *b && p.base < b + l);
        if covered {
            beh_names.push(p.name);
        } else {
            latch_names.push(p.name);
        }
    }
    let bus_total = (beh_names.len() + latch_names.len()) as u32;
    println!(
        "## {fam}: {b}/{t} behavioral ({c} core-owned)",
        b = beh_names.len(),
        t = bus_total,
        c = core_names.len()
    );
    println!();
    println!(
        "behavioral ({n}): {list}",
        n = beh_names.len(),
        list = beh_names.join(" ")
    );
    println!();
    println!(
        "latch ({n}): {list}",
        n = latch_names.len(),
        list = latch_names.join(" ")
    );
    println!();
}

//! `vioarm-svd`: turn a CMSIS `*.svd` device description into the register
//! tables `vioarm-core` consumes.
//!
//! Defaults point at the workspace's STM32F103 pack; pass two paths to emit a
//! different device to a different output file.

mod emit;
mod svd;

use std::env;
use std::fs;

const DEFAULT_SVD: &str = "packs/Keil.STM32F1xx_DFP/SVD/STM32F103xx.svd";
const DEFAULT_OUT: &str = "vioarm-core/src/device/svd/stm32f103.rs";

fn main() {
    let args: Vec<String> = env::args().collect();
    let (svd_path, out_path) = match args.as_slice() {
        [_] | [] => (DEFAULT_SVD, DEFAULT_OUT),
        [_, svd, out] => (svd.as_str(), out.as_str()),
        [_, ..] => {
            eprintln!("usage: vioarm-svd [<svd> <out.rs>]");
            std::process::exit(1);
        }
    };

    let xml = fs::read_to_string(svd_path)
        .unwrap_or_else(|e| die(&format!("cannot read {svd_path}: {e}")));
    let model = svd::parse(&xml).unwrap_or_else(|e| die(&e.to_string()));
    let code = emit::emit(&model, svd_path, out_path);
    fs::write(out_path, code).unwrap_or_else(|e| die(&format!("cannot write {out_path}: {e}")));

    let periphs = model.peripherals.len();
    let regs: usize = model.peripherals.iter().map(|p| p.registers.len()).sum();
    let fields: usize = model
        .peripherals
        .iter()
        .flat_map(|p| p.registers.iter())
        .map(|r| r.fields.len())
        .sum();
    println!(
        "wrote {out_path}: {periphs} peripherals, {regs} registers, {fields} fields (device {})",
        model.device
    );
}

fn die(msg: &str) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every committed table must regenerate byte-for-byte from its SVD
    /// source. This is the codegen-hardening invariant: the generator is
    /// deterministic (no timestamps, no map iteration order leaking through),
    /// and the checked-in tables stay in lockstep with the packs. A drift here
    /// means the tables were hand-edited or `emit` gained nondeterminism.
    ///
    /// The leading `//!` doc header embeds the exact CLI `source`/`out` path
    /// strings it was produced with (which legitimately differ between
    /// machines and invocations), so comparison starts at the `use super::`
    /// import that opens the actual table.
    #[test]
    fn committed_tables_regenerate_identically() {
        let workspace = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("vioarm-svd should sit one level under the workspace root");
        let pairs: &[(&str, &str)] = &[
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F030.svd",
                "vioarm-core/src/device/svd/stm32f030.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F031x.svd",
                "vioarm-core/src/device/svd/stm32f031.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F042x.svd",
                "vioarm-core/src/device/svd/stm32f042.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/SVD/STM32F072x.svd",
                "vioarm-core/src/device/svd/stm32f072.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F091x.svd",
                "vioarm-core/src/device/svd/stm32f091.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F0x0.svd",
                "vioarm-core/src/device/svd/stm32f0x0.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F0x1.svd",
                "vioarm-core/src/device/svd/stm32f0x1.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F0x2.svd",
                "vioarm-core/src/device/svd/stm32f0x2.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F0x8.svd",
                "vioarm-core/src/device/svd/stm32f0x8.rs",
            ),
            (
                "packs/Keil.STM32F0xx_DFP/CMSIS/SVD/STM32F0xx.svd",
                "vioarm-core/src/device/svd/stm32f0xx.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F100xx.svd",
                "vioarm-core/src/device/svd/stm32f100.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F101xx.svd",
                "vioarm-core/src/device/svd/stm32f101.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F102xx.svd",
                "vioarm-core/src/device/svd/stm32f102.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F103xx.svd",
                "vioarm-core/src/device/svd/stm32f103.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F105xx.svd",
                "vioarm-core/src/device/svd/stm32f105.rs",
            ),
            (
                "packs/Keil.STM32F1xx_DFP/SVD/STM32F107xx.svd",
                "vioarm-core/src/device/svd/stm32f107.rs",
            ),
            (
                "packs/Keil.STM32F3xx_DFP/SVD/STM32F303xE.svd",
                "vioarm-core/src/device/svd/stm32f303xe.rs",
            ),
            (
                "packs/STM32F407.svd",
                "vioarm-core/src/device/svd/stm32f407.rs",
            ),
            (
                "packs/Keil.STM32L4xx_DFP/SVD/STM32L476.svd",
                "vioarm-core/src/device/svd/stm32l476.rs",
            ),
            (
                "packs/Keil.STM32G4xx_DFP/SVD/STM32G474xx.svd",
                "vioarm-core/src/device/svd/stm32g474xx.rs",
            ),
            (
                "packs/Keil.STM32H7xx_DFP/SVD/STM32H743.svd",
                "vioarm-core/src/device/svd/stm32h743.rs",
            ),
            (
                "packs/Keil.STM32F2xx_DFP/CMSIS/SVD/STM32F207.svd",
                "vioarm-core/src/device/svd/stm32f207.rs",
            ),
            (
                "packs/Keil.STM32F2xx_DFP/CMSIS/SVD/STM32F217.svd",
                "vioarm-core/src/device/svd/stm32f217.rs",
            ),
            (
                "packs/Keil.STM32F7xx_DFP/CMSIS/SVD/STM32F722.svd",
                "vioarm-core/src/device/svd/stm32f722.rs",
            ),
            (
                "packs/Keil.STM32F7xx_DFP/CMSIS/SVD/STM32F730.svd",
                "vioarm-core/src/device/svd/stm32f730.rs",
            ),
            (
                "packs/Keil.STM32F7xx_DFP/CMSIS/SVD/STM32F746.svd",
                "vioarm-core/src/device/svd/stm32f746.rs",
            ),
            (
                "packs/Keil.STM32F7xx_DFP/CMSIS/SVD/STM32F767.svd",
                "vioarm-core/src/device/svd/stm32f767.rs",
            ),
            (
                "packs/Keil.STM32G0xx_DFP/CMSIS/SVD/STM32G031.svd",
                "vioarm-core/src/device/svd/stm32g031.rs",
            ),
            (
                "packs/Keil.STM32G0xx_DFP/CMSIS/SVD/STM32G050.svd",
                "vioarm-core/src/device/svd/stm32g050.rs",
            ),
            (
                "packs/Keil.STM32G0xx_DFP/CMSIS/SVD/STM32G061.svd",
                "vioarm-core/src/device/svd/stm32g061.rs",
            ),
            (
                "packs/Keil.STM32G0xx_DFP/CMSIS/SVD/STM32G081.svd",
                "vioarm-core/src/device/svd/stm32g081.rs",
            ),
            (
                "packs/Keil.STM32G0xx_DFP/CMSIS/SVD/STM32G0C1.svd",
                "vioarm-core/src/device/svd/stm32g0c1.rs",
            ),
        ];
        let mut checked = 0;
        for (svd_rel, out_rel) in pairs {
            let svd_path = workspace.join(svd_rel);
            let out_path = workspace.join(out_rel);
            // Packs are gitignored: on machines without them (CI) skip the
            // pair instead of failing; `xtask packs --fetch` restores it.
            if !svd_path.is_file() {
                eprintln!("skipping {svd_rel}: packs not vendored here");
                continue;
            }
            checked += 1;
            let xml = fs::read_to_string(&svd_path)
                .unwrap_or_else(|e| panic!("cannot read {}: {e}", svd_path.display()));
            let model =
                svd::parse(&xml).unwrap_or_else(|e| panic!("parse {}: {e}", svd_path.display()));
            let generated = emit::emit(&model, svd_rel, out_rel);
            let committed = fs::read_to_string(&out_path)
                .unwrap_or_else(|e| panic!("cannot read {}: {e}", out_path.display()));
            let generated_body = table_body(&generated);
            let committed_body = table_body(&committed);
            assert_eq!(
                generated_body,
                committed_body,
                "{out_rel} is stale: regenerate it with `cargo run -p vioarm-svd -- {svd_rel} {out_rel}`"
            );
        }
        eprintln!("checked {checked} committed tables against packs");
    }

    /// Strip the `//!` header so the regenerated table can be compared with a
    /// committed one regardless of the path strings the header embeds.
    fn table_body(src: &str) -> &str {
        let marker = "use super::{Access, Device, FieldDef, IrqDef, PeripheralDef, RegisterDef};";
        src.split_once(marker).map_or(src, |(_, body)| body)
    }
}

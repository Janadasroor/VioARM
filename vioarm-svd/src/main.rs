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
    let code = emit::emit(&model, svd_path);
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

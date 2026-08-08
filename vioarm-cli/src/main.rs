//! VioARM CLI: point a binary or ELF at the ARM Cortex-M core, run it, print
//! the halt state.

use std::path::PathBuf;

use clap::Parser;

use vioarm_core::error::HaltCause;
use vioarm_core::loader;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Raw binary image to load.
    #[arg(short, long)]
    bin: Option<PathBuf>,

    /// ELF image to load.
    #[arg(short, long)]
    elf: Option<PathBuf>,

    /// Load base address for `--bin`.
    #[arg(long, default_value_t = 0)]
    base: u32,

    /// Instruction budget (how far a runaway loop is allowed to go).
    #[arg(long, default_value_t = 1_000_000)]
    budget: u64,
}

fn main() {
    let args = Args::parse();

    let img = if let Some(path) = args.elf {
        let bytes = std::fs::read(path).expect("read elf file");
        loader::load_elf(&bytes).expect("parse elf")
    } else if let Some(path) = args.bin {
        let bytes = std::fs::read(path).expect("read binary file");
        loader::load_flat(args.base, &bytes)
    } else {
        eprintln!("error: provide --bin or --elf");
        std::process::exit(2);
    };

    // Simple layout: 64 KiB of RAM at 0x2000_0000, plus a flash bank at 0.
    let mut m = Machine::new(FlatMemory::with_layout(
        &[0, 0x2000_0000],
        &[0x4000, 0x10000],
    ));
    m.load_image(&img);

    if let Err(cause) = m.reset() {
        eprintln!("reset failed: {cause}");
        std::process::exit(3);
    }

    match m.run(args.budget) {
        Err(HaltCause::CycleBudgetExhausted) => {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "exhausted cycle budget after {cycles} instructions: pc={:#010x} sp={:#010x} xpsr={:#010x}",
                regs.pc,
                regs.sp,
                xpsr.get()
            );
        }
        Err(cause) => {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "halted at {cause} after {cycles} instructions: pc={:#010x} sp={:#010x} r0={:#010x} xpsr={:#010x}",
                regs.pc,
                regs.sp,
                regs.r[0],
                xpsr.get()
            );
        }
        Ok(()) => unreachable!(),
    }
}

//! `VioARM` CLI: point a binary, `ELF`, or Intel HEX image at the ARM
//! `Cortex-M` core, run it, print the halt state (and optionally a per-step
//! trace).

use std::path::PathBuf;

use clap::Parser;

use vioarm_core::board;
use vioarm_core::error::HaltCause;
use vioarm_core::loader;
use vioarm_core::machine::Machine;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Raw binary image to load.
    #[arg(long)]
    bin: Option<PathBuf>,

    /// ELF image to load.
    #[arg(long)]
    elf: Option<PathBuf>,

    /// Intel HEX image to load.
    #[arg(long)]
    hex: Option<PathBuf>,

    /// Load base address for `--bin`. Accepts `0x`-prefixed hex or decimal.
    #[arg(long, default_value_t = 0, value_parser = parse_u32)]
    base: u32,

    /// Instruction budget (how far a runaway loop is allowed to go).
    #[arg(long, default_value_t = 1_000_000)]
    budget: u64,

    /// Print one line of register state per retired instruction.
    #[arg(long)]
    trace: bool,

    /// Print image layout and exit without running.
    #[arg(long)]
    info: bool,
}

fn parse_u32(s: &str) -> Result<u32, String> {
    let (radix, digits) = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .map_or((10, s), |rest| (16, rest));
    u32::from_str_radix(digits, radix).map_err(|e| e.to_string())
}

fn main() {
    let args = Args::parse();
    let img = if let Some(path) = args.elf {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!("error: cannot read {}: {e}", path.display());
            std::process::exit(2);
        });
        loader::load_elf(&bytes).unwrap_or_else(|e| {
            eprintln!("error: {}: {e}", path.display());
            std::process::exit(2);
        })
    } else if let Some(path) = args.hex {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!("error: cannot read {}: {e}", path.display());
            std::process::exit(2);
        });
        loader::load_hex(&bytes).unwrap_or_else(|e| {
            eprintln!("error: {}: {e}", path.display());
            std::process::exit(2);
        })
    } else if let Some(path) = args.bin {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!("error: cannot read {}: {e}", path.display());
            std::process::exit(2);
        });
        loader::load_flat(args.base, &bytes)
    } else {
        eprintln!("error: provide --bin, --elf, or --hex");
        std::process::exit(2);
    };

    if args.info {
        for (addr, bytes) in &img.segments {
            let end = u32::try_from(bytes.len()).map_or(u32::MAX, |len| addr.saturating_add(len));
            println!("segment: {addr:#010x}..{end:#010x} ({} bytes)", bytes.len());
        }
        if let Some(entry) = img.entry {
            println!("entry: {entry:#010x}");
        }
        return;
    }

    // The generic virtual board: flash + RAM + GPIO + UART + SysTick.
    let mut m = Machine::new(board::virtual_board());
    m.load_image(&img);

    if let Err(cause) = m.reset() {
        eprintln!("reset failed: {cause}");
        std::process::exit(3);
    }

    if args.trace {
        run_traced(&mut m, args.budget);
    } else {
        run_quiet(&mut m, args.budget);
    }

    for device in m.core.mem().devices_mut() {
        let tx = device.drain_output();
        if !tx.is_empty() {
            println!("uart: {}", String::from_utf8_lossy(&tx));
        }
    }
}

fn run_quiet(m: &mut Machine<board::BoardMemory>, budget: u64) {
    match m.run(budget) {
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

fn run_traced(m: &mut Machine<board::BoardMemory>, budget: u64) {
    for cycle in 0..budget {
        let (regs, xpsr, _) = m.core.snapshot();
        println!(
            "{cycle:6} pc={:#010x} sp={:#010x} r0={:#010x} r1={:#010x} xpsr={:#010x}",
            regs.pc,
            regs.sp,
            regs.r[0],
            regs.r[1],
            xpsr.get()
        );
        if let Err(cause) = m.step() {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "halted at {cause} after {cycles} instructions: pc={:#010x} sp={:#010x} r0={:#010x} xpsr={:#010x}",
                regs.pc,
                regs.sp,
                regs.r[0],
                xpsr.get()
            );
            return;
        }
    }
    let (regs, xpsr, cycles) = m.core.snapshot();
    println!(
        "exhausted cycle budget after {cycles} instructions: pc={:#010x} sp={:#010x} xpsr={:#010x}",
        regs.pc,
        regs.sp,
        xpsr.get()
    );
}

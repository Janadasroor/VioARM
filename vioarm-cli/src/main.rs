//! `VioARM` CLI — cool terminal with subcommands, ANSI, and REPL.

use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};
use colored::Colorize;

use vioarm_core::board;
use vioarm_core::error::HaltCause;
use vioarm_core::loader::{self, LoadedImage};
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

/// The concrete boards the CLI can boot (21 SVD tables + 500 Dnames via core_only, 13 full).
#[derive(Clone, Copy, Debug, ValueEnum, PartialEq, Eq)]
enum Board {
    Virtual,
    Stm32f100,
    Stm32f101,
    Stm32f102,
    Stm32f103,
    Stm32f105,
    Stm32f107,
    Stm32f401,
    Stm32f405,
    Stm32f411,
    Stm32f412,
    Stm32f415,
    Stm32f429,
    Stm32f437,
    Stm32f439,
    Stm32f469,
    Stm32f746,
    Stm32f756,
    Stm32h750,
    Stm32f769,
    Stm32f407,
    Stm32f072,
    Stm32f030,
    Stm32f031,
    Stm32f042,
    Stm32f0x0,
    Stm32f091,
    Stm32f0x1,
    Stm32f0x2,
    Stm32f0x8,
    Stm32f0xx,
    Stm32f303xe,
    Stm32g474xx,
    Stm32l476,
    Stm32h743,
}

impl Board {
    fn mem(self) -> board::BoardMemory {
        match self {
            Board::Virtual => board::virtual_board(),
            Board::Stm32f100 => board::stm32f100(),
            Board::Stm32f101 => board::stm32f101(),
            Board::Stm32f102 => board::stm32f102(),
            Board::Stm32f103 => board::stm32f103(),
            Board::Stm32f105 => board::stm32f105(),
            Board::Stm32f107 => board::stm32f107(),
            Board::Stm32f401 => board::stm32f401(),
            Board::Stm32f405 => board::stm32f405(),
            Board::Stm32f411 => board::stm32f411(),
            Board::Stm32f412 => board::stm32f412(),
            Board::Stm32f415 => board::stm32f415(),
            Board::Stm32f429 => board::stm32f429(),
            Board::Stm32f437 => board::stm32f437(),
            Board::Stm32f439 => board::stm32f439(),
            Board::Stm32f469 => board::stm32f469(),
            Board::Stm32f746 => board::stm32f746(),
            Board::Stm32f756 => board::stm32f756(),
            Board::Stm32h750 => board::stm32h750(),
            Board::Stm32f769 => board::stm32f769(),
            Board::Stm32f407 => board::stm32f407(),
            Board::Stm32f072 => board::stm32f072(),
            Board::Stm32f030 => board::stm32f030(),
            Board::Stm32f031 => board::stm32f031(),
            Board::Stm32f042 => board::stm32f042(),
            Board::Stm32f0x0 => board::stm32f0x0(),
            Board::Stm32f091 => board::stm32f091(),
            Board::Stm32f0x1 => board::stm32f0x1(),
            Board::Stm32f0x2 => board::stm32f0x2(),
            Board::Stm32f0x8 => board::stm32f0x8(),
            Board::Stm32f0xx => board::stm32f0xx(),
            Board::Stm32f303xe => board::stm32f303xe(),
            Board::Stm32g474xx => board::stm32g474xx(),
            Board::Stm32l476 => board::stm32l476(),
            Board::Stm32h743 => board::stm32h743(),
        }
    }
    fn name(self) -> &'static str {
        match self {
            Board::Virtual => "virtual",
            Board::Stm32f100 => "stm32f100",
            Board::Stm32f101 => "stm32f101",
            Board::Stm32f102 => "stm32f102",
            Board::Stm32f103 => "stm32f103",
            Board::Stm32f105 => "stm32f105",
            Board::Stm32f107 => "stm32f107",
            Board::Stm32f401 => "stm32f401",
            Board::Stm32f405 => "stm32f405",
            Board::Stm32f411 => "stm32f411",
            Board::Stm32f412 => "stm32f412",
            Board::Stm32f415 => "stm32f415",
            Board::Stm32f429 => "stm32f429",
            Board::Stm32f437 => "stm32f437",
            Board::Stm32f439 => "stm32f439",
            Board::Stm32f469 => "stm32f469",
            Board::Stm32f746 => "stm32f746",
            Board::Stm32f756 => "stm32f756",
            Board::Stm32h750 => "stm32h750",
            Board::Stm32f769 => "stm32f769",
            Board::Stm32f407 => "stm32f407",
            Board::Stm32f072 => "stm32f072",
            Board::Stm32f030 => "stm32f030",
            Board::Stm32f031 => "stm32f031",
            Board::Stm32f042 => "stm32f042",
            Board::Stm32f0x0 => "stm32f0x0",
            Board::Stm32f091 => "stm32f091",
            Board::Stm32f0x1 => "stm32f0x1",
            Board::Stm32f0x2 => "stm32f0x2",
            Board::Stm32f0x8 => "stm32f0x8",
            Board::Stm32f0xx => "stm32f0xx",
            Board::Stm32f303xe => "stm32f303xe",
            Board::Stm32g474xx => "stm32g474xx",
            Board::Stm32l476 => "stm32l476",
            Board::Stm32h743 => "stm32h743",
        }
    }
}

fn parse_u32(s: &str) -> Result<u32, String> {
    let (radix, digits) = s
        .strip_prefix("0x")
        .or_else(|| s.strip_prefix("0X"))
        .map_or((10, s), |rest| (16, rest));
    u32::from_str_radix(digits, radix).map_err(|e| e.to_string())
}

#[derive(Parser, Debug)]
#[command(author, version, about = format!("{} — cool CLI", "VioARM".bright_cyan().bold()), long_about = None, color = clap::ColorChoice::Auto)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
    // Legacy flags for backward compat (no subcommand => run)
    #[arg(long)]
    bin: Option<PathBuf>,
    #[arg(long)]
    elf: Option<PathBuf>,
    #[arg(long)]
    hex: Option<PathBuf>,
    #[arg(long, value_enum, default_value_t = Board::Virtual)]
    board: Board,
    #[arg(long, default_value_t = 0, value_parser = parse_u32)]
    base: u32,
    #[arg(long, default_value_t = 1_000_000)]
    budget: u64,
    #[arg(long)]
    trace: bool,
    #[arg(long)]
    info: bool,
    #[arg(long)]
    gdb: Option<u16>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Run firmware
    Run {
        /// Image path (.bin/.elf/.hex auto-detected)
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        #[arg(long, default_value_t = 1_000_000, alias = "max-cycles")]
        budget: u64,
    },
    /// Trace cycle-by-cycle
    Trace {
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        #[arg(long, default_value_t = 1_000_000, alias = "max-cycles")]
        budget: u64,
    },
    /// Show image layout (segments, entry)
    Info {
        image: PathBuf,
    },
    /// List all boards (21 SVD tables + 500 Dnames)
    Boards,
    /// Instant terminal docs
    Docs {
        /// Topic: boards, periphs, arch, dma, tim, adc, etc.
        topic: Option<String>,
    },
    /// Interactive debug REPL
    Debug {
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
    },
    /// Serve GDB RSP on TCP port (target remote :<port>)
    Gdb {
        image: PathBuf,
        #[arg(long, default_value_t = 3333)]
        port: u16,
        #[arg(long, value_enum, default_value_t = Board::Virtual)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
    },
}

fn banner() {
    println!(
        "{} {} {}",
        "VioARM".bright_cyan().bold(),
        format!("v{}", env!("CARGO_PKG_VERSION")).dimmed(),
        "— cycle-accurate Cortex-M ISS".dimmed()
    );
    println!(
        "{}",
        "  boards: 21 SVD → 500 Dnames | periphs: 197 | 615 lib tests".dimmed()
    );
}

fn load_image_auto(path: &PathBuf, base: u32) -> (LoadedImage, String) {
    let bytes = std::fs::read(path).unwrap_or_else(|e| {
        eprintln!("{} cannot read {}: {e}", "error:".red().bold(), path.display());
        std::process::exit(2);
    });
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("").to_ascii_lowercase();
    let kind = if ext == "elf" { "elf" } else if ext == "hex" { "hex" } else if ext == "bin" { "bin" } else {
        // auto-detect by magic
        if bytes.starts_with(b"\x7fELF") { "elf" } else if bytes.starts_with(b":") { "hex" } else { "bin" }
    };
    let img = match kind {
        "elf" => loader::load_elf(&bytes).unwrap_or_else(|e| {
            eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
            std::process::exit(2);
        }),
        "hex" => loader::load_hex(&bytes).unwrap_or_else(|e| {
            eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
            std::process::exit(2);
        }),
        _ => loader::load_flat(base, &bytes),
    };
    (img, kind.to_string())
}

fn print_info(img: &LoadedImage) {
    for (addr, bytes) in &img.segments {
        let end = u32::try_from(bytes.len()).map_or(u32::MAX, |len| addr.saturating_add(len));
        println!(
            "{} {addr:#010x}..{end:#010x} ({} bytes)",
            "segment:".bright_green().bold(),
            bytes.len()
        );
    }
    if let Some(entry) = img.entry {
        println!("{} {entry:#010x}", "entry:".bright_green().bold());
    }
}

fn machine_for(board: Board, img: &LoadedImage) -> Machine<board::BoardMemory> {
    let mem = board.mem();
    let mut m = Machine::new(mem);
    m.load_image(img);
    if let Err(cause) = m.reset() {
        eprintln!("{} reset failed: {cause}", "error:".red().bold());
        std::process::exit(3);
    }
    m
}

fn run_quiet(m: &mut Machine<board::BoardMemory>, budget: u64) {
    match m.run(budget) {
        Err(HaltCause::CycleBudgetExhausted) => {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "{} cycle budget after {cycles} insn: {}={:#010x} {}={:#010x} {}={:#010x}",
                "exhausted".yellow().bold(),
                "pc".bright_cyan(),
                regs.pc,
                "sp".bright_cyan(),
                regs.sp,
                "xpsr".bright_cyan(),
                xpsr.get()
            );
        }
        Err(cause) => {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "{} at {} after {cycles} insn: {}={:#010x} {}={:#010x} {}={:#010x} {}={:#010x}",
                "halted".green().bold(),
                format!("{cause}").bright_yellow(),
                "pc".bright_cyan(),
                regs.pc,
                "sp".bright_cyan(),
                regs.sp,
                "r0".bright_cyan(),
                regs.r[0],
                "xpsr".bright_cyan(),
                xpsr.get()
            );
        }
        Ok(()) => unreachable!(),
    }
    for device in m.core.mem().devices_mut() {
        let tx = device.drain_output();
        if !tx.is_empty() {
            println!("{} {}", "uart:".bright_magenta().bold(), String::from_utf8_lossy(&tx));
        }
    }
}

fn run_traced(m: &mut Machine<board::BoardMemory>, budget: u64) {
    for cycle in 0..budget {
        let (regs, xpsr, _) = m.core.snapshot();
        println!(
            "{cycle:6} {}={:#010x} {}={:#010x} {}={:#010x} {}={:#010x} {}={:#010x}",
            "pc".bright_cyan(),
            regs.pc,
            "sp".bright_cyan(),
            regs.sp,
            "r0".bright_cyan(),
            regs.r[0],
            "r1".bright_cyan(),
            regs.r[1],
            "xpsr".bright_cyan(),
            xpsr.get()
        );
        if let Err(cause) = m.step() {
            let (regs, xpsr, cycles) = m.core.snapshot();
            println!(
                "{} at {} after {cycles} insn: {}={:#010x} {}={:#010x} {}={:#010x} {}={:#010x}",
                "halted".green().bold(),
                format!("{cause}").bright_yellow(),
                "pc".bright_cyan(),
                regs.pc,
                "sp".bright_cyan(),
                regs.sp,
                "r0".bright_cyan(),
                regs.r[0],
                "xpsr".bright_cyan(),
                xpsr.get()
            );
            return;
        }
    }
    let (regs, xpsr, cycles) = m.core.snapshot();
    println!(
        "{} cycle budget after {cycles} insn: {}={:#010x} {}={:#010x} {}={:#010x}",
        "exhausted".yellow().bold(),
        "pc".bright_cyan(),
        regs.pc,
        "sp".bright_cyan(),
        regs.sp,
        "xpsr".bright_cyan(),
        xpsr.get()
    );
}

fn docs_topic(topic: Option<String>) {
    let t = topic.as_deref().unwrap_or("").to_ascii_lowercase();
    match t.as_str() {
        "" | "help" => {
            println!("{}", "VioARM docs — instant terminal docs".bright_cyan().bold());
            println!("{}", "topics: boards periphs arch dma tim adc usart spi i2c can usb eth sdio fmc h7 core".dimmed());
            println!("  {} — 21 SVD → 500 Dnames, 197 union periphs", "boards".bright_green());
            println!("  {} — GPIO/RCC/USART/TIM/ADC/SPI/I2C/CAN/DMA/USB/ETH/SDIO/FMC/H7", "periphs".bright_green());
            println!("  {} — ARMv6-M/7-M/7E-M/8M-baseline M4F M7 FPv5-D16, DWT cyccnt, MPU", "arch".bright_green());
            println!("  {} — run `vioarm boards` for 21 tables, `vioarm run <hex> --board stm32h743`", "quick".bright_yellow());
        }
        "boards" => {
            println!("{}", "Boards — 21 SVD tables → 500 Dnames (vioarm-core/src/board.rs:2119)".bright_cyan().bold());
            for (name, peris) in board::all_chip_tables() {
                let len = peris.len();
                println!("  {:12} {:3} periphs {}", name.bright_green(), len.to_string().dimmed(), format!("0x{:08x}", peris[0].base).dimmed());
            }
        }
        "dma" => println!("{}", "DMA — F1 7ch MEM2MEM 1/tick PL arbitrate, F4 8 streams MBURST/PBURST INCR4/8/16 FTH quarter/half/3q/full (dma_f4.rs:196) — per-request USART1 ch4/5".bright_green()),
        "tim" => println!("{}", "TIM — 16/32b PSC→PCLK, ARR, UIF, OCxM 000-111 PWM1/2 OPM, BDTR MOE/BKIN DTG, encoder SMS 011 Hall 100 (timer.rs:112)".bright_green()),
        "adc" => println!("{}", "ADC — F1 CR2 SWSTART, F4 dual C_ADC CDR combine, F0/H7/F3 ISR ADRDY ADEN ADSTART, JSQR/JDR JEOC, HTR/LTR AWD (adc.rs:113)".bright_green()),
        "periphs" => println!("{}", "Periphs — GPIO F1 BSRR F4 MODER, RCC HSIRDY SWS→SW, PWR WUF, RTC WPR INIT, CAN 28-bank filter, USB OTG HS FS double-buffer, ETH H7 MAC, SDIO CRC16".bright_green()),
        _ => println!("{} '{}' — try `vioarm docs help`", "unknown topic".red(), t),
    }
}

fn debug_repl(image: PathBuf, board: Board, base: u32) {
    let (img, _) = load_image_auto(&image, base);
    let mut m = machine_for(board, &img);
    println!(
        "{} {} — type {} for help",
        "VioARM debug".bright_cyan().bold(),
        format!("{} {}", board.name(), image.display()).dimmed(),
        "help".bright_green()
    );
    let (regs, xpsr, _) = m.core.snapshot();
    println!("{} pc={:#010x} sp={:#010x} xpsr={:#010x}", "attached:".dimmed(), regs.pc, regs.sp, xpsr.get());
    let mut rl = String::new();
    loop {
        print!("{}", "(vioarm) ".bright_magenta().bold());
        use std::io::Write;
        let _ = std::io::stdout().flush();
        rl.clear();
        if std::io::stdin().read_line(&mut rl).is_err() { break; }
        let cmd = rl.trim();
        match cmd {
            "help" | "h" | "?" => {
                println!("{}: step(s)/next(n) continue(c) regs(r) mem <addr> break <pc> budget <n> quit(q)", "commands".bright_green());
            }
            "step" | "s" | "n" | "next" => {
                match m.step() {
                    Ok(_) => {
                        let (regs, xpsr, cycles) = m.core.snapshot();
                        println!("{} pc={:#010x} r0={:#010x} xpsr={:#010x} cycles={cycles}", "step".dimmed(), regs.pc, regs.r[0], xpsr.get());
                    }
                    Err(e) => println!("{} {e}", "halted:".yellow(), ),
                }
            }
            "regs" | "r" => {
                let (regs, xpsr, cycles) = m.core.snapshot();
                println!("pc={:#010x} sp={:#010x} lr={:#010x} r0={:#010x} r1={:#010x} r2={:#010x} r3={:#010x} xpsr={:#010x} cycles={cycles}",
                    regs.pc, regs.sp, regs.lr, regs.r[0], regs.r[1], regs.r[2], regs.r[3], xpsr.get());
            }
            c if c.starts_with("mem ") => {
                let addr = c[4..].trim();
                if let Ok(a) = parse_u32(addr) {
                    match m.core.mem().read_word(a) {
                        Ok(v) => println!("{a:#010x}: {v:#010x}"),
                        Err(e) => println!("{} {e}", "bus error:".red()),
                    }
                } else { println!("{} mem <hex addr>", "usage:".yellow()); }
            }
            c if c.starts_with("break ") => {
                let pc = c[6..].trim();
                if let Ok(a) = parse_u32(pc) {
                    println!("{} break at {a:#010x} — run `continue` to hit BKPT", "break".bright_green());
                    // simple: run until pc
                    let mut hit = false;
                    for _ in 0..1_000_000 {
                        let (regs, _, _) = m.core.snapshot();
                        if regs.pc == a { hit = true; break; }
                        if m.step().is_err() { break; }
                    }
                    println!("{}", if hit { "hit".green() } else { "not hit in 1M".yellow() });
                }
            }
            c if c.starts_with("budget ") => {
                if let Ok(n) = c[7..].trim().parse::<u64>() {
                    match m.run(n) {
                        Err(HaltCause::CycleBudgetExhausted) => println!("{}", "budget exhausted".yellow()),
                        Err(e) => println!("{} {e}", "halted:".green()),
                        Ok(()) => {}
                    }
                }
            }
            "continue" | "c" | "run" => {
                match m.run(1_000_000) {
                    Err(HaltCause::CycleBudgetExhausted) => println!("{}", "budget exhausted — 1M".yellow()),
                    Err(e) => println!("{} {e}", "halted:".green()),
                    Ok(()) => {}
                }
            }
            "quit" | "q" | "exit" => break,
            "" => continue,
            _ => println!("{} — try help", "unknown".red()),
        }
    }
}

fn main() {
    let cli = Cli::parse();
    if let Some(cmd) = cli.command {
        match cmd {
            Commands::Run { image, board, base, budget } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img);
                banner();
                run_quiet(&mut m, budget);
            }
            Commands::Trace { image, board, base, budget } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img);
                banner();
                run_traced(&mut m, budget);
            }
            Commands::Info { image } => {
                let (img, _) = load_image_auto(&image, 0);
                print_info(&img);
            }
            Commands::Boards => {
                banner();
                println!("{}", "Boards — 21 SVD tables → 500 Dnames".bright_cyan().bold());
                for (name, peris) in board::all_chip_tables() {
                    println!("  {:12} {:3} periphs {}", name.bright_green(), peris.len().to_string().dimmed(), format!("board::{}", name).dimmed());
                }
                println!("{}", "run: vioarm run firmware.hex --board stm32h743".dimmed());
            }
            Commands::Docs { topic } => docs_topic(topic),
            Commands::Debug { image, board, base } => debug_repl(image, board, base),
            Commands::Gdb { image, port, board, base } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img);
                let listener = std::net::TcpListener::bind(("127.0.0.1", port)).unwrap_or_else(|e| {
                    eprintln!("{} cannot listen on port {port}: {e}", "error:".red().bold());
                    std::process::exit(3);
                });
                println!("{} waiting for gdb on 127.0.0.1:{port}", "gdbserver:".bright_magenta().bold());
                if let Err(e) = vioarm_gdb::serve(&mut m, &listener) {
                    eprintln!("{} gdb session error: {e}", "error:".red().bold());
                    std::process::exit(3);
                }
            }
        }
        return;
    }
    // Legacy flat flags fallback (vioarm --bin/--elf/--hex --board --trace --gdb)
    let img = if let Some(path) = cli.elf {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| { eprintln!("{} cannot read {}: {e}", "error:".red().bold(), path.display()); std::process::exit(2); });
        loader::load_elf(&bytes).unwrap_or_else(|e| { eprintln!("{} {}: {e}", "error:".red().bold(), path.display()); std::process::exit(2); })
    } else if let Some(path) = cli.hex {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| { eprintln!("{} cannot read {}: {e}", "error:".red().bold(), path.display()); std::process::exit(2); });
        loader::load_hex(&bytes).unwrap_or_else(|e| { eprintln!("{} {}: {e}", "error:".red().bold(), path.display()); std::process::exit(2); })
    } else if let Some(path) = cli.bin {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| { eprintln!("{} cannot read {}: {e}", "error:".red().bold(), path.display()); std::process::exit(2); });
        loader::load_flat(cli.base, &bytes)
    } else {
        eprintln!("{} provide --bin/--elf/--hex or subcommand `run`", "error:".red().bold());
        Cli::parse_from(["vioarm", "--help"]);
        std::process::exit(2);
    };
    if cli.info {
        for (addr, bytes) in &img.segments {
            let end = u32::try_from(bytes.len()).map_or(u32::MAX, |len| addr.saturating_add(len));
            println!("segment: {addr:#010x}..{end:#010x} ({} bytes)", bytes.len());
        }
        if let Some(entry) = img.entry { println!("entry: {entry:#010x}"); }
        return;
    }
    let mut m = machine_for(cli.board, &img);
    if let Some(port) = cli.gdb {
        let listener = std::net::TcpListener::bind(("127.0.0.1", port)).unwrap_or_else(|e| { eprintln!("{} cannot listen on port {port}: {e}", "error:".red().bold()); std::process::exit(3); });
        println!("{} waiting for gdb on 127.0.0.1:{port}", "gdbserver:".bright_magenta().bold());
        if let Err(e) = vioarm_gdb::serve(&mut m, &listener) { eprintln!("{} gdb session error: {e}", "error:".red().bold()); std::process::exit(3); }
        return;
    }
    banner();
    if cli.trace { run_traced(&mut m, cli.budget); } else { run_quiet(&mut m, cli.budget); }
    for device in m.core.mem().devices_mut() {
        let tx = device.drain_output();
        if !tx.is_empty() { println!("{} {}", "uart:".bright_magenta().bold(), String::from_utf8_lossy(&tx)); }
    }
}

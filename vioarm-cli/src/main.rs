#![warn(missing_docs)]
//! `VioARM` CLI — cool terminal with subcommands, ANSI, and REPL.

use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};
use colored::Colorize;

use vioarm_core::board;
use vioarm_core::error::HaltCause;
use vioarm_core::hooks::HookEvent;
use vioarm_core::loader::{self, LoadedImage};
use vioarm_core::machine::Machine;
use vioarm_core::mem::bus::Bus;

/// Trace output format.
#[derive(Clone, Copy, Debug, ValueEnum, PartialEq, Eq)]
enum TraceFormat {
    /// Human-readable per-cycle register snapshot (legacy).
    Text,
    /// `{"cycle":N,"pc":"0x..."}` per retired instruction, hook-driven.
    Jsonl,
    /// Value-change dump (`$date/$var/#ts`) of `pc` plus `--pin-log`
    /// pins, one row per change; opens in a waveform viewer.
    Vcd,
}

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
    Stm32f446,
    Stm32f469,
    Stm32f722,
    Stm32f746,
    Stm32f756,
    Stm32f765,
    Stm32f767,
    Stm32h7b0,
    Stm32h730,
    Stm32h743,
    Stm32h745,
    Stm32h747,
    Stm32h750,
    Stm32h753,
    Stm32f413,
    Stm32f423,
    Stm32h723,
    Stm32f479,
    Stm32g431,
    Stm32l452,
    Stm32f302,
    Stm32f334,
    Stm32l432,
    Stm32f373,
    Stm32l073,
    Stm32f358,
    Stm32l152,
    Stm32f070,
    Stm32l053,
    Stm32g070,
    Stm32f378,
    Stm32f303,
    Stm32l412,
    Stm32g031,
    Stm32g071,
    Stm32f301,
    Stm32l433,
    Stm32f410,
    Stm32g441,
    Stm32l443,
    Stm32f318,
    Stm32f051,
    Stm32f207,
    Stm32g030,
    Stm32g041,
    Stm32l010,
    Stm32l021,
    Stm32h563,
    Stm32u575,
    Stm32wb55,
    Stm32f098,
    Stm32f038,
    Stm32f048,
    Stm32f071,
    Stm32f333,
    Stm32g050,
    Stm32l011,
    Stm32l031,
    Stm32f723,
    Stm32h562,
    Stm32wle5,
    Stm32f215,
    Stm32f217,
    Stm32f328,
    Stm32g081,
    Stm32g0b1,
    Stm32l052,
    Stm32l062,
    Stm32f730,
    Stm32h503,
    Stm32u585,
    Stm32f205,
    Stm32f427,
    Stm32f398,
    Stm32g491,
    Stm32l151,
    Stm32f777,
    Stm32h735,
    Stm32u5a5,
    Stm32l162,
    Stm32wl55,
    Stm32f417,
    Stm32f732,
    Stm32g0c1,
    Stm32l431,
    Stm32h755,
    Stm32u5g9,
    Stm32wba52,
    Stm32l071,
    Stm32h7a3,
    Stm32l051,
    Stm32g061,
    Stm32g473,
    Stm32l041,
    Stm32l100,
    Stm32l471,
    Stm32h533,
    Stm32h742,
    Stm32u595,
    Stm32wle4,
    Stm32f745,
    Stm32c011,
    Stm32c031,
    Stm32u535,
    Stm32wba55,
    Stm32wl33,
    Stm32l552,
    Stm32l562,
    Stm32h7b3,
    Stm32u5f9,
    Stm32mp157,
    Stm32f103c8,
    Stm32f103rb,
    Stm32f103ze,
    Stm32f401re,
    Stm32f411re,
    Stm32f429zi,
    Stm32f746zg,
    Stm32g431rb,
    Stm32l476rg,
    Stm32h743zi,
    Stm32f030c6,
    Stm32f030cc,
    Stm32f042k6,
    Stm32f042t6,
    Stm32f072c8,
    Stm32f091rc,
    Stm32f100c8,
    Stm32f101c8,
    Stm32f102cb,
    Stm32f103c6,
    Stm32f103re,
    Stm32f105rb,
    Stm32f107vc,
    Stm32f205re,
    Stm32f207ze,
    Stm32f215re,
    Stm32f217ig,
    Stm32f301k8,
    Stm32f302cb,
    Stm32f303cc,
    Stm32f373cc,
    Stm32f378cc,
    Stm32f401cc,
    Stm32f401ce,
    Stm32f410c8,
    Stm32f412ce,
    Stm32f413ch,
    Stm32f423ih,
    Stm32f427ig,
    Stm32f429ig,
    Stm32f437ii,
    Stm32f446re,
    Stm32f469ai,
    Stm32f722ie,
    Stm32f723ie,
    Stm32f732ie,
    Stm32f745ie,
    Stm32f746be,
    Stm32f756zg,
    Stm32f765bi,
    Stm32f767zi,
    Stm32f777ai,
    Stm32g030f6,
    Stm32g031f8,
    Stm32g070rb,
    Stm32g071rb,
    Stm32g474re,
    Stm32h743ii,
    Stm32h753ii,
    Stm32l476je,
    Stm32f030k6,
    Stm32f038k6,
    Stm32f042g6,
    Stm32f051g8,
    Stm32f072rb,
    Stm32f091cb,
    Stm32f100rb,
    Stm32f101re,
    Stm32f102re,
    Stm32f103vd,
    Stm32f105re,
    Stm32f107re,
    Stm32f205rb,
    Stm32f207rb,
    Stm32f215ze,
    Stm32f217ve,
    Stm32f301c8,
    Stm32f302cc,
    Stm32f303re,
    Stm32f373cb,
    Stm32f378be,
    Stm32f401rb,
    Stm32f401ve,
    Stm32f410rb,
    Stm32f412zg,
    Stm32f413rg,
    Stm32f423ze,
    Stm32f427ai,
    Stm32f429be,
    Stm32f437ai,
    Stm32f446ze,
    Stm32f469ae,
    Stm32f722ze,
    Stm32f723ze,
    Stm32f732vg,
    Stm32f745vg,
    Stm32f746ig,
    Stm32f756ig,
    Stm32f765ig,
    Stm32f767ig,
    Stm32f777vg,
    Stm32g030k6,
    Stm32g031k8,
    Stm32g070cb,
    Stm32g071cb,
    Stm32g474ce,
    Stm32h743xi,
    Stm32h753xi,
    Stm32l476jg,
    Stm32f103x49,
    Stm32f030r8,
    Stm32f030v6,
    Stm32f038r8,
    Stm32f038v6,
    Stm32f042c8,
    Stm32f042r8,
    Stm32f051r8,
    Stm32f072r8,
    Stm32f091rb,
    Stm32f100re,
    Stm32f101rb,
    Stm32f102rb,
    Stm32f103r8,
    Stm32f105rg,
    Stm32f107rg,
    Stm32f205rg,
    Stm32f207rg,
    Stm32f215rg,
    Stm32f217rg,
    Stm32f301r8,
    Stm32f302r8,
    Stm32f303rg,
    Stm32f373rg,
    Stm32f378rg,
    Stm32f401rg,
    Stm32f410re,
    Stm32f412re,
    Stm32f413re,
    Stm32f423rg,
    Stm32f427rg,
    Stm32f429rg,
    Stm32f437rg,
    Stm32f446rg,
    Stm32f469rg,
    Stm32f722rg,
    Stm32f723rg,
    Stm32f732rg,
    Stm32f745rg,
    Stm32f746rg,
    Stm32f756rg,
    Stm32f765rg,
    Stm32f767rg,
    Stm32f777rg,
    Stm32g030c6,
    Stm32g031c6,
    Stm32g070re,
    Stm32g071re,
    Stm32g474rg,
    Stm32h743xg,
    Stm32h753xg,
    Stm32f030t6,
    Stm32f038t6,
    Stm32f042k4,
    Stm32f051k4,
    Stm32f072k8,
    Stm32f091vb,
    Stm32f100rc,
    Stm32f101rc,
    Stm32f102rc,
    Stm32f103rc,
    Stm32f105rc,
    Stm32f107rc,
    Stm32f205rc,
    Stm32f207rc,
    Stm32f215rc,
    Stm32f217rc,
    Stm32f301k6,
    Stm32f302k6,
    Stm32f303k8,
    Stm32f373k8,
    Stm32f378k8,
    Stm32f401k6,
    Stm32f410k8,
    Stm32f412k6,
    Stm32f413k6,
    Stm32f423k6,
    Stm32f427k6,
    Stm32f429k6,
    Stm32f437k6,
    Stm32f446k6,
    Stm32f469k6,
    Stm32f722k6,
    Stm32f723k6,
    Stm32f732k6,
    Stm32f745k6,
    Stm32f746k6,
    Stm32f756k6,
    Stm32f765k6,
    Stm32f767k6,
    Stm32f777k6,
    Stm32g030g4,
    Stm32g031g4,
    Stm32g070g4,
    Stm32g071g4,
    Stm32g474kb,
    Stm32h743ag,
    Stm32h753ag,
    Stm32l476ke,
    Stm32c011f6,
    Stm32c031f6,
    Stm32f030a0,
    Stm32f030a1,
    Stm32f042a0,
    Stm32f042a1,
    Stm32f072a0,
    Stm32f072a1,
    Stm32f091a0,
    Stm32f091a1,
    Stm32f100a0,
    Stm32f100a1,
    Stm32f101a0,
    Stm32f101a1,
    Stm32f102a0,
    Stm32f102a1,
    Stm32f103a0,
    Stm32f103a1,
    Stm32f105a0,
    Stm32f105a1,
    Stm32f107a0,
    Stm32f107a1,
    Stm32f205a0,
    Stm32f205a1,
    Stm32f207a0,
    Stm32f207a1,
    Stm32f215a0,
    Stm32f215a1,
    Stm32f217a0,
    Stm32f217a1,
    Stm32f301a0,
    Stm32f301a1,
    Stm32f302a0,
    Stm32f302a1,
    Stm32f303a0,
    Stm32f303a1,
    Stm32f373a0,
    Stm32f373a1,
    Stm32f378a0,
    Stm32f378a1,
    Stm32f401a0,
    Stm32f401a1,
    Stm32f410a0,
    Stm32f410a1,
    Stm32f412a0,
    Stm32f412a1,
    Stm32f413a0,
    Stm32f413a1,
    Stm32f423a0,
    Stm32f423a1,
    Stm32f427a0,
    Stm32f427a1,
    Stm32f429a0,
    Stm32f429a1,
    Stm32f437a0,
    Stm32f437a1,
    Stm32f446a0,
    Stm32f446a1,
    Stm32f469a0,
    Stm32f469a1,
    Stm32f722a0,
    Stm32f722a1,
    Stm32f723a0,
    Stm32f723a1,
    Stm32f732a0,
    Stm32f732a1,
    Stm32f745a0,
    Stm32f745a1,
    Stm32f746a0,
    Stm32f746a1,
    Stm32f756a0,
    Stm32f756a1,
    Stm32f765a0,
    Stm32f765a1,
    Stm32f767a0,
    Stm32f767a1,
    Stm32f777a0,
    Stm32f777a1,
    Stm32g030a0,
    Stm32g030a1,
    Stm32g031a0,
    Stm32g031a1,
    Stm32g070a0,
    Stm32g070a1,
    Stm32g071a0,
    Stm32g071a1,
    Stm32g474a0,
    Stm32g474a1,
    Stm32h743a0,
    Stm32h743a1,
    Stm32h753a0,
    Stm32h753a1,
    Stm32l476a0,
    Stm32l476a1,
    Stm32c011a0,
    Stm32c011a1,
    Stm32c031a0,
    Stm32c031a1,
    Stm32u575a0,
    Stm32u575a1,
    Stm32wba55a0,
    Stm32wba55a1,
    Stm32f030b0,
    Stm32f042b0,
    Stm32f072b0,
    Stm32f091b0,
    Stm32f100b0,
    Stm32f101b0,
    Stm32f102b0,
    Stm32f103b0,
    Stm32f105b0,
    Stm32f107b0,
    Stm32f205b0,
    Stm32f207b0,
    Stm32f215b0,
    Stm32f217b0,
    Stm32f301b0,
    Stm32f302b0,
    Stm32f303b0,
    Stm32f373b0,
    Stm32f378b0,
    Stm32f401b0,
    Stm32f410b0,
    Stm32f412b0,
    Stm32f413b0,
    Stm32f423b0,
    Stm32f427b0,
    Stm32f429b0,
    Stm32f437b0,
    Stm32f446b0,
    Stm32f469b0,
    Stm32f722b0,
    Stm32f723b0,
    Stm32f732b0,
    Stm32f745b0,
    Stm32f746b0,
    Stm32f756b0,
    Stm32f765b0,
    Stm32f767b0,
    Stm32f777b0,
    Stm32g030b0,
    Stm32g031b0,
    Stm32g070b0,
    Stm32g071b0,
    Stm32g474b0,
    Stm32h743b0,
    Stm32h753b0,
    Stm32l476b0,
    Stm32c011b0,
    Stm32c031b0,
    Stm32u575b0,
    Stm32wba55b0,
    Stm32f030b1,
    Stm32f042b1,
    Stm32f072b1,
    Stm32f091b1,
    Stm32f100b1,
    Stm32f101b1,
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
    Tm4c123gh6pm,
    Tm4c1294ncpdt,
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
            Board::Stm32f446 => board::stm32f446(),
            Board::Stm32f469 => board::stm32f469(),
            Board::Stm32f722 => board::stm32f722(),
            Board::Stm32f746 => board::stm32f746(),
            Board::Stm32f756 => board::stm32f756(),
            Board::Stm32f765 => board::stm32f765(),
            Board::Stm32f767 => board::stm32f767(),
            Board::Stm32h7b0 => board::stm32h7b0(),
            Board::Stm32h730 => board::stm32h730(),
            Board::Stm32h743 => board::stm32h743(),
            Board::Stm32h745 => board::stm32h745(),
            Board::Stm32h747 => board::stm32h747(),
            Board::Stm32h750 => board::stm32h750(),
            Board::Stm32h753 => board::stm32h753(),
            Board::Stm32f413 => board::stm32f413(),
            Board::Stm32f423 => board::stm32f423(),
            Board::Stm32h723 => board::stm32h723(),
            Board::Stm32f479 => board::stm32f479(),
            Board::Stm32g431 => board::stm32g431(),
            Board::Stm32l452 => board::stm32l452(),
            Board::Stm32f302 => board::stm32f302(),
            Board::Stm32f334 => board::stm32f334(),
            Board::Stm32l432 => board::stm32l432(),
            Board::Stm32f373 => board::stm32f373(),
            Board::Stm32l073 => board::stm32l073(),
            Board::Stm32f358 => board::stm32f358(),
            Board::Stm32l152 => board::stm32l152(),
            Board::Stm32f070 => board::stm32f070(),
            Board::Stm32l053 => board::stm32l053(),
            Board::Stm32g070 => board::stm32g070(),
            Board::Stm32f378 => board::stm32f378(),
            Board::Stm32f303 => board::stm32f303(),
            Board::Stm32l412 => board::stm32l412(),
            Board::Stm32g031 => board::stm32g031(),
            Board::Stm32g071 => board::stm32g071(),
            Board::Stm32f301 => board::stm32f301(),
            Board::Stm32l433 => board::stm32l433(),
            Board::Stm32f410 => board::stm32f410(),
            Board::Stm32g441 => board::stm32g441(),
            Board::Stm32l443 => board::stm32l443(),
            Board::Stm32f318 => board::stm32f318(),
            Board::Stm32f051 => board::stm32f051(),
            Board::Stm32f207 => board::stm32f207(),
            Board::Stm32g030 => board::stm32g030(),
            Board::Stm32g041 => board::stm32g041(),
            Board::Stm32l010 => board::stm32l010(),
            Board::Stm32l021 => board::stm32l021(),
            Board::Stm32h563 => board::stm32h563(),
            Board::Stm32u575 => board::stm32u575(),
            Board::Stm32wb55 => board::stm32wb55(),
            Board::Stm32f098 => board::stm32f098(),
            Board::Stm32f038 => board::stm32f038(),
            Board::Stm32f048 => board::stm32f048(),
            Board::Stm32f071 => board::stm32f071(),
            Board::Stm32f333 => board::stm32f333(),
            Board::Stm32g050 => board::stm32g050(),
            Board::Stm32l011 => board::stm32l011(),
            Board::Stm32l031 => board::stm32l031(),
            Board::Stm32f723 => board::stm32f723(),
            Board::Stm32h562 => board::stm32h562(),
            Board::Stm32wle5 => board::stm32wle5(),
            Board::Stm32f215 => board::stm32f215(),
            Board::Stm32f217 => board::stm32f217(),
            Board::Stm32f328 => board::stm32f328(),
            Board::Stm32g081 => board::stm32g081(),
            Board::Stm32g0b1 => board::stm32g0b1(),
            Board::Stm32l052 => board::stm32l052(),
            Board::Stm32l062 => board::stm32l062(),
            Board::Stm32f730 => board::stm32f730(),
            Board::Stm32h503 => board::stm32h503(),
            Board::Stm32u585 => board::stm32u585(),
            Board::Stm32f205 => board::stm32f205(),
            Board::Stm32f427 => board::stm32f427(),
            Board::Stm32f398 => board::stm32f398(),
            Board::Stm32g491 => board::stm32g491(),
            Board::Stm32l151 => board::stm32l151(),
            Board::Stm32f777 => board::stm32f777(),
            Board::Stm32h735 => board::stm32h735(),
            Board::Stm32u5a5 => board::stm32u5a5(),
            Board::Stm32l162 => board::stm32l162(),
            Board::Stm32wl55 => board::stm32wl55(),
            Board::Stm32f417 => board::stm32f417(),
            Board::Stm32f732 => board::stm32f732(),
            Board::Stm32g0c1 => board::stm32g0c1(),
            Board::Stm32l431 => board::stm32l431(),
            Board::Stm32h755 => board::stm32h755(),
            Board::Stm32u5g9 => board::stm32u5g9(),
            Board::Stm32wba52 => board::stm32wba52(),
            Board::Stm32l071 => board::stm32l071(),
            Board::Stm32h7a3 => board::stm32h7a3(),
            Board::Stm32l051 => board::stm32l051(),
            Board::Stm32g061 => board::stm32g061(),
            Board::Stm32g473 => board::stm32g473(),
            Board::Stm32l041 => board::stm32l041(),
            Board::Stm32l100 => board::stm32l100(),
            Board::Stm32l471 => board::stm32l471(),
            Board::Stm32h533 => board::stm32h533(),
            Board::Stm32h742 => board::stm32h742(),
            Board::Stm32u595 => board::stm32u595(),
            Board::Stm32wle4 => board::stm32wle4(),
            Board::Stm32f745 => board::stm32f745(),
            Board::Stm32c011 => board::stm32c011(),
            Board::Stm32c031 => board::stm32c031(),
            Board::Stm32u535 => board::stm32u535(),
            Board::Stm32wba55 => board::stm32wba55(),
            Board::Stm32wl33 => board::stm32wl33(),
            Board::Stm32l552 => board::stm32l552(),
            Board::Stm32l562 => board::stm32l562(),
            Board::Stm32h7b3 => board::stm32h7b3(),
            Board::Stm32u5f9 => board::stm32u5f9(),
            Board::Stm32mp157 => board::stm32mp157(),
            Board::Stm32f103c8 => board::stm32f103c8(),
            Board::Stm32f103rb => board::stm32f103rb(),
            Board::Stm32f103ze => board::stm32f103ze(),
            Board::Stm32f401re => board::stm32f401re(),
            Board::Stm32f411re => board::stm32f411re(),
            Board::Stm32f429zi => board::stm32f429zi(),
            Board::Stm32f746zg => board::stm32f746zg(),
            Board::Stm32g431rb => board::stm32g431rb(),
            Board::Stm32l476rg => board::stm32l476rg(),
            Board::Stm32h743zi => board::stm32h743zi(),
            Board::Stm32f030c6 => board::stm32f030c6(),
            Board::Stm32f030cc => board::stm32f030cc(),
            Board::Stm32f042k6 => board::stm32f042k6(),
            Board::Stm32f042t6 => board::stm32f042t6(),
            Board::Stm32f072c8 => board::stm32f072c8(),
            Board::Stm32f091rc => board::stm32f091rc(),
            Board::Stm32f100c8 => board::stm32f100c8(),
            Board::Stm32f101c8 => board::stm32f101c8(),
            Board::Stm32f102cb => board::stm32f102cb(),
            Board::Stm32f103c6 => board::stm32f103c6(),
            Board::Stm32f103re => board::stm32f103re(),
            Board::Stm32f105rb => board::stm32f105rb(),
            Board::Stm32f107vc => board::stm32f107vc(),
            Board::Stm32f205re => board::stm32f205re(),
            Board::Stm32f207ze => board::stm32f207ze(),
            Board::Stm32f215re => board::stm32f215re(),
            Board::Stm32f217ig => board::stm32f217ig(),
            Board::Stm32f301k8 => board::stm32f301k8(),
            Board::Stm32f302cb => board::stm32f302cb(),
            Board::Stm32f303cc => board::stm32f303cc(),
            Board::Stm32f373cc => board::stm32f373cc(),
            Board::Stm32f378cc => board::stm32f378cc(),
            Board::Stm32f401cc => board::stm32f401cc(),
            Board::Stm32f401ce => board::stm32f401ce(),
            Board::Stm32f410c8 => board::stm32f410c8(),
            Board::Stm32f412ce => board::stm32f412ce(),
            Board::Stm32f413ch => board::stm32f413ch(),
            Board::Stm32f423ih => board::stm32f423ih(),
            Board::Stm32f427ig => board::stm32f427ig(),
            Board::Stm32f429ig => board::stm32f429ig(),
            Board::Stm32f437ii => board::stm32f437ii(),
            Board::Stm32f446re => board::stm32f446re(),
            Board::Stm32f469ai => board::stm32f469ai(),
            Board::Stm32f722ie => board::stm32f722ie(),
            Board::Stm32f723ie => board::stm32f723ie(),
            Board::Stm32f732ie => board::stm32f732ie(),
            Board::Stm32f745ie => board::stm32f745ie(),
            Board::Stm32f746be => board::stm32f746be(),
            Board::Stm32f756zg => board::stm32f756zg(),
            Board::Stm32f765bi => board::stm32f765bi(),
            Board::Stm32f767zi => board::stm32f767zi(),
            Board::Stm32f777ai => board::stm32f777ai(),
            Board::Stm32g030f6 => board::stm32g030f6(),
            Board::Stm32g031f8 => board::stm32g031f8(),
            Board::Stm32g070rb => board::stm32g070rb(),
            Board::Stm32g071rb => board::stm32g071rb(),
            Board::Stm32g474re => board::stm32g474re(),
            Board::Stm32h743ii => board::stm32h743ii(),
            Board::Stm32h753ii => board::stm32h753ii(),
            Board::Stm32l476je => board::stm32l476je(),
            Board::Stm32f030k6 => board::stm32f030k6(),
            Board::Stm32f038k6 => board::stm32f038k6(),
            Board::Stm32f042g6 => board::stm32f042g6(),
            Board::Stm32f051g8 => board::stm32f051g8(),
            Board::Stm32f072rb => board::stm32f072rb(),
            Board::Stm32f091cb => board::stm32f091cb(),
            Board::Stm32f100rb => board::stm32f100rb(),
            Board::Stm32f101re => board::stm32f101re(),
            Board::Stm32f102re => board::stm32f102re(),
            Board::Stm32f103vd => board::stm32f103vd(),
            Board::Stm32f105re => board::stm32f105re(),
            Board::Stm32f107re => board::stm32f107re(),
            Board::Stm32f205rb => board::stm32f205rb(),
            Board::Stm32f207rb => board::stm32f207rb(),
            Board::Stm32f215ze => board::stm32f215ze(),
            Board::Stm32f217ve => board::stm32f217ve(),
            Board::Stm32f301c8 => board::stm32f301c8(),
            Board::Stm32f302cc => board::stm32f302cc(),
            Board::Stm32f303re => board::stm32f303re(),
            Board::Stm32f373cb => board::stm32f373cb(),
            Board::Stm32f378be => board::stm32f378be(),
            Board::Stm32f401rb => board::stm32f401rb(),
            Board::Stm32f401ve => board::stm32f401ve(),
            Board::Stm32f410rb => board::stm32f410rb(),
            Board::Stm32f412zg => board::stm32f412zg(),
            Board::Stm32f413rg => board::stm32f413rg(),
            Board::Stm32f423ze => board::stm32f423ze(),
            Board::Stm32f427ai => board::stm32f427ai(),
            Board::Stm32f429be => board::stm32f429be(),
            Board::Stm32f437ai => board::stm32f437ai(),
            Board::Stm32f446ze => board::stm32f446ze(),
            Board::Stm32f469ae => board::stm32f469ae(),
            Board::Stm32f722ze => board::stm32f722ze(),
            Board::Stm32f723ze => board::stm32f723ze(),
            Board::Stm32f732vg => board::stm32f732vg(),
            Board::Stm32f745vg => board::stm32f745vg(),
            Board::Stm32f746ig => board::stm32f746ig(),
            Board::Stm32f756ig => board::stm32f756ig(),
            Board::Stm32f765ig => board::stm32f765ig(),
            Board::Stm32f767ig => board::stm32f767ig(),
            Board::Stm32f777vg => board::stm32f777vg(),
            Board::Stm32g030k6 => board::stm32g030k6(),
            Board::Stm32g031k8 => board::stm32g031k8(),
            Board::Stm32g070cb => board::stm32g070cb(),
            Board::Stm32g071cb => board::stm32g071cb(),
            Board::Stm32g474ce => board::stm32g474ce(),
            Board::Stm32h743xi => board::stm32h743xi(),
            Board::Stm32h753xi => board::stm32h753xi(),
            Board::Stm32l476jg => board::stm32l476jg(),
            Board::Stm32f103x49 => board::stm32f103x49(),
            Board::Stm32f030r8 => board::stm32f030r8(),
            Board::Stm32f030v6 => board::stm32f030v6(),
            Board::Stm32f038r8 => board::stm32f038r8(),
            Board::Stm32f038v6 => board::stm32f038v6(),
            Board::Stm32f042c8 => board::stm32f042c8(),
            Board::Stm32f042r8 => board::stm32f042r8(),
            Board::Stm32f051r8 => board::stm32f051r8(),
            Board::Stm32f072r8 => board::stm32f072r8(),
            Board::Stm32f091rb => board::stm32f091rb(),
            Board::Stm32f100re => board::stm32f100re(),
            Board::Stm32f101rb => board::stm32f101rb(),
            Board::Stm32f102rb => board::stm32f102rb(),
            Board::Stm32f103r8 => board::stm32f103r8(),
            Board::Stm32f105rg => board::stm32f105rg(),
            Board::Stm32f107rg => board::stm32f107rg(),
            Board::Stm32f205rg => board::stm32f205rg(),
            Board::Stm32f207rg => board::stm32f207rg(),
            Board::Stm32f215rg => board::stm32f215rg(),
            Board::Stm32f217rg => board::stm32f217rg(),
            Board::Stm32f301r8 => board::stm32f301r8(),
            Board::Stm32f302r8 => board::stm32f302r8(),
            Board::Stm32f303rg => board::stm32f303rg(),
            Board::Stm32f373rg => board::stm32f373rg(),
            Board::Stm32f378rg => board::stm32f378rg(),
            Board::Stm32f401rg => board::stm32f401rg(),
            Board::Stm32f410re => board::stm32f410re(),
            Board::Stm32f412re => board::stm32f412re(),
            Board::Stm32f413re => board::stm32f413re(),
            Board::Stm32f423rg => board::stm32f423rg(),
            Board::Stm32f427rg => board::stm32f427rg(),
            Board::Stm32f429rg => board::stm32f429rg(),
            Board::Stm32f437rg => board::stm32f437rg(),
            Board::Stm32f446rg => board::stm32f446rg(),
            Board::Stm32f469rg => board::stm32f469rg(),
            Board::Stm32f722rg => board::stm32f722rg(),
            Board::Stm32f723rg => board::stm32f723rg(),
            Board::Stm32f732rg => board::stm32f732rg(),
            Board::Stm32f745rg => board::stm32f745rg(),
            Board::Stm32f746rg => board::stm32f746rg(),
            Board::Stm32f756rg => board::stm32f756rg(),
            Board::Stm32f765rg => board::stm32f765rg(),
            Board::Stm32f767rg => board::stm32f767rg(),
            Board::Stm32f777rg => board::stm32f777rg(),
            Board::Stm32g030c6 => board::stm32g030c6(),
            Board::Stm32g031c6 => board::stm32g031c6(),
            Board::Stm32g070re => board::stm32g070re(),
            Board::Stm32g071re => board::stm32g071re(),
            Board::Stm32g474rg => board::stm32g474rg(),
            Board::Stm32h743xg => board::stm32h743xg(),
            Board::Stm32h753xg => board::stm32h753xg(),
            Board::Stm32f030t6 => board::stm32f030t6(),
            Board::Stm32f038t6 => board::stm32f038t6(),
            Board::Stm32f042k4 => board::stm32f042k4(),
            Board::Stm32f051k4 => board::stm32f051k4(),
            Board::Stm32f072k8 => board::stm32f072k8(),
            Board::Stm32f091vb => board::stm32f091vb(),
            Board::Stm32f100rc => board::stm32f100rc(),
            Board::Stm32f101rc => board::stm32f101rc(),
            Board::Stm32f102rc => board::stm32f102rc(),
            Board::Stm32f103rc => board::stm32f103rc(),
            Board::Stm32f105rc => board::stm32f105rc(),
            Board::Stm32f107rc => board::stm32f107rc(),
            Board::Stm32f205rc => board::stm32f205rc(),
            Board::Stm32f207rc => board::stm32f207rc(),
            Board::Stm32f215rc => board::stm32f215rc(),
            Board::Stm32f217rc => board::stm32f217rc(),
            Board::Stm32f301k6 => board::stm32f301k6(),
            Board::Stm32f302k6 => board::stm32f302k6(),
            Board::Stm32f303k8 => board::stm32f303k8(),
            Board::Stm32f373k8 => board::stm32f373k8(),
            Board::Stm32f378k8 => board::stm32f378k8(),
            Board::Stm32f401k6 => board::stm32f401k6(),
            Board::Stm32f410k8 => board::stm32f410k8(),
            Board::Stm32f412k6 => board::stm32f412k6(),
            Board::Stm32f413k6 => board::stm32f413k6(),
            Board::Stm32f423k6 => board::stm32f423k6(),
            Board::Stm32f427k6 => board::stm32f427k6(),
            Board::Stm32f429k6 => board::stm32f429k6(),
            Board::Stm32f437k6 => board::stm32f437k6(),
            Board::Stm32f446k6 => board::stm32f446k6(),
            Board::Stm32f469k6 => board::stm32f469k6(),
            Board::Stm32f722k6 => board::stm32f722k6(),
            Board::Stm32f723k6 => board::stm32f723k6(),
            Board::Stm32f732k6 => board::stm32f732k6(),
            Board::Stm32f745k6 => board::stm32f745k6(),
            Board::Stm32f746k6 => board::stm32f746k6(),
            Board::Stm32f756k6 => board::stm32f756k6(),
            Board::Stm32f765k6 => board::stm32f765k6(),
            Board::Stm32f767k6 => board::stm32f767k6(),
            Board::Stm32f777k6 => board::stm32f777k6(),
            Board::Stm32g030g4 => board::stm32g030g4(),
            Board::Stm32g031g4 => board::stm32g031g4(),
            Board::Stm32g070g4 => board::stm32g070g4(),
            Board::Stm32g071g4 => board::stm32g071g4(),
            Board::Stm32g474kb => board::stm32g474kb(),
            Board::Stm32h743ag => board::stm32h743ag(),
            Board::Stm32h753ag => board::stm32h753ag(),
            Board::Stm32l476ke => board::stm32l476ke(),
            Board::Stm32c011f6 => board::stm32c011f6(),
            Board::Stm32c031f6 => board::stm32c031f6(),
            Board::Stm32f030a0 => board::stm32f030a0(),
            Board::Stm32f030a1 => board::stm32f030a1(),
            Board::Stm32f042a0 => board::stm32f042a0(),
            Board::Stm32f042a1 => board::stm32f042a1(),
            Board::Stm32f072a0 => board::stm32f072a0(),
            Board::Stm32f072a1 => board::stm32f072a1(),
            Board::Stm32f091a0 => board::stm32f091a0(),
            Board::Stm32f091a1 => board::stm32f091a1(),
            Board::Stm32f100a0 => board::stm32f100a0(),
            Board::Stm32f100a1 => board::stm32f100a1(),
            Board::Stm32f101a0 => board::stm32f101a0(),
            Board::Stm32f101a1 => board::stm32f101a1(),
            Board::Stm32f102a0 => board::stm32f102a0(),
            Board::Stm32f102a1 => board::stm32f102a1(),
            Board::Stm32f103a0 => board::stm32f103a0(),
            Board::Stm32f103a1 => board::stm32f103a1(),
            Board::Stm32f105a0 => board::stm32f105a0(),
            Board::Stm32f105a1 => board::stm32f105a1(),
            Board::Stm32f107a0 => board::stm32f107a0(),
            Board::Stm32f107a1 => board::stm32f107a1(),
            Board::Stm32f205a0 => board::stm32f205a0(),
            Board::Stm32f205a1 => board::stm32f205a1(),
            Board::Stm32f207a0 => board::stm32f207a0(),
            Board::Stm32f207a1 => board::stm32f207a1(),
            Board::Stm32f215a0 => board::stm32f215a0(),
            Board::Stm32f215a1 => board::stm32f215a1(),
            Board::Stm32f217a0 => board::stm32f217a0(),
            Board::Stm32f217a1 => board::stm32f217a1(),
            Board::Stm32f301a0 => board::stm32f301a0(),
            Board::Stm32f301a1 => board::stm32f301a1(),
            Board::Stm32f302a0 => board::stm32f302a0(),
            Board::Stm32f302a1 => board::stm32f302a1(),
            Board::Stm32f303a0 => board::stm32f303a0(),
            Board::Stm32f303a1 => board::stm32f303a1(),
            Board::Stm32f373a0 => board::stm32f373a0(),
            Board::Stm32f373a1 => board::stm32f373a1(),
            Board::Stm32f378a0 => board::stm32f378a0(),
            Board::Stm32f378a1 => board::stm32f378a1(),
            Board::Stm32f401a0 => board::stm32f401a0(),
            Board::Stm32f401a1 => board::stm32f401a1(),
            Board::Stm32f410a0 => board::stm32f410a0(),
            Board::Stm32f410a1 => board::stm32f410a1(),
            Board::Stm32f412a0 => board::stm32f412a0(),
            Board::Stm32f412a1 => board::stm32f412a1(),
            Board::Stm32f413a0 => board::stm32f413a0(),
            Board::Stm32f413a1 => board::stm32f413a1(),
            Board::Stm32f423a0 => board::stm32f423a0(),
            Board::Stm32f423a1 => board::stm32f423a1(),
            Board::Stm32f427a0 => board::stm32f427a0(),
            Board::Stm32f427a1 => board::stm32f427a1(),
            Board::Stm32f429a0 => board::stm32f429a0(),
            Board::Stm32f429a1 => board::stm32f429a1(),
            Board::Stm32f437a0 => board::stm32f437a0(),
            Board::Stm32f437a1 => board::stm32f437a1(),
            Board::Stm32f446a0 => board::stm32f446a0(),
            Board::Stm32f446a1 => board::stm32f446a1(),
            Board::Stm32f469a0 => board::stm32f469a0(),
            Board::Stm32f469a1 => board::stm32f469a1(),
            Board::Stm32f722a0 => board::stm32f722a0(),
            Board::Stm32f722a1 => board::stm32f722a1(),
            Board::Stm32f723a0 => board::stm32f723a0(),
            Board::Stm32f723a1 => board::stm32f723a1(),
            Board::Stm32f732a0 => board::stm32f732a0(),
            Board::Stm32f732a1 => board::stm32f732a1(),
            Board::Stm32f745a0 => board::stm32f745a0(),
            Board::Stm32f745a1 => board::stm32f745a1(),
            Board::Stm32f746a0 => board::stm32f746a0(),
            Board::Stm32f746a1 => board::stm32f746a1(),
            Board::Stm32f756a0 => board::stm32f756a0(),
            Board::Stm32f756a1 => board::stm32f756a1(),
            Board::Stm32f765a0 => board::stm32f765a0(),
            Board::Stm32f765a1 => board::stm32f765a1(),
            Board::Stm32f767a0 => board::stm32f767a0(),
            Board::Stm32f767a1 => board::stm32f767a1(),
            Board::Stm32f777a0 => board::stm32f777a0(),
            Board::Stm32f777a1 => board::stm32f777a1(),
            Board::Stm32g030a0 => board::stm32g030a0(),
            Board::Stm32g030a1 => board::stm32g030a1(),
            Board::Stm32g031a0 => board::stm32g031a0(),
            Board::Stm32g031a1 => board::stm32g031a1(),
            Board::Stm32g070a0 => board::stm32g070a0(),
            Board::Stm32g070a1 => board::stm32g070a1(),
            Board::Stm32g071a0 => board::stm32g071a0(),
            Board::Stm32g071a1 => board::stm32g071a1(),
            Board::Stm32g474a0 => board::stm32g474a0(),
            Board::Stm32g474a1 => board::stm32g474a1(),
            Board::Stm32h743a0 => board::stm32h743a0(),
            Board::Stm32h743a1 => board::stm32h743a1(),
            Board::Stm32h753a0 => board::stm32h753a0(),
            Board::Stm32h753a1 => board::stm32h753a1(),
            Board::Stm32l476a0 => board::stm32l476a0(),
            Board::Stm32l476a1 => board::stm32l476a1(),
            Board::Stm32c011a0 => board::stm32c011a0(),
            Board::Stm32c011a1 => board::stm32c011a1(),
            Board::Stm32c031a0 => board::stm32c031a0(),
            Board::Stm32c031a1 => board::stm32c031a1(),
            Board::Stm32u575a0 => board::stm32u575a0(),
            Board::Stm32u575a1 => board::stm32u575a1(),
            Board::Stm32wba55a0 => board::stm32wba55a0(),
            Board::Stm32wba55a1 => board::stm32wba55a1(),
            Board::Stm32f030b0 => board::stm32f030b0(),
            Board::Stm32f042b0 => board::stm32f042b0(),
            Board::Stm32f072b0 => board::stm32f072b0(),
            Board::Stm32f091b0 => board::stm32f091b0(),
            Board::Stm32f100b0 => board::stm32f100b0(),
            Board::Stm32f101b0 => board::stm32f101b0(),
            Board::Stm32f102b0 => board::stm32f102b0(),
            Board::Stm32f103b0 => board::stm32f103b0(),
            Board::Stm32f105b0 => board::stm32f105b0(),
            Board::Stm32f107b0 => board::stm32f107b0(),
            Board::Stm32f205b0 => board::stm32f205b0(),
            Board::Stm32f207b0 => board::stm32f207b0(),
            Board::Stm32f215b0 => board::stm32f215b0(),
            Board::Stm32f217b0 => board::stm32f217b0(),
            Board::Stm32f301b0 => board::stm32f301b0(),
            Board::Stm32f302b0 => board::stm32f302b0(),
            Board::Stm32f303b0 => board::stm32f303b0(),
            Board::Stm32f373b0 => board::stm32f373b0(),
            Board::Stm32f378b0 => board::stm32f378b0(),
            Board::Stm32f401b0 => board::stm32f401b0(),
            Board::Stm32f410b0 => board::stm32f410b0(),
            Board::Stm32f412b0 => board::stm32f412b0(),
            Board::Stm32f413b0 => board::stm32f413b0(),
            Board::Stm32f423b0 => board::stm32f423b0(),
            Board::Stm32f427b0 => board::stm32f427b0(),
            Board::Stm32f429b0 => board::stm32f429b0(),
            Board::Stm32f437b0 => board::stm32f437b0(),
            Board::Stm32f446b0 => board::stm32f446b0(),
            Board::Stm32f469b0 => board::stm32f469b0(),
            Board::Stm32f722b0 => board::stm32f722b0(),
            Board::Stm32f723b0 => board::stm32f723b0(),
            Board::Stm32f732b0 => board::stm32f732b0(),
            Board::Stm32f745b0 => board::stm32f745b0(),
            Board::Stm32f746b0 => board::stm32f746b0(),
            Board::Stm32f756b0 => board::stm32f756b0(),
            Board::Stm32f765b0 => board::stm32f765b0(),
            Board::Stm32f767b0 => board::stm32f767b0(),
            Board::Stm32f777b0 => board::stm32f777b0(),
            Board::Stm32g030b0 => board::stm32g030b0(),
            Board::Stm32g031b0 => board::stm32g031b0(),
            Board::Stm32g070b0 => board::stm32g070b0(),
            Board::Stm32g071b0 => board::stm32g071b0(),
            Board::Stm32g474b0 => board::stm32g474b0(),
            Board::Stm32h743b0 => board::stm32h743b0(),
            Board::Stm32h753b0 => board::stm32h753b0(),
            Board::Stm32l476b0 => board::stm32l476b0(),
            Board::Stm32c011b0 => board::stm32c011b0(),
            Board::Stm32c031b0 => board::stm32c031b0(),
            Board::Stm32u575b0 => board::stm32u575b0(),
            Board::Stm32wba55b0 => board::stm32wba55b0(),
            Board::Stm32f030b1 => board::stm32f030b1(),
            Board::Stm32f042b1 => board::stm32f042b1(),
            Board::Stm32f072b1 => board::stm32f072b1(),
            Board::Stm32f091b1 => board::stm32f091b1(),
            Board::Stm32f100b1 => board::stm32f100b1(),
            Board::Stm32f101b1 => board::stm32f101b1(),
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
            Board::Tm4c123gh6pm => board::tm4c123gh6pm(),
            Board::Tm4c1294ncpdt => board::tm4c1294ncpdt(),
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
            Board::Stm32f446 => "stm32f446",
            Board::Stm32f469 => "stm32f469",
            Board::Stm32f722 => "stm32f722",
            Board::Stm32f746 => "stm32f746",
            Board::Stm32f756 => "stm32f756",
            Board::Stm32f765 => "stm32f765",
            Board::Stm32f767 => "stm32f767",
            Board::Stm32h7b0 => "stm32h7b0",
            Board::Stm32h730 => "stm32h730",
            Board::Stm32h743 => "stm32h743",
            Board::Stm32h745 => "stm32h745",
            Board::Stm32h747 => "stm32h747",
            Board::Stm32h750 => "stm32h750",
            Board::Stm32h753 => "stm32h753",
            Board::Stm32f413 => "stm32f413",
            Board::Stm32f423 => "stm32f423",
            Board::Stm32h723 => "stm32h723",
            Board::Stm32f479 => "stm32f479",
            Board::Stm32g431 => "stm32g431",
            Board::Stm32l452 => "stm32l452",
            Board::Stm32f302 => "stm32f302",
            Board::Stm32f334 => "stm32f334",
            Board::Stm32l432 => "stm32l432",
            Board::Stm32f373 => "stm32f373",
            Board::Stm32l073 => "stm32l073",
            Board::Stm32f358 => "stm32f358",
            Board::Stm32l152 => "stm32l152",
            Board::Stm32f070 => "stm32f070",
            Board::Stm32l053 => "stm32l053",
            Board::Stm32g070 => "stm32g070",
            Board::Stm32f378 => "stm32f378",
            Board::Stm32f303 => "stm32f303",
            Board::Stm32l412 => "stm32l412",
            Board::Stm32g031 => "stm32g031",
            Board::Stm32g071 => "stm32g071",
            Board::Stm32f301 => "stm32f301",
            Board::Stm32l433 => "stm32l433",
            Board::Stm32f410 => "stm32f410",
            Board::Stm32g441 => "stm32g441",
            Board::Stm32l443 => "stm32l443",
            Board::Stm32f318 => "stm32f318",
            Board::Stm32f051 => "stm32f051",
            Board::Stm32f207 => "stm32f207",
            Board::Stm32g030 => "stm32g030",
            Board::Stm32g041 => "stm32g041",
            Board::Stm32l010 => "stm32l010",
            Board::Stm32l021 => "stm32l021",
            Board::Stm32h563 => "stm32h563",
            Board::Stm32u575 => "stm32u575",
            Board::Stm32wb55 => "stm32wb55",
            Board::Stm32f098 => "stm32f098",
            Board::Stm32f038 => "stm32f038",
            Board::Stm32f048 => "stm32f048",
            Board::Stm32f071 => "stm32f071",
            Board::Stm32f333 => "stm32f333",
            Board::Stm32g050 => "stm32g050",
            Board::Stm32l011 => "stm32l011",
            Board::Stm32l031 => "stm32l031",
            Board::Stm32f723 => "stm32f723",
            Board::Stm32h562 => "stm32h562",
            Board::Stm32wle5 => "stm32wle5",
            Board::Stm32f215 => "stm32f215",
            Board::Stm32f217 => "stm32f217",
            Board::Stm32f328 => "stm32f328",
            Board::Stm32g081 => "stm32g081",
            Board::Stm32g0b1 => "stm32g0b1",
            Board::Stm32l052 => "stm32l052",
            Board::Stm32l062 => "stm32l062",
            Board::Stm32f730 => "stm32f730",
            Board::Stm32h503 => "stm32h503",
            Board::Stm32u585 => "stm32u585",
            Board::Stm32f205 => "stm32f205",
            Board::Stm32f427 => "stm32f427",
            Board::Stm32f398 => "stm32f398",
            Board::Stm32g491 => "stm32g491",
            Board::Stm32l151 => "stm32l151",
            Board::Stm32f777 => "stm32f777",
            Board::Stm32h735 => "stm32h735",
            Board::Stm32u5a5 => "stm32u5a5",
            Board::Stm32l162 => "stm32l162",
            Board::Stm32wl55 => "stm32wl55",
            Board::Stm32f417 => "stm32f417",
            Board::Stm32f732 => "stm32f732",
            Board::Stm32g0c1 => "stm32g0c1",
            Board::Stm32l431 => "stm32l431",
            Board::Stm32h755 => "stm32h755",
            Board::Stm32u5g9 => "stm32u5g9",
            Board::Stm32wba52 => "stm32wba52",
            Board::Stm32l071 => "stm32l071",
            Board::Stm32h7a3 => "stm32h7a3",
            Board::Stm32l051 => "stm32l051",
            Board::Stm32g061 => "stm32g061",
            Board::Stm32g473 => "stm32g473",
            Board::Stm32l041 => "stm32l041",
            Board::Stm32l100 => "stm32l100",
            Board::Stm32l471 => "stm32l471",
            Board::Stm32h533 => "stm32h533",
            Board::Stm32h742 => "stm32h742",
            Board::Stm32u595 => "stm32u595",
            Board::Stm32wle4 => "stm32wle4",
            Board::Stm32f745 => "stm32f745",
            Board::Stm32c011 => "stm32c011",
            Board::Stm32c031 => "stm32c031",
            Board::Stm32u535 => "stm32u535",
            Board::Stm32wba55 => "stm32wba55",
            Board::Stm32wl33 => "stm32wl33",
            Board::Stm32l552 => "stm32l552",
            Board::Stm32l562 => "stm32l562",
            Board::Stm32h7b3 => "stm32h7b3",
            Board::Stm32u5f9 => "stm32u5f9",
            Board::Stm32mp157 => "stm32mp157",
            Board::Stm32f103c8 => "stm32f103c8",
            Board::Stm32f103rb => "stm32f103rb",
            Board::Stm32f103ze => "stm32f103ze",
            Board::Stm32f401re => "stm32f401re",
            Board::Stm32f411re => "stm32f411re",
            Board::Stm32f429zi => "stm32f429zi",
            Board::Stm32f746zg => "stm32f746zg",
            Board::Stm32g431rb => "stm32g431rb",
            Board::Stm32l476rg => "stm32l476rg",
            Board::Stm32h743zi => "stm32h743zi",
            Board::Stm32f030c6 => "stm32f030c6",
            Board::Stm32f030cc => "stm32f030cc",
            Board::Stm32f042k6 => "stm32f042k6",
            Board::Stm32f042t6 => "stm32f042t6",
            Board::Stm32f072c8 => "stm32f072c8",
            Board::Stm32f091rc => "stm32f091rc",
            Board::Stm32f100c8 => "stm32f100c8",
            Board::Stm32f101c8 => "stm32f101c8",
            Board::Stm32f102cb => "stm32f102cb",
            Board::Stm32f103c6 => "stm32f103c6",
            Board::Stm32f103re => "stm32f103re",
            Board::Stm32f105rb => "stm32f105rb",
            Board::Stm32f107vc => "stm32f107vc",
            Board::Stm32f205re => "stm32f205re",
            Board::Stm32f207ze => "stm32f207ze",
            Board::Stm32f215re => "stm32f215re",
            Board::Stm32f217ig => "stm32f217ig",
            Board::Stm32f301k8 => "stm32f301k8",
            Board::Stm32f302cb => "stm32f302cb",
            Board::Stm32f303cc => "stm32f303cc",
            Board::Stm32f373cc => "stm32f373cc",
            Board::Stm32f378cc => "stm32f378cc",
            Board::Stm32f401cc => "stm32f401cc",
            Board::Stm32f401ce => "stm32f401ce",
            Board::Stm32f410c8 => "stm32f410c8",
            Board::Stm32f412ce => "stm32f412ce",
            Board::Stm32f413ch => "stm32f413ch",
            Board::Stm32f423ih => "stm32f423ih",
            Board::Stm32f427ig => "stm32f427ig",
            Board::Stm32f429ig => "stm32f429ig",
            Board::Stm32f437ii => "stm32f437ii",
            Board::Stm32f446re => "stm32f446re",
            Board::Stm32f469ai => "stm32f469ai",
            Board::Stm32f722ie => "stm32f722ie",
            Board::Stm32f723ie => "stm32f723ie",
            Board::Stm32f732ie => "stm32f732ie",
            Board::Stm32f745ie => "stm32f745ie",
            Board::Stm32f746be => "stm32f746be",
            Board::Stm32f756zg => "stm32f756zg",
            Board::Stm32f765bi => "stm32f765bi",
            Board::Stm32f767zi => "stm32f767zi",
            Board::Stm32f777ai => "stm32f777ai",
            Board::Stm32g030f6 => "stm32g030f6",
            Board::Stm32g031f8 => "stm32g031f8",
            Board::Stm32g070rb => "stm32g070rb",
            Board::Stm32g071rb => "stm32g071rb",
            Board::Stm32g474re => "stm32g474re",
            Board::Stm32h743ii => "stm32h743ii",
            Board::Stm32h753ii => "stm32h753ii",
            Board::Stm32l476je => "stm32l476je",
            Board::Stm32f030k6 => "stm32f030k6",
            Board::Stm32f038k6 => "stm32f038k6",
            Board::Stm32f042g6 => "stm32f042g6",
            Board::Stm32f051g8 => "stm32f051g8",
            Board::Stm32f072rb => "stm32f072rb",
            Board::Stm32f091cb => "stm32f091cb",
            Board::Stm32f100rb => "stm32f100rb",
            Board::Stm32f101re => "stm32f101re",
            Board::Stm32f102re => "stm32f102re",
            Board::Stm32f103vd => "stm32f103vd",
            Board::Stm32f105re => "stm32f105re",
            Board::Stm32f107re => "stm32f107re",
            Board::Stm32f205rb => "stm32f205rb",
            Board::Stm32f207rb => "stm32f207rb",
            Board::Stm32f215ze => "stm32f215ze",
            Board::Stm32f217ve => "stm32f217ve",
            Board::Stm32f301c8 => "stm32f301c8",
            Board::Stm32f302cc => "stm32f302cc",
            Board::Stm32f303re => "stm32f303re",
            Board::Stm32f373cb => "stm32f373cb",
            Board::Stm32f378be => "stm32f378be",
            Board::Stm32f401rb => "stm32f401rb",
            Board::Stm32f401ve => "stm32f401ve",
            Board::Stm32f410rb => "stm32f410rb",
            Board::Stm32f412zg => "stm32f412zg",
            Board::Stm32f413rg => "stm32f413rg",
            Board::Stm32f423ze => "stm32f423ze",
            Board::Stm32f427ai => "stm32f427ai",
            Board::Stm32f429be => "stm32f429be",
            Board::Stm32f437ai => "stm32f437ai",
            Board::Stm32f446ze => "stm32f446ze",
            Board::Stm32f469ae => "stm32f469ae",
            Board::Stm32f722ze => "stm32f722ze",
            Board::Stm32f723ze => "stm32f723ze",
            Board::Stm32f732vg => "stm32f732vg",
            Board::Stm32f745vg => "stm32f745vg",
            Board::Stm32f746ig => "stm32f746ig",
            Board::Stm32f756ig => "stm32f756ig",
            Board::Stm32f765ig => "stm32f765ig",
            Board::Stm32f767ig => "stm32f767ig",
            Board::Stm32f777vg => "stm32f777vg",
            Board::Stm32g030k6 => "stm32g030k6",
            Board::Stm32g031k8 => "stm32g031k8",
            Board::Stm32g070cb => "stm32g070cb",
            Board::Stm32g071cb => "stm32g071cb",
            Board::Stm32g474ce => "stm32g474ce",
            Board::Stm32h743xi => "stm32h743xi",
            Board::Stm32h753xi => "stm32h753xi",
            Board::Stm32l476jg => "stm32l476jg",
            Board::Stm32f103x49 => "stm32f103x49",
            Board::Stm32f030r8 => "stm32f030r8",
            Board::Stm32f030v6 => "stm32f030v6",
            Board::Stm32f038r8 => "stm32f038r8",
            Board::Stm32f038v6 => "stm32f038v6",
            Board::Stm32f042c8 => "stm32f042c8",
            Board::Stm32f042r8 => "stm32f042r8",
            Board::Stm32f051r8 => "stm32f051r8",
            Board::Stm32f072r8 => "stm32f072r8",
            Board::Stm32f091rb => "stm32f091rb",
            Board::Stm32f100re => "stm32f100re",
            Board::Stm32f101rb => "stm32f101rb",
            Board::Stm32f102rb => "stm32f102rb",
            Board::Stm32f103r8 => "stm32f103r8",
            Board::Stm32f105rg => "stm32f105rg",
            Board::Stm32f107rg => "stm32f107rg",
            Board::Stm32f205rg => "stm32f205rg",
            Board::Stm32f207rg => "stm32f207rg",
            Board::Stm32f215rg => "stm32f215rg",
            Board::Stm32f217rg => "stm32f217rg",
            Board::Stm32f301r8 => "stm32f301r8",
            Board::Stm32f302r8 => "stm32f302r8",
            Board::Stm32f303rg => "stm32f303rg",
            Board::Stm32f373rg => "stm32f373rg",
            Board::Stm32f378rg => "stm32f378rg",
            Board::Stm32f401rg => "stm32f401rg",
            Board::Stm32f410re => "stm32f410re",
            Board::Stm32f412re => "stm32f412re",
            Board::Stm32f413re => "stm32f413re",
            Board::Stm32f423rg => "stm32f423rg",
            Board::Stm32f427rg => "stm32f427rg",
            Board::Stm32f429rg => "stm32f429rg",
            Board::Stm32f437rg => "stm32f437rg",
            Board::Stm32f446rg => "stm32f446rg",
            Board::Stm32f469rg => "stm32f469rg",
            Board::Stm32f722rg => "stm32f722rg",
            Board::Stm32f723rg => "stm32f723rg",
            Board::Stm32f732rg => "stm32f732rg",
            Board::Stm32f745rg => "stm32f745rg",
            Board::Stm32f746rg => "stm32f746rg",
            Board::Stm32f756rg => "stm32f756rg",
            Board::Stm32f765rg => "stm32f765rg",
            Board::Stm32f767rg => "stm32f767rg",
            Board::Stm32f777rg => "stm32f777rg",
            Board::Stm32g030c6 => "stm32g030c6",
            Board::Stm32g031c6 => "stm32g031c6",
            Board::Stm32g070re => "stm32g070re",
            Board::Stm32g071re => "stm32g071re",
            Board::Stm32g474rg => "stm32g474rg",
            Board::Stm32h743xg => "stm32h743xg",
            Board::Stm32h753xg => "stm32h753xg",
            Board::Stm32f030t6 => "stm32f030t6",
            Board::Stm32f038t6 => "stm32f038t6",
            Board::Stm32f042k4 => "stm32f042k4",
            Board::Stm32f051k4 => "stm32f051k4",
            Board::Stm32f072k8 => "stm32f072k8",
            Board::Stm32f091vb => "stm32f091vb",
            Board::Stm32f100rc => "stm32f100rc",
            Board::Stm32f101rc => "stm32f101rc",
            Board::Stm32f102rc => "stm32f102rc",
            Board::Stm32f103rc => "stm32f103rc",
            Board::Stm32f105rc => "stm32f105rc",
            Board::Stm32f107rc => "stm32f107rc",
            Board::Stm32f205rc => "stm32f205rc",
            Board::Stm32f207rc => "stm32f207rc",
            Board::Stm32f215rc => "stm32f215rc",
            Board::Stm32f217rc => "stm32f217rc",
            Board::Stm32f301k6 => "stm32f301k6",
            Board::Stm32f302k6 => "stm32f302k6",
            Board::Stm32f303k8 => "stm32f303k8",
            Board::Stm32f373k8 => "stm32f373k8",
            Board::Stm32f378k8 => "stm32f378k8",
            Board::Stm32f401k6 => "stm32f401k6",
            Board::Stm32f410k8 => "stm32f410k8",
            Board::Stm32f412k6 => "stm32f412k6",
            Board::Stm32f413k6 => "stm32f413k6",
            Board::Stm32f423k6 => "stm32f423k6",
            Board::Stm32f427k6 => "stm32f427k6",
            Board::Stm32f429k6 => "stm32f429k6",
            Board::Stm32f437k6 => "stm32f437k6",
            Board::Stm32f446k6 => "stm32f446k6",
            Board::Stm32f469k6 => "stm32f469k6",
            Board::Stm32f722k6 => "stm32f722k6",
            Board::Stm32f723k6 => "stm32f723k6",
            Board::Stm32f732k6 => "stm32f732k6",
            Board::Stm32f745k6 => "stm32f745k6",
            Board::Stm32f746k6 => "stm32f746k6",
            Board::Stm32f756k6 => "stm32f756k6",
            Board::Stm32f765k6 => "stm32f765k6",
            Board::Stm32f767k6 => "stm32f767k6",
            Board::Stm32f777k6 => "stm32f777k6",
            Board::Stm32g030g4 => "stm32g030g4",
            Board::Stm32g031g4 => "stm32g031g4",
            Board::Stm32g070g4 => "stm32g070g4",
            Board::Stm32g071g4 => "stm32g071g4",
            Board::Stm32g474kb => "stm32g474kb",
            Board::Stm32h743ag => "stm32h743ag",
            Board::Stm32h753ag => "stm32h753ag",
            Board::Stm32l476ke => "stm32l476ke",
            Board::Stm32c011f6 => "stm32c011f6",
            Board::Stm32c031f6 => "stm32c031f6",
            Board::Stm32f030a0 => "stm32f030a0",
            Board::Stm32f030a1 => "stm32f030a1",
            Board::Stm32f042a0 => "stm32f042a0",
            Board::Stm32f042a1 => "stm32f042a1",
            Board::Stm32f072a0 => "stm32f072a0",
            Board::Stm32f072a1 => "stm32f072a1",
            Board::Stm32f091a0 => "stm32f091a0",
            Board::Stm32f091a1 => "stm32f091a1",
            Board::Stm32f100a0 => "stm32f100a0",
            Board::Stm32f100a1 => "stm32f100a1",
            Board::Stm32f101a0 => "stm32f101a0",
            Board::Stm32f101a1 => "stm32f101a1",
            Board::Stm32f102a0 => "stm32f102a0",
            Board::Stm32f102a1 => "stm32f102a1",
            Board::Stm32f103a0 => "stm32f103a0",
            Board::Stm32f103a1 => "stm32f103a1",
            Board::Stm32f105a0 => "stm32f105a0",
            Board::Stm32f105a1 => "stm32f105a1",
            Board::Stm32f107a0 => "stm32f107a0",
            Board::Stm32f107a1 => "stm32f107a1",
            Board::Stm32f205a0 => "stm32f205a0",
            Board::Stm32f205a1 => "stm32f205a1",
            Board::Stm32f207a0 => "stm32f207a0",
            Board::Stm32f207a1 => "stm32f207a1",
            Board::Stm32f215a0 => "stm32f215a0",
            Board::Stm32f215a1 => "stm32f215a1",
            Board::Stm32f217a0 => "stm32f217a0",
            Board::Stm32f217a1 => "stm32f217a1",
            Board::Stm32f301a0 => "stm32f301a0",
            Board::Stm32f301a1 => "stm32f301a1",
            Board::Stm32f302a0 => "stm32f302a0",
            Board::Stm32f302a1 => "stm32f302a1",
            Board::Stm32f303a0 => "stm32f303a0",
            Board::Stm32f303a1 => "stm32f303a1",
            Board::Stm32f373a0 => "stm32f373a0",
            Board::Stm32f373a1 => "stm32f373a1",
            Board::Stm32f378a0 => "stm32f378a0",
            Board::Stm32f378a1 => "stm32f378a1",
            Board::Stm32f401a0 => "stm32f401a0",
            Board::Stm32f401a1 => "stm32f401a1",
            Board::Stm32f410a0 => "stm32f410a0",
            Board::Stm32f410a1 => "stm32f410a1",
            Board::Stm32f412a0 => "stm32f412a0",
            Board::Stm32f412a1 => "stm32f412a1",
            Board::Stm32f413a0 => "stm32f413a0",
            Board::Stm32f413a1 => "stm32f413a1",
            Board::Stm32f423a0 => "stm32f423a0",
            Board::Stm32f423a1 => "stm32f423a1",
            Board::Stm32f427a0 => "stm32f427a0",
            Board::Stm32f427a1 => "stm32f427a1",
            Board::Stm32f429a0 => "stm32f429a0",
            Board::Stm32f429a1 => "stm32f429a1",
            Board::Stm32f437a0 => "stm32f437a0",
            Board::Stm32f437a1 => "stm32f437a1",
            Board::Stm32f446a0 => "stm32f446a0",
            Board::Stm32f446a1 => "stm32f446a1",
            Board::Stm32f469a0 => "stm32f469a0",
            Board::Stm32f469a1 => "stm32f469a1",
            Board::Stm32f722a0 => "stm32f722a0",
            Board::Stm32f722a1 => "stm32f722a1",
            Board::Stm32f723a0 => "stm32f723a0",
            Board::Stm32f723a1 => "stm32f723a1",
            Board::Stm32f732a0 => "stm32f732a0",
            Board::Stm32f732a1 => "stm32f732a1",
            Board::Stm32f745a0 => "stm32f745a0",
            Board::Stm32f745a1 => "stm32f745a1",
            Board::Stm32f746a0 => "stm32f746a0",
            Board::Stm32f746a1 => "stm32f746a1",
            Board::Stm32f756a0 => "stm32f756a0",
            Board::Stm32f756a1 => "stm32f756a1",
            Board::Stm32f765a0 => "stm32f765a0",
            Board::Stm32f765a1 => "stm32f765a1",
            Board::Stm32f767a0 => "stm32f767a0",
            Board::Stm32f767a1 => "stm32f767a1",
            Board::Stm32f777a0 => "stm32f777a0",
            Board::Stm32f777a1 => "stm32f777a1",
            Board::Stm32g030a0 => "stm32g030a0",
            Board::Stm32g030a1 => "stm32g030a1",
            Board::Stm32g031a0 => "stm32g031a0",
            Board::Stm32g031a1 => "stm32g031a1",
            Board::Stm32g070a0 => "stm32g070a0",
            Board::Stm32g070a1 => "stm32g070a1",
            Board::Stm32g071a0 => "stm32g071a0",
            Board::Stm32g071a1 => "stm32g071a1",
            Board::Stm32g474a0 => "stm32g474a0",
            Board::Stm32g474a1 => "stm32g474a1",
            Board::Stm32h743a0 => "stm32h743a0",
            Board::Stm32h743a1 => "stm32h743a1",
            Board::Stm32h753a0 => "stm32h753a0",
            Board::Stm32h753a1 => "stm32h753a1",
            Board::Stm32l476a0 => "stm32l476a0",
            Board::Stm32l476a1 => "stm32l476a1",
            Board::Stm32c011a0 => "stm32c011a0",
            Board::Stm32c011a1 => "stm32c011a1",
            Board::Stm32c031a0 => "stm32c031a0",
            Board::Stm32c031a1 => "stm32c031a1",
            Board::Stm32u575a0 => "stm32u575a0",
            Board::Stm32u575a1 => "stm32u575a1",
            Board::Stm32wba55a0 => "stm32wba55a0",
            Board::Stm32wba55a1 => "stm32wba55a1",
            Board::Stm32f030b0 => "stm32f030b0",
            Board::Stm32f042b0 => "stm32f042b0",
            Board::Stm32f072b0 => "stm32f072b0",
            Board::Stm32f091b0 => "stm32f091b0",
            Board::Stm32f100b0 => "stm32f100b0",
            Board::Stm32f101b0 => "stm32f101b0",
            Board::Stm32f102b0 => "stm32f102b0",
            Board::Stm32f103b0 => "stm32f103b0",
            Board::Stm32f105b0 => "stm32f105b0",
            Board::Stm32f107b0 => "stm32f107b0",
            Board::Stm32f205b0 => "stm32f205b0",
            Board::Stm32f207b0 => "stm32f207b0",
            Board::Stm32f215b0 => "stm32f215b0",
            Board::Stm32f217b0 => "stm32f217b0",
            Board::Stm32f301b0 => "stm32f301b0",
            Board::Stm32f302b0 => "stm32f302b0",
            Board::Stm32f303b0 => "stm32f303b0",
            Board::Stm32f373b0 => "stm32f373b0",
            Board::Stm32f378b0 => "stm32f378b0",
            Board::Stm32f401b0 => "stm32f401b0",
            Board::Stm32f410b0 => "stm32f410b0",
            Board::Stm32f412b0 => "stm32f412b0",
            Board::Stm32f413b0 => "stm32f413b0",
            Board::Stm32f423b0 => "stm32f423b0",
            Board::Stm32f427b0 => "stm32f427b0",
            Board::Stm32f429b0 => "stm32f429b0",
            Board::Stm32f437b0 => "stm32f437b0",
            Board::Stm32f446b0 => "stm32f446b0",
            Board::Stm32f469b0 => "stm32f469b0",
            Board::Stm32f722b0 => "stm32f722b0",
            Board::Stm32f723b0 => "stm32f723b0",
            Board::Stm32f732b0 => "stm32f732b0",
            Board::Stm32f745b0 => "stm32f745b0",
            Board::Stm32f746b0 => "stm32f746b0",
            Board::Stm32f756b0 => "stm32f756b0",
            Board::Stm32f765b0 => "stm32f765b0",
            Board::Stm32f767b0 => "stm32f767b0",
            Board::Stm32f777b0 => "stm32f777b0",
            Board::Stm32g030b0 => "stm32g030b0",
            Board::Stm32g031b0 => "stm32g031b0",
            Board::Stm32g070b0 => "stm32g070b0",
            Board::Stm32g071b0 => "stm32g071b0",
            Board::Stm32g474b0 => "stm32g474b0",
            Board::Stm32h743b0 => "stm32h743b0",
            Board::Stm32h753b0 => "stm32h753b0",
            Board::Stm32l476b0 => "stm32l476b0",
            Board::Stm32c011b0 => "stm32c011b0",
            Board::Stm32c031b0 => "stm32c031b0",
            Board::Stm32u575b0 => "stm32u575b0",
            Board::Stm32wba55b0 => "stm32wba55b0",
            Board::Stm32f030b1 => "stm32f030b1",
            Board::Stm32f042b1 => "stm32f042b1",
            Board::Stm32f072b1 => "stm32f072b1",
            Board::Stm32f091b1 => "stm32f091b1",
            Board::Stm32f100b1 => "stm32f100b1",
            Board::Stm32f101b1 => "stm32f101b1",
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
            Board::Tm4c123gh6pm => "tm4c123gh6pm",
            Board::Tm4c1294ncpdt => "tm4c1294ncpdt",
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

/// `--board` help: the core follows the board family (WP-K1).
const BOARD_HELP: &str = "Board to boot (core follows family: virtual/F0/C0/G0/L0 = M0, F1/F2/F3/F4/G4/L4 = M3/M4, F7/H7 = M7, U5/H5/WBA/L5 = M33; Thumb-2 .w needs an M3+ board)";

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
    #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
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
    /// Arm semihosting (`BKPT 0xAB`) plus ITM stimulus capture
    #[arg(long)]
    semihost: bool,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Run firmware
    Run {
        /// Image path (.bin/.elf/.hex auto-detected)
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        #[arg(long, default_value_t = 1_000_000, alias = "max-cycles")]
        budget: u64,
        /// Arm semihosting (`BKPT 0xAB`) plus ITM stimulus capture
        #[arg(long)]
        semihost: bool,
        /// Fast-mode quantum (`0` = accurate mode; see `Machine::set_fast_quantum`)
        #[arg(long, default_value_t = 0)]
        fast: u64,
        /// Translated-block engine (default off; needs `--fast 0`.
        /// Modes: accurate (`--fast 0`, no `--jit`), fast-coarse
        /// (`--fast K`, K > 0), JIT (`--jit`, approximate `cyccnt` per D7
        /// while state stays exact).
        #[arg(long)]
        jit: bool,
        /// Sample GPIO pins during the run (`c13`, `a0,a1`; repeatable or
        /// comma-separated). Prints one plain `t=<s> <pin>=<0|1>` line per
        /// sample, starting at t=0, for waveform checks without a debugger.
        #[arg(long, value_delimiter = ',')]
        pin_log: Vec<String>,
        /// Retired instructions between `--pin-log` samples
        /// (deterministic: reruns print identical lines).
        #[arg(long, default_value_t = 1000)]
        pin_sample: u64,
    },
    /// Record a golden trace (deterministic replay artifact)
    Record {
        /// Image path (.bin/.elf/.hex auto-detected)
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        #[arg(long, default_value_t = 1_000_000, alias = "max-cycles")]
        budget: u64,
        /// Arm semihosting (`BKPT 0xAB`) plus ITM stimulus capture
        #[arg(long)]
        semihost: bool,
        /// Fast-mode quantum (`0` = accurate mode; stored in the golden)
        #[arg(long, default_value_t = 0)]
        fast: u64,
        /// Golden file to write
        #[arg(long)]
        output: PathBuf,
        /// Record in translated-block mode (marks the golden JIT;
        /// replay then compares state only, counters excluded per D7;
        /// needs `--fast 0`)
        #[arg(long)]
        jit: bool,
        /// RAM spans to digest (`BASE:LEN`, repeatable; default snapshots
        /// the first 4 KiB of `0x20000000`)
        #[arg(long)]
        snap: Vec<String>,
    },
    /// Replay a golden trace and diff (exit 1 on any difference)
    Replay {
        /// Image path (digest must match the golden)
        image: PathBuf,
        /// Golden file from `record`
        golden: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        /// Memory patches standing in for a mutated model (`ADDR=VAL`,
        /// repeatable; applied before running)
        #[arg(long)]
        patch: Vec<String>,
    },
    /// Trace cycle-by-cycle
    Trace {
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        #[arg(long, default_value_t = 1_000_000, alias = "max-cycles")]
        budget: u64,
        /// Arm semihosting (`BKPT 0xAB`) plus ITM stimulus capture
        #[arg(long)]
        semihost: bool,
        /// Trace format: human-readable text (default), `jsonl` streaming
        /// (`{"cycle":N,"pc":"0x..."}` per retired instruction, driven by
        /// the hook registry), or `vcd` waveform (needs `--pin-log`).
        #[arg(long, value_enum, default_value_t = TraceFormat::Text)]
        format: TraceFormat,
        /// Write the trace here instead of stdout.
        #[arg(long)]
        output: Option<PathBuf>,
        /// Sample GPIO pins into the trace (`c13`, `a0,a1`; repeatable or
        /// comma-separated). Only `vcd` consumes these today.
        #[arg(long, value_delimiter = ',')]
        pin_log: Vec<String>,
        /// Retired instructions between `vcd` samples (reruns identical).
        #[arg(long, default_value_t = 1)]
        pin_sample: u64,
    },
    /// Measure throughput on this machine (field benchmark, no criterion)
    Benchmark {
        #[arg(long, value_enum, default_value_t = Board::Stm32f103, help = BOARD_HELP)]
        board: Board,
        /// Retired steps per kernel (kernels repeat fixed windows to it).
        #[arg(long, default_value_t = 1_000_000)]
        cycles: u64,
        /// Exit 1 when any kernel lands below this MIPS (CI floor).
        #[arg(long)]
        fail_under: Option<f64>,
    },
    /// Show image layout (segments, entry)
    Info { image: PathBuf },
    /// List all boards (21 SVD tables + 500 Dnames)
    Boards {
        /// Print the pin-label catalog instead (one board, or every table
        /// with `--pins all`): `PA9: USART1_TX` default-AF labels for HIL
        /// wiring. Unknown boards error with the valid list.
        #[arg(long, value_name = "BOARD", num_args = 0..=1, default_missing_value = "all")]
        pins: Option<String>,
    },
    /// Instant terminal docs
    Docs {
        /// Topic: boards, periphs, arch, dma, tim, adc, etc.
        topic: Option<String>,
    },
    /// Interactive debug REPL
    Debug {
        image: PathBuf,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
    },
    /// Serve GDB RSP on TCP port (`target remote :<port>`)
    Gdb {
        image: PathBuf,
        #[arg(long, default_value_t = 3333)]
        port: u16,
        /// Debug translated blocks (X2: per-insn stops from TB metadata,
        /// quanta continue, breakpoint pages bypass translation)
        #[arg(long)]
        jit: bool,
        #[arg(long, value_enum, default_value_t = Board::Virtual, help = BOARD_HELP)]
        board: Board,
        #[arg(long, default_value_t = 0, value_parser = parse_u32)]
        base: u32,
        /// Arm semihosting (`BKPT 0xAB`); firmware `SYS_EXIT` ends the session
        #[arg(long)]
        semihost: bool,
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
        eprintln!(
            "{} cannot read {}: {e}",
            "error:".red().bold(),
            path.display()
        );
        std::process::exit(2);
    });
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let kind = if ext == "elf" {
        "elf"
    } else if ext == "hex" {
        "hex"
    } else if ext == "bin" {
        "bin"
    } else {
        // auto-detect by magic
        if bytes.starts_with(b"\x7fELF") {
            "elf"
        } else if bytes.starts_with(b":") {
            "hex"
        } else {
            "bin"
        }
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

/// Image bytes in address order for digesting (segments concatenated).
fn flat_image_bytes(img: &LoadedImage) -> Vec<u8> {
    let mut segs: Vec<(u32, &[u8])> = img.segments.iter().map(|(a, b)| (*a, &b[..])).collect();
    segs.sort_by_key(|(addr, _)| *addr);
    segs.into_iter()
        .flat_map(|(_, b)| b.iter().copied())
        .collect()
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

/// Render the pin-label catalog for `which` (one board short name,
/// case-insensitive, or `"all"` for every table): one `PIN: FUNCTION` row
/// per line under a `board <name>` header. Unknown boards fail closed with
/// the valid list so lab setups never wire to a typo.
fn render_pin_tables(which: &str) -> Result<String, String> {
    use vioarm_core::pins::{pin_labels, PIN_BOARDS};
    let mut out = String::new();
    if which.eq_ignore_ascii_case("all") {
        for board in PIN_BOARDS {
            let table = pin_labels(board).expect("catalog board resolves");
            out.push_str(&format!("board {board}\n"));
            for (pin, function) in table {
                out.push_str(&format!("  {pin:5} {function}\n"));
            }
        }
        return Ok(out);
    }
    let Some(table) = pin_labels(which) else {
        return Err(format!(
            "unknown board '{which}' — valid: {}",
            PIN_BOARDS.join(", ")
        ));
    };
    out.push_str(&format!("board {}\n", which.to_ascii_lowercase()));
    for (pin, function) in table {
        out.push_str(&format!("  {pin:5} {function}\n"));
    }
    Ok(out)
}

fn machine_for(
    board: Board,
    img: &LoadedImage,
    vector_base: Option<u32>,
) -> Machine<board::BoardMemory> {
    let mem = board.mem();
    // WP-K1: the core follows the board family; the dispatch lives in
    // `Machine::for_mem` (shared with the co-simulation embedding path).
    // `virtual` stays the explicit M0 home (see `board::cortex_m0`).
    let name = board.name();
    let mut m = Machine::for_mem(name, mem);
    m.load_image(img);
    // Raw binaries carry no vector-table address: boot from their load
    // base when the caller passes one (ELF/HEX keep the default table).
    if let Some(base) = vector_base {
        m.set_vector_base(base);
    }
    if let Err(cause) = m.reset() {
        eprintln!("{} reset failed: {cause}", "error:".red().bold());
        if vector_base.is_some() {
            eprintln!(
                "  {} raw binaries boot from their load base; on STM32 load at the 0x0 flash alias — stores to the gated 0x0800_0000 window are dropped while the FLASH controller is locked",
                "hint:".bright_cyan().bold()
            );
        }
        std::process::exit(3);
    }
    m
}

/// A `--pin-log` entry: GPIO port (0 = A) + pin + the original text.
/// Ports run `a`..`k` (validated against the board at run time); pins
/// `0`..`31` (GPIO flavors narrow further).
fn parse_pin_spec(s: &str) -> Result<(u8, u8, String), String> {
    let err = || format!("bad --pin-log entry '{s}': want PORTPIN like c13 (port a-k, pin 0-31)");
    let s = s.trim();
    if s.is_empty() {
        return Err(err());
    }
    let letter = s.chars().next().ok_or_else(err)?;
    if !letter.is_ascii_alphabetic() {
        return Err(err());
    }
    let port = letter.to_ascii_lowercase() as u8 - b'a';
    if port >= 16 {
        return Err(err());
    }
    let pin: u8 = s.get(1..).ok_or_else(err)?.parse().map_err(|_| err())?;
    if pin >= 32 {
        return Err(err());
    }
    Ok((port, pin, s.to_owned()))
}

/// Print one `--pin-log` sample line: `t=<seconds> <pin>=<0|1> ...`.
/// Stamps are true post-retire `cyccnt` (TRM cycle model, not retired-step
/// counts), so `t=` is real board time; an early halt prints the lines so
/// far plus the usual halt report below.
fn sample_pins(
    m: &mut Machine<board::BoardMemory>,
    pins: &[(u8, u8, String)],
    hz: f64,
    cyccnt: u64,
) {
    // Precision loss below one cycle at astronomical budgets; the
    // printed lines stay rerun-identical regardless.
    #[allow(clippy::cast_precision_loss)]
    let seconds = cyccnt as f64 / hz;
    let mut line = format!("t={seconds:.6}");
    for (port, pin, name) in pins {
        let level = m
            .gpio_pin(*port, *pin)
            .map_or(0, |(_, level)| u8::from(level));
        line.push_str(&format!(" {name}={level}"));
    }
    println!("{line}");
}

/// Format one backtrace opcode: the halfword at `pc`, plus `:h1` when the
/// opcode prefix marks a 32-bit Thumb-2 encoding (top five bits `11101`,
/// `11110`, or `11111`) and the second halfword reads. Narrow opcodes
/// print alone so the next instruction's bytes never masquerade as part
/// of this one; unreadable code prints placeholders.
fn format_backtrace_op(e: &vioarm_core::machine::BacktraceEntry) -> String {
    let Some(h0) = e.h0 else {
        return "????????".to_owned();
    };
    let wide = matches!(h0 & 0xF800, 0xE800 | 0xF000 | 0xF800);
    match (wide, e.h1) {
        (true, Some(h1)) => format!("{h0:#06x}:{h1:#06x}"),
        _ => format!("{h0:#06x}"),
    }
}

/// Print the last-retired-instruction ring after a fault halt (at most
/// `max` entries, oldest first). Empty rings print nothing, so
/// budget-exhausted and disabled-ring runs stay quiet. The faulting
/// instruction itself never retires, so this prefix pairs with the live
/// `pc` on the halt line above.
fn print_backtrace(m: &mut Machine<board::BoardMemory>, max: usize) {
    let entries = m.backtrace(max);
    if entries.is_empty() {
        return;
    }
    println!("{} (last {}):", "backtrace".dimmed().bold(), entries.len());
    for (i, e) in entries.iter().enumerate() {
        println!(
            "  #{i} pc={:#010x} op={} cyccnt={}",
            e.pc,
            format_backtrace_op(e),
            e.cyccnt
        );
    }
}

fn run_quiet(
    m: &mut Machine<board::BoardMemory>,
    budget: u64,
    jit: bool,
    pins: &[(u8, u8, String)],
    sample: u64,
) {
    // Pin sampling: print the initial state, then one line per `sample`
    // retired instructions.
    let hz = f64::from(m.cpu_hz());
    let mut remaining = budget;
    if !pins.is_empty() {
        if sample == 0 {
            eprintln!("{} --pin-sample must be at least 1", "error:".red().bold());
            std::process::exit(2);
        }
        sample_pins(m, pins, hz, m.core.cyccnt);
    }
    let mut runner = jit.then(vioarm_core::jit::TbRunner::new);
    let outcome = loop {
        if remaining == 0 {
            break Err(HaltCause::CycleBudgetExhausted);
        }
        let chunk = if pins.is_empty() {
            remaining
        } else {
            sample.min(remaining)
        };
        let step = match runner.as_mut() {
            Some(runner) => m.run_tb(runner, chunk),
            None => m.run(chunk),
        };
        match step {
            Err(HaltCause::CycleBudgetExhausted) => {
                remaining -= chunk;
                if !pins.is_empty() {
                    sample_pins(m, pins, hz, m.core.cyccnt);
                }
            }
            other => break other,
        }
    };
    match outcome {
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
            print_backtrace(m, 32);
        }
        Ok(()) => unreachable!(),
    }
    for device in m.core.mem().devices_mut() {
        let tx = device.drain_output();
        if !tx.is_empty() {
            println!(
                "{} {}",
                "uart:".bright_magenta().bold(),
                String::from_utf8_lossy(&tx)
            );
        }
    }
    let sh = m.semihost_output();
    if !sh.is_empty() {
        println!(
            "{} {}",
            "semihost:".bright_magenta().bold(),
            String::from_utf8_lossy(&sh)
        );
    }
    let itm = m.drain_channel("itm");
    if !itm.is_empty() {
        println!(
            "{} {}",
            "itm:".bright_magenta().bold(),
            String::from_utf8_lossy(&itm)
        );
    }
    if let Some(code) = m.semihost_exit() {
        println!("{} {code}", "exit:".bright_magenta().bold());
    }
}

/// JSONL trace: one `{"cycle":N,"pc":"0x..."}` line per retired
/// instruction, streamed through the hook registry (pre-instruction hook,
/// full PC range). `output` selects a file, `None` streams to stdout.
fn run_traced_jsonl(m: &mut Machine<board::BoardMemory>, budget: u64, output: Option<PathBuf>) {
    use std::io::Write;
    let sink: Box<dyn Write + Send> = match output {
        Some(path) => Box::new(
            std::fs::File::create(&path)
                .unwrap_or_else(|e| panic!("trace output {}: {e}", path.display())),
        ),
        // `Stdout` (not the non-`Send` lock guard): each write locks briefly.
        None => Box::new(std::io::stdout()),
    };
    let sink = std::sync::Arc::new(std::sync::Mutex::new(sink));
    let writer = sink.clone();
    m.on_instruction(true, 0, u32::MAX, move |e| {
        if let HookEvent::Instruction { pc, cycles } = *e {
            let mut out = writer.lock().unwrap_or_else(|e| e.into_inner());
            writeln!(out, "{{\"cycle\":{cycles},\"pc\":\"{pc:#010x}\"}}").ok();
        }
    });
    for _ in 0..budget {
        if m.step().is_err() {
            break;
        }
    }
}

/// Minimal value-change-dump writer: header with one wire per signal plus
/// a 32-bit `pc` bus, then `#<ns>` timestamps with one row per change.
/// `$date` is fixed text so reruns are byte-identical.
struct VcdWriter<W: std::io::Write> {
    out: W,
    ids: Vec<char>,
    widths: Vec<u8>,
    last: Vec<u64>,
    last_ts: Option<u64>,
}

impl<W: std::io::Write> VcdWriter<W> {
    fn new(mut out: W, signals: &[(String, u8)]) -> std::io::Result<Self> {
        let mut ids = Vec::with_capacity(signals.len());
        let mut widths = Vec::with_capacity(signals.len());
        let mut last = Vec::with_capacity(signals.len());
        let mut head = format!(
            "$date VioARM {}$end\n$version VioARM {}$end\n$timescale 1ns $end\n$scope module top $end\n",
            env!("CARGO_PKG_VERSION"),
            env!("CARGO_PKG_VERSION")
        );
        for (i, (name, width)) in signals.iter().enumerate() {
            let id = char::from_u32(33 + i as u32).unwrap_or('?');
            head.push_str(&format!("$var wire {width} {id} {name} $end\n"));
            ids.push(id);
            widths.push(*width);
            last.push(u64::MAX);
        }
        head.push_str("$upscope $end\n$enddefinitions $end\n#0\n");
        out.write_all(head.as_bytes())?;
        Ok(Self {
            out,
            ids,
            widths,
            last,
            last_ts: Some(0),
        })
    }

    fn stamp_ns(cycles: u64, hz: u32) -> u64 {
        if hz == 0 {
            return cycles;
        }
        #[allow(clippy::cast_possible_truncation)]
        {
            (u128::from(cycles) * 1_000_000_000 / u128::from(hz)) as u64
        }
    }

    fn sample(&mut self, cycles: u64, hz: u32, values: &[u64]) -> std::io::Result<()> {
        debug_assert_eq!(values.len(), self.ids.len());
        let ts = Self::stamp_ns(cycles, hz);
        let mut pending = Vec::new();
        for (i, (&value, &width)) in values.iter().zip(self.widths.iter()).enumerate() {
            if self.last[i] != value {
                self.last[i] = value;
                if width == 1 {
                    pending.push(format!("{}{}\n", value & 1, self.ids[i]));
                } else {
                    pending.push(format!(
                        "b{value:0width$b} {}\n",
                        self.ids[i],
                        width = width as usize
                    ));
                }
            }
        }
        if pending.is_empty() {
            return Ok(());
        }
        if self.last_ts != Some(ts) {
            self.last_ts = Some(ts);
            self.out.write_all(format!("#{ts}\n").as_bytes())?;
        }
        for row in pending {
            self.out.write_all(row.as_bytes())?;
        }
        Ok(())
    }
}

/// VCD trace: `pc` plus `--pin-log` pins sampled every `sample` retired
/// instructions. `output` selects a file, `None` streams to stdout.
fn run_traced_vcd(
    m: &mut Machine<board::BoardMemory>,
    budget: u64,
    output: Option<PathBuf>,
    pins: &[(u8, u8, String)],
    sample: u64,
) {
    use std::io::Write;
    let sink: Box<dyn Write + Send> = match output {
        Some(path) => Box::new(
            std::fs::File::create(&path)
                .unwrap_or_else(|e| panic!("trace output {}: {e}", path.display())),
        ),
        None => Box::new(std::io::stdout()),
    };
    let hz = m.cpu_hz();
    let mut signals: Vec<(String, u8)> =
        pins.iter().map(|(_, _, name)| (name.clone(), 1)).collect();
    signals.push(("pc".to_owned(), 32));
    let mut vcd = VcdWriter::new(sink, &signals).unwrap_or_else(|e| panic!("trace output: {e}"));
    let dump = |m: &mut Machine<board::BoardMemory>, vcd: &mut VcdWriter<Box<dyn Write + Send>>| {
        let (regs, _, _) = m.core.snapshot();
        let mut values: Vec<u64> = pins
            .iter()
            .map(|(port, pin, _)| {
                u64::from(
                    m.gpio_pin(*port, *pin)
                        .map_or(0, |(_, level)| u8::from(level)),
                )
            })
            .collect();
        values.push(u64::from(regs.pc));
        vcd.sample(m.core.cyccnt, hz, &values).ok();
    };
    dump(m, &mut vcd);
    for i in 1..=budget {
        if m.step().is_err() {
            break;
        }
        if i % sample == 0 {
            dump(m, &mut vcd);
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
            print_backtrace(m, 32);
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

/// Field benchmark kernels (WP-R3): the same shapes as the criterion
/// group in `vioarm-core/benches/mips.rs`, reimplemented inline so the
/// CLI needs no dev-dependencies at runtime. Every kernel boots from a
/// vector table and repeats a fixed retired-step window until it reaches
/// `cycles`; only stepping is timed (setup and `reset()` stay outside,
/// mirroring `iter_batched`), so the number is stepping throughput.
struct BenchResult {
    kernel: &'static str,
    steps: u64,
    wall_s: f64,
    cyccnt: u64,
    /// Trailing `# ...` comment (e.g. USART bytes drained).
    note: Option<String>,
}

impl BenchResult {
    #[allow(clippy::cast_precision_loss)]
    fn mips(&self) -> f64 {
        self.steps as f64 / self.wall_s / 1e6
    }
    fn wall_ms(&self) -> f64 {
        self.wall_s * 1e3
    }
}

/// One stdout row: `kernel mips wall-ms cyccnt [# note]` — plain numbers
/// scripts parse; humans read the optional comment.
fn bench_row(r: &BenchResult) -> String {
    let mut s = format!(
        "{} {:.3} {:.1} {}",
        r.kernel,
        r.mips(),
        r.wall_ms(),
        r.cyccnt
    );
    if let Some(note) = &r.note {
        s.push_str(&format!(" # {note}"));
    }
    s
}

/// Kernels below this MIPS trip `--fail-under`.
fn bench_floor_failures(results: &[BenchResult], floor: f64) -> Vec<&str> {
    results
        .iter()
        .filter(|r| r.mips() < floor)
        .map(|r| r.kernel)
        .collect()
}

/// Step exactly `steps` retired instructions, timing the stepping only.
/// A fault aborts the kernel loudly — benchmarks never swallow halts.
fn bench_window<M: vioarm_core::mem::PeripheralBus>(
    m: &mut Machine<M>,
    steps: u64,
) -> Result<(f64, u64), HaltCause> {
    let c0 = m.core.cyccnt;
    let start = std::time::Instant::now();
    for _ in 0..steps {
        m.step()?;
    }
    Ok((
        start.elapsed().as_secs_f64(),
        m.core.cyccnt.wrapping_sub(c0),
    ))
}

/// Vector-table image at `base`: SP + reset vector, padding, then `code`
/// at `base + 0x10`. Every kernel boots from it via `reset()` or a
/// direct pc set, like field firmware.
fn bench_image(base: u32, code: &[u8]) -> Vec<u8> {
    let mut img = vec![0u8; 0x10];
    img[0..4].copy_from_slice(&0x2000_8000u32.to_le_bytes());
    img[4..8].copy_from_slice(&(base.wrapping_add(0x10) | 1).to_le_bytes());
    img.extend_from_slice(code);
    img
}

/// `nops` NOPs (`0xBF00`, Thumb-1 on every core) plus a `b .` landing.
fn bench_nop_code(nops: usize) -> Vec<u8> {
    let mut code = Vec::with_capacity(nops * 2 + 2);
    for _ in 0..nops {
        code.extend_from_slice(&[0x00, 0xBF]);
    }
    code.extend_from_slice(&[0xFE, 0xE7]);
    code
}

/// Bare NOP loop on `FlatMemory` (mirrors `mips/nop_loop_1k`, ~4.5 MIPS):
/// no bus scan, no device ticks — the ceiling the board kernels divide by.
fn bench_bare_nop(cycles: u64) -> Result<BenchResult, String> {
    use vioarm_core::mem::flat::FlatMemory;
    const BASE: u32 = 0x0800_0000;
    const ENTRY: u32 = BASE + 0x10;
    const NOPS: u32 = 1000;
    const WINDOW: u64 = 1000;
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[BASE], &[0x10000]));
    m.load_image(&loader::load_flat(
        BASE,
        &bench_image(BASE, &bench_nop_code(1000)),
    ));
    if m.core.mem().read_half(ENTRY + NOPS * 2) != Ok(0xE7FE) {
        return Err("bare-nop: image did not land".to_owned());
    }
    let reps = cycles.div_ceil(WINDOW).max(1);
    let (mut wall, mut cyccnt, mut steps) = (0.0, 0u64, 0u64);
    for _ in 0..reps {
        // Flat memory holds no peripheral state: re-arming the pc is the
        // whole reboot (mirrors the bench's direct pc set).
        m.core.regs.pc = ENTRY;
        m.core.xpsr.set(0x0100_0000);
        let (dt, dcy) = bench_window(&mut m, WINDOW)
            .map_err(|e| format!("bare-nop: faulted after {steps} steps: {e}"))?;
        wall += dt;
        cyccnt += dcy;
        steps += WINDOW;
    }
    Ok(BenchResult {
        kernel: "bare-nop",
        steps,
        wall_s: wall,
        cyccnt,
        note: None,
    })
}

/// Board boot-to-halt shape (mirrors `board/stm32f103_boot_10k_nop`):
/// vector-table boot at the flash alias, 10k NOPs per window, the
/// family-correct core via `Machine::for_mem`. The bus scan plus device
/// ticks are the measured cost over `bare-nop`.
fn bench_board_boot(board: Board, cycles: u64) -> Result<BenchResult, String> {
    const BASE: u32 = 0x0000_0000;
    const ENTRY: u32 = 0x10;
    const NOPS: u32 = 10_000;
    const WINDOW: u64 = 10_000;
    let mut m = Machine::for_mem(board.name(), board.mem());
    m.load_image(&loader::load_flat(
        BASE,
        &bench_image(BASE, &bench_nop_code(NOPS as usize)),
    ));
    if m.core.mem().read_half(ENTRY + NOPS * 2) != Ok(0xE7FE) {
        return Err(format!(
            "board-boot: image did not land on {} (no flash alias at 0x0?)",
            board.name()
        ));
    }
    let reps = cycles.div_ceil(WINDOW).max(1);
    let (mut wall, mut cyccnt, mut steps) = (0.0, 0u64, 0u64);
    for _ in 0..reps {
        m.reset()
            .map_err(|e| format!("board-boot: reboot failed: {e}"))?;
        let (dt, dcy) = bench_window(&mut m, WINDOW)
            .map_err(|e| format!("board-boot: faulted after {steps} steps: {e}"))?;
        wall += dt;
        cyccnt += dcy;
        steps += WINDOW;
    }
    Ok(BenchResult {
        kernel: "board-boot",
        steps,
        wall_s: wall,
        cyccnt,
        note: None,
    })
}

/// USART1 base for `board` from its SVD chip table (`None` for the
/// virtual board, which carries a `uart` instead of a `usart1`).
fn bench_usart1_base(board_name: &str) -> Option<u32> {
    board::all_chip_tables()
        .into_iter()
        .find(|(name, _)| *name == board_name)
        .and_then(|(_, peris)| peris.iter().find(|p| p.name == "USART1").map(|p| p.base))
}

/// USART-TX soak: F1-map firmware (`CR1=UE|TE`, then 200 `STR` to `DR`
/// per window) on the selected board. The peripheral addresses stamp in
/// as literals, so any F1-map family (F1/F2/F4/F7) runs; other maps fail
/// the host-side TX probe below and skip the kernel (`Ok(None)`) instead
/// of timing a dead store.
fn bench_usart_tx(board: Board, cycles: u64) -> Result<Option<BenchResult>, String> {
    const BASE: u32 = 0x0000_0000;
    const ENTRY: u32 = 0x10;
    const TRIPS: u64 = 200;
    const WINDOW: u64 = 6 + TRIPS * 3; // setup + 200×(str,subs,bne)
    const CR1_OFF: u32 = 0x0C;
    const DR_OFF: u32 = 0x04;
    const UE_TE: u32 = (1 << 13) | (1 << 3);
    let Some(u1) = bench_usart1_base(board.name()) else {
        return Ok(None);
    };
    let mut m = Machine::for_mem(board.name(), board.mem());
    // Hand-encoded Thumb-1, literals word-aligned after the code:
    //   0x10: ldr r3,[pc,#16]  ; [0x24] = USART1_CR1
    //   0x12: ldr r0,[pc,#20]  ; [0x28] = UE|TE
    //   0x14: str r0,[r3]      ; enable TX
    //   0x16: ldr r3,[pc,#20]  ; [0x2C] = USART1_DR
    //   0x18: movs r0,#'A'; 0x1A: movs r2,#200
    //   0x1C: str r0,[r3]; 0x1E: subs r2,r2,#1; 0x20: bne 0x1C; 0x22: b .
    let mut img = bench_image(
        BASE,
        &[
            0x04, 0x4B, 0x05, 0x48, 0x18, 0x60, 0x05, 0x4B, 0x41, 0x20, 0xC8, 0x22, 0x18, 0x60,
            0x52, 0x1E, 0xFC, 0xD1, 0xFE, 0xE7,
        ],
    );
    // Room for the three literal words at 0x24/0x28/0x2C.
    img.resize(0x30, 0);
    img[0x24..0x28].copy_from_slice(&(u1 + CR1_OFF).to_le_bytes());
    img[0x28..0x2C].copy_from_slice(&UE_TE.to_le_bytes());
    img[0x2C..0x30].copy_from_slice(&(u1 + DR_OFF).to_le_bytes());
    m.load_image(&loader::load_flat(BASE, &img));
    // Host-side TX probe: the F1 map answers with our byte; anything else
    // (new-gen ISR/RDR/TDR map) skips the kernel, no bogus row.
    m.core
        .mem()
        .write_word(u1 + CR1_OFF, UE_TE)
        .map_err(|e| format!("usart-tx: CR1 poke failed: {e}"))?;
    m.core
        .mem()
        .write_word(u1 + DR_OFF, u32::from(b'X'))
        .map_err(|e| format!("usart-tx: DR poke failed: {e}"))?;
    if m.drain_channel("usart1") != b"X" {
        return Ok(None);
    }
    let reps = cycles.div_ceil(WINDOW).max(1);
    let (mut wall, mut cyccnt, mut steps) = (0.0, 0u64, 0u64);
    let (mut tx_total, mut tx_ok) = (0u64, true);
    for _ in 0..reps {
        m.reset()
            .map_err(|e| format!("usart-tx: reboot failed: {e}"))?;
        // `reset()` clears the TX enable: re-arm outside the timer.
        m.core
            .mem()
            .write_word(u1 + CR1_OFF, UE_TE)
            .map_err(|e| format!("usart-tx: CR1 re-arm failed: {e}"))?;
        let (dt, dcy) = bench_window(&mut m, WINDOW)
            .map_err(|e| format!("usart-tx: faulted after {steps} steps: {e}"))?;
        wall += dt;
        cyccnt += dcy;
        steps += WINDOW;
        // Peripheral reset clears the queue each reboot, so drain per
        // window (host-side, untimed) and accumulate the count.
        let chunk = m.drain_channel("usart1");
        tx_ok &= chunk.len() == TRIPS as usize && chunk.iter().all(|&b| b == b'A');
        tx_total += u64::try_from(chunk.len()).unwrap_or(u64::MAX);
    }
    let expected = reps.saturating_mul(TRIPS);
    let mut note = format!("tx={tx_total}B");
    if tx_total != expected || !tx_ok {
        // Loud, not wrong: the steps retired, but the peripheral mix is
        // not the soak we claim — say so on the row.
        note.push_str(" MISMATCH");
    }
    Ok(Some(BenchResult {
        kernel: "usart-tx",
        steps,
        wall_s: wall,
        cyccnt,
        note: Some(note),
    }))
}

/// Parse a REPL `set` register name (`r0`-`r12`, `sp`, `lr`, `pc`,
/// case-insensitive, `r13`-`r15` aliases included).
fn parse_reg_name(name: &str) -> Option<u8> {
    match name.to_ascii_lowercase().as_str() {
        "r0" => Some(0),
        "r1" => Some(1),
        "r2" => Some(2),
        "r3" => Some(3),
        "r4" => Some(4),
        "r5" => Some(5),
        "r6" => Some(6),
        "r7" => Some(7),
        "r8" => Some(8),
        "r9" => Some(9),
        "r10" => Some(10),
        "r11" => Some(11),
        "r12" => Some(12),
        "sp" | "r13" => Some(13),
        "lr" | "r14" => Some(14),
        "pc" | "r15" => Some(15),
        _ => None,
    }
}

/// A REPL watchpoint: four bytes at `addr`, tripping on reads and/or
/// writes through the hook registry (observe-only; the machine runs
/// untouched — reads are real bus reads, so MMIO side effects apply).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Watch {
    /// First watched byte (covers `addr..addr+4`).
    addr: u32,
    /// Trip on reads.
    read: bool,
    /// Trip on writes.
    write: bool,
}

/// Arm one hook direction for `addr..addr+4`, recording into `tripped`
/// only while the watch is still in `watches` — stale closures no-op, so
/// `unwatch` needs no registry removal. Returns whether the board
/// carries a hook registry at all.
fn add_watch(
    m: &mut Machine<board::BoardMemory>,
    watches: std::rc::Rc<std::cell::RefCell<Vec<Watch>>>,
    tripped: std::rc::Rc<std::cell::RefCell<Option<(u32, bool)>>>,
    addr: u32,
    read: bool,
    write: bool,
) -> bool {
    let end = addr.wrapping_add(4);
    // Record the watch once (idempotent on re-arm).
    {
        let mut set = watches.borrow_mut();
        if !set
            .iter()
            .any(|w| w.addr == addr && w.read == read && w.write == write)
        {
            set.push(Watch { addr, read, write });
        }
    }
    let mut armed = false;
    if read {
        let watches = watches.clone();
        let tripped = tripped.clone();
        armed |= m.on_read(addr, end, move |event| {
            if let HookEvent::MemRead { addr: at, .. } = *event {
                let watched = watches
                    .borrow()
                    .iter()
                    .any(|w| w.read && at.wrapping_sub(w.addr) < 4);
                if watched {
                    *tripped.borrow_mut() = Some((at, true));
                }
            }
        });
    }
    if write {
        let watches = watches.clone();
        let tripped = tripped.clone();
        armed |= m.on_write(addr, end, move |event| {
            if let HookEvent::MemWrite { addr: at, .. } = *event {
                let watched = watches
                    .borrow()
                    .iter()
                    .any(|w| w.write && at.wrapping_sub(w.addr) < 4);
                if watched {
                    *tripped.borrow_mut() = Some((at, false));
                }
            }
        });
    }
    armed
}

/// Resolve `<hex pc|name>` for `delete` (no conditions).
fn resolve_addr(args: &str, symbols: &[vioarm_core::loader::Symbol]) -> Option<u32> {
    parse_u32(args.trim())
        .ok()
        .or_else(|| find_symbol(symbols, args.trim()))
}

/// Where a breakpoint-aware REPL run stopped.
#[derive(Debug)]
enum ReplStop {
    /// Parked on a persistent breakpoint.
    Hit(u32),
    /// A watchpoint tripped: the data address plus whether it was a read.
    Watch { addr: u32, read: bool },
    /// The machine halted (fault, BKPT, WFI, ...).
    Halt(HaltCause),
    /// The step budget ran out.
    Exhausted,
}

/// A conditional breakpoint: `break <pc> [if <reg> ==|!= <val>]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Breakpoint {
    /// Stop address.
    pc: u32,
    /// Optional register-equality gate.
    cond: Option<BreakCond>,
}

/// A register-equality gate for [`Breakpoint`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BreakCond {
    /// Register number (as [`parse_reg_name`]).
    reg: u8,
    /// `true` for `!=`, `false` for `==`.
    invert: bool,
    /// Compared value.
    val: u32,
}

/// Parse `break` arguments: `<pc|name> [if <reg> ==|!= <val>]`. Names
/// resolve through `symbols` (exact match); anything else must parse as a
/// number. Returns the address plus the optional condition.
fn parse_break_args(
    args: &str,
    symbols: &[vioarm_core::loader::Symbol],
) -> Option<(u32, Option<BreakCond>)> {
    let (target, cond) = match args.split_once(" if ") {
        Some((t, c)) => (t.trim(), Some(c.trim())),
        None => (args.trim(), None),
    };
    let pc = parse_u32(target)
        .ok()
        .or_else(|| find_symbol(symbols, target))?;
    let cond = match cond {
        None => None,
        Some(spec) => {
            let (reg, val, invert) = match spec.split_once("==") {
                Some((r, v)) => (r, v, false),
                None => match spec.split_once("!=") {
                    Some((r, v)) => (r, v, true),
                    None => return None,
                },
            };
            Some(BreakCond {
                reg: parse_reg_name(reg.trim())?,
                invert,
                val: parse_u32(val.trim()).ok()?,
            })
        }
    };
    Some((pc, cond))
}

/// Whether `bp` stops the machine in its current state.
fn break_hits(m: &Machine<board::BoardMemory>, bp: &Breakpoint) -> bool {
    if m.core.regs.pc != bp.pc {
        return false;
    }
    match bp.cond {
        None => true,
        Some(cond) => {
            let val = m.core.regs.read(cond.reg);
            (val == cond.val) != cond.invert
        }
    }
}

/// Exact symbol lookup by name.
fn find_symbol(symbols: &[vioarm_core::loader::Symbol], name: &str) -> Option<u32> {
    symbols.iter().find(|s| s.name == name).map(|s| s.addr)
}

/// Nearest symbol at or below `addr` plus the offset (`None` with no
/// symbols, or when `addr` precedes every symbol).
fn symbolize(symbols: &[vioarm_core::loader::Symbol], addr: u32) -> Option<(&str, u32)> {
    symbols
        .iter()
        .filter(|s| s.addr <= addr)
        .max_by_key(|s| s.addr)
        .map(|s| (s.name.as_str(), addr.wrapping_sub(s.addr)))
}

/// Step until a persistent breakpoint, a watchpoint trip, a halt, or
/// `budget` steps. A breakpoint at the entry pc is stepped over once so
/// `continue` resumes instead of re-stopping where it already sits.
/// `tripped` carries hook watchpoint hits out of the stepping loop.
fn run_until_break(
    m: &mut Machine<board::BoardMemory>,
    breakpoints: &[Breakpoint],
    tripped: &std::cell::RefCell<Option<(u32, bool)>>,
    mut budget: u64,
) -> ReplStop {
    if breakpoints.iter().any(|bp| break_hits(m, bp)) {
        match m.step() {
            Err(cause) => return ReplStop::Halt(cause),
            Ok(()) => budget = budget.saturating_sub(1),
        }
    }
    while budget > 0 {
        if breakpoints.iter().any(|bp| break_hits(m, bp)) {
            return ReplStop::Hit(m.core.regs.pc);
        }
        match m.step() {
            Err(cause) => return ReplStop::Halt(cause),
            Ok(()) => budget -= 1,
        }
        if let Some((addr, read)) = tripped.borrow_mut().take() {
            return ReplStop::Watch { addr, read };
        }
    }
    ReplStop::Exhausted
}

/// Advertised `docs` topics: the help line and the regression test share
/// this list so no topic rots into "unknown topic" again.
const DOC_TOPICS: &[&str] = &[
    "boards",
    "periphs",
    "arch",
    "core",
    "dma",
    "tim",
    "adc",
    "usart",
    "spi",
    "i2c",
    "can",
    "usb",
    "eth",
    "sdio",
    "fmc",
    "h7",
    "run",
    "trace",
    "record",
    "benchmark",
    "scoreboard",
    "gdb",
    "debug",
    "embedding",
];

/// One-line topic text (`None` for unknown topics).
fn docs_text(topic: &str) -> Option<String> {
    match topic {
        "dma" => Some(format!("{}", "DMA — F1 7ch MEM2MEM 1/tick PL arbitrate, F4 8 streams MBURST/PBURST INCR4/8/16 FTH quarter/half/3q/full (dma_f4.rs:196) — per-request USART1 ch4/5".bright_green())),
        "tim" => Some(format!("{}", "TIM — 16/32b PSC→PCLK, ARR, UIF, OCxM 000-111 PWM1/2 OPM, BDTR MOE/BKIN DTG, encoder SMS 011 Hall 100 (timer.rs:112)".bright_green())),
        "adc" => Some(format!("{}", "ADC — F1 CR2 SWSTART, F4 dual C_ADC CDR combine, F0/H7/F3 ISR ADRDY ADEN ADSTART, JSQR/JDR JEOC, HTR/LTR AWD (adc.rs:113)".bright_green())),
        "periphs" => Some(format!("{}", "Periphs — GPIO F1 BSRR F4 MODER, RCC HSIRDY SWS→SW, PWR WUF, RTC WPR INIT, CAN 28-bank filter, USB OTG HS FS double-buffer, ETH H7 MAC, SDIO CRC16".bright_green())),
        "arch" => Some(format!("{}", "Arch — ARMv6-M (M0/M0+) / v7-M (M3/M4) / v7E-M (M7) / v8M-baseline (M23) / v8M-mainline (M33+DSP+FPv5-SP); DWT CYCCNT TRM CPI vs 1c/insn cycles; MPU regions; Thumb-2 .w needs M3+ (cpu/mod.rs:44)".bright_green())),
        "core" => Some(format!("{}", "Core — NVIC + SysTick + DWT per machine; semihosting BKPT 0xAB (WRITEC/WRITE0/READC/CLOCK/ISTTY/EXIT); ITM stimulus + RTT channel 0; backtrace ring 256, history on halt (machine.rs, semihost.rs)".bright_green())),
        "usart" => Some(format!("{}", "USART — SR/DR/BRR/CR1..3 (F1 SR@0x00 UE13, F0 ISR/RDR/TDR UE0); TXE/TC/RXNE read-to-clear; RXNEIE+UE level IRQ; LIN break LBD; DMA1 TX ch4/RX ch5 routing (usart.rs:1)".bright_green())),
        "spi" => Some(format!("{}", "SPI — CR1 SPE/MSTR/BR/DFF/CRCEN/SSM+NSS, SR TXE/RXNE/BSY/MODF, TXCRCR/RXCRCR, I2S gate; master/slave pairs over the fabric, MOSI drain + MISO inject (spi.rs:1)".bright_green())),
        "i2c" => Some(format!("{}", "I2C — master state machine; CR1 START/STOP/ACK, SR1 SB/ADDR/BTF/TXE/RXNE, 7-bit addr via DR, CCR/TRISE timing (i2c.rs:1)".bright_green())),
        "can" => Some(format!("{}", "CAN — bxCAN INRQ/INAK handshake, TX mailboxes TXRQ/RQCP/TXOK, RX FIFOs FMP/FULL/FOVR+RFOM, 28-bank mask/list filters, TX/RX0/RX1/SCE lines; FDCAN full/cut/stub tiers on H7/G0/M33 (can.rs:1, fdcan.rs)".bright_green())),
        "usb" => Some(format!("{}", "USB — OTG FS/HS reunited (GLOBAL/HOST/DEVICE/PWRCLK + FIFO RAM); device ENUMDNE/SETUP/bulk; host HPRT/SOF scheduler + virtual CDC-ACM (usb.rs:1)".bright_green())),
        "eth" => Some(format!("{}", "ETH — F1/F4 MAC+DMA descriptor rings with host inject/drain; H7 EthH7 enhanced descriptors, MACPFR filter, MDIO link-up PHY, static-IP ARP/ICMP echo (eth.rs:1)".bright_green())),
        "sdio" => Some(format!("{}", "SDIO — CMD/ARG/RESPCMD/RESP1, CMDSENT/CMDREND/DATAEND/DBCKEND, DCTRL/DLEN blocks, real CRC-16, ICR W1C, IRQ 49 (sdio.rs:1)".bright_green())),
        "fmc" => Some(format!("{}", "FMC — LTDC VSYNC, DMA2D blit TC, BDMA/MDMA channels, QSPI/OCTOSPI gates, SAI/DFSDM/HRTIM ticks; external RAM at 0xC0000000/0x90000000 (fmc.rs:1)".bright_green())),
        "h7" => Some(format!("{}", "H7 — stm32h743 2M flash/512K RAM (DTCM+AXI), FDCAN1 full M_CAN, LPTIM1-5, PLL1 + TIMPRE x4 timer ratios, FMC/QSPI (boards.md)".bright_green())),
        "run" => Some(format!("{}", "Run — .bin/.elf/.hex auto-detected; --board (family core) --base --budget (default 1M) --semihost --fast K/--jit --pin-log c13 --pin-sample N; reruns byte-identical".bright_green())),
        "trace" => Some(format!("{}", "Trace — text per-cycle snapshot, jsonl per-retire stream, vcd waveform (32-bit pc + --pin-log pins, board-clock #ns, change-only, fixed $date); --output file, stdout stays text/jsonl".bright_green())),
        "record" => Some(format!("{}", "Record — golden vioarm-golden-v1 (image digest, halt, cycles/cyccnt, channels, RAM snaps; --snap BASE:LEN default 0x20000000:0x1000); replay diffs field-by-field, --patch ADDR=VAL, exit 1 on any diff".bright_green())),
        "benchmark" => Some(format!("{}", "Benchmark — bare-nop/board-boot/usart-tx kernels on the release Machine; --cycles N --board B --fail-under <mips> exits 1; plain kernel mips wall-ms rows (docs/src/benchmark.md)".bright_green())),
        "scoreboard" => Some(format!("{}", "Scoreboard — third-party firmware proof: Dhrystone 1.135 DMIPS/MHz, mbedTLS selftests PASSED, ThreadX demo PASSED, Embench-IoT 19/19, TFLM hello_world bit-exact vs x86, CoreMark 2.067/MHz, FreeRTOS 250ms exact (docs/src/scoreboard.md)".bright_green())),
        "gdb" => Some(format!("{}", "GDB — RSP g/G regs, m/M mem, c/s continue/step, Z0 sw breakpoints (bkpt swap + step-over re-arm), H faked, D/k quit, W exit on semihosting; no watchpoints; --jit TB stops + quanta continue (vioarm-gdb/src/lib.rs:296)".bright_green())),
        "embedding" => Some(format!("{}", "Embedding — C ABI: vioarm_cosim_api_version, snapshot_save/restore (CPU+0x20000000:0x1000), get_pc/get_sp/get_cycles/get_instructions, read_data/write_data (VIOARM_ERR_BUS on unmapped), device_* / board_* catalog incl. board_pin (see docs/src/embedding.md)".bright_green())),
        "debug" => Some(format!("{}", "Debug — REPL: step(s)/next(n) continue(c) regs(r) set <reg> <val> mem <addr> [val] disas [addr] [N] break <pc|name> [if <reg> ==|!= <val>] breaks delete clear watch|rwatch|awatch <addr> unwatch snapshot restore budget history quit; ELF symbols label disas + resolve break names; halts print the 32-deep backtrace. GDB: vioarm gdb image --port 3333, then target remote :3333 (breakpoints, single-step, quanta continue, semihosting W exit)".bright_green())),
        _ => None,
    }
}

fn docs_topic(topic: Option<String>) {
    let t = topic.as_deref().unwrap_or("").to_ascii_lowercase();
    match t.as_str() {
        "" | "help" => {
            println!(
                "{}",
                "VioARM docs — instant terminal docs".bright_cyan().bold()
            );
            println!("{}", format!("topics: {}", DOC_TOPICS.join(" ")).dimmed());
            println!(
                "  {} — 21 SVD → 500 Dnames, 197 union periphs",
                "boards".bright_green()
            );
            println!(
                "  {} — GPIO/RCC/USART/TIM/ADC/SPI/I2C/CAN/DMA/USB/ETH/SDIO/FMC/H7",
                "periphs".bright_green()
            );
            println!(
                "  {} — ARMv6-M/7-M/7E-M/8M-baseline M4F M7 FPv5-D16, DWT cyccnt, MPU",
                "arch".bright_green()
            );
            println!("  {} — C ABI version/snapshots/introspection/memory/catalog (see docs/src/embedding.md)", "embedding".bright_green());
            println!(
                "  {} — run `vioarm boards` for 21 tables, `vioarm run <hex> --board stm32h743`",
                "quick".bright_yellow()
            );
        }
        "boards" => {
            println!(
                "{}",
                "Boards — 21 SVD tables → 500 Dnames (vioarm-core/src/board.rs:2119)"
                    .bright_cyan()
                    .bold()
            );
            for (name, peris) in board::all_chip_tables() {
                let len = peris.len();
                println!(
                    "  {:12} {:3} periphs {}",
                    name.bright_green(),
                    len.to_string().dimmed(),
                    format!("0x{:08x}", peris[0].base).dimmed()
                );
            }
            println!("{}", "pins: vioarm boards --pins stm32f103 (13 tables, default-AF labels; see docs/src/boards.md)".dimmed());
        }
        _ => match docs_text(t.as_str()) {
            Some(text) => println!("{text}"),
            None => println!("{} '{t}' — try `vioarm docs help`", "unknown topic".red()),
        },
    }
}

fn debug_repl(image: PathBuf, board: Board, base: u32) {
    let (img, kind) = load_image_auto(&image, base);
    let symbols = img.symbols.clone();
    if !symbols.is_empty() {
        println!("{} {} symbols", "symbols:".dimmed(), symbols.len());
    }
    let mut m = machine_for(board, &img, (kind == "bin").then_some(base));
    println!(
        "{} {} — type {} for help",
        "VioARM debug".bright_cyan().bold(),
        format!("{} {}", board.name(), image.display()).dimmed(),
        "help".bright_green()
    );
    let (regs, xpsr, _) = m.core.snapshot();
    println!(
        "{} pc={:#010x} sp={:#010x} xpsr={:#010x}",
        "attached:".dimmed(),
        regs.pc,
        regs.sp,
        xpsr.get()
    );
    let mut rl = String::new();
    // Session state: persistent breakpoints (`break`, honored by
    // `continue`/`budget`), hook watchpoints (`watch`, tripping through
    // `tripped`), and one snapshot slot (`snapshot`/`restore`).
    let mut breakpoints: Vec<Breakpoint> = Vec::new();
    let watches: std::rc::Rc<std::cell::RefCell<Vec<Watch>>> = Default::default();
    let tripped: std::rc::Rc<std::cell::RefCell<Option<(u32, bool)>>> = Default::default();
    let mut snapshot: Option<Vec<u8>> = None;
    loop {
        print!("{}", "(vioarm) ".bright_magenta().bold());
        use std::io::Write;
        let _ = std::io::stdout().flush();
        rl.clear();
        if std::io::stdin().read_line(&mut rl).is_err() {
            break;
        }
        let cmd = rl.trim();
        match cmd {
            "help" | "h" | "?" => {
                println!("{}: step(s)/next(n) continue(c) regs(r) set <reg> <val> mem <addr> [val] disas [addr] [N] break <pc|name> [if <reg> ==|!= <val>] breaks delete <pc|name> clear watch|rw|aw <addr> unwatch <addr> snapshot restore budget <n> history [N] quit(q)", "commands".bright_green());
            }
            "step" | "s" | "n" | "next" => match m.step() {
                Ok(_) => {
                    let (regs, xpsr, _) = m.core.snapshot();
                    let cyccnt = m.core.cyccnt;
                    println!(
                        "{} pc={:#010x} r0={:#010x} xpsr={:#010x} cyccnt={cyccnt}",
                        "step".dimmed(),
                        regs.pc,
                        regs.r[0],
                        xpsr.get()
                    );
                }
                Err(e) => {
                    println!("{} {e}", "halted:".yellow(),);
                    print_backtrace(&mut m, 32);
                }
            },
            "regs" | "r" => {
                let (regs, xpsr, _) = m.core.snapshot();
                let cyccnt = m.core.cyccnt;
                println!("pc={:#010x} sp={:#010x} lr={:#010x} r0={:#010x} r1={:#010x} r2={:#010x} r3={:#010x} xpsr={:#010x} cyccnt={cyccnt}",
                    regs.pc, regs.sp, regs.lr, regs.r[0], regs.r[1], regs.r[2], regs.r[3], xpsr.get());
            }
            c if c.starts_with("set ") => {
                // `set r0 0x12` / `set sp 0x20001000` / `set pc <addr>`.
                let mut parts = c.split_whitespace();
                let (name, val) = (parts.nth(1), parts.next());
                match (name, val) {
                    (Some(name), Some(val)) => {
                        let reg = parse_reg_name(name);
                        match (reg, parse_u32(val)) {
                            (Some(n), Ok(v)) => {
                                m.core.regs.write(n, v);
                                println!("{} {name}={v:#010x}", "set".dimmed());
                            }
                            _ => println!("{} set <r0-r12|sp|lr|pc> <hex val>", "usage:".yellow()),
                        }
                    }
                    _ => println!("{} set <r0-r12|sp|lr|pc> <hex val>", "usage:".yellow()),
                }
            }
            c if c.starts_with("mem ") => {
                // `mem <addr>` reads one word; `mem <addr> <val>` writes it.
                let mut parts = c.split_whitespace();
                let (addr, val) = (parts.nth(1), parts.next());
                match (addr, val) {
                    (Some(addr), None) => match parse_u32(addr) {
                        Ok(a) => match m.core.mem().read_word(a) {
                            Ok(v) => println!("{a:#010x}: {v:#010x}"),
                            Err(e) => println!("{} {e}", "bus error:".red()),
                        },
                        Err(_) => println!("{} mem <hex addr> [hex val]", "usage:".yellow()),
                    },
                    (Some(addr), Some(val)) => match (parse_u32(addr), parse_u32(val)) {
                        (Ok(a), Ok(v)) => match m.core.mem().write_word(a, v) {
                            Ok(()) => println!("{} {a:#010x} <- {v:#010x}", "wrote".dimmed()),
                            Err(e) => println!("{} {e}", "bus error:".red()),
                        },
                        _ => println!("{} mem <hex addr> [hex val]", "usage:".yellow()),
                    },
                    _ => println!("{} mem <hex addr> [hex val]", "usage:".yellow()),
                }
            }
            c if c == "disas" || c.starts_with("disas ") => {
                // `disas [addr] [N]`: disassemble N instructions (default 8
                // at the live pc) over the shared decoders.
                let mut parts = c.split_whitespace();
                let live_pc = m.core.regs.pc;
                let addr = parts
                    .nth(1)
                    .map_or(Ok(live_pc), parse_u32)
                    .unwrap_or_else(|_| {
                        println!("{} disas [hex addr] [count]", "usage:".yellow());
                        live_pc
                    });
                let count = parts
                    .next()
                    .map_or(8, |n| n.parse::<usize>().unwrap_or(8).clamp(1, 64));
                let isa = m.core.isa;
                let mut at = addr;
                for _ in 0..count {
                    match m.core.mem().read_half(at) {
                        Ok(h0) => {
                            let h1 = m.core.mem().read_half(at.wrapping_add(2)).ok();
                            let (text, width) = vioarm_core::disas::disassemble(isa, at, h0, h1);
                            let sym = symbolize(&symbols, at)
                                .map(|(name, off)| {
                                    if off == 0 {
                                        format!(" <{name}>")
                                    } else {
                                        format!(" <{name}+{off:#x}>")
                                    }
                                })
                                .unwrap_or_default();
                            if h1.is_some_and(|h| width == 4) {
                                println!(
                                    "{at:#010x}: {h0:04x} {h:04x}  {text}{sym}",
                                    h = h1.expect("checked")
                                );
                            } else {
                                println!("{at:#010x}: {h0:04x}       {text}{sym}");
                            }
                            at = at.wrapping_add(width);
                        }
                        Err(e) => {
                            println!("{at:#010x}: {} {e}", "bus error:".red());
                            break;
                        }
                    }
                }
            }
            c if c.starts_with("break ") => {
                // `break <pc|name> [if <reg> ==|!= <val>]`: arms a
                // persistent conditional breakpoint (listed by `breaks`,
                // honored by `continue`/`budget`, dropped by
                // `delete`/`clear`).
                match parse_break_args(c[6..].trim(), &symbols) {
                    Some((a, cond)) => {
                        if breakpoints.iter().any(|bp| bp.pc == a) {
                            println!("{} break at {a:#010x} already armed", "break".yellow());
                        } else {
                            breakpoints.push(Breakpoint { pc: a, cond });
                            breakpoints.sort_by_key(|bp| bp.pc);
                            match cond {
                                None => println!("{} break at {a:#010x}", "break".bright_green()),
                                Some(gate) => println!(
                                    "{} break at {a:#010x} if {} {} {:#010x}",
                                    "break".bright_green(),
                                    vioarm_core::disas::reg_name(gate.reg),
                                    if gate.invert { "!=" } else { "==" },
                                    gate.val
                                ),
                            }
                        }
                    }
                    None => println!(
                        "{} break <hex pc|name> [if <reg> ==|!= <hex val>]",
                        "usage:".yellow()
                    ),
                }
            }
            "break" | "breaks" => {
                if breakpoints.is_empty() {
                    println!("{}", "no breakpoints".dimmed());
                } else {
                    for (i, bp) in breakpoints.iter().enumerate() {
                        match bp.cond {
                            None => println!("  {i}: {:#010x}", bp.pc),
                            Some(gate) => println!(
                                "  {i}: {:#010x} if {} {} {:#010x}",
                                bp.pc,
                                vioarm_core::disas::reg_name(gate.reg),
                                if gate.invert { "!=" } else { "==" },
                                gate.val
                            ),
                        }
                    }
                }
                let set = watches.borrow();
                if !set.is_empty() {
                    for w in set.iter() {
                        let kind = match (w.read, w.write) {
                            (true, true) => "awatch",
                            (true, false) => "rwatch",
                            _ => "watch",
                        };
                        println!("  {kind} {:#010x}", w.addr);
                    }
                }
            }
            c if c.starts_with("delete ") => match resolve_addr(c[7..].trim(), &symbols) {
                Some(a) => {
                    if let Some(pos) = breakpoints.iter().position(|bp| bp.pc == a) {
                        breakpoints.remove(pos);
                        println!("{} break at {a:#010x} cleared", "delete".dimmed());
                    } else {
                        println!("{} no break at {a:#010x}", "delete".yellow());
                    }
                }
                None => println!("{} delete <hex pc|name>", "usage:".yellow()),
            },
            "clear" => {
                breakpoints.clear();
                watches.borrow_mut().clear();
                tripped.borrow_mut().take();
                println!("{}", "breakpoints and watches cleared".dimmed());
            }
            c if c == "watch" || c.starts_with("watch ") => {
                // `watch <addr>`: stop after a step that writes the word.
                match c.split_whitespace().nth(1) {
                    None => {
                        let set = watches.borrow();
                        if set.iter().any(|w| w.write) {
                            for w in set.iter().filter(|w| w.write) {
                                println!("  watch {:#010x}", w.addr);
                            }
                        } else {
                            println!("{}", "no write watches".dimmed());
                        }
                    }
                    Some(arg) => match parse_u32(arg) {
                        Ok(a) => {
                            if add_watch(&mut m, watches.clone(), tripped.clone(), a, false, true) {
                                println!("{} watch {:#010x} (writes)", "watch".bright_green(), a);
                            } else {
                                println!("{} watchpoints need a hook registry", "watch".yellow());
                            }
                        }
                        Err(_) => println!("{} watch <hex addr>", "usage:".yellow()),
                    },
                }
            }
            c if c.starts_with("rwatch ") => {
                // `rwatch <addr>`: stop after a step that reads the word.
                match parse_u32(c[7..].trim()) {
                    Ok(a) => {
                        if add_watch(&mut m, watches.clone(), tripped.clone(), a, true, false) {
                            println!("{} watch {:#010x} (reads)", "rwatch".bright_green(), a);
                        } else {
                            println!("{} watchpoints need a hook registry", "rwatch".yellow());
                        }
                    }
                    Err(_) => println!("{} rwatch <hex addr>", "usage:".yellow()),
                }
            }
            c if c.starts_with("awatch ") => {
                // `awatch <addr>`: stop after a step that reads or writes.
                match parse_u32(c[7..].trim()) {
                    Ok(a) => {
                        if add_watch(&mut m, watches.clone(), tripped.clone(), a, true, true) {
                            println!("{} watch {:#010x} (access)", "awatch".bright_green(), a);
                        } else {
                            println!("{} watchpoints need a hook registry", "awatch".yellow());
                        }
                    }
                    Err(_) => println!("{} awatch <hex addr>", "usage:".yellow()),
                }
            }
            c if c.starts_with("unwatch ") => match parse_u32(c[8..].trim()) {
                Ok(a) => {
                    let mut set = watches.borrow_mut();
                    let before = set.len();
                    set.retain(|w| w.addr != a);
                    if set.len() < before {
                        println!("{} watch {:#010x} cleared", "unwatch".dimmed(), a);
                    } else {
                        println!("{} no watch at {a:#010x}", "unwatch".yellow());
                    }
                }
                Err(_) => println!("{} unwatch <hex addr>", "usage:".yellow()),
            },
            "snapshot" => {
                // Freeze CPU + the snapshot RAM span into the session slot.
                let mut buf = vec![0u8; m.snapshot_len()];
                match m.snapshot_save(&mut buf) {
                    Ok(len) => {
                        buf.truncate(len);
                        snapshot = Some(buf);
                        println!("{} snapshot {len} bytes", "snapshot".bright_green());
                    }
                    Err(e) => println!("{} snapshot failed: {e}", "error:".red().bold()),
                }
            }
            "restore" => {
                // Rewind to the session snapshot (kept for repeated rewinds).
                match &snapshot {
                    None => println!("{} no snapshot — run `snapshot` first", "restore".yellow()),
                    Some(buf) => match m.snapshot_restore(buf) {
                        Ok(()) => println!(
                            "{} rewound to pc={:#010x}",
                            "restore".bright_green(),
                            m.core.regs.pc
                        ),
                        Err(e) => println!("{} restore failed: {e}", "error:".red().bold()),
                    },
                }
            }
            c if c.starts_with("budget ") => {
                if let Ok(n) = c[7..].trim().parse::<u64>() {
                    match run_until_break(&mut m, &breakpoints, &tripped, n) {
                        ReplStop::Exhausted => {
                            println!("{}", "budget exhausted".yellow())
                        }
                        ReplStop::Hit(pc) => {
                            println!("{} break at {pc:#010x}", "hit".green())
                        }
                        ReplStop::Watch { addr, read } => {
                            println!(
                                "{} {} at {addr:#010x}",
                                "watchpoint".green(),
                                if read { "read" } else { "write" }
                            );
                        }
                        ReplStop::Halt(e) => {
                            println!("{} {e}", "halted:".green());
                            print_backtrace(&mut m, 32);
                        }
                    }
                }
            }
            "continue" | "c" | "run" => {
                match run_until_break(&mut m, &breakpoints, &tripped, 1_000_000) {
                    ReplStop::Exhausted => {
                        println!("{}", "budget exhausted — 1M".yellow())
                    }
                    ReplStop::Hit(pc) => {
                        println!("{} break at {pc:#010x}", "hit".green())
                    }
                    ReplStop::Watch { addr, read } => {
                        println!(
                            "{} {} at {addr:#010x}",
                            "watchpoint".green(),
                            if read { "read" } else { "write" }
                        );
                    }
                    ReplStop::Halt(e) => {
                        println!("{} {e}", "halted:".green());
                        print_backtrace(&mut m, 32);
                    }
                }
            }
            c if c == "history"
                || c == "hist"
                || c.starts_with("history ")
                || c.starts_with("hist ") =>
            {
                let arg = c.split_whitespace().nth(1);
                let max = arg.map_or(32, |n| n.parse::<usize>().unwrap_or(32));
                print_backtrace(&mut m, max.max(1));
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
            Commands::Run {
                image,
                board,
                base,
                budget,
                semihost,
                fast,
                jit,
                pin_log,
                pin_sample,
            } => {
                if jit && fast != 0 {
                    eprintln!(
                        "{} --jit needs --fast 0 (quanta need the accurate time model)",
                        "error:".red().bold()
                    );
                    std::process::exit(2);
                }
                let (img, kind) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img, (kind == "bin").then_some(base));
                m.semihost_enable(semihost);
                m.set_fast_quantum(fast);
                let mut pins = Vec::with_capacity(pin_log.len());
                for spec in &pin_log {
                    match parse_pin_spec(spec) {
                        Ok(parsed) => pins.push(parsed),
                        Err(cause) => {
                            eprintln!("{} {cause}", "error:".red().bold());
                            std::process::exit(2);
                        }
                    }
                }
                for (port, pin, name) in &pins {
                    if m.gpio_pin(*port, *pin).is_none() {
                        eprintln!(
                            "{} --pin-log entry '{name}': no such pin on this board",
                            "error:".red().bold()
                        );
                        std::process::exit(2);
                    }
                }
                banner();
                run_quiet(&mut m, budget, jit, &pins, pin_sample);
            }
            Commands::Record {
                image,
                board,
                base,
                budget,
                semihost,
                fast,
                output,
                snap,
                jit,
            } => {
                use vioarm_core::replay::{parse_snap, record, RecordMeta};
                let (img, _) = load_image_auto(&image, base);
                let image_bytes = flat_image_bytes(&img);
                let mut m = machine_for(board, &img, None);
                m.semihost_enable(semihost);
                m.set_fast_quantum(fast);
                let spans: Vec<(u32, u32)> = if snap.is_empty() {
                    vec![(0x2000_0000, 0x1000)]
                } else {
                    snap.iter()
                        .map(|s| {
                            parse_snap(s).unwrap_or_else(|e| {
                                eprintln!("{} {e}", "error:".red().bold());
                                std::process::exit(2);
                            })
                        })
                        .collect()
                };
                if jit && fast != 0 {
                    eprintln!(
                        "{} --jit needs --fast 0 (quanta need the accurate time model)",
                        "error:".red().bold()
                    );
                    std::process::exit(2);
                }
                let meta = RecordMeta {
                    board: format!("{board:?}").to_ascii_lowercase(),
                    base,
                    budget,
                    semihost,
                    fast,
                    jit,
                };
                match record(&mut m, &meta, &image_bytes, &spans) {
                    Ok(golden) => {
                        std::fs::write(&output, golden.serialize()).unwrap_or_else(|e| {
                            eprintln!(
                                "{} cannot write {}: {e}",
                                "error:".red().bold(),
                                output.display()
                            );
                            std::process::exit(2);
                        });
                        println!(
                            "{} {} halt={} cycles={} cyccnt={}",
                            "recorded".green().bold(),
                            output.display(),
                            golden.halt,
                            golden.cycles,
                            golden.cyccnt
                        );
                    }
                    Err(e) => {
                        eprintln!("{} record failed: {e}", "error:".red().bold());
                        std::process::exit(1);
                    }
                }
            }
            Commands::Replay {
                image,
                golden,
                board,
                base,
                patch,
            } => {
                use vioarm_core::replay::{parse_patch, replay, Golden};
                let (img, _) = load_image_auto(&image, base);
                let image_bytes = flat_image_bytes(&img);
                let text = std::fs::read_to_string(&golden).unwrap_or_else(|e| {
                    eprintln!(
                        "{} cannot read {}: {e}",
                        "error:".red().bold(),
                        golden.display()
                    );
                    std::process::exit(2);
                });
                let golden = Golden::parse(&text).unwrap_or_else(|e| {
                    eprintln!("{} bad golden file: {e}", "error:".red().bold());
                    std::process::exit(2);
                });
                let patches: Vec<(u32, u8)> = patch
                    .iter()
                    .map(|s| {
                        parse_patch(s).unwrap_or_else(|e| {
                            eprintln!("{} {e}", "error:".red().bold());
                            std::process::exit(2);
                        })
                    })
                    .collect();
                let spans: Vec<(u32, u32)> =
                    golden.snaps.iter().map(|(b, l, _)| (*b, *l)).collect();
                let mut m = machine_for(board, &img, None);
                m.semihost_enable(golden.semihost);
                let mut diffs = Vec::new();
                let want_board = format!("{board:?}").to_ascii_lowercase();
                if want_board != golden.board {
                    diffs.push(format!(
                        "board: golden {} != replay {want_board}",
                        golden.board
                    ));
                }
                diffs.extend(replay(&mut m, &golden, &image_bytes, &patches, &spans));
                if diffs.is_empty() {
                    println!("{}", "MATCH".green().bold());
                } else {
                    for d in &diffs {
                        println!("{} {d}", "diff:".yellow().bold());
                    }
                    std::process::exit(1);
                }
            }
            Commands::Trace {
                image,
                board,
                base,
                budget,
                semihost,
                format,
                output,
                pin_log,
                pin_sample,
            } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img, None);
                m.semihost_enable(semihost);
                banner();
                match format {
                    TraceFormat::Jsonl => run_traced_jsonl(&mut m, budget, output),
                    TraceFormat::Text => run_traced(&mut m, budget),
                    TraceFormat::Vcd => {
                        if pin_sample == 0 {
                            eprintln!("{} --pin-sample must be at least 1", "error:".red().bold());
                            std::process::exit(2);
                        }
                        let mut pins = Vec::with_capacity(pin_log.len());
                        for spec in &pin_log {
                            match parse_pin_spec(spec) {
                                Ok(parsed) => pins.push(parsed),
                                Err(cause) => {
                                    eprintln!("{} {cause}", "error:".red().bold());
                                    std::process::exit(2);
                                }
                            }
                        }
                        for (port, pin, name) in &pins {
                            if m.gpio_pin(*port, *pin).is_none() {
                                eprintln!(
                                    "{} --pin-log entry '{name}': no such pin on this board",
                                    "error:".red().bold()
                                );
                                std::process::exit(2);
                            }
                        }
                        run_traced_vcd(&mut m, budget, output, &pins, pin_sample);
                    }
                }
            }
            Commands::Benchmark {
                board,
                cycles,
                fail_under,
            } => {
                banner();
                let mut results = Vec::new();
                match bench_bare_nop(cycles) {
                    Ok(r) => results.push(r),
                    Err(e) => {
                        eprintln!("{} {e}", "error:".red().bold());
                        std::process::exit(1);
                    }
                }
                match bench_board_boot(board, cycles) {
                    Ok(r) => results.push(r),
                    Err(e) => {
                        eprintln!("{} {e}", "error:".red().bold());
                        std::process::exit(1);
                    }
                }
                match bench_usart_tx(board, cycles) {
                    Ok(Some(r)) => results.push(r),
                    Ok(None) => eprintln!(
                        "{} usart-tx skipped: no F1-style USART1 on {}",
                        "note:".dimmed(),
                        board.name()
                    ),
                    Err(e) => {
                        eprintln!("{} {e}", "error:".red().bold());
                        std::process::exit(1);
                    }
                }
                for r in &results {
                    println!("{}", bench_row(r));
                }
                if let Some(floor) = fail_under {
                    let bad = bench_floor_failures(&results, floor);
                    if !bad.is_empty() {
                        eprintln!(
                            "{} benchmark floor {floor} MIPS missed by: {}",
                            "fail:".red().bold(),
                            bad.join(", ")
                        );
                        std::process::exit(1);
                    }
                }
            }
            Commands::Info { image } => {
                let (img, _) = load_image_auto(&image, 0);
                print_info(&img);
            }
            Commands::Boards { pins } => {
                if let Some(which) = pins {
                    match render_pin_tables(&which) {
                        Ok(text) => print!("{text}"),
                        Err(message) => {
                            eprintln!("{} {message}", "error:".red().bold());
                            std::process::exit(2);
                        }
                    }
                    return;
                }
                banner();
                println!(
                    "{}",
                    "Boards — 21 SVD tables → 500 Dnames".bright_cyan().bold()
                );
                for (name, peris) in board::all_chip_tables() {
                    println!(
                        "  {:12} {:3} periphs {}",
                        name.bright_green(),
                        peris.len().to_string().dimmed(),
                        format!("board::{}", name).dimmed()
                    );
                }
                println!(
                    "{}",
                    "run: vioarm run firmware.hex --board stm32h743".dimmed()
                );
            }
            Commands::Docs { topic } => docs_topic(topic),
            Commands::Debug { image, board, base } => debug_repl(image, board, base),
            Commands::Gdb {
                image,
                port,
                board,
                base,
                semihost,
                jit,
            } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img, None);
                m.semihost_enable(semihost);
                let listener =
                    std::net::TcpListener::bind(("127.0.0.1", port)).unwrap_or_else(|e| {
                        eprintln!(
                            "{} cannot listen on port {port}: {e}",
                            "error:".red().bold()
                        );
                        std::process::exit(3);
                    });
                println!(
                    "{} waiting for gdb on 127.0.0.1:{port}",
                    "gdbserver:".bright_magenta().bold()
                );
                if let Err(e) = vioarm_gdb::serve_jit(&mut m, &listener, jit) {
                    eprintln!("{} gdb session error: {e}", "error:".red().bold());
                    std::process::exit(3);
                }
            }
        }
        return;
    }
    // Legacy flat flags fallback (vioarm --bin/--elf/--hex --board --trace --gdb)
    // `flat_base` tracks a raw-binary load so it boots from its base.
    let (img, flat_base): (LoadedImage, Option<u32>) = if let Some(path) = cli.elf {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        (
            loader::load_elf(&bytes).unwrap_or_else(|e| {
                eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
                std::process::exit(2);
            }),
            None,
        )
    } else if let Some(path) = cli.hex {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        (
            loader::load_hex(&bytes).unwrap_or_else(|e| {
                eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
                std::process::exit(2);
            }),
            None,
        )
    } else if let Some(path) = cli.bin {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        (loader::load_flat(cli.base, &bytes), Some(cli.base))
    } else {
        eprintln!(
            "{} provide --bin/--elf/--hex or subcommand `run`",
            "error:".red().bold()
        );
        Cli::parse_from(["vioarm", "--help"]);
        std::process::exit(2);
    };
    if cli.info {
        for (addr, bytes) in &img.segments {
            let end = u32::try_from(bytes.len()).map_or(u32::MAX, |len| addr.saturating_add(len));
            println!("segment: {addr:#010x}..{end:#010x} ({} bytes)", bytes.len());
        }
        if let Some(entry) = img.entry {
            println!("entry: {entry:#010x}");
        }
        return;
    }
    let mut m = machine_for(cli.board, &img, flat_base);
    m.semihost_enable(cli.semihost);
    if let Some(port) = cli.gdb {
        let listener = std::net::TcpListener::bind(("127.0.0.1", port)).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot listen on port {port}: {e}",
                "error:".red().bold()
            );
            std::process::exit(3);
        });
        println!(
            "{} waiting for gdb on 127.0.0.1:{port}",
            "gdbserver:".bright_magenta().bold()
        );
        if let Err(e) = vioarm_gdb::serve_jit(&mut m, &listener, false) {
            eprintln!("{} gdb session error: {e}", "error:".red().bold());
            std::process::exit(3);
        }
        return;
    }
    banner();
    if cli.trace {
        run_traced(&mut m, cli.budget);
    } else {
        run_quiet(&mut m, cli.budget, false, &[], 0);
    }
    for device in m.core.mem().devices_mut() {
        let tx = device.drain_output();
        if !tx.is_empty() {
            println!(
                "{} {}",
                "uart:".bright_magenta().bold(),
                String::from_utf8_lossy(&tx)
            );
        }
    }
}

#[cfg(test)]
mod pin_spec_tests {
    use super::parse_pin_spec;

    #[test]
    fn accepts_boards_pins() {
        assert_eq!(parse_pin_spec("c13").unwrap(), (2, 13, "c13".to_owned()));
        assert_eq!(parse_pin_spec("A0").unwrap(), (0, 0, "A0".to_owned()));
        assert_eq!(
            parse_pin_spec("  k15 ").unwrap(),
            (10, 15, "k15".to_owned())
        );
    }

    #[test]
    fn rejects_garbage() {
        for bad in ["", "13", "c", "c32", "z0", "c-1", "cc1", "c1x"] {
            assert!(parse_pin_spec(bad).is_err(), "{bad}");
        }
    }
}

#[cfg(test)]
mod docs_tests {
    use super::{docs_text, DOC_TOPICS};

    #[test]
    fn every_advertised_topic_resolves() {
        // `boards` renders a table (not through `docs_text`); everything
        // else advertised must resolve to a line.
        assert!(DOC_TOPICS.contains(&"boards"));
        for topic in DOC_TOPICS {
            if *topic == "boards" {
                continue;
            }
            let text = docs_text(topic).unwrap_or_else(|| panic!("dead topic: {topic}"));
            assert!(!text.is_empty(), "{topic}");
        }
    }

    #[test]
    fn unknown_topics_stay_unknown() {
        assert!(docs_text("nope").is_none());
        assert!(docs_text("").is_none());
        assert!(docs_text("BOARDs").is_none(), "lookup is lowercase-exact");
    }

    #[test]
    fn topic_lines_name_their_subject() {
        let gdb = docs_text("gdb").expect("gdb");
        assert!(gdb.contains("Z0"), "{gdb}");
        assert!(gdb.contains("watchpoint"), "{gdb}");
        let run = docs_text("run").expect("run");
        assert!(run.contains("--budget"), "{run}");
        let record = docs_text("record").expect("record");
        assert!(record.contains("--snap"), "{record}");
    }
}

#[cfg(test)]
mod pin_catalog_tests {

    use super::render_pin_tables;

    #[test]
    fn single_board_renders_default_af_rows() {
        let text = render_pin_tables("stm32f103").expect("known board");
        assert!(text.contains("board stm32f103"), "{text}");
        assert!(text.contains("PA9"), "{text}");
        assert!(text.contains("USART1_TX"), "{text}");
        assert!(text.contains("PA10"), "{text}");
        assert!(text.contains("USART1_RX"), "{text}");
        assert!(text.contains("SWDIO"), "{text}");
        assert!(text.contains("SWCLK"), "{text}");
        assert!(text.contains("PC13"), "{text}");
    }

    #[test]
    fn all_renders_twenty_tables() {
        let text = render_pin_tables("all").expect("all tables");
        for board in vioarm_core::pins::PIN_BOARDS {
            assert!(text.contains(&format!("board {board}")), "{board}");
        }
        assert_eq!(
            text.lines()
                .filter(|line| line.starts_with("board "))
                .count(),
            20
        );
    }

    #[test]
    fn unknown_board_errors_with_valid_list() {
        let err = render_pin_tables("stm32f999").expect_err("unknown fails");
        assert!(err.contains("stm32f999"), "{err}");
        assert!(err.contains("stm32f103"), "{err}");
        assert!(err.contains("stm32h743"), "{err}");
    }
}

#[cfg(test)]
mod repl_tests {
    use super::{
        add_watch, break_hits, find_symbol, parse_break_args, parse_reg_name, resolve_addr,
        run_until_break, symbolize, Breakpoint, ReplStop, Watch,
    };
    use vioarm_core::loader::{load_flat, Symbol};
    use vioarm_core::machine::Machine;

    const ENTRY: u32 = 0x0000_0100;

    fn boot(code: &[u8]) -> Machine<vioarm_core::board::BoardMemory> {
        let mut m = Machine::new(vioarm_core::board::virtual_board());
        m.load_image(&load_flat(ENTRY, code));
        m.core.regs.pc = ENTRY;
        m.core.xpsr.set(0x0100_0000);
        m
    }

    fn no_watch() -> std::rc::Rc<std::cell::RefCell<Option<(u32, bool)>>> {
        std::rc::Rc::new(std::cell::RefCell::new(None))
    }

    fn bp(pc: u32) -> Breakpoint {
        Breakpoint { pc, cond: None }
    }

    #[test]
    fn reg_names_cover_aliases() {
        assert_eq!(parse_reg_name("r0"), Some(0));
        assert_eq!(parse_reg_name("R12"), Some(12));
        assert_eq!(parse_reg_name("sp"), Some(13));
        assert_eq!(parse_reg_name("r13"), Some(13));
        assert_eq!(parse_reg_name("LR"), Some(14));
        assert_eq!(parse_reg_name("pc"), Some(15));
        assert_eq!(parse_reg_name("r15"), Some(15));
        assert_eq!(parse_reg_name("r16"), None);
        assert_eq!(parse_reg_name("xpsr"), None);
        assert_eq!(parse_reg_name(""), None);
    }

    #[test]
    fn break_hits_and_steps_over_parked() {
        // `movs r0, #1; movs r1, #2; b .` — breakpoint on the loop parks
        // after retiring the two moves.
        let code = [0x01, 0x20, 0x02, 0x21, 0xFE, 0xE7];
        let mut m = boot(&code);
        match run_until_break(&mut m, &[bp(ENTRY + 4)], &no_watch(), 16) {
            ReplStop::Hit(pc) => assert_eq!(pc, ENTRY + 4),
            other => panic!("expected a hit, got {other:?}"),
        }
        assert_eq!(m.core.regs.r[0], 1);
        assert_eq!(m.core.regs.r[1], 2);
        // Parked on the breakpoint: `continue` steps over once, then
        // re-stops (the spin never leaves).
        match run_until_break(&mut m, &[bp(ENTRY + 4)], &no_watch(), 16) {
            ReplStop::Hit(pc) => assert_eq!(pc, ENTRY + 4),
            other => panic!("expected a re-hit, got {other:?}"),
        }
    }

    #[test]
    fn empty_breakpoints_exhausts_budget() {
        let code = [0xFE, 0xE7];
        let mut m = boot(&code);
        match run_until_break(&mut m, &[], &no_watch(), 4) {
            ReplStop::Exhausted => {}
            other => panic!("expected exhaustion, got {other:?}"),
        }
        assert_eq!(m.core.regs.pc, ENTRY);
    }

    #[test]
    fn faulting_code_halts_with_cause() {
        // Bare `BKPT` faults without a debugger attached.
        let code = [0x00, 0xBE];
        let mut m = boot(&code);
        match run_until_break(&mut m, &[bp(0xDEAD_BEEF)], &no_watch(), 4) {
            ReplStop::Halt(_) => {}
            other => panic!("expected a halt, got {other:?}"),
        }
    }

    #[test]
    fn conditional_break_fires_only_on_match() {
        // `movs r0, #1; b .`: the gate sees r0 == 1 only after the move.
        let code = [0x01, 0x20, 0xFE, 0xE7];
        let gate = super::BreakCond {
            reg: 0,
            invert: false,
            val: 1,
        };
        let mut m = boot(&code);
        // At ENTRY with r0 == 0 the gate is closed: steps over, exhausts.
        let closed = [Breakpoint {
            pc: ENTRY,
            cond: Some(gate),
        }];
        match run_until_break(&mut m, &closed, &no_watch(), 1) {
            ReplStop::Exhausted => {}
            other => panic!("gate closed, expected exhaustion, got {other:?}"),
        }
        // Spinning at ENTRY+2 with r0 == 1: the same gate fires.
        let open = [Breakpoint {
            pc: ENTRY + 2,
            cond: Some(gate),
        }];
        match run_until_break(&mut m, &open, &no_watch(), 4) {
            ReplStop::Hit(pc) => assert_eq!(pc, ENTRY + 2),
            other => panic!("gate open, expected a hit, got {other:?}"),
        }
        // Inverted gate never fires here.
        let never = [Breakpoint {
            pc: ENTRY + 2,
            cond: Some(super::BreakCond {
                reg: 0,
                invert: true,
                val: 1,
            }),
        }];
        match run_until_break(&mut m, &never, &no_watch(), 2) {
            ReplStop::Exhausted => {}
            other => panic!("inverted gate, expected exhaustion, got {other:?}"),
        }
    }

    #[test]
    fn break_args_parse_conditions_and_names() {
        let syms = [Symbol {
            addr: 0x100,
            name: "main".to_owned(),
        }];
        assert_eq!(parse_break_args("0x10", &[]), Some((0x10, None)));
        assert_eq!(parse_break_args("main", &syms), Some((0x100, None)));
        assert_eq!(
            parse_break_args("0x10 if r0 == 0x5a", &[]),
            Some((
                0x10,
                Some(super::BreakCond {
                    reg: 0,
                    invert: false,
                    val: 0x5a
                })
            ))
        );
        assert_eq!(
            parse_break_args("main if SP != 0x2000", &syms),
            Some((
                0x100,
                Some(super::BreakCond {
                    reg: 13,
                    invert: true,
                    val: 0x2000
                })
            ))
        );
        assert_eq!(parse_break_args("nope", &syms), None);
        assert_eq!(parse_break_args("0x10 if r99 == 1", &[]), None);
        assert_eq!(parse_break_args("0x10 if r0 = 1", &[]), None);
        assert_eq!(resolve_addr("main", &syms), Some(0x100));
        assert_eq!(resolve_addr("0x20", &syms), Some(0x20));
        assert_eq!(resolve_addr("nope", &syms), None);
    }

    #[test]
    fn symbols_resolve_nearest_below() {
        let syms = [
            Symbol {
                addr: 0x100,
                name: "reset".to_owned(),
            },
            Symbol {
                addr: 0x200,
                name: "main".to_owned(),
            },
        ];
        assert_eq!(find_symbol(&syms, "main"), Some(0x200));
        assert_eq!(find_symbol(&syms, "missing"), None);
        assert_eq!(symbolize(&syms, 0x100), Some(("reset", 0)));
        assert_eq!(symbolize(&syms, 0x204), Some(("main", 4)));
        assert_eq!(symbolize(&syms, 0x80), None);
        assert_eq!(symbolize(&[], 0x100), None);
    }

    #[test]
    fn break_hits_helper_respects_gates() {
        let code = [0x01, 0x20, 0xFE, 0xE7];
        let m = boot(&code);
        assert!(break_hits(&m, &bp(ENTRY)));
        let gated = Breakpoint {
            pc: ENTRY,
            cond: Some(super::BreakCond {
                reg: 0,
                invert: false,
                val: 1,
            }),
        };
        assert!(!break_hits(&m, &gated), "r0 is still 0");
        assert!(!break_hits(&m, &bp(ENTRY + 2)));
    }

    #[test]
    fn write_watch_trips_through_run_until_break() {
        // `movs r0, #0x5A; str r0, [r1]` with r1 preset to RAM: the store
        // trips a write watch on the word.
        const RAM: u32 = 0x2000_0000;
        let code = [0x5A, 0x20, 0x08, 0x60, 0xFE, 0xE7];
        let mut m = boot(&code);
        m.core.regs.r[1] = RAM;
        let watches: std::rc::Rc<std::cell::RefCell<Vec<Watch>>> = Default::default();
        let tripped = no_watch();
        assert!(add_watch(
            &mut m,
            watches,
            tripped.clone(),
            RAM,
            false,
            true
        ));
        match run_until_break(&mut m, &[], &tripped, 8) {
            ReplStop::Watch { addr, read } => {
                // Hooks fire per byte; the word store trips inside the
                // watched word (last byte wins).
                assert!(addr.wrapping_sub(RAM) < 4, "{addr:#010x}");
                assert!(!read);
            }
            other => panic!("expected a watch trip, got {other:?}"),
        }
        // Unwatched runs exhaust instead.
        let mut m = boot(&code);
        m.core.regs.r[1] = RAM;
        match run_until_break(&mut m, &[], &no_watch(), 8) {
            ReplStop::Exhausted => {}
            other => panic!("expected exhaustion, got {other:?}"),
        }
    }
}

#[cfg(test)]
mod vcd_tests {
    use super::VcdWriter;

    fn writer() -> (Vec<u8>, Vec<(String, u8)>) {
        (
            Vec::new(),
            vec![("c13".to_owned(), 1), ("pc".to_owned(), 32)],
        )
    }

    #[test]
    fn header_names_every_signal_and_timescale() {
        let (mut out, signals) = writer();
        VcdWriter::new(&mut out, &signals).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("$timescale 1ns $end"));
        assert!(text.contains("$var wire 1 ! c13 $end"));
        assert!(text.contains("$var wire 32 \" pc $end"));
        assert!(text.contains("$enddefinitions $end"));
        assert!(text.ends_with("#0\n"));
    }

    #[test]
    fn changes_emit_once_with_cycle_true_timestamps() {
        let (mut out, signals) = writer();
        let mut w = VcdWriter::new(&mut out, &signals).unwrap();
        w.sample(0, 16_000_000, &[0, 0x0800_0000]).unwrap();
        w.sample(1, 16_000_000, &[0, 0x0800_0002]).unwrap();
        w.sample(2, 16_000_000, &[1, 0x0800_0002]).unwrap();
        drop(w);
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("0!\n"));
        assert!(text.contains("b00001000000000000000000000000000 \"\n"));
        assert!(text.contains("#62\n"));
        assert!(text.contains("b00001000000000000000000000000010 \"\n"));
        assert!(text.contains("#125\n"));
        assert!(text.contains("1!\n"));
        assert_eq!(text.matches("1!\n").count(), 1);
    }

    #[test]
    fn steady_state_emits_no_rows() {
        let (mut out, signals) = writer();
        let mut w = VcdWriter::new(&mut out, &signals).unwrap();
        w.sample(0, 16_000_000, &[1, 0x2000_0000]).unwrap();
        let len = w.out.len();
        w.sample(10, 16_000_000, &[1, 0x2000_0000]).unwrap();
        assert_eq!(w.out.len(), len);
    }

    #[test]
    fn reruns_are_byte_identical() {
        let run = || {
            let (mut out, signals) = writer();
            let mut w = VcdWriter::new(&mut out, &signals).unwrap();
            for (cycles, pin, pc) in [
                (0, 0, 0x0800_0000),
                (3, 1, 0x0800_0004),
                (7, 0, 0x0800_0008),
            ] {
                w.sample(cycles, 8_000_000, &[pin, pc]).unwrap();
            }
            drop(w);
            out
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn stamp_scales_by_board_clock() {
        assert_eq!(VcdWriter::<Vec<u8>>::stamp_ns(1, 16_000_000), 62);
        assert_eq!(
            VcdWriter::<Vec<u8>>::stamp_ns(16_000_000, 16_000_000),
            1_000_000_000
        );
        assert_eq!(VcdWriter::<Vec<u8>>::stamp_ns(5, 0), 5);
    }
}

#[cfg(test)]
mod backtrace_tests {
    use super::format_backtrace_op;
    use vioarm_core::machine::BacktraceEntry;

    fn entry(pc: u32, h0: Option<u16>, h1: Option<u16>) -> BacktraceEntry {
        BacktraceEntry {
            pc,
            h0,
            h1,
            cyccnt: 0,
        }
    }

    #[test]
    fn narrow_opcode_prints_alone() {
        // `movs r0, #7` (0x2007) followed by `subs` bytes: the neighbour
        // halfword is context, not part of this opcode.
        assert_eq!(
            format_backtrace_op(&entry(0x10, Some(0x2007), Some(0x1ec0))),
            "0x2007"
        );
    }

    #[test]
    fn wide_prefix_pairs_with_second_halfword() {
        // `bl` prefix 0xF000 + target halfword: one 32-bit opcode.
        assert_eq!(
            format_backtrace_op(&entry(0x10, Some(0xF000), Some(0xF800))),
            "0xf000:0xf800"
        );
        // Unreadable second halfword: prefix alone, never placeholders.
        assert_eq!(
            format_backtrace_op(&entry(0x10, Some(0xF000), None)),
            "0xf000"
        );
    }

    #[test]
    fn unreadable_code_prints_placeholders() {
        assert_eq!(format_backtrace_op(&entry(0x10, None, None)), "????????");
    }
}

#[cfg(test)]
mod benchmark_tests {
    use super::{
        bench_bare_nop, bench_board_boot, bench_floor_failures, bench_row, bench_usart1_base,
        bench_usart_tx, BenchResult, Board,
    };

    #[test]
    fn rows_parse_as_plain_fields() {
        let r = bench_bare_nop(2000).expect("bare kernel runs");
        assert_eq!(r.steps, 2000);
        assert!(r.cyccnt > 0);
        let row = bench_row(&r);
        let fields: Vec<&str> = row.split_whitespace().collect();
        assert_eq!(fields.len(), 4, "{row}");
        assert_eq!(fields[0], "bare-nop");
        assert!(fields[1].parse::<f64>().unwrap() > 0.0);
        assert!(fields[2].parse::<f64>().unwrap() >= 0.0);
        assert_eq!(fields[3].parse::<u64>().unwrap(), r.cyccnt);
    }

    #[test]
    fn board_boot_runs_windows_to_budget() {
        let r = bench_board_boot(Board::Stm32f103, 20_000).expect("board kernel runs");
        assert_eq!(r.steps, 20_000);
        assert!(r.mips() > 0.0);
    }

    #[test]
    fn usart_firmware_transmits_a_per_trip() {
        // One 606-step window on stm32f103: validates the hand-encoded
        // Thumb-1 (a wrong halfword faults or mistargets instead).
        let r = bench_usart_tx(Board::Stm32f103, 606)
            .expect("soak runs")
            .expect("f103 has USART1");
        assert_eq!(r.steps, 606);
        let note = r.note.clone().unwrap();
        assert!(note.starts_with("tx=200B"), "{note}");
        assert!(!note.contains("MISMATCH"), "{note}");
    }

    #[test]
    fn floor_trips_only_below_min() {
        let rs = vec![BenchResult {
            kernel: "k",
            steps: 1000,
            wall_s: 0.001,
            cyccnt: 1000,
            note: None,
        }]; // exactly 1.0 MIPS
        assert!(bench_floor_failures(&rs, 0.5).is_empty());
        assert_eq!(bench_floor_failures(&rs, 2.0), vec!["k"]);
    }

    #[test]
    fn usart_base_resolves_on_f103_and_absent_on_virtual() {
        assert_eq!(bench_usart1_base("stm32f103"), Some(0x4001_3800));
        assert_eq!(bench_usart1_base("virtual"), None);
    }
}

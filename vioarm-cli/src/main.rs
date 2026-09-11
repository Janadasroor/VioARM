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
        /// Trace format: human-readable text (default) or `jsonl` streaming
        /// (`{"cycle":N,"pc":"0x..."}` per retired instruction, driven by
        /// the hook registry).
        #[arg(long, value_enum, default_value_t = TraceFormat::Text)]
        format: TraceFormat,
        /// Write the trace here instead of stdout.
        #[arg(long)]
        output: Option<PathBuf>,
    },
    /// Show image layout (segments, entry)
    Info { image: PathBuf },
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

fn machine_for(board: Board, img: &LoadedImage) -> Machine<board::BoardMemory> {
    let mem = board.mem();
    // WP-K1: the core follows the board family; the dispatch lives in
    // `Machine::for_mem` (shared with the co-simulation embedding path).
    // `virtual` stays the explicit M0 home (see `board::cortex_m0`).
    let name = board.name();
    let mut m = Machine::for_mem(name, mem);
    m.load_image(img);
    if let Err(cause) = m.reset() {
        eprintln!("{} reset failed: {cause}", "error:".red().bold());
        std::process::exit(3);
    }
    m
}

fn run_quiet(m: &mut Machine<board::BoardMemory>, budget: u64, jit: bool) {
    let outcome = if jit {
        let mut runner = vioarm_core::jit::TbRunner::new();
        m.run_tb(&mut runner, budget)
    } else {
        m.run(budget)
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
    println!(
        "{} pc={:#010x} sp={:#010x} xpsr={:#010x}",
        "attached:".dimmed(),
        regs.pc,
        regs.sp,
        xpsr.get()
    );
    let mut rl = String::new();
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
                println!("{}: step(s)/next(n) continue(c) regs(r) mem <addr> break <pc> budget <n> quit(q)", "commands".bright_green());
            }
            "step" | "s" | "n" | "next" => match m.step() {
                Ok(_) => {
                    let (regs, xpsr, cycles) = m.core.snapshot();
                    println!(
                        "{} pc={:#010x} r0={:#010x} xpsr={:#010x} cycles={cycles}",
                        "step".dimmed(),
                        regs.pc,
                        regs.r[0],
                        xpsr.get()
                    );
                }
                Err(e) => println!("{} {e}", "halted:".yellow(),),
            },
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
                } else {
                    println!("{} mem <hex addr>", "usage:".yellow());
                }
            }
            c if c.starts_with("break ") => {
                let pc = c[6..].trim();
                if let Ok(a) = parse_u32(pc) {
                    println!(
                        "{} break at {a:#010x} — run `continue` to hit BKPT",
                        "break".bright_green()
                    );
                    // simple: run until pc
                    let mut hit = false;
                    for _ in 0..1_000_000 {
                        let (regs, _, _) = m.core.snapshot();
                        if regs.pc == a {
                            hit = true;
                            break;
                        }
                        if m.step().is_err() {
                            break;
                        }
                    }
                    println!(
                        "{}",
                        if hit {
                            "hit".green()
                        } else {
                            "not hit in 1M".yellow()
                        }
                    );
                }
            }
            c if c.starts_with("budget ") => {
                if let Ok(n) = c[7..].trim().parse::<u64>() {
                    match m.run(n) {
                        Err(HaltCause::CycleBudgetExhausted) => {
                            println!("{}", "budget exhausted".yellow())
                        }
                        Err(e) => println!("{} {e}", "halted:".green()),
                        Ok(()) => {}
                    }
                }
            }
            "continue" | "c" | "run" => match m.run(1_000_000) {
                Err(HaltCause::CycleBudgetExhausted) => {
                    println!("{}", "budget exhausted — 1M".yellow())
                }
                Err(e) => println!("{} {e}", "halted:".green()),
                Ok(()) => {}
            },
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
            } => {
                if jit && fast != 0 {
                    eprintln!(
                        "{} --jit needs --fast 0 (quanta need the accurate time model)",
                        "error:".red().bold()
                    );
                    std::process::exit(2);
                }
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img);
                m.semihost_enable(semihost);
                m.set_fast_quantum(fast);
                banner();
                run_quiet(&mut m, budget, jit);
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
                let mut m = machine_for(board, &img);
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
                let mut m = machine_for(board, &img);
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
            } => {
                let (img, _) = load_image_auto(&image, base);
                let mut m = machine_for(board, &img);
                m.semihost_enable(semihost);
                banner();
                match format {
                    TraceFormat::Jsonl => run_traced_jsonl(&mut m, budget, output),
                    TraceFormat::Text => run_traced(&mut m, budget),
                }
            }
            Commands::Info { image } => {
                let (img, _) = load_image_auto(&image, 0);
                print_info(&img);
            }
            Commands::Boards => {
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
                let mut m = machine_for(board, &img);
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
    let img = if let Some(path) = cli.elf {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        loader::load_elf(&bytes).unwrap_or_else(|e| {
            eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
            std::process::exit(2);
        })
    } else if let Some(path) = cli.hex {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        loader::load_hex(&bytes).unwrap_or_else(|e| {
            eprintln!("{} {}: {e}", "error:".red().bold(), path.display());
            std::process::exit(2);
        })
    } else if let Some(path) = cli.bin {
        let bytes = std::fs::read(&path).unwrap_or_else(|e| {
            eprintln!(
                "{} cannot read {}: {e}",
                "error:".red().bold(),
                path.display()
            );
            std::process::exit(2);
        });
        loader::load_flat(cli.base, &bytes)
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
    let mut m = machine_for(cli.board, &img);
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
        run_quiet(&mut m, cli.budget, false);
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

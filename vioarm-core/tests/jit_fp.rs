//! P3-1 FP callout verification matrix (D3): NaN-payload / denormal /
//! FTZ / infinity / zero edge vectors through the native VFP side-exit
//! tails vs the interpreter — must match bitwise. Both engines share the
//! softfloat code (there is deliberately no native FP: host SSE/Neon
//! differs on exactly these edges); the test proves the plumbing hands
//! every edge through identically, including FPSCR control (FZ/DN) and
//! cumulative exception bits.
//!
//! Boundaries: default rounding only (rounding modes are oracle
//! territory); single-precision only (doubles are v7E-M); fixed-point
//! converts stay with the directed/fuzz suites for state compare.

#![cfg(feature = "jit")]

use vioarm_core::cpu::decode32::{decode32_v7m, Inst32, Vfp2Op, Vfp3Op, VrintOp};
use vioarm_core::jit::TbRunner;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const FLASH: u32 = 0x0000_0000;
const RAM: u32 = 0x2000_0000;
const ENTRY: u32 = 0x0000_0010;
const INIT_SP: u32 = 0x2000_8000;
const DATA: u32 = 0x2000_0100;

/// f32 edge bits: zeros, subnormal edges, min normal, finite values,
///
/// infinities, quiet/signaling NaNs with and without payloads.
const EDGES: &[u32] = &[
    0x0000_0000, // +0
    0x8000_0000, // -0
    0x0000_0001, // min subnormal
    0x0040_0000, // subnormal
    0x0080_0000, // min normal
    0x3F80_0000, // 1.0
    0xBF80_0000, // -1.0
    0x4049_0FDB, // pi-ish
    0x7F7F_FFFF, // max finite
    0x7F80_0000, // +inf
    0xFF80_0000, // -inf
    0x7FC0_0000, // qNaN
    0x7FC1_2345, // qNaN payload
    0x7F80_0001, // sNaN
    0x7FA0_0000, // sNaN payload-ish
];

/// FPSCR mode words: plain, flush-to-zero, default-NaN, both.
const MODES: &[u32] = &[0x0000_0000, 0x0100_0000, 0x0200_0000, 0x0300_0000];

/// One matrix op: machine halves plus operand roles. `S0=a`, `S1=b`,
/// `S2` accumulates (MLA family) or receives; `r0` points at DATA.
struct VecOp {
    name: &'static str,
    h0: u16,
    h1: u16,
    /// Needs the DATA word pre-seeded (VLDR) or compares it (VSTR).
    mem: bool,
    /// Reads `r3` (VMOV-gp/VMSR) or writes it (VMOV-gp/VMRS).
    r3: bool,
}

fn v3(name: &'static str, op: Vfp3Op, h0: u16, h1: u16) -> VecOp {
    assert_eq!(
        decode32_v7m(h0, h1),
        Ok(Inst32::Vfp3 {
            op,
            vd: 2,
            vn: 0,
            vm: 1,
            is_dp: false,
        }),
        "{name} decodes"
    );
    VecOp {
        name,
        h0,
        h1,
        mem: false,
        r3: false,
    }
}

fn ops() -> Vec<VecOp> {
    let mut v = vec![
        v3("vadd", Vfp3Op::Vadd, 0xEE30, 0x1A20),
        v3("vsub", Vfp3Op::Vsub, 0xEE30, 0x1A60),
        v3("vmul", Vfp3Op::Vmul, 0xEE20, 0x1A20),
        v3("vdiv", Vfp3Op::Vdiv, 0xEE80, 0x1A20),
        v3("vmla", Vfp3Op::Vmla, 0xEE00, 0x1A20),
        v3("vmls", Vfp3Op::Vmls, 0xEE00, 0x1A60),
        v3("vnmla", Vfp3Op::Vnmla, 0xEE10, 0x1A60),
        v3("vfma", Vfp3Op::Vfma, 0xEEA0, 0x1A20),
    ];
    // Two-register arithmetic (vd=2, vm=1).
    for (name, op, h0, h1) in [
        ("vmov", Vfp2Op::Vmov, 0xEEB0, 0x1A60),
        ("vabs", Vfp2Op::Vabs, 0xEEB0, 0x1AE0),
        ("vneg", Vfp2Op::Vneg, 0xEEB1, 0x1A60),
        ("vsqrt", Vfp2Op::Vsqrt, 0xEEB1, 0x1AE0),
    ] {
        assert_eq!(
            decode32_v7m(h0, h1),
            Ok(Inst32::Vfp2 {
                op,
                vd: 2,
                vm: 1,
                is_dp: false,
            }),
            "{name} decodes"
        );
        v.push(VecOp {
            name,
            h0,
            h1,
            mem: false,
            r3: false,
        });
    }
    // Compare (S1 vs S0 — flags land in vfp.fpscr).
    for (name, e, h1) in [("vcmp", false, 0x0A40), ("vcmpe", true, 0x0AC0)] {
        assert_eq!(
            decode32_v7m(0xEEF4, h1),
            Ok(Inst32::VfpCmp {
                vd: 1,
                vm: 0,
                e,
                z: false,
                is_dp: false,
            }),
            "{name} decodes"
        );
        v.push(VecOp {
            name,
            h0: 0xEEF4,
            h1,
            mem: false,
            r3: false,
        });
    }
    // Round-to-integral.
    for (name, op, h1) in [
        ("vrintr", VrintOp::R, 0x1A60),
        ("vrintz", VrintOp::Z, 0x1AE0),
    ] {
        assert_eq!(
            decode32_v7m(0xEEB6, h1),
            Ok(Inst32::VfpVrint {
                op,
                vd: 2,
                vm: 1,
                is_dp: false,
            }),
            "{name} decodes"
        );
        v.push(VecOp {
            name,
            h0: 0xEEB6,
            h1,
            mem: false,
            r3: false,
        });
    }
    // Int↔float converts (S2↔S1).
    assert_eq!(
        decode32_v7m(0xEEBD, 0x1A60),
        Ok(Inst32::VfpCvtInt {
            to_float: false,
            signed: true,
            rz: false,
            vd: 2,
            vm: 1,
            is_dp: false,
        }),
        "vcvt.s32 decodes"
    );
    v.push(VecOp {
        name: "vcvt.s32",
        h0: 0xEEBD,
        h1: 0x1A60,
        mem: false,
        r3: false,
    });
    assert_eq!(
        decode32_v7m(0xEEBD, 0x1AE0),
        Ok(Inst32::VfpCvtInt {
            to_float: false,
            signed: true,
            rz: true,
            vd: 2,
            vm: 1,
            is_dp: false,
        }),
        "vcvtr.s32 decodes"
    );
    v.push(VecOp {
        name: "vcvtr.s32",
        h0: 0xEEBD,
        h1: 0x1AE0,
        mem: false,
        r3: false,
    });
    assert_eq!(
        decode32_v7m(0xEEB8, 0x1AE0),
        Ok(Inst32::VfpCvtInt {
            to_float: true,
            signed: true,
            rz: false,
            vd: 2,
            vm: 1,
            is_dp: false,
        }),
        "vcvt.f32.s32 decodes"
    );
    v.push(VecOp {
        name: "vcvt.f32.s32",
        h0: 0xEEB8,
        h1: 0x1AE0,
        mem: false,
        r3: false,
    });
    assert_eq!(
        decode32_v7m(0xEEB8, 0x1A60),
        Ok(Inst32::VfpCvtInt {
            to_float: true,
            signed: false,
            rz: false,
            vd: 2,
            vm: 1,
            is_dp: false,
        }),
        "vcvt.f32.u32 decodes"
    );
    v.push(VecOp {
        name: "vcvt.f32.u32",
        h0: 0xEEB8,
        h1: 0x1A60,
        mem: false,
        r3: false,
    });
    // GP↔VFP moves (S2↔r3).
    assert_eq!(
        decode32_v7m(0xEE01, 0x3A10),
        Ok(Inst32::VfpMov {
            to_gp: false,
            vd: 2,
            rt: 3,
        }),
        "vmov.s2,r3 decodes"
    );
    v.push(VecOp {
        name: "vmov.s2,r3",
        h0: 0xEE01,
        h1: 0x3A10,
        mem: false,
        r3: true,
    });
    assert_eq!(
        decode32_v7m(0xEE11, 0x3A10),
        Ok(Inst32::VfpMov {
            to_gp: true,
            vd: 2,
            rt: 3,
        }),
        "vmov.r3,s2 decodes"
    );
    v.push(VecOp {
        name: "vmov.r3,s2",
        h0: 0xEE11,
        h1: 0x3A10,
        mem: false,
        r3: true,
    });
    // FPSCR moves.
    assert_eq!(
        decode32_v7m(0xEEF1, 0x3A10),
        Ok(Inst32::VfpMrsVmsr {
            is_read: true,
            rt: 3,
        }),
        "vmrs decodes"
    );
    v.push(VecOp {
        name: "vmrs",
        h0: 0xEEF1,
        h1: 0x3A10,
        mem: false,
        r3: true,
    });
    assert_eq!(
        decode32_v7m(0xEEE1, 0x3A10),
        Ok(Inst32::VfpMrsVmsr {
            is_read: false,
            rt: 3,
        }),
        "vmsr decodes"
    );
    v.push(VecOp {
        name: "vmsr",
        h0: 0xEEE1,
        h1: 0x3A10,
        mem: false,
        r3: true,
    });
    // Memory round-trip (edge bits live in RAM).
    assert_eq!(
        decode32_v7m(0xEDD0, 0x0A04),
        Ok(Inst32::VfpLdSt {
            is_load: true,
            vd: 1,
            rn: 0,
            add: true,
            imm: 16,
        }),
        "vldr decodes"
    );
    v.push(VecOp {
        name: "vldr",
        h0: 0xEDD0,
        h1: 0x0A04,
        mem: true,
        r3: false,
    });
    assert_eq!(
        decode32_v7m(0xEDC0, 0x0A05),
        Ok(Inst32::VfpLdSt {
            is_load: false,
            vd: 1,
            rn: 0,
            add: true,
            imm: 20,
        }),
        "vstr decodes"
    );
    v.push(VecOp {
        name: "vstr",
        h0: 0xEDC0,
        h1: 0x0A05,
        mem: true,
        r3: false,
    });
    v
}

fn boot(code: &[u8]) -> Machine<FlatMemory> {
    let mut m = Machine::armv7m(FlatMemory::with_layout(&[FLASH, RAM], &[0x10000, 0x10000]));
    m.load_image(&load_flat(ENTRY, code));
    m.core.mem().write_word(FLASH, INIT_SP).unwrap();
    m.core.mem().write_word(FLASH + 4, ENTRY | 1).unwrap();
    m.reset().expect("reset from vector table");
    m.core.regs.sp = INIT_SP;
    m.core.regs.r[0] = DATA;
    m
}

fn halves(h0: u16, h1: u16) -> Vec<u8> {
    vec![
        (h0 & 0xFF) as u8,
        (h0 >> 8) as u8,
        (h1 & 0xFF) as u8,
        (h1 >> 8) as u8,
        0xFE,
        0xE7,
    ]
}

/// Run one op natively (side-exit tail) and via the stepper; compare VFP
/// state, involved GP regs, touched RAM, and retired cycles bit-for-bit.
static COMPARED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn check_op(op: &VecOp, a: u32, b: u32, mode: u32, r3v: u32) {
    COMPARED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let code = halves(op.h0, op.h1);
    let setup = |m: &mut Machine<FlatMemory>| {
        m.core.vfp.s[0] = a;
        m.core.vfp.s[1] = b;
        m.core.vfp.s[2] = 0x3F80_0000;
        m.core.vfp.fpscr = mode;
        m.core.regs.r[3] = r3v;
        m.core.mem().write_word(DATA + 16, a).unwrap();
        m.core.mem().write_word(DATA + 20, 0xDEAD_BEEF).unwrap();
    };
    let mut mn = boot(&code);
    setup(&mut mn);
    let mut runner = TbRunner::new();
    // One block exactly (VFP seals System): the native tail side-exits
    // and the driver re-executes through the interpreter.
    match runner.run_block(&mut mn.core) {
        Ok(Some(outcome)) => assert_eq!(outcome.ops, 1, "{} runs one op", op.name),
        Ok(None) => panic!("{} native refused", op.name),
        Err(e) => panic!("{} native faulted: {e:?} (a={a:#X} b={b:#X})", op.name),
    }
    let mut ms = boot(&code);
    setup(&mut ms);
    if let Err(e) = ms.step() {
        panic!("{} stepper faulted: {e:?} (a={a:#X} b={b:#X})", op.name);
    }
    assert_eq!(
        mn.core.vfp, ms.core.vfp,
        "{} vfp (a={a:#X} b={b:#X} m={mode:#X})",
        op.name
    );
    assert_eq!(mn.core.regs.r[0], ms.core.regs.r[0], "{} r0", op.name);
    assert_eq!(mn.core.regs.r[3], ms.core.regs.r[3], "{} r3", op.name);
    assert_eq!(
        mn.core.mem().read_word(DATA + 20).unwrap(),
        ms.core.mem().read_word(DATA + 20).unwrap(),
        "{} ram",
        op.name
    );
    assert_eq!(mn.core.cycles, ms.core.cycles, "{} cycles", op.name);
}

#[test]
fn fp_edge_matrix_matches_bitwise() {
    let ops = ops();
    // Two-input arithmetic over the full edge square × FPSCR modes.
    let arith = [
        "vadd", "vsub", "vmul", "vdiv", "vmla", "vmls", "vnmla", "vfma",
    ];
    for op in ops.iter().filter(|o| arith.contains(&o.name)) {
        for &mode in MODES {
            for &a in EDGES {
                for &b in EDGES {
                    check_op(op, a, b, mode, 0);
                }
            }
        }
    }
    // Single-input ops over edges × modes.
    let single = [
        "vmov",
        "vabs",
        "vneg",
        "vsqrt",
        "vrintr",
        "vrintz",
        "vcmp",
        "vcmpe",
        "vcvt.s32",
        "vcvtr.s32",
        "vcvt.f32.s32",
        "vcvt.f32.u32",
    ];
    for op in ops.iter().filter(|o| single.contains(&o.name)) {
        for &mode in MODES {
            for &b in EDGES {
                check_op(op, 0x3F80_0000, b, mode, 0);
            }
        }
    }
    // Moves: edge bits through r3/S2/RAM.
    for op in ops
        .iter()
        .filter(|o| ["vmov.s2,r3", "vmov.r3,s2", "vmrs", "vmsr", "vldr", "vstr"].contains(&o.name))
    {
        for &mode in MODES {
            for &a in EDGES {
                check_op(op, a, a, mode, a);
            }
        }
    }
    eprintln!(
        "fp matrix: {} bitwise comparisons",
        COMPARED.load(std::sync::atomic::Ordering::Relaxed)
    );
}

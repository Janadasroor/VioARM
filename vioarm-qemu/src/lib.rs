//! QEMU conformance harness.
//!
//! Runs identical raw Thumb binaries in `vioarm-core` and in
//! `qemu-system-arm` (Cortex-M3) and diffs the per-instruction register
//! traces, validating the whole implemented ISA against a reference
//! implementation. Tests in this crate skip gracefully when no `qemu` binary
//! is found.

pub mod asm;
pub mod qemu;
pub mod vioarm;

use std::fmt;

/// The xPSR bits both implementations model and agree on: `N:Z:C:V`, the
/// sticky saturation flag `Q`, the four ARMv7-M `GE` flags, the Thumb bit
/// `T`, and the ARMv7-M `ITSTATE` (bits 26:25 and 15:10), which carries the
/// conditional-execution state through an IT block. `IPSR` (exception number)
/// is excluded — the conformance traces run in Thread mode where it is zero
/// anyway — as are the reserved bits.
pub const XPSR_MASK: u32 = 0xF90F_0000 | 0x0600_FC00;

/// A single architectural register snapshot: `r0..r12`, `r13` = SP,
/// `r14` = LR, `r15` = PC, plus the `xPSR`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuState {
    pub r: [u32; 16],
    pub xpsr: u32,
}

impl CpuState {
    #[must_use]
    pub const fn masked_xpsr(self) -> u32 {
        self.xpsr & XPSR_MASK
    }
}

impl fmt::Display for CpuState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "pc={:08x} sp={:08x} lr={:08x} xpsr(masked)={:08x} \
             r0..r3={:08x},{:08x},{:08x},{:08x} \
             r4..r7={:08x},{:08x},{:08x},{:08x} \
             r8..r11={:08x},{:08x},{:08x},{:08x} r12={:08x}",
            self.r[15] & !1,
            self.r[13],
            self.r[14],
            self.masked_xpsr(),
            self.r[0],
            self.r[1],
            self.r[2],
            self.r[3],
            self.r[4],
            self.r[5],
            self.r[6],
            self.r[7],
            self.r[8],
            self.r[9],
            self.r[10],
            self.r[11],
            self.r[12],
        )
    }
}

/// Compare two conformance traces.
///
/// Index 0 is the reset state. The two implementations legitimately differ in
/// the *initial flags* (QEMU boots the Cortex-M3 with `Z` set, VioARM with all
/// flags clear) so index 0 must agree on every register but not on `xPSR`.
/// Every later index must agree on all registers and the modeled xPSR bits.
///
/// # Errors
///
/// Returns a human-readable description of the first divergence (step index,
/// register, and both machine states) or a trace-length mismatch.
pub fn compare_traces(label: &str, vio: &[CpuState], qemu: &[CpuState]) -> Result<(), String> {
    let common = vio.len().min(qemu.len());
    if vio[0].r != qemu[0].r {
        let mut diffs = Vec::new();
        for (i, (a, b)) in vio[0].r.iter().zip(qemu[0].r.iter()).enumerate() {
            if a != b {
                diffs.push(format!("r{i}: vio={a:08x} qemu={b:08x}"));
            }
        }
        return Err(format!(
            "{label}: reset registers differ: {}\n  vioarm: {}\n  qemu:  {}",
            diffs.join(", "),
            vio[0],
            qemu[0]
        ));
    }
    for k in 1..common {
        let a = vio[k];
        let b = qemu[k];
        let mut diffs = Vec::new();
        for i in 0..16 {
            let av = if i == 15 { a.r[i] & !1 } else { a.r[i] };
            let bv = if i == 15 { b.r[i] & !1 } else { b.r[i] };
            if av != bv {
                diffs.push(format!("r{i}: vio={av:08x} qemu={bv:08x}"));
            }
        }
        if a.masked_xpsr() != b.masked_xpsr() {
            diffs.push(format!(
                "xpsr: vio={:08x} qemu={:08x}",
                a.masked_xpsr(),
                b.masked_xpsr()
            ));
        }
        if !diffs.is_empty() {
            return Err(format!(
                "{label}: step {k} diverges (after the {k}-th executed instruction): {}\n  vioarm: {}\n  qemu:   {}",
                diffs.join(", "),
                a,
                b
            ));
        }
    }
    if vio.len() != qemu.len() {
        return Err(format!(
            "{label}: registers agree but trace lengths differ: vioarm={} qemu={} \
             (violating instruction at step {}: vioarm pc={:#x} qemu pc={:#x})",
            vio.len(),
            qemu.len(),
            common - 1,
            vio[common - 1].r[15] & !1,
            qemu[common - 1].r[15] & !1,
        ));
    }
    Ok(())
}
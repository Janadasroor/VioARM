//! M10-J1 SPIKE — NOT product code, NOT a roadmap commitment.
//!
//! Question: what does ARMv7-A cost — full core, stage-1 MMU, GIC, timers —
//! to a Linux shell prompt? This spike measures the smallest faithful piece
//! (A32 decode subset + short-descriptor page-table walk + translated
//! hello-world) so the projection in `roadmap.md` M10 is measured, not
//! guessed. Deliberately out of scope here: Thumb-2 (A-profile), NEON/VFP,
//! CP15 beyond `SCTLR.M`/`TTBR0` (set from the harness, not the guest),
//! domains/`DACR`, TEX remap, XN/PXN/UXN, supersections, LPAE, TLB
//! maintenance, banked modes/stacks, GIC, generic timers, boot/SMP.
//!
//! Run: `cargo run -p vioarm-core --example a_profile_spike`.
//!
//! Guest program (hand-assembled, bytes verified with
//! `arm-none-eabi-objdump`; `ldr`/`str`/`b`/`bl`/`mov`/`add`-less minimal
//! set — `sub`/`cmp` included for flags):
//! ```asm
//! _start: ldr r1, ram_phys  @ 0x00
//!         ldr r2, ram_virt  @ 0x04
//!         mov r0, #0x5A     @ 0x08
//!         str r0, [r1]      @ 0x0C  PA 0x100000 = 0x5A
//!         ldr r3, [r2]      @ 0x10  VA 0x40000000 -> PA 0x100000 ?
//!         cmp r3, r0        @ 0x14
//!         bne fail          @ 0x18
//!         mov r4, #1        @ 0x1C
//!         b done            @ 0x20
//! fail:   mov r4, #0        @ 0x24
//! done:   str r4, [r1, #4]  @ 0x28  PASS flag to RAM
//!         bkpt #0           @ 0x2C
//! ```

use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

/// Short-descriptor MMU state: `TTBR0` + the enable bit (the harness stands
/// in for the CP15 the guest would otherwise need).
#[derive(Clone, Copy, Debug, Default)]
struct SpikeMmu {
    ttbr0: u32,
    enabled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fault {
    /// No descriptor covers the address.
    Translation { va: u32 },
    /// Covered, but `AP[1:0] == 0b00`.
    Permission { va: u32 },
}

/// Stage-1 short-descriptor walk (`N = 0`): 1 MB sections, 4 KB small
/// pages. TEX/C/B, domains, XN and supersections are read but ignored
/// (projection, not behavior — see the module docs).
fn translate(mem: &mut FlatMemory, mmu: SpikeMmu, va: u32, _write: bool) -> Result<u32, Fault> {
    if !mmu.enabled {
        return Ok(va);
    }
    let l1 = mem
        .read_word(mmu.ttbr0.wrapping_add((va >> 20).wrapping_mul(4)))
        .map_err(|_| Fault::Translation { va })?;
    match l1 & 3 {
        // Section: `APX`@15, `AP[1:0]`@11:10.
        0b10 => {
            if l1 & 0xC00 == 0 {
                return Err(Fault::Permission { va });
            }
            Ok((l1 & 0xFFF0_0000) | (va & 0x000F_FFFF))
        }
        // Page table: 1 KB table of 256 small pages.
        0b01 => {
            let pt = l1 & 0xFFFF_FC00;
            let l2 = mem
                .read_word(pt.wrapping_add(((va >> 12) & 0xFF).wrapping_mul(4)))
                .map_err(|_| Fault::Translation { va })?;
            // Small page: `APX`@9, `AP[1:0]`@5:4.
            match l2 & 3 {
                0b10 => {
                    if l2 & 0x30 == 0 {
                        return Err(Fault::Permission { va });
                    }
                    Ok((l2 & 0xFFFF_F000) | (va & 0xFFF))
                }
                _ => Err(Fault::Translation { va }),
            }
        }
        _ => Err(Fault::Translation { va }),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Halt {
    Bkpt,
    Fault(Fault),
    Unimplemented(u32),
}

/// Full 4-bit condition table (only `AL`/`EQ`/`NE` appear in the guest).
const fn cond_pass(cpsr: u32, cond: u32) -> bool {
    let (n, z, c, v) = (
        cpsr & (1 << 31) != 0,
        cpsr & (1 << 30) != 0,
        cpsr & (1 << 29) != 0,
        cpsr & (1 << 28) != 0,
    );
    match cond {
        0 => z,
        1 => !z,
        2 => c,
        3 => !c,
        4 => n,
        5 => !n,
        6 => v,
        7 => !v,
        8 => c && !z,
        9 => !c || z,
        10 => n == v,
        11 => n != v,
        12 => !z && n == v,
        13 => z || n != v,
        14 => true,
        _ => false,
    }
}

fn ror(value: u32, rot: u32) -> u32 {
    value.rotate_right(2 * rot)
}

struct Cpu {
    r: [u32; 16],
    cpsr: u32,
    mem: FlatMemory,
    mmu: SpikeMmu,
    steps: u64,
    translations: u64,
}

impl Cpu {
    fn read_mem(&mut self, va: u32) -> Result<u32, Halt> {
        let pa = translate(&mut self.mem, self.mmu, va, false).map_err(Halt::Fault)?;
        self.translations += 1;
        self.mem
            .read_word(pa)
            .map_err(|_| Halt::Fault(Fault::Translation { va }))
    }

    fn write_mem(&mut self, va: u32, val: u32) -> Result<(), Halt> {
        let pa = translate(&mut self.mem, self.mmu, va, true).map_err(Halt::Fault)?;
        self.translations += 1;
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            self.mem
                .write(pa.wrapping_add(i as u32), *b)
                .map_err(|_| Halt::Fault(Fault::Translation { va }))?;
        }
        Ok(())
    }

    fn reg(&self, n: usize) -> u32 {
        if n == 15 {
            self.r[15].wrapping_add(8)
        } else {
            self.r[n]
        }
    }

    fn step(&mut self) -> Result<(), Halt> {
        let pc = self.r[15];
        let insn = self.read_mem(pc)?;
        let cond = insn >> 28;
        // `BKPT #0` (exact pattern first: it overlaps data-processing space).
        if insn == 0xE120_0070 {
            return Err(Halt::Bkpt);
        }
        if !cond_pass(self.cpsr, cond) {
            self.r[15] = pc.wrapping_add(4);
            self.steps += 1;
            return Ok(());
        }
        // Branch / branch-and-link.
        if insn & 0x0E00_0000 == 0x0A00_0000 {
            let imm = insn & 0x00FF_FFFF;
            let off = if imm & 0x0080_0000 != 0 {
                imm | 0xFF00_0000
            } else {
                imm
            }
            .wrapping_mul(4);
            if insn & 0x0100_0000 != 0 {
                self.r[14] = pc.wrapping_add(4);
            }
            self.r[15] = pc.wrapping_add(8).wrapping_add(off);
            self.steps += 1;
            return Ok(());
        }
        // Single data transfer (immediate offset, pre-indexed, no writeback).
        if insn & 0x0E00_0000 == 0x0400_0000 {
            let p = insn & 0x0100_0000 != 0;
            let u = insn & 0x0080_0000 != 0;
            let byte = insn & 0x0040_0000 != 0;
            let w = insn & 0x0020_0000 != 0;
            let load = insn & 0x0010_0000 != 0;
            if !p || w || byte {
                return Err(Halt::Unimplemented(insn));
            }
            let rn = ((insn >> 16) & 0xF) as usize;
            let rd = ((insn >> 12) & 0xF) as usize;
            let off = insn & 0xFFF;
            let addr = if u {
                self.reg(rn).wrapping_add(off)
            } else {
                self.reg(rn).wrapping_sub(off)
            };
            if load {
                self.r[rd] = self.read_mem(addr)?;
            } else {
                self.write_mem(addr, self.r[rd])?;
            }
            self.r[15] = pc.wrapping_add(4);
            self.steps += 1;
            return Ok(());
        }
        // Data processing (bits[27:26] == 00): immediate operand, or a
        // register with no shift (anything else is outside the subset).
        if insn & 0x0C00_0000 == 0x0000_0000 {
            let opcode = (insn >> 21) & 0xF;
            let rn = ((insn >> 16) & 0xF) as usize;
            let rd = ((insn >> 12) & 0xF) as usize;
            let imm = if insn & 0x0200_0000 != 0 {
                ror(insn & 0xFF, (insn >> 8) & 0xF)
            } else if insn & 0xFF0 == 0 {
                self.reg((insn & 0xF) as usize)
            } else {
                return Err(Halt::Unimplemented(insn));
            };
            match opcode {
                // MOV
                0b1101 => self.r[rd] = imm,
                // CMP: full N/Z/C/V (the only flag-setter in the subset).
                0b1010 => {
                    let (res, borrow) = self.reg(rn).overflowing_sub(imm);
                    let n = res & 0x8000_0000 != 0;
                    let z = res == 0;
                    let c = !borrow;
                    let v = ((self.reg(rn) ^ imm) & (self.reg(rn) ^ res) & 0x8000_0000) != 0;
                    self.cpsr = (u32::from(n) << 31)
                        | (u32::from(z) << 30)
                        | (u32::from(c) << 29)
                        | (u32::from(v) << 28);
                }
                // ADD / SUB
                0b1000 => self.r[rd] = self.reg(rn).wrapping_add(imm),
                0b0100 => self.r[rd] = self.reg(rn).wrapping_sub(imm),
                _ => return Err(Halt::Unimplemented(insn)),
            }
            self.r[15] = pc.wrapping_add(4);
            self.steps += 1;
            return Ok(());
        }
        Err(Halt::Unimplemented(insn))
    }
}

/// Guest bytes from the verified disassembly (`ldr r1,[pc,#40]` …).
const GUEST: [u8; 56] = [
    0x28, 0x10, 0x9F, 0xE5, 0x28, 0x20, 0x9F, 0xE5, 0x5A, 0x00, 0xA0, 0xE3, 0x00, 0x00, 0x81, 0xE5,
    0x00, 0x30, 0x92, 0xE5, 0x00, 0x00, 0x53, 0xE1, 0x01, 0x00, 0x00, 0x1A, 0x01, 0x40, 0xA0, 0xE3,
    0x00, 0x00, 0x00, 0xEA, 0x00, 0x40, 0xA0, 0xE3, 0x04, 0x40, 0x81, 0xE5, 0x70, 0x00, 0x20, 0xE1,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x40,
];

fn write_word(mem: &mut FlatMemory, addr: u32, val: u32) {
    use vioarm_core::mem::Bus;
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        mem.write(addr.wrapping_add(i as u32), *b).unwrap();
    }
}

fn main() {
    // 2 MB of physical memory at zero: code, tables, RAM side by side.
    let mem = FlatMemory::with_layout(&[0], &[0x20_0000]);
    let mut cpu = Cpu {
        r: [0; 16],
        cpsr: 0x1D3, // SVC32, IRQ/FIQ masked, Thumb clear
        mem,
        mmu: SpikeMmu::default(),
        steps: 0,
        translations: 0,
    };
    // Guest at physical zero (identity-mapped below, so VA == PA there).
    for (i, b) in GUEST.iter().enumerate() {
        use vioarm_core::mem::Bus;
        cpu.mem.write(i as u32, *b).unwrap();
    }
    // Level-1 table at 0x4000 (`N = 0`, 4096 entries over 0x4000-0x7FFF):
    // identity section for code, RAM section at 1 MB, page table for the
    // translated window (the L2 lives clear at 0x8000).
    write_word(&mut cpu.mem, 0x4000, 0x0000_0C02); // VA 0x00000000 -> PA 0
    write_word(&mut cpu.mem, 0x4004, 0x0010_0C02); // VA 0x00100000 -> PA same
    write_word(&mut cpu.mem, 0x5000, 0x0000_8001); // VA 0x40000000 -> L2
                                                   // Level-2 table at 0x8000: VA 0x40000000 -> PA 0x00100000 (AP full).
    write_word(&mut cpu.mem, 0x8000, 0x0010_0032);
    cpu.mmu = SpikeMmu {
        ttbr0: 0x4000,
        enabled: true,
    };
    // Run to the guest's BKPT.
    let halt = loop {
        if cpu.steps > 1_000 {
            panic!("spike guest did not halt in 1000 steps");
        }
        let pc = cpu.r[15];
        match cpu.step() {
            Ok(()) => {}
            Err(h) => {
                eprintln!("halt at pc={pc:#010x}: {h:?}");
                break h;
            }
        }
    };
    assert_eq!(halt, Halt::Bkpt, "guest must reach its BKPT, got {halt:?}");
    let pass = cpu.mem.read_word(0x0010_0004).unwrap();
    assert_eq!(pass, 1, "PASS flag: translation proof failed");
    println!("hello from translated silicon: PASS flag = {pass}");
    println!(
        "steps = {}, translated accesses = {}",
        cpu.steps, cpu.translations
    );
    // Fault probe: unmapped VA must fault, not silently alias.
    match translate(&mut cpu.mem, cpu.mmu, 0x8000_0000, false) {
        Err(Fault::Translation { va }) => {
            println!("fault probe: VA {va:#010x} faults as specified")
        }
        other => panic!("unmapped VA must fault, got {other:?}"),
    }
    // Permission probe: revoke the page's AP bits, expect a permission fault.
    write_word(&mut cpu.mem, 0x8000, 0x0010_0002 & !0x30);
    match translate(&mut cpu.mem, cpu.mmu, 0x4000_0000, true) {
        Err(Fault::Permission { va }) => {
            println!("permission probe: VA {va:#010x} faults on AP==0b00")
        }
        other => panic!("AP==0b00 must fault, got {other:?}"),
    }
    println!("spike complete: decode subset + walk + hello-world hold; see roadmap M10 for the projection");
}

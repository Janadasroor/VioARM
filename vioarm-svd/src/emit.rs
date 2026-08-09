//! Render a [`Model`] as the Rust register tables `vioarm-core` consumes.

use crate::svd::{Access, Model, Peripheral, Register};

/// Emit the `stm32f103.rs` device table for `model`.
#[must_use]
pub fn emit(model: &Model, source: &str) -> String {
    let mut out = String::with_capacity(model.peripherals.len() * 512);
    w(
        &mut out,
        &format!("//! Generated from `{source}` by `vioarm-svd`."),
    );
    w(&mut out, "//!");
    w(&mut out, "//! Do not edit by hand. Regenerate with:");
    w(&mut out, "//!");
    w(&mut out, "//! ```sh");
    w(&mut out, "//! cargo run -p vioarm-svd -- \\");
    w(
        &mut out,
        "//!   packs/Keil.STM32F1xx_DFP/SVD/STM32F103xx.svd \\",
    );
    w(&mut out, "//!   vioarm-core/src/device/svd/stm32f103.rs");
    w(&mut out, "//! ```");
    w(&mut out, "");
    w(
        &mut out,
        "use super::{Access, FieldDef, IrqDef, PeripheralDef, RegisterDef};",
    );
    w(&mut out, "");
    w(
        &mut out,
        "/// Find the generated descriptor for a peripheral by SVD name.",
    );
    w(&mut out, "#[must_use]");
    w(
        &mut out,
        "pub fn peripheral(name: &str) -> Option<&'static PeripheralDef> {",
    );
    w(&mut out, "    PERIPHERALS.iter().find(|p| p.name == name)");
    w(&mut out, "}");
    w(&mut out, "");
    w(
        &mut out,
        "/// Every peripheral the SVD describes, in file order.",
    );
    w(&mut out, "pub const PERIPHERALS: &[PeripheralDef] = &[");
    for p in &model.peripherals {
        emit_peripheral(&mut out, p);
    }
    w(&mut out, "];");
    out
}

fn w(out: &mut String, line: &str) {
    out.push_str(line);
    out.push('\n');
}

fn emit_peripheral(out: &mut String, p: &Peripheral) {
    w(out, "    PeripheralDef {");
    w(out, &format!("        name: {:?},", p.name));
    w(out, &format!("        base: {},", hex(p.base)));
    w(out, &format!("        window: {},", hex(p.window)));
    if p.irqs.is_empty() {
        w(out, "        irqs: &[],");
    } else if p.irqs.len() == 1 {
        let i = &p.irqs[0];
        w(out, "        irqs: &[IrqDef {");
        w(out, &format!("            number: {},", i.number));
        w(out, &format!("            name: {:?},", i.name));
        w(out, "        }],");
    } else {
        w(out, "        irqs: &[");
        for i in &p.irqs {
            w(out, "            IrqDef {");
            w(out, &format!("                number: {},", i.number));
            w(out, &format!("                name: {:?},", i.name));
            w(out, "            },");
        }
        w(out, "        ],");
    }
    w(out, "        registers: &[");
    for r in &p.registers {
        emit_register(out, r);
    }
    w(out, "        ],");
    w(out, "    },");
}

fn emit_register(out: &mut String, r: &Register) {
    w(out, "            RegisterDef {");
    w(out, &format!("                name: {:?},", r.name));
    w(out, &format!("                offset: 0x{:02x},", r.offset));
    w(out, &format!("                width: {},", r.size));
    w(
        out,
        &format!("                access: {},", access_ident(r.access)),
    );
    w(out, &format!("                reset: {},", hex(r.reset)));
    if r.fields.is_empty() {
        w(out, "                fields: &[],");
    } else if r.fields.len() == 1 {
        let f = &r.fields[0];
        w(out, "                fields: &[FieldDef {");
        w(out, &format!("                    name: {:?},", f.name));
        w(out, &format!("                    offset: {},", f.offset));
        w(out, &format!("                    width: {},", f.width));
        w(
            out,
            &format!("                    access: {},", access_ident(f.access)),
        );
        w(out, "                }],");
    } else {
        w(out, "                fields: &[");
        for f in &r.fields {
            w(out, "                    FieldDef {");
            w(out, &format!("                        name: {:?},", f.name));
            w(
                out,
                &format!("                        offset: {},", f.offset),
            );
            w(out, &format!("                        width: {},", f.width));
            w(
                out,
                &format!(
                    "                        access: {},",
                    access_ident(f.access)
                ),
            );
            w(out, "                    },");
        }
        w(out, "                ],");
    }
    w(out, "            },");
}

const fn access_ident(a: Access) -> &'static str {
    match a {
        Access::ReadWrite => "Access::ReadWrite",
        Access::ReadOnly => "Access::ReadOnly",
        Access::WriteOnly => "Access::WriteOnly",
    }
}

/// `0x0000_0083`-style formatting for `u32` literals.
fn hex(v: u32) -> String {
    let s = format!("{v:08x}");
    format!("0x{}_{}", &s[0..4], &s[4..8])
}

#[cfg(test)]
mod tests {
    use super::hex;

    #[test]
    fn hex_uses_underscore_separators() {
        assert_eq!(hex(0x0000_0083), "0x0000_0083");
        assert_eq!(hex(0x4002_1000), "0x4002_1000");
        assert_eq!(hex(0), "0x0000_0000");
        assert_eq!(hex(u32::MAX), "0xffff_ffff");
    }
}

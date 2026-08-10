//! Render a [`Model`] as the Rust register tables `vioarm-core` consumes.

use crate::svd::{Access, Model, Peripheral, Register};

/// Emit the device table for `model`, reporting the regenerate command as the
/// `source`/`out` pair it was produced from.
#[must_use]
pub fn emit(model: &Model, source: &str, out: &str) -> String {
    let mut out_s = String::with_capacity(model.peripherals.len() * 512);
    w(
        &mut out_s,
        &format!("//! Generated from `{source}` by `vioarm-svd`."),
    );
    w(&mut out_s, "//!");
    w(&mut out_s, "//! Do not edit by hand. Regenerate with:");
    w(&mut out_s, "//!");
    w(&mut out_s, "//! ```sh");
    w(&mut out_s, "//! cargo run -p vioarm-svd -- \\");
    w(&mut out_s, &format!("//!   {source} \\"));
    w(&mut out_s, &format!("//!   {out}"));
    w(&mut out_s, "//! ```");
    w(&mut out_s, "");
    w(
        &mut out_s,
        "use super::{Access, Device, FieldDef, IrqDef, PeripheralDef, RegisterDef};",
    );
    w(&mut out_s, "");
    w(
        &mut out_s,
        "/// Find the generated descriptor for a peripheral by SVD name.",
    );
    w(&mut out_s, "#[must_use]");
    w(
        &mut out_s,
        "pub fn peripheral(name: &str) -> Option<&'static PeripheralDef> {",
    );
    w(
        &mut out_s,
        "    PERIPHERALS.iter().find(|p| p.name == name)",
    );
    w(&mut out_s, "}");
    w(&mut out_s, "");
    w(
        &mut out_s,
        "/// The chip type that mounts [`Device`] behavior on this table.",
    );
    w(&mut out_s, "#[derive(Clone, Copy, Debug, Default)]");
    w(
        &mut out_s,
        &format!("pub struct {};", chip_type(&model.device)),
    );
    w(&mut out_s, "");
    w(
        &mut out_s,
        &format!("impl Device for {} {{", chip_type(&model.device)),
    );
    w(
        &mut out_s,
        "    fn peripheral(name: &str) -> Option<&'static PeripheralDef> {",
    );
    w(&mut out_s, "        peripheral(name)");
    w(&mut out_s, "    }");
    w(
        &mut out_s,
        "    fn by_base(base: u32) -> Option<&'static PeripheralDef> {",
    );
    w(
        &mut out_s,
        "        PERIPHERALS.iter().find(|p| p.base == base)",
    );
    w(&mut out_s, "    }");
    w(&mut out_s, "}");
    w(&mut out_s, "");
    w(
        &mut out_s,
        "/// Every peripheral the SVD describes, in file order.",
    );
    w(&mut out_s, "pub const PERIPHERALS: &[PeripheralDef] = &[");
    for p in &model.peripherals {
        emit_peripheral(&mut out_s, p);
    }
    w(&mut out_s, "];");
    out_s
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
    match p.registers.len() {
        0 => w(out, "        registers: &[],"),
        1 => {
            w(out, "        registers: &[RegisterDef {");
            emit_register_body(out, &p.registers[0], "            ");
            w(out, "        }],");
        }
        _ => {
            w(out, "        registers: &[");
            for r in &p.registers {
                emit_register(out, r);
            }
            w(out, "        ],");
        }
    }
    w(out, "    },");
}

fn emit_register(out: &mut String, r: &Register) {
    w(out, "            RegisterDef {");
    emit_register_body(out, r, "                ");
    w(out, "            },");
}

/// The fields of a `RegisterDef` at `indent`, for single-element arrays where
/// rustfmt telescopes the brackets onto the one element.
fn emit_register_body(out: &mut String, r: &Register, indent: &str) {
    w(out, &format!("{indent}name: {:?},", r.name));
    w(out, &format!("{indent}offset: 0x{:02x},", r.offset));
    w(out, &format!("{indent}width: {},", r.size));
    w(out, &format!("{indent}access: {},", access_ident(r.access)));
    w(out, &format!("{indent}reset: {},", hex(r.reset)));
    match r.fields.len() {
        0 => w(out, &format!("{indent}fields: &[],")),
        1 => {
            let f = &r.fields[0];
            w(out, &format!("{indent}fields: &[FieldDef {{"));
            w(out, &format!("{indent}    name: {:?},", f.name));
            w(out, &format!("{indent}    offset: {},", f.offset));
            w(out, &format!("{indent}    width: {},", f.width));
            w(
                out,
                &format!("{indent}    access: {},", access_ident(f.access)),
            );
            w(out, &format!("{indent}}}],"));
        }
        _ => {
            w(out, &format!("{indent}fields: &["));
            for f in &r.fields {
                w(out, &format!("{indent}    FieldDef {{"));
                w(out, &format!("{indent}        name: {:?},", f.name));
                w(out, &format!("{indent}        offset: {},", f.offset));
                w(out, &format!("{indent}        width: {},", f.width));
                w(
                    out,
                    &format!("{indent}        access: {},", access_ident(f.access)),
                );
                w(out, &format!("{indent}    }},"));
            }
            w(out, &format!("{indent}],"));
        }
    }
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

/// The Rust type name for a device: `STM32F107xx` -> `Stm32f107`.
fn chip_type(device: &str) -> String {
    let trimmed = device.trim_end_matches(['x', 'X']);
    let mut chars = trimmed.chars();
    let first = chars
        .next()
        .map(|c| c.to_uppercase().collect::<String>())
        .unwrap_or_default();
    format!("{first}{}", chars.as_str().to_lowercase())
}

#[cfg(test)]
mod tests {
    use super::{chip_type, hex};

    #[test]
    fn hex_uses_underscore_separators() {
        assert_eq!(hex(0x0000_0083), "0x0000_0083");
        assert_eq!(hex(0x4002_1000), "0x4002_1000");
        assert_eq!(hex(0), "0x0000_0000");
        assert_eq!(hex(u32::MAX), "0xffff_ffff");
    }

    #[test]
    fn chip_type_strips_the_variant_suffix() {
        assert_eq!(chip_type("STM32F103xx"), "Stm32f103");
        assert_eq!(chip_type("STM32F107xx"), "Stm32f107");
        assert_eq!(chip_type("STM32F407"), "Stm32f407");
    }
}

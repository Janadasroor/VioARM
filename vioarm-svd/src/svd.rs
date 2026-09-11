//! A small, deterministic CMSIS-SVD reader.
//!
//! Only the elements `vioarm-svd` emits are parsed; descriptions, enumerated
//! values, and reset masks are skipped. Malformed *structure* is an error;
//! unknown *content* is ignored.

use std::error::Error;
use std::fmt;

use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
use quick_xml::XmlVersion;

/// The access vocabulary used by the supported SVD files.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    /// Readable and writable.
    ReadWrite,
    /// Readable only.
    ReadOnly,
    /// Writable only.
    WriteOnly,
}

/// A register bit field.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Field {
    /// Field name.
    pub name: String,
    /// Bit offset within the register.
    pub offset: u32,
    /// Bit width.
    pub width: u32,
    /// Access vocabulary.
    pub access: Access,
}

/// A single register.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Register {
    /// Register name.
    pub name: String,
    /// Byte offset within the peripheral.
    pub offset: u32,
    /// Register size in bits.
    pub size: u32,
    /// Access vocabulary.
    pub access: Access,
    /// Reset value.
    pub reset: u32,
    /// Bit fields.
    pub fields: Vec<Field>,
}

/// An interrupt line the peripheral can raise.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Irq {
    /// Interrupt name.
    pub name: String,
    /// Interrupt number.
    pub number: u8,
}

/// A fully-resolved peripheral: `derivedFrom` aliases are already copied in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Peripheral {
    /// Peripheral name.
    pub name: String,
    /// Base address.
    pub base: u32,
    /// Window size in bytes.
    pub window: u32,
    /// Interrupt lines.
    pub irqs: Vec<Irq>,
    /// Registers.
    pub registers: Vec<Register>,
}

/// The parsed, derived-from-resolved device model.
#[derive(Debug)]
pub struct Model {
    /// Device name.
    pub device: String,
    /// Peripherals.
    pub peripherals: Vec<Peripheral>,
}

/// Any SVD structure the reader cannot represent.
#[derive(Debug)]
pub struct ParseError(String);

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Error for ParseError {}

/// Element names whose text content is consumed.
const SCALARS: &[&[u8]] = &[
    b"name",
    b"baseAddress",
    b"addressOffset",
    b"size",
    b"access",
    b"resetValue",
    b"bitOffset",
    b"bitWidth",
    b"value",
];

#[derive(Clone, Default)]
struct FieldB {
    name: String,
    offset: u32,
    width: u32,
    access: Option<Access>,
}

#[derive(Clone, Default)]
struct RegisterB {
    name: String,
    offset: u32,
    size: u32,
    access: Option<Access>,
    reset: u32,
    fields: Vec<FieldB>,
}

#[derive(Clone, Default)]
struct IrqB {
    name: String,
    number: u8,
}

#[derive(Clone, Default)]
struct PeripheralB {
    name: String,
    base: u32,
    window: u32,
    irqs: Vec<IrqB>,
    registers: Vec<RegisterB>,
    derived_from: Option<String>,
}

/// An open builder: only element groups push one of these.
enum Node {
    Peripheral(PeripheralB),
    Register(RegisterB),
    Field(FieldB),
    Irq(IrqB),
}

/// Parse `xml` into a [`Model`], resolving `derivedFrom` peripherals.
///
/// # Errors
/// Returns [`ParseError`] on malformed XML or unknown structural elements.
#[allow(clippy::too_many_lines)]
pub fn parse(xml: &str) -> Result<Model, ParseError> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);

    let mut device = String::new();
    let mut nodes: Vec<Node> = Vec::new();
    let mut peripherals: Vec<PeripheralB> = Vec::new();
    let mut text = String::new();
    let mut capturing = false;
    let mut buf = Vec::new();

    loop {
        buf.clear();
        match reader.read_event_into(&mut buf) {
            Err(e) => return Err(ParseError(format!("xml error: {e}"))),
            Ok(Event::Eof) => break,
            Ok(Event::Start(e) | Event::Empty(e)) => {
                text.clear();
                capturing = SCALARS.contains(&e.name().as_ref());
                match e.name().as_ref() {
                    b"peripheral" => {
                        let derived_from = attr(&e, b"derivedFrom");
                        nodes.push(Node::Peripheral(PeripheralB {
                            derived_from,
                            ..PeripheralB::default()
                        }));
                    }
                    b"register" => nodes.push(Node::Register(RegisterB::default())),
                    b"field" => nodes.push(Node::Field(FieldB::default())),
                    b"interrupt" => nodes.push(Node::Irq(IrqB::default())),
                    _ => {}
                }
            }
            Ok(Event::Text(e)) => {
                if capturing {
                    let s = e
                        .xml_content(XmlVersion::Implicit1_0)
                        .map_err(|e| ParseError(format!("xml: {e}")))?;
                    text.push_str(&s);
                }
            }
            Ok(Event::End(e)) => {
                let qname = e.name();
                let name: &[u8] = qname.as_ref();
                if name == b"name" && nodes.is_empty() && device.is_empty() {
                    device = text.trim().to_string();
                } else if SCALARS.contains(&name) && capturing {
                    if let Some(node) = nodes.last_mut() {
                        assign(node, name, text.trim());
                    }
                }
                text.clear();
                capturing = false;
                match name {
                    b"peripheral" => {
                        let node = nodes
                            .pop()
                            .ok_or_else(|| ParseError("unbalanced </peripheral>".to_string()))?;
                        match node {
                            Node::Peripheral(p) => peripherals.push(p),
                            _ => {
                                return Err(ParseError(
                                    "</peripheral> closed the wrong node".to_string(),
                                ))
                            }
                        }
                    }
                    b"register" => {
                        let node = nodes
                            .pop()
                            .ok_or_else(|| ParseError("unbalanced </register>".to_string()))?;
                        match node {
                            Node::Register(r) => push_register(&mut nodes, r),
                            _ => {
                                return Err(ParseError(
                                    "</register> closed the wrong node".to_string(),
                                ))
                            }
                        }
                    }
                    b"field" => {
                        let node = nodes
                            .pop()
                            .ok_or_else(|| ParseError("unbalanced </field>".to_string()))?;
                        match node {
                            Node::Field(f) => push_field(&mut nodes, f),
                            _ => {
                                return Err(ParseError(
                                    "</field> closed the wrong node".to_string(),
                                ))
                            }
                        }
                    }
                    b"interrupt" => {
                        let node = nodes
                            .pop()
                            .ok_or_else(|| ParseError("unbalanced </interrupt>".to_string()))?;
                        match node {
                            Node::Irq(i) => push_irq(&mut nodes, i),
                            _ => {
                                return Err(ParseError(
                                    "</interrupt> closed the wrong node".to_string(),
                                ))
                            }
                        }
                    }
                    _ => {}
                }
            }
            Ok(_) => {}
        }
    }

    resolve(device, &peripherals)
}

/// Copy the value of attribute `key` from an element, if present.
fn attr(e: &BytesStart<'_>, key: &[u8]) -> Option<String> {
    e.attributes()
        .flatten()
        .find(|a| a.key.as_ref() == key)
        .and_then(|a| a.normalized_value(XmlVersion::Implicit1_0).ok())
        .map(|s| s.to_string())
}

fn push_register(nodes: &mut [Node], reg: RegisterB) {
    if let Some(Node::Peripheral(p)) = nodes.last_mut() {
        p.registers.push(reg);
    }
}

fn push_field(nodes: &mut [Node], field: FieldB) {
    if let Some(Node::Register(r)) = nodes.last_mut() {
        r.fields.push(field);
    }
}

fn push_irq(nodes: &mut [Node], irq: IrqB) {
    if let Some(Node::Peripheral(p)) = nodes.last_mut() {
        p.irqs.push(irq);
    }
}

fn assign(node: &mut Node, tag: &[u8], value: &str) {
    match (node, tag) {
        (Node::Peripheral(p), b"name") => p.name = value.to_string(),
        (Node::Peripheral(p), b"baseAddress") => p.base = parse_u32(value),
        (Node::Peripheral(p), b"size") => p.window = parse_u32(value),
        (Node::Register(r), b"name") => r.name = value.to_string(),
        (Node::Register(r), b"addressOffset") => r.offset = parse_u32(value),
        (Node::Register(r), b"size") => r.size = parse_u32(value),
        (Node::Register(r), b"access") => r.access = Some(parse_access(value)),
        (Node::Register(r), b"resetValue") => r.reset = parse_u32(value),
        (Node::Field(f), b"name") => f.name = value.to_string(),
        (Node::Field(f), b"bitOffset") => f.offset = parse_u32(value),
        (Node::Field(f), b"bitWidth") => f.width = parse_u32(value),
        (Node::Field(f), b"access") => f.access = Some(parse_access(value)),
        (Node::Irq(i), b"name") => i.name = value.to_string(),
        (Node::Irq(i), b"value") => {
            i.number = u8::try_from(parse_u32(value)).unwrap_or(u8::MAX);
        }
        _ => {}
    }
}

fn parse_u32(s: &str) -> u32 {
    s.trim()
        .strip_prefix("0x")
        .or_else(|| s.trim().strip_prefix("0X"))
        .map_or_else(
            || s.trim().parse().unwrap_or(0),
            |hex| u32::from_str_radix(hex, 16).unwrap_or(0),
        )
}

fn parse_access(s: &str) -> Access {
    match s.trim() {
        "read-only" => Access::ReadOnly,
        "write-only" => Access::WriteOnly,
        _ => Access::ReadWrite,
    }
}

fn resolve(device: String, parts: &[PeripheralB]) -> Result<Model, ParseError> {
    let mut out = Vec::with_capacity(parts.len());
    for p in parts {
        let src = match &p.derived_from {
            None => p,
            Some(d) => parts
                .iter()
                .find(|q| &q.name == d)
                .ok_or_else(|| ParseError(format!("derivedFrom {d} has no definition")))?,
        };
        out.push(Peripheral {
            name: p.name.clone(),
            base: p.base,
            window: if p.window != 0 { p.window } else { src.window },
            irqs: if p.irqs.is_empty() {
                src.irqs.iter().cloned().map(finish_irq).collect()
            } else {
                p.irqs.iter().cloned().map(finish_irq).collect()
            },
            registers: if p.registers.is_empty() {
                src.registers.iter().cloned().map(finish_register).collect()
            } else {
                p.registers.iter().cloned().map(finish_register).collect()
            },
        });
    }
    Ok(Model {
        device,
        peripherals: out,
    })
}

fn finish_register(r: RegisterB) -> Register {
    Register {
        name: r.name,
        offset: r.offset,
        size: r.size,
        access: r.access.unwrap_or(Access::ReadWrite),
        reset: r.reset,
        fields: r
            .fields
            .into_iter()
            .map(|f| Field {
                name: f.name,
                offset: f.offset,
                width: f.width,
                access: f.access.unwrap_or(Access::ReadWrite),
            })
            .collect(),
    }
}

fn finish_irq(i: IrqB) -> Irq {
    Irq {
        name: i.name,
        number: i.number,
    }
}

#[cfg(test)]
mod tests {
    use super::{parse, Access, Model};

    const SAMPLE: &str = r#"<device>
      <name>Sample</name>
      <peripheral derivedFrom="GPIOA">
        <name>GPIOB</name>
        <baseAddress>0x40010C00</baseAddress>
      </peripheral>
      <peripheral>
        <name>GPIOA</name>
        <baseAddress>0x40010800</baseAddress>
        <addressBlock>
          <offset>0x0</offset>
          <size>0x400</size>
          <usage>registers</usage>
        </addressBlock>
        <interrupt>
          <name>EXTI0</name>
          <value>6</value>
        </interrupt>
        <registers>
          <register>
            <name>CRL</name>
            <addressOffset>0x0</addressOffset>
            <size>0x20</size>
            <access>read-write</access>
            <resetValue>0x44444444</resetValue>
            <fields>
              <field>
                <name>MODE0</name>
                <bitOffset>0</bitOffset>
                <bitWidth>2</bitWidth>
                <access>read-write</access>
              </field>
              <field>
                <name>CNF0</name>
                <bitOffset>2</bitOffset>
                <bitWidth>2</bitWidth>
              </field>
            </fields>
          </register>
        </registers>
      </peripheral>
    </device>"#;

    #[test]
    fn parses_and_resolves_derived_peripherals() {
        let model: Model = parse(SAMPLE).unwrap();
        assert_eq!(model.device, "Sample");
        assert_eq!(model.peripherals.len(), 2);
        let source = &model.peripherals[1];
        let derived = &model.peripherals[0];
        assert_eq!(derived.base, 0x4001_0C00);
        assert_eq!(derived.window, source.window);
        assert_eq!(derived.registers.len(), source.registers.len());
    }

    #[test]
    fn parses_hex_and_decimal_numbers() {
        let model = parse(SAMPLE).unwrap();
        let reg = &model.peripherals[1].registers[0];
        assert_eq!(reg.offset, 0);
        assert_eq!(reg.size, 32);
        assert_eq!(reg.reset, 0x4444_4444);
        assert_eq!(reg.access, Access::ReadWrite);
        assert_eq!(reg.fields.len(), 2);
        assert_eq!(reg.fields[0].offset, 0);
        assert_eq!(reg.fields[0].width, 2);
        assert_eq!(reg.fields[1].access, Access::ReadWrite);
    }

    #[test]
    fn interrupt_value_is_parsed() {
        let model = parse(SAMPLE).unwrap();
        assert_eq!(model.peripherals[1].irqs[0].number, 6);
        assert_eq!(model.peripherals[1].irqs[0].name, "EXTI0");
    }

    #[test]
    fn cpu_name_does_not_override_the_device_name() {
        let with_cpu = r"<device>
          <name>STM32F407</name>
          <cpu><name>CM4</name></cpu>
          <peripheral><name>RNG</name><baseAddress>0x50060800</baseAddress></peripheral>
        </device>";
        let model = parse(with_cpu).unwrap();
        assert_eq!(model.device, "STM32F407");
        assert_eq!(model.peripherals.len(), 1);
    }

    #[test]
    fn missing_derived_source_is_an_error() {
        let bad = r#"<device><name>Bad</name>
          <peripheral derivedFrom="NOPE"><name>X</name><baseAddress>0x1</baseAddress></peripheral>
        </device>"#;
        assert!(parse(bad).is_err());
    }
}

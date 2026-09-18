//! Loading executable images into a `FlatMemory`.
//!
//! We support a plain binary (load at a fixed address), a minimal ELF32
//! loader that understands the output of `arm-none-eabi-gcc` / `rustc` for
//! Cortex-M (`PT_LOAD` segments), and Intel HEX (`.hex`), the format emitted
//! by most embedded toolchains and `objcopy -O ihex`.

use crate::error::ImageError;

/// A loaded program: relocation-free segments plus the entry point (if known).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LoadedImage {
    /// `(address, bytes)` to place into memory, in no particular order.
    pub segments: Vec<(u32, Vec<u8>)>,
    /// The ELF entry address, when provided.
    pub entry: Option<u32>,
    /// Named code/data addresses from the ELF symbol table (empty for
    /// flat/HEX images, or when the ELF carries no usable table). Used by
    /// the debug REPL (`disas` labels, `break <name>`); loading never
    /// fails for symbol reasons.
    pub symbols: Vec<Symbol>,
}

/// A named address from an ELF symbol table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Symbol {
    /// Symbol address with the Thumb bit normalized off.
    pub addr: u32,
    /// Symbol name as stored (may be empty for anonymous locals, which
    /// are skipped).
    pub name: String,
}

/// Load a raw binary at `base` (no entry-point metadata).
#[must_use]
pub fn load_flat(base: u32, bytes: &[u8]) -> LoadedImage {
    LoadedImage {
        segments: vec![(base, bytes.to_vec())],
        entry: None,
        symbols: Vec::new(),
    }
}

/// Parse a loadable ELF32 file into segments. Every `PT_LOAD` is placed at
/// its physical address (`p_paddr`, falling back to `p_vaddr` when zero):
/// that is where the bytes live in the programmed flash image, and the
/// firmware's own startup code copies `.data` to RAM and zeroes `.bss`
/// from there — exactly like QEMU's `-kernel` loader and like silicon.
/// A trailing `memsz - filesz` tail (standard `.bss` padding) is
/// zero-filled. `filesz > memsz` is malformed.
pub fn load_elf(bytes: &[u8]) -> Result<LoadedImage, ImageError> {
    if bytes.len() < 52 {
        return Err(ImageError::Truncated);
    }
    if &bytes[0..4] != b"\x7fELF" {
        return Err(ImageError::BadMagic);
    }
    if bytes[4] != 1 {
        return Err(ImageError::UnsupportedElfClass);
    }
    // Machine 40 == EM_ARM
    let e_machine = u16::from_le_bytes(bytes[18..20].try_into().unwrap());
    if e_machine != 40 {
        return Err(ImageError::UnsupportedMachine);
    }
    let entry = u32::from_le_bytes(bytes[24..28].try_into().unwrap());
    let phoff = u32::from_le_bytes(bytes[28..32].try_into().unwrap()) as usize;
    let phentsize = u16::from_le_bytes(bytes[42..44].try_into().unwrap()) as usize;
    let phnum = u16::from_le_bytes(bytes[44..46].try_into().unwrap()) as usize;

    let mut segments = Vec::new();
    for i in 0..phnum {
        let off = phoff + i * phentsize;
        let end = off.saturating_add(32);
        if end > bytes.len() {
            return Err(ImageError::Truncated);
        }
        let p_type = u32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
        if p_type != 1 {
            continue; // PT_LOAD only
        }
        let p_offset = u32::from_le_bytes(bytes[off + 4..off + 8].try_into().unwrap()) as usize;
        let p_vaddr = u32::from_le_bytes(bytes[off + 8..off + 12].try_into().unwrap());
        let p_paddr = u32::from_le_bytes(bytes[off + 12..off + 16].try_into().unwrap());
        let p_filesz = u32::from_le_bytes(bytes[off + 16..off + 20].try_into().unwrap()) as usize;
        let p_memsz = u32::from_le_bytes(bytes[off + 20..off + 24].try_into().unwrap()) as usize;
        if p_filesz > p_memsz {
            return Err(ImageError::UnsupportedBss);
        }
        let data_end = p_offset.saturating_add(p_filesz);
        if data_end > bytes.len() {
            return Err(ImageError::Truncated);
        }
        let mut data = vec![0u8; p_memsz];
        data[..p_filesz].copy_from_slice(&bytes[p_offset..data_end]);
        // Physical placement: the `.data` image lives in flash at its LMA.
        let place = if p_paddr != 0 { p_paddr } else { p_vaddr };
        segments.push((place, data));
    }
    if segments.is_empty() {
        return Err(ImageError::NoLoadableSegments);
    }
    Ok(LoadedImage {
        segments,
        entry: if entry != 0 { Some(entry) } else { None },
        symbols: elf_symbols(bytes),
    })
}

/// Best-effort ELF symbol harvest for the debug REPL: `(address, name)`
/// for defined function/object symbols. Any inconsistency (truncated
/// tables, missing string table, absurd counts) yields an empty vec —
/// symbols are advisory and never fail a load.
fn elf_symbols(bytes: &[u8]) -> Vec<Symbol> {
    harvest_symbols(bytes).unwrap_or_default()
}

fn harvest_symbols(bytes: &[u8]) -> Option<Vec<Symbol>> {
    const SYMTAB: u32 = 2;
    const MAX_SYMBOLS: usize = 4096;
    let mut out = Vec::new();
    let u16_at = |off: usize| -> Option<u16> {
        bytes
            .get(off..off + 2)
            .map(|b| u16::from_le_bytes([b[0], b[1]]))
    };
    let u32_at = |off: usize| -> Option<u32> {
        bytes
            .get(off..off + 4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    };
    let shoff = u32_at(32)? as usize;
    let shentsize = u16_at(46)? as usize;
    let shnum = u16_at(48)? as usize;
    if shentsize < 40 || shnum == 0 || shnum > 256 {
        return None;
    }
    // Section headers: sh_name(0) sh_type(4) sh_addr(12) sh_offset(16)
    // sh_size(20) sh_link(24) sh_info(28).
    let mut symtab: Option<(usize, usize, usize)> = None; // (offset, size, strtab-index)
    let mut sections: Vec<(usize, usize)> = Vec::new(); // (offset, size) by index
    for i in 0..shnum {
        let base = shoff.checked_add(i.checked_mul(shentsize)?)?;
        let sh_type = u32_at(base.checked_add(4)?)?;
        let sh_offset = u32_at(base.checked_add(16)?)? as usize;
        let sh_size = u32_at(base.checked_add(20)?)? as usize;
        let sh_link = u32_at(base.checked_add(24)?)? as usize;
        sections.push((sh_offset, sh_size));
        if sh_type == SYMTAB && symtab.is_none() {
            symtab = Some((sh_offset, sh_size, sh_link));
        }
    }
    let (sym_off, sym_size, str_idx) = symtab?;
    let (str_off, str_size) = *sections.get(str_idx)?;
    let str_end = str_off.checked_add(str_size)?;
    if str_end > bytes.len() || sym_off.checked_add(sym_size)? > bytes.len() {
        return None;
    }
    let count = (sym_size / 16).min(MAX_SYMBOLS);
    for i in 0..count {
        let base = sym_off + i * 16;
        let st_name = u32_at(base)? as usize;
        let st_value = u32_at(base + 4)?;
        let st_info = *bytes.get(base + 12)?;
        let st_shndx = u16_at(base + 14)?;
        // Defined (non-UND) function/object/notype symbols only; skip
        // file/section markers and anonymous locals.
        if st_shndx == 0 {
            continue;
        }
        let bind = st_info >> 4;
        let ty = st_info & 0xF;
        if !matches!(bind, 0..=2) || !matches!(ty, 0..=2) {
            continue;
        }
        let name_off = str_off.checked_add(st_name)?;
        if name_off >= str_end {
            continue;
        }
        let name_end = bytes[name_off..str_end]
            .iter()
            .position(|&b| b == 0)
            .map(|len| name_off + len)
            .unwrap_or(str_end);
        if name_end == name_off {
            continue;
        }
        let name = String::from_utf8_lossy(&bytes[name_off..name_end]).into_owned();
        out.push(Symbol {
            addr: st_value & !1, // Thumb bit normalized off
            name,
        });
    }
    out.sort_by_key(|s| s.addr);
    out.dedup_by(|a, b| a.addr == b.addr && a.name == b.name);
    Some(out)
}

/// Parse an Intel HEX stream into segments.
///
/// Records are `:llaaaatt[dd]cc` lines with an 8-bit checksum such that all
/// bytes on the line (count, address, type, data, checksum) sum to zero mod
/// 256. The 16-bit record address is extended by type 04 (linear) or type 02
/// (segment) records; type 05 records supply the entry point. Data bytes are
/// accumulated into contiguous segments, and a missing EOF record is
/// tolerated (many tools omit it).
pub fn load_hex(bytes: &[u8]) -> Result<LoadedImage, ImageError> {
    let mut segments: Vec<(u32, Vec<u8>)> = Vec::new();
    let mut upper: u32 = 0;
    let mut entry: Option<u32> = None;

    for line in split_lines(bytes) {
        if line.is_empty() {
            continue;
        }
        if line.len() < 11 || line[0] != b':' {
            return Err(ImageError::HexSyntax);
        }
        let count = ((nib(line[1])? << 4) | nib(line[2])?) as usize;
        let addr_hi = (nib(line[3])? << 4) | nib(line[4])?;
        let addr_lo = (nib(line[5])? << 4) | nib(line[6])?;
        let rtype = (nib(line[7])? << 4) | nib(line[8])?;
        if line.len() != 11 + 2 * count {
            return Err(ImageError::HexSyntax);
        }

        let mut sum = u8::try_from(count).map_err(|_| ImageError::HexSyntax)?;
        sum = sum
            .wrapping_add(addr_hi)
            .wrapping_add(addr_lo)
            .wrapping_add(rtype);
        let mut data = vec![0u8; count];
        for (i, slot) in data.iter_mut().enumerate() {
            *slot = (nib(line[9 + 2 * i])? << 4) | nib(line[10 + 2 * i])?;
            sum = sum.wrapping_add(*slot);
        }
        let checksum = (nib(line[9 + 2 * count])? << 4) | nib(line[10 + 2 * count])?;
        if sum.wrapping_add(checksum) != 0 {
            return Err(ImageError::HexChecksum);
        }

        match rtype {
            0 => {
                let addr = upper
                    .checked_add(u32::from(addr_hi) << 8 | u32::from(addr_lo))
                    .ok_or(ImageError::HexAddressOverflow)?;
                let len = u32::try_from(data.len()).map_err(|_| ImageError::HexAddressOverflow)?;
                addr.checked_add(len)
                    .ok_or(ImageError::HexAddressOverflow)?;
                push_segment(&mut segments, addr, &data);
            }
            1 => break, // end-of-file record
            2 => upper = (u32::from(data[0]) << 8 | u32::from(data[1])) << 4,
            4 => upper = (u32::from(data[0]) << 8 | u32::from(data[1])) << 16,
            5 => {
                entry = Some(
                    u32::from(data[0]) << 24
                        | u32::from(data[1]) << 16
                        | u32::from(data[2]) << 8
                        | u32::from(data[3]),
                );
            }
            _ => return Err(ImageError::HexSyntax),
        }
    }
    if segments.is_empty() {
        return Err(ImageError::NoLoadableSegments);
    }
    Ok(LoadedImage {
        segments,
        entry: entry.filter(|&e| e != 0),
        symbols: Vec::new(),
    })
}

const fn nib(c: u8) -> Result<u8, ImageError> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(ImageError::HexSyntax),
    }
}

/// Append `bytes` at `addr`, merging into the previous segment when it is
/// contiguous (records in one page typically arrive in address order).
fn push_segment(segments: &mut Vec<(u32, Vec<u8>)>, addr: u32, bytes: &[u8]) {
    let mut merged = false;
    if let Some((prev_addr, prev)) = segments.last_mut() {
        if let Ok(prev_len) = u32::try_from(prev.len()) {
            if prev_addr.saturating_add(prev_len) == addr {
                prev.extend_from_slice(bytes);
                merged = true;
            }
        }
    }
    if !merged {
        segments.push((addr, bytes.to_vec()));
    }
}

fn split_lines(bytes: &[u8]) -> impl Iterator<Item = &[u8]> {
    bytes
        .split(|&b| b == b'\n')
        .map(|line| line.strip_suffix(b"\r").unwrap_or(line))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write as _;

    #[test]
    fn flat_load() {
        let img = load_flat(0x0000_0000, &[1, 2, 3, 4]);
        assert_eq!(img.segments[0].1, vec![1, 2, 3, 4]);
        assert!(img.entry.is_none());
    }

    #[test]
    fn bad_magic() {
        assert_eq!(load_elf(&[0; 64]), Err(ImageError::BadMagic));
    }

    /// Build an Intel HEX record line with a correct checksum.
    #[allow(clippy::cast_possible_truncation)]
    fn record(addr: u16, rtype: u8, data: &[u8]) -> String {
        let mut line = String::from(":");
        let _ = write!(line, "{:02X}{:04X}{:02X}", data.len(), addr, rtype);
        let mut sum = data.len() as u8 + (addr >> 8) as u8 + addr as u8 + rtype;
        for &b in data {
            let _ = write!(line, "{b:02X}");
            sum = sum.wrapping_add(b);
        }
        let _ = write!(line, "{:02X}", 0u8.wrapping_sub(sum));
        line
    }

    #[test]
    fn hex_loads_contiguous_records() {
        let text = format!(
            "{}\n{}\n{}\n",
            record(0x0100, 0, &[0x10, 0x20, 0x30]),
            record(0x0103, 0, &[0x40, 0x50]),
            record(0x0000, 1, &[])
        );
        let img = load_hex(text.as_bytes()).unwrap();
        assert_eq!(
            img.segments,
            vec![(0x0100, vec![0x10, 0x20, 0x30, 0x40, 0x50])]
        );
        assert!(img.entry.is_none());
    }

    #[test]
    fn hex_extended_linear_address() {
        let text = format!(
            "{}\n{}\n{}\n",
            record(0, 4, &[0x20, 0x00]),
            record(0x0000, 0, &[0xAA, 0xBB]),
            record(0x0000, 1, &[])
        );
        let img = load_hex(text.as_bytes()).unwrap();
        assert_eq!(img.segments, vec![(0x2000_0000, vec![0xAA, 0xBB])]);
    }

    #[test]
    fn hex_start_linear_address_sets_entry() {
        let text = format!(
            "{}\n{}\n{}\n",
            record(0x0100, 0, &[0x01, 0x02]),
            record(0, 5, &[0x00, 0x01, 0x02, 0x03]),
            record(0x0000, 1, &[])
        );
        let img = load_hex(text.as_bytes()).unwrap();
        assert_eq!(img.entry, Some(0x0001_0203));
    }

    #[test]
    fn hex_missing_eof_is_tolerated() {
        let text = format!("{}\n", record(0x0000, 0, &[1, 2, 3]));
        let img = load_hex(text.as_bytes()).unwrap();
        assert_eq!(img.segments, vec![(0x0000, vec![1, 2, 3])]);
    }

    #[test]
    fn hex_bad_checksum_is_rejected() {
        let mut line = record(0x0000, 0, &[1, 2, 3]);
        line.pop();
        line.push('0');
        assert_eq!(load_hex(line.as_bytes()), Err(ImageError::HexChecksum));
    }

    #[test]
    fn hex_syntax_errors_are_rejected() {
        assert_eq!(load_hex(b"no colon here"), Err(ImageError::HexSyntax));
        let truncated = record(0x0000, 0, &[1, 2, 3]);
        assert_eq!(
            load_hex(&truncated.as_bytes()[..10]),
            Err(ImageError::HexSyntax)
        );
        assert_eq!(load_hex(b":00"), Err(ImageError::HexSyntax));
    }

    #[test]
    fn hex_empty_stream_is_no_loadable_segments() {
        assert_eq!(load_hex(b"\n\n"), Err(ImageError::NoLoadableSegments));
    }

    /// Minimal ELF32 builder: one PT_LOAD plus `.symtab`/`.strtab` so the
    /// symbol harvest has something exact to chew on.
    fn elf_with_symbols() -> Vec<u8> {
        let mut img = vec![0u8; 52];
        img[0..4].copy_from_slice(b"\x7fELF");
        img[4] = 1; // 32-bit
        img[5] = 1; // little-endian
        img[6] = 1; // version
        img[16..18].copy_from_slice(&2u16.to_le_bytes()); // EXEC
        img[18..20].copy_from_slice(&40u16.to_le_bytes()); // EM_ARM
        img[20..24].copy_from_slice(&1u32.to_le_bytes());
        img[24..28].copy_from_slice(&0x100u32.to_le_bytes()); // entry
        img[28..32].copy_from_slice(&52u32.to_le_bytes()); // phoff
        img[42..44].copy_from_slice(&32u16.to_le_bytes()); // phentsize
        img[44..46].copy_from_slice(&1u16.to_le_bytes()); // phnum
        img[46..48].copy_from_slice(&40u16.to_le_bytes()); // shentsize
        img[48..50].copy_from_slice(&4u16.to_le_bytes()); // shnum
        img[50..52].copy_from_slice(&3u16.to_le_bytes()); // shstrndx
                                                          // PHDR: PT_LOAD, 4 bytes at 0x100.
        let text_off = 52 + 32;
        img.extend_from_slice(&1u32.to_le_bytes());
        img.extend_from_slice(&(text_off as u32).to_le_bytes());
        img.extend_from_slice(&0x100u32.to_le_bytes());
        img.extend_from_slice(&0x100u32.to_le_bytes());
        img.extend_from_slice(&4u32.to_le_bytes());
        img.extend_from_slice(&4u32.to_le_bytes());
        img.extend_from_slice(&5u32.to_le_bytes());
        img.extend_from_slice(&4u32.to_le_bytes());
        // .text: `b .` spin.
        img.extend_from_slice(&[0xFE, 0xE7, 0x00, 0x00]);
        // .strtab: "\0main\0".
        let str_off = img.len();
        img.extend_from_slice(b"\0main\0");
        // .symtab: NULL + `main` FUNC/GLOBAL at 0x105 (Thumb bit set).
        let sym_off = img.len();
        img.extend_from_slice(&[0u8; 16]);
        img.extend_from_slice(&1u32.to_le_bytes()); // st_name
        img.extend_from_slice(&0x105u32.to_le_bytes()); // st_value
        img.extend_from_slice(&0u32.to_le_bytes()); // st_size
        img.push(0x12); // GLOBAL FUNC
        img.push(0);
        img.extend_from_slice(&1u16.to_le_bytes()); // defined
        img.push(0); // st_other padding to 16 bytes
                     // .shstrtab (index 3): "\0.symtab\0.strtab\0.shstrtab\0".
        let shstr_off = img.len();
        img.extend_from_slice(b"\0.symtab\0.strtab\0.shstrtab\0");
        // Section headers at shoff.
        let shoff = img.len() as u32;
        img[32..36].copy_from_slice(&shoff.to_le_bytes());
        let mut sh = |name: u32, ty: u32, off: usize, size: usize, link: u32| {
            img.extend_from_slice(&name.to_le_bytes());
            img.extend_from_slice(&ty.to_le_bytes());
            img.extend_from_slice(&0u32.to_le_bytes()); // flags
            img.extend_from_slice(&0u32.to_le_bytes()); // addr
            img.extend_from_slice(&(off as u32).to_le_bytes());
            img.extend_from_slice(&(size as u32).to_le_bytes());
            img.extend_from_slice(&link.to_le_bytes());
            img.extend_from_slice(&0u32.to_le_bytes()); // info
            img.extend_from_slice(&4u32.to_le_bytes()); // addralign
            img.extend_from_slice(&0u32.to_le_bytes()); // entsize
        };
        sh(0, 0, 0, 0, 0); // NULL
        sh(1, 2, sym_off, 32, 2); // .symtab -> strtab index 2
        sh(9, 3, str_off, 6, 0); // .strtab
        sh(17, 3, shstr_off, 27, 0); // .shstrtab
        img
    }

    #[test]
    fn elf_symbols_harvest_defined_names() {
        let img = load_elf(&elf_with_symbols()).unwrap();
        assert_eq!(img.entry, Some(0x100));
        assert_eq!(
            img.symbols,
            vec![Symbol {
                addr: 0x104, // Thumb bit normalized off
                name: "main".to_owned(),
            }]
        );
    }

    #[test]
    fn elf_symbols_missing_table_is_empty_not_error() {
        // Cut the section tables away (keep the PHDR + .text): segments
        // still load, symbols come back empty.
        let mut bytes = elf_with_symbols();
        bytes.truncate(88);
        let img = load_elf(&bytes).unwrap();
        assert!(!img.segments.is_empty());
        assert!(img.symbols.is_empty());
    }

    /// A `.data` `PT_LOAD` whose LMA (`p_paddr`, in flash) differs from its
    /// VMA (`p_vaddr`, in RAM) must land at the LMA with a zero tail — that
    /// is where the startup code copies it from. (Found running Dhrystone:
    /// the old `p_vaddr` placement left `.data` uninitialized in flash and
    /// newlib's `_impure_ptr` read NULL.)
    #[test]
    fn elf_data_segment_lands_at_lma_with_zero_tail() {
        let mut img = vec![0u8; 52];
        img[0..4].copy_from_slice(b"\x7fELF");
        img[4] = 1;
        img[18..20].copy_from_slice(&40u16.to_le_bytes()); // EM_ARM
        img[24..28].copy_from_slice(&0x100u32.to_le_bytes()); // entry
        img[28..32].copy_from_slice(&52u32.to_le_bytes()); // phoff
        img[42..44].copy_from_slice(&32u16.to_le_bytes()); // phentsize
        img[44..46].copy_from_slice(&2u16.to_le_bytes()); // phnum
        let text_off = 52 + 64;
        let data_off = text_off + 4;
        // PHDR0: .text at 0x100 (LMA == VMA).
        img.extend_from_slice(&1u32.to_le_bytes());
        img.extend_from_slice(&(text_off as u32).to_le_bytes());
        img.extend_from_slice(&0x100u32.to_le_bytes()); // vaddr
        img.extend_from_slice(&0x100u32.to_le_bytes()); // paddr
        img.extend_from_slice(&4u32.to_le_bytes()); // filesz
        img.extend_from_slice(&4u32.to_le_bytes()); // memsz
        img.extend_from_slice(&5u32.to_le_bytes()); // flags
        img.extend_from_slice(&4u32.to_le_bytes()); // align
                                                    // PHDR1: .data, 2 file bytes at LMA 0x200, VMA RAM, 6 mem bytes.
        img.extend_from_slice(&1u32.to_le_bytes());
        img.extend_from_slice(&(data_off as u32).to_le_bytes());
        img.extend_from_slice(&0x2000_0000u32.to_le_bytes()); // vaddr
        img.extend_from_slice(&0x200u32.to_le_bytes()); // paddr (LMA)
        img.extend_from_slice(&2u32.to_le_bytes()); // filesz
        img.extend_from_slice(&6u32.to_le_bytes()); // memsz (.bss tail)
        img.extend_from_slice(&6u32.to_le_bytes()); // flags
        img.extend_from_slice(&4u32.to_le_bytes()); // align
        img.extend_from_slice(&[0xFE, 0xE7, 0x00, 0x00]); // .text
        img.extend_from_slice(&[0xAA, 0xBB]); // .data image
        let img = load_elf(&img).unwrap();
        assert_eq!(
            img.segments,
            vec![
                (0x100, vec![0xFE, 0xE7, 0x00, 0x00]),
                (0x200, vec![0xAA, 0xBB, 0, 0, 0, 0]),
            ]
        );
    }
}

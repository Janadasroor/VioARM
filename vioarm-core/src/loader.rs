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
}

/// Load a raw binary at `base` (no entry-point metadata).
#[must_use]
pub fn load_flat(base: u32, bytes: &[u8]) -> LoadedImage {
    LoadedImage {
        segments: vec![(base, bytes.to_vec())],
        entry: None,
    }
}

/// Parse a loadable ELF32 file into segments. Only `PT_LOAD` with
/// `filesz == memsz` is supported for the first milestone; `.bss` zero-padding
/// returns `ImageError::UnsupportedElfClass`... err, a distinct error below.
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
        let p_filesz = u32::from_le_bytes(bytes[off + 16..off + 20].try_into().unwrap()) as usize;
        let p_memsz = u32::from_le_bytes(bytes[off + 20..off + 24].try_into().unwrap()) as usize;
        if p_filesz != p_memsz {
            return Err(ImageError::UnsupportedBss);
        }
        let data_end = p_offset.saturating_add(p_filesz);
        if data_end > bytes.len() {
            return Err(ImageError::Truncated);
        }
        let data = bytes[p_offset..data_end].to_vec();
        segments.push((p_vaddr, data));
    }
    if segments.is_empty() {
        return Err(ImageError::NoLoadableSegments);
    }
    Ok(LoadedImage {
        segments,
        entry: if entry != 0 { Some(entry) } else { None },
    })
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
}

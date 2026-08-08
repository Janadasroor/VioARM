//! Loading executable images into a `FlatMemory`.
//!
//! For the first milestone we support a plain binary (load at a fixed
//! address) and a minimal ELF32 loader that understands the output of
//! `arm-none-eabi-gcc` / `rustc` for Cortex-M (`PT_LOAD` segments).

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

#[cfg(test)]
mod tests {
    use super::*;

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
}

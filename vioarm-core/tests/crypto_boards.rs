//! Crypto behaviors on real board constructors: AES/SAES/HASH on `u585`,
//! HASH on `u575`, AES on `l476`/`g081` (incl. G0 256-bit), AES+RNG on
//! `g041`. Register pokes through the board memory prove the windows live
//! at silicon addresses with NIST vectors, not just in unit tests.

use vioarm_core::board;
use vioarm_core::mem::Bus;

const AES_CR: u32 = 0x00;
const AES_SR: u32 = 0x04;
const AES_DINR: u32 = 0x08;
const AES_DOUTR: u32 = 0x0c;
const AES_KEYR0: u32 = 0x10;
const HASH_CR: u32 = 0x00;
const HASH_DIN: u32 = 0x04;
const HASH_STR: u32 = 0x08;
const HASH_HR0: u32 = 0x310;

fn hex(s: &[u8]) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(std::str::from_utf8(&s[i..i + 2]).unwrap(), 16).unwrap())
        .collect()
}

fn be_words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .chunks_exact(4)
        .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

/// One ECB block through the AES window at `base`.
fn aes_ecb(mem: &mut board::BoardMemory, base: u32, key: &[u8], pt: &[u8]) -> Vec<u8> {
    mem.write_word(base + AES_CR, 0).unwrap();
    for (i, w) in be_words(key).iter().enumerate() {
        let off = if i < 4 {
            AES_KEYR0 + 4 * i as u32
        } else {
            0x30 + 4 * (i - 4) as u32
        };
        mem.write_word(base + off, *w).unwrap();
    }
    mem.write_word(base + AES_CR, 1).unwrap();
    for w in be_words(pt) {
        mem.write_word(base + AES_DINR, w).unwrap();
    }
    assert_ne!(mem.read_word(base + AES_SR).unwrap() & 1, 0, "CCF");
    let mut out = Vec::new();
    for _ in 0..4 {
        out.extend(mem.read_word(base + AES_DOUTR).unwrap().to_be_bytes());
    }
    out
}

/// One SHA-256 message through the HASH window at `base`.
fn sha256(mem: &mut board::BoardMemory, base: u32, msg: &[u8]) -> Vec<u32> {
    mem.write_word(base + HASH_CR, (3 << 17) | (2 << 4) | (1 << 2))
        .unwrap();
    let mut chunks = msg.chunks_exact(4);
    for c in &mut chunks {
        mem.write_word(
            base + HASH_DIN,
            u32::from_le_bytes([c[0], c[1], c[2], c[3]]),
        )
        .unwrap();
    }
    let rem = chunks.remainder();
    if !rem.is_empty() {
        let mut last = [0u8; 4];
        last[..rem.len()].copy_from_slice(rem);
        mem.write_word(base + HASH_DIN, u32::from_le_bytes(last))
            .unwrap();
    }
    let nblw = if rem.is_empty() {
        0
    } else {
        rem.len() as u32 * 8
    };
    mem.write_word(base + HASH_STR, (1 << 8) | nblw).unwrap();
    assert_ne!(mem.read_word(base + 0x24).unwrap() & (1 << 1), 0, "DCIS");
    (0..8)
        .map(|i| mem.read_word(base + HASH_HR0 + 4 * i).unwrap())
        .collect()
}

#[test]
fn u585_aes_saes_hash_nist() {
    let mut mem = board::stm32u585();
    let key = hex(b"000102030405060708090a0b0c0d0e0f");
    let pt = hex(b"00112233445566778899aabbccddeeff");
    let ct = hex(b"69c4e0d86a7b0430d8cdb78070b4c55a");
    // AES + SAES share the core through their own windows.
    assert_eq!(aes_ecb(&mut mem, 0x420C_0000, &key, &pt), ct, "AES");
    assert_eq!(aes_ecb(&mut mem, 0x420C_0C00, &key, &pt), ct, "SAES");
    // 256-bit through AES.
    let key256 = hex(b"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
    mem.write_word(0x420C_0000 + AES_CR, 0).unwrap();
    for (i, w) in be_words(&key256).iter().enumerate() {
        let off = if i < 4 {
            AES_KEYR0 + 4 * i as u32
        } else {
            0x30 + 4 * (i - 4) as u32
        };
        mem.write_word(0x420C_0000 + off, *w).unwrap();
    }
    mem.write_word(0x420C_0000 + AES_CR, (1 << 18) | 1).unwrap();
    for w in be_words(&pt) {
        mem.write_word(0x420C_0000 + AES_DINR, w).unwrap();
    }
    let mut out = Vec::new();
    for _ in 0..4 {
        out.extend(
            mem.read_word(0x420C_0000 + AES_DOUTR)
                .unwrap()
                .to_be_bytes(),
        );
    }
    assert_eq!(out, hex(b"8ea2b7ca516745bfeafc49904b496089"), "AES-256");
    // HASH SHA-256("abc").
    assert_eq!(
        sha256(&mut mem, 0x420C_0400, b"abc")[0],
        0xba7816bf,
        "SHA-256(abc)"
    );
}

#[test]
fn u575_hash_upgraded_from_latch() {
    let mut mem = board::stm32u575();
    assert_eq!(
        sha256(&mut mem, 0x420C_0400, b"abc")[0],
        0xba7816bf,
        "SHA-256(abc)"
    );
}

#[test]
fn l476_aes_ecb() {
    let mut mem = board::stm32l476();
    let key = hex(b"000102030405060708090a0b0c0d0e0f");
    let pt = hex(b"00112233445566778899aabbccddeeff");
    assert_eq!(
        aes_ecb(&mut mem, 0x5006_0000, &key, &pt),
        hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"),
        "L476 AES"
    );
}

#[test]
fn g081_aes_supports_256_bit() {
    let mut mem = board::stm32g081();
    let key256 = hex(b"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
    let pt = hex(b"00112233445566778899aabbccddeeff");
    mem.write_word(0x4002_6000 + AES_CR, 0).unwrap();
    for (i, w) in be_words(&key256).iter().enumerate() {
        let off = if i < 4 {
            AES_KEYR0 + 4 * i as u32
        } else {
            0x30 + 4 * (i - 4) as u32
        };
        mem.write_word(0x4002_6000 + off, *w).unwrap();
    }
    mem.write_word(0x4002_6000 + AES_CR, (1 << 18) | 1).unwrap();
    for w in be_words(&pt) {
        mem.write_word(0x4002_6000 + AES_DINR, w).unwrap();
    }
    let mut out = Vec::new();
    for _ in 0..4 {
        out.extend(
            mem.read_word(0x4002_6000 + AES_DOUTR)
                .unwrap()
                .to_be_bytes(),
        );
    }
    assert_eq!(out, hex(b"8ea2b7ca516745bfeafc49904b496089"), "G0 AES-256");
}

#[test]
fn g041_aes_rng_manual_windows() {
    let mut mem = board::stm32g041();
    let key = hex(b"000102030405060708090a0b0c0d0e0f");
    let pt = hex(b"00112233445566778899aabbccddeeff");
    assert_eq!(
        aes_ecb(&mut mem, 0x4002_6000, &key, &pt),
        hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"),
        "G041 AES"
    );
    // RNG produces data once enabled.
    mem.write_word(0x4002_5000, 1 << 2).unwrap(); // RNGEN
    mem.tick_devices(1);
    assert_ne!(mem.read_word(0x4002_5000 + 0x04).unwrap() & 1, 0, "DRDY");
}

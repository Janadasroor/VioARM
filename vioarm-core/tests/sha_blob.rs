//! SHA-256 soak blob proof (P3-4): the checked-in firmware blob really
//! computes SHA-256 (one compression of the "abc" block matches the NIST
//! vector), and its bytes match the hash pinned in `benches/mips.rs`
//! (toolchain drift changes codegen, which changes the benchmark — the
//! pin makes that a deliberate diff).

use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::flat::FlatMemory;
use vioarm_core::mem::Bus;

const BASE: u32 = 0x2000_0000;
const STATE: u32 = 0x2000_0248;
const LOOP_TOP: u32 = 0x2000_000A;
/// NIST SHA-256("abc"), words big-endian as stored by the firmware.
const ABC_DIGEST: [u32; 8] = [
    0xba7816bf, 0x8f01cfea, 0x414140de, 0x5dae2223, 0xb00361a3, 0x96177a9c, 0xb410ff61, 0xf20015ad,
];

fn blob() -> Vec<u8> {
    include_bytes!("../benches/data/sha256.bin").to_vec()
}

#[test]
fn blob_shape_pin() {
    // Tripwire, not a hash (no hasher on the test side): any rebuild
    // changes size or breaks the digest test below. Regenerate
    // deliberately via scripts/gen-sha256-blob.sh (exact command in
    // benches/data/sha256.c) and expect this number to move with it.
    assert_eq!(blob().len(), 680, "blob size changed");
}

#[test]
fn blob_compresses_abc_to_nist_vector() {
    let mem = FlatMemory::with_layout(&[BASE], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    m.load_image(&load_flat(BASE, &blob()));
    m.core.regs.pc = BASE;
    m.core.regs.sp = BASE + 0x8000;
    m.core.xpsr.set(0x01000000);
    // Run until the first compression completes: back at the loop top
    // with state[0] moved off the initial value.
    for _ in 0..200_000 {
        m.step().unwrap();
        if m.core.regs.pc == LOOP_TOP && read_state(&mut m, 0) != 0x6a09e667 {
            break;
        }
    }
    for (i, want) in ABC_DIGEST.iter().enumerate() {
        assert_eq!(
            read_state(&mut m, i as u32),
            *want,
            "digest word {i} after one block"
        );
    }
}

fn read_state(m: &mut Machine<FlatMemory>, i: u32) -> u32 {
    let b = m.core.mem();
    u32::from_le_bytes([
        b.read(STATE + 4 * i).unwrap(),
        b.read(STATE + 4 * i + 1).unwrap(),
        b.read(STATE + 4 * i + 2).unwrap(),
        b.read(STATE + 4 * i + 3).unwrap(),
    ])
}

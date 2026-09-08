/* SHA-256 compression soak firmware (P3-4 benchmarks).
 *
 * Repeatedly compresses one "abc"-padded 512-bit block with the standard
 * initial state. K/H/block tables are exact (see scripts/gen-sha256.py,
 * verified against NIST vectors and the on-chip HASH engine in
 * tests/crypto_sha_blob.rs). Built with:
 *   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -ffreestanding \
 *     -nostdlib -T sha256.ld -o sha256.elf sha256.c
 *   arm-none-eabi-objcopy -O binary sha256.elf sha256.bin
 * The checked-in blob is pinned by hash in benches/mips.rs (rebuilds
 * with another toolchain change codegen, which changes the benchmark).
 */

typedef unsigned int u32;

static const u32 K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
    0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
    0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
    0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
    0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
    0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
    0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

/* H initial state, and the "abc" message block (padded, big-endian). */
static volatile u32 state[8] = {
    0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
    0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
};

static volatile u32 block[16] = {
    0x61626380u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000018u,
};

static u32 rotr(u32 x, unsigned n) {
    return (x >> n) | (x << (32 - n));
}

static void sha256_block(void) {
    u32 a = state[0], b = state[1], c = state[2], d = state[3];
    u32 e = state[4], f = state[5], g = state[6], h = state[7];
    /* volatile w[] keeps the schedule in RAM (realistic spill traffic). */
    volatile u32 w[64];
    unsigned t;
    for (t = 0; t < 16; t++) {
        w[t] = block[t];
    }
    for (t = 16; t < 64; t++) {
        u32 s0 = rotr(w[t - 15], 7) ^ rotr(w[t - 15], 18) ^ (w[t - 15] >> 3);
        u32 s1 = rotr(w[t - 2], 17) ^ rotr(w[t - 2], 19) ^ (w[t - 2] >> 10);
        w[t] = w[t - 16] + s0 + w[t - 7] + s1;
    }
    for (t = 0; t < 64; t++) {
        u32 s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        u32 ch = (e & f) ^ (~e & g);
        u32 t1 = h + s1 + ch + K[t] + w[t];
        u32 s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        u32 maj = (a & b) ^ (a & c) ^ (b & c);
        u32 t2 = s0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
}

void _start(void) {
    for (;;) {
        sha256_block();
    }
}

#!/bin/sh
# Regenerate the SHA-256 soak blob (P3-4). Requires arm-none-eabi-gcc.
# The checked-in blob is hash-pinned in benches/mips.rs: rebuilding with
# another toolchain changes codegen (and the benchmark), so the pin must
# move deliberately with the diff.
set -eu
D="$(dirname "$0")/../vioarm-core/benches/data"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -ffreestanding -nostdlib \
    -T "$D/sha256.ld" -o "$D/sha256.elf" "$D/sha256.c"
arm-none-eabi-objcopy -O binary "$D/sha256.elf" "$D/sha256.bin"
rm -f "$D/sha256.elf"
ls -l "$D/sha256.bin"

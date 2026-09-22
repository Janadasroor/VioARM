#!/bin/bash
# Build mbedTLS 4.2.0 AES + SHA-256 self-tests for VioARM (stm32f103).
# Same prelude the library sources use (tf_psa_crypto_common.h); the
# platform zeroize is port glue (upstream platform_util.c drags in
# entropy/time requirements unneeded for KATs).
set -e
. "$(dirname "$0")/../../common.sh"
M="$KINGSWORK/src/mbedtls"
T="$M/tf-psa-crypto"
P="$KINGSHOME/ports/mbedtls"
B="$KINGSWORK/build/mbedtls"
mkdir -p "$B"
FLAGS="$CC_BASE -I$HARNESS -I$T/include -I$M/include -I$T/drivers/builtin/include -I$T/core -I$T/drivers/builtin/src"
arm-none-eabi-gcc $FLAGS -c "$T/drivers/builtin/src/aes.c" -o "$B/aes.o"
arm-none-eabi-gcc $FLAGS -c "$T/drivers/builtin/src/sha256.c" -o "$B/sha256.o"
arm-none-eabi-gcc $FLAGS -c "$T/platform/platform.c" -o "$B/platform.o"
arm-none-eabi-gcc $FLAGS -c "$P/tls_test.c" -o "$B/tls_test.o"
kings_link f103 "$B/tls-vioarm.elf" "$B/tls_test.o" "$B/aes.o" "$B/sha256.o" "$B/platform.o"
arm-none-eabi-size "$B/tls-vioarm.elf"

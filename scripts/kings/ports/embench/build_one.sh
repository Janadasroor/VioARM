#!/bin/bash
# Build one Embench-IoT benchmark for VioARM (stm32f107).
# Usage: build_one.sh <bench>. Old-C sources need gnu99 plus explicit
# -Wno- switches (GCC 14 hard-errors implicit decls and int-conversion);
# truly-missing symbols still fail loudly at link time.
set -e
. "$(dirname "$0")/../../common.sh"
B=$1
E="$KINGSWORK/src/embench-iot"
P="$KINGSHOME/ports/embench"
SRC="$E/src/$B"
OUT="$KINGSWORK/build/embench/$B"
mkdir -p "$OUT"
FLAGS="-std=gnu99 -Wno-implicit-function-declaration -Wno-implicit-int -Wno-int-conversion -Wno-return-mismatch $CC_BASE -include support.h -I$HARNESS -I$P -I$E/support -I$SRC -DBENCH=\"$B\""
OBJS=""
for c in "$SRC"/*.c; do
  o="$OUT/$(basename "$c" .c).o"
  arm-none-eabi-gcc $FLAGS -c "$c" -o "$o"
  OBJS="$OBJS $o"
done
arm-none-eabi-gcc $FLAGS -c "$P/emb_driver.c" -o "$OUT/driver.o"
arm-none-eabi-gcc $FLAGS -c "$E/support/beebsc.c" -o "$OUT/beebsc.o"
kings_link f107 "$OUT/$B-vioarm.elf" "$OUT/driver.o" "$OUT/beebsc.o" $OBJS
echo "OK $B"

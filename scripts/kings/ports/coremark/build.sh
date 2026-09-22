#!/bin/bash
# Build CoreMark 2K performance run for VioARM (stm32f103).
# Usage: build.sh [iterations] (default 40; 200 for the >=10s valid run).
set -e
. "$(dirname "$0")/../../common.sh"
ITERS="${1:-40}"
C="$KINGSWORK/src/coremark"
P="$KINGSHOME/ports/coremark"
B="$KINGSWORK/build/coremark_$ITERS"
mkdir -p "$B"
FLAGS="$CC_BASE -I$HARNESS -I$C -I$P -DITERATIONS=$ITERS -DMAIN_HAS_NOARGC=1"
for f in core_list_join core_main core_matrix core_state core_util; do
  arm-none-eabi-gcc $FLAGS -c "$C/$f.c" -o "$B/$f.o"
done
arm-none-eabi-gcc $FLAGS -c "$P/ee_printf_port.c" -o "$B/ee_printf.o"
arm-none-eabi-gcc $FLAGS -c "$P/cm_portme.c" -o "$B/cm_portme.o"
kings_link f103 "$B/cm-vioarm.elf" "$B"/core_*.o "$B/ee_printf.o" "$B/cm_portme.o"
arm-none-eabi-size "$B/cm-vioarm.elf"

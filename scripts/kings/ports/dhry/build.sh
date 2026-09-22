#!/bin/bash
# Build Dhrystone 2.1 for VioARM (stm32f103). K&R sources need -std=gnu89;
# the harness needs default std: compile separately, then link.
set -e
. "$(dirname "$0")/../../common.sh"
D="$KINGSWORK/src/dhry"
B="$KINGSWORK/build/dhry"
mkdir -p "$B"
python3 "$KINGSHOME/ports/dhry/patch_dhry.py" "$D/dhry_1.c" "$B/dhry_1_port.c"
KFLAGS="-std=gnu89 $CC_BASE -I$HARNESS -I$D -DHZ=8000000"
arm-none-eabi-gcc $KFLAGS -c "$B/dhry_1_port.c" -o "$B/dhry_1.o"
arm-none-eabi-gcc $KFLAGS -c "$D/dhry_2.c" -o "$B/dhry_2.o"
kings_link f103 "$B/dhry-vioarm.elf" "$B/dhry_1.o" "$B/dhry_2.o"
arm-none-eabi-size "$B/dhry-vioarm.elf"

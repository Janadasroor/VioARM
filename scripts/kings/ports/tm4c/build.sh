#!/bin/bash
# Build kings firmware for Tiva C (tm4c123gh6pm or tm4c1294ncpdt).
# Usage: build.sh [e2e|dhry|cm40|cm400] [tm4c|tm4c129] (defaults: e2e tm4c).
# CoreMark valid runs need >=10 s: at 16 MHz PIOSC that is 400
# iterations (the STM32 valid run is 200 at 8 MHz — same method).
# Same method as the STM32 ports: upstream fetched at runtime by
# fetch.sh (never vendored), clean-room harness + Tiva board support.
set -e
. "$(dirname "$0")/../../common.sh"
BENCH="${1:-e2e}"
BOARD="${2:-tm4c}"
case "$BOARD" in
  tm4c|tm4c129) ;;
  *) echo "build.sh: unknown board $BOARD (tm4c|tm4c129)" >&2; exit 2 ;;
esac
P="$KINGSHOME/ports/tm4c"
B="$KINGSWORK/build/${BOARD}_$BENCH"
mkdir -p "$B"
E2E_SRC="$P/e2e.c"
[ "$BOARD" = "tm4c129" ] && E2E_SRC="$P/e2e129.c"

case "$BENCH" in
  e2e)
    arm-none-eabi-gcc $CC_BASE -I"$HARNESS" -c "$E2E_SRC" -o "$B/e2e.o"
    kings_link "$BOARD" "$B/e2e-vioarm.elf" "$B/e2e.o"
    ;;
  dhry)
    D="$KINGSWORK/src/dhry"
    python3 "$KINGSHOME/ports/dhry/patch_dhry.py" "$D/dhry_1.c" "$B/dhry_1_port.c"
    KFLAGS="-std=gnu89 $CC_BASE -I$HARNESS -I$D -DHZ=16000000"
    arm-none-eabi-gcc $KFLAGS -c "$B/dhry_1_port.c" -o "$B/dhry_1.o"
    arm-none-eabi-gcc $KFLAGS -c "$D/dhry_2.c" -o "$B/dhry_2.o"
    kings_link "$BOARD" "$B/dhry-vioarm.elf" "$B/dhry_1.o" "$B/dhry_2.o"
    ;;
  cm40|cm400)
    ITERS=40; [ "$BENCH" = "cm400" ] && ITERS=400
    C="$KINGSWORK/src/coremark"
    FLAGS="$CC_BASE -I$HARNESS -I$C -I$KINGSHOME/ports/coremark -DITERATIONS=$ITERS -DMAIN_HAS_NOARGC=1"
    for f in core_list_join core_main core_matrix core_state core_util; do
      arm-none-eabi-gcc $FLAGS -c "$C/$f.c" -o "$B/$f.o"
    done
    arm-none-eabi-gcc $FLAGS -c "$KINGSHOME/ports/coremark/ee_printf_port.c" -o "$B/ee_printf.o"
    arm-none-eabi-gcc $FLAGS -c "$P/cm_portme_tm4c.c" -o "$B/cm_portme.o"
    kings_link "$BOARD" "$B/cm-vioarm.elf" "$B"/core_*.o "$B/ee_printf.o" "$B/cm_portme.o"
    ;;
  *) echo "build.sh: unknown bench $BENCH (e2e|dhry|cm40|cm400)" >&2; exit 2 ;;
esac
arm-none-eabi-size "$B"/*-vioarm.elf

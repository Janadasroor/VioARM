#!/bin/bash
# Build the FreeRTOS tick-accuracy probe for VioARM (stm32f103).
# Two tasks + vTaskDelayUntil; LED task stamps CYCCNT every 250 ticks
# (expect exactly 2000000 cycles at 1 kHz from 8 MHz).
set -e
. "$(dirname "$0")/../../common.sh"
K="$KINGSWORK/src/freertos"
P="$KINGSHOME/ports/freertos"
B="$KINGSWORK/build/freertos"
mkdir -p "$B"
# SVC/PendSV vectors to the port handlers (startup.s pattern).
sed 's/\.word Default_Handler   \/\* SVCall \*\//.word vPortSVCHandler/; s/\.word Default_Handler   \/\* PendSV \*\//.word xPortPendSVHandler/' \
  "$HARNESS/startup.s" > "$B/startup_fr.s"
cat >> "$B/startup_fr.s" <<'EOF'

    .section .text.Tx_SvcStub, "ax", %progbits
    .thumb_func
    .globl Tx_SvcStub
    .type Tx_SvcStub, %function
Tx_SvcStub:
    b Tx_SvcStub
    .size Tx_SvcStub, .-Tx_SvcStub
EOF
FLAGS="$CC_BASE -I$P -I$K/include -I$K/portable/GCC/ARM_CM3 -I$HARNESS"
for f in tasks list queue port heap_4; do
  case $f in
    tasks|list|queue) S="$K/$f.c" ;;
    port) S="$K/portable/GCC/ARM_CM3/port.c" ;;
    heap_4) S="$K/portable/MemMang/heap_4.c" ;;
  esac
  arm-none-eabi-gcc $FLAGS -c "$S" -o "$B/$f.o"
done
arm-none-eabi-gcc $FLAGS -c "$P/fr_test.c" -o "$B/fr_test.o"
arm-none-eabi-gcc $CC_BASE -c "$B/startup_fr.s" -o "$B/startup_fr.o"
KINGS_STARTUP="$B/startup_fr.o" kings_link f103 "$B/fr-vioarm.elf" "$B/fr_test.o" \
  "$B/tasks.o" "$B/list.o" "$B/queue.o" "$B/port.o" "$B/heap_4.o"
arm-none-eabi-size "$B/fr-vioarm.elf"

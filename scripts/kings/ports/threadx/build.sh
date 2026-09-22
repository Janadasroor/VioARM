#!/bin/bash
# Build stock demo_threadx for VioARM (stm32f107, 100 Hz tick, 2020-tick
# supervisor). Upstream kernel + demo pristine; tx_user.h/tx_ll_kings.c
# are the only port glue (low-level init + tick counting in C).
set -e
. "$(dirname "$0")/../../common.sh"
X="$KINGSWORK/src/threadx"
P="$KINGSHOME/ports/threadx"
B="$KINGSWORK/build/threadx"
mkdir -p "$B/obj"
FLAGS="$CC_BASE -I$HARNESS -I$P -I$X/common/inc -I$X/ports/cortex_m3/gnu/inc"
for c in "$X"/common/src/*.c; do
  arm-none-eabi-gcc $FLAGS -c "$c" -o "$B/obj/$(basename "$c" .c).o"
done
for s in "$X"/ports/cortex_m3/gnu/src/*.S; do
  arm-none-eabi-gcc $FLAGS -c "$s" -o "$B/obj/$(basename "$s" .S).o"
done
arm-none-eabi-gcc $FLAGS -c "$X/samples/demo_threadx.c" -o "$B/obj/demo.o"
arm-none-eabi-gcc $FLAGS -c "$P/tx_ll_kings.c" -o "$B/obj/tx_ll_kings.o"
sed 's/\.word Default_Handler   \/\* SVCall \*\//.word Tx_SvcStub/; s/\.word Default_Handler   \/\* PendSV \*\//.word __tx_PendSVHandler/' \
  "$HARNESS/startup.s" > "$B/startup_tx.s"
cat >> "$B/startup_tx.s" <<'EOF'

    .section .text.Tx_SvcStub, "ax", %progbits
    .thumb_func
    .globl Tx_SvcStub
    .type Tx_SvcStub, %function
Tx_SvcStub:
    b Tx_SvcStub
    .size Tx_SvcStub, .-Tx_SvcStub
EOF
arm-none-eabi-gcc $CC_BASE -c "$B/startup_tx.s" -o "$B/obj/startup_tx.o"
# NOTE: tx_*.o excludes txe_*.o in glob (literal "tx_"), list both.
KINGS_STARTUP="$B/obj/startup_tx.o" kings_link f107 "$B/tx-vioarm.elf" "$B/obj/demo.o" \
  "$B"/obj/tx_*.o "$B"/obj/txe_*.o
arm-none-eabi-size "$B/tx-vioarm.elf"

# Pinned kings expectations. Functional gates hard-fail; cycle counts
# must match within 5% (hard fail) — tighter would false-fail on
# toolchain codegen drift; looser would miss model regressions.
# Re-pin with a full manual re-verification, never to silence CI.
DHRY_DMIPS=9.124
COREMARK_MHZ=2.062
# Tiva C (tm4c123gh6pm @ 16 MHz PIOSC) oracles, same method: identical
# codegen (-mcpu=cortex-m3) on the cycle-exact core reproduces the
# STM32 pins exactly. Valid CoreMark run is 400 iterations (>=10 s at
# 16 MHz; the STM32 valid run is 200 at 8 MHz).
DHRY_DMIPS_TM4C=9.124
COREMARK_MHZ_TM4C=2.062
# ThreadX oracle @100 Hz, 2020 ticks:
TX_TICKS=2020
TX_SENT=646372
TX_BACKLOG_MAX=101

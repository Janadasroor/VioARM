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
# Tiva C 129 (tm4c1294ncpdt @ 16 MHz PIOSC) oracles, same method:
# identical codegen on the cycle-exact core reproduces the pins exactly
# (129 CoreMark-40 measured the same 19397092 ticks as the 123 run).
DHRY_DMIPS_TM4C129=9.124
COREMARK_MHZ_TM4C129=2.062
# Second 129 board (tm4c129xnczad): same ELFs, same pins (verified
# tick-identical on e2e/dhry/cm40).
DHRY_DMIPS_TM4C129X=9.124
COREMARK_MHZ_TM4C129X=2.062
# 123 donor boards (tm4c123gh6pz, tm4c123gh6zrb): same ELFs, same pins
# (verified byte-identical logs on e2e/dhry, tick-identical cm40).
DHRY_DMIPS_TM4C123PZ=9.124
COREMARK_MHZ_TM4C123PZ=2.062
DHRY_DMIPS_TM4C123ZRB=9.124
COREMARK_MHZ_TM4C123ZRB=2.062
# ThreadX oracle @100 Hz, 2020 ticks:
TX_TICKS=2020
TX_SENT=646372
TX_BACKLOG_MAX=101

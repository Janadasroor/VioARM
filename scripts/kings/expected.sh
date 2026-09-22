# Pinned kings expectations. Functional gates hard-fail; cycle counts
# must match within 5% (hard fail) — tighter would false-fail on
# toolchain codegen drift; looser would miss model regressions.
# Re-pin with a full manual re-verification, never to silence CI.
DHRY_DMIPS=9.124
COREMARK_MHZ=2.062
# ThreadX oracle @100 Hz, 2020 ticks:
TX_TICKS=2020
TX_SENT=646372
TX_BACKLOG_MAX=101

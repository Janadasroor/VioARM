#!/bin/bash
# Kings end-to-end re-verification: build all firmware, run on release
# VioARM, check functional gates (hard fail) + cycle pins (5% hard fail).
# Usage: run_all.sh. Expects: fetch.sh already run, release vioarm on PATH
# (or $VIOARM_BIN), arm-none-eabi-gcc + host g++ installed. Slow (~2h).
set -e
KINGS="$(cd "$(dirname "$0")" && pwd)"
. "$KINGS/common.sh"
. "$KINGS/expected.sh"
VIO="${VIOARM_BIN:-}"
if [ -z "$VIO" ]; then
  VIO="$(command -v vioarm || command -v vioarm-cli || echo MISSING)"
fi
[ -x "$VIO" ] || { echo "no vioarm binary (set VIOARM_BIN)" >&2; exit 2; }
FAIL=0

pass() { echo "KINGS-PASS $1 $2"; }
fail() { echo "KINGS-FAIL $1 $2"; FAIL=1; }

# within <pct>% of <expected>? (integer math)
within() {
  python3 -c "import sys; e=float(sys.argv[1]); g=float(sys.argv[2]); p=float(sys.argv[3]); sys.exit(0 if abs(g-e)/e*100 <= p else 1)" "$1" "$2" "$3"
}

run_uart() { # <board> <elf> <budget> <outfile>
  "$VIO" run --board "$1" --budget "$3" "$2" > "$4" 2>&1 || true
  kings_uart "$4" > "$4.uart" 2>/dev/null || true
}

# Build one firmware; a failure is reported and recorded but never aborts
# the run, so one broken port cannot hide every other result.
build() { # <label> <cmd...>
  local label="$1"; shift
  if "$@" > "$KINGSWORK/build_$label.log" 2>&1; then
    return 0
  fi
  echo "KINGS-FAIL $label build"
  FAIL=1
  return 0
}

echo "== build =="
build dhry "$KINGS/ports/dhry/build.sh"
build coremark_40 "$KINGS/ports/coremark/build.sh" 40
build coremark_200 "$KINGS/ports/coremark/build.sh" 200
build tm4c_e2e "$KINGS/ports/tm4c/build.sh" e2e
build tm4c_dhry "$KINGS/ports/tm4c/build.sh" dhry
build tm4c_cm40 "$KINGS/ports/tm4c/build.sh" cm40
build tm4c_cm400 "$KINGS/ports/tm4c/build.sh" cm400
build tm4c129_e2e "$KINGS/ports/tm4c/build.sh" e2e tm4c129
build tm4c129_dhry "$KINGS/ports/tm4c/build.sh" dhry tm4c129
build tm4c129_cm40 "$KINGS/ports/tm4c/build.sh" cm40 tm4c129
build tm4c129_cm400 "$KINGS/ports/tm4c/build.sh" cm400 tm4c129
build embench "$KINGS/ports/embench/build_all.sh"
build freertos "$KINGS/ports/freertos/build.sh"
build mbedtls "$KINGS/ports/mbedtls/build.sh"
build threadx "$KINGS/ports/threadx/build.sh"
build tflm "$KINGS/ports/tflm/build.sh"

# Gate phase: every check reports explicitly via fail(); a miss (e.g. a
# firmware that did not build) is a recorded FAIL, never a fatal abort.
set +e

echo "== dhrystone =="
run_uart stm32f103 "$KINGSWORK/build/dhry/dhry-vioarm.elf" 200000000 "$KINGSWORK/dhry.log"
U="$KINGSWORK/dhry.log.uart"
grep -q DHRY_DONE "$U" || fail dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS" "$DMIPS" 5; then pass dhry "dmips=$DMIPS"; else fail dhry "dmips=$DMIPS want $DHRY_DMIPS"; fi

echo "== coremark =="
run_uart stm32f103 "$KINGSWORK/build/coremark_40/cm-vioarm.elf" 50000000 "$KINGSWORK/cm40.log"
U="$KINGSWORK/cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
# 40 iterations in (ticks/8e6) s, per MHz: 40*8e6/ticks/8 = 40e6/ticks.
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ" "$MHZ" 5; then pass coremark40 "mhz=$MHZ crc-ok"; else fail coremark40 "mhz=$MHZ"; fi
run_uart stm32f103 "$KINGSWORK/build/coremark_200/cm-vioarm.elf" 150000000 "$KINGSWORK/cm200.log"
grep -q "Correct operation validated" "$KINGSWORK/cm200.log.uart" && pass coremark200 valid || fail coremark200 "not validated"

echo "== embench =="
while read -r name cycles; do
  [ -n "$name" ] || continue
  case "$name" in \#*) continue;; esac
  run_uart stm32f107 "$KINGSWORK/build/embench/$name/$name-vioarm.elf" 40000000 "$KINGSWORK/emb_$name.log"
  U="$KINGSWORK/emb_$name.log.uart"
  C=$(grep -oE "correct=[0-9]+" "$U" | grep -oE "[0-9]+$" || echo MISSING)
  [ "$C" = "1" ] || { fail "embench-$name" "correct=$C"; continue; }
  G=$(grep -oE "cycles=[0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
  if within "$cycles" "$G" 5; then pass "embench-$name" "cycles=$G"; else fail "embench-$name" "cycles=$G want $cycles"; fi
done < "$KINGS/expected_embench.tsv"

echo "== freertos =="
run_uart stm32f103 "$KINGSWORK/build/freertos/fr-vioarm.elf" 60000000 "$KINGSWORK/fr.log"
U="$KINGSWORK/fr.log.uart"
grep -q "PERIODS: 2000000 2000000 2000000 2000000 2000000 2000000 2000000 2000000 2000000 PASS" "$U" \
  && pass freertos exact-periods || fail freertos "periods"

echo "== mbedtls =="
run_uart stm32f103 "$KINGSWORK/build/mbedtls/tls-vioarm.elf" 300000000 "$KINGSWORK/tls.log"
U="$KINGSWORK/tls.log.uart"
grep -q "OVERALL: PASSED" "$U" && pass mbedtls overall || fail mbedtls "no PASS"

echo "== threadx =="
run_uart stm32f107 "$KINGSWORK/build/threadx/tx-vioarm.elf" 300000000 "$KINGSWORK/tx.log"
U="$KINGSWORK/tx.log.uart"
grep -q "TX_DONE" "$U" || fail threadx "no DONE"
T=$(grep -oE "TICKS=[0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
[ "$T" = "$TX_TICKS" ] || fail threadx "ticks=$T"
S=$(grep -oE "sent=[0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
if within "$TX_SENT" "$S" 5; then pass threadx "sent=$S"; else fail threadx "sent=$S want $TX_SENT"; fi
B=$(grep -oE "backlog=[0-9]+" "$U" | grep -oE "[0-9]+$" || echo 999)
[ "$B" -le "$TX_BACKLOG_MAX" ] && pass threadx "backlog=$B" || fail threadx "backlog=$B"
grep -q " PASS" "$U" || fail threadx "no PASS"

echo "== tflm =="
run_uart stm32f107 "$KINGSWORK/build/tflm/tflm-vioarm.elf" 20000000 "$KINGSWORK/tflm.log"
U="$KINGSWORK/tflm.log.uart"
grep -q "TFLM_ALL_PASS" "$U" || fail tflm "no ALL_PASS"
# x86 reference cross-check (bit-exact predictions).
if g++ -O0 -std=c++17 -fno-exceptions -fno-rtti -w -DHOST_REF -ffunction-sections -fdata-sections \
  -I"$KINGSWORK/src/tflite-micro" -I"$KINGSHOME/ports/tflm" -I"$HARNESS" \
  -I"$KINGSWORK/build/tflm/genpath" \
  -I"$KINGSWORK/src/flatbuffers/include" -I"$KINGSWORK/src/gemmlowp" -I"$KINGSWORK/src/ruy" \
  "$KINGSHOME/ports/tflm/tflm_test.cc" "$KINGSHOME/ports/tflm/host_board.cc" \
  "$KINGSWORK/build/tflm/gen"/hello_world_*_model_data.cc \
  "$KINGSWORK"/src/tflite-micro/tensorflow/lite/micro/cortex_m_generic/debug_log.cc \
  $(for c in "$KINGSWORK"/src/tflite-micro/tensorflow/lite/micro/cortex_m_generic/debug_log.cc \
    "$KINGSWORK"/src/tflite-micro/tensorflow/lite/micro/*.cc; do
    case $(basename "$c") in *test*|*debug_log.cc|*test_helper*|*mock*) continue;; esac; echo "$c"; done) \
  $(for d in kernels memory_planner arena_allocator tflite_bridge; do
    for c in "$KINGSWORK"/src/tflite-micro/tensorflow/lite/micro/$d/*.cc; do
    case $(basename "$c") in *test*) continue;; esac; echo "$c"; done; done) \
  "$KINGSWORK"/src/tflite-micro/tensorflow/lite/core/c/common.cc \
  "$KINGSWORK"/src/tflite-micro/tensorflow/lite/core/api/flatbuffer_conversions.cc \
  "$KINGSWORK"/src/tflite-micro/tensorflow/compiler/mlir/lite/core/api/error_reporter.cc \
  "$KINGSWORK"/src/tflite-micro/tensorflow/compiler/mlir/lite/schema/schema_utils.cc \
  "$KINGSWORK"/src/tflite-micro/tensorflow/lite/kernels/kernel_util.cc \
  $(for f in common portable_tensor_utils quantization_util runtime_shape tensor_ctypes tensor_utils; do
    echo "$KINGSWORK/src/tflite-micro/tensorflow/lite/kernels/internal/$f.cc"; done) \
  -o "$KINGSWORK/tflm-host-ref" -Wl,--gc-sections -lm; then
  stdbuf -o0 "$KINGSWORK/tflm-host-ref" > "$KINGSWORK/tflm-host.log" 2>&1 || true
  if diff "$KINGSWORK/tflm-host.log" <(grep -v "^$" "$U") >/dev/null; then
    pass tflm "bit-exact-vs-x86"
  else
    fail tflm "host-diff"
  fi
else
  fail tflm "host-ref-build"
fi

echo "== tiva =="
run_uart tm4c123gh6pm "$KINGSWORK/build/tm4c_e2e/e2e-vioarm.elf" 500000 "$KINGSWORK/tm4c_e2e.log"
U="$KINGSWORK/tm4c_e2e.log.uart"
for m in E2E_UART_OK E2E_GPIO_OK E2E_TIMER_OK E2E_DONE; do
  grep -q "$m" "$U" || fail tm4c-e2e "missing $m"
done
grep -q E2E_DONE "$U" && pass tm4c-e2e all-markers
run_uart tm4c123gh6pm "$KINGSWORK/build/tm4c_dhry/dhry-vioarm.elf" 500000000 "$KINGSWORK/tm4c_dhry.log"
U="$KINGSWORK/tm4c_dhry.log.uart"
grep -q DHRY_DONE "$U" || fail tm4c-dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail tm4c-dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS_TM4C" "$DMIPS" 5; then pass tm4c-dhry "dmips=$DMIPS"; else fail tm4c-dhry "dmips=$DMIPS want $DHRY_DMIPS_TM4C"; fi
run_uart tm4c123gh6pm "$KINGSWORK/build/tm4c_cm40/cm-vioarm.elf" 50000000 "$KINGSWORK/tm4c_cm40.log"
U="$KINGSWORK/tm4c_cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail tm4c-coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ_TM4C" "$MHZ" 5; then pass tm4c-coremark40 "mhz=$MHZ crc-ok"; else fail tm4c-coremark40 "mhz=$MHZ"; fi
run_uart tm4c123gh6pm "$KINGSWORK/build/tm4c_cm400/cm-vioarm.elf" 600000000 "$KINGSWORK/tm4c_cm400.log"
grep -q "Correct operation validated" "$KINGSWORK/tm4c_cm400.log.uart" && pass tm4c-coremark400 valid || fail tm4c-coremark400 "not validated"

echo "== tiva129 =="
run_uart tm4c1294ncpdt "$KINGSWORK/build/tm4c129_e2e/e2e-vioarm.elf" 500000 "$KINGSWORK/tm4c129_e2e.log"
U="$KINGSWORK/tm4c129_e2e.log.uart"
for m in E2E_UART_OK E2E_GPIO_OK E2E_TIMER_OK E2E_DONE; do
  grep -q "$m" "$U" || fail tm4c129-e2e "missing $m"
done
grep -q E2E_DONE "$U" && pass tm4c129-e2e all-markers
run_uart tm4c1294ncpdt "$KINGSWORK/build/tm4c129_dhry/dhry-vioarm.elf" 500000000 "$KINGSWORK/tm4c129_dhry.log"
U="$KINGSWORK/tm4c129_dhry.log.uart"
grep -q DHRY_DONE "$U" || fail tm4c129-dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail tm4c129-dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS_TM4C129" "$DMIPS" 5; then pass tm4c129-dhry "dmips=$DMIPS"; else fail tm4c129-dhry "dmips=$DMIPS want $DHRY_DMIPS_TM4C129"; fi
run_uart tm4c1294ncpdt "$KINGSWORK/build/tm4c129_cm40/cm-vioarm.elf" 50000000 "$KINGSWORK/tm4c129_cm40.log"
U="$KINGSWORK/tm4c129_cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail tm4c129-coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ_TM4C129" "$MHZ" 5; then pass tm4c129-coremark40 "mhz=$MHZ crc-ok"; else fail tm4c129-coremark40 "mhz=$MHZ"; fi
run_uart tm4c1294ncpdt "$KINGSWORK/build/tm4c129_cm400/cm-vioarm.elf" 600000000 "$KINGSWORK/tm4c129_cm400.log"
grep -q "Correct operation validated" "$KINGSWORK/tm4c129_cm400.log.uart" && pass tm4c129-coremark400 valid || fail tm4c129-coremark400 "not validated"

echo "== tiva129x =="
run_uart tm4c129xnczad "$KINGSWORK/build/tm4c129_e2e/e2e-vioarm.elf" 500000 "$KINGSWORK/tm4c129x_e2e.log"
U="$KINGSWORK/tm4c129x_e2e.log.uart"
for m in E2E_UART_OK E2E_GPIO_OK E2E_TIMER_OK E2E_DONE; do
  grep -q "$m" "$U" || fail tm4c129x-e2e "missing $m"
done
grep -q E2E_DONE "$U" && pass tm4c129x-e2e all-markers
run_uart tm4c129xnczad "$KINGSWORK/build/tm4c129_dhry/dhry-vioarm.elf" 500000000 "$KINGSWORK/tm4c129x_dhry.log"
U="$KINGSWORK/tm4c129x_dhry.log.uart"
grep -q DHRY_DONE "$U" || fail tm4c129x-dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail tm4c129x-dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS_TM4C129X" "$DMIPS" 5; then pass tm4c129x-dhry "dmips=$DMIPS"; else fail tm4c129x-dhry "dmips=$DMIPS want $DHRY_DMIPS_TM4C129X"; fi
run_uart tm4c129xnczad "$KINGSWORK/build/tm4c129_cm40/cm-vioarm.elf" 50000000 "$KINGSWORK/tm4c129x_cm40.log"
U="$KINGSWORK/tm4c129x_cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail tm4c129x-coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ_TM4C129X" "$MHZ" 5; then pass tm4c129x-coremark40 "mhz=$MHZ crc-ok"; else fail tm4c129x-coremark40 "mhz=$MHZ"; fi
run_uart tm4c129xnczad "$KINGSWORK/build/tm4c129_cm400/cm-vioarm.elf" 600000000 "$KINGSWORK/tm4c129x_cm400.log"
grep -q "Correct operation validated" "$KINGSWORK/tm4c129x_cm400.log.uart" && pass tm4c129x-coremark400 valid || fail tm4c129x-coremark400 "not validated"

echo "== tiva123pz =="
run_uart tm4c123gh6pz "$KINGSWORK/build/tm4c_e2e/e2e-vioarm.elf" 500000 "$KINGSWORK/tm4c123pz_e2e.log"
U="$KINGSWORK/tm4c123pz_e2e.log.uart"
for m in E2E_UART_OK E2E_GPIO_OK E2E_TIMER_OK E2E_DONE; do
  grep -q "$m" "$U" || fail tm4c123pz-e2e "missing $m"
done
grep -q E2E_DONE "$U" && pass tm4c123pz-e2e all-markers
run_uart tm4c123gh6pz "$KINGSWORK/build/tm4c_dhry/dhry-vioarm.elf" 500000000 "$KINGSWORK/tm4c123pz_dhry.log"
U="$KINGSWORK/tm4c123pz_dhry.log.uart"
grep -q DHRY_DONE "$U" || fail tm4c123pz-dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail tm4c123pz-dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS_TM4C123PZ" "$DMIPS" 5; then pass tm4c123pz-dhry "dmips=$DMIPS"; else fail tm4c123pz-dhry "dmips=$DMIPS want $DHRY_DMIPS_TM4C123PZ"; fi
run_uart tm4c123gh6pz "$KINGSWORK/build/tm4c_cm40/cm-vioarm.elf" 50000000 "$KINGSWORK/tm4c123pz_cm40.log"
U="$KINGSWORK/tm4c123pz_cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail tm4c123pz-coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ_TM4C123PZ" "$MHZ" 5; then pass tm4c123pz-coremark40 "mhz=$MHZ crc-ok"; else fail tm4c123pz-coremark40 "mhz=$MHZ"; fi
run_uart tm4c123gh6pz "$KINGSWORK/build/tm4c_cm400/cm-vioarm.elf" 600000000 "$KINGSWORK/tm4c123pz_cm400.log"
grep -q "Correct operation validated" "$KINGSWORK/tm4c123pz_cm400.log.uart" && pass tm4c123pz-coremark400 valid || fail tm4c123pz-coremark400 "not validated"

echo "== tiva123zrb =="
run_uart tm4c123gh6zrb "$KINGSWORK/build/tm4c_e2e/e2e-vioarm.elf" 500000 "$KINGSWORK/tm4c123zrb_e2e.log"
U="$KINGSWORK/tm4c123zrb_e2e.log.uart"
for m in E2E_UART_OK E2E_GPIO_OK E2E_TIMER_OK E2E_DONE; do
  grep -q "$m" "$U" || fail tm4c123zrb-e2e "missing $m"
done
grep -q E2E_DONE "$U" && pass tm4c123zrb-e2e all-markers
run_uart tm4c123gh6zrb "$KINGSWORK/build/tm4c_dhry/dhry-vioarm.elf" 500000000 "$KINGSWORK/tm4c123zrb_dhry.log"
U="$KINGSWORK/tm4c123zrb_dhry.log.uart"
grep -q DHRY_DONE "$U" || fail tm4c123zrb-dhry "no DONE"
[ "$(grep -c "should be:" "$U")" -ge 20 ] || fail tm4c123zrb-dhry "self-checks missing"
DMIPS=$(grep -oE "DMIPS \(dps/1757, x1000\): +[0-9]+\.[0-9]+" "$U" | grep -oE "[0-9]+\.[0-9]+$")
if within "$DHRY_DMIPS_TM4C123ZRB" "$DMIPS" 5; then pass tm4c123zrb-dhry "dmips=$DMIPS"; else fail tm4c123zrb-dhry "dmips=$DMIPS want $DHRY_DMIPS_TM4C123ZRB"; fi
run_uart tm4c123gh6zrb "$KINGSWORK/build/tm4c_cm40/cm-vioarm.elf" 50000000 "$KINGSWORK/tm4c123zrb_cm40.log"
U="$KINGSWORK/tm4c123zrb_cm40.log.uart"
for c in "crclist *: 0xe714" "crcmatrix *: 0x1fd7" "crcstate *: 0x8e3a"; do
  grep -qE "$c" "$U" || fail tm4c123zrb-coremark40 "missing $c"
done
MHZ=$(grep -oE "Total ticks *: [0-9]+" "$U" | grep -oE "[0-9]+$" || echo 0)
MHZ=$(python3 -c "print(40000000.0/$MHZ if $MHZ else 0)")
if within "$COREMARK_MHZ_TM4C123ZRB" "$MHZ" 5; then pass tm4c123zrb-coremark40 "mhz=$MHZ crc-ok"; else fail tm4c123zrb-coremark40 "mhz=$MHZ"; fi
run_uart tm4c123gh6zrb "$KINGSWORK/build/tm4c_cm400/cm-vioarm.elf" 600000000 "$KINGSWORK/tm4c123zrb_cm400.log"
grep -q "Correct operation validated" "$KINGSWORK/tm4c123zrb_cm400.log.uart" && pass tm4c123zrb-coremark400 valid || fail tm4c123zrb-coremark400 "not validated"

echo "== done: FAIL=$FAIL =="
exit $FAIL

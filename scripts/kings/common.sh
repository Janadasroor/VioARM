#!/bin/bash
# Shared kings environment. Source, don't execute.
#   KINGSWORK: scratch dir for upstream checkouts + build products
#              (default: $RUNNER_TEMP/kings locally /tmp/kings).
#   KINGSHOME: this repo's scripts/kings (harness + ports), derived.
: "${KINGSWORK:=/tmp/kings}"
export KINGSWORK
KINGSHOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export KINGSHOME
H="$KINGSHOME/harness"
export HARNESS="$H"

CC_BASE="-mcpu=cortex-m3 -mthumb -O2 --specs=nano.specs -ffunction-sections -fdata-sections -w"
export CC_BASE

# kings_link <board:f103|f107|tm4c> <out.elf> <objects...>
# Links with the harness startup + board support. VioARM boots STM32
# boards from the 0x0 flash alias, hence mem_f10*.ld at ORIGIN 0x0;
# the Tiva boots natively at 0x0 (mem_tm4c.ld). KINGS_STARTUP overrides
# the startup object (FreeRTOS/ThreadX bring their own vector tables);
# -lm is always linked (wikisort needs sqrt).
kings_link() {
  local board=$1 out=$2
  shift 2
  local ld board_src board_obj
  case $board in
    f103) ld="$H/mem_f103.ld"; board_src="$H/board.c" ;;
    f107) ld="$H/mem_f107.ld"; board_src="$H/board.c" ;;
    tm4c) ld="$H/mem_tm4c.ld"; board_src="$H/board_tm4c.c" ;;
    *) echo "kings_link: unknown board $board" >&2; return 1 ;;
  esac
  board_obj="$KINGSWORK/harness/$(basename "$board_src" .c).o"
  startup="${KINGS_STARTUP:-$KINGSWORK/harness/startup.o}"
  [ -f "$startup" ] || \
    arm-none-eabi-gcc $CC_BASE -c "$H/startup.s" -o "$startup"
  [ -f "$board_obj" ] || \
    arm-none-eabi-gcc $CC_BASE -I"$H" -c "$board_src" -o "$board_obj"
  arm-none-eabi-gcc $CC_BASE -T "$ld" "$startup" \
    "$board_obj" "$@" -o "$out" -Wl,--gc-sections -lm
}

# kings_run <board> <elf> <budget> — vioarm release binary expected on PATH
# (CI builds it) or at $VIOARM_BIN. Prints the run's stdout.
kings_run() {
  local board=$1 elf=$2 budget=$3
  local vio="${VIOARM_BIN:-vioarm}"
  "$vio" run --board "$board" --budget "$budget" "$elf"
}

# kings_uart <run-log> — drain the trailing `uart:` section to stdout.
kings_uart() {
  awk '/^uart:/{f=1; sub(/^uart: /,""); print; next} f' "$1" | tr -d '\r'
}

mkdir -p "$KINGSWORK/harness"

#!/bin/bash
# Build TFLM hello_world for VioARM (stm32f107).
# C++17, no exceptions/RTTI, nano specs; the kernel subset is the
# target-independent core (generic kernels, no xtensa/cmsis_nn/arc).
# Needs: flatbuffers @25.9.23 (schema-asserted), gemmlowp, ruy (headers).
set -e
. "$(dirname "$0")/../../common.sh"
R="$KINGSWORK/src/tflite-micro"
T="$R/tensorflow/lite"
P="$KINGSHOME/ports/tflm"
B="$KINGSWORK/build/tflm"
FB="$KINGSWORK/src/flatbuffers/include"
GEMM="$KINGSWORK/src/gemmlowp"
RUY="$KINGSWORK/src/ruy"
mkdir -p "$B/obj" "$B/gen" "$B/genpath/tensorflow/lite/micro/examples/hello_world/models"
python3 "$P/gen_models.py" "$R/tensorflow/lite/micro/examples/hello_world/models" "$B/gen"
cp "$B"/gen/hello_world_*_model_data.h \
  "$B/genpath/tensorflow/lite/micro/examples/hello_world/models/"
CXX="-mcpu=cortex-m3 -mthumb -O2 -std=c++17 -fno-exceptions -fno-rtti --specs=nano.specs -ffunction-sections -fdata-sections -w"
INC="-I$HARNESS -I$R -I$P -I$B/genpath -I$FB -I$GEMM -I$RUY -I$KINGSWORK/src/kissfft"
SRCS="$P/tflm_test.cc $B/gen/hello_world_float_model_data.cc $B/gen/hello_world_int8_model_data.cc"
SRCS="$SRCS $T/micro/cortex_m_generic/debug_log.cc"
for c in "$T"/micro/*.cc; do
  # NOTE: match the basename only — $KINGSWORK itself may contain "test".
  case $(basename "$c") in *test*|*debug_log.cc|*test_helper*|*mock*) continue ;; esac
  SRCS="$SRCS $c"
done
for d in kernels memory_planner arena_allocator tflite_bridge; do
  for c in "$T"/micro/$d/*.cc; do
    case $(basename "$c") in *test*) continue ;; esac
    SRCS="$SRCS $c"
  done
done
# kissfft backs the RFFT/FFT kernels (upstream pins it the same way).
SRCS="$SRCS $KINGSWORK/src/kissfft/kiss_fft.c"
# tflite-micro@71fba59 still vendors the TFLite library itself under
# tensorflow/lite (upstream stopped later, which is why `main` has only
# micro/). Upstream's TENSORFLOW_ROOT is this same tree, so these are the
# library sources its micro kernels link against -- TfLiteTypeGetName /
# ConvertTensorType (core/c/common.cc), ParseFullyConnected
# (core/api/flatbuffer_conversions.cc), GetBuiltinCode (kernels/kernel_util.cc).
SRCS="$SRCS $T/core/c/common.cc $T/core/api/flatbuffer_conversions.cc"
SRCS="$SRCS $T/core/api/error_reporter.cc"
SRCS="$SRCS $T/schema/schema_utils.cc $T/kernels/kernel_util.cc"
for f in common portable_tensor_utils quantization_util runtime_shape tensor_ctypes tensor_utils; do
  SRCS="$SRCS $T/kernels/internal/$f.cc"
done
n=0
for c in $SRCS; do
  # shellcheck disable=SC2086
  arm-none-eabi-g++ $CXX $INC -c "$c" -o "$B/obj/n$n.o" || { echo "FAIL_$c"; exit 1; }
  n=$((n + 1))
done
echo "compiled $n objects"
BASE="-mcpu=cortex-m3 -mthumb -O2 --specs=nano.specs -ffunction-sections -fdata-sections -w"
# The TFLM phase builds no STM32 port, so nothing else has compiled the
# harness objects yet (kings_link's lazy build is not on this path).
arm-none-eabi-gcc $BASE -c "$HARNESS/startup.s" -o "$KINGSWORK/harness/startup.o"
arm-none-eabi-gcc $BASE -I"$HARNESS" -c "$HARNESS/board.c" -o "$KINGSWORK/harness/board.o"
arm-none-eabi-gcc $BASE -T "$HARNESS/mem_f107.ld" "$KINGSWORK/harness/startup.o" \
  "$KINGSWORK/harness/board.o" "$B"/obj/*.o -o "$B/tflm-vioarm.elf" \
  -Wl,--gc-sections -lstdc++ -lm
arm-none-eabi-size "$B/tflm-vioarm.elf"

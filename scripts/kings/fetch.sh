#!/bin/bash
# Fetch + pin all kings upstream sources into $KINGSWORK/src.
# Rerunning is idempotent (existing checkouts are left alone).
# Pins live here; bump with a full re-verification, never silently.
#
# KINGS_PHASES mirrors run_all.sh: each CI job fetches only the sources
# its phase builds, so the parallel jobs do not clone the whole world.
set -e
. "$(dirname "$0")/common.sh"
SRC="$KINGSWORK/src"
mkdir -p "$SRC"

PHASES="${KINGS_PHASES:-core,tflm,tiva1,tiva2}"
has_phase() { case ",$PHASES," in *",$1,"*) return 0;; *) return 1;; esac; }

if has_phase core || has_phase tiva1 || has_phase tiva2; then
  # Dhrystone 2.1: netlib dhry-c shar archive (unversioned; we hash the
  # extracted files after download — see ports/dhry/patch_dhry.py).
  # Shared: the STM32 core phase and the Tiva phases both build dhry +
  # coremark firmware from these same sources.
  if [ ! -f "$SRC/dhry/dhry_1.c" ]; then
    mkdir -p "$SRC/dhry"
    curl -sL https://www.netlib.org/benchmark/dhry-c -o "$SRC/dhry/shar.html"
    python3 "$KINGSHOME/ports/dhry/extract_shar.py" "$SRC/dhry/shar.html" "$SRC/dhry"
  fi

  # CoreMark (rolling HEAD; CRCs are version-stable for the 2K config).
  [ -d "$SRC/coremark" ] || \
    git clone --depth 1 https://github.com/eembc/coremark.git "$SRC/coremark"
fi

if has_phase core; then
  # Embench-IoT @09c2ed8 (2026-02-13; cycle tables are pinned to this snap).
  [ -d "$SRC/embench-iot" ] || \
    git clone --depth 1 https://github.com/embench/embench-iot.git "$SRC/embench-iot"
  git -C "$SRC/embench-iot" fetch --depth 1 origin 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || true
  git -C "$SRC/embench-iot" checkout -q 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || \
    echo "embench pin 09c2ed8 missing upstream; using clone HEAD (tables may drift!)"

  # FreeRTOS-Kernel (rolling HEAD; API-stable for this test).
  [ -d "$SRC/freertos" ] || \
    git clone --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git "$SRC/freertos"

  # mbedTLS v4.2.0 + TF-PSA-Crypto submodule.
  if [ ! -d "$SRC/mbedtls" ]; then
    git clone --depth 1 --branch v4.2.0 https://github.com/Mbed-TLS/mbedtls.git "$SRC/mbedtls"
    git -C "$SRC/mbedtls" submodule update --init --depth 1 tf-psa-crypto
  fi

  # Eclipse ThreadX (rolling HEAD; demo_threadx is API-stable).
  [ -d "$SRC/threadx" ] || \
    git clone --depth 1 https://github.com/eclipse-threadx/threadx.git "$SRC/threadx"
fi

if has_phase tflm; then
  # TFLM pins. The port builds tflite-micro's `tensorflow/lite/micro`
  # together with the shared TFLite core from TensorFlow, so BOTH sides
  # must come from one mutually consistent era -- previously both were
  # `--depth 1` of moving branches, which silently skews the APIs
  # (tflite::micro::TfLiteTensor vs the C struct, removed tflite_bridge,
  # kernels that need a newer flatbuffers).
  #   tflite-micro @71fba59  (2024-06): `input()` returns the C TfLiteTensor
  #     struct the port expects, and still ships micro/tflite_bridge.
  #   tensorflow v2.16.1: matching lite/{core,kernels,schema} sources.
  #   flatbuffers v23.5.26: the version TFLM@71fba59 pins in
  #     tensorflow/lite/micro/tools/make/flatbuffers_download.sh.
  TFLM_SHA=71fba59b5bebfd64b48e167bfcb669fff568ec74
  TF_TAG=v2.16.1
  FLATBUFFERS_TAG=v23.5.26

  [ -d "$SRC/tflite-micro/.git" ] || \
    git clone https://github.com/tensorflow/tflite-micro.git "$SRC/tflite-micro"
  git -C "$SRC/tflite-micro" fetch -q --depth 1 origin "$TFLM_SHA" 2>/dev/null || true
  git -C "$SRC/tflite-micro" checkout -q "$TFLM_SHA"

  # Sparse-fetch only the shared TFLite subtrees the port compiles, then
  # overlay them into the tflite-micro tree it builds from.
  if [ ! -d "$SRC/tensorflow/tensorflow/lite/core" ]; then
    git clone --filter=blob:none --sparse --branch "$TF_TAG" \
      https://github.com/tensorflow/tensorflow.git "$SRC/tensorflow"
    git -C "$SRC/tensorflow" sparse-checkout set \
      tensorflow/lite/core tensorflow/lite/kernels tensorflow/lite/schema
  else
    git -C "$SRC/tensorflow" fetch -q --depth 1 origin "refs/tags/$TF_TAG:refs/tags/$TF_TAG" 2>/dev/null || true
    git -C "$SRC/tensorflow" checkout -q "$TF_TAG"
  fi
  TFLM="$SRC/tflite-micro"
  mkdir -p "$TFLM/tensorflow/lite"
  rm -rf "$TFLM/tensorflow/lite/core" "$TFLM/tensorflow/lite/kernels" \
    "$TFLM/tensorflow/lite/schema"
  for d in core kernels schema; do
    ln -sfn "$SRC/tensorflow/tensorflow/lite/$d" "$TFLM/tensorflow/lite/$d"
  done

  if [ ! -d "$SRC/flatbuffers/include/flatbuffers" ]; then
    git clone --depth 1 --branch "$FLATBUFFERS_TAG" https://github.com/google/flatbuffers.git "$SRC/flatbuffers"
  fi
  [ -d "$SRC/gemmlowp" ] || \
    git clone --depth 1 https://github.com/google/gemmlowp.git "$SRC/gemmlowp"
  [ -d "$SRC/ruy" ] || \
    git clone --depth 1 https://github.com/google/ruy.git "$SRC/ruy"
fi

echo "fetch done in $SRC"
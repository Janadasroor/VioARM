#!/bin/bash
# Fetch + pin all kings upstream sources into $KINGSWORK/src.
# Rerunning is idempotent (existing checkouts are left alone).
# Pins live here; bump with a full re-verification, never silently.
set -e
. "$(dirname "$0")/common.sh"
SRC="$KINGSWORK/src"
mkdir -p "$SRC"

# Dhrystone 2.1: netlib dhry-c shar archive (unversioned; we hash the
# extracted files after download — see ports/dhry/patch_dhry.py).
if [ ! -f "$SRC/dhry/dhry_1.c" ]; then
  mkdir -p "$SRC/dhry"
  curl -sL https://www.netlib.org/benchmark/dhry-c -o "$SRC/dhry/shar.html"
  python3 "$KINGSHOME/ports/dhry/extract_shar.py" "$SRC/dhry/shar.html" "$SRC/dhry"
fi

# Embench-IoT @09c2ed8 (2026-02-13; cycle tables are pinned to this snap).
[ -d "$SRC/embench-iot" ] || \
  git clone --depth 1 https://github.com/embench/embench-iot.git "$SRC/embench-iot"
git -C "$SRC/embench-iot" fetch --depth 1 origin 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || true
git -C "$SRC/embench-iot" checkout -q 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || \
  echo "embench pin 09c2ed8 missing upstream; using clone HEAD (tables may drift!)"

# CoreMark (rolling HEAD; CRCs are version-stable for the 2K config).
[ -d "$SRC/coremark" ] || \
  git clone --depth 1 https://github.com/eembc/coremark.git "$SRC/coremark"

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

# TFLM + exact dependency pins (flatbuffers is version-asserted by the
# schema header: 25.9.23; gemmlowp/ruy are header-only users here).
[ -d "$SRC/tflite-micro" ] || \
  git clone --depth 1 https://github.com/tensorflow/tflite-micro.git "$SRC/tflite-micro"
if [ ! -d "$SRC/flatbuffers/include/flatbuffers" ]; then
  git clone --depth 1 https://github.com/google/flatbuffers.git "$SRC/flatbuffers"
  git -C "$SRC/flatbuffers" fetch --depth 1 origin tag v25.9.23
  git -C "$SRC/flatbuffers" checkout -q v25.9.23
fi
[ -d "$SRC/gemmlowp" ] || \
  git clone --depth 1 https://github.com/google/gemmlowp.git "$SRC/gemmlowp"
[ -d "$SRC/ruy" ] || \
  git clone --depth 1 https://github.com/google/ruy.git "$SRC/ruy"

echo "fetch done in $SRC"

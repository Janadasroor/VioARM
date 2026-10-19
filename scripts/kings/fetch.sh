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

# GitHub fetches here fail intermittently (early EOF / sideband
# disconnect), and a clone has to be redone from scratch when it does, so
# every remote op retries a few times before giving up.
git_try() {
  local n=0
  until "$@"; do
    n=$((n + 1))
    [ "$n" -ge 4 ] && { echo "git failed after $n attempts: $*" >&2; return 1; }
    echo "  git attempt $n failed, retrying..."
    sleep $((n * 5))
  done
}

clone_try() { # <url> <dir>
  [ -d "$2/.git" ] && return 0
  git_try git clone --depth 1 "$1" "$2"
}

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
  clone_try https://github.com/eembc/coremark.git "$SRC/coremark"
fi

if has_phase core; then
  # Embench-IoT @09c2ed8 (2026-02-13; cycle tables are pinned to this snap).
  clone_try https://github.com/embench/embench-iot.git "$SRC/embench-iot"
  git -C "$SRC/embench-iot" fetch --depth 1 origin 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || true
  git -C "$SRC/embench-iot" checkout -q 09c2ed8c3b7008c95d08b038de4a3f6dc103ed70 2>/dev/null || \
    echo "embench pin 09c2ed8 missing upstream; using clone HEAD (tables may drift!)"

  # FreeRTOS-Kernel (rolling HEAD; API-stable for this test).
  clone_try https://github.com/FreeRTOS/FreeRTOS-Kernel.git "$SRC/freertos"

  # mbedTLS v4.2.0 + TF-PSA-Crypto submodule.
  if [ ! -d "$SRC/mbedtls" ]; then
    clone_try https://github.com/Mbed-TLS/mbedtls.git "$SRC/mbedtls"
    git -C "$SRC/mbedtls" fetch -q --depth 1 origin tag v4.2.0 || true
    git -C "$SRC/mbedtls" checkout -q v4.2.0
    git -C "$SRC/mbedtls" submodule update --init --depth 1 tf-psa-crypto
  fi

  # Eclipse ThreadX (rolling HEAD; demo_threadx is API-stable).
  clone_try https://github.com/eclipse-threadx/threadx.git "$SRC/threadx"
fi

if has_phase tflm; then
  # TFLM pins. Upstream tflite-micro builds standalone: its Makefile
  # compiles only tensorflow/lite/micro/{,kernels,memory_planner,
  # arena_allocator,tflite_bridge} plus third-party headers -- NOT
  # tensorflow/lite/{core,kernels,schema} from the TF monorepo. An
  # earlier version of this script overlaid those from a moving TF
  # branch, which silently skewed the APIs (tflite_bridge removed
  # upstream, Eigen-only kernel headers, kTfLiteBFloat16 skew).
  #   tflite-micro @71fba59 (2024-06): `input()` returns the C
  #     TfLiteTensor struct the port expects; micro/tflite_bridge exists.
  #   flatbuffers v23.5.26 + kissfft v130: TFLM@71fba59's own pins, from
  #     tensorflow/lite/micro/tools/make/{flatbuffers,kissfft}_download.sh
  # gemmlowp/ruy are header-only and API-stable (cloned at HEAD, as before).
  TFLM_SHA=71fba59b5bebfd64b48e167bfcb669fff568ec74
  FLATBUFFERS_TAG=v23.5.26

  # Shallow-fetch just the pinned commit (a full tflite-micro clone is
  # ~200 MB of history for one tree we need).
  if [ ! -d "$SRC/tflite-micro/.git" ]; then
    mkdir -p "$SRC/tflite-micro"
    git -C "$SRC/tflite-micro" init -q
    git -C "$SRC/tflite-micro" remote add origin https://github.com/tensorflow/tflite-micro.git
  fi
  [ "$(git -C "$SRC/tflite-micro" rev-parse HEAD 2>/dev/null || true)" = "$TFLM_SHA" ] || \
    git_try git -C "$SRC/tflite-micro" fetch -q --depth 1 origin "$TFLM_SHA"
  git -C "$SRC/tflite-micro" checkout -q "$TFLM_SHA"

  if [ ! -d "$SRC/flatbuffers/include/flatbuffers" ]; then
    clone_try https://github.com/google/flatbuffers.git "$SRC/flatbuffers"
    git_try git -C "$SRC/flatbuffers" fetch -q --depth 1 origin "refs/tags/$FLATBUFFERS_TAG:refs/tags/$FLATBUFFERS_TAG"
    git -C "$SRC/flatbuffers" checkout -q "$FLATBUFFERS_TAG"
  fi
  clone_try https://github.com/google/gemmlowp.git "$SRC/gemmlowp"
  clone_try https://github.com/google/ruy.git "$SRC/ruy"
  clone_try https://github.com/mborgerding/kissfft.git "$SRC/kissfft"
  git_try git -C "$SRC/kissfft" fetch -q --depth 1 origin "refs/tags/v130:refs/tags/v130"
  git -C "$SRC/kissfft" checkout -q v130
fi

echo "fetch done in $SRC"
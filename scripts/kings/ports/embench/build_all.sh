#!/bin/bash
# Build all 19 Embench-IoT benchmarks.
set -e
D="$(dirname "$0")"
for b in aha-mont64 crc32 depthconv edn huffbench matmult-int md5sum nettle-aes nettle-sha256 nsichneu picojpeg qrduino sglib-combined slre statemate tarfind ud wikisort xgboost; do
  "$D/build_one.sh" "$b"
done

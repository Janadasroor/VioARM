# Third-party firmware scoreboard

VioARM proves itself against public firmware with independently known
behavior — not hand-written tests. Each entry below is upstream code,
built with `arm-none-eabi-gcc 14.2`, run on a board model, with the
expected result coming from the firmware's own self-checks, NIST vectors,
or published silicon figures. Nothing here was tuned to the simulator.

## Headliners

| Firmware | Version | What runs | Known-good behavior | VioARM result |
|---|---|---|---|---|
| Dhrystone | 2.1 (Weicker) | 100000 runs, `cortex-m3 -O2`, board `stm32f103` @8 MHz | All 20 "should be" self-checks; 1.0–1.25 DMIPS/MHz published band for gcc-built M3 | **all self-checks pass, 9.08 DMIPS = 1.135 DMIPS/MHz** |
| mbedTLS | 4.2.0 + TF-PSA-Crypto | `mbedtls_aes_self_test` (ECB/CBC × 128/192/256) + `mbedtls_sha256_self_test` (NIST vectors) | every vector prints `passed` | **OVERALL: PASSED** (AES 161.7M cycles, SHA-256 57.3M cycles) |
| Eclipse ThreadX | 6.x (Eclipse) | stock `demo_threadx` (8 threads, queue, semaphore, mutex, event flags, byte/block pools) + supervisor | no thread ever errors; 2000+20 ticks, all counters grow, queue backlog ≤ capacity | **PASSED** (645K messages, sent−recvd = 41 ≤ 101) |
| TFLite Micro | 2026-09 (`cc2d43b`) | `hello_world` sine inference: float + int8 models, 8 golden inputs vs `sinf` (ε=0.05), `FullyConnected`-only resolver, 3 KiB arena | upstream self-checks pass; x86 reference output | **ALL TESTS PASSED, 17/17 lines bit-exact vs x86** (≈335K cycles, 44.5 KiB text) |
| Embench-IoT | 2026-09 | all 19 speed benchmarks, board `stm32f107` | each `verify_benchmark` returns correct | **19/19 correct=1** (cycles table below) |
| CoreMark | 2K perf | 40 iterations, `stm32f103` @8 MHz | CRCs `e714/1fd7/8e3a`; ~2.05–2.1/MHz published | **CRCs match, 2.067/MHz** |
| FreeRTOS | 10/11 kernel | two demo-style tasks + tick counter | exact 250.000 ms LED periods | **250 ms exact** |

## Embench-IoT cycle counts (`stm32f107` timing model, 8 MHz board clock)

One timed `benchmark()` call per bench after warmup (`correct=1` all):

| Benchmark | cycles | Benchmark | cycles |
|---|---|---|---|
| aha-mont64 | 4386382 | nsichneu | 5399901 |
| crc32 | 5051925 | picojpeg | 3997997 |
| depthconv | 4833438 | qrduino | 4248820 |
| edn | 4515152 | sglib-combined | 4292276 |
| huffbench | 3596910 | slre | 4041250 |
| matmult-int | 4366588 | statemate | 3726301 |
| md5sum | 3096492 | tarfind | 1758610 |
| nettle-aes | 4018551 | ud | 4501811 |
| nettle-sha256 | 3534461 | wikisort | 2544177 |
| | | xgboost | 4757028 |

## What the scoreboard found in the simulator

Testing famous firmware is a bug-finding method, and it paid off four
times — each a real compatibility gap no unit test covered:

1. **ELF loader ignored `p_paddr`** (`vioarm-core/src/loader.rs`): the
   `.data` init image never reached flash, so firmware copied zeros over
   `.data` (newlib's `_impure_ptr` was NULL — all stdio dead). Segments
   now load at their LMA. Found by Dhrystone; regression test
   `elf_data_segment_lands_at_lma_with_zero_tail`.
2. **`SCB.DEMCR` unmapped** (`vioarm-core/src/device/scb.rs`): the
   CMSIS-standard `DEMCR |= TRCENA` faulted. `TRCENA` now latches.
   Regression test `demcr_trcena_latches_and_resets`.
3. **Exception return kept the stacked Thumb bit** (`vioarm-core/src/cpu/mod.rs`):
   first dispatch into any ThreadX thread died with a fetch abort at an
   odd PC (FreeRTOS masks it in software, which is why only ThreadX
   tripped it). Now masked like the v8-M path. Regression test
   `v7m_exception_return_masks_stacked_thumb_bit`.
4. **`ORN` had swapped operands** (`exec_dp32` + both JIT templates):
   `op2 | !rn` instead of `rn | !op2`. Embench's `md5sum`/`qrduino`
   caught it; bisected to a single instruction against a native model.
   Regression test `v7m_orn_is_rn_or_not_op2`.

TFLite Micro, by contrast, ran first-try with zero simulator changes —
C++ (`-fno-exceptions -fno-rtti`), flatbuffer parsing, float and
quantized-int8 kernels all just worked, and its UART log matches the x86
reference build byte-for-byte (17/17 lines, raw prediction bits
included). QEMU concurs on the functional output as well.

## Reproduce it

The harnesses (board support, linker scripts, build/run scripts) live
outside the repo (they vendor GPL/unrelated-license sources); the method
is: upstream sources + `arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb`
+ newlib-nano + USART1 `_write` retarget + `vioarm run --board <board>`
with a cycle budget, `uart:` output drained at the end. QEMU
(`qemu-system-arm 10.2.1`, `stm32vldiscovery`/`netduino2`) serves as a
second opinion on functional output wherever it can run the binary —
with one honest caveat: QEMU does not model `DWT_CYCCNT`, so all cycle
numbers are VioARM-only by construction.

## Re-verification (2026-09-21)

The headliners above were re-proven end to end after the Phase C board
work, rebuilt from upstream (harnesses are intentionally not vendored).
Board is `stm32f103` @8 MHz unless noted; second opinion is QEMU
`mps2-an385` (functional text byte-identical wherever compared).

| Firmware | Sources | Re-verified result |
|---|---|---|
| Dhrystone 2.1 | netlib `dhry-c` shar | 20/20 checks, **9.124 DMIPS** (1.14/MHz); VioARM ≡ QEMU |
| CoreMark 2K perf | eembc/coremark, 40 iters | CRCs `e714/1fd7/8e3a`, **2.062/MHz**; VioARM ≡ QEMU |
| CoreMark 2K perf | same, 200 iters | 12 s (≥10 s rule), **"Correct operation validated"**, 16 it/s |
| Embench-IoT | embench-iot @`09c2ed8` (2026-02), `stm32f107` | **19/19 `correct=1`** (cycle table in `scripts/kings/`) |
| FreeRTOS 10/11 kernel | FreeRTOS-Kernel, 2 tasks + `vTaskDelayUntil` | **9/9 periods exactly 2000000 cycles (250.000 ms)** |
| mbedTLS 4.2.0 + TF-PSA-Crypto | `v4.2.0` + submodule | **OVERALL: PASSED** (AES + SHA-256) |
| Eclipse ThreadX 6.x | Eclipse ThreadX, stock `demo_threadx` (100 Hz tick) | **2020 ticks, ~645K messages, all counters grow, backlog ≤ cap**; QEMU concurs on all tick-driven counters |
| TFLite Micro | tflite-micro, `hello_world` float + int8, 8 goldens | **8/8 PASS, raw prediction bits bit-exact vs x86-64 reference** |

Two harness notes, both root-caused, neither a simulator defect: the
Dhrystone figure needs a tuned `strcmp` (a byte-loop libc measures 7.27 —
same binary, same sim); Embench cycle counts are pinned to the February
snapshot and differ from the September table on retuned benches
(`tarfind`, `statemate`, `md5sum`), with `correct=1` throughout. The
cycle model itself is confirmed by the band matches (Dhrystone 1.14 vs
1.0–1.25 published, CoreMark 2.062 vs ~2.05–2.1 published).

## Automation

`scripts/kings/` rebuilds this whole table from upstream on demand
(`fetch.sh` pins, `ports/*/build.sh` build, `run_all.sh` runs and
checks; `.github/workflows/kings.yml` runs it weekly). Functional gates
hard-fail; cycle pins allow 5% for toolchain codegen drift across build
roots (buffer alignment shifts newlib's word paths ~0–2%). Re-pin tables
only with a full manual re-verification, never to silence CI.

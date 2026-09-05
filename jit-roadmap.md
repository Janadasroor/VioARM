# JIT Roadmap — VioARM translated fast engine (50× target)

Steering status: **approved for phased execution with go/no-go gates**
(see Phase gates). This is the execution plan for the 50× JIT accepted in
principle under M11; the shipped M11-coarse engine (~2×, `set_fast_quantum`)
stays the default fast path until this roadmap's acceptance is met.

Convention in this file: every work item is a `- [ ]` checkbox.
Unchecked = **pending**. Checked = done (with the completing commit noted).

## 0. Why a JIT, and what it must kill

Measured board-step budget (~2.6 µs, M7 attribution):

| Bucket | Cost/step | Killed by |
|---|---|---|
| Dispatch + decode | ~0.2 µs | Translate once, execute many (TB cache) |
| Bus + MPU + wait-states | ~0.8 µs | Direct RAM/flash access behind page guards |
| Device ticks + drains + NVIC | ~1.2 µs | Tickless quanta (M11-style, K≈1024) |
| Actual emulation work | ~0.4 µs | Host codegen instead of interpretation |

50× means ~50 ns/guest-insn (~10–15 host insns). Missing **any** row
lands at 5–10×, not 50×. Calibrated landing zone: **~20 MIPS** on compute
loops — deliberately NOT QEMU-TCG parity (hundreds of MIPS), which would
require deleting the byte-exact device models, i.e. deleting the product.

## 1. Decisions recorded (do not re-litigate without new data)

- [ ] D1: emitter is **dynasm-rs** (`dynasm` + `dynasmrt`, synced versions),
  not Cranelift/LLVM (dependency weight, compile latency, MSRV drag vs our
  `deny.toml` + MSRV 1.83 posture) and not a fully custom emitter yet
  (custom emitter stays the fallback if dynasm's proc-macro or audit
  surface proves unfit — see step P0-4). Rationale: x86-64 + AArch64
  coverage (Linux dev + mac CI), MSRV 1.77 ≤ our 1.83, two small crates.
- [ ] D2: integer ALU / load-store / branches are emitted; **everything
  else is a callout** into the existing executor (SVC/BKPT, barriers,
  DSP edge cases, exclusives initially, FP always — see D3).
- [ ] D3: **FP always goes through the existing softfloat callouts.**
  Host SSE/Neon differs on NaN payload / denormal / FTZ edges; bit-exactness
  wins over FP speed. FP-heavy code simply gains less (documented).
- [ ] D4: time model is **synchronized quanta** (run ~1024 insns, flush
  devices, check IRQs) — same contract family as M11-coarse, looser bound.
  No per-device `next_event()` API (rejected: touches every timed
  peripheral for little gain over quanta).
- [ ] D5: gating is an **opt-in `jit` cargo feature, default off**, so
  non-JIT builds (MSRV floor, embedded consumers) never pull dynasm.
  CI tests JIT legs with `--features jit`.
- [ ] D6: placement is a new module **`vioarm-core/src/jit/`** (not a
  separate crate — it shares decoders, bus types, and softfloat directly;
  a crate boundary would add API friction for zero benefit).
- [ ] D7: `cyccnt`/`cycles` are **approximate in JIT mode** (per-TB block
  estimates). Unlike M11-coarse (exact counters), the JIT differential
  suite compares memory/device state only — never counters.
- [ ] D8: single-threaded execution is preserved (no worker threads, ever —
  determinism requirement). The emitter runs on the emulation thread at
  translation time.

## 2. Scope: ISA coverage of the emitter

In scope for 50× acceptance (integer subset + callouts):

- [ ] S0: 16-bit Thumb data-processing (MOV/ADD/SUB/CMP/AND/ORR/EOR/LSL/LSR/ASR/ROR, imm + reg, flag-setting correctly — N/Z/C/V bit-exact vs interpreter).
- [ ] S1: loads/stores (LDR/STR/word/halfword/byte, imm offset, pre/post-indexed, writeback) with the memory template (guard → fast path / side exit).
- [ ] S2: branches (B/BL/BX/BLX-reg/CBZ/CBNZ/TBB-TBH? TBB/TBH stay callouts initially — jump tables are rare in soak loops; list explicitly).
- [ ] S3: PUSH/POP/LDM/STM (soak loops spill; implement, not callout).
- [ ] S4: IT blocks (correct predication — soak code from real compilers uses them; test against interpreter).
- [ ] S5: 32-bit Thumb-2 ALU/load-store common subset (ADD.W/LDR.W/STR.W/B.W/BL) — NOT the full 32-bit space initially (uncovered 32-bit encodings are callouts that end the TB).
- [ ] S6: callouts (always): SVC, BKPT/semihosting, DMB/DSB/ISB (also end the TB — ISB flushes the pipeline like silicon), MRS/MSR, CPSIE/CPSID, LDREX/STREX (monitor lives in runtime; inline later only if profiled hot), WFI/WFE/SEV (side exit to runtime sleep handling), all VFP/NEON (D3), all DSP (initially; promote hot ones only on profile evidence).

Out of scope (stays interpreted via side exits, forever unless re-scoped):
plain data-processing edge encodings outside S0/S5, PKH/SSAT/USAT, table
branches initially (S2), VMRS/VMSR, CPACR-gated traps (checked in callout).

## 3. Translation-block cache

- [ ] T1: TB key = `(PC, security-state, MPU-epoch, ISA-variant)`. Security
  state is load-bearing (same VA maps different devices/bytes secure vs
  non-secure on v8-M). ISA-variant covers v6M/v7M/v8M decode differences.
  PRIMASK/BASEPRI/FAULTMASK are NOT in the key (they affect delivery, which
  happens at quanta boundaries, never mid-TB).
- [ ] T2: TB boundaries: any branch/call/return, any S6 callout insn, any
  MMIO-touching access (first access ends the block — MMIO never executes
  translated), page-boundary cross, max-length cap (e.g. 64 insns, stops
  pathological mega-blocks; tune later).
- [ ] T3: metadata per TB: guest length, touched guest pages (for
  invalidation), MPU-epoch at translation, cycle estimate, side-exit table.
- [ ] T4: cache sizing + eviction (LRU, cap e.g. 8–16k TBs; eviction must
  also drop patched chaining jumps — see T6).
- [ ] T5: cold-miss path: translate → verify (optional debug mode:
  single-step the TB in the interpreter and compare state — the dev-time
  differential seed) → insert → execute.
- [ ] T6: block chaining: hot exit edges patched to jump directly to the
  next TB (dynasm relocation/patch points); fallback helper exit to the
  runtime. Unpatch on invalidation (T4/T7).
- [ ] T7: MPU-epoch source of truth: any MPU/SCB write affecting regions,
  permissions, or enablement (MPU CTRL/RNR/RBAR/RLAR, SCB SHCSR-ish paths,
  SAU/IDAU reprogramming where modeled) bumps the epoch; TBs stamped with
  older epochs fault to retranslation. Audit every MPU-affecting write path
  and list it in code comments.

## 4. Memory virtualization + invalidation (the correctness core)

- [ ] M1: guest-page → host-slice map (TLB-style guard): `(VPN, epoch)` →
  `(host_ptr, perms)`. RAM/flash direct-mapped; everything else is a side
  exit. Guard check inline per block (hoisted: one guard per page per TB,
  not per access, where the TB's footprint proves it safe).
- [ ] M2: permission model: read/write per page from MPU decode at
  translation time; privileged/user mode changes do NOT invalidate (access
  checks that depend on mode stay dynamic in the template — document why:
  CONTROL/FAULTMASK flips are cheap, TB flushes are not).
- [ ] M3: unaligned accesses: template honors `CCR.UNALIGN_TRP` (fault via
  side exit when set; x86/AArch64 hosts tolerate unaligned natively, so
  only the *fault* needs modeling, not the slowdown).
- [ ] M4: bit-banding is ALWAYS a side exit (alias regions must never map
  to translated fast paths — same host bytes reachable two ways breaks the
  page-guard invariant). Regression test: alias write then execute.
- [ ] M5: invalidation hooks (missing one = silent wrong execution — the
  highest-severity bug class in this roadmap): CPU writes to code pages,
  DMA/mailbox drain writes, flash-gate writes, GDB-inserted breakpoints
  (`BKPT` via the gdb server's `mem.write` path — same hooks, explicitly
  tested), hook-armed ranges (see X4), and MPU-epoch bumps (T7) all flush
  overlapping TBs. Each hook site gets a comment + a dedicated test that
  writes code-then-executes and expects retranslation.
- [ ] M6: invalidation granularity 1–4 KB pages; overlapping-TB lookup is
  page-indexed (no linear TB scan on the hot path).
- [ ] M7: self-modifying-code torture test (write → execute → rewrite →
  execute, mixed with DMA writes into code pages).

## 5. Time, interrupts, sleep, exceptions

- [ ] E1: quanta loop: execute chained TBs until (a) next TB missing, (b)
  quantum cycle budget spent, (c) side exit, (d) MMIO touch; then flush
  devices, drain mailboxes, latch NVIC, deliver pending IRQ via the
  EXISTING `enter_exception` path (reuse, never reimplement).
- [ ] E2: default quantum 1024 (tune with data; bigger = faster + looser).
- [ ] E3: WFI/WFE/SEV are side exits; sleeping naps in the accurate stepper
  until an IRQ (no spinning translated idle loops — document).
- [ ] E4: exception entry AND return are side exits both ways; EXC_RETURN
  handling reuses the existing runtime (same `just_returned` discipline).
- [ ] E5: NVIC-disabled / PRIMASK-masked stretches still respect quanta
  (pending latches, delivery at next scan — matches the M11 never-early
  property; re-prove it for JIT quanta in tests).
- [ ] E6: DWT_CYCCNT/SysTick reads inside TBs return the approximate
  block-estimated counter (D7); test pins monotonicity, not exactness.

## 6. Host integration surfaces (nothing silently degrades)

- [ ] X1: hooks (`on_instruction` pre/post, watchpoints): any TB
  overlapping an armed range is bypassed (execute interpreted) — traces
  stay exact in JIT mode. Test: trace in JIT mode equals trace in accurate
  mode, byte for byte.
- [ ] X2: GDB breakpoints: breakpoint pages bypass translation (same
  mechanism as X1 via M5 hooks); single-step in JIT mode steps translated
  blocks but reports per-insn stops (decode from TB metadata).
- [ ] X3: semihosting/BKPT inside TBs are callouts (already S6; test a
  semihosting hello under JIT).
- [ ] X4: coverage example works unchanged in JIT mode (falls back per X1).
- [ ] X5: replay goldens gain the JIT mode marker; replay compares
  memory/device state only under JIT (counters excluded per D7); record
  documents the mode it ran in.
- [ ] X6: CLI `--jit` flag (default off; independent of `--fast K` —
  document the interaction matrix: accurate / fast-coarse / JIT).
- [ ] X7: fabric determinism: multi-machine runs with JIT members rerun
  byte-identical (the fabric e2e extended with one JIT member).

## 7. Phases, gates, and effort

### Phase 0 — done (M11-coarse, ~2×)

- [x] P0-0: coarse device time + differential suite + fidelity contract
  (shipped; the fallback every later phase leans on).

### Phase 1 — TB cache + quanta, interpreted execution (3–4 wk)

Goal: validate invalidation (M5/M6) + quanta (E1/E2) + chaining (T6)
before any emitter exists. Expected gain only ~1.2–1.5× (re-decode dies;
everything else stays) — the value is risk retirement, not speed.

- [ ] P1-1: TB data structures: key (T1), metadata (T3), LRU + cap (T4);
  placement per D6 (`vioarm-core/src/jit/`, `jit` feature skeleton with no
  dynasm dependency yet).
- [ ] P1-2: block builder over the EXISTING decoders (no new decode logic:
  reuse `decode16`/`decode32`, stop at T2 boundaries).
- [ ] P1-3: interpreted TB execution loop (per-insn executor calls, chained
  dispatch) + cold-miss path (T5, with debug verify mode).
- [ ] P1-4: page-guard map (M1) + invalidation hooks (M5) + M6 index.
- [ ] P1-5: quanta loop (E1/E2) reusing `enter_exception`; WFI/exception
  side exits (E3/E4).
- [ ] P1-6: differential tests TB-vs-interpreter on existing suites
  (conformance programs, AES/SHA loops, board boots).
- [ ] GATE G1: invalidation torture (M7) green + differential green +
  ≥1.2× measured, else stop and re-scope (do NOT start the emitter on a
  shaky cache).

### Phase 2 — dynasm emitter (6–10 wk)

Goal: 10–30× on compute loops.

- [ ] P2-0: dependency gate: add `dynasm`/`dynasmrt` behind the `jit`
  feature; `cargo deny` + `cargo audit` + SBOM baseline updated; MSRV
  1.83 verified (dynasm MSRV 1.77 ≤ floor — re-verify at integration);
  x86-64 + AArch64 CI legs assemble a smoke template (Windows leg
  explicitly included — codegen must exist there too).
- [ ] P2-1: emitter skeleton: register allocation (fixed guest→host
  mapping for r0–r12/sp/lr/pc + flags word — simplescalar, no graph
  coloring; document the choice), prologue/epilogue, callout ABI.
- [ ] P2-2: integer ALU templates (S0) with bit-exact flags (each template
  unit-tested against the interpreter across random inputs — the template
  test harness from this step serves all later templates).
- [ ] P2-3: load/store templates with guard + fast path + side exit (S1,
  M1–M4 incl. UNALIGN_TRP + bitband exits).
- [ ] P2-4: branch templates + chaining (S2, T6: patchable hot edges).
- [ ] P2-5: PUSH/POP/LDM/STM (S3), IT blocks (S4), 32-bit subset (S5).
- [ ] P2-6: callout plumbing for all of S6 (SVC/BKPT/barriers/MRS-MSR/
  exclusives/WFI-FE-FE/FP/DSP) + TB-ending rules.
- [ ] P2-7: per-TB cycle estimates (D7 accounting) + monotonicity tests.
- [ ] GATE G2: template fuzz (each template vs interpreter, randomized)
  green + soak ≥10× on x86-64, else stop (emitter approach wrong — fall
  back to custom emitter spike per D1 before more templates).

### Phase 3 — hardening to acceptance (4–6 wk)

- [ ] P3-1: FP callout verification matrix (D3): NaN payload / denormal /
  FTZ edge vectors through softfloat callouts vs interpreter — must match
  bitwise (they share the code; the test proves the plumbing).
- [ ] P3-2: differential FUZZING in CI (new `fuzz/fuzz_jit.rs` target +
  nightly-fuzz corpus seeds + a short bounded run in the CI fuzz job):
  random Thumb programs, TB-vs-interpreter state compare after N steps.
  Non-negotiable (M4/M5 bug class).
- [ ] P3-3: host surfaces X1–X7 implemented + tested.
- [ ] P3-4: per-platform baselines recorded (x86-64 Linux + mac ARM;
  Windows smoke): soak workloads = (a) board NOP soak 1M steps,
  (b) SHA-256 loop, (c) ALU torture loop. Gate numbers on x86-64.
- [ ] P3-5: book section (JIT architecture, contract, when-NOT-to-use),
  CHANGELOG, `BENCHMARKS.md` JIT table.
- [ ] GATE G3 (acceptance): geometric mean ≥50× over the three soak
  workloads vs accurate on x86-64 release + all suites green + fuzz
  corpus clean → JIT becomes selectable (still default-off until a
  release decision says otherwise).

Effort rollup: P1 3–4 wk + P2 6–10 wk + P3 4–6 wk ≈ **13–20 agent-weeks**,
gated (any gate may stop or re-scope the rest).

## 8. Risks (with mitigations, not just names)

- [ ] R1: missed invalidation → silent wrong execution (SEVEREST class).
  Mitigated by M5-per-site tests + M7 torture + P3-2 fuzzing (all three,
  not one).
- [ ] R2: dynasm audit/MSRV/platform fit (P2-0 gate before emitter work).
- [ ] R3: Amdahl on driver-heavy code (side exits dominate) — calibrated
  by measuring the soak workloads (compute loops), never claimed generally.
- [ ] R4: FP edge divergence — eliminated by construction (callouts share
  softfloat) + P3-1 matrix proves it.
- [ ] R5: Windows codegen divergence (emitter templates are arch-specific;
  x64-Windows ABI/calling-convention differences in callouts) — Windows CI
  leg assembles + runs template tests from P2-0 onward.
- [ ] R6: TB-cache memory growth on huge firmware (cap + LRU in T4;
  soak-tested in P3-4).
- [ ] R7: debuggability loss (translated code is opaque) — TB metadata
  carries guest-PC maps; X2 single-step + debug verify mode (T5).

## 9. Non-goals (restated so scope cannot creep)

- [ ] N1: no A-profile (M10 no-go stands — this JIT is Thumb-2 M-profile).
- [ ] N2: no SMP/multi-threaded execution (single thread, ever).
- [ ] N3: no QEMU-parity claim (~20 MIPS band, §0).
- [ ] N4: no self-modifying-code optimization (always invalidate, M5).
- [ ] N5: no LPAE/long-descriptor MMU work (M-profile MPU only — the MPU
  epoch in T1/T7 is the M-profile MPU + SAU/IDAU attribution).
- [ ] N6: no trace/ETM fidelity inside translated blocks (hooks fall back
  to interpretation, X1 — traces stay exact by construction).

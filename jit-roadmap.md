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

- [x] D1: emitter is **dynasm-rs** (`dynasm` + `dynasmrt` 5.1.0, synced),
  not Cranelift/LLVM (dependency weight, compile latency, MSRV drag vs our
  `deny.toml` + MSRV 1.83 posture) and not a fully custom emitter yet
  (custom emitter stays the fallback if dynasm's proc-macro or audit
  surface proves unfit — see step P0-4). Pinned 5.1.0 (MPL-2.0, allowed
  per `deny.toml` colored precedent); MSRV proven by building + testing
  under 1.83 locally (no declared MSRV upstream — empirical proof
  instead). P2-0 FINDING: `workspace.lints.rust.unsafe_code = "forbid"`
  cannot be allow-overridden, while calling emitted code is inherently
  unsafe — so the emitter lives in `vioarm-core/src/jit/` per this
  decision (assembly is safe API), but the final jump lives in the
  micro-crate `vioarm-jit-call` (two safe wrappers + SAFETY CASE +
  template/fuzz coverage as the soundness legs). Rationale stands:
  x86-64 + AArch64 coverage, two small crates + one micro-crate.
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

### Phase 1 — TB cache + quanta, interpreted execution (3–4 wk; DONE 2026-09-07)

Goal: validate invalidation (M5/M6) + quanta (E1/E2) + chaining (T6)
before any emitter exists. Expected gain only ~1.2–1.5× (re-decode dies;
everything else stays) — the value is risk retirement, not speed.

- [x] P1-1: TB data structures: key (T1), metadata (T3), LRU + cap (T4);
  placement per D6 (`vioarm-core/src/jit/`, `jit` feature skeleton with no
  dynasm dependency yet). Done: `TbKey`/`MpuEpoch`/`TbMeta`/`TbCache` +
  page-overlap index (M6) + `bump_epoch` purge (T7), 7 unit tests.
- [x] P1-2: block builder over the EXISTING decoders (no new decode logic:
  reuse `decode16`/`decode32`, stop at T2 boundaries). Done: `build_tb`
  mirrors the stepper's per-ISA dispatch, seals inclusive on
  ControlFlow/System (S6 incl. IT/WFI/DSP/FP/unpredictable-PC forms),
  exclusive on PageCross/Halt, refuses on ITSTATE/odd-PC/entry failure;
  15 unit tests. P1-3 must still gate block entry on MPU fetch permission
  and side-exit on first MMIO touch (documented in `builder.rs`).
- [x] P1-3: interpreted TB execution loop (per-insn executor calls, chained
  dispatch) + cold-miss path (T5, with debug verify mode). Done:
  `TbRunner::run_block`/`run_chain` with MPU-fetch gate, sleep handling,
  per-op `commit_step` accounting (cycle-identical to `step`), PC-continuity
  exits, T5 re-decode verify mode, hit/miss stats; 9 runner tests incl. a
  4-op stepper-parity check. Pending interrupts are NOT polled between
  chained blocks (same as back-to-back steps; P1-5 owns quanta).
- [x] P1-4: page-guard map (M1) + invalidation hooks (M5) + M6 index.
  Done Phase-1-style: FNV-1a content checksum stamped at insert and
  re-validated on every hit (source-agnostic — CPU/DMA/debugger/flash
  writes all funnel through the bus `write` chokepoint, so no per-site
  audit can miss one), plus a bus `config_epoch` polled at block entry
  and after every op (`SystemMemory` bumps it on any `0xE000_ED00–EE00`
  write; `FlatMemory` reports constant; untracked buses poll as
  always-changed: safe, slow). 4 new runner tests (SMC halfword+word
  rebuild, inter-block purge, mid-block STRB-exit, untracked single-op).
  The M1 page→host-slice map proper ships with the emitter (P2-3);
  hook-armed ranges arrive with X1 (P3-3).
- [x] P1-5: quanta loop (E1/E2) reusing `enter_exception`; WFI/exception
  side exits (E3/E4). Done: `Machine::run_tb_quantum(runner, max_ops)`
  (runner passed in — no `Machine` struct churn) with one boundary per
  quantum (single device flush of consumed `cyccnt` + single IRQ delivery;
  M11-coarse never-early contract), refusals served by one accurate `step`,
  sleep naps one cycle, `just_returned` skips the scan, halt flushes before
  `Err`, `fast_quantum != 0` debug-asserted out. E1(d) MMIO exit is
  Phase-2-only (shared-executor ops are already MMIO-exact); E6 approx
  counters are Phase-2 (Phase 1 is cycle-exact). 7 quanta tests incl.
  unmapped-handler boundary proof, mask latching, wake-into-handler,
  IT-fallback resumption.
- [x] P1-6: differential tests TB-vs-interpreter on existing suites
  (conformance programs, AES/SHA loops, board boots). Done:
  `tests/jit_differential.rs` (6 tests: ALU+flags+IT-in-loop, mem ops,
  BL calls, SVC round-trip via vectors, firmware SMC, host-poke
  interleaving — full regs/flags/RAM/code/cycles/cyccnt snapshots), plus a
  `just_returned` scan-skip unit test. Board-boot/AES-SHA-loop
  differentials deferred with reason: device-timing divergence is by design
  under quanta (M11 contract) — polled-device firmware converges on values
  but not on cycle counts, so cycle-exact differentials only hold for
  IRQ-quiescent firmware; board boots with SysTick/timers are P3-4
  differential-fuzzing territory, not G1. Two perf refinements landed here:
  `store_mask`-driven revalidation (replaces per-op polling; also closes
  same-block SMC) and write-generation-gated entry checks (new
  `Bus::write_epoch`, bumped in `FlatMemory`/`SystemMemory::write`;
  untracked buses revalidate every entry).
- [x] GATE G1: PASS 2026-09-07. Invalidation torture green (firmware SMC,
  host pokes halfword+word, mid-block STRB exit, epoch purge, untracked
  single-path); differential 6/6 green; **1.27×** on the RAM soak
  (`tb/quanta_ram_4k` 345.7 µs vs `tb/step_ram_4k` 439.9 µs, n=30, tight
  intervals; flash 1.14× as context — stepper decode cache already covers
  flash). The emitter (Phase 2) is unblocked on a sound cache.

### Phase 2 — dynasm emitter (6–10 wk)

Goal: 10–30× on compute loops.

- [x] P2-0: dependency gate. Done 2026-09-07: `dynasm`/`dynasmrt` 5.1.0
  optional behind `jit` (+ `vioarm-jit-call` versioned path dep, published
  inside-out per RELEASING.md); `cargo deny check --deny warnings` clean
  (MPL-2.0 allowed; transitives bitflags/byteorder/fnv/lazy_static/libc/
  memmap2/proc-macro*/quote/syn/unicode-ident all allow-listed);
  `cargo audit` clean (106 crates); SBOMs at 95 components (+10 P2-0
  crates, additive-only diff); MSRV proven (`cargo +1.83 build` and 46 jit
  + 2 trampoline tests green under 1.83); smoke assembles AND executes
  (`mov eax,42; ret` → 42) via `jit/emit.rs` + trampoline tests using only
  safe API in `vioarm-core` (`forbid` intact); `jit-smoke` CI job added
  (ubuntu/macos/windows × stable: jit unit tests, trampoline tests,
  feature-gated bench build, feature-gated clippy).
- [x] P2-1: emitter skeleton. Done 2026-09-07 (`jit/native.rs`):
  zero-copy model (no writeback path to forget — `Core` stays the single
  source of truth; addresses via `offset_of!` + `repr(C)` ctx +
  `repr(transparent)` flags, no layout assumptions); fixed map r0–r3 +
  xPSR word in callee-saved pins (rbx/r12–r15, x19–x23; `rbp` refused for
  unwind-friendliness; r4–r12/sp/lr/pc memory-backed with P2-2 template
  rules pending); prologue/epilogue with 16-alignment + ctx spill slot
  discipline for templates; exit codes `(reason<<32)|pc`;
  `extern "C"` callout ABI (callouts must not unwind — P2-2 enforces) with
  an end-to-end callout test proving mechanics + pin survival across a
  real Rust call. Cache holds `BlockPayload::{Interp,Native}` (`Arc`
  mapping; eviction leak-proven by `strong_count`). Caught by testing:
  dynasm parses `r0`–`r3` idents as x64 registers (renamed offsets), and
  the AArch64 arms needed a local cross-check (installed target;
  displacement `u32`, arch-gated `DynasmApi` import).
- [x] P2-2: integer ALU templates (S0) with bit-exact flags. Done
  2026-09-07 (`jit/templates*.rs` + shared differential harness serving
  P2-2–P2-5): full S0 set on x64 + AArch64 mirrors (12 template tests:
  random + directed edges, pinned + mem + high lanes), block emission +
  runner native dispatch (block-level D7 accounting; `set_native` gate
  keeps an interp-exact mode for the P1-6 differential, which now runs
  both modes). Minimal terminator exits included (B/Bcond/CBZ static +
  select-to-dynamic-epilogue — without them nothing goes native, since
  every block ends in control flow); 32-bit/reg/table branches + chaining
  stay P2-4. Caught by testing (6 bugs: merge_nz V-misevaluation, SUB/SBC
  borrow inversion, lahf-before-store, ROR undefined SF/ZF, stale `esi`
  across mem stores, dh-garbage in select, GT/LE dh-wipe ordering).
  Checkpoint soak: 2.63× RAM / 3.26× flash (not G2 yet).
  Soundness side-find (fixed here, applies to Phase 1 too): blocks cached
  under `itstate == 0` must refuse hits under `itstate != 0` (plain `exec`
  does not predicate) — runner gates + regression test.
- [x] P2-3: load/store templates with guard + fast path + side exit.
  Done 2026-09-07: S1 single transfers (16-bit all forms + 32-bit Mem
  with reg/imm offsets, pre/post-index, writeback; dual/multiple/
  exclusive stay P2-5/P2-6) on x64 + AArch64 mirrors; 64-entry
  direct-mapped TLB (`(vpn<<1)|priv` tags per M2, R/W perms, whole-page
  entries) via new `Bus::translate` (Flat direct / SystemMemory mirroring
  dispatch incl. QSPI + flash-gate exclusions) with shared `fill_tlb`;
  side exits re-execute the op through the interpreter (identical MMIO +
  precise faults) with per-op-exact prefix commit; page-cross always
  exits, UNALIGN_TRP baked at emit (x64 host-tolerates, AArch64 always
  strict — values identical), bitband always exits (M4), `unprivileged`
  ignored to match the interpreter (fidelity gap noted); builder seals
  after stores (entry revalidation after every store, both engines);
  fast stores bump the write generation (new `write_epoch_ptr`; untracked
  buses keep store blocks interpreted); `last_load_rd` set from the last
  op at native exit; hooks gate (any armed hook → interp, P3-3 refines).
  Caught by testing (8 bugs incl. double-subtracted TLB base, `edi`
  pointer truncation, stale `esi` across mem stores, split/partial pages,
  flash erase-on-read epoch hole — now bumped in both program + erase
  paths). Soak: mem-mix 3.19× RAM / 3.73× flash; ALU soak holds 2.47× /
  3.10×. Soundness side-find (fixed here): the write-generation gate must
  be per-entry, not global (a store in B must not mark A's stale bytes
  fresh) + regression coverage.
- [x] P2-4: branch templates + chaining (S2, T6). Done 2026-09-07:
  `BL` (native LR link), `B.W`, `Bcond.W`, `BX`/`BLX`-reg on x64 +
  AArch64 mirrors; `BLX`-imm (always faults) and table branches
  (memory+branch combo, P2-5) stay interpreted. Chaining uses runner-owned
  `LinkSlot`s in plain writable memory (no SMC — emitted code only
  reads): static edges (incl. `FALL` + taken-then-fall cond pairs) and
  single-target dynamic edges gate on entry-stamped generation + link
  epoch; per-call op/block budgets in `NativeCtx` bound native execution
  (quanta overshoot stays atomic, exact counts preserved); chained blocks
  contribute driver-identical accounting via accumulators flushed on every
  exit. `BX` fast path is plain-Thumb-target-only (magic-range check
  BEFORE the even check — the interpreter dispatches magic first;
  even targets take the new `EXIT_HALT` → `Err(NonThumbTarget)`, magic
  side-exits re-execute via the interpreter). Generalized side exit
  (`EXIT_MMIO` also covers complex-tail re-execution, no fill when
  `mem_access` is `None`). Caught by testing (5 bugs: harness/driver-entry
  stub budget starvation, missing driver-entry jump-over (double-counted
  acc), BX magic/even ordering divergence on even magics,
  `reg_off(14)` reading `msp` instead of `lr` — broke `BX lr` AND latent
  P2-2/P2-3 r14 lanes, fixed in both backends with regression tests;
  two test-vector slips of mine). Soak: ALU 8.7× RAM / 10.6× flash,
  mem-mix 8.0× / 9.5×, call-ret 5.6× / 5.7× — within reach of G2 already.
- [x] P2-5: PUSH/POP/LDM/STM (S3), IT blocks (S4), 32-bit subset (S5).
  Done 2026-09-07: 16-bit PUSH/POP/LDM/STM + 32-bit LDM/STM (IA/DB,
  writeback, loaded-rn-wins static) + LDRD/STRD (offset/pre/post-index,
  literal) + TBB/TBH (guarded table read + dynamic edge) + MOVW/T,
  DpReg/DpImm (all 16 ops, shifts incl. RRX, expand-carry rule),
  MUL/MLA/MLS, DIV (zero side-exits, MIN/-1 saturates), SXTx/UXTx,
  REV/REV16/REVSH on x64 + AArch64 mirrors. Multi-word uses two-phase
  guarded transfers (probe all words effect-free, then transfer —
  identical MMIO/partial-fault behavior); `POP {pc}`/`LDM pc` use the new
  `EXIT_POP_BRANCH` (driver runs only the branch dispatch — no transfer
  re-execution, no double reads). IT blocks: builder continuation mode
  (`ItBreak` seal, `it_entry` match-gated), no-op `IT` template, static
  cond guards with 16-bit merge suppression (`_it` variants), single exit
  store (never materialized mid-block). v8-M `PUSH` stays interpreted
  (stack-limit check; runner-gated). Caught by testing (9 bugs: R9D guard
  ordering SEGV in 3 emitters, MIN/-1 fallthrough SIGFPE, SBC carry seed,
  TST/TEQ live-V seed, TST merge_nz-vs-nzcv, REV16-as-bswap, logical-tail
  retest, `reg_off(14)` preload, interp-mode predicated execution hole —
  predicated continuations are native-or-fallback now). Plus a bench
  typo of mine (0xFC for 0xFB) that framed the JIT for a runaway stack.
  Soak: push-loop 0.64× RAM / 0.76× flash — store loops cannot chain on
  a global write generation (a store always intervenes between link and
  traversal), so they pay full driver trips; native is still 2×+ better
  than the interp-blocks fallback (0.29×) there. Follow-up: split
  code/data write generations so data-only loops chain.
- [x] P2-6: S6 coverage + TB-ending rules. Done 2026-09-07: no FFI
  callouts were needed — the space splits into inline templates (no-ops:
  barriers/NOP/YIELD; field stores: SEV/WFI/WFE; MRS/MSR field-only
  forms with runtime unprivileged/bank rules; MSR flag merges) and
  flush + driver side-exit tails (SVC/BKPT/CPS/exclusives/acquire-
  release/DSP/all VFP/TT/SG/hairy MRS-MSR — re-executed identically, no
  double effects). TB rules: barriers/SEV continue, TT flips to System
  (mid-block re-entry churned overlapping entries), everything else keeps
  its seal; `STREX` joins the store mask. Prerequisite fixes done here:
  `cur_pc` tracking (chained side exits committed foreign prefixes,
  misattributed the tracker, and mislinked — caught by a new cross-page
  chained-exit differential that FetchAborts unfixed), uniform pin flush
  on side exits (latent stale-`Rm` on `BX`-magic re-execution), and a
  sleeping-aware `fell_through`. Caught by testing (4 more: MRS spot
  expectation (mine), TST-needs-seed recurrence in TEQ form, `revsh`
  encoding gap on AArch64 (rev16+sxth), VMOV-S1 bit derivation). No new
  soak: S6's prize is coverage (prefixes stay native up to the faulting
  op), measured by the existing suites going native more often.
- [x] P2-7: per-TB cycle estimates (D7 accounting) + monotonicity tests.
  Done 2026-09-08: the builder now emits TRM-faithful static costs
  (fall-assumed bases incl. n+1 multi-word, 4c `BL`, 3c branch shapes,
  MUL/DIV/VFP classes, intra-block load-use stalls, fetch-region and
  16-bit-literal data latencies) with a `jit_cycles` battery pinning every
  arm against single-step `cyccnt`. Conditional-taken costs ride a
  `branch_delta` (+2): edge stubs add it on linked taken traversals, the
  driver adds it when the `BRANCH` exit PC is the taken-first static
  target (a cond fall edge also reports `BRANCH` — gating on the reason
  alone overcounts falls, caught by `cbz-fall`). Flash wait states are
  sampled live per native entry over flash-gated ops (`acc_flash_ops`;
  no bus reads when nothing is flash) so ACR changes need no purge.
  Documented D7 gaps: nonzero-`DIV` minimum, reg-relative data latency,
  cross-block stalls, IT-suppressed overcount. E6: firmware `DWT_CYCCNT`
  reads across quanta are non-decreasing and advance (the first attempt
  caught a test-firmware bug — one-shot prefix plus spin never re-samples
  — not a product bug). No soak movement (accounting-only; native
  absolutes flat-to-better: 49.7→45.8 / 54.2→52.1 / 111.6→100.8 µs).
- [x] GATE G2: template fuzz (each template vs interpreter, randomized)
  green + soak ≥10× on x86-64, else stop (emitter approach wrong — fall
  back to custom emitter spike per D1 before more templates).
  PASSED 2026-09-08 on owner ruling (accept on sustained): fuzz leg
  green (600 deterministic trials, 10/10 template families, full state
  compare; found + fixed a real `DIV rm>=13` accounting panic).
  Sustained ALU ~45× (margin); cold-4k ALU 9.6–10.3× (mean ≈ 10.0,
  groups moved it from 7.6–8.2×), call 8.3–9.0×, mem 4.8–7.4× (noisy
  box ±20% on streaming shapes), push 0.56× (structural — needs split
  generations). Cold-4k stays as the startup-latency signal; an exec
  page pool remains the known per-miss lever (P3-4). Work:
  `tests/jit_fuzz.rs`, emission groups, the `DIV` fix, BENCHMARKS G2
  section.

### Phase 3 — hardening to acceptance (4–6 wk)

- [x] P3-1: FP callout verification matrix (D3): NaN payload / denormal /
  FTZ edge vectors through softfloat callouts vs interpreter — must match
  bitwise (they share the code; the test proves the plumbing).
  Done 2026-09-08: `tests/jit_fp.rs` runs 8,280 bitwise comparisons
  (VFP state incl. FPSCR control/cumulative bits, GP regs, touched RAM,
  retired cycles) over 15 f32 edge values × FPSCR modes (plain/FTZ/DN/
  both) across 26 ops (VADD/VSUB/VMUL/VDIV/VMLA/VMLS/VNMLA/VFMA,
  VMOV/VABS/VNEG/VSQRT, VCMP/VCMPE, VRINTR/VRINTZ, int↔float converts
  both directions, GP moves, VMRS/VMSR, VLDR/VSTR) — native side-exit
  tails vs stepper, all green. Every vector's halves are decode-asserted
  in-test (caught two vd-bit derivation slips before they could hide).
  Boundaries kept: default rounding, single-precision, no fixed-point
  converts.
- [x] P3-2: differential FUZZING in CI (new `fuzz/fuzz_jit.rs` target +
  nightly-fuzz corpus seeds + a short bounded run in the CI fuzz job):
  random Thumb programs, TB-vs-interpreter state compare after N steps.
  Non-negotiable (M4/M5 bug class).
  Done 2026-09-08: the G2 generator moved into `vioarm_core::jit::fuzz`
  (single source: `tests/jit_fuzz.rs` deterministic trials + the new
  binary share draws, shapes, state, and compare). `fuzz_jit` runs
  `--trials N`, `--dump-trial K` (seed generation), and raw seed files
  (libFuzzer-shaped entry for later). 12 corpus seeds in
  `fuzz/corpus/fuzz_jit/` (all four shapes, all replay green); new
  `jit-fuzz` CI job builds the target, runs the corpus, and runs a
  bounded `--trials 150` smoke. Nightly extends via `--trials 20000`.
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

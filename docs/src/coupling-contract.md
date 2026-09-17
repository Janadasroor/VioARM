# Live-coupling contract (machine-driven bridge mode)

Two coupling directions, one timebase. SPICE-master (the `cosim/` shim +
`vioarm.cm` bridges) owns virtual time and pulls the machine in 1µs
quanta. Machine-driven (this page) reverses it for long digital soaks
with sparse analog events: the machine runs ahead at most one quantum
and publishes timestamped pin/analog changes; the consumer steps to the
stamps. No in-tree solver, ever; virtual time only.

Core API: `vioarm-core/src/couple.rs` (`run_quantum`,
`run_quantum_seconds`, `CoupledEvent`, `CoupledQueue`,
`seconds_to_cycles`, `cyccnt_to_seconds`). C ABI:
`vioarm_cosim_run_quantum` (additive — `step_duration` untouched).

## MUST-1 — cycle-to-time mapping

`MUST`: time is `t_seconds = cyccnt / cpu_hz`, where `cpu_hz` is the
live `ClockState` rate (`SystemMemory::cpu_hz`, through H7 `D1CPRE`, so
PLL reconfiguration mid-run just works). Stamps stay `cyccnt` (`u64`,
exact); the consumer converts with the rate in force — past stamps are
never rewritten.

`MUST`: conversion helpers are total: `seconds_to_cycles` saturates,
maps non-finite/negative/`hz == 0` to `0`; `cyccnt_to_seconds` maps
`hz == 0` to `0.0`.

Test: `couple::tests::time_mapping_saturates_and_converts`,
`couple_quantum::cycle_time_uses_live_cpu_hz`.

## MUST-2 — quantum

`MUST`: a quantum is a forced sync point — `run_quantum(max_cycles)`
retires until `cyccnt - start >= max_cycles`, then returns. Overshoot is
bounded by one step (same bound as `run_cycles`); a halt stops early
with `halt: Some(cause)`. `max_cycles == 0` retires nothing.

Test: `couple_quantum::quantum_never_exceeds_budget`,
`zero_quantum_retires_nothing`.

## MUST-3 — timestamps

`MUST`: every pin event and every analog event carries the exact
`cyccnt` immediately after its retiring step. Firmware that stages one
`apin` word per strobe yields one analog event per strobe; steady pins
emit nothing (the bridge holds its last value, like the held-DAC
bridge).

Test: `couple_quantum::stamps_within_quantum_bounds`,
`pin_toggle_emits_change_only_events`,
`analog_strobe_emits_one_event_per_word`.

## MUST-4 — order and drift

`MUST`: events inside a quantum are never reordered — retire order,
pin changes in `watch` order then the step's analog word — and stamps
are monotone non-decreasing. The consumer either interpolates between
stamps or steps to each stamp; its choice, documented here: SPICE-side
linear interpolation is fine for RC loads, edge-exact consumers step
the analog solver to the stamp.

Test: `couple_quantum::stamps_monotone_and_ordered`.

## MUST-5 — determinism

`MUST`: same firmware + same quanta + same host inputs reruns
byte-identically: same event list (stamps and payloads), same final
`pc`/`cyccnt`/pins.

Test: `couple_quantum::rerun_is_byte_identical`.

## MUST-6 — direction equivalence

`MUST`: machine-driven repeated quanta end in the same state as the
SPICE-master single `run_cycles` call for the `gpio_toggle` (pin echo)
and `adc_dc` (analog echo) shapes: same `pc`, same `cyccnt`, same pin
levels, same drained analog word.

Test: `couple_quantum::quanta_match_run_cycles_gpio`,
`quanta_match_run_cycles_analog`.

## MUST-7 — threading

`MUST`: the machine stays single-threaded — no threads, ever.
`CoupledQueue` is single-producer (machine) / single-consumer (bridge):
`push` refuses past capacity instead of growing, `drain` preserves
retire order.

Test: `couple::tests::queue_refuses_past_capacity_in_order`.

## MUST-8 — SPICE-master untouched

`MUST`: the existing `step_duration` / `run_cycles` path is behaviorally
untouched — its suites (`analog_pair`, `vioarm-cosim` smoke/pins/analog
loopbacks, `cosim/tests/run.py` scenarios) stay green unchanged.

Test: those suites, unmodified.

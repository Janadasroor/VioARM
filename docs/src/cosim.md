# Co-simulation

VioARM couples to an analog engine (VioMATRIXC, the ngspice fork) for
mixed-signal scenarios: firmware PWM into RC filters, ADC capture of
analog levels, DAC playback into loads. SPICE owns virtual time; the
digital side consumes quanta. No in-tree solver, ever (roadmap M15-N3).

## Architecture

Three layers, each with one job:

- `vioarm-cosim` (Rust, C ABI): create/step/pin/analog surface over a
  live machine. All `unsafe` lives here behind a SAFETY CASE; see its
  `src/lib.rs`. Status codes: `0` ok, `1` halted, negative = failed
  closed (detail in `vioarm_cosim_last_error`).
- `cosim/vioarm_cosim_shim.cpp` (`libvioarm_cosim_shim.so`): the XSPICE
  `d_cosim` payload. Accumulates SPICE `vtime`, advances the machine in
  1µs quanta (`base=`/`quantum=` sim args), polls 64 pins (ports A–D),
  and forwards changes as `{level, STRONG}` (driven) or `{level, HI-Z}`
  (inputs; open-drain releases for HIGH).
- `cosim/vioarm_analog_bridge/` (`vioarm.cm`): `vioarm_adc_bridge`
  (node volts → `apin` injection), `vioarm_dac_bridge` (staged word →
  node volts, held-DAC semantics), `vioarm_vcc_bridge` (rail mirror;
  no injection yet — brown-out plugs in here later).

Units: volts on the wire, millivolts at the seam. Conversion lives in
exactly one place — the C ABI.

## Netlist authoring

```spice
a_mcu [ ... 64 digital inputs ... ] [ ... 64 digital outputs ... ] mcu_model
.model mcu_model d_cosim(simulation="./libvioarm_cosim_shim.so"
    sim_args=["stm32f103","fw.hex"] queue_size=1024 irreversible=1 delay=1e-9)
```

- LTspice-compat mode is always on; author LTspice-style netlists.
- `.spiceinit` must load `digital.cm analog.cm` plus `vioarm.cm`
  before parsing.
- Firmware loads at the board boot alias (`0x0` on STM32 — gated
  main-flash stores drop while locked); the shim rebases nothing.
- `simulation=` and `sim_args` paths must be lowercase (the tree
  lowercases `.model` params); keep staged filenames lowercase.
- Fixed solver contract (drift is a test failure, not tuning):
  `.options reltol=0.005 method=gear trtol=7 chgtol=1e-13`.

## Determinism contract

Same netlist + firmware + seed reruns byte-identically at the digital
boundary (edge times, captured means). Proven by `cosim/tests/run.py
--rerun 2`. Tolerance sources, named per test: 1µs quantum phase, 1mV
quantization, RC ripple, firmware-loop lag, solver reltol.

## Validation

`python3 cosim/tests/run.py` (needs the VioMATRIXC release build; see
`VIOARM_NGSPICE` env): gpio_toggle, adc_dc, adc_ramp, dac_playback,
spice_button. Slow (minutes per transient at 1µs quanta).

## Troubleshooting

- Convergence failures are SPICE-side: GMIN stepping, tighter
  tolerances — never papered over digitally.
- `d_cosim failed to load`: check the `.so` path is lowercase and the
  cdylib it links resolves (`ldd`).
- Past ~136 d_cosim ports the ngspice tree corrupts memory (fork bug);
  stay at 64 until fixed.
- Injection into a disabled `apin` window is dropped by design —
  firmware must set `EN` first, like enabling an ADC.

"""Host-side rate limiter (E3): pace fixed quanta at a wall-clock scale.

The library has no clock: virtual time advances only through stepping
calls. This host measures each quantum and sleeps the remainder so that
``wall_seconds ~= sim_seconds / scale``. When the sim outruns the budget
(scale too ambitious for the workload) it reports lag honestly instead
of spiraling — it never sleeps negative, never steps extra, never
catches up.

Run: VIOARM_LIB=/path/to/libvioarm_cosim.so \
       PYTHONPATH=bindings/python python3 packaging/examples/rate_limited.py \
       [--scale 0.01] [--quanta 40] [--quantum-us 100]

Prints per-quantum ``(sim_us, wall_ms, drift_ms)`` lines plus a summary
with max drift and a lag flag.
"""

import argparse
import time

from vioarm import Machine, STEP_OK

# Same hand-assembled store-and-spin firmware as the minimal hosts.
FW = bytes([
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00,
    0x01, 0x48, 0x5A, 0x21, 0x01, 0x60, 0xFE, 0xE7,
    0x00, 0x00, 0x00, 0x20,
])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, default=0.01)
    parser.add_argument("--quanta", type=int, default=40)
    parser.add_argument("--quantum-us", type=float, default=100.0)
    args = parser.parse_args()

    quantum_s = args.quantum_us / 1e6
    with Machine("virtual", FW) as m:
        sim_s = 0.0
        max_abs_drift = 0.0
        t_start = time.perf_counter()
        for _ in range(args.quanta):
            assert m.step(quantum_s) == STEP_OK
            sim_s += quantum_s
            wall_s = time.perf_counter() - t_start
            # Hold the cadence; never chase (no negative sleeps).
            sleep_for = sim_s / args.scale - wall_s
            if sleep_for > 0:
                time.sleep(sleep_for)
                wall_s = time.perf_counter() - t_start
            drift_ms = (wall_s - sim_s / args.scale) * 1e3
            max_abs_drift = max(max_abs_drift, abs(drift_ms))
            print(f"sim_us={sim_s * 1e6:.0f} wall_ms={wall_s * 1e3:.1f} "
                  f"drift_ms={drift_ms:+.1f}", flush=True)
        wall_s = time.perf_counter() - t_start
        lag_ms = (wall_s - sim_s / args.scale) * 1e3
        print(f"SUMMARY quanta={args.quanta} scale={args.scale} "
              f"max_abs_drift_ms={max_abs_drift:.1f} lag_ms={lag_ms:+.1f}")


if __name__ == "__main__":
    main()

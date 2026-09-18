"""Seeded ABI fuzzer (E4): random call sequences with random (but
contract-honest) arguments against the built cdylib. Asserts nothing but
survival + valid codes + bounded RSS — deeper oracles live in the
conformance suite.

Contract honesty matters: `cap` always matches the real allocation
(a lying `cap` is caller UB, like `strcpy`, and untestable from here).
Crashes segfault the subprocess (pytest sees nonzero exit); hangs are
impossible by construction (every stepping call has a tiny budget) with
the `timeout` backstop in the pytest driver.

Run: VIOARM_LIB=... python3 pytest/fuzz_abi.py --seed 7 --iters 3000
Prints `FUZZ-OK <calls> peak_rss_kb=<n>`.
"""

import argparse
import ctypes
import os
import random
import resource
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bindings", "python"))
import vioarm

# Hand-assembled store-and-spin firmware (mirrors tests/host.rs).
STORE_FW = bytes([
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00,
    0x01, 0x48, 0x5A, 0x21, 0x01, 0x60, 0xFE, 0xE7,
    0x00, 0x00, 0x00, 0x20,
])

STATUS_CODES = {0, 1, 2, -1, -2, -3, -4, -5, -6}
CHANNELS = [b"itm", b"semihost", b"uart", b"usart1", b"nope", b""]
PINS = [(0, 0), (0, 5), (0, 15), (3, 0), (15, 31), (99, 0), (0, 99)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--iters", type=int, default=3000)
    args = parser.parse_args()
    rng = random.Random(args.seed)

    lib = vioarm._lib
    calls = 0

    def check(code):
        assert code in STATUS_CODES, f"wild status {code}"

    # A couple of live handles plus periodic fresh ones.
    machines = [vioarm.Machine("virtual", STORE_FW) for _ in range(2)]
    # Installed callbacks must stay alive while armed (ctypes owns no
    # reference once handed over).
    live_cbs = []

    for _ in range(args.iters):
        m = rng.choice(machines)
        op = rng.randrange(14)
        if op == 0:
            check(m.step(rng.choice([0.0, 0.00001, 0.001])))
        elif op == 1:
            code, _, _ = m.run_quantum(rng.choice([0.0, 0.00001, 0.001]), cap=rng.choice([0, 1, 64]))
            check(code)
        elif op == 2:
            port, pin = rng.choice(PINS)
            try:
                m.pin_get(port, pin)
            except vioarm.VioError as e:
                assert "error" in str(e)
        elif op == 3:
            port, pin = rng.choice(PINS)
            try:
                m.pin_set(port, pin, rng.choice([0, 1, 2]))
            except vioarm.VioError:
                pass
        elif op == 4:
            addr = rng.choice([0x20000000, 0x20000100, 0x50000000, 0x0, 0xFFFFFFFF])
            try:
                if rng.random() < 0.5:
                    m.read_data(addr)
                else:
                    m.write_data(addr, rng.randrange(256))
            except vioarm.VioError:
                pass
        elif op == 5:
            reg = rng.choice([0, 1, 12, 13, 14, 15, 16, 17, 99])
            try:
                if rng.random() < 0.5:
                    m.reg_read(reg)
                else:
                    m.reg_write(reg, rng.randrange(2**32))
            except vioarm.VioError:
                pass
        elif op == 6:
            ch = rng.choice(CHANNELS)
            try:
                m.drain(ch.decode(), cap=rng.choice([0, 4, 64]))
            except (vioarm.VioError, UnicodeDecodeError):
                pass
        elif op == 7:
            ch = rng.choice(CHANNELS)
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(8)))
            try:
                m.inject(ch.decode(), data)
            except (vioarm.VioError, UnicodeDecodeError):
                pass
        elif op == 8:
            if rng.random() < 0.5:
                m.reset()
            else:
                try:
                    m.load(STORE_FW if rng.random() < 0.8 else b"\x00" * 8)
                except vioarm.VioError:
                    pass
        elif op == 9:
            pc = rng.choice([0x08, 0x0C, 0xDEAD0000])
            try:
                if rng.random() < 0.5:
                    m.break_set(pc)
                else:
                    m.break_clear(pc)
            except vioarm.VioError:
                pass
        elif op == 10:
            blob = m.snapshot()
            if rng.random() < 0.3 and blob:
                blob = bytearray(blob)
                blob[rng.randrange(len(blob))] ^= 1 << rng.randrange(8)
                blob = bytes(blob)
            try:
                m.restore(blob)
            except vioarm.VioError:
                pass
        elif op == 11:
            if rng.random() < 0.5:
                cb_count = [0]
                live_cbs.append(m.set_callback(_counting_cb(cb_count)))
            else:
                live_cbs.clear()
                m.set_callback(None)
        elif op == 12:
            m.halt_reason()
            m.pc()
            m.cycles()
        else:
            # Fresh handle churn (create/destroy paths).
            try:
                with vioarm.Machine("virtual", STORE_FW):
                    pass
            except vioarm.VioError:
                pass
        calls += 1

    for m in machines:
        m.close()
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    print(f"FUZZ-OK {calls} peak_rss_kb={peak}")


def _counting_cb(holder):
    def cb(event, ctx):
        holder[0] += 1
    return cb


if __name__ == "__main__":
    main()

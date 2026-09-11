#!/usr/bin/env python3
"""Time-boxed mutation soak for the `fuzz/` stdin harnesses (M14-M1).

Each target reads one input on stdin and exits 0 (any nonzero exit is a
finding: a mismatch for `fuzz_jit`, a panic/crash otherwise). This driver
mutates random corpus seeds and feeds them to the release binary until the
time budget expires, saving any finding next to the corpus.

Usage: `python3 soak.py [--seconds N] [--bin DIR] [target ...]`
Defaults: 300 s per target, `./target/release`, all six targets.
Stdlib only; no new dependencies.
"""

import os
import random
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
TARGETS = ["fuzz_elf", "fuzz_hex", "fuzz_decode", "fuzz_rsp", "fuzz_svd", "fuzz_jit"]


def mutate(rng: random.Random, data: bytes) -> bytes:
    if not data:
        return bytes(rng.randrange(256) for _ in range(rng.randrange(1, 64)))
    buf = bytearray(data)
    for _ in range(rng.randrange(1, 8)):
        op = rng.randrange(4)
        pos = rng.randrange(len(buf))
        if op == 0:
            buf[pos] = rng.randrange(256)
        elif op == 1:
            buf[pos : pos + 1] = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 8)))
        elif op == 2 and len(buf) > 1:
            del buf[pos : pos + rng.randrange(1, min(8, len(buf)))]
        else:
            buf[pos:pos] = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 4)))
    return bytes(buf)


def soak(target: str, seconds: float, bindir: Path) -> tuple[int, int, list[str]]:
    corpus = sorted((HERE / "corpus" / target).iterdir())
    seeds = [p.read_bytes() for p in corpus if p.is_file()]
    if not seeds:
        return 0, 0, [f"no corpus for {target}"]
    binary = bindir / target
    rng = random.Random(0xC10C)
    deadline = time.monotonic() + seconds
    runs = 0
    findings: list[str] = []
    while time.monotonic() < deadline:
        data = mutate(rng, rng.choice(seeds))
        try:
            proc = subprocess.run(
                [str(binary)],
                input=data,
                capture_output=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired:
            out = HERE / "corpus" / target / f"TIMEOUT-{runs}.bin"
            out.write_bytes(data)
            findings.append(f"timeout -> {out.name}")
            continue
        runs += 1
        if proc.returncode != 0:
            out = HERE / "corpus" / target / f"FINDING-{runs}.bin"
            out.write_bytes(data)
            findings.append(f"exit {proc.returncode} -> {out.name}")
            stderr = proc.stderr.decode("utf-8", "replace")[:500]
            if stderr.strip():
                findings.append(f"  stderr: {stderr.strip().splitlines()[0]}")
    return runs, len(findings), findings


def main() -> int:
    seconds = 300.0
    bindir = HERE / "target" / "release"
    wanted = list(TARGETS)
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--seconds" and i + 1 < len(args):
            seconds = float(args[i + 1])
            i += 2
        elif args[i] == "--bin" and i + 1 < len(args):
            bindir = Path(args[i + 1])
            i += 2
        elif args[i] in TARGETS:
            if wanted == TARGETS:
                wanted = []
            wanted.append(args[i])
            i += 1
        else:
            print(f"usage: soak.py [--seconds N] [--bin DIR] [{'|'.join(TARGETS)}]")
            return 2
    failed = False
    for target in wanted:
        runs, nfind, findings = soak(target, seconds, bindir)
        status = "CLEAN" if nfind == 0 else "FINDINGS"
        print(f"{target}: {runs} runs in {seconds:g}s: {status}", flush=True)
        for finding in findings[:10]:
            print(f"  {finding}")
        failed = failed or nfind > 0
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

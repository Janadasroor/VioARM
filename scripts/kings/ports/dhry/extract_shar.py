#!/usr/bin/env python3
"""Extract netlib dhry-c shar archive (an HTML page of `sed` stanzas).
Usage: extract_shar.py <shar.html> <outdir>. Parses, never executes.
Verifies the three files against pinned line counts + markers.
"""
import re
import sys

src = open(sys.argv[1]).read().splitlines()
outdir = sys.argv[2]
files, cur = {}, None
pat = re.compile(r"sed >(\S+)")
for ln in src:
    m = pat.match(ln)
    if m and 'GO.SYSIN' in ln:
        cur = m.group(1)
        files[cur] = []
        continue
    if ln.startswith('//GO.SYSIN DD'):
        cur = None
        continue
    if cur is not None:
        files[cur].append(ln[1:] if ln.startswith('-') else ln)

want = {'dhry.h': 423, 'dhry_1.c': 385, 'dhry_2.c': 192}
for name, lines in files.items():
    if name in want:
        open(f'{outdir}/{name}', 'w').write('\n'.join(lines) + '\n')
for name, n in want.items():
    got = sum(1 for _ in open(f'{outdir}/{name}'))
    assert got == n, f'{name}: {got} lines, want {n} (shar page changed!)'
src1 = open(f'{outdir}/dhry_1.c').read()
assert 'Dhrystone Benchmark, Version 2.1' in src1
assert 'Number_Of_Runs' in src1
print('dhry sources verified')

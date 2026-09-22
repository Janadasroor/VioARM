#!/usr/bin/env python3
"""Embed the hello_world .tflite models as C arrays with exact upstream
symbol names (g_hello_world_{float,int8}_model_data). C++ const globals
have internal linkage, so the definitions carry an explicit `extern`.
Usage: gen_models.py <models-dir> <outdir>."""
import os
import sys

models, outdir = sys.argv[1], sys.argv[2]
for name in ('float', 'int8'):
    data = open(os.path.join(models, f'hello_world_{name}.tflite'), 'rb').read()
    sym = f'g_hello_world_{name}_model_data'
    open(os.path.join(outdir, f'hello_world_{name}_model_data.h'), 'w').write(
        f'#ifndef HW_{name.upper()}_H\n#define HW_{name.upper()}_H\n'
        f'extern const unsigned char {sym}[];\nextern const int {sym}_len;\n#endif\n')
    lines = [f'extern const unsigned char {sym}[] = {{']
    for i in range(0, len(data), 12):
        lines.append('  ' + ', '.join(f'0x{b:02x}' for b in data[i:i + 12]) + ',')
    lines.append('};')
    lines.append(f'const int {sym}_len = {len(data)};')
    open(os.path.join(outdir, f'hello_world_{name}_model_data.cc'), 'w').write(
        '\n'.join(lines) + '\n')
    print(name, len(data), 'bytes')

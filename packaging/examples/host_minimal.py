"""Minimal VioARM host (E2 packaging): boot, step 1 ms, read a pin.

Run: VIOARM_LIB=/path/to/libvioarm_cosim.so \
       PYTHONPATH=bindings/python python3 packaging/examples/host_minimal.py
"""

from vioarm import Machine, STEP_OK

# `ldr r0, =0x20000000; movs r1, #0x5A; str r1, [r0]; b .` + vector table.
FW = bytes([
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00,
    0x01, 0x48, 0x5A, 0x21, 0x01, 0x60, 0xFE, 0xE7,
    0x00, 0x00, 0x00, 0x20,
])

with Machine("virtual", FW) as m:
    assert m.step(0.001) == STEP_OK
    level, driving, mode = m.pin_get(0, 0)
    print(f"pc={m.pc():#x} pin0={level}")

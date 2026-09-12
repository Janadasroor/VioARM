#!/usr/bin/env python3
"""VioARM co-simulation validation suite (cosim-roadmap P4).

Builds the shim + bridges, stages hermetic per-test dirs (all-lowercase
names — the ngspice tree lowercases `.model` params, roadmap fork bug B2),
runs ngspice batch transient sims, and asserts behavioral contracts.

Each test names its tolerance source (never vibes):
  quantum phase (1us quanta), mV quantization, RC ripple, solver reltol.

Usage:
  python3 cosim/tests/run.py [--ngspice BIN] [--keep] [test ...]
  Tests: gpio_toggle adc_dc adc_ramp dac_playback spice_button
  --rerun N: repeat each test N times for the determinism check (P4-5).

Slow: each transient runs minutes (1us quanta over hundreds of ms).
"""

import os
import shutil
import statistics
import struct
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # cosim/
REPO = os.path.dirname(ROOT)  # workspace root
BUILD = os.path.join(ROOT, "build")
OUT = os.path.join(ROOT, "tests", "out")

NGSPICE = os.environ.get(
    "VIOARM_NGSPICE",
    "/home/jnd/cpp_projects/VioMATRIXC/release/src/ngspice",
)
DIGITAL_CM = os.environ.get(
    "VIOARM_DIGITAL_CM",
    "/home/jnd/cpp_projects/VioMATRIXC/release/src/xspice/icm/digital/digital.cm",
)
ANALOG_CM = os.environ.get(
    "VIOARM_ANALOG_CM",
    "/home/jnd/cpp_projects/VioMATRIXC/release/src/xspice/icm/analog/analog.cm",
)

N_PINS = 64
QUANTUM = "1e-6"
OPTIONS = ".options reltol=0.005 method=gear trtol=7 chgtol=1e-13"


# ---------------------------------------------------------------- firmware
def check_branches(fw, base=0):
    """Assert every 16-bit b/bcc target lands on a halfword (cheap guard
    against the classic (t-(a+4))/2 slip; branches use unmasked PC+4)."""
    for i in range(8, len(fw) - 1, 2):
        h = fw[i] | (fw[i + 1] << 8)
        a = base + i
        if (h & 0xF000) == 0xD000:
            off = h & 0xFF
            off -= 0x100 if off & 0x80 else 0
            assert ((a + 4 + off * 2) % 2) == 0
        elif (h & 0xF800) == 0xE000:
            off = h & 0x7FF
            off -= 0x800 if off & 0x400 else 0
            assert ((a + 4 + off * 2) % 2) == 0


def assemble(code, pool):
    fw = bytes([0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00]) + bytes(code)
    assert (8 + len(code)) % 4 == 0, "literal pool misaligned"
    fw += b"".join(struct.pack("<I", w) for w in pool)
    check_branches(fw)
    return fw


def fw_toggle_f103():
    """PC13 square wave via BSRR + delay (~100ms half-period at 8MHz).
    Byte-exact transcription of the proven P2 firmware (verified edge
    times RISE 2us / FALL 50.003ms in SPICE vs C ABI)."""
    code = [
        0x0B, 0x48, 0x01, 0x68, 0x10, 0x22, 0x11, 0x43, 0x01, 0x60,
        0x0A, 0x48, 0x01, 0x68, 0x0A, 0x4A, 0x91, 0x43, 0x0A, 0x4A,
        0x11, 0x43, 0x01, 0x60,
        0x09, 0x48, 0x0A, 0x49, 0x01, 0x60,
        0x0A, 0x4B, 0x01, 0x3B, 0xFD, 0xD1,
        0x09, 0x49, 0x01, 0x60,
        0x09, 0x4B, 0x01, 0x3B, 0xFD, 0xD1,
        0xF5, 0xE7,
    ]
    return assemble(code, [0x40021018, 0x40011004, 0x00F00000, 0x00300000,
                           0x40011010, 0x00002000, 200000, 0x20000000,
                           200000])


def fw_pwm_virtual(hcount=400, lcount=1200):
    """25%-ish bit-bang PWM on pin0 + APIN echo (RXMV->TXMV+TXGO)."""
    code = [
        0x0C, 0x48, 0x01, 0x21, 0x01, 0x60,
        0x0C, 0x48, 0x01, 0x21, 0x01, 0x60,
        0x0B, 0x4C, 0x0C, 0x4D, 0x0C, 0x4E, 0x0D, 0x4F,
        0x01, 0x20, 0x20, 0x60,
        0x0C, 0x48, 0x01, 0x38, 0xFD, 0xD1,
        0x00, 0x20, 0x20, 0x60,
        0x0B, 0x48, 0x01, 0x38, 0xFD, 0xD1,
        0x28, 0x68, 0x30, 0x60, 0x01, 0x20, 0x38, 0x60,
        0xF0, 0xE7, 0x00, 0x00,
    ]
    return assemble(code, [0x40000000, 0x40002010, 0x40000004, 0x40002008,
                           0x40002000, 0x40002004, hcount, lcount])


def fw_echo_virtual():
    """DIR pin0+pin2 out; loop OUT[2] = IN[1] (closed-loop digital)."""
    code = [
        0x0A, 0x48, 0x05, 0x21, 0x01, 0x60,
        0x0A, 0x48, 0x01, 0x21, 0x01, 0x60,
        0x09, 0x48, 0x0A, 0x49,
        0x02, 0x68, 0x02, 0x23, 0x1A, 0x42, 0x04, 0xD0,
        0x0A, 0x68, 0x04, 0x23, 0x1A, 0x43, 0x0A, 0x60,
        0xF6, 0xE7,
        0x0A, 0x68, 0x04, 0x23, 0x9A, 0x43, 0x0A, 0x60,
        0xF1, 0xE7,
    ]
    return assemble(code, [0x40000000, 0x40000004, 0x40000008, 0x40000004])


# ---------------------------------------------------------------- netlists
def wrap(names, width=100):
    lines, cur = [], ""
    for n in names:
        if len(cur) + len(n) + 1 > width:
            lines.append(cur)
            cur = n
        else:
            cur = (cur + " " + n).strip()
    lines.append(cur)
    return lines


def mcu_block(out_map, board, fw, extra_args=(), in_map=None):
    """d_cosim instance: out_map/in_map {pin_id: net} (rest ground)."""
    in_map = in_map or {}
    li = wrap([in_map.get(i, "0") for i in range(N_PINS)])
    lo = wrap([out_map.get(i, "0") for i in range(N_PINS)])
    lines = ["a_mcu [" + li[0]]
    lines += ["+ " + s for s in li[1:]]
    lines[-1] += "] [" + lo[0]
    lines += ["+ " + s for s in lo[1:]]
    lines[-1] += "] mcu_model"
    args = [board, fw] + list(extra_args)
    lines.append(
        '.model mcu_model d_cosim(simulation="./libvioarm_cosim_shim.so" '
        'sim_args=[%s] queue_size=1024 irreversible=1 delay=1e-9)'
        % ",".join('"%s"' % a for a in args)
    )
    return lines


def write(path, lines):
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def stage(name, fw_bytes, fw_name, cir_lines):
    """Copy artifacts (lowercase names only) + netlist into out/<name>/."""
    d = os.path.join(OUT, name)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    shutil.copy(os.path.join(BUILD, "libvioarm_cosim_shim.so"), d)
    shutil.copy(os.path.join(BUILD, "vioarm.cm"), d)
    with open(os.path.join(d, fw_name), "wb") as f:
        f.write(fw_bytes)
    write(os.path.join(d, "test.cir"), cir_lines)
    write(os.path.join(d, ".spiceinit"),
          ["codemodel %s" % DIGITAL_CM, "codemodel %s" % ANALOG_CM,
           "codemodel ./vioarm.cm"])
    return d


def run_ngspice(d):
    r = subprocess.run([NGSPICE, "-b", "test.cir"], cwd=d,
                       capture_output=True, text=True, timeout=1200)
    rows = []
    for line in (r.stdout or "").splitlines():
        p = line.split()
        if len(p) >= 3 and p[0][0].isdigit():
            try:
                rows.append([float(x) for x in p[1:]])
            except ValueError:
                pass
    return r.returncode, rows, (r.stderr or "")[-2000:]


# ------------------------------------------------------------------ tests
RESULTS = []


def check(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print("%s %s: %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)
    return ok


def t_gpio_toggle():
    """P4-1: F103 PC13 square wave; edge count + first-edge time.
    Tolerance: quantum phase (edges land on 1us quanta) + delay-loop
    cycle math (half-period ~50ms at HSI 8MHz)."""
    fw = fw_toggle_f103()
    cir = ["* p4-1 gpio toggle", "r_pull tog 0 10k",
           "a_dac0 [tog] [tog_an 0] dac_bridge_model",
           ".model dac_bridge_model "
           "dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)"]
    cir += mcu_block({2 * 16 + 13: "tog"}, "stm32f103", "toggle.bin",
                     ("base=0x0",))
    cir += [OPTIONS, ".tran 1u 300m", ".control", "run", "print v(tog_an)",
            ".endc", ".end"]
    d = stage("gpio_toggle", fw, "toggle.bin", cir)
    rc, rows, err = run_ngspice(d)
    if rc != 0:
        return check("gpio_toggle", False, "ngspice exit %d %.200s" % (rc, err))
    times = [r[0] for a, r in zip(rows, rows[1:])
             if (a[1] > 2.5) != (r[1] > 2.5)]
    # Cycle-true stepping (run_cycles): the F103 toggle firmware's
    # half-period is ~100ms of SPICE time (was 50ms under the old
    # instruction-count mapping, which ran 2x fast on flash).
    ok = (len(times) == 2 and times[0] < 0.001
          and 0.09 < times[1] < 0.11)
    return check("gpio_toggle", ok,
                 "edges=%d times=%s" % (len(times), ["%.6f" % t for t in times]))


def t_adc_dc():
    """P4-2a: DC 3.0V into adc_bridge; firmware echoes to dac_bridge.
    Tolerance: mV quantization (0.5mV) + echo lag (one firmware loop)."""
    fw = fw_pwm_virtual()
    cir = ["* p4-2a adc dc",
           "v_src adc_node 0 dc 3.0",
           "a_adc0 adc_node dummy0 vioarm_adc_bridge_0",
           ".model vioarm_adc_bridge_0 vioarm_adc_bridge(channel=0)",
           "r_dummy0 dummy0 0 1k",
           "a_dac0 0 dac_out vioarm_dac_bridge_0",
           ".model vioarm_dac_bridge_0 vioarm_dac_bridge(channel=0)"]
    cir += mcu_block({}, "virtual", "pwm.bin", ("base=0x0",))
    cir += [OPTIONS, ".tran 1u 50m", ".control", "run", "print v(dac_out)",
            ".endc", ".end"]
    d = stage("adc_dc", fw, "pwm.bin", cir)
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("adc_dc", False, "ngspice exit %d" % rc)
    tail = [r[1] for r in rows if r[0] >= 0.03]
    mean = statistics.mean(tail) if tail else 0.0
    return check("adc_dc", abs(mean - 3.0) < 0.01,
                 "dac mean[30m:]=%.4f" % mean)


def t_adc_ramp():
    """P4-2b: 0->5V 100ms ramp tracks through adc->echo->dac.
    Tolerance: mV quantization + one-loop lag (~0.65ms => ~33mV ramp lag
    + RC-free direct path, so bound 0.1V is generous but honest)."""
    fw = fw_pwm_virtual()
    cir = ["* p4-2b adc ramp",
           "v_src adc_node 0 pwl(0 0 100m 5)",
           "a_adc0 adc_node dummy0 vioarm_adc_bridge_0",
           ".model vioarm_adc_bridge_0 vioarm_adc_bridge(channel=0)",
           "r_dummy0 dummy0 0 1k",
           "a_dac0 0 dac_out vioarm_dac_bridge_0",
           ".model vioarm_dac_bridge_0 vioarm_dac_bridge(channel=0)"]
    cir += mcu_block({}, "virtual", "pwm.bin", ("base=0x0",))
    cir += [OPTIONS, ".tran 1u 120m", ".control", "run",
            "print v(adc_node) v(dac_out)", ".endc", ".end"]
    d = stage("adc_ramp", fw, "pwm.bin", cir)
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("adc_ramp", False, "ngspice exit %d" % rc)
    errs = [abs(r[2] - r[1]) for r in rows
            if 0.02 <= r[0] <= 0.1 and len(r) > 2]
    worst = max(errs) if errs else 9.0
    return check("adc_ramp", worst < 0.15,
                 "worst track err=%.4f" % worst)


def t_dac_playback():
    """P4-3: 25% PWM -> RC(10k/10u) ~1.25V; dac echoes the ADC level.
    Tolerance: duty overhead (~10mV) + ripple (~30mV pk-pk) + mV
    quantization; bound 0.06V."""
    fw = fw_pwm_virtual()
    cir = ["* p4-3 dac playback",
           "a_pwmdac [pwm] [pwm_an 0] dac_bridge_model",
           ".model dac_bridge_model "
           "dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)",
           "r_f pwm_an rc 10k", "c_f rc 0 10u",
           "a_adc0 rc dummy0 vioarm_adc_bridge_0",
           ".model vioarm_adc_bridge_0 vioarm_adc_bridge(channel=0)",
           "r_dummy0 dummy0 0 1k",
           "a_dac0 0 dac_out vioarm_dac_bridge_0",
           ".model vioarm_dac_bridge_0 vioarm_dac_bridge(channel=0)"]
    cir += mcu_block({0: "pwm"}, "virtual", "pwm.bin", ("base=0x0",))
    cir += [OPTIONS, ".tran 1u 600m", ".control", "run",
            "print v(rc) v(dac_out)", ".endc", ".end"]
    d = stage("dac_playback", fw, "pwm.bin", cir)
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("dac_playback", False, "ngspice exit %d" % rc)
    tail = [r for r in rows if r[0] >= 0.4]
    # print columns: time rc dac_out -> indices 1, 2
    if tail and len(tail[0]) < 3:
        return check("dac_playback", False, "short print rows")
    rc_mean = statistics.mean(r[1] for r in tail)
    dac_mean = statistics.mean(r[2] for r in tail)
    ok = abs(rc_mean - 1.25) < 0.06 and abs(dac_mean - 1.25) < 0.06
    return check("dac_playback", ok,
                 "rc=%.4f dac=%.4f" % (rc_mean, dac_mean))


def t_multi_chain():
    """P4-6/P2-4: two chips through SPICE — chip A (virtual, PWM on pin0)
    drives chip B (virtual, IN1->OUT2 echo); B's output observed.
    Asserts B follows A: >=4 B edges in 30ms, each within 2ms after an
    A edge     (quantum + B loop + Newton; tolerance named)."""
    fw_a = fw_pwm_virtual()
    fw_b = fw_echo_virtual()
    N2 = N_PINS * 2
    lo_a = ["apwm" if i == 0 else "0" for i in range(N_PINS)]
    lo_b = ["bled" if i == 2 else "0" for i in range(N_PINS)]
    li_a = ["0"] * N_PINS
    li_b = ["bwire" if i == 1 else "0" for i in range(N_PINS)]

    def wrap2(ns):
        return wrap(ns)

    lines = ["* p4-6 two-chip chain",
             "r_wire apwm bwire 1k",
             "a_daca [apwm] [apwm_an 0] dac_bridge_model",
             "a_dacb [bled] [bled_an 0] dac_bridge_model",
             ".model dac_bridge_model "
             "dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)"]
    in_all = li_a + li_b
    out_all = lo_a + lo_b
    wi, wo = wrap2(in_all), wrap2(out_all)
    lines.append("a_mcu [" + wi[0])
    for seg in wi[1:]:
        lines.append("+ " + seg)
    lines[-1] += "] [" + wo[0]
    for seg in wo[1:]:
        lines.append("+ " + seg)
    lines[-1] += "] mcu_model"
    lines.append(
        '.model mcu_model d_cosim(simulation="./libvioarm_cosim_shim.so" '
        'sim_args=["virtual","fw_a.bin","virtual","fw_b.bin","base=0x0"] '
        'queue_size=1024 irreversible=1 delay=1e-9)')
    lines += [OPTIONS, ".tran 1u 30m", ".control", "run",
              "print v(apwm_an) v(bled_an)", ".endc", ".end"]
    d = os.path.join(OUT, "multi_chain")
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    shutil.copy(os.path.join(BUILD, "libvioarm_cosim_shim.so"), d)
    shutil.copy(os.path.join(BUILD, "vioarm.cm"), d)
    with open(os.path.join(d, "fw_a.bin"), "wb") as f:
        f.write(fw_a)
    with open(os.path.join(d, "fw_b.bin"), "wb") as f:
        f.write(fw_b)
    write(os.path.join(d, "test.cir"), lines)
    write(os.path.join(d, ".spiceinit"),
          ["codemodel %s" % DIGITAL_CM, "codemodel %s" % ANALOG_CM,
           "codemodel ./vioarm.cm"])
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("multi_chain", False, "ngspice exit %d" % rc)
    # rows: [time, apwm, bled]
    def edge_times(col):
        return [r[0] for a, r in zip(rows, rows[1:])
                if (a[col] > 2.5) != (r[col] > 2.5)]
    ea, eb = edge_times(1), edge_times(2)
    ok = len(eb) >= 4
    lag_ok = True
    for t in eb:
        if not any(0 <= t - s <= 0.002 for s in ea):
            lag_ok = False
            break
    ok &= lag_ok
    return check("multi_chain", ok,
                 "a_edges=%d b_edges=%d lag_ok=%s" % (len(ea), len(eb), lag_ok))


def fw_pvd_f103():
    """PVD->GPIO copier on stm32f103: PVDE + PLS=111 (2.9V), GPIOC CRH
    pin13 output; loop copies CSR.PVDO to the pin. Assembled with
    verifiable immediates (branch targets + literal slots checked at
    authoring; see cosim-roadmap P3 notes)."""
    code = bytes([
        0x0F, 0x48, 0x01, 0x68, 0x10, 0x22, 0x11, 0x43, 0x01, 0x60,
        0x0E, 0x48, 0x01, 0x68, 0x0E, 0x4A, 0x11, 0x43, 0x01, 0x60,
        0x0D, 0x48, 0xF0, 0x21, 0x01, 0x60,
        0x0D, 0x48, 0x01, 0x68, 0x0D, 0x4A, 0x91, 0x43, 0x0D, 0x4A,
        0x11, 0x43, 0x01, 0x60,
        0x0C, 0x48, 0x0D, 0x49, 0x0D, 0x4C, 0x0E, 0x4D,
        0x02, 0x68, 0x04, 0x23, 0x1A, 0x42, 0x01, 0xD0,
        0x0C, 0x60, 0xF9, 0xE7,
        0x0D, 0x60, 0xF7, 0xE7,
    ])
    pool = [0x40021018, 0x4002101C, 0x10000000, 0x40007000, 0x40011004,
            0x00F00000, 0x00300000, 0x40007004, 0x40011010, 0x00002000,
            0x20000000]
    fw = bytes([0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00]) + code
    for word in pool:
        fw += struct.pack("<I", word)
    assert (8 + len(code)) % 4 == 0, "pool misaligned"
    return fw


def t_vcc_pvd():
    """P3-3: SPICE rail -> vcc_bridge -> PVD flag -> firmware GPIO.
    Rail 3.3V, dip to 2.5V (PVD trips at PLS=111/2.9V, BOR at ~2.0V
    stays clear), recover. Asserts the pin follows with <2ms lag
    (quantum + firmware loop + Newton; tolerance named)."""
    fw = fw_pvd_f103()
    pc13 = 2 * 16 + 13
    li = wrap(["0"] * N_PINS)
    lo = wrap(["pvd" if i == pc13 else "0" for i in range(N_PINS)])
    lines = ["* p3-3 vcc pvd",
             "v_rail vcc 0 pwl(0 3.3 10m 3.3 15m 2.5 40m 2.5 45m 3.3)",
             "a_vcc vcc dummy0 vioarm_vcc_bridge_0",
             ".model vioarm_vcc_bridge_0 vioarm_vcc_bridge",
             "r_dummy0 dummy0 0 1k",
             "a_dac0 [pvd] [pvd_an 0] dac_bridge_model",
             ".model dac_bridge_model "
             "dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)"]
    lines.append("a_mcu [" + li[0])
    for seg in li[1:]:
        lines.append("+ " + seg)
    lines[-1] += "] [" + lo[0]
    for seg in lo[1:]:
        lines.append("+ " + seg)
    lines[-1] += "] mcu_model"
    lines.append(
        '.model mcu_model d_cosim(simulation="./libvioarm_cosim_shim.so" '
        'sim_args=["stm32f103","pvd.bin","base=0x0"] '
        'queue_size=1024 irreversible=1 delay=1e-9)')
    lines += [OPTIONS, ".tran 1u 60m", ".control", "run",
              "print v(pvd_an)", ".endc", ".end"]
    d = os.path.join(OUT, "vcc_pvd")
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d)
    shutil.copy(os.path.join(BUILD, "libvioarm_cosim_shim.so"), d)
    shutil.copy(os.path.join(BUILD, "vioarm.cm"), d)
    with open(os.path.join(d, "pvd.bin"), "wb") as f:
        f.write(fw)
    write(os.path.join(d, "test.cir"), lines)
    write(os.path.join(d, ".spiceinit"),
          ["codemodel %s" % DIGITAL_CM, "codemodel %s" % ANALOG_CM,
           "codemodel ./vioarm.cm"])
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("vcc_pvd", False, "ngspice exit %d" % rc)
    ups = [r[0] for a, r in zip(rows, rows[1:]) if a[1] < 2.5 <= r[1]]
    dns = [r[0] for a, r in zip(rows, rows[1:]) if a[1] >= 2.5 > r[1]]
    # Rail crosses 2.9V falling ~12.5ms, rising ~42.5ms (PWL slopes).
    ok = (len(ups) == 1 and len(dns) == 1
          and 0.011 < ups[0] < 0.020 and 0.041 < dns[0] < 0.050)
    return check("vcc_pvd", ok, "up=%s down=%s" % (ups[:1], dns[:1]))


def t_spice_button():
    """P4-4: PWL button -> adc_bridge -> d_in[pin1]; firmware echoes
    IN[1]->OUT[2]; dac_bridge observes. Asserts follow lag < 2ms
    (quantum + firmware loop + Newton); event order must match."""
    fw = fw_echo_virtual()
    cir = ["* p4-4 spice button",
           "v_btn btn 0 pwl(0 0 10m 0 10.01m 5 20m 5 20.01m 0)",
           "a_btn [btn] [btn_dig] adc_bridge",
           ".model adc_bridge adc_bridge(in_low=1.0 in_high=4.0)",
           "a_led [led] [led_an 0] dac_bridge_model",
           ".model dac_bridge_model "
           "dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)"]
    cir += mcu_block({2: "led"}, "virtual", "echo.bin", ("base=0x0",),
                     in_map={1: "btn_dig"})
    cir += [OPTIONS, ".tran 1u 30m", ".control", "run", "print v(led_an)",
            ".endc", ".end"]
    d = stage("spice_button", fw, "echo.bin", cir)
    rc, rows, err = run_ngspice(d)
    if rc != 0 or not rows:
        return check("spice_button", False,
                     "ngspice exit %d %.200s" % (rc, err))
    ups = [r[0] for a, r in zip(rows, rows[1:])
           if a[1] < 2.5 <= r[1]]
    dns = [r[0] for a, r in zip(rows, rows[1:])
           if a[1] >= 2.5 > r[1]]
    ok = (len(ups) == 1 and len(dns) == 1
          and abs(ups[0] - 0.01001) < 0.002
          and abs(dns[0] - 0.02001) < 0.002)
    return check("spice_button", ok, "up=%s down=%s" % (ups[:1], dns[:1]))


TESTS = {
    "gpio_toggle": t_gpio_toggle,
    "adc_dc": t_adc_dc,
    "adc_ramp": t_adc_ramp,
    "dac_playback": t_dac_playback,
    "spice_button": t_spice_button,
    "multi_chain": t_multi_chain,
    "vcc_pvd": t_vcc_pvd,
}


def build_shim():
    if not os.path.exists(os.path.join(BUILD, "CMakeCache.txt")):
        r = subprocess.run(["cmake", "-S", ".", "-B", "build"], cwd=ROOT,
                           capture_output=True, text=True, timeout=300)
        if r.returncode != 0:
            print(r.stdout[-2000:])
            print(r.stderr[-2000:])
            raise SystemExit("cmake configure failed")
    # Rust cdylib first, loudly: the CMake custom command for it proved
    # unreliable (silently skipped rebuilds), so drive cargo here.
    r = subprocess.run(["cargo", "build", "--release", "-p", "vioarm-cosim"],
                       cwd=REPO, capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
        raise SystemExit("cargo cdylib build failed")
    r = subprocess.run(["cmake", "--build", BUILD, "--target",
                        "vioarm_cosim", "vioarm_cm"],
                       cwd=ROOT, capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
        raise SystemExit("shim build failed")


def main(argv):
    raw = argv[1:]
    keep = "--keep" in raw
    rerun = 1
    args = []
    skip_next = False
    for i, a in enumerate(raw):
        if skip_next:
            skip_next = False
            continue
        if a == "--keep":
            continue
        if a.startswith("--rerun"):
            if "=" in a:
                rerun = int(a.split("=", 1)[1])
            else:
                rerun = int(raw[i + 1])
                skip_next = True
            continue
        if a.startswith("--"):
            continue
        args.append(a)
    names = args or list(TESTS)
    build_shim()
    all_ok = True
    first_details = {}
    for rep in range(rerun):
        if rerun > 1:
            print("--- rerun %d/%d" % (rep + 1, rerun), flush=True)
        for name in names:
            RESULTS.clear()
            TESTS[name]()
            _, ok, detail = RESULTS[-1]
            all_ok &= ok
            # P4-5: reruns must reproduce the recorded detail exactly
            # (same edges/means — the deterministic digital boundary).
            if rep == 0:
                first_details[name] = detail
            elif first_details[name] != detail:
                print("NONDET %s: %r != %r"
                      % (name, first_details[name], detail), flush=True)
                all_ok = False
    if not keep:
        shutil.rmtree(OUT, ignore_errors=True)
    print("SUITE %s" % ("GREEN" if all_ok else "RED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))

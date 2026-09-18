"""ctypes wrapper for the VioARM embedding ABI (E2 packaging).

Zero dependencies beyond the standard library. The shared library is
located via ``VIOARM_LIB`` (full path to ``libvioarm_cosim.so``), else by
the loader default (`libvioarm_cosim.so` on the search path).

Conventions mirror the frozen contract (`docs/src/embedding.md`):
negative returns are errors (use :func:`last_error` for detail),
``STEP_OK``/``STEP_HALTED``/``STEP_BREAKPOINT`` are outcomes, not errors.
"""

import ctypes
import os

_lib_path = os.environ.get("VIOARM_LIB", "libvioarm_cosim.so")
_lib = ctypes.CDLL(_lib_path)

# Step outcomes.
STEP_OK = 0
STEP_HALTED = 1
STEP_BREAKPOINT = 2
# Error codes (negative).
ERR_NULL_HANDLE = -1
ERR_PANIC = -2
ERR_BAD_ARG = -3
ERR_BUS = -4
ERR_VERSION = -5
ERR_REENTRY = -6
# Firmware formats.
FW_ELF = 0
FW_HEX = 1
FW_BIN = 2
# Pin modes.
MODE_INPUT_FLOATING = 0
MODE_INPUT_PULL_UP = 1
MODE_INPUT_PULL_DOWN = 2
MODE_ANALOG = 3
MODE_OUTPUT_PUSH_PULL = 4
MODE_OUTPUT_OPEN_DRAIN = 5
MODE_ALTERNATE_PUSH_PULL = 6
MODE_ALTERNATE_OPEN_DRAIN = 7
# Event kinds.
EVENT_PIN = 0
EVENT_ANALOG = 1
# Halt reasons.
HALT_COMPLETED = 0
HALT_BREAKPOINT = 1
HALT_FAULT = 2
# Registers: 0-12 = r0-r12, 13 = sp, 14 = lr, 15 = pc, 16 = xpsr (ro).
REG_XPSR = 16


class ApiVersion(ctypes.Structure):
    _fields_ = [("major", ctypes.c_uint32),
                ("minor", ctypes.c_uint32),
                ("patch", ctypes.c_uint32)]


class Event(ctypes.Structure):
    _fields_ = [("cyccnt", ctypes.c_uint64),
                ("kind", ctypes.c_uint32),
                ("port", ctypes.c_uint32),
                ("pin", ctypes.c_uint32),
                ("level", ctypes.c_uint32),
                ("driving", ctypes.c_uint32),
                ("mv", ctypes.c_uint32)]


class BacktraceEntry(ctypes.Structure):
    _fields_ = [("pc", ctypes.c_uint32),
                ("op0", ctypes.c_uint16),
                ("op1", ctypes.c_uint16),
                ("has_op1", ctypes.c_uint32),
                ("cyccnt", ctypes.c_uint64)]


_EVENT_CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(Event), ctypes.c_void_p)


def _fn(name, restype, *argtypes):
    fn = getattr(_lib, name)
    fn.restype = restype
    fn.argtypes = list(argtypes)
    return fn


_c_api_version = _fn("vioarm_cosim_api_version", ApiVersion)
_c_last_error = _fn("vioarm_cosim_last_error", ctypes.c_char_p)
_c_create = _fn("vioarm_cosim_create", ctypes.c_void_p, ctypes.c_char_p,
                ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint32, ctypes.c_uint32)
_c_destroy = _fn("vioarm_cosim_destroy", None, ctypes.c_void_p)
_c_step = _fn("vioarm_cosim_step_duration", ctypes.c_int32, ctypes.c_void_p, ctypes.c_double)
_c_run_quantum = _fn("vioarm_cosim_run_quantum", ctypes.c_int32, ctypes.c_void_p,
                     ctypes.c_double, ctypes.POINTER(Event), ctypes.c_size_t,
                     ctypes.POINTER(ctypes.c_size_t))
_c_cpu_hz = _fn("vioarm_cosim_cpu_hz", ctypes.c_uint32, ctypes.c_void_p)
_c_pin_get = _fn("vioarm_cosim_pin_get", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                 ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8),
                 ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint32))
_c_pin_set = _fn("vioarm_cosim_pin_set", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                 ctypes.c_uint32, ctypes.c_uint8)
_c_snapshot_size = _fn("vioarm_cosim_snapshot_size", ctypes.c_size_t, ctypes.c_void_p)
_c_snapshot_save = _fn("vioarm_cosim_snapshot_save", ctypes.c_int32, ctypes.c_void_p,
                        ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t))
_c_snapshot_restore = _fn("vioarm_cosim_snapshot_restore", ctypes.c_int32, ctypes.c_void_p,
                           ctypes.c_char_p, ctypes.c_size_t)
_c_get_pc = _fn("vioarm_cosim_get_pc", ctypes.c_uint32, ctypes.c_void_p)
_c_get_sp = _fn("vioarm_cosim_get_sp", ctypes.c_uint32, ctypes.c_void_p)
_c_get_cycles = _fn("vioarm_cosim_get_cycles", ctypes.c_uint64, ctypes.c_void_p)
_c_get_instructions = _fn("vioarm_cosim_get_instructions", ctypes.c_uint64, ctypes.c_void_p)
_c_read_data = _fn("vioarm_cosim_read_data", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                   ctypes.POINTER(ctypes.c_uint8))
_c_write_data = _fn("vioarm_cosim_write_data", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                    ctypes.c_uint8)
_c_reset = _fn("vioarm_cosim_reset", ctypes.c_int32, ctypes.c_void_p)
_c_load = _fn("vioarm_cosim_load", ctypes.c_int32, ctypes.c_void_p, ctypes.c_char_p,
              ctypes.c_size_t, ctypes.c_uint32, ctypes.c_uint32)
_c_break_set = _fn("vioarm_cosim_break_set", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32)
_c_break_clear = _fn("vioarm_cosim_break_clear", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32)
_c_halt_reason = _fn("vioarm_cosim_halt_reason", ctypes.c_uint32, ctypes.c_void_p)
_c_reg_read = _fn("vioarm_cosim_reg_read", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                  ctypes.POINTER(ctypes.c_uint32))
_c_reg_write = _fn("vioarm_cosim_reg_write", ctypes.c_int32, ctypes.c_void_p, ctypes.c_uint32,
                   ctypes.c_uint32)
_c_drain = _fn("vioarm_cosim_drain", ctypes.c_int32, ctypes.c_void_p, ctypes.c_char_p,
               ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t))
_c_inject = _fn("vioarm_cosim_inject", ctypes.c_int32, ctypes.c_void_p, ctypes.c_char_p,
                ctypes.c_char_p, ctypes.c_size_t)
_c_set_callback = _fn("vioarm_cosim_set_event_callback", ctypes.c_int32, ctypes.c_void_p,
                      _EVENT_CB, ctypes.c_void_p)
_c_device_count = _fn("vioarm_cosim_device_count", ctypes.c_size_t, ctypes.c_void_p)
_c_board_count = _fn("vioarm_cosim_board_count", ctypes.c_size_t)


class VioError(RuntimeError):
    """A negative status from the ABI, with the library's detail."""


def last_error():
    """Detail string for the last failing call on this thread."""
    return _c_last_error().decode("utf-8", "replace")


def _check(code):
    if code < 0:
        raise VioError(f"vioarm error {code}: {last_error()}")
    return code


def api_version():
    """(major, minor, patch) — tracks the crate version."""
    v = _c_api_version()
    return (v.major, v.minor, v.patch)


class Machine:
    """One simulated machine (owns its handle; destroy on close)."""

    def __init__(self, board, firmware, fmt=FW_BIN, base=0):
        self._h = _c_create(board.encode(), firmware, len(firmware), fmt, base)
        if not self._h:
            raise VioError(f"create failed: {last_error()}")

    def close(self):
        if self._h:
            _c_destroy(self._h)
            self._h = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def step(self, seconds):
        """Step `seconds` of virtual time; returns STEP_OK/HALTED/BREAKPOINT."""
        return _check(_c_step(self._h, seconds))

    def run_quantum(self, seconds, cap=256):
        """Step and return ([Event], total_count)."""
        buf = (Event * cap)()
        total = ctypes.c_size_t(0)
        code = _check(_c_run_quantum(self._h, seconds, buf, cap, ctypes.byref(total)))
        return code, [buf[i] for i in range(min(total.value, cap))], total.value

    def pin_get(self, port, pin):
        """(level, driving, mode)."""
        level, driving, mode = ctypes.c_uint8(), ctypes.c_uint8(), ctypes.c_uint32()
        _check(_c_pin_get(self._h, port, pin, ctypes.byref(level),
                          ctypes.byref(driving), ctypes.byref(mode)))
        return (level.value, driving.value, mode.value)

    def pin_set(self, port, pin, level):
        _check(_c_pin_set(self._h, port, pin, level))

    def pc(self):
        return _c_get_pc(self._h)

    def sp(self):
        return _c_get_sp(self._h)

    def cycles(self):
        return _c_get_cycles(self._h)

    def instructions(self):
        return _c_get_instructions(self._h)

    def reset(self):
        _check(_c_reset(self._h))

    def load(self, firmware, fmt=FW_BIN, base=0):
        _check(_c_load(self._h, firmware, len(firmware), fmt, base))

    def halt_reason(self):
        return _c_halt_reason(self._h)

    def break_set(self, pc):
        _check(_c_break_set(self._h, pc))

    def break_clear(self, pc):
        _check(_c_break_clear(self._h, pc))

    def reg_read(self, reg):
        val = ctypes.c_uint32()
        _check(_c_reg_read(self._h, reg, ctypes.byref(val)))
        return val.value

    def reg_write(self, reg, val):
        _check(_c_reg_write(self._h, reg, val))

    def read_data(self, addr):
        val = ctypes.c_uint8()
        _check(_c_read_data(self._h, addr, ctypes.byref(val)))
        return val.value

    def write_data(self, addr, val):
        _check(_c_write_data(self._h, addr, val))

    def snapshot(self):
        """Save and return the snapshot bytes."""
        need = _c_snapshot_size(self._h)
        buf = ctypes.create_string_buffer(need)
        out_len = ctypes.c_size_t(0)
        _check(_c_snapshot_save(self._h, buf, need, ctypes.byref(out_len)))
        return bytes(buf.raw[:out_len.value])

    def restore(self, blob):
        _check(_c_snapshot_restore(self._h, blob, len(blob)))

    def drain(self, channel, cap=256):
        """Drain a channel; returns (bytes, total)."""
        buf = ctypes.create_string_buffer(cap)
        total = ctypes.c_size_t(0)
        _check(_c_drain(self._h, channel.encode(), buf, cap, ctypes.byref(total)))
        return bytes(buf.raw[:min(total.value, cap)]), total.value

    def inject(self, channel, data):
        _check(_c_inject(self._h, channel.encode(), data, len(data)))

    def set_callback(self, fn, ctx=None):
        """Install an event callback (or None to disable)."""
        cb = _EVENT_CB(fn) if fn else None
        _check(_c_set_callback(self._h, cb, ctx))
        return cb  # keep alive while installed

/* ABI conformance suite (E4): every `vioarm-cosim` function, every
 * status code, null probes, short buffers, version gates, snapshot
 * round-trips — from C, against the installed-style header + library.
 *
 * Layout assertions (`_Static_assert` on sizes/offsets/codes) catch the
 * bug class Rust-side tests cannot: header/representation drift.
 * Prints `ok <n> <name>` per check and `ABI-CONFORM-OK <total>` at the
 * end; any failure prints `not ok` and exits 1 immediately.
 *
 * Firmware blobs are hand-assembled 16-bit Thumb for the `virtual`
 * board (little-endian halfwords), mirroring `tests/host.rs`.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vioarm_cosim.h"

static int checks = 0;

#define CHECK(cond, name)                                                     \
    do {                                                                      \
        ++checks;                                                             \
        if (!(cond)) {                                                        \
            printf("not ok %d %s (%s:%d) err=%s\n", checks, name, __FILE__,   \
                   __LINE__, vioarm_cosim_last_error());                      \
            return 1;                                                         \
        }                                                                     \
        printf("ok %d %s\n", checks, name);                                   \
    } while (0)

/* Layout: catches header drift from the C side. */
_Static_assert(sizeof(VioArmEvent) == 32, "VioArmEvent size");
_Static_assert(offsetof(VioArmEvent, cyccnt) == 0, "event.cyccnt");
_Static_assert(offsetof(VioArmEvent, kind) == 8, "event.kind");
_Static_assert(offsetof(VioArmEvent, port) == 12, "event.port");
_Static_assert(offsetof(VioArmEvent, pin) == 16, "event.pin");
_Static_assert(offsetof(VioArmEvent, level) == 20, "event.level");
_Static_assert(offsetof(VioArmEvent, driving) == 24, "event.driving");
_Static_assert(offsetof(VioArmEvent, mv) == 28, "event.mv");
_Static_assert(sizeof(VioArmBacktraceEntry) == 24, "backtrace entry size");
_Static_assert(offsetof(VioArmBacktraceEntry, pc) == 0, "bt.pc");
_Static_assert(offsetof(VioArmBacktraceEntry, op0) == 4, "bt.op0");
_Static_assert(offsetof(VioArmBacktraceEntry, op1) == 6, "bt.op1");
_Static_assert(offsetof(VioArmBacktraceEntry, has_op1) == 8, "bt.has_op1");
_Static_assert(offsetof(VioArmBacktraceEntry, cyccnt) == 16, "bt.cyccnt");
_Static_assert(sizeof(VioArmApiVersion) == 12, "version size");
_Static_assert(VIOARM_STEP_OK == 0, "codes");
_Static_assert(VIOARM_STEP_HALTED == 1, "codes");
_Static_assert(VIOARM_STEP_BREAKPOINT == 2, "codes");
_Static_assert(VIOARM_ERR_NULL_HANDLE == -1, "codes");
_Static_assert(VIOARM_ERR_PANIC == -2, "codes");
_Static_assert(VIOARM_ERR_BAD_ARG == -3, "codes");
_Static_assert(VIOARM_ERR_BUS == -4, "codes");
_Static_assert(VIOARM_ERR_VERSION == -5, "codes");
_Static_assert(VIOARM_ERR_REENTRY == -6, "codes");
_Static_assert(VIOARM_FW_ELF == 0 && VIOARM_FW_HEX == 1 && VIOARM_FW_BIN == 2,
              "formats");
_Static_assert(VIOARM_HALT_COMPLETED == 0 && VIOARM_HALT_BREAKPOINT == 1 &&
                   VIOARM_HALT_FAULT == 2,
              "halt reasons");

/* store 0x5A @0x20000000, spin. */
static const uint8_t FW_STORE[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0x5A, 0x21,
    0x01, 0x60, 0xFE, 0xE7, 0x00, 0x00, 0x00, 0x20,
};
/* store 0xA5 @0x20000100, spin. */
static const uint8_t FW_A5[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0xA5, 0x21,
    0x01, 0x60, 0xFE, 0xE7, 0x00, 0x01, 0x00, 0x20,
};
/* virtual-GPIO toggle loop (port 0 pin 0 events). */
static const uint8_t FW_TOGGLE[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x05, 0x48, 0x01, 0x21,
    0x01, 0x60, 0x01, 0x22, 0x41, 0x68, 0x51, 0x40, 0x41, 0x60, 0xFB, 0xE7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
};
/* virtual-UART TX 'H', spin. */
static const uint8_t FW_UART[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x02, 0x48, 0x48, 0x21,
    0x01, 0x70, 0xFE, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x40,
};
/* unmapped load (faults). */
static const uint8_t FW_FAULT[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0x00, 0x68,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50,
};

static VioArmHandle *must_create(const char *board, const uint8_t *fw,
                                 size_t len) {
    VioArmHandle *h =
        vioarm_cosim_create(board, fw, len, VIOARM_FW_BIN, 0);
    if (!h) {
        printf("create failed: %s\n", vioarm_cosim_last_error());
        exit(1);
    }
    return h;
}

static int saw_events = 0;
static VioArmHandle *reentry_handle = 0;

static void counting_cb(const VioArmEvent *event, void *ctx) {
    (void)event;
    (*(int *)ctx)++;
}

static void reentry_cb(const VioArmEvent *event, void *ctx) {
    (void)event;
    (void)ctx;
    /* Must be refused while the outer quantum runs. */
    int32_t nested =
        vioarm_cosim_step_duration(reentry_handle, 0.0);
    if (nested != VIOARM_ERR_REENTRY) {
        printf("reentry not refused: nested=%d err=%s\n", (int)nested,
               vioarm_cosim_last_error());
        exit(1);
    }
    saw_events++;
}

int main(void) {
    VioArmApiVersion v = vioarm_cosim_api_version();
    printf("# api %u.%u.%u\n", v.major, v.minor, v.patch);
    CHECK(v.major >= 1, "api version sane");

    /* Bad creates return null with a reason. */
    CHECK(vioarm_cosim_create(0, FW_STORE, sizeof FW_STORE, VIOARM_FW_BIN, 0) == 0,
          "null board fails");
    CHECK(strlen(vioarm_cosim_last_error()) > 0, "last error set");
    CHECK(vioarm_cosim_create("virtual", 0, 0, VIOARM_FW_BIN, 0) == 0,
          "empty firmware fails");
    CHECK(vioarm_cosim_create("virtual", FW_STORE, sizeof FW_STORE, 99, 0) == 0,
          "bad format fails");
    CHECK(vioarm_cosim_create("nope-board", FW_STORE, sizeof FW_STORE, VIOARM_FW_BIN, 0) == 0,
          "unknown board fails");

    VioArmHandle *h = must_create("virtual", FW_STORE, sizeof FW_STORE);
    CHECK(vioarm_cosim_get_active_handle() == h, "active handle tracks");
    CHECK(vioarm_cosim_cpu_hz(0) == 0, "null hz reads 0");
    CHECK(vioarm_cosim_cpu_hz(h) == 8000000u, "virtual clock 8 MHz");

    /* Step + introspection. */
    CHECK(vioarm_cosim_step_duration(h, 0.00001) == VIOARM_STEP_OK, "step ok");
    CHECK(vioarm_cosim_halt_reason(h) == VIOARM_HALT_COMPLETED, "reason completed");
    CHECK(vioarm_cosim_get_pc(h) == 0x0E || vioarm_cosim_get_pc(h) == 0x0C,
          "spinning in store loop");
    CHECK(vioarm_cosim_get_sp(h) != 0, "stack set");
    CHECK(vioarm_cosim_get_cycles(h) > 0, "cycles elapsed");
    CHECK(vioarm_cosim_get_instructions(h) > 0, "retired");
    CHECK(vioarm_cosim_snapshot_size(0) == 0, "null size reads 0");
    CHECK(vioarm_cosim_device_count(h) > 0, "devices attached");
    CHECK(vioarm_cosim_device_count(0) == 0, "null device count reads 0");
    CHECK(vioarm_cosim_board_count() == 511, "frozen board list");
    CHECK(vioarm_cosim_step_duration(0, 0.001) == VIOARM_ERR_NULL_HANDLE,
          "null step fails");
    CHECK(vioarm_cosim_step_duration(h, -1.0) == VIOARM_ERR_BAD_ARG,
          "negative duration fails");

    /* Pins. */
    uint8_t level = 9, driving = 9;
    uint32_t mode = 99;
    CHECK(vioarm_cosim_pin_get(h, 0, 0, &level, &driving, &mode) == 0, "pin get");
    CHECK(vioarm_cosim_pin_get(h, 99, 0, &level, &driving, &mode) == VIOARM_ERR_BAD_ARG,
          "bad port fails");
    CHECK(vioarm_cosim_pin_get(h, 0, 0, 0, &driving, &mode) == VIOARM_ERR_BAD_ARG,
          "null out fails");
    CHECK(vioarm_cosim_pin_get(0, 0, 0, &level, &driving, &mode) == VIOARM_ERR_NULL_HANDLE,
          "null pin get fails");
    CHECK(vioarm_cosim_pin_set(h, 0, 5, 1) == 0, "pin set");
    CHECK(vioarm_cosim_pin_set(h, 0, 0, 2) == VIOARM_ERR_BAD_ARG, "bad level fails");
    CHECK(vioarm_cosim_pin_set(0, 0, 0, 0) == VIOARM_ERR_NULL_HANDLE, "null pin set fails");

    /* Analog + supply. */
    CHECK(vioarm_cosim_set_external_voltage(h, 1, 1.0) == VIOARM_ERR_BAD_ARG,
          "bad channel fails");
    CHECK(vioarm_cosim_set_external_voltage(h, 0, 1.65) == 0, "voltage injects");
    CHECK(vioarm_cosim_set_operating_voltage(h, 3.3) == 0, "rail drives");
    double volts = -1.0;
    CHECK(vioarm_cosim_get_dac_voltage(h, 0, &volts) == 0, "dac reads");
    CHECK(vioarm_cosim_get_dac_voltage(h, 0, 0) == VIOARM_ERR_BAD_ARG,
          "null dac out fails");

    /* Memory. */
    uint8_t val = 0;
    CHECK(vioarm_cosim_read_data(h, 0x20000000, &val) == 0 && val == 0x5A,
          "firmware store visible");
    CHECK(vioarm_cosim_write_data(h, 0x20000100, 0xAB) == 0, "ram write");
    CHECK(vioarm_cosim_read_data(h, 0x20000100, &val) == 0 && val == 0xAB,
          "ram readback");
    CHECK(vioarm_cosim_read_data(h, 0x50000000, &val) == VIOARM_ERR_BUS,
          "unmapped read refuses");
    CHECK(vioarm_cosim_write_data(h, 0x50000000, 1) == VIOARM_ERR_BUS,
          "unmapped write refuses");
    CHECK(vioarm_cosim_read_data(h, 0x20000000, 0) == VIOARM_ERR_BAD_ARG,
          "null read out fails");

    /* Snapshots: size, save, restore, version gates. */
    size_t need = vioarm_cosim_snapshot_size(h);
    CHECK(need > 0x1000, "snapshot covers regs + span");
    CHECK(vioarm_cosim_snapshot_size(0) == 0, "null size reads 0");
    uint8_t *snap = malloc(need);
    size_t out_len = 0;
    CHECK(vioarm_cosim_snapshot_save(h, snap, need, &out_len) == 0, "save ok");
    CHECK(out_len == need, "save reports size");
    CHECK(vioarm_cosim_snapshot_save(h, snap, 16, &out_len) == VIOARM_ERR_BAD_ARG,
          "short save fails");
    CHECK(out_len == need, "short save still reports need");
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need) == 0, "restore ok");
    snap[0] ^= 0xFF;
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need) == VIOARM_ERR_BAD_ARG,
          "bad magic fails");
    snap[0] ^= 0xFF;
    snap[4] += 1; /* layout version +1 */
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need) == VIOARM_ERR_VERSION,
          "bad layout version fails");
    snap[4] -= 1;
    snap[140] += 1; /* api major +1 */
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need) == VIOARM_ERR_VERSION,
          "bad api major fails");
    snap[140] -= 1;
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need - 1) == VIOARM_ERR_BAD_ARG,
          "truncated restore fails");
    CHECK(vioarm_cosim_snapshot_restore(h, snap, need) == 0, "good restore again");
    free(snap);

    /* Catalog. */
    CHECK(vioarm_cosim_board_count() == 511, "frozen board list");
    CHECK(vioarm_cosim_device_count(h) > 0, "devices attached");
    CHECK(vioarm_cosim_device_count(0) == 0, "null device count reads 0");
    char name[64];
    CHECK(vioarm_cosim_board_name(0, name, sizeof name) == 0 && name[0], "board named");
    CHECK(vioarm_cosim_device_name(h, 0, name, sizeof name) == 0 && name[0],
          "device named");
    uint32_t hz = 0;
    CHECK(vioarm_cosim_board_clock("stm32f103", &hz) == 0 && hz == 8000000u,
          "f103 clock");
    char pin[32];
    CHECK(vioarm_cosim_board_pin("stm32f103", "PA9", pin, sizeof pin) == 0 &&
              strcmp(pin, "USART1_TX") == 0,
          "pin label resolves");

    /* Reset + load. */
    CHECK(vioarm_cosim_reset(h) == 0, "reset ok");
    CHECK(vioarm_cosim_get_pc(h) == 0x08, "reset vector");
    CHECK(vioarm_cosim_load(h, FW_A5, sizeof FW_A5, VIOARM_FW_BIN, 0) == 0, "load ok");
    CHECK(vioarm_cosim_step_duration(h, 0.00001) == VIOARM_STEP_OK, "new image runs");
    CHECK(vioarm_cosim_read_data(h, 0x20000100, &val) == 0 && val == 0xA5,
          "new image stored");
    CHECK(vioarm_cosim_load(h, FW_A5, sizeof FW_A5, 77, 0) == VIOARM_ERR_BAD_ARG,
          "bad load format fails");

    /* Registers. */
    uint32_t reg = 0;
    CHECK(vioarm_cosim_reg_read(h, 0, &reg) == 0, "r0 reads");
    CHECK(vioarm_cosim_reg_write(h, 2, 0x1234) == 0, "r2 writes");
    CHECK(vioarm_cosim_reg_read(h, 2, &reg) == 0 && reg == 0x1234, "r2 reads back");
    CHECK(vioarm_cosim_reg_read(h, 16, &reg) == 0 && (reg & 0x01000000u) != 0,
          "xpsr reads");
    CHECK(vioarm_cosim_reg_read(h, 17, &reg) == VIOARM_ERR_BAD_ARG, "bad reg fails");
    CHECK(vioarm_cosim_reg_write(h, 16, 0) == VIOARM_ERR_BAD_ARG, "xpsr ro");

    /* Backtrace. */
    struct VioArmBacktraceEntry bt[8];
    size_t bt_len = 0;
    CHECK(vioarm_cosim_backtrace(h, bt, 8, &bt_len) == 0 && bt_len > 0,
          "backtrace lists retires");
    CHECK(bt[0].cyccnt > 0, "stamps present");

    /* Breakpoints. */
    CHECK(vioarm_cosim_break_set(h, 0x08) == 0, "break set");
    CHECK(vioarm_cosim_break_set(h, 0x08) == VIOARM_ERR_BAD_ARG, "dup break fails");
    CHECK(vioarm_cosim_reset(h) == 0, "reset for park test");
    CHECK(vioarm_cosim_step_duration(h, 0.001) == VIOARM_STEP_BREAKPOINT,
          "parks on breakpoint");
    CHECK(vioarm_cosim_halt_reason(h) == VIOARM_HALT_BREAKPOINT, "reason breakpoint");
    CHECK(vioarm_cosim_get_pc(h) == 0x08, "parked before the step");
    CHECK(vioarm_cosim_break_clear(h, 0x08) == 0, "break cleared");
    CHECK(vioarm_cosim_step_duration(h, 0.00001) == VIOARM_STEP_OK, "runs through");
    vioarm_cosim_destroy(h);

    /* Fault halt reason on its own handle. */
    VioArmHandle *fh = must_create("virtual", FW_FAULT, sizeof FW_FAULT);
    CHECK(vioarm_cosim_step_duration(fh, 0.001) == VIOARM_STEP_HALTED, "fault halts");
    CHECK(vioarm_cosim_halt_reason(fh) == VIOARM_HALT_FAULT, "reason fault");
    vioarm_cosim_destroy(fh);

    /* Drain / inject. */
    VioArmHandle *uh = must_create("virtual", FW_UART, sizeof FW_UART);
    CHECK(vioarm_cosim_step_duration(uh, 0.00001) == VIOARM_STEP_OK, "uart runs");
    uint8_t tx[8];
    size_t tx_len = 99;
    CHECK(vioarm_cosim_drain(uh, "uart", tx, sizeof tx, &tx_len) == 0 && tx_len == 1 &&
              tx[0] == 'H',
          "uart byte drains");
    CHECK(vioarm_cosim_drain(uh, "nope", tx, sizeof tx, &tx_len) == VIOARM_ERR_BAD_ARG,
          "unknown channel fails");
    CHECK(vioarm_cosim_inject(uh, "semihost", (const uint8_t *)"hi", 2) == 0,
          "semihost injects");
    CHECK(vioarm_cosim_inject(uh, "nope", (const uint8_t *)"hi", 2) == VIOARM_ERR_BAD_ARG,
          "unknown inject fails");
    vioarm_cosim_destroy(uh);

    /* Callbacks: counting + reentry refusal over the toggle loop. */
    VioArmHandle *th = must_create("virtual", FW_TOGGLE, sizeof FW_TOGGLE);
    int count = 0;
    CHECK(vioarm_cosim_set_event_callback(th, counting_cb, &count) == 0,
          "callback installs");
    VioArmEvent evts[64];
    size_t ev_len = 0;
    CHECK(vioarm_cosim_run_quantum(th, 0.001, evts, 64, &ev_len) == VIOARM_STEP_OK,
          "evented quantum runs");
    CHECK(ev_len > 2 && count == (int)ev_len, "callback saw every event");
    CHECK(vioarm_cosim_set_event_callback(th, 0, 0) == 0, "callback uninstalls");
    saw_events = 0;
    reentry_handle = th;
    CHECK(vioarm_cosim_set_event_callback(th, reentry_cb, 0) == 0, "reentry cb installs");
    CHECK(vioarm_cosim_run_quantum(th, 0.001, evts, 64, &ev_len) == VIOARM_STEP_OK,
          "outer call survives reentry");
    CHECK(saw_events > 0, "reentry attempted and refused");
    vioarm_cosim_destroy(th);

    /* Null destroy is a silent no-op. */
    vioarm_cosim_destroy(0);

    printf("ABI-CONFORM-OK %d\n", checks);
    return 0;
}

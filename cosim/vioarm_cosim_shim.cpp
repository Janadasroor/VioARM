/* vioarm_cosim_shim.cpp — d_cosim co-simulation shim for VioARM
 *
 * Shared library loaded by ngspice's d_cosim XSPICE code model
 * (`simulation=".../libvioarm_cosim.so"`). Embeds ONE VioARM machine
 * (multi-instance is cosim-roadmap P2-4) and advances it in fixed
 * `quantum` slices (default 1µs) of SPICE virtual time. No daemon or SHM.
 *
 * Pin I/O design (mirrors VioAVR's shim, adapted to STM32 semantics via
 * the C ABI mode codes):
 *   Driven pins (output/alternate): {level, STRONG} — the chip owns them.
 *     Open-drain outputs drive LOW actively and release (HI-Z) for HIGH.
 *   Undriven pins (input/analog): {level, HI_IMPEDANCE} — the chip never
 *     drives; external circuitry defines the bus freely. Emitted once on
 *     an output->input transition (release), then left alone so input
 *     traffic never fights the external driver.
 *
 * Netlist port vector: 64 d_in + 64 d_out, ports A..D x 16 pins.
 * Pin ID = port * 16 + pin (port 0 = A). Ports a board does not have
 * read back undriven and ignore drives (the C ABI fails those lookups
 * closed; the shim skips them after the first probe).
 *
 * NOTE (fork bug, 2026-09-11): d_cosim instances with more than ~136
 * ports corrupt memory in this ngspice tree (non-monotonic crashes past
 * the threshold even with wrapped lines; our shim is never reached), so
 * the vector stays at 64 until the fork fixes it. Ports E..K ride with
 * that fix (roadmap P2-4); the absent-port path above makes growing the
 * vector safe and netlist-compatible.
 *
 * sim_args: ["board", "fwfile", "key=value", ...]
 *   board      an `all_boards` short name (`stm32f103`, ...) or `virtual`
 *   fwfile     firmware image; format by extension (.elf/.hex/.bin)
 *   base=      load/boot base for .bin (default 0 — the STM32 boot alias,
 *              matching the host loading convention; see `create` docs)
 *   quantum=   seconds per MCU advance (default 1e-6)
 */

extern "C" {
#include "ngspice/cmtypes.h"
#include "ngspice/cosim.h"
}
#include "vioarm_cosim.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VIOARM_MAX_PORTS 4
#define VIOARM_PINS_PER_PORT 16
#define VIOARM_TOTAL_PINS (VIOARM_MAX_PORTS * VIOARM_PINS_PER_PORT)
#define VIOARM_DEFAULT_QUANTUM 1.0e-6
#define VIOARM_MAX_QUANTA_PER_STEP 1000000

struct ShimPin {
    Digital_t prev;
    uint8_t prev_driven;
    uint8_t present;
    uint8_t present_probed;
};

struct VioArmCosimInst {
    VioArmHandle *arm;
    double last_vtime;
    double accumulated;
    double quantum;
    struct ShimPin pins[VIOARM_TOTAL_PINS];
    int pins_initialized;
    int halt_logged;
};

static int pin_is_open_drain(uint32_t mode)
{
    return mode == VIOARM_MODE_OUTPUT_OPEN_DRAIN ||
           mode == VIOARM_MODE_ALTERNATE_OPEN_DRAIN;
}

static uint32_t xsup_mode_default(void) { return VIOARM_MODE_INPUT_FLOATING; }

/* Poll every pin after a time advance and forward changes. */
static void poll_pins(struct co_info *info, struct VioArmCosimInst *inst)
{
    for (unsigned int id = 0; id < VIOARM_TOTAL_PINS; id++) {
        struct ShimPin *pin = &inst->pins[id];
        unsigned int port = id / VIOARM_PINS_PER_PORT;
        unsigned int bit = id % VIOARM_PINS_PER_PORT;

        uint8_t level = 0, driving = 0;
        uint32_t mode = xsup_mode_default();
        int32_t rc = vioarm_cosim_pin_get(inst->arm, port, bit, &level,
                                          &driving, &mode);
        if (rc != 0) {
            /* No such port on this board: park HI-Z once, never ask again. */
            if (!pin->present_probed) {
                pin->present_probed = 1;
                pin->present = 0;
                Digital_t hiz = {ZERO, HI_IMPEDANCE};
                pin->prev = hiz;
                (*info->out_fn)(info, id, &hiz);
            }
            continue;
        }
        if (!pin->present_probed) {
            pin->present_probed = 1;
            pin->present = 1;
        }

        Digital_t val;
        if (driving) {
            if (pin_is_open_drain(mode)) {
                val.state = ZERO;
                val.strength = level ? HI_IMPEDANCE : STRONG;
            } else {
                val.state = level ? ONE : ZERO;
                val.strength = STRONG;
            }
        } else if (pin->prev_driven) {
            /* Output -> input transition: release the bus once. */
            val.state = level ? ONE : ZERO;
            val.strength = HI_IMPEDANCE;
        } else {
            /* Pure input traffic: never forward (would fight drivers). */
            pin->prev_driven = 0;
            continue;
        }
        pin->prev_driven = driving ? 1 : 0;

        if (val.state != pin->prev.state || val.strength != pin->prev.strength) {
            pin->prev = val;
            (*info->out_fn)(info, id, &val);
        }
    }
}

static void shim_step(struct co_info *info)
{
    struct VioArmCosimInst *inst = (struct VioArmCosimInst *)info->handle;
    if (!inst || !inst->arm)
        return;

    /* First step: release every pin to HI-Z (mirrors the AVR shim — pull
     * resistors and external drivers own the bus from time 0). Per
     * instance: ngspice can run several simulations in one process. */
    if (!inst->pins_initialized) {
        inst->pins_initialized = 1;
        for (unsigned int i = 0; i < VIOARM_TOTAL_PINS; i++) {
            Digital_t hiz = {ZERO, HI_IMPEDANCE};
            inst->pins[i].prev = hiz;
            inst->pins[i].prev_driven = 0;
            (*info->out_fn)(info, i, &hiz);
        }
    }

    double delta = info->vtime - inst->last_vtime;
    if (delta <= 0.0)
        return;
    inst->last_vtime = info->vtime;
    inst->accumulated += delta;

    long quanta = 0;
    while (inst->accumulated >= inst->quantum) {
        if (++quanta > VIOARM_MAX_QUANTA_PER_STEP) {
            fprintf(stderr,
                    "[VIOARM-COSIM] quantum backlog exceeded at %.9f s; "
                    "dropping %.9f s (shrink .tran step or grow quantum=)\n",
                    info->vtime, inst->accumulated);
            inst->accumulated = 0.0;
            break;
        }
        int32_t rc = vioarm_cosim_step_duration(inst->arm, inst->quantum);
        inst->accumulated -= inst->quantum;
        if (rc == VIOARM_STEP_HALTED) {
            if (!inst->halt_logged) {
                inst->halt_logged = 1;
                fprintf(stderr, "[VIOARM-COSIM] core halted at %.9f s: %s\n",
                        info->vtime, vioarm_cosim_last_error());
            }
            inst->accumulated = 0.0;
            break;
        }
        if (rc != VIOARM_STEP_OK) {
            fprintf(stderr, "[VIOARM-COSIM] step error at %.9f s: %s\n",
                    info->vtime, vioarm_cosim_last_error());
            inst->accumulated = 0.0;
            break;
        }
    }

    poll_pins(info, inst);
}

static void shim_input(struct co_info *info, unsigned int bit, Digital_t *value)
{
    struct VioArmCosimInst *inst = (struct VioArmCosimInst *)info->handle;
    if (!inst || !inst->arm)
        return;
    if (bit >= VIOARM_TOTAL_PINS)
        return;
    unsigned int port = bit / VIOARM_PINS_PER_PORT;
    unsigned int pin = bit % VIOARM_PINS_PER_PORT;
    uint8_t level = (value->state == ONE) ? 1 : 0;
    /* Driven pins ignore this in the model (the MCU owns the net),
     * exactly like the AVR shim's HI-Z input rule. */
    vioarm_cosim_pin_set(inst->arm, port, pin, level);
}

static void shim_cleanup(struct co_info *info)
{
    struct VioArmCosimInst *inst = (struct VioArmCosimInst *)info->handle;
    if (inst) {
        if (inst->arm)
            vioarm_cosim_destroy(inst->arm);
        free(inst);
        info->handle = NULL;
    }
}

static void nop_step(struct co_info *info) { (void)info; }
static void nop_input(struct co_info *info, unsigned int bit, Digital_t *value)
{
    (void)info;
    (void)bit;
    (void)value;
}
static void nop_cleanup(struct co_info *info) { (void)info; }

/* Read a whole file into memory (firmware image for `create`). */
static uint8_t *read_file(const char *path, size_t *len_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[VIOARM-COSIM] cannot open '%s'\n", path);
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long len = ftell(f);
    if (len <= 0) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    uint8_t *buf = (uint8_t *)malloc((size_t)len);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    size_t got = fread(buf, 1, (size_t)len, f);
    fclose(f);
    if (got != (size_t)len) {
        free(buf);
        return NULL;
    }
    *len_out = (size_t)len;
    return buf;
}

static uint32_t format_for_path(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (dot) {
        if (strcmp(dot, ".elf") == 0)
            return VIOARM_FW_ELF;
        if (strcmp(dot, ".hex") == 0 || strcmp(dot, ".ihex") == 0)
            return VIOARM_FW_HEX;
        if (strcmp(dot, ".bin") == 0)
            return VIOARM_FW_BIN;
    }
    return VIOARM_FW_HEX; /* toolchains default to Intel HEX */
}

extern "C" void Cosim_setup(struct co_info *info)
{
    info->handle = NULL;
    info->in_count = 0;
    info->out_count = 0;
    info->inout_count = 0;
    info->step = nop_step;
    info->in_fn = nop_input;
    info->cleanup = nop_cleanup;
    info->method = Normal;

    if (info->sim_argc < 2 || !info->sim_argv || !info->sim_argv[0] ||
        !info->sim_argv[1]) {
        fprintf(stderr,
                "[VIOARM-COSIM] sim_args=[\"board\",\"fwfile\",...] required\n");
        return;
    }
    const char *board = info->sim_argv[0];
    const char *fwpath = info->sim_argv[1];

    uint32_t format = format_for_path(fwpath);
    uint32_t base = 0;
    double quantum = VIOARM_DEFAULT_QUANTUM;
    for (unsigned int i = 2; i < info->sim_argc; i++) {
        const char *arg = info->sim_argv[i];
        if (!arg)
            continue;
        if (strncmp(arg, "base=", 5) == 0)
            base = (uint32_t)strtoul(arg + 5, NULL, 0);
        else if (strncmp(arg, "quantum=", 8) == 0)
            quantum = strtod(arg + 8, NULL);
        else if (strncmp(arg, "format=", 7) == 0) {
            if (strcmp(arg + 7, "elf") == 0)
                format = VIOARM_FW_ELF;
            else if (strcmp(arg + 7, "hex") == 0)
                format = VIOARM_FW_HEX;
            else if (strcmp(arg + 7, "bin") == 0)
                format = VIOARM_FW_BIN;
        }
    }
    if (!(quantum > 0.0) || quantum > 1.0) {
        fprintf(stderr, "[VIOARM-COSIM] bad quantum, using default\n");
        quantum = VIOARM_DEFAULT_QUANTUM;
    }

    size_t fw_len = 0;
    uint8_t *fw = read_file(fwpath, &fw_len);
    if (!fw)
        return;

    VioArmHandle *arm = vioarm_cosim_create(board, fw, fw_len, format, base);
    free(fw);
    if (!arm) {
        fprintf(stderr, "[VIOARM-COSIM] create failed: %s\n",
                vioarm_cosim_last_error());
        return;
    }

    struct VioArmCosimInst *inst =
        (struct VioArmCosimInst *)calloc(1, sizeof(*inst));
    if (!inst) {
        vioarm_cosim_destroy(arm);
        return;
    }
    inst->arm = arm;
    inst->quantum = quantum;
    for (unsigned int i = 0; i < VIOARM_TOTAL_PINS; i++) {
        inst->pins[i].prev.state = ZERO;
        inst->pins[i].prev.strength = HI_IMPEDANCE;
    }

    uint32_t hz = vioarm_cosim_cpu_hz(arm);
    fprintf(stderr, "[VIOARM-COSIM] board=%s cpu=%u Hz quantum=%.3e s\n", board,
            hz, quantum);

    info->handle = inst;
    info->in_count = VIOARM_TOTAL_PINS;
    info->out_count = VIOARM_TOTAL_PINS;
    info->step = shim_step;
    info->in_fn = shim_input;
    info->cleanup = shim_cleanup;
}

/* vioarm_cosim_shim.cpp — d_cosim co-simulation shim for VioARM
 *
 * Shared library loaded by ngspice's d_cosim XSPICE code model
 * (`simulation=".../libvioarm_cosim_shim.so"`). Embeds up to two VioARM
 * machines (P2-4 multi-instance) and advances each in fixed `quantum`
 * slices (default 1µs) of SPICE virtual time. No daemon or SHM.
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
 * Netlist port vector per chip: 64 d_in + 64 d_out, ports A..D x 16 pins.
 * Pin ID = chip * 64 + port * 16 + pin (chip 0 = first pair). Ports a
 * board does not have read back undriven and ignore drives (the C ABI
 * fails those lookups closed; the shim skips them after the first probe).
 *
 * Two chips fit the proven-stable envelope (128 ports/side; the fork's
 * wide-vector crashes are fixed, but headroom is cheap prudence).
 * Growing pins-per-chip (ports E..K) or chip count is additive.
 *
 * sim_args: ["board1", "fwfile1", ("board2", "fwfile2",) "key=value", ...]
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
#define VIOARM_PINS_PER_CHIP (VIOARM_MAX_PORTS * VIOARM_PINS_PER_PORT)
#define VIOARM_MAX_CHIPS 2
#define VIOARM_DEFAULT_QUANTUM 1.0e-6
#define VIOARM_MAX_QUANTA_PER_STEP 1000000

struct ShimPin {
    Digital_t prev;
    uint8_t prev_driven;
    uint8_t present;
    uint8_t present_probed;
};

struct ChipState {
    VioArmHandle *arm;
    double last_vtime;
    double accumulated;
    struct ShimPin pins[VIOARM_PINS_PER_CHIP];
    int pins_initialized;
    int halt_logged;
};

struct MultiCosimInst {
    struct ChipState chips[VIOARM_MAX_CHIPS];
    int num_chips;
    double quantum;
};

static int pin_is_open_drain(uint32_t mode)
{
    return mode == VIOARM_MODE_OUTPUT_OPEN_DRAIN ||
           mode == VIOARM_MODE_ALTERNATE_OPEN_DRAIN;
}

static uint32_t xsup_mode_default(void) { return VIOARM_MODE_INPUT_FLOATING; }

/* Poll one chip's pins after a time advance and forward changes;
 * `base_id` is the chip's first global pin ID. */
static void poll_pins(struct co_info *info, struct ChipState *chip,
                      unsigned int base_id)
{
    for (unsigned int id = 0; id < VIOARM_PINS_PER_CHIP; id++) {
        struct ShimPin *pin = &chip->pins[id];
        unsigned int port = id / VIOARM_PINS_PER_PORT;
        unsigned int bit = id % VIOARM_PINS_PER_PORT;

        uint8_t level = 0, driving = 0;
        uint32_t mode = xsup_mode_default();
        int32_t rc = vioarm_cosim_pin_get(chip->arm, port, bit, &level,
                                          &driving, &mode);
        if (rc != 0) {
            /* No such port on this board: park HI-Z once, never ask again. */
            if (!pin->present_probed) {
                pin->present_probed = 1;
                pin->present = 0;
                Digital_t hiz = {ZERO, HI_IMPEDANCE};
                pin->prev = hiz;
                (*info->out_fn)(info, base_id + id, &hiz);
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
            (*info->out_fn)(info, base_id + id, &val);
        }
    }
}

static void step_chip(struct co_info *info, struct ChipState *chip,
                      unsigned int base_id, double quantum)
{
    double delta = info->vtime - chip->last_vtime;
    if (delta <= 0.0)
        return;
    chip->last_vtime = info->vtime;
    chip->accumulated += delta;

    long quanta = 0;
    while (chip->accumulated >= quantum) {
        if (++quanta > VIOARM_MAX_QUANTA_PER_STEP) {
            fprintf(stderr,
                    "[VIOARM-COSIM] quantum backlog exceeded at %.9f s; "
                    "dropping %.9f s (shrink .tran step or grow quantum=)\n",
                    info->vtime, chip->accumulated);
            chip->accumulated = 0.0;
            break;
        }
        int32_t rc = vioarm_cosim_step_duration(chip->arm, quantum);
        chip->accumulated -= quantum;
        if (rc == VIOARM_STEP_HALTED) {
            if (!chip->halt_logged) {
                chip->halt_logged = 1;
                fprintf(stderr, "[VIOARM-COSIM] core halted at %.9f s: %s\n",
                        info->vtime, vioarm_cosim_last_error());
            }
            chip->accumulated = 0.0;
            break;
        }
        if (rc != VIOARM_STEP_OK) {
            fprintf(stderr, "[VIOARM-COSIM] step error at %.9f s: %s\n",
                    info->vtime, vioarm_cosim_last_error());
            chip->accumulated = 0.0;
            break;
        }
    }

    poll_pins(info, chip, base_id);
}

static void shim_step(struct co_info *info)
{
    struct MultiCosimInst *multi = (struct MultiCosimInst *)info->handle;
    if (!multi || multi->num_chips <= 0)
        return;

    /* First step: release every pin to HI-Z (mirrors the AVR shim — pull
     * resistors and external drivers own the bus from time 0). Per
     * instance: ngspice can run several simulations in one process. */
    for (int c = 0; c < multi->num_chips; c++) {
        struct ChipState *chip = &multi->chips[c];
        if (chip->pins_initialized || !chip->arm)
            continue;
        chip->pins_initialized = 1;
        for (unsigned int i = 0; i < VIOARM_PINS_PER_CHIP; i++) {
            Digital_t hiz = {ZERO, HI_IMPEDANCE};
            chip->pins[i].prev = hiz;
            chip->pins[i].prev_driven = 0;
            (*info->out_fn)(info, (unsigned int)(c * VIOARM_PINS_PER_CHIP + i),
                            &hiz);
        }
    }

    for (int c = 0; c < multi->num_chips; c++) {
        struct ChipState *chip = &multi->chips[c];
        if (!chip->arm)
            continue;
        step_chip(info, chip, (unsigned int)(c * VIOARM_PINS_PER_CHIP),
                  multi->quantum);
    }
}

static void shim_input(struct co_info *info, unsigned int bit, Digital_t *value)
{
    struct MultiCosimInst *multi = (struct MultiCosimInst *)info->handle;
    if (!multi)
        return;
    int total = multi->num_chips * VIOARM_PINS_PER_CHIP;
    if (bit >= (unsigned int)total)
        return;
    int chip_idx = (int)bit / VIOARM_PINS_PER_CHIP;
    unsigned int local = bit % VIOARM_PINS_PER_CHIP;
    struct ChipState *chip = &multi->chips[chip_idx];
    if (!chip->arm)
        return;
    unsigned int port = local / VIOARM_PINS_PER_PORT;
    unsigned int pin = local % VIOARM_PINS_PER_PORT;
    uint8_t level = (value->state == ONE) ? 1 : 0;
    /* Driven pins ignore this in the model (the MCU owns the net),
     * exactly like the AVR shim's HI-Z input rule. */
    vioarm_cosim_pin_set(chip->arm, port, pin, level);
}

static void shim_cleanup(struct co_info *info)
{
    struct MultiCosimInst *multi = (struct MultiCosimInst *)info->handle;
    if (multi) {
        for (int c = 0; c < multi->num_chips; c++) {
            if (multi->chips[c].arm)
                vioarm_cosim_destroy(multi->chips[c].arm);
        }
        free(multi);
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

    /* Positional [board, fwfile] pairs first, then key=value options.
     * A lone trailing board/fwfile without its partner is an error. */
    const char *boards[VIOARM_MAX_CHIPS] = {NULL};
    const char *fwpaths[VIOARM_MAX_CHIPS] = {NULL};
    uint32_t format = VIOARM_FW_HEX;
    int forced = 0;
    uint32_t base = 0;
    double quantum = VIOARM_DEFAULT_QUANTUM;
    int nchips = 0;
    int positional = 1;
    int expect_fw = 0;
    for (unsigned int i = 0; i < info->sim_argc; i++) {
        const char *arg = info->sim_argv[i];
        if (!arg)
            continue;
        if (strchr(arg, '=') != NULL)
            positional = 0;
        if (!positional) {
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
                forced = 1;
            }
            continue;
        }
        if (nchips >= VIOARM_MAX_CHIPS) {
            fprintf(stderr,
                    "[VIOARM-COSIM] at most %d chips; ignoring '%s'\n",
                    VIOARM_MAX_CHIPS, arg);
            positional = 0;
            continue;
        }
        /* Pair up: even positionals are boards, odd are firmware. */
        if (!expect_fw) {
            boards[nchips] = arg;
            expect_fw = 1;
        } else {
            fwpaths[nchips] = arg;
            if (!forced)
                format = format_for_path(arg);
            nchips++;
            expect_fw = 0;
        }
    }
    if (expect_fw) {
        fprintf(stderr,
                "[VIOARM-COSIM] dangling board without firmware (need pairs)\n");
        return;
    }
    if (nchips <= 0 || !boards[0] || !fwpaths[0]) {
        fprintf(stderr,
                "[VIOARM-COSIM] sim_args=[\"board\",\"fwfile\",...] required\n");
        return;
    }
    if (!(quantum > 0.0) || quantum > 1.0) {
        fprintf(stderr, "[VIOARM-COSIM] bad quantum, using default\n");
        quantum = VIOARM_DEFAULT_QUANTUM;
    }

    struct MultiCosimInst *multi =
        (struct MultiCosimInst *)calloc(1, sizeof(*multi));
    if (!multi)
        return;
    multi->quantum = quantum;

    for (int c = 0; c < nchips; c++) {
        size_t fw_len = 0;
        uint8_t *fw = read_file(fwpaths[c], &fw_len);
        if (!fw)
            goto fail;
        VioArmHandle *arm =
            vioarm_cosim_create(boards[c], fw, fw_len, format, base);
        free(fw);
        if (!arm) {
            fprintf(stderr, "[VIOARM-COSIM] create failed for chip %d: %s\n",
                    c, vioarm_cosim_last_error());
            goto fail;
        }
        multi->chips[c].arm = arm;
        multi->chips[c].last_vtime = 0.0;
        multi->chips[c].accumulated = 0.0;
        for (unsigned int p = 0; p < VIOARM_PINS_PER_CHIP; p++) {
            multi->chips[c].pins[p].prev.state = ZERO;
            multi->chips[c].pins[p].prev.strength = HI_IMPEDANCE;
        }
        uint32_t hz = vioarm_cosim_cpu_hz(arm);
        fprintf(stderr,
                "[VIOARM-COSIM] chip %d: board=%s cpu=%u Hz quantum=%.3e s\n",
                c, boards[c], hz, quantum);
        multi->num_chips++;
    }

    {
        int total = multi->num_chips * VIOARM_PINS_PER_CHIP;
        info->handle = multi;
        info->in_count = (unsigned int)total;
        info->out_count = (unsigned int)total;
        info->step = shim_step;
        info->in_fn = shim_input;
        info->cleanup = shim_cleanup;
    }
    return;

fail:
    for (int c = 0; c < multi->num_chips; c++) {
        if (multi->chips[c].arm)
            vioarm_cosim_destroy(multi->chips[c].arm);
    }
    free(multi);
}

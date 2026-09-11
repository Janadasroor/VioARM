#include "ngspice/cm.h"
extern void cm_vioarm_adc_bridge(Mif_Private_t *);
extern void cm_vioarm_dac_bridge(Mif_Private_t *);
extern void cm_vioarm_vcc_bridge(Mif_Private_t *);

#include <dlfcn.h>
#include <string.h>

/*
 * cm_vioarm_adc_bridge — XSPICE analog code model
 *
 * Reads analog voltage from circuit node and injects it into the
 * VioARM `apin` window (the N3 lane endpoint) for processing by the
 * emulated ADC firmware.
 *
 * Units: volts on the wire. Millivolt conversion lives in exactly one
 * place — the `vioarm-cosim` C ABI (`set_external_voltage`); this file
 * passes volts straight through.
 *
 * Ports:
 *   in  — analog voltage input (scalar)
 *   out — unity-gain voltage output (mirrors input)
 *
 * Parameters:
 *   channel — kept for netlist compatibility; the N3 seam is a single
 *     window, so only 0 exists (anything else behaves as 0).
 *
 * Lazy-initializes via dlsym to find the active VioARM handle
 * exported by libvioarm_cosim_shim.so (loaded by d_cosim).
 *
 * Netlist usage:
 *   A_arm_adc [adc_node 0] [dummy 0] vioarm_adc_bridge
 *   .model vioarm_adc_bridge vioarm_adc_bridge(channel=0)
 */

/* Function pointer cache (shared across instances) */
static void* (*g_get_handle)(void) = NULL;
static int (*g_set_voltage)(void*, int, double) = NULL;
static void* g_arm_handle = NULL;
static int   g_initialized = 0;

static void try_resolve_symbols(void)
{
    if (g_get_handle) return;

    /* Resolve from any loaded library (RTLD_DEFAULT).
     * libvioarm_cosim_shim.so must be loaded with RTLD_GLOBAL for this
     * to work (d_cosim loads it that way). */
    g_get_handle = (void* (*)(void))dlsym(RTLD_DEFAULT,
                                          "vioarm_cosim_get_active_handle");
    g_set_voltage = (int (*)(void*, int, double))dlsym(
        RTLD_DEFAULT, "vioarm_cosim_set_external_voltage");

    if (g_get_handle) {
        g_arm_handle = g_get_handle();
        if (g_arm_handle) {
            g_initialized = 1;
        }
    }
}

void cm_vioarm_adc_bridge(Mif_Private_t *mif_private)
{
    double voltage;
    int channel;

    /* On first call, try to resolve symbols from the loaded cosim shim. */
    if (!g_initialized) {
        try_resolve_symbols();
    }

    /* Read the analog input voltage */
    voltage = mif_private->conn[0]->port[0]->input.rvalue;

    /* Read channel parameter (single window: anything but 0 is 0). */
    channel = (int)mif_private->param[0]->element[0].rvalue;
    if (channel != 0) channel = 0;

    /* Inject into the apin window (if handle is alive) */
    if (g_initialized && g_arm_handle && g_set_voltage) {
        /* Attempt to get fresh handle each time (handle may change
         * across simulation restarts within the same ngspice session) */
        void* current_handle = g_get_handle ? g_get_handle() : NULL;
        if (current_handle) {
            g_arm_handle = current_handle;
            g_set_voltage(g_arm_handle, channel, voltage);
        }
    }

    /* Output: unity gain (mirror input for Newton coupling) */
    mif_private->conn[1]->port[0]->output.rvalue = voltage;
    mif_private->conn[1]->port[0]->partial[0].port[0] = 1.0;
}

/* ------------------------------------------------------------------ */
/* cm_vioarm_dac_bridge — XSPICE analog code model                       */
/*                                                                    */
/* Reads the most recently staged apin word via the C ABI and presents*/
/* it as an analog node voltage to ngspice. Volts on the wire; the    */
/* millivolt conversion lives in the C ABI, not here.                 */
/*                                                                    */
/* Ports:                                                             */
/*   out — analog voltage output (scalar)                             */
/*                                                                    */
/* Parameters:                                                        */
/*   channel — kept for netlist compatibility; only 0 exists.         */
/*                                                                    */
/* Lazy-initializes via dlsym to find the active VioARM handle        */
/* exported by libvioarm_cosim_shim.so (loaded by d_cosim).           */
/*                                                                    */
/* Netlist usage:                                                     */
/*   A_arm_dac [dac_node 0] vioarm_dac_bridge                            */
/*   .model vioarm_dac_bridge vioarm_dac_bridge(channel=0)                  */
/* ------------------------------------------------------------------ */

/* Function pointer cache for get_dac_voltage */
static int (*g_get_dac_voltage)(void*, int, double*) = NULL;
static int g_dac_initialized = 0;

static void try_resolve_dac_symbols(void)
{
    if (g_get_dac_voltage) return;

    if (!g_get_handle) {
        g_get_handle = (void* (*)(void))dlsym(RTLD_DEFAULT,
                                              "vioarm_cosim_get_active_handle");
    }
    g_get_dac_voltage = (int (*)(void*, int, double*))dlsym(
        RTLD_DEFAULT, "vioarm_cosim_get_dac_voltage");

    if (g_get_handle && g_get_dac_voltage) {
        void* h = g_get_handle();
        if (h) {
            g_arm_handle = h;
            g_initialized = 1;
            g_dac_initialized = 1;
        }
    }
}

void cm_vioarm_dac_bridge(Mif_Private_t *mif_private)
{
    int channel;

    if (!g_dac_initialized) {
        try_resolve_dac_symbols();
    }

    /* Single apin window: only channel 0 exists. */
    channel = (int)mif_private->param[0]->element[0].rvalue;
    if (channel != 0) channel = 0;

    double vout = 0.0;
    /* Held-DAC semantics: firmware stages ~1 word per millisecond while
     * SPICE evaluates this model on every Newton iteration. The C ABI
     * drain consumes, so a naive read would show mostly nothing staged.
     * A real DAC output holds its register — mirror that here: keep the
     * last staged word (NaN sentinel detects "nothing staged", which the
     * C ABI reports by leaving the output untouched). */
    static double held_vout = 0.0;
    double staged = 0.0 / 0.0;
    if (g_dac_initialized && g_arm_handle && g_get_dac_voltage) {
        void* current_handle = g_get_handle ? g_get_handle() : NULL;
        if (current_handle) {
            g_arm_handle = current_handle;
            if (g_get_dac_voltage(g_arm_handle, channel, &staged) == 0 &&
                staged == staged) {
                held_vout = staged;
            }
        }
    }
    vout = held_vout;

    /* Input: dummy read (grounded) */
    double vin = mif_private->conn[0]->port[0]->input.rvalue;
    (void)vin;

    /* Output: DAC voltage */
    mif_private->conn[1]->port[0]->output.rvalue = vout;
    mif_private->conn[1]->port[0]->partial[0].port[0] = 1.0;
}

/* ------------------------------------------------------------------ */
/* cm_vioarm_vcc_bridge — supply-rail mirror for co-simulation          */
/*                                                                     */
/* MIRROR ONLY (P3-3 partial): there is deliberately no injection —   */
/* VioARM models have no operating-voltage concept yet, so there is   */
/* nothing honest to drive (no brown-out, no VDD-coupled ADC). The   */
/* unity-gain mirror keeps netlists forward-compatible: when a core   */
/* VDD hook lands, this is where it plugs in, and existing netlists  */
/* keep working unchanged.                                            */
/*                                                                     */
/* Ports:                                                             */
/*   vcc — analog voltage input (VCC node)                            */
/*   out — unity-gain voltage output (mirrors input for Newton)       */
/*                                                                     */
/* Netlist usage:                                                     */
/*   A_arm_vcc [vcc_node 0] [dummy 0] vioarm_vcc_bridge                  */
/*   .model vioarm_vcc_bridge vioarm_vcc_bridge                             */
/* ------------------------------------------------------------------ */

void cm_vioarm_vcc_bridge(Mif_Private_t *mif_private)
{
    /* Read VCC voltage from analog input port */
    double vcc = mif_private->conn[0]->port[0]->input.rvalue;

    /* No injection point exists yet (see above) — mirror only. */

    /* Output: unity gain mirror for Newton coupling */
    mif_private->conn[1]->port[0]->output.rvalue = vcc;
    mif_private->conn[1]->port[0]->partial[0].port[0] = 1.0;
}

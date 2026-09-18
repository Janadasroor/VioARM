/* Minimal VioARM host (E2 packaging): boot, step 1 ms, read a pin.
 *
 * Firmware: `ldr r0, =0x20000000; movs r1, #0x5A; str r1, [r0]; b .`
 * (hand-assembled Thumb, so no toolchain is needed to build *this* file).
 *
 * Build: cmake -S packaging/examples -B /tmp/host-ex \
 *          -DVioARM_ROOT=$HOME/rust_projects/VioARM/target
 * (or install the lib + header and drop -D: find_package does the rest).
 */
#include <stdint.h>
#include <stdio.h>
#include "vioarm_cosim.h"

static const uint8_t FW[] = {
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00,
    0x01, 0x48, 0x5A, 0x21, 0x01, 0x60, 0xFE, 0xE7,
    0x00, 0x00, 0x00, 0x20,
};

int main(void) {
    VioArmHandle *m = vioarm_cosim_create("virtual", FW, sizeof FW, VIOARM_FW_BIN, 0);
    if (!m) { fprintf(stderr, "create: %s\n", vioarm_cosim_last_error()); return 1; }
    if (vioarm_cosim_step_duration(m, 0.001) < 0) {
        fprintf(stderr, "step: %s\n", vioarm_cosim_last_error());
        return 1;
    }
    uint8_t level, driving;
    uint32_t mode;
    if (vioarm_cosim_pin_get(m, 0, 0, &level, &driving, &mode) < 0) {
        fprintf(stderr, "pin: %s\n", vioarm_cosim_last_error());
        return 1;
    }
    printf("pc=%#x pin0=%u\n", vioarm_cosim_get_pc(m), level);
    vioarm_cosim_destroy(m);
    return 0;
}

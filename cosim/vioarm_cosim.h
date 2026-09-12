/* vioarm_cosim.h — C ABI mirror of the `vioarm-cosim` Rust crate.
 *
 * Hand-written (no cbindgen in the tree): every signature below must match
 * `vioarm-cosim/src/lib.rs` exactly. The `VIOARM_MODE_*` codes are the
 * `GpioPinMode` discriminants (`vioarm-core/src/device.rs`); new modes
 * append at the end so the wire stays backward compatible.
 *
 * Conventions (see the SAFETY CASE in `lib.rs`): handles are
 * single-threaded, `destroy` on null is a no-op, every function fails
 * closed (null / negative status) with detail in `last_error`.
 */

#ifndef VIOARM_COSIM_H
#define VIOARM_COSIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VioArmHandle VioArmHandle;

/* Step outcomes. */
#define VIOARM_STEP_OK   0
#define VIOARM_STEP_HALTED 1
/* Error codes (negative). */
#define VIOARM_ERR_NULL_HANDLE -1
#define VIOARM_ERR_PANIC       -2
#define VIOARM_ERR_BAD_ARG     -3

/* Firmware formats for `vioarm_cosim_create`. */
#define VIOARM_FW_ELF 0
#define VIOARM_FW_HEX 1
#define VIOARM_FW_BIN 2

/* Pin modes for `vioarm_cosim_pin_get`. */
#define VIOARM_MODE_INPUT_FLOATING       0
#define VIOARM_MODE_INPUT_PULL_UP        1
#define VIOARM_MODE_INPUT_PULL_DOWN      2
#define VIOARM_MODE_ANALOG               3
#define VIOARM_MODE_OUTPUT_PUSH_PULL     4
#define VIOARM_MODE_OUTPUT_OPEN_DRAIN    5
#define VIOARM_MODE_ALTERNATE_PUSH_PULL  6
#define VIOARM_MODE_ALTERNATE_OPEN_DRAIN 7

VioArmHandle *vioarm_cosim_create(const char *board, const uint8_t *fw,
                                 size_t fw_len, uint32_t fw_format,
                                 uint32_t bin_base);
void vioarm_cosim_destroy(VioArmHandle *handle);
VioArmHandle *vioarm_cosim_get_active_handle(void);
int32_t vioarm_cosim_step_duration(VioArmHandle *handle, double seconds);
uint32_t vioarm_cosim_cpu_hz(VioArmHandle *handle);
int32_t vioarm_cosim_pin_get(VioArmHandle *handle, uint32_t port, uint32_t pin,
                             uint8_t *level_out, uint8_t *driving_out,
                             uint32_t *mode_out);
int32_t vioarm_cosim_pin_set(VioArmHandle *handle, uint32_t port, uint32_t pin,
                             uint8_t level);
int32_t vioarm_cosim_set_external_voltage(VioArmHandle *handle, int32_t channel,
                                          double volts);
int32_t vioarm_cosim_get_dac_voltage(VioArmHandle *handle, int32_t channel,
                                     double *volts_out);
int32_t vioarm_cosim_set_operating_voltage(VioArmHandle *handle,
                                           double volts);
const char *vioarm_cosim_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* VIOARM_COSIM_H */

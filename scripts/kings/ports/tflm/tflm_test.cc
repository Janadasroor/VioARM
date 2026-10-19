/* TFLM hello_world kings runner: float + int8 sine inference.
 * Follows upstream examples/hello_world/hello_world_test.cc (same models,
 * same FullyConnected-only resolver, same 3 KiB arena, same 4+4 golden
 * inputs vs sinf with eps 0.05), but prints one line per input with the
 * raw prediction bits and a PASS/FAIL verdict instead of using the test
 * framework, ending with DONE + park. Upstream sources untouched. */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/examples/hello_world/models/hello_world_float_model_data.h"
#include "tensorflow/lite/micro/examples/hello_world/models/hello_world_int8_model_data.h"
#include "tensorflow/lite/micro/cortex_m_generic/debug_log_callback.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"

extern "C" {
extern void __libc_init_array(void);
}

namespace {
// Upstream cortex_m_generic log backend delivers formatted lines here.
void uart_log_sink(const char* s) { uart_puts(s); }
}  // namespace

namespace {
using HelloWorldOpResolver = tflite::MicroMutableOpResolver<1>;

void RegisterOps(HelloWorldOpResolver& op_resolver) {
  if (op_resolver.AddFullyConnected() != kTfLiteOk) {
    uart_puts("RESOLVER_FAIL\n");
    for (;;) {
    }
  }
}

uint32_t fbits(float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof(u));
  return u;
}

int failures = 0;

void check(const char* tag, float x, float y_pred) {
  float y_true = sinf(x);
  float err = y_true - y_pred;
  if (err < 0) err = -err;
  int ok = err <= 0.05f;
  if (!ok) failures++;
  uart_puts(tag);
  uart_puts(" xbits=");
  uart_puthex32(fbits(x));
  uart_puts(" ybits=");
  uart_puthex32(fbits(y_pred));
  uart_puts(ok ? " PASS\n" : " FAIL\n");
}
}  // namespace

int main() {
#ifdef HOST_REF
  // Native reference build: libc runtime already live, stdio for UART.
#else
  __libc_init_array();
#endif
  board_init();
  RegisterDebugLogCallback(uart_log_sink);
  tflite::InitializeTarget();
  uart_puts("TFLM HELLO_WORLD\n");

  {
    const tflite::Model* model =
        ::tflite::GetModel(g_hello_world_float_model_data);
    HelloWorldOpResolver op_resolver;
    RegisterOps(op_resolver);
    constexpr int kArenaSize = 3000;
    static uint8_t arena[kArenaSize];
    tflite::MicroInterpreter interpreter(model, op_resolver, arena,
                                         kArenaSize);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
      uart_puts("FLOAT_ALLOC_FAIL\n");
      return 1;
    }
    const float xs[4] = {0.f, 1.f, 3.f, 5.f};
    for (int i = 0; i < 4; i++) {
      interpreter.input(0)->data.f[0] = xs[i];
      if (interpreter.Invoke() != kTfLiteOk) {
        uart_puts("FLOAT_INVOKE_FAIL\n");
        return 1;
      }
      check("F", xs[i], interpreter.output(0)->data.f[0]);
    }
  }

  {
    const tflite::Model* model =
        ::tflite::GetModel(g_hello_world_int8_model_data);
    HelloWorldOpResolver op_resolver;
    RegisterOps(op_resolver);
    constexpr int kArenaSize = 3000;
    static uint8_t arena[kArenaSize];
    tflite::MicroInterpreter interpreter(model, op_resolver, arena,
                                         kArenaSize);
    if (interpreter.AllocateTensors() != kTfLiteOk) {
      uart_puts("INT8_ALLOC_FAIL\n");
      return 1;
    }
    TfLiteTensor* input = interpreter.input(0);
    const TfLiteTensor* out = interpreter.output(0);
    float output_scale = out->params.scale;
    int output_zero_point = out->params.zero_point;
    // Upstream goldens: (x / input_scale + input_zero_point).
    const float xs[4] = {0.77f, 1.57f, 2.3f, 3.14f};
    const int8_t xq[4] = {-96, -63, -34, 0};
    for (int i = 0; i < 4; i++) {
      input->data.int8[0] = xq[i];
      if (interpreter.Invoke() != kTfLiteOk) {
        uart_puts("INT8_INVOKE_FAIL\n");
        return 1;
      }
      float y_pred =
          (static_cast<float>(out->data.int8[0]) - output_zero_point) *
          output_scale;
      check("Q", xs[i], y_pred);
    }
  }

  uart_puts(failures == 0 ? "TFLM_ALL_PASS\n" : "TFLM_FAILURES\n");
  uart_puts("TFLM_DONE\n");
#ifdef HOST_REF
  // The x86 reference build must terminate: it is diffed against the
  // device's UART capture, so parking here would hang the CI job.
  return failures == 0 ? 0 : 1;
#else
  for (;;) {
  }
  return 0;
#endif
}

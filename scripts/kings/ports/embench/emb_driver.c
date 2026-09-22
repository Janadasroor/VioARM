/* Embench-IoT generic driver: init, warmup, one timed benchmark() call,
 * verify, report. Matches the scoreboard method (correct=1, cycles table).
 * Compile with -DBENCH="name". */
#include "board.h"

extern void initialise_benchmark(void);
extern int benchmark(void);
extern int verify_benchmark(int r);

int main(void) {
    uint32_t t0, t1;
    int r, ok;
    board_init();
    uart_puts("BENCH ");
    uart_puts(BENCH);
    uart_puts("\n");
    initialise_benchmark();
    benchmark(); /* warmup */
    t0 = cycnt();
    r = benchmark();
    t1 = cycnt();
    ok = verify_benchmark(r);
    uart_puts("cycles=");
    uart_putdec32(t1 - t0);
    uart_puts(" correct=");
    uart_putdec32((uint32_t)ok);
    uart_puts("\nDONE\n");
    for (;;) {}
    return 0;
}

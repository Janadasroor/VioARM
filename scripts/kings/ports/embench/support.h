/* Minimal stand-in for Embench's generated support/config headers.
 * Upstream convention (benchmark_speed.py default): GLOBAL_SCALE_FACTOR 1.
 * Only what the 19 benchmarks reference. */
#ifndef EMBENCH_PORT_SUPPORT_H
#define EMBENCH_PORT_SUPPORT_H
#define GLOBAL_SCALE_FACTOR 1
#define CPU_MHZ 8
#endif
/* Upstream beebsc PRNG signatures (support/beebsc.h); linked from upstream
 * support/beebsc.c so benchmark sequences are canonical. */
int rand_beebs(void);
void srand_beebs(unsigned int seed);
/* Upstream support/beebsc.h line 39, verbatim (some benches call it
 * without including beebsc.h; the macro makes the call site correct). */
#define assert_beebs(expr) { if (!(expr)) while (1); }

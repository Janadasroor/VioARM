#!/usr/bin/env python3
"""Port Dhrystone 2.1 (netlib) to bare-metal stm32f103.
Usage: patch_dhry.py <dhry_1.c> <out>. Upstream stays pristine.
Patches, all confined to main():
  1. board_init() FIRST (clocks+USART+CYCCNT before any printf — with
     UE=0 the TXE poll hangs, and CYCCNT never counts).
  2. scanf run count -> fixed 100000 (no stdin on target).
  3. times()/time() -> DWT CYCCNT; HZ=8000000 via -D; Too_Small_Time =
     2 s in cycles (keeps the "meaningful" gate honest).
  4. %f result math -> integer tenths (nano -u _printf_float _dtoa costs
     ~400M sim instructions; identical values, hand-placed point) + DMIPS.
  5. fflush + DONE marker + park loop (verifiable completion).
All 20 "should be" self-checks and every Proc_/Func_ call are untouched.
"""
import os
import sys

src = open(sys.argv[1]).read()

header_old = """  printf ("\\n");
  printf ("Dhrystone Benchmark, Version 2.1 (Language: C)\\n");"""
header_new = """  board_init (); /* FIRST: clocks + USART + CYCCNT before any printf */
  printf ("\\n");
  printf ("Dhrystone Benchmark, Version 2.1 (Language: C)\\n");"""
assert header_old in src, 'header anchor'
src = src.replace(header_old, header_new)

scanf_old = """  printf ("Please give the number of runs through the benchmark: ");
  {
    int n;
    scanf ("%d", &n);
    Number_Of_Runs = n;
  }"""
scanf_new = """  printf ("Bare-metal port: fixed 100000 runs (no stdin)\\n");
  Number_Of_Runs = 100000;"""
assert scanf_old in src
src = src.replace(scanf_old, scanf_new)

begin_old = """#ifdef TIMES
  times (&time_info);
  Begin_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
  Begin_Time = time ( (long *) 0);
#endif"""
assert begin_old in src
src = src.replace(begin_old, "  Begin_Time = (long) cycnt ();")

end_old = """#ifdef TIMES
  times (&time_info);
  End_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
  End_Time = time ( (long *) 0);
#endif"""
assert end_old in src
src = src.replace(end_old, "  End_Time = (long) cycnt ();")

inc_old = '#include "dhry.h"'
inc_new = ('#include "dhry.h"\n#include "board.h"\n#include <string.h>\n#include <stdio.h>\n'
           '#undef TIMES /* dead decl conflicts with newlib; we use CYCCNT */\n'
           '#undef Too_Small_Time\n#define Too_Small_Time (2*8000000)')
assert inc_old in src
src = src.replace(inc_old, inc_new, 1)

float_old = """#ifdef TIME
    Microseconds = (float) User_Time * Mic_secs_Per_Second 
                        / (float) Number_Of_Runs;
    Dhrystones_Per_Second = (float) Number_Of_Runs / (float) User_Time;
#else
    Microseconds = (float) User_Time * Mic_secs_Per_Second 
                        / ((float) HZ * ((float) Number_Of_Runs));
    Dhrystones_Per_Second = ((float) HZ * (float) Number_Of_Runs)
                        / (float) User_Time;
#endif
    printf ("Microseconds for one run through Dhrystone: ");
    printf ("%6.1f \\n", Microseconds);
    printf ("Dhrystones per Second:                      ");
    printf ("%6.1f \\n", Dhrystones_Per_Second);
    printf ("\\n");"""
float_new = """    {
      long long dps = (8000000LL * (long long) Number_Of_Runs)
                      / (long long) User_Time;
      long long us10 = ((long long) User_Time * 10LL)
                       / (8000000LL * (long long) Number_Of_Runs / 1000000LL);
      long long dmips1000 = dps * 1000LL / 1757LL;
      printf ("Microseconds for one run through Dhrystone: ");
      printf ("%ld.%ld \\n", (long)(us10 / 10LL), (long)(us10 % 10LL));
      printf ("Dhrystones per Second:                      ");
      printf ("%ld \\n", (long)dps);
      printf ("DMIPS (dps/1757, x1000):                    ");
      printf ("%ld.%03ld \\n", (long)(dmips1000 / 1000LL),
              (long)(dmips1000 % 1000LL));
      printf ("\\n");
    }"""
assert float_old in src
src = src.replace(float_old, float_new)

tail_old = "\n}\n\n\nProc_1 (Ptr_Val_Par)"
tail_new = ('\n  fflush (stdout); /* nano buffers libc output; no exit() runs */\n'
            '  uart_puts("DHRY_DONE\\n");\n  for (;;) {}\n}\n'
            '\n\nProc_1 (Ptr_Val_Par)')
assert tail_old in src
src = src.replace(tail_old, tail_new)

open(sys.argv[2], 'w').write(src)
print('patched ok')

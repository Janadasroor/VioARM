# TI audit: analog / timer / memory (SPMS376E §§7–13, §19–21)

Source PDF: `datasheets/spms376e.pdf`.
Date: 2026-09-25. Status legend: `[ ]` open, `[x]` fixed + regression-tested.

## Watchdog (`vioarm-core/src/device/ti/wdt.rs`, §12)

- [x] `wdt.rs:44`, `WDTLOCK` reset: we stamp `0x0000_0001` locked; §12
      `WDTLOCK` Type RW reset `0x0000_0000` (unlocked, p786) —
      **wrong-behavior**
- [x] `wdt.rs:173`, `WDTTEST` lock: we block all but `ICR`/`LOCK` when locked;
      §12 p786 lock excepts `WDTTEST` (and `WDTICR` always services p778) —
      **wrong-behavior**

## Flash (`vioarm-core/src/device/ti/flash.rs`, §8)

- [x] `flash.rs:154`, `FMC.WRKEY`: we accept only `0xA442`; FMC p544 `0xA442`
      or `0x71D5` per `BOOTCFG.KEY`, else ignored — **wrong-behavior**

## EEPROM (`vioarm-core/src/device/ti/eeprom.rs`, §8)

- [x] `eeprom.rs:78`, `EEOFFSET` on `NOPERM`: we return without inc; §8
      `EERDWRINC`: "In all cases/Regardless of error OFFSET incremented" —
      **wrong-behavior**
- [x] `eeprom.rs:115`, `EERDWRINC` read: we return word no inc; §8
      `EERDWRINC` read-or-write then inc `OFFSET` — **wrong-behavior**

## ADC (`vioarm-core/src/device/ti/adc.rs`, §13)

- [x] `adc.rs:260`, `ADCOSTAT`/`ADCUSTAT`: generic `merged/set`; §13 Type
      `RW1C` "cleared by writing 1" — **missing-latch-needed**
- [x] `adc.rs:161`, `ADCSSFSTATn.HPTR/TPTR`: we synthesize `len|len<<4`; §13
      p862 head=next-write, tail=next-read pointers, reset `0x100`, diverge
      on wrap/pop — **wrong-behavior**
- [x] `adc.rs:246`, `ADCPSSI` order: we run `0..3` fixed; §13 p845
      simultaneous triggers follow `ADCSSPRI` encodings — **wrong-behavior**

## Hibernate (`vioarm-core/src/device/ti/hib.rs`, §7)

- [x] `hib.rs:86`, `HIBRIS.RTCALT0`: we fire on `RTCC==RTCM0`; §7 p516 fire
      only if `RTCC==RTCM0` and `RTCSSC==RTCSSM` — **wrong-behavior**

## Timer (`vioarm-core/src/device/ti/timer.rs`, §11)

- [x] `timer.rs:59`, `GPTMTBILR/TBMATCHR/TBR/TBV` resets: we stamp only
      `TAILR/TAMATCHR/TAR/TAV=0xFFFF_FFFF`; §11 p757 `TBILR
      0x0000_FFFF/0xFFFF_FFFF wide`, same pattern `TBMATCHR/TBR/TBV`, left at
      SVD `0` — **missing-latch-needed**

## GPIO (`vioarm-core/src/device/ti/gpio.rs`, §10)

- [x] ~~`gpio.rs:409`, `GPIOCR` relock~~ — INVALID FINDING (overturned
      2026-09-25 against the primary source): p684 LOCK field states "a
      write of any other value **or a write to the GPIOCR register
      reapplies the lock**". The original code was correct; the change was
      reverted and the test restored to unlock → commit (relocks) →
      `AFSEL` lands while locked.
- [x] `gpio.rs:152`, `GPIOPUR/PDR`: no mutual clear; §10 p677/p679 setting
      `PUR` clears `PDR` and converse — **wrong-behavior**
- [x] `gpio.rs:64`, `GPIOA/B PCTL` reset `0`: Table 10-5 p659
      `PA[1:0]=0x1, PA[5:2]=0x2, PB[3:2]=0x3` (don't-care while `AFSEL=0`) —
      **cosmetic**

## PWM (`vioarm-core/src/device/ti/pwm.rs`, §20)

- [x] `pwm.rs:32`, Table cite: we cite "Table 20-7"; §20 register map is Table
      20-2 p1240 (stride `0x040+0x40n` itself correct) — **cosmetic**

## Comparator (`vioarm-core/src/device/ti/comp.rs`, §19)

- [x] `comp.rs:197`, `ACCTLn` write `poll()`: config change can latch edge
      with no output transition; §19 edge sense only on output transition —
      **wrong-behavior**

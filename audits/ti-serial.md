# TI audit: serial peripherals (SPMS376E §§14–18)

Source PDF: `datasheets/spms376e.pdf`.
Date: 2026-09-25. Status legend: `[ ]` open, `[x]` fixed + regression-tested.

## UART (`vioarm-core/src/device/ti/uart.rs`, §14 Table 14-3)

- [x] `uart.rs:83` `UART9BITAMASK/UARTPeriphIDn/PCellIDn` — we stamp only
      `FR/CTL/IFLS/UARTPP`, leave rest 0 — Table 14-3 p904: `9BITAMASK=0x00FF`,
      `PeriphID0=0x60,ID2=0x18,ID3=0x01,PCellID0=0x0D,ID1=0xF0,ID2=0x05,
      ID3=0xB1` — **wrong-behavior**
- [x] `uart.rs:93` `UARTIBRD/FBRD/HSE` — we do `clk/(16*(IBRD+FBRD/64))`
      always — §14.3.2 p896: `BRD=UARTSysClk/(ClkDiv*Baud)`,
      `ClkDiv=16 HSE=0 /8 HSE=1` — **wrong-behavior**
- [x] `uart.rs:209` `UARTFR.BUSY` — we set iff `tx.non_empty()`, instant drain
      => always 0 — p911-912: set on write to TX FIFO, clear only when FIFO
      empty *and* shifter incl. stop bits done — **missing-latch-needed**
- [x] `uart.rs:261` `UARTICR.OEIC/BEIC/PEIC/FEIC` — we clear sticky `errors`
      (=`RSR`) — p908 vs p933: `RSR` cleared only by `ECR` any-write, `RIS`
      cleared by `ICR` W1C, separate latches — **wrong-behavior**
- [x] `uart.rs:379` `UARTDR[11:8]` — we `popped_flags=0` always, lane1 always
      0 — p906-907: `OE/BE/PE/FE` associated with popped char, `RSR` reflects
      prior `DR` — **missing-latch-needed**

## SSI (`vioarm-core/src/device/ti/ssi.rs`, §15)

- [x] `ssi.rs:62` `SSIPeriphID0/2/3/PCellID0-3` — we stamp only `SR=0x03,
      RIS=0x08` — Table 15-2 p967: `ID0=0x22,ID2=0x18,ID3=0x01,PCell0=0x0D,
      Cell1=0xF0,Cell2=0x05,Cell3=0xB1` — **cosmetic**
- [x] `ssi.rs:100` `SSICR0.DSS` — we map reserved `0x0-0x2` to 16bit — p970:
      `0x0-0x2 Reserved` — **cosmetic**
- [x] `ssi.rs:212` `SSICPSR[31:8]` — we `&!0xFF000001` keeps bits 8-23
      writable — p976: `31:8 RO 0`, only `7:0 CPSDVSR`, LSB hard-0 —
      **wrong-behavior**
- [x] `ssi.rs:164` `SSISR.BSY` — we set iff `tx.non_empty()`, instant drain
      => 0 — p974: `1` while Tx/Rx *or* TX FIFO non-empty (shifter hold after
      FIFO empty) — **missing-latch-needed**
- [x] `ssi.rs:192` `SSIDR` — we push only on `shift==8` full-word store —
      p973: 16-bit `DR`, right-justified `<16b`, TivaWare word-store assumed,
      byte/halfword lost — **cosmetic**

## I2C (`vioarm-core/src/device/ti/i2c.rs`, §16)

- [x] `i2c.rs:152` `I2CMCS[3:0]/ACK Table 16-5` — we `let _=ack`, treat Illegal
      as legal — p1023-1024: e.g. Idle `R/S=1 ACK=1 STOP=1 START=1 RUN=1`
      Illegal, Master-Rx `ACK=1 STOP=1 START=0 RUN=1` Illegal/NOP —
      **wrong-behavior**

## CAN (`vioarm-core/src/device/ti/can.rs`, §17)

- [x] `can.rs:159` `CANIFnCRQ/MSK` reset — we `crq=0,msk=0` — Table 17-5 p1067:
      `CRQ=0x01`, `MSK1/MSK2=0xFFFF` (`MXTD/MDIR=1`) — **wrong-behavior**
- [x] `can.rs:664` `CANBRPE` — we always writable — p1080: write-enabled by
      `CCE` (cf `CANBIT` p1076 `CCE&&INIT` which we do gate) —
      **wrong-behavior**
- [x] `can.rs:649` `CANSTS.TXOK/RXOK/LEC` — we `sts=(sts&mask)|w` allows
      set-by-1 — p1073: `TXOK/RXOK` must be cleared by 0, `LEC=0x7` manual
      invalid, write does not generate int — **wrong-behavior**

## USB (`vioarm-core/src/device/ti/usb.rs`, §18)

- [x] `usb.rs:136` `USBFSEOF/USBLSEOF` — we don't stamp, read 0 — Table 18-5
      p1115: `FSEOF=0x77,LSEOF=0x72` — **wrong-behavior**
- [x] `usb.rs:161` `USBRXMAXPn` — we `maxp()` reads `TXMAXP` for RX `FULL`/cap
      — p1157/1175: separate `TXMAXP/RXMAXP MAXLOAD[10:0]` — **wrong-behavior**
- [x] `usb.rs:161` `USBMAXP==0` — we default to 64 — p1157 reset `0x0000`, EP0
      64B reserved but EP1-7 `0`≠64 — **cosmetic**
- [x] `usb.rs:91` `USBRXCSRLn.STALL` — we define/return `RX_STALL=1<<6` —
      Device p1178: `STALL=5,STALLED=6,CLRDT=7 W1C,DATAERR=3 RO,OVER=2,
      FULL=1 RO` — firmware `STALL(5)` ignored — **wrong-behavior**
- [x] `usb.rs:196` `TXCSRL.STALLED/CSRL0.STALLED/DATAEND` — we never set
      `STALLED`, never auto-clear `DATAEND` — p1160-1161/1169: `STALL`
      auto-cleared after handshake, `STALLED` set+SW-clear, `DATAEND`
      auto-cleared — **missing-latch-needed**
- [x] `usb.rs:339` `USB RESET` — we clear `FADDR/FIFO/TXIS/RXIS/IS.RESET`
      only — §18.3.1.8 p1106: also clear `EPIDX`, all `CSR`, enable all EP
      ints — **missing-latch-needed**
- [x] `usb.rs:393` `USBRXCSRLn.RXRDY/AUTOCLR` — we never auto-clear, manual
      write-0 only — p1180: if `AUTOCLR` set and `MAXP` bytes unloaded,
      auto-clear — **missing-latch-needed**

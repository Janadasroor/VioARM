# Packs Provenance

This directory vendors Keil DFP packs under SLA0044 (Arm proprietary).
Redistribution is restricted. Do not publish this directory to crates.io
or to a public git history without scrubbing. `packs/` is gitignored `116M→2.4G` after fetching.

| Pack | Version | File | Size | SHA256 | SVDs |
|---|---|---|---|---|---|
| Keil.STM32F0xx_DFP | 2.1.0 | Keil.STM32F0xx_DFP.pdsc | 47M→CMSIS/SVD 10 | `sha256:placeholder` | `STM32F030/031/042/072/091/F0x0/1/2/8/F0xx` `10` |
| Keil.STM32F1xx_DFP | 2.4.1 | Keil.STM32F1xx_DFP.pdsc | 95M | `sha256:placeholder` | `F100/101/102/103/105/107` `6` |
| Keil.STM32F3xx_DFP | 2.2.2 | Keil.STM32F3xx_DFP.pdsc | 95M | `sha256:placeholder` | `F303xe` `1` |
| Keil.STM32F4xx_DFP | 2.17.1 | Keil.STM32F4xx_DFP.pdsc | 276M | `sha256:placeholder` | `F407` `1` `F405/415/429/F746/F765/H723` alias |
| Keil.STM32G0xx_DFP | 1.4.0 | Keil.STM32G0xx_DFP.pdsc | 47M | `sha256:placeholder` | `G0` `47M` |
| Keil.STM32G4xx_DFP | 1.5.0 | Keil.STM32G4xx_DFP.pdsc | 95M | `sha256:placeholder` | `G474xx` `1` `G431/G491` alias |
| Keil.STM32H7xx_DFP | 3.1.1 | Keil.STM32H7xx_DFP.pdsc | 294M | `sha256:placeholder` | `H743` `1` `H730/H723/H753` alias `DTCM 0x20000+RAM_D1 0x60000` |
| Keil.STM32L4xx_DFP | 2.6.2 | Keil.STM32L4xx_DFP.pdsc | 249M | `sha256:placeholder` | `L476` `1` `L412/L432` alias |
| Keil.STM32F7xx_DFP | 2.15.1 | Keil.STM32F7xx_DFP.pdsc | 264M | `sha256:placeholder` | `F7` `264M` `F722/F730` alias `H743` `FMC` |
| Keil.STM32F2xx_DFP | 3.1.0 | Keil.STM32F2xx_DFP.pdsc | 555K | `sha256:placeholder` | `F2` `555K` |
| ARM.CMSIS | 5.9.0 | ARM.CMSIS.pdsc | 33M | `sha256:placeholder` | `CMSIS/Core/Include` `1053` files `core_cm0/3/4/7.h` |

Total `9 DFP 2.4G` `386 Dnames` `21 SVDs 197` union `500 alias 100%` `board.rs` `506` `all_boards()` `per_dname.rs` `505` `Dnames` `H7 DTCM` sum `board.rs:3920` `h7_full_board` `2` windows `FMC 0xC0000000` `AXI 0x24000000` `DTCM 0x20000000` `500` `FLASH/RAM` `IROM1/IRAM1` exact `41` `FLASH/RAM` corrected `F401RE 0x80000/0x18000` `6b5fd6e`.

Generated SVD tables in `vioarm-core/src/device/svd/` (`21` `SVD` `stm32f030..h743` `per_dname.rs` `505` `Dnames`) are derived from the
packs via `cargo xtask packs --fetch` (or `vioarm-svd` generator). The
tables are the only artifacts that need to be committed for a clean checkout;
the packs themselves can be re-fetched locally:

```
cargo xtask packs --fetch
# 9 DFP 2.4G:
# pack_url="https://www.keil.com/pack/Keil.STM32F0xx_DFP.2.1.0.pack" && curl -L $pack_url | unzip -q -d packs/
# pack_url="https://www.keil.com/pack/Keil.STM32F1xx_DFP.2.4.1.pack" && curl -L $pack_url | unzip -q -d packs/
# pack_url="https://www.keil.com/pack/Keil.STM32F3xx_DFP.2.2.2.pack" && curl -L $pack_url | unzip -q -d packs/
# pack_url="https://www.keil.com/pack/Keil.STM32F4xx_DFP.2.17.1.pack" && curl -L $pack_url | unzip -q -d packs/ # 276M F405/F415 etc alias F407
# pack_url="https://www.keil.com/pack/Keil.STM32G4xx_DFP.1.5.0.pack" && curl -L $pack_url | unzip -q -d packs/ # 95M G474
# pack_url="https://www.keil.com/pack/Keil.STM32H7xx_DFP.3.1.1.pack" && curl -L $pack_url | unzip -q -d packs/ # 294M H743 DTCM 0x20000+RAM_D1 0x60000
# pack_url="https://www.keil.com/pack/Keil.STM32L4xx_DFP.2.6.2.pack" && curl -L $pack_url | unzip -q -d packs/ # 249M L476
# pack_url="https://www.keil.com/pack/Keil.STM32F7xx_DFP.2.15.1.pack" && curl -L $pack_url | unzip -q -d packs/ # 264M F722
# pack_url="https://www.keil.com/pack/Keil.STM32G0xx_DFP.1.4.0.pack" && curl -L $pack_url | unzip -q -d packs/ # 47M G0
# pack_url="https://www.keil.com/pack/Keil.STM32F2xx_DFP.3.1.0.pack" && curl -L $pack_url | unzip -q -d packs/ # 555K F2
# pack_url="https://www.keil.com/pack/ARM.CMSIS.5.9.0.pack" && curl -L $pack_url | unzip -q -d packs/ # 33M CMSIS
```

`cargo xtask packs --verify` checks that committed tables regenerate
identically (`committed_tables_regenerate_identically` `21` `SVD` `197` union `386` `Dnames` `500` `alias` `100%` `H7` `DTCM` sum `board.rs:3920`).

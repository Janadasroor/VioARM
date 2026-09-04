# Packs Provenance

This directory vendors Keil DFP packs under SLA0044 (Arm proprietary).
Redistribution is restricted. Do not publish this directory to crates.io
or to a public git history without scrubbing. `packs/` is gitignored `116M→2.4G` after fetching.

| Pack | Version | File | Size | SHA256 | SVDs |
|---|---|---|---|---|---|
| Keil.STM32F0xx_DFP | 2.1.0 | Keil.STM32F0xx_DFP.2.1.0.pack | 47M→CMSIS/SVD 10 | `sha256:ba8a448690a4cb7a0dd7284e179f93256e6d7b7710e4e8ca75b360a2456b8684` | `STM32F030/031/042/072/091/F0x0/1/2/8/F0xx` `10` |
| Keil.STM32F1xx_DFP | 2.4.1 | Keil.STM32F1xx_DFP.2.4.1.pack | 95M | `sha256:807ea15da5b172b916bbc47b2b87f1e621240ad208d38e82a417a2ef8191e9d1` | `F100/101/102/103/105/107` `6` |
| Keil.STM32F3xx_DFP | 2.2.2 | Keil.STM32F3xx_DFP.2.2.2.pack | 95M | `sha256:47a41da7268a2cba41eab8a2f6222ac76aad6cc67afc2f892e9069c46e038e30` | `F303xe` `1` |
| Keil.STM32F4xx_DFP | 2.17.1 | Keil.STM32F4xx_DFP.2.17.1.pack | 276M | `sha256:c7e0f3595aa5ee4c2da2c0de79598b8a91fc5f4f77e16c3b39c674429fb0f4eb` | `F407` `1` `F405/415/429/F746/F765/H723` alias |
| Keil.STM32G0xx_DFP | 1.4.0 | Keil.STM32G0xx_DFP.pdsc (extracted, pack not retained) | 47M | `sha256:126d39f56f80079c7ce13620ed1c1e8e943427f10650c1b44c1404123b5e1e22` (pdsc) | `G0` `47M` |
| Keil.STM32G4xx_DFP | 1.5.0 | Keil.STM32G4xx_DFP.1.5.0.pack | 95M | `sha256:5c878bb8b2de2120153b9570e789ea216b9a1a0903dbe1e154ceb1ff0a4d2972` | `G474xx` `1` `G431/G491` alias |
| Keil.STM32H7xx_DFP | 3.1.1 | Keil.STM32H7xx_DFP.3.1.1.pack | 294M | `sha256:53ef531aa7ec36ec5b9c489789fdeda6de5abeb3f9c1b91fb1d96f941b98bbd7` | `H743` `1` `H730/H723/H753` alias `DTCM 0x20000+RAM_D1 0x60000` |
| Keil.STM32L4xx_DFP | 2.6.2 | Keil.STM32L4xx_DFP.2.6.2.pack | 249M | `sha256:28c4985f0d0fcbcf2e97eea1cc496d0f243e8a54955008ed32e08c45c47c7c5b` | `L476` `1` `L412/L432` alias |
| Keil.STM32F7xx_DFP | 2.15.1 | Keil.STM32F7xx_DFP.pdsc (extracted, pack not retained) | 264M | `sha256:299eee6a35991383b00a3736cfa1209793078ebe57d616033080bf394501e0c5` (pdsc) | `F7` `264M` `F722/F730` alias `H743` `FMC` |
| Keil.STM32F2xx_DFP | 3.1.0 | Keil.STM32F2xx_DFP.3.1.0.pack | 555K | `sha256:0c36d60606fd52d559d7527ac780c9021014ab50c69418ec810a40ed2d7b6ae5` | `F2` `555K` |
| Keil.STM32U5xx_DFP | 3.2.0 | Keil.STM32U5xx_DFP.3.2.0.pack | 8.5M slim (SVD+Flash, no HAL) | `sha256:e320687fe534f2fe6902e9bcdee981315abea26c4ca547142af9b9439e958be6` | `U535/U545/U575/U585/U595/U599/U5A5/U5A9/U5F7/U5F9/U5Fx/U5G7/U5G9/U5Gx/U5xx` `15` |
| Keil.STM32H5xx_DFP | 2.3.1 | Keil.STM32H5xx_DFP.2.3.1.pack | 6.3M slim | `sha256:906b2f8802845ed21c4bc31512a816065991bd9839f1441d0126a667a50347b1` | `H503/H523/H533/H543/H553/H562/H563/H573/H5E4/H5E5/H5F4/H5F5` `12` |
| Keil.STM32WBAxx_DFP | 2.2.0 | Keil.STM32WBAxx_DFP.2.2.0.pack | 2.7M slim | `sha256:629532e024093307f0a46e47abbf1fe7826923720d9ad108684b0955316ea51a` | `WBA23/25/50/52/54/55/62/63/64/65/6M` `11` |
| Keil.STM32L5xx_DFP | 2.1.0 | Keil.STM32L5xx_DFP.2.1.0.pack | 968K slim | `sha256:190ebdd8615107f9be69cd7a8bd9710e4c4240ed69c5886a5d7694a6d8c74c31` | `L552/L562` `2` |
| ARM.CMSIS | 5.9.0 | ARM.CMSIS.5.9.0.pack | 33M | `sha256:14b366f2821ee5d32f0d3bf48ef9657ca45347261d0531263580848e9d36f8f4` | `CMSIS/Core/Include` `1053` files `core_cm0/3/4/7.h` |

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

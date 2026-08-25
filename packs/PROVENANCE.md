# Packs Provenance

This directory vendors Keil DFP packs under SLA0044 (Arm proprietary).
Redistribution is restricted. Do not publish this directory to crates.io
or to a public git history without scrubbing.

| Pack | Version | File | SHA256 |
|---|---|---|---|
| Keil.STM32F1xx_DFP | 2.4.1 | Keil.STM32F1xx_DFP.pdsc | `sha256:placeholder` |
| Keil.STM32F4xx_DFP | 2.17.1 | Keil.STM32F4xx_DFP.pdsc | `sha256:placeholder` |
| Keil.STM32H7xx_DFP | 3.1.1 | Keil.STM32H7xx_DFP.pdsc | `sha256:placeholder` |

Generated SVD tables in `vioarm-core/src/device/svd/` are derived from the
packs via `cargo xtask packs --fetch` (or `vioarm-svd` generator). The
tables are the only artifacts that need to be committed for a clean checkout;
the packs themselves can be re-fetched locally:

```
cargo xtask packs --fetch
# or manually:
# pack_url="https://www.keil.com/pack/Keil.STM32F1xx_DFP.2.4.1.pack"
# curl -L $pack_url | unzip -q -d packs/
```

`cargo xtask packs --verify` checks that committed tables regenerate
identically (`committed_tables_regenerate_identically` test).

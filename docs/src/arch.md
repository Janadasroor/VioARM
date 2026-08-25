# Architecture

- `Machine` + `SystemMemory` + `FlatMemory` + `Peripheral` trait
- `RegisterFile` + `Behavior` + SVD tables (`vioarm-svd`)
- `DWT` `cyccnt`, `SCB` `MPU` `TrustZone`, `FPU` softfloat
- Determinism: single-threaded, `BTreeMap`, no `HashMap` in state

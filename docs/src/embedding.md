# Embedding

```rust
use vioarm_core::{board, Machine, loader::load_elf};
let mut m = Machine::armv7m(board::stm32f103());
let img = load_elf(&elf_bytes).unwrap();
m.load_image(&img);
m.reset().unwrap();
m.run(1_000_000);
```

Implement `Peripheral` trait for custom devices and `attach` to `SystemMemory`.

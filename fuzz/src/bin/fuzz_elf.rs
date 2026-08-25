fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    // Real fuzz target would call vioarm_core::loader::load_elf(&buf);
    std::hint::black_box(buf);
}

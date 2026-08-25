fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    let _ = vioarm_core::loader::load_flat(0x0800_0000, &buf);
    std::hint::black_box(buf);
}

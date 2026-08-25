fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    let _ = vioarm_core::loader::load_hex(&buf);
    std::hint::black_box(buf);
}

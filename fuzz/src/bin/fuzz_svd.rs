fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    // CMSIS-SVD XML through the table parser; the model is dropped (parse
    // must be total — any panic is a finding). Non-UTF8 input is lossy
    // converted the way a truncated file read would present it.
    let text = String::from_utf8_lossy(&buf);
    let _ = vioarm_svd::svd::parse(&text);
    std::hint::black_box(buf);
}

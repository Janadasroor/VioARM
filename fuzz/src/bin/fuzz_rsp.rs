fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    // Raw RSP-stream bytes through packet framing and request dispatch
    // against a scratch machine; replies discarded, resumes bounded to one
    // step each. Any panic is a finding (parse paths must be total).
    vioarm_gdb::fuzz_stream(&buf);
    std::hint::black_box(buf);
}

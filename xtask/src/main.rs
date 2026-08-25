fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--fetch") {
        println!("Fetching packs: see packs/PROVENANCE.md for URLs");
        return;
    }
    if args.iter().any(|a| a == "--verify") {
        println!("Verify: cargo test committed_tables_regenerate_identically");
        return;
    }
    println!("Usage: cargo xtask packs --fetch | --verify");
}

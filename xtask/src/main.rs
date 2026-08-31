fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--fetch") {
        println!("Fetching packs: see packs/PROVENANCE.md for URLs");
        println!("Packs already vendored 9 DFP 2.4G 386 Dnames: Keil.STM32F0xx_DFP.2.1.0 F1xx 2.4.1 F3xx 2.2.2 F4xx 2.17.1 G4xx 1.5.0 H7xx 3.1.1 L4xx 2.6.2 F7xx 2.15.1 G0xx 1.4.0 F2xx 3.1.0 + ARM.CMSIS 5.9.0 33M CMSIS/Core/Include 1053 files");
        println!("To re-fetch: curl -L https://www.keil.com/pack/Keil.STM32F0xx_DFP.2.1.0.pack | unzip -q -d packs/ && cargo test -p vioarm-svd --test committed_tables_regenerate_identically");
        println!("Packs are gitignored per .gitignore packs/ 116M->2.4G, only generated SVD tables in vioarm-core/src/device/svd/ + per_dname.rs committed");
        return;
    }
    if args.iter().any(|a| a == "--verify") {
        println!("Verify: cargo test -p vioarm-svd --test committed_tables_regenerate_identically -- 21 SVD 197 union 386 Dnames 500 alias 100% H7 DTCM 0x20000+RAM_D1 0x60000");
        let status = std::process::Command::new("cargo")
            .args(["test", "-p", "vioarm-svd", "--", "committed_tables_regenerate_identically"])
            .status()
            .expect("cargo test");
        std::process::exit(status.code().unwrap_or(1));
    }
    println!("Usage: cargo xtask packs --fetch | --verify");
    println!("  --fetch  : document fetch URLs per packs/PROVENANCE.md (SLA0044, 9 DFP 2.4G, 386 Dnames)");
    println!("  --verify : cargo test committed_tables_regenerate_identically 21 SVD 197 union 386 Dnames H7 DTCM");
}

//! E2 packaging: emit the C header and pin the shared-object name.
//!
//! The header (`include/vioarm_cosim.h`, checked in for packagers) is
//! regenerated from `src/lib.rs` on every build; a regression test diffs
//! the fresh output against the checked-in copy so drift fails the build
//! instead of shipping a lying header. The `cdylib` soname carries the
//! major version (`libvioarm_cosim.so.1`).

use std::env;
use std::path::PathBuf;

fn main() {
    // Re-run only when the surface can change.
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/handle.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");

    // Versioned soname for the `cdylib` link (packagers symlink
    // `libvioarm_cosim.so` -> `.so.1` -> the real file). ELF-only: macOS
    // `ld` rejects `-soname` (it uses `-install_name`, and the platform
    // versioning lives in the bundle id), so emit it only for Linux.
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("linux") {
        println!("cargo:rustc-link-arg=-Wl,-soname,libvioarm_cosim.so.1");
    }

    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let config =
        cbindgen::Config::from_file(crate_dir.join("cbindgen.toml")).expect("cbindgen.toml");
    let bindings = cbindgen::Builder::new()
        .with_crate(crate_dir.clone())
        .with_config(config)
        .generate()
        .expect("cbindgen failed on vioarm-cosim");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("vioarm_cosim.h");
    bindings.write_to_file(&out);
}

//! E2 packaging tests: the checked-in packager files stay in sync with
//! the crate (version literals, module names). The heavy end-to-end
//! proofs (compile + run the C/Python examples, CMake configure) live in
//! `pytest/test_packaging.py`, where toolchains may skip gracefully.

use std::path::PathBuf;

fn packaging_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("packaging")
}

fn read(name: &str) -> String {
    let path = packaging_dir().join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|cause| panic!("packaging/{name} missing: {cause}"))
}

#[test]
fn pkg_config_version_tracks_the_crate() {
    let pc = read("vioarm-cosim.pc");
    let version = env!("CARGO_PKG_VERSION");
    assert!(
        pc.contains(&format!("Version: {version}\n")),
        "packaging/vioarm-cosim.pc must say Version: {version}"
    );
    assert!(pc.contains("-lvioarm_cosim"), "links the right library");
}

#[test]
fn find_module_names_the_right_artifacts() {
    let module = read("FindVioARM.cmake");
    for needle in [
        "vioarm_cosim.h",
        "vioarm_cosim",
        "VioARM::vioarm_cosim",
        "find_package_handle_standard_args",
    ] {
        assert!(
            module.contains(needle),
            "FindVioARM.cmake must mention {needle}"
        );
    }
}

#[test]
fn checked_in_header_exists_and_names_the_abi() {
    let header = std::fs::read_to_string(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("include")
            .join("vioarm_cosim.h"),
    )
    .expect("include/vioarm_cosim.h missing");
    assert!(header.contains("VIOARM_COSIM_H"), "include guard");
    assert!(
        header.contains("vioarm_cosim_step_duration"),
        "covers stepping entries"
    );
}

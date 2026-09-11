# Release Policy

- Semver: `Peripheral` trait is major, register fixes minor, docs patch.
- MSRV 1.75, `cargo publish` metadata on all five crates.
- Signed tags, `cargo cyclonedx` SBOM per release.
- Boards: F103/F107/F407/F072/F303/G474/L476/H743 (H7 display/audio handshake-only per B6).

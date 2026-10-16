# Release Policy

- Semver: `Peripheral` trait is major, register fixes minor, docs patch.
- MSRV 1.83, `cargo publish` metadata on all five crates.
- Signed tags, `cargo cyclonedx` SBOM per release.
- Boards: F103/F107/F407/F072/F303/G474/L476/H743 (H7 display/audio handshake-only per B6).

## Support

- Board list frozen at 511 (`board::all_boards()`); per-board fidelity in
  `docs/src/boards.md`; golden boot snapshots (`golden_traces`) catch
  model drift between releases.
- Private until complete: no publishes, tags, or mirrors (see RELEASING.md).

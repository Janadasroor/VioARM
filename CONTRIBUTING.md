# Contributing

## Development

```sh
cargo fmt --check
cargo clippy --workspace --all-targets
cargo test --workspace
cargo publish --dry-run -p vioarm-core
```

The differential conformance suite needs `qemu-system-arm` on `PATH`; clang-compiled firmware tests need `clang`/`llvm-objcopy`.

## Pull requests

- Keep `cargo fmt --check` clean (CI enforces).
- Add a `CHANGELOG.md` entry under `Unreleased`.
- Regenerate SVD tables with `cargo run -p vioarm-svd -- <svd> <out>` and do not hand-edit `vioarm-core/src/device/svd/*.rs`.

## Reporting issues

Use the issue templates in `.github/ISSUE_TEMPLATE/`.

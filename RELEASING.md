# Releasing VioARM

## Publish order (crates.io)

`vioarm-svd` is `publish = false` (offline codegen tool, never shipped).
The rest publish inside-out, because each layer depends on the previous one
from crates.io:

1. `cargo publish -p vioarm-core`
2. `cargo publish -p vioarm-gdb` and `cargo publish -p vioarm-qemu` (either order)
3. `cargo publish -p vioarm-cli` (needs core + gdb on the registry)

Dry-run each layer first (`cargo publish --dry-run -p <crate>`); CI runs the
core dry-run on every push to `main`. A dry-run of gdb/qemu/cli fails until
core is really on crates.io — that is expected, not a bug.

## Credentials

Publishing needs a crates.io token with publish scope:

```
cargo login   # once per machine, or export CARGO_REGISTRY_TOKEN
```

## Checklist

1. Bump `workspace.package.version` in the root `Cargo.toml`.
2. Move the `Unreleased` section of `CHANGELOG.md` under the new version.
3. `cargo fmt --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
   `cargo test --workspace`, `cargo deny check --deny warnings`, `cargo audit`,
   `cargo llvm-cov --workspace --fail-under-lines 85` — all green.
4. Push to `main` and wait for CI (build matrix, audit, coverage, book,
   publish dry-run) to go green.
5. Publish in the order above.
6. Tag the release **signed** (see below) and push the tag.

## Signed tags

Tags must be signed. With an SSH signing key (GitHub verifies these):

```
git config gpg.format ssh
git config user.signingkey ~/.ssh/id_ed25519.pub
git tag -s vX.Y.Z -m "VioARM vX.Y.Z"
git push origin vX.Y.Z
```

or with GPG (`git tag -s` with `user.signingkey` set to the key id).
No signing key lives in CI or in this repo — signing is always a local,
human step.

# Releasing VioARM

> **Private gate (2026-09-11): this repository stays private until the
> work is fully complete. Until the gate lifts: no `cargo publish`
> (dry-runs only), no release tags, no public mirrors. Steps 5–6 below
> are blocked; everything above them must stay green so the release is a
> formality when the gate lifts.**

## Publish order (crates.io)

`vioarm-svd` is `publish = false` (offline codegen tool, never shipped).
`vioarm-cosim` is `publish = false` (C ABI surface for the co-simulation
shim, linked from C++ sources — never a registry dependency).
The rest publish inside-out, because each layer depends on the previous one
from crates.io:

0. `cargo publish -p vioarm-jit-call` (the JIT call trampoline; core's
   `jit` feature depends on it by versioned path, so it must be on the
   registry before core publishes — but only matters for releases that
   ship the `jit` feature)
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

## Co-simulation gate (VioMATRIXC)

Linking against VioMATRIXC's `libngspice.so` (the P1+ co-sim path in
`cosim-roadmap.md`) is blocked until the VioMATRIXC repo deletes the
autogen-regressed `COPYING` (GPLv3 text, re-added by `b2478fe`'s
`autogen.sh` run — not a license change, but it must go plus a gitignore
rule) and the owner confirms the P0-1 verdict recorded there. No
`-lspice` in any build file before that.

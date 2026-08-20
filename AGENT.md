# AGENTS.md

Guidance for AI agents working in this repository.

## Project

VioARM is a ARM Cortex-M simulator written in Rust. It is intended to be solid
(safety/soundness focused) and extendable via a clean internal architecture.

## Commands

- Build: `cargo build`
- Test: `cargo test`
- Lint/format: `cargo clippy` and `cargo fmt`
- Release build: `cargo build --release`

## Conventions

- No comments unless the code genuinely needs them for clarity.
- Match existing code style and module layout when adding features.
- Keep the `packs/` directory out of version control; it contains third-party
  device packs used as reference data.
- Prefer small, focused modules over large monoliths.
- Keep commit messages short.

## Notes

- `roadmap.md` tracks the project roadmap and status; it is committed and kept
  current as features land.
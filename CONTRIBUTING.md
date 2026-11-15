# Contributing

Thanks for your interest in `usbasram`. This project is a Rust rewrite of an
earlier Windows driver prototype, and it aims to stay small, testable, and
honest about the limits of USB storage.

## Before you start

- Read [ARCHITECTURE.md](ARCHITECTURE.md) for the format and lock ordering.
- Search existing issues before opening a new one.
- For a large change, open an issue first so the design can be discussed.

## Development

The workspace requires Rust 1.85 or newer.

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo build --workspace --release
```

All of these run in GitHub Actions on Linux, Windows, and macOS. Please make sure
your change passes locally before opening a pull request.

## Code guidelines

- Keep `usbasram-core` free of `unsafe` code. It uses `#![forbid(unsafe_code)]`.
- Preserve the lock order documented in `ARCHITECTURE.md`:
  `cache shard -> writer -> file -> index`.
- Add a regression test for every bug fix that affects data integrity.
- Prefer explicit error handling over panics in library code. Panics are
  acceptable only for programmer errors that cannot be recovered.
- Do not claim that USB storage is faster than DDR3. Benchmark hot and cold
  tiers separately.
- Keep public APIs documented. `cargo doc --workspace --no-deps` should be clean.

## Tests

The test suite covers:

- write/read/reopen persistence,
- sparse zero-filled reads,
- compression ratios,
- cache eviction under a tiny cache,
- TRIM/WRITE_ZEROES durability,
- repeated overwrites and compaction,
- capacity mismatch rejection,
- NBD handshake, write, flush, and read,
- superblock and record corruption detection.

Add tests in the crate that owns the behavior. Integration tests use
`tempfile` so they do not depend on a real USB device.

## Commits and pull requests

- Write imperative, descriptive commit messages.
- Keep unrelated changes in separate commits.
- Describe the motivation and the user-visible effect in the pull request.
- Link the issue being fixed, if any.
- CI must pass before merge.

## License

By contributing, you agree that your contribution is dual licensed under the MIT
License and the Apache License, Version 2.0, as described in the README.

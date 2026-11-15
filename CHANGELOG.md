# Changelog

All notable changes to this project are documented here. The format follows
Keep a Changelog, and the project adheres to Semantic Versioning.

## [1.0.0] - 2026-11-15

### Added

- First stable Rust release of the tiered RAM + USB storage engine.
- `usbasram-core`: 4 KiB paged store with a sharded CLOCK cache, LZ4-compressed
  append-only cold log, CRC32C records, A/B superblock checkpoints, recovery
  scanning, and atomic sidecar compaction.
- `usbasram-nbd`: fixed-newstyle NBD server with `READ`, `WRITE`, `DISC`,
  `FLUSH`, `FUA`, `TRIM`, `WRITE_ZEROES`, option negotiation, and multi-connection
  support.
- `usbasram-cli`: `init`, `info`, `serve`, `verify`, `compact`, and `bench`
  commands with human-readable and JSON benchmark output.
- Cross-platform CI on Linux, Windows, and macOS, plus formatting, Clippy, MSRV,
  release builds, and a benchmark artifact.
- Documentation for the architecture, format, recovery matrix, security model,
  benchmarking, and contribution process.

### Changed

- Replaced the original C++ Windows KMDF driver prototype with a portable Rust
  workspace. The old prototype did not implement its DMA, write-back, or
  compression paths.

### Fixed

- N/A (initial release).

[1.0.0]: https://github.com/Jonah-Joshua/UsbAsRam-Project/releases/tag/v1.0.0

# Changelog

All notable changes to this project are documented here. The format follows
Keep a Changelog, and the project adheres to Semantic Versioning.

## [1.1.0] - 2026-11-15

### Added

- Thread-per-core NBD request pipeline: one reader per connection, a shared
  bounded MPMC submission queue, work-stealing parallel compression, and
  per-connection reply serialization for out-of-order NBD replies.
- FLUSH and FUA are ordered barriers: they drain the connection's outstanding
  requests, flush the store, and only then reply, matching the NBD specification.
- 4 KiB-aligned page buffers, 128-byte-aligned cache shards, shared read locks,
  and per-shard free lists for buffer recycling.
- Hot-read p50/p99 latency reporting in the benchmark and JSON output.
- Parallel payload compression and checksumming outside the writer lock.
- One positioned write per flush batch and a batched compaction writer.
- Direct cold reads into aligned cache pages, with no intermediate allocation
  for uncompressed records.
- Concurrent store and NBD pipeline tests.

### Changed

- Generations are assigned while the page's shard lock is held, so generation
  order matches page order under concurrent same-page writes.
- A dirty eviction snapshots the page and appends outside the shard lock; the
  page remains visible until the new record is indexed.
- Partial-page writes load cold data before locking and replay the modification
  if another writer wins the race.
- Flush callers are serialized so the same dirty batch cannot be appended twice.
- Workspace version is now 1.1.0.

### Fixed

- Cold reads and partial-page writes revalidate the index generation before
  publishing a loaded page. A flush, eviction, or newer write racing with the
  load can no longer install a stale clean page or resurrect an older page body.
- `make_space` reuses a loaded page across a full-shard retry, avoiding a second
  cold read on every cache miss when the shard is full.

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

[1.1.0]: https://github.com/Jonah-Joshua/UsbAsRam-Project/releases/tag/v1.1.0
[1.0.0]: https://github.com/Jonah-Joshua/UsbAsRam-Project/releases/tag/v1.0.0

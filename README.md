# usbasram

A crash-safe, tiered RAM + USB storage engine written in Rust.

`usbasram` revisits the original "use a USB drive as RAM" idea with an honest
architecture: **hot pages live in RAM at DRAM speed, cold pages live in a
compressed, checksummed log on the USB device.** The result is a portable block
device that behaves like a fast write-back cache with a durable, recoverable
backing store.

> **The physical limit.** A USB flash drive cannot be faster than DDR3. The USB
> bus, the flash controller, and NAND latency put a hard floor on cold access
> time. `usbasram` does not pretend otherwise. It makes the hot working set fast
> by keeping it in RAM, and it makes the cold tier as small, sequential, and
> recoverable as possible. Overall performance is therefore workload-dependent:
> cache-resident I/O runs at RAM speed; cold I/O is limited by the USB device.

## Why this exists

The original project was a Windows KMDF driver prototype whose DMA, write-back,
and compression paths were placeholders. It could not actually expose a USB
drive as RAM. This repository is a ground-up Rust rewrite that delivers a
working, testable, cross-platform storage engine rather than a kernel stub.

## Highlights

- **4 KiB paged block device** with a simple, documented on-disk format.
- **Sharded CLOCK page cache** with cache-line-padded read/write locks,
  4 KiB-aligned page buffers, and per-shard buffer recycling.
- **Optional accelerated I/O**: portable buffered I/O by default, plus
  `O_DIRECT`/`FILE_FLAG_NO_BUFFERING`/`F_NOCACHE` direct I/O, Linux io_uring
  with a registered fixed buffer and optional SQPOLL, and Windows IOCP. Every
  optional path falls back cleanly and reports the effective mode.
- **Opt-in hugepages** for cache pages and transfer buffers, using
  `MAP_HUGETLB` on Linux and `MEM_LARGE_PAGES` on Windows when available.
- **Adaptive write coalescing** that groups sequential flushes into
  erase-block-friendly windows from 128 KiB to 4 MiB and adjusts the window
  from observed write-plus-fsync latency.
- **Thread-per-core NBD pipeline**: each connection has a reader that feeds a
  bounded MPMC queue; a shared worker pool executes READ, WRITE, and TRIM in
  parallel, with replies serialized per connection. FLUSH and FUA act as ordered
  barriers so durability semantics match the NBD specification.
- **LZ4-compressed cold pages** with automatic fallback to raw pages when
  compression would expand data.
- **CRC32C on every record and every superblock** for corruption detection.
- **A/B superblock checkpoints** so a torn commit cannot make the store
  unrecoverable.
- **Recovery scan beyond the last checkpoint**, replaying valid records written
  before a crash.
- **Atomic compaction** through a sidecar file and rename, with crash recovery
  for every intermediate state.
- **NBD server** (fixed-newstyle negotiation) with `FLUSH`, `FUA`, `TRIM`,
  `WRITE_ZEROES`, and multi-connection support.
- **CLI** for creating, inspecting, verifying, compacting, benchmarking, and
  serving stores.
- **CI on Linux, Windows, and macOS**, including tests, Clippy, MSRV, feature
  matrices, Miri, AddressSanitizer, release builds, and benchmark artifacts.

## Quick start

Build from source (Rust 1.85 or newer):

```bash
cargo build --release
```

The default build is portable and contains no unsafe code. To include the
optional direct-I/O, io_uring, IOCP, and hugepage backends in the binary:

```bash
cargo build --release --features hardware
```

Create a 64 MiB store with the default 64 MiB hot cache:

```bash
./target/release/usbasram init ./usb-store.usr --size 64M
```

Inspect it:

```bash
./target/release/usbasram info ./usb-store.usr
```

Verify every cold record and checksum:

```bash
./target/release/usbasram verify ./usb-store.usr
```

Serve it over NBD on loopback:

```bash
./target/release/usbasram serve ./usb-store.usr --cache 128M
```

On Linux, attach the export as a block device:

```bash
sudo modprobe nbd
sudo nbd-client 127.0.0.1 10809 /dev/nbd0 -N usbasram
sudo mkfs.ext4 /dev/nbd0
sudo mount /dev/nbd0 /mnt/usbasram
```

Use a different NBD client on other platforms. The server speaks the standard
NBD fixed-newstyle protocol over TCP.

Run the benchmark:

```bash
./target/release/usbasram bench --size 64M --cold-cache 4M --threads 4 --seconds 2
```

Machine-readable output for CI:

```bash
./target/release/usbasram bench --quick --json
```

### I/O engines and memory

Every command that opens a store accepts the same engine flags:

```bash
# Portable buffered I/O (default).
./target/release/usbasram info ./usb-store.usr

# Sector-aligned direct I/O. The store file is padded to a sector boundary.
./target/release/usbasram info ./usb-store.usr --io direct

# Linux io_uring with SQPOLL when the kernel permits it.
./target/release/usbasram serve ./usb-store.usr --io uring

# Windows overlapped I/O through an I/O completion port.
./target/release/usbasram info ./usb-store.usr --io iocp

# Try large pages for the hot cache and transfer buffers.
./target/release/usbasram info ./usb-store.usr --io direct --hugepages

# Disable or retune the adaptive coalescer.
./target/release/usbasram serve ./usb-store.usr --no-coalesce
./target/release/usbasram serve ./usb-store.usr \
  --coalesce-min 128K --coalesce-max 4M --erase-block 128K
```

A mode that cannot be initialized on the current platform or filesystem falls
back to portable I/O. `info` prints the effective engine, whether direct mode
and SQPOLL are active, and whether hugepages are in use.

## Commands

| Command | Purpose |
| --- | --- |
| `init <path> --size <size>` | Create a store. Use `--cache`, `--log-size`, `--no-compress`, or `--force` as needed. |
| `info <path>` | Print capacity, cache/log sizes, head, epoch, checkpoint generation, and counters. |
| `serve <path>` | Serve the store over NBD. Use `--size` to create it if missing. |
| `verify <path>` | Scan every live cold record, verify CRC32C, and decompress it. |
| `compact <path>` | Flush and rewrite the log so it contains only live pages. |
| `bench` | Measure hot reads/writes at several queue depths, tail latency, cold reads, and raw backend throughput. |

Sizes accept `4096`, `64K`, `64KiB`, `1M`, `2GiB`, `1.5T`, and similar forms.

## How it works

```text
application / NBD client
          │
          ▼
┌───────────────────────────────┐
│ sharded CLOCK cache (RAM)     │  hot reads and writes complete here
│ 4 KiB pages, dirty tracking   │
└───────────────┬───────────────┘
                │ flush / eviction
                ▼
┌───────────────────────────────┐
│ append-only cold log (USB)    │  CRC32C records, A/B checkpoints
│ LZ4-compressed pages          │
└───────────────┬───────────────┘
                │ when full
                ▼
┌───────────────────────────────┐
│ atomic compaction             │  rewrite live pages to a sidecar,
│ sidecar + rename              │  fsync, then rename into place
└───────────────────────────────┘
```

The data file starts with two 4 KiB superblock slots. The append-only log starts
at offset 8192. Every record has a 48-byte header containing the page number,
generation, epoch, stored length, and CRC32C. A checkpoint commits a new head
only after the corresponding records have been fsynced.

See [ARCHITECTURE.md](ARCHITECTURE.md) for the format, recovery matrix, and
concurrency rules.

## Performance expectations

- **Hot reads** take a shared shard lock, copy from a 4 KiB-aligned page, and
  return without touching the log. Readers never serialize against other
  readers on the same shard.
- **Hot writes** take an exclusive shard lock, assign a generation under that
  lock, and return. Full-page writes never touch the cold tier.
- **Cold reads** pay for a positioned file read, CRC32C verification, and LZ4
  decompression. Uncompressed records are read directly into the aligned cache
  page, with no intermediate userspace buffer. The USB device is the bottleneck.
- **Flushes** compress and checksum pages in parallel, then issue one positioned
  write for an adaptive 128 KiB–4 MiB window, fsync the data, and fsync the
  commit superblock. This is the best access pattern for flash, and the window
  tunes itself from the measured write-plus-fsync cost.
- **Direct I/O** bypasses the OS page cache on the backing file. Because log
  records are only 8-byte aligned, the backend bounces unaligned spans through
  a sector-aligned buffer and performs read-modify-write on partial head and
  tail sectors.
- **io_uring and IOCP** keep the same positioned-I/O contract behind the same
  engine. They are selected at store open; if the platform, kernel, privileges,
  or filesystem reject the setup, the store continues on portable I/O and
  reports that fact.
- **Hugepages** reduce TLB pressure for the hot cache and transfer buffers when
  the operating system has reserved large pages. The store works identically
  without them.
- **Compression** reduces bytes written to USB for compressible data. Text,
  zero-filled pages, and sparse files benefit the most; already-compressed data
  is stored raw when LZ4 would expand it.
- **Compaction** costs one rewrite of the live set. Size the log to avoid
  frequent compactions, or call `compact` during maintenance windows.
- **Latency**: `bench` reports hot-read and cold-read p50/p99/p99.9 latency
  alongside queue-depth throughput.

`usbasram` is not a replacement for physical RAM. It is a durable, portable,
crash-safe tier that keeps the hot set in RAM and pushes the cold set to USB.
The data path is a userspace thread-per-core engine with batched positioned
I/O. Accelerated backends remain optional; the portable path is the reference
implementation and the fallback.

## Durability model

- A write becomes visible in RAM immediately.
- A write becomes durable when `flush` returns, or when the NBD client sends
  `FLUSH`/`FUA`.
- A crash during a flush is recovered by scanning valid records beyond the last
  checkpoint.
- A crash during compaction is recovered from `.compact` or `.old` sidecars.
- A torn superblock write is recovered from the other A/B slot.
- Corruption is detected with CRC32C. Detection is not repair; keep backups.

## Testing and CI

All builds and tests run in GitHub Actions. The workflow runs:

- formatting and Clippy,
- the test suite on Linux, Windows, and macOS,
- a minimum-supported-Rust check,
- feature-matrix checks for direct I/O, hugepages, io_uring, IOCP, and all
  features,
- Miri over the default-feature library tests and AddressSanitizer over the
  store integration tests,
- hardware smoke tests that exercise `--io direct`, `--io uring`, and
  `--io iocp` with a clean fallback as a valid outcome,
- release builds,
- benchmarks whose JSON output is uploaded as a build artifact.

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-targets
cargo test -p usbasram-core --features hardware --test store
cargo build --workspace --release
```

## Project layout

```text
crates/usbasram-core   storage engine, cache, format, recovery, compaction
crates/usbasram-nbd    NBD protocol server
crates/usbasram-cli    command-line interface and benchmark
```

## Limitations

- NBD has no authentication or encryption. The default bind is loopback. Do not
  expose it to an untrusted network.
- The store file is not locked against multiple processes. Open it from one
  process at a time.
- This is a userspace block-device backend, not a Windows kernel driver.
- `TRIM` and `WRITE_ZEROES` zero pages. They do not issue ATA/SCSI discard
  commands to the USB device.
- Compaction temporarily needs space for a second copy of the live set.
- Direct I/O requires a filesystem that supports aligned direct access. If the
  open is rejected, the store falls back to portable I/O and says so.
- io_uring may be disabled by a container seccomp policy or an old kernel.
  SQPOLL additionally depends on kernel version and privileges. A failed setup
  is not fatal.
- Hugepages require pre-reserved pages on Linux and `SeLockMemoryPrivilege` on
  Windows. Without them the store falls back to ordinary 4 KiB pages.
- The io_uring and IOCP backends submit one operation at a time per store;
  concurrency comes from caller threads. They are not a multi-queue block
  driver.
- The benchmark's raw phases measure the backend directly and do not include
  the store's record format, compression, or recovery logic.

## Roadmap

- Batch and pipeline io_uring/IOCP requests across caller threads while keeping
  the portable contract and fallback behavior.
- Queue-depth and worker-count tuning guidance for NBD clients.
- Optional device-level discard passthrough for `TRIM`.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this project by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.

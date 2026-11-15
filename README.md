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
- **Sharded CLOCK page cache** with per-shard locks for concurrent hot I/O.
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
- **CI on Linux, Windows, and macOS**, including tests, Clippy, MSRV, release
  builds, and a benchmark artifact.

## Quick start

Build from source (Rust 1.85 or newer):

```bash
cargo build --release
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

## Commands

| Command | Purpose |
| --- | --- |
| `init <path> --size <size>` | Create a store. Use `--cache`, `--log-size`, `--no-compress`, or `--force` as needed. |
| `info <path>` | Print capacity, cache/log sizes, head, epoch, checkpoint generation, and counters. |
| `serve <path>` | Serve the store over NBD. Use `--size` to create it if missing. |
| `verify <path>` | Scan every live cold record, verify CRC32C, and decompress it. |
| `compact <path>` | Flush and rewrite the log so it contains only live pages. |
| `bench` | Measure hot reads, hot writes, and cold reads. |

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

- **Hot reads/writes** hit the sharded RAM cache and are bounded by memory
  bandwidth and lock contention, not by USB.
- **Cold reads** pay for a positioned file read, CRC32C verification, and LZ4
  decompression. The USB device is the bottleneck.
- **Flushes** are sequential appends with one fsync per batch; this is the best
  access pattern for flash.
- **Compression** reduces bytes written to USB for compressible data. Text,
  zero-filled pages, and sparse files benefit the most; already-compressed data
  is stored raw when LZ4 would expand it.
- **Compaction** costs one rewrite of the live set. Size the log to avoid
  frequent compactions, or call `compact` during maintenance windows.

`usbasram` is not a replacement for physical RAM. It is a durable, portable,
crash-safe tier that keeps the hot set in RAM and pushes the cold set to USB.

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
- release builds,
- a benchmark whose JSON output is uploaded as a build artifact.

Local commands, if you have a Rust toolchain:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
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

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this project by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.

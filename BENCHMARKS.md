# Benchmarks

Performance is measured, not claimed. The benchmark separates the RAM-resident
hot tier from the USB-backed cold tier so the two are never conflated.

## Running the benchmark

```bash
cargo run --release -p usbasram-cli --features hardware -- bench \
  --size 64M \
  --cold-cache 4M \
  --threads 4 \
  --queue-depths 1,4,16,64 \
  --seconds 2 \
  --io portable
```

Select a backend with `--io direct`, `--io uring`, or `--io iocp`. Add
`--hugepages` to request large pages for the cache and transfer buffers. The
benchmark prints the effective mode, so a fallback to portable I/O is visible
in both text and JSON.

For CI or a quick smoke test:

```bash
cargo run --release -p usbasram-cli --features hardware -- bench --quick --json
```

The JSON output includes the effective engine (`io_mode`, `io_direct`,
`io_sqpoll`, `hugepages`), the queue depths tested, p50/p99/p99.9 latencies for
hot and cold reads, and `ops`, `bytes`, elapsed seconds, IOPS, and GiB/s for
every phase. CI uploads the latest result as the `benchmark-json` artifact on
the `CI` workflow run.

## Phases

| Phase | Cache | What it measures |
| --- | --- | --- |
| `hot_read` | Entire logical size | Shared-lock cache hits across all shards, 4 KiB-aligned copies, and memory bandwidth at each queue depth |
| `hot_write` | Entire logical size | Exclusive-lock cache writes and generation allocation at each queue depth |
| `cold_read` | 4 MiB | Positioned file reads, CRC32C verification, LZ4 decompression, and cache fills at the highest queue depth |
| `raw_write` | none | Sequential 1 MiB writes through the selected backend, bypassing the log format and compression |
| `raw_read` | none | Sequential 1 MiB reads through the selected backend, bypassing the cache and log format |

The separate latency pass records 50,000 hot-read samples and 20,000 cold-read
samples. `queue_depth` is the number of caller threads issuing 4 KiB requests;
the engine itself is synchronous, so this is how the benchmark applies
concurrent pressure. The raw phases are single-threaded and use an aligned
transfer path so direct mode can be measured without the store's 8-byte record
headers getting in the way.

The 64 MiB benchmark uses 128 cache shards on a 64 MiB cache, each protected by
a 128-byte-aligned `RwLock`. Increase `--queue-depths` and `--threads` to see how
far the shared read path scales; the default depths are `1,4,16,64`.

## Reference CI run

The numbers below are retained from the v1.1.0 release run, which used the
earlier benchmark schema (no per-queue-depth phases, no raw backend phases, and
no p99.9 latency). They are still useful as a baseline for the portable path,
but the tables will be replaced with v1.2.0 measurements that include direct
I/O and queue-depth scaling.

The following run is from the `CI` workflow on 2026-11-15. It is a **software
path measurement on a GitHub-hosted runner**, not a USB device measurement. The
backing file lives on the runner's local disk and may be served from the OS page
cache, so `cold_read` here overstates what a real USB flash drive can do.

- Store: 64 MiB, compression on
- Hot cache: 64 MiB
- Cold cache: 4 MiB
- Threads: 4
- Phase duration: 0.5 s
- Runner: `ubuntu-latest` (shared GitHub-hosted hardware)

Run-to-run variance on shared runners is large enough that a few percent
difference is noise. Compare changes only against runs on the same runner class.

| Phase | Operations | IOPS | Throughput |
| --- | ---: | ---: | ---: |
| `hot_read` | 3,418,288 | 6,834,978 | 26.07 GiB/s |
| `hot_write` | 3,203,212 | 6,404,706 | 24.43 GiB/s |
| `cold_read` | 486,776 | 973,281 | 3.71 GiB/s |

Single-thread hot-read latency over 20,000 samples:

| Percentile | Latency |
| --- | ---: |
| p50 | 380 ns |
| p99 | 771 ns |

These are userspace round-trip timings for a 4 KiB page that is already in the
cache. They exclude the NBD socket, the kernel block layer, and the client.

Use these numbers to compare code changes on the same class of runner, not to
predict USB performance. Attach a real USB device and rerun the benchmark for
device-specific results. The latest JSON is always available as the
`benchmark-json` artifact on the CI run.

## How to read the numbers

- `hot_read` and `hot_write` are upper bounds for this engine. They depend on
  CPU, memory bandwidth, thread count, queue depth, and cache sharding.
- `cold_read` is the number that reflects the USB device. It is dominated by
  the backing storage and the amount of compressible data. It will be orders of
  magnitude slower than the hot phases.
- `raw_write` and `raw_read` isolate the selected backend. Compare them with
  the matching portable run to see what direct I/O or io_uring changes on the
  same device. They exclude compression and the record format.
- `hot_write` does not include flush or fsync cost. Run a workload with periodic
  flushes (or use NBD with `FUA`) to measure durability overhead, and watch
  whether the adaptive coalescer settles at a larger or smaller window.
- The benchmark uses random 4 KiB pages and a fixed-seed PRNG. Compression
  ratios for real files depend on the data.
- A run that reports `io_mode: "portable"` after requesting direct, uring, or
  iocp measured the fallback path, not the requested accelerator.

## Reference environment

Do not treat numbers from another machine as a prediction. Use the CI artifact
or run the command locally, and record:

- CPU model and core count,
- RAM size,
- backing device model and interface (USB 2.0, USB 3.0, USB 3.2, NVMe, etc.),
- logical store size and cache size,
- thread count,
- whether compression was enabled.

## Interpreting the result honestly

A USB flash drive cannot reach DDR3 bandwidth or latency. A good result shows:

1. hot phases significantly faster than cold reads,
2. cold reads limited by the device, not by avoidable copying,
3. compressed stores smaller than raw stores for compressible data,
4. no corruption or recovery failures under the test suite.

The benchmark is a tool for finding bottlenecks, not a marketing claim.

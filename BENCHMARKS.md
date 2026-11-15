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

The tables below are the v1.2.0 reference runs from the `CI` workflow on
2026-11-15. They use the expanded schema: queue-depth scaling, p99.9 tail
latency, and a raw backend phase. The v1.1.0 tables used the earlier
single-thread schema and are not directly comparable.

This is a **software path measurement on a GitHub-hosted runner**, not a USB
device measurement. The backing file lives on the runner's local disk. The
portable run is served largely from the OS page cache, so its `cold_read`
overstates what a real USB flash drive can do. The direct run bypasses the page
cache and shows the storage path.

- Store: 64 MiB, compression on
- Hot cache: 64 MiB
- Cold cache: 4 MiB
- Threads: 4
- Phase duration: 0.5 s
- Runner: `ubuntu-latest` (shared GitHub-hosted hardware)
- Effective mode: `portable` and `direct`, no hugepages, no SQPOLL

### Portable backend

| Phase | Queue depth | IOPS | Throughput |
| --- | ---: | ---: | ---: |
| `hot_read` | 1 | 1,938,695 | 7.40 GiB/s |
| `hot_read` | 4 | 6,886,605 | 26.27 GiB/s |
| `hot_read` | 16 | 6,999,709 | 26.70 GiB/s |
| `hot_read` | 64 | 6,912,489 | 26.37 GiB/s |
| `hot_write` | 1 | 2,181,135 | 8.32 GiB/s |
| `hot_write` | 4 | 6,910,574 | 26.36 GiB/s |
| `hot_write` | 16 | 6,547,630 | 24.98 GiB/s |
| `hot_write` | 64 | 6,257,602 | 23.87 GiB/s |
| `cold_read` | 64 | 1,269,270 | 4.84 GiB/s |
| `raw_write` | 1 | 67,472 | 0.257 GiB/s |
| `raw_read` | 1 | 7,806,202 | 29.78 GiB/s |

### Direct backend

| Phase | Queue depth | IOPS | Throughput |
| --- | ---: | ---: | ---: |
| `hot_read` | 1 | 1,878,475 | 7.17 GiB/s |
| `hot_read` | 4 | 6,889,905 | 26.28 GiB/s |
| `hot_read` | 16 | 6,790,270 | 25.90 GiB/s |
| `hot_read` | 64 | 6,722,906 | 25.65 GiB/s |
| `hot_write` | 1 | 2,032,276 | 7.75 GiB/s |
| `hot_write` | 4 | 6,498,711 | 24.79 GiB/s |
| `hot_write` | 16 | 6,494,506 | 24.78 GiB/s |
| `hot_write` | 64 | 5,800,993 | 22.13 GiB/s |
| `cold_read` | 64 | 15,627 | 0.060 GiB/s |
| `raw_write` | 1 | 69,347 | 0.265 GiB/s |
| `raw_read` | 1 | 136,392 | 0.520 GiB/s |

The direct raw phases are the honest storage numbers: `O_DIRECT` bypasses the
page cache, so `raw_read` falls from 29.78 GiB/s to 0.52 GiB/s while the hot
phases stay in RAM. The portable `cold_read` is fast because the runner's page
cache already holds the file. Use a real USB device and the direct backend to
measure the device itself.

### Tail latency

| Backend | Phase | p50 | p99 | p99.9 |
| --- | --- | ---: | ---: | ---: |
| `portable` | hot read | 470 ns | 902 ns | 1,091 ns |
| `portable` | cold read | 2,133 ns | 2,533 ns | 9,764 ns |
| `direct` | hot read | 501 ns | 902 ns | 1,091 ns |
| `direct` | cold read | 270,444 ns | 617,633 ns | 671,012 ns |

The latency pass records 50,000 hot-read samples and 20,000 cold-read samples.
Hot numbers are userspace round-trip timings for a 4 KiB page that is already in
the cache. They exclude the NBD socket, the kernel block layer, and the client.

Run-to-run variance on shared runners is large enough that a few percent
difference is noise. Compare changes only against runs on the same runner class.
Use these numbers to compare code changes, not to predict USB performance.
Attach a real USB device and rerun the benchmark for device-specific results.
The latest JSON is always available as the `benchmark-json` artifact on the CI
run.

## How to read the numbers

- `hot_read` and `hot_write` are upper bounds for this engine. They depend on
  CPU, memory bandwidth, thread count, queue depth, and cache sharding.
- `cold_read` is the number that reflects the USB device in direct mode. It is
  dominated by the backing storage and the amount of compressible data. It will
  be orders of magnitude slower than the hot phases. In portable mode the OS
  page cache can make cold reads look much faster than the device.
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

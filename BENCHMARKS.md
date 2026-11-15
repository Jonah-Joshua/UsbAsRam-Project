# Benchmarks

Performance is measured, not claimed. The benchmark separates the RAM-resident
hot tier from the USB-backed cold tier so the two are never conflated.

## Running the benchmark

```bash
cargo run --release -p usbasram-cli -- bench \
  --size 64M \
  --cold-cache 4M \
  --threads 4 \
  --seconds 2
```

For CI or a quick smoke test:

```bash
cargo run --release -p usbasram-cli -- bench --quick --json
```

The JSON output includes `ops`, `bytes`, elapsed seconds, IOPS, and GiB/s for
each phase. CI uploads the latest result as the `benchmark-json` artifact on the
`CI` workflow run.

## Phases

| Phase | Cache | What it measures |
| --- | --- | --- |
| `hot_read` | Entire logical size | RAM cache hit path, shard lock contention, and copy bandwidth |
| `hot_write` | Entire logical size | RAM cache write path and generation allocation |
| `cold_read` | 4 MiB | Positioned file reads, CRC32C verification, LZ4 decompression, and cache fills |

## Reference CI run

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

| Phase | Operations | IOPS | Throughput |
| --- | ---: | ---: | ---: |
| `hot_read` | 2,883,810 | 5,766,337 | 22.00 GiB/s |
| `hot_write` | 2,999,948 | 5,998,607 | 22.88 GiB/s |
| `cold_read` | 718,827 | 1,437,364 | 5.48 GiB/s |

Use these numbers to compare code changes on the same class of runner, not to
predict USB performance. Attach a real USB device and rerun the benchmark for
device-specific results. The latest JSON is always available as the
`benchmark-json` artifact on the CI run.

## How to read the numbers

- `hot_read` and `hot_write` are upper bounds for this engine. They depend on
  CPU, memory bandwidth, thread count, and cache sharding.
- `cold_read` is the number that reflects the USB device. It is dominated by
  the backing storage and the amount of compressible data. It will be orders of
  magnitude slower than the hot phases.
- `hot_write` does not include flush or fsync cost. Run a workload with periodic
  flushes (or use NBD with `FUA`) to measure durability overhead.
- The benchmark uses random 4 KiB pages and a fixed-seed PRNG. Compression
  ratios for real files depend on the data.

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

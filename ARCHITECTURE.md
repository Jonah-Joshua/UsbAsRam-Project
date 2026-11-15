# Architecture

This document describes the on-disk format, in-memory design, recovery rules,
and the concurrency model of `usbasram`.

## 1. Design goals

1. **Hot data at RAM speed.** The working set must not touch USB for every read
   or write.
2. **Durability on demand.** `flush` must mean that all writes acknowledged
   before the call survive a crash.
3. **Cheap writes to flash.** Cold writes should be sequential and compressed,
   not random in-place updates.
4. **Crash safety without a journaling filesystem.** Recovery must be possible
   from the store file alone.
5. **Portability.** The same engine must build and pass tests on Linux, Windows,
   and macOS.

The design is a write-back cache over an append-only log. It is deliberately
not a kernel driver: a userspace process owns the store and exposes it through
NBD.

## 2. Tiers

```text
┌─────────────────────────────────────────────────────────────┐
│ Hot tier: sharded CLOCK page cache in RAM                   │
│ - 4 KiB-aligned pages, 128-byte aligned shard locks         │
│ - shared read locks, exclusive write locks                  │
│ - dirty / atomic referenced bits, per-page generation       │
│ - per-shard free list for page-buffer recycling             │
└───────────────────────────┬─────────────────────────────────┘
                            │ eviction or explicit flush
┌───────────────────────────▼─────────────────────────────────┐
│ Cold tier: append-only log on the backing file              │
│ - CRC32C-protected records                                  │
│ - optional LZ4 payloads                                     │
│ - A/B superblock checkpoints                                │
└───────────────────────────┬─────────────────────────────────┘
                            │ log high-water mark
┌───────────────────────────▼─────────────────────────────────┐
│ Maintenance: atomic compaction into a sidecar file          │
└─────────────────────────────────────────────────────────────┘
```

The logical block device is `capacity_pages * 4096` bytes. Pages are always
4096 bytes.

## 3. On-disk format

All integers are little-endian.

### 3.1 Layout

```text
offset 0        superblock slot A (4096 bytes)
offset 4096     superblock slot B (4096 bytes)
offset 8192     append-only record log
                ...
                DATA_OFFSET + log_bytes (end of log)
```

`log_bytes` is stored in the superblock and is the capacity of the record log,
not the current file length. The file grows as records are appended. A new
store is only 8192 bytes long until the first flush.

### 3.2 Superblock

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 8 | magic `USRSTORE` |
| 8 | 4 | format version (1) |
| 12 | 4 | page size (4096) |
| 16 | 8 | capacity in pages |
| 24 | 8 | log capacity in bytes |
| 32 | 8 | committed head offset |
| 40 | 8 | epoch |
| 48 | 8 | checkpoint generation |
| 56 | 4 | flags (bit 0: compression enabled) |
| 64 | 4 | CRC32C of bytes 0..64 |
| 68 | 4028 | zero padding |

Two slots are used in an A/B fashion. A checkpoint writes the inactive slot and
fsyncs it. On open, the valid slot with the highest checkpoint generation wins.
If the newest slot is torn, the older slot is still a valid checkpoint. The
recovery scan then replays records written after that older head.

### 3.3 Record header

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 4 | magic `0x55535252` |
| 4 | 2 | record version (1) |
| 6 | 2 | flags (bit 0: LZ4 payload) |
| 8 | 8 | page number |
| 16 | 8 | generation |
| 24 | 8 | epoch |
| 32 | 4 | raw length (4096) |
| 36 | 4 | stored length |
| 40 | 4 | CRC32C of the payload |
| 44 | 4 | CRC32C of bytes 0..44 |
| 48 | n | payload |
| 48+n | 0..7 | zero padding to an 8-byte boundary |

A record's total length is `align_up(48 + stored_len, 8)`.

### 3.4 Generations and epochs

- A **generation** is a process-wide monotonically increasing counter assigned
  to every page mutation. The newest generation for a page wins.
- An **epoch** identifies one incarnation of the log. Compaction increments the
  epoch and rewrites every live record with the new epoch. Records from another
  epoch are ignored during recovery.
- Epochs and generations are both nonzero. Zero is reserved for "page has never
  been written", which reads as all zeroes.

## 4. Write path

1. Locate the page and page offset.
2. If the page is cached, take the shard's write lock, allocate a generation,
   copy the full page or apply the sub-page range, mark it dirty, and return.
   Allocating the generation while holding the shard lock is what makes the
   generation order match the page order.
3. On a partial-page miss, load the old page without holding the shard lock,
   then retry under the lock. If another writer inserted the page meanwhile,
   the modification is replayed against the now-cached page.
4. Mark the page dirty. The write is now visible to readers.
5. On eviction or flush, snapshot dirty payloads, compress them in parallel,
   then append the batch to the log.
6. `fsync` the batch, then commit a new superblock checkpoint with the new
   head. Only then is the write durable.

The cache is sharded. Each shard is a `RwLock` padded to 128 bytes, so readers
on different shards never share a cache line and readers on the same shard run
concurrently. A CLOCK (second-chance) hand selects clean victims. A dirty victim
is snapshotted and appended while the shard lock is released; the page stays
visible in the cache until the append commits, at which point it is removed or
marked clean. This prevents a reader from observing the older cold copy and
keeps `fsync` latency out of the shard lock.

## 5. Read path

1. Look up the page in the cache shard under a shared lock. A hit marks the page
   referenced with an atomic store, copies the requested range, and returns.
   Readers do not serialize against each other.
2. On a miss, take a file read lock, then the index read lock, and find the
   newest extent for the page.
3. If there is no extent, the page is zero-filled.
4. Otherwise read the record header, verify the header CRC and payload CRC32C,
   and verify that the header matches the index.
5. Uncompressed payloads are read directly into the 4 KiB-aligned cache page.
   Compressed payloads use a per-call 5 KiB stack buffer and are decompressed
   into the aligned page.
6. Publish the clean page under the index read lock and copy the requested range
   while the shard's write lock is held. If the index moved on, or a newer page
   is already cached, discard the loaded page and retry. This prevents a cold
   read that raced with a flush and an eviction from installing a stale clean
   page or returning stale data.

Taking the file lock before the index lock is important: compaction takes the
file write lock before replacing the index, so a reader cannot resolve an extent
against a file that has already been renamed away.

## 6. Flush and checkpoints

`Store::flush` captures the current generation high-water mark, then repeatedly
collects up to 256 dirty pages whose generation is below the mark. Writes that
start after the mark are not chased; they remain dirty for the next flush. This
prevents a continuous writer from making flush run forever.

For each batch:

1. Reserve log space. If the batch does not fit, compact first.
2. Compress and checksum every payload in parallel with the work-stealing pool.
   Payload encoding is independent of the log epoch, so this happens before the
   writer lock is taken.
3. Build one contiguous record batch under the writer lock, using the current
   epoch for the headers, and issue one positioned write.
4. `fsync` the record data.
5. Write and `fsync` the inactive superblock slot.
6. Publish the new extents in the index while still holding the writer lock.
7. Clear the dirty bit only if the page generation still matches the flushed
   generation.

Flush callers are serialized by a dedicated flush mutex, so two flushers cannot
append the same dirty batch twice. Hot reads and writes never take that mutex.

If a checkpoint fails, the writer records that a checkpoint is still needed and
returns the error. The next flush retries it. Even if the process crashes before
that retry, the recovery scan finds the valid records beyond the old head.

## 7. Compaction

Compaction runs when the append would exceed `log_bytes`, or when the user calls
`Store::compact`.

1. Flush all dirty pages below the current mark.
2. Take the writer lock, then the file write lock.
3. Snapshot the live index.
4. Create `<store>.compact` and write a fresh file:
   - all live records, rewritten with `epoch + 1`,
   - a new superblock at slot A with `head = new_head`,
   - zeroes in slot B.
5. `fsync` the sidecar.
6. Close the old file handle (required on Windows before renaming).
7. Rename `<store>` to `<store>.old`.
8. Rename `<store>.compact` to `<store>`.
9. Reopen the store, install the new index, bump the epoch and checkpoint
   generation, and delete `<store>.old`.

Every live page is copied in the worst case, so the minimum log size includes
one full logical capacity plus a full flush batch. The default log adds 25%
headroom above the logical size to reduce compaction frequency; use
`--log-size` to choose a different value. The sidecar requires enough free space
for a second copy of the live set.

## 8. Recovery

On open, the engine tries both superblock slots and picks the valid one with the
highest checkpoint generation. It then scans from `DATA_OFFSET` to
`min(file_len, DATA_OFFSET + log_bytes)`, accepting only records whose header
and payload checksums are valid, whose epoch matches, and whose page number is
in range. The newest generation per page wins.

Recovery outcomes:

| Crash point | State | Recovery |
| --- | --- | --- |
| During a flush, before checkpoint | Records beyond the old head are valid | Scan replays them; open commits a fresh checkpoint |
| During a checkpoint write | One slot is torn | The other slot is used; the tail is replayed |
| After records, before `fsync` | Tail records may be incomplete | Scan stops at the first invalid record; the committed prefix survives |
| During compaction, before rename | `.compact` exists, main is intact | Delete the sidecar and open the main file |
| After main -> `.old`, before `.compact` -> main | Main is missing, `.old` and `.compact` exist | Prefer a validated `.compact`; otherwise restore `.old` |
| After `.compact` -> main, before `.old` deletion | Main is the new file | Open main and delete `.old` |
| Log scan ends before the committed head | Committed data is missing or corrupt | Open fails with a corruption error |

A valid superblock is not enough to prove that the whole store is healthy. Run
`usbasram verify` to scan every live record.

## 9. Concurrency model

The lock order for operations that take more than one of the log locks is:

```text
writer -> file -> index
```

Cache shard locks are normally acquired alone; no shard lock is held while
taking the writer, file, or index lock. This keeps `fsync` and compaction off
the hot path.

The one exception is the cold-fill path. It holds the index read lock while
taking the destination shard's write lock:

```text
index (read) -> cache shard (write)
```

This makes "the index still names the generation I loaded" and "the cache does
not already have a newer page" a single atomic decision. It is safe because the
cold-fill path never takes the writer or file lock, and no other path holds a
shard lock while waiting for the index.

Rules:

- Never acquire the writer or file lock while holding a cache shard lock.
- Never acquire the writer lock while holding the file lock.
- Compaction holds the writer and file write locks while replacing the file and
  the index.
- Appends publish index entries while still holding the writer lock, so a
  compaction cannot replace the index between an append and its visibility.
- Partial-page writes load cold data before taking the shard lock. If a full-page
  write wins the race, the partial update is replayed on the cached page. If a
  flush publishes a newer record while the cold load is in flight, the index
  generation is rechecked and the partial update is replayed on the newer body,
  so no update is lost and an older body cannot be resurrected.
- `verify` holds the writer and a file read lock for the whole scan, so the head
  and file cannot change underneath it.

The cache is sharded into power-of-two buckets selected by the page number.
Each bucket is an `RwLock<Shard>` aligned to 128 bytes. Hot reads take shared
locks and do not contend with each other. The writer lock serializes log
appends but is not taken by hot reads or full-page cache writes.

## 10. NBD server

The server implements NBD fixed-newstyle negotiation:

- `NBD_OPT_LIST`, `NBD_OPT_INFO`, `NBD_OPT_GO`, and the legacy
  `NBD_OPT_EXPORT_NAME`.
- Simple replies.
- `READ`, `WRITE`, `DISC`, `FLUSH`, `TRIM`, and `WRITE_ZEROES`.
- `FUA` on writes and zeroing requests.
- Advertised block sizes: minimum 1 byte, preferred 4096, maximum 32 MiB.

Each connection has a reader thread that parses requests and submits jobs to a
bounded MPMC queue. A fixed pool of workers, sized to the hardware thread
count, executes the jobs. Replies are framed under a per-connection writer
mutex. NBD cookies make out-of-order replies legal, so a single pipelining client
can keep every worker busy. READ jobs reuse one reply buffer per worker; WRITE
jobs own their payload buffer until the write completes.

The NBD specification allows out-of-order command processing. `FLUSH` and `FUA`
are the exceptions: the reader drains the connection's outstanding requests
before executing them, then calls `Store::flush` before replying. A write batch
that a client completes before a flush is therefore covered by that flush. As
the specification requires, clients must not pipeline overlapping writes without
ordering; independent pages are safe to pipeline.

`TRIM` and `WRITE_ZEROES` are implemented as zeroing writes. They are visible
immediately and become durable on the next flush. The server advertises
multi-connection support because all connections share one store and a flush
covers writes from every connection.

## 11. Why this cannot beat DDR3

A DDR3 module moves data at tens of gigabytes per second with nanosecond-scale
latency. USB 3.x delivers a serial link with protocol overhead, and NAND flash
adds microseconds to milliseconds of latency plus erase-before-write behavior.
No software layer can turn a USB flash device into DDR3.

What software can do is keep the hot set out of the slow tier, reduce bytes
written with compression, make cold I/O sequential, and make crashes
recoverable. That is the trade this project makes. The benchmark reports hot and
cold phases separately so the numbers are not misrepresented as one "RAM speed"
claim.

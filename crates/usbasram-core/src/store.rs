use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use parking_lot::{Mutex, RwLock};
use rayon::prelude::*;

use crate::cache::{AlignedPage, CachePage, PageCache};
use crate::error::{Error, Result};
use crate::format::{
    RecordHeader, Superblock, DATA_OFFSET, MAX_STORED_LEN, RECORD_FLAG_COMPRESSED,
    RECORD_HEADER_SIZE, SUPERBLOCK_SIZE, SUPERBLOCK_SLOTS,
};
use crate::io::{align_up, read_exact_at, write_all_at};
use crate::metrics::{Metrics, Stats};
use crate::PAGE_SIZE;

const MAX_FLUSH_BATCH: usize = 256;
const ZERO_PAGE: [u8; PAGE_SIZE] = [0u8; PAGE_SIZE];
const MAX_RECORD_LEN: u64 = ((RECORD_HEADER_SIZE + PAGE_SIZE + 7) & !7) as u64;
const COMPACT_BATCH_BYTES: usize = 1024 * 1024;

type DirtyPage = (u64, Box<AlignedPage>, u64);
type PageWrite<'a> = (u64, &'a [u8], u64);

/// A page whose payload has been compressed (or copied) but whose record header
/// has not been built yet. Payload encoding is independent of the log epoch, so
/// it can run outside the writer lock and in parallel across cores.
struct EncodedPage {
    page_no: u64,
    generation: u64,
    payload: Vec<u8>,
    payload_crc: u32,
    compressed: bool,
}

/// Configuration for a tiered store.
#[derive(Clone, Debug)]
pub struct StoreConfig {
    /// Path of the backing file (normally on a USB drive).
    pub path: PathBuf,
    /// Logical block-device size in bytes. Must be a multiple of 4096.
    pub size_bytes: u64,
    /// Bytes of RAM used for the hot page cache.
    pub cache_bytes: u64,
    /// Maximum size of the append-only cold log in bytes.
    ///
    /// If `None`, a log with 25% headroom above the logical size is used (but
    /// never smaller than [`min_log_bytes`]). A smaller log is rejected because
    /// compaction must be able to hold every page in the worst case.
    pub log_bytes: Option<u64>,
    /// Compress cold pages with LZ4 before writing them.
    pub compress: bool,
    /// Create the store if the backing file does not exist.
    pub create: bool,
}

impl StoreConfig {
    /// Create a configuration with compression enabled, a 256 MiB hot cache,
    /// and a cold log with 25% headroom above the logical size.
    pub fn new(path: impl Into<PathBuf>, size_bytes: u64) -> Self {
        Self {
            path: path.into(),
            size_bytes,
            cache_bytes: size_bytes.min(256 * 1024 * 1024),
            log_bytes: None,
            compress: true,
            create: true,
        }
    }

    /// Set the hot-cache size.
    pub fn with_cache_bytes(mut self, cache_bytes: u64) -> Self {
        self.cache_bytes = cache_bytes;
        self
    }

    /// Override the cold-log size.
    pub fn with_log_bytes(mut self, log_bytes: u64) -> Self {
        self.log_bytes = Some(log_bytes);
        self
    }

    /// Enable or disable LZ4 compression of cold pages.
    pub fn with_compression(mut self, compress: bool) -> Self {
        self.compress = compress;
        self
    }

    /// Allow creating a new store file.
    pub fn with_create(mut self, create: bool) -> Self {
        self.create = create;
        self
    }

    fn validate(&self) -> Result<()> {
        if self.path.as_os_str().is_empty() {
            return Err(Error::Config("store path is empty".into()));
        }
        if self.size_bytes == 0 || self.size_bytes % PAGE_SIZE as u64 != 0 {
            return Err(Error::Config(
                "size_bytes must be a non-zero multiple of 4096".into(),
            ));
        }
        if self.cache_bytes == 0 {
            return Err(Error::Config(
                "cache_bytes must be greater than zero".into(),
            ));
        }
        let minimum = checked_min_log_bytes(self.size_bytes)?;
        if let Some(log_bytes) = self.log_bytes {
            if log_bytes < minimum {
                return Err(Error::Config(format!(
                    "log_bytes must be at least {minimum} for a {} byte store",
                    self.size_bytes
                )));
            }
            if log_bytes % PAGE_SIZE as u64 != 0 {
                return Err(Error::Config("log_bytes must be a multiple of 4096".into()));
            }
        }
        Ok(())
    }
}

/// Metadata about an open store.
#[derive(Clone, Debug)]
pub struct StoreInfo {
    pub path: PathBuf,
    pub capacity_bytes: u64,
    pub page_size: u32,
    pub cache_bytes: u64,
    pub log_bytes: u64,
    pub head: u64,
    pub epoch: u64,
    pub checkpoint_generation: u64,
    pub compressed: bool,
    pub pages_on_disk: usize,
    pub superblock_slot: usize,
}

/// Result of a full integrity scan of the cold log.
#[derive(Clone, Debug, Default)]
pub struct VerifyReport {
    pub pages: u64,
    pub raw_bytes: u64,
    pub stored_bytes: u64,
    pub compressed_pages: u64,
    pub elapsed_ms: u128,
}

/// The smallest log that can always hold every live page during compaction.
pub fn min_log_bytes(size_bytes: u64) -> u64 {
    checked_min_log_bytes(size_bytes).unwrap_or(u64::MAX)
}

fn checked_min_log_bytes(size_bytes: u64) -> Result<u64> {
    let pages = size_bytes / PAGE_SIZE as u64;
    // Leave room for one full flush batch while every logical page is live.
    // Compaction rewrites each live page, then the batch is appended before the
    // index is updated, so a margin is required to avoid a false "log full".
    let records = pages
        .checked_add(MAX_FLUSH_BATCH as u64)
        .ok_or_else(|| Error::Config("store is too large for this format".into()))?;
    records
        .checked_mul(MAX_RECORD_LEN)
        .map(|bytes| align_up(bytes, PAGE_SIZE as u64))
        .ok_or_else(|| Error::Config("store is too large for this format".into()))
}

fn default_log_bytes(size_bytes: u64) -> Result<u64> {
    let minimum = checked_min_log_bytes(size_bytes)?;
    let preferred = size_bytes
        .checked_add(size_bytes / 4)
        .ok_or_else(|| Error::Config("store is too large for this format".into()))?;
    Ok(align_up(preferred.max(minimum), PAGE_SIZE as u64))
}

#[derive(Clone, Copy, Debug)]
struct Extent {
    offset: u64,
    stored_len: u32,
    generation: u64,
    epoch: u64,
    compressed: bool,
}

impl Extent {
    fn record_len(&self) -> u64 {
        align_up(RECORD_HEADER_SIZE as u64 + self.stored_len as u64, 8)
    }
}

struct WriterState {
    head: u64,
    epoch: u64,
    checkpoint_generation: u64,
    superblock_slot: usize,
    log_bytes: u64,
    needs_checkpoint: bool,
}

/// A crash-safe, tiered page store.
///
/// Written pages live in a sharded, lock-per-shard RAM cache. A background (or
/// caller-driven) [`Store::flush`] compresses dirty pages and appends them to a
/// checksummed log. When the log fills, [`Store::compact`] rewrites live pages
/// into a sidecar file and atomically replaces the old log.
pub struct Store {
    config: StoreConfig,
    capacity_bytes: u64,
    capacity_pages: u64,
    cache_bytes: u64,
    log_bytes: u64,
    compress: bool,
    file: RwLock<Option<File>>,
    index: RwLock<HashMap<u64, Extent>>,
    cache: PageCache,
    writer: Mutex<WriterState>,
    flush_lock: Mutex<()>,
    next_generation: AtomicU64,
    metrics: Metrics,
}

impl Store {
    /// Open or create a store.
    pub fn open(config: StoreConfig) -> Result<Self> {
        config.validate()?;
        let capacity_pages = config.size_bytes / PAGE_SIZE as u64;
        let cache_pages = usize::try_from(
            config
                .cache_bytes
                .div_ceil(PAGE_SIZE as u64)
                .min(capacity_pages)
                .max(1),
        )
        .map_err(|_| Error::Config("cache is too large for this platform".into()))?;
        let cache_bytes = cache_pages as u64 * PAGE_SIZE as u64;

        recover_sidecars(&config.path, config.create)?;

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(config.create)
            .open(&config.path)?;
        if !file.metadata()?.is_file() {
            return Err(Error::NotAFile(config.path.clone()));
        }
        let len = file.metadata()?.len();

        let (superblock, superblock_slot) = if len < DATA_OFFSET {
            if !config.create {
                return Err(Error::corrupt("store file is incomplete"));
            }
            let log_bytes = match config.log_bytes {
                Some(log_bytes) => log_bytes,
                None => default_log_bytes(config.size_bytes)?,
            };
            let superblock = Superblock::new(
                capacity_pages,
                log_bytes,
                DATA_OFFSET,
                1,
                1,
                config.compress,
            );
            file.set_len(DATA_OFFSET)?;
            write_all_at(&file, &superblock.encode(), 0)?;
            write_all_at(&file, &[0u8; SUPERBLOCK_SIZE], SUPERBLOCK_SIZE as u64)?;
            file.sync_all()?;
            (superblock, 0usize)
        } else {
            let (superblock, slot) = load_best_superblock(&file, len)?;
            if superblock.capacity_pages != capacity_pages {
                return Err(Error::Config(format!(
                    "store capacity is {} pages, configuration says {capacity_pages}",
                    superblock.capacity_pages
                )));
            }
            if let Some(log_bytes) = config.log_bytes {
                if log_bytes != superblock.log_bytes {
                    return Err(Error::Config(format!(
                        "store log is {} bytes, configuration says {log_bytes}",
                        superblock.log_bytes
                    )));
                }
            }
            (superblock, slot)
        };

        let log_bytes = superblock.log_bytes;
        let (index, recovered_head) = scan_log(&file, &superblock)?;
        let max_generation = index
            .values()
            .map(|extent| extent.generation)
            .max()
            .unwrap_or(0);
        let next_generation = max_generation
            .checked_add(1)
            .ok_or_else(|| Error::corrupt("generation counter overflow"))?;
        let writer = WriterState {
            head: recovered_head,
            epoch: superblock.epoch,
            checkpoint_generation: superblock.checkpoint_generation,
            superblock_slot,
            log_bytes,
            needs_checkpoint: recovered_head != superblock.head,
        };

        let store = Self {
            config,
            capacity_bytes: capacity_pages * PAGE_SIZE as u64,
            capacity_pages,
            cache_bytes,
            log_bytes,
            compress: superblock.compressed(),
            file: RwLock::new(Some(file)),
            index: RwLock::new(index),
            cache: PageCache::new(cache_pages),
            writer: Mutex::new(writer),
            flush_lock: Mutex::new(()),
            next_generation: AtomicU64::new(next_generation),
            metrics: Metrics::default(),
        };

        // Persist a recovered post-checkpoint tail immediately and clean up any
        // sidecar left by an interrupted compaction.
        if recovered_head != superblock.head {
            store.flush()?;
        }
        let _ = std::fs::remove_file(sidecar(&store.config.path, ".compact"));
        let _ = std::fs::remove_file(sidecar(&store.config.path, ".old"));
        Ok(store)
    }

    /// Logical capacity in bytes.
    pub fn capacity_bytes(&self) -> u64 {
        self.capacity_bytes
    }

    /// Open an existing store without requiring the caller to know its size.
    pub fn open_existing(path: impl Into<PathBuf>, cache_bytes: u64) -> Result<Self> {
        let path = path.into();
        let superblock = {
            let file = File::open(&path)?;
            let len = file.metadata()?.len();
            if len < DATA_OFFSET {
                return Err(Error::corrupt("store file is incomplete"));
            }
            load_best_superblock(&file, len)?.0
        };
        let size_bytes = superblock
            .capacity_pages
            .checked_mul(PAGE_SIZE as u64)
            .ok_or_else(|| Error::corrupt("store capacity overflows"))?;
        Self::open(StoreConfig {
            path,
            size_bytes,
            cache_bytes,
            log_bytes: Some(superblock.log_bytes),
            compress: superblock.compressed(),
            create: false,
        })
    }

    /// Logical capacity in 4 KiB pages.
    pub fn capacity_pages(&self) -> u64 {
        self.capacity_pages
    }

    /// Metadata about the open store.
    pub fn info(&self) -> StoreInfo {
        let writer = self.writer.lock();
        let pages_on_disk = self.index.read().len();
        StoreInfo {
            path: self.config.path.clone(),
            capacity_bytes: self.capacity_bytes,
            page_size: PAGE_SIZE as u32,
            cache_bytes: self.cache_bytes,
            log_bytes: self.log_bytes,
            head: writer.head,
            epoch: writer.epoch,
            checkpoint_generation: writer.checkpoint_generation,
            compressed: self.compress,
            pages_on_disk,
            superblock_slot: writer.superblock_slot,
        }
    }

    /// Current counters and occupancy.
    pub fn stats(&self) -> Stats {
        let (cache_pages, dirty_pages) = self.cache.counts();
        let pages_on_disk = self.index.read().len();
        let writer = self.writer.lock();
        self.metrics.snapshot(
            cache_pages,
            dirty_pages,
            pages_on_disk,
            self.capacity_bytes,
            self.log_bytes,
            writer.head,
            writer.epoch,
        )
    }
}

fn sidecar(path: &Path, suffix: &str) -> PathBuf {
    let mut name = path.as_os_str().to_os_string();
    name.push(suffix);
    PathBuf::from(name)
}

fn recover_sidecars(path: &Path, create: bool) -> Result<()> {
    let compact = sidecar(path, ".compact");
    let old = sidecar(path, ".old");
    let main_len = std::fs::metadata(path).map(|meta| meta.len()).unwrap_or(0);

    if main_len < DATA_OFFSET {
        for candidate in [&compact, &old] {
            if candidate.exists() && superblock_file_valid(candidate) {
                if path.exists() {
                    std::fs::remove_file(path)?;
                }
                std::fs::rename(candidate, path)?;
                break;
            }
        }
    }

    if !path.exists() && !create {
        return Err(Error::Io(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            format!("store {} does not exist", path.display()),
        )));
    }

    let _ = std::fs::remove_file(&compact);
    let _ = std::fs::remove_file(&old);
    Ok(())
}

fn superblock_file_valid(path: &Path) -> bool {
    let Ok(file) = File::open(path) else {
        return false;
    };
    let Ok(len) = file.metadata().map(|meta| meta.len()) else {
        return false;
    };
    if len < DATA_OFFSET {
        return false;
    }
    load_best_superblock(&file, len).is_ok()
}

fn load_best_superblock(file: &File, len: u64) -> Result<(Superblock, usize)> {
    let mut best: Option<(Superblock, usize)> = None;
    for slot in 0..SUPERBLOCK_SLOTS {
        let offset = slot * SUPERBLOCK_SIZE as u64;
        if offset + SUPERBLOCK_SIZE as u64 > len {
            continue;
        }
        let mut buffer = [0u8; SUPERBLOCK_SIZE];
        if read_exact_at(file, &mut buffer, offset).is_err() {
            continue;
        }
        let Ok(superblock) = Superblock::decode(&buffer) else {
            continue;
        };
        if superblock.head > len {
            continue;
        }
        match best {
            Some((current, _))
                if current.checkpoint_generation >= superblock.checkpoint_generation => {}
            _ => best = Some((superblock, slot as usize)),
        }
    }
    best.ok_or_else(|| Error::corrupt("no valid superblock found"))
}

fn scan_log(file: &File, superblock: &Superblock) -> Result<(HashMap<u64, Extent>, u64)> {
    let limit = DATA_OFFSET + superblock.log_bytes;
    let file_len = file.metadata()?.len();
    let scan_end = file_len.min(limit);
    let mut offset = DATA_OFFSET;
    let mut index: HashMap<u64, Extent> = HashMap::new();

    while offset + RECORD_HEADER_SIZE as u64 <= scan_end {
        let mut header_buffer = [0u8; RECORD_HEADER_SIZE];
        if read_exact_at(file, &mut header_buffer, offset).is_err() {
            break;
        }
        if header_buffer.iter().all(|byte| *byte == 0) {
            break;
        }
        let Ok(header) = RecordHeader::decode(&header_buffer) else {
            break;
        };
        if header.epoch != superblock.epoch || header.stored_len as usize > MAX_STORED_LEN {
            break;
        }
        if header.page_no >= superblock.capacity_pages {
            break;
        }
        let record_len = header.record_len();
        if offset + record_len > limit {
            break;
        }
        let mut payload = vec![0u8; header.stored_len as usize];
        if read_exact_at(file, &mut payload, offset + RECORD_HEADER_SIZE as u64).is_err() {
            break;
        }
        if crc32c::crc32c(&payload) != header.payload_crc {
            break;
        }
        let extent = Extent {
            offset,
            stored_len: header.stored_len,
            generation: header.generation,
            epoch: header.epoch,
            compressed: header.is_compressed(),
        };
        match index.get(&header.page_no) {
            Some(existing) if existing.generation >= extent.generation => {}
            _ => {
                index.insert(header.page_no, extent);
            }
        }
        offset += record_len;
    }

    if offset < superblock.head {
        return Err(Error::corrupt(format!(
            "log scan stopped at {offset} before checkpoint head {}",
            superblock.head
        )));
    }
    Ok((index, offset))
}

impl Store {
    /// Read from the logical block device. Returns the number of bytes copied,
    /// which is short only at the end of the device.
    pub fn read_at(&self, offset: u64, buf: &mut [u8]) -> Result<usize> {
        if buf.is_empty() || offset >= self.capacity_bytes {
            return Ok(0);
        }
        let len = ((self.capacity_bytes - offset).min(buf.len() as u64)) as usize;
        let mut done = 0;
        while done < len {
            let absolute = offset + done as u64;
            let page_no = absolute / PAGE_SIZE as u64;
            let page_offset = (absolute as usize) % PAGE_SIZE;
            let count = (PAGE_SIZE - page_offset).min(len - done);
            let target = &mut buf[done..done + count];
            if self.cache.read_into(page_no, page_offset, target) {
                Metrics::inc(&self.metrics.cache_hits);
            } else {
                Metrics::inc(&self.metrics.cache_misses);
                // A cold load can race with a write, a flush, and an eviction.
                // Publish the loaded page only if the index still names that
                // generation, and copy the requested range under the shard lock
                // so the caller never observes a stale version.
                loop {
                    if self.cache.read_into(page_no, page_offset, target) {
                        break;
                    }
                    let (page, generation) = self.load_page_data(page_no)?;
                    match self.publish_clean_page(page_no, generation, page, page_offset, target)? {
                        CleanPublish::Written => break,
                        CleanPublish::Cached | CleanPublish::Retry => continue,
                    }
                }
                Metrics::inc(&self.metrics.page_loads);
            }
            done += count;
        }
        Metrics::inc(&self.metrics.reads);
        Metrics::add(&self.metrics.bytes_read, len as u64);
        Ok(len)
    }

    /// Write to the logical block device. Data is admitted to RAM immediately;
    /// durability is provided by [`Store::flush`].
    pub fn write_at(&self, offset: u64, buf: &[u8]) -> Result<usize> {
        if buf.is_empty() || offset >= self.capacity_bytes {
            return Ok(0);
        }
        let len = ((self.capacity_bytes - offset).min(buf.len() as u64)) as usize;
        let mut done = 0;
        while done < len {
            let absolute = offset + done as u64;
            let page_no = absolute / PAGE_SIZE as u64;
            let page_offset = (absolute as usize) % PAGE_SIZE;
            let count = (PAGE_SIZE - page_offset).min(len - done);
            if page_offset == 0 && count == PAGE_SIZE {
                self.cache_write_full(page_no, &buf[done..done + count])?;
            } else {
                let source = &buf[done..done + count];
                self.modify_page(page_no, page_offset, count, |page| {
                    page.copy_from_slice(source);
                })?;
            }
            done += count;
        }
        Metrics::inc(&self.metrics.writes);
        Metrics::add(&self.metrics.bytes_written, len as u64);
        Ok(len)
    }

    /// Zero a range. This is used for NBD `TRIM`/`WRITE_ZEROES` and is
    /// immediately visible to readers, even before it reaches the cold log.
    pub fn trim(&self, offset: u64, len: u64) -> Result<()> {
        let end = offset.saturating_add(len).min(self.capacity_bytes);
        if offset >= end {
            return Ok(());
        }
        let mut position = offset;
        while position < end {
            let page_no = position / PAGE_SIZE as u64;
            let page_offset = (position as usize) % PAGE_SIZE;
            let count = (PAGE_SIZE - page_offset).min((end - position) as usize);
            if page_offset == 0 && count == PAGE_SIZE {
                self.cache_write_full(page_no, &ZERO_PAGE)?;
            } else {
                self.modify_page(page_no, page_offset, count, |page| {
                    page.fill(0);
                })?;
            }
            position += count as u64;
        }
        Metrics::inc(&self.metrics.trims);
        Ok(())
    }

    fn load_page_data(&self, page_no: u64) -> Result<(Box<AlignedPage>, u64)> {
        // Take the file lock before the index lock. Compaction takes them in
        // the same order, so an extent read here cannot refer to a file that
        // has already been renamed away.
        let guard = self.file.read();
        let file = guard.as_ref().ok_or(Error::Closed)?;
        let extent = {
            let index = self.index.read();
            index.get(&page_no).copied()
        };
        let Some(extent) = extent else {
            return Ok((AlignedPage::zeroed(), 0));
        };
        if extent.offset < DATA_OFFSET
            || extent.offset + extent.record_len() > DATA_OFFSET + self.log_bytes
        {
            return Err(Error::corrupt(format!(
                "index extent for page {page_no} is outside the log"
            )));
        }
        let mut header_buffer = [0u8; RECORD_HEADER_SIZE];
        read_exact_at(file, &mut header_buffer, extent.offset).map_err(|error| {
            Error::corrupt(format!(
                "reading record header at {}: {error}",
                extent.offset
            ))
        })?;
        let header = RecordHeader::decode(&header_buffer)?;
        if header.page_no != page_no
            || header.generation != extent.generation
            || header.epoch != extent.epoch
            || header.stored_len != extent.stored_len
            || header.is_compressed() != extent.compressed
        {
            return Err(Error::corrupt(format!(
                "record for page {page_no} does not match the index"
            )));
        }
        let stored = header.stored_len as usize;
        let payload_offset = extent.offset + RECORD_HEADER_SIZE as u64;
        let mut data = AlignedPage::zeroed();
        if header.is_compressed() {
            let mut payload = [0u8; MAX_STORED_LEN];
            read_exact_at(file, &mut payload[..stored], payload_offset)?;
            if crc32c::crc32c(&payload[..stored]) != header.payload_crc {
                return Err(Error::corrupt(format!(
                    "payload checksum mismatch at {}",
                    extent.offset
                )));
            }
            let written = lz4_flex::block::decompress_into(&payload[..stored], &mut data.0[..])
                .map_err(|error| Error::Compression(format!("{error:?}")))?;
            if written != PAGE_SIZE {
                return Err(Error::corrupt(format!(
                    "page {page_no} decompressed to {written} bytes"
                )));
            }
        } else {
            if stored != PAGE_SIZE {
                return Err(Error::corrupt(format!(
                    "page {page_no} has {stored} stored bytes"
                )));
            }
            // Read the payload straight into the aligned cache page. This is
            // the zero-bounce-buffer path: one kernel copy, no intermediate
            // userspace allocation.
            read_exact_at(file, &mut data.0, payload_offset)?;
            if crc32c::crc32c(&data.0) != header.payload_crc {
                return Err(Error::corrupt(format!(
                    "payload checksum mismatch at {}",
                    extent.offset
                )));
            }
        }
        Ok((data, extent.generation))
    }

    fn cache_write_full(&self, page_no: u64, data: &[u8]) -> Result<()> {
        if data.len() != PAGE_SIZE {
            return Err(Error::Config(
                "page writes must be exactly 4096 bytes".into(),
            ));
        }
        loop {
            {
                let mut shard = self.cache.shard_for(page_no).lock.write();
                if let Some(page) = shard.pages.get_mut(&page_no) {
                    let generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
                    page.data.0.copy_from_slice(data);
                    page.dirty = true;
                    page.referenced.store(true, Ordering::Relaxed);
                    page.generation = generation;
                    return Ok(());
                }
                if shard.pages.len() < shard.capacity {
                    let generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
                    let mut buffer = shard.acquire();
                    buffer.0.copy_from_slice(data);
                    shard.insert(page_no, CachePage::new(buffer, generation, true));
                    return Ok(());
                }
            }
            // The shard is full. Make room without holding its lock; eviction
            // may need to append a dirty page to the cold log.
            self.make_space(page_no)?;
        }
    }

    /// Read-modify-write a sub-page range.
    ///
    /// A cold load happens before the shard lock is taken so a partial write
    /// never stalls hot readers. If another thread inserts the page while the
    /// load is in flight, the modification is replayed against the cached page
    /// under the lock. If a flush publishes a newer version of the page while
    /// the load is in flight, the load is retried, so a partial write can never
    /// resurrect an older page body.
    fn modify_page<F>(&self, page_no: u64, page_offset: usize, len: usize, modify: F) -> Result<()>
    where
        F: Fn(&mut [u8]),
    {
        if len == 0 || page_offset + len > PAGE_SIZE {
            return Err(Error::Config("invalid page range".into()));
        }
        let end = page_offset + len;
        loop {
            // Fast path: the page is already cached.
            {
                let mut shard = self.cache.shard_for(page_no).lock.write();
                if let Some(page) = shard.pages.get_mut(&page_no) {
                    let generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
                    modify(&mut page.data.0[page_offset..end]);
                    page.dirty = true;
                    page.referenced.store(true, Ordering::Relaxed);
                    page.generation = generation;
                    return Ok(());
                }
                if shard.pages.len() >= shard.capacity {
                    drop(shard);
                    self.make_space(page_no)?;
                    continue;
                }
            }

            // Cold path. Keep the loaded page across make_space retries so a
            // full shard does not force a second cold read.
            let (mut page, mut generation) = self.load_page_data(page_no)?;
            modify(&mut page.0[page_offset..end]);
            loop {
                // Validate freshness under the index read lock, then apply the
                // modification to whichever page is newest.
                let index = self.index.read();
                let current = index.get(&page_no).map_or(0, |extent| extent.generation);
                if current != generation {
                    drop(index);
                    (page, generation) = self.load_page_data(page_no)?;
                    modify(&mut page.0[page_offset..end]);
                    continue;
                }
                let mut shard = self.cache.shard_for(page_no).lock.write();
                if shard.pages.contains_key(&page_no) {
                    let new_generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
                    let cached = shard.pages.get_mut(&page_no).expect("page is present");
                    modify(&mut cached.data.0[page_offset..end]);
                    cached.dirty = true;
                    cached.referenced.store(true, Ordering::Relaxed);
                    cached.generation = new_generation;
                    shard.recycle(page);
                    return Ok(());
                }
                if shard.pages.len() >= shard.capacity {
                    drop(shard);
                    drop(index);
                    self.make_space(page_no)?;
                    continue;
                }
                let new_generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
                shard.insert(page_no, CachePage::new(page, new_generation, true));
                return Ok(());
            }
        }
    }

    fn publish_clean_page(
        &self,
        page_no: u64,
        generation: u64,
        page: Box<AlignedPage>,
        page_offset: usize,
        target: &mut [u8],
    ) -> Result<CleanPublish> {
        debug_assert!(page_offset + target.len() <= PAGE_SIZE);
        loop {
            // Hold the index read while inserting. Without this check a clean
            // page loaded before a flush could be inserted after that flush's
            // newer record became visible, leaving a stale clean page in the
            // cache.
            //
            // This is the only path that takes an index lock before a shard
            // lock. It never takes the writer or file locks, and no path holds
            // a shard lock while waiting for the index, so the order is safe.
            let index = self.index.read();
            let current = index.get(&page_no).map_or(0, |extent| extent.generation);
            if current != generation {
                return Ok(CleanPublish::Retry);
            }
            let mut shard = self.cache.shard_for(page_no).lock.write();
            if shard.pages.contains_key(&page_no) {
                shard.recycle(page);
                return Ok(CleanPublish::Cached);
            }
            if shard.pages.len() >= shard.capacity {
                drop(shard);
                drop(index);
                // make_space appends dirty victims and needs the writer, file,
                // and index locks. Those must not be taken while the index read
                // guard is held. The loaded page stays alive across the retry so
                // a full cache does not force a second cold read.
                if let Err(error) = self.make_space(page_no) {
                    Metrics::inc(&self.metrics.cache_insert_failures);
                    return Err(error);
                }
                continue;
            }
            target.copy_from_slice(&page.0[page_offset..page_offset + target.len()]);
            shard.insert(page_no, CachePage::new(page, generation, false));
            return Ok(CleanPublish::Written);
        }
    }

    /// Ensure the shard that owns `page_no` has a free slot.
    ///
    /// Clean pages are evicted directly. Dirty pages are snapshotted, appended
    /// while the shard lock is released, and then removed once the new record
    /// is in the index. Keeping the dirty page visible until the append commits
    /// prevents a reader from observing the older cold copy.
    fn make_space(&self, page_no: u64) -> Result<()> {
        let mut rounds = 0usize;
        loop {
            let snapshot = {
                let mut shard = self.cache.shard_for(page_no).lock.write();
                if shard.pages.len() < shard.capacity {
                    return Ok(());
                }
                if let Some(victim) = shard.evict_clean() {
                    shard.recycle(victim.data);
                    Metrics::inc(&self.metrics.evictions_clean);
                    return Ok(());
                }
                shard.dirty_snapshot()
            };

            let Some((victim_no, data, generation)) = snapshot else {
                return Err(Error::CacheFull);
            };
            self.spill_page(victim_no, &data.0, generation)?;
            Metrics::inc(&self.metrics.evictions_dirty);

            let removed = {
                let mut shard = self.cache.shard_for(victim_no).lock.write();
                let unchanged = shard
                    .pages
                    .get(&victim_no)
                    .is_some_and(|page| page.generation == generation);
                if unchanged {
                    shard.remove(victim_no)
                } else {
                    shard.mark_clean(victim_no, generation);
                    None
                }
            };
            let mut shard = self.cache.shard_for(page_no).lock.write();
            shard.recycle(data);
            if let Some(page) = removed {
                shard.recycle(page.data);
                return Ok(());
            }
            drop(shard);

            rounds += 1;
            if rounds > 256 {
                return Err(Error::CacheFull);
            }
        }
    }

    fn spill_page(&self, page_no: u64, data: &[u8], generation: u64) -> Result<()> {
        let extents = self.append_pages(&[(page_no, data, generation)])?;
        if extents.is_empty() {
            return Err(Error::corrupt("append returned no extent"));
        }
        Metrics::inc(&self.metrics.page_stores);
        Ok(())
    }
}

/// Outcome of publishing a page loaded from the cold tier.
enum CleanPublish {
    /// The page was inserted and the requested range was copied into the
    /// caller's buffer.
    Written,
    /// Another thread already has a version of the page in the cache. The
    /// caller should read the cached page instead.
    Cached,
    /// The index moved on or the shard was full. The caller should retry; a
    /// retry either hits the cache or reloads the newest cold record.
    Retry,
}

fn read_record(file: &File, offset: u64) -> Result<(RecordHeader, Vec<u8>)> {
    let mut header_buffer = [0u8; RECORD_HEADER_SIZE];
    read_exact_at(file, &mut header_buffer, offset)
        .map_err(|error| Error::corrupt(format!("reading record header at {offset}: {error}")))?;
    let header = RecordHeader::decode(&header_buffer)?;
    let mut payload = vec![0u8; header.stored_len as usize];
    read_exact_at(file, &mut payload, offset + RECORD_HEADER_SIZE as u64)
        .map_err(|error| Error::corrupt(format!("reading record payload at {offset}: {error}")))?;
    if crc32c::crc32c(&payload) != header.payload_crc {
        return Err(Error::corrupt(format!(
            "payload checksum mismatch at {offset}"
        )));
    }
    Ok((header, payload))
}

impl Store {
    /// Make every write that completed before this call durable.
    ///
    /// The generation high-water mark is captured first, so writes that begin
    /// while a flush is in progress are not chased indefinitely. They remain
    /// dirty and will be covered by the next flush.
    pub fn flush(&self) -> Result<()> {
        // Serialize flushers so two callers cannot append the same dirty batch
        // twice. The hot data path never takes this lock.
        let _flush = self.flush_lock.lock();
        let mark = self.next_generation.load(Ordering::SeqCst);
        let mut did_work = false;

        loop {
            let mut batch: Vec<DirtyPage> = Vec::new();
            'shards: for shard in self.cache.shards() {
                let guard = shard.lock.read();
                for (page_no, page) in &guard.pages {
                    if page.dirty && page.generation < mark {
                        batch.push((*page_no, page.data.clone(), page.generation));
                        if batch.len() >= MAX_FLUSH_BATCH {
                            break 'shards;
                        }
                    }
                }
            }
            if batch.is_empty() {
                break;
            }

            let references: Vec<PageWrite<'_>> = batch
                .iter()
                .map(|(page_no, data, generation)| (*page_no, &data.0[..], *generation))
                .collect();
            let extents = self.append_pages(&references)?;
            if extents.len() != batch.len() {
                return Err(Error::corrupt("append returned the wrong extent count"));
            }

            for (page_no, _, generation) in &batch {
                let mut shard = self.cache.shard_for(*page_no).lock.write();
                shard.mark_clean(*page_no, *generation);
            }

            Metrics::add(&self.metrics.page_stores, batch.len() as u64);
            did_work = true;
        }

        let checkpoint_needed = self.writer.lock().needs_checkpoint;
        if checkpoint_needed {
            let mut writer = self.writer.lock();
            if writer.needs_checkpoint {
                let guard = self.file.read();
                let file = guard.as_ref().ok_or(Error::Closed)?;
                self.checkpoint_locked(&mut writer, file)?;
                writer.needs_checkpoint = false;
                did_work = true;
            }
        }

        if did_work {
            Metrics::inc(&self.metrics.flushes);
        }
        Ok(())
    }

    /// Flush dirty pages and rewrite the log so it contains only live pages.
    pub fn compact(&self) -> Result<()> {
        self.flush()?;
        let mut writer = self.writer.lock();
        self.compact_locked(&mut writer)
    }

    /// Read every live cold record, verify its checksum, and decompress it.
    pub fn verify(&self) -> Result<VerifyReport> {
        self.flush()?;
        let started = std::time::Instant::now();
        // Hold the writer and a file read lock for the whole scan. Appends and
        // compaction therefore cannot move the head or replace the file while
        // the report is being built.
        let writer = self.writer.lock();
        let guard = self.file.read();
        let file = guard.as_ref().ok_or(Error::Closed)?;
        let epoch = writer.epoch;
        let head = writer.head;
        let live: Vec<(u64, Extent)> = self
            .index
            .read()
            .iter()
            .map(|(page_no, extent)| (*page_no, *extent))
            .collect();
        let mut report = VerifyReport::default();

        for (page_no, extent) in live {
            if extent.epoch != epoch
                || extent.offset < DATA_OFFSET
                || extent.offset + extent.record_len() > head
            {
                return Err(Error::corrupt(format!(
                    "page {page_no} has an invalid extent"
                )));
            }
            let (header, payload) = read_record(file, extent.offset)?;
            if header.page_no != page_no
                || header.generation != extent.generation
                || header.epoch != epoch
                || header.stored_len != extent.stored_len
            {
                return Err(Error::corrupt(format!(
                    "page {page_no} record does not match the index"
                )));
            }
            if header.is_compressed() {
                let mut output = [0u8; PAGE_SIZE];
                let written = lz4_flex::block::decompress_into(&payload, &mut output)
                    .map_err(|error| Error::Compression(format!("{error:?}")))?;
                if written != PAGE_SIZE {
                    return Err(Error::corrupt(format!(
                        "page {page_no} decompressed to {written} bytes"
                    )));
                }
                report.compressed_pages += 1;
            }
            report.pages += 1;
            report.raw_bytes += PAGE_SIZE as u64;
            report.stored_bytes += header.stored_len as u64;
        }

        report.elapsed_ms = started.elapsed().as_millis();
        Ok(report)
    }

    fn append_pages(&self, pages: &[PageWrite<'_>]) -> Result<Vec<Extent>> {
        if pages.is_empty() {
            return Ok(Vec::new());
        }
        // Compress and checksum outside the writer lock. Large flushes use the
        // work-stealing pool so payload encoding scales across all cores; the
        // actual log append remains a single ordered batch.
        let encoded: Vec<EncodedPage> = if pages.len() >= 8 {
            pages
                .par_iter()
                .map(|(page_no, data, generation)| {
                    encode_page(*page_no, *data, *generation, self.compress)
                })
                .collect::<Vec<Result<EncodedPage>>>()
                .into_iter()
                .collect::<Result<Vec<_>>>()?
        } else {
            pages
                .iter()
                .map(|(page_no, data, generation)| {
                    encode_page(*page_no, *data, *generation, self.compress)
                })
                .collect::<Result<Vec<_>>>()?
        };
        self.append_encoded(&encoded)
    }

    fn append_encoded(&self, pages: &[EncodedPage]) -> Result<Vec<Extent>> {
        if pages.is_empty() {
            return Ok(Vec::new());
        }
        let mut needed: u64 = 0;
        for page in pages {
            needed = needed
                .checked_add(align_up(
                    RECORD_HEADER_SIZE as u64 + page.payload.len() as u64,
                    8,
                ))
                .ok_or_else(|| Error::Config("batch is too large".into()))?;
        }
        let capacity = usize::try_from(needed)
            .map_err(|_| Error::Config("batch is too large for this platform".into()))?;

        let mut writer = self.writer.lock();
        loop {
            let end = writer.head.checked_add(needed).ok_or(Error::LogFull {
                needed,
                available: 0,
            })?;
            if end <= DATA_OFFSET + writer.log_bytes {
                break;
            }
            self.compact_locked(&mut writer)?;
            let end = writer.head.checked_add(needed).ok_or(Error::LogFull {
                needed,
                available: 0,
            })?;
            if end > DATA_OFFSET + writer.log_bytes {
                return Err(Error::LogFull {
                    needed,
                    available: (DATA_OFFSET + writer.log_bytes).saturating_sub(writer.head),
                });
            }
        }

        let guard = self.file.read();
        let file = guard.as_ref().ok_or(Error::Closed)?;
        let mut batch = Vec::with_capacity(capacity);
        let mut extents = Vec::with_capacity(pages.len());
        for page in pages {
            let offset = writer.head + batch.len() as u64;
            let header = RecordHeader {
                flags: if page.compressed {
                    RECORD_FLAG_COMPRESSED
                } else {
                    0
                },
                page_no: page.page_no,
                generation: page.generation,
                epoch: writer.epoch,
                raw_len: PAGE_SIZE as u32,
                stored_len: page.payload.len() as u32,
                payload_crc: page.payload_crc,
            };
            let record_len = header.record_len() as usize;
            let start = batch.len();
            batch.resize(start + record_len, 0);
            batch[start..start + RECORD_HEADER_SIZE].copy_from_slice(&header.encode());
            let payload_start = start + RECORD_HEADER_SIZE;
            batch[payload_start..payload_start + page.payload.len()]
                .copy_from_slice(&page.payload);
            extents.push(Extent {
                offset,
                stored_len: header.stored_len,
                generation: header.generation,
                epoch: header.epoch,
                compressed: header.is_compressed(),
            });
        }

        // One positioned write for the entire batch. This is the access
        // pattern flash controllers handle best and removes one syscall per
        // page from the flush path.
        write_all_at(file, &batch, writer.head)?;
        writer.head += batch.len() as u64;
        file.sync_data()?;
        if let Err(error) = self.checkpoint_locked(&mut writer, file) {
            writer.needs_checkpoint = true;
            return Err(error);
        }
        writer.needs_checkpoint = false;

        // Publish the new extents while the writer lock is still held. This
        // keeps compaction from replacing the index between the append and the
        // visibility update.
        {
            let mut index = self.index.write();
            for (page, extent) in pages.iter().zip(extents.iter()) {
                match index.get(&page.page_no) {
                    Some(existing) if existing.generation > extent.generation => {}
                    _ => {
                        index.insert(page.page_no, *extent);
                    }
                }
            }
        }
        for page in pages {
            if page.compressed {
                Metrics::inc(&self.metrics.compressed_pages);
            } else {
                Metrics::inc(&self.metrics.uncompressed_pages);
            }
            Metrics::add(&self.metrics.stored_bytes, page.payload.len() as u64);
            Metrics::add(&self.metrics.raw_bytes, PAGE_SIZE as u64);
        }
        Ok(extents)
    }

    fn checkpoint_locked(&self, writer: &mut WriterState, file: &File) -> Result<()> {
        let generation = writer
            .checkpoint_generation
            .checked_add(1)
            .ok_or_else(|| Error::corrupt("checkpoint generation overflow"))?;
        let target = 1 - writer.superblock_slot;
        let superblock = Superblock::new(
            self.capacity_pages,
            writer.log_bytes,
            writer.head,
            writer.epoch,
            generation,
            self.compress,
        );
        write_all_at(
            file,
            &superblock.encode(),
            target as u64 * SUPERBLOCK_SIZE as u64,
        )?;
        writer.checkpoint_generation = generation;
        writer.superblock_slot = target;
        file.sync_data()?;
        Ok(())
    }

    fn compact_locked(&self, writer: &mut WriterState) -> Result<()> {
        let mut file_guard = self.file.write();
        let old_file = file_guard.as_ref().ok_or(Error::Closed)?.try_clone()?;
        let live: Vec<(u64, Extent)> = self
            .index
            .read()
            .iter()
            .map(|(page_no, extent)| (*page_no, *extent))
            .collect();

        let compact_path = sidecar(&self.config.path, ".compact");
        let compact = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(&compact_path)?;

        let new_epoch = writer
            .epoch
            .checked_add(1)
            .ok_or_else(|| Error::corrupt("epoch overflow"))?;
        let new_generation = writer
            .checkpoint_generation
            .checked_add(1)
            .ok_or_else(|| Error::corrupt("checkpoint generation overflow"))?;
        let mut new_head = DATA_OFFSET;
        let mut new_index = HashMap::with_capacity(live.len());
        let mut batch: Vec<u8> = Vec::with_capacity(COMPACT_BATCH_BYTES + MAX_RECORD_LEN as usize);
        let mut payload = Vec::new();

        for (page_no, extent) in live {
            if extent.offset < DATA_OFFSET
                || extent.offset + extent.record_len() > DATA_OFFSET + writer.log_bytes
            {
                return Err(Error::corrupt(format!(
                    "index extent for page {page_no} is outside the log"
                )));
            }
            let mut header_buffer = [0u8; RECORD_HEADER_SIZE];
            read_exact_at(&old_file, &mut header_buffer, extent.offset)?;
            let header = RecordHeader::decode(&header_buffer)?;
            if header.page_no != page_no
                || header.generation != extent.generation
                || header.epoch != extent.epoch
                || header.stored_len != extent.stored_len
            {
                return Err(Error::corrupt(format!(
                    "record for page {page_no} does not match the index"
                )));
            }
            payload.resize(header.stored_len as usize, 0);
            read_exact_at(
                &old_file,
                &mut payload,
                extent.offset + RECORD_HEADER_SIZE as u64,
            )?;
            if crc32c::crc32c(&payload) != header.payload_crc {
                return Err(Error::corrupt(format!(
                    "payload checksum mismatch for page {page_no}"
                )));
            }
            let new_header = RecordHeader {
                epoch: new_epoch,
                ..header
            };
            let record_len = new_header.record_len();
            let record_offset = new_head
                .checked_add(batch.len() as u64)
                .ok_or(Error::LogFull {
                    needed: record_len,
                    available: 0,
                })?;
            let end = record_offset
                .checked_add(record_len)
                .ok_or(Error::LogFull {
                    needed: record_len,
                    available: 0,
                })?;
            if end > DATA_OFFSET + writer.log_bytes {
                return Err(Error::LogFull {
                    needed: record_len,
                    available: (DATA_OFFSET + writer.log_bytes).saturating_sub(record_offset),
                });
            }
            let start = batch.len();
            batch.resize(start + record_len as usize, 0);
            batch[start..start + RECORD_HEADER_SIZE].copy_from_slice(&new_header.encode());
            let payload_start = start + RECORD_HEADER_SIZE;
            batch[payload_start..payload_start + payload.len()].copy_from_slice(&payload);
            new_index.insert(
                page_no,
                Extent {
                    offset: record_offset,
                    stored_len: new_header.stored_len,
                    generation: new_header.generation,
                    epoch: new_header.epoch,
                    compressed: new_header.is_compressed(),
                },
            );
            if batch.len() >= COMPACT_BATCH_BYTES {
                write_all_at(&compact, &batch, new_head)?;
                new_head += batch.len() as u64;
                batch.clear();
            }
        }
        if !batch.is_empty() {
            write_all_at(&compact, &batch, new_head)?;
            new_head += batch.len() as u64;
            batch.clear();
        }

        compact.sync_all()?;
        let superblock = Superblock::new(
            self.capacity_pages,
            writer.log_bytes,
            new_head,
            new_epoch,
            new_generation,
            self.compress,
        );
        write_all_at(&compact, &superblock.encode(), 0)?;
        write_all_at(&compact, &[0u8; SUPERBLOCK_SIZE], SUPERBLOCK_SIZE as u64)?;
        compact.sync_all()?;
        drop(compact);
        drop(old_file);

        // Close the old handle before renaming on Windows.
        let old_handle = file_guard.take();
        drop(old_handle);

        let old_path = sidecar(&self.config.path, ".old");
        let _ = std::fs::remove_file(&old_path);
        if let Err(error) = std::fs::rename(&self.config.path, &old_path) {
            if let Ok(reopened) = OpenOptions::new()
                .read(true)
                .write(true)
                .open(&self.config.path)
            {
                *file_guard = Some(reopened);
            }
            return Err(error.into());
        }
        if let Err(error) = std::fs::rename(&compact_path, &self.config.path) {
            let _ = std::fs::rename(&old_path, &self.config.path);
            if let Ok(reopened) = OpenOptions::new()
                .read(true)
                .write(true)
                .open(&self.config.path)
            {
                *file_guard = Some(reopened);
            }
            return Err(error.into());
        }
        let _ = std::fs::remove_file(&old_path);
        let new_file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&self.config.path)?;
        *file_guard = Some(new_file);

        *self.index.write() = new_index;
        writer.head = new_head;
        writer.epoch = new_epoch;
        writer.checkpoint_generation = new_generation;
        writer.superblock_slot = 0;
        writer.needs_checkpoint = false;
        Metrics::inc(&self.metrics.compactions);
        Ok(())
    }
}

/// Compress (or copy) one page payload and checksum it. The result is
/// independent of the log epoch, which lets callers encode a whole batch in
/// parallel before taking the writer lock.
fn encode_page(
    page_no: u64,
    data: &[u8],
    generation: u64,
    compress: bool,
) -> Result<EncodedPage> {
    if data.len() != PAGE_SIZE {
        return Err(Error::Config(
            "page writes must be exactly 4096 bytes".into(),
        ));
    }
    if compress {
        let maximum = lz4_flex::block::get_maximum_output_size(PAGE_SIZE);
        let mut payload = vec![0u8; maximum];
        match lz4_flex::block::compress_into(data, &mut payload) {
            Ok(length) if length < data.len() => {
                payload.truncate(length);
                Ok(EncodedPage {
                    page_no,
                    generation,
                    payload_crc: crc32c::crc32c(&payload),
                    payload,
                    compressed: true,
                })
            }
            Ok(_) => Ok(EncodedPage {
                page_no,
                generation,
                payload_crc: crc32c::crc32c(data),
                payload: data.to_vec(),
                compressed: false,
            }),
            Err(error) => Err(Error::Compression(format!("{error:?}"))),
        }
    } else {
        Ok(EncodedPage {
            page_no,
            generation,
            payload_crc: crc32c::crc32c(data),
            payload: data.to_vec(),
            compressed: false,
        })
    }
}

impl std::fmt::Debug for Store {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("Store")
            .field("path", &self.config.path)
            .field("capacity_bytes", &self.capacity_bytes)
            .field("cache_bytes", &self.cache_bytes)
            .field("log_bytes", &self.log_bytes)
            .field("compressed", &self.compress)
            .finish_non_exhaustive()
    }
}

impl Drop for Store {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}

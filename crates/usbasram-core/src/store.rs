use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use parking_lot::{Mutex, RwLock};

use crate::cache::{CachePage, PageCache, Shard};
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

type DirtyPage = (u64, Box<[u8; PAGE_SIZE]>, u64);
type PageWrite<'a> = (u64, &'a [u8], u64);

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
    /// If `None`, [`min_log_bytes`] is used. A smaller log is rejected because
    /// compaction must be able to hold every page in the worst case.
    pub log_bytes: Option<u64>,
    /// Compress cold pages with LZ4 before writing them.
    pub compress: bool,
    /// Create the store if the backing file does not exist.
    pub create: bool,
}

impl StoreConfig {
    /// Create a configuration with compression enabled, a 256 MiB hot cache,
    /// and a log sized for the full logical capacity.
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
            return Err(Error::Config("cache_bytes must be greater than zero".into()));
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
                return Err(Error::Config(
                    "log_bytes must be a multiple of 4096".into(),
                ));
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
    let per_record = (PAGE_SIZE + RECORD_HEADER_SIZE) as u64;
    records
        .checked_mul(per_record)
        .map(|bytes| align_up(bytes, PAGE_SIZE as u64))
        .ok_or_else(|| Error::Config("store is too large for this format".into()))
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

        let mut file = OpenOptions::new()
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
            let minimum = checked_min_log_bytes(config.size_bytes)?;
            let log_bytes = config.log_bytes.unwrap_or(minimum);
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
        let max_generation = index.values().map(|extent| extent.generation).max().unwrap_or(0);
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
            Some((current, _)) if current.checkpoint_generation >= superblock.checkpoint_generation => {}
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
                let (page, generation) = self.load_page_data(page_no)?;
                target.copy_from_slice(&page[page_offset..page_offset + count]);
                if self.cache_insert_clean(page_no, page, generation).is_err() {
                    Metrics::inc(&self.metrics.cache_insert_failures);
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
            let generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
            if page_offset == 0 && count == PAGE_SIZE {
                self.cache_write_full(page_no, &buf[done..done + count], generation)?;
            } else {
                let source = &buf[done..done + count];
                self.modify_page(page_no, page_offset, count, generation, |page| {
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
            let generation = self.next_generation.fetch_add(1, Ordering::SeqCst);
            if page_offset == 0 && count == PAGE_SIZE {
                self.cache_write_full(page_no, &ZERO_PAGE, generation)?;
            } else {
                self.modify_page(page_no, page_offset, count, generation, |page| {
                    page.fill(0);
                })?;
            }
            position += count as u64;
        }
        Metrics::inc(&self.metrics.trims);
        Ok(())
    }

    fn load_page_data(&self, page_no: u64) -> Result<(Box<[u8; PAGE_SIZE]>, u64)> {
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
            return Ok((Box::new([0u8; PAGE_SIZE]), 0));
        };
        if extent.offset < DATA_OFFSET
            || extent.offset + extent.record_len() > DATA_OFFSET + self.log_bytes
        {
            return Err(Error::corrupt(format!(
                "index extent for page {page_no} is outside the log"
            )));
        }
        let (header, payload) = read_record(file, extent.offset)?;
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
        let mut data = Box::new([0u8; PAGE_SIZE]);
        if header.is_compressed() {
            let written = lz4_flex::block::decompress_into(&payload, &mut data[..])
                .map_err(|error| Error::Compression(format!("{error:?}")))?;
            if written != PAGE_SIZE {
                return Err(Error::corrupt(format!(
                    "page {page_no} decompressed to {written} bytes"
                )));
            }
        } else {
            if payload.len() != PAGE_SIZE {
                return Err(Error::corrupt(format!(
                    "page {page_no} has {} stored bytes",
                    payload.len()
                )));
            }
            data.copy_from_slice(&payload);
        }
        Ok((data, extent.generation))
    }

    fn cache_write_full(&self, page_no: u64, data: &[u8], generation: u64) -> Result<()> {
        if data.len() != PAGE_SIZE {
            return Err(Error::Config("page writes must be exactly 4096 bytes".into()));
        }
        let mut shard = self.cache.shard_for(page_no).lock();
        if let Some(page) = shard.pages.get_mut(&page_no) {
            page.data.copy_from_slice(data);
            page.dirty = true;
            page.referenced = true;
            page.generation = generation;
            return Ok(());
        }
        self.ensure_space_locked(&mut shard)?;
        let mut page = Box::new([0u8; PAGE_SIZE]);
        page.copy_from_slice(data);
        shard.insert(page_no, CachePage::new(page, generation, true));
        Ok(())
    }

    /// Read-modify-write a sub-page range while holding the cache shard lock.
    ///
    /// The shard lock keeps a full-page write from replacing the page between
    /// the load and the insertion, which would silently lose an update.
    fn modify_page<F>(
        &self,
        page_no: u64,
        page_offset: usize,
        len: usize,
        generation: u64,
        modify: F,
    ) -> Result<()>
    where
        F: FnOnce(&mut [u8]),
    {
        if len == 0 || page_offset + len > PAGE_SIZE {
            return Err(Error::Config("invalid page range".into()));
        }
        let end = page_offset + len;
        let mut shard = self.cache.shard_for(page_no).lock();
        if let Some(page) = shard.pages.get_mut(&page_no) {
            modify(&mut page.data[page_offset..end]);
            page.dirty = true;
            page.referenced = true;
            page.generation = generation;
            return Ok(());
        }
        let (mut page, _) = self.load_page_data(page_no)?;
        modify(&mut page[page_offset..end]);
        self.ensure_space_locked(&mut shard)?;
        shard.insert(page_no, CachePage::new(page, generation, true));
        Ok(())
    }

    fn cache_insert_clean(
        &self,
        page_no: u64,
        data: Box<[u8; PAGE_SIZE]>,
        generation: u64,
    ) -> Result<()> {
        let mut shard = self.cache.shard_for(page_no).lock();
        if shard.pages.contains_key(&page_no) {
            return Ok(());
        }
        self.ensure_space_locked(&mut shard)?;
        shard.insert(page_no, CachePage::new(data, generation, false));
        Ok(())
    }

    fn ensure_space_locked(&self, shard: &mut Shard) -> Result<()> {
        while shard.pages.len() >= shard.capacity {
            let (page_no, victim) = shard.evict_candidate().ok_or(Error::CacheFull)?;
            if victim.dirty {
                if let Err(error) = self.spill_page(page_no, &victim.data[..], victim.generation) {
                    shard.insert(page_no, victim);
                    return Err(error);
                }
                Metrics::inc(&self.metrics.evictions_dirty);
            } else {
                Metrics::inc(&self.metrics.evictions_clean);
            }
        }
        Ok(())
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

fn read_record(file: &File, offset: u64) -> Result<(RecordHeader, Vec<u8>)> {
    let mut header_buffer = [0u8; RECORD_HEADER_SIZE];
    read_exact_at(file, &mut header_buffer, offset).map_err(|error| {
        Error::corrupt(format!("reading record header at {offset}: {error}"))
    })?;
    let header = RecordHeader::decode(&header_buffer)?;
    let mut payload = vec![0u8; header.stored_len as usize];
    read_exact_at(file, &mut payload, offset + RECORD_HEADER_SIZE as u64).map_err(|error| {
        Error::corrupt(format!("reading record payload at {offset}: {error}"))
    })?;
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
        let mark = self.next_generation.load(Ordering::SeqCst);
        let mut did_work = false;

        loop {
            let mut batch: Vec<DirtyPage> = Vec::new();
            for shard in self.cache.shards() {
                let guard = shard.lock();
                for (page_no, page) in &guard.pages {
                    if page.dirty && page.generation < mark {
                        batch.push((*page_no, page.data.clone(), page.generation));
                        if batch.len() >= MAX_FLUSH_BATCH {
                            break;
                        }
                    }
                }
                if batch.len() >= MAX_FLUSH_BATCH {
                    break;
                }
            }
            if batch.is_empty() {
                break;
            }

            let references: Vec<PageWrite<'_>> = batch
                .iter()
                .map(|(page_no, data, generation)| (*page_no, &data[..], *generation))
                .collect();
            let extents = self.append_pages(&references)?;
            if extents.len() != batch.len() {
                return Err(Error::corrupt("append returned the wrong extent count"));
            }

            for (page_no, _, generation) in &batch {
                let mut shard = self.cache.shard_for(*page_no).lock();
                if let Some(page) = shard.pages.get_mut(page_no) {
                    if page.generation == *generation {
                        page.dirty = false;
                    }
                }
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
        let worst = (pages.len() as u64)
            .checked_mul((PAGE_SIZE + RECORD_HEADER_SIZE + 8) as u64)
            .ok_or_else(|| Error::Config("batch is too large".into()))?;

        let mut writer = self.writer.lock();
        loop {
            let end = writer
                .head
                .checked_add(worst)
                .ok_or(Error::LogFull {
                    needed: worst,
                    available: 0,
                })?;
            if end <= DATA_OFFSET + writer.log_bytes {
                break;
            }
            self.compact_locked(&mut writer)?;
            let end = writer
                .head
                .checked_add(worst)
                .ok_or(Error::LogFull {
                    needed: worst,
                    available: 0,
                })?;
            if end > DATA_OFFSET + writer.log_bytes {
                return Err(Error::LogFull {
                    needed: worst,
                    available: (DATA_OFFSET + writer.log_bytes).saturating_sub(writer.head),
                });
            }
        }

        let guard = self.file.read();
        let file = guard.as_ref().ok_or(Error::Closed)?;
        let mut extents = Vec::with_capacity(pages.len());
        for (page_no, data, generation) in pages {
            if data.len() != PAGE_SIZE {
                return Err(Error::Config(
                    "page writes must be exactly 4096 bytes".into(),
                ));
            }
            let (record, header) =
                encode_record(*page_no, data, *generation, writer.epoch, self.compress)?;
            let end = writer
                .head
                .checked_add(record.len() as u64)
                .ok_or(Error::LogFull {
                    needed: record.len() as u64,
                    available: 0,
                })?;
            if end > DATA_OFFSET + writer.log_bytes {
                return Err(Error::LogFull {
                    needed: record.len() as u64,
                    available: (DATA_OFFSET + writer.log_bytes).saturating_sub(writer.head),
                });
            }
            write_all_at(file, &record, writer.head)?;
            let extent = Extent {
                offset: writer.head,
                stored_len: header.stored_len,
                generation: header.generation,
                epoch: header.epoch,
                compressed: header.is_compressed(),
            };
            writer.head = end;
            extents.push(extent);
            if header.is_compressed() {
                Metrics::inc(&self.metrics.compressed_pages);
            } else {
                Metrics::inc(&self.metrics.uncompressed_pages);
            }
            Metrics::add(&self.metrics.stored_bytes, header.stored_len as u64);
            Metrics::add(&self.metrics.raw_bytes, PAGE_SIZE as u64);
        }
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
            for ((page_no, _, _), extent) in pages.iter().zip(extents.iter()) {
                match index.get(page_no) {
                    Some(existing) if existing.generation > extent.generation => {}
                    _ => {
                        index.insert(*page_no, *extent);
                    }
                }
            }
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
            let mut payload = vec![0u8; header.stored_len as usize];
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
            let record = encode_record_with_payload(&new_header, &payload)?;
            let end = new_head
                .checked_add(record.len() as u64)
                .ok_or(Error::LogFull {
                    needed: record.len() as u64,
                    available: 0,
                })?;
            if end > DATA_OFFSET + writer.log_bytes {
                return Err(Error::LogFull {
                    needed: record.len() as u64,
                    available: (DATA_OFFSET + writer.log_bytes).saturating_sub(new_head),
                });
            }
            write_all_at(&compact, &record, new_head)?;
            new_index.insert(
                page_no,
                Extent {
                    offset: new_head,
                    stored_len: new_header.stored_len,
                    generation: new_header.generation,
                    epoch: new_header.epoch,
                    compressed: new_header.is_compressed(),
                },
            );
            new_head = end;
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

fn encode_record(
    page_no: u64,
    data: &[u8],
    generation: u64,
    epoch: u64,
    compress: bool,
) -> Result<(Vec<u8>, RecordHeader)> {
    if data.len() != PAGE_SIZE {
        return Err(Error::Config(
            "page writes must be exactly 4096 bytes".into(),
        ));
    }
    let (payload, flags) = if compress {
        let maximum = lz4_flex::block::get_maximum_output_size(PAGE_SIZE);
        let mut buffer = vec![0u8; maximum];
        match lz4_flex::block::compress_into(data, &mut buffer) {
            Ok(length) if length < data.len() => {
                buffer.truncate(length);
                (buffer, RECORD_FLAG_COMPRESSED)
            }
            Ok(_) => (data.to_vec(), 0),
            Err(error) => return Err(Error::Compression(format!("{error:?}"))),
        }
    } else {
        (data.to_vec(), 0)
    };
    let header = RecordHeader {
        flags,
        page_no,
        generation,
        epoch,
        raw_len: PAGE_SIZE as u32,
        stored_len: payload.len() as u32,
        payload_crc: crc32c::crc32c(&payload),
    };
    let record = encode_record_with_payload(&header, &payload)?;
    Ok((record, header))
}

fn encode_record_with_payload(header: &RecordHeader, payload: &[u8]) -> Result<Vec<u8>> {
    if payload.len() != header.stored_len as usize {
        return Err(Error::corrupt("record payload length does not match header"));
    }
    let length = header.record_len() as usize;
    if length < RECORD_HEADER_SIZE + payload.len() {
        return Err(Error::corrupt("record length is too small"));
    }
    let mut record = Vec::with_capacity(length);
    record.extend_from_slice(&header.encode());
    record.extend_from_slice(payload);
    record.resize(length, 0);
    Ok(record)
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

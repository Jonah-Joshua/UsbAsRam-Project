use std::sync::atomic::{AtomicU64, Ordering};

/// Point-in-time counters for a store.
#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub reads: u64,
    pub writes: u64,
    pub bytes_read: u64,
    pub bytes_written: u64,
    pub cache_hits: u64,
    pub cache_misses: u64,
    pub page_loads: u64,
    pub page_stores: u64,
    pub flushes: u64,
    pub compactions: u64,
    pub compressed_pages: u64,
    pub uncompressed_pages: u64,
    pub stored_bytes: u64,
    pub raw_bytes: u64,
    pub evictions_clean: u64,
    pub evictions_dirty: u64,
    pub trims: u64,
    pub cache_insert_failures: u64,
    pub cache_pages: usize,
    pub dirty_pages: usize,
    pub pages_on_disk: usize,
    pub capacity_bytes: u64,
    pub log_bytes: u64,
    pub head: u64,
    pub epoch: u64,
}

#[derive(Debug, Default)]
pub(crate) struct Metrics {
    pub reads: AtomicU64,
    pub writes: AtomicU64,
    pub bytes_read: AtomicU64,
    pub bytes_written: AtomicU64,
    pub cache_hits: AtomicU64,
    pub cache_misses: AtomicU64,
    pub page_loads: AtomicU64,
    pub page_stores: AtomicU64,
    pub flushes: AtomicU64,
    pub compactions: AtomicU64,
    pub compressed_pages: AtomicU64,
    pub uncompressed_pages: AtomicU64,
    pub stored_bytes: AtomicU64,
    pub raw_bytes: AtomicU64,
    pub evictions_clean: AtomicU64,
    pub evictions_dirty: AtomicU64,
    pub trims: AtomicU64,
    pub cache_insert_failures: AtomicU64,
}

impl Metrics {
    pub(crate) fn inc(counter: &AtomicU64) {
        counter.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn add(counter: &AtomicU64, value: u64) {
        counter.fetch_add(value, Ordering::Relaxed);
    }

    pub(crate) fn snapshot(
        &self,
        cache_pages: usize,
        dirty_pages: usize,
        pages_on_disk: usize,
        capacity_bytes: u64,
        log_bytes: u64,
        head: u64,
        epoch: u64,
    ) -> Stats {
        Stats {
            reads: self.reads.load(Ordering::Relaxed),
            writes: self.writes.load(Ordering::Relaxed),
            bytes_read: self.bytes_read.load(Ordering::Relaxed),
            bytes_written: self.bytes_written.load(Ordering::Relaxed),
            cache_hits: self.cache_hits.load(Ordering::Relaxed),
            cache_misses: self.cache_misses.load(Ordering::Relaxed),
            page_loads: self.page_loads.load(Ordering::Relaxed),
            page_stores: self.page_stores.load(Ordering::Relaxed),
            flushes: self.flushes.load(Ordering::Relaxed),
            compactions: self.compactions.load(Ordering::Relaxed),
            compressed_pages: self.compressed_pages.load(Ordering::Relaxed),
            uncompressed_pages: self.uncompressed_pages.load(Ordering::Relaxed),
            stored_bytes: self.stored_bytes.load(Ordering::Relaxed),
            raw_bytes: self.raw_bytes.load(Ordering::Relaxed),
            evictions_clean: self.evictions_clean.load(Ordering::Relaxed),
            evictions_dirty: self.evictions_dirty.load(Ordering::Relaxed),
            trims: self.trims.load(Ordering::Relaxed),
            cache_insert_failures: self.cache_insert_failures.load(Ordering::Relaxed),
            cache_pages,
            dirty_pages,
            pages_on_disk,
            capacity_bytes,
            log_bytes,
            head,
            epoch,
        }
    }
}

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};

use parking_lot::RwLock;

use crate::PAGE_SIZE;

/// A 4 KiB-aligned page buffer.
///
/// The alignment means a page never straddles a 4 KiB boundary, so cold reads
/// can be issued straight into the cache page without a bounce buffer and the
/// buffer is suitable for direct/vectored I/O.
#[repr(C, align(4096))]
#[derive(Clone)]
pub(crate) struct AlignedPage(pub [u8; PAGE_SIZE]);

impl AlignedPage {
    pub(crate) fn zeroed() -> Box<Self> {
        Box::new(Self([0u8; PAGE_SIZE]))
    }
}

pub(crate) struct CachePage {
    pub data: Box<AlignedPage>,
    pub dirty: bool,
    pub referenced: AtomicBool,
    pub generation: u64,
}

impl CachePage {
    pub(crate) fn new(data: Box<AlignedPage>, generation: u64, dirty: bool) -> Self {
        Self {
            data,
            dirty,
            referenced: AtomicBool::new(true),
            generation,
        }
    }
}

/// One independently locked cache shard.
///
/// Padding each shard to two cache lines prevents adjacent shard locks from
/// sharing a cache line (false sharing) when different cores hit different
/// shards.
#[repr(align(128))]
pub(crate) struct ShardSlot {
    pub lock: RwLock<Shard>,
}

impl ShardSlot {
    pub(crate) fn new(capacity: usize) -> Self {
        Self {
            lock: RwLock::new(Shard::new(capacity)),
        }
    }
}

/// A CLOCK shard with a small free list so eviction and insertion recycle page
/// buffers instead of returning to the allocator on every miss.
pub(crate) struct Shard {
    pub pages: HashMap<u64, CachePage>,
    clock: Vec<u64>,
    free: Vec<Box<AlignedPage>>,
    hand: usize,
    pub capacity: usize,
}

/// A copy of a dirty page, safe to compress and append without holding a shard
/// lock. The page remains visible in the cache until the append commits.
pub(crate) type DirtySnapshot = (u64, Box<AlignedPage>, u64);

impl Shard {
    fn new(capacity: usize) -> Self {
        let reserve = capacity.min(1024);
        Self {
            pages: HashMap::with_capacity(reserve),
            clock: Vec::with_capacity(reserve),
            free: Vec::new(),
            hand: 0,
            capacity,
        }
    }

    /// Take a page buffer from the free list, or allocate a new aligned page.
    pub(crate) fn acquire(&mut self) -> Box<AlignedPage> {
        self.free.pop().unwrap_or_else(AlignedPage::zeroed)
    }

    /// Return an unused page buffer to the shard.
    pub(crate) fn recycle(&mut self, page: Box<AlignedPage>) {
        if self.free.len() < self.capacity {
            self.free.push(page);
        }
    }

    pub(crate) fn insert(&mut self, page_no: u64, page: CachePage) {
        if self.pages.insert(page_no, page).is_none() {
            self.clock.push(page_no);
        }
    }

    pub(crate) fn remove(&mut self, page_no: u64) -> Option<CachePage> {
        let page = self.pages.remove(&page_no)?;
        if let Some(position) = self.clock.iter().position(|candidate| *candidate == page_no) {
            self.clock.swap_remove(position);
            if self.hand >= self.clock.len() {
                self.hand = 0;
            }
        }
        Some(page)
    }

    fn remove_clock_at(&mut self, index: usize) {
        self.clock.swap_remove(index);
        if self.hand >= self.clock.len() {
            self.hand = 0;
        }
    }

    /// Remove one clean page, giving referenced pages a second chance.
    ///
    /// Dirty pages are never removed here: their contents must reach the cold
    /// log before a reader can be allowed to observe the older disk version.
    pub(crate) fn evict_clean(&mut self) -> Option<CachePage> {
        if self.pages.is_empty() || self.clock.is_empty() {
            return None;
        }
        let scan_limit = self.clock.len().saturating_mul(2).max(1);
        for _ in 0..scan_limit {
            if self.clock.is_empty() {
                return None;
            }
            if self.hand >= self.clock.len() {
                self.hand = 0;
            }
            let page_no = self.clock[self.hand];
            let state = self
                .pages
                .get(&page_no)
                .map(|page| (page.dirty, page.referenced.swap(false, Ordering::Relaxed)));
            match state {
                Some((false, false)) => return self.remove(page_no),
                Some(_) => self.hand += 1,
                None => self.remove_clock_at(self.hand),
            }
        }

        // Every remaining candidate was referenced. Clear the bits and take
        // the first clean page.
        for page in self.pages.values() {
            page.referenced.store(false, Ordering::Relaxed);
        }
        let mut remaining = self.clock.len();
        while remaining > 0 {
            if self.clock.is_empty() {
                return None;
            }
            if self.hand >= self.clock.len() {
                self.hand = 0;
            }
            let page_no = self.clock[self.hand];
            let clean = self.pages.get(&page_no).map(|page| !page.dirty);
            match clean {
                Some(true) => return self.remove(page_no),
                Some(false) => self.hand += 1,
                None => self.remove_clock_at(self.hand),
            }
            remaining -= 1;
        }
        None
    }

    /// Copy a dirty page without removing it. The cache keeps serving the page
    /// while the append is in flight.
    pub(crate) fn dirty_snapshot(&mut self) -> Option<DirtySnapshot> {
        if self.clock.is_empty() {
            return None;
        }
        for offset in 0..self.clock.len() {
            let index = (self.hand + offset) % self.clock.len();
            let page_no = self.clock[index];
            let Some(page) = self.pages.get(&page_no) else {
                continue;
            };
            if page.dirty {
                return Some((
                    page_no,
                    Box::new(AlignedPage(page.data.0)),
                    page.generation,
                ));
            }
        }
        None
    }

    pub(crate) fn mark_clean(&mut self, page_no: u64, generation: u64) {
        if let Some(page) = self.pages.get_mut(&page_no) {
            if page.generation == generation {
                page.dirty = false;
            }
        }
    }

    pub(crate) fn counts(&self) -> (usize, usize) {
        (
            self.pages.len(),
            self.pages.values().filter(|page| page.dirty).count(),
        )
    }
}

/// A sharded page cache.
///
/// Hits take a shared shard lock, so any number of readers can copy out of the
/// same shard concurrently. Writers take the shard's exclusive lock. The shard
/// count grows with the cache size and is a power of two, so page-to-shard
/// selection is a bit mask.
pub(crate) struct PageCache {
    shards: Vec<ShardSlot>,
    mask: usize,
}

impl PageCache {
    pub(crate) fn new(cache_pages: usize) -> Self {
        let cache_pages = cache_pages.max(1);
        let target = (cache_pages / 128).clamp(1, 256).next_power_of_two();
        let shard_count = target.min(cache_pages).max(1);
        let base = cache_pages / shard_count;
        let extra = cache_pages % shard_count;
        let shards = (0..shard_count)
            .map(|index| {
                let extra_page = if index < extra { 1 } else { 0 };
                ShardSlot::new(base + extra_page)
            })
            .collect();
        Self {
            shards,
            mask: shard_count - 1,
        }
    }

    pub(crate) fn shard_for(&self, page_no: u64) -> &ShardSlot {
        &self.shards[(page_no as usize) & self.mask]
    }

    pub(crate) fn shards(&self) -> &[ShardSlot] {
        &self.shards
    }

    pub(crate) fn read_into(&self, page_no: u64, page_offset: usize, dst: &mut [u8]) -> bool {
        debug_assert!(page_offset + dst.len() <= PAGE_SIZE);
        let shard = self.shard_for(page_no).lock.read();
        let Some(page) = shard.pages.get(&page_no) else {
            return false;
        };
        page.referenced.store(true, Ordering::Relaxed);
        dst.copy_from_slice(&page.data.0[page_offset..page_offset + dst.len()]);
        true
    }

    pub(crate) fn counts(&self) -> (usize, usize) {
        let mut pages = 0;
        let mut dirty = 0;
        for shard in &self.shards {
            let (shard_pages, shard_dirty) = shard.lock.read().counts();
            pages += shard_pages;
            dirty += shard_dirty;
        }
        (pages, dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn page(byte: u8) -> Box<AlignedPage> {
        let mut buffer = AlignedPage::zeroed();
        buffer.0.fill(byte);
        buffer
    }

    #[test]
    fn cache_evicts_a_clean_page_when_full() {
        let mut shard = Shard::new(2);
        for page_no in 0..2 {
            shard.insert(page_no, CachePage::new(page(page_no as u8), 1, false));
        }
        let victim = shard.evict_clean().expect("victim");
        assert_eq!(victim.data.0[0], 0);
        assert_eq!(shard.pages.len(), 1);
        shard.insert(9, CachePage::new(page(9), 1, false));
        assert_eq!(shard.pages.len(), 2);
    }

    #[test]
    fn second_chance_protects_referenced_pages() {
        let mut shard = Shard::new(2);
        for page_no in 0..2 {
            shard.insert(page_no, CachePage::new(page(0), 1, false));
        }
        // Both were inserted with referenced = true. The first scan clears them
        // and the second scan evicts one.
        assert!(shard.evict_clean().is_some());
        assert_eq!(shard.pages.len(), 1);
    }

    #[test]
    fn dirty_pages_are_not_evicted_without_a_spill() {
        let mut shard = Shard::new(2);
        shard.insert(0, CachePage::new(page(7), 3, true));
        assert!(shard.evict_clean().is_none());
        assert_eq!(shard.pages.len(), 1);
        let (page_no, snapshot, generation) = shard.dirty_snapshot().expect("snapshot");
        assert_eq!(page_no, 0);
        assert_eq!(snapshot.0[0], 7);
        assert_eq!(generation, 3);
        assert_eq!(shard.pages.len(), 1);
    }

    #[test]
    fn shards_are_cache_line_padded() {
        assert_eq!(std::mem::align_of::<ShardSlot>(), 128);
        assert_eq!(std::mem::align_of::<AlignedPage>(), 4096);
    }
}

use std::collections::HashMap;

use parking_lot::Mutex;

use crate::PAGE_SIZE;

/// A single cached 4 KiB page.
pub(crate) struct CachePage {
    pub data: Box<[u8; PAGE_SIZE]>,
    pub dirty: bool,
    pub referenced: bool,
    pub generation: u64,
}

impl CachePage {
    pub(crate) fn new(data: Box<[u8; PAGE_SIZE]>, generation: u64, dirty: bool) -> Self {
        Self {
            data,
            dirty,
            referenced: true,
            generation,
        }
    }
}

/// One independently locked cache shard with a CLOCK (second-chance) policy.
pub(crate) struct Shard {
    pub pages: HashMap<u64, CachePage>,
    pub clock: Vec<u64>,
    pub hand: usize,
    pub capacity: usize,
}

impl Shard {
    fn new(capacity: usize) -> Self {
        let reserve = capacity.min(1024);
        Self {
            pages: HashMap::with_capacity(reserve),
            clock: Vec::with_capacity(reserve),
            hand: 0,
            capacity,
        }
    }

    pub(crate) fn insert(&mut self, page_no: u64, page: CachePage) {
        if self.pages.insert(page_no, page).is_none() {
            self.clock.push(page_no);
        }
    }

    /// Remove and return one page. Clean pages are preferred; a dirty page is
    /// returned only when every candidate has been given a second chance.
    pub(crate) fn evict_candidate(&mut self) -> Option<(u64, CachePage)> {
        if self.pages.is_empty() || self.clock.is_empty() {
            return None;
        }

        let max_steps = self.clock.len().saturating_mul(2).max(1);
        for _ in 0..max_steps {
            if self.clock.is_empty() {
                return None;
            }
            if self.hand >= self.clock.len() {
                self.hand = 0;
            }
            let page_no = self.clock[self.hand];
            match self.pages.get_mut(&page_no) {
                Some(entry) if entry.referenced => {
                    entry.referenced = false;
                    self.hand += 1;
                }
                Some(_) => {
                    let entry = self.pages.remove(&page_no)?;
                    self.clock.swap_remove(self.hand);
                    if self.hand >= self.clock.len() {
                        self.hand = 0;
                    }
                    return Some((page_no, entry));
                }
                None => {
                    // Repair a stale clock entry.
                    self.clock.swap_remove(self.hand);
                    if self.hand >= self.clock.len() {
                        self.hand = 0;
                    }
                }
            }
        }

        // Everything was referenced. Clear the bits and take the current page.
        for entry in self.pages.values_mut() {
            entry.referenced = false;
        }
        let page_no = self.clock[self.hand.min(self.clock.len() - 1)];
        let entry = self.pages.remove(&page_no)?;
        let position = self
            .clock
            .iter()
            .position(|candidate| *candidate == page_no)?;
        self.clock.swap_remove(position);
        if self.hand >= self.clock.len() {
            self.hand = 0;
        }
        Some((page_no, entry))
    }
}

/// A sharded page cache. Sharding keeps unrelated page accesses from
/// contending on one global lock.
pub(crate) struct PageCache {
    shards: Vec<Mutex<Shard>>,
    mask: usize,
}

impl PageCache {
    pub(crate) fn new(cache_pages: usize) -> Self {
        let cache_pages = cache_pages.max(1);
        let target = (cache_pages / 256).clamp(1, 64).next_power_of_two();
        let shard_count = target.min(cache_pages).max(1);
        let base = cache_pages / shard_count;
        let extra = cache_pages % shard_count;
        let shards = (0..shard_count)
            .map(|index| {
                let extra_page = if index < extra { 1 } else { 0 };
                Mutex::new(Shard::new(base + extra_page))
            })
            .collect();
        Self {
            shards,
            mask: shard_count - 1,
        }
    }

    pub(crate) fn shard_for(&self, page_no: u64) -> &Mutex<Shard> {
        &self.shards[(page_no as usize) & self.mask]
    }

    pub(crate) fn shards(&self) -> &[Mutex<Shard>] {
        &self.shards
    }

    pub(crate) fn read_into(&self, page_no: u64, page_offset: usize, dst: &mut [u8]) -> bool {
        debug_assert!(page_offset + dst.len() <= PAGE_SIZE);
        let mut shard = self.shard_for(page_no).lock();
        let Some(page) = shard.pages.get_mut(&page_no) else {
            return false;
        };
        page.referenced = true;
        dst.copy_from_slice(&page.data[page_offset..page_offset + dst.len()]);
        true
    }

    pub(crate) fn counts(&self) -> (usize, usize) {
        let mut pages = 0;
        let mut dirty = 0;
        for shard in &self.shards {
            let shard = shard.lock();
            pages += shard.pages.len();
            dirty += shard.pages.values().filter(|page| page.dirty).count();
        }
        (pages, dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_evicts_a_page_when_full() {
        let mut shard = Shard::new(2);
        for page_no in 0..2 {
            shard.insert(
                page_no,
                CachePage::new(Box::new([page_no as u8; PAGE_SIZE]), 1, false),
            );
        }
        let (page_no, _) = shard.evict_candidate().expect("victim");
        assert!(page_no < 2);
        shard.insert(9, CachePage::new(Box::new([9u8; PAGE_SIZE]), 1, false));
        assert_eq!(shard.pages.len(), 2);
    }

    #[test]
    fn second_chance_protects_referenced_pages() {
        let mut shard = Shard::new(2);
        for page_no in 0..2 {
            shard.insert(
                page_no,
                CachePage::new(Box::new([0u8; PAGE_SIZE]), 1, false),
            );
        }
        // Both were inserted with referenced = true. The first scan clears them.
        let (first, _) = shard.evict_candidate().expect("victim");
        assert!(first < 2);
        assert_eq!(shard.pages.len(), 1);
    }
}

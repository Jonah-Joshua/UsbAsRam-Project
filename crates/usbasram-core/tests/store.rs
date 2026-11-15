use usbasram_core::{IoMode, Store, StoreConfig, PAGE_SIZE};

fn config(path: &std::path::Path, pages: u64, cache_pages: u64) -> StoreConfig {
    StoreConfig::new(path, pages * PAGE_SIZE as u64)
        .with_cache_bytes(cache_pages * PAGE_SIZE as u64)
}

fn pattern(seed: u64, len: usize) -> Vec<u8> {
    let mut state = seed | 1;
    let mut out = Vec::with_capacity(len);
    for _ in 0..len {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        out.push(state as u8);
    }
    out
}

#[test]
fn write_read_and_reopen() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let data = pattern(7, 3 * PAGE_SIZE + 123);

    {
        let store = Store::open(config(&path, 64, 16)).unwrap();
        assert_eq!(store.write_at(17, &data).unwrap(), data.len());
        let mut out = vec![0u8; data.len()];
        assert_eq!(store.read_at(17, &mut out).unwrap(), data.len());
        assert_eq!(out, data);
        store.flush().unwrap();
    }

    let store = Store::open(config(&path, 64, 16)).unwrap();
    let mut out = vec![0u8; data.len()];
    store.read_at(17, &mut out).unwrap();
    assert_eq!(out, data);
    let report = store.verify().unwrap();
    assert!(report.pages >= 4);
}

#[test]
fn zero_filled_sparse_pages_read_as_zero() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let store = Store::open(config(&path, 32, 4)).unwrap();
    let mut out = vec![0xffu8; PAGE_SIZE * 2];
    store.read_at(0, &mut out).unwrap();
    assert!(out.iter().all(|byte| *byte == 0));
}

#[test]
fn compression_reduces_stored_bytes() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let store = Store::open(config(&path, 64, 64)).unwrap();
    let zeros = vec![0u8; PAGE_SIZE * 16];
    store.write_at(0, &zeros).unwrap();
    store.flush().unwrap();
    let stats = store.stats();
    assert!(stats.compressed_pages >= 16);
    assert!(stats.stored_bytes < stats.raw_bytes / 2);
    store.verify().unwrap();
}

#[test]
fn tiny_cache_evicts_and_preserves_data() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let pages = 64u64;
    let data = pattern(99, pages as usize * PAGE_SIZE);
    {
        let store = Store::open(config(&path, pages, 3)).unwrap();
        store.write_at(0, &data).unwrap();
        store.flush().unwrap();
        let stats = store.stats();
        assert!(stats.evictions_dirty > 0);
    }

    let store = Store::open(config(&path, pages, 2)).unwrap();
    let mut out = vec![0u8; data.len()];
    store.read_at(0, &mut out).unwrap();
    assert_eq!(out, data);
}

#[test]
fn trim_returns_zeroes_even_after_reopen() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let data = pattern(5, PAGE_SIZE);
    {
        let store = Store::open(config(&path, 16, 8)).unwrap();
        store.write_at(PAGE_SIZE as u64, &data).unwrap();
        store.trim(PAGE_SIZE as u64, PAGE_SIZE as u64).unwrap();
        store.flush().unwrap();
    }
    let store = Store::open(config(&path, 16, 8)).unwrap();
    let mut out = vec![1u8; PAGE_SIZE];
    store.read_at(PAGE_SIZE as u64, &mut out).unwrap();
    assert!(out.iter().all(|byte| *byte == 0));
}

#[test]
fn repeated_overwrites_trigger_compaction() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let pages = 16u64;
    let rounds = 40u8;
    let store = Store::open(config(&path, pages, 4)).unwrap();
    let data = pattern(3, PAGE_SIZE);
    for round in 0..rounds {
        for page in 0..pages {
            let mut block = data.clone();
            block[0] = round;
            block[1] = page as u8;
            store.write_at(page * PAGE_SIZE as u64, &block).unwrap();
        }
        store.flush().unwrap();
    }
    let stats = store.stats();
    assert!(stats.compactions > 0, "{stats:?}");
    store.verify().unwrap();

    let mut out = vec![0u8; PAGE_SIZE];
    store.read_at(0, &mut out).unwrap();
    assert_eq!(out[0], rounds - 1);
    drop(store);

    let reopened = Store::open(config(&path, pages, 4)).unwrap();
    reopened.verify().unwrap();
    reopened.read_at(0, &mut out).unwrap();
    assert_eq!(out[0], rounds - 1);
}

#[test]
fn minimum_log_survives_repeated_full_capacity_flushes() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let pages = 64u64;
    let capacity = pages * PAGE_SIZE as u64;
    let log_bytes = usbasram_core::min_log_bytes(capacity);
    let config = StoreConfig::new(&path, capacity)
        .with_cache_bytes(capacity)
        .with_log_bytes(log_bytes);
    let store = Store::open(config.clone()).unwrap();
    let data = pattern(11, PAGE_SIZE);

    for round in 0..8u8 {
        for page in 0..pages {
            let mut block = data.clone();
            block[0] = round;
            store.write_at(page * PAGE_SIZE as u64, &block).unwrap();
        }
        store.flush().unwrap();
    }

    assert!(store.stats().compactions > 0);
    store.verify().unwrap();
    let mut out = vec![0u8; PAGE_SIZE];
    store.read_at(0, &mut out).unwrap();
    assert_eq!(out[0], 7);
    drop(store);

    let reopened = Store::open(config).unwrap();
    reopened.verify().unwrap();
    reopened.read_at(0, &mut out).unwrap();
    assert_eq!(out[0], 7);
}

#[test]
fn concurrent_disjoint_writes_survive_reopen() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let pages = 128u64;
    let threads = 4u64;
    let per_thread = pages / threads;

    {
        let store = Store::open(config(&path, pages, 16)).unwrap();
        std::thread::scope(|scope| {
            for thread_id in 0..threads {
                let store = &store;
                scope.spawn(move || {
                    let start = thread_id * per_thread;
                    for page in start..start + per_thread {
                        let mut block = pattern(page, PAGE_SIZE);
                        block[0] = thread_id as u8;
                        store.write_at(page * PAGE_SIZE as u64, &block).unwrap();
                    }
                });
            }
        });
        store.flush().unwrap();
    }

    let store = Store::open(config(&path, pages, 8)).unwrap();
    let mut out = vec![0u8; PAGE_SIZE];
    for page in 0..pages {
        store.read_at(page * PAGE_SIZE as u64, &mut out).unwrap();
        let mut expected = pattern(page, PAGE_SIZE);
        expected[0] = (page / per_thread) as u8;
        assert_eq!(out, expected, "page {page} was corrupted");
    }
    store.verify().unwrap();
}

#[test]
fn rejects_wrong_capacity() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    {
        let store = Store::open(config(&path, 16, 8)).unwrap();
        store.write_at(0, b"hello").unwrap();
        store.flush().unwrap();
    }
    let error = Store::open(config(&path, 32, 8)).unwrap_err();
    assert!(matches!(error, usbasram_core::Error::Config(_)));
}

#[test]
fn coalescing_window_is_reported() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let store = Store::open(config(&path, 64, 16).with_coalescing(
        true,
        128 * 1024,
        4 * 1024 * 1024,
        128 * 1024,
    ))
    .unwrap();
    let info = store.info();
    assert_eq!(info.coalesce_target_bytes, 128 * 1024);
    assert_eq!(info.coalesce_min_bytes, 128 * 1024);
    assert_eq!(info.coalesce_max_bytes, 4 * 1024 * 1024);
}

#[test]
fn existing_store_reopens_with_tuning_options() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let data = pattern(31, PAGE_SIZE);
    {
        let store = Store::open(config(&path, 64, 16)).unwrap();
        store.write_at(0, &data).unwrap();
        store.flush().unwrap();
    }
    let store = Store::open_existing_with(
        StoreConfig::new(&path, 0)
            .with_cache_bytes(16 * PAGE_SIZE as u64)
            .with_hugepages(true)
            .with_coalescing(false, PAGE_SIZE as u64, PAGE_SIZE as u64, PAGE_SIZE as u64),
    )
    .unwrap();
    let mut out = vec![0u8; PAGE_SIZE];
    store.read_at(0, &mut out).unwrap();
    assert_eq!(out, data);
    let info = store.info();
    assert_eq!(info.coalesce_target_bytes, PAGE_SIZE as u64);
    assert!(info.hugepages_requested);
}

#[cfg(feature = "direct-io")]
#[test]
fn raw_direct_io_roundtrip() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("raw.bin");
    let report = usbasram_core::raw::run(&path, IoMode::Direct, 1024 * 1024, false).unwrap();
    assert_eq!(report.bytes, 1024 * 1024);
    assert!(report.write_seconds > 0.0);
    assert!(report.read_seconds > 0.0);
}

#[test]
fn coalesced_flush_covers_multiple_batches() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let pages = 300u64;
    let data: Vec<Vec<u8>> = (0..pages)
        .map(|seed| pattern(seed + 99, PAGE_SIZE))
        .collect();
    {
        let config = config(&path, 1024, 512)
            .with_coalescing(true, 128 * 1024, 4 * 1024 * 1024, 128 * 1024);
        let store = Store::open(config).unwrap();
        for (page, page_data) in data.iter().enumerate() {
            store
                .write_at(page as u64 * PAGE_SIZE as u64, page_data)
                .unwrap();
        }
        store.flush().unwrap();
        assert_eq!(store.stats().dirty_pages, 0);
    }
    let store = Store::open_existing(&path, 512 * PAGE_SIZE as u64).unwrap();
    let mut out = vec![0u8; PAGE_SIZE];
    for (page, page_data) in data.iter().enumerate() {
        store
            .read_at(page as u64 * PAGE_SIZE as u64, &mut out)
            .unwrap();
        assert_eq!(&out, page_data, "page {page} was corrupted");
    }
    store.verify().unwrap();
}

#[cfg(feature = "direct-io")]
#[test]
fn direct_mode_roundtrip_if_available() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let data = pattern(77, 3 * PAGE_SIZE);
    let store = Store::open(
        config(&path, 64, 16)
            .with_io_mode(IoMode::Direct)
            .with_coalescing(true, 128 * 1024, 4 * 1024 * 1024, 128 * 1024),
    )
    .unwrap();
    if !store.info().io_direct {
        // The filesystem rejected the direct-open flags. The contract is a
        // clean fallback to portable I/O, so there is nothing else to test.
        return;
    }
    store.write_at(17, &data).unwrap();
    store.flush().unwrap();
    let mut out = vec![0u8; data.len()];
    assert_eq!(store.read_at(17, &mut out).unwrap(), data.len());
    assert_eq!(out, data);
    drop(store);

    let store = Store::open_existing_with(
        StoreConfig::new(&path, 0)
            .with_cache_bytes(16 * PAGE_SIZE as u64)
            .with_io_mode(IoMode::Direct),
    )
    .unwrap();
    let mut out = vec![0u8; data.len()];
    assert_eq!(store.read_at(17, &mut out).unwrap(), data.len());
    assert_eq!(out, data);
    store.verify().unwrap();
}

#[cfg(feature = "direct-io")]
#[test]
fn direct_mode_pads_an_unaligned_store_tail() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("store.usr");
    let data = pattern(123, PAGE_SIZE);
    {
        let store = Store::open(config(&path, 64, 16).with_compression(false)).unwrap();
        store.write_at(0, &data).unwrap();
        store.flush().unwrap();
    }
    let before = std::fs::metadata(&path).unwrap().len();
    assert_ne!(before % PAGE_SIZE as u64, 0);
    let store = Store::open(
        StoreConfig::new(&path, 64 * PAGE_SIZE as u64)
            .with_cache_bytes(16 * PAGE_SIZE as u64)
            .with_io_mode(IoMode::Direct),
    )
    .unwrap();
    if !store.info().io_direct {
        return;
    }
    let after = std::fs::metadata(&path).unwrap().len();
    assert_eq!(after % PAGE_SIZE as u64, 0);
    assert!(after > before);
    let mut out = vec![0u8; PAGE_SIZE];
    store.read_at(0, &mut out).unwrap();
    assert_eq!(out, data);
}

use std::hint::black_box;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use usbasram_core::{raw, IoMode, Store, StoreConfig, PAGE_SIZE};

use crate::size::human_bytes;

pub struct BenchOptions {
    pub size_bytes: u64,
    pub cold_cache_bytes: u64,
    pub threads: usize,
    pub queue_depths: Vec<usize>,
    pub seconds: f64,
    pub json: bool,
    pub io_mode: IoMode,
    pub hugepages: bool,
    pub coalesce: bool,
    pub coalesce_min_bytes: u64,
    pub coalesce_max_bytes: u64,
    pub erase_block_bytes: u64,
}

struct Phase {
    name: &'static str,
    queue_depth: usize,
    ops: u64,
    bytes: u64,
    elapsed: Duration,
}

impl Phase {
    fn gib_per_second(&self) -> f64 {
        self.bytes as f64 / self.elapsed.as_secs_f64() / (1024.0 * 1024.0 * 1024.0)
    }

    fn ops_per_second(&self) -> f64 {
        self.ops as f64 / self.elapsed.as_secs_f64()
    }

    fn seconds(&self) -> f64 {
        self.elapsed.as_secs_f64()
    }
}

struct Latency {
    p50: u64,
    p99: u64,
    p999: u64,
}

impl Latency {
    fn empty() -> Self {
        Self {
            p50: 0,
            p99: 0,
            p999: 0,
        }
    }
}

pub fn run(options: BenchOptions) -> Result<()> {
    let queue_depths = if options.queue_depths.is_empty() {
        vec![options.threads.max(1)]
    } else {
        options.queue_depths.clone()
    };
    if queue_depths
        .iter()
        .any(|depth| *depth == 0 || *depth > 1024)
    {
        bail!("--queue-depths values must be between 1 and 1024");
    }
    if options.seconds <= 0.0 {
        bail!("--seconds must be greater than zero");
    }
    if options.size_bytes == 0 || options.size_bytes % PAGE_SIZE as u64 != 0 {
        bail!("benchmark size must be a non-zero multiple of 4096");
    }
    let pages = options.size_bytes / PAGE_SIZE as u64;
    let cold_cache = options
        .cold_cache_bytes
        .clamp(PAGE_SIZE as u64, options.size_bytes);
    let max_depth = queue_depths.iter().copied().max().unwrap_or(1);

    let directory = tempfile::tempdir()?;
    let path = directory.path().join("bench.usr");
    let mut phases = Vec::new();
    let hot_latency;
    let cold_latency;
    let (effective_mode, effective_direct, effective_sqpoll, effective_hugepages);

    {
        let store = Store::open(tuned_config(
            &path,
            options.size_bytes,
            options.size_bytes,
            &options,
        ))?;
        let info = store.info();
        effective_mode = info.io_mode;
        effective_direct = info.io_direct;
        effective_sqpoll = info.io_sqpoll;
        effective_hugepages = info.hugepages_active;
        let fill = random_page(0x5eed_1234);
        for page in 0..pages {
            store.write_at(page * PAGE_SIZE as u64, &fill)?;
        }
        store.flush()?;

        for depth in &queue_depths {
            let started = Instant::now();
            let (ops, bytes) = read_phase(&store, pages, *depth, options.seconds);
            phases.push(Phase {
                name: "hot_read",
                queue_depth: *depth,
                ops,
                bytes,
                elapsed: started.elapsed(),
            });
        }
        hot_latency = read_latency(&store, pages, 50_000);

        for depth in &queue_depths {
            let started = Instant::now();
            let (ops, bytes) = write_phase(&store, pages, *depth, options.seconds);
            phases.push(Phase {
                name: "hot_write",
                queue_depth: *depth,
                ops,
                bytes,
                elapsed: started.elapsed(),
            });
        }
        store.flush()?;
    }

    {
        let store = Store::open(tuned_config(
            &path,
            options.size_bytes,
            cold_cache,
            &options,
        ))?;
        let started = Instant::now();
        let (ops, bytes) = read_phase(&store, pages, max_depth, options.seconds);
        phases.push(Phase {
            name: "cold_read",
            queue_depth: max_depth,
            ops,
            bytes,
            elapsed: started.elapsed(),
        });
        cold_latency = read_latency(&store, pages, 20_000);
    }

    let raw_bytes = options.size_bytes.clamp(4 * 1024 * 1024, 64 * 1024 * 1024);
    let raw_path = directory.path().join("raw.bin");
    let raw_report = raw::run(&raw_path, options.io_mode, raw_bytes, options.hugepages)?;
    phases.push(Phase {
        name: "raw_write",
        queue_depth: 1,
        ops: raw_report.bytes / PAGE_SIZE as u64,
        bytes: raw_report.bytes,
        elapsed: Duration::from_secs_f64(raw_report.write_seconds),
    });
    phases.push(Phase {
        name: "raw_read",
        queue_depth: 1,
        ops: raw_report.bytes / PAGE_SIZE as u64,
        bytes: raw_report.bytes,
        elapsed: Duration::from_secs_f64(raw_report.read_seconds),
    });

    if options.json {
        println!(
            "{{\"size_bytes\":{},\"hot_cache_bytes\":{},\"cold_cache_bytes\":{},\"io_mode\":\"{}\",\"io_direct\":{},\"io_sqpoll\":{},\"hugepages\":{},\"queue_depths\":[{}],\"hot_read_latency\":{},\"cold_read_latency\":{},\"phases\":[{}]}}",
            options.size_bytes,
            options.size_bytes,
            cold_cache,
            effective_mode.as_str(),
            effective_direct,
            effective_sqpoll,
            effective_hugepages,
            queue_depths
                .iter()
                .map(|depth| depth.to_string())
                .collect::<Vec<_>>()
                .join(","),
            latency_json(&hot_latency),
            latency_json(&cold_latency),
            phases
                .iter()
                .map(|phase| format!(
                    "{{\"name\":\"{}\",\"queue_depth\":{},\"ops\":{},\"bytes\":{},\"seconds\":{:.6},\"iops\":{:.1},\"gib_per_s\":{:.3}}}",
                    phase.name,
                    phase.queue_depth,
                    phase.ops,
                    phase.bytes,
                    phase.seconds(),
                    phase.ops_per_second(),
                    phase.gib_per_second()
                ))
                .collect::<Vec<_>>()
                .join(",")
        );
        return Ok(());
    }

    println!("usbasram benchmark");
    println!(
        "  logical size: {}, hot cache: {}, cold cache: {}",
        human_bytes(options.size_bytes),
        human_bytes(options.size_bytes),
        human_bytes(cold_cache)
    );
    println!(
        "  I/O engine: {}{}{}, hugepages: {}",
        effective_mode.as_str(),
        if effective_direct { " (direct)" } else { "" },
        if effective_sqpoll { " (SQPOLL)" } else { "" },
        if effective_hugepages {
            "active"
        } else if options.hugepages {
            "requested (unavailable)"
        } else {
            "off"
        }
    );
    println!();
    println!(
        "  {:<12} {:>4} {:>14} {:>14} {:>16}",
        "phase", "qd", "ops", "IOPS", "throughput"
    );
    for phase in &phases {
        println!(
            "  {:<12} {:>4} {:>14} {:>14.0} {:>13.2} GiB/s",
            phase.name,
            phase.queue_depth,
            phase.ops,
            phase.ops_per_second(),
            phase.gib_per_second()
        );
    }
    println!();
    println!(
        "  hot read latency: p50 {} ns, p99 {} ns, p99.9 {} ns",
        hot_latency.p50, hot_latency.p99, hot_latency.p999
    );
    println!(
        "  cold read latency: p50 {} ns, p99 {} ns, p99.9 {} ns",
        cold_latency.p50, cold_latency.p99, cold_latency.p999
    );
    println!(
        "  Note: hot phases are RAM-cache hits; raw phases bypass the store and measure the selected backend directly."
    );
    Ok(())
}

fn tuned_config(
    path: &std::path::Path,
    size_bytes: u64,
    cache_bytes: u64,
    options: &BenchOptions,
) -> StoreConfig {
    StoreConfig::new(path, size_bytes)
        .with_cache_bytes(cache_bytes)
        .with_io_mode(options.io_mode)
        .with_hugepages(options.hugepages)
        .with_coalescing(
            options.coalesce,
            options.coalesce_min_bytes,
            options.coalesce_max_bytes,
            options.erase_block_bytes,
        )
}

fn latency_json(latency: &Latency) -> String {
    format!(
        "{{\"p50_ns\":{},\"p99_ns\":{},\"p99_9_ns\":{}}}",
        latency.p50, latency.p99, latency.p999
    )
}

fn read_latency(store: &Store, pages: u64, samples: usize) -> Latency {
    let mut rng = Rng::new(0x1a7e_11ce);
    let mut buffer = [0u8; PAGE_SIZE];
    // Warm the cache path before recording.
    for _ in 0..2048 {
        let page = rng.next() % pages;
        let _ = store.read_at(page * PAGE_SIZE as u64, &mut buffer);
    }
    let mut timings = Vec::with_capacity(samples);
    for _ in 0..samples {
        let page = rng.next() % pages;
        let started = Instant::now();
        store
            .read_at(page * PAGE_SIZE as u64, &mut buffer)
            .expect("latency read failed");
        timings.push(started.elapsed().as_nanos() as u64);
    }
    black_box(&buffer);
    timings.sort_unstable();
    if timings.is_empty() {
        return Latency::empty();
    }
    let per_mille = |permille: usize| timings[(timings.len() - 1) * permille / 1000];
    Latency {
        p50: timings[timings.len() / 2],
        p99: per_mille(990),
        p999: per_mille(999),
    }
}

fn read_phase(store: &Store, pages: u64, threads: usize, seconds: f64) -> (u64, u64) {
    let deadline = Instant::now() + Duration::from_secs_f64(seconds);
    thread::scope(|scope| {
        let mut handles = Vec::with_capacity(threads);
        for thread_id in 0..threads {
            handles.push(scope.spawn(move || {
                let mut rng = Rng::new(thread_id as u64 + 1);
                let mut buffer = [0u8; PAGE_SIZE];
                let mut ops = 0u64;
                'outer: loop {
                    for _ in 0..256 {
                        if Instant::now() >= deadline {
                            break 'outer;
                        }
                        let page = rng.next() % pages;
                        store
                            .read_at(page * PAGE_SIZE as u64, &mut buffer)
                            .expect("benchmark read failed");
                        ops += 1;
                    }
                }
                black_box(&buffer);
                (ops, ops * PAGE_SIZE as u64)
            }));
        }
        let mut total_ops = 0;
        let mut total_bytes = 0;
        for handle in handles {
            let (ops, bytes) = handle.join().expect("benchmark thread panicked");
            total_ops += ops;
            total_bytes += bytes;
        }
        (total_ops, total_bytes)
    })
}

fn write_phase(store: &Store, pages: u64, threads: usize, seconds: f64) -> (u64, u64) {
    let deadline = Instant::now() + Duration::from_secs_f64(seconds);
    thread::scope(|scope| {
        let mut handles = Vec::with_capacity(threads);
        for thread_id in 0..threads {
            handles.push(scope.spawn(move || {
                let mut rng = Rng::new((thread_id as u64 + 1) * 0x9e37_79b9);
                let mut block = random_page(thread_id as u64 + 7);
                let mut ops = 0u64;
                'outer: loop {
                    for _ in 0..256 {
                        if Instant::now() >= deadline {
                            break 'outer;
                        }
                        let page = rng.next() % pages;
                        block[0] = ops as u8;
                        store
                            .write_at(page * PAGE_SIZE as u64, &block)
                            .expect("benchmark write failed");
                        ops += 1;
                    }
                }
                black_box(&block);
                (ops, ops * PAGE_SIZE as u64)
            }));
        }
        let mut total_ops = 0;
        let mut total_bytes = 0;
        for handle in handles {
            let (ops, bytes) = handle.join().expect("benchmark thread panicked");
            total_ops += ops;
            total_bytes += bytes;
        }
        (total_ops, total_bytes)
    })
}

fn random_page(seed: u64) -> [u8; PAGE_SIZE] {
    let mut rng = Rng::new(seed);
    let mut page = [0u8; PAGE_SIZE];
    for chunk in page.chunks_mut(8) {
        let value = rng.next().to_le_bytes();
        chunk.copy_from_slice(&value[..chunk.len()]);
    }
    page
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next(&mut self) -> u64 {
        let mut value = self.0;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.0 = value;
        value
    }
}

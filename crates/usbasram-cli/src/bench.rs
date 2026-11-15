use std::hint::black_box;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use usbasram_core::{Store, StoreConfig, PAGE_SIZE};

use crate::size::human_bytes;

pub struct BenchOptions {
    pub size_bytes: u64,
    pub cold_cache_bytes: u64,
    pub threads: usize,
    pub seconds: f64,
    pub json: bool,
}

struct Phase {
    name: &'static str,
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

pub fn run(options: BenchOptions) -> Result<()> {
    if options.threads == 0 {
        bail!("--threads must be greater than zero");
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

    let directory = tempfile::tempdir()?;
    let path = directory.path().join("bench.usr");
    let mut phases = Vec::new();

    {
        let store = Store::open(
            StoreConfig::new(&path, options.size_bytes).with_cache_bytes(options.size_bytes),
        )?;
        let fill = random_page(0x5eed_1234);
        for page in 0..pages {
            store.write_at(page * PAGE_SIZE as u64, &fill)?;
        }
        store.flush()?;

        let started = Instant::now();
        let (ops, bytes) = read_phase(&store, pages, options.threads, options.seconds);
        phases.push(Phase {
            name: "hot_read",
            ops,
            bytes,
            elapsed: started.elapsed(),
        });

        let started = Instant::now();
        let (ops, bytes) = write_phase(&store, pages, options.threads, options.seconds);
        phases.push(Phase {
            name: "hot_write",
            ops,
            bytes,
            elapsed: started.elapsed(),
        });
        store.flush()?;
    }

    {
        let store = Store::open(
            StoreConfig::new(&path, options.size_bytes).with_cache_bytes(cold_cache),
        )?;
        let started = Instant::now();
        let (ops, bytes) = read_phase(&store, pages, options.threads, options.seconds);
        phases.push(Phase {
            name: "cold_read",
            ops,
            bytes,
            elapsed: started.elapsed(),
        });
    }

    if options.json {
        println!(
            "{{\"size_bytes\":{},\"hot_cache_bytes\":{},\"cold_cache_bytes\":{},\"threads\":{},\"phases\":[{}]}}",
            options.size_bytes,
            options.size_bytes,
            cold_cache,
            options.threads,
            phases
                .iter()
                .map(|phase| format!(
                    "{{\"name\":\"{}\",\"ops\":{},\"bytes\":{},\"seconds\":{:.6},\"iops\":{:.1},\"gib_per_s\":{:.3}}}",
                    phase.name,
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
        "  logical size: {}, hot cache: {}, cold cache: {}, threads: {}",
        human_bytes(options.size_bytes),
        human_bytes(options.size_bytes),
        human_bytes(cold_cache),
        options.threads
    );
    println!();
    println!("  {:<12} {:>14} {:>14} {:>14}", "phase", "ops", "IOPS", "throughput");
    for phase in &phases {
        println!(
            "  {:<12} {:>14} {:>14.0} {:>14.2} GiB/s",
            phase.name,
            phase.ops,
            phase.ops_per_second(),
            phase.gib_per_second()
        );
    }
    println!();
    println!(
        "  Note: hot phases are RAM-cache hits. cold_read is bounded by the backing file/device."
    );
    Ok(())
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

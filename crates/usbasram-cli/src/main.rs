mod bench;
mod size;

use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use usbasram_core::{Store, StoreConfig, StoreInfo};

use crate::bench::BenchOptions;
use crate::size::{human_bytes, parse_size};

#[derive(Parser, Debug)]
#[command(
    name = "usbasram",
    version,
    about = "A tiered RAM + USB storage engine with a crash-safe compressed cold tier"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Create a new store.
    Init(InitArgs),
    /// Show store metadata and counters.
    Info(OpenArgs),
    /// Serve the store over NBD.
    Serve(ServeArgs),
    /// Verify every cold record and checksum.
    Verify(OpenArgs),
    /// Flush the cache and compact the cold log.
    Compact(OpenArgs),
    /// Run the RAM-cache and cold-tier benchmark.
    Bench(BenchArgs),
}

#[derive(Args, Debug)]
struct InitArgs {
    /// Path of the store file.
    path: PathBuf,
    /// Logical block-device size (for example 64M or 1GiB).
    #[arg(long, default_value = "64M")]
    size: String,
    /// RAM hot-cache size (defaults to min(size, 64 MiB)).
    #[arg(long)]
    cache: Option<String>,
    /// Cold log size (defaults to the format minimum for the given size).
    #[arg(long)]
    log_size: Option<String>,
    /// Store pages without LZ4 compression.
    #[arg(long)]
    no_compress: bool,
    /// Delete an existing store and its sidecar files first.
    #[arg(long)]
    force: bool,
}

#[derive(Args, Debug)]
struct OpenArgs {
    /// Path of the store file.
    path: PathBuf,
    /// RAM cache to use for metadata operations.
    #[arg(long, default_value = "64M")]
    cache: String,
}

#[derive(Args, Debug)]
struct ServeArgs {
    /// Path of the store file.
    path: PathBuf,
    /// Create the store with this logical size if it does not exist.
    #[arg(long)]
    size: Option<String>,
    /// RAM hot-cache size.
    #[arg(long, default_value = "64M")]
    cache: String,
    /// NBD listen address. Keep the default loopback bind unless you need remote access.
    #[arg(long, default_value = "127.0.0.1:10809")]
    bind: String,
    /// NBD export name.
    #[arg(long, default_value = "usbasram")]
    export: String,
    /// Flush dirty pages every N seconds (0 disables the background flusher).
    #[arg(long, default_value_t = 5)]
    flush_interval: u64,
}

#[derive(Args, Debug)]
struct BenchArgs {
    /// Logical size of the benchmark store.
    #[arg(long, default_value = "64M")]
    size: String,
    /// Cache size used by the cold-read phase.
    #[arg(long, default_value = "4M")]
    cold_cache: String,
    /// Number of benchmark threads.
    #[arg(long, default_value_t = 4)]
    threads: usize,
    /// Duration of each phase in seconds.
    #[arg(long, default_value_t = 2.0)]
    seconds: f64,
    /// Use short 0.5 second phases (useful in CI).
    #[arg(long)]
    quick: bool,
    /// Print machine-readable JSON.
    #[arg(long)]
    json: bool,
}

fn main() {
    let cli = Cli::parse();
    if let Err(error) = execute(cli.command) {
        eprintln!("usbasram: {error:#}");
        std::process::exit(1);
    }
}

fn execute(command: Command) -> Result<()> {
    match command {
        Command::Init(args) => init(args),
        Command::Info(args) => info(args),
        Command::Serve(args) => serve(args),
        Command::Verify(args) => verify(args),
        Command::Compact(args) => compact(args),
        Command::Bench(args) => run_bench(args),
    }
}

fn init(args: InitArgs) -> Result<()> {
    let size = parse_size(&args.size)?;
    let cache = match args.cache {
        Some(value) => parse_size(&value)?,
        None => size.min(64 * 1024 * 1024),
    };
    if args.force {
        remove_store_files(&args.path)?;
    }
    let mut config = StoreConfig::new(&args.path, size)
        .with_cache_bytes(cache)
        .with_compression(!args.no_compress);
    if let Some(log_size) = args.log_size {
        config = config.with_log_bytes(parse_size(&log_size)?);
    }
    let store = Store::open(config)
        .with_context(|| format!("creating store at {}", args.path.display()))?;
    store.flush()?;
    println!("created {}", args.path.display());
    print_info(&store.info());
    print_stats(&store.stats());
    Ok(())
}

fn info(args: OpenArgs) -> Result<()> {
    let store = open_existing(&args)?;
    print_info(&store.info());
    print_stats(&store.stats());
    Ok(())
}

fn verify(args: OpenArgs) -> Result<()> {
    let store = open_existing(&args)?;
    let report = store.verify()?;
    println!("verified {}", store.info().path.display());
    println!("  pages: {}", report.pages);
    println!("  raw data: {}", human_bytes(report.raw_bytes));
    println!("  stored data: {}", human_bytes(report.stored_bytes));
    println!("  compressed pages: {}", report.compressed_pages);
    println!("  elapsed: {} ms", report.elapsed_ms);
    Ok(())
}

fn compact(args: OpenArgs) -> Result<()> {
    let store = open_existing(&args)?;
    store.compact()?;
    println!("compacted {}", store.info().path.display());
    print_info(&store.info());
    print_stats(&store.stats());
    Ok(())
}

fn serve(args: ServeArgs) -> Result<()> {
    let cache = parse_size(&args.cache)?;
    let store = if let Some(size) = &args.size {
        let size = parse_size(size)?;
        Store::open(StoreConfig::new(&args.path, size).with_cache_bytes(cache))
            .with_context(|| format!("opening or creating {}", args.path.display()))?
    } else {
        Store::open_existing(&args.path, cache)
            .with_context(|| format!("opening {}", args.path.display()))?
    };
    let store = Arc::new(store);

    if args.flush_interval > 0 {
        let flush_store = Arc::clone(&store);
        let interval = Duration::from_secs(args.flush_interval);
        thread::spawn(move || loop {
            thread::sleep(interval);
            if let Err(error) = flush_store.flush() {
                eprintln!("usbasram: background flush failed: {error}");
            }
        });
    }

    let listener = TcpListener::bind(&args.bind)
        .with_context(|| format!("binding NBD listener to {}", args.bind))?;
    let address = listener.local_addr()?;
    println!(
        "serving {} as NBD export {:?} on {}",
        args.path.display(),
        args.export,
        address
    );
    println!(
        "  Linux client: sudo nbd-client {} {} -N {}",
        address.ip(),
        address.port(),
        args.export
    );
    usbasram_nbd::serve_tcp(listener, store, args.export)?;
    Ok(())
}

fn run_bench(args: BenchArgs) -> Result<()> {
    let seconds = if args.quick { 0.5 } else { args.seconds };
    bench::run(BenchOptions {
        size_bytes: parse_size(&args.size)?,
        cold_cache_bytes: parse_size(&args.cold_cache)?,
        threads: args.threads,
        seconds,
        json: args.json,
    })
}

fn open_existing(args: &OpenArgs) -> Result<Store> {
    let cache = parse_size(&args.cache)?;
    Store::open_existing(&args.path, cache)
        .with_context(|| format!("opening {}", args.path.display()))
}

fn remove_store_files(path: &Path) -> Result<()> {
    for suffix in ["", ".compact", ".old"] {
        let mut name = path.as_os_str().to_os_string();
        name.push(suffix);
        let candidate = PathBuf::from(name);
        if candidate.exists() {
            std::fs::remove_file(&candidate)
                .with_context(|| format!("removing {}", candidate.display()))?;
        }
    }
    Ok(())
}

fn print_info(info: &StoreInfo) {
    println!("store: {}", info.path.display());
    println!("  capacity: {}", human_bytes(info.capacity_bytes));
    println!("  hot cache: {}", human_bytes(info.cache_bytes));
    println!("  cold log: {}", human_bytes(info.log_bytes));
    println!("  head: {} ({})", info.head, human_bytes(info.head));
    println!("  epoch: {}", info.epoch);
    println!("  checkpoint generation: {}", info.checkpoint_generation);
    println!("  active superblock: {}", info.superblock_slot);
    println!("  on-disk pages: {}", info.pages_on_disk);
    println!(
        "  compression: {}",
        if info.compressed { "on" } else { "off" }
    );
}

fn print_stats(stats: &usbasram_core::Stats) {
    println!(
        "  reads: {} ({}), writes: {} ({})",
        stats.reads,
        human_bytes(stats.bytes_read),
        stats.writes,
        human_bytes(stats.bytes_written)
    );
    println!(
        "  cache: {} pages ({} dirty), hits {}, misses {}",
        stats.cache_pages, stats.dirty_pages, stats.cache_hits, stats.cache_misses
    );
    println!(
        "  flushes: {}, compactions: {}, page stores: {}",
        stats.flushes, stats.compactions, stats.page_stores
    );
    if stats.raw_bytes > 0 {
        let ratio = stats.stored_bytes as f64 / stats.raw_bytes as f64 * 100.0;
        println!(
            "  cold data: {} stored from {} raw ({ratio:.1}%)",
            human_bytes(stats.stored_bytes),
            human_bytes(stats.raw_bytes)
        );
    }
}

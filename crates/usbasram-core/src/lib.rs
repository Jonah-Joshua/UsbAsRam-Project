//! # usbasram-core
//!
//! A crash-safe, tiered storage engine for the "USB as RAM" idea.
//!
//! USB flash cannot be faster than DDR3. This crate takes the only honest
//! approach to the original goal: it keeps the hot working set in a fast,
//! sharded RAM cache and pushes cold pages to a compressed, checksummed log on
//! the USB device. Hot reads and writes never touch the USB bus; cold data is
//! still durable and recoverable.
//!
//! ## Guarantees
//!
//! * Every cold record is protected by CRC32C.
//! * A flush acknowledges durability only after the log data and its commit
//!   superblock have been fsynced.
//! * Compaction is atomic: the new snapshot is written to a sidecar file and
//!   renamed into place only after it is complete.
//! * Two superblock slots make a torn checkpoint recoverable.
//! * A recovery scan replays valid records beyond the last checkpoint, so a
//!   crash during the checkpoint write does not lose the preceding flush.
//!
//! ## Example
//!
//! ```no_run
//! use usbasram_core::{Store, StoreConfig};
//!
//! # fn main() -> usbasram_core::Result<()> {
//! let store = Store::open(StoreConfig::new("usb-store.usr", 64 * 1024 * 1024))?;
//! let mut page = [0u8; 4096];
//! store.write_at(0, b"hello, tiered world")?;
//! store.read_at(0, &mut page)?;
//! store.flush()?;
//! # Ok(())
//! # }
//! ```
#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

mod cache;
mod format;
mod io;
mod metrics;
mod store;

pub mod error;

pub use error::{Error, Result};
pub use metrics::Stats;
pub use store::{min_log_bytes, Store, StoreConfig, StoreInfo, VerifyReport};

/// The page size used by the engine: 4096 bytes.
pub const PAGE_SIZE: usize = 4096;

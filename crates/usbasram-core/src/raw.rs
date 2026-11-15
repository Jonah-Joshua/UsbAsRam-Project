//! Raw positioned-I/O benchmark helper.
//!
//! This bypasses the page cache and the log format so the benchmark can report
//! the throughput of the selected backend on its own. It is intentionally
//! small and synchronous: one aligned 1 MiB window is written sequentially and
//! then read back.

use std::path::Path;
use std::time::Instant;

use crate::error::Result;
use crate::io::{Io, IoMode, OpenSpec};

/// Throughput of a raw sequential write followed by a raw sequential read.
#[derive(Clone, Copy, Debug)]
pub struct RawIoReport {
    pub mode: IoMode,
    pub direct: bool,
    pub sqpoll: bool,
    pub bytes: u64,
    pub write_seconds: f64,
    pub read_seconds: f64,
}

impl RawIoReport {
    pub fn write_gib_per_second(&self) -> f64 {
        self.bytes as f64 / self.write_seconds.max(f64::MIN_POSITIVE) / (1024.0 * 1024.0 * 1024.0)
    }

    pub fn read_gib_per_second(&self) -> f64 {
        self.bytes as f64 / self.read_seconds.max(f64::MIN_POSITIVE) / (1024.0 * 1024.0 * 1024.0)
    }
}

/// Measure raw positioned I/O through the requested backend.
///
/// The file is truncated and overwritten. Use a scratch path, never a live
/// store.
pub fn run(path: &Path, mode: IoMode, bytes: u64, hugepages: bool) -> Result<RawIoReport> {
    let mut io = Io::new(mode, hugepages);
    let mut status = io.status();
    let bytes = (bytes.max(4096) / 4096) * 4096;
    let chunk = 1024 * 1024usize;
    let mut buffer = vec![0xABu8; chunk];
    for (index, byte) in buffer.iter_mut().enumerate() {
        *byte = (index as u8).wrapping_mul(31);
    }

    let file = match io.open(path, OpenSpec::CREATE) {
        Ok(file) => file,
        Err(_) if status.direct => {
            // Match the store's behavior: the benchmark should still produce a
            // portable number on filesystems that reject direct opens.
            io = Io::new(IoMode::Portable, hugepages);
            status = io.status();
            io.open(path, OpenSpec::CREATE)?
        }
        Err(error) => return Err(error.into()),
    };

    let started = Instant::now();
    let mut offset = 0u64;
    while offset < bytes {
        let len = (bytes - offset).min(chunk as u64) as usize;
        io.write_all_at(&file, &buffer[..len], offset)?;
        offset += len as u64;
    }
    file.sync_all()?;
    let write_seconds = started.elapsed().as_secs_f64();

    let started = Instant::now();
    let mut offset = 0u64;
    while offset < bytes {
        let len = (bytes - offset).min(chunk as u64) as usize;
        io.read_exact_at(&file, &mut buffer[..len], offset)?;
        offset += len as u64;
    }
    let read_seconds = started.elapsed().as_secs_f64();

    Ok(RawIoReport {
        mode: status.mode,
        direct: status.direct,
        sqpoll: status.sqpoll,
        bytes,
        write_seconds,
        read_seconds,
    })
}

use std::io;
use std::path::PathBuf;

use thiserror::Error;

/// Errors returned by the storage engine.
#[derive(Debug, Error)]
pub enum Error {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),

    #[error("invalid configuration: {0}")]
    Config(String),

    #[error("store format is corrupt: {0}")]
    Corrupt(String),

    #[error("unsupported store format version {0}")]
    UnsupportedVersion(u32),

    #[error("store file is closed")]
    Closed,

    #[error("the page cache is full and no page could be evicted")]
    CacheFull,

    #[error("the log is full (need {needed} bytes, {available} available)")]
    LogFull { needed: u64, available: u64 },

    #[error("page {page} is outside the store (capacity: {capacity} pages)")]
    PageOutOfRange { page: u64, capacity: u64 },

    #[error("compression error: {0}")]
    Compression(String),

    #[error("{0} is not a regular file")]
    NotAFile(PathBuf),
}

impl Error {
    /// True when the error means that on-disk data failed validation.
    pub fn is_corrupt(&self) -> bool {
        matches!(self, Self::Corrupt(_) | Self::UnsupportedVersion(_))
    }

    pub(crate) fn corrupt(message: impl Into<String>) -> Self {
        Self::Corrupt(message.into())
    }
}

/// Convenient result alias used throughout the crate.
pub type Result<T> = std::result::Result<T, Error>;

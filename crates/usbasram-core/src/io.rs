//! Positioned I/O backends.
//!
//! The portable backend is always available and uses ordinary buffered
//! positioned reads and writes. Optional features add:
//!
//! * `direct-io`: `O_DIRECT` on Linux, `FILE_FLAG_NO_BUFFERING` on Windows,
//!   and `F_NOCACHE` on macOS. Payloads are bounced through sector-aligned
//!   buffers because log records are only 8-byte aligned.
//! * `io-uring`: Linux io_uring with a pre-registered fixed buffer and an
//!   optional SQPOLL ring. Any setup failure falls back to the portable path.
//! * `iocp`: Windows overlapped I/O completed through an I/O completion port.
//!
//! Every backend implements the same positioned read/write contract, so the
//! storage engine never has to know which one is active.

use std::fs::{File, OpenOptions};
use std::io;
use std::path::Path;

#[cfg(feature = "direct-io")]
use crate::mem::AlignedBuffer;

/// Alignment required by direct and asynchronous backends.
#[cfg(any(
    feature = "direct-io",
    all(target_os = "linux", feature = "io-uring"),
    all(windows, feature = "iocp")
))]
pub(crate) const IO_ALIGNMENT: usize = 4096;

/// Requested or effective I/O engine.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IoMode {
    /// Buffered positioned I/O. Always available.
    #[default]
    Portable,
    /// Sector-aligned direct I/O without OS page-cache participation.
    Direct,
    /// Linux io_uring, using SQPOLL when the kernel permits it.
    Uring,
    /// Windows overlapped I/O completed through an I/O completion port.
    Iocp,
}

impl IoMode {
    /// Stable lowercase name used by CLI output and documentation.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Portable => "portable",
            Self::Direct => "direct",
            Self::Uring => "uring",
            Self::Iocp => "iocp",
        }
    }
}

/// Detailed description of the backend that is actually in use.
#[derive(Clone, Copy, Debug)]
pub struct IoStatus {
    pub mode: IoMode,
    /// The file handle bypasses the OS page cache.
    pub direct: bool,
    /// The Linux ring runs without submission syscalls.
    pub sqpoll: bool,
    /// The Windows handle was opened for overlapped completion.
    pub overlapped: bool,
}

/// Options for opening a store or sidecar file through a backend.
#[derive(Clone, Copy, Debug)]
pub(crate) struct OpenSpec {
    pub read: bool,
    pub write: bool,
    pub create: bool,
    pub truncate: bool,
}

impl OpenSpec {
    pub(crate) const READ_WRITE: Self = Self {
        read: true,
        write: true,
        create: false,
        truncate: false,
    };

    pub(crate) const READ_WRITE_CREATE: Self = Self {
        read: true,
        write: true,
        create: true,
        truncate: false,
    };

    pub(crate) const CREATE: Self = Self {
        read: true,
        write: true,
        create: true,
        truncate: true,
    };
}

trait Backend: Send + Sync {
    fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()>;
    fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()>;
    fn status(&self) -> IoStatus;
}

/// A positioned-I/O backend selected at store open time.
pub(crate) struct Io {
    backend: Box<dyn Backend>,
    status: IoStatus,
}

impl Io {
    /// Create a backend, falling back to portable I/O when the requested
    /// feature is unavailable or cannot be initialized on this machine.
    pub(crate) fn new(requested: IoMode, hugepages: bool) -> Self {
        let backend: Box<dyn Backend> = match requested {
            IoMode::Portable => Box::new(PortableBackend),
            IoMode::Direct => direct_backend(hugepages),
            IoMode::Uring => uring_backend(hugepages),
            IoMode::Iocp => iocp_backend(hugepages),
        };
        let status = backend.status();
        Self { backend, status }
    }

    pub(crate) fn status(&self) -> IoStatus {
        self.status
    }

    pub(crate) fn open(&self, path: &Path, spec: OpenSpec) -> io::Result<File> {
        // The IOCP backend opens its own handle so the overlapped and
        // no-buffering flags are explicit rather than dependent on how the
        // standard library translates `custom_flags`.
        #[cfg(all(windows, feature = "iocp"))]
        if self.status.mode == IoMode::Iocp {
            return iocp::open(path, spec);
        }

        let mut options = OpenOptions::new();
        options
            .read(spec.read)
            .write(spec.write)
            .create(spec.create)
            .truncate(spec.truncate);

        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            #[cfg(any(target_os = "linux", target_os = "android"))]
            if self.status.direct {
                options.custom_flags(libc::O_DIRECT);
            }
            let file = options.open(path)?;
            #[cfg(all(target_os = "macos", feature = "direct-io"))]
            if self.status.direct {
                use std::os::unix::io::AsRawFd;
                // F_NOCACHE is 48 on macOS. It is advisory: the file still
                // supports normal byte-granularity I/O.
                // SAFETY: `fd` is a live descriptor owned by `file`.
                unsafe {
                    libc::fcntl(file.as_raw_fd(), 48, 1);
                }
            }
            Ok(file)
        }

        #[cfg(windows)]
        {
            use std::os::windows::fs::OpenOptionsExt;
            const FILE_FLAG_NO_BUFFERING: u32 = 0x2000_0000;
            const FILE_FLAG_OVERLAPPED: u32 = 0x4000_0000;
            let mut flags = 0;
            if self.status.direct {
                flags |= FILE_FLAG_NO_BUFFERING;
            }
            if self.status.overlapped {
                flags |= FILE_FLAG_OVERLAPPED;
            }
            if flags != 0 {
                options.custom_flags(flags);
            }
            options.open(path)
        }

        #[cfg(not(any(unix, windows)))]
        {
            options.open(path)
        }
    }

    pub(crate) fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()> {
        self.backend.read_exact_at(file, buf, offset)
    }

    pub(crate) fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()> {
        self.backend.write_all_at(file, buf, offset)
    }
}

impl std::fmt::Debug for Io {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("Io")
            .field("mode", &self.status.mode)
            .field("direct", &self.status.direct)
            .field("sqpoll", &self.status.sqpoll)
            .field("overlapped", &self.status.overlapped)
            .finish()
    }
}

/// Round `value` up to the next multiple of `align` (a power of two).
pub(crate) fn align_up(value: u64, align: u64) -> u64 {
    debug_assert!(align.is_power_of_two());
    (value + align - 1) & !(align - 1)
}

#[cfg(feature = "direct-io")]
fn aligned(offset: u64, len: usize) -> bool {
    offset % IO_ALIGNMENT as u64 == 0 && len % IO_ALIGNMENT == 0
}

#[cfg(feature = "direct-io")]
fn aligned_pointer(buf: &[u8]) -> bool {
    buf.as_ptr() as usize % IO_ALIGNMENT == 0
}

#[cfg(any(
    feature = "direct-io",
    all(target_os = "linux", feature = "io-uring"),
    all(windows, feature = "iocp")
))]
fn aligned_span(offset: u64, len: usize) -> (u64, usize, usize) {
    let base = offset & !(IO_ALIGNMENT as u64 - 1);
    let head = (offset - base) as usize;
    let span = align_up((head + len) as u64, IO_ALIGNMENT as u64) as usize;
    (base, head, span)
}

// ---------------------------------------------------------------------------
// Portable buffered I/O
// ---------------------------------------------------------------------------

struct PortableBackend;

impl Backend for PortableBackend {
    fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()> {
        portable_read_exact_at(file, buf, offset)
    }

    fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()> {
        portable_write_all_at(file, buf, offset)
    }

    fn status(&self) -> IoStatus {
        IoStatus {
            mode: IoMode::Portable,
            direct: false,
            sqpoll: false,
            overlapped: false,
        }
    }
}

fn portable_read_exact_at(file: &File, mut buf: &mut [u8], mut offset: u64) -> io::Result<()> {
    while !buf.is_empty() {
        let read = read_at_once(file, buf, offset)?;
        if read == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "unexpected end of store file",
            ));
        }
        buf = &mut buf[read..];
        offset += read as u64;
    }
    Ok(())
}

fn portable_write_all_at(file: &File, mut buf: &[u8], mut offset: u64) -> io::Result<()> {
    while !buf.is_empty() {
        let written = write_at_once(file, buf, offset)?;
        if written == 0 {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "could not write to store file",
            ));
        }
        buf = &buf[written..];
        offset += written as u64;
    }
    Ok(())
}

#[cfg(unix)]
fn read_at_once(file: &File, buf: &mut [u8], offset: u64) -> io::Result<usize> {
    use std::os::unix::fs::FileExt;
    file.read_at(buf, offset)
}

#[cfg(windows)]
fn read_at_once(file: &File, buf: &mut [u8], offset: u64) -> io::Result<usize> {
    use std::os::windows::fs::FileExt;
    file.seek_read(buf, offset)
}

#[cfg(unix)]
fn write_at_once(file: &File, buf: &[u8], offset: u64) -> io::Result<usize> {
    use std::os::unix::fs::FileExt;
    file.write_at(buf, offset)
}

#[cfg(windows)]
fn write_at_once(file: &File, buf: &[u8], offset: u64) -> io::Result<usize> {
    use std::os::windows::fs::FileExt;
    file.seek_write(buf, offset)
}

// ---------------------------------------------------------------------------
// Direct I/O
// ---------------------------------------------------------------------------

fn direct_backend(hugepages: bool) -> Box<dyn Backend> {
    #[cfg(feature = "direct-io")]
    {
        Box::new(DirectBackend { hugepages })
    }
    #[cfg(not(feature = "direct-io"))]
    {
        let _ = hugepages;
        Box::new(PortableBackend)
    }
}

#[cfg(feature = "direct-io")]
struct DirectBackend {
    hugepages: bool,
}

#[cfg(feature = "direct-io")]
thread_local! {
    static DIRECT_BOUNCE: std::cell::RefCell<Option<AlignedBuffer>> =
        const { std::cell::RefCell::new(None) };
}

#[cfg(feature = "direct-io")]
fn with_bounce<T>(
    span: usize,
    hugepages: bool,
    operation: impl FnOnce(&mut AlignedBuffer) -> io::Result<T>,
) -> io::Result<T> {
    DIRECT_BOUNCE.with(|slot| {
        let mut slot = slot.borrow_mut();
        if slot.as_ref().is_none_or(|buffer| buffer.len() < span) {
            *slot = Some(AlignedBuffer::new(span, hugepages)?);
        }
        operation(slot.as_mut().expect("bounce buffer was just initialized"))
    })
}

#[cfg(feature = "direct-io")]
impl Backend for DirectBackend {
    fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }
        if aligned(offset, buf.len()) && aligned_pointer(buf) {
            return portable_read_exact_at(file, buf, offset);
        }
        let (base, head, span) = aligned_span(offset, buf.len());
        with_bounce(span, self.hugepages, |bounce| {
            portable_read_exact_at(file, &mut bounce.as_mut_slice()[..span], base)?;
            buf.copy_from_slice(&bounce.as_slice()[head..head + buf.len()]);
            Ok(())
        })
    }

    fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }
        if aligned(offset, buf.len()) && aligned_pointer(buf) {
            return portable_write_all_at(file, buf, offset);
        }
        let (base, head, span) = aligned_span(offset, buf.len());
        let end = offset + buf.len() as u64;
        let aligned_end = base + span as u64;
        with_bounce(span, self.hugepages, |bounce| {
            if head != 0 || aligned_end > end {
                // Read-modify-write the first and last partial sectors. The
                // file length is always sector aligned because every write
                // performed by this backend covers whole sectors.
                let file_len = file.metadata()?.len();
                let existing = file_len.min(aligned_end).saturating_sub(base) as usize;
                debug_assert_eq!(existing % IO_ALIGNMENT, 0);
                if existing > 0 {
                    portable_read_exact_at(file, &mut bounce.as_mut_slice()[..existing], base)?;
                }
                // A reused bounce buffer may hold bytes from an earlier
                // operation. Sectors beyond the current end of file must be
                // zero-filled, not stale.
                if existing < span {
                    bounce.as_mut_slice()[existing..span].fill(0);
                }
            }
            bounce.as_mut_slice()[head..head + buf.len()].copy_from_slice(buf);
            portable_write_all_at(file, &bounce.as_slice()[..span], base)
        })
    }

    fn status(&self) -> IoStatus {
        IoStatus {
            mode: IoMode::Direct,
            direct: true,
            sqpoll: false,
            overlapped: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Linux io_uring
// ---------------------------------------------------------------------------

fn uring_backend(hugepages: bool) -> Box<dyn Backend> {
    #[cfg(all(target_os = "linux", feature = "io-uring"))]
    {
        if let Some(backend) = uring::UringBackend::new(hugepages) {
            return Box::new(backend);
        }
    }
    let _ = hugepages;
    Box::new(PortableBackend)
}

#[cfg(all(target_os = "linux", feature = "io-uring"))]
mod uring {
    use std::io;
    use std::os::unix::io::{AsRawFd, RawFd};
    use std::sync::Mutex;

    use io_uring::{opcode, types, IoUring};

    use crate::mem::AlignedBuffer;

    use super::{aligned_span, Backend, File, IoMode, IoStatus, IO_ALIGNMENT};

    const RING_ENTRIES: u32 = 64;
    const BUFFER_BYTES: usize = 4 * 1024 * 1024 + 64 * 1024;
    const BUFFER_INDEX: u16 = 0;

    struct State {
        ring: IoUring,
        buffer: AlignedBuffer,
    }

    pub(super) struct UringBackend {
        state: Mutex<State>,
        sqpoll: bool,
    }

    impl UringBackend {
        pub(super) fn new(hugepages: bool) -> Option<Self> {
            let mut buffer = AlignedBuffer::new(BUFFER_BYTES, hugepages).ok()?;
            let (ring, sqpoll) = match IoUring::builder().setup_sqpoll(2_000).build(RING_ENTRIES) {
                Ok(ring) => (ring, true),
                Err(_) => (IoUring::new(RING_ENTRIES).ok()?, false),
            };
            let iovec = libc::iovec {
                iov_base: buffer.as_mut_ptr().cast(),
                iov_len: buffer.len(),
            };
            // SAFETY: the buffer is owned by the `State` that also owns the
            // ring, so it stays alive and unmoved (the allocation is stable)
            // until after the ring is dropped.
            unsafe {
                ring.submitter().register_buffers(&[iovec]).ok()?;
            }
            Some(Self {
                state: Mutex::new(State { ring, buffer }),
                sqpoll,
            })
        }
    }

    impl Backend for UringBackend {
        fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()> {
            if buf.is_empty() {
                return Ok(());
            }
            let fd = file.as_raw_fd();
            let mut done = 0usize;
            while done < buf.len() {
                let chunk = (buf.len() - done).min(BUFFER_BYTES - IO_ALIGNMENT);
                let (base, head, span) = aligned_span(offset + done as u64, chunk);
                debug_assert!(span <= BUFFER_BYTES);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| io::Error::other("io_uring state poisoned"))?;
                let pointer = state.buffer.as_mut_ptr();
                submit_read(&mut state.ring, fd, base, span, pointer)?;
                let source = &state.buffer.as_slice()[head..head + chunk];
                buf[done..done + chunk].copy_from_slice(source);
                done += chunk;
            }
            Ok(())
        }

        fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()> {
            if buf.is_empty() {
                return Ok(());
            }
            let fd = file.as_raw_fd();
            let mut file_len = file.metadata()?.len();
            let mut done = 0usize;
            while done < buf.len() {
                let chunk = (buf.len() - done).min(BUFFER_BYTES - IO_ALIGNMENT);
                let start = offset + done as u64;
                let (base, head, span) = aligned_span(start, chunk);
                let end = start + chunk as u64;
                let aligned_end = base + span as u64;
                debug_assert!(span <= BUFFER_BYTES);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| io::Error::other("io_uring state poisoned"))?;
                let pointer = state.buffer.as_mut_ptr();
                state.buffer.as_mut_slice()[..span].fill(0);
                if head != 0 || aligned_end > end {
                    let existing = file_len.min(aligned_end).saturating_sub(base) as usize;
                    debug_assert_eq!(existing % IO_ALIGNMENT, 0);
                    if existing > 0 {
                        submit_read(&mut state.ring, fd, base, existing, pointer)?;
                    }
                }
                state.buffer.as_mut_slice()[head..head + chunk]
                    .copy_from_slice(&buf[done..done + chunk]);
                submit_write(&mut state.ring, fd, base, span, pointer)?;
                file_len = file_len.max(aligned_end);
                done += chunk;
            }
            Ok(())
        }

        fn status(&self) -> IoStatus {
            IoStatus {
                mode: IoMode::Uring,
                direct: true,
                sqpoll: self.sqpoll,
                overlapped: false,
            }
        }
    }

    fn submit_read(
        ring: &mut IoUring,
        fd: RawFd,
        mut offset: u64,
        mut len: usize,
        mut pointer: *mut u8,
    ) -> io::Result<()> {
        while len > 0 {
            let entry = opcode::ReadFixed::new(types::Fd(fd), pointer, len as u32, BUFFER_INDEX)
                .offset(offset)
                .build();
            // SAFETY: the kernel reads from the registered buffer, which is
            // owned by the caller and remains valid until the completion is
            // reaped before this function returns.
            let read = unsafe { push_and_wait(ring, &entry)? };
            if read == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "unexpected end of store file",
                ));
            }
            if read > len {
                return Err(io::Error::other("io_uring read exceeded the request"));
            }
            len -= read;
            offset += read as u64;
            // SAFETY: `read` bytes were written into the registered buffer,
            // and `read <= len` keeps the pointer inside the allocation.
            pointer = unsafe { pointer.add(read) };
        }
        Ok(())
    }

    fn submit_write(
        ring: &mut IoUring,
        fd: RawFd,
        mut offset: u64,
        mut len: usize,
        mut pointer: *mut u8,
    ) -> io::Result<()> {
        while len > 0 {
            let entry = opcode::WriteFixed::new(types::Fd(fd), pointer, len as u32, BUFFER_INDEX)
                .offset(offset)
                .build();
            // SAFETY: same contract as `submit_read`.
            let written = unsafe { push_and_wait(ring, &entry)? };
            if written == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "could not write to store file",
                ));
            }
            if written > len {
                return Err(io::Error::other("io_uring write exceeded the request"));
            }
            len -= written;
            offset += written as u64;
            // SAFETY: `written <= len` keeps the pointer inside the buffer.
            pointer = unsafe { pointer.add(written) };
        }
        Ok(())
    }

    /// # Safety
    ///
    /// The caller must guarantee that the memory referenced by `entry` stays
    /// valid until the completion is reaped by this function.
    unsafe fn push_and_wait(
        ring: &mut IoUring,
        entry: &io_uring::squeue::Entry,
    ) -> io::Result<usize> {
        let pushed = unsafe { ring.submission().push(entry) };
        pushed.map_err(|_| io::Error::other("io_uring submission queue is full"))?;
        ring.submit_and_wait(1)?;
        let completion = ring
            .completion()
            .next()
            .ok_or_else(|| io::Error::other("io_uring completion queue is empty"))?;
        let result = completion.result();
        if result < 0 {
            Err(io::Error::from_raw_os_error(-result))
        } else {
            Ok(result as usize)
        }
    }
}

// ---------------------------------------------------------------------------
// Windows I/O completion ports
// ---------------------------------------------------------------------------

fn iocp_backend(hugepages: bool) -> Box<dyn Backend> {
    #[cfg(all(windows, feature = "iocp"))]
    {
        if let Some(backend) = iocp::IocpBackend::new(hugepages) {
            return Box::new(backend);
        }
    }
    let _ = hugepages;
    Box::new(PortableBackend)
}

#[cfg(all(windows, feature = "iocp"))]
mod iocp {
    use std::io;
    use std::mem::zeroed;
    use std::os::windows::io::AsRawHandle;
    use std::path::Path;
    use std::ptr;
    use std::sync::Mutex;

    use windows_sys::Win32::Foundation::{
        CloseHandle, GetHandleInformation, ERROR_IO_PENDING, GENERIC_READ, GENERIC_WRITE, HANDLE,
        INVALID_HANDLE_VALUE,
    };
    use windows_sys::Win32::Storage::FileSystem::{ReadFile, WriteFile};
    use windows_sys::Win32::System::IO::{
        CreateIoCompletionPort, GetQueuedCompletionStatus, OVERLAPPED,
    };

    use crate::mem::AlignedBuffer;

    use super::{aligned_span, Backend, File, IoMode, IoStatus, OpenSpec, IO_ALIGNMENT};

    const BUFFER_BYTES: usize = 4 * 1024 * 1024 + 64 * 1024;
    const INFINITE: u32 = u32::MAX;

    struct State {
        port: HANDLE,
        buffer: AlignedBuffer,
    }

    impl Drop for State {
        fn drop(&mut self) {
            // SAFETY: the port was created by this backend and is closed once.
            unsafe {
                CloseHandle(self.port);
            }
        }
    }

    pub(super) struct IocpBackend {
        state: Mutex<State>,
    }

    // SAFETY: the completion port handle and buffer are only used while the
    // mutex is held. Windows handles are process-wide kernel objects.
    unsafe impl Send for IocpBackend {}
    unsafe impl Sync for IocpBackend {}

    impl IocpBackend {
        pub(super) fn new(hugepages: bool) -> Option<Self> {
            let buffer = AlignedBuffer::new(BUFFER_BYTES, hugepages).ok()?;
            // SAFETY: INVALID_HANDLE_VALUE creates a new, unassociated port.
            let port =
                unsafe { CreateIoCompletionPort(INVALID_HANDLE_VALUE, std::ptr::null_mut(), 0, 0) };
            if port.is_null() || port == INVALID_HANDLE_VALUE {
                return None;
            }
            Some(Self {
                state: Mutex::new(State { port, buffer }),
            })
        }
    }

    impl Backend for IocpBackend {
        fn read_exact_at(&self, file: &File, buf: &mut [u8], offset: u64) -> io::Result<()> {
            if buf.is_empty() {
                return Ok(());
            }
            let handle = file.as_raw_handle();
            let mut done = 0usize;
            while done < buf.len() {
                let chunk = (buf.len() - done).min(BUFFER_BYTES - IO_ALIGNMENT);
                let (base, head, span) = aligned_span(offset + done as u64, chunk);
                debug_assert!(span <= BUFFER_BYTES);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| io::Error::other("IOCP state poisoned"))?;
                associate(state.port, handle).map_err(|error| {
                    io::Error::new(error.kind(), format!("IOCP associate: {error}"))
                })?;
                let pointer = state.buffer.as_mut_ptr();
                overlapped_io(state.port, handle, pointer, span, base, true)?;
                let source = &state.buffer.as_slice()[head..head + chunk];
                buf[done..done + chunk].copy_from_slice(source);
                done += chunk;
            }
            Ok(())
        }

        fn write_all_at(&self, file: &File, buf: &[u8], offset: u64) -> io::Result<()> {
            if buf.is_empty() {
                return Ok(());
            }
            let handle = file.as_raw_handle();
            let mut file_len = file.metadata()?.len();
            let mut done = 0usize;
            while done < buf.len() {
                let chunk = (buf.len() - done).min(BUFFER_BYTES - IO_ALIGNMENT);
                let start = offset + done as u64;
                let (base, head, span) = aligned_span(start, chunk);
                let end = start + chunk as u64;
                let aligned_end = base + span as u64;
                debug_assert!(span <= BUFFER_BYTES);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| io::Error::other("IOCP state poisoned"))?;
                associate(state.port, handle).map_err(|error| {
                    io::Error::new(error.kind(), format!("IOCP associate: {error}"))
                })?;
                let pointer = state.buffer.as_mut_ptr();
                state.buffer.as_mut_slice()[..span].fill(0);
                if head != 0 || aligned_end > end {
                    let existing = file_len.min(aligned_end).saturating_sub(base) as usize;
                    debug_assert_eq!(existing % IO_ALIGNMENT, 0);
                    if existing > 0 {
                        overlapped_io(state.port, handle, pointer, existing, base, true)?;
                    }
                }
                state.buffer.as_mut_slice()[head..head + chunk]
                    .copy_from_slice(&buf[done..done + chunk]);
                overlapped_io(state.port, handle, pointer, span, base, false)?;
                file_len = file_len.max(aligned_end);
                done += chunk;
            }
            Ok(())
        }

        fn status(&self) -> IoStatus {
            IoStatus {
                mode: IoMode::Iocp,
                direct: true,
                sqpoll: false,
                overlapped: true,
            }
        }
    }

    fn associate(port: HANDLE, handle: HANDLE) -> io::Result<()> {
        // SAFETY: both handles are live; re-associating an already associated
        // file returns the existing port and is harmless.
        let mut info = 0u32;
        // SAFETY: both handles are live kernel handles owned by this backend.
        let port_ok = unsafe { GetHandleInformation(port, &mut info) };
        // SAFETY: `handle` is the live file handle passed by the caller.
        let file_ok = unsafe { GetHandleInformation(handle, &mut info) };
        eprintln!(
            "DBG associate port={port:?} file={handle:?} port_ok={port_ok} file_ok={file_ok}"
        );
        let result = unsafe { CreateIoCompletionPort(handle, port, 0, 0) };
        if result.is_null() {
            let error = io::Error::last_os_error();
            // SAFETY: creates a fresh, unassociated port for diagnostics.
            let fresh =
                unsafe { CreateIoCompletionPort(INVALID_HANDLE_VALUE, ptr::null_mut(), 0, 0) };
            // SAFETY: both handles are live for the duration of the call.
            let retry = unsafe { CreateIoCompletionPort(handle, fresh, 0, 0) };
            let retry_error = io::Error::last_os_error();
            eprintln!(
                "DBG retry fresh={fresh:?} retry_null={} retry_error={retry_error:?}",
                retry.is_null()
            );
            // SAFETY: `fresh` is owned by this block.
            unsafe {
                CloseHandle(fresh);
            }
            return Err(error);
        }
        Ok(())
    }

    /// Open a file handle with explicit Windows flags for overlapped direct
    /// I/O. `std::fs::OpenOptions::custom_flags` is documented to pass these
    /// through, but opening the handle here keeps the IOCP contract visible
    /// and independent of standard-library changes.
    pub(super) fn open(path: &Path, spec: OpenSpec) -> io::Result<File> {
        use std::os::windows::ffi::OsStrExt;
        use std::os::windows::io::FromRawHandle;
        use windows_sys::Win32::Storage::FileSystem::{
            CreateFileW, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, FILE_FLAG_NO_BUFFERING,
            FILE_FLAG_OVERLAPPED, FILE_SHARE_DELETE, FILE_SHARE_READ, FILE_SHARE_WRITE,
            OPEN_ALWAYS, OPEN_EXISTING,
        };

        let mut wide: Vec<u16> = path.as_os_str().encode_wide().collect();
        wide.push(0);
        let access = (if spec.read { GENERIC_READ } else { 0 })
            | (if spec.write { GENERIC_WRITE } else { 0 });
        let disposition = if spec.truncate {
            CREATE_ALWAYS
        } else if spec.create {
            OPEN_ALWAYS
        } else {
            OPEN_EXISTING
        };
        let flags = FILE_ATTRIBUTE_NORMAL | FILE_FLAG_NO_BUFFERING | FILE_FLAG_OVERLAPPED;
        // SAFETY: `wide` is a NUL-terminated UTF-16 path that outlives the call
        // and every other argument is a plain value.
        let handle = unsafe {
            CreateFileW(
                wide.as_ptr(),
                access,
                FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                ptr::null_mut(),
                disposition,
                flags,
                ptr::null_mut(),
            )
        };
        if handle == INVALID_HANDLE_VALUE {
            return Err(io::Error::last_os_error());
        }
        eprintln!("DBG iocp open {} handle={handle:?}", path.display());
        // SAFETY: `CreateFileW` returned a fresh handle owned by this function.
        Ok(unsafe { File::from_raw_handle(handle) })
    }

    fn overlapped_io(
        port: HANDLE,
        handle: HANDLE,
        pointer: *mut u8,
        len: usize,
        offset: u64,
        read: bool,
    ) -> io::Result<()> {
        // SAFETY: OVERLAPPED is a plain data structure; zero is a valid value
        // for every field.
        let mut overlapped: OVERLAPPED = unsafe { zeroed() };
        overlapped.Anonymous.Anonymous.Offset = offset as u32;
        overlapped.Anonymous.Anonymous.OffsetHigh = (offset >> 32) as u32;
        let mut transferred = 0u32;
        let operation = if read { "read" } else { "write" };
        // SAFETY: `pointer` points to `len` bytes in the backend's registered
        // buffer, and `overlapped` lives until the completion is reaped. The
        // raw pointer is cast to the `c_void` element type expected by Win32.
        let started = unsafe {
            if read {
                ReadFile(
                    handle,
                    pointer.cast(),
                    len as u32,
                    &mut transferred,
                    &mut overlapped,
                )
            } else {
                WriteFile(
                    handle,
                    pointer.cast(),
                    len as u32,
                    &mut transferred,
                    &mut overlapped,
                )
            }
        };
        if started == 0 {
            let error = io::Error::last_os_error();
            if error.raw_os_error() != Some(ERROR_IO_PENDING as i32) {
                return Err(io::Error::new(
                    error.kind(),
                    format!("IOCP {operation} at offset {offset} ({len} bytes): {error}"),
                ));
            }
        }
        wait(port, &mut overlapped, len).map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("IOCP {operation} at offset {offset} ({len} bytes): {error}"),
            )
        })
    }

    fn wait(port: HANDLE, overlapped: &mut OVERLAPPED, expected: usize) -> io::Result<()> {
        let wanted = overlapped as *mut OVERLAPPED;
        loop {
            let mut transferred = 0u32;
            let mut key = 0usize;
            let mut completed: *mut OVERLAPPED = ptr::null_mut();
            // SAFETY: the port is live and `completed` receives a completion
            // packet owned by this backend.
            let ok = unsafe {
                GetQueuedCompletionStatus(
                    port,
                    &mut transferred,
                    &mut key,
                    &mut completed,
                    INFINITE,
                )
            };
            if completed.is_null() {
                return Err(io::Error::last_os_error());
            }
            if completed != wanted {
                continue;
            }
            if ok == 0 {
                return Err(io::Error::last_os_error());
            }
            if transferred as usize != expected {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    format!("overlapped transfer moved {transferred} of {expected} bytes"),
                ));
            }
            return Ok(());
        }
    }
}

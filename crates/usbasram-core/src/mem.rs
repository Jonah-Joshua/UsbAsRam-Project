//! Aligned page buffers and optional hugepage-backed memory.
//!
//! The default build uses ordinary heap allocations with an explicit 4 KiB
//! alignment and contains no unsafe code. The `hugepages` feature adds an
//! opt-in arena that tries `MAP_HUGETLB` on Linux and `MEM_LARGE_PAGES` on
//! Windows, falling back to 4 KiB-aligned ordinary memory when the operating
//! system or the current user cannot provide large pages.

use std::fmt;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

use crate::PAGE_SIZE;

/// A 4 KiB-aligned page with a unique mutable owner.
#[repr(C, align(4096))]
struct HeapPage([u8; PAGE_SIZE]);

pub(crate) struct AlignedPage {
    inner: PageInner,
}

enum PageInner {
    Heap(Box<HeapPage>),
    #[cfg(feature = "hugepages")]
    Huge {
        chunk: Arc<HugeChunk>,
        index: usize,
    },
}

/// Shared handle to a cache or transfer page.
///
/// Cache pages are never cloned while mutable: flush and eviction take a
/// `copy()` snapshot instead. The `DerefMut` implementation therefore only
/// succeeds when the handle is unique, and that invariant is checked in debug
/// builds and enforced with `Arc::get_mut`.
#[derive(Clone)]
pub(crate) struct PageHandle(Arc<AlignedPage>);

impl PageHandle {
    pub(crate) fn new(page: AlignedPage) -> Self {
        Self(Arc::new(page))
    }

    pub(crate) fn zeroed() -> Self {
        AlignedPage::zeroed_with(false)
    }

    pub(crate) fn zeroed_with(hugepages: bool) -> Self {
        AlignedPage::zeroed_with(hugepages)
    }

    /// Copy the page into a new allocation backed by the same memory class.
    pub(crate) fn copy(&self) -> Self {
        let mut copy = AlignedPage::zeroed_with(self.is_huge());
        copy[..].copy_from_slice(&self[..]);
        copy
    }

    pub(crate) fn is_huge(&self) -> bool {
        #[cfg(feature = "hugepages")]
        {
            matches!(
                &self.0.inner,
                PageInner::Huge { chunk, .. } if chunk.huge
            )
        }
        #[cfg(not(feature = "hugepages"))]
        {
            false
        }
    }

    #[cfg(test)]
    pub(crate) fn shares_allocation(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

impl AlignedPage {
    pub(crate) fn zeroed_with(hugepages: bool) -> PageHandle {
        #[cfg(feature = "hugepages")]
        if hugepages {
            if let Some((chunk, index)) = huge_pool().allocate() {
                let mut handle = PageHandle::new(AlignedPage {
                    inner: PageInner::Huge { chunk, index },
                });
                handle[..].fill(0);
                return handle;
            }
        }
        let _ = hugepages;
        PageHandle::new(AlignedPage {
            inner: PageInner::Heap(Box::new(HeapPage([0u8; PAGE_SIZE]))),
        })
    }

    pub(crate) fn as_slice(&self) -> &[u8] {
        match &self.inner {
            PageInner::Heap(page) => &page.0,
            #[cfg(feature = "hugepages")]
            PageInner::Huge { chunk, index } => {
                // SAFETY: `index` was handed out by `HugePool` for this chunk
                // and cannot be handed out again until this page is dropped.
                unsafe { chunk.slice(*index) }
            }
        }
    }

    pub(crate) fn as_mut_slice(&mut self) -> &mut [u8] {
        match &mut self.inner {
            PageInner::Heap(page) => &mut page.0,
            #[cfg(feature = "hugepages")]
            PageInner::Huge { chunk, index } => {
                // SAFETY: the mutable borrow of `self` guarantees exclusive
                // access to this slot, and `index` is unique to this page.
                unsafe { chunk.slice_mut(*index) }
            }
        }
    }
}

impl Deref for AlignedPage {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for AlignedPage {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl Deref for PageHandle {
    type Target = AlignedPage;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PageHandle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        Arc::get_mut(&mut self.0).expect("page handle must be unique for mutation")
    }
}

impl fmt::Debug for AlignedPage {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("AlignedPage")
            .field("huge", &PageHandle::is_huge_handle(self))
            .finish()
    }
}

impl PageHandle {
    fn is_huge_handle(page: &AlignedPage) -> bool {
        #[cfg(feature = "hugepages")]
        {
            matches!(&page.inner, PageInner::Huge { chunk, .. } if chunk.huge)
        }
        #[cfg(not(feature = "hugepages"))]
        {
            false
        }
    }
}

#[cfg(feature = "hugepages")]
impl Drop for AlignedPage {
    fn drop(&mut self) {
        if let PageInner::Huge { chunk, index } = &self.inner {
            huge_pool().release(Arc::clone(chunk), *index);
        }
    }
}

// ---------------------------------------------------------------------------
// Optional hugepage arena
// ---------------------------------------------------------------------------

#[cfg(feature = "hugepages")]
const HUGE_CHUNK_BYTES: usize = 2 * 1024 * 1024;
#[cfg(feature = "hugepages")]
const HUGE_CHUNK_PAGES: usize = HUGE_CHUNK_BYTES / PAGE_SIZE;

#[cfg(feature = "hugepages")]
struct HugeChunk {
    platform: platform::Chunk,
    huge: bool,
}

#[cfg(feature = "hugepages")]
impl HugeChunk {
    fn allocate(len: usize) -> Option<(Arc<Self>, bool)> {
        let (platform, huge) = platform::allocate(len)?;
        Some((
            Arc::new(Self {
                platform,
                huge,
            }),
            huge,
        ))
    }

    fn len(&self) -> usize {
        self.platform.len()
    }

    /// # Safety
    ///
    /// `index` must refer to a 4 KiB slot inside this chunk, and the caller
    /// must have exclusive access to that slot.
    unsafe fn slice(&self, index: usize) -> &[u8] {
        let offset = index * PAGE_SIZE;
        debug_assert!(offset + PAGE_SIZE <= self.len());
        // SAFETY: upheld by the caller and checked by the allocation contract.
        unsafe { std::slice::from_raw_parts(self.platform.as_ptr().add(offset), PAGE_SIZE) }
    }

    /// # Safety
    ///
    /// Same requirements as [`HugeChunk::slice`], plus exclusive access.
    #[allow(clippy::mut_from_ref)]
    unsafe fn slice_mut(&self, index: usize) -> &mut [u8] {
        let offset = index * PAGE_SIZE;
        debug_assert!(offset + PAGE_SIZE <= self.len());
        // SAFETY: upheld by the caller and checked by the allocation contract.
        unsafe {
            std::slice::from_raw_parts_mut(self.platform.as_ptr().add(offset), PAGE_SIZE)
        }
    }
}

#[cfg(feature = "hugepages")]
unsafe impl Send for HugeChunk {}
#[cfg(feature = "hugepages")]
unsafe impl Sync for HugeChunk {}

#[cfg(feature = "hugepages")]
struct HugePool {
    state: std::sync::Mutex<PoolState>,
}

#[cfg(feature = "hugepages")]
struct PoolState {
    free: Vec<(Arc<HugeChunk>, usize)>,
    current: Option<Arc<HugeChunk>>,
    next: usize,
    huge_chunks: usize,
}

#[cfg(feature = "hugepages")]
fn huge_pool() -> &'static HugePool {
    static POOL: std::sync::OnceLock<HugePool> = std::sync::OnceLock::new();
    POOL.get_or_init(|| HugePool {
        state: std::sync::Mutex::new(PoolState {
            free: Vec::new(),
            current: None,
            next: 0,
            huge_chunks: 0,
        }),
    })
}

#[cfg(feature = "hugepages")]
impl HugePool {
    fn allocate(&self) -> Option<(Arc<HugeChunk>, usize)> {
        let mut state = self.state.lock().ok()?;
        if let Some(slot) = state.free.pop() {
            return Some(slot);
        }
        loop {
            if let Some(chunk) = &state.current {
                if state.next < HUGE_CHUNK_PAGES {
                    let index = state.next;
                    state.next += 1;
                    return Some((Arc::clone(chunk), index));
                }
            }
            let (chunk, huge) = HugeChunk::allocate(HUGE_CHUNK_BYTES)?;
            if huge {
                state.huge_chunks += 1;
            }
            state.current = Some(chunk);
            state.next = 0;
        }
    }

    fn release(&self, chunk: Arc<HugeChunk>, index: usize) {
        if let Ok(mut state) = self.state.lock() {
            state.free.push((chunk, index));
        }
    }
}

/// Whether at least one large-page chunk is in use.
pub fn hugepages_active() -> bool {
    #[cfg(feature = "hugepages")]
    {
        huge_pool()
            .state
            .lock()
            .map(|state| state.huge_chunks > 0)
            .unwrap_or(false)
    }
    #[cfg(not(feature = "hugepages"))]
    {
        false
    }
}

/// Whether the current platform can request large pages.
pub fn hugepages_supported() -> bool {
    #[cfg(all(
        feature = "hugepages",
        any(target_os = "linux", target_os = "windows")
    ))]
    {
        true
    }
    #[cfg(not(all(
        feature = "hugepages",
        any(target_os = "linux", target_os = "windows")
    )))]
    {
        false
    }
}

// ---------------------------------------------------------------------------
// Platform chunk allocation
// ---------------------------------------------------------------------------

#[cfg(all(feature = "hugepages", target_os = "linux"))]
mod platform {
    use std::io;
    use std::ptr::NonNull;

    pub(super) struct Chunk {
        ptr: NonNull<u8>,
        len: usize,
    }

    impl Chunk {
        pub(super) fn as_ptr(&self) -> *mut u8 {
            self.ptr.as_ptr()
        }

        pub(super) fn len(&self) -> usize {
            self.len
        }
    }

    impl Drop for Chunk {
        fn drop(&mut self) {
            // SAFETY: the pointer and length came from a successful
            // `mmap` call and are released exactly once here.
            unsafe {
                libc::munmap(self.ptr.as_ptr().cast(), self.len);
            }
        }
    }

    fn map(len: usize, extra: libc::c_int) -> io::Result<NonNull<u8>> {
        // SAFETY: a fresh anonymous mapping has no aliases. The returned
        // pointer is checked against MAP_FAILED before it is used.
        let pointer = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | extra,
                -1,
                0,
            )
        };
        if pointer == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }
        NonNull::new(pointer.cast::<u8>())
            .ok_or_else(|| io::Error::other("mmap returned null"))
    }

    pub(super) fn allocate(len: usize) -> Option<(Chunk, bool)> {
        if let Ok(ptr) = map(len, libc::MAP_HUGETLB | libc::MAP_HUGE_2MB) {
            return Some((Chunk { ptr, len }, true));
        }
        // Hugepages may be disabled or unreserved. A normal anonymous mapping
        // is still 4 KiB aligned, which satisfies the sector requirement.
        map(len, 0).ok().map(|ptr| (Chunk { ptr, len }, false))
    }
}

#[cfg(all(feature = "hugepages", target_os = "windows"))]
mod platform {
    use std::ffi::c_void;
    use std::ptr::NonNull;

    const MEM_COMMIT: u32 = 0x0000_1000;
    const MEM_RESERVE: u32 = 0x0000_2000;
    const MEM_LARGE_PAGES: u32 = 0x2000_0000;
    const MEM_RELEASE: u32 = 0x0000_8000;
    const PAGE_READWRITE: u32 = 0x04;

    #[link(name = "kernel32")]
    #[allow(non_snake_case)]
    extern "system" {
        fn VirtualAlloc(
            address: *const c_void,
            size: usize,
            allocation_type: u32,
            protect: u32,
        ) -> *mut c_void;
        fn VirtualFree(address: *mut c_void, size: usize, free_type: u32) -> i32;
    }

    pub(super) struct Chunk {
        ptr: NonNull<u8>,
        len: usize,
    }

    impl Chunk {
        pub(super) fn as_ptr(&self) -> *mut u8 {
            self.ptr.as_ptr()
        }

        pub(super) fn len(&self) -> usize {
            self.len
        }
    }

    impl Drop for Chunk {
        fn drop(&mut self) {
            // SAFETY: the pointer came from VirtualAlloc and is released once.
            unsafe {
                VirtualFree(self.ptr.as_ptr().cast(), 0, MEM_RELEASE);
            }
        }
    }

    pub(super) fn allocate(len: usize) -> Option<(Chunk, bool)> {
        // SAFETY: a fresh reservation has no aliases.
        let pointer = unsafe {
            VirtualAlloc(
                std::ptr::null(),
                len,
                MEM_RESERVE | MEM_COMMIT | MEM_LARGE_PAGES,
                PAGE_READWRITE,
            )
        };
        if let Some(ptr) = NonNull::new(pointer.cast::<u8>()) {
            return Some((Chunk { ptr, len }, true));
        }
        // Large pages require SeLockMemoryPrivilege. Fall back to a normal
        // reservation when that privilege is unavailable.
        // SAFETY: same as above.
        let pointer = unsafe {
            VirtualAlloc(
                std::ptr::null(),
                len,
                MEM_RESERVE | MEM_COMMIT,
                PAGE_READWRITE,
            )
        };
        NonNull::new(pointer.cast::<u8>()).map(|ptr| (Chunk { ptr, len }, false))
    }
}

#[cfg(all(
    feature = "hugepages",
    not(any(target_os = "linux", target_os = "windows"))
))]
#[allow(dead_code)]
mod platform {
    use std::ptr::NonNull;

    pub(super) struct Chunk {
        ptr: NonNull<u8>,
        len: usize,
    }

    impl Chunk {
        pub(super) fn as_ptr(&self) -> *mut u8 {
            self.ptr.as_ptr()
        }

        pub(super) fn len(&self) -> usize {
            self.len
        }
    }

    pub(super) fn allocate(_len: usize) -> Option<(Chunk, bool)> {
        None
    }
}

// ---------------------------------------------------------------------------
// Aligned transfer buffers
// ---------------------------------------------------------------------------

/// A 4 KiB-aligned, optionally hugepage-backed scratch buffer for direct or
/// asynchronous I/O.
#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
pub(crate) struct AlignedBuffer {
    ptr: std::ptr::NonNull<u8>,
    len: usize,
    heap: bool,
    huge: bool,
    #[cfg(feature = "hugepages")]
    _chunk: Option<Arc<HugeChunk>>,
}

#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
unsafe impl Send for AlignedBuffer {}
#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
unsafe impl Sync for AlignedBuffer {}

#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
impl AlignedBuffer {
    pub(crate) fn new(len: usize, hugepages: bool) -> std::io::Result<Self> {
        let len = len.max(PAGE_SIZE).next_multiple_of(PAGE_SIZE);
        #[cfg(feature = "hugepages")]
        let len = if hugepages {
            // Large-page mappings must be a whole number of large pages.
            len.next_multiple_of(HUGE_CHUNK_BYTES)
        } else {
            len
        };
        #[cfg(feature = "hugepages")]
        if hugepages {
            if let Some((chunk, huge)) = HugeChunk::allocate(len) {
                let pointer = std::ptr::NonNull::new(chunk.platform.as_ptr())
                    .ok_or_else(|| std::io::Error::other("huge buffer returned null"))?;
                return Ok(Self {
                    ptr: pointer,
                    len,
                    heap: false,
                    huge,
                    _chunk: Some(chunk),
                });
            }
        }
        let _ = hugepages;
        let layout = std::alloc::Layout::from_size_align(len, PAGE_SIZE)
            .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?;
        // SAFETY: `layout` has a non-zero size and the returned pointer is
        // checked before use.
        let pointer = unsafe { std::alloc::alloc_zeroed(layout) };
        let pointer = std::ptr::NonNull::new(pointer)
            .ok_or_else(|| std::io::Error::other("aligned allocation failed"))?;
        Ok(Self {
            ptr: pointer,
            len,
            heap: true,
            huge: false,
            #[cfg(feature = "hugepages")]
            _chunk: None,
        })
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn as_slice(&self) -> &[u8] {
        // SAFETY: the allocation is valid for `len` bytes for the lifetime of
        // `self`, and the pointer is 4 KiB aligned.
        unsafe { std::slice::from_raw_parts(self.as_ptr(), self.len) }
    }

    pub(crate) fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: the mutable borrow guarantees exclusive access.
        unsafe { std::slice::from_raw_parts_mut(self.as_mut_ptr(), self.len) }
    }

    pub(crate) fn as_ptr(&self) -> *const u8 {
        self.ptr.as_ptr()
    }

    pub(crate) fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr.as_ptr()
    }

    pub(crate) fn is_huge(&self) -> bool {
        self.huge
    }
}

#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
impl Drop for AlignedBuffer {
    fn drop(&mut self) {
        if self.heap {
            let layout = std::alloc::Layout::from_size_align(self.len, PAGE_SIZE)
                .expect("allocation layout was valid");
            // SAFETY: the pointer came from `alloc_zeroed` with this layout.
            unsafe {
                std::alloc::dealloc(self.ptr.as_ptr(), layout);
            }
        }
        // Hugepage-backed buffers are owned by the `Arc<HugeChunk>` field.
    }
}

#[cfg(any(
    feature = "direct-io",
    feature = "io-uring",
    feature = "iocp"
))]
impl fmt::Debug for AlignedBuffer {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("AlignedBuffer")
            .field("len", &self.len)
            .field("huge", &self.is_huge())
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pages_are_sector_aligned() {
        let page = PageHandle::zeroed();
        assert_eq!(page.as_ptr() as usize % PAGE_SIZE, 0);
        assert_eq!(page.len(), PAGE_SIZE);
    }

    #[test]
    fn page_copy_is_independent() {
        let mut source = PageHandle::zeroed();
        source[0] = 7;
        let copy = source.copy();
        source[0] = 9;
        assert_eq!(copy[0], 7);
        assert!(!source.shares_allocation(&copy));
    }

    #[cfg(any(
        feature = "direct-io",
        feature = "io-uring",
        feature = "iocp"
    ))]
    #[test]
    fn transfer_buffers_are_aligned() {
        let buffer = AlignedBuffer::new(8192, false).expect("buffer");
        assert_eq!(buffer.as_ptr() as usize % PAGE_SIZE, 0);
        assert_eq!(buffer.len(), 8192);
    }
}

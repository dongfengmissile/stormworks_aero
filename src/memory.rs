//! Raw process memory helpers: resolve a game function by RVA, verify a
//! function still matches its recorded prologue, and read primitives.

use std::path::PathBuf;
use std::sync::atomic::Ordering;

use windows_sys::Win32::Foundation::HINSTANCE;

use windows_sys::Win32::System::LibraryLoader::GetModuleFileNameW;
use windows_sys::Win32::System::Memory::{VirtualQuery, MEMORY_BASIC_INFORMATION, MEM_COMMIT, PAGE_GUARD, PAGE_NOACCESS};


/// Plausible userland pointer range for a game object: below `PTR_MIN` is the
/// null page / small integers, above `PTR_MAX` is non-canonical.
pub(crate) const PTR_MIN: usize = 0x10000;
pub(crate) const PTR_MAX: usize = 0x7fff_ffff_ffff;

use crate::state::SELF_MODULE;

pub(crate) fn bytes_match(address: usize, expected: &[u8]) -> bool {
    unsafe {
        let actual = std::slice::from_raw_parts(address as *const u8, expected.len());
        actual == expected
    }
}

pub(crate) fn self_module_path() -> Option<PathBuf> {
    let module = SELF_MODULE.load(Ordering::SeqCst);
    if module == 0 {
        return None;
    }
    let mut capacity = 512usize;
    loop {
        let mut buffer = vec![0u16; capacity];
        let len =
            unsafe { GetModuleFileNameW(module as HINSTANCE, buffer.as_mut_ptr(), capacity as u32) };
        if len == 0 {
            return None;
        }
        if (len as usize) < capacity - 1 {
            return Some(PathBuf::from(String::from_utf16_lossy(
                &buffer[..len as usize],
            )));
        }
        capacity *= 2;
    }
}

// ---------------------------------------------------------------------------
// Guarded primitive reads
// ---------------------------------------------------------------------------

// Unaligned: game offsets are not always aligned, and a debug build aborts on a misaligned deref.
// Null/kernel/non-canonical addresses return 0 instead of faulting (cheap; freed memory still faults).
#[inline]
fn plausible_address(addr: usize) -> bool {
    (0x10000..0x7FFF_FFFF_0000).contains(&addr)
}

/// Logs the first few garbage reads with their caller so the bad pointer source can be fixed.
#[track_caller]
#[cold]
fn bad_read(addr: usize) {
    static COUNT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if COUNT.fetch_add(1, Ordering::Relaxed) < 8 {
        let at = std::panic::Location::caller();
        crate::logging::log_line(&format!("BAD READ address={addr:#x} from {}:{}", at.file(), at.line()));
    }
}
#[track_caller]
pub(crate) unsafe fn read_usize_at(addr: usize) -> usize {
    if !plausible_address(addr) {
        bad_read(addr);
        return 0;
    }
    (addr as *const usize).read_unaligned()
}
#[track_caller]
pub(crate) unsafe fn read_u32_at(addr: usize) -> u32 {
    if !plausible_address(addr) {
        bad_read(addr);
        return 0;
    }
    (addr as *const u32).read_unaligned()
}

/// Best-effort check that `len` bytes at `address` are committed and readable,
/// so a hook can refuse to touch a pointer it did not get from a register
/// argument. Uses region descriptors rather than `IsBadReadPtr`, which
/// documents itself as unreliable when another thread mutates the address space
/// (the game rebuilds block containers while the scene walk runs).
///
/// The region is cached per CALL only: a region validated on an earlier tick can
/// already be decommitted (the game frees containers it rebuilt), and a stale
/// cache would let every later probe pass while the pages are unmapped.
pub(crate) fn readable(address: usize, len: usize) -> bool {
    if let Some(hit) = REGION_CACHE.with(|cache| {
        let cache = cache.borrow();
        if !cache.active {
            return None;
        }
        let end = address.checked_add(len)?;
        cache
            .regions
            .iter()
            .any(|&(base, region_end)| base <= address && end <= region_end)
            .then_some(true)
    }) {
        return hit;
    }
    region_ok(address, len)
}

/// Readable regions verified during one synchronous game-thread pass (a sim
/// tick, a mesh-map walk). `VirtualQuery` is a syscall per probe and the sim
/// probes every component of every body; inside one pass on the game's own
/// thread the game cannot free those containers under us, so positive results
/// are reused until the scope ends. Outside a scope nothing is cached (see the
/// stale-cache note above).
struct RegionCache {
    active: bool,
    regions: Vec<(usize, usize)>,
}

thread_local! {
    static REGION_CACHE: std::cell::RefCell<RegionCache> =
        const { std::cell::RefCell::new(RegionCache { active: false, regions: Vec::new() }) };
}

/// Enables the per-thread region cache until dropped (not reentrant-safe by
/// design: a nested scope just keeps the outer one's cache).
pub(crate) struct RegionCacheScope {
    outer: bool,
}

impl RegionCacheScope {
    pub(crate) fn new() -> Self {
        let outer = REGION_CACHE.with(|cache| {
            let mut cache = cache.borrow_mut();
            let was = cache.active;
            cache.active = true;
            was
        });
        RegionCacheScope { outer }
    }
}

impl Drop for RegionCacheScope {
    fn drop(&mut self) {
        if !self.outer {
            REGION_CACHE.with(|cache| {
                let mut cache = cache.borrow_mut();
                cache.active = false;
                cache.regions.clear();
            });
        }
    }
}

fn remember_region(base: usize, end: usize) {
    REGION_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        if !cache.active {
            return;
        }
        if cache.regions.len() >= 64 {
            cache.regions.remove(0);
        }
        cache.regions.push((base, end));
    });
}

fn region_ok(address: usize, len: usize) -> bool {
    use core::ffi::c_void;
    if address < PTR_MIN {
        return false;
    }
    let Some(end) = address.checked_add(len) else {
        return false;
    };
    let mut cursor = address;
    let mut cached: Option<(usize, usize)> = None;
    while cursor < end {
        if let Some((base, region_end)) = cached {
            if base <= cursor && cursor < region_end {
                cursor = region_end;
                continue;
            }
        }
        let mut info = MEMORY_BASIC_INFORMATION::default();
        let written = unsafe {
            VirtualQuery(
                cursor as *const c_void,
                &mut info,
                std::mem::size_of::<MEMORY_BASIC_INFORMATION>(),
            )
        };
        if written == 0 || info.State != MEM_COMMIT || info.Protect == 0 {
            return false;
        }
        if info.Protect & (PAGE_NOACCESS | PAGE_GUARD) != 0 {
            return false;
        }
        let base = info.BaseAddress as usize;
        let Some(region_end) = base.checked_add(info.RegionSize) else {
            return false;
        };
        if region_end <= cursor {
            return false;
        }
        cached = Some((base, region_end));
        remember_region(base, region_end);
        cursor = region_end;
    }
    true
}

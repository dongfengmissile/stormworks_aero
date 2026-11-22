//! Hooks two plugins may both place (the editor tick, the toolbar, the vehicle file, the block
//! paint): each plugin checks a function's original prologue before it hooks it, and a sibling's
//! hook replaces that prologue with a jump. Plugins install their hooks one at a time (`Exclusive`)
//! and list every function they verified in a per-process table, so the next plugin hooks the
//! sibling's jump instead (MinHook follows a leading `jmp rel32`, so the detours chain).

use std::sync::atomic::{AtomicUsize, Ordering};

use windows_sys::Win32::Foundation::{CloseHandle, HANDLE, INVALID_HANDLE_VALUE};
use windows_sys::Win32::System::Memory::{CreateFileMappingW, MapViewOfFile, FILE_MAP_ALL_ACCESS, PAGE_READWRITE};
use windows_sys::Win32::System::Threading::{CreateMutexW, GetCurrentProcessId, ReleaseMutex, WaitForSingleObject, INFINITE};

use crate::memory::bytes_match;

const SLOTS: usize = 511;

/// The shared table: count, then the verified function addresses.
#[repr(C)]
struct Table {
    count: u64,
    addrs: [u64; SLOTS],
}

static TABLE: AtomicUsize = AtomicUsize::new(0);

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain([0]).collect()
}

fn name(what: &str) -> Vec<u16> {
    wide(&format!("Local\\StormworksPluginHooks.{what}.{}", unsafe { GetCurrentProcessId() }))
}

/// Held while a plugin installs its hooks; the other plugins wait.
pub struct Exclusive(HANDLE);

impl Exclusive {
    pub fn take() -> Exclusive {
        unsafe {
            let m = CreateMutexW(std::ptr::null(), 0, name("lock").as_ptr());
            if !m.is_null() {
                WaitForSingleObject(m, INFINITE); // an abandoned lock is still ours
            }
            Exclusive(m)
        }
    }
}

impl Drop for Exclusive {
    fn drop(&mut self) {
        if !self.0.is_null() {
            unsafe {
                ReleaseMutex(self.0);
                CloseHandle(self.0);
            }
        }
    }
}

fn table() -> Option<&'static mut Table> {
    let mut t = TABLE.load(Ordering::Acquire);
    if t == 0 {
        unsafe {
            let size = std::mem::size_of::<Table>() as u32;
            let h = CreateFileMappingW(INVALID_HANDLE_VALUE, std::ptr::null(), PAGE_READWRITE, 0, size, name("table").as_ptr());
            if h.is_null() {
                return None;
            }
            // The handle stays open for the process's life: the table must outlive this plugin's view.
            t = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size as usize).Value as usize;
        }
        if t == 0 {
            return None;
        }
        TABLE.store(t, Ordering::Release);
    }
    // Only touched under `Exclusive`.
    Some(unsafe { &mut *(t as *mut Table) })
}

/// Whether the function at `addr` may be hooked: its prologue is the recorded one (then listed for
/// the plugins after this one), or a sibling plugin verified it and replaced it with its jump. Call
/// under `Exclusive`.
pub fn verify(addr: usize, prologue: &[u8]) -> bool {
    let Some(t) = table() else { return bytes_match(addr, prologue) };
    let n = (t.count as usize).min(SLOTS);
    let listed = t.addrs[..n].contains(&(addr as u64));
    if bytes_match(addr, prologue) {
        if !listed && n < SLOTS {
            t.addrs[n] = addr as u64;
            t.count += 1;
        }
        return true;
    }
    listed && bytes_match(addr, &[0xE9])
}

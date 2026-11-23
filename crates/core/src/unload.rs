//! Taking a plugin out of the running game for the proxy mod loader (`swml_ready` / `swml_unload`),
//! so it can be swapped for a new build without restarting.
//!
//! The loader unloads plugins newest first, so ours are the top hooks on every function two plugins
//! share (`crate::hook`) and on the window procedure: switching them off puts back what the plugin
//! below put there. Then no game thread may still be in our code before the DLL is freed.

use std::ffi::c_void;
use std::mem::{size_of, zeroed};
use std::ptr::null_mut;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use windows_sys::Win32::Foundation::{CloseHandle, HWND, INVALID_HANDLE_VALUE};
use windows_sys::Win32::Graphics::Gdi::WindowFromDC;
use windows_sys::Win32::Graphics::OpenGL::wglGetCurrentDC;
use windows_sys::Win32::System::Diagnostics::Debug::{GetThreadContext, RtlLookupFunctionEntry, RtlVirtualUnwind, CONTEXT, CONTEXT_FULL_AMD64, UNW_FLAG_NHANDLER};
use windows_sys::Win32::System::Diagnostics::ToolHelp::{CreateToolhelp32Snapshot, Thread32First, Thread32Next, TH32CS_SNAPTHREAD, THREADENTRY32};
use windows_sys::Win32::System::Memory::{VirtualQuery, MEMORY_BASIC_INFORMATION, MEM_COMMIT, MEM_IMAGE, PAGE_GUARD, PAGE_NOACCESS};
use windows_sys::Win32::System::Threading::{GetCurrentProcessId, GetCurrentThreadId, OpenThread, ResumeThread, SuspendThread, THREAD_GET_CONTEXT, THREAD_QUERY_INFORMATION, THREAD_SUSPEND_RESUME};
use windows_sys::Win32::UI::WindowsAndMessaging::{GetWindowLongPtrW, SetWindowLongPtrW, GWLP_WNDPROC};

use crate::logging::log_line;
use crate::state::SELF_MODULE;

/// Set when unloading starts: background loops end.
pub static STOP: AtomicBool = AtomicBool::new(false);
/// The bootstrap finished: every hook it installs is in.
pub static BOOTED: AtomicBool = AtomicBool::new(false);
/// The overlay drew its first frame (its window procedure is in), or there is no overlay.
pub static OVERLAY_UP: AtomicBool = AtomicBool::new(false);
/// The game window and its procedure before the overlay subclassed it.
static WINDOW: Mutex<Option<(usize, isize)>> = Mutex::new(None);

/// Sets `BOOTED` when dropped: at the end of the bootstrap, or at any early return.
pub struct Booted;
impl Drop for Booted {
    fn drop(&mut self) {
        BOOTED.store(true, Ordering::SeqCst);
    }
}

/// `swml_ready`: the loader loads the next plugin only after this, so hooks and window procedures
/// stack in load order.
pub fn ready() -> bool {
    BOOTED.load(Ordering::SeqCst) && OVERLAY_UP.load(Ordering::SeqCst)
}

/// Render thread, in the overlay's `initialize`: hudhook subclasses the window right after it.
pub fn note_window() {
    unsafe {
        let hwnd = WindowFromDC(wglGetCurrentDC());
        if !hwnd.is_null() {
            *WINDOW.lock().unwrap_or_else(|e| e.into_inner()) = Some((hwnd as usize, GetWindowLongPtrW(hwnd, GWLP_WNDPROC)));
        }
    }
    OVERLAY_UP.store(true, Ordering::SeqCst);
}

/// This DLL's address range.
fn image() -> std::ops::Range<u64> {
    let base = SELF_MODULE.load(Ordering::SeqCst);
    if base == 0 {
        return 0..0;
    }
    // PE header: e_lfanew at 0x3C, SizeOfImage at 0x50 into the NT headers.
    let size = unsafe { ((base + *((base + 0x3C) as *const u32) as usize + 0x50) as *const u32).read_unaligned() };
    base as u64..base as u64 + size as u64
}

pub(crate) fn region(addr: u64) -> Option<MEMORY_BASIC_INFORMATION> {
    let mut info: MEMORY_BASIC_INFORMATION = unsafe { zeroed() };
    let ok = unsafe { VirtualQuery(addr as *const c_void, &mut info, size_of::<MEMORY_BASIC_INFORMATION>()) } != 0;
    (ok && info.State == MEM_COMMIT && info.Protect & (PAGE_GUARD | PAGE_NOACCESS) == 0).then_some(info)
}

#[repr(C, align(16))]
struct Ctx(CONTEXT);

/// Stops thread `tid` and walks its stack by the modules' unwind tables. Allocates nothing while
/// the thread is stopped: it may hold the heap lock.
pub(crate) unsafe fn stack_of(tid: u32, out: &mut [u64; 40]) -> usize {
    let h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, 0, tid);
    if h.is_null() {
        return 0;
    }
    let mut n = 0;
    if SuspendThread(h) != u32::MAX {
        let mut c: Ctx = zeroed();
        c.0.ContextFlags = CONTEXT_FULL_AMD64;
        if GetThreadContext(h, &mut c.0) != 0 {
            let c = &mut c.0;
            while n < out.len() && c.Rip != 0 {
                out[n] = c.Rip;
                n += 1;
                // Code outside any module (Wine's Unix side, generated code): no tables to go on.
                if region(c.Rip).is_none_or(|r| r.Type != MEM_IMAGE) {
                    break;
                }
                let mut base = 0u64;
                let f = RtlLookupFunctionEntry(c.Rip, &mut base, null_mut());
                if f.is_null() {
                    // A leaf function: the return address is on top of the stack.
                    if region(c.Rsp).is_none() {
                        break;
                    }
                    c.Rip = *(c.Rsp as *const u64);
                    c.Rsp += 8;
                } else {
                    let (mut data, mut frame) = (null_mut(), 0u64);
                    RtlVirtualUnwind(UNW_FLAG_NHANDLER, base, c.Rip, f, c, &mut data, &mut frame, null_mut());
                }
            }
        }
        ResumeThread(h);
    }
    CloseHandle(h);
    n
}

/// Every thread of the game but the calling one.
pub(crate) fn other_threads() -> Vec<u32> {
    let (me, pid) = unsafe { (GetCurrentThreadId(), GetCurrentProcessId()) };
    let mut tids = Vec::new();
    unsafe {
        let snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
        if snap == INVALID_HANDLE_VALUE {
            return tids;
        }
        let mut e: THREADENTRY32 = zeroed();
        e.dwSize = size_of::<THREADENTRY32>() as u32;
        let mut ok = Thread32First(snap, &mut e) != 0;
        while ok {
            if e.th32OwnerProcessID == pid && e.th32ThreadID != me {
                tids.push(e.th32ThreadID);
            }
            ok = Thread32Next(snap, &mut e) != 0;
        }
        CloseHandle(snap);
    }
    tids
}

/// Waits until no other thread runs or returns into this DLL (hook detours finishing, our own
/// threads ending). False after `timeout`.
fn quiet(timeout: Duration) -> bool {
    let range = image();
    let until = Instant::now() + timeout;
    loop {
        let busy = other_threads().into_iter().find(|&tid| {
            let mut pcs = [0u64; 40];
            let n = unsafe { stack_of(tid, &mut pcs) };
            pcs[..n].iter().any(|pc| range.contains(pc))
        });
        match busy {
            None => return true,
            Some(tid) if Instant::now() >= until => {
                log_line(&format!("unload: thread {tid} is still in the plugin"));
                return false;
            }
            Some(_) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

fn wgl_swap_buffers() -> usize {
    use windows_sys::Win32::System::LibraryLoader::{GetModuleHandleA, GetProcAddress};
    unsafe { GetProcAddress(GetModuleHandleA(c"opengl32.dll".as_ptr().cast()), c"wglSwapBuffers".as_ptr().cast()) }.map_or(0, |f| f as usize)
}

/// `at` starts with MinHook's jump (`jmp rel32` to a relay `jmp [rip+0]` with the detour's
/// address) into this DLL.
fn hooked_here(at: usize) -> bool {
    if at == 0 || region(at as u64).is_none() || unsafe { *(at as *const u8) } != 0xE9 {
        return false;
    }
    let relay = (at as i64 + 5 + unsafe { ((at + 1) as *const i32).read_unaligned() } as i64) as usize;
    let mut to = relay as u64;
    if region(relay as u64).is_some() && unsafe { std::slice::from_raw_parts(relay as *const u8, 6) } == [0xFF, 0x25, 0, 0, 0, 0] {
        to = unsafe { ((relay + 6) as *const u64).read_unaligned() };
    }
    image().contains(&to)
}

/// `swml_unload`. `before`: put back direct patches into game code or data (while our hooks still
/// run); `after`: put back game state once no thread is in our code any more. False: not safe to
/// free (the reason is logged); before the first change, the plugin is left as it was.
pub fn unload(before: impl FnOnce(), after: impl FnOnce()) -> bool {
    if !ready() {
        log_line("unload refused: the plugin is still starting");
        return false;
    }
    let window = *WINDOW.lock().unwrap_or_else(|e| e.into_inner());
    if let Some((hwnd, _)) = window {
        let proc = unsafe { GetWindowLongPtrW(hwnd as HWND, GWLP_WNDPROC) } as u64;
        if !image().contains(&proc) {
            log_line("unload refused: another overlay subclassed the game window after ours; unload that one first");
            return false;
        }
    }
    STOP.store(true, Ordering::SeqCst);
    // Buttons and shelves out of the workbench toolbar (game thread), before any hook goes.
    crate::toolbar::take_out();
    before();
    // One MinHook per DLL (hudhook's wglSwapBuffers hook included): ours are the top hooks, so
    // each target gets back what was there before us. The trampolines stay allocated for threads
    // still on their way through them.
    if let Err(e) = unsafe { minhook::MinHook::disable_all_hooks() } {
        log_line(&format!("unload: hooks not disabled ({e:?}); not unloaded"));
        return false;
    }
    // shortcut: assumes hudhook's MinHook is the same copy as ours (one per DLL at link time);
    // if it ever is not, its hook would still lead here, and the unload stops.
    if hooked_here(wgl_swap_buffers()) {
        log_line("unload: wglSwapBuffers still leads into the plugin; not unloaded");
        return false;
    }
    if let Some((hwnd, previous)) = window {
        unsafe { SetWindowLongPtrW(hwnd as HWND, GWLP_WNDPROC, previous) };
    }
    crate::fault::uninstall();
    if !quiet(Duration::from_secs(5)) {
        return false;
    }
    after();
    log_line("unloaded");
    true
}

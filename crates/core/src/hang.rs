//! Hang watch (logging builds): when the game stops drawing or its memory jumps, logs every game
//! thread's stack as module+offset (ours: addr2line on the debug DLL; the game's: IDA), and ends a
//! hung game before it takes all the system's memory.

use std::ffi::c_void;
use std::mem::{size_of, zeroed};
use std::ptr::null_mut;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use windows_sys::Win32::Foundation::{CloseHandle, INVALID_HANDLE_VALUE};
use windows_sys::Win32::System::Diagnostics::Debug::{GetThreadContext, RtlLookupFunctionEntry, RtlVirtualUnwind, CONTEXT, CONTEXT_FULL_AMD64, UNW_FLAG_NHANDLER};
use windows_sys::Win32::System::Diagnostics::ToolHelp::{CreateToolhelp32Snapshot, Thread32First, Thread32Next, TH32CS_SNAPTHREAD, THREADENTRY32};
use windows_sys::Win32::System::LibraryLoader::GetModuleFileNameW;
use windows_sys::Win32::System::Memory::{VirtualQuery, MEMORY_BASIC_INFORMATION, MEM_COMMIT, MEM_IMAGE, PAGE_GUARD, PAGE_NOACCESS};
use windows_sys::Win32::System::ProcessStatus::{K32GetProcessMemoryInfo, PROCESS_MEMORY_COUNTERS};
use windows_sys::Win32::System::SystemInformation::{GlobalMemoryStatusEx, MEMORYSTATUSEX};
use windows_sys::Win32::System::Threading::{GetCurrentProcess, GetCurrentProcessId, GetCurrentThreadId, OpenThread, ResumeThread, SuspendThread, TerminateProcess, THREAD_GET_CONTEXT, THREAD_QUERY_INFORMATION, THREAD_SUSPEND_RESUME};

use crate::logging::log_line;

/// Overlay frames drawn (render thread).
pub static FRAMES: AtomicU64 = AtomicU64::new(0);

const MB: u64 = 1 << 20;

/// The game's resident memory, MB.
pub fn rss_mb() -> u64 {
    let mut c: PROCESS_MEMORY_COUNTERS = unsafe { zeroed() };
    c.cb = size_of::<PROCESS_MEMORY_COUNTERS>() as u32;
    if unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut c, c.cb) } != 0 { c.WorkingSetSize as u64 / MB } else { 0 }
}

fn total_mb() -> u64 {
    let mut m: MEMORYSTATUSEX = unsafe { zeroed() };
    m.dwLength = size_of::<MEMORYSTATUSEX>() as u32;
    if unsafe { GlobalMemoryStatusEx(&mut m) } != 0 { m.ullTotalPhys / MB } else { u64::MAX }
}

pub struct Watch {
    frames: u64,
    since: Instant,
    samples: usize,
    /// Memory 2 s and 1 s ago, MB.
    mem: [u64; 2],
    last_jump: Option<Instant>,
}

impl Watch {
    pub fn new() -> Self {
        let m = rss_mb();
        Watch { frames: 0, since: Instant::now(), samples: 0, mem: [m, m], last_jump: None }
    }

    /// Once a second.
    pub fn check(&mut self) {
        let (frames, mem, now) = (FRAMES.load(Ordering::Relaxed), rss_mb(), Instant::now());
        if frames != self.frames {
            (self.frames, self.since, self.samples) = (frames, now, 0);
        }
        let stalled = frames > 0 && now - self.since >= Duration::from_secs(3);
        // Three looks per stall: the same place each time is a loop (a world load moves on).
        let due = stalled && [3, 8, 20].get(self.samples).is_some_and(|&s| now - self.since >= Duration::from_secs(s));
        let jump = mem > self.mem[0] + 1024 && self.last_jump.is_none_or(|t| now - t >= Duration::from_secs(10));
        if due || jump {
            if due {
                self.samples += 1;
            } else {
                self.last_jump = Some(now);
            }
            let what = if due { "the game stopped drawing" } else { "memory jumped" };
            let stage = crate::fault::stage_name(crate::fault::STAGE.load(Ordering::Relaxed));
            log_line(&format!("hang watch: {what} (no new frame for {} s, {mem} MB, {} MB 2 s ago, plugin stage {stage}); thread stacks:", (now - self.since).as_secs(), self.mem[0]));
            dump_threads();
        }
        self.mem = [self.mem[1], mem];
        // A hung game holding 80% of the RAM never comes back: end it before the system runs out.
        let total = total_mb();
        if stalled && mem > total / 10 * 8 {
            log_line(&format!("hang watch: hung at {mem} MB of {total} MB; ending the game. Last stacks:"));
            dump_threads();
            unsafe { TerminateProcess(GetCurrentProcess(), 0xDEAD) };
        }
    }
}

fn region(addr: u64) -> Option<MEMORY_BASIC_INFORMATION> {
    let mut info: MEMORY_BASIC_INFORMATION = unsafe { zeroed() };
    let ok = unsafe { VirtualQuery(addr as *const c_void, &mut info, size_of::<MEMORY_BASIC_INFORMATION>()) } != 0;
    (ok && info.State == MEM_COMMIT && info.Protect & (PAGE_GUARD | PAGE_NOACCESS) == 0).then_some(info)
}

#[repr(C, align(16))]
struct Ctx(CONTEXT);

/// Stops thread `tid` and walks its stack by the modules' unwind tables. Allocates nothing while
/// the thread is stopped: it may hold the heap lock.
unsafe fn stack_of(tid: u32, out: &mut [u64; 40]) -> usize {
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

/// `module+0xoffset` of a code address.
fn named(pc: u64, names: &mut Vec<(usize, String)>) -> String {
    let Some(r) = region(pc).filter(|r| r.Type == MEM_IMAGE) else { return format!("{pc:#x}") };
    let base = r.AllocationBase as usize;
    if !names.iter().any(|n| n.0 == base) {
        let mut buf = [0u16; 260];
        let len = unsafe { GetModuleFileNameW(base as _, buf.as_mut_ptr(), 260) } as usize;
        let full = String::from_utf16_lossy(&buf[..len]);
        names.push((base, full.rsplit(['\\', '/']).next().unwrap_or("?").to_string()));
    }
    let name = &names.iter().find(|n| n.0 == base).unwrap().1;
    format!("{name}+{:#x}", pc as usize - base)
}

/// Logs the stack of every thread of the game that is in the game's code or ours.
fn dump_threads() {
    let (me, pid) = unsafe { (GetCurrentThreadId(), GetCurrentProcessId()) };
    let mut tids = Vec::new();
    unsafe {
        let snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
        if snap == INVALID_HANDLE_VALUE {
            return log_line("  (no thread list)");
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
    let mut stacks = Vec::with_capacity(tids.len());
    for tid in tids {
        let mut pcs = [0u64; 40];
        let n = unsafe { stack_of(tid, &mut pcs) };
        stacks.push((tid, pcs, n));
    }
    let (mut names, mut quiet) = (Vec::new(), 0);
    for (tid, pcs, n) in stacks {
        let frames: Vec<String> = pcs[..n].iter().map(|&pc| named(pc, &mut names)).collect();
        if !frames.iter().any(|f| f.to_ascii_lowercase().starts_with("stormworks")) {
            quiet += 1; // Wine's own threads
            continue;
        }
        log_line(&format!("  thread {tid}: {}", frames.join(" < ")));
    }
    log_line(&format!("  ({quiet} other threads only in system code)"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walks_a_busy_thread() {
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            tx.send(unsafe { GetCurrentThreadId() }).unwrap();
            let mut v = Vec::new();
            loop {
                v.push(0u8);
                if v.len() > 1 << 20 {
                    v = Vec::new();
                }
                std::hint::black_box(&v);
            }
        });
        let tid = rx.recv().unwrap();
        std::thread::sleep(Duration::from_millis(50));
        for _ in 0..20 {
            let mut pcs = [0u64; 40];
            let n = unsafe { stack_of(tid, &mut pcs) };
            let mut names = Vec::new();
            let frames: Vec<String> = pcs[..n].iter().map(|&pc| named(pc, &mut names)).collect();
            assert!(frames.iter().filter(|f| f.contains(".exe+")).count() >= 3, "{frames:?}");
            if std::env::var_os("HANG_PRINT").is_some() {
                println!("{frames:?}");
            }
        }
    }
}

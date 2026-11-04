//! DLL entry, bootstrap (log, fault handler, kill switches), editor hook and overlay start.

use std::ffi::c_void;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::thread;
use std::time::Duration;

use windows_sys::Win32::Foundation::HINSTANCE;
use windows_sys::Win32::System::LibraryLoader::{DisableThreadLibraryCalls, GetModuleFileNameW, GetModuleHandleW};

use crate::logging::{log_line, set_log_path};
use crate::memory::self_module_path;
use crate::offsets::{GAME_BUILD_LABEL, PLUGIN_BUILD_LABEL};
use crate::state::{MODULE_BASE, SELF_MODULE};

/// Kill switch: `<game>/<marker>` file or `<env>=1`, so a crash can be bisected without a rebuild.
fn disabled(dir: &Path, marker: &str, env: &str) -> bool {
    std::env::var(env).as_deref() == Ok("1") || dir.join(marker).exists()
}

static PAINT_OFF: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub(crate) fn paint_disabled() -> bool {
    PAINT_OFF.load(Ordering::SeqCst)
}

fn exe_dir() -> Option<PathBuf> {
    let mut buf = vec![0u16; 1024];
    let len = unsafe { GetModuleFileNameW(std::ptr::null_mut(), buf.as_mut_ptr(), buf.len() as u32) } as usize;
    (len > 0 && len < buf.len()).then(|| PathBuf::from(String::from_utf16_lossy(&buf[..len])).parent().map(Path::to_path_buf))?
}

#[cfg(feature = "logging")]
fn install_panic_hook() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let message = info
            .payload()
            .downcast_ref::<&str>()
            .map(|s| s.to_string())
            .or_else(|| info.payload().downcast_ref::<String>().cloned())
            .unwrap_or_default();
        let at = info.location().map(|l| format!("{}:{}", l.file(), l.line())).unwrap_or_default();
        log_line(&format!("PANIC: {} at {at} stage={}", message.replace('\n', " "), crate::fault::stage_name(crate::fault::STAGE.load(Ordering::Relaxed))));
        previous(info);
    }));
}

fn bootstrap() {
    let Some(dll_path) = self_module_path() else { return };
    let plugin_dir = dll_path.parent().unwrap_or_else(|| Path::new(".")).to_path_buf();
    let log_dir = plugin_dir.join("logs");
    let _ = fs::create_dir_all(&log_dir);
    let log_path = log_dir.join("aero.log");
    if log_path.exists() {
        let _ = fs::rename(&log_path, log_dir.join("aero.prev.log"));
    }
    set_log_path(log_path.clone());
    #[cfg(feature = "logging")]
    install_panic_hook();
    crate::fault::install(&log_path);
    log_line(&format!("aero bootstrap start build={GAME_BUILD_LABEL} plugin={PLUGIN_BUILD_LABEL} dll={}", dll_path.display()));

    let base = unsafe { GetModuleHandleW(std::ptr::null()) } as usize;
    if base == 0 {
        log_line("could not resolve the main module base");
        return;
    }
    MODULE_BASE.store(base, Ordering::SeqCst);

    if disabled(&plugin_dir, "aero_disable_hook", "AERO_NO_HOOK") {
        log_line("editor hook disabled by request (aero_disable_hook / AERO_NO_HOOK)");
    } else {
        match unsafe { crate::hooks::install_hooks() } {
            Ok(()) => log_line("hooks installed"),
            Err(e) => log_line(&e),
        }
    }

    {
        let game = exe_dir().unwrap_or_else(|| plugin_dir.clone());
        let appdata = std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Stormworks")).unwrap_or_default();
        let _ = crate::flight::PATHS.set((game, appdata));
    }
    if disabled(&plugin_dir, "aero_disable_flight", "AERO_NO_FLIGHT") {
        log_line("in-game flight forces disabled by request (aero_disable_flight / AERO_NO_FLIGHT)");
    } else {
        match unsafe { crate::hooks::install_flight_hooks() } {
            Ok(()) => crate::flight::HOOKED.store(true, Ordering::SeqCst),
            Err(e) => log_line(&e),
        }
        PAINT_OFF.store(disabled(&plugin_dir, "aero_disable_paint", "AERO_NO_PAINT"), Ordering::SeqCst);
        if let Err(e) = unsafe { crate::hooks::install_camera_hook() } {
            log_line(&e);
        }
    }

    if disabled(&plugin_dir, "aero_disable_overlay", "AERO_NO_OVERLAY") {
        log_line("overlay disabled by request (aero_disable_overlay / AERO_NO_OVERLAY)");
    } else {
        let game = exe_dir().unwrap_or_else(|| plugin_dir.clone());
        let appdata = std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Stormworks")).unwrap_or_default();
        log_line(&format!("game dir {} appdata {}", game.display(), appdata.display()));
        start_overlay(game, appdata);
    }

    // Verbose builds: per-10-second counters, so "never called" and "rejected" are distinguishable.
    #[cfg(feature = "logging")]
    loop {
        thread::sleep(Duration::from_secs(10));
        log_line(&format!(
            "heartbeat: editor ticks={} active={} snapshots={} rejects={} | flight on={} ticks={} vehicles={} models={} applied={}",
            crate::editor::TICKS.load(Ordering::Relaxed),
            crate::editor::editor_active(),
            crate::editor::SNAPSHOTS.load(Ordering::Relaxed),
            crate::editor::REJECTS.load(Ordering::Relaxed),
            crate::flight::ENABLED.load(Ordering::Relaxed),
            crate::flight::TICKS.load(Ordering::Relaxed),
            crate::flight::VEHICLES.load(Ordering::Relaxed),
            crate::flight::MODELS.load(Ordering::Relaxed),
            crate::flight::APPLIED.load(Ordering::Relaxed),
        ));
    }
}

fn start_overlay(game: PathBuf, appdata: PathBuf) {
    use hudhook::hooks::opengl3::ImguiOpenGl3Hooks;
    use hudhook::{windows, Hudhook};
    let module_raw = SELF_MODULE.load(Ordering::SeqCst);
    let apply = move || -> Result<(), String> {
        let overlay = crate::overlay::Overlay::new(game, appdata);
        // SAFETY: hudhook hooks wglSwapBuffers and the window procedure with its own MinHook
        // instance; it chains with swQolSuite / video_get doing the same.
        Hudhook::builder()
            .with::<ImguiOpenGl3Hooks>(overlay)
            .with_hmodule(windows::Win32::Foundation::HINSTANCE(module_raw as isize))
            .build()
            .apply()
            .map_err(|status| format!("hudhook apply failed: {status:?}"))
    };
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(apply)) {
        Ok(Ok(())) => log_line("overlay hooks applied"),
        Ok(Err(e)) => log_line(&format!("overlay failed: {e}")),
        Err(_) => log_line("overlay setup panicked"),
    }
}

#[no_mangle]
pub extern "system" fn DllMain(module: HINSTANCE, reason: u32, reserved: *mut c_void) -> i32 {
    const DLL_PROCESS_ATTACH: u32 = 1;
    const DLL_PROCESS_DETACH: u32 = 0;
    if reason == DLL_PROCESS_DETACH {
        crate::fault::note_detach(!reserved.is_null());
    }
    if reason == DLL_PROCESS_ATTACH {
        unsafe {
            let _ = DisableThreadLibraryCalls(module);
        }
        SELF_MODULE.store(module as usize, Ordering::SeqCst);
        let _ = thread::Builder::new().name("stormworks-aero-bootstrap".into()).spawn(|| {
            thread::sleep(Duration::from_millis(500));
            bootstrap();
        });
    }
    1
}

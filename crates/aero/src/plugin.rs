//! DLL entry, bootstrap (log, fault handler, kill switches), editor hook and overlay start.

use std::ffi::c_void;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::thread;

use windows_sys::Win32::Foundation::HINSTANCE;
use windows_sys::Win32::System::LibraryLoader::{DisableThreadLibraryCalls, GetModuleFileNameW, GetModuleHandleW};

use crate::logging::{log_line, set_log_path};
use crate::memory::self_module_path;
use crate::offsets::GAME_BUILD_LABEL;

/// Compile-time stamp of THIS binary, logged at bootstrap: tells which DLL the game loaded.
const PLUGIN_BUILD_LABEL: &str = env!("AERO_BUILD_STAMP");
use crate::state::{MODULE_BASE, SELF_MODULE};

/// Kill switch: `<game>/<marker>` file or `<env>=1`, so a crash can be bisected without a rebuild.
fn disabled(dir: &Path, marker: &str, env: &str) -> bool {
    std::env::var(env).as_deref() == Ok("1") || dir.join(marker).exists()
}

static TOOLBAR_OFF: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub(crate) fn toolbar_disabled() -> bool {
    TOOLBAR_OFF.load(Ordering::SeqCst)
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
        let message =
            info.payload().downcast_ref::<&str>().map(|s| s.to_string()).or_else(|| info.payload().downcast_ref::<String>().cloned()).unwrap_or_default();
        let at = info.location().map(|l| format!("{}:{}", l.file(), l.line())).unwrap_or_default();
        log_line(&format!("PANIC: {} at {at} stage={}", message.replace('\n', " "), crate::fault::stage_name(crate::fault::STAGE.load(Ordering::Relaxed))));
        previous(info);
    }));
}

fn bootstrap() {
    let booted = stormworks_core::unload::Booted;
    let Some(dll_path) = self_module_path() else {
        return;
    };
    let plugin_dir = dll_path.parent().unwrap_or_else(|| Path::new(".")).to_path_buf();
    let log_dir = exe_dir().unwrap_or_else(|| plugin_dir.clone()).join("UserData").join("logs");
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

    // The other plugins wait until our hooks are in (crate::hook).
    let installing = crate::hook::Exclusive::take();
    if disabled(&plugin_dir, "aero_disable_hook", "AERO_NO_HOOK") {
        log_line("editor hook disabled by request (aero_disable_hook / AERO_NO_HOOK)");
    } else {
        TOOLBAR_OFF.store(disabled(&plugin_dir, "aero_disable_toolbar", "AERO_NO_TOOLBAR"), Ordering::SeqCst);
        match unsafe { crate::hooks::install_hooks() } {
            Ok(()) => log_line("hooks installed"),
            Err(e) => log_line(&e),
        }
        unsafe { crate::markhooks::install(b"aero\0", None) };
    }

    {
        let game = exe_dir().unwrap_or_else(|| plugin_dir.clone());
        let appdata = std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Stormworks")).unwrap_or_default();
        let _ = crate::flight::PATHS.set((game, appdata));
        crate::settings::load();
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
        stormworks_core::unload::OVERLAY_UP.store(true, Ordering::SeqCst);
    } else {
        let game = exe_dir().unwrap_or_else(|| plugin_dir.clone());
        let appdata = std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Stormworks")).unwrap_or_default();
        log_line(&format!("game dir {} appdata {}", game.display(), appdata.display()));
        start_overlay(game, appdata);
    }
    drop(installing);
    drop(booted);

    // Verbose builds: per-10-second counters, so "never called" and "rejected" are distinguishable;
    // the hang watch looks every second.
    #[cfg(feature = "logging")]
    let mut watch = crate::hang::Watch::new();
    #[cfg(feature = "logging")]
    for second in 1u64.. {
        thread::sleep(std::time::Duration::from_secs(1));
        if stormworks_core::unload::STOP.load(Ordering::Relaxed) {
            break;
        }
        watch.check();
        if second % 10 != 0 {
            continue;
        }
        log_line(&format!(
            "heartbeat: {} MB, {} frames | editor ticks={} active={} snapshots={} rejects={} | flight on={} ticks={} vehicles={} models={} applied={}",
            crate::hang::rss_mb(),
            crate::hang::FRAMES.load(Ordering::Relaxed),
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
    let failed = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(apply)) {
        Ok(Ok(())) => return log_line("overlay hooks applied"),
        Ok(Err(e)) => format!("overlay failed: {e}"),
        Err(_) => "overlay setup panicked".into(),
    };
    log_line(&failed);
    // No overlay frame will come: ready for the mod loader without one.
    stormworks_core::unload::OVERLAY_UP.store(true, Ordering::SeqCst);
}
/// Mod loader: hooks in (and the overlay's first frame drawn), so the next plugin may load.
#[no_mangle]
pub extern "C" fn swml_ready() -> bool {
    stormworks_core::unload::ready()
}

/// Mod loader: out of the game, so the DLL can be freed (and a new build loaded).
#[no_mangle]
pub extern "C" fn swml_unload() -> bool {
    stormworks_core::unload::unload(|| unsafe { crate::hooks::restore_jet_slot() }, crate::flight::restore_air_constants)
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
            bootstrap();
        });
    }
    1
}

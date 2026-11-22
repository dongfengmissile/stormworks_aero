//! DLL entry and bootstrap (log, fault handler, kill switches, hooks, overlay).

use std::ffi::c_void;
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::OnceLock;
use std::thread;
use std::time::Duration;

use stormworks_core::plugin::{appdata, disabled, exe_dir};
use windows_sys::Win32::Foundation::HINSTANCE;
use windows_sys::Win32::System::LibraryLoader::DisableThreadLibraryCalls;

use crate::logging::log_line;
use crate::offsets::GAME_BUILD_LABEL;
use crate::state::SELF_MODULE;

/// Compile-time stamp of THIS binary, logged at bootstrap: tells which DLL the game loaded.
const PLUGIN_BUILD_LABEL: &str = env!("MECH_BUILD_STAMP");

/// The game's folder and `%APPDATA%/Stormworks`.
pub(crate) static PATHS: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();

fn bootstrap() {
    let Some(dir) = stormworks_core::plugin::start("mechanisms", GAME_BUILD_LABEL, PLUGIN_BUILD_LABEL) else { return };
    let game = exe_dir().unwrap_or_else(|| dir.clone());
    let _ = PATHS.set((game, appdata()));
    // The other plugins wait until our hooks are in (crate::hook).
    let installing = crate::hook::Exclusive::take();
    if disabled(&dir, "mechanisms_disable_hook", "MECH_NO_HOOK") {
        log_line("hooks disabled by request (mechanisms_disable_hook / MECH_NO_HOOK)");
    } else {
        match unsafe { crate::hooks::install_hooks() } {
            Ok(()) => log_line("hooks installed"),
            Err(e) => log_line(&format!("{e}; the plugin stays off")),
        }
        // Joint settings in the `mech` attribute; older files have them in the aero plugin's.
        unsafe { crate::markhooks::install(b"mech\0", Some((b"aero\0", crate::marks::from_aero))) };
        if disabled(&dir, "mechanisms_disable_paint", "MECH_NO_PAINT") {
            log_line("block paint disabled by request (mechanisms_disable_paint)");
        } else {
            unsafe { crate::paint::install() };
        }
    }
    if disabled(&dir, "mechanisms_disable_overlay", "MECH_NO_OVERLAY") {
        log_line("overlay disabled by request (mechanisms_disable_overlay / MECH_NO_OVERLAY)");
    } else {
        start_overlay();
    }
    drop(installing);

    // Verbose builds: the hang watch looks every second, a heartbeat every ten.
    #[cfg(feature = "logging")]
    {
        let mut watch = crate::hang::Watch::new();
        for second in 1u64.. {
            thread::sleep(Duration::from_secs(1));
            watch.check();
            if second % 10 == 0 {
                log_line(&format!(
                    "heartbeat: {} MB, {} frames | editor ticks={} active={} snapshots={} mode={}",
                    crate::hang::rss_mb(),
                    crate::hang::FRAMES.load(Ordering::Relaxed),
                    crate::editor::TICKS.load(Ordering::Relaxed),
                    crate::editor::editor_active(),
                    crate::editor::SNAPSHOTS.load(Ordering::Relaxed),
                    crate::toolbar::mech_active(),
                ));
            }
        }
    }
}

fn start_overlay() {
    use hudhook::hooks::opengl3::ImguiOpenGl3Hooks;
    use hudhook::{windows, Hudhook};
    let module_raw = SELF_MODULE.load(Ordering::SeqCst);
    let apply = move || -> Result<(), String> {
        // hudhook hooks wglSwapBuffers and the window procedure with its own MinHook instance; it
        // chains with the aero plugin and swQolSuite doing the same.
        Hudhook::builder()
            .with::<ImguiOpenGl3Hooks>(crate::overlay::Overlay::default())
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
        let _ = thread::Builder::new().name("stormworks-mechanisms-bootstrap".into()).spawn(|| {
            thread::sleep(Duration::from_millis(500));
            bootstrap();
        });
    }
    1
}

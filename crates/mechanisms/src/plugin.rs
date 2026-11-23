//! DLL entry and bootstrap (log, fault handler, kill switches, hooks, overlay).

use std::ffi::{c_char, c_void, CString};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::OnceLock;
use std::thread;

use stormworks_core::plugin::{appdata, disabled, exe_dir};
use stormworks_core::swml::{self, Bind, Setting, Value};
use windows_sys::Win32::Foundation::HINSTANCE;
use windows_sys::Win32::System::LibraryLoader::DisableThreadLibraryCalls;

use crate::logging::log_line;
use crate::offsets::GAME_BUILD_LABEL;
use crate::state::SELF_MODULE;

/// Compile-time stamp of THIS binary, logged at bootstrap: tells which DLL the game loaded.
const PLUGIN_BUILD_LABEL: &str = env!("MECH_BUILD_STAMP");

pub(crate) static MECH_KEY: Bind = Bind::new("Shift");
pub(crate) static PLAY_KEY: Bind = Bind::new("Space");
pub(crate) static DELETE_KEY: Bind = Bind::new("Delete");
pub(crate) static CANCEL_KEY: Bind = Bind::new("Escape");

/// Every key, for the proxy mod loader's manager.
fn settings() -> [Setting; 4] {
    [
        Setting {
            id: "mech_modifier",
            label: "Fix several sub-grids (mechanisms click)",
            tip: "Hold and click to fix more than one sub-grid.",
            value: Value::Hold(&MECH_KEY),
        },
        Setting { id: "sequence_play", label: "Play sequence (sequence window)", tip: "Play the sequence.", value: Value::Key(&PLAY_KEY) },
        Setting { id: "sequence_delete_move", label: "Delete move (sequence window)", tip: "Delete the selected move.", value: Value::Key(&DELETE_KEY) },
        Setting { id: "dialog_cancel", label: "Cancel dialog (sequence window)", tip: "Close the open dialog.", value: Value::Key(&CANCEL_KEY) },
    ]
}

#[no_mangle]
pub extern "C" fn swml_describe() -> *const c_char {
    static TEXT: OnceLock<CString> = OnceLock::new();
    TEXT.get_or_init(|| CString::new(swml::describe(&settings())).unwrap_or_default()).as_ptr()
}

/// # Safety
/// NUL-terminated strings, from the mod loader.
#[no_mangle]
pub unsafe extern "C" fn swml_set(id: *const c_char, value: *const c_char) {
    swml::set(&settings(), id, value)
}

/// The game's folder and `%APPDATA%/Stormworks`.
pub(crate) static PATHS: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();

fn bootstrap() {
    let booted = stormworks_core::unload::Booted;
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
        #[cfg(feature = "logging")]
        {
            crate::contacts::set_path(PATHS.get().map_or(&dir, |p| &p.0).join("UserData").join("logs").join("mechanisms_contacts.log"));
            unsafe { crate::hooks::install_contact_hook() };
        }
        if disabled(&dir, "mechanisms_disable_paint", "MECH_NO_PAINT") {
            log_line("block paint disabled by request (mechanisms_disable_paint)");
        } else {
            unsafe { crate::paint::install() };
        }
    }
    if disabled(&dir, "mechanisms_disable_overlay", "MECH_NO_OVERLAY") {
        log_line("overlay disabled by request (mechanisms_disable_overlay / MECH_NO_OVERLAY)");
        stormworks_core::unload::OVERLAY_UP.store(true, Ordering::SeqCst);
    } else {
        start_overlay();
    }
    drop(installing);
    drop(booted);

    // Verbose builds: the hang watch looks every second, a heartbeat every ten.
    #[cfg(feature = "logging")]
    {
        let mut watch = crate::hang::Watch::new();
        for second in 1u64.. {
            thread::sleep(std::time::Duration::from_secs(1));
            if stormworks_core::unload::STOP.load(Ordering::Relaxed) {
                break;
            }
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
    stormworks_core::unload::unload(|| {}, || {})
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
            bootstrap();
        });
    }
    1
}

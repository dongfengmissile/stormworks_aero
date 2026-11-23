//! What every plugin DLL does at load: its log, the fault handler, the game's base, kill switches.

use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;

use windows_sys::Win32::System::LibraryLoader::{GetModuleFileNameW, GetModuleHandleW};

use crate::logging::{log_line, set_log_path};

/// Kill switch: `<Mods>/<marker>` file or `<env>=1`, so a crash can be bisected without a rebuild.
pub fn disabled(dir: &Path, marker: &str, env: &str) -> bool {
    std::env::var(env).as_deref() == Ok("1") || dir.join(marker).exists()
}

/// The game's folder (where stormworks64.exe is).
pub fn exe_dir() -> Option<PathBuf> {
    let mut buf = vec![0u16; 1024];
    let len = unsafe { GetModuleFileNameW(std::ptr::null_mut(), buf.as_mut_ptr(), buf.len() as u32) } as usize;
    (len > 0 && len < buf.len()).then(|| PathBuf::from(String::from_utf16_lossy(&buf[..len])).parent().map(Path::to_path_buf))?
}

/// `%APPDATA%/Stormworks` (saved vehicles, settings).
pub fn appdata() -> PathBuf {
    std::env::var_os("APPDATA").map(|a| PathBuf::from(a).join("Stormworks")).unwrap_or_default()
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

/// Opens `<game>/UserData/logs/<name>.log` (the last run's kept as `<name>.prev.log`), installs the
/// fault handler and records the game's base. Returns the DLL's folder; None if the game's base is
/// unknown (then nothing may be hooked).
pub fn start(name: &str, game_build: &str, plugin_build: &str) -> Option<PathBuf> {
    let dll_path = crate::memory::self_module_path()?;
    let plugin_dir = dll_path.parent().unwrap_or_else(|| Path::new(".")).to_path_buf();
    let log_dir = exe_dir().unwrap_or_else(|| plugin_dir.clone()).join("UserData").join("logs");
    let _ = std::fs::create_dir_all(&log_dir);
    let log_path = log_dir.join(format!("{name}.log"));
    if log_path.exists() {
        let _ = std::fs::rename(&log_path, log_dir.join(format!("{name}.prev.log")));
    }
    set_log_path(log_path.clone());
    #[cfg(feature = "logging")]
    install_panic_hook();
    crate::fault::install(&log_path);
    log_line(&format!("{name} bootstrap start build={game_build} plugin={plugin_build} dll={}", dll_path.display()));
    let base = unsafe { GetModuleHandleW(std::ptr::null()) } as usize;
    if base == 0 {
        log_line("could not resolve the main module base");
        return None;
    }
    crate::state::MODULE_BASE.store(base, Ordering::SeqCst);
    Some(plugin_dir)
}

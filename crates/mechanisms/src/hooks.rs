//! The plugin's game hooks, each gated on its recorded prologue (skip + log on mismatch); the
//! ones the aero plugin also places chain through `crate::hook`.

use std::ffi::c_void;
use std::sync::atomic::Ordering;
use std::sync::OnceLock;

use crate::hook::install;
use crate::logging::log_line;
use crate::offsets::*;
use crate::state::MODULE_BASE;

type EditorUpdate = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize);
static ORIGINAL_EDITOR_UPDATE: OnceLock<EditorUpdate> = OnceLock::new();

unsafe extern "system" fn editor_update_detour(a1: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize) {
    if let Some(original) = ORIGINAL_EDITOR_UPDATE.get() {
        original(a1, a2, a3, a4, a5, a6);
    }
    // A panic cannot unwind out of an extern "system" fn: contain it here.
    let _ = std::panic::catch_unwind(|| crate::editor::on_tick(a1));
    let _ = std::panic::catch_unwind(|| crate::sound::on_editor_tick(a1));
    let _ = std::panic::catch_unwind(|| crate::toolbar::panel_rows_tick(a1));
    let _ = std::panic::catch_unwind(|| stormworks_core::toolbar::on_tick(a1));
}

#[cfg(feature = "logging")]
type PostStepSync = unsafe extern "system" fn(usize);
#[cfg(feature = "logging")]
static ORIGINAL_POST_STEP_SYNC: OnceLock<PostStepSync> = OnceLock::new();

#[cfg(feature = "logging")]
unsafe extern "system" fn post_step_sync_detour(physics_scene: usize) {
    if let Some(original) = ORIGINAL_POST_STEP_SYNC.get() {
        original(physics_scene);
    }
    let _ = std::panic::catch_unwind(|| crate::contacts::on_post_step_sync(physics_scene));
}

/// Logging builds, in game: the contact log (`contacts`). Call under `crate::hook::Exclusive`.
#[cfg(feature = "logging")]
pub(crate) unsafe fn install_contact_hook() {
    let at = MODULE_BASE.load(Ordering::SeqCst) + RVA_POST_STEP_SYNC;
    match install("physics_scene_post_step_sync", at, &POST_STEP_SYNC_PROLOGUE, post_step_sync_detour as *mut c_void, &ORIGINAL_POST_STEP_SYNC) {
        Ok(()) => log_line("contact log on: UserData/logs/mechanisms_contacts.log"),
        Err(e) => log_line(&format!("{e}: no contact log")),
    }
}

/// The editor tick, the toolbar, the pose in the game's view and the joint panel rows. Call under
/// `crate::hook::Exclusive`.
pub(crate) unsafe fn install_hooks() -> Result<(), String> {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    install("editor update", base + RVA_EDITOR_UPDATE, &EDITOR_UPDATE_PROLOGUE, editor_update_detour as *mut c_void, &ORIGINAL_EDITOR_UPDATE)?;
    log_line(&format!("editor update hooked at {:#x}", base + RVA_EDITOR_UPDATE));
    crate::toolbar::install();
    // Workbench pose drawn in the game's view (sub-grids at their joints' workbench angles).
    if let Err(e) = install("editor scene draw", base + RVA_EDITOR_RENDER_SCENE, &EDITOR_RENDER_SCENE_PROLOGUE, crate::posing::render_scene_detour as *mut c_void, &crate::posing::ORIGINAL_RENDER_SCENE) {
        log_line(&format!("{e}: sub-grids are not moved in the workbench view"));
    }
    if let Err(e) = install("joint config panel", base + crate::toolbar::RVA_JOINT_PANEL, &crate::toolbar::JOINT_PANEL_PROLOGUE, crate::toolbar::joint_panel_detour as *mut c_void, &crate::toolbar::ORIGINAL_JOINT_PANEL) {
        log_line(&format!("{e}: no Workbench Angle row"));
    }
    Ok(())
}

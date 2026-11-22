//! The workbench vehicle (`stormworks_core::editor`) plus what this plugin does on each editor tick.

pub use stormworks_core::editor::*;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use crate::memory::{read_u32_at, read_usize_at, RegionCacheScope};
use crate::offsets::*;

/// Called after every editor tick, on the game thread.
pub(crate) unsafe fn on_tick(editor: usize) {
    stormworks_core::editor::on_tick(editor);
    crate::sound::on_editor_tick(editor);
    // Once a second, panel open or not: every body's first part carries the vehicle settings.
    if TICKS.load(Ordering::Relaxed) % 60 == 1 {
        let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
        let _cache = RegionCacheScope::new();
        if let Some((raw, ptrs)) = read_raw(editor) {
            let parts: Vec<(usize, u32)> = ptrs.iter().zip(&raw).map(|(&p, r)| (p, r.3)).collect();
            crate::marks::spread_settings(&parts);
        }
    }
}

// --- Workbench pose in the game's view (knowledge/rendering.md "Workbench body draw matrix"):
// while the Aero wings mode is on, each posed sub-grid's draw matrix (`body+0x2C8`) is swapped in
// for the editor's scene draw and put back after it, so editing and picking keep the build pose.

/// The posing joints (`model::joint_links`), the vehicle's body count and each joint's target
/// setting (`pose::joint_settings`): what the workbench shows while a plugin mode is on.
pub(crate) static POSE_TARGET: Mutex<Option<(Arc<Vec<crate::model::JointLink>>, usize, Vec<f64>)>> = Mutex::new(None);
static POSE_ANIM: std::sync::LazyLock<Mutex<crate::pose::PoseAnim>> = std::sync::LazyLock::new(Mutex::default);
static EPOCH: std::sync::LazyLock<std::time::Instant> = std::sync::LazyLock::new(std::time::Instant::now);

pub(crate) fn publish_pose_target(joints: &[crate::model::JointLink], bodies: usize, settings: &[f64]) {
    POSE_SNAP.store(false, Ordering::Relaxed);
    *POSE_TARGET.lock().unwrap_or_else(|p| p.into_inner()) = Some((Arc::new(joints.to_vec()), bodies, settings.to_vec()));
}

/// The pose shown at once, not eased in: a Mechanisms sequence frame.
static POSE_SNAP: AtomicBool = AtomicBool::new(false);
pub(crate) fn publish_pose_now(joints: &[crate::model::JointLink], bodies: usize, settings: &[f64]) {
    *POSE_TARGET.lock().unwrap_or_else(|p| p.into_inner()) = Some((Arc::new(joints.to_vec()), bodies, settings.to_vec()));
    POSE_SNAP.store(true, Ordering::Relaxed);
}

pub(crate) type RenderScene = unsafe extern "system" fn(usize, usize, usize, usize, usize);
pub(crate) static ORIGINAL_RENDER_SCENE: std::sync::OnceLock<RenderScene> = std::sync::OnceLock::new();

pub(crate) unsafe extern "system" fn render_scene_detour(editor: usize, a2: usize, a3: usize, a4: usize, a5: usize) {
    let Some(original) = ORIGINAL_RENDER_SCENE.get() else { return };
    let saved = std::panic::catch_unwind(|| pose_bodies(editor)).unwrap_or_default();
    original(editor, a2, a3, a4, a5);
    for (at, m) in saved {
        std::ptr::copy_nonoverlapping(m.as_ptr(), at as *mut u32, 16);
    }
}

/// Writes the posed draw matrices; returns each body's matrix address and old contents.
unsafe fn pose_bodies(editor: usize) -> Vec<(usize, [u32; 16])> {
    let Some((links, n, settings)) = POSE_TARGET.lock().unwrap_or_else(|p| p.into_inner()).clone() else { return Vec::new() };
    if !ptr_ok(editor + OFF_EDITOR_VEHICLE, 8) {
        return Vec::new();
    }
    let Some(bodies) = vehicle_bodies(read_usize_at(editor + OFF_EDITOR_VEHICLE)) else { return Vec::new() };
    if bodies.len() != n {
        return Vec::new(); // the vehicle changed since the pose was worked out
    }
    let on = (crate::toolbar::active() && crate::toolbar::pose_shown()) || crate::toolbar::mech_active();
    let now = EPOCH.elapsed().as_secs_f64();
    let Some(poses) = POSE_ANIM.lock().unwrap_or_else(|p| p.into_inner()).poses(&links, n, &settings, on, POSE_SNAP.load(Ordering::Relaxed), now) else { return Vec::new() };
    let mut saved = Vec::new();
    for (&body, t) in bodies.iter().zip(&poses) {
        if *t == crate::pose::Pose::IDENTITY || !ptr_ok(body, OFF_BODY_ORIGIN + 12) || !ptr_ok(body, OFF_BODY_MATRIX + 64) {
            continue;
        }
        let at = body + OFF_BODY_MATRIX;
        let old: [u32; 16] = std::array::from_fn(|k| read_u32_at(at + 4 * k));
        let m: [f64; 16] = old.map(|v| f32::from_bits(v) as f64);
        let o: [f64; 3] = std::array::from_fn(|k| f32::from_bits(read_u32_at(body + OFF_BODY_ORIGIN + 4 * k)) as f64);
        let posed = crate::pose::draw_matrix(&crate::pose::grid_move(t), o, &m);
        if !posed.iter().all(|v| v.is_finite()) {
            continue;
        }
        for (k, v) in posed.iter().enumerate() {
            *((at + 4 * k) as *mut f32) = *v as f32;
        }
        saved.push((at, old));
    }
    saved
}

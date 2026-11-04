//! MinHook detours, each gated on its recorded prologue (skip + log on mismatch).

use std::ffi::c_void;
use std::sync::atomic::Ordering;
use std::sync::OnceLock;

use minhook::MinHook;

use crate::logging::log_line;
use crate::memory::bytes_match;
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
}

type PostStepSync = unsafe extern "system" fn(usize);
static ORIGINAL_POST_STEP_SYNC: OnceLock<PostStepSync> = OnceLock::new();

unsafe extern "system" fn post_step_sync_detour(physics_scene: usize) {
    if let Some(original) = ORIGINAL_POST_STEP_SYNC.get() {
        original(physics_scene);
    }
    let _ = std::panic::catch_unwind(|| crate::flight::on_post_step_sync(physics_scene));
}

type SetDamping = unsafe extern "system" fn(usize, usize, f32, f32);
static ORIGINAL_SET_DAMPING: OnceLock<SetDamping> = OnceLock::new();

/// Bullet velocity damping is the game's other air loss: none while our forces fly.
unsafe extern "system" fn set_damping_detour(scene: usize, obj: usize, linear: f32, angular: f32) {
    let Some(original) = ORIGINAL_SET_DAMPING.get() else { return };
    if crate::flight::ENABLED.load(Ordering::Relaxed) {
        original(scene, obj, 0.0, 0.0);
    } else {
        original(scene, obj, linear, angular);
    }
}

type SimRender = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize) -> usize;
static ORIGINAL_SIM_RENDER: OnceLock<SimRender> = OnceLock::new();

/// Records the simulation state (owner of the main camera) for the force arrows.
unsafe extern "system" fn sim_render_detour(g: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize) -> usize {
    crate::flight::note_sim_state(g);
    let ts = crate::flight::frame_build_ts(g);
    crate::drawcap::begin(g);
    crate::paint::main_view(true);
    let r = match ORIGINAL_SIM_RENDER.get() {
        Some(original) => original(g, a2, a3, a4, a5, a6),
        None => 0,
    };
    crate::paint::main_view(false);
    crate::drawcap::end();
    crate::flight::note_sim_frame(ts);
    r
}

/// Main-camera hook for the in-flight force arrows; failure only loses the arrows.
pub(crate) unsafe fn install_camera_hook() -> Result<(), String> {
    let at = MODULE_BASE.load(Ordering::SeqCst) + RVA_SIM_RENDER_SLOT7;
    if !bytes_match(at, &SIM_RENDER_SLOT7_PROLOGUE) {
        return Err(format!("sim render slot 7 prologue mismatch at {at:#x}; in-flight force arrows disabled"));
    }
    let o = MinHook::create_hook(at as *mut c_void, sim_render_detour as *mut c_void).map_err(|e| format!("create camera hook: {e:?}"))?;
    let _ = ORIGINAL_SIM_RENDER.set(std::mem::transmute::<*mut c_void, SimRender>(o));
    MinHook::enable_hook(at as *mut c_void).map_err(|e| format!("enable camera hook: {e:?}"))?;
    let interp = MODULE_BASE.load(Ordering::SeqCst) + RVA_MATRIX_INTERP;
    if bytes_match(interp, &MATRIX_INTERP_PROLOGUE) {
        // Detour it to capture the exact per-body draw matrices; our own calls go to the trampoline.
        match MinHook::create_hook(interp as *mut c_void, crate::drawcap::interp_detour as *mut c_void) {
            Ok(o) => {
                let f = std::mem::transmute::<*mut c_void, crate::drawcap::MatrixInterp>(o);
                let _ = crate::drawcap::ORIGINAL.set(f);
                let _ = crate::flight::MATRIX_INTERP.set(f);
                if let Err(e) = MinHook::enable_hook(interp as *mut c_void) {
                    log_line(&format!("enable draw capture hook: {e:?}; arrows use our interpolation"));
                }
            }
            Err(e) => {
                log_line(&format!("create draw capture hook: {e:?}; arrows use our interpolation"));
                let _ = crate::flight::MATRIX_INTERP.set(std::mem::transmute::<usize, crate::flight::MatrixInterp>(interp));
            }
        }
    } else {
        log_line(&format!("matrix_interp_prev_cur prologue mismatch at {interp:#x}; arrows use a linear blend"));
    }
    install_paint_hook();
    log_line(&format!("camera hook installed ({at:#x})"));
    Ok(())
}

/// Stalled-voxel paint in the game render; failure only loses the red blocks.
unsafe fn install_paint_hook() {
    let at = MODULE_BASE.load(Ordering::SeqCst) + RVA_CHUNK_DRAW;
    if crate::plugin::paint_disabled() {
        log_line("stall paint disabled by request (aero_disable_paint)");
        return;
    }
    if !bytes_match(at, &CHUNK_DRAW_PROLOGUE) {
        log_line(&format!("opaque_chunk_draw_item prologue mismatch at {at:#x}; stalled blocks not painted"));
        return;
    }
    match MinHook::create_hook(at as *mut c_void, crate::paint::chunk_draw_detour as *mut c_void) {
        Ok(o) => {
            let _ = crate::paint::ORIGINAL.set(std::mem::transmute::<*mut c_void, crate::paint::ChunkDraw>(o));
            match MinHook::enable_hook(at as *mut c_void) {
                Ok(()) => log_line(&format!("stall paint hook installed ({at:#x})")),
                Err(e) => log_line(&format!("enable stall paint hook: {e:?}")),
            }
        }
        Err(e) => log_line(&format!("create stall paint hook: {e:?}")),
    }
}

type QueueForce = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize) -> usize;
static ORIGINAL_THRESHOLD: OnceLock<QueueForce> = OnceLock::new();
static ORIGINAL_CLAMPED: OnceLock<QueueForce> = OnceLock::new();
/// Caller of the current queue-force call, stored by the naked shims (update thread only, so one slot is enough).
static RET_THRESHOLD: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static RET_CLAMPED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

// At entry [rsp] is the return address; rax is volatile and carries no argument.
#[unsafe(naked)]
unsafe extern "system" fn threshold_shim() {
    core::arch::naked_asm!("mov rax, [rsp]", "mov [rip + {ret}], rax", "jmp {detour}", ret = sym RET_THRESHOLD, detour = sym threshold_detour);
}
#[unsafe(naked)]
unsafe extern "system" fn clamped_shim() {
    core::arch::naked_asm!("mov rax, [rsp]", "mov [rip + {ret}], rax", "jmp {detour}", ret = sym RET_CLAMPED, detour = sym clamped_detour);
}

/// Forces queued from a known thrust call site (jets, rockets, propellers/rotors, RCS, water jets) are thrust (world frame) on `phys_obj`'s rigid body.
unsafe fn note_thrust(ret: usize, returns: &[(usize, u8)], phys_obj: usize, force: usize, rel: usize, max: usize, scale: usize) {
    let base = MODULE_BASE.load(Ordering::Relaxed);
    if !crate::flight::ENABLED.load(Ordering::Relaxed) || force == 0 {
        return;
    }
    let Some(&(_, source)) = returns.iter().find(|(r, _)| base + r == ret) else { return };
    if !crate::editor::ptr_ok(phys_obj + OFF_OBJ_RIGID_BODY, 8) {
        return;
    }
    let f = (force as *const [f32; 3]).read_unaligned();
    let rb = crate::memory::read_usize_at(phys_obj + OFF_OBJ_RIGID_BODY);
    // Point of action, from the centre of mass (world axes); unverified for the clamped wrapper, so only if sane.
    let rel = (rel != 0 && crate::memory::readable(rel, 12))
        .then(|| (rel as *const [f32; 3]).read_unaligned().map(|x| x as f64))
        .filter(|r| r.iter().all(|x| x.is_finite() && x.abs() < 200.0));
    crate::flight::note_engine_force(rb, f.map(|x| x as f64), source, rel, f32::from_bits(max as u32), f32::from_bits(scale as u32));
}

unsafe extern "system" fn threshold_detour(a1: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, a7: usize) -> usize {
    let ret = RET_THRESHOLD.load(Ordering::Relaxed);
    // A jet nozzle's thrust follows its spool: queue a reshaped copy of the force instead.
    let shaped: [f32; 3];
    let mut force = a3;
    if crate::flight::ENABLED.load(Ordering::Relaxed) && a3 != 0 && ret == MODULE_BASE.load(Ordering::Relaxed) + JET_NOZZLE_RETURN {
        let f = (a3 as *const [f32; 3]).read_unaligned();
        if let Some(g) = crate::flight::jet_sample(a4.wrapping_sub(JET_THIS_FROM_ARG4), a2, f, a5 as *const f32) {
            shaped = g;
            force = shaped.as_ptr() as usize;
        }
    }
    note_thrust(ret, &THRESHOLD_THRUST_RETURNS, a2, force, a5, a6, a7);
    ORIGINAL_THRESHOLD.get().map_or(0, |o| o(a1, a2, force, a4, a5, a6, a7))
}

unsafe extern "system" fn clamped_detour(a1: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, a7: usize) -> usize {
    note_thrust(RET_CLAMPED.load(Ordering::Relaxed), &CLAMPED_THRUST_RETURNS, a2, a3, a5, a6, a7);
    ORIGINAL_CLAMPED.get().map_or(0, |o| o(a1, a2, a3, a4, a5, a6, a7))
}

/// Engine thrust capture; failure leaves the applyForce sum / estimate in F9.
unsafe fn install_thrust_hooks() {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    for (name, rva, prologue, shim, orig) in [
        ("apply_force_threshold", RVA_APPLY_FORCE_THRESHOLD, &APPLY_FORCE_THRESHOLD_PROLOGUE[..], threshold_shim as *mut c_void, &ORIGINAL_THRESHOLD),
        ("apply_force_clamped", RVA_APPLY_FORCE_CLAMPED, &APPLY_FORCE_CLAMPED_PROLOGUE[..], clamped_shim as *mut c_void, &ORIGINAL_CLAMPED),
    ] {
        let at = base + rva;
        if !bytes_match(at, prologue) {
            log_line(&format!("{name} prologue mismatch at {at:#x}; engine thrust not captured from it"));
            continue;
        }
        match MinHook::create_hook(at as *mut c_void, shim) {
            Ok(o) => {
                let _ = orig.set(std::mem::transmute::<*mut c_void, QueueForce>(o));
                if let Err(e) = MinHook::enable_hook(at as *mut c_void) {
                    log_line(&format!("enable {name} hook: {e:?}"));
                }
            }
            Err(e) => log_line(&format!("create {name} hook: {e:?}")),
        }
    }
}

/// In-game flight forces: post-step-sync tick, damping override, applyForce. All or nothing.
pub(crate) unsafe fn install_flight_hooks() -> Result<(), String> {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    let (sync, damping, apply) = (base + RVA_POST_STEP_SYNC, base + RVA_SET_DAMPING, base + RVA_APPLY_FORCE);
    for (name, at, ok) in [
        ("physics_scene_post_step_sync", sync, bytes_match(sync, &POST_STEP_SYNC_PROLOGUE)),
        ("physics_scene_set_damping", damping, bytes_match(damping, &SET_DAMPING_PROLOGUE)),
        ("btRigidBody::applyForce", apply, bytes_match(apply, &APPLY_FORCE_PROLOGUE)),
    ] {
        if !ok {
            return Err(format!("{name} prologue mismatch at {at:#x} (hooked by another plugin, or not build {GAME_BUILD_LABEL}); in-game flight forces disabled"));
        }
    }
    // Not detoured: its callers rely on registers it leaves untouched (rax at 0x1A1605, r9 at 0x1A171B),
    // which a detour can't guarantee. Thrust comes from the queue-force wrappers instead.
    let _ = crate::flight::APPLY_FORCE.set(std::mem::transmute::<usize, unsafe extern "system" fn(usize, *const f64, *const f64)>(apply));
    let o = MinHook::create_hook(damping as *mut c_void, set_damping_detour as *mut c_void).map_err(|e| format!("create damping hook: {e:?}"))?;
    let _ = ORIGINAL_SET_DAMPING.set(std::mem::transmute::<*mut c_void, SetDamping>(o));
    let o = MinHook::create_hook(sync as *mut c_void, post_step_sync_detour as *mut c_void).map_err(|e| format!("create post-step hook: {e:?}"))?;
    let _ = ORIGINAL_POST_STEP_SYNC.set(std::mem::transmute::<*mut c_void, PostStepSync>(o));
    MinHook::enable_hook(damping as *mut c_void).map_err(|e| format!("enable damping hook: {e:?}"))?;
    MinHook::enable_hook(sync as *mut c_void).map_err(|e| format!("enable post-step hook: {e:?}"))?;
    install_thrust_hooks();
    // Ground effect: the game's raycast and water height (called, not hooked).
    let (ray, water) = (base + RVA_RAYCAST_CLOSEST, base + RVA_WATER_HEIGHT);
    if bytes_match(ray, &RAYCAST_CLOSEST_PROLOGUE) && bytes_match(water, &WATER_HEIGHT_PROLOGUE) {
        let _ = crate::flight::RAYCAST.set(std::mem::transmute::<usize, crate::flight::Raycast>(ray));
        let _ = crate::flight::WATER_HEIGHT.set(std::mem::transmute::<usize, crate::flight::WaterHeight>(water));
    } else {
        log_line("raycast / water height prologue mismatch: no ground effect");
    }
    log_line(&format!("flight hooks installed (post-step sync {sync:#x}, damping {damping:#x})"));
    Ok(())
}

pub(crate) unsafe fn install_hooks() -> Result<(), String> {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    let target = base + RVA_EDITOR_UPDATE;
    if !bytes_match(target, &EDITOR_UPDATE_PROLOGUE) {
        return Err(format!(
            "editor update prologue mismatch at {target:#x} (another plugin hooked it, or this game build is not {GAME_BUILD_LABEL}); workbench analysis disabled"
        ));
    }
    let original = MinHook::create_hook(target as *mut c_void, editor_update_detour as *mut c_void)
        .map_err(|e| format!("create editor update hook: {e:?}"))?;
    let _ = ORIGINAL_EDITOR_UPDATE.set(std::mem::transmute::<*mut c_void, EditorUpdate>(original));
    MinHook::enable_hook(target as *mut c_void).map_err(|e| format!("enable editor update hook: {e:?}"))?;
    log_line(&format!("editor update hooked at {target:#x}"));
    Ok(())
}

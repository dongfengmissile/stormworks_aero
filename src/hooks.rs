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
    if TOOLBAR_HOOKED.load(Ordering::Relaxed) {
        let _ = std::panic::catch_unwind(|| crate::toolbar::on_tick(a1));
    }
}

static TOOLBAR_HOOKED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Native "Aero wings" toolbar button: the toolbar init, editor event and mouse button hooks.
/// All three or none (a button that does nothing, or a mode nobody leaves, is worse than none).
unsafe fn install_toolbar_hooks() {
    use crate::toolbar::*;
    let base = MODULE_BASE.load(Ordering::SeqCst);
    let targets: [(&str, usize, &[u8], *mut c_void); 3] = [
        (
            "toolbar init",
            RVA_TOOLBAR_INIT,
            &[
                0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x08, 0xfe, 0xff,
                0xff, 0x48, 0x81, 0xec, 0xc0, 0x02, 0x00, 0x00,
            ],
            toolbar_init_detour as *mut c_void,
        ),
        (
            "editor event",
            RVA_EDITOR_EVENT,
            &[
                0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x78, 0xfc, 0xff, 0xff, 0x48, 0x81,
                0xec, 0x48, 0x04, 0x00, 0x00, 0x0f, 0x29, 0x70,
            ],
            editor_event_detour as *mut c_void,
        ),
        (
            "editor mouse button",
            RVA_EDITOR_MOUSE_BUTTON,
            &[
                0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0xb8, 0xfb, 0xff, 0xff, 0x48, 0x81,
                0xec, 0x08, 0x05, 0x00, 0x00, 0x0f, 0x29, 0x70,
            ],
            mouse_button_detour as *mut c_void,
        ),
    ];
    for (name, rva, pro, _) in &targets {
        if !bytes_match(base + rva, pro) {
            log_line(&format!("{name} prologue mismatch at {:#x}; no Aero wings toolbar button", base + rva));
            return;
        }
    }
    let mut origs = [std::ptr::null_mut(); 3];
    for (i, (name, rva, _, detour)) in targets.iter().enumerate() {
        match MinHook::create_hook((base + rva) as *mut c_void, *detour) {
            Ok(o) => origs[i] = o,
            Err(e) => {
                log_line(&format!("create {name} hook: {e:?}; no Aero wings toolbar button"));
                return;
            }
        }
    }
    let _ = ORIGINAL_INIT.set(std::mem::transmute::<*mut c_void, ToolbarInit>(origs[0]));
    let _ = ORIGINAL_EVENT.set(std::mem::transmute::<*mut c_void, EditorEvent>(origs[1]));
    let _ = ORIGINAL_MOUSE.set(std::mem::transmute::<*mut c_void, MouseButton>(origs[2]));
    for (name, rva, _, _) in &targets {
        if let Err(e) = MinHook::enable_hook((base + rva) as *mut c_void) {
            log_line(&format!("enable {name} hook: {e:?}"));
        }
    }
    TOOLBAR_HOOKED.store(true, Ordering::Relaxed);
    log_line("toolbar hooks installed (Aero wings button)");
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
    let Some(original) = ORIGINAL_SET_DAMPING.get() else {
        return;
    };
    if crate::flight::flies_obj(obj) {
        original(scene, obj, 0.0, 0.0);
    } else {
        original(scene, obj, linear, angular);
    }
}

type AirNodeForces = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize);
static ORIGINAL_AIR_NODE: OnceLock<AirNodeForces> = OnceLock::new();

/// The game's hull-face air forces, skipped for the bodies of vehicles the plugin flies (worker threads).
unsafe extern "system" fn air_node_detour(ctx: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize) {
    let body = (ctx as *const usize).read().wrapping_sub(OFF_BODY_AIR_RECORDS);
    let obj = ((body + OFF_BODY_PHYS_OBJ) as *const usize).read();
    if obj != 0 && crate::flight::flies_obj(obj) {
        return;
    }
    if let Some(original) = ORIGINAL_AIR_NODE.get() {
        original(ctx, a2, a3, a4, a5, a6);
    }
}

/// Per-body switch for the game's hull air forces; without it the air constants are zeroed for all.
unsafe fn install_air_node_hook() {
    let at = MODULE_BASE.load(Ordering::SeqCst) + RVA_AIR_NODE_FORCES;
    if !bytes_match(at, &AIR_NODE_FORCES_PROLOGUE) {
        log_line(&format!("fluid_air_node_forces prologue mismatch at {at:#x}: vehicle settings cannot keep the game's hull air forces"));
        return;
    }
    match MinHook::create_hook(at as *mut c_void, air_node_detour as *mut c_void) {
        Ok(o) => {
            let _ = ORIGINAL_AIR_NODE.set(std::mem::transmute::<*mut c_void, AirNodeForces>(o));
            if MinHook::enable_hook(at as *mut c_void).is_ok() {
                crate::flight::AIR_PER_BODY.store(true, Ordering::Relaxed);
                log_line(&format!("hull air forces switched per vehicle at {at:#x}"));
            }
        }
        Err(e) => log_line(&format!("create air node hook: {e:?}")),
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
    install_tint_hooks();
    install_blur_hooks();
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

/// Rotor blur: the three pushes first (they route tagged items), the draw last, all or nothing,
/// since tagged colours reaching an unhooked push would draw wrong.
unsafe fn install_blur_hooks() {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    let mut push = PUSH_PROLOGUE;
    let mut ok = true;
    for (rva, frame) in [(RVA_PUSH_232, 0xF9), (RVA_PUSH_SHADOW_A, 0xF1), (RVA_PUSH_SHADOW_B, 0xE9)] {
        push[28] = frame;
        ok &= bytes_match(base + rva, &push);
    }
    ok &= bytes_match(base + RVA_PUSH_DITHER, &PUSH_DITHER_PROLOGUE) && bytes_match(base + RVA_FORCE_EMITTER_DRAW, &FORCE_EMITTER_DRAW_PROLOGUE);
    if !ok {
        log_line("rotor blur: prologue mismatch, blades drawn sharp");
        return;
    }
    let _ = crate::blur::PUSH_DITHER.set(std::mem::transmute::<usize, _>(base + RVA_PUSH_DITHER));
    let hook = |rva: usize, detour: *mut c_void| -> Option<*mut c_void> {
        let at = (base + rva) as *mut c_void;
        let o = MinHook::create_hook(at, detour).map_err(|e| log_line(&format!("rotor blur: create hook {rva:#x}: {e:?}"))).ok()?;
        MinHook::enable_hook(at).map_err(|e| log_line(&format!("rotor blur: enable hook {rva:#x}: {e:?}"))).ok()?;
        Some(o)
    };
    let Some(o) = hook(RVA_PUSH_232, crate::blur::push232_detour as *mut c_void) else { return };
    let _ = crate::blur::ORIGINAL_232.set(std::mem::transmute::<*mut c_void, crate::blur::Push232>(o));
    for (rva, detour, slot) in [
        (RVA_PUSH_SHADOW_A, crate::blur::shadow_a_detour as *mut c_void, &crate::blur::ORIGINAL_SHADOW_A),
        (RVA_PUSH_SHADOW_B, crate::blur::shadow_b_detour as *mut c_void, &crate::blur::ORIGINAL_SHADOW_B),
    ] {
        let Some(o) = hook(rva, detour) else { return };
        let _ = slot.set(std::mem::transmute::<*mut c_void, crate::blur::PushShadow>(o));
    }
    let Some(o) = hook(RVA_FORCE_EMITTER_DRAW, crate::blur::draw_detour as *mut c_void) else { return };
    let _ = crate::blur::ORIGINAL_DRAW.set(std::mem::transmute::<*mut c_void, crate::blur::ClassDraw>(o));
    log_line("rotor blur hooks installed");
}

/// Tint hooks on the rocket / rudder class draws (missile fins, control surfaces); failure only
/// leaves those meshes unpainted.
unsafe fn install_tint_hooks() {
    if crate::plugin::paint_disabled() {
        return;
    }
    let base = MODULE_BASE.load(Ordering::SeqCst);
    for (name, rva, prologue, detour, orig) in [
        (
            "solid_rocket_draw",
            RVA_SOLID_ROCKET_DRAW,
            &SOLID_ROCKET_DRAW_PROLOGUE[..],
            crate::paint::rocket_draw_detour as *mut c_void,
            &crate::paint::ORIGINAL_ROCKET_DRAW,
        ),
        ("rudder_draw", RVA_RUDDER_DRAW, &RUDDER_DRAW_PROLOGUE[..], crate::paint::rudder_draw_detour as *mut c_void, &crate::paint::ORIGINAL_RUDDER_DRAW),
    ] {
        let at = base + rva;
        if !bytes_match(at, prologue) {
            log_line(&format!("{name} prologue mismatch at {at:#x}; its moving mesh is not painted"));
            continue;
        }
        match MinHook::create_hook(at as *mut c_void, detour) {
            Ok(o) => {
                let _ = orig.set(std::mem::transmute::<*mut c_void, crate::paint::ClassDraw>(o));
                if let Err(e) = MinHook::enable_hook(at as *mut c_void) {
                    log_line(&format!("enable {name} hook: {e:?}"));
                }
            }
            Err(e) => log_line(&format!("create {name} hook: {e:?}")),
        }
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
    if force == 0 || !crate::flight::flies_obj(phys_obj) {
        return;
    }
    let Some(&(_, source)) = returns.iter().find(|(r, _)| base + r == ret) else {
        return;
    };
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
    // A propeller the plugin flies: its blade-element forces replace the game's thrust.
    if ret == MODULE_BASE.load(Ordering::Relaxed) + PROP_THRUST_RETURN && crate::flight::prop_modeled(a4.wrapping_sub(OFF_PROP_FORCE_ACC)) {
        return 0;
    }
    // A jet nozzle's thrust follows its spool: queue a reshaped copy of the force instead.
    let shaped: [f32; 3];
    let mut force = a3;
    if a3 != 0 && ret == MODULE_BASE.load(Ordering::Relaxed) + JET_NOZZLE_RETURN && crate::flight::flies_obj(a2) {
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
    let ret = RET_CLAMPED.load(Ordering::Relaxed);
    // The game's fin / wing air force: ours replace it (callers ignore the return value).
    if GAME_AIR_FORCE_RETURNS.iter().any(|r| MODULE_BASE.load(Ordering::Relaxed) + r == ret) && crate::flight::flies_obj(a2) {
        return 0;
    }
    // Coaxial rotors' anti-torque couple: the plugin's two counter-rotating discs replace it.
    if PROP_COUPLE_RETURNS.iter().any(|r| MODULE_BASE.load(Ordering::Relaxed) + r == ret) && crate::flight::any_prop_modeled() && crate::flight::flies_obj(a2) {
        return 0;
    }
    note_thrust(ret, &CLAMPED_THRUST_RETURNS, a2, a3, a5, a6, a7);
    ORIGINAL_CLAMPED.get().map_or(0, |o| o(a1, a2, a3, a4, a5, a6, a7))
}

/// Engine thrust capture; failure leaves the applyForce sum / estimate in F9.
type PropTick = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
static ORIGINAL_PROP_TICK: OnceLock<PropTick> = OnceLock::new();

/// `force_emitter_tick_propeller_rotor`: note the speed it starts from, then let our blades'
/// torque set the speed it leaves (`flight::prop_after_tick`).
unsafe extern "system" fn prop_tick_detour(comp: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, a7: usize, a8: usize) -> usize {
    let Some(original) = ORIGINAL_PROP_TICK.get() else {
        return 0;
    };
    let w_in = if crate::editor::ptr_ok(comp, OFF_PROP_FORCE_ACC) {
        let from = if read_usize(comp + OFF_PROP_LINK) != 0 { OFF_PROP_SHAFT } else { OFF_PROP_SPEED };
        Some(f32::from_bits(crate::memory::read_u32_at(comp + from)))
    } else {
        None
    };
    let angle = w_in.map(|_| f32::from_bits(crate::memory::read_u32_at(comp + OFF_PROP_ANGLE)));
    let r = original(comp, a2, a3, a4, a5, a6, a7, a8);
    if let Some(w) = w_in {
        let _ = std::panic::catch_unwind(|| crate::flight::prop_after_tick(comp, w));
    }
    if let (Some(before), Some(w)) = (angle, w_in) {
        true_spin(comp, before, w);
        let step = f32::from_bits(crate::memory::read_u32_at(comp + OFF_PROP_ANGLE)) - before;
        let _ = std::panic::catch_unwind(|| crate::blur::track_hinge(comp, step as f64));
    }
    r
}

/// Air props and rotors drawn at their real speed (shaft RPS × rotation_speed rev/s). The game
/// turns them by `w_prev·rotation_speed·0.5/60`, 4π too slow, and `w_prev` is the speed after
/// the load, not the shaft's: use the shaft speed the tick started from.
unsafe fn true_spin(comp: usize, before: f32, w_in: f32) {
    let def = read_usize(comp + OFF_COMP_DEF);
    if !crate::editor::ptr_ok(def + OFF_DEF_PROP_FLAGS, 4) || crate::memory::read_u32_at(def + OFF_DEF_PROP_FLAGS) & 2 != 0 {
        return;
    }
    if !crate::editor::ptr_ok(def, OFF_DEF_ROTATION_SPEED + 4) {
        return;
    }
    let tau = std::f32::consts::TAU;
    let at = (comp + OFF_PROP_ANGLE) as *mut f32;
    let rot = f32::from_bits(crate::memory::read_u32_at(def + OFF_DEF_ROTATION_SPEED));
    let next = (before + w_in * rot * tau / 60.0).rem_euclid(tau);
    if next.is_finite() {
        at.write_unaligned(next);
    }
}

unsafe fn read_usize(at: usize) -> usize {
    crate::memory::read_usize_at(at)
}

unsafe fn install_prop_hook() {
    let at = MODULE_BASE.load(Ordering::SeqCst) + RVA_PROP_TICK;
    if !bytes_match(at, &PROP_TICK_PROLOGUE) {
        log_line(&format!("force_emitter tick prologue mismatch at {at:#x}; propellers keep the game's forces"));
        return;
    }
    match MinHook::create_hook(at as *mut c_void, prop_tick_detour as *mut c_void) {
        Ok(o) => {
            let _ = ORIGINAL_PROP_TICK.set(std::mem::transmute::<*mut c_void, PropTick>(o));
            match MinHook::enable_hook(at as *mut c_void) {
                Ok(()) => crate::flight::PROPS_HOOKED.store(true, Ordering::Relaxed),
                Err(e) => log_line(&format!("enable force_emitter tick hook: {e:?}")),
            }
        }
        Err(e) => log_line(&format!("create force_emitter tick hook: {e:?}")),
    }
}

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
            return Err(format!(
                "{name} prologue mismatch at {at:#x} (hooked by another plugin, or not build {GAME_BUILD_LABEL}); in-game flight forces disabled"
            ));
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
    install_air_node_hook();
    install_thrust_hooks();
    install_prop_hook();
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
    let original = MinHook::create_hook(target as *mut c_void, editor_update_detour as *mut c_void).map_err(|e| format!("create editor update hook: {e:?}"))?;
    let _ = ORIGINAL_EDITOR_UPDATE.set(std::mem::transmute::<*mut c_void, EditorUpdate>(original));
    MinHook::enable_hook(target as *mut c_void).map_err(|e| format!("enable editor update hook: {e:?}"))?;
    log_line(&format!("editor update hooked at {target:#x}"));
    if crate::plugin::toolbar_disabled() {
        log_line("Aero wings toolbar button disabled by request (aero_disable_toolbar)");
    } else {
        install_toolbar_hooks();
    }
    Ok(())
}

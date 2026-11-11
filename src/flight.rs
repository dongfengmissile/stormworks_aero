//! In-game flight: replaces the game's air forces on every vehicle with the plugin's model.
//!
//! Runs on the update thread after `physics_scene_post_step_sync`, while physics is idle:
//! - zeroes the six vanilla air-force constants (water forces use other fields);
//! - per vehicle, builds the analysis model off-thread (cached by bodies + component counts);
//! - each physics step, puts the model's forces on each body's btRigidBody, where that body
//!   is now (a deflected hinged sub-grid is just a tilted panel).
//! The Bullet velocity damping the game sets per body is zeroed by the `set_damping` hook.

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, LazyLock, Mutex, OnceLock};

use crate::defs::Library;
use crate::editor::{body_comp_count, ptr_ok, ring, vehicle_bodies, vehicle_comps};
use crate::logging::log_line;
use crate::math::{v3, V3};
use crate::memory::{read_u32_at, read_usize_at, RegionCacheScope};
use crate::model::{Designations, LiveBody, LiveSrc, Model};
use crate::offsets::*;
use crate::state::MODULE_BASE;
use crate::vehicle::Comp;

/// Plugin flight model on (set by the overlay). Off: the game's own air forces.
pub(crate) static ENABLED: AtomicBool = AtomicBool::new(true);
/// The flight hooks are installed (the overlay only offers the switch then).
pub(crate) static HOOKED: AtomicBool = AtomicBool::new(false);
/// Game folder and %APPDATA%/Stormworks, set at bootstrap.
pub(crate) static PATHS: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();
/// `btRigidBody::applyForce`, resolved when the hooks are installed.
pub(crate) static APPLY_FORCE: OnceLock<unsafe extern "system" fn(usize, *const f64, *const f64)> = OnceLock::new();
pub(crate) type Raycast = unsafe extern "system" fn(usize, *const f64, *mut f64, *mut usize, i32, i32) -> u8;
pub(crate) type WaterHeight = unsafe extern "system" fn(usize, *const f64) -> f32;
pub(crate) static RAYCAST: OnceLock<Raycast> = OnceLock::new();
pub(crate) static WATER_HEIGHT: OnceLock<WaterHeight> = OnceLock::new();

/// Height of world point `p` above whatever is below it (ground, buildings, water) within `max` metres,
/// ignoring the vehicle's own bodies (`own` physics objects). None = nothing that close.
unsafe fn height_above_surface(physics_scene: usize, c_scene: usize, p: V3, max: f64, own: &dyn Fn(usize, V3) -> bool) -> Option<f64> {
    let mut top: Option<f64> = None;
    if let Some(ray) = RAYCAST.get() {
        let mut from = p;
        for _ in 0..8 {
            let r = [from.x, from.y, from.z, from.x, p.y - max, from.z];
            let (mut hit, mut obj) = ([0f64; 3], 0usize);
            if from.y <= p.y - max || ray(physics_scene, r.as_ptr(), hit.as_mut_ptr(), &mut obj, 0x7F, 0xFF) == 0 {
                break;
            }
            if own(obj, v3(hit[0], hit[1], hit[2])) {
                from = v3(hit[0], hit[1] - 0.05, hit[2]); // our own landing gear or fuselage: look past it
                continue;
            }
            top = Some(hit[1]);
            break;
        }
    }
    if let Some(water) = WATER_HEIGHT.get() {
        let w = water(c_scene + OFF_SCENE_WATER_FIELD, [p.x, p.y, p.z].as_ptr()) as f64;
        if w.is_finite() && p.y - w < max {
            top = Some(top.map_or(w, |t| t.max(w)));
        }
    }
    top.map(|t| (p.y - t).max(0.0))
}

/// Ground-effect factor φ per panel (vertical surfaces get none); empty when the vehicle is well clear.
unsafe fn ground_factors(physics_scene: usize, c_scene: usize, m: &Model, lbs: &[LiveBody], main: usize, bodies: &[usize]) -> (Vec<f64>, Option<f64>) {
    let span = m.b_ref;
    if span <= 0.0 || RAYCAST.get().is_none() {
        return (Vec::new(), None);
    }
    // Our own bodies, and anything inside our voxel box (wheels and other parts with their own colliders).
    let objs: HashSet<usize> = bodies.iter().map(|&b| read_usize_at(b + OFF_BODY_PHYS_OBJ)).collect();
    // Box in grid metres (cell c spans c ± 0.5; one empty padding cell all round). Its floor is the bottom
    // face of the lowest block, so a runway touching the wheels is ground, not us.
    let (lo, hi) = (
        v3(m.grid.min[0] as f64, (m.grid.min[1] + 1) as f64 - 0.5, m.grid.min[2] as f64) * 0.25 + v3(0.0, 0.02, 0.0),
        v3((m.grid.min[0] + m.grid.dims[0] as i32) as f64, (m.grid.min[1] + m.grid.dims[1] as i32) as f64, (m.grid.min[2] + m.grid.dims[2] as i32) as f64)
            * 0.25,
    );
    let mb = &lbs[main];
    let own = |obj: usize, hit: V3| {
        let g = mb.world_to_grid(hit);
        objs.contains(&obj) || (g.x > lo.x && g.x < hi.x && g.y > lo.y && g.y < hi.y && g.z > lo.z && g.z < hi.z)
    };
    let max = 1.5 * span; // φ > 0.99 above
    if height_above_surface(physics_scene, c_scene, lbs[main].to_world(m.com), max + 5.0, &own).is_none() {
        return (Vec::new(), None);
    }
    let mut lowest: Option<f64> = None;
    let phis: Vec<f64> = m
        .panels
        .iter()
        .map(|p| {
            let lb = &lbs[m.panel_body(p).min(lbs.len() - 1)];
            let Some(h) = height_above_surface(physics_scene, c_scene, lb.to_world(p.centroid), max, &own) else {
                return 1.0;
            };
            let level = lb.dir_to_world(p.n).y.abs(); // a fin standing upright feels none
            if level > 0.7 {
                lowest = Some(lowest.map_or(h, |l: f64| l.min(h)));
            }
            1.0 - (1.0 - crate::model::ground_effect(h, span)) * level
        })
        .collect();
    static LAST: AtomicU64 = AtomicU64::new(0);
    let now = crate::editor::now_ms();
    if now.saturating_sub(LAST.load(Ordering::Relaxed)) > 10_000 {
        LAST.store(now, Ordering::Relaxed);
        let min_phi = phis.iter().copied().fold(1.0, f64::min);
        log_line(&format!(
            "ground effect: lowest level panel {:?} m above the surface (span {span:.1} m), strongest induced-drag factor {min_phi:.2}",
            lowest.map(|h| (h * 100.0).round() / 100.0)
        ));
    }
    (phis, lowest)
}

/// A jet nozzle's spool model and where its thrust acts (last game call), keyed by nozzle definition and
/// grid position so it survives respawning the vehicle.
struct Jet {
    spool: crate::spool::Spool,
    rb: usize,
    rel: [f64; 3],
    dir: [f64; 3],
    called: bool,
    seen_ms: u64,
}
static JETS: LazyLock<Mutex<HashMap<(usize, [i32; 3]), Jet>>> = LazyLock::new(|| Mutex::new(HashMap::new()));

/// A nozzle queued thrust `f` (world axes) at `rel` (from the centre of mass) on `phys_obj`'s rigid body:
/// feed its spool model and return the thrust to queue instead. None = not a nozzle we can read.
pub(crate) unsafe fn jet_sample(this: usize, phys_obj: usize, f: [f32; 3], rel: *const f32) -> Option<[f32; 3]> {
    use crate::editor::{ptr_ok, ring};
    let def_type = |c: usize| -> Option<u32> {
        let def = ptr_ok(c, 0x60).then(|| read_usize_at(c + 0x58))?;
        ptr_ok(def, OFF_JET_DEF_TYPE + 4).then(|| read_u32_at(def + OFF_JET_DEF_TYPE))
    };
    if def_type(this)? != 4 || !ptr_ok(phys_obj + OFF_OBJ_RIGID_BODY, 8) {
        return None;
    }
    // Shaft RPS from the spooling parts upstream (compressor / combustor / turbine share it).
    let rps = ring(this + OFF_JET_UPSTREAM, 64)?
        .into_iter()
        .filter(|&n| matches!(def_type(n), Some(1..=3)) && ptr_ok(n, OFF_JET_RPS + 4))
        .map(|n| f32::from_bits(read_u32_at(n + OFF_JET_RPS)) as f64)
        .fold(f64::NAN, f64::max);
    if !rps.is_finite() {
        return None;
    }
    // The game scales jet thrust by its own linear altitude factor; use its air density curve instead
    // (what its propellers, rotors and the plugin's aerodynamics use): thrust ∝ air mass flow.
    let rb = read_usize_at(phys_obj + OFF_OBJ_RIGID_BODY);
    let h = if ptr_ok(rb + OFF_RB_COM, 24) { f64::from_bits(read_usize_at(rb + OFF_RB_COM + 8) as u64) } else { 0.0 };
    let linear = (1.0 - h * 1.5 / 40_000.0).clamp(0.0, 1.0);
    let density = crate::model::atmosphere(h).rho / f64::from_bits(crate::model::SEA_LEVEL_RHO.load(Ordering::Relaxed)).max(1e-6);
    let k = if linear > 0.01 { density / linear } else { 0.0 };
    let f = f.map(|x| (x as f64 * k) as f32);
    let n = f.iter().map(|&x| (x as f64).powi(2)).sum::<f64>().sqrt();
    let key = (read_usize_at(this + 0x58), [0, 4, 8].map(|o| read_u32_at(this + 0x18 + o) as i32));
    let now = crate::editor::now_ms();
    let mut jets = JETS.lock().unwrap_or_else(|p| p.into_inner());
    if jets.len() > 512 {
        jets.clear();
    }
    let j = jets.entry(key).or_insert(Jet { spool: Default::default(), rb: 0, rel: [0.0; 3], dir: [0.0; 3], called: false, seen_ms: 0 });
    j.spool.sample(rps, n);
    if n > 1e-6 {
        j.dir = f.map(|x| x as f64 / n);
    }
    j.rb = rb;
    if !rel.is_null() {
        j.rel = [0, 1, 2].map(|k| rel.add(k).read_unaligned() as f64);
    }
    j.called = true;
    j.seen_ms = now;
    let out = j.spool.output();
    static LAST: AtomicU64 = AtomicU64::new(0);
    if now.saturating_sub(LAST.load(Ordering::Relaxed)) > 10_000 {
        LAST.store(now, Ordering::Relaxed);
        log_line(&format!("jet spool: nozzle {this:#x} shaft {rps:.1} RPS, game thrust {n:.0} N, applied {out:.0} N"));
    }
    Some(j.dir.map(|d| (d * out) as f32))
}

/// Once per physics step for a vehicle's rigid bodies `rbs`: nozzles whose combustion queued nothing since
/// the last step get their smoothed thrust applied here (else a PID chopping throttle drops thrust to 0 for
/// those ticks). Returns the thrust filled in (world, summed).
unsafe fn fill_jets(rbs: &HashSet<usize>, apply: unsafe extern "system" fn(usize, *const f64, *const f64)) -> (V3, u32, V3, f64) {
    let now = crate::editor::now_ms();
    let mut sum = (V3::ZERO, 0, V3::ZERO, 0.0);
    for j in JETS.lock().unwrap_or_else(|p| p.into_inner()).values_mut().filter(|j| rbs.contains(&j.rb)) {
        if std::mem::replace(&mut j.called, false) || now.saturating_sub(j.seen_ms) > 2000 {
            continue;
        }
        j.spool.idle();
        let out = j.spool.output();
        if out < 1.0 {
            continue;
        }
        let f = [j.dir[0] * out, j.dir[1] * out, j.dir[2] * out, 0.0];
        let r = [j.rel[0], j.rel[1], j.rel[2], 0.0];
        apply(j.rb, f.as_ptr(), r.as_ptr());
        let at = if crate::editor::ptr_ok(j.rb + OFF_RB_COM, 24) { v3_f64(j.rb + OFF_RB_COM) } else { V3::ZERO } + v3(r[0], r[1], r[2]);
        sum = (sum.0 + v3(f[0], f[1], f[2]), sum.1 + 1, sum.2 + at * out, sum.3 + out);
    }
    sum
}

/// Engine thrust queued per rigid body since our last tick: (sum, calls, source bits).
/// Value: (force sum, calls, source bits, Σ|F|·point, Σ|F|) — the last two give the centre of thrust.
static ENGINE_FORCES: LazyLock<Mutex<HashMap<usize, (V3, u32, u8, V3, f64)>>> = LazyLock::new(|| Mutex::new(HashMap::new()));

pub(crate) fn note_engine_force(rb: usize, f: [f64; 3], source: u8, rel: Option<[f64; 3]>, max: f32, scale: f32) {
    if !f.iter().all(|x| x.is_finite()) {
        return;
    }
    // The wrapper clamps |F|/scale to max: log the first time a capture would exceed it (then ours reads high).
    static WARNED: AtomicBool = AtomicBool::new(false);
    let n = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
    if scale.abs() > 1e-9 && n / scale.abs() as f64 > max as f64 && !WARNED.swap(true, Ordering::Relaxed) {
        log_line(&format!("thrust: engine force {n:.0} N over the wrapper cap ({max} × {scale}); the reading may be high"));
    }
    let mut m = ENGINE_FORCES.lock().unwrap_or_else(|p| p.into_inner());
    let com = unsafe {
        if crate::editor::ptr_ok(rb + OFF_RB_COM, 24) {
            v3_f64(rb + OFF_RB_COM)
        } else {
            V3::ZERO
        }
    };
    let at = com + rel.map_or(V3::ZERO, |r| v3(r[0], r[1], r[2]));
    let e = m.entry(rb).or_insert((V3::ZERO, 0, 0, V3::ZERO, 0.0));
    e.0 += v3(f[0], f[1], f[2]);
    e.1 += 1;
    e.2 |= source;
    e.3 += at * n;
    e.4 += n;
}

pub(crate) static TICKS: AtomicU64 = AtomicU64::new(0);
pub(crate) static APPLIED: AtomicU64 = AtomicU64::new(0);
pub(crate) static VEHICLES: AtomicU64 = AtomicU64::new(0);
pub(crate) static MODELS: AtomicU64 = AtomicU64::new(0);
/// A model build is running (one at a time: a world can hold many vehicles).
static BUILDING: AtomicBool = AtomicBool::new(false);

type Built = Arc<Mutex<Option<(u64, Arc<Model>, usize, String)>>>;

/// One force for the debug arrows: world point, lift part (across the airflow), drag part.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Arrow {
    pub at: V3,
    pub lift: V3,
    pub drag: V3,
    /// Body pointer and the same arrow in its grid frame (voxel·0.25 m), so the overlay can
    /// place it with the body's render matrix every frame instead of at physics-tick positions.
    pub body: usize,
    pub local_at: V3,
    pub local_lift: V3,
    pub local_drag: V3,
    /// Panel (true) or fuselage (false).
    pub wing: bool,
}

/// Flight data of one vehicle this tick (FAR's flight data readout).
#[derive(Clone, Debug, Default)]
pub(crate) struct VehicleDebug {
    pub id: u32,
    pub designations: String,
    pub airspeed: f64,
    pub mach: f64,
    pub alt: f64,
    pub q: f64,
    /// Air density at the vehicle, kg/m³.
    pub rho: f64,
    pub aoa: f64,
    pub beta: f64,
    pub cl: f64,
    pub cd: f64,
    pub lift: f64,
    pub drag: f64,
    pub weight: f64,
    /// Vehicle mass gravity acts on, kg: the physics bodies plus the fluid in their tanks (`fluid_mass`).
    pub mass: f64,
    /// Estimated thrust along the nose, N.
    pub thrust: f64,
    /// Game-applied force along the nose last step (engines, propellers), N, if the game applied any.
    pub thrust_exact: Option<f64>,
    pub thrust_source: String,
    /// Measured physics step, s (game time per step).
    pub step: f64,
    /// Lowest wing panel's height above the surface, m, and its induced-drag factor φ (ground effect).
    pub ground: Option<(f64, f64)>,
    /// Main body pointer, and the CG / centre of lift as (world point, main-body grid position).
    pub main_body: usize,
    pub cg: Option<(V3, V3)>,
    pub col: Option<(V3, V3)>,
    /// Static margin: CG ahead of the centre of lift, m (positive = stable), and the reference chord.
    pub margin: Option<f64>,
    pub mac: f64,
    /// Centre of thrust (world, main-body grid) and the total thrust vector (world, N).
    pub thrust_at: Option<((V3, V3), V3)>,
    /// Live centre of lift (world, main-body grid).
    pub lift_at: Option<(V3, V3)>,
    pub com: V3,
    pub arrows: Vec<Arrow>,
    pub stalled: usize,
    pub panels: usize,
    pub props: Vec<PropDebug>,
}

/// One propeller or rotor in the flight data.
#[derive(Clone, Debug, Default)]
pub(crate) struct PropDebug {
    pub name: String,
    /// Blade speed, RPS.
    pub rps: f64,
    /// Shaft speed, RPS (the game's unit).
    pub shaft: f64,
    pub thrust: f64,
    /// Power the air takes from the shaft, W (negative: the air drives it).
    pub power: f64,
    pub vrs: f64,
    /// Blade angle at 0.75 R, degrees, and the stalled share of the blades (0..1).
    pub blade_deg: f64,
    pub stalled: f64,
    /// Body pointer, then hub and net force as (world, the body's grid frame): the overlay places
    /// them with the body's render matrix every frame.
    pub body: usize,
    pub hub: (V3, V3),
    pub force: (V3, V3),
    /// Slipstream of a propeller under power: thrust axis (world, grid) and
    /// [disc radius m, flight speed, induced speed m/s, swirl r·vθ m²/s signed by the spin].
    pub wash: Option<((V3, V3), [f64; 4])>,
}

/// Set by the overlay while the flight data window or the arrows are shown.
pub(crate) static DEBUG_WANTED: AtomicBool = AtomicBool::new(false);
static SIM_STATE: AtomicUsize = AtomicUsize::new(0);
static SIM_STATE_MS: AtomicU64 = AtomicU64::new(0);

pub(crate) fn note_sim_state(g: usize) {
    SIM_STATE.store(g, Ordering::Relaxed);
    SIM_STATE_MS.store(crate::editor::now_ms(), Ordering::Relaxed);
}

/// Main in-game camera viewproj (world f64, row vectors p·M), if the simulation drew this moment.
pub(crate) fn main_viewproj() -> Option<[f64; 16]> {
    if crate::editor::now_ms().saturating_sub(SIM_STATE_MS.load(Ordering::Relaxed)) > 250 {
        return None; // menu or workbench: the simulation is not drawing
    }
    let g = SIM_STATE.load(Ordering::Relaxed);
    let base = crate::state::MODULE_BASE.load(Ordering::Relaxed);
    let m = g + OFF_SIM_MAIN_CAMERA + OFF_CAM_VIEWPROJ_F64;
    unsafe {
        if !crate::editor::ptr_ok(g, 8) || read_usize_at(g) != base + RVA_SIM_STATE_VTABLE || !crate::editor::ptr_ok(m, 128) {
            return None;
        }
        let vp: [f64; 16] = std::array::from_fn(|k| f64::from_bits(read_usize_at(m + 8 * k) as u64));
        // w comes from column 3: all zero means an unset camera.
        (vp.iter().all(|v| v.is_finite()) && [vp[3], vp[7], vp[11], vp[15]].iter().any(|&w| w != 0.0)).then_some(vp)
    }
}

/// What the game drew this render frame with: alpha, the build it interpolated toward, the camera.
#[derive(Clone, Copy)]
pub(crate) struct FrameSnap {
    pub alpha: f32,
    pub ts: u64,
    pub vp: [f64; 16],
}
static FRAME_SNAP: Mutex<Option<(FrameSnap, u64)>> = Mutex::new(None);

/// Build time of the frame the game is about to draw (call before the original slot 7).
pub(crate) fn frame_build_ts(g: usize) -> Option<u64> {
    unsafe {
        let frame = read_usize_at(g + OFF_SIM_FRAME);
        crate::editor::ptr_ok(frame, OFF_FRAME_BUILD_NS + 8).then(|| read_usize_at(frame + OFF_FRAME_BUILD_NS) as u64)
    }
}

/// After the game drew the frame (slot 7 returned): keep its alpha and camera with the build time read before.
pub(crate) fn note_sim_frame(ts: Option<u64>) {
    let (Some(ts), Some((alpha, _)), Some(vp)) = (ts, render_frame(), main_viewproj()) else {
        return;
    };
    *FRAME_SNAP.lock().unwrap_or_else(|p| p.into_inner()) = Some((FrameSnap { alpha, ts, vp }, crate::editor::now_ms()));
}

/// The last drawn frame's snapshot, if the simulation is drawing.
pub(crate) fn frame_snap() -> Option<FrameSnap> {
    let g = FRAME_SNAP.lock().unwrap_or_else(|p| p.into_inner());
    g.filter(|(_, ms)| crate::editor::now_ms().saturating_sub(*ms) < 250).map(|(f, _)| f)
}

/// Per body: the last few frame builds (build time, built pose), recorded on the update thread where
/// the game's build runs, so time and pose always belong together.
pub(crate) static BUILT_HISTORY: LazyLock<Mutex<HashMap<usize, Vec<(u64, [f64; 16])>>>> = LazyLock::new(|| Mutex::new(HashMap::new()));

pub(crate) type MatrixInterp = crate::drawcap::MatrixInterp;
/// The game's own per-frame draw interpolation (rotation blend + linear translation).
pub(crate) static MATRIX_INTERP: OnceLock<MatrixInterp> = OnceLock::new();

/// This render frame's interpolation alpha and the time of the last frame build.
pub(crate) fn render_frame() -> Option<(f32, u64)> {
    let g = SIM_STATE.load(Ordering::Relaxed);
    unsafe {
        let frame = read_usize_at(g + OFF_SIM_FRAME);
        // ptr_ok wants 8-byte alignment: check the frame base over the whole 0xEE0-byte struct.
        if !crate::editor::ptr_ok(frame, OFF_FRAME_BUILD_NS + 8) {
            return None;
        }
        let alpha = f32::from_bits(read_u32_at(frame + OFF_FRAME_ALPHA));
        alpha.is_finite().then_some((alpha.clamp(0.0, 1.0), read_usize_at(frame + OFF_FRAME_BUILD_NS) as u64))
    }
}

/// Body render transform in world space at the last physics tick: `body+0x2C8` (f32, body -> vehicle) · `vehicle+0x190`
/// (f64, vehicle -> world), as in the workbench view. Row vectors: world = grid_m·M.
pub(crate) fn body_render_matrix(body: usize) -> Option<[f64; 16]> {
    body_matrix(body, OFF_BODY_RENDER, OFF_VEHICLE_WORLD)
}

/// The same pose as of the last frame build (`body+0x308` · `vehicle+0x210`): what the renderer interpolates toward.
pub(crate) fn body_built_matrix(body: usize) -> Option<[f64; 16]> {
    body_matrix(body, OFF_BODY_RENDER_BUILT, OFF_VEHICLE_WORLD_BUILT)
}

fn body_matrix(body: usize, off_body: usize, off_vehicle: usize) -> Option<[f64; 16]> {
    use crate::editor::ptr_ok;
    unsafe {
        if !ptr_ok(body + OFF_BODY_VEHICLE, 8) || !ptr_ok(body + off_body, 64) {
            return None;
        }
        let vehicle = read_usize_at(body + OFF_BODY_VEHICLE);
        if !ptr_ok(vehicle + off_vehicle, 128) {
            return None;
        }
        let b: [f64; 16] = std::array::from_fn(|k| f32::from_bits(read_u32_at(body + off_body + 4 * k)) as f64);
        let v: [f64; 16] = std::array::from_fn(|k| f64::from_bits(read_usize_at(vehicle + off_vehicle + 8 * k) as u64));
        let m: [f64; 16] = std::array::from_fn(|k| (0..4).map(|j| b[k / 4 * 4 + j] * v[j * 4 + k % 4]).sum());
        m.iter().all(|x| x.is_finite()).then_some(m)
    }
}

/// Latest flight data of every vehicle the plugin flies (update thread -> overlay).
pub(crate) static DEBUG: Mutex<Vec<VehicleDebug>> = Mutex::new(Vec::new());

struct Craft {
    key: u64,
    bodies: Vec<usize>,
    /// Component pointers in `Model::comps` order (live fin angles).
    comp_ptrs: Vec<usize>,
    built: Built,
    designations: String,
    stall: Vec<f64>,
    /// Thrust estimate: vehicle CoM velocity and our applied air force last tick, filtered thrust.
    prev_vel: Option<V3>,
    prev_aero: V3,
    thrust: f64,
    /// Main body CoM last tick and the measured physics step, s.
    prev_com: Option<V3>,
    dt: f64,
    /// Main body's centre of mass when forces were last applied: unchanged = no step since.
    last_com: [f64; 3],
    warned: bool,
    /// Per `Model::props`: mean inflow state (m/s), and the load calibration by (blades, length).
    prop_v: Vec<f64>,
    /// Vehicle key whose marks `fly` / `vrs` were read from (they gate the build).
    read_key: u64,
    /// The vehicle's own settings (None: the global `ENABLED` / `VRS` / `PROPS`).
    fly: Option<bool>,
    vrs: Option<bool>,
    props: Option<bool>,
    ground: Option<bool>,
}

/// Physics object (`body + OFF_BODY_PHYS_OBJ`) -> the plugin flies its vehicle, from the last
/// tick. The force hooks drop the game's air forces only for these.
/// Read from the game's worker threads (hull air), so a read-write lock.
/// Value: (the plugin flies it, with the plugin's propellers and rotors).
static FLOWN: LazyLock<std::sync::RwLock<HashMap<usize, (bool, bool)>>> = LazyLock::new(Default::default);
/// The game's hull air forces are skipped per body (`fluid_air_node_forces` hook), so the
/// settings air constants stay vanilla.
pub(crate) static AIR_PER_BODY: AtomicBool = AtomicBool::new(false);

/// The plugin flies this physics object's vehicle (unknown objects: the global setting).
pub(crate) fn flies_obj(obj: usize) -> bool {
    FLOWN.read().unwrap_or_else(|p| p.into_inner()).get(&obj).map_or_else(|| ENABLED.load(Ordering::Relaxed), |f| f.0)
}

/// This physics object's vehicle uses the plugin's propellers and rotors (unknown objects: the global settings).
pub(crate) fn props_obj(obj: usize) -> bool {
    FLOWN.read().unwrap_or_else(|p| p.into_inner()).get(&obj).map_or_else(|| ENABLED.load(Ordering::Relaxed) && PROPS.load(Ordering::Relaxed), |f| f.1)
}

/// Propellers and rotors blade by blade. Off: the game's own thrust and shaft load, with the
/// plugin's wings and fuselage (no propwash on them: the game's propellers have none).
pub(crate) static PROPS: AtomicBool = AtomicBool::new(true);
/// Ground effect on the wings. Off: the same lift and induced drag at any height.
pub(crate) static GROUND: AtomicBool = AtomicBool::new(true);

/// The propeller tick hook is in: the plugin may take over propellers (else the game's stay).
pub(crate) static PROPS_HOOKED: AtomicBool = AtomicBool::new(false);
/// Vortex ring state on (F7 Settings tab; a vehicle may override it): rotors sinking into their own wake lose lift.
pub(crate) static VRS: AtomicBool = AtomicBool::new(false);
/// Physics steps seen (prop load freshness).
static STEP: AtomicU64 = AtomicU64::new(0);

/// A propeller the plugin flies, by component pointer: the air's torque against the spin (N·m),
/// the shaft-speed direction it opposes (±1), the speed drop per N·m and tick, and the step.
#[derive(Clone, Copy)]
struct PropLoad {
    torque: f64,
    sign: f64,
    per_nm: f64,
    step: u64,
}
static PROP_LOADS: std::sync::LazyLock<Mutex<HashMap<usize, PropLoad>>> = std::sync::LazyLock::new(Default::default);

fn prop_loads() -> std::sync::MutexGuard<'static, HashMap<usize, PropLoad>> {
    PROP_LOADS.lock().unwrap_or_else(|p| p.into_inner())
}

/// The plugin flies this propeller now: the game's own thrust for it is dropped.
pub(crate) fn prop_modeled(comp: usize) -> bool {
    prop_loads().get(&comp).is_some_and(|l| l.step + 30 >= STEP.load(Ordering::Relaxed))
}
pub(crate) fn any_prop_modeled() -> bool {
    !prop_loads().is_empty()
}

/// After the game's propeller tick: the shaft speed loses what our blades' torque takes (or gains
/// what the air gives a windmilling rotor), instead of the game's own speed drop.
/// `w_in`: the speed the tick started from (shaft network speed when linked).
pub(crate) unsafe fn prop_after_tick(comp: usize, w_in: f32) {
    let Some(l) = prop_loads().get(&comp).copied() else {
        return;
    };
    if l.step + 30 < STEP.load(Ordering::Relaxed) || !w_in.is_finite() {
        return;
    }
    let w = w_in as f64 * 0.9999;
    let mut next = w - l.per_nm * l.torque * l.sign;
    // Drag stops a shaft, it does not reverse it.
    if l.torque > 0.0 && next * l.sign < 0.0 {
        next = 0.0;
    }
    ((comp + OFF_PROP_SPEED) as *mut f32).write_unaligned(next as f32);
}

/// Live state of a propeller part.
unsafe fn prop_live_state(comp: usize, rotor: bool) -> Option<(crate::model::PropLive, bool, bool)> {
    if !ptr_ok(comp, OFF_PROP_FORCE_ACC) {
        return None;
    }
    let f = |o: usize| f32::from_bits(read_u32_at(comp + o)) as f64;
    if *((comp + OFF_PROP_FOLDED) as *const u8) != 0 {
        return None;
    }
    let count = read_u32_at(comp + OFF_PROP_BLADES).clamp(2, 8);
    let active = (read_u32_at(comp + OFF_PROP_ACTIVE) & ((1u32 << count) - 1)).count_ones();
    let pitch = (read_u32_at(comp + OFF_PROP_PITCH) as i32 as f64 + f(OFF_PROP_COLLECTIVE)).clamp(-1.0, 1.0);
    let linked = read_usize_at(comp + OFF_PROP_LINK) != 0;
    let live = crate::model::PropLive {
        // Linked, comp+0x150 is the speed handed back to the network (after our load), not the shaft's.
        w: f(if linked { OFF_PROP_SHAFT } else { OFF_PROP_SPEED }),
        pitch,
        cyclic: if rotor { [f(OFF_PROP_CYCLIC), f(OFF_PROP_CYCLIC + 4)] } else { [0.0; 2] },
        blades: active,
        length: f(OFF_PROP_LENGTH),
        reversed: *((comp + OFF_PROP_REVERSE) as *const u8) != 0,
        flip: crate::blur::hinge_against(comp),
    };
    let ok = [live.w, live.pitch, live.cyclic[0], live.cyclic[1], live.length].iter().all(|v| v.is_finite()) && active > 0;
    ok.then_some((live, linked, read_usize_at(comp + OFF_PROP_HINGE) != 0))
}

struct State {
    crafts: HashMap<u32, Craft>,
    /// Vanilla air constants, saved while ours replace them.
    saved: Option<[f32; 6]>,
    scene: usize,
    lib: Option<Arc<Mutex<Library>>>,
    /// Net client, per body: the last replicated pose (tick stamp, CoM, axes) and the motion between the last two.
    net: HashMap<usize, (u32, V3, [V3; 3], V3, V3)>,
}

/// This game is a multiplayer client: the host flies the vehicles, bodies here only follow its poses.
pub(crate) fn net_client() -> bool {
    let base = MODULE_BASE.load(Ordering::Relaxed);
    unsafe { *((base + RVA_NET_IS_MULTIPLAYER) as *const u8) != 0 && *((base + RVA_NET_IS_AUTHORITATIVE) as *const u8) == 0 }
}

static STATE: std::sync::LazyLock<Mutex<State>> = std::sync::LazyLock::new(|| Mutex::new(State { crafts: HashMap::new(), saved: None, scene: 0, lib: None, net: HashMap::new() }));

unsafe fn f64_at(a: usize) -> f64 {
    f64::from_bits(read_usize_at(a) as u64)
}
unsafe fn f32_at(a: usize) -> f32 {
    f32::from_bits(read_u32_at(a))
}
unsafe fn v3_f64(a: usize) -> V3 {
    v3(f64_at(a), f64_at(a + 8), f64_at(a + 16))
}
unsafe fn v3_f32(a: usize) -> V3 {
    v3(f32_at(a) as f64, f32_at(a + 4) as f64, f32_at(a + 8) as f64)
}

/// Current game angle of a live panel, radians. Rudder: `comp+0x274` about the hinge. Rocket fin
/// pair A (else C) about its `FinCtrl` axis (`solid_rocket_draw` 0x43ECC0: α_A = −(s13 + s11)/√2,
/// α_C = (s13 − s11)/√2, the two channel angles signed by the inversion bytes).
unsafe fn live_angle(comp: usize, src: LiveSrc) -> Option<f64> {
    let a = match src {
        LiveSrc::Rudder => {
            let t = if ptr_ok(comp, OFF_RUDDER_ANGLE + 4) {
                f32_at(comp + OFF_RUDDER_ANGLE) as f64
            } else {
                return None;
            };
            return (t.is_finite() && t.abs() < 1.6).then_some(t);
        }
        LiveSrc::Rocket { a } => a,
    };
    if !ptr_ok(comp, OFF_ROCKET_FIN_ANGLE + 8) {
        return None;
    }
    let ang = |o: usize| f32_at(comp + OFF_ROCKET_FIN_ANGLE + o) as f64;
    let inv = |o: usize| *((comp + OFF_ROCKET_FIN_INVERT + o) as *const u8) != 0;
    let s11 = if inv(0) { 1.0 } else { -1.0 } * ang(0);
    let s13 = if inv(1) { -1.0 } else { 1.0 } * ang(4);
    let alpha = if a { -(s13 + s11) } else { s13 - s11 } * std::f64::consts::FRAC_1_SQRT_2;
    (alpha.is_finite() && alpha.abs() < 1.6).then_some(alpha)
}

/// Write the vanilla air constants: zeros while enabled, the saved values when switched off.
unsafe fn air_constants(st: &mut State, enabled: bool) {
    let base = MODULE_BASE.load(Ordering::Relaxed) + RVA_GAME_SETTINGS;
    if !ptr_ok(base, 0xF0) {
        return;
    }
    let rho0 = ((base + OFF_SETTINGS_AIR_DENSITY) as *const f32).read() as f64;
    if rho0.is_finite() && rho0 > 0.0 {
        crate::model::SEA_LEVEL_RHO.store(rho0.to_bits(), Ordering::Relaxed);
    }
    let at = |i: usize| (base + AIR_CONSTANTS[i]) as *mut f32;
    match (enabled, st.saved) {
        (true, None) => {
            let v: [f32; 6] = std::array::from_fn(|i| at(i).read());
            log_line(&format!("flight: vanilla air constants {v:?} replaced"));
            st.saved = Some(v);
        }
        (false, Some(v)) => {
            (0..6).for_each(|i| at(i).write(v[i]));
            st.saved = None;
            log_line("flight: vanilla air constants restored");
            return;
        }
        _ => {}
    }
    if enabled {
        (0..6).for_each(|i| at(i).write(0.0));
    }
}

/// Mass the game's gravity adds for the fluid in a body's tanks (`body_fluid_force_compute`
/// 0x48D5B0: a downward force, not body mass), kg. Every liquid weighs as water; gas only past
/// the tank's capacity. shortcut: the game settings' defaults (water density 1000, factors 1,
/// buoyancy mass 10..20 over 2000..25000 kg); read them from settings if a mod changes them.
unsafe fn fluid_mass(body: usize, body_mass: f64) -> f64 {
    const LIQUIDS: [usize; 7] = [0, 1, 2, 5, 6, 8, 9];
    const GASES: [usize; 6] = [3, 4, 7, 10, 11, 12];
    let Some(items) = crate::editor::ring(body + OFF_BODY_COMPARTMENTS, 1 << 16) else {
        return 0.0;
    };
    let v14 = 10.0 + ((body_mass - 2000.0) / 23000.0).clamp(0.0, 1.0) * 10.0;
    let mut sum = 0.0;
    for item in items {
        if !ptr_ok(item, OFF_COMPARTMENT_AMOUNTS + 13 * 4) {
            continue;
        }
        let at = |t: usize| f32_at(item + OFF_COMPARTMENT_AMOUNTS + 4 * t) as f64;
        let liq: f64 = LIQUIDS.iter().map(|&t| at(t)).sum();
        let gas = (GASES.iter().map(|&t| at(t)).sum::<f64>() - f32_at(item + OFF_COMPARTMENT_CAPACITY) as f64 / 60.0).max(0.0);
        let m = 0.02 * 1000.0 * (liq + gas) / v14;
        if m.is_finite() && m > 0.0 {
            sum += m;
        }
    }
    sum
}

/// Live pose and motion of one body; None if it is not a simulated, movable body.
unsafe fn live_body(body: usize, client: bool) -> Option<(LiveBody, usize, f64)> {
    let obj = read_usize_at(body + OFF_BODY_PHYS_OBJ);
    if !ptr_ok(obj, OFF_OBJ_RIGID_BODY + 8) || *((body + OFF_BODY_NO_FORCE) as *const u8) != 0 {
        return None;
    }
    let mass = f32_at(obj + OFF_OBJ_MASS) as f64;
    let rb = read_usize_at(obj + OFF_OBJ_RIGID_BODY);
    if !(mass > 0.0) || !ptr_ok(rb, OFF_RB_ANGULAR_FACTOR + 24) || (!client && read_u32_at(rb + OFF_RB_FLAGS) & 0x800 != 0) {
        return None;
    }
    if read_u32_at(rb + OFF_RB_ACTIVATION) == 2 {
        return None; // asleep: not moving, nothing to do
    }
    let m: [f64; 16] = std::array::from_fn(|k| f64_at(obj + OFF_OBJ_WORLD + 8 * k));
    let lb = LiveBody {
        origin: v3_f32(body + OFF_BODY_ORIGIN),
        rot: [v3(m[0], m[1], m[2]), v3(m[4], m[5], m[6]), v3(m[8], m[9], m[10])],
        trans: v3(m[12], m[13], m[14]),
        com: v3_f64(rb + OFF_RB_COM),
        vel: v3_f64(rb + OFF_RB_LIN_VEL),
        omega: v3_f64(rb + OFF_RB_ANG_VEL),
        wind: v3_f32(body + OFF_BODY_WIND),
    };
    let finite = [lb.trans, lb.com, lb.vel, lb.omega, lb.wind, lb.origin].iter().all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
    finite.then_some((lb, rb, mass))
}

fn spawn_build(lib: Arc<Mutex<Library>>, comps: Vec<Comp>, marked: Option<Designations>, key: u64, out: Built, id: u32) {
    let main = {
        let mut count: HashMap<u32, usize> = HashMap::new();
        comps.iter().for_each(|c| *count.entry(c.body).or_default() += 1);
        count.into_iter().max_by_key(|&(b, n)| (n, std::cmp::Reverse(b))).map_or(0, |(b, _)| b as usize)
    };
    BUILDING.store(true, Ordering::SeqCst);
    let spawned = std::thread::Builder::new().name("aero-flight-model".into()).spawn(move || {
        let _done = Done;
        let (des, name) = match marked {
            Some(d) => (d, "on the parts".to_string()),
            None => Default::default(),
        };
        let n = comps.len();
        let built = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut lib = lib.lock().unwrap_or_else(|p| p.into_inner());
            Model::build(comps, &mut lib, &des)
        }));
        match built {
            Ok(m) => {
                log_line(&format!(
                    "flight: vehicle {id}: model of {n} parts, {} panels, mass {:.0} kg, designations {}",
                    m.panels.len(),
                    m.mass,
                    if name.is_empty() { "none (vanilla wings only)".into() } else { name.clone() }
                ));
                MODELS.fetch_add(1, Ordering::Relaxed);
                *out.lock().unwrap_or_else(|p| p.into_inner()) = Some((key, Arc::new(m), main, name));
            }
            Err(_) => log_line(&format!("flight: vehicle {id}: model build panicked")),
        }
    });
    if spawned.is_err() {
        BUILDING.store(false, Ordering::SeqCst);
    }
}

/// Clears `BUILDING` when the build thread ends, however it ends.
struct Done;
impl Drop for Done {
    fn drop(&mut self) {
        BUILDING.store(false, Ordering::SeqCst);
    }
}

/// After the game synced physics state, before it computes this tick's vehicle forces.
pub(crate) unsafe fn on_post_step_sync(physics_scene: usize) {
    let enabled = ENABLED.load(Ordering::Relaxed);
    let step = STEP.fetch_add(1, Ordering::Relaxed) + 1;
    prop_loads().retain(|_, l| l.step + 30 >= step);
    let mut st = STATE.lock().unwrap_or_else(|p| p.into_inner());
    air_constants(&mut st, enabled && !AIR_PER_BODY.load(Ordering::Relaxed));
    let mut flown: HashMap<usize, (bool, bool)> = HashMap::new();
    TICKS.fetch_add(1, Ordering::Relaxed);
    let _fault = crate::fault::enter(crate::fault::stage::FLIGHT);
    let _cache = RegionCacheScope::new();
    let scene = physics_scene.wrapping_sub(OFF_SCENE_PHYSICS);
    if scene != st.scene {
        // World load / respawn swaps the scene: every cached pointer is stale.
        st.crafts.clear();
        st.net.clear();
        st.scene = scene;
    }
    let Some(records) = ring(scene + OFF_SCENE_VEHICLES, 1 << 16) else {
        return;
    };
    let lib = st
        .lib
        .get_or_insert_with(|| {
            let (game, appdata) = PATHS.get().cloned().unwrap_or_default();
            Arc::new(Mutex::new(Library::new(&game, &appdata)))
        })
        .clone();
    let Some(apply_force) = APPLY_FORCE.get().copied() else {
        return;
    };
    let mut seen = HashSet::new();
    let client = net_client();
    static WAS_CLIENT: AtomicBool = AtomicBool::new(false);
    if WAS_CLIENT.swap(client, Ordering::Relaxed) != client {
        log_line(if client { "flight: multiplayer client: flight data from the host's poses, no forces applied" } else { "flight: host or single player: forces applied" });
    }
    let mut net = std::mem::take(&mut st.net);
    let want_debug = DEBUG_WANTED.load(Ordering::Relaxed);
    // The game's forces of the last step (applied after our previous tick, before the step).
    let engine_forces = std::mem::take(&mut *ENGINE_FORCES.lock().unwrap_or_else(|p| p.into_inner()));
    let mut debug = Vec::new();
    let mut stalled: HashMap<usize, HashSet<[i32; 3]>> = HashMap::new();
    // Stalled moving meshes (missile fins, vanilla control surfaces): component pointer -> red.
    let mut stalled_parts: HashMap<usize, u32> = HashMap::new();
    let mut stepped = false;
    for rec in records {
        if !ptr_ok(rec, 0x18) || *(rec as *const u8) != 0 || read_u32_at(rec + 0x10) != 5 {
            continue;
        }
        let id = read_u32_at(rec + 4);
        let Some(bodies) = vehicle_bodies(read_usize_at(rec + 8)) else {
            continue;
        };
        if bodies.is_empty() {
            continue;
        }
        seen.insert(id);
        // Change test: bodies and their component counts.
        let mut key: u64 = 0xcbf29ce484222325;
        for &b in &bodies {
            for v in [b as u64, body_comp_count(b) as u64] {
                key = (key ^ v).wrapping_mul(0x100000001b3);
            }
        }
        let craft = st.crafts.entry(id).or_insert_with(|| Craft {
            key: 0,
            bodies: Vec::new(),
            comp_ptrs: Vec::new(),
            built: Arc::default(),
            designations: String::new(),
            stall: Vec::new(),
            prev_vel: None,
            prev_aero: V3::ZERO,
            thrust: 0.0,
            prev_com: None,
            dt: 1.0 / 60.0,
            last_com: [f64::NAN; 3],
            warned: false,
            prop_v: Vec::new(),
            read_key: 0,
            fly: None,
            vrs: None,
            props: None,
            ground: None,
        });
        // The vehicle's own settings ride on its parts' marks.
        if craft.read_key != key {
            if let Some((comps, _, ptrs)) = vehicle_comps(read_usize_at(rec + 8)) {
                craft.read_key = key;
                let d = crate::marks::designations(&comps, &ptrs);
                craft.fly = d.as_ref().and_then(|d| d.fly);
                craft.vrs = d.as_ref().and_then(|d| d.vrs);
                craft.props = d.as_ref().and_then(|d| d.props);
                craft.ground = d.as_ref().and_then(|d| d.ground);
            }
        }
        let fly = craft.fly.unwrap_or(enabled);
        let props_on = PROPS_HOOKED.load(Ordering::Relaxed) && craft.props.unwrap_or_else(|| PROPS.load(Ordering::Relaxed));
        for &b in &bodies {
            if ptr_ok(b + OFF_BODY_PHYS_OBJ, 8) {
                flown.insert(read_usize_at(b + OFF_BODY_PHYS_OBJ), (fly, fly && props_on));
            }
        }
        if !fly {
            continue;
        }
        // (Re)build when the vehicle changed and can move; one build at a time, so a vehicle
        // waiting its turn keeps its old key and retries next tick.
        if craft.key != key && !BUILDING.load(Ordering::SeqCst) && bodies.iter().any(|&b| live_body(b, client).is_some()) {
            if let Some((comps, _, ptrs)) = vehicle_comps(read_usize_at(rec + 8)) {
                craft.key = key;
                craft.bodies = bodies.clone();
                let marked = crate::marks::designations(&comps, &ptrs);
                craft.comp_ptrs = ptrs;
                craft.built = Arc::default();
                craft.stall.clear();
                craft.warned = false;
                spawn_build(lib.clone(), comps, marked, key, craft.built.clone(), id);
            }
        }
        let Some((bkey, model, main, des_name)) = craft.built.lock().unwrap_or_else(|p| p.into_inner()).clone() else {
            continue;
        };
        craft.designations = des_name;
        if bkey != key {
            continue;
        }
        // Live state of every body; a body that is not simulated keeps its index but no forces.
        let mut live: Vec<Option<(LiveBody, usize, f64)>> = bodies.iter().map(|&b| live_body(b, client)).collect();
        if client {
            // The host's poses arrive every few ticks and Bullet's velocity is zero between them: the motion
            // is the change between the last two poses, held until the next one.
            for (&b, l) in bodies.iter().zip(&mut live) {
                let Some((lb, _, _)) = l else { continue };
                let stamp = read_u32_at(b + OFF_BODY_NET_STAMP);
                let e = net.entry(b).or_insert((stamp, lb.com, lb.rot, V3::ZERO, V3::ZERO));
                if lb.com != e.1 || lb.rot != e.2 {
                    let ticks = stamp.wrapping_sub(e.0);
                    if (1..=120).contains(&ticks) {
                        let (v, w) = crate::model::pose_motion(e.1, &e.2, lb.com, &lb.rot, ticks as f64 / 60.0);
                        // A jump (respawn, teleport) is not motion.
                        (e.3, e.4) = if v.len() < 2000.0 && w.len() < 60.0 { (v, w) } else { (V3::ZERO, V3::ZERO) };
                    }
                    (e.0, e.1, e.2) = (stamp, lb.com, lb.rot);
                }
                (lb.vel, lb.omega) = (e.3, e.4);
            }
        }
        let Some((mb, _, _)) = live.get(main).copied().flatten() else {
            continue;
        };
        let com = [mb.com.x, mb.com.y, mb.com.z];
        if com == craft.last_com && !client {
            continue; // physics has not stepped since our last forces (second sync call)
        }
        // Frame check: the model's centre of mass must land near Bullet's.
        let off = (mb.to_world(model.com) - mb.com).len();
        if off > 5.0 {
            if !craft.warned {
                craft.warned = true;
                log_line(&format!("flight: vehicle {id}: model CoM lands {off:.1} m from the physics CoM; forces not applied (frame mismatch)"));
            }
            continue;
        }
        if !craft.warned {
            // Once per model: where each body's parts land vs its physics CoM (frame check).
            craft.warned = true;
            let mut lines = Vec::new();
            for (bi, l) in live.iter().enumerate() {
                let Some((lb, _, mass)) = l else { continue };
                let pts: Vec<V3> = model.comps.iter().filter(|c| c.body as usize == bi).map(|c| crate::body::aero_pos(c.pos)).collect();
                if pts.is_empty() {
                    continue;
                }
                let c = pts.iter().fold(V3::ZERO, |a, p| a + *p) * (1.0 / pts.len() as f64);
                lines.push(format!("b{bi} {} parts {mass:.0} kg: parts centre {:.2} m from CoM", pts.len(), (lb.to_world(c) - lb.com).len()));
            }
            log_line(&format!("flight: vehicle {id}: main body {main}, model CoM {off:.2} m from physics CoM; {}", lines.join("; ")));
            // Arrow frame check: the body render matrix must place the model CoM where physics does.
            let r = body_render_matrix(bodies[main]).unwrap_or([0.0; 16]);
            let g = mb.world_to_grid(mb.to_world(model.com));
            let p =
                v3(g.x * r[0] + g.y * r[4] + g.z * r[8] + r[12], g.x * r[1] + g.y * r[5] + g.z * r[9] + r[13], g.x * r[2] + g.y * r[6] + g.z * r[10] + r[14]);
            log_line(&format!(
                "flight: vehicle {id}: render matrix places the CoM {:.2} m from physics (arrows fall back to tick positions above 20 m)",
                (p - mb.to_world(model.com)).len()
            ));
        }
        craft.last_com = com;
        stepped = true;
        if craft.stall.len() != model.panels.len() {
            craft.stall = model.fresh_stall();
        }
        let filler = mb;
        let lbs: Vec<LiveBody> = live.iter().map(|l| l.map_or(filler, |x| x.0)).collect();
        let total_mass: f64 = live.iter().flatten().map(|l| l.2).sum();
        // Tank fluid adds weight but not inertia: the game applies it as a force.
        let weight_mass = total_mass + bodies.iter().zip(&live).filter_map(|(&b, l)| l.map(|l| fluid_mass(b, l.2))).sum::<f64>();
        // Safety net against a blow-up: no single force above 60 g of the whole vehicle.
        let fmax = 60.0 * 9.81 * total_mass.max(1.0);
        let rbs: HashSet<usize> = live.iter().flatten().map(|l| l.1).collect();
        let jet_fill = fill_jets(&rbs, apply_force);
        let (ground, wing_height) = if craft.ground.unwrap_or_else(|| GROUND.load(Ordering::Relaxed)) {
            ground_factors(physics_scene, scene, &model, &lbs, main, &bodies)
        } else {
            (Vec::new(), None)
        };
        let deflect: Vec<f64> =
            model.fin_live.iter().map(|f| f.and_then(|f| Some(f.sign * live_angle(*craft.comp_ptrs.get(f.comp)?, f.src)?)).unwrap_or(0.0)).collect();
        // Propeller slipstreams, from each one's inflow as of the last tick.
        craft.prop_v.resize(model.props.len(), 0.0);
        let slips: Vec<Option<crate::model::Slipstream>> = (0..model.props.len())
            .map(|k| {
                let u = &model.props[k];
                let lb = live.get(u.body as usize).copied().flatten()?.0;
                let (s, linked, _) = prop_live_state(*craft.comp_ptrs.get(u.comp)?, u.def.rotor())?;
                (props_on && linked && lb.to_world(u.hub).y > 0.0).then(|| model.slipstream(k, &lb, &s, craft.prop_v[k])).flatten()
            })
            .collect();
        let washes: Vec<crate::model::Slipstream> = slips.iter().flatten().copied().collect();
        let mut forces = model.live_forces(&lbs, main, &mut craft.stall, &ground, &deflect, &washes);
        // Wings and fuselage only: the flight data's lift, drag and coefficients leave out rotor thrust.
        let n_aero = forces.len();
        // Propellers and rotors: blade-element forces at each one's live speed and pitch.
        craft.prop_v.resize(model.props.len(), 0.0);
        let step = STEP.load(Ordering::Relaxed);
        let vrs = craft.vrs.unwrap_or_else(|| VRS.load(Ordering::Relaxed));
        let mut prop_out: Vec<(usize, crate::model::PropOut, f64)> = Vec::new();
        if !props_on {
            // Switched off: hand thrust and shaft load back to the game this tick, not when the entries age out.
            let mut loads = prop_loads();
            model.props.iter().filter_map(|u| craft.comp_ptrs.get(u.comp)).for_each(|p| {
                loads.remove(p);
            });
        }
        for (k, u) in model.props.iter().enumerate().filter(|_| props_on) {
            let Some(&ptr) = craft.comp_ptrs.get(u.comp) else {
                continue;
            };
            let Some(lb) = live.get(u.body as usize).copied().flatten().map(|l| l.0) else {
                continue;
            };
            let state = prop_live_state(ptr, u.def.rotor()).map(|(s, linked, hinged)| {
                let per_nm = model.prop_load(k, s.blades);
                (s, linked, hinged, per_nm)
            });
            // Under water (air props only push in air) or folded: the game's own behaviour.
            let Some((s, linked, _, per_nm)) = state.filter(|_| lb.to_world(u.hub).y > 0.0) else {
                prop_loads().remove(&ptr);
                continue;
            };
            let out = model.prop_live(k, &lb, &s, craft.prop_v[k], vrs);
            if !(out.force.len().is_finite() && out.moment.len().is_finite() && out.v_target.is_finite()) {
                continue;
            }
            craft.prop_v[k] += (craft.dt / out.tau).min(1.0) * (out.v_target - craft.prop_v[k]);
            if cfg!(feature = "logging") && step % 60 == 0 {
                let at = lb.to_world(u.hub);
                let axis = lb.dir_to_world(u.axis).norm() * if s.reversed { -1.0 } else { 1.0 };
                let fwd = (lb.vel + lb.omega.cross(at - lb.com) - lb.wind).dot(axis);
                log_line(&format!(
                    "prop {k}: shaft {:.2} blade {:.2} rev/s, input {:.2} angle {:.1} deg, axial speed {fwd:.1} m/s, inflow {:.1}/{:.1} m/s, thrust {:.0} N (along nose {:.0}), torque {:.0} Nm, power {:.0} kW, stalled {:.2}, shaft drop {:.3}/tick, rev {} flip {} linked {linked}",
                    s.w,
                    out.omega / (2.0 * std::f64::consts::PI),
                    s.pitch,
                    out.blade_angle.to_degrees(),
                    craft.prop_v[k],
                    out.v_target,
                    out.thrust,
                    out.force.dot(mb.dir_to_world(v3(1.0, 0.0, 0.0))),
                    out.torque,
                    out.torque * out.omega / 1000.0,
                    out.stalled,
                    per_nm * out.torque,
                    s.reversed,
                    s.flip
                ));
            }
            prop_loads().insert(ptr, PropLoad { torque: out.torque, sign: if s.w < 0.0 { -1.0 } else { 1.0 }, per_nm, step });
            // The air's torque on the spin axis reaches the airframe through a driven shaft (the
            // engine or hinge motor holds it); a free rotor just changes speed. A hinge motor only
            // holds the hub's speed: the air never loads the hub, so it reacts with nothing itself.
            // A driven propeller's slipstream swirl hands part of that torque back through the wing and tail.
            let torque = if linked { out.moment + slips[k].map_or(V3::ZERO, |sl| model.swirl_recovery(&sl, &lbs, &out)) } else { out.moment - out.axial };
            forces.push(crate::model::LiveForce { body: u.body as usize, force: out.force, at: out.at, torque });
            prop_out.push((k, out, s.w));
        }
        // Thrust = what accelerates the vehicle besides our air forces and gravity (engines, props;
        // also ground contact while rolling), along the nose.
        if let Some(pc) = craft.prev_com.filter(|_| !client) {
            craft.dt = crate::model::measure_step(craft.dt, mb.com - pc, mb.vel);
        }
        craft.prev_com = Some(mb.com);
        let vcom = live.iter().flatten().fold(V3::ZERO, |a, l| a + l.0.vel * l.2) * (1.0 / total_mass.max(1e-9));
        if let Some(pv) = craft.prev_vel {
            let other = (vcom - pv) * (total_mass / craft.dt) - craft.prev_aero - v3(0.0, -9.81 * weight_mass, 0.0);
            let raw = other.dot(mb.dir_to_world(v3(1.0, 0.0, 0.0)));
            if raw.is_finite() {
                craft.thrust += (raw - craft.thrust) * 0.05;
            }
        }
        craft.prev_vel = Some(vcom);
        craft.prev_aero = forces.iter().map(|f| if f.force.len() > fmax { f.force * (fmax / f.force.len()) } else { f.force }).fold(V3::ZERO, |a, f| a + f);
        if want_debug {
            // Missile fins: their own mesh, not their cells (that would paint the body).
            for (j, _) in craft.stall.iter().enumerate().filter(|(_, &st)| st > 0.5) {
                let part = model
                    .fin_comp
                    .get(j)
                    .copied()
                    .flatten()
                    .map(|c| c as usize)
                    .or_else(|| model.fin_live.get(j).copied().flatten().filter(|f| f.src == LiveSrc::Rudder).map(|f| f.comp));
                if let Some(&ptr) = part.and_then(|c| craft.comp_ptrs.get(c)) {
                    stalled_parts.insert(ptr, crate::paint::rgba(crate::paint::RED));
                }
            }
            for (p, _) in model.panels.iter().zip(&craft.stall).zip(&model.fin_comp).filter(|((_, &st), f)| st > 0.5 && f.is_none()).map(|(x, _)| x) {
                let body = bodies.get(model.panel_body(p).min(bodies.len().saturating_sub(1))).copied().unwrap_or(0);
                stalled.entry(body).or_default().extend(p.cells.iter().map(|&c| model.grid.cell(c)));
            }
            let mut d = flight_data(id, &craft.designations, &model, &lbs, &bodies, main, &forces[..n_aero], &craft.stall, weight_mass);
            let nose = mb.dir_to_world(v3(1.0, 0.0, 0.0));
            let (mut ef, mut ecalls, mut bits, mut wp, mut w) = live
                .iter()
                .flatten()
                .filter_map(|l| engine_forces.get(&l.1))
                .fold((V3::ZERO, 0, 0u8, V3::ZERO, 0.0), |a, e| (a.0 + e.0, a.1 + e.1, a.2 | e.2, a.3 + e.3, a.4 + e.4));
            if jet_fill.1 > 0 {
                // Jets the plugin kept pushing this tick.
                (ef, ecalls, bits, wp, w) = (ef + jet_fill.0, ecalls + jet_fill.1, bits | 1, wp + jet_fill.2, w + jet_fill.3);
            }
            for (k, o, shaft) in &prop_out {
                let along = o.force.len();
                (ef, ecalls, bits, wp, w) = (ef + o.force, ecalls + 1, bits | 4, wp + o.at * along, w + along);
                let pb = (model.props[*k].body as usize).min(lbs.len() - 1);
                let plb = &lbs[pb];
                d.props.push(PropDebug {
                    name: model.comps[model.props[*k].comp].def.clone(),
                    rps: o.omega / (2.0 * std::f64::consts::PI),
                    shaft: *shaft,
                    thrust: o.thrust,
                    power: o.torque * o.omega,
                    vrs: o.vrs,
                    blade_deg: o.blade_angle.to_degrees(),
                    stalled: o.stalled,
                    body: bodies.get(pb).copied().unwrap_or(0),
                    hub: (o.at, plb.world_to_grid(o.at)),
                    force: (o.force, plb.world_dir_to_grid(o.force)),
                    wash: slips[*k].map(|sl| {
                        let flux = crate::model::atmosphere(o.at.y).rho * (sl.v0 + sl.vi) * std::f64::consts::PI * sl.radius * sl.radius;
                        ((sl.axis, plb.world_dir_to_grid(sl.axis)), [sl.radius, sl.v0, sl.vi, o.spin * o.torque.max(0.0) / flux])
                    }),
                });
            }
            if w > 1.0 {
                let at = wp * (1.0 / w);
                d.thrust_at = Some(((at, mb.world_to_grid(at)), ef));
            }
            // Thrust sources exactly; else the acceleration estimate.
            (d.thrust_exact, d.thrust_source) = if ecalls > 0 {
                let names: Vec<&str> = THRUST_SOURCE_NAMES.iter().enumerate().filter(|(i, _)| bits & (1 << i) != 0).map(|(_, n)| *n).collect();
                (Some(ef.dot(nose)), names.join(", "))
            } else {
                (None, "est. from acceleration".into())
            };
            static LAST_LOG: AtomicU64 = AtomicU64::new(0);
            let now = crate::editor::now_ms();
            if now.saturating_sub(LAST_LOG.load(Ordering::Relaxed)) > 10_000 {
                LAST_LOG.store(now, Ordering::Relaxed);
                log_line(&format!(
                    "thrust: vehicle {id}: {ecalls} engine force calls last step ({}), {:.0} N along nose; weight {:.0} N (tank fluid {:.0} kg), estimate {:.0} N",
                    d.thrust_source,
                    ef.dot(nose),
                    weight_mass * 9.81,
                    weight_mass - total_mass,
                    craft.thrust
                ));
            }
            d.thrust = craft.thrust;
            // Live CG (the game's own centre of mass; tank fluid is not in it, the game hangs its weight on
            // the tanks as a force) and centre of lift (neutral
            // point at this Mach), with their main-body grid positions so the overlay can place them smoothly.
            let cg = live.iter().flatten().fold(V3::ZERO, |a, l| a + l.0.com * l.2) * (1.0 / total_mass.max(1e-9));
            d.main_body = bodies[main];
            d.cg = Some((cg, mb.world_to_grid(cg)));
            if let Some(np) = model.neutral_point(d.mach.min(0.95), d.alt) {
                let w = mb.to_world(np);
                d.col = Some((w, mb.world_to_grid(w)));
                d.margin = Some((cg - w).dot(nose));
            }
            d.mac = model.mac_ref;
            d.ground = wing_height.map(|h| (h, crate::model::ground_effect(h, model.b_ref)));
            d.step = craft.dt;
            debug.push(d);
        }
        // Every air load goes to the main body at its true point: same net force and moment on the
        // vehicle, but hinges and pivots no longer carry it, so sub-grid surfaces don't bend.
        // shortcut: assumes the vehicle's bodies are rigidly joined; per-body forces if loose parts matter.
        let Some((lb, rb, _)) = live.get(main).copied().flatten().filter(|_| !client) else {
            continue; // a client only shows the flight data: the host applies the forces
        };
        for f in forces {
            let scale = if f.force.len() > fmax { fmax / f.force.len() } else { 1.0 };
            let force = f.force * scale;
            let rel = f.at - lb.com;
            if !(force.len().is_finite() && rel.len().is_finite() && f.torque.len().is_finite()) {
                continue;
            }
            let fv = [force.x, force.y, force.z, 0.0];
            let rv = [rel.x, rel.y, rel.z, 0.0];
            apply_force(rb, fv.as_ptr(), rv.as_ptr());
            if f.torque != V3::ZERO {
                let t = f.torque * scale;
                for (k, c) in [t.x, t.y, t.z].into_iter().enumerate() {
                    let at = (rb + OFF_RB_TOTAL_TORQUE + 8 * k) as *mut f64;
                    at.write_unaligned(at.read_unaligned() + c * f64_at(rb + OFF_RB_ANGULAR_FACTOR + 8 * k));
                }
            }
        }
        APPLIED.fetch_add(1, Ordering::Relaxed);
    }
    *FLOWN.write().unwrap_or_else(|p| p.into_inner()) = flown;
    if net.len() > 4096 {
        net.clear(); // bodies of despawned vehicles
    }
    st.net = net;
    st.crafts.retain(|id, _| seen.contains(id));
    VEHICLES.store(st.crafts.len() as u64, Ordering::Relaxed);
    // Empty while F9 is closed, so the render thread stops painting; kept on a repeat sync call.
    if stepped || !want_debug || st.crafts.is_empty() {
        crate::paint::publish(stalled);
        crate::paint::publish_flight_tint(std::mem::take(&mut stalled_parts));
    }
    if want_debug {
        *DEBUG.lock().unwrap_or_else(|p| p.into_inner()) = debug;
        record_builds(st.crafts.values().flat_map(|c| c.bodies.iter().copied()));
    }
}

/// Same thread as the game's frame build, so `frame+0xED0` and `body+0x308` are from one build.
fn record_builds(bodies: impl Iterator<Item = usize>) {
    let Some(ts) = frame_build_ts(SIM_STATE.load(Ordering::Relaxed)) else {
        return;
    };
    let mut map = BUILT_HISTORY.lock().unwrap_or_else(|p| p.into_inner());
    let mut keep = HashSet::new();
    for b in bodies {
        keep.insert(b);
        let h = map.entry(b).or_default();
        if h.last().is_some_and(|(t, _)| *t == ts) {
            continue;
        }
        let Some(m) = body_built_matrix(b) else {
            continue;
        };
        h.push((ts, m));
        if h.len() > 4 {
            h.remove(0);
        }
    }
    map.retain(|b, _| keep.contains(b));
}

/// Airflow, coefficients and lift/drag arrows (fuselage + one per whole wing) from this tick's forces.
#[allow(clippy::too_many_arguments)]
fn flight_data(
    id: u32,
    des: &str,
    m: &Model,
    lbs: &[LiveBody],
    bodies: &[usize],
    main: usize,
    forces: &[crate::model::LiveForce],
    stall: &[f64],
    mass: f64,
) -> VehicleDebug {
    let mb = &lbs[main];
    let com = mb.to_world(m.com);
    let v = mb.vel + mb.omega.cross(com - mb.com) - mb.wind;
    let speed = v.len();
    let at = crate::model::atmosphere(com.y);
    let q = 0.5 * at.rho * speed * speed;
    let va = mb.dir_to_aero(v);
    let d = if speed > 1e-6 { v * (1.0 / speed) } else { V3::ZERO };
    let split = |f: V3| {
        let drag = d * f.dot(d);
        (f - drag, drag)
    };
    // Group: 0 = fuselage, 1 + wing id per panel (forces[1 + i] is panel i).
    let mut groups: HashMap<usize, (V3, V3, f64, usize, f64)> = HashMap::new(); // force, Σ|F|·at, Σ|F|, body, best area
    let (mut lift, mut drag) = (V3::ZERO, V3::ZERO);
    for (i, f) in forces.iter().enumerate() {
        let (l, dr) = split(f.force);
        lift += l;
        drag += dr;
        let (key, area) =
            if i == 0 { (0, f64::MAX) } else { (1 + m.wing_of.get(i - 1).copied().unwrap_or(i - 1), m.panels.get(i - 1).map_or(0.0, |p| p.area)) };
        let g = groups.entry(key).or_insert((V3::ZERO, V3::ZERO, 0.0, f.body, -1.0));
        let w = f.force.len().max(1e-6);
        g.0 += f.force;
        g.1 += f.at * w;
        g.2 += w;
        if area > g.4 {
            (g.3, g.4) = (f.body, area); // the surface rides on its biggest panel's body
        }
    }
    let mut arrows: Vec<Arrow> = groups
        .into_iter()
        .map(|(key, (force, sum_at, w, b, _))| {
            let at = sum_at * (1.0 / w);
            let (l, dr) = split(force);
            let lb = &lbs[b.min(lbs.len() - 1)];
            Arrow {
                at,
                lift: l,
                drag: dr,
                body: bodies.get(b).copied().unwrap_or(0),
                local_at: lb.world_to_grid(at),
                local_lift: lb.world_dir_to_grid(l),
                local_drag: lb.world_dir_to_grid(dr),
                wing: key > 0,
            }
        })
        .collect();
    arrows.sort_by_key(|a| a.wing);
    // Lift is positive toward the vehicle's up (aero −z).
    let up = mb.dir_to_world(v3(0.0, 0.0, -1.0));
    let qs = (q * m.s_ref).max(1e-9);
    // Centre of lift (KSP's CoL): where the live lift acts, so it moves with flaps, AoA and stall.
    let (wl, wlp) = forces.iter().fold((0.0, V3::ZERO), |a, f| {
        let l = split(f.force).0.dot(up);
        (a.0 + l, a.1 + f.at * l)
    });
    let lift_at = (wl.abs() > 0.02 * mass * 9.81).then(|| {
        let p = wlp * (1.0 / wl);
        (p, mb.world_to_grid(p))
    });
    VehicleDebug {
        lift_at,
        id,
        designations: des.to_string(),
        airspeed: speed,
        mach: speed / at.a,
        alt: com.y,
        q,
        rho: at.rho,
        aoa: va.z.atan2(va.x),
        beta: (va.y / speed.max(1e-9)).clamp(-1.0, 1.0).asin(),
        cl: lift.dot(up).signum() * lift.len() / qs,
        cd: -drag.dot(d) / qs,
        lift: lift.len(),
        drag: drag.len(),
        weight: mass * 9.81,
        mass,
        thrust: 0.0,
        thrust_exact: None,
        thrust_source: String::new(),
        ground: None,
        main_body: 0,
        cg: None,
        col: None,
        margin: None,
        mac: 0.0,
        thrust_at: None,
        step: 0.0,
        com,
        arrows,
        stalled: stall.iter().filter(|&&s| s > 0.5).count(),
        panels: m.panels.len(),
        props: Vec::new(),
    }
}

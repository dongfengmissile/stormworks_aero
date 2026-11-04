//! Live workbench vehicle: read on the game thread from the editor update hook and
//! published to the overlay as an immutable snapshot whenever it changes.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable, RegionCacheScope, PTR_MAX, PTR_MIN};
use crate::offsets::*;
use crate::vehicle::{transform_ok, Comp};

pub(crate) struct Snapshot {
    pub gen: u64,
    pub comps: Vec<Comp>,
}

/// Latest snapshot (None until the editor has been seen with the window open).
pub(crate) static LATEST: Mutex<Option<Arc<Snapshot>>> = Mutex::new(None);
/// Set by the overlay while its window is open; reading is skipped otherwise.
pub(crate) static WANTED: AtomicBool = AtomicBool::new(false);
/// Set by the overlay while the plugin model is drawn in the workbench view.
pub(crate) static WORLD_WANTED: AtomicBool = AtomicBool::new(false);
/// Body frame (aero axes, metres) -> OpenGL clip space of the editor camera, row vectors.
pub(crate) static WORLD_CLIP: Mutex<Option<[f64; 16]>> = Mutex::new(None);

/// Milliseconds (since bootstrap) of the last editor tick: the editor is open if recent.
pub(crate) static LAST_TICK_MS: AtomicU64 = AtomicU64::new(0);
pub(crate) static TICKS: AtomicU64 = AtomicU64::new(0);
pub(crate) static SNAPSHOTS: AtomicU64 = AtomicU64::new(0);
pub(crate) static REJECTS: AtomicU64 = AtomicU64::new(0);

pub(crate) fn now_ms() -> u64 {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    START.get_or_init(std::time::Instant::now).elapsed().as_millis() as u64
}

pub(crate) fn editor_active() -> bool {
    let last = LAST_TICK_MS.load(Ordering::Relaxed);
    last != 0 && now_ms().saturating_sub(last) < 1500
}

pub(crate) fn ptr_ok(p: usize, len: usize) -> bool {
    (PTR_MIN..PTR_MAX).contains(&p) && p % 8 == 0 && readable(p, len)
}

/// Ring buffer `{items, mod, offset, count}` at `at`: the element pointers, or None if implausible.
pub(crate) unsafe fn ring(at: usize, max: u32) -> Option<Vec<usize>> {
    if !ptr_ok(at, 0x18) {
        return None;
    }
    let items = read_usize_at(at);
    let (modulo, offset, count) = (read_u32_at(at + 8), read_u32_at(at + 0xC), read_u32_at(at + 0x10));
    if count == 0 {
        return Some(Vec::new());
    }
    if count > modulo || modulo > max || !ptr_ok(items, modulo as usize * 8) {
        return None;
    }
    Some((0..count).map(|i| read_usize_at(items + ((i + offset) % modulo) as usize * 8)).collect())
}

/// One pass over the editor vehicle: (def*, pos, rot, body) per component.
unsafe fn read_raw(editor: usize) -> Option<Vec<(usize, [i32; 3], [i32; 9], u32)>> {
    read_vehicle_raw(read_usize_at(editor + OFF_EDITOR_VEHICLE)).map(|(raw, _)| raw)
}

/// Body pointers of a vehicle, in the order `Comp::body` indexes them.
pub(crate) unsafe fn vehicle_bodies(vehicle: usize) -> Option<Vec<usize>> {
    if !ptr_ok(vehicle, 0x408) {
        return None;
    }
    let bodies = ring(vehicle + OFF_VEHICLE_BODIES, 4096)?;
    bodies.iter().all(|&b| ptr_ok(b, 0x408)).then_some(bodies)
}

/// Component count of a body (cheap change test).
pub(crate) unsafe fn body_comp_count(body: usize) -> u32 {
    read_u32_at(body + OFF_BODY_COMPONENTS + 0x10)
}

/// Every component of a vehicle with its definition name, and the body pointers.
pub(crate) unsafe fn vehicle_comps(vehicle: usize) -> Option<(Vec<Comp>, Vec<usize>)> {
    let (raw, bodies) = read_vehicle_raw(vehicle)?;
    let mut game = GAME.lock().unwrap_or_else(|p| p.into_inner());
    let mut comps = Vec::with_capacity(raw.len());
    for (def, pos, rot, body) in raw {
        let name = match game.names.get(&def) {
            Some(n) => n.clone(),
            None => {
                let n = def_name(def)?;
                game.names.insert(def, n.clone());
                n
            }
        };
        comps.push(Comp { def: name, pos, rot, body });
    }
    Some((comps, bodies))
}

unsafe fn read_vehicle_raw(vehicle: usize) -> Option<(Vec<(usize, [i32; 3], [i32; 9], u32)>, Vec<usize>)> {
    let bodies = vehicle_bodies(vehicle)?;
    let mut out = Vec::new();
    for (bi, &body) in bodies.iter().enumerate() {
        for comp in ring(body + OFF_BODY_COMPONENTS, 1 << 20)? {
            if !ptr_ok(comp, OFF_COMP_DEF + 8) {
                return None;
            }
            let p = |o: usize| read_u32_at(comp + o) as i32;
            let pos = [p(OFF_COMP_POS), p(OFF_COMP_POS + 4), p(OFF_COMP_POS + 8)];
            let rot: [i32; 9] = std::array::from_fn(|k| p(OFF_COMP_ROT + k * 4));
            let def = read_usize_at(comp + OFF_COMP_DEF);
            let rot = if transform_ok(&rot) { rot } else { crate::vehicle::IDENTITY };
            let mirror = if ptr_ok(def, OFF_DEF_MIRROR + 4) { read_u32_at(def + OFF_DEF_MIRROR) as i32 & 7 } else { 0 };
            out.push((def, pos, crate::vehicle::flipped(rot, mirror), bi as u32));
        }
    }
    Some((out, bodies))
}

unsafe fn def_name(def: usize) -> Option<String> {
    if !ptr_ok(def, OFF_DEF_NAME + 0x10) {
        return None;
    }
    let data = read_usize_at(def + OFF_DEF_NAME);
    let len = read_u32_at(def + OFF_DEF_NAME + 8) as usize;
    if len == 0 || len > 256 || !(PTR_MIN..PTR_MAX).contains(&data) || !readable(data, len) {
        return None;
    }
    let bytes = std::slice::from_raw_parts(data as *const u8, len);
    std::str::from_utf8(bytes).ok().map(str::to_string)
}

struct GameThread {
    last_hash: u64,
    names: HashMap<usize, String>,
}

static GAME: std::sync::LazyLock<Mutex<GameThread>> =
    std::sync::LazyLock::new(|| Mutex::new(GameThread { last_hash: 0, names: HashMap::new() }));

fn mul4(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    std::array::from_fn(|k| (0..4).map(|j| a[k / 4 * 4 + j] * b[j * 4 + k % 4]).sum())
}

/// Clip matrix of the editor camera for points in the body frame of the analysis.
unsafe fn read_clip(editor: usize) -> Option<[f64; 16]> {
    let f32s = |at: usize| -> [f64; 16] { std::array::from_fn(|k| f32::from_bits(read_u32_at(at + 4 * k)) as f64) };
    let cam = editor + OFF_EDITOR_CAMERA;
    if !ptr_ok(cam, OFF_CAM_PROJ + 64) {
        return None;
    }
    let (view, proj) = (f32s(cam + OFF_CAM_VIEW), f32s(cam + OFF_CAM_PROJ));
    let vehicle = read_usize_at(editor + OFF_EDITOR_VEHICLE);
    if !ptr_ok(vehicle, 0x408) {
        return None;
    }
    let veh: [f64; 16] = std::array::from_fn(|k| f64::from_bits(read_usize_at(vehicle + OFF_VEHICLE_WORLD + 8 * k) as u64));
    // shortcut: every body is drawn with the first body's transform (one shared grid in the editor), per-body if sub-grids drift.
    let body = *ring(vehicle + OFF_VEHICLE_BODIES, 4096)?.first()?;
    if !ptr_ok(body, OFF_BODY_MATRIX + 64) {
        return None;
    }
    let body = f32s(body + OFF_BODY_MATRIX);
    // Aero body frame (x fwd, y right, z down) -> vehicle grid metres (x, y up, z fwd).
    let aero: [f64; 16] = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let m = mul4(&mul4(&mul4(&mul4(&aero, &body), &veh), &view), &proj);
    // A perspective projection has -1 in [11]; anything else is not the camera we expect.
    ((proj[11] + 1.0).abs() < 1e-3 && m.iter().all(|v| v.is_finite())).then_some(m)
}

/// Called after every editor tick, on the game thread.
pub(crate) unsafe fn on_tick(editor: usize) {
    LAST_TICK_MS.store(now_ms().max(1), Ordering::Relaxed);
    let tick = TICKS.fetch_add(1, Ordering::Relaxed);
    if WORLD_WANTED.load(Ordering::Relaxed) {
        let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
        let _cache = RegionCacheScope::new();
        let clip = read_clip(editor);
        if clip.is_none() && REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
            log_line(&format!("editor: camera/vehicle transform rejected (editor {editor:#x})"));
        }
        *WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner()) = clip;
    }
    if !WANTED.load(Ordering::Relaxed) || tick % 10 != 0 {
        return;
    }
    let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
    let _cache = RegionCacheScope::new();
    let Some(raw) = read_raw(editor) else {
        if REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
            log_line(&format!("editor: vehicle walk rejected (editor {editor:#x}, vehicle {:#x})", read_usize_at(editor + OFF_EDITOR_VEHICLE)));
        }
        return;
    };
    // Cheap change test before touching any name: FNV over (def, pos, rot, body).
    let mut h: u64 = 0xcbf29ce484222325;
    let mut mix = |v: u64| h = (h ^ v).wrapping_mul(0x100000001b3);
    for (def, pos, rot, body) in &raw {
        mix(*def as u64);
        pos.iter().chain(rot.iter()).for_each(|v| mix(*v as u32 as u64));
        mix(*body as u64);
    }
    mix(raw.len() as u64);
    let mut game = GAME.lock().unwrap_or_else(|p| p.into_inner());
    if h == game.last_hash {
        return;
    }
    let mut comps = Vec::with_capacity(raw.len());
    for (def, pos, rot, body) in raw {
        let name = match game.names.get(&def) {
            Some(n) => n.clone(),
            None => {
                let Some(n) = def_name(def) else {
                    if REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
                        // Evidence for re-checking OFF_DEF_NAME after a game update.
                        let dump = |o: usize| if readable(def + o, 16) { format!("{:02x?}", std::slice::from_raw_parts((def + o) as *const u8, 16)) } else { "unreadable".into() };
                        log_line(&format!("editor: no definition name at def {def:#x}: +0x288 {} +0x298 {}", dump(OFF_DEF_NAME), dump(0x298)));
                    }
                    return;
                };
                game.names.insert(def, n.clone());
                n
            }
        };
        comps.push(Comp { def: name, pos, rot, body });
    }
    game.last_hash = h;
    drop(game);
    let gen = SNAPSHOTS.fetch_add(1, Ordering::Relaxed) + 1;
    if gen <= 3 || gen % 100 == 0 {
        let sample: Vec<String> = comps.iter().take(4).map(|c| format!("{}@{:?}", c.def, c.pos)).collect();
        log_line(&format!("editor snapshot #{gen}: {} components in {} bodies, e.g. {}", comps.len(), comps.iter().map(|c| c.body).max().map_or(0, |b| b + 1), sample.join(", ")));
    }
    *LATEST.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(Snapshot { gen, comps }));
}

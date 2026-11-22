//! Live workbench vehicle: read on the game thread from the editor update hook and
//! published to the overlay as an immutable snapshot whenever it changes.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable, RegionCacheScope, PTR_MAX, PTR_MIN};
use crate::offsets::*;
use crate::vehicle::{transform_ok, Comp};

pub struct Snapshot {
    pub gen: u64,
    pub comps: Vec<Comp>,
    /// Component pointers, parallel to `comps` (keys of `marks`).
    pub ptrs: Vec<usize>,
}

/// Latest snapshot (None until the editor has been seen with the window open).
pub static LATEST: Mutex<Option<Arc<Snapshot>>> = Mutex::new(None);
/// Set by the overlay while its window is open; reading is skipped otherwise.
pub static WANTED: AtomicBool = AtomicBool::new(false);
/// Set by the overlay while it picks blocks on the workbench vehicle with the cursor.
pub static PICK_WANTED: AtomicBool = AtomicBool::new(false);
/// Body frame (aero axes, metres) -> OpenGL clip space of the editor camera, row vectors.
pub static WORLD_CLIP: Mutex<Option<[f64; 16]>> = Mutex::new(None);
/// Workbench paint (Aero wings mode), set by the overlay: a key that changes with the colours, voxel
/// colours per body (as `Comp::body` numbers them) and moving-mesh tints by component position. None = off.
pub static PAINT: Mutex<Option<(u64, Vec<Arc<crate::paint::VoxelColours>>, Arc<Vec<([i32; 3], u32)>>)>> = Mutex::new(None);
/// Paint key the tints were last resolved to component pointers for.
static TINT_KEY: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Milliseconds (since bootstrap) of the last editor tick: the editor is open if recent.
pub static LAST_TICK_MS: AtomicU64 = AtomicU64::new(0);
pub static TICKS: AtomicU64 = AtomicU64::new(0);
pub static SNAPSHOTS: AtomicU64 = AtomicU64::new(0);
pub static REJECTS: AtomicU64 = AtomicU64::new(0);

pub fn now_ms() -> u64 {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    START.get_or_init(std::time::Instant::now).elapsed().as_millis() as u64
}

pub fn editor_active() -> bool {
    let last = LAST_TICK_MS.load(Ordering::Relaxed);
    last != 0 && now_ms().saturating_sub(last) < 1500
}

pub fn ptr_ok(p: usize, len: usize) -> bool {
    (PTR_MIN..PTR_MAX).contains(&p) && p % 8 == 0 && readable(p, len)
}

/// Ring buffer `{items, mod, offset, count}` at `at`: the element pointers, or None if implausible.
pub unsafe fn ring(at: usize, max: u32) -> Option<Vec<usize>> {
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

/// One pass over the editor vehicle: (def*, pos, rot, body) per component, and the component pointers.
pub unsafe fn read_raw(editor: usize) -> Option<(Vec<(usize, [i32; 3], [i32; 9], u32, i32)>, Vec<usize>)> {
    read_vehicle_raw(read_usize_at(editor + OFF_EDITOR_VEHICLE)).map(|(raw, _, ptrs)| (raw, ptrs))
}

/// Body pointers of a vehicle, in the order `Comp::body` indexes them.
pub unsafe fn vehicle_bodies(vehicle: usize) -> Option<Vec<usize>> {
    if !ptr_ok(vehicle, 0x408) {
        return None;
    }
    let bodies = ring(vehicle + OFF_VEHICLE_BODIES, 4096)?;
    bodies.iter().all(|&b| ptr_ok(b, 0x408)).then_some(bodies)
}

/// Component count of a body (cheap change test).
pub unsafe fn body_comp_count(body: usize) -> u32 {
    read_u32_at(body + OFF_BODY_COMPONENTS + 0x10)
}

/// A raw component as a `Comp` (definition names are cached). None: its definition has no readable name.
unsafe fn comp_of(names: &mut HashMap<usize, String>, (def, pos, rot, body, mirror): (usize, [i32; 3], [i32; 9], u32, i32), ptr: usize) -> Option<Comp> {
    let name = match names.get(&def) {
        Some(n) => n.clone(),
        None => {
            let n = def_name(def)?;
            names.insert(def, n.clone());
            n
        }
    };
    let (size, wheel_size) = part_size(ptr, def);
    Some(Comp { def: name, pos, rot, body, mirror, prop: prop_set(ptr, def), size, wheel_size, power: joint_power(ptr, def) })
}

/// Every component of a vehicle with its definition name, and the body pointers.
/// Components (in `Model::comps` order), body pointers, component pointers.
pub unsafe fn vehicle_comps(vehicle: usize) -> Option<(Vec<Comp>, Vec<usize>, Vec<usize>)> {
    let (raw, bodies, ptrs) = read_vehicle_raw(vehicle)?;
    let mut game = GAME.lock().unwrap_or_else(|p| p.into_inner());
    let mut comps = Vec::with_capacity(raw.len());
    for (r, &ptr) in raw.into_iter().zip(&ptrs) {
        comps.push(comp_of(&mut game.names, r, ptr)?);
    }
    Some((comps, bodies, ptrs))
}

/// Live microcontroller width x length and advanced-wheel size setting (knowledge/vehicle-model.md
/// "Per-component size settings").
pub unsafe fn part_size(comp: usize, def: usize) -> (Option<[i32; 2]>, Option<(f64, bool)>) {
    if !ptr_ok(def, OFF_DEF_CLASS + 4) || !ptr_ok(comp, OFF_WHEEL_SIZE + 4) {
        return (None, None);
    }
    match read_u32_at(def + OFF_DEF_CLASS) {
        37 => (Some([read_u32_at(comp + OFF_MC_WIDTH) as i32, read_u32_at(comp + OFF_MC_WIDTH + 4) as i32]), None),
        41 => (None, Some(f32::from_bits(read_u32_at(comp + OFF_WHEEL_SIZE)) as f64).filter(|s| s.is_finite()).map(|s| (s, read_u32_at(comp + OFF_WHEEL_DOUBLE) & 0xFF != 0))),
        _ => (None, None),
    }
}

/// A driven joint's max power setting (`Comp::power`): the parent half of a type-7 part with
/// flag 0x100, its config panel "%" slider (knowledge/editor-ui.md).
pub unsafe fn joint_power(comp: usize, def: usize) -> Option<f64> {
    if !ptr_ok(def, OFF_DEF_PROP_FLAGS + 4) || !ptr_ok(comp, OFF_JOINT_POWER + 4) || read_u32_at(def + OFF_DEF_CLASS) != 7 || read_u32_at(def + OFF_DEF_PROP_FLAGS) & 0x140 != 0x140 {
        return None;
    }
    Some(f32::from_bits(read_u32_at(comp + OFF_JOINT_POWER)) as f64).filter(|s| s.is_finite())
}

/// Live propeller / rotor settings of a type-2 part (knowledge/vehicle-model.md "Type 2 force
/// emitters" Q3): blade count, pitch setting + collective input, blade length.
pub unsafe fn prop_set(comp: usize, def: usize) -> Option<crate::vehicle::PropSet> {
    if !ptr_ok(def, OFF_DEF_CLASS + 4) || read_u32_at(def + OFF_DEF_CLASS) != 2 || !ptr_ok(comp, OFF_PROP_LENGTH + 4) {
        return None;
    }
    let f = |o: usize| f32::from_bits(read_u32_at(comp + o)) as f64;
    let pitch = read_u32_at(comp + OFF_PROP_PITCH) as i32 as f64 + f(OFF_PROP_COLLECTIVE);
    let length = f(OFF_PROP_LENGTH);
    Some(crate::vehicle::PropSet {
        blades: Some(read_u32_at(comp + OFF_PROP_BLADES).clamp(2, 8)),
        pitch: pitch.is_finite().then(|| pitch.clamp(-1.0, 1.0)),
        length: (length.is_finite() && length.abs() < 100.0).then_some(length),
    })
}

unsafe fn read_vehicle_raw(vehicle: usize) -> Option<(Vec<(usize, [i32; 3], [i32; 9], u32, i32)>, Vec<usize>, Vec<usize>)> {
    let bodies = vehicle_bodies(vehicle)?;
    let mut out = Vec::new();
    let mut ptrs = Vec::new();
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
            out.push((def, pos, crate::vehicle::flipped(rot, mirror), bi as u32, mirror));
            ptrs.push(comp);
        }
    }
    Some((out, bodies, ptrs))
}

unsafe fn def_name(def: usize) -> Option<String> {
    def_string(def, OFF_DEF_NAME)
}

/// A string field of a definition (data pointer, then u32 length).
unsafe fn def_string(def: usize, off: usize) -> Option<String> {
    if !ptr_ok(def, off + 0x10) {
        return None;
    }
    let data = read_usize_at(def + off);
    let len = read_u32_at(def + off + 8) as usize;
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

static GAME: std::sync::LazyLock<Mutex<GameThread>> = std::sync::LazyLock::new(|| Mutex::new(GameThread { last_hash: 0, names: HashMap::new() }));

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

/// The editor seen by the last tick (for the part panel, which is not given it).
static LAST_EDITOR: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// A linear track base's travel in the workbench vehicle, metres (`model::track_travel`, the same
/// test as `model::joint_links`); None if `comp` is not one. Game thread.
pub unsafe fn track_travel_of(comp: usize) -> Option<(f64, f64)> {
    let editor = LAST_EDITOR.load(Ordering::Relaxed);
    if !ptr_ok(editor + OFF_EDITOR_VEHICLE, 8) {
        return None;
    }
    let (raw, _, ptrs) = read_vehicle_raw(read_usize_at(editor + OFF_EDITOR_VEHICLE))?;
    let (def, pos, rot, body, mirror) = raw[ptrs.iter().position(|&p| p == comp)?];
    let ext = def_string(def, OFF_DEF_EXTENDER)?;
    let r = crate::vehicle::flipped(rot, mirror);
    let mut at: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for (k, o) in raw.iter().enumerate() {
        at.entry(o.1).or_default().push(k);
    }
    Some(crate::joints::track_travel(pos, [r[6], r[7], r[8]], |p| {
        at.get(&p).is_some_and(|ks| {
            ks.iter().any(|&k| {
                let o = &raw[k];
                o.3 == body && crate::vehicle::flipped(o.2, o.4)[3..6] == r[3..6] && (o.0 == def || def_name(o.0).as_deref() == Some(ext.as_str()))
            })
        })
    }))
}

/// Called after every editor tick, on the game thread.
pub unsafe fn on_tick(editor: usize) {
    LAST_EDITOR.store(editor, Ordering::Relaxed);
    LAST_TICK_MS.store(now_ms().max(1), Ordering::Relaxed);
    let tick = TICKS.fetch_add(1, Ordering::Relaxed);
    if PICK_WANTED.load(Ordering::Relaxed) {
        let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
        let _cache = RegionCacheScope::new();
        let clip = read_clip(editor);
        if clip.is_none() && REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
            log_line(&format!("editor: camera/vehicle transform rejected (editor {editor:#x})"));
        }
        *WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner()) = clip;
    }
    let paint = PAINT.lock().unwrap_or_else(|p| p.into_inner()).clone();
    {
        let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
        let _cache = RegionCacheScope::new();
        let (key, colours, tints) = match paint {
            Some((key, per_body, tints)) => {
                let bodies = vehicle_bodies(read_usize_at(editor + OFF_EDITOR_VEHICLE)).unwrap_or_default();
                (key, bodies.into_iter().zip(per_body).filter(|(_, c)| !c.is_empty()).collect(), tints)
            }
            None => (0, HashMap::new(), Arc::default()),
        };
        crate::paint::publish_workbench(key, &colours);
        // Moving meshes: component pointers of the tinted positions, once per paint change.
        if TINT_KEY.swap(key, Ordering::Relaxed) != key {
            let mut t = HashMap::new();
            if !tints.is_empty() {
                if let Some((raw, _, ptrs)) = read_vehicle_raw(read_usize_at(editor + OFF_EDITOR_VEHICLE)) {
                    let at: HashMap<[i32; 3], u32> = tints.iter().copied().collect();
                    t = raw.iter().zip(ptrs).filter_map(|(r, p)| Some((p, *at.get(&r.1)?))).collect();
                }
            }
            crate::paint::publish_bench_tint(t);
        }
    }
    if !WANTED.load(Ordering::Relaxed) || tick % 10 != 0 {
        return;
    }
    let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
    let _cache = RegionCacheScope::new();
    let Some((raw, ptrs)) = read_raw(editor) else {
        if REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
            log_line(&format!("editor: vehicle walk rejected (editor {editor:#x}, vehicle {:#x})", read_usize_at(editor + OFF_EDITOR_VEHICLE)));
        }
        return;
    };
    // Cheap change test before touching any name: FNV over (def, pos, rot, body).
    let mut h: u64 = 0xcbf29ce484222325;
    let mut mix = |v: u64| h = (h ^ v).wrapping_mul(0x100000001b3);
    for ((def, pos, rot, body, _), &ptr) in raw.iter().zip(&ptrs) {
        if let Some(p) = prop_set(ptr, *def) {
            [p.blades.unwrap_or(0) as u64, p.pitch.unwrap_or(0.0).to_bits(), p.length.unwrap_or(0.0).to_bits()].into_iter().for_each(&mut mix);
        }
        let (size, wheel) = part_size(ptr, *def);
        size.into_iter().flatten().for_each(|v| mix(v as u64));
        wheel.into_iter().for_each(|(s, d)| mix(s.to_bits() ^ d as u64));
        joint_power(ptr, *def).into_iter().for_each(|s| mix(s.to_bits()));
        mix(*def as u64);
        pos.iter().chain(rot.iter()).for_each(|v| mix(*v as u32 as u64));
        mix(*body as u64);
    }
    mix(raw.len() as u64);
    // New parts (paste, undo) and marks the game moved count as changes too.
    ptrs.iter().for_each(|&p| mix(p as u64));
    mix(crate::marks::GEN.load(Ordering::Relaxed));
    let mut game = GAME.lock().unwrap_or_else(|p| p.into_inner());
    if h == game.last_hash {
        return;
    }
    let mut comps = Vec::with_capacity(raw.len());
    for (r, &ptr) in raw.into_iter().zip(&ptrs) {
        let def = r.0;
        let Some(c) = comp_of(&mut game.names, r, ptr) else {
            if REJECTS.fetch_add(1, Ordering::Relaxed) < 3 {
                // Evidence for re-checking OFF_DEF_NAME after a game update.
                let dump = |o: usize| {
                    if readable(def + o, 16) {
                        format!("{:02x?}", std::slice::from_raw_parts((def + o) as *const u8, 16))
                    } else {
                        "unreadable".into()
                    }
                };
                log_line(&format!("editor: no definition name at def {def:#x}: +0x288 {} +0x298 {}", dump(OFF_DEF_NAME), dump(0x298)));
            }
            return;
        };
        comps.push(c);
    }
    game.last_hash = h;
    drop(game);
    let gen = SNAPSHOTS.fetch_add(1, Ordering::Relaxed) + 1;
    if gen <= 3 || gen % 100 == 0 {
        let sample: Vec<String> = comps.iter().take(4).map(|c| format!("{}@{:?}", c.def, c.pos)).collect();
        log_line(&format!(
            "editor snapshot #{gen}: {} components in {} bodies, e.g. {}",
            comps.len(),
            comps.iter().map(|c| c.body).max().map_or(0, |b| b + 1),
            sample.join(", ")
        ));
    }
    *LATEST.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(Snapshot { gen, comps, ptrs }));
}


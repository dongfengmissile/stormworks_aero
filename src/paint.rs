//! Paints chosen voxels in the game's own render: stalled wing voxels red in flight (F9 open,
//! plugin flight on), and wing voxels by role / aerofoil on the workbench vehicle (F8).
//!
//! The game draws every vehicle chunk mesh through `opaque_chunk_draw_item` (0x680C00) with the
//! opaque program current. After the game's own draw we call it again for the index runs of the
//! stalled voxels' triangles, with `is_force_color = 1` and a red `override_color_1`: the same
//! program, matrices, depth, normals and velocity, so the red blocks are lit, shadowed and hidden
//! exactly like the game's. Chunk vertices are body-local voxel·0.25 (the vehicle file frame).
//!
//! Data flow: the flight tick (game thread) maps each stalled body's chunk meshes (IBO id, from the
//! `body+0x540/+0x588` ringvecs as in the infrared plugin) to the body's stalled voxels; the render
//! thread reads each mesh's triangles back once (`glGetBufferSubData`), finds each triangle's voxel
//! and caches the runs of stalled triangles until the stalled set changes.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet};
use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

use windows_sys::Win32::System::LibraryLoader::{GetProcAddress, LoadLibraryA};

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable, PTR_MIN};
use crate::voxel::{colour_runs, triangle_voxels};

pub(crate) type ChunkDraw = unsafe extern "system" fn(
    usize,        // bindset (renderer+0x920)
    usize,        // mesh
    u32,          // first index
    u32,          // index count (0 = whole mesh)
    *const f32,   // mat_world (interpolated)
    *const f32,   // mat_world_prev
    *const f32,   // mat_world_next
    f32,          // light_factor
    u32,          // override_color_1 (RGBA8, R in the low byte)
    u32,          // override_color_2
    u32,          // override_color_3
    u8,           // is_preview
    u8,           // is_force_color
    u8,           // is_disable_color_replace
    usize,        // unused
    u8,           // wireframe pass
);
pub(crate) static ORIGINAL: OnceLock<ChunkDraw> = OnceLock::new();

/// Stall red as [r, g, b] (the shader linearises colours like vertex colours).
const RED: [u8; 3] = [230, 32, 32];

/// Voxel -> colour (RGBA8, R in the low byte) of one body.
pub(crate) type VoxelColours = HashMap<[i32; 3], u32>;

pub(crate) fn rgba(c: [u8; 3]) -> u32 {
    c[0] as u32 | (c[1] as u32) << 8 | (c[2] as u32) << 16 | 0xFF00_0000
}
const MESH_VBO: usize = 0x60;
const MESH_IBO: usize = 0x64;
// Logged in game: +0x54 = 28 (stride), +0x58 = vertex count, +0x5C = index count (multiple of 3).
const MESH_STRIDE: usize = 0x54;
const MESH_VCOUNT: usize = 0x58;
const MESH_ICOUNT: usize = 0x5C;
const BODY_CHUNK_TABLES: [usize; 2] = [0x540, 0x588];
const MAX_CHUNK_ITEMS: u32 = 8192;
/// Extra draws per frame at most (each stalled run is one).
const MAX_RUNS_PER_FRAME: u32 = 4000;

/// One paint source: mesh IBO id -> voxel colours of its body, with a version that bumps on change,
/// and the last published (body -> colours key, IBO -> body) to skip unchanged ticks.
struct Source {
    version: u64,
    map: Option<HashMap<u32, Arc<VoxelColours>>>,
    last: Option<(u64, HashMap<u32, usize>)>,
}
static FLIGHT: Mutex<Source> = Mutex::new(Source { version: 0, map: None, last: None });
static WORKBENCH: Mutex<Source> = Mutex::new(Source { version: 0, map: None, last: None });
static FLIGHT_ACTIVE: AtomicBool = AtomicBool::new(false);
static WORKBENCH_ACTIVE: AtomicBool = AtomicBool::new(false);
static VERSION: AtomicU64 = AtomicU64::new(0);

thread_local! {
    /// Inside the main-view draw (sim render slot 7).
    static MAIN: Cell<bool> = const { Cell::new(false) };
    static CACHE: RefCell<Cache> = RefCell::new(Cache::default());
}

/// Diagnostics (render thread): chunk draws, in the main view, IBO hits, readback failures.
static N_DRAWS: AtomicU64 = AtomicU64::new(0);
static N_MAIN: AtomicU64 = AtomicU64::new(0);
static N_HIT: AtomicU64 = AtomicU64::new(0);
static N_NO_TRIS: AtomicU64 = AtomicU64::new(0);
static N_PAINTED: AtomicU64 = AtomicU64::new(0);
static LAST_DIAG: AtomicU64 = AtomicU64::new(0);

#[derive(Default)]
struct Cache {
    /// IBO ids drawn in the main view since the last diagnostics line.
    seen: HashSet<u32>,
    /// mesh -> (IBO, index count, voxel per triangle).
    tris: HashMap<usize, (u32, u32, Arc<Vec<[i32; 3]>>)>,
    /// (mesh, first, count) -> (version, runs of painted triangles as (first index, index count, colour)).
    runs: HashMap<(usize, u32, u32), (u64, Arc<Vec<(u32, u32, u32)>>)>,
    logged: HashSet<u32>,
}

/// Extra draws issued since the last frame was presented (reset by the overlay each frame).
static FRAME_RUNS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

pub(crate) fn main_view(on: bool) {
    MAIN.with(|m| m.set(on));
}

/// Once per presented frame (overlay render, before SwapBuffers).
pub(crate) fn new_frame() {
    FRAME_RUNS.store(0, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------------------------
// Game thread: stalled voxels per body -> per chunk mesh
// ---------------------------------------------------------------------------------------------

unsafe fn ringvec_elems(base: usize, stride: usize) -> Vec<usize> {
    if base < PTR_MIN || !readable(base, 0x14) {
        return Vec::new();
    }
    let ptr = read_usize_at(base);
    let (cap, start, count) = (read_u32_at(base + 8), read_u32_at(base + 0xC), read_u32_at(base + 0x10));
    if ptr < PTR_MIN || cap == 0 || cap > MAX_CHUNK_ITEMS || count > cap || !readable(ptr, cap as usize * stride) {
        return Vec::new();
    }
    (0..count as usize).map(|i| ptr + stride * ((i + start as usize) % cap as usize)).collect()
}

/// IBO ids of a body's chunk meshes (88-byte chunk entries, each a ringvec of 96-byte instances
/// with geometry elements at +0/+0x10/+0x20; mesh = *(*element + 8)).
unsafe fn body_ibos(body: usize) -> Vec<u32> {
    let mut out = Vec::new();
    for table in BODY_CHUNK_TABLES {
        for entry in ringvec_elems(body + table, 88) {
            for inst in ringvec_elems(entry, 96) {
                for slot in [0usize, 0x10, 0x20] {
                    let el = inst + slot;
                    if !readable(el, 8) {
                        continue;
                    }
                    let group = read_usize_at(el);
                    if group < PTR_MIN || !readable(group + 8, 8) {
                        continue;
                    }
                    let mesh = read_usize_at(group + 8);
                    if mesh >= PTR_MIN && readable(mesh + MESH_IBO, 4) {
                        let ibo = read_u32_at(mesh + MESH_IBO);
                        if ibo != 0 {
                            out.push(ibo);
                        }
                    }
                }
            }
        }
    }
    out
}

/// Publish `colours` (body pointer -> voxel colours; `key` changes whenever the colours do) into a
/// source. Game thread only: it walks the bodies' chunk containers.
fn publish_into(src: &Mutex<Source>, active: &AtomicBool, key: u64, colours: &HashMap<usize, Arc<VoxelColours>>) {
    let mut ibo_body: HashMap<u32, usize> = HashMap::new();
    for &body in colours.keys() {
        if body >= PTR_MIN {
            for ibo in unsafe { body_ibos(body) } {
                ibo_body.insert(ibo, body);
            }
        }
    }
    let mut s = src.lock().unwrap_or_else(|p| p.into_inner());
    if s.last.as_ref().is_some_and(|(k, m)| *k == key && *m == ibo_body) {
        return;
    }
    static PUBLISH_LOGS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if !colours.is_empty() && PUBLISH_LOGS.fetch_add(1, Ordering::Relaxed) < 12 {
        let per: Vec<String> = colours.iter().map(|(&b, v)| format!("{b:#x}: {} voxels, {} meshes", v.len(), ibo_body.values().filter(|&&x| x == b).count())).collect();
        log_line(&format!("paint: publish {} bodies [{}]", colours.len(), per.join("; ")));
    }
    let map: HashMap<u32, Arc<VoxelColours>> = ibo_body.iter().filter_map(|(&ibo, b)| Some((ibo, colours.get(b)?.clone()))).collect();
    active.store(!map.is_empty(), Ordering::Relaxed);
    s.version = VERSION.fetch_add(1, Ordering::Relaxed) + 1;
    s.map = Some(map);
    s.last = Some((key, ibo_body));
}

/// Flight tick (game thread): stalled voxels per body, painted red.
pub(crate) fn publish(stalled: HashMap<usize, HashSet<[i32; 3]>>) {
    let mut key: u64 = 0xcbf29ce484222325;
    let mut cells: Vec<(usize, [i32; 3])> = stalled.iter().flat_map(|(&b, v)| v.iter().map(move |&c| (b, c))).collect();
    cells.sort_unstable();
    for (b, c) in cells {
        for v in [b as u64, c[0] as u32 as u64, c[1] as u32 as u64, c[2] as u32 as u64] {
            key = (key ^ v).wrapping_mul(0x100000001b3);
        }
    }
    let colours: HashMap<usize, Arc<VoxelColours>> = stalled.into_iter().map(|(b, v)| (b, Arc::new(v.into_iter().map(|c| (c, rgba(RED))).collect()))).collect();
    publish_into(&FLIGHT, &FLIGHT_ACTIVE, key, &colours);
}

/// Editor tick (game thread): the workbench vehicle's voxel colours per body (empty = off).
pub(crate) fn publish_workbench(key: u64, colours: &HashMap<usize, Arc<VoxelColours>>) {
    publish_into(&WORKBENCH, &WORKBENCH_ACTIVE, key, colours);
}

// ---------------------------------------------------------------------------------------------
// Render thread
// ---------------------------------------------------------------------------------------------

pub(crate) struct Gl {
    pub(crate) bind_buffer: unsafe extern "system" fn(u32, u32),
    get_buffer_sub_data: unsafe extern "system" fn(u32, isize, isize, *mut c_void),
    pub(crate) get_integerv: unsafe extern "system" fn(u32, *mut i32),
    depth_func: unsafe extern "system" fn(u32),
    pub(crate) bind_texture: unsafe extern "system" fn(u32, u32),
    pub(crate) compressed_tex_sub_image_2d: unsafe extern "system" fn(u32, i32, i32, i32, i32, i32, u32, i32, *const c_void),
}

pub(crate) fn gl() -> Option<&'static Gl> {
    static GL: OnceLock<Option<Gl>> = OnceLock::new();
    GL.get_or_init(|| unsafe {
        let module = LoadLibraryA(c"opengl32.dll".as_ptr().cast());
        if module.is_null() {
            return None;
        }
        let wgl = GetProcAddress(module, c"wglGetProcAddress".as_ptr().cast())?;
        let wgl: unsafe extern "system" fn(*const u8) -> usize = std::mem::transmute(wgl);
        let sym = |name: &std::ffi::CStr| -> Option<usize> {
            if let Some(f) = GetProcAddress(module, name.as_ptr().cast()) {
                return Some(f as usize);
            }
            let p = wgl(name.as_ptr().cast()); // small sentinels mean "unknown"
            (p > 0x10000).then_some(p)
        };
        let gl = Gl {
            bind_buffer: std::mem::transmute::<usize, _>(sym(c"glBindBuffer")?),
            get_buffer_sub_data: std::mem::transmute::<usize, _>(sym(c"glGetBufferSubData")?),
            get_integerv: std::mem::transmute::<usize, _>(sym(c"glGetIntegerv")?),
            depth_func: std::mem::transmute::<usize, _>(sym(c"glDepthFunc")?),
            bind_texture: std::mem::transmute::<usize, _>(sym(c"glBindTexture")?),
            compressed_tex_sub_image_2d: std::mem::transmute::<usize, _>(sym(c"glCompressedTexSubImage2D")?),
        };
        Some(gl)
    })
    .as_ref()
}

const GL_COPY_READ_BUFFER: u32 = 0x8F36;
const GL_DEPTH_FUNC: u32 = 0x0B74;
const GL_LEQUAL: u32 = 0x0203;

/// Why a mesh's triangles could not be read (first few logged with the mesh's raw fields).
fn readback_fail(mesh: usize, why: &str) -> Option<Arc<Vec<[i32; 3]>>> {
    static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if N.fetch_add(1, Ordering::Relaxed) < 6 {
        let raw: Vec<String> = if readable(mesh + 0x20, 0x50) {
            (0..0x50).step_by(4).map(|o| format!("+{:#x}={:#x}", 0x20 + o, unsafe { read_u32_at(mesh + 0x20 + o) })).collect()
        } else {
            vec!["unreadable".into()]
        };
        log_line(&format!("paint: mesh {mesh:#x} not read back: {why}; {}", raw.join(" ")));
    }
    None
}

/// Reads the mesh's vertices and indices (CPU copy if the game kept one, else from GL, once per
/// mesh) and maps triangles to voxels.
unsafe fn mesh_tris(gl: Option<&Gl>, mesh: usize, ibo: u32) -> Option<Arc<Vec<[i32; 3]>>> {
    if !readable(mesh + 0x28, 0x40) {
        return readback_fail(mesh, "fields unreadable");
    }
    let u = |o: usize| read_u32_at(mesh + o);
    let icount_now = u(MESH_ICOUNT);
    if let Some((i, n, t)) = CACHE.with(|c| c.borrow().tris.get(&mesh).cloned()) {
        if i == ibo && n == icount_now {
            return Some(t);
        }
    }
    let (vcount, stride, icount) = (u(MESH_VCOUNT), u(MESH_STRIDE), icount_now);
    if stride != 28 || vcount == 0 || vcount > 65536 || icount == 0 || icount > 3 << 20 {
        return readback_fail(mesh, &format!("counts v={vcount} stride={stride} i={icount}"));
    }
    let mut verts = vec![0u8; vcount as usize * 28];
    let mut idx = vec![0u16; icount as usize];
    let (cpu_v, cpu_i) = (read_usize_at(mesh + 0x28), read_usize_at(mesh + 0x38));
    if cpu_v >= PTR_MIN && cpu_i >= PTR_MIN && readable(cpu_v, verts.len()) && readable(cpu_i, idx.len() * 2) {
        std::ptr::copy_nonoverlapping(cpu_v as *const u8, verts.as_mut_ptr(), verts.len());
        std::ptr::copy_nonoverlapping(cpu_i as *const u16, idx.as_mut_ptr(), idx.len());
    } else {
        let Some(gl) = gl else { return readback_fail(mesh, "GL entry points missing (glBindBuffer/glGetBufferSubData)") };
        let vbo = u(MESH_VBO);
        if vbo == 0 {
            return readback_fail(mesh, "no VBO id");
        }
        (gl.bind_buffer)(GL_COPY_READ_BUFFER, vbo);
        (gl.get_buffer_sub_data)(GL_COPY_READ_BUFFER, 0, verts.len() as isize, verts.as_mut_ptr().cast());
        (gl.bind_buffer)(GL_COPY_READ_BUFFER, ibo);
        (gl.get_buffer_sub_data)(GL_COPY_READ_BUFFER, 0, idx.len() as isize * 2, idx.as_mut_ptr().cast());
        (gl.bind_buffer)(GL_COPY_READ_BUFFER, 0);
    }
    let tris = Arc::new(triangle_voxels(&verts, 28, &idx));
    CACHE.with(|c| c.borrow_mut().tris.insert(mesh, (ibo, icount_now, tris.clone())));
    Some(tris)
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn chunk_draw_detour(
    a1: usize, mesh: usize, first: u32, count: u32, m: *const f32, mp: *const f32, mn: *const f32, light: f32,
    c1: u32, c2: u32, c3: u32, preview: u8, force: u8, no_replace: u8, a15: usize, wire: u8,
) {
    let Some(&orig) = ORIGINAL.get() else { return };
    orig(a1, mesh, first, count, m, mp, mn, light, c1, c2, c3, preview, force, no_replace, a15, wire);
    // Workbench: any view of the editor vehicle's meshes. Flight: main view only, F9 open.
    // The editor must still be ticking: its meshes' GL ids are recycled once it closes.
    let bench = WORKBENCH_ACTIVE.load(Ordering::Relaxed) && crate::editor::editor_active();
    let flight = FLIGHT_ACTIVE.load(Ordering::Relaxed) && crate::flight::DEBUG_WANTED.load(Ordering::Relaxed) && crate::flight::ENABLED.load(Ordering::Relaxed);
    if !bench && !flight {
        return;
    }
    N_DRAWS.fetch_add(1, Ordering::Relaxed);
    let _ = std::panic::catch_unwind(diag);
    if mesh < PTR_MIN {
        return;
    }
    let flight = flight && MAIN.with(|v| v.get());
    if flight {
        N_MAIN.fetch_add(1, Ordering::Relaxed);
    }
    if !bench && !flight {
        return;
    }
    let _ = std::panic::catch_unwind(|| paint(orig, bench, flight, a1, mesh, first, count, m, mp, mn, light, c2, c3, preview, no_replace, a15));
}

fn log_once_gl() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| log_line("paint: GL entry points missing (opengl32 / wglGetProcAddress)"));
}

/// Every 5 s while painting is wanted: where the pipeline stops.
fn diag() {
    let now = crate::editor::now_ms();
    let last = LAST_DIAG.load(Ordering::Relaxed);
    if now.saturating_sub(last) < 5000 || LAST_DIAG.compare_exchange(last, now, Ordering::Relaxed, Ordering::Relaxed).is_err() {
        return;
    }
    let seen = CACHE.with(|c| std::mem::take(&mut c.borrow_mut().seen));
    let keys = |src: &Mutex<Source>| src.lock().ok().and_then(|s| s.map.as_ref().map(|m| m.keys().copied().collect::<Vec<u32>>())).unwrap_or_default();
    let targets: Vec<u32> = [keys(&FLIGHT), keys(&WORKBENCH)].concat();
    let common = targets.iter().filter(|i| seen.contains(i)).count();
    let mut sample: Vec<u32> = seen.iter().copied().collect();
    sample.sort_unstable();
    sample.truncate(12);
    let mut tsample = targets.clone();
    tsample.sort_unstable();
    tsample.truncate(12);
    log_line(&format!(
        "paint: draws {} main-view {} target hits {} no-readback {} painted {} | main-view ibos {} (e.g. {sample:?}), target ibos {} (e.g. {tsample:?}), common {common}",
        N_DRAWS.swap(0, Ordering::Relaxed), N_MAIN.swap(0, Ordering::Relaxed), N_HIT.swap(0, Ordering::Relaxed),
        N_NO_TRIS.swap(0, Ordering::Relaxed), N_PAINTED.swap(0, Ordering::Relaxed), seen.len(), targets.len()
    ));
}

#[allow(clippy::too_many_arguments)]
unsafe fn paint(orig: ChunkDraw, bench: bool, flight: bool, a1: usize, mesh: usize, first: u32, count: u32, m: *const f32, mp: *const f32, mn: *const f32, light: f32, c2: u32, c3: u32, preview: u8, no_replace: u8, a15: usize) {
    if !readable(mesh + MESH_IBO, 4) {
        return;
    }
    let ibo = read_u32_at(mesh + MESH_IBO);
    CACHE.with(|c| {
        let mut c = c.borrow_mut();
        if c.seen.len() < 4096 {
            c.seen.insert(ibo);
        }
    });
    let look = |src: &Mutex<Source>| src.lock().ok().and_then(|s| Some((s.version, s.map.as_ref()?.get(&ibo)?.clone())));
    let Some((version, set)) = bench.then(|| look(&WORKBENCH)).flatten().or_else(|| flight.then(|| look(&FLIGHT)).flatten()) else { return };
    N_HIT.fetch_add(1, Ordering::Relaxed);
    let gl = gl();
    if gl.is_none() {
        log_once_gl();
    }
    let Some(tris) = mesh_tris(gl, mesh, ibo) else {
        N_NO_TRIS.fetch_add(1, Ordering::Relaxed);
        return;
    };
    let count = if count == 0 { tris.len() as u32 * 3 } else { count };
    let key = (mesh, first, count);
    let runs = match CACHE.with(|c| c.borrow().runs.get(&key).cloned()) {
        Some((v, r)) if v == version => r,
        _ => {
            let r = Arc::new(colour_runs(&tris, first, count, &set));
            CACHE.with(|c| {
                let mut c = c.borrow_mut();
                if c.runs.len() > 4096 {
                    c.runs.clear();
                }
                if c.tris.len() > 1024 {
                    c.tris.clear();
                }
                c.runs.insert(key, (version, r.clone()));
                if c.logged.insert(ibo) && c.logged.len() <= 16 {
                    // Frame check: the stalled voxels should fall inside this mesh's voxel box.
                    let ok = tris.iter().filter(|v| v[0] != i32::MIN);
                    let (lo, hi) = ok.fold(([i32::MAX; 3], [i32::MIN; 3]), |(lo, hi), v| (std::array::from_fn(|k| lo[k].min(v[k])), std::array::from_fn(|k| hi[k].max(v[k]))));
                    let (slo, shi) = set.keys().fold(([i32::MAX; 3], [i32::MIN; 3]), |(lo, hi), v| (std::array::from_fn(|k| lo[k].min(v[k])), std::array::from_fn(|k| hi[k].max(v[k]))));
                    let hit: HashSet<[i32; 3]> = tris.iter().filter(|v| set.contains_key(*v)).copied().collect();
                    log_line(&format!(
                        "paint: ibo {ibo}: {} tris, voxels {lo:?}..{hi:?}; body painted {} voxels {slo:?}..{shi:?}; {} found in this mesh, {} runs",
                        tris.len(), set.len(), hit.len(), r.len()
                    ));
                }
            });
            r
        }
    };
    if runs.is_empty() {
        return;
    }
    let budget = MAX_RUNS_PER_FRAME.saturating_sub(FRAME_RUNS.fetch_add(runs.len() as u32, Ordering::Relaxed)) as usize;
    if budget == 0 {
        return;
    }
    let Some(gl) = gl else { return };
    let mut prev = 0i32;
    (gl.get_integerv)(GL_DEPTH_FUNC, &mut prev);
    // Same program and inputs give identical depth: LEQUAL lets the red pass over the game's own draw.
    (gl.depth_func)(GL_LEQUAL);
    N_PAINTED.fetch_add(1, Ordering::Relaxed);
    for &(f, n, col) in runs.iter().take(budget) {
        orig(a1, mesh, f, n, m, mp, mn, light, col, c2, c3, preview, 1, no_replace, a15, 0);
    }
    (gl.depth_func)(prev as u32);
}

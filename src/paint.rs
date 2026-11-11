//! Paints chosen voxels in the game's own render: stalled wing voxels red in flight (F9 open,
//! plugin flight on), and wing voxels by role / aerofoil on the workbench vehicle (Aero wings mode).
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
use crate::voxel::{colour_runs, groups, triangle_voxels, Group};

pub(crate) type ChunkDraw = unsafe extern "system" fn(
    usize,      // bindset (renderer+0x920)
    usize,      // mesh
    u32,        // first index
    u32,        // index count (0 = whole mesh)
    *const f32, // mat_world (interpolated)
    *const f32, // mat_world_prev
    *const f32, // mat_world_next
    f32,        // light_factor
    u32,        // override_color_1 (RGBA8, R in the low byte)
    u32,        // override_color_2
    u32,        // override_color_3
    u8,         // is_preview
    u8,         // is_force_color
    u8,         // is_disable_color_replace
    usize,      // unused
    u8,         // wireframe pass
);
pub(crate) static ORIGINAL: OnceLock<ChunkDraw> = OnceLock::new();

/// Stall red as [r, g, b] (the shader linearises colours like vertex colours).
pub(crate) const RED: [u8; 3] = [230, 32, 32];

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
/// Grid lines left unpainted split the runs, roughly one per painted block.
const MAX_RUNS_PER_FRAME: u32 = 12000;

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

/// Moving component meshes (missile fins, control surfaces) to tint: component pointer -> RGB
/// colour, for the workbench vehicle and for flight (stalled, F9 open). Their class draw passes
/// the colour with `TINT_MARK` as alpha; `chunk_draw_detour` then draws it forced, like voxels.
static BENCH_TINT: Mutex<Option<HashMap<usize, u32>>> = Mutex::new(None);
static FLIGHT_TINT: Mutex<Option<HashMap<usize, u32>>> = Mutex::new(None);
const TINT_MARK: u32 = 0x5A00_0000;

pub(crate) fn publish_bench_tint(t: HashMap<usize, u32>) {
    *BENCH_TINT.lock().unwrap_or_else(|p| p.into_inner()) = (!t.is_empty()).then_some(t);
}
pub(crate) fn publish_flight_tint(t: HashMap<usize, u32>) {
    *FLIGHT_TINT.lock().unwrap_or_else(|p| p.into_inner()) = (!t.is_empty()).then_some(t);
}

pub(crate) type ClassDraw = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) static ORIGINAL_ROCKET_DRAW: OnceLock<ClassDraw> = OnceLock::new();
pub(crate) static ORIGINAL_RUDDER_DRAW: OnceLock<ClassDraw> = OnceLock::new();

/// Tint for this component's draw now (game thread), if any.
fn tint_of(comp: usize) -> Option<u32> {
    let look = |m: &Mutex<Option<HashMap<usize, u32>>>| m.lock().ok().and_then(|g| g.as_ref()?.get(&comp).copied());
    let bench = || crate::editor::editor_active().then(|| look(&BENCH_TINT)).flatten();
    let flight =
        || (crate::flight::DEBUG_WANTED.load(Ordering::Relaxed) && crate::flight::ENABLED.load(Ordering::Relaxed)).then(|| look(&FLIGHT_TINT)).flatten();
    bench().or_else(flight).map(|c| (c & 0x00FF_FFFF) | TINT_MARK)
}

#[allow(clippy::too_many_arguments)]
unsafe fn class_draw(
    orig: Option<&ClassDraw>,
    c: usize,
    q: usize,
    ma: usize,
    mb: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    a8: usize,
    a9: usize,
    a10: usize,
) -> usize {
    let Some(orig) = orig else { return 0 };
    match tint_of(c) {
        Some(col) => orig(c, q, ma, mb, a5, a6, a7, 1, a9, col as usize),
        None => orig(c, q, ma, mb, a5, a6, a7, a8, a9, a10),
    }
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn rocket_draw_detour(
    c: usize,
    q: usize,
    ma: usize,
    mb: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    a8: usize,
    a9: usize,
    a10: usize,
) -> usize {
    class_draw(ORIGINAL_ROCKET_DRAW.get(), c, q, ma, mb, a5, a6, a7, a8, a9, a10)
}
#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn rudder_draw_detour(
    c: usize,
    q: usize,
    ma: usize,
    mb: usize,
    a5: usize,
    a6: usize,
    a7: usize,
    a8: usize,
    a9: usize,
    a10: usize,
) -> usize {
    class_draw(ORIGINAL_RUDDER_DRAW.get(), c, q, ma, mb, a5, a6, a7, a8, a9, a10)
}

thread_local! {
    /// Inside the main-view draw (sim render slot 7).
    static MAIN: Cell<bool> = const { Cell::new(false) };
    static CACHE: RefCell<Cache> = RefCell::new(Cache::default());
    /// Runs (first indices, index counts, how many) the next `glDrawElements` draws instead of its
    /// own range; they point into the `Group` `paint` holds across that draw.
    static MULTI: Cell<(*const u32, *const i32, usize)> = const { Cell::new(NO_MULTI) };
}

const NO_MULTI: (*const u32, *const i32, usize) = (std::ptr::null(), std::ptr::null(), 0);
/// Per source (workbench, flight): bit `ibo & 4095` set for every mesh IBO it paints, so a draw of
/// any other mesh costs two loads (the workbench makes about 2000 chunk draws a frame).
static IBO_BITS: [[AtomicU64; 64]; 2] = [const { [const { AtomicU64::new(0) }; 64] }; 2];

type DrawElements = unsafe extern "system" fn(u32, i32, u32, *const c_void);
type MultiDrawElements = unsafe extern "system" fn(u32, *const i32, u32, *const *const c_void, i32);
pub(crate) static ORIGINAL_DRAW_ELEMENTS: OnceLock<DrawElements> = OnceLock::new();
/// Off once a batched draw did not reach `glDrawElements`: each run is then drawn on its own.
static BATCH_OK: AtomicBool = AtomicBool::new(true);

/// `glDrawElements` (opengl32 export, render thread). With runs queued by `paint`, the game's own
/// draw call (its state and uniforms set) draws all of them in one `glMultiDrawElements`, from
/// the same base offset the game used for its own first index.
pub(crate) unsafe extern "system" fn draw_elements_detour(mode: u32, count: i32, ty: u32, indices: *const c_void) {
    let Some(&orig) = ORIGINAL_DRAW_ELEMENTS.get() else { return };
    let (firsts, counts, n) = MULTI.with(|m| m.get());
    let multi = if n == 0 { None } else { gl().and_then(|g| g.multi_draw_elements) };
    let Some(multi) = multi else {
        return orig(mode, count, ty, indices);
    };
    MULTI.with(|m| m.set(NO_MULTI));
    let size = match ty {
        GL_UNSIGNED_SHORT => 2,
        GL_UNSIGNED_INT => 4,
        _ => 1,
    };
    // The game asked for the first run: its offset gives the buffer base for the others.
    let firsts = std::slice::from_raw_parts(firsts, n);
    let base = (indices as usize).wrapping_sub(firsts[0] as usize * size);
    let offs: Vec<*const c_void> = firsts.iter().map(|&f| (base + f as usize * size) as *const c_void).collect();
    multi(mode, counts, ty, offs.as_ptr(), n as i32);
}

/// Diagnostics (render thread): chunk draws, in the main view, IBO hits, readback failures.
static N_HIT: AtomicU64 = AtomicU64::new(0);
static N_NO_TRIS: AtomicU64 = AtomicU64::new(0);
static N_PAINTED: AtomicU64 = AtomicU64::new(0);

static LAST_DIAG: AtomicU64 = AtomicU64::new(0);

#[derive(Default)]
struct Cache {
    /// mesh -> (IBO, index count, voxel per triangle).
    tris: HashMap<usize, (u32, u32, Arc<Vec<crate::voxel::TriVoxels>>)>,
    /// (mesh, first, count) -> (version, runs of painted triangles by colour).
    runs: HashMap<(usize, u32, u32), (u64, Arc<Vec<Group>>)>,
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
    if WORKBENCH_ACTIVE.load(Ordering::Relaxed) || FLIGHT_ACTIVE.load(Ordering::Relaxed) {
        let _ = std::panic::catch_unwind(diag);
    }
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
fn publish_into(src: &Mutex<Source>, active: &AtomicBool, bits: &[AtomicU64; 64], key: u64, colours: &HashMap<usize, Arc<VoxelColours>>) {
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
        let per: Vec<String> =
            colours.iter().map(|(&b, v)| format!("{b:#x}: {} voxels, {} meshes", v.len(), ibo_body.values().filter(|&&x| x == b).count())).collect();
        log_line(&format!("paint: publish {} bodies [{}]", colours.len(), per.join("; ")));
    }
    let map: HashMap<u32, Arc<VoxelColours>> = ibo_body.iter().filter_map(|(&ibo, b)| Some((ibo, colours.get(b)?.clone()))).collect();
    let mut words = [0u64; 64];
    for &ibo in map.keys() {
        words[(ibo as usize >> 6) & 63] |= 1 << (ibo & 63);
    }
    // Word by word, so an IBO painted before and after never reads as unpainted in between.
    for (w, v) in bits.iter().zip(words) {
        w.store(v, Ordering::Relaxed);
    }
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
    publish_into(&FLIGHT, &FLIGHT_ACTIVE, &IBO_BITS[1], key, &colours);
}

/// Editor tick (game thread): the workbench vehicle's voxel colours per body (empty = off).
pub(crate) fn publish_workbench(key: u64, colours: &HashMap<usize, Arc<VoxelColours>>) {
    publish_into(&WORKBENCH, &WORKBENCH_ACTIVE, &IBO_BITS[0], key, colours);
}

// ---------------------------------------------------------------------------------------------
// Render thread
// ---------------------------------------------------------------------------------------------

pub(crate) struct Gl {
    pub(crate) bind_buffer: unsafe extern "system" fn(u32, u32),
    get_buffer_sub_data: unsafe extern "system" fn(u32, isize, isize, *mut c_void),
    pub(crate) get_integerv: unsafe extern "system" fn(u32, *mut i32),
    multi_draw_elements: Option<MultiDrawElements>,
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
            multi_draw_elements: sym(c"glMultiDrawElements").map(|p| std::mem::transmute::<usize, MultiDrawElements>(p)),
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
const GL_UNSIGNED_SHORT: u32 = 0x1403;
const GL_UNSIGNED_INT: u32 = 0x1405;

const GL_LEQUAL: u32 = 0x0203;

/// Why a mesh's triangles could not be read (first few logged with the mesh's raw fields).
fn readback_fail(mesh: usize, why: &str) -> Option<Arc<Vec<crate::voxel::TriVoxels>>> {
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
unsafe fn mesh_tris(gl: Option<&Gl>, mesh: usize, ibo: u32) -> Option<Arc<Vec<crate::voxel::TriVoxels>>> {
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
        let Some(gl) = gl else {
            return readback_fail(mesh, "GL entry points missing (glBindBuffer/glGetBufferSubData)");
        };
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
    a1: usize,
    mesh: usize,
    first: u32,
    count: u32,
    m: *const f32,
    mp: *const f32,
    mn: *const f32,
    light: f32,
    c1: u32,
    c2: u32,
    c3: u32,
    preview: u8,
    force: u8,
    no_replace: u8,
    a15: usize,
    wire: u8,
) {
    let Some(&orig) = ORIGINAL.get() else { return };
    // A tinted moving mesh (`class_draw`): its colour, forced over the texture.
    if c1 == c2 && c2 == c3 && c1 & 0xFF00_0000 == TINT_MARK {
        let c = c1 | 0xFF00_0000;
        return orig(a1, mesh, first, count, m, mp, mn, light, c, c, c, preview, 1, no_replace, a15, wire);
    }
    // `wire` != 0 (the workbench block grid on): this call draws the triangles, then the same
    // triangles again as black lines. The paint below passes it on, so the lines stay on top.
    orig(a1, mesh, first, count, m, mp, mn, light, c1, c2, c3, preview, force, no_replace, a15, wire);
    if mesh < PTR_MIN {
        return;
    }
    // The game just drew from this mesh, so its fields are readable.
    let ibo = read_u32_at(mesh + MESH_IBO);
    let (w, b) = ((ibo as usize >> 6) & 63, 1u64 << (ibo & 63));
    let (in_bench, in_flight) = (IBO_BITS[0][w].load(Ordering::Relaxed) & b != 0, IBO_BITS[1][w].load(Ordering::Relaxed) & b != 0);
    if !in_bench && !in_flight {
        return;
    }
    // Workbench: any view of the editor vehicle's meshes. Flight: main view only, F9 open.
    // The editor must still be ticking: its meshes' GL ids are recycled once it closes.
    let bench = in_bench && WORKBENCH_ACTIVE.load(Ordering::Relaxed) && crate::editor::editor_active();
    let flight = in_flight
        && FLIGHT_ACTIVE.load(Ordering::Relaxed)
        && crate::flight::DEBUG_WANTED.load(Ordering::Relaxed)
        && crate::flight::ENABLED.load(Ordering::Relaxed)
        && MAIN.with(|v| v.get());
    if !bench && !flight {
        return;
    }
    let _ = std::panic::catch_unwind(|| paint(orig, bench, flight, a1, mesh, first, count, m, mp, mn, light, c1, c2, c3, preview, force, no_replace, a15, wire));
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
    log_line(&format!(
        "paint: target hits {} no-readback {} painted {}",
        N_HIT.swap(0, Ordering::Relaxed),
        N_NO_TRIS.swap(0, Ordering::Relaxed),
        N_PAINTED.swap(0, Ordering::Relaxed)
    ));
}

#[allow(clippy::too_many_arguments)]
unsafe fn paint(
    orig: ChunkDraw,
    bench: bool,
    flight: bool,
    a1: usize,
    mesh: usize,
    first: u32,
    count: u32,
    m: *const f32,
    mp: *const f32,
    mn: *const f32,
    light: f32,
    c1: u32,
    c2: u32,
    c3: u32,
    preview: u8,
    force: u8,
    no_replace: u8,
    a15: usize,
    wire: u8,
) {
    let ibo = read_u32_at(mesh + MESH_IBO);
    let look = |src: &Mutex<Source>| src.lock().ok().and_then(|s| Some((s.version, s.map.as_ref()?.get(&ibo)?.clone())));
    // The workbench colours list the unpainted parts' cells too (colour 0), so corners may fill in.
    let Some((version, set, corners)) =
        bench.then(|| look(&WORKBENCH)).flatten().map(|(v, s)| (v, s, true)).or_else(|| flight.then(|| look(&FLIGHT)).flatten().map(|(v, s)| (v, s, false)))
    else {
        return;
    };
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
            let r = Arc::new(groups(colour_runs(&tris, first, count, &set, corners)));
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
                    let ok = tris.iter().map(|t| &t[0]).filter(|v| v[0] != i32::MIN);
                    let (lo, hi) = ok.fold(([i32::MAX; 3], [i32::MIN; 3]), |(lo, hi), v| {
                        (std::array::from_fn(|k| lo[k].min(v[k])), std::array::from_fn(|k| hi[k].max(v[k])))
                    });
                    let (slo, shi) = set.keys().fold(([i32::MAX; 3], [i32::MIN; 3]), |(lo, hi), v| {
                        (std::array::from_fn(|k| lo[k].min(v[k])), std::array::from_fn(|k| hi[k].max(v[k])))
                    });
                    let hit: HashSet<[i32; 3]> = tris.iter().map(|t| t[0]).filter(|v| set.get(v).is_some_and(|&c| c != 0)).collect();
                    log_line(&format!(
                        "paint: ibo {ibo}: {} tris, voxels {lo:?}..{hi:?}; body painted {} voxels {slo:?}..{shi:?}; {} found in this mesh, {} runs",
                        tris.len(),
                        set.len(),
                        hit.len(),
                        r.iter().map(|g| g.firsts.len()).sum::<usize>()
                    ));
                }
            });
            r
        }
    };
    if runs.is_empty() {
        return;
    }
    // Batched, a mesh costs a draw per colour whatever its run count; one by one, runs are rationed.
    // `wire` makes the game draw twice per call, and only the first would take the batch.
    let batched = wire == 0 && ORIGINAL_DRAW_ELEMENTS.get().is_some();
    let total: usize = runs.iter().map(|g| g.firsts.len()).sum();
    if !(batched && BATCH_OK.load(Ordering::Relaxed)) && FRAME_RUNS.fetch_add(total as u32, Ordering::Relaxed) >= MAX_RUNS_PER_FRAME {
        return;
    }
    let Some(gl) = gl else { return };
    let mut prev = 0i32;
    (gl.get_integerv)(GL_DEPTH_FUNC, &mut prev);
    // Same program and inputs give identical depth: LEQUAL lets the red pass over the game's own draw.
    (gl.depth_func)(GL_LEQUAL);
    N_PAINTED.fetch_add(1, Ordering::Relaxed);
    for g in runs.iter() {
        let draw = |f: u32, n: i32| {
            if g.col == 0 {
                // Grid lines: drawn as the game did, over the paint.
                orig(a1, mesh, f, n as u32, m, mp, mn, light, c1, c2, c3, preview, force, no_replace, a15, wire);
            } else {
                orig(a1, mesh, f, n as u32, m, mp, mn, light, g.col, c2, c3, preview, 1, no_replace, a15, wire);
            }
        };
        let mut from = 0;
        if batched && g.firsts.len() > 1 && BATCH_OK.load(Ordering::Relaxed) {
            MULTI.with(|q| q.set((g.firsts.as_ptr(), g.counts.as_ptr(), g.firsts.len())));
            draw(g.firsts[0], g.counts[0]);
            if MULTI.with(|q| q.replace(NO_MULTI)).2 == 0 {
                continue;
            }
            // The game's draw did not go through glDrawElements: only the first run was drawn.
            BATCH_OK.store(false, Ordering::Relaxed);
            log_line("paint: batched draw did not reach glDrawElements; drawing runs one by one");
            from = 1;
        }
        for i in from..g.firsts.len() {
            draw(g.firsts[i], g.counts[i]);
        }
    }
    (gl.depth_func)(prev as u32);
}

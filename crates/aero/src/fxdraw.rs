//! Draws the aerodynamic effects (`fx`) into the main view's lit scene, before TAA, bloom and the
//! tonemap (knowledge/rendering.md, "Main-view translucent insertion point"). A detour on
//! `gl_set_depth_test_mask` acts on one call site only: the one right after the game's own
//! translucent passes in `render_scene_with_cameras`, where the scene colour target is bound.

use std::ffi::{c_void, CStr, CString};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::OnceLock;

use crate::fx::Head;
use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at};

/// Return address of the one call the detour acts on; the trampoline to the original.
static RET_SITE: AtomicUsize = AtomicUsize::new(usize::MAX);
/// Snapshots drawn, how many placed by the exact drawn pose, and how many of those a step or more
/// off the timing model (logged every 10 s).
static DRAWN: AtomicUsize = AtomicUsize::new(0);
static EXACT: AtomicUsize = AtomicUsize::new(0);
static AHEAD: AtomicUsize = AtomicUsize::new(0);
static AHEAD_LOG_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static ORIGINAL: AtomicUsize = AtomicUsize::new(0);

// At that call r12 = render target set, r15 = view state, r13 = renderer (non-volatile, still the
// caller's on entry). Draw with them, then go on to the original with its arguments untouched.
std::arch::global_asm!(
    ".globl aero_fx_depth_mask_detour",
    "aero_fx_depth_mask_detour:",
    "mov rax, [rsp]",
    "cmp rax, qword ptr [rip + {ret}]",
    "jne 2f",
    "push rcx",
    "push rdx",
    "push r8",
    "sub rsp, 0x20",
    "mov rcx, r12",
    "mov rdx, r15",
    "mov r8, r13",
    "call {draw}",
    "add rsp, 0x20",
    "pop r8",
    "pop rdx",
    "pop rcx",
    "2:",
    "jmp qword ptr [rip + {orig}]",
    ret = sym RET_SITE,
    draw = sym draw,
    orig = sym ORIGINAL,
);
extern "system" {
    fn aero_fx_depth_mask_detour();
}

/// Hooks `gl_set_depth_test_mask` for the effects. `Err`: the log line.
pub(crate) unsafe fn install(base: usize) -> Result<(), String> {
    use crate::offsets::*;
    let (at, site) = (base + RVA_GL_SET_DEPTH_TEST_MASK, base + RVA_FX_CALL_SITE);
    if !crate::hook::verify(at, &GL_SET_DEPTH_TEST_MASK_PROLOGUE) || !crate::hook::verify(site, &FX_CALL_SITE_BYTES) {
        return Err(format!("effects: gl_set_depth_test_mask {at:#x} or its call site {site:#x} not as expected; no effects drawn"));
    }
    let o = minhook::MinHook::create_hook(at as *mut c_void, aero_fx_depth_mask_detour as *mut c_void).map_err(|e| format!("effects: create hook: {e:?}"))?;
    ORIGINAL.store(o as usize, Ordering::SeqCst);
    RET_SITE.store(site + FX_CALL_SITE_BYTES.len() + 4, Ordering::SeqCst);
    minhook::MinHook::enable_hook(at as *mut c_void).map_err(|e| format!("effects: enable hook: {e:?}"))?;
    log_line(&format!("effects: drawing at {site:#x}"));
    Ok(())
}

extern "system" fn draw(rt_set: usize, view: usize, renderer: usize) {
    let _ = std::panic::catch_unwind(|| unsafe { draw_main(rt_set, view, renderer) });
}

unsafe fn draw_main(rt_set: usize, view: usize, renderer: usize) {
    use crate::offsets::*;
    // The main game view only: not monitors (camera feeds) or the editor's preview.
    if rt_set != renderer + OFF_RENDERER_MAIN_TARGETS || read_usize_at(view + OFF_VIEW_FEED) != 0 || ((view + OFF_VIEW_EDITOR) as *const u8).read() != 0 {
        return;
    }
    let mut snaps = if crate::fx::ENABLED.load(Ordering::Relaxed) { crate::fx::SNAPSHOT.lock().map(|s| s.clone()).unwrap_or_default() } else { Vec::new() };
    // The bodies the next frame's draw capture looks for: where the game draws them, exactly.
    let mut track: Vec<usize> = Vec::new();
    for s in &snaps {
        let heads = s.trails.iter().filter_map(|(_, h)| h.as_ref().map(|h| h.body));
        for b in s.pace.map(|p| p.0).into_iter().chain(heads).chain(s.clouds.iter().map(|c| c.body)) {
            if b != 0 && !track.contains(&b) {
                track.push(b);
            }
        }
    }
    *crate::drawcap::TRACK_FX.lock().unwrap_or_else(|p| p.into_inner()) = track;
    if snaps.is_empty() {
        return;
    }
    let (Some(vp), Some((alpha, _))) = (crate::flight::main_viewproj(), crate::flight::render_frame()) else {
        return;
    };
    let Some(eye) = crate::math::camera_eye(&vp) else {
        return;
    };
    // A body's point where the game draws it this frame: exactly as drawn (draw capture), else its
    // two latest builds blended by the frame's alpha.
    let drawn = |body: usize, local: crate::math::V3| -> Option<(crate::math::V3, bool)> {
        if let Some(m) = crate::drawcap::drawn_now(body) {
            return Some((crate::math::xf_point(&m, local), true));
        }
        crate::watch::built_anchor(body, local).map(|(cur, prev)| (prev + (cur - prev) * alpha as f64, false))
    };
    // The moment each snapshot is drawn at: the main body's drawn pose against where it was at the
    // snapshot, in its travel. The meshes take (1 − alpha) steps back from the snapshot; the rest
    // is where the game's frame is off that (a step or more when the update thread ran ahead,
    // which put the trails' newest points past the tips and made the mist's streaks jump).
    for s in &mut snaps {
        DRAWN.fetch_add(1, Ordering::Relaxed);
        let Some((body, local, at, vel)) = s.pace else { continue };
        let step = vel * s.dt;
        let Some((now_at, exact)) = drawn(body, local).filter(|_| step.dot(step) > 1e-4) else { continue };
        let behind = ((at - now_at).dot(step) / step.dot(step)).clamp(-1.0, 3.0);
        let off = if exact { behind - (1.0 - alpha as f64) } else { (behind - (1.0 - alpha as f64)).round() };
        EXACT.fetch_add(exact as usize, Ordering::Relaxed);
        AHEAD.fetch_add((off.abs() > 0.5) as usize, Ordering::Relaxed);
        s.back_up(off);
    }
    let ms = crate::editor::now_ms();
    let log_now = ms.saturating_sub(AHEAD_LOG_MS.load(Ordering::Relaxed)) > 10_000;
    if log_now {
        AHEAD_LOG_MS.store(ms, Ordering::Relaxed);
        log_line(&format!(
            "effects: {} snapshots drawn, {} placed by the drawn pose, {} a step or more off the frame's alpha",
            DRAWN.swap(0, Ordering::Relaxed),
            EXACT.swap(0, Ordering::Relaxed),
            AHEAD.swap(0, Ordering::Relaxed)
        ));
    }
    let head = |h: &Head| drawn(h.body, h.local).map(|d| d.0);
    let mesh = crate::fx::ribbon_mesh(&snaps, eye, alpha as f64, &head);
    let anchor = |body: usize, local: crate::math::V3| drawn(body, local).map(|d| d.0);
    let clouds = crate::fxwing::cloud_mesh(&snaps, eye, alpha as f64, &anchor);
    if mesh.is_empty() && clouds.is_empty() {
        return;
    }
    let target = read_usize_at(rt_set + 8);
    let size = (read_u32_at(target + OFF_TARGET_WIDTH) as f32, read_u32_at(target + OFF_TARGET_HEIGHT) as f32);
    let depth = read_u32_at(read_usize_at(rt_set) + OFF_GBUFFER_DEPTH_TEX);
    let fog = f32::from_bits(read_u32_at(view + OFF_VIEW_FOG_DENSITY));
    let fog = if fog.is_finite() { fog.clamp(0.0, 1.0) } else { 0.0 };
    // Camera-relative view-projection (row vectors, clip = [x y z 1]·M): row 3 takes in the eye, in f64.
    let mut rel = vp;
    for j in 0..4 {
        rel[12 + j] = eye.x * vp[j] + eye.y * vp[4 + j] + eye.z * vp[8 + j] + vp[12 + j];
    }
    let rel: [f32; 16] = rel.map(|v| v as f32);
    if let Some(r) = renderer_gl() {
        let light = read_light(view);
        if log_now {
            log_line(&format!("effects: light sun {:.2?} ambient {:.2?}..{:.2?}", light.sun, light.amb_low, light.amb_high));
        }
        // Mist under the trails: both are see-through, the trails mostly off the wing.
        r.draw(&r.clouds, clouds.as_flattened(), &CLOUD_ATTRS, &rel, size, depth, fog, &light);
        r.draw(&r.ribbons, mesh.as_flattened(), &RIBBON_ATTRS, &rel, size, depth, fog, &light);
    }
}

/// The game's lights for this view (`fxglsl::light_glsl`): sun, moon and sky from the view state,
/// as the light pass uploads them.
#[derive(Clone, Copy, Default)]
struct Light {
    sun: [f32; 3],
    amb_low: [f32; 3],
    amb_high: [f32; 3],
}

unsafe fn read_light(view: usize) -> Light {
    use crate::offsets::*;
    let f = |off: usize| Some(f32::from_bits(read_u32_at(view + off))).filter(|v| v.is_finite()).unwrap_or(0.0);
    let v3 = |off: usize| [f(off), f(off + 4), f(off + 8)];
    let rgb = |off: usize| {
        let c = read_u32_at(view + off).to_le_bytes();
        [c[0], c[1], c[2]].map(|b| b as f32 / 255.0)
    };
    let sun_i = (1.0 - (1.0 - f(OFF_VIEW_SUN_DIM_A)) * f(OFF_VIEW_SUN_DIM_B)) * f(OFF_VIEW_SUN_INTENSITY);
    let (sun, moon, moon_i) = (v3(OFF_VIEW_SUN_COLOR), v3(OFF_VIEW_MOON_COLOR), f(OFF_VIEW_MOON_INTENSITY));
    Light {
        sun: std::array::from_fn(|i| (sun[i] * sun_i + moon[i] * moon_i).max(0.0)),
        amb_low: rgb(OFF_VIEW_AMBIENT_LOW),
        amb_high: rgb(OFF_VIEW_AMBIENT_HIGH),
    }
}

use crate::fxglsl::{CLOUD_ATTRS, CLOUD_FRAGMENT, CLOUD_VERTEX, FRAGMENT, RIBBON_ATTRS, VERTEX};


macro_rules! gl_fns {
    ($($f:ident = $name:literal: fn($($a:ty),*) $(-> $r:ty)?;)*) => {
        #[allow(non_snake_case)]
        struct Gl { $($f: unsafe extern "system" fn($($a),*) $(-> $r)?,)* }
        unsafe fn load_gl() -> Option<Gl> {
            Some(Gl { $($f: std::mem::transmute::<usize, unsafe extern "system" fn($($a),*) $(-> $r)?>(crate::paint::gl_proc($name)?),)* })
        }
    };
}
gl_fns! {
    CreateShader = c"glCreateShader": fn(u32) -> u32;
    ShaderSource = c"glShaderSource": fn(u32, i32, *const *const i8, *const i32);
    CompileShader = c"glCompileShader": fn(u32);
    GetShaderiv = c"glGetShaderiv": fn(u32, u32, *mut i32);
    GetShaderInfoLog = c"glGetShaderInfoLog": fn(u32, i32, *mut i32, *mut u8);
    CreateProgram = c"glCreateProgram": fn() -> u32;
    AttachShader = c"glAttachShader": fn(u32, u32);
    LinkProgram = c"glLinkProgram": fn(u32);
    GetProgramiv = c"glGetProgramiv": fn(u32, u32, *mut i32);
    GetProgramInfoLog = c"glGetProgramInfoLog": fn(u32, i32, *mut i32, *mut u8);
    GetUniformLocation = c"glGetUniformLocation": fn(u32, *const i8) -> i32;
    UseProgram = c"glUseProgram": fn(u32);
    UniformMatrix4fv = c"glUniformMatrix4fv": fn(i32, i32, u8, *const f32);
    Uniform1i = c"glUniform1i": fn(i32, i32);
    Uniform1f = c"glUniform1f": fn(i32, f32);
    Uniform2f = c"glUniform2f": fn(i32, f32, f32);
    Uniform3f = c"glUniform3f": fn(i32, f32, f32, f32);
    GenBuffers = c"glGenBuffers": fn(i32, *mut u32);
    BindBuffer = c"glBindBuffer": fn(u32, u32);
    BufferData = c"glBufferData": fn(u32, isize, *const c_void, u32);
    VertexAttribPointer = c"glVertexAttribPointer": fn(u32, i32, u32, u8, i32, *const c_void);
    EnableVertexAttribArray = c"glEnableVertexAttribArray": fn(u32);
    DisableVertexAttribArray = c"glDisableVertexAttribArray": fn(u32);
    DrawArrays = c"glDrawArrays": fn(u32, i32, i32);
    Enable = c"glEnable": fn(u32);
    Disable = c"glDisable": fn(u32);
    BlendFunc = c"glBlendFunc": fn(u32, u32);
    DepthMask = c"glDepthMask": fn(u8);
    ColorMask = c"glColorMask": fn(u8, u8, u8, u8);
    ActiveTexture = c"glActiveTexture": fn(u32);
    BindTexture = c"glBindTexture": fn(u32, u32);
}

struct Program {
    id: u32,
    u_view_proj: i32,
    u_gdepth: i32,
    u_size: i32,
    u_fog: i32,
    u_sun: i32,
    u_amb_low: i32,
    u_amb_high: i32,
}

struct Renderer {
    gl: Gl,
    vbo: u32,
    ribbons: Program,
    clouds: Program,
}

/// Made on the first draw (the render thread, the only GL thread); None if it failed (logged).
fn renderer_gl() -> Option<&'static Renderer> {
    struct Shared(Option<Renderer>);
    unsafe impl Sync for Shared {}
    unsafe impl Send for Shared {}
    static R: OnceLock<Shared> = OnceLock::new();
    R.get_or_init(|| {
        let r = unsafe { make_renderer() };
        if let Err(e) = &r {
            log_line(&format!("effects: {e}"));
        }
        Shared(r.ok())
    })
    .0
    .as_ref()
}

unsafe fn make_renderer() -> Result<Renderer, String> {
    let gl = load_gl().ok_or("GL entry points missing")?;
    let ribbons = link(&gl, VERTEX, FRAGMENT).map_err(|e| format!("trail {e}"))?;
    let clouds = link(&gl, CLOUD_VERTEX, CLOUD_FRAGMENT).map_err(|e| format!("wing mist {e}"))?;
    let mut vbo = 0;
    (gl.GenBuffers)(1, &mut vbo);
    log_line("effects: shaders ready");
    Ok(Renderer { gl, vbo, ribbons, clouds })
}

unsafe fn link(gl: &Gl, vertex: &str, fragment: &str) -> Result<Program, String> {
    let compile = |kind: u32, src: &str| -> Result<u32, String> {
        let src = CString::new(src).map_err(|e| e.to_string())?;
        let s = (gl.CreateShader)(kind);
        (gl.ShaderSource)(s, 1, &src.as_ptr(), std::ptr::null());
        (gl.CompileShader)(s);
        let mut ok = 0;
        (gl.GetShaderiv)(s, GL_COMPILE_STATUS, &mut ok);
        if ok == 0 {
            let mut buf = [0u8; 1024];
            let mut n = 0;
            (gl.GetShaderInfoLog)(s, buf.len() as i32, &mut n, buf.as_mut_ptr());
            return Err(format!("shader compile: {}", String::from_utf8_lossy(&buf[..n.clamp(0, 1024) as usize])));
        }
        Ok(s)
    };
    let (vs, fs) = (compile(GL_VERTEX_SHADER, vertex)?, compile(GL_FRAGMENT_SHADER, fragment)?);
    let program = (gl.CreateProgram)();
    (gl.AttachShader)(program, vs);
    (gl.AttachShader)(program, fs);
    (gl.LinkProgram)(program);
    let mut ok = 0;
    (gl.GetProgramiv)(program, GL_LINK_STATUS, &mut ok);
    if ok == 0 {
        let mut buf = [0u8; 1024];
        let mut n = 0;
        (gl.GetProgramInfoLog)(program, buf.len() as i32, &mut n, buf.as_mut_ptr());
        return Err(format!("shader link: {}", String::from_utf8_lossy(&buf[..n.clamp(0, 1024) as usize])));
    }
    let u = |n: &CStr| (gl.GetUniformLocation)(program, n.as_ptr());
    Ok(Program {
        id: program,
        u_view_proj: u(c"view_proj"),
        u_gdepth: u(c"gdepth"),
        u_size: u(c"target_size"),
        u_fog: u(c"fog_density"),
        u_sun: u(c"sun_col"),
        u_amb_low: u(c"amb_low"),
        u_amb_high: u(c"amb_high"),
    })
}

const GL_VERTEX_SHADER: u32 = 0x8B31;
const GL_FRAGMENT_SHADER: u32 = 0x8B30;
const GL_COMPILE_STATUS: u32 = 0x8B81;
const GL_LINK_STATUS: u32 = 0x8B82;
const GL_ARRAY_BUFFER: u32 = 0x8892;
const GL_STREAM_DRAW: u32 = 0x88E0;
const GL_FLOAT: u32 = 0x1406;
const GL_TRIANGLES: u32 = 0x0004;
const GL_BLEND: u32 = 0x0BE2;
const GL_DEPTH_TEST: u32 = 0x0B71;
const GL_CULL_FACE: u32 = 0x0B44;
const GL_ONE: u32 = 1;
const GL_ONE_MINUS_SRC_ALPHA: u32 = 0x0303;
const GL_TEXTURE0: u32 = 0x84C0;
const GL_TEXTURE_2D: u32 = 0x0DE1;

impl Renderer {
    /// Premultiplied-alpha triangles over the lit scene; hidden behind geometry by the shader (the
    /// G-buffer depth), so no depth test. Leaves the state the game expects after its own passes
    /// (it sets depth, cull and blend itself right after this call).
    #[allow(clippy::too_many_arguments)]
    unsafe fn draw(&self, p: &Program, data: &[f32], attrs: &[usize], view_proj: &[f32; 16], size: (f32, f32), depth: u32, fog: f32, light: &Light) {
        let stride: usize = attrs.iter().sum();
        if data.len() < stride {
            return;
        }
        let gl = &self.gl;
        (gl.UseProgram)(p.id);
        (gl.UniformMatrix4fv)(p.u_view_proj, 1, 0, view_proj.as_ptr());
        (gl.Uniform1i)(p.u_gdepth, 0);
        (gl.Uniform2f)(p.u_size, size.0.max(1.0), size.1.max(1.0));
        (gl.Uniform1f)(p.u_fog, fog);
        let v3 = |at: i32, v: [f32; 3]| (gl.Uniform3f)(at, v[0], v[1], v[2]);
        v3(p.u_sun, light.sun);
        v3(p.u_amb_low, light.amb_low);
        v3(p.u_amb_high, light.amb_high);
        (gl.ActiveTexture)(GL_TEXTURE0);
        (gl.BindTexture)(GL_TEXTURE_2D, depth);
        (gl.BindBuffer)(GL_ARRAY_BUFFER, self.vbo);
        (gl.BufferData)(GL_ARRAY_BUFFER, std::mem::size_of_val(data) as isize, data.as_ptr().cast(), GL_STREAM_DRAW);
        let mut at = 0;
        for (i, &n) in attrs.iter().enumerate() {
            (gl.VertexAttribPointer)(i as u32, n as i32, GL_FLOAT, 0, (stride * 4) as i32, (at * 4) as *const c_void);
            (gl.EnableVertexAttribArray)(i as u32);
            at += n;
        }
        (gl.Disable)(GL_DEPTH_TEST);
        (gl.DepthMask)(0);
        (gl.Disable)(GL_CULL_FACE);
        (gl.ColorMask)(1, 1, 1, 1);
        (gl.Enable)(GL_BLEND);
        (gl.BlendFunc)(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        (gl.DrawArrays)(GL_TRIANGLES, 0, (data.len() / stride) as i32);
        (gl.Disable)(GL_BLEND);
        for i in 0..attrs.len() {
            (gl.DisableVertexAttribArray)(i as u32);
        }
        (gl.BindBuffer)(GL_ARRAY_BUFFER, 0);
        (gl.BindTexture)(GL_TEXTURE_2D, 0);
        (gl.UseProgram)(0);
    }
}


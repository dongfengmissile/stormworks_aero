//! Motion-blurred propeller and rotor blades: a fast blade is drawn as several screen-door
//! (dithered) copies spread over the angle it sweeps in about 1/20 s, like a camera or the eye
//! sees it, instead of one sharp blade. The game has no alpha blending for vehicle meshes, so
//! the copies go through its dithered opaque pass (lit, depth-tested; knowledge/rendering.md
//! "Translucent draws / rotor blur").
//!
//! `force_emitter_draw` is called once more as usual ("keep": hub drawn, blades dropped) and N
//! times rotated about the shaft ("copy": blades only). The calls only queue 352-byte items, so
//! each call tags its colours with a magic carrying a table index; the item reaches
//! `push_render_item_232` later on the same thread, where the tag picks the route.

use std::cell::{Cell, RefCell};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::collections::HashMap;
use std::sync::{LazyLock, Mutex, OnceLock};

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable};
use crate::offsets::*;

/// Settings tab switch.
pub(crate) static ENABLED: AtomicBool = AtomicBool::new(true);

pub(crate) type ClassDraw = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) type Push232 =
    unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) type PushShadow = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
type PushDither = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;

pub(crate) static ORIGINAL_DRAW: OnceLock<ClassDraw> = OnceLock::new();
pub(crate) type Mesh1Draw = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) static ORIGINAL_MESH1: OnceLock<Mesh1Draw> = OnceLock::new();
pub(crate) type EditorDraw = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) static ORIGINAL_EDITOR_DRAW: OnceLock<EditorDraw> = OnceLock::new();
pub(crate) static ORIGINAL_232: OnceLock<Push232> = OnceLock::new();
pub(crate) static ORIGINAL_SHADOW_A: OnceLock<PushShadow> = OnceLock::new();
pub(crate) static ORIGINAL_SHADOW_B: OnceLock<PushShadow> = OnceLock::new();
pub(crate) static PUSH_DITHER: OnceLock<PushDither> = OnceLock::new();

/// Colour tag: alpha 0xB1 (game colours have 0xFF, paint tints 0x5A), bit 23 copy/keep,
/// bits 8..23 table index, low byte dither opacity.
const MAGIC: u32 = 0xB100_0000;
const COPY: u32 = 0x0080_0000;
const SLOTS: usize = 1 << 15;
/// Sweep shown as one blurred blade, s (between a camera shutter and the eye's persistence).
const EXPOSURE: f64 = 1.0 / 20.0;
/// Sweep below which a blade is drawn sharp, rad.
const MIN_SWEEP: f64 = 0.15;
/// A blade's angular width (rad) for its smeared opacity, and the copy spacing.
const BLADE_WIDTH: f64 = 0.12;
const STEP: f64 = 0.08;
const MAX_COPIES: usize = 24;

/// Hinged rotors (`comp+0x2F0`): the hub is drawn at the hinge's physical angle, which turns
/// slower than the blades' real speed. Per part: last hinge angle, and the extra drawn angle (rad)
/// that makes the drawn hub advance by the real step each tick, in the hinge's direction, and
/// whether the hinge turns against the shaft speed's sign (the model's default spin).
static HINGE_EXTRA: LazyLock<Mutex<HashMap<usize, (f64, f64, bool, u32)>>> = LazyLock::new(Default::default);

fn wrap_pi(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// After a prop tick (sim thread): `true_step` = this tick's real blade travel (any wrap), rad.
pub(crate) unsafe fn track_hinge(comp: usize, true_step: f64) {
    let hinge = read_usize_at(comp + OFF_PROP_HINGE);
    let def = read_usize_at(comp + 0x58);
    if hinge == 0 || !readable(hinge + OFF_HINGE_ANGLE, 4) || !readable(def + OFF_DEF_MIRROR, 1) {
        return;
    }
    // Drawn hub angle = −hinge angle · sy (air props).
    let sy = if *((def + OFF_DEF_MIRROR) as *const u8) & 2 != 0 { -1.0 } else { 1.0 };
    let h = -(f32::from_bits(read_u32_at(hinge + OFF_HINGE_ANGLE)) as f64) * sy;
    let mut m = HINGE_EXTRA.lock().unwrap_or_else(|p| p.into_inner());
    if m.len() > 4096 {
        m.clear(); // dead parts
    }
    let e = m.entry(comp).or_insert((h, 0.0, false, 0));
    let dh = wrap_pi(h - e.0);
    e.0 = h;
    if dh.abs() > 1e-6 && true_step.is_finite() {
        e.1 = (e.1 + dh.signum() * wrap_pi(true_step).abs() - dh).rem_euclid(std::f64::consts::TAU);
        // The model turns the way comp+0x148 does (part frame, before the part's own mirroring).
        let against = dh.signum() != wrap_pi(true_step).signum();
        // Spin-up and spin-down jitter: switch only after half a second of steady turning.
        e.3 = if against != e.2 && dh.abs() > 0.01 && wrap_pi(true_step).abs() > 0.01 { e.3 + 1 } else { 0 };
        if e.3 >= 30 {
            (e.2, e.3) = (against, 0);
            log_line(&format!("rotor {comp:#x}: hinge turns {} the shaft speed's spin; the model follows the hinge", if against { "against" } else { "with" }));
        }
    }
}

/// The hinge drives this rotor the other way from its shaft speed's sign: the model flips its spin.
pub(crate) fn hinge_against(comp: usize) -> bool {
    HINGE_EXTRA.lock().unwrap_or_else(|p| p.into_inner()).get(&comp).is_some_and(|e| e.2)
}

fn hinge_extra(comp: usize) -> f64 {
    HINGE_EXTRA.lock().unwrap_or_else(|p| p.into_inner()).get(&comp).map_or(0.0, |e| e.1)
}

#[derive(Clone, Copy, Default)]
struct Slot {
    blade: usize,
    colours: [u32; 3],
}

thread_local! {
    static SLOTS_T: RefCell<Vec<Slot>> = const { RefCell::new(Vec::new()) };
    static NEXT: Cell<usize> = const { Cell::new(0) };
    /// matA pointer of the item whose shadow pushes are dropped (blur copies, hidden blades).
    static SWALLOW: Cell<usize> = const { Cell::new(0) };
    /// While a propeller part draws: its definition and blade width (x the mesh's own).
    static WIDEN: Cell<(usize, f64)> = const { Cell::new((0, 1.0)) };
}

/// Diagnostics: copy items routed as blade / dropped as hub.
static N_BLADE: AtomicU64 = AtomicU64::new(0);
static N_HUB: AtomicU64 = AtomicU64::new(0);
static LOGGED: AtomicBool = AtomicBool::new(false);

fn new_slot(s: Slot) -> u32 {
    let i = NEXT.with(|n| {
        let i = n.get();
        n.set((i + 1) % SLOTS);
        i
    });
    SLOTS_T.with(|t| {
        let mut t = t.borrow_mut();
        if t.len() < SLOTS {
            t.resize(SLOTS, Slot::default());
        }
        t[i] = s;
    });
    i as u32
}

fn slot(i: u32) -> Slot {
    SLOTS_T.with(|t| t.borrow().get(i as usize).copied().unwrap_or_default())
}

/// The blur this draw: (blade mesh holder, shaft axis, sweep rad, copies, opacity), or None.
unsafe fn blur_of(comp: usize) -> Option<(usize, [f64; 3], f64, usize, f32)> {
    if !ENABLED.load(Ordering::Relaxed) || !readable(comp, OFF_PROP_ACTIVE + 4) {
        return None;
    }
    let def = read_usize_at(comp + 0x58);
    if !readable(def + OFF_DEF_PROP_FLAGS, 4) || read_u32_at(def + OFF_DEF_PROP_FLAGS) & 2 != 0 || !readable(def + OFF_DEF_FORCE_DIR, 12) {
        return None;
    }
    if *((comp + OFF_PROP_FOLDED) as *const u8) != 0 {
        return None;
    }
    let base = read_usize_at(def);
    if !readable(base + OFF_DEF_MESH1 + 0x30, 8) {
        return None;
    }
    // The pushes' a2 is the holder itself: `*(mesh_obj + 0x30)` (push_block_items 0x2CDF84).
    let blade = read_usize_at(base + OFF_DEF_MESH1 + 0x30);
    if blade == 0 || !readable(blade, 16) {
        return None;
    }
    let f = |o: usize| f32::from_bits(read_u32_at(comp + o)) as f64;
    let tau = std::f64::consts::TAU;
    let d = (f(OFF_PROP_ANGLE) - f(OFF_PROP_ANGLE + 4) + std::f64::consts::PI).rem_euclid(tau) - std::f64::consts::PI;
    let blades = read_u32_at(comp + OFF_PROP_BLADES).clamp(1, 8) as f64;
    // d is one 60 Hz tick of travel.
    let sweep = (d.abs() * 60.0 * EXPOSURE).min(tau / blades);
    if !(sweep > MIN_SWEEP) {
        return None;
    }
    let axis = force_dir(comp)?;
    let copies = ((sweep / STEP).ceil() as usize).clamp(2, MAX_COPIES);
    let opacity = (BLADE_WIDTH / sweep).clamp(0.1, 1.0) as f32;
    Some((blade, axis, sweep, copies, opacity))
}

/// The part's spin axis (def `force_dir`), unit length.
unsafe fn force_dir(comp: usize) -> Option<[f64; 3]> {
    let def = read_usize_at(comp + 0x58);
    if !readable(def + OFF_DEF_FORCE_DIR, 12) {
        return None;
    }
    let a: [f64; 3] = std::array::from_fn(|k| f32::from_bits(read_u32_at(def + OFF_DEF_FORCE_DIR + 4 * k)) as f64);
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    (n > 0.5).then(|| a.map(|v| v / n))
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn draw_detour(c: usize, q: usize, ma: usize, mb: usize, a5: usize, a6: usize, a7: usize, a8: usize, a9: usize, a10: usize) -> usize {
    widened(c, || draw_part(c, q, ma, mb, a5, a6, a7, a8, a9, a10))
}

/// The editor's highlight copy (Select tool): widened like the part itself.
#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn editor_draw_detour(c: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, a7: usize, a8: usize) -> usize {
    let Some(orig) = ORIGINAL_EDITOR_DRAW.get() else { return 0 };
    widened(c, || orig(c, a2, a3, a4, a5, a6, a7, a8))
}

/// Run a part's draw with its blade width set for `mesh1_detour`.
unsafe fn widened(c: usize, draw: impl FnOnce() -> usize) -> usize {
    let w = crate::marks::get(c).and_then(|m| m.blade_width).map_or(1.0, |w| w as f64);
    if w == 1.0 || !readable(c + 0x58, 8) {
        return draw();
    }
    WIDEN.with(|x| x.set((read_usize_at(c + 0x58), w)));
    let r = draw();
    WIDEN.with(|x| x.set((0, 1.0)));
    r
}

/// One blade (`draw_def_mesh1`): widened across its span while a part with a blade width draws.
/// The blade mesh spans along x (`force_emitter_draw`: Pre = S·P·RotY·R_hub), so scaling the
/// mesh's y and z (rows 1 and 2 of the row-vector matrix) widens chord and thickness together
/// and keeps the twist.
#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn mesh1_detour(def: usize, q: usize, ma: usize, mb: usize, a5: usize, a6: usize, a7: usize, a8: usize, a9: usize) -> usize {
    let Some(orig) = ORIGINAL_MESH1.get() else { return 0 };
    let (d, w) = WIDEN.with(|x| x.get());
    if d != def || w == 1.0 || !readable(ma, 128) || !readable(mb, 128) {
        return orig(def, q, ma, mb, a5, a6, a7, a8, a9);
    }
    let widen = |p: usize| {
        let mut m = (p as *const [f64; 16]).read_unaligned();
        for j in 4..12 {
            m[j] *= w;
        }
        m
    };
    let (wa, wb) = (widen(ma), widen(mb));
    orig(def, q, wa.as_ptr() as usize, wb.as_ptr() as usize, a5, a6, a7, a8, a9)
}

#[allow(clippy::too_many_arguments)]
unsafe fn draw_part(c: usize, q: usize, ma: usize, mb: usize, a5: usize, a6: usize, a7: usize, a8: usize, a9: usize, a10: usize) -> usize {
    let Some(orig) = ORIGINAL_DRAW.get() else { return 0 };
    let blur = std::panic::catch_unwind(|| blur_of(c)).ok().flatten();
    let phi = hinge_extra(c);
    if (blur.is_none() && phi == 0.0) || !readable(ma, 128) || !readable(mb, 128) {
        return orig(c, q, ma, mb, a5, a6, a7, a8, a9, a10);
    }
    let (m_a, m_b) = ((ma as *const [f64; 16]).read_unaligned(), (mb as *const [f64; 16]).read_unaligned());
    let Some((blade, axis, sweep, copies, opacity)) = blur else {
        // A slow hinged rotor: sharp, at its real angle.
        let Some(axis) = force_dir(c) else { return orig(c, q, ma, mb, a5, a6, a7, a8, a9, a10) };
        let (ra, rb) = (crate::math::rotated4(&m_a, axis, phi), crate::math::rotated4(&m_b, axis, phi));
        return orig(c, q, ra.as_ptr() as usize, rb.as_ptr() as usize, a5, a6, a7, a8, a9, a10);
    };
    // Hub in its own colours (or the caller's override), blades dropped.
    let colours = if a8 & 0xFF != 0 { [a10 as u32; 3] } else { [read_u32_at(c + 0x60), read_u32_at(c + 0x64), read_u32_at(c + 0x68)] };
    let keep = MAGIC | new_slot(Slot { blade, colours }) << 8;
    let (ka, kb) = (crate::math::rotated4(&m_a, axis, phi), crate::math::rotated4(&m_b, axis, phi));
    let r = orig(c, q, ka.as_ptr() as usize, kb.as_ptr() as usize, a5, a6, a7, 1, a9, keep as usize);
    let tag = MAGIC | COPY | new_slot(Slot { blade, colours }) << 8 | (opacity * 255.0).round().clamp(1.0, 255.0) as u32;
    for k in 0..copies {
        let delta = phi + sweep * (k as f64 / (copies - 1) as f64 - 0.5);
        let (ra, rb) = (crate::math::rotated4(&m_a, axis, delta), crate::math::rotated4(&m_b, axis, delta));
        orig(c, q, ra.as_ptr() as usize, rb.as_ptr() as usize, a5, a6, a7, 1, a9, tag as usize);
    }
    r
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn push232_detour(
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    ma: usize,
    mb: usize,
    bounds: usize,
    light: usize,
    c1: usize,
    c2: usize,
    c3: usize,
    a12: usize,
    a13: usize,
    a14: usize,
    a15: usize,
    a16: usize,
) -> usize {
    let Some(orig) = ORIGINAL_232.get() else { return 0 };
    SWALLOW.with(|s| s.set(0));
    let tag = c3 as u32;
    if tag & 0xFF00_0000 != MAGIC {
        return orig(a1, a2, a3, a4, ma, mb, bounds, light, c1, c2, c3, a12, a13, a14, a15, a16);
    }
    let s = slot((tag >> 8) & 0x7FFF);
    let is_blade = a2 == s.blade;
    if tag & COPY == 0 {
        if is_blade {
            return 0; // the sharp blade, replaced by the copies (its shadow stays)
        }
        let [k1, k2, k3] = s.colours.map(|c| c as usize);
        return orig(a1, a2, a3, a4, ma, mb, bounds, light, k1, k2, k3, a12, a13, a14, a15, a16);
    }
    SWALLOW.with(|s| s.set(ma));
    if !is_blade {
        // A blade holder that never matches leaves the sharp blades (kept as "hub"): say so once.
        if N_HUB.fetch_add(1, Ordering::Relaxed) == 5000 && N_BLADE.load(Ordering::Relaxed) == 0 && !LOGGED.swap(true, Ordering::Relaxed) {
            log_line("rotor blur: blade mesh never matched (OFF_DEF_MESH1 holder); blades stay sharp");
        }
        return 0;
    }
    let n = N_BLADE.fetch_add(1, Ordering::Relaxed);
    if n == 1000 && !LOGGED.swap(true, Ordering::Relaxed) {
        log_line(&format!("rotor blur: {} blade copies drawn, {} hub copies dropped", n, N_HUB.load(Ordering::Relaxed)));
    }
    let Some(dither) = PUSH_DITHER.get() else { return 0 };
    let opacity = (tag & 0xFF) as f32 / 255.0;
    let r = dither(a1, a2, 0, 0, ma, mb, bounds, 1.0f32.to_bits() as usize, s.colours[0] as usize, 0, 0, 0, opacity.to_bits() as usize);
    // Mirrored parts cull front faces: the new item takes the original's flag.
    let ring = a1 + OFF_QUEUE_DITHER_RING;
    let (base, cap, off, count) = (read_usize_at(ring), read_u32_at(ring + 8), read_u32_at(ring + 0xC), read_u32_at(ring + 0x10));
    if base != 0 && cap != 0 && count != 0 {
        let item = base + 240 * ((count - 1 + off) % cap) as usize;
        *((item + 0xDC) as *mut u8) = a14 as u8;
    }
    r
}

#[allow(clippy::too_many_arguments)]
unsafe fn shadow(orig: Option<&PushShadow>, a1: usize, a2: usize, a3: usize, a4: usize, ma: usize, a6: usize, a7: usize, a8: usize, a9: usize) -> usize {
    let Some(orig) = orig else { return 0 };
    if ma != 0 && SWALLOW.with(|s| s.get()) == ma {
        return 0;
    }
    orig(a1, a2, a3, a4, ma, a6, a7, a8, a9)
}

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn shadow_a_detour(a1: usize, a2: usize, a3: usize, a4: usize, ma: usize, a6: usize, a7: usize, a8: usize, a9: usize) -> usize {
    shadow(ORIGINAL_SHADOW_A.get(), a1, a2, a3, a4, ma, a6, a7, a8, a9)
}
#[allow(clippy::too_many_arguments)]
pub(crate) unsafe extern "system" fn shadow_b_detour(a1: usize, a2: usize, a3: usize, a4: usize, ma: usize, a6: usize, a7: usize, a8: usize, a9: usize) -> usize {
    shadow(ORIGINAL_SHADOW_B.get(), a1, a2, a3, a4, ma, a6, a7, a8, a9)
}

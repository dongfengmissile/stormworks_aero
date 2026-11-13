//! Watch camera: the game's own third-person orbit, centred on a vehicle picked in the F9 window
//! (knowledge/camera.md). The flight tick publishes the watched vehicle's CoM; the render thread's
//! camera step writes it into the controller's anchor (A now, B the tick before, which the game
//! blends), so mouse orbit and scroll zoom are the game's.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, OnceLock};

use crate::math::V3;
use crate::memory::{bytes_match, readable};

pub(crate) const RVA_CAMERA_STEP: usize = 0x0071_b630;
pub(crate) const CAMERA_STEP_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0xF3, 0x0F, 0x11, 0x50, 0x18, 0x55, 0x57, 0x48, 0x81, 0xEC, 0xC8, 0x00, 0x00, 0x00, 0x0F, 0x29, 0x78, 0xC8, 0x48, 0x8B, 0xF9];
pub(crate) const RVA_CAMERA_STATUS: usize = 0x0071_be90;
pub(crate) const CAMERA_STATUS_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x18, 0x48, 0x89, 0x70, 0x20, 0x55, 0x57, 0x41, 0x56, 0x48, 0x8D, 0x68, 0xA1, 0x48, 0x81, 0xEC, 0xC0, 0x00];
/// `game_window_set_cursor_disabled(W, u8 disabled)`: the game sets the GLFW cursor mode with it
/// every frame (knowledge/input.md). Bytes past the 12th are a RIP-relative operand.
pub(crate) const RVA_SET_CURSOR_DISABLED: usize = 0x001b_dc70;
pub(crate) const SET_CURSOR_DISABLED_PROLOGUE: [u8; 12] = [0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x83, 0x3D];
pub(crate) type SetCursorDisabled = unsafe extern "system" fn(usize, usize) -> usize;
pub(crate) static ORIGINAL_SET_CURSOR: OnceLock<SetCursorDisabled> = OnceLock::new();
/// The overlay wants a free cursor (the F8 list is open).
pub(crate) static FREE_CURSOR: AtomicBool = AtomicBool::new(false);

/// While `FREE_CURSOR`, the game's per-frame "hide and lock" becomes "normal cursor".
pub(crate) unsafe extern "system" fn set_cursor_detour(w: usize, disabled: usize) -> usize {
    let Some(&orig) = ORIGINAL_SET_CURSOR.get() else { return 0 };
    orig(w, if FREE_CURSOR.load(Ordering::Relaxed) { 0 } else { disabled })
}

/// The camera controller inside the simulation game state (`G`).
const OFF_SIM_CAMERA_CTRL: usize = 0xCE8;
/// After the watched vehicle is gone: the camera stays at its last place this long (see the impact).
const HOLD_MS: u64 = 2000;
/// Not published for this long: the vehicle is gone (destroyed, despawned, out of the flight model).
const LOST_MS: u64 = 300;

pub(crate) const RVA_CAMERA_SYNC: usize = 0x0071_af80;
/// `camera_ctrl_sync_from_frame`; bytes past the 22nd are a RIP-relative operand.
pub(crate) const CAMERA_SYNC_PROLOGUE: [u8; 22] =
    [0x4C, 0x8B, 0xDC, 0x55, 0x56, 0x57, 0x41, 0x54, 0x49, 0x8D, 0x6B, 0xA1, 0x48, 0x81, 0xEC, 0xC8, 0x00, 0x00, 0x00, 0x48, 0x8B, 0x05];
pub(crate) const RVA_MOUSE_LOOK_ALLOWED: usize = 0x007b_5eb0;
pub(crate) const MOUSE_LOOK_ALLOWED_PROLOGUE: [u8; 16] = [0x0F, 0xB6, 0x91, 0x70, 0x3C, 0x00, 0x00, 0x84, 0xD2, 0x74, 0x09, 0x80, 0xB9, 0xF8, 0x1A, 0x00];
/// In the simulation state: "mouse button 1 held", which lets the mouse look while seated.
const OFF_SIM_LOOK_BUTTON: usize = 0x1AF8;
pub(crate) type CameraSync = unsafe extern "system" fn(usize, usize, usize, usize) -> usize;
pub(crate) type MouseLookAllowed = unsafe extern "system" fn(usize, usize, usize, usize) -> u8;
pub(crate) static ORIGINAL_SYNC: OnceLock<CameraSync> = OnceLock::new();
/// `game_state_in_game_build_frame(S, ..)` (sim thread, once per tick): its part-detail gates
/// (750 m: every component drawn with its own mesh, e.g. rocket fins) measure from the camera
/// block's anchor `S+0xAC0+0x60` (knowledge/camera.md, LOD).
pub(crate) const RVA_BUILD_FRAME: usize = 0x0074_c470;
pub(crate) const BUILD_FRAME_PROLOGUE: [u8; 32] = [
    0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xA8, 0x18, 0xFB, 0xFF, 0xFF, 0x48, 0x81,
    0xEC, 0xB0, 0x05, 0x00, 0x00,
];
const OFF_SIM_SCENE_ANCHOR_POS: usize = 0xAC0 + 0x60;
pub(crate) type BuildFrame = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize) -> usize;
pub(crate) static ORIGINAL_BUILD_FRAME: OnceLock<BuildFrame> = OnceLock::new();
pub(crate) static ORIGINAL_LOOK: OnceLock<MouseLookAllowed> = OnceLock::new();
pub(crate) type CameraStep = unsafe extern "system" fn(usize, usize, f32, usize) -> usize;
pub(crate) type CameraStatus = unsafe extern "system" fn(usize, usize, usize, usize) -> usize;
pub(crate) static ORIGINAL_STEP: OnceLock<CameraStep> = OnceLock::new();
pub(crate) static ORIGINAL_STATUS: OnceLock<CameraStatus> = OnceLock::new();
/// Both hooks are in: the F9 window offers Watch.
pub(crate) static AVAILABLE: AtomicBool = AtomicBool::new(false);
/// The orbit is overridden this frame (the status hook then keeps the player's look and listener).
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// One vehicle for the F8 list (flight tick, every vehicle the plugin models).
#[derive(Clone, Copy)]
pub(crate) struct Row {
    pub id: u32,
    /// CoM and velocity, world.
    pub com: V3,
    pub vel: V3,
    pub mass: f64,
    pub parts: usize,
}
/// Rows being gathered this tick, and the last complete tick's.
static GATHER: Mutex<Vec<Row>> = Mutex::new(Vec::new());
static ROWS: Mutex<Vec<Row>> = Mutex::new(Vec::new());

/// Flight tick: one vehicle's row.
pub(crate) fn add_row(r: Row) {
    GATHER.lock().unwrap_or_else(|p| p.into_inner()).push(r);
}

/// Flight tick end: the gathered rows become the list.
pub(crate) fn publish_rows() {
    let rows = std::mem::take(&mut *GATHER.lock().unwrap_or_else(|p| p.into_inner()));
    *ROWS.lock().unwrap_or_else(|p| p.into_inner()) = rows;
}

pub(crate) fn rows() -> Vec<Row> {
    ROWS.lock().map(|r| r.clone()).unwrap_or_default()
}

struct Target {
    id: u32,
    /// Tick positions of the CoM (fallback when no frame-matched pose is recorded).
    prev: Option<V3>,
    cur: Option<V3>,
    /// The main body and the CoM in its grid frame: placed with the game's own built poses.
    body: usize,
    local: V3,
    seen_ms: u64,
    /// Last anchor written (held after the vehicle is gone).
    last: Option<V3>,
}

static TARGET: Mutex<Option<Target>> = Mutex::new(None);

/// Start watching vehicle `id` (None: back to the player).
pub(crate) fn set(id: Option<u32>) {
    *TARGET.lock().unwrap_or_else(|p| p.into_inner()) = id.map(|id| Target { id, prev: None, cur: None, body: 0, local: V3::ZERO, seen_ms: crate::editor::now_ms(), last: None });
}

pub(crate) fn watched() -> Option<u32> {
    TARGET.lock().ok()?.as_ref().map(|t| t.id)
}

/// Flight tick (game thread), every vehicle: its CoM (world), main body and the CoM in its grid frame.
pub(crate) fn note(id: u32, com: V3, body: usize, local: V3) {
    let mut g = TARGET.lock().unwrap_or_else(|p| p.into_inner());
    if let Some(t) = g.as_mut().filter(|t| t.id == id && com.len().is_finite()) {
        t.prev = t.cur.or(Some(com));
        t.cur = Some(com);
        (t.body, t.local) = (body, local);
        t.seen_ms = crate::editor::now_ms();
    }
}

/// The CoM in the two builds the game blends for the frame being drawn (as the body itself is
/// drawn): the camera then moves exactly with the vehicle, without the tick lag.
fn built_anchor(body: usize, local: V3) -> Option<(V3, V3)> {
    // Read here, inside slot 7 after the game swapped in this frame's build: read before slot 7
    // it is still the previous build whenever a new one arrived (the double image).
    let ts = crate::flight::frame_build_ts(crate::flight::sim_state())?;
    if body == 0 {
        return None;
    }
    let h = crate::flight::BUILT_HISTORY.lock().ok()?.get(&body).cloned().unwrap_or_default();
    let (prev, cur) = match h.iter().position(|(t, _)| *t == ts) {
        Some(i) => (h[i.saturating_sub(1)].1, h[i].1),
        // Built again after our last record: that pose is already in the body.
        None if h.last().is_some_and(|(t, _)| *t < ts) => (h.last()?.1, crate::flight::body_built_matrix(body)?),
        None => return None,
    };
    let place = |m: [f64; 16]| crate::math::v3(
        local.x * m[0] + local.y * m[4] + local.z * m[8] + m[12],
        local.x * m[1] + local.y * m[5] + local.z * m[9] + m[13],
        local.x * m[2] + local.y * m[6] + local.z * m[10] + m[14],
    );
    Some((place(cur), place(prev)))
}

/// The anchor for this frame: (now, tick before). Ends the watch once the hold is over.
fn anchor() -> Option<(V3, V3)> {
    let mut g = TARGET.lock().unwrap_or_else(|p| p.into_inner());
    let t = g.as_mut()?;
    let age = crate::editor::now_ms().saturating_sub(t.seen_ms);
    if age > LOST_MS + HOLD_MS {
        *g = None;
        return None;
    }
    // Lost: hold still at the last place.
    if age > LOST_MS {
        let p = t.last.or(t.cur)?;
        return Some((p, p));
    }
    let cur = t.cur?;
    let a = built_anchor(t.body, t.local).filter(|(now, _)| (*now - cur).len() < 100.0).unwrap_or((cur, t.prev.unwrap_or(cur)));
    t.last = Some(a.0);
    Some(a)
}

/// Watch anchor into a camera block (`C` itself, or the sim's block the sync copies into `C`):
/// A = now, B = the tick before, identity rotation (orbit about world up).
unsafe fn write_anchor(base: usize, now: V3, before: V3) {
    for (at, p) in [(base, now), (base + 0x80, before)] {
        let mut m = [0.0f64; 16];
        (m[0], m[5], m[10], m[15]) = (1.0, 1.0, 1.0, 1.0);
        (m[12], m[13], m[14]) = (p.x, p.y, p.z);
        (at as *mut [f64; 16]).write_unaligned(m);
    }
}

/// `camera_ctrl_sync_from_frame(C, block)`: copies the sim's camera block into `C` once per new
/// tick, and zeroes yaw/pitch when its flags disagree with `C`'s (a seated player's first-person
/// look). So the watch flags go into the block, and `C` and the block always agree.
pub(crate) unsafe extern "system" fn sync_detour(c: usize, block: usize, a3: usize, a4: usize) -> usize {
    let Some(&orig) = ORIGINAL_SYNC.get() else { return 0 };
    let _ = std::panic::catch_unwind(|| unsafe {
        if let Some((now, before)) = anchor() {
            if c == crate::flight::sim_state() + OFF_SIM_CAMERA_CTRL && readable(block, 0x110) {
                write_anchor(block, now, before);
                // Block flags +0x108..: seat look limits, seated, third person, free cam x2, ?, orbit, ?
                for (k, v) in [0u8, 1, 1, 0, 0, 0, 0, 0].into_iter().enumerate() {
                    *((block + 0x108 + k) as *mut u8) = v;
                }
                *((block + 0x104) as *mut f32) = f32::MAX;
            }
        }
    });
    orig(c, block, a3, a4)
}

/// Build the frame with the anchor at the watched vehicle (its tick CoM), so its parts keep their
/// detail however far the player is; the player's own anchor is put back right after (tile
/// streaming and the listener read it outside this call).
pub(crate) unsafe extern "system" fn build_frame_detour(s: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize) -> usize {
    let Some(&orig) = ORIGINAL_BUILD_FRAME.get() else { return 0 };
    let at = (s + OFF_SIM_SCENE_ANCHOR_POS) as *mut [f64; 3];
    let pos = TARGET
        .lock()
        .ok()
        .and_then(|g| g.as_ref().filter(|t| crate::editor::now_ms().saturating_sub(t.seen_ms) <= LOST_MS).and_then(|t| t.cur))
        .filter(|_| readable(s + OFF_SIM_SCENE_ANCHOR_POS, 24));
    let Some(p) = pos else { return orig(s, a2, a3, a4, a5, a6) };
    let saved = at.read_unaligned();
    at.write_unaligned([p.x, p.y, p.z]);
    let r = orig(s, a2, a3, a4, a5, a6);
    at.write_unaligned(saved);
    r
}

/// `game_state_sim_mouse_look_allowed(G)`: seated, the mouse only looks while a mouse button is
/// held. While watching it counts as held, so the mouse orbits; every other condition (menus,
/// dialogs, the map) is still the game's.
pub(crate) unsafe extern "system" fn look_detour(g: usize, a2: usize, a3: usize, a4: usize) -> u8 {
    let Some(&orig) = ORIGINAL_LOOK.get() else { return 0 };
    let held = (g + OFF_SIM_LOOK_BUTTON) as *mut u8;
    if !ACTIVE.load(Ordering::Relaxed) || g != crate::flight::sim_state() || !readable(g + OFF_SIM_LOOK_BUTTON, 1) {
        return orig(g, a2, a3, a4);
    }
    // shortcut: a real button press landing between these lines is lost until the next press; fine for a debug view.
    let was = *held;
    *held = 1;
    let r = orig(g, a2, a3, a4);
    *held = was;
    r
}

/// `camera_ctrl_step` (render thread, `C` = camera controller, dt in xmm2).
pub(crate) unsafe extern "system" fn step_detour(c: usize, flag: usize, dt: f32, a4: usize) -> usize {
    let Some(&orig) = ORIGINAL_STEP.get() else { return 0 };
    let on = std::panic::catch_unwind(|| unsafe { take_over(c) }).unwrap_or(false);
    ACTIVE.store(on, Ordering::Relaxed);
    orig(c, flag, dt, a4)
}

/// Overwrite the third-person anchor and flags (`camera_ctrl_sync_from_frame` restored the
/// game's ones this frame, so nothing needs undoing when the watch ends).
unsafe fn take_over(c: usize) -> bool {
    let Some((now, before)) = anchor() else { return false };
    if c != crate::flight::sim_state() + OFF_SIM_CAMERA_CTRL || !readable(c, 0x2A8) {
        return false;
    }
    write_anchor(c, now, before);
    // Third person, seated (scroll zooms), free of the player's collision limit; no free cam,
    // seat look limits or cinematic orbit. The same as `sync_detour` puts in the block.
    let flags = c + 0x1A0;
    for (k, v) in [1u8, 0, 1, 0, 0, 0, 0, 0].into_iter().enumerate() {
        *((flags + k) as *mut u8) = v;
    }
    *((c + 0x188) as *mut f32) = f32::MAX;
    true
}

/// `camera_ctrl_send_status`: skipped while watching, so the orbit neither turns the player
/// nor moves the audio listener to the missile.
pub(crate) unsafe extern "system" fn status_detour(c: usize, a2: usize, a3: usize, a4: usize) -> usize {
    let Some(&orig) = ORIGINAL_STATUS.get() else { return 0 };
    if ACTIVE.load(Ordering::Relaxed) {
        return 0;
    }
    orig(c, a2, a3, a4)
}

pub(crate) fn prologues_match(base: usize) -> bool {
    bytes_match(base + RVA_CAMERA_STEP, &CAMERA_STEP_PROLOGUE)
        && bytes_match(base + RVA_CAMERA_STATUS, &CAMERA_STATUS_PROLOGUE)
        && bytes_match(base + RVA_CAMERA_SYNC, &CAMERA_SYNC_PROLOGUE)
        && bytes_match(base + RVA_MOUSE_LOOK_ALLOWED, &MOUSE_LOOK_ALLOWED_PROLOGUE)
}

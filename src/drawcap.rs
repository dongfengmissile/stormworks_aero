//! Exact per-body draw transforms for the force arrows. While the game draws the main view (inside
//! slot 7 on the render thread) every chunk item's `mat_world` comes out of `matrix_interp_prev_cur`;
//! a detour on it matches the item's `cur` matrix against the bodies the arrows ride on and keeps the
//! game's output, so the arrows use exactly what was drawn (no timing model of ours involved).

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, OnceLock};

use crate::logging::log_line;
use crate::memory::read_usize_at;
use crate::offsets::OFF_SIM_FRAME;

pub(crate) type MatrixInterp = unsafe extern "system" fn(*const f32, *const f32, f32, *mut f32);
pub(crate) static ORIGINAL: OnceLock<MatrixInterp> = OnceLock::new();

/// Bodies to look for, published by the overlay.
pub(crate) static TRACK: Mutex<Vec<usize>> = Mutex::new(Vec::new());
/// Candidates per body: the recorded builds (oldest first, up to 4), then the built and live poses,
/// all read at the draw's first item.
const CANDS: usize = 6;
/// Last main-view draw: when (ms) and the drawn world matrix per tracked body.
pub(crate) static CAPTURED: Mutex<(u64, Vec<(usize, [f64; 16])>)> = Mutex::new((0, Vec::new()));

struct Best {
    err: f64,
    world: [f64; 16],
    cur_is: usize,
    prev_is: usize,
}

#[derive(Default)]
struct Ctx {
    g: usize,
    /// `setup` ran for this draw.
    ready: bool,
    origin: [f64; 3],
    track: Vec<(usize, [[f64; 16]; CANDS])>,
    best: HashMap<usize, Best>,
}

thread_local! {
    static IN_DRAW: Cell<bool> = const { Cell::new(false) };
    static CTX: RefCell<Ctx> = RefCell::new(Ctx::default());
}

/// Before the game draws the frame.
pub(crate) fn begin(g: usize) {
    if !crate::flight::DEBUG_WANTED.load(Ordering::Relaxed) {
        return; // arrows hidden: no work in the draw
    }
    CTX.with(|c| {
        let mut c = c.borrow_mut();
        (c.g, c.ready) = (g, false);
        c.track.clear();
        c.best.clear();
    });
    IN_DRAW.with(|d| d.set(true));
}

/// At the draw's first item: the floating origin of the item matrices and the poses to look for.
/// Not in `begin`: the game swaps in the frame it draws after that, and when a new build had just
/// arrived the old frame's origin and poses matched nothing (arrows off the vehicle for a frame).
fn setup(c: &mut Ctx) {
    c.ready = true;
    let frame = unsafe { read_usize_at(c.g + OFF_SIM_FRAME) };
    if !crate::editor::ptr_ok(frame, 24) {
        return;
    }
    let f64_at = |o: usize| f64::from_bits(unsafe { read_usize_at(frame + o) } as u64);
    c.origin = [f64_at(0), f64_at(8), f64_at(16)];
    let hist = crate::flight::BUILT_HISTORY.lock().map(|h| h.clone()).unwrap_or_default();
    // The items' matrices are one of these to f32 precision.
    c.track = TRACK
        .lock()
        .map(|t| t.clone())
        .unwrap_or_default()
        .into_iter()
        .filter_map(|b| {
            let live = crate::flight::body_render_matrix(b)?;
            let mut cands = [live; CANDS];
            cands[4] = crate::flight::body_built_matrix(b).unwrap_or(live);
            for (k, (_, m)) in hist.get(&b).map_or(&[][..], |h| h.as_slice()).iter().rev().take(4).enumerate() {
                cands[3 - k] = *m;
            }
            Some((b, cands))
        })
        .collect();
}

/// After the draw: publish what was matched.
pub(crate) fn end() {
    if !IN_DRAW.with(|d| d.replace(false)) {
        return;
    }
    let found: Vec<(usize, [f64; 16])> = CTX.with(|c| {
        let c = c.borrow();
        for b in c.best.values() {
            diag(b);
        }
        c.best.iter().map(|(&body, b)| (body, b.world)).collect()
    });
    *CAPTURED.lock().unwrap_or_else(|p| p.into_inner()) = (crate::editor::now_ms(), found);
}

/// The drawn world matrix of `body` in the last main-view frame, if it was matched.
pub(crate) fn drawn(body: usize) -> Option<[f64; 16]> {
    let c = CAPTURED.lock().ok()?;
    if crate::editor::now_ms().saturating_sub(c.0) > 100 {
        return None;
    }
    c.1.iter().find(|(b, _)| *b == body).map(|(_, m)| *m)
}

/// Distance between a game item matrix (f32, origin-relative) and a world candidate: metres of
/// translation plus 10× the summed rotation-entry differences.
fn err(item: &[f32; 16], origin: &[f64; 3], m: &[f64; 16]) -> f64 {
    let mut rot = 0.0;
    for k in [0, 1, 2, 4, 5, 6, 8, 9, 10] {
        rot += (item[k] as f64 - m[k]).abs();
    }
    let t = (0..3).map(|i| (item[12 + i] as f64 - origin[i] - m[12 + i]).powi(2)).sum::<f64>().sqrt();
    t + 10.0 * rot
}

pub(crate) unsafe extern "system" fn interp_detour(prev: *const f32, cur: *const f32, alpha: f32, out: *mut f32) {
    let Some(original) = ORIGINAL.get() else { return };
    original(prev, cur, alpha, out);
    if !IN_DRAW.with(|d| d.get()) || prev.is_null() || cur.is_null() || out.is_null() {
        return;
    }
    let (p, c, o) = (prev.cast::<[f32; 16]>().read_unaligned(), cur.cast::<[f32; 16]>().read_unaligned(), out.cast::<[f32; 16]>().read_unaligned());
    CTX.with(|ctx| {
        let mut ctx = ctx.borrow_mut();
        let ctx = &mut *ctx;
        if !ctx.ready {
            setup(ctx);
        }
        for (body, cands) in &ctx.track {
            // Items should be origin-relative; a zero origin is tried too in case they are not.
            let pick = |origin: &[f64; 3]| cands.map(|m| err(&c, origin, &m)).iter().copied().enumerate().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or((0, f64::MAX));
            let (with, without) = (pick(&ctx.origin), pick(&[0.0; 3]));
            let (origin, (cur_is, e)) = if with.1 <= without.1 { (ctx.origin, with) } else { ([0.0; 3], without) };
            // Near-exact only (f32 item vs f64 pose is millimetres): a looser match can be another part's item.
            if e > 0.05 || ctx.best.get(body).is_some_and(|b| b.err <= e) {
                continue;
            }
            let prev_is = cands.map(|m| err(&p, &origin, &m)).iter().copied().enumerate().min_by(|a, b| a.1.total_cmp(&b.1)).map_or(0, |x| x.0);
            let world = std::array::from_fn(|k| o[k] as f64 - if (12..15).contains(&k) { origin[k - 12] } else { 0.0 });
            ctx.best.insert(*body, Best { err: e, world, cur_is, prev_is });
        }
    });
}

/// Every ~2000 matches: which candidate the game's `cur` and `prev` were (tells our timing model's error).
fn diag(b: &Best) {
    static N: Mutex<([u32; CANDS], [u32; CANDS], f64, u32)> = Mutex::new(([0; CANDS], [0; CANDS], 0.0, 0));
    static LINES: AtomicU32 = AtomicU32::new(0);
    let mut n = N.lock().unwrap_or_else(|p| p.into_inner());
    n.0[b.cur_is.min(CANDS - 1)] += 1;
    n.1[b.prev_is.min(CANDS - 1)] += 1;
    n.2 += b.err;
    n.3 += 1;
    if n.3 >= 2000 {
        if LINES.fetch_add(1, Ordering::Relaxed) < 200 {
            log_line(&format!(
                "draw capture: {} matches; game cur = [rec n-3, n-2, n-1, n, built now, live now] {:?}, prev = {:?}, mean match error {:.4}",
                n.3, n.0, n.1, n.2 / n.3 as f64
            ));
        }
        *n = ([0; CANDS], [0; CANDS], 0.0, 0);
    }
}

//! Aerodynamic effects, visual only: nothing here changes a force.
//!
//! Wingtip vortex condensation. Each free wing tip sheds a vortex of the circulation its half of
//! the wing carries, Γ = 4L / (πρVs) (elliptic loading, Γ at the root). Its core is a Rankine
//! vortex: the air expands adiabatically into the low pressure at the centre and cools, and where
//! it cools past the dew point the water condenses. So the trail appears at high lift (pulling g,
//! slow with flaps) and in humid air, as on real aircraft, and its width is the radius inside which
//! the core is saturated. Behind the wing the core spreads (Squire's eddy viscosity ν = aΓ), the
//! pressure drop eases and the trail thins out and ends on its own.

use std::collections::VecDeque;
use std::sync::atomic::AtomicBool;

use crate::math::V3;
use crate::model::{LiveBody, LiveForce, Model};
use crate::wing::Panel;

/// Display switch (F7 Settings, Display): the effects are drawn.
pub static ENABLED: AtomicBool = AtomicBool::new(true);

/// Display switch: condensation as in humid air whatever the weather.
pub static ANY_WEATHER: AtomicBool = AtomicBool::new(false);

/// The effects take the air as at most this humid: wetter, the condensation looks overdone.
pub const MAX_RH: f64 = 0.7;

/// Relative humidity of the air from the game's fog and rain (0..`MAX_RH`), as its humidity sensor
/// reads them: dry air (0) never condenses, fog or rain is the most humid. With `ANY_WEATHER`,
/// always the most humid.
pub fn humidity(fog: f64, rain: f64) -> f64 {
    let wet = if ANY_WEATHER.load(std::sync::atomic::Ordering::Relaxed) { 1.0 } else { fog.max(rain).clamp(0.0, 1.0) };
    // At 0.7 a fighter's tips trail in moderate pulls; at 0.4 only in hard pulls above ~Mach 0.5.
    (0.85 * wet).min(MAX_RH)
}

const R_AIR: f64 = 287.05;
/// Squire's eddy viscosity over circulation for a trailing vortex core.
const SQUIRE: f64 = 2e-4;
/// A trail point lives at most this long, s.
const MAX_AGE: f64 = 6.0;
/// Points kept per trail (one per physics tick).
const MAX_POINTS: usize = 1500;
/// The drawn thread over the saturated region's width: photos show threads far thinner than the
/// saturated region, whose faint outer part reads as a fat tube on screen.
const THREAD: f64 = 0.25;
/// Drier air mixed into the vortex evaporates its condensate: optical depth falls by e per this, s.
// shortcut: tuned to look (trails gone within a couple of seconds), not from a mixing model.
const MIX_TIME: f64 = 0.45;
/// Peak swirl of a tip vortex over the airspeed: about 0.5 at moderate and 0.7 at high angle of
/// attack (NACA 0015, Giuni, Benard & Green 2010). A tighter Rankine core would spin faster than
/// the plane flies and drop its centre's pressure below vacuum in hard pulls, so the core widens.
const MAX_SWIRL: f64 = 0.7;
/// The wake's own rate Γ/2πb₀² (1/s) where the trails' turbulence grows as tuned: an F-4 pulling
/// 3 g at 200 m/s. A slower plane at the same g sheds a stronger vortex (Γ ∝ L/V) whose wake
/// writhes and breaks up sooner, so its trail meanders more, and over less distance.
const RATE_REF: f64 = 0.6;
/// Hard pulls: the drawn thread's radius (m) and its optical depth through the middle stop growing
/// here, so they stay a rope instead of a solid band.
const MAX_THREAD: f64 = 0.15;
const MAX_DEPTH: f64 = 1.6;

/// Smooth noise in −1..1 along `x` (one cell per unit), a different curve for each `seed`.
fn noise1(x: f64, seed: u64) -> f64 {
    let h = |n: i64| {
        let mut z = (n as u64).wrapping_add(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 52) as f64 - 1.0
    };
    let i = x.floor();
    let t = x - i;
    let (a, b) = (h(i as i64), h(i as i64 + 1));
    a + (b - a) * t * t * (3.0 - 2.0 * t)
}

/// Saturation vapour pressure over water, Pa (Magnus, Alduchov & Eskridge 1996).
pub fn saturation_pressure(t: f64) -> f64 {
    let c = t - 273.15;
    610.94 * (17.625 * c / (c + 243.04)).exp()
}

/// How far below `p` the pressure must drop, by adiabatic expansion, for air at `p` (Pa), `t` (K)
/// and relative humidity `rh` to saturate, Pa. Infinite when even a 40 % drop does not do it.
pub fn saturation_drop(p: f64, t: f64, rh: f64) -> f64 {
    // Vapour pressure falls with the air's pressure; the saturation pressure with its temperature.
    let rh_at = |ratio: f64| rh * ratio * saturation_pressure(t) / saturation_pressure(t * ratio.powf(1.0 - 1.0 / 1.4));
    if !(rh > 0.0) || !(p > 0.0) || !(t > 150.0) {
        return f64::INFINITY;
    }
    if rh_at(1.0) >= 1.0 {
        return 0.0;
    }
    let (mut lo, mut hi) = (0.6, 1.0);
    if rh_at(lo) < 1.0 {
        return f64::INFINITY;
    }
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if rh_at(mid) >= 1.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    p * (1.0 - lo)
}

/// Radius of the droplets a fast expansion condenses into, m (many small ones: a bright, thin mist).
const DROPLET_R: f64 = 3e-6;

/// Light extinction per pascal of expansion past saturation at `p` (Pa), `t` (K), 1/(m·Pa): the
/// water that condenses (the latent heat it releases warms the air and limits it), as droplets of
/// `DROPLET_R` (extinction cross-section twice their area). So the visible part of the saturated
/// region is where the pressure drops well past saturation: the core, not the faint edge.
pub fn extinction_per_pa(p: f64, t: f64) -> f64 {
    let c = t - 273.15;
    let qs = 0.622 * saturation_pressure(t) / p;
    let dqs_dt = qs * 17.625 * 243.04 / ((c + 243.04) * (c + 243.04));
    let dq_dt = dqs_dt / (1.0 + 2.501e6 / 1005.0 * dqs_dt);
    let dt_dp = (1.0 - 1.0 / 1.4) * t / p;
    let rho = p / (R_AIR * t);
    3.0 * rho * dq_dt * dt_dp / (1000.0 * DROPLET_R)
}

/// Optical depth across a saturated vortex along a line of sight `x` (0..1) of the condensed
/// radius R from its axis, over the extinction at the edge's scale (see `Shape`), for a core radius
/// `c` = r_c/R: the excess pressure drop integrated along the chord. Mirrors the shader.
pub fn chord_depth(x: f64, c: f64) -> f64 {
    let x = x.abs().min(1.0);
    let l = (1.0 - x * x).sqrt();
    if c >= 1.0 {
        // Saturated inside the core only: excess ∝ R² − r².
        return 4.0 / 3.0 * l * l * l;
    }
    let lc = (c * c - x * x).max(0.0).sqrt().min(l);
    // Outside the core the excess over the edge's is 1/r² − 1, inside 2/c² − r²/c⁴ − 1 (R = 1).
    let outside = if x > 1e-6 { (x * (l - lc) / (x * x + l * lc)).atan() / x } else { 1.0 / lc - 1.0 / l };
    let outside = 2.0 * (outside - (l - lc));
    let inside = 2.0 * lc * (2.0 / (c * c) - x * x / (c * c * c * c) - 1.0) - 2.0 / 3.0 * lc * lc * lc / (c * c * c * c);
    outside + inside
}

/// Radius inside which a Rankine vortex of circulation `gamma` (m²/s) and core radius `r_core` (m)
/// is saturated, m: where its pressure drop reaches `dp_sat`. 0 when the centre does not.
pub fn condensed_radius(gamma: f64, r_core: f64, rho: f64, dp_sat: f64) -> f64 {
    let vc = gamma.abs() / (2.0 * std::f64::consts::PI * r_core);
    let centre = rho * vc * vc;
    if !(dp_sat < centre) || !(r_core > 0.0) {
        return 0.0;
    }
    if dp_sat <= 0.5 * centre {
        // Outside the core: Δp = ρΓ²/(8π²r²).
        gamma.abs() / (2.0 * std::f64::consts::PI) * (rho / (2.0 * dp_sat)).sqrt()
    } else {
        // Inside: Δp = ρvc²(1 − r²/2rc²).
        r_core * (2.0 * (1.0 - dp_sat / centre)).sqrt()
    }
}

/// Core radius `age` seconds after shedding, m.
pub fn core_radius(r0: f64, gamma: f64, age: f64) -> f64 {
    (r0 * r0 + 4.0 * SQUIRE * gamma.abs() * age.max(0.0)).sqrt()
}

/// A free wing tip, from the model as built (vehicle aero frame).
#[derive(Clone, Debug)]
pub struct TipSpec {
    /// Body (`Comp::body`) of the panel at the tip.
    pub body: usize,
    /// The tip's trailing edge.
    pub at: V3,
    /// Span axis of the tip panel (root to tip).
    pub span: V3,
    /// Panels whose lift this tip's vortex carries (its side of its wing).
    pub panels: Vec<usize>,
    /// Root to tip along `span`, m.
    pub semispan: f64,
    /// Core radius as shed, m.
    pub core: f64,
}

/// Root and tip trailing-edge points and the tip chord of a trapezoidal panel.
fn panel_ends(p: &Panel) -> (V3, V3, f64) {
    let lambda = p.taper.max(0.01);
    let y_c = p.b * (1.0 + 2.0 * lambda) / (3.0 * (1.0 + lambda));
    let c_tip = 2.0 * p.mac * lambda / (1.0 + lambda);
    let out = p.b - y_c;
    let tip_mid = p.centroid + p.s * out - p.u * (p.tan_sweep * out);
    (p.centroid - p.s * y_c, tip_mid - p.u * (0.5 * c_tip), c_tip)
}

/// The free tips of the wings: panel ends with nothing beyond them, the ends of a wing's
/// surfaces at one tip (wing and aileron) merged into one, each wing panel feeding the nearest
/// tip of its own wing. `wing_of`: wing group per panel; `body_of`: body per panel.
pub fn free_tips(panels: &[Panel], wing_of: &[usize], body_of: &dyn Fn(usize) -> usize) -> Vec<TipSpec> {
    // (wing, tip panel, point, chord)
    let mut ends: Vec<(usize, usize, V3, f64)> = Vec::new();
    for (i, p) in panels.iter().enumerate() {
        // Only more wing continues a wing: a tip capped with plain blocks still sheds its vortex.
        if p.cells.is_empty() || p.tip_next.is_some_and(|(j, _)| j != i) {
            continue;
        }
        let (_, tip, chord) = panel_ends(p);
        let w = wing_of.get(i).copied().unwrap_or(i);
        // Ends at one tip (a wing and its aileron, on one body or two) are one vortex.
        match ends.iter_mut().find(|e| (e.2 - tip).len() < 1.5 * e.3.max(chord).max(0.5) && panels[e.1].s.dot(p.s) > 0.7) {
            // The longer chord's end stands for both (the wing, not its aileron).
            Some(e) if chord > e.3 => *e = (w, i, tip, chord),
            Some(_) => {}
            None => ends.push((w, i, tip, chord)),
        }
    }
    // An end with more wing beyond it, in line with it (a flap's outer edge next to the aileron, a
    // radiator flap under the wing), is not a tip: the wing's lift carries on past it.
    // shortcut: flap-edge vortices (the jump in lift at a deflected flap's end) are not modelled.
    let span = |i: usize| panels[i].s;
    // Chord of a whole wing (its panels may be narrow chordwise strips): the longest root chord.
    let wing_chord = |w: usize| {
        panels.iter().enumerate().filter(|(j, _)| wing_of.get(*j).copied().unwrap_or(*j) == w).map(|(_, p)| 2.0 * p.mac / (1.0 + p.taper.max(0.01))).fold(0.0, f64::max)
    };
    let inner: Vec<bool> = ends
        .iter()
        .map(|e| {
            ends.iter().any(|o| {
                let d = o.2 - e.2;
                let along = d.dot(span(e.1));
                along > 0.5 && span(o.1).dot(span(e.1)) > 0.7 && (d - span(e.1) * along).len() < (0.5 * along).max(1.5).max(1.2 * wing_chord(e.0).max(wing_chord(o.0)))
            })
        })
        .collect();
    let ends: Vec<(usize, usize, V3, f64)> = ends.into_iter().zip(inner).filter(|(_, i)| !i).map(|(e, _)| e).collect();
    let mut tips: Vec<TipSpec> = ends
        .iter()
        .map(|&(_, i, at, _)| TipSpec { body: body_of(i), at, span: panels[i].s, panels: Vec::new(), semispan: panels[i].b, core: 0.0 })
        .collect();
    for (j, p) in panels.iter().enumerate().filter(|(_, p)| !p.cells.is_empty()) {
        let w = wing_of.get(j).copied().unwrap_or(j);
        let dist = |e: &(usize, usize, V3, f64)| (e.2 - p.centroid).len();
        let nearest = |ok: &dyn Fn(&(usize, usize, V3, f64)) -> bool| ends.iter().enumerate().filter(|(_, e)| ok(e)).min_by(|a, b| dist(a.1).total_cmp(&dist(b.1))).map(|(k, _)| k);
        // Its own wing's tip; else (an inner wing on its own body, its tip on the outer panel's) the
        // nearest tip out along its span.
        let own = nearest(&|e| e.0 == w);
        let k = own.or_else(|| nearest(&|e| e.0 != w && span(e.1).dot(p.s) > 0.7 && (e.2 - p.centroid).dot(p.s) > 0.0));
        if let Some(k) = k {
            tips[k].panels.push(j);
            let (root, _, _) = panel_ends(p);
            let reach = (tips[k].at - root).dot(tips[k].span);
            tips[k].semispan = tips[k].semispan.max(reach);
        }
    }
    // Every tip's core is sized by the main wing (5 % of its semispan): a tailplane's or fin's own
    // small span would give it a tiny core spinning fast enough to condense in gentle flight (a
    // supersonic F-4's stabilator before its wings), which real tails do not show.
    // shortcut: one core size as shed; real near-field cores vary with the tip's shape.
    let main = tips.iter().map(|t| t.semispan).fold(0.0, f64::max);
    for t in &mut tips {
        t.core = (0.05 * main).clamp(0.05, 0.6);
    }
    tips
}

/// One tip's vortex this tick (world frame).
#[derive(Clone, Copy, Debug)]
pub struct TipVortex {
    pub tip: usize,
    pub at: V3,
    /// Circulation, m²/s.
    pub gamma: f64,
    pub core: f64,
    /// How the shed core moves through the world: the wind plus its descent in the downwash.
    pub drift: V3,
    /// Direction of travel through the air (the trail streams the other way) and speed, m/s.
    pub fwd: V3,
    pub speed: f64,
    /// Turning sense about `-fwd` (downstream): +1 counter-clockwise seen from behind.
    pub spin: f64,
    pub rho: f64,
    /// Pressure drop that saturates the air here, Pa.
    pub dp_sat: f64,
    /// `extinction_per_pa` here.
    pub ext: f64,
    /// How fast its wake's turbulence grows, over `RATE_REF`'s.
    pub rate: f64,
}

impl TipVortex {
    /// The trail point it sheds at trail time `t`, `dist` metres along the trail.
    pub fn shed(&self, t: f64, dist: f64) -> TrailPoint {
        TrailPoint { pos: self.at, t, gamma: self.gamma, core: self.core, drift: self.drift, spin: self.spin, rho: self.rho, dp_sat: self.dp_sat, ext: self.ext, rate: self.rate, dist, seed: self.tip as u64, cut: false }
    }
}

/// The tips' vortices from this tick's forces (`forces[1 + i]` is panel i, `Model::live_forces`).
pub fn tip_vortices(m: &Model, bodies: &[LiveBody], forces: &[LiveForce], rh: f64) -> Vec<TipVortex> {
    if forces.len() < 1 + m.panels.len() {
        return Vec::new();
    }
    let mut out = Vec::new();
    for (k, t) in m.tips.iter().enumerate() {
        let Some(lb) = bodies.get(t.body) else { continue };
        let at = lb.to_world(t.at);
        let air = lb.vel + lb.omega.cross(at - lb.com) - lb.wind;
        let speed = air.len();
        if !(speed > 5.0) || !speed.is_finite() {
            continue;
        }
        let fwd = air * (1.0 / speed);
        let lift = t.panels.iter().map(|&j| forces[1 + j].force).fold(V3::ZERO, |a, f| a + (f - fwd * f.dot(fwd)));
        let atmo = crate::model::atmosphere(at.y);
        let gamma = 4.0 * lift.len() / (std::f64::consts::PI * atmo.rho * speed * t.semispan.max(0.1));
        if !gamma.is_finite() {
            continue;
        }
        // The pair's mutual induction: w = Γ / 2πb₀, spacing b₀ = πs/2 for elliptic loading.
        let b0 = std::f64::consts::FRAC_PI_2 * t.semispan.max(0.1);
        let down = gamma / (2.0 * std::f64::consts::PI * b0);
        let rate = (down / b0 / RATE_REF).clamp(0.3, 3.0);
        let lift_dir = if lift.len() > 1e-9 { lift * (1.0 / lift.len()) } else { V3::ZERO };
        let spin = if lb.dir_to_world(t.span).cross(lift).dot(fwd * -1.0) >= 0.0 { 1.0 } else { -1.0 };
        let p = atmo.rho * R_AIR * atmo.temp;
        let (dp_sat, ext) = (saturation_drop(p, atmo.temp, rh), extinction_per_pa(p, atmo.temp));
        let core = t.core.max(gamma.abs() / (2.0 * std::f64::consts::PI * MAX_SWIRL * speed));
        out.push(TipVortex { tip: k, at, gamma, core, drift: lb.wind - lift_dir * down, fwd, speed, spin, rho: atmo.rho, dp_sat, ext, rate });
    }
    out
}

/// A point of a condensation trail: where the core was shed, and what it needs to age.
#[derive(Clone, Copy, Debug)]
pub struct TrailPoint {
    pub pos: V3,
    /// Shed at this trail time, s.
    pub t: f64,
    pub gamma: f64,
    pub core: f64,
    pub drift: V3,
    pub spin: f64,
    pub rho: f64,
    pub dp_sat: f64,
    pub ext: f64,
    pub rate: f64,
    /// Distance along the trail when shed, m: fixes the wisps' pattern to the air.
    pub dist: f64,
    /// Picks the trail's own turbulence (its tip).
    pub seed: u64,
    /// Starts a new strip (not joined to the point before).
    pub cut: bool,
}

impl TrailPoint {
    /// Where it is `now` (trail time), m: carried by the air, and pushed about by turbulence that
    /// grows as it ages (long slow meanders and short kinks, writhing over time).
    pub fn pos_at(&self, now: f64) -> V3 {
        let age = (now - self.t).max(0.0);
        let drift = self.drift * age;
        let age = age * self.rate;
        let wander = |len: f64, amp: f64, seed: u64| {
            let x = self.dist / len + 0.5 * age;
            crate::math::v3(noise1(x, seed), noise1(x, seed + 1), noise1(x, seed + 2)) * amp
        };
        let s = 16 * self.seed;
        self.pos
            + drift
            + wander(30.0, (1.2 * age).min(2.0), s)
            + wander(5.0, (0.9 * age).min(0.6), s + 3)
            + wander(1.5, (0.4 * age).min(0.15), s + 10)
    }
    /// Condensed radius `now`, m (0: clear air).
    pub fn radius_at(&self, now: f64) -> f64 {
        let age = (now - self.t).max(0.0);
        if age > MAX_AGE {
            return 0.0;
        }
        condensed_radius(self.gamma, core_radius(self.core, self.gamma, age), self.rho, self.dp_sat)
    }
    /// How it looks `now`: radius, optical depth scale, core over radius, and how far the wisps
    /// have turned about the axis, rad.
    pub fn shape_at(&self, now: f64) -> Shape {
        let age = (now - self.t).max(0.0);
        let r = self.radius_at(now);
        let rc = core_radius(self.core, self.gamma, age);
        if r <= 0.0 {
            return Shape { radius: 0.0, depth: 0.0, core: 1.0, turn: 0.0, amp: 0.0, phase: 0.0 };
        }
        let d = self.rho * self.gamma * self.gamma / (8.0 * std::f64::consts::PI * std::f64::consts::PI);
        // Scale of `chord_depth`: the excess drop's unit (the edge's drop outside the core, D/rc⁴·R² inside) times R.
        let depth = if r > rc { self.ext * self.dp_sat * r } else { self.ext * d * r.powi(3) / rc.powi(4) } * (-age / MIX_TIME).exp();
        // The air at the edge turns at Γ/2πR², but shown at most at 1.5 turns a second (faster flickers).
        let w = (self.gamma.abs() / (2.0 * std::f64::consts::PI * r.max(rc).powi(2))).min(9.0);
        let turn = self.spin * w * age;
        // The thread winds round the vortex's axis in a growing coil (a turn per 12 initial core
        // radii), turning with it: the "folding" of real trails. Uneven: wider and tighter by turns.
        let s = 16 * self.seed + 8;
        let thread = (r * THREAD).min(MAX_THREAD);
        let depth = depth.min(MAX_DEPTH / chord_depth(0.0, rc / r).max(1e-9));
        let amp = thread * (4.0 * age * self.rate).min(1.4) * (0.7 + 0.5 * noise1(self.dist / 8.0, s));
        let phase = (2.0 * std::f64::consts::PI * self.dist / (12.0 * self.core.max(0.05)) + 2.5 * noise1(self.dist / 15.0, s + 1) + turn) % (2.0 * std::f64::consts::PI * 4096.0);
        Shape { radius: thread, depth, core: rc / r, turn, amp, phase }
    }
}

/// See `TrailPoint::shape_at`.
#[derive(Clone, Copy, Debug)]
pub struct Shape {
    pub radius: f64,
    pub depth: f64,
    pub core: f64,
    pub turn: f64,
    /// How far the thread is off the axis, m, and where round it, rad.
    pub amp: f64,
    pub phase: f64,
}

/// One vehicle's condensation trails, one per tip, advanced each physics step.
#[derive(Clone, Debug, Default)]
pub struct Trails {
    /// Trail time, s: the physics steps this vehicle has made.
    pub now: f64,
    pub tips: Vec<VecDeque<TrailPoint>>,
}

impl Trails {
    /// One physics step of `dt` seconds with this tick's vortices.
    pub fn step(&mut self, dt: f64, vortices: &[TipVortex], n_tips: usize) {
        self.now += dt.max(0.0);
        self.tips.resize_with(n_tips, VecDeque::new);
        let now = self.now;
        for v in vortices {
            let Some(tr) = self.tips.get_mut(v.tip) else { continue };
            let last = tr.back().copied();
            let dist = last.map_or(0.0, |l| l.dist + (v.at - l.pos_at(now)).len().min(3.0 * v.speed * dt + 5.0));
            let p = v.shed(now, dist);
            // Also the first clear point after condensing ones: the trail tapers to nothing at the tip.
            if p.radius_at(now) > 0.0 || last.is_some_and(|l| l.radius_at(l.t) > 0.0) {
                // A jump (respawn, teleport) starts a new strip instead of a streak across the map.
                let cut = last.is_none_or(|l| (l.pos_at(now) - p.pos).len() > 3.0 * v.speed * dt + 5.0);
                tr.push_back(TrailPoint { cut, ..p });
            }
        }
        for tr in &mut self.tips {
            while tr.len() > MAX_POINTS || tr.front().is_some_and(|f| f.radius_at(now) <= 0.0 && tr.get(1).is_none_or(|n| n.cut || n.radius_at(now) <= 0.0)) {
                tr.pop_front();
            }
            if let Some(f) = tr.front_mut() {
                f.cut = true;
            }
        }
    }

    pub fn is_empty(&self) -> bool {
        self.tips.iter().all(|t| t.is_empty())
    }
}

/// Where a tip is now, for drawing the trail's live end with the body's drawn pose: the body
/// pointer and the point in its grid frame, and the world point at the physics step.
#[derive(Clone, Copy, Debug)]
pub struct Head {
    pub body: usize,
    pub local: V3,
    pub at: V3,
}

/// One vehicle's trails as of its last physics step, for the render thread.
#[derive(Clone, Debug)]
pub struct Snapshot {
    pub id: u32,
    pub now: f64,
    pub dt: f64,
    /// Per tip: its points (oldest first) and its head, when the tip is shedding.
    pub trails: Vec<(Vec<TrailPoint>, Option<Head>)>,
    /// Wing panels condensing over their suction side (`fxwing`), bodies as the game's.
    pub clouds: Vec<crate::fxwing::Cloud>,
    /// The main body's centre of mass this step (game body, grid-local, world) and its velocity:
    /// where it is drawn tells how many steps the drawn frame is behind this snapshot.
    pub pace: Option<(usize, V3, V3, V3)>,
}

impl Snapshot {
    /// Drawn `steps` steps behind this snapshot (the render thread can draw the build before
    /// it while this one is already out): back it up to that moment, so trails do not run on
    /// past the tips and the mist's streaks do not jump.
    pub fn back_up(&mut self, steps: f64) {
        self.now -= steps * self.dt;
        for c in &mut self.clouds {
            c.flow -= steps * self.dt * c.speed;
        }
    }
}

/// The vehicles' trails, replaced after each physics step.
pub static SNAPSHOT: std::sync::Mutex<Vec<Snapshot>> = std::sync::Mutex::new(Vec::new());

/// Trails are made for vehicles this close to the camera, m.
pub const RANGE: f64 = 4000.0;

/// Ribbon vertex: position relative to the camera (m); across the ribbon from its middle, m; the
/// thread's radius, optical depth scale and core over radius (`Shape`); the coil's phase; the trail
/// distance over the radius (for the wisps); the coil's radius, m.
pub type Vertex = [f32; 10];

/// Camera-facing ribbons for the trails, as triangles. `alpha`: how far the drawn frame is from
/// the previous physics step to the last (the game's interpolation). `head`: where a tip is drawn
/// this frame (the body's drawn pose), so the trail stays on the wing.
pub fn ribbon_mesh(snaps: &[Snapshot], eye: V3, alpha: f64, head: &dyn Fn(&Head) -> Option<V3>) -> Vec<Vertex> {
    let mut out = Vec::new();
    for snap in snaps {
        let now = snap.now - (1.0 - alpha.clamp(0.0, 1.0)) * snap.dt;
        for (points, h) in &snap.trails {
            // (position, shape, trail distance, joined to the one before), up to the drawn moment: the
            // physics runs up to a tick ahead of what is drawn.
            let mut line: Vec<(V3, Shape, f64, bool)> = points.iter().filter(|p| p.t <= now + 1e-6).map(|p| (p.pos_at(now), p.shape_at(now), p.dist, !p.cut)).collect();
            if let (Some(h), Some(newest)) = (h, points.last()) {
                // Then on to the tip where it is drawn.
                if let Some(at) = head(h).filter(|at| (*at - h.at).len() < 50.0) {
                    line.push((at, newest.shape_at(newest.t), newest.dist + (at - newest.pos).len(), true));
                }
            }
            for k in 1..line.len() {
                let (a, b) = (line[k - 1], line[k]);
                if !b.3 || (a.1.depth <= 0.0 && b.1.depth <= 0.0) {
                    continue;
                }
                let side = |i: usize| {
                    let t = line[(i + 1).min(line.len() - 1)].0 - line[i.saturating_sub(1)].0;
                    let s = t.cross(line[i].0 - eye);
                    if s.len() > 1e-9 { s * (1.0 / s.len()) } else { V3::ZERO }
                };
                // Wide enough for the thread wherever the coil puts it.
                let (wa, wb) = (a.1.radius + a.1.amp, b.1.radius + b.1.amp);
                let (sa, sb) = (side(k - 1) * wa, side(k) * wb);
                // The wisps' scale: the wider of the two ends, so a strip tapering to nothing keeps it.
                let scale = a.1.radius.max(b.1.radius).max(0.05);
                let v = |p: V3, across: f64, s: Shape, dist: f64| {
                    let d = p - eye;
                    let r = s.radius as f32;
                    [d.x as f32, d.y as f32, d.z as f32, across as f32, r, s.depth as f32, s.core as f32, s.phase as f32, ((dist / scale) % 4096.0) as f32, s.amp as f32]
                };
                let quad = [v(a.0 - sa, -wa, a.1, a.2), v(a.0 + sa, wa, a.1, a.2), v(b.0 + sb, wb, b.1, b.2), v(b.0 - sb, -wb, b.1, b.2)];
                out.extend([quad[0], quad[1], quad[2], quad[0], quad[2], quad[3]]);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn humid_air_saturates_with_a_few_kilopascals_of_expansion() {
        // 15 °C, 80 %: at 5 % lower pressure the air is 4.2 K cooler (10.9 °C, saturated at 13.0 hPa)
        // and its vapour 5 % thinner (13.6 → 12.95 hPa): saturated, Δp ≈ 5 kPa.
        let dp = saturation_drop(101_325.0, 288.15, 0.8);
        assert!((4_500.0..5_500.0).contains(&dp), "{dp}");
        assert!(saturation_drop(101_325.0, 288.15, 0.95) < dp);
        assert_eq!(saturation_drop(101_325.0, 288.15, 1.0), 0.0);
        assert!(saturation_drop(101_325.0, 288.15, 0.0).is_infinite());
    }

    #[test]
    fn hard_pull_condenses_and_cruise_does_not() {
        let dp = saturation_drop(101_325.0, 288.15, 0.8);
        // 10 t fighter, 5 m semispan, 150 m/s: 5 g gives Γ ≈ 340 m²/s, 1 g at 200 m/s Γ ≈ 51 m²/s.
        let gamma = |g: f64, v: f64| 4.0 * 0.5 * 10_000.0 * 9.81 * g / (std::f64::consts::PI * 1.225 * v * 5.0);
        let r5 = condensed_radius(gamma(5.0, 150.0), 0.25, 1.225, dp);
        assert!(r5 > 0.25 && r5 < 1.5, "{r5}");
        assert_eq!(condensed_radius(gamma(1.0, 200.0), 0.25, 1.225, dp), 0.0);
        // The radius is continuous across the core edge.
        let g = 200.0;
        let edge = 1.225 * (g / (2.0 * std::f64::consts::PI * 0.25)).powi(2) * 0.5;
        let (a, b) = (condensed_radius(g, 0.25, 1.225, edge * 0.999), condensed_radius(g, 0.25, 1.225, edge * 1.001));
        assert!((a - b).abs() < 1e-3 && (a - 0.25).abs() < 1e-3, "{a} {b}");
    }

    #[test]
    fn trail_thins_out_behind_the_wing_and_ends() {
        let dp = saturation_drop(101_325.0, 288.15, 0.8);
        let v = TipVortex { tip: 0, at: V3::ZERO, gamma: 340.0, core: 0.25, drift: V3::ZERO, fwd: crate::math::v3(1.0, 0.0, 0.0), speed: 150.0, spin: 1.0, rho: 1.225, dp_sat: dp, ext: extinction_per_pa(101_325.0, 288.15), rate: 1.0 };
        let mut tr = Trails::default();
        let mut w = Vec::new();
        for i in 0..600 {
            let at = crate::math::v3(2.5 * i as f64, 0.0, 0.0);
            tr.step(1.0 / 60.0, &[TipVortex { at, ..v }], 1);
            if i == 120 {
                w = tr.tips[0].iter().map(|p| p.radius_at(tr.now)).collect();
            }
        }
        // Widest at the tip, narrowing downstream, gone a few seconds back.
        assert!(w.last().unwrap() > w.first().unwrap(), "{w:?}");
        let alive = tr.tips[0].iter().filter(|p| p.radius_at(tr.now) > 0.0).count();
        assert!((30..590).contains(&alive), "{alive} points still showing");
        // Easing off: the trail stops at the tip and the rest fades.
        for i in 600..1200 {
            tr.step(1.0 / 60.0, &[TipVortex { at: crate::math::v3(2.5 * i as f64, 0.0, 0.0), gamma: 40.0, ..v }], 1);
        }
        assert!(tr.is_empty(), "{} points left", tr.tips[0].len());
    }

    #[test]
    fn optical_depth_matches_the_integral_and_is_densest_on_the_axis() {
        // Numerically: excess drop over the edge's (R = 1) along the chord at x.
        let excess = |r: f64, c: f64| if r >= 1.0 { 0.0 } else if c >= 1.0 { 1.0 - r * r } else if r > c { 1.0 / (r * r) - 1.0 } else { 2.0 / (c * c) - r * r / c.powi(4) - 1.0 };
        for c in [0.2, 0.5, 0.9, 1.0, 2.0] {
            for x in [0.0f64, 0.1, 0.3, 0.6, 0.95] {
                let n = 20_000;
                let l = (1.0 - x * x).sqrt();
                let num: f64 = (0..n).map(|i| excess((x * x + (-l + (i as f64 + 0.5) * 2.0 * l / n as f64).powi(2)).sqrt(), c) * 2.0 * l / n as f64).sum();
                let d = chord_depth(x, c);
                assert!((d - num).abs() < 1e-3 * num.max(1.0), "c {c} x {x}: {d} vs {num}");
            }
        }
        assert!(chord_depth(0.0, 0.3) > 5.0 * chord_depth(0.5, 0.3));
        // 15 °C at sea level: a kilopascal past saturation is a mist of about 0.25 per metre.
        let e = extinction_per_pa(101_325.0, 288.15) * 1000.0;
        assert!((0.1..0.5).contains(&e), "{e}");
    }

    #[test]
    fn ribbon_faces_the_camera_and_holds_the_coiled_thread() {
        let dp = saturation_drop(101_325.0, 288.15, 0.8);
        let v = TipVortex { tip: 0, at: V3::ZERO, gamma: 340.0, core: 0.25, drift: V3::ZERO, fwd: crate::math::v3(1.0, 0.0, 0.0), speed: 150.0, spin: 1.0, rho: 1.225, dp_sat: dp, ext: extinction_per_pa(101_325.0, 288.15), rate: 1.0 };
        let mut tr = Trails::default();
        for i in 0..30 {
            tr.step(1.0 / 60.0, &[TipVortex { at: crate::math::v3(2.5 * i as f64, 0.0, 0.0), ..v }], 1);
        }
        let snap = Snapshot { id: 1, now: tr.now, dt: 1.0 / 60.0, trails: vec![(tr.tips[0].iter().copied().collect(), None)], clouds: Vec::new(), pace: None };
        // Seen from above: the ribbon lies flat in x-z, across z.
        let eye = crate::math::v3(30.0, 100.0, 0.0);
        let mesh = ribbon_mesh(&[snap], eye, 1.0, &|_| None);
        assert_eq!(mesh.len(), 29 * 6);
        let last = &mesh[mesh.len() - 6..];
        let (a, b) = (last[0], last[1]);
        let width = (((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)) as f64).sqrt();
        // Wide enough for the thread anywhere on its coil.
        let s = tr.tips[0][28].shape_at(tr.now);
        let r = s.radius + s.amp;
        assert!(s.amp > 0.0 && (width - 2.0 * r).abs() < 1e-3 && ((b[1] - a[1]) as f64).abs() < 0.05 * width, "{width} vs {r}");
    }
}

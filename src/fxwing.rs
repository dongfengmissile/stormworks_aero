//! Condensation over the wings' suction side. The air speeds up over the lifting surface, so it
//! expands and cools (isentropically, from the free stream); where the pressure falls past the
//! drop that saturates it (`fx::saturation_drop`) a mist forms, densest near the leading edge's
//! suction peak, ending where the flow slows again, or sharply at the shock that ends a
//! supersonic pocket (the transonic "vapour blanket"). Mist does not clear the moment the air
//! warms again: its droplets take a while to evaporate, so it streams back over the wing and off
//! the trailing edge in the flow, thinning as it goes. Lift does not jump between neighbouring
//! panels, so neither does the mist: it blends across their junction. Per panel: a chordwise profile of the
//! surface pressure drop from thin-aerofoil theory with simple sweep and Prandtl-Glauert
//! compressibility, carried up into the air above as a harmonic field (`field`), so the blanket
//! domes over the stretch of chord that condenses. Drawn by ray-marching that field through a box
//! on the panel (`fxdraw`; the shader mirrors `field`), its top billowing in lumps on the flow.

use crate::fx::{extinction_per_pa, saturation_drop};
use crate::math::V3;
use crate::model::{LiveBody, LiveForce, Model};

/// Chord stations of a profile, at x = (i + 0.5) / STATIONS from the leading edge.
pub const STATIONS: usize = 8;
/// `field` is taken no closer to the surface than this (chords).
const MIN_YP: f64 = 0.002;
/// The layer's thickness swells and shrinks by this share in lumps (the shader's 0.6).
const LUMP: f64 = 0.6;
const CP_AIR: f64 = 1004.5;
const R_AIR: f64 = 287.05;
/// Past this local Mach the flow does not expand further (the shock and separation take over).
const MAX_LOCAL_MACH: f64 = 1.6;
/// Prandtl-Glauert's 1/β grows without bound at Mach 1; real transonic perturbations do not.
const MIN_BETA: f64 = 0.35;
/// Half a voxel: the panel's surface above its mid-plane, m.
const HALF_THICK: f64 = 0.125;
/// Supersaturation the air must reach over a wing for the mist to show. It crosses the wing in
/// milliseconds (a vortex holds its air for seconds), too short for droplets on the few aerosols
/// that activate near saturation to grow visible; strong supersaturation activates many tiny ones.
// shortcut: one threshold, set so near-level flight in very humid air stays clear below about
// Mach 0.85 while hard pulls mist (about 10 K past the dew point); a droplet-growth model would set it.
const S_VISIBLE: f64 = 2.0;
/// Those droplets are smaller than a trail's (`fx::DROPLET_R`, 3 µm), so the same water is denser.
const DENSER: f64 = 3.0;
/// Wings whose tip spans at least this share of the largest one's (not tails and fins).
const MAIN_WING: f64 = 0.7;
/// Evaporation of a mist droplet of radius r in air at relative humidity RH: r·dr/dt = G·(RH − 1)
/// (diffusional growth, G for water near 15 °C), so it lasts r²/(2G(1 − RH)): 1 µm droplets
/// (`DENSER`) about 30 ms in 85 % air, 7 ms in 30 %. At 250 m/s that is metres of flow.
const G_GROWTH: f64 = 1e-10;
const MIST_DROPLET_R: f64 = 1e-6;
/// Share of the mist a shock clears at most: its warming evaporates the small droplets at once,
/// the largest ride on and evaporate over the next metres (a veil, not bare wing).
const SHOCK_CLEAR: f64 = 0.6;
/// The mist eases in and out over these, s, so a tick's jitter in the lift does not flash it.
const RISE_S: f64 = 0.08;
const FALL_S: f64 = 0.25;
/// The flow distance wraps here, m (a jump in the streaks every few minutes; f32 stays precise).
const FLOW_WRAP: f64 = 1e5;

/// Per vehicle: each panel's eased ratio profile, eased layer thickness (its streaks' scale), m,
/// and how far its streaks have flowed past, in streak units.
#[derive(Default)]
pub struct Mist {
    ratio: Vec<[f64; STATIONS]>,
    thick: Vec<f64>,
    phase: Vec<f64>,
}

/// How fast the streaks' scale follows the layer's thickness, s: the flow phase is integrated at
/// that scale, so a change slides the pattern instead of scrambling it.
const THICK_S: f64 = 0.4;

/// The streaks' scale for a layer `thick` m: as big as the layer is thick, from 10 cm (as the shader).
fn streak_scale(thick: f64) -> f64 {
    0.1 / thick.max(0.1)
}

/// Suction-side pressure drop along the chord, as a fraction of the ambient pressure (negative:
/// compressed), the shock's chord position (1: none), how strong it is (0..1: a nose's spike, 0,
/// up to a real shock, 1), β, and the drop just behind the shock (0 past the trailing edge,
/// where the air is back to the free stream's), for a section at free-stream `mach`,
/// section lift coefficient `cl`, thickness `t_c`, mid-chord sweep `tan_sweep` and air at `t` K.
/// Neither theory holds near Mach 1 (normal to the span), so past it the subsonic one's rooftop
/// gives way to the supersonic one's profile by Mach 1.5, not at a switch.
pub fn suction_profile(mach: f64, cl: f64, t_c: f64, tan_sweep: f64, t: f64) -> ([f64; STATIONS], f64, f64, f64, f64) {
    let mn = mach / (1.0 + tan_sweep * tan_sweep).sqrt();
    let w = ((mn - 1.0) / 0.5).clamp(0.0, 1.0);
    let w = w * w * (3.0 - 2.0 * w);
    if w <= 0.0 || w >= 1.0 {
        return section(mach, cl, t_c, tan_sweep, t, w >= 1.0);
    }
    let (sub, sup) = (section(mach, cl, t_c, tan_sweep, t, false), section(mach, cl, t_c, tan_sweep, t, true));
    let mut out = sub.0;
    out.iter_mut().zip(sup.0).for_each(|(o, s)| *o += (s - *o) * w);
    (out, 1.0, 1.0, sub.3 + (sup.3 - sub.3) * w, 0.0)
}

/// `suction_profile` by subsonic theory (whatever the Mach) or supersonic (`sup`).
fn section(mach: f64, cl: f64, t_c: f64, tan_sweep: f64, t: f64, sup: bool) -> ([f64; STATIONS], f64, f64, f64, f64) {
    let cos = 1.0 / (1.0 + tan_sweep * tan_sweep).sqrt();
    // Simple sweep: only the flow normal to the span sees the section.
    let mn = mach * cos;
    let (cl_n, tc_n) = (cl.abs() / (cos * cos), t_c / cos);
    let beta = (1.0 - mn * mn).abs().sqrt().max(MIN_BETA);
    let a = (1.4 * R_AIR * t).sqrt();
    let (vn, vt) = (mach * a * cos, mach * a * (1.0 - cos * cos).max(0.0).sqrt());
    let total = t + (vn * vn + vt * vt) / (2.0 * CP_AIR);
    let t_min = total / (1.0 + 0.2 * MAX_LOCAL_MACH * MAX_LOCAL_MACH);
    let (mut out, mut shock, mut pocket, mut first, mut prev_ml2) = ([0.0; STATIONS], 1.0, false, STATIONS, 0.0);
    for (i, o) in out.iter_mut().enumerate() {
        let x = (i as f64 + 0.5) / STATIONS as f64;
        // Speed-up over the normal flow: thickness (an ellipse's, uniform) plus the lifting vortex
        // sheet's (flat plate, γ/2 = Vα√((1−x)/x)); supersonic: Ackeret, the upper side of a
        // biconvex section (compressed ahead of its crest, expanded behind it).
        // shortcut: the lift's share takes Prandtl-Glauert's 1/β though `cl` already has it, so
        // the suction grows towards Mach 1 as real transonic rooftops do (linear theory at a set
        // cl would have it shrink); a transonic small-disturbance model would do it properly.
        let du = if !sup { (tc_n + cl_n / (2.0 * std::f64::consts::PI) * ((1.0 - x) / x).sqrt()) / beta } else { cl_n / 4.0 + 2.0 * tc_n * (2.0 * x - 1.0) / beta };
        let v2 = (vn * (1.0 + du)).powi(2) + vt * vt;
        let tl = (total - v2 / (2.0 * CP_AIR)).max(t_min);
        *o = 1.0 - (tl / t).powf(3.5);
        // A swept wing's shock stands where the flow normal to the span turns subsonic.
        let ml2 = (vn * (1.0 + du)).powi(2) / (1.4 * R_AIR * tl);
        if !sup {
            if ml2 > 1.0 {
                if !pocket {
                    first = i;
                }
                pocket = true;
            } else if pocket && shock == 1.0 {
                // Where the local Mach falls through 1, between this station and the last.
                shock = (i as f64 - 0.5 + (prev_ml2 - 1.0) / (prev_ml2 - ml2)) / STATIONS as f64;
            }
        }
        prev_ml2 = ml2;
    }
    // A pocket that closes near the leading edge is thin-aerofoil theory's spike (a real nose's
    // radius rounds it into a tiny, weak one); one reaching further aft ends in a shock strong
    // enough to clear the mist. Eased between, so the mist does not flip at a threshold.
    // Supersonic, or a pocket reaching the trailing edge: the trailing edge's shock recompresses it.
    let k = ((shock - 0.15) / 0.4).clamp(0.0, 1.0);
    let strong = if sup { 1.0 } else if pocket { k * k * (3.0 - 2.0 * k) } else { 0.0 };
    // A supersonic pocket flattens into a rooftop: near-uniform suction from the leading edge to
    // the shock, not thin-aerofoil theory's leading-edge spike.
    // Behind the shock the flow normal to the span drops from the rooftop's Mach to the normal
    // shock's subsonic one, warming the air: no station there is colder than that.
    let mut after = 0.0;
    if pocket {
        let end = ((shock * STATIONS as f64).round() as usize).clamp(first + 1, STATIONS);
        let mean = out[first..end].iter().sum::<f64>() / (end - first) as f64;
        out[..end].iter_mut().for_each(|o| *o += (mean.max(0.0) - *o) * strong);
        if shock < 1.0 {
            let t1 = t * (1.0 - mean.max(0.0)).powf(1.0 / 3.5);
            let m2 = ((2.0 * CP_AIR * (total - t1) - vt * vt).max(0.0) / (1.4 * R_AIR * t1)).max(1.0);
            let t2 = t1 * (2.8 * m2 - 0.4) * (0.4 * m2 + 2.0) / (5.76 * m2);
            after = 1.0 - (t2 / t).powf(3.5);
            out[end..].iter_mut().for_each(|o| *o += (o.min(after) - *o) * strong);
        }
    }
    (out, shock, strong, beta, after)
}

/// Streamline heights the condensate is followed along, chords (Prandtl-Glauert stretched):
/// finer near the surface, where most of it is.
const LEVELS: usize = 12;
fn level(j: usize) -> f64 {
    0.004 * (j * j) as f64
}
/// Steps along the chord per streamline (a whole number per station).
const STEPS: usize = 32;
/// Condensate (pressure-drop ratio past 1) at which a wake behind the trailing edge stops showing.
const WAKE_Q: f64 = 0.05;

/// The condensate the air carries along the chord, in units of the pressure-drop ratio past 1
/// (what it would hold at once): per station its column over the surface (ratio·m) and mean
/// height (m), and how far it trails behind the trailing edge, m. Each streamline over the chord
/// sees the stations' `ratio` carried up by `field`; its condensate relaxes towards (ratio − 1)
/// over `grow(ratio)` metres of flow while supersaturated (the droplets' growth), evaporates by e
/// per `fade` metres otherwise, and loses the share `clear` at the shock (`shock` 1: at the
/// trailing edge, so none trails; behind it, it grows again where `ratio` is still past 1). Air high over the wing
/// crosses the cold zone too fast to grow much, so the visible layer hugs the surface though the
/// pressure field reaches further up.
pub fn condense(ratio: [f64; STATIONS], shock: f64, clear: f64, chord: f64, beta: f64, fade: f64, grow: &dyn Fn(f64) -> f64) -> ([f32; STATIONS], [f32; STATIONS], f64) {
    let surf = ratio.map(|r| r as f32);
    let per = STEPS / STATIONS;
    let dx = chord / STEPS as f64;
    let height = |j: usize| level(j) * chord / beta.max(0.1);
    let (mut q, mut col, mut mean) = ([0.0f64; LEVELS], [0f32; STATIONS], [0f32; STATIONS]);
    let mut past = false;
    for k in 0..STEPS {
        let x = (k as f64 + 0.5) / STEPS as f64;
        if x >= shock && !past {
            past = true;
            q.iter_mut().for_each(|v| *v *= 1.0 - clear);
        }
        for (j, v) in q.iter_mut().enumerate() {
            let r = field(&surf, x, level(j));
            let eq = (r - 1.0).max(0.0);
            let len = if eq > *v { grow(r) } else { fade };
            *v = eq + (*v - eq) * (-dx / len.max(1e-6)).exp();
        }
        // The end of the step at a station's centre: its column and mean height (trapezoids).
        if k % per == per / 2 - 1 {
            let (mut c, mut m) = (0.0, 0.0);
            for j in 1..LEVELS {
                let (y0, y1) = (height(j - 1), height(j));
                let a = 0.5 * (q[j] + q[j - 1]) * (y1 - y0);
                c += a;
                m += a * 0.5 * (y0 + y1);
            }
            col[k / per] = c as f32;
            mean[k / per] = if c > 0.0 { (m / c) as f32 } else { 0.0 };
        }
    }
    let at_te = q.iter().fold(0.0, |a: f64, &v| a.max(v)) * if shock >= 1.0 { 1.0 - clear } else { 1.0 };
    let wake = if at_te > WAKE_Q { fade * (at_te / WAKE_Q).ln() } else { 0.0 };
    (col, mean, wake)
}

/// One panel's condensation this tick.
#[derive(Clone, Debug)]
pub struct Cloud {
    /// Body index into the live bodies; `flight` swaps in the game's body for drawing.
    pub body: usize,
    /// Grid-local points of that body: the root's mid-chord on the suction surface, and one
    /// metre from it along the chord (to the leading edge), the span (to the tip) and the
    /// suction side's normal.
    pub pts: [V3; 4],
    pub b: f64,
    pub c_root: f64,
    pub taper: f64,
    pub tan_sweep: f64,
    /// Condensate at the stations (`condense`): column over the surface and its mean height, m.
    pub col: [f32; STATIONS],
    pub mean: [f32; STATIONS],
    /// Extinction per unit of condensate, 1/m.
    pub k: f64,
    /// Box height, m (the layer's top at its highest lumps, and the wake's rise); its largest mean
    /// height, m (its lumps and streaks scale with it); shock chord position (1: none).
    pub height: f64,
    pub thick: f64,
    pub shock: f64,
    /// The mist's evaporation length in the flow, m: it thins by e over this.
    pub fade: f64,
    /// How far it trails behind the trailing edge, m, and how far the flow it rides rises off the
    /// chord plane per metre aft (the angle of attack: the air leaves along the flight path).
    pub wake: f64,
    pub slope: f64,
    /// The panel ends in a free tip, where its lift (and the mist) fades out.
    pub free_tip: bool,
    /// Wing panel index.
    pub panel: usize,
    /// How far the mist runs on past the root and the tip end into the neighbouring panel there,
    /// m (0: none). Both halve it at the junction and fade their share out across the other: the
    /// two add up to one seamless sheet, so lift jumping between panels does not show.
    pub spill: [f64; 2],
    /// The streaks' span coordinate at the root, m (continuous along a wing), how far the air has
    /// flowed past this tick and its speed, in streak units (m × `streak_scale(thick)`): the
    /// streaks ride the flow.
    pub span0: f64,
    pub flow: f64,
    pub speed: f64,
}

/// The mist ratio at chord position `x` (0 leading edge, 1 trailing edge) and height `yp` = βy/c
/// above the surface, from the stations' `drop`: a pressure perturbation is harmonic (in
/// Prandtl-Glauert's stretched coordinates), so the surface values reach up into the air through
/// the half-plane's Poisson kernel and the blanket stands about as tall as the stretch of chord
/// under it is long. Each station's strip of chord adds its value times the angle it subtends.
pub fn field(drop: &[f32; STATIONS], x: f64, yp: f64) -> f64 {
    let yp = yp.max(MIN_YP);
    let mut a0 = (x / yp).atan();
    let mut sum = 0.0;
    for (i, &d) in drop.iter().enumerate() {
        let a1 = ((x - (i + 1) as f64 / STATIONS as f64) / yp).atan();
        sum += d as f64 * (a0 - a1);
        a0 = a1;
    }
    sum / std::f64::consts::PI
}

/// The panels whose suction side condenses, from this tick's forces (`forces[1 + j]` is panel j),
/// eased in and out over `dt` steps in `mist`.
pub fn wing_clouds(m: &Model, bodies: &[LiveBody], forces: &[LiveForce], rh: f64, dt: f64, mist: &mut Mist) -> Vec<Cloud> {
    let mut out = Vec::new();
    if bodies.is_empty() || forces.len() < 1 + m.panels.len() {
        return out;
    }
    let mb = &bodies[m.main_body.min(bodies.len() - 1)];
    let speed0 = (mb.vel - mb.wind).len();
    mist.ratio.resize(m.panels.len(), [0.0; STATIONS]);
    mist.thick.resize(m.panels.len(), 0.0);
    mist.phase.resize(m.panels.len(), 0.0);
    for (ph, &t) in mist.phase.iter_mut().zip(&mist.thick) {
        *ph = (*ph + speed0 * streak_scale(t) * dt) % FLOW_WRAP;
    }
    let biggest = m.tips.iter().map(|t| t.semispan).fold(0.0, f64::max);
    let main: Vec<usize> = m.tips.iter().filter(|t| t.semispan >= MAIN_WING * biggest).flat_map(|t| t.panels.iter().copied()).collect();
    let next = |j: usize| m.panels[j].tip_next.map(|t| t.0).filter(|&k| k != j && main.contains(&k));
    // A flap or aileron behind a wing panel is the aft part of that wing's chord: no blanket of
    // its own (no suction peak at its hinge); the wing's runs on over it.
    let behind = |j: usize| m.panels[j].fwd.iter().any(|&(k, w)| k != j && w > 0.1 && main.contains(&k));
    let ext = |j: usize| m.panels[j].aft.iter().filter(|&&(k, _)| k != j && main.contains(&k) && behind(k)).map(|&(k, w)| w * m.panels[k].mac).sum::<f64>();
    let ease = |s: &mut [f64; STATIONS], raw: [f64; STATIONS]| {
        for (e, r) in s.iter_mut().zip(raw) {
            *e += (r - *e) * (1.0 - (-dt / if r > *e { RISE_S } else { FALL_S }).exp());
        }
    };
    for (j, p) in m.panels.iter().enumerate() {
        if p.cells.is_empty() || !(p.b > 0.0) || !(p.area > 0.0) || !main.contains(&j) || behind(j) {
            continue;
        }
        let bi = m.panel_body(p).min(bodies.len() - 1);
        let lb = &bodies[bi];
        let at = lb.to_world(p.centroid);
        let air = lb.vel + lb.omega.cross(at - lb.com) - lb.wind;
        let speed = air.len();
        let atmo = crate::model::atmosphere(at.y);
        let pa = atmo.rho * R_AIR * atmo.temp;
        let dp_sat = saturation_drop(pa, atmo.temp, rh / S_VISIBLE);
        if !(speed > 30.0) || !speed.is_finite() || !dp_sat.is_finite() {
            ease(&mut mist.ratio[j], [0.0; STATIONS]);
            continue;
        }
        let fwd = air * (1.0 / speed);
        let f = forces[1 + j].force;
        let lift = f - fwd * f.dot(fwd);
        let cl = lift.len() / (0.5 * atmo.rho * speed * speed * p.area);
        let foil = &crate::wing::AEROFOILS[p.foil.min(crate::wing::AEROFOILS.len() - 1)];
        let (prof, shock, strong, beta, after) = suction_profile(speed / atmo.a, cl, foil.t_c, p.tan_sweep, atmo.temp);
        // A shock clears the mist as far as it leaves the air short of saturation, so the tiny
        // droplets evaporate at once. Well below Mach 1 a hard pull's shock leaves it still
        // supersaturated, and the mist rides on. Eased over ±50 % of the drop that saturates it
        // (droplets of many sizes: the bigger, the slower they go).
        let k = (1.5 - after * pa / saturation_drop(pa, atmo.temp, rh)).clamp(0.0, 1.0);
        let clear = SHOCK_CLEAR * strong * k * k * (3.0 - 2.0 * k);
        ease(&mut mist.ratio[j], prof.map(|d| d * pa / dp_sat.max(1.0)));
        let fade = speed * MIST_DROPLET_R * MIST_DROPLET_R / (2.0 * G_GROWTH * (1.0 - rh).max(0.02));
        // Growth: r·dr/dt = G·(S − 1); the saturation ratio S is S_VISIBLE at ratio 1 and grows
        // about exponentially with the drop (saturation pressure falls exponentially as it cools).
        let rh_ = rh.max(1e-3);
        let grow = |r: f64| speed * MIST_DROPLET_R * MIST_DROPLET_R / (2.0 * G_GROWTH * (rh_ * (S_VISIBLE / rh_).powf(r) - 1.0).max(1e-3));
        // The section runs on over the flaps behind it (their chord, as a share of the span).
        let (lambda0, ext) = (p.taper.max(0.01), ext(j));
        let c0 = 2.0 * p.mac / (1.0 + lambda0);
        let (c_root, lambda) = (c0 + ext, (c0 * lambda0 + ext) / (c0 + ext));
        let c_max = c_root * lambda.max(1.0);
        let (col, mean, wake) = condense(mist.ratio[j], shock, clear, p.mac + ext, beta, fade, &grow);
        let wake = wake.min(3.0 * c_max);
        // The shader's layer ends at 4 mean heights, lumps thickening it by up to LUMP.
        // shortcut: the 2D field reaches up about a chord near Mach 1; a finite wing's relief
        // keeps the blanket under half its chord (transonic photos), so it is squeezed under that.
        let mean = mean.map(|m| m.min((0.5 * c_max / (4.0 * (1.0 + LUMP))) as f32));
        let visible = |i: usize| mean[i] > 0.0 && col[i] / mean[i] > 0.005;
        let thick = (0..STATIONS).filter(|&i| visible(i)).map(|i| mean[i] as f64).fold(0.0, f64::max);
        let height = 4.0 * thick * (1.0 + LUMP);
        if !(height > 0.01) {
            continue;
        }
        let te = &mut mist.thick[j];
        *te = if *te > 0.0 { *te + (thick - *te) * (1.0 - (-dt / THICK_S).exp()) } else { thick };
        let (thick, sc) = (*te, streak_scale(*te));
        // The suction side is the one the lift points to.
        let up = if lift.dot(lb.dir_to_world(p.n)) >= 0.0 { p.n } else { p.n * -1.0 };
        let (u_w, up_w) = (lb.dir_to_world(p.u), lb.dir_to_world(up));
        let slope = if air.dot(u_w) > 1e-6 { (-air.dot(up_w) / air.dot(u_w)).clamp(0.0, 0.5) } else { 0.0 };
        let y_c = p.b * (1.0 + 2.0 * lambda0) / (3.0 * (1.0 + lambda0));
        let root = p.centroid - p.s * y_c + p.u * (p.tan_sweep * y_c - 0.5 * ext) + up * HALF_THICK;
        let local = |v: V3| lb.world_to_grid(lb.to_world(v));
        let has_root = (0..m.panels.len()).any(|k| k != j && main.contains(&k) && next(k) == Some(j));
        let spill = [if has_root { (0.4 * c_root).min(0.5 * p.b) } else { 0.0 }, if next(j).is_some() { (0.4 * c_root * lambda).min(0.5 * p.b) } else { 0.0 }];
        out.push(Cloud {
            body: bi,
            pts: [local(root), local(root + p.u), local(root + p.s), local(root + up)],
            b: p.b,
            c_root,
            taper: lambda,
            tan_sweep: p.tan_sweep,
            col,
            mean,
            k: DENSER * extinction_per_pa(pa, atmo.temp) * dp_sat,
            height: height + slope * wake,
            thick,
            shock,
            fade,
            wake,
            slope,
            free_tip: p.tip_next.is_none(),
            panel: j,
            spill,
            // Mirror wings get different streaks.
            span0: root.dot(p.s) + if p.centroid.y < 0.0 { 37.0 } else { 0.0 },
            flow: mist.phase[j],
            speed: speed0 * sc,
        });
    }
    out
}

/// Cloud vertex: position relative to the camera; the panel's root, chord, span and normal axes
/// (camera-relative root, unit axes); (semispan, root chord, taper, tan sweep); the 8 stations'
/// columns and mean heights; (k, thick, height, shock); (free tip, flow, span0, fade); (slope, wake,
/// root and tip spill).
/// The position's fourth float is 1 when the camera is in (or next to) the box.
pub type CloudVertex = [f32; 48];

/// Closer than this to a box, m, the camera may be in it or its near plane may clip the faces
/// turned to it: the faces turned away are drawn instead, marched from the camera.
const INSIDE_M: f64 = 1.0;

/// The boxes' faces turned to the camera, as triangles (the shader marches the ray on through
/// the box from there), or turned away from it when it is in the box. `anchor(body, grid-local
/// point)`: where it is drawn. `time`: per vehicle.
pub fn cloud_mesh(snaps: &[crate::fx::Snapshot], eye: V3, alpha: f64, anchor: &dyn Fn(usize, V3) -> Option<V3>) -> Vec<CloudVertex> {
    let mut out = Vec::new();
    for snap in snaps {
        let back = (1.0 - alpha.clamp(0.0, 1.0)) * snap.dt;
        for c in &snap.clouds {
            let Some([r, pu, ps, pn]) = c.pts.iter().map(|&q| anchor(c.body, q)).collect::<Option<Vec<_>>>().and_then(|v| <[V3; 4]>::try_from(v).ok()) else {
                continue;
            };
            let (u, s, n) = (pu - r, ps - r, pn - r);
            // x 0 is the leading edge, 1 the trailing edge, 2 the end of the wake behind it.
            let corner = |eta: f64, x: f64, y: f64| {
                let chord = c.c_root * (1.0 + (c.taper - 1.0) * eta);
                let aft = chord * x.min(1.0) + c.wake * (x - 1.0).max(0.0);
                r + s * (eta * c.b) + u * (-c.tan_sweep * eta * c.b + 0.5 * chord - aft) + n * y
            };
            let mid = corner(0.5, if c.wake > 0.0 { 1.0 } else { 0.5 }, 0.5 * c.height);
            // The span ends, past the spills.
            let (e0, e1) = (-c.spill[0] / c.b, 1.0 + c.spill[1] / c.b);
            let head: [f32; 44] = {
                let mut h = [0f32; 44];
                let rel = r - eye;
                let vals = [
                    rel.x, rel.y, rel.z, u.x, u.y, u.z, s.x, s.y, s.z, n.x, n.y, n.z, c.b, c.c_root, c.taper, c.tan_sweep,
                ];
                for (k, v) in vals.iter().enumerate() {
                    h[k] = *v as f32;
                }
                h[16..24].copy_from_slice(&c.col);
                h[24..32].copy_from_slice(&c.mean);
                let rest = [c.k, c.thick, c.height, c.shock, if c.free_tip { 1.0 } else { 0.0 }, c.flow - back * c.speed, c.span0, c.fade, c.slope, c.wake, c.spill[0], c.spill[1]];
                for (k, v) in rest.iter().enumerate() {
                    h[32 + k] = *v as f32;
                }
                h
            };
            let vert = |p: V3, inside: bool| {
                let d = p - eye;
                let mut v = [0f32; 48];
                v[..3].copy_from_slice(&[d.x as f32, d.y as f32, d.z as f32]);
                v[3] = if inside { 1.0 } else { 0.0 };
                v[4..].copy_from_slice(&head);
                v
            };
            // (eta, x, y) corners of each face, as a quad.
            let (h, e) = (c.height, if c.wake > 0.0 { 2.0 } else { 1.0 });
            let faces = [
                [(e0, 0.0, 0.0), (e1, 0.0, 0.0), (e1, e, 0.0), (e0, e, 0.0)],
                [(e0, 0.0, h), (e1, 0.0, h), (e1, e, h), (e0, e, h)],
                [(e0, 0.0, 0.0), (e0, e, 0.0), (e0, e, h), (e0, 0.0, h)],
                [(e1, 0.0, 0.0), (e1, e, 0.0), (e1, e, h), (e1, 0.0, h)],
                [(e0, 0.0, 0.0), (e1, 0.0, 0.0), (e1, 0.0, h), (e0, 0.0, h)],
                [(e0, e, 0.0), (e1, e, 0.0), (e1, e, h), (e0, e, h)],
            ];
            // Each face's corners, and how far the eye is out past its plane, m.
            let quads = faces.map(|f| {
                let q = f.map(|(e, x, y)| corner(e, x, y));
                let centre = (q[0] + q[1] + q[2] + q[3]) * 0.25;
                let normal = (q[1] - q[0]).cross(q[3] - q[0]);
                let outward = if normal.dot(centre - mid) >= 0.0 { normal } else { normal * -1.0 };
                (q, outward.dot(eye - centre) / outward.len().max(1e-9))
            });
            let inside = quads.iter().all(|&(_, past)| past < INSIDE_M);
            for (q, past) in quads {
                if (past > 0.0) != inside {
                    out.extend([q[0], q[1], q[2], q[0], q[2], q[3]].map(|p| vert(p, inside)));
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn suction_peaks_at_the_leading_edge_and_grows_with_lift() {
        let (low, s_low, _, _, _) = suction_profile(0.5, 0.2, 0.12, 0.0, 288.15);
        let (high, s_high, _, _, _) = suction_profile(0.5, 0.8, 0.12, 0.0, 288.15);
        assert!(low.windows(2).all(|w| w[0] >= w[1]), "{low:?}");
        assert!(high.iter().zip(&low).all(|(h, l)| h > l));
        // NACA 0012-ish at cl 0.8: a suction peak of a few tens of percent of q, not of p.
        let q_over_p = 0.7 * 0.5 * 0.5;
        assert!((0.5..4.0).contains(&(high[0] / q_over_p)), "{}", high[0] / q_over_p);
        assert_eq!((s_low, s_high), (1.0, 1.0));
    }

    #[test]
    fn a_transonic_pocket_ends_in_a_shock_that_moves_aft_with_mach() {
        let (_, s1, c1, _, a1) = suction_profile(0.78, 0.5, 0.12, 0.0, 288.15);
        let (_, s2, c2, _, _) = suction_profile(0.86, 0.5, 0.12, 0.0, 288.15);
        assert!(s1 < 1.0 && s2 > s1 && c1 > 0.0 && c2 == 1.0, "{s1} {s2} {c1} {c2}");
        // Flat ahead of the shock.
        let (roof, _, _, _, _) = suction_profile(0.86, 0.5, 0.12, 0.0, 288.15);
        assert!((roof[0] - roof[1]).abs() < 1e-12);
        // Behind it the air is warmer than ahead, and nowhere colder than just behind it.
        assert!(a1 < 0.5 * roof[0] && a1 > -0.05, "{a1}");
        let (prof, s7, _, _, a7) = suction_profile(0.78, 0.5, 0.12, 0.0, 288.15);
        assert!(prof[(s7 * 8.0).round() as usize..].iter().all(|&o| o <= a7 + 1e-12));
        // The further below Mach 1, the colder the air stays behind it (a hard pull at Mach 0.72).
        let (_, s8, _, _, a8) = suction_profile(0.85, 0.3, 0.06, 0.0, 288.15);
        let (_, s9, _, _, a9) = suction_profile(0.72, 1.0, 0.06, 0.0, 288.15);
        assert!(s8 < 1.0 && s9 < 1.0 && a9 > a8 + 0.03, "{a8} {a9}");
        // Sweep shrinks the pocket to the nose's spike, which clears nothing; faster, it is back.
        let (_, _, c3, _, _) = suction_profile(0.78, 0.5, 0.12, 0.8, 288.15);
        assert!(c3 < 0.05, "{c3}");
        let (_, s5, c5, _, _) = suction_profile(0.95, 0.5, 0.12, 0.8, 288.15);
        assert!(s5 < 1.0 && c5 > 0.5, "{s5} {c5}");
        let (_, _, c4, _, _) = suction_profile(0.6, 0.5, 0.12, 0.8, 288.15);
        assert_eq!(c4, 0.0);
        // Supersonic: the trailing edge's shock clears it all.
        let (_, s6, c6, _, _) = suction_profile(1.12, 0.5, 0.12, 0.0, 288.15);
        assert_eq!((s6, c6), (1.0, 1.0));
        // The shock moves smoothly with Mach, not a station at a time.
        let s: Vec<f64> = (0..6).map(|i| suction_profile(0.73 + 0.002 * i as f64, 0.5, 0.12, 0.0, 288.15).1).collect();
        assert!(s.windows(2).all(|w| w[1] > w[0] && w[1] - w[0] < 0.03), "{s:?}");
    }

    #[test]
    fn mist_hugs_the_surface_streams_aft_and_stops_at_a_shock() {
        let peak = [3.0, 2.0, 1.5, 1.2, 1.0, 0.8, 0.6, 0.5];
        let grow = |_: f64| 1.0;
        let (col, mean, wake) = condense(peak, 1.0, 0.0, 3.0, 1.0, 5.0, &grow);
        assert!(col.iter().all(|&c| c > 0.0) && wake > 0.0, "{col:?} {wake}");
        // The condensate sits far lower than the top of the region past the threshold.
        let d = peak.map(|r| r as f32);
        let top = (0..200).map(|k| k as f64 * 0.005).filter(|&yp| (0..=40).any(|i| field(&d, i as f64 / 40.0, yp) > 1.0)).fold(0.0, f64::max) * 3.0;
        assert!(mean.iter().all(|&m| (m as f64) < 0.4 * top), "{mean:?} {top}");
        // Slower growth: less of it, and lower still.
        let (slow, slow_mean, _) = condense(peak, 1.0, 0.0, 3.0, 1.0, 5.0, &|_| 4.0);
        assert!(slow[3] < col[3] && slow_mean[3] < mean[3]);
        // Drier air evaporates it sooner.
        let (_, _, dry_wake) = condense(peak, 1.0, 0.0, 3.0, 1.0, 1.0, &grow);
        assert!(dry_wake < wake);
        let (cut, _, none) = condense([3.0, 2.0, 1.5, 1.2, 0.6, 0.5, 0.4, 0.3], 0.5, 1.0, 3.0, 1.0, 5.0, &grow);
        assert!(cut[4..].iter().all(|&c| c == 0.0) && none == 0.0, "{cut:?}");
        let (_, _, at_te) = condense(peak, 1.0, 1.0, 3.0, 1.0, 5.0, &grow);
        assert_eq!(at_te, 0.0);
    }

    #[test]
    fn the_field_matches_the_surface_and_reaches_up_as_far_as_the_stretch_is_long() {
        let d = [2.0f32; STATIONS];
        assert!((field(&d, 0.5, 0.0) - 2.0).abs() < 0.01);
        // Half a chord up over a uniform chord: the chord subtends 90°, so half the value.
        assert!((field(&d, 0.5, 0.5) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn humid_air_condenses_over_a_hard_pulling_wing_dry_air_does_not() {
        let (p, t) = (101_325.0, 288.15);
        let (prof, _, _, _, _) = suction_profile(0.6, 0.9, 0.06, 0.8, t);
        assert!(prof[0] * p > saturation_drop(p, t, 0.85 / S_VISIBLE));
        assert!(saturation_drop(p, t, 0.0).is_infinite());
        // Cruise at 1 g in moderately humid air: clear.
        let (cruise, _, _, _, _) = suction_profile(0.6, 0.15, 0.06, 0.8, t);
        assert!(cruise[0] * p < saturation_drop(p, t, 0.4 / S_VISIBLE), "{}", cruise[0] * p);
    }
}

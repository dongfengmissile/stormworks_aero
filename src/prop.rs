//! Blade-element propellers and rotors: each blade's lift and drag from its speed, pitch and
//! the air at the hub, averaged over a turn (blades spin far faster than a physics step).
//!
//! Inflow is one mean induced velocity over the disc (uniform-inflow blade element, as real-time
//! rotor simulations use), with Prandtl tip loss on the lift. Its quasi-steady value comes from
//! Johnson's model (NASA TP-2005-213477, tables 3 and 4): momentum theory in climb, forward
//! flight and the windmill brake state, a cubic bridge through ideal autorotation, and optionally
//! the vortex ring state increment that makes descent at 0.45..1.5 v_h unstable. Live, the
//! inflow follows it with Johnson's first-order lag, τ = 0.7·2πR / v_h.
//!
//! Rotors flap: the in-plane moment a rigid disc would feel (lift asymmetry, cyclic) becomes a
//! tilt of the thrust, 90° later in the turn, and only a small share stays a hub moment.
//! Propellers are rigid and keep the whole moment (P-factor).

use std::f64::consts::PI;

use crate::math::V3;
use crate::wing::{skin_friction, AEROFOILS};

/// Rotor geometry.
#[derive(Clone, Copy, Debug)]
pub struct Prop {
    /// Tip radius and blade root radius, m.
    pub radius: f64,
    pub hub: f64,
    /// Blade chord, m (constant).
    pub chord: f64,
    pub blades: u32,
    /// Index into `AEROFOILS`.
    pub foil: usize,
    /// Propeller: twisted to a constant-pitch helix through the 0.75 R angle, rigid hub.
    /// Rotor (false): linearly twisted blades (`ROTOR_TWIST`) that flap.
    pub propeller: bool,
}

/// Operating point.
#[derive(Clone, Copy, Debug)]
pub struct PropFlow {
    /// Thrust axis (unit): positive pitch pushes the vehicle this way.
    pub axis: V3,
    /// Spin: +1 blades move along `axis × r`, −1 the other way.
    pub spin: f64,
    /// rad/s, ≥ 0.
    pub omega: f64,
    /// Blade angle at 0.75 R, rad (0 = flat, 90° = feathered edge-on).
    pub pitch: f64,
    /// Cyclic: extra blade angle `cyclic · r̂` at blade direction r̂ (rad; in the disc plane).
    pub cyclic: V3,
    /// Air velocity relative to the hub (air motion; flying forward along `axis` gives −V·axis).
    pub air: V3,
    pub rho: f64,
    /// Speed of sound, m/s.
    pub a: f64,
}

/// Forces at a given inflow.
#[derive(Clone, Copy, Debug, Default)]
pub struct PropForce {
    /// Net force (N) and moment about the hub (N·m), the inputs' frame.
    pub force: V3,
    pub moment: V3,
    /// Thrust along the axis, N, and the air's torque against the spin, N·m (> 0 absorbs power).
    pub thrust: f64,
    pub torque: f64,
    /// Share of the outer half of the blades (by area and turn) past its stall angle, 0..1; reverse
    /// flow does not count.
    pub stalled: f64,
}

/// Johnson's VRS model parameters (table 4, velocities / v_h).
const VZ_A: f64 = -1.5;
const VZ_B: f64 = -2.1;
const VX_C: f64 = 0.75;
const VZ_D: f64 = -0.2;
const VZ_N: f64 = -0.45;
const VV_N: f64 = 0.85;
const VZ_X: f64 = -1.5;
const VV_X: f64 = 1.25;
const VZ_E: f64 = -2.0;
const VX_M: f64 = 0.95;

/// Momentum theory, v·sqrt(vx² + (vz + v)²) = 1: the largest root (helicopter branch) or the
/// smallest (windmill brake state).
fn momentum(vz: f64, vx: f64, helicopter: bool) -> f64 {
    let g = |v: f64| v * (vx * vx + (vz + v) * (vz + v)).sqrt() - 1.0;
    let hi = vz.abs() + 1.5;
    let step = 0.02;
    let (mut a, mut b);
    if helicopter {
        let mut v = hi;
        while v > 0.0 && g(v) >= 0.0 {
            v -= step;
        }
        a = v.max(0.0);
        b = (v + step).min(hi);
    } else {
        let mut v = 0.0;
        while v < hi && g(v) < 0.0 {
            v += step;
        }
        a = (v - step).max(0.0);
        b = v.min(hi);
    }
    for _ in 0..40 {
        let m = 0.5 * (a + b);
        if g(m) < 0.0 {
            a = m
        } else {
            b = m
        }
    }
    0.5 * (a + b)
}

/// Solve the 3×3 system m·x = r (Cramer).
fn solve3(m: [[f64; 3]; 3], r: [f64; 3]) -> [f64; 3] {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(m);
    std::array::from_fn(|k| {
        let mut mk = m;
        (0..3).for_each(|i| mk[i][k] = r[i]);
        det(mk) / d
    })
}

/// Cubic through (z0, f0, s0) and (z1, f1, s1) (Hermite), at z.
fn hermite(z: f64, z0: f64, f0: f64, s0: f64, z1: f64, f1: f64, s1: f64) -> f64 {
    let h = z1 - z0;
    let t = (z - z0) / h;
    let (t2, t3) = (t * t, t * t * t);
    (2.0 * t3 - 3.0 * t2 + 1.0) * f0 + (t3 - 2.0 * t2 + t) * h * s0 + (-2.0 * t3 + 3.0 * t2) * f1 + (t3 - t2) * h * s1
}

/// Baseline inflow (Johnson table 3 step 2): momentum theory, bridged through ideal autorotation.
fn baseline(vz: f64, vx: f64) -> f64 {
    if vz >= 0.0 || vx >= VX_C {
        return momentum(vz, vx, true);
    }
    let r = vx / VX_C;
    let za = VZ_A + 0.2 * r * r;
    let mut zb = VZ_B + 0.2 * r * r;
    if r > 0.5 {
        zb += 0.7 * (za - zb) * (2.0 * r - 1.0).powi(3);
    }
    if vz >= za {
        return momentum(vz, vx, true);
    }
    if vz <= zb {
        return momentum(vz, vx, false);
    }
    let va = momentum(za, vx, true);
    let sa = (momentum(za + 1e-4, vx, true) - momentum(za - 1e-4, vx, true)) / 2e-4;
    let vb = momentum(zb, vx, false);
    let [b, c, d] = solve3([[za, za * za, za.powi(3)], [1.0, 2.0 * za, 3.0 * za * za], [zb, zb * zb, zb.powi(3)]], [va, sa, vb]);
    b * vz + c * vz * vz + d * vz.powi(3)
}

/// Mean induced velocity / v_h at climb `vz` and edgewise speed `vx` (both / v_h), and the depth
/// in the vortex ring state (0..1). `vrs`: include the ring's unstable increment.
pub fn induced(vz: f64, vx: f64, vrs: bool) -> (f64, f64) {
    let vx = vx.abs();
    let base = baseline(vz, vx);
    if vz >= 0.0 || vx >= VX_M {
        return (base, 0.0);
    }
    let r2 = (vx / VX_M).powi(2);
    let mid = 0.5 * (VZ_N + VZ_X);
    let half = 0.5 * (VZ_N - VZ_X);
    let (zd, zn, zx) = (VZ_D, mid + half * (1.0 - r2).powf(0.2), mid - half * (1.0 - r2).powf(1.5));
    let ze = VZ_E + (zx - VZ_X);
    if vz <= ze || vz >= zd {
        return (base, 0.0);
    }
    let depth = if vz > zn {
        (zd - vz) / (zd - zn)
    } else if vz < zx {
        (vz - ze) / (zx - ze)
    } else {
        1.0
    };
    let fade = (1.0 - r2.powi(3)).max(0.0).sqrt();
    if !vrs {
        return (base, depth * fade);
    }
    let slope = |z: f64| (baseline(z + 1e-4, vx) - baseline(z - 1e-4, vx)) / 2e-4;
    let (dn, sn) = ((VV_N - (zn + baseline(zn, vx))) * fade, -(1.0 + slope(zn)));
    let (dx, sx) = ((VV_X - (zx + baseline(zx, vx))) * fade, -(1.0 + slope(zx)));
    let delta = if vz >= zn {
        hermite(vz, zd, 0.0, 0.0, zn, dn, sn)
    } else if vz >= zx {
        hermite(vz, zn, dn, sn, zx, dx, sx)
    } else {
        // X to E: Johnson leaves the slope at E free; zero there too keeps the curve smooth.
        hermite(vz, ze, 0.0, 0.0, zx, dx, sx)
    };
    (base + delta, depth * fade)
}

/// Section lift and drag coefficients at any angle (rad, wrapped), Mach and Reynolds number:
/// thin-aerofoil lift to the stall, flat plate beyond, with the section's drag rise.
/// Zero-lift angle, lift slope (per rad) and stall angle past it (rad) at `mach`.
fn stall_angle(foil: usize, mach: f64) -> (f64, f64, f64) {
    let f = &AEROFOILS[foil.min(AEROFOILS.len() - 1)];
    let m = mach.min(0.95);
    let slope = 2.0 * PI / (1.0 - m * m).max(0.1).sqrt().max(0.3);
    (-f.camber_aoa(), slope, f.cl_max / slope)
}

pub fn section(foil: usize, alpha: f64, mach: f64, re: f64) -> (f64, f64) {
    let f = &AEROFOILS[foil.min(AEROFOILS.len() - 1)];
    let alpha = (alpha + PI).rem_euclid(2.0 * PI) - PI;
    let (a0, slope, stall) = stall_angle(foil, mach);
    let attached = slope * (alpha - a0);
    let (s, c) = alpha.sin_cos();
    let plate_cl = 2.0 * s * c;
    let cd0 = 2.0 * skin_friction(re, mach) * f.form_factor();
    // Blend over 4° past the stall angle.
    let w = (((alpha - a0).abs() - stall) / (4f64).to_radians()).clamp(0.0, 1.0);
    let cl = attached.clamp(-f.cl_max, f.cl_max) * (1.0 - w) + plate_cl * w;
    let mut cd = cd0 + 0.01 * cl * cl * (1.0 - w) + (2.0 * s * s + cd0) * w;
    // Drag divergence (Korn), then wave drag past Mach 1.
    let mdd = f.kappa - f.t_c - cl.abs() / 10.0;
    if mach > mdd {
        cd += 20.0 * (mach.min(1.0) - mdd).powi(4);
    }
    if mach > 1.0 {
        cd += f.k_wave * f.t_c * f.t_c / (mach * mach - 1.0).sqrt().max(0.3);
    }
    (cl, cd)
}

const STATIONS: usize = 10;
const AZIMUTHS: usize = 8;
const DESIGN_PITCH: f64 = 25.0 * PI / 180.0;
/// Rotor blade twist root to tip, rad (UH-1 about −10.9°); zero at 0.75 R, where collective sets the pitch.
const ROTOR_TWIST: f64 = -10.0 * PI / 180.0;
/// Share of a rotor's in-plane moment the hub keeps (hinge offset / hub stiffness); flapping
/// turns the rest into thrust tilt. Two-blade rotors teeter about the shaft and keep none.
const HUB_STIFFNESS: f64 = 0.15;
/// Largest flapping tilt, rad.
const MAX_FLAP: f64 = 0.3;

impl Prop {
    fn twist(&self, r: f64) -> f64 {
        if self.propeller {
            (0.75 * self.radius * DESIGN_PITCH.tan() / r).atan() - DESIGN_PITCH
        } else {
            ROTOR_TWIST * (r - 0.75 * self.radius) / (self.radius - self.hub).max(1e-6)
        }
    }

    /// Blade forces at mean induced velocity `v` (m/s, along −axis for positive thrust).
    pub fn forces(&self, f: &PropFlow, v: f64) -> PropForce {
        let mut out = PropForce::default();
        if self.radius <= self.hub || self.blades == 0 || f.rho <= 0.0 {
            return out;
        }
        let ax = f.axis.norm();
        let u_ip = f.air.reject(ax);
        let e1 = if u_ip.len() > 1e-6 {
            u_ip.norm()
        } else {
            let t = if ax.x.abs() < 0.9 { crate::math::v3(1.0, 0.0, 0.0) } else { crate::math::v3(0.0, 1.0, 0.0) };
            t.reject(ax).norm()
        };
        let e2 = ax.cross(e1);
        let dr = (self.radius - self.hub) / STATIONS as f64;
        let b = self.blades as f64;
        let (mut n_all, mut n_stall) = (0.0, 0.0);
        // Drees: in forward flight the wake skews back, so inflow is larger at the rear (ψ = 0,
        // downwind along e1) and smaller at the front; ky on the advancing/retreating sides.
        let (mut kx, mut ky) = (0.0, 0.0);
        let vtip = f.omega * self.radius;
        if !self.propeller && vtip > 1.0 {
            let mu = u_ip.len() / vtip;
            let lam = ((v - f.air.dot(ax)) / vtip).abs();
            let chi = mu.atan2(lam);
            if chi.sin() > 1e-3 {
                kx = (4.0 / 3.0) * (1.0 - chi.cos() - 1.8 * mu * mu) / chi.sin();
            }
            ky = -2.0 * mu * f.spin;
        }
        for i in 0..STATIONS {
            let r = self.hub + (i as f64 + 0.5) * dr;
            for k in 0..AZIMUTHS {
                let psi = 2.0 * PI * (k as f64 + 0.5) / AZIMUTHS as f64;
                let rh = e1 * psi.cos() + e2 * psi.sin();
                let q = ax.cross(rh) * f.spin; // blade motion direction
                let beta = f.pitch + self.twist(r) + f.cyclic.dot(rh);
                let vi = v * (1.0 + (r / self.radius) * (kx * psi.cos() + ky * psi.sin()));
                let w = f.air - ax * vi - q * (f.omega * r); // air relative to the element
                let (up, ut) = (-w.dot(ax), -w.dot(q)); // inflow from ahead, and along the motion
                let ww = (up * up + ut * ut).sqrt();
                if ww < 1e-6 {
                    continue;
                }
                let phi = up.atan2(ut);
                // Prandtl tip loss, while the blade's own speed dominates.
                let tip = if f.omega * r > 0.3 * ww {
                    (2.0 / PI) * (-(b * (self.radius - r)) / (2.0 * r * phi.sin().abs().max(0.02))).exp().clamp(0.0, 1.0).acos()
                } else {
                    1.0
                };
                let re = f.rho * ww * self.chord / 1.8e-5;
                let (cl, cd) = section(self.foil, beta - phi, ww / f.a, re);
                let qd = 0.5 * f.rho * ww * ww * self.chord * dr;
                if r >= 0.5 * self.radius {
                    let (a0, _, stall) = stall_angle(self.foil, ww / f.a);
                    let aoa = (beta - phi + PI).rem_euclid(2.0 * PI) - PI;
                    n_all += 1.0;
                    if ut > 0.0 && (aoa - a0).abs() > stall {
                        n_stall += 1.0;
                    }
                }
                let (l, d) = (qd * cl * tip, qd * cd);
                let fp = (l * ut - d * up) / ww;
                let fq = (-l * up - d * ut) / ww;
                let fe = ax * fp + q * fq;
                out.force += fe;
                out.moment += (rh * r).cross(fe);
                out.thrust += fp;
                out.torque -= fq * r;
            }
        }
        let n = b / AZIMUTHS as f64;
        out.stalled = if n_all > 0.0 { n_stall / n_all } else { 0.0 };
        out.force = out.force * n;
        out.moment = out.moment * n;
        out.thrust *= n;
        out.torque *= n;
        if !self.propeller {
            // Flapping: the blades rise where a rigid disc would be pushed up, a quarter turn
            // later, and the thrust tilts away from that high side.
            let m_ip = out.moment.reject(ax);
            let free = m_ip * (1.0 - if self.blades == 2 { 0.0 } else { HUB_STIFFNESS });
            let k = b * f.rho * 2.0 * PI * self.chord * (f.omega * f.omega).max(1.0) * self.radius.powi(4) / 16.0;
            let tilt = (free.len() / k).min(MAX_FLAP);
            if tilt > 0.0 {
                let high = ax.cross(ax.cross(free.norm())) * f.spin; // a quarter turn on from the would-be high side
                out.force -= high * (tilt * out.thrust);
            }
            out.moment -= free;
        }
        out
    }

    /// Quasi-steady mean induced velocity (m/s) for `thrust`, the vortex ring depth (0..1),
    /// and the inflow time constant (s).
    pub fn inflow(&self, f: &PropFlow, thrust: f64, vrs: bool) -> (f64, f64, f64) {
        let area = PI * self.radius * self.radius;
        let vh = (thrust.abs() / (2.0 * f.rho.max(1e-6) * area)).sqrt();
        if vh < 0.05 {
            return (0.0, 0.0, 0.5);
        }
        let ax = f.axis.norm();
        let sign = thrust.signum();
        let climb = -f.air.dot(ax) * sign;
        let (v, depth) = induced(climb / vh, f.air.reject(ax).len() / vh, vrs);
        (sign * v * vh, depth, (0.7 * 2.0 * PI * self.radius / vh).clamp(0.05, 3.0))
    }

    /// Steady state: forces with the inflow settled (analysis, tests).
    pub fn trim(&self, f: &PropFlow, vrs: bool) -> (PropForce, f64) {
        let mut v = 0.0;
        for _ in 0..80 {
            let pf = self.forces(f, v);
            let (vq, _, _) = self.inflow(f, pf.thrust, vrs);
            v += 0.3 * (vq - v);
        }
        (self.forces(f, v), v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;

    fn prop() -> Prop {
        Prop { radius: 1.0, hub: 0.15, chord: 0.12, blades: 3, foil: 2, propeller: true }
    }
    fn flow(omega: f64, pitch_deg: f64, air: V3) -> PropFlow {
        PropFlow { axis: v3(1.0, 0.0, 0.0), spin: 1.0, omega, pitch: pitch_deg.to_radians(), cyclic: V3::ZERO, air, rho: 1.225, a: 340.0 }
    }

    #[test]
    fn static_thrust_and_figure_of_merit() {
        let (f, _) = prop().trim(&flow(200.0, 15.0, V3::ZERO), false);
        let power = f.torque * 200.0;
        let ideal = f.thrust.powf(1.5) / (2.0 * 1.225 * PI).sqrt();
        println!("static T {:.0} N P {power:.0} W FM {:.2}", f.thrust, ideal / power);
        assert!(f.thrust > 500.0 && power > 0.0);
        assert!((0.4..0.9).contains(&(ideal / power)), "figure of merit {}", ideal / power);
        assert!(f.force.y.abs() + f.force.z.abs() < 1e-6 * f.thrust);
    }

    #[test]
    fn windmilling_and_feathering_drag() {
        let p = prop();
        let wind = flow(5.0, 10.0, v3(-60.0, 0.0, 0.0));
        let flat = p.trim(&wind, false).0.thrust;
        let feathered = p.trim(&PropFlow { pitch: 88f64.to_radians(), omega: 0.0, ..wind }, false).0.thrust;
        println!("windmill {flat:.0} N, feathered {feathered:.0} N");
        assert!(flat < 0.0 && feathered < 0.0);
        assert!(feathered.abs() < 0.3 * flat.abs());
        // Faster than the pitch speed: the prop drags even under power.
        assert!(p.trim(&flow(60.0, 10.0, v3(-100.0, 0.0, 0.0)), false).0.thrust < 0.0);
    }

    #[test]
    fn edgewise_flow_gives_a_normal_force_and_spin_mirrors() {
        let p = prop();
        // Climbing nose-up: air arrives from below the disc, moving up (+z) relative to it.
        let (f, _) = p.trim(&flow(200.0, 25.0, v3(-50.0, 0.0, 8.0)), false);
        println!("normal {:.0} N of thrust {:.0}", f.force.z, f.thrust);
        assert!(f.force.z > 0.0);
        let (g, _) = p.trim(&PropFlow { spin: -1.0, ..flow(200.0, 25.0, v3(-50.0, 0.0, 8.0)) }, false);
        assert!((g.thrust - f.thrust).abs() < 1e-6 * f.thrust.abs());
        assert!((g.moment.x + f.moment.x).abs() < 1e-6 * f.moment.x.abs());
    }

    #[test]
    fn section_is_continuous_and_symmetric() {
        let (cl, _) = section(4, 5f64.to_radians(), 0.2, 1e6);
        let (cln, _) = section(4, -5f64.to_radians(), 0.2, 1e6);
        assert!((cl + cln).abs() < 1e-9 && cl > 0.4);
        let mut prev = section(2, -PI, 0.3, 1e6);
        for i in 1..=720 {
            let s = section(2, -PI + i as f64 * PI / 360.0, 0.3, 1e6);
            assert!((s.0 - prev.0).abs() < 0.2 && (s.1 - prev.1).abs() < 0.2, "jump at {i}");
            prev = s;
        }
    }

    #[test]
    fn johnson_inflow_curve() {
        // Momentum theory in hover and the windmill brake state; continuous through the bridge.
        assert!((induced(0.0, 0.0, true).0 - 1.0).abs() < 1e-6);
        assert!((induced(-3.0, 0.0, true).0 - (1.5 - 1.25f64.sqrt())).abs() < 1e-6);
        for vrs in [false, true] {
            let mut prev = induced(-3.0, 0.0, vrs).0;
            for i in 1..=400 {
                let z = -3.0 + i as f64 * 0.01;
                let v = induced(z, 0.0, vrs).0;
                assert!((v - prev).abs() < 0.1, "jump at {z} (vrs {vrs})"); // steep but continuous next to the windmill branch
                prev = v;
            }
        }
        // The ring: total inflow 0.85 at N, 1.25 at X, rising as the descent deepens (unstable).
        let total = |z: f64| z + induced(z, 0.0, true).0;
        assert!((total(VZ_N) - VV_N).abs() < 1e-6 && (total(VZ_X) - VV_X).abs() < 1e-6);
        assert!(total(-1.2) > total(-0.8));
        // Baseline (no ring): total inflow falls as the descent deepens (stable).
        let base = |z: f64| z + induced(z, 0.0, false).0;
        assert!(base(-1.2) < base(-0.8));
        // Forward speed: no ring.
        assert_eq!(induced(-1.0, 1.0, true).1, 0.0);
    }
}

#[cfg(test)]
mod rotors {
    use super::*;
    use crate::math::v3;

    fn rotor() -> Prop {
        Prop { radius: 4.0, hub: 0.3, chord: 0.25, blades: 3, foil: 4, propeller: false }
    }
    fn flow(omega: f64, pitch_deg: f64, air: V3) -> PropFlow {
        PropFlow { axis: v3(0.0, 0.0, 1.0), spin: 1.0, omega, pitch: pitch_deg.to_radians(), cyclic: V3::ZERO, air, rho: 1.225, a: 340.0 }
    }

    /// At fixed collective, sinking faster in the ring lifts less (the descent runs away);
    /// without the ring option it lifts more (stable). Forward speed removes it.
    #[test]
    fn vortex_ring_state() {
        let r = rotor();
        let hover = r.trim(&flow(35.0, 10.0, V3::ZERO), true).0.thrust;
        let vh = (hover / (2.0 * 1.225 * PI * 16.0)).sqrt();
        let at = |sink: f64, fwd: f64, vrs: bool| r.trim(&flow(35.0, 10.0, v3(-fwd * vh, 0.0, sink * vh)), vrs).0.thrust;
        println!(
            "v_h {vh:.1} | hover {hover:.0} | ring 0.6 {:.0} 1.2 {:.0} | baseline 0.6 {:.0} 1.2 {:.0} | fwd 1.5: {:.0}",
            at(0.6, 0.0, true),
            at(1.2, 0.0, true),
            at(0.6, 0.0, false),
            at(1.2, 0.0, false),
            at(1.2, 1.5, true)
        );
        assert!(at(1.2, 0.0, true) < at(0.6, 0.0, true));
        assert!(at(1.2, 0.0, false) > at(0.6, 0.0, false));
        let (_, depth, _) = r.inflow(&flow(35.0, 10.0, v3(-1.5 * vh, 0.0, 1.0 * vh)), hover, true);
        assert_eq!(depth, 0.0);
    }

    #[test]
    fn rotor_twist_spans_root_to_tip() {
        let r = rotor();
        assert!(r.twist(0.75 * r.radius).abs() < 1e-12);
        assert!((r.twist(r.radius) - r.twist(r.hub) - ROTOR_TWIST).abs() < 1e-12);
    }

    /// Stall share: none in a normal hover, most of the outer blade past the thrust peak, and the
    /// retreating side first in fast forward flight.
    #[test]
    fn stall_share() {
        let r = Prop { blades: 2, ..rotor() };
        let at = |pitch: f64, fwd: f64| r.trim(&flow(35.0, pitch, v3(-fwd, 0.0, 0.0)), false).0.stalled;
        println!("hover 8° {:.2} 25° {:.2} | 60 m/s 8° {:.2} 14° {:.2}", at(8.0, 0.0), at(25.0, 0.0), at(8.0, 60.0), at(14.0, 60.0));
        assert!(at(8.0, 0.0) < 0.05 && at(25.0, 0.0) > 0.5);
        assert!(at(14.0, 60.0) > at(8.0, 60.0));
    }

    /// Forward flight: the skewed wake (Drees) tilts the disc towards the advancing side
    /// (transverse flow effect); `spin` mirrors it.
    #[test]
    fn transverse_flow_tilts_to_the_advancing_side() {
        let r = Prop { blades: 2, ..rotor() };
        for spin in [1.0, -1.0] {
            let (f, _) = r.trim(&PropFlow { spin, ..flow(35.0, 8.0, v3(-30.0, 0.0, 0.0)) }, false);
            // Air from +x: e1 = -x (downwind), advancing side e2·spin = z × e1 · spin = -y · spin.
            println!("spin {spin}: thrust {:.0} side {:.0}", f.thrust, f.force.y);
            assert!(-f.force.y * spin > 0.01 * f.thrust);
        }
    }

    /// Unpowered, disc tilted back into a 30 m/s wind: the air spins it up to where its torque
    /// is zero, and it lifts there.
    #[test]
    fn autorotation() {
        let r = Prop { radius: 4.0, hub: 0.3, chord: 0.2, blades: 2, foil: 4, propeller: false };
        let at = |omega: f64| {
            let (f, _) = r.trim(
                &PropFlow {
                    axis: v3(0.0, 0.0, 1.0),
                    spin: 1.0,
                    omega,
                    pitch: 2f64.to_radians(),
                    cyclic: V3::ZERO,
                    air: v3(-30.0, 0.0, 3.0),
                    rho: 1.225,
                    a: 340.0,
                },
                false,
            );
            (f.torque, f.thrust)
        };
        let mut eq = None;
        for i in 2..200 {
            let w = i as f64 * 0.5;
            if eq.is_none() && at(w).0 > 0.0 && at(w - 0.5).0 < 0.0 {
                eq = Some((w, at(w).1));
            }
        }
        let (w, lift) = eq.expect("no autorotation speed");
        println!("autorotates at {w} rad/s ({:.2} RPS), lift {lift:.0} N", w / (2.0 * PI));
        assert!(lift > 500.0);
    }

    /// Cyclic tilts the thrust toward the side asked for, a quarter turn after the pitch change.
    #[test]
    fn cyclic_tilts_the_thrust() {
        let r = rotor();
        let target = v3(1.0, 0.0, 0.0);
        let ax = v3(0.0, 0.0, 1.0);
        let cyclic = ax.cross(target) * 0.05; // spin +1
        let (f, _) = r.trim(&PropFlow { cyclic, ..flow(35.0, 10.0, V3::ZERO) }, false);
        println!("tilt force {:?} thrust {:.0}", f.force, f.thrust);
        assert!(f.force.x > 0.02 * f.thrust && f.force.y.abs() < 0.3 * f.force.x);
    }
}

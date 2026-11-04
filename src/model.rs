//! The analysed vehicle: voxels + body sections + wing panels + mass, and the
//! editor simulations built on it (FAR `InstantConditionSim`, `SweepSim`,
//! `StabilityDerivCalculator`, `StabilityDerivApproxSim`; GPLv3).

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use crate::body::{aero_pos, BodyAero, Flow};
use crate::defs::Library;
use crate::math::{v3, V3};
use crate::vehicle::Comp;
use crate::voxel::{Grid, EMPTY, INTERIOR};
use crate::wing::{build_panels, panel_forces, skin_friction, Key, Panel, Role, WingFlow, MAX_DEFLECT};

pub const G: f64 = 9.81;

/// User choices that are not in the vehicle file (sidecar `.aero`).
#[derive(Clone, Default, PartialEq)]
pub struct Designations {
    /// Grid positions of components marked as wing blocks.
    pub wings: HashSet<[i32; 3]>,
    /// Control role by component position: a vanilla control surface, or any block of a sub-grid.
    pub roles: HashMap<[i32; 3], Role>,
    /// Aerofoil (index into `wing::AEROFOILS`) of the panel holding this component.
    pub foils: HashMap<[i32; 3], usize>,
    /// Aerofoil of every other panel.
    pub default_foil: usize,
}

impl Designations {
    pub fn parse(text: &str) -> Designations {
        let mut d = Designations::default();
        for line in text.lines() {
            let f: Vec<&str> = line.split_whitespace().collect();
            let pos = |i: usize| -> Option<[i32; 3]> {
                Some([f.get(i)?.parse().ok()?, f.get(i + 1)?.parse().ok()?, f.get(i + 2)?.parse().ok()?])
            };
            match f.first() {
                Some(&"wing") => {
                    d.wings.extend(pos(1));
                }
                Some(&"role") => {
                    if let (Some(r), Some(p)) = (f.get(1).and_then(|s| Role::parse(s)), pos(2)) {
                        d.roles.insert(p, r);
                    }
                }
                Some(&"foil") => {
                    if let (Some(a), Some(p)) = (f.get(1).and_then(|s| crate::wing::aerofoil_index(s)), pos(2)) {
                        d.foils.insert(p, a);
                    }
                }
                Some(&"foil_default") => {
                    d.default_foil = f.get(1).and_then(|s| crate::wing::aerofoil_index(s)).unwrap_or(0);
                }
                _ => {}
            }
        }
        d
    }

    /// Share of the marked positions present in `comps` (None with fewer than 3 marks), and how many are missing.
    /// Marks go stale when blocks are removed or moved after designating.
    pub fn match_score(&self, comps: &[Comp]) -> Option<(f64, usize)> {
        let have: HashSet<[i32; 3]> = comps.iter().map(|c| c.pos).collect();
        let marks: Vec<&[i32; 3]> = self.wings.iter().chain(self.roles.keys()).chain(self.foils.keys()).collect();
        if marks.len() < 3 {
            return None;
        }
        let missing = marks.iter().filter(|m| !have.contains(**m)).count();
        Some((1.0 - missing as f64 / marks.len() as f64, missing))
    }

    /// Drop marks on positions with no component (deleted blocks). Returns how many went.
    pub fn retain_present(&mut self, comps: &[Comp]) -> usize {
        let have: HashSet<[i32; 3]> = comps.iter().map(|c| c.pos).collect();
        let n = self.wings.len() + self.roles.len() + self.foils.len();
        self.wings.retain(|p| have.contains(p));
        self.roles.retain(|p, _| have.contains(p));
        self.foils.retain(|p, _| have.contains(p));
        n - self.wings.len() - self.roles.len() - self.foils.len()
    }

    /// The `.aero` file in `dir` whose marks best match `comps` (at least `min` of them present):
    /// (designations, file stem, share found, marks missing). Vehicles don't carry their file name.
    pub fn best_match(dir: &std::path::Path, comps: &[Comp], min: f64) -> Option<(Designations, String, f64, usize)> {
        let mut best: Option<(Designations, String, f64, usize)> = None;
        for e in std::fs::read_dir(dir).into_iter().flatten().flatten() {
            let p = e.path();
            if p.extension().is_none_or(|x| x != "aero") {
                continue;
            }
            let Ok(t) = std::fs::read_to_string(&p) else { continue };
            let d = Designations::parse(&t);
            let Some((score, missing)) = d.match_score(comps) else { continue };
            if score >= min && best.as_ref().is_none_or(|b| score > b.2) {
                best = Some((d, p.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default(), score, missing));
            }
        }
        best
    }

    pub fn to_text(&self) -> String {
        let mut lines = vec!["# stormworks_aero designations: grid positions of components".to_string()];
        let mut w: Vec<_> = self.wings.iter().collect();
        w.sort();
        lines.extend(w.iter().map(|p| format!("wing {} {} {}", p[0], p[1], p[2])));
        let mut r: Vec<_> = self.roles.iter().filter(|(_, r)| **r != Role::None).collect();
        r.sort_by_key(|(p, _)| **p);
        lines.extend(r.iter().map(|(p, r)| format!("role {} {} {} {}", r.name(), p[0], p[1], p[2])));
        let id = |i: usize| crate::wing::AEROFOILS[i.min(crate::wing::AEROFOILS.len() - 1)].id;
        if self.default_foil != 0 {
            lines.push(format!("foil_default {}", id(self.default_foil)));
        }
        let mut f: Vec<_> = self.foils.iter().collect();
        f.sort_by_key(|(p, _)| **p);
        lines.extend(f.iter().map(|(p, a)| format!("foil {} {} {} {}", id(**a), p[0], p[1], p[2])));
        lines.join("\n") + "\n"
    }
}

/// Whole wings: union of panels with face-adjacent cells (a wing and the surfaces hinged on it).
fn wing_groups(grid: &Grid, panel_of: &[usize], n: usize) -> Vec<usize> {
    let mut parent: Vec<usize> = (0..n).collect();
    fn root(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    for (c, &a) in panel_of.iter().enumerate() {
        if a == usize::MAX {
            continue;
        }
        let at = grid.cell(c);
        for d in [[1, 0, 0], [0, 1, 0], [0, 0, 1]] {
            let Some(b) = grid.index([at[0] + d[0], at[1] + d[1], at[2] + d[2]]).map(|j| panel_of[j]) else { continue };
            if b != usize::MAX {
                let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
                parent[ra] = rb;
            }
        }
    }
    (0..n).map(|i| root(&mut parent, i)).collect()
}

/// Physics step from the last move: Bullet integrates `x += v·dt` with the post-step velocity, so
/// dt = Δx·v / |v|². Kept only when moving and plausible, then filtered; the previous value otherwise.
pub fn measure_step(dt: f64, dx: V3, v: V3) -> f64 {
    let v2 = v.dot(v);
    if v2 < 4.0 {
        return dt; // under 2 m/s the ratio is noise
    }
    let raw = dx.dot(v) / v2;
    if (0.002..0.25).contains(&raw) { dt + (raw - dt) * 0.2 } else { dt }
}

/// Ground effect factor on induced drag (McCormick): φ = (16h/b)² / (1 + (16h/b)²) for a wing of span `b`
/// at height `h` above the surface; the wing behaves as if its aspect ratio were AR/φ.
pub fn ground_effect(h: f64, span: f64) -> f64 {
    if !(h.is_finite() && span > 0.0) {
        return 1.0;
    }
    let x = (16.0 * h.max(0.0) / span).powi(2);
    x / (1.0 + x)
}


/// A vehicle body's pose and motion in the world, for `Model::live_forces`.
/// World point of a model point: `(grid metres - origin)·rot + trans` (rows of `rot` are the
/// images of the grid axes, as in the game's row-vector matrices).
#[derive(Clone, Copy, Debug)]
pub struct LiveBody {
    pub origin: V3,
    pub rot: [V3; 3],
    pub trans: V3,
    /// Centre of mass, linear and angular velocity, wind: world frame.
    pub com: V3,
    pub vel: V3,
    pub omega: V3,
    pub wind: V3,
}

/// Aero body frame (x fwd, y right, z down) -> vehicle grid metres (x right, y up, z fwd).
fn aero_to_grid(p: V3) -> V3 {
    v3(p.y, -p.z, p.x)
}

fn grid_to_aero(g: V3) -> V3 {
    v3(g.z, g.x, -g.y)
}

impl LiveBody {
    pub fn to_world(&self, p: V3) -> V3 {
        self.grid_dir_to_world(aero_to_grid(p) - self.origin) + self.trans
    }
    pub fn dir_to_world(&self, d: V3) -> V3 {
        self.grid_dir_to_world(aero_to_grid(d))
    }
    fn grid_dir_to_world(&self, g: V3) -> V3 {
        self.rot[0] * g.x + self.rot[1] * g.y + self.rot[2] * g.z
    }
    /// Grid metres (the vehicle file's voxel·0.25 frame) of a world point: inverse of `to_world` before the aero map.
    pub fn world_to_grid(&self, w: V3) -> V3 {
        self.world_dir_to_grid(w - self.trans) + self.origin
    }
    pub fn world_dir_to_grid(&self, w: V3) -> V3 {
        v3(self.rot[0].dot(w), self.rot[1].dot(w), self.rot[2].dot(w))
    }
    /// Inverse of `dir_to_world` (the rotation is orthonormal).
    pub fn dir_to_aero(&self, w: V3) -> V3 {
        grid_to_aero(v3(self.rot[0].dot(w), self.rot[1].dot(w), self.rot[2].dot(w)))
    }
    // The game's grid (x right, y up, z fwd) is left-handed and the aero frame right-handed:
    // the map between them is a mirror, so axial vectors (ω, torque) flip sign across it.
    pub fn axial_to_aero(&self, w: V3) -> V3 {
        -self.dir_to_aero(w)
    }
    pub fn axial_to_world(&self, a: V3) -> V3 {
        -self.dir_to_world(a)
    }
}

/// A force on body `body` at world point `at`, plus a torque (world frame).
#[derive(Clone, Copy, Debug)]
pub struct LiveForce {
    pub body: usize,
    pub force: V3,
    pub at: V3,
    pub torque: V3,
}

/// International Standard Atmosphere (troposphere + lower stratosphere).
#[derive(Clone, Copy, Debug)]
pub struct Atmo {
    pub rho: f64,
    pub temp: f64,
    pub a: f64,
    pub gamma: f64,
}

pub fn atmosphere(alt: f64) -> Atmo {
    // Density: Stormworks' own curve (air_density_ratio 0x2B8F90), sea-level value from the game's settings.
    let ratio = if alt > 40_000.0 { 0.0 } else { (1.0 - alt.max(-1000.0) * 2.241_379_3e-5).max(0.0).powf(5.359_502_3) };
    let rho = f64::from_bits(SEA_LEVEL_RHO.load(std::sync::atomic::Ordering::Relaxed)) * ratio;
    // Temperature (speed of sound, viscosity): the game has none, so Earth's ISA.
    let h = alt.clamp(-500.0, 20000.0);
    let t = if h < 11000.0 { 288.15 - 0.0065 * h } else { 216.65 };
    Atmo { rho, temp: t, a: (1.4 * 287.05 * t).sqrt(), gamma: 1.4 }
}

/// Sea-level air density, kg/m³ (the game's `air_density` setting, read in flight; f64 bits).
pub static SEA_LEVEL_RHO: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0x3FF3_9999_9999_999A); // 1.225

/// Sutherland viscosity at FAR's reference (wall-ish) temperature for this Mach.
fn reynolds_per_m(at: &Atmo, mach: f64) -> f64 {
    let g = at.gamma;
    let t_ref = at.temp * (1.0 + 0.032 * mach * mach + 0.58 * 0.843 * 0.5 * (g - 1.0) * mach * mach);
    let mu = 1.7894e-5 * (288.16 + 110.0) / (t_ref + 110.0) * (t_ref / 288.16).powf(1.5);
    at.rho * mach * at.a / mu
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Cond {
    /// Angle of attack and sideslip, radians.
    pub alpha: f64,
    pub beta: f64,
    /// Body rates p, q, r (roll right, pitch up, yaw right), rad/s.
    pub rates: [f64; 3],
    pub mach: f64,
    pub alt: f64,
    /// Control commands −1..1 and flap 0..1.
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub flap: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Coeffs {
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
    pub cy: f64,
    pub cn: f64,
    pub croll: f64,
    pub wing_cl: f64,
    pub wing_cd: f64,
    pub wing_cdi: f64,
    pub body_cl: f64,
    pub body_cd: f64,
}

pub struct Model {
    pub comps: Vec<Comp>,
    pub grid: Grid,
    pub body: BodyAero,
    pub panels: Vec<Panel>,
    /// Panel index per grid cell (usize::MAX = none), for the voxel view.
    pub panel_of: Vec<usize>,
    /// Whole-wing id per panel: panels whose cells touch belong to one wing (wing + its ailerons/flaps).
    pub wing_of: Vec<usize>,
    pub mass: f64,
    pub com: V3,
    /// Ixx, Iyy, Izz, Ixy, Iyz, Ixz about the CoM, body frame, kg·m².
    pub inertia: [f64; 6],
    pub s_ref: f64,
    pub mac_ref: f64,
    pub b_ref: f64,
}

impl Model {
    pub fn build(comps: Vec<Comp>, lib: &mut Library, des: &Designations) -> Model {
        let grid = Grid::build(&comps, lib);
        let (mass, com, inertia) = mass_properties(&comps, &grid, lib);

        let mut body_role: HashMap<u32, Role> = HashMap::new();
        let mut comp_role: HashMap<u32, Role> = HashMap::new();
        let index: HashMap<[i32; 3], u32> = comps.iter().enumerate().map(|(i, c)| (c.pos, i as u32)).collect();
        let mut is_cs = vec![false; comps.len()];
        let mut is_wing = vec![false; comps.len()];
        for (i, c) in comps.iter().enumerate() {
            if let Some(d) = lib.get(&c.def) {
                is_cs[i] = d.is_control_surface();
                is_wing[i] = d.is_wing();
            }
            is_wing[i] |= des.wings.contains(&c.pos);
        }
        for (pos, role) in &des.roles {
            if let Some(&i) = index.get(pos) {
                if is_cs[i as usize] {
                    comp_role.insert(i, *role);
                } else if *role != Role::None {
                    body_role.insert(comps[i as usize].body, *role);
                }
            }
        }
        let key_of = |o: u32| -> Option<(Key, Role)> {
            let c = &comps[o as usize];
            if is_cs[o as usize] {
                return Some((Key::Comp(o), comp_role.get(&o).copied().unwrap_or_default()));
            }
            if let Some(r) = body_role.get(&c.body) {
                return Some((Key::Body(c.body), *r));
            }
            is_wing[o as usize].then_some((Key::Wing(c.body), Role::None))
        };
        let mut panels = build_panels(&grid, &key_of, com);
        // Section per panel: the aerofoil set on any of its components, else the default.
        for p in &mut panels {
            p.foil = p
                .cells
                .iter()
                .map(|&c| grid.owner[c])
                .filter(|&o| (o as usize) < comps.len())
                .find_map(|o| des.foils.get(&comps[o as usize].pos).copied())
                .unwrap_or(des.default_foil);
        }
        let mut panel_of = vec![usize::MAX; grid.fill.len()];
        for (i, p) in panels.iter().enumerate() {
            for &c in &p.cells {
                panel_of[c] = i;
            }
        }

        let no_skin: Vec<bool> = panel_of.iter().map(|&p| p != usize::MAX).collect();
        let body = BodyAero::build(&grid, &no_skin);

        let s: f64 = panels.iter().map(|p| p.area).sum();
        let (s_ref, mac_ref, b_ref) = if s > 1e-6 {
            let mac = panels.iter().map(|p| p.mac * p.area).sum::<f64>() / s;
            // Tip-to-tip extent of the lifting surfaces across the flow.
            let ys = panels.iter().flat_map(|p| p.cells.iter().map(|&c| aero_pos(grid.cell(c)).y));
            let (lo, hi) = ys.fold((f64::MAX, f64::MIN), |(a, b), y| (a.min(y), b.max(y)));
            (s, mac, (hi - lo + 0.25).max(0.25))
        } else {
            (body.max_area.max(0.0625), body.length.max(0.25), 1.0)
        };
        let wing_of = wing_groups(&grid, &panel_of, panels.len());
        Model { comps, grid, body, panels, panel_of, wing_of, mass, com, inertia, s_ref, mac_ref, b_ref }
    }

    /// One flight condition (FAR `GetClCdCmSteady`), coefficients on S, MAC and span.
    pub fn eval(&self, c: &Cond, stall: &mut [f64]) -> Coeffs {
        let at = atmosphere(c.alt);
        let mach = c.mach.max(0.001);
        let speed = mach * at.a;
        let q = 0.5 * at.rho * speed * speed;
        let (sa, ca) = c.alpha.sin_cos();
        let (sb, cb) = c.beta.sin_cos();
        let xw = v3(ca * cb, sb, sa * cb);
        let yw = v3(-ca * sb, cb, -sa * sb);
        let zw = v3(-sa, 0.0, ca);
        let vel = xw * speed;
        let omega = v3(c.rates[0], c.rates[1], c.rates[2]);
        let re_m = reynolds_per_m(&at, mach);

        let len = self.body.length.max(0.25);
        let re = re_m * len;
        let flow = Flow { mach, reynolds_per_m: re_m, pseudo_knudsen: mach / (re + mach), cf: skin_friction(re, mach) };
        let (fb, mb) = self.body.forces(&flow, -vel, omega, self.com, at.rho);

        let deflect: Vec<f64> = self
            .panels
            .iter()
            .map(|p| {
                let cmd = match p.role {
                    Role::None => 0.0,
                    Role::Pitch => c.pitch,
                    Role::Roll => c.roll,
                    Role::Yaw => c.yaw,
                    Role::Flap => c.flap,
                };
                p.ctrl_sign * cmd.clamp(-1.0, 1.0) * MAX_DEFLECT
            })
            .collect();
        let wf = WingFlow { mach, rho: at.rho, re_per_m: re_m, gamma: at.gamma };
        let com = self.com;
        let pf = panel_forces(&self.panels, &wf, &|_, p| vel + omega.cross(p - com), &deflect, stall);
        let mut fw = V3::ZERO;
        let mut mw = V3::ZERO;
        let mut cdi = 0.0;
        for (p, f) in self.panels.iter().zip(&pf) {
            fw += f.force;
            mw += (f.at - com).cross(f.force);
            cdi += f.cd_induced * p.area;
        }
        let qs = q * self.s_ref;
        let f = fw + fb;
        let m = mw + mb;
        Coeffs {
            cl: -f.dot(zw) / qs,
            cd: -f.dot(xw) / qs,
            cy: f.dot(yw) / qs,
            croll: m.x / (qs * self.b_ref),
            cm: m.y / (qs * self.mac_ref),
            cn: m.z / (qs * self.b_ref),
            wing_cl: -fw.dot(zw) / qs,
            wing_cd: -fw.dot(xw) / qs,
            wing_cdi: cdi / self.s_ref,
            body_cl: -fb.dot(zw) / qs,
            body_cd: -fb.dot(xw) / qs,
        }
    }

    /// Forces for one physics tick of the vehicle flying in the world (the plugin's
    /// replacement for the game's air forces). `bodies[b]` is the live pose and motion of
    /// body `b` (the `Comp::body` index); `main` carries the fuselage forces. World frame:
    /// metres, y up. Returns forces at world points and torques, per body.
    /// `ground[i]`: ground-effect factor φ of panel i (1 = none, see `ground_effect`); empty = no ground effect.
    pub fn live_forces(&self, bodies: &[LiveBody], main: usize, stall: &mut [f64], ground: &[f64]) -> Vec<LiveForce> {
        let Some(mb) = bodies.get(main) else { return Vec::new() };
        let com_w = mb.to_world(self.com);
        let air = |b: &LiveBody, p: V3| b.vel + b.omega.cross(p - b.com) - b.wind;
        let alt = com_w.y;
        let at = atmosphere(alt);
        let v_main = air(mb, com_w);
        let speed = v_main.len();
        if speed < 1.0 || !speed.is_finite() {
            stall.iter_mut().for_each(|s| *s = 0.0);
            return Vec::new();
        }
        let mach = speed / at.a;
        let re_m = reynolds_per_m(&at, mach);
        let mut out = Vec::new();

        // Fuselage (slender body) in the main body's frame.
        let len = self.body.length.max(0.25);
        let re = re_m * len;
        let flow = Flow { mach, reynolds_per_m: re_m, pseudo_knudsen: mach / (re + mach), cf: skin_friction(re, mach) };
        let (fb, mbm) = self.body.forces(&flow, -mb.dir_to_aero(v_main), mb.axial_to_aero(mb.omega), self.com, at.rho);
        out.push(LiveForce { body: main, force: mb.dir_to_world(fb), at: com_w, torque: mb.axial_to_world(mbm) });

        // Lifting panels where their body is now: a deflected sub-grid is a tilted panel.
        let owner: Vec<usize> = self.panels.iter().map(|p| self.panel_body(p).min(bodies.len().saturating_sub(1))).collect();
        let world: Vec<Panel> = self
            .panels
            .iter()
            .zip(&owner)
            .enumerate()
            .map(|(i, (p, &b))| {
                let lb = &bodies[b];
                let mut w = p.clone();
                if let Some(&phi) = ground.get(i) {
                    w.ar = (p.ar / phi.clamp(0.25, 1.0)).min(30.0); // induced drag ×φ, lift slope up (Helmbold)
                }
                w.centroid = lb.to_world(p.centroid);
                w.n = lb.dir_to_world(p.n);
                w.u = lb.dir_to_world(p.u);
                w.s = lb.dir_to_world(p.s);
                w
            })
            .collect();
        let wf = WingFlow { mach, rho: at.rho, re_per_m: re_m, gamma: at.gamma };
        let zero = vec![0.0; world.len()];
        let pf = panel_forces(&world, &wf, &|i, p| air(&bodies[owner[i]], p), &zero, stall);
        for (f, &b) in pf.iter().zip(&owner) {
            out.push(LiveForce { body: b, force: f.force, at: f.at, torque: V3::ZERO });
        }
        out
    }

    /// Body (`Comp::body`) a panel's cells belong to.
    pub fn panel_body(&self, p: &Panel) -> usize {
        p.cells
            .iter()
            .map(|&c| self.grid.owner[c])
            .find(|&o| (o as usize) < self.comps.len())
            .map_or(0, |o| self.comps[o as usize].body as usize)
    }

    pub fn fresh_stall(&self) -> Vec<f64> {
        vec![0.0; self.panels.len()]
    }

    /// AoA sweep up then back down (stall hysteresis), FAR `AngleOfAttackSweep`.
    /// Returns (alpha deg, up pass, down pass).
    pub fn aoa_sweep(&self, base: Cond, lo: f64, hi: f64, n: usize) -> (Vec<f64>, Vec<Coeffs>, Vec<Coeffs>) {
        let n = n.max(2);
        let mut stall = self.fresh_stall();
        let alphas: Vec<f64> = (0..n).map(|i| lo + (hi - lo) * i as f64 / (n - 1) as f64).collect();
        let run = |a: f64, stall: &mut Vec<f64>| self.eval(&Cond { alpha: a.to_radians(), ..base }, stall);
        let up: Vec<Coeffs> = alphas.iter().map(|&a| run(a, &mut stall)).collect();
        let mut down: Vec<Coeffs> = alphas.iter().rev().map(|&a| run(a, &mut stall)).collect();
        down.reverse();
        (alphas, up, down)
    }

    pub fn mach_sweep(&self, base: Cond, lo: f64, hi: f64, n: usize) -> (Vec<f64>, Vec<Coeffs>) {
        let n = n.max(2);
        let mut stall = self.fresh_stall();
        let machs: Vec<f64> = (0..n).map(|i| lo + (hi - lo) * i as f64 / (n - 1) as f64).collect();
        let out = machs.iter().map(|&m| self.eval(&Cond { mach: m.max(0.001), ..base }, &mut stall)).collect();
        (machs, out)
    }

    /// Aerodynamic centre (neutral point) along the body x axis, from dCm/dCL at small AoA.
    pub fn neutral_point(&self, mach: f64, alt: f64) -> Option<V3> {
        let c = |a: f64| self.eval(&Cond { alpha: a.to_radians(), mach, alt, ..Default::default() }, &mut self.fresh_stall());
        let (a, b) = (c(1.0), c(3.0));
        let dcl = b.cl - a.cl;
        if dcl.abs() < 1e-6 {
            return None;
        }
        // Lift increment ΔL at x ahead of the CoM gives ΔM = x·ΔL (Cm nose-up): x = c·dCm/dCL, aft when stable.
        let dx = (b.cm - a.cm) / dcl * self.mac_ref;
        Some(self.com + v3(dx, 0.0, 0.0))
    }

    pub fn stability(&self, mach: f64, alt: f64, flap: f64) -> Stability {
        let at = atmosphere(alt);
        let u0 = at.a * mach;
        let q = 0.5 * at.rho * u0 * u0;
        let (s, c, b, m) = (self.s_ref, self.mac_ref, self.b_ref, self.mass.max(1e-3));
        let [ix, iy, iz, ixy, iyz, ixz] = self.inertia;
        let cl_req = m * G / (q * s);
        let base = Cond { mach, alt, flap, ..Default::default() };
        let at_alpha = |a: f64, extra: &dyn Fn(&mut Cond)| {
            let mut cond = Cond { alpha: a, ..base };
            extra(&mut cond);
            self.eval(&cond, &mut self.fresh_stall())
        };
        // Trim: secant on CL(α) = CL_req from 0° and 10° (FAR).
        let f = |a: f64| at_alpha(a, &|_| {}).cl - cl_req;
        let (mut x0, mut x1) = (0.0, 10f64.to_radians());
        let (mut f0, mut f1) = (f(x0), f(x1));
        let mut converged = false;
        for _ in 0..50 {
            if (f1 - f0).abs() < 1e-12 {
                break;
            }
            let x2 = (x1 - f1 * (x1 - x0) / (f1 - f0)).clamp(-PI / 2.0, PI / 2.0);
            if (x2 - x1).abs() <= 1e-4 + 1e-4 * x1.abs() {
                x1 = x2;
                converged = true;
                break;
            }
            x0 = x1;
            f0 = f1;
            x1 = x2;
            f1 = f(x1);
        }
        let alpha = if converged { x1 } else { 0.0 };
        let nom = at_alpha(alpha, &|_| {});
        let trim_note = if (nom.cl - cl_req).abs() / cl_req.abs().max(1e-9) > 0.1 {
            if nom.cl > cl_req {
                "<"
            } else {
                ">"
            }
        } else {
            ""
        };
        // Central differences.
        let d = |h: f64, set: &dyn Fn(&mut Cond, f64)| {
            let p = at_alpha(alpha, &|c| set(c, h));
            let n = at_alpha(alpha, &|c| set(c, -h));
            let k = 1.0 / (2.0 * h);
            Coeffs {
                cl: (p.cl - n.cl) * k,
                cd: (p.cd - n.cd) * k,
                cm: (p.cm - n.cm) * k,
                cy: (p.cy - n.cy) * k,
                cn: (p.cn - n.cn) * k,
                croll: (p.croll - n.croll) * k,
                ..Default::default()
            }
        };
        let da = d(1f64.to_radians(), &|c, h| c.alpha += h);
        let dm = d(0.02, &|c, h| c.mach += h);
        let dq = d(0.05, &|c, h| c.rates[1] += h);
        let de = d(0.1, &|c, h| c.pitch += h);
        let db = d(1f64.to_radians(), &|c, h| c.beta += h);
        let dp = d(0.05, &|c, h| c.rates[0] += h);
        let dr = d(0.05, &|c, h| c.rates[2] += h);

        let k = q * s / (m * u0);
        Stability {
            mach,
            alt,
            u0,
            q,
            cl: nom.cl,
            cd: nom.cd,
            cm: nom.cm,
            cl_req,
            alpha,
            trim_note,
            s,
            mac: c,
            b,
            mass: m,
            inertia: [ix, iy, iz, ixy, iyz, ixz],
            // Longitudinal (Etkin, body x fwd z down).
            zw: -k * (da.cl + nom.cd),
            xw: -k * (da.cd - nom.cl),
            mw: q * s * c / (iy * u0) * da.cm,
            zu: -k * (dm.cl * mach + 2.0 * nom.cl),
            xu: -k * (dm.cd * mach + 2.0 * nom.cd),
            mu: q * s * c / (u0 * iy) * dm.cm * mach,
            zq: -q * s / m * dq.cl,
            xq: -q * s / m * dq.cd,
            mq: q * s * c / iy * dq.cm,
            zde: -q * s / m * de.cl,
            xde: -q * s / m * de.cd,
            mde: q * s * c / iy * de.cm,
            // Lateral.
            yb: q * s / m * db.cy,
            lb: q * s * b / ix * db.croll,
            nb: q * s * b / iz * db.cn,
            yp: q * s / m * dp.cy,
            lp: q * s * b / ix * dp.croll,
            np: q * s * b / iz * dp.cn,
            yr: q * s / m * dr.cy,
            lr: q * s * b / ix * dr.croll,
            nr: q * s * b / iz * dr.cn,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Stability {
    pub mach: f64,
    pub alt: f64,
    pub u0: f64,
    pub q: f64,
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
    pub cl_req: f64,
    pub alpha: f64,
    pub trim_note: &'static str,
    pub s: f64,
    pub mac: f64,
    pub b: f64,
    pub mass: f64,
    pub inertia: [f64; 6],
    pub zw: f64,
    pub xw: f64,
    pub mw: f64,
    pub zu: f64,
    pub xu: f64,
    pub mu: f64,
    pub zq: f64,
    pub xq: f64,
    pub mq: f64,
    pub zde: f64,
    pub xde: f64,
    pub mde: f64,
    pub yb: f64,
    pub lb: f64,
    pub nb: f64,
    pub yp: f64,
    pub lp: f64,
    pub np: f64,
    pub yr: f64,
    pub lr: f64,
    pub nr: f64,
}

impl Stability {
    /// Linearised free response (FAR `StabilityDerivApproxSim`), RK4.
    /// Longitudinal state (w m/s, u m/s, q rad/s, θ rad); lateral (β, p, r, φ) in rad, rad/s.
    pub fn simulate(&self, lateral: bool, x0: [f64; 4], end: f64, dt: f64) -> (Vec<f64>, Vec<[f64; 4]>) {
        let s = self;
        let a: [[f64; 4]; 4] = if !lateral {
            [
                [s.zw, s.zu, s.zq + s.u0, 0.0],
                [s.xw, s.xu, s.xq, -G],
                [s.mw, s.mu, s.mq, 0.0],
                [0.0, 0.0, 1.0, 0.0],
            ]
        } else {
            let [ix, _, iz, _, _, ixz] = s.inertia;
            let d = 1.0 - ixz * ixz / (ix * iz);
            let (lb, nb, lp, np, lr, nr) = (s.lb / d, s.nb / d, s.lp / d, s.np / d, s.lr / d, s.nr / d);
            let (ki, kn) = (ixz / ix, ixz / iz);
            [
                [s.yb / s.u0, s.yp / s.u0, s.yr / s.u0 - 1.0, G * s.alpha.cos() / s.u0],
                [lb + ki * nb, lp + ki * np, lr + ki * nr, 0.0],
                [nb + kn * lb, np + kn * lp, nr + kn * lr, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ]
        };
        let f = |x: &[f64; 4]| -> [f64; 4] {
            let mut o = [0.0; 4];
            for (i, row) in a.iter().enumerate() {
                o[i] = row.iter().zip(x).map(|(a, x)| a * x).sum();
            }
            o
        };
        let steps = (end / dt).ceil().max(1.0) as usize;
        let mut t = Vec::with_capacity(steps);
        let mut xs = Vec::with_capacity(steps);
        let mut x = x0;
        let add = |x: &[f64; 4], k: &[f64; 4], h: f64| [0, 1, 2, 3].map(|i| x[i] + k[i] * h);
        for i in 0..steps {
            t.push(i as f64 * dt);
            xs.push(x);
            let k1 = f(&x);
            let k2 = f(&add(&x, &k1, dt / 2.0));
            let k3 = f(&add(&x, &k2, dt / 2.0));
            let k4 = f(&add(&x, &k3, dt));
            x = [0, 1, 2, 3].map(|j| x[j] + dt / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]));
            if x.iter().any(|v| !v.is_finite()) {
                break;
            }
        }
        (t, xs)
    }
}

/// Mass, CoM and inertia from definition masses spread over each part's voxels.
fn mass_properties(comps: &[Comp], grid: &Grid, lib: &mut Library) -> (f64, V3, [f64; 6]) {
    let mut cells: Vec<Vec<V3>> = vec![Vec::new(); comps.len()];
    for i in 0..grid.owner.len() {
        let o = grid.owner[i];
        if o != EMPTY && o != INTERIOR {
            cells[o as usize].push(aero_pos(grid.cell(i)));
        }
    }
    let mut pts: Vec<(V3, f64)> = Vec::new();
    for (i, c) in comps.iter().enumerate() {
        let m = lib.get(&c.def).map(|d| d.mass as f64).unwrap_or(1.0);
        if cells[i].is_empty() {
            pts.push((aero_pos(c.pos), m));
        } else {
            let k = m / cells[i].len() as f64;
            pts.extend(cells[i].iter().map(|p| (*p, k)));
        }
    }
    let mass: f64 = pts.iter().map(|p| p.1).sum();
    if mass <= 0.0 {
        return (0.0, V3::ZERO, [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]);
    }
    let com = pts.iter().fold(V3::ZERO, |a, (p, m)| a + *p * *m) * (1.0 / mass);
    let own = 0.25 * 0.25 / 6.0; // a cube's own inertia per unit mass about any axis
    let mut i = [0.0; 6];
    for (p, m) in &pts {
        let d = *p - com;
        i[0] += m * (d.y * d.y + d.z * d.z + own);
        i[1] += m * (d.x * d.x + d.z * d.z + own);
        i[2] += m * (d.x * d.x + d.y * d.y + own);
        i[3] -= m * d.x * d.y;
        i[4] -= m * d.y * d.z;
        i[5] -= m * d.x * d.z;
    }
    (mass, com, i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_from_motion() {
        let v = v3(100.0, -5.0, 20.0);
        let mut dt = 1.0 / 60.0;
        for _ in 0..60 {
            dt = measure_step(dt, v * 0.025, v); // the game slowed to 40 steps per game second
        }
        assert!((dt - 0.025).abs() < 1e-4);
        assert_eq!(measure_step(dt, v3(0.01, 0.0, 0.0), v3(0.5, 0.0, 0.0)), dt); // too slow to measure
    }

    #[test]
    fn touching_panels_form_one_wing() {
        let grid = Grid { min: [0, 0, 0], dims: [5, 1, 1], fill: vec![1.0; 5], owner: vec![0; 5], solids: Vec::new(), missing: Vec::new() };
        let w = wing_groups(&grid, &[0, 1, usize::MAX, 2, 2], 3);
        assert_eq!(w[0], w[1]);
        assert_ne!(w[0], w[2]);
    }

    #[test]
    fn ground_effect_fades_with_height() {
        // A 12 m span at 1.2 m: induced drag about halved; at one span up it is nearly gone.
        let low = ground_effect(1.2, 12.0);
        assert!(low > 0.6 && low < 0.8, "{low}");
        assert!(ground_effect(12.0, 12.0) > 0.99);
        assert_eq!(ground_effect(0.0, 12.0), 0.0);
    }

    #[test]
    fn stale_marks_lower_the_score() {
        let comp = |pos| Comp { def: String::new(), pos, rot: crate::vehicle::IDENTITY, body: 0 };
        let mut d = Designations::default();
        d.wings.extend([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]]);
        assert_eq!(d.match_score(&[comp([0, 0, 0]), comp([1, 0, 0]), comp([2, 0, 0])]), Some((0.75, 1)));
        assert_eq!(Designations::default().match_score(&[]), None);
        assert_eq!(d.retain_present(&[comp([0, 0, 0])]), 3);
        assert_eq!(d.wings.len(), 1);
    }

    #[test]
    fn designations_round_trip() {
        let mut d = Designations::default();
        d.wings.insert([1, -2, 3]);
        d.roles.insert([0, 0, -9], Role::Pitch);
        d.foils.insert([4, 0, 1], 3);
        d.default_foil = 1;
        let back = Designations::parse(&d.to_text());
        assert!(back == d);
        assert!((atmosphere(0.0).rho - 1.225).abs() < 1e-9);
        assert!((atmosphere(10_000.0).rho - 0.3143).abs() < 0.001, "Stormworks curve at 10 km");
        assert_eq!(atmosphere(41_000.0).rho, 0.0);
        assert!((atmosphere(0.0).a - 340.3).abs() < 0.5);
    }
}

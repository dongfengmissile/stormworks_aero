//! The analysed vehicle: voxels + body sections + wing panels + mass, and the
//! editor simulations built on it (FAR `InstantConditionSim`, `SweepSim`,
//! `StabilityDerivCalculator`, `StabilityDerivApproxSim`; GPLv3).

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::sync::Arc;

use crate::body::{aero_pos, BodyAero, Flow};
use crate::defs::Library;
use crate::math::{v3, V3};
use crate::vehicle::Comp;
use crate::voxel::{Grid, EMPTY, INTERIOR, VOXEL_M};
use crate::wing::{build_panels, panel_forces, skin_friction, Key, Panel, Role, WingFlow, MAX_DEFLECT};

pub const G: f64 = 9.81;

/// User choices, keyed by component position (stored per part: `marks`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Designations {
    /// Grid positions of components marked as wing blocks.
    pub wings: HashSet<[i32; 3]>,
    /// Control role by component position: a vanilla control surface, or any block of a sub-grid.
    pub roles: HashMap<[i32; 3], Role>,
    /// Aerofoil (index into `wing::AEROFOILS`) of the panel holding this component.
    pub foils: HashMap<[i32; 3], usize>,
    /// Aerofoil of every other panel.
    pub default_foil: usize,
    /// Aerofoil of every other missile fin panel (None: `wing::FIN_DEFAULT`).
    pub default_fin: Option<usize>,
    /// Wing blocks marked as a separate wing, by wing number (1, 2, ...): each number forms its
    /// own panels and wing (for another aerofoil on part of a wing). Unmarked blocks are wing 0.
    pub splits: HashMap<[i32; 3], u32>,
    /// Per-vehicle settings (None: the global one): plugin flight model, vortex ring state.
    pub fly: Option<bool>,
    pub vrs: Option<bool>,
    /// Plugin propellers and rotors (off: the game's own thrust and shaft load).
    pub props: Option<bool>,
    pub ground: Option<bool>,
    /// Propeller/rotor blade width by component position, x the part's own.
    pub widths: HashMap<[i32; 3], f64>,
    /// Propeller/rotor blade shape chosen by position: true propeller, false rotor (else auto).
    pub shapes: HashMap<[i32; 3], bool>,
}

/// Whole wings: union of panels with face-adjacent cells (a wing and the surfaces hinged on it).
/// Panels on different bodies join only through a control surface or a shared wing number, so a
/// static sub-grid (a wingtip built as its own grid) is a wing of its own, while ailerons and
/// flaps stay with their wing.
fn wing_groups(grid: &Grid, panel_of: &[usize], n: usize, joins: &dyn Fn(usize, usize) -> bool) -> Vec<usize> {
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
            let Some(b) = grid.index([at[0] + d[0], at[1] + d[1], at[2] + d[2]]).map(|j| panel_of[j]) else {
                continue;
            };
            if b != usize::MAX && joins(a, b) {
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
    if (0.002..0.25).contains(&raw) {
        dt + (raw - dt) * 0.2
    } else {
        dt
    }
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

/// Velocity and spin of a body between two poses `dt` apart (centre of mass, body axes in world):
/// ω = ½ Σ aᵢ × ȧᵢ. For net clients, whose bodies are moved by the host's poses and carry no velocity.
pub fn pose_motion(com0: V3, rot0: &[V3; 3], com1: V3, rot1: &[V3; 3], dt: f64) -> (V3, V3) {
    let w = (0..3).fold(V3::ZERO, |a, i| a + rot0[i].cross(rot1[i] - rot0[i])) * (0.5 / dt);
    ((com1 - com0) * (1.0 / dt), w)
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

/// A jet's full-throttle thrust at `mach` and altitude over its thrust standing still at sea level:
/// the installed thrust lapse of an afterburning turbojet, Mattingly, Heiser & Pratt, Aircraft
/// Engine Design 2nd ed. eq. 2.55a. θ0, δ0: total temperature and pressure over sea level's.
/// shortcut: throttle ratio 1.05, the middle of the book's 1.0 to 1.08 for fighters.
pub fn jet_thrust_lapse(mach: f64, alt: f64) -> f64 {
    const TR: f64 = 1.05;
    let (at, sl) = (atmosphere(alt), atmosphere(0.0));
    let theta = at.temp / sl.temp;
    let ram = 1.0 + 0.2 * mach * mach;
    let theta0 = theta * ram;
    let delta0 = at.rho / sl.rho.max(1e-9) * theta * ram.powf(3.5);
    let hot = if theta0 > TR { 1.5 * (theta0 - TR) / theta0 } else { 0.0 };
    (delta0 * (1.0 - 0.3 * (theta0 - 1.0) - 0.1 * mach.sqrt() - hot)).max(0.0)
}

/// The same for the large intake's fan: a high bypass turbofan, eq. 2.53 of the same book.
/// shortcut: the book fits it to Mach 0.9; past that it keeps falling, to nothing near Mach 1.5.
pub fn fan_thrust_lapse(mach: f64, alt: f64) -> f64 {
    const TR: f64 = 1.05;
    let (at, sl) = (atmosphere(alt), atmosphere(0.0));
    let theta = at.temp / sl.temp;
    let ram = 1.0 + 0.2 * mach * mach;
    let theta0 = theta * ram;
    let delta0 = at.rho / sl.rho.max(1e-9) * theta * ram.powf(3.5);
    let hot = if theta0 > TR { 3.0 * (theta0 - TR) / (1.5 + mach) } else { 0.0 };
    (delta0 * (1.0 - 0.49 * mach.sqrt() - hot)).max(0.0)
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

/// Which live angle of the game deflects a panel (`flight::live_angle`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LiveSrc {
    /// Solid rocket fin pair A (else C).
    Rocket { a: bool },
    /// Type-18 fin / control surface (`comp+0x274`).
    Rudder,
}

/// A panel the game deflects live: component index, angle source, and the sign turning the
/// game's angle into this panel's deflection (`panel_forces`).
#[derive(Clone, Copy, Debug)]
pub struct FinLive {
    pub comp: usize,
    pub src: LiveSrc,
    pub sign: f64,
}

/// Sign of a game rotation about `axis` (part frame) as this panel's deflection: turn the chord
/// and see which way it tilts. `after_mirror`: the game turns the mesh after its mirror flags
/// (rudder_draw `v·F·R·C`) rather than before (solid_rocket_draw `v·R·F·C`). 0 = no tilt.
fn live_sign(c: &Comp, p: &Panel, axis: V3, after_mirror: bool) -> f64 {
    let lin = |v: V3| c.part_to_aero(v) - c.part_to_aero(V3::ZERO);
    let m = if after_mirror { c.mirror } else { 0 };
    let flip = |v: V3| v3(if m & 1 != 0 { -v.x } else { v.x }, if m & 2 != 0 { -v.y } else { v.y }, if m & 4 != 0 { -v.z } else { v.z });
    let e = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    let u_part = e.iter().fold(V3::ZERO, |a, &ei| a + ei * lin(ei).dot(p.u));
    let tilt = lin(flip(game_axis_angle(flip(u_part), axis, 0.01))).dot(p.n);
    if tilt.abs() < 1e-6 {
        0.0
    } else {
        -tilt.signum()
    }
}

/// A rocket fin piece (part frame; rocket axis y) that is a grid fin, its lattice face across the
/// flow: the same points with the face's sideways width turned to run along the rocket axis, a
/// planar fin of the face's area (the lattice's lift acts across the face like a planar fin's).
/// None for a planar fin, which lies along the axis already.
fn grid_face(pts: &[V3]) -> Option<Vec<V3>> {
    let n = pts.len().max(1) as f64;
    let mid = pts.iter().fold(V3::ZERO, |a, &p| a + p) * (1.0 / n);
    let y = v3(0.0, 1.0, 0.0);
    let r = v3(mid.x, 0.0, mid.z).norm();
    let w = y.cross(r);
    let span = |d: V3| pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.dot(d)), hi.max(p.dot(d))));
    let ((y0, y1), (w0, w1)) = (span(y), span(w));
    if !(r.len() > 0.5 && y1 - y0 < 0.5 * (w1 - w0)) {
        return None;
    }
    let (ym, wm) = ((y0 + y1) * 0.5, (w0 + w1) * 0.5);
    Some(pts.iter().map(|&p| r * p.dot(r) + w * wm + y * (ym + p.dot(w) - wm)).collect())
}

/// The game's `mat4_set_axis_angle_rotation` 0x179850 applied as a row vector (p·M).
fn game_axis_angle(p: V3, a: V3, t: f64) -> V3 {
    let (s, c) = t.sin_cos();
    let k = 1.0 - c;
    let r0 = v3(a.x * a.x * k + c, a.x * a.y * k + a.z * s, a.x * a.z * k - a.y * s);
    let r1 = v3(a.x * a.y * k - a.z * s, a.y * a.y * k + c, a.y * a.z * k + a.x * s);
    let r2 = v3(a.x * a.z * k + a.y * s, a.y * a.z * k - a.x * s, a.z * a.z * k + c);
    r0 * p.x + r1 * p.y + r2 * p.z
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
    /// Panel's wing number (`Designations::splits`; 0 unmarked).
    pub separate: Vec<u32>,
    /// Panel's wing counted among the unnumbered wings in panel order (0, 1, ...): a colour index.
    pub wing_rank: Vec<usize>,
    /// Per panel: the rocket fin it is, deflected live by the game (`flight::fin_angle`).
    pub fin_live: Vec<Option<FinLive>>,
    /// Per panel: the component of a missile fin made from its mesh (default section
    /// `Designations::default_fin`). These panels are not in `panel_of`: they share their cells
    /// with the part's body, which is not a wing.
    pub fin_comp: Vec<Option<u32>>,
    /// Component -> its first fin panel, and whether its fins are separate moving solids
    /// (`Grid::moving`; else the whole part stands for them in the views).
    pub fin_of: HashMap<u32, (usize, bool)>,
    pub mass: f64,
    pub com: V3,
    /// Ixx, Iyy, Izz, Ixy, Iyz, Ixz about the CoM, body frame, kg·m².
    pub inertia: [f64; 6],
    pub s_ref: f64,
    pub mac_ref: f64,
    pub b_ref: f64,
    /// Air propellers and rotors the plugin flies (`prop.rs`).
    pub props: Vec<PropUnit>,
    /// Bodies without lifting panels, each its own small fuselage (`bluff_forces`).
    pub bluff: Vec<Bluff>,
    /// The main body (the one with the most parts) and its head-on silhouette: per grid (x, y)
    /// column, whether its hull fills any cell along the length.
    pub main_body: usize,
    pub silhouette: Vec<bool>,
    /// Cells the main fuselage model leaves out (wing panels, bodies with their own model).
    pub body_skip: Vec<bool>,
}

/// A body without lifting panels (a landing gear leg, a door, anything on a pivot) as its own
/// small fuselage: the main body's section model (`BodyAero`), sliced along its longest side.
pub struct Bluff {
    pub body: usize,
    /// Its cells in the vehicle grid.
    pub cells: Vec<usize>,
    /// Grid axis the slices run along.
    axis: usize,
    aero: BodyAero,
    /// Middle of its cells in the slicing frame, m.
    mid: V3,
    /// Solid volume, m³: a body too short to slice drags as a lump of this size.
    volume: f64,
    /// A body one cell thick (an airbrake, a door): its grid normal axis, face area m² and aspect
    /// ratio. Across it, it drags as a flat plate, not as the slices of a fuselage.
    plate: Option<(usize, f64, f64)>,
}

impl Bluff {
    pub fn debug(&self) -> String {
        format!("axis {} length {:.2} max_area {:.3} fineness {:.2} sections {} xs areas {:?}", self.axis, self.aero.length, self.aero.max_area, self.aero.fineness, self.aero.sections.len(), self.aero.xs.iter().map(|x| (x.area * 1000.0).round() / 1000.0).collect::<Vec<_>>())
    }
}

/// Grid vector or cell turned so that axis `a` becomes the grid's forward axis (a rotation: `unturn` undoes it).
fn turn<T: Copy>(c: [T; 3], a: usize) -> [T; 3] {
    [c[(a + 1) % 3], c[(a + 2) % 3], c[a]]
}
fn unturn<T: Copy>(c: [T; 3], a: usize) -> [T; 3] {
    turn(turn(c, a), a)
}


/// One propeller or rotor part.
#[derive(Clone, Debug)]
pub struct PropUnit {
    pub comp: usize,
    pub body: u32,
    /// Blade-plane centre and spin axis, the part's x and z axes (cyclic): aero body frame.
    pub hub: V3,
    pub axis: V3,
    pub ex: V3,
    pub ez: V3,
    /// Spin for positive shaft speed: mirrored parts turn the other way (`force_emitter_draw`).
    pub spin: f64,
    pub def: crate::defs::PropDef,
    /// Blade width, x the part's own (`Designations::widths`).
    pub width: f64,
    /// Propeller blades (cambered, helical twist, rigid), else rotor: the choice, or `def.airscrew`.
    pub airscrew: bool,
}

/// A propeller's live state, read from the game each step.
#[derive(Clone, Copy, Debug, Default)]
pub struct PropLive {
    /// Shaft speed (game units: the blades turn `w · rotation_speed` rev/s).
    pub w: f64,
    /// Blade pitch −1..1 (blade angle 0.75·pitch rad, as drawn) and the cyclic inputs.
    pub pitch: f64,
    pub cyclic: [f64; 2],
    pub blades: u32,
    pub length: f64,
    pub reversed: bool,
    /// The hinge turns the hub against the shaft speed's sign (`blur::hinge_against`).
    pub flip: bool,
}

/// A propeller's forces this step (world frame).
#[derive(Clone, Copy, Debug, Default)]
pub struct PropOut {
    pub force: V3,
    pub at: V3,
    /// Moment about the hub: in-plane (P-factor, hub stiffness) and the air's torque on the spin axis.
    pub moment: V3,
    pub axial: V3,
    pub thrust: f64,
    /// The air's torque against the spin (N·m) and the spin direction it was computed for.
    pub torque: f64,
    pub spin: f64,
    pub omega: f64,
    /// Quasi-steady inflow (m/s) the disc's inflow lags towards, its time constant (s), and the
    /// vortex ring depth (0..1).
    pub v_target: f64,
    pub tau: f64,
    pub vrs: f64,
    /// Blade angle at 0.75 R (rad, collective only) and the stalled share of the blades (0..1).
    pub blade_angle: f64,
    pub stalled: f64,
}

/// Shaft inertia per unit of the game's slot mass, kg·m². The shaft network conserves slot
/// mass × speed (`vehicle_shaft_network_step` 0x4C02A0), so a torque removes `Q·dt / (2π·I)`.
/// The game has no torque unit, so this is a choice: set so an aircraft propeller at its best static
/// pitch drains about half of what the game's own model takes for the same thrust (`T/1500` momentum
/// per tick). Matching the game there (2.4) stalled engines at coarse pitch on take-off, where stalled
/// blades take ~3x the power per thrust; at half, load passes the game's from about 50 m/s (it follows
/// power, the game's follows thrust). Smaller rotors load the shaft less still.
/// shortcut: one factor for all parts; retune from players' readings.
const SHAFT_INERTIA: f64 = 4.8;

/// A propeller's slipstream this tick, world frame: a tube behind the disc that speeds up from
/// `v0 + vi` at the disc to `v0 + 2·vi` far behind, and narrows to keep its flow (momentum theory).
#[derive(Clone, Copy, Debug)]
pub struct Slipstream {
    pub hub: V3,
    /// Thrust direction: the tube runs the other way.
    pub axis: V3,
    pub radius: f64,
    /// Flight speed along the axis and the induced speed at the disc, m/s.
    pub v0: f64,
    pub vi: f64,
}

impl Slipstream {
    /// The air's speed along the tube and the tube's radius, `x` m behind the disc.
    pub fn section(&self, x: f64) -> (f64, f64) {
        let speed = self.v0 + self.vi * (1.0 + x / (x * x + self.radius * self.radius).sqrt());
        (speed, self.radius * ((self.v0 + self.vi) / speed).sqrt())
    }

    /// The slipstream as line segments for the F9 view: (from, to, opacity 0..1, thickness px).
    /// Rings along the narrowing tube and streamlines that twist with the swirl (`circ` = r·vθ, m²/s,
    /// signed about the thrust axis), with bright bands drifting downstream over `time` (s).
    /// Brightest at the disc, fading to nothing 8 radii behind (the wash itself has no end).
    pub fn wash_lines(&self, circ: f64, time: f64) -> Vec<(V3, V3, f32, f32)> {
        use std::f64::consts::TAU;
        const RINGS: usize = 8;
        const AROUND: usize = 24;
        const LINES: usize = 6;
        const STEPS: usize = 32;
        let len = 8.0 * self.radius;
        // A weak wash (idling) is fainter: full strength from 30 m/s over the free stream.
        let strength = (2.0 * self.vi / 30.0).clamp(0.15, 1.0);
        let side = if self.axis.y.abs() < 0.9 { v3(0.0, 1.0, 0.0) } else { v3(1.0, 0.0, 0.0) };
        let e1 = self.axis.cross(side).norm();
        let e2 = self.axis.cross(e1);
        let at = |x: f64, r: f64, th: f64| self.hub - self.axis * x + (e1 * th.cos() + e2 * th.sin()) * r;
        let fade = |x: f64| (1.0 - x / len).max(0.0).powi(2);
        let mut out = Vec::with_capacity(RINGS * AROUND + LINES * STEPS);
        for i in 0..RINGS {
            let x = len * i as f64 / RINGS as f64;
            let r = self.section(x).1;
            for k in 0..AROUND {
                out.push((at(x, r, TAU * k as f64 / AROUND as f64), at(x, r, TAU * (k + 1) as f64 / AROUND as f64), (0.7 * strength * fade(x)) as f32, 2.0));
            }
        }
        let dx = len / STEPS as f64;
        // Bands a radius and a half apart, drifting at a quarter of the wash's speed (readable, not a flicker).
        let drift = 0.25 * (self.v0 + 2.0 * self.vi) * time;
        for j in 0..LINES {
            let mut th = TAU * j as f64 / LINES as f64;
            let mut prev = at(0.0, 0.75 * self.radius, th);
            for i in 1..=STEPS {
                let x = dx * i as f64;
                let (speed, tube) = self.section(x);
                let r = 0.75 * tube;
                // The air turns vθ/r = circ/r² while it moves `speed` along the tube.
                th += circ / (r * r * speed) * dx;
                let p = at(x, r, th);
                let band = 0.5 + 0.5 * (TAU * (x - drift) / (1.5 * self.radius)).cos();
                out.push((prev, p, (0.9 * strength * fade(x - 0.5 * dx) * (0.3 + 0.7 * band)) as f32, 2.0));
                prev = p;
            }
        }
        out
    }

    /// Inside the tube at world point `p`: the air's speed along it there and the radial vector from the axis.
    pub fn at(&self, p: V3) -> Option<(f64, V3)> {
        let d = p - self.hub;
        let x = -d.dot(self.axis);
        if x <= 0.0 {
            return None; // ahead of the disc
        }
        let rv = d + self.axis * x;
        let (speed, tube) = self.section(x);
        (rv.len() <= tube).then_some((speed, rv))
    }
}

/// Cyclic blade angle per unit input, rad.
const CYCLIC_RAD: f64 = 0.2;

impl Model {
    /// Guess what nobody marked yet (the Detect button): wing blocks, and the roles of vanilla
    /// control surfaces without one. Existing marks stay.
    ///
    /// Wing blocks: slabs at most 4 cells thick standing in open air (`voxel::slab_cells`'s
    /// rule), of at least half a square metre, at least twice as wide as thick both ways (so no
    /// thin tail boom), and with nothing of the vehicle above and below most of them (so no
    /// wall or floor of a hollow hull, which faces the opposite wall).
    /// Roles: a surface standing upright is a rudder; a flat one on the widest wing is an aileron
    /// on its outer half and a flap on its inner half; any other flat one is an elevator. With no
    /// elevator found, the flaps are elevons.
    pub fn detect(&self, des: &Designations, lib: &mut Library) -> Designations {
        let g = &self.grid;
        let n = g.owner.len();
        let is_cs: Vec<bool> = self.comps.iter().map(|c| lib.get(&c.def).is_some_and(|d| d.is_control_surface())).collect();
        let solid = |i: usize| g.owner[i] != EMPTY && g.owner[i] != INTERIOR;
        let plain = |i: usize| solid(i) && !is_cs[g.owner[i] as usize];
        let body = |i: usize| self.comps[g.owner[i] as usize].body;
        let step = |i: usize, axis: usize, s: i32| {
            let mut c = g.cell(i);
            c[axis] += s;
            g.index(c)
        };
        let mut out = des.clone();
        for axis in 0..3 {
            let mut seen = vec![false; n];
            for start in 0..n {
                if seen[start] || !plain(start) {
                    continue;
                }
                let b = body(start);
                let member = |i: usize| plain(i) && body(i) == b;
                let (mut cells, mut cols, mut clear) = (Vec::new(), 0usize, 0usize);
                let mut queue = std::collections::VecDeque::from([start]);
                while let Some(i) = queue.pop_front() {
                    if seen[i] || !member(i) {
                        continue;
                    }
                    let mut col = vec![i];
                    let mut ends = [None; 2];
                    for (k, s) in [1, -1].into_iter().enumerate() {
                        let mut j = step(i, axis, s);
                        while let Some(x) = j.filter(|&x| member(x)) {
                            col.push(x);
                            j = step(x, axis, s);
                        }
                        ends[k] = j;
                    }
                    col.iter().for_each(|&c| seen[c] = true);
                    if col.len() > 4 || ends.iter().any(|e| e.is_some_and(|j| g.owner[j] == INTERIOR)) {
                        continue;
                    }
                    // Nothing of the vehicle further out along the axis, either way.
                    let open = |k: usize, s: i32| {
                        let mut j = ends[k];
                        while let Some(x) = j {
                            if g.owner[x] != EMPTY {
                                return false;
                            }
                            j = step(x, axis, s);
                        }
                        true
                    };
                    cols += 1;
                    clear += (open(0, 1) && open(1, -1)) as usize;
                    for &c in &col {
                        cells.push((c, col.len() as i32));
                        let at = g.cell(c);
                        for d in crate::voxel::DIRS.iter().filter(|d| d[axis] == 0) {
                            if let Some(j) = g.index([at[0] + d[0], at[1] + d[1], at[2] + d[2]]) {
                                if !seen[j] {
                                    queue.push_back(j);
                                }
                            }
                        }
                    }
                }
                if cols < 8 || clear * 2 < cols {
                    continue;
                }
                // Each cell: the slab runs at least twice its thickness through it, both ways.
                let inside: HashSet<usize> = cells.iter().map(|c| c.0).collect();
                let run = |c: usize, a: usize| {
                    let reach = |s: i32| std::iter::successors(step(c, a, s), |&j| step(j, a, s)).take_while(|j| inside.contains(j)).count() as i32;
                    1 + reach(1) + reach(-1)
                };
                let wide = cells.iter().filter(|&&(c, thick)| (0..3).filter(|&a| a != axis).all(|a| run(c, a) >= 2 * thick));
                out.wings.extend(wide.map(|&(c, _)| self.comps[g.owner[c] as usize].pos));
            }
        }
        // Whole wings of the marked blocks: the width they span (body y), lying flat, and the
        // length and height they span (body x and z).
        let wing = |i: usize| plain(i) && out.wings.contains(&self.comps[g.owner[i] as usize].pos);
        let mut group_of = vec![usize::MAX; n];
        let mut groups: Vec<((f64, f64), bool, (f64, f64), (f64, f64))> = Vec::new();
        for start in 0..n {
            if group_of[start] != usize::MAX || !wing(start) {
                continue;
            }
            let (mut x, mut y, mut z) = ((f64::MAX, f64::MIN), (f64::MAX, f64::MIN), (f64::MAX, f64::MIN));
            let mut queue = vec![start];
            group_of[start] = groups.len();
            while let Some(i) = queue.pop() {
                let p = aero_pos(g.cell(i));
                (x, y, z) = ((x.0.min(p.x), x.1.max(p.x)), (y.0.min(p.y), y.1.max(p.y)), (z.0.min(p.z), z.1.max(p.z)));
                let at = g.cell(i);
                for d in crate::voxel::DIRS {
                    if let Some(j) = g.index([at[0] + d[0], at[1] + d[1], at[2] + d[2]]).filter(|&j| group_of[j] == usize::MAX && wing(j)) {
                        group_of[j] = groups.len();
                        queue.push(j);
                    }
                }
            }
            groups.push((y, z.1 - z.0 < 0.5 * (y.1 - y.0), x, z));
        }
        // The flat wings' middle and half width; the widest reach at least 80% of it.
        let (y0, y1) = groups.iter().filter(|w| w.1).fold((f64::MAX, f64::MIN), |a, w| (a.0.min(w.0 .0), a.1.max(w.0 .1)));
        let (mid, span) = if y1 > y0 { (0.5 * (y0 + y1), 0.5 * (y1 - y0)) } else { (self.com.y, 0.0) };
        let reach = |y: (f64, f64)| (y.0 - mid).abs().max((y.1 - mid).abs());
        // On the widest wings (a left and a right half are two): beside or just behind one. Not
        // by touching cells: a surface's hinge leaves a gap to the wing.
        let on_main = |c: V3| {
            let within = |v: f64, r: (f64, f64), pad: f64| v >= r.0 - pad && v <= r.1 + pad;
            groups.iter().any(|w| w.1 && reach(w.0) >= 0.8 * span && within(c.x, w.2, 0.75) && within(c.z, w.3, 0.5))
        };
        let mut guessed: Vec<([i32; 3], Role)> = Vec::new();
        for p in &self.panels {
            let Some(&o) = p.cells.first().map(|&c| &g.owner[c]).filter(|&&o| (o as usize) < is_cs.len() && is_cs[o as usize]) else {
                continue;
            };
            let pos = self.comps[o as usize].pos;
            if des.roles.contains_key(&pos) {
                continue;
            }
            let role = if p.n.y.abs() > 0.7 {
                Role::Yaw
            } else if !on_main(p.centroid) {
                Role::Pitch
            } else if (p.centroid.y - mid).abs() > 0.5 * span {
                Role::Roll
            } else {
                Role::Flap
            };
            guessed.push((pos, role));
        }
        let pitch = des.roles.values().chain(guessed.iter().map(|g| &g.1)).any(|r| *r == Role::Pitch);
        for (pos, role) in guessed {
            out.roles.insert(pos, if role == Role::Flap && !pitch { Role::Pitch } else { role });
        }
        out
    }

    /// Shaft speed drop per tick per N·m of our blades' torque: the part's slot mass is
    /// `max_force/1000 · blades/8` (blade parts), shaft torque = blade torque × `rotation_speed`.
    pub fn prop_load(&self, k: usize, blades: u32) -> f64 {
        let d = &self.props[k].def;
        let mass = d.max_force / 1000.0 * blades.clamp(2, 8) as f64 / 8.0;
        if mass > 0.0 {
            d.rotation_speed / (60.0 * 2.0 * PI * SHAFT_INERTIA * mass)
        } else {
            0.0
        }
    }

    /// The pitch input (−1..1, blade angle 0.75·p rad) where part `k` makes the most static
    /// sea-level thrust at blade speed `rps`: the knee where the blades stall and thrust levels
    /// off (more pitch then costs power for no thrust). Returns (pitch, thrust N, power W, blade stall share there).
    pub fn prop_peak(&self, k: usize, blades: u32, length: f64, rps: f64) -> Option<(f64, f64, f64, f64)> {
        let u = &self.props[k];
        if !(rps > 0.0) || u.def.rotation_speed <= 0.0 {
            return None;
        }
        let mut best: Option<(f64, PropOut)> = None;
        for i in 1..=100 {
            let p = i as f64 * 0.01;
            let o = self.prop_settled(k, blades, length, rps, p, 0.0);
            match &best {
                // Past the stall thrust levels off while power keeps climbing: stop at the knee.
                Some((_, b)) if o.stalled > 0.0 && o.thrust < 1.005 * b.thrust => break,
                Some((_, b)) if o.thrust <= b.thrust => {}
                _ => best = Some((p, o)),
            }
        }
        best.map(|(p, o)| (p, o.thrust, o.torque * o.omega, o.stalled))
    }

    /// Part `k` at sea level with its inflow settled: blade speed `rps`, pitch input `pitch`,
    /// flying along its thrust axis at `speed` m/s.
    pub fn prop_settled(&self, k: usize, blades: u32, length: f64, rps: f64, pitch: f64, speed: f64) -> PropOut {
        let u = &self.props[k];
        let mut lb = LiveBody {
            origin: V3::ZERO,
            rot: [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)],
            trans: v3(0.0, 100.0, 0.0),
            com: V3::ZERO,
            vel: V3::ZERO,
            omega: V3::ZERO,
            wind: V3::ZERO,
        };
        lb.vel = lb.dir_to_world(u.axis).norm() * speed;
        let s = PropLive { w: rps / u.def.rotation_speed, pitch, cyclic: [0.0; 2], blades, length, reversed: false, flip: false };
        // Bisect the inflow: more inflow, less thrust, so `v_target − v` falls with v. (Fixed-point
        // iteration oscillates near zero thrust.)
        let vtip = 2.0 * PI * rps * self.prop_geom(k, blades, length).radius;
        let (mut lo, mut hi) = (-0.5 * vtip, 0.5 * vtip);
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            if self.prop_live(k, &lb, &s, mid, false).v_target > mid {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        self.prop_live(k, &lb, &s, 0.5 * (lo + hi), false)
    }

    /// Blade geometry at the part's blade count and length.
    pub fn prop_geom(&self, k: usize, blades: u32, length: f64) -> crate::prop::Prop {
        let d = &self.props[k].def;
        let s = crate::defs::PropDef::scale(length);
        crate::prop::Prop {
            radius: d.radius * s,
            hub: d.root * s,
            chord: d.chord * s * self.props[k].width,
            blades: blades.clamp(2, 8),
            foil: if self.props[k].airscrew { 2 } else { 4 }, // NACA 2412 propellers, NACA 0012 rotors
            propeller: self.props[k].airscrew,
        }
    }

    /// Forces of propeller `k` on body pose `lb` at mean inflow `v` (m/s).
    pub fn prop_live(&self, k: usize, lb: &LiveBody, s: &PropLive, v: f64, vrs: bool) -> PropOut {
        let u = &self.props[k];
        let geom = self.prop_geom(k, s.blades, s.length);
        let at = lb.to_world(u.hub);
        let axis = lb.dir_to_world(u.axis).norm() * if s.reversed { -1.0 } else { 1.0 };
        let air = lb.wind - (lb.vel + lb.omega.cross(at - lb.com));
        let atm = atmosphere(at.y);
        // Turning backwards (a counter-rotating rotor on a reversing gearbox) it is the mirror-image rotor:
        // left-handed blades at the opposite pitch, so thrust follows sign(speed · pitch) as in the game.
        // The game has no left-handed blades, so the trailing edge leading is not held against it.
        let back = s.w < 0.0;
        let spin = u.spin * if back { -1.0 } else { 1.0 } * if s.flip { -1.0 } else { 1.0 };
        let beta = 0.75 * s.pitch.clamp(-1.0, 1.0);
        let mut cyclic = V3::ZERO;
        if u.def.rotor() {
            let t = lb.dir_to_world(u.ex) * s.cyclic[0] + lb.dir_to_world(u.ez) * s.cyclic[1];
            cyclic = axis.cross(t) * (CYCLIC_RAD * spin);
        }
        let flow = crate::prop::PropFlow {
            axis,
            spin,
            omega: 2.0 * PI * s.w.abs() * u.def.rotation_speed,
            pitch: if back { -beta } else { beta },
            cyclic,
            air,
            rho: atm.rho,
            a: atm.a,
        };
        // "Coaxial" rotors are one disc with a power output on top for a second rotor.
        let f = geom.forces(&flow, v);
        let (v_target, vrs_depth, tau) = geom.inflow(&flow, f.thrust, vrs);
        PropOut {
            force: f.force,
            at,
            moment: f.moment,
            axial: axis * f.moment.dot(axis),
            thrust: f.thrust,
            torque: f.torque,
            spin,
            omega: flow.omega,
            v_target,
            tau,
            vrs: vrs_depth,
            blade_angle: flow.pitch,
            stalled: f.stalled,
        }
    }

    /// The slipstream of propeller `k` (None for rotors, and when it is stopped or windmilling).
    /// `v`: induced speed at the disc, m/s.
    pub fn slipstream(&self, k: usize, lb: &LiveBody, s: &PropLive, v: f64) -> Option<Slipstream> {
        let u = &self.props[k];
        let axis = lb.dir_to_world(u.axis).norm() * if s.reversed { -1.0 } else { 1.0 };
        let hub = lb.to_world(u.hub);
        let v0 = (lb.vel + lb.omega.cross(hub - lb.com) - lb.wind).dot(axis).max(0.0);
        let radius = self.prop_geom(k, s.blades, s.length).radius;
        (u.airscrew && v > 0.5 && radius > 0.0).then_some(Slipstream { hub, axis, radius, v0, vi: v })
    }

    /// Rolling moment (world, N·m) the propeller's rotating slipstream puts on the wing and tail
    /// cells inside it: it opposes the torque reaction. Momentum theory, constant circulation: the
    /// wake carries the torque as swirl, r·vθ = Q / (ρ·V_disc·πR²). A cell meets the swirl as a
    /// change of angle of attack at the wake's speed; its lift slope is that of a wing as wide as
    /// the tube, and it stalls at cl 1.2. Never more than Q: that is all the swirl there is.
    /// NACA TN 1146 measured 40-50 % cancelled by wing + fuselage on a low-wing single without its tail.
    pub fn swirl_recovery(&self, slip: &Slipstream, bodies: &[LiveBody], out: &PropOut) -> V3 {
        if !(out.torque > 1.0) || bodies.is_empty() {
            return V3::ZERO;
        }
        let (axis, radius) = (slip.axis, slip.radius);
        let rho = atmosphere(slip.hub.y).rho;
        let circ = out.torque / (rho * (slip.v0 + slip.vi) * PI * radius * radius);
        // Per panel: how far behind the disc it sits, and the moment the full swirl would give it.
        let mut parts: Vec<(f64, f64)> = Vec::new();
        for p in self.panels.iter().filter(|p| !p.cells.is_empty()) {
            let b = &bodies[self.panel_body(p).min(bodies.len() - 1)];
            let n = b.dir_to_world(p.n);
            let cell_area = p.area / p.cells.len() as f64;
            let ar = 2.0 * radius / p.mac.max(0.05);
            let slope = 2.0 * PI * ar / (ar + 2.0);
            let mut m = 0.0;
            for &c in &p.cells {
                let Some((speed, rv)) = slip.at(b.to_world(crate::body::aero_pos(self.grid.cell(c)))) else { continue };
                let r = rv.len().max(1e-6);
                // The air turns the way the blades do: along spin·(axis × r).
                let swirl = axis.cross(rv) * (out.spin * circ / (r * r.max(0.2 * radius)));
                let cl = (slope * swirl.dot(n) / speed).clamp(-1.2, 1.2);
                let f = n * (0.5 * rho * speed * speed * cl * cell_area);
                m += rv.cross(f).dot(axis) * out.spin;
            }
            if m != 0.0 {
                parts.push((-(b.to_world(p.centroid) - slip.hub).dot(axis), m));
            }
        }
        // Front to back: each surface meets only the swirl the ones ahead of it left.
        parts.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut left = out.torque;
        for (_, m) in parts {
            left -= (m * left / out.torque).clamp(-left, left);
        }
        // Against the torque reaction (−spin·Q about the axis) only; `left` never passes zero, so at
        // most all of it: a design with enough surface in the slipstream straightens it completely.
        axis * ((out.torque - left).clamp(0.0, out.torque) * out.spin)
    }

    pub fn build(comps: Vec<Comp>, lib: &mut Library, des: &Designations) -> Model {
        let grid = Grid::build(&comps, lib);
        let (mass, com, inertia) = mass_properties(&comps, &grid, lib);

        let mut body_role: HashMap<u32, Role> = HashMap::new();
        let mut comp_role: HashMap<u32, Role> = HashMap::new();
        let index: HashMap<[i32; 3], u32> = comps.iter().enumerate().map(|(i, c)| (c.pos, i as u32)).collect();
        let mut is_cs = vec![false; comps.len()];
        let mut is_wing = vec![false; comps.len()];
        // Parts whose flat mesh pieces lift as their own panels (missile fins far below voxel
        // size): rocket fins always, other non-block parts once marked as wing. Vanilla wing
        // parts stay voxel panels, so they keep joining into whole wings.
        let mut plates: Vec<(usize, Arc<crate::defs::Def>)> = Vec::new();
        for (i, c) in comps.iter().enumerate() {
            if let Some(d) = lib.get(&c.def) {
                is_cs[i] = d.is_control_surface();
                is_wing[i] = d.is_wing();
                // A marked joint half (the pivot head on a flap) is a wing block like its neighbours,
                // not its own fins.
                let marked = des.wings.contains(&c.pos) && ![crate::defs::TYPE_RUDDER, crate::defs::TYPE_MULTIBODY].contains(&d.type_) && !d.block;
                if !d.plates.is_empty() && (d.fins || marked) {
                    plates.push((i, d.clone()));
                    continue;
                }
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
            if !is_wing[o as usize] {
                return None;
            }
            Some((des.splits.get(&c.pos).map_or(Key::Wing(c.body), |&g| Key::Split(c.body, g)), Role::None))
        };
        let mut panels = build_panels(&grid, &key_of, com);
        // Section per panel: the aerofoil set on any of its components, else its wing's (below),
        // else the default.
        let mut marked_foil = vec![false; panels.len()];
        for (p, marked) in panels.iter_mut().zip(&mut marked_foil) {
            let mark = p.cells.iter().map(|&c| grid.owner[c]).filter(|&o| (o as usize) < comps.len()).find_map(|o| des.foils.get(&comps[o as usize].pos).copied());
            *marked = mark.is_some();
            p.foil = mark.unwrap_or(des.default_foil);
        }
        // Voxel panels replace the skin of their cells; plate panels share cells with the body.
        let mut no_skin = vec![false; grid.fill.len()];
        for p in &panels {
            for &c in &p.cells {
                no_skin[c] = true;
            }
        }
        // Vanilla fins and control surfaces: one panel set per component (`Key::Comp`).
        let mut fin_live: Vec<Option<FinLive>> = panels
            .iter()
            .map(|p| {
                let o = *grid.owner.get(*p.cells.first()?)? as usize;
                let d = lib.get(&comps.get(o)?.def).filter(|_| is_cs[o])?;
                let sign = live_sign(&comps[o], p, d.hinge, true);
                (sign != 0.0).then_some(FinLive { comp: o, src: LiveSrc::Rudder, sign })
            })
            .collect();
        let mut fin_comp: Vec<Option<u32>> = vec![None; panels.len()];
        for (i, d) in &plates {
            let c = &comps[*i];
            for (piece, ctrl) in &d.plates {
                let raw: Vec<V3> = piece.iter().flat_map(|t| t.0).collect();
                let grid_fin = ctrl.is_some() && grid_face(&raw).is_some();
                let pts: Vec<V3> = grid_face(&raw).filter(|_| grid_fin).unwrap_or(raw).into_iter().map(|p| c.part_to_aero(p)).collect();
                // Cells the fin reaches, its part's own cell first (its body for live forces).
                let mut cells: Vec<usize> = std::iter::once(c.pos)
                    .chain(pts.iter().map(|p| [(p.y / VOXEL_M).round() as i32, (-p.z / VOXEL_M).round() as i32, (p.x / VOXEL_M).round() as i32]))
                    .filter_map(|g| grid.index(g))
                    .collect();
                let mut seen = std::collections::HashSet::new();
                cells.retain(|x| seen.insert(*x));
                let tris: Vec<[V3; 3]> = piece.iter().map(|t| t.0.map(|q| c.part_to_aero(q))).collect();
                if let Some(mut p) = crate::wing::plate_panel(&pts, &tris, cells, Role::None, com) {
                    p.foil = des.foils.get(&c.pos).copied().unwrap_or(des.default_fin.unwrap_or(crate::wing::FIN_DEFAULT));
                    let live = ctrl.map(|f| FinLive { comp: *i, src: LiveSrc::Rocket { a: f.a }, sign: live_sign(c, &p, f.axis, false) });
                    panels.push(p);
                    fin_live.push(live);
                    fin_comp.resize(panels.len() - 1, None);
                    fin_comp.push(Some(*i as u32));
                }
            }
        }
        fin_comp.resize(panels.len(), None);
        let mut panel_of = vec![usize::MAX; grid.fill.len()];
        for (i, p) in panels.iter().enumerate().filter(|(i, _)| fin_comp[*i].is_none()) {
            for &c in &p.cells {
                if panel_of[c] == usize::MAX {
                    panel_of[c] = i;
                }
            }
        }
        let body_of: Vec<usize> = panels
            .iter()
            .map(|p| p.cells.iter().map(|&c| grid.owner[c]).find(|&o| (o as usize) < comps.len()).map_or(0, |o| comps[o as usize].body as usize))
            .collect();
        // Same rule as the flight loop's main body: most parts, lowest index on a tie.
        let mut count: HashMap<usize, usize> = HashMap::new();
        comps.iter().for_each(|c| *count.entry(c.body as usize).or_default() += 1);
        let main_body = count.into_iter().max_by_key(|&(b, n)| (n, std::cmp::Reverse(b))).map_or(0, |(b, _)| b);
        let column = grid.dims[0] * grid.dims[1];
        let mut silhouette = vec![false; column];
        for (c, &o) in grid.owner.iter().enumerate() {
            if o == INTERIOR || comps.get(o as usize).is_some_and(|k| k.body as usize == main_body) {
                silhouette[c % column] = true;
            }
        }
        // Bodies with no lifting panel get their own model. Their cells that stick out of the main
        // body's outline as built leave the main fuselage, so they are not counted twice.
        let mut by_body: HashMap<usize, Vec<usize>> = HashMap::new();
        for (c, &o) in grid.owner.iter().enumerate().filter(|(c, &o)| (o as usize) < comps.len() && grid.fill[*c] > 0.0) {
            by_body.entry(comps[o as usize].body as usize).or_default().push(c);
        }
        let lifting: std::collections::HashSet<usize> = body_of.iter().copied().collect();
        let mut skip = vec![false; grid.fill.len()];
        let mut bluff: Vec<Bluff> = by_body
            .into_iter()
            .filter(|(b, _)| *b != main_body && !lifting.contains(b))
            .map(|(body, cells)| {
                let at: Vec<[i32; 3]> = cells.iter().map(|&c| grid.cell(c)).collect();
                let extent = |a: usize| at.iter().map(|p| p[a]).max().unwrap_or(0) - at.iter().map(|p| p[a]).min().unwrap_or(0);
                // Longest side; on a tie the one nearest the vehicle's own axis.
                let axis = [0, 1, 2].into_iter().max_by_key(|&a| (extent(a), a)).unwrap_or(2);
                let local: Vec<([i32; 3], f32, u32)> = cells.iter().zip(&at).map(|(&c, &p)| (turn(p, axis), grid.fill[c], 0)).collect();
                let aero = BodyAero::build(&Grid::from_cells(&local), &[], &[]);
                let mid = local.iter().fold(V3::ZERO, |a, l| a + aero_pos(l.0)) * (1.0 / local.len() as f64);
                let volume = cells.iter().map(|&c| grid.fill[c] as f64).sum::<f64>() * VOXEL_M.powi(3);
                for &c in &cells {
                    skip[c] = !silhouette[c % column];
                }
                // shortcut: only one-cell-thick bodies count as plates; thicker slabs keep the slice model.
                let thin: Vec<usize> = (0..3).filter(|&a| extent(a) == 0).collect();
                let plate = match thin[..] {
                    [n] if extent((n + 1) % 3) > 0 && extent((n + 2) % 3) > 0 => {
                        let area = at.iter().map(|p| (p[(n + 1) % 3], p[(n + 2) % 3])).collect::<HashSet<_>>().len() as f64 * VOXEL_M * VOXEL_M;
                        Some((n, area, (extent((n + 1) % 3) + 1) as f64 / (extent((n + 2) % 3) + 1) as f64))
                    }
                    _ => None,
                };
                Bluff { body, cells, axis, aero, mid, volume, plate }
            })
            .collect();
        bluff.sort_by_key(|b| b.body);
        // Wing panel cells are the wing model's alone: as fuselage volume too, a wing of square
        // blocks dragged as a blunt slab whatever its aerofoil, and twice when broadside.
        for (s, &w) in skip.iter_mut().zip(&no_skin) {
            *s |= w;
        }
        let body = BodyAero::build(&grid, &no_skin, &skip);

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
        // Separate-wing marks: a panel joins only panels of the same wing number.
        let separate: Vec<u32> = panels
            .iter()
            .map(|p| p.cells.iter().map(|&c| grid.owner[c]).filter(|&o| (o as usize) < comps.len()).find_map(|o| des.splits.get(&comps[o as usize].pos).copied()).unwrap_or(0))
            .collect();
        let joins =
            |a: usize, b: usize| separate[a] == separate[b] && (body_of[a] == body_of[b] || separate[a] != 0 || panels[a].role != Role::None || panels[b].role != Role::None);
        let mut wing_of = wing_groups(&grid, &panel_of, panels.len(), &joins);
        // One part's fins are one wing (one click sets them all).
        let mut fin_of: HashMap<u32, (usize, bool)> = HashMap::new();
        for (j, c) in fin_comp.iter().enumerate() {
            if let Some(c) = *c {
                let first = fin_of.entry(c).or_insert((j, false)).0;
                wing_of[j] = wing_of[first];
            }
        }
        // A wing's panels without an aerofoil mark take the one set on the wing (a panel painted
        // onto a wing later gets the wing's aerofoil).
        let mut wing_foil: HashMap<usize, usize> = HashMap::new();
        for (j, p) in panels.iter().enumerate() {
            if fin_comp[j].is_none() && marked_foil[j] {
                wing_foil.entry(wing_of[j]).or_insert(p.foil);
            }
        }
        for (j, p) in panels.iter_mut().enumerate() {
            if fin_comp[j].is_none() && !marked_foil[j] {
                p.foil = wing_foil.get(&wing_of[j]).copied().unwrap_or(p.foil);
            }
        }
        for &k in &grid.moving {
            if let Some(f) = fin_of.get_mut(&grid.solids[k].0) {
                f.1 = true;
            }
        }
        let props = comps
            .iter()
            .enumerate()
            .filter_map(|(i, c)| {
                let def = lib.get(&c.def)?.prop.clone()?;
                let r = c.rot.map(|v| v as f64);
                // Part frame -> grid: p·R (+ voxel position), as `voxel::place`.
                let dir = |p: V3| v3(p.x * r[0] + p.y * r[3] + p.z * r[6], p.x * r[1] + p.y * r[4] + p.z * r[7], p.x * r[2] + p.y * r[5] + p.z * r[8]);
                let pos = v3(c.pos[0] as f64, c.pos[1] as f64, c.pos[2] as f64) * VOXEL_M;
                let det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
                Some(PropUnit {
                    comp: i,
                    body: c.body,
                    hub: grid_to_aero(pos + dir(v3(0.0, def.blade_height, 0.0))),
                    axis: grid_to_aero(dir(def.axis)),
                    ex: grid_to_aero(dir(v3(1.0, 0.0, 0.0))),
                    ez: grid_to_aero(dir(v3(0.0, 0.0, 1.0))),
                    spin: if det < 0.0 { -1.0 } else { 1.0 },
                    width: des.widths.get(&c.pos).map_or(1.0, |&w| w.clamp(0.25, 2.0)),
                    airscrew: des.shapes.get(&c.pos).copied().unwrap_or(def.airscrew),
                    def,
                })
            })
            .collect();
        let mut seen: Vec<usize> = Vec::new();
        let wing_rank = (0..wing_of.len())
            .map(|j| match separate[j] {
                0 => seen.iter().position(|s| *s == wing_of[j]).unwrap_or_else(|| {
                    seen.push(wing_of[j]);
                    seen.len() - 1
                }),
                _ => 0,
            })
            .collect();
        Model { comps, grid, body, panels, panel_of, wing_of, separate, wing_rank, fin_live, fin_comp, fin_of, mass, com, inertia, s_ref, mac_ref, b_ref, props, bluff, main_body, silhouette, body_skip: skip }
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
        let (mut fb, mut mb) = self.body.forces(&flow, -vel, omega, self.com, at.rho);
        // Bodies without wings (landing gear) as built: counted with the fuselage.
        let frame = LiveBody { origin: V3::ZERO, rot: [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)], trans: V3::ZERO, com: V3::ZERO, vel: V3::ZERO, omega: V3::ZERO, wind: V3::ZERO };
        let frame = LiveBody { com: frame.to_world(self.com), vel: frame.dir_to_world(vel), omega: frame.axial_to_world(omega), ..frame };
        let nb = self.comps.iter().map(|k| k.body as usize).max().map_or(1, |b| b + 1);
        for (_, f, p) in self.bluff_forces(&vec![frame; nb], self.main_body, &at) {
            let f = frame.dir_to_aero(f);
            fb += f;
            mb += (grid_to_aero(frame.world_to_grid(p)) - self.com).cross(f);
        }

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
    /// `deflect`: live control deflection per panel, radians (empty = none).
    /// `slips`: propeller slipstreams. A panel's cells inside one meet its faster air (strip theory by
    /// the share of cells inside), so a tail in the propwash steers before the aeroplane moves.
    pub fn live_forces(&self, bodies: &[LiveBody], main: usize, stall: &mut [f64], ground: &[f64], deflect: &[f64], slips: &[Slipstream]) -> Vec<LiveForce> {
        let Some(mb) = bodies.get(main) else {
            return Vec::new();
        };
        let com_w = mb.to_world(self.com);
        let air = |b: &LiveBody, p: V3| b.vel + b.omega.cross(p - b.com) - b.wind;
        let alt = com_w.y;
        let at = atmosphere(alt);
        let v_main = air(mb, com_w);
        let speed = v_main.len();
        let owner: Vec<usize> = self.panels.iter().map(|p| self.panel_body(p).min(bodies.len().saturating_sub(1))).collect();
        // Per panel: the extra speed along each propeller's axis that gives its mix of free and
        // slipstream air the right dynamic pressure, q = (1 − φ)·q_free + φ·q_slip.
        let mut wash = vec![V3::ZERO; self.panels.len()];
        for sl in slips {
            for ((p, &b), w) in self.panels.iter().zip(&owner).zip(&mut wash).filter(|((p, _), _)| !p.cells.is_empty()) {
                let lb = &bodies[b];
                let (mut n, mut q) = (0usize, 0.0);
                for &c in &p.cells {
                    if let Some((v, _)) = sl.at(lb.to_world(crate::body::aero_pos(self.grid.cell(c)))) {
                        n += 1;
                        q += v * v;
                    }
                }
                if n > 0 {
                    let phi = n as f64 / p.cells.len() as f64;
                    let free = air(lb, lb.to_world(p.centroid)).dot(sl.axis).max(0.0);
                    *w += sl.axis * ((((1.0 - phi) * free * free + q / p.cells.len() as f64).sqrt() - free).max(0.0));
                }
            }
        }
        let washed = wash.iter().map(|w| w.len()).fold(0.0, f64::max);
        if speed.max(washed) < 1.0 || !speed.is_finite() {
            stall.iter_mut().for_each(|s| *s = 0.0);
            return Vec::new();
        }
        // Standing still in the propwash: the panels' Mach and Reynolds numbers come from the wash.
        let speed = speed.max(washed);
        let mach = speed / at.a;
        let re_m = reynolds_per_m(&at, mach);
        let mut out = Vec::new();

        // Fuselage (slender body) in the main body's frame.
        let len = self.body.length.max(0.25);
        let re = re_m * len;
        let flow = Flow { mach, reynolds_per_m: re_m, pseudo_knudsen: mach / (re + mach), cf: skin_friction(re, mach) };
        let (fb, mbm) = if v_main.len() >= 1.0 {
            self.body.forces(&flow, -mb.dir_to_aero(v_main), mb.axial_to_aero(mb.omega), self.com, at.rho)
        } else {
            (V3::ZERO, V3::ZERO)
        };
        out.push(LiveForce { body: main, force: mb.dir_to_world(fb), at: com_w, torque: mb.axial_to_world(mbm) });

        // Lifting panels where their body is now: a deflected sub-grid is a tilted panel.
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
        let deflect = if deflect.len() == world.len() { deflect } else { &zero };
        let pf = panel_forces(&world, &wf, &|i, p| air(&bodies[owner[i]], p) + wash[i], deflect, stall);
        for (f, &b) in pf.iter().zip(&owner) {
            out.push(LiveForce { body: b, force: f.force, at: f.at, torque: V3::ZERO });
        }
        // After the panels (the flight data reads force 1 + i as panel i): each gets its own drag arrow.
        for (b, force, point) in self.bluff_forces(bodies, main, &at) {
            out.push(LiveForce { body: b, force, at: point, torque: V3::ZERO });
        }
        out
    }

    /// Air forces on the bodies without lifting panels, where they are now (a retracting leg
    /// swings with its pivot). Each is its own small fuselage in its own flow: skin friction with
    /// the form factor of its fineness along it, crossflow drag across it (FAR's section model).
    /// Only the share of its cells outside the main body's head-on outline counts: inside it a
    /// cell is part of the streamlined shape (a leg in its well, a body continuing the fuselage).
    /// Returns (body, force, point), world frame.
    pub fn bluff_forces(&self, bodies: &[LiveBody], main: usize, at: &Atmo) -> Vec<(usize, V3, V3)> {
        let Some(mb) = bodies.get(main).filter(|_| main == self.main_body) else { return Vec::new() };
        let shielded = |w: V3| {
            let g = mb.world_to_grid(w);
            let (x, y) = ((g.x / VOXEL_M).round() as i32 - self.grid.min[0], (g.y / VOXEL_M).round() as i32 - self.grid.min[1]);
            x >= 0 && y >= 0 && (x as usize) < self.grid.dims[0] && (y as usize) < self.grid.dims[1] && self.silhouette[y as usize * self.grid.dims[0] + x as usize]
        };
        let mut out = Vec::new();
        for bl in &self.bluff {
            let Some(lb) = bodies.get(bl.body) else { continue };
            let open: Vec<(V3, f64)> = bl.cells.iter().map(|&c| (lb.to_world(aero_pos(self.grid.cell(c))), self.grid.fill[c] as f64)).filter(|&(w, _)| !shielded(w)).collect();
            // By filled volume, not cell count: a thin plate left flush outside the outline is nearly no drag.
            let open_fill: f64 = open.iter().map(|o| o.1).sum();
            if !(open_fill > 0.0) {
                continue;
            }
            let point = open.iter().fold(V3::ZERO, |a, &(w, f)| a + w * f) * (1.0 / open_fill);
            let v = lb.vel + lb.omega.cross(point - lb.com) - lb.wind;
            let speed = v.len();
            if !(speed >= 1.0) || !speed.is_finite() {
                continue;
            }
            let mach = (speed / at.a).max(0.001);
            let re_m = reynolds_per_m(at, mach);
            let force = if bl.aero.sections.is_empty() {
                // One or two cells long: a lump, drag across it on a face of its size.
                let size = bl.volume.cbrt();
                v * (-0.5 * at.rho * speed * crate::body::crossflow_drag(mach, re_m * size) * size * size)
            } else {
                let g = lb.world_dir_to_grid(v);
                let [x, y, z] = turn([g.x, g.y, g.z], bl.axis);
                let re = re_m * bl.aero.length.max(VOXEL_M);
                let flow = Flow { mach, reynolds_per_m: re_m, pseudo_knudsen: mach / (re + mach), cf: skin_friction(re, mach) };
                let f = aero_to_grid(bl.aero.forces(&flow, -grid_to_aero(v3(x, y, z)), V3::ZERO, bl.mid, at.rho).0);
                let [x, y, z] = unturn([f.x, f.y, f.z], bl.axis);
                lb.dir_to_world(grid_to_aero(v3(x, y, z)))
            };
            let force = match bl.plate {
                // Keep the slice model's force along the face (friction), swap the one across it for a
                // flat plate's normal force, Cn = Cd90·sin α (Hoerner's post-stall plate).
                Some((axis, area, ar)) => {
                    let mut e = [0.0; 3];
                    e[axis] = 1.0;
                    let n = lb.grid_dir_to_world(v3(e[0], e[1], e[2]));
                    let along = force - n * force.dot(n);
                    along - n * (0.5 * at.rho * speed * v.dot(n) * area * crate::wing::flat_plate_cd(mach, ar))
                }
                None => force,
            };
            out.push((bl.body, force * (open_fill / bl.cells.iter().map(|&c| self.grid.fill[c] as f64).sum::<f64>()), point));
        }
        out
    }

    /// Body (`Comp::body`) a panel's cells belong to.
    pub fn panel_body(&self, p: &Panel) -> usize {
        p.cells.iter().map(|&c| self.grid.owner[c]).find(|&o| (o as usize) < self.comps.len()).map_or(0, |o| self.comps[o as usize].body as usize)
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
            [[s.zw, s.zu, s.zq + s.u0, 0.0], [s.xw, s.xu, s.xq, -G], [s.mw, s.mu, s.mq, 0.0], [0.0, 0.0, 1.0, 0.0]]
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
        // A microcontroller weighs its width x length (the game's one mass override).
        let m = c.size.map(|s| (s[0] * s[1]) as f64).or_else(|| lib.get(&c.def).map(|d| d.mass as f64)).unwrap_or(1.0);
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
    fn grid_fins_lift_across_their_face() {
        // A modded grid fin rocket (lattice faces across the flow): four live planar-equivalent
        // panels of the face's area, normal across the rocket axis, with the default fin section.
        let home = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default());
        let game = home.join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = home.join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let mut lib = crate::defs::Library::new(&game, &appdata);
        let name = "solid_rocket_grid_fin_small";
        if lib.get(name).is_none() {
            return;
        }
        // Rocket axis (part y) pointing forward: grid z, aero x.
        let comp = Comp { def: name.into(), pos: [0, 0, 0], rot: [1, 0, 0, 0, 0, 1, 0, -1, 0], body: 0, mirror: 0, prop: None, size: None, wheel_size: None };
        let m = Model::build(vec![comp], &mut lib, &Designations::default());
        let axis = v3(1.0, 0.0, 0.0);
        let fins: Vec<(&Panel, &FinLive)> = m.panels.iter().zip(&m.fin_live).filter_map(|(p, f)| Some((p, f.as_ref()?))).collect();
        assert_eq!(fins.len(), 4);
        // The rocket body is no wing: fin panels stay out of the cell map, the fins are their own solids.
        assert!(m.panel_of.iter().all(|&p| p == usize::MAX || m.fin_comp[p].is_none()));
        assert_eq!(m.fin_of.get(&0).map(|f| f.1), Some(true));
        for (p, f) in fins {
            assert_eq!(p.foil, crate::wing::FIN_DEFAULT);
            assert!(p.n.dot(axis).abs() < 0.05 && p.u.dot(axis).abs() > 0.95, "normal {:?} chord {:?}", p.n, p.u);
            assert!((0.02..0.035).contains(&p.area), "face area {}", p.area);
            assert!(f.sign != 0.0);
        }
    }

    #[test]
    fn vanilla_control_surfaces_deflect_live() {
        // Every vanilla control-surface panel of the saved planes reads its live angle, and the
        // mirror handling holds: mirroring along the hinge keeps the turn, across it reverses it
        // (F·R(a,θ)·F = R(F·a, det F·θ)).
        let home = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default());
        let game = home.join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = home.join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let mut lib = crate::defs::Library::new(&game, &appdata);
        let mut checked = 0;
        for name in ["f4e", "beaver", "bf109", "ar234b2", "c27j"] {
            let Ok(text) = std::fs::read_to_string(appdata.join(format!("data/vehicles/{name}.xml"))) else {
                continue;
            };
            let m = Model::build(crate::vehicle::parse_vehicle_xml(&text), &mut lib, &Designations::default());
            for (p, f) in m.panels.iter().zip(&m.fin_live) {
                let o = m.grid.owner[p.cells[0]] as usize;
                let d = lib.get(&m.comps[o].def).unwrap();
                if !d.is_control_surface() {
                    assert!(!matches!(f, Some(FinLive { src: LiveSrc::Rudder, .. })), "{name}: live non-control panel");
                    continue;
                }
                let f = f.unwrap_or_else(|| panic!("{name}: control surface {} has no live angle", m.comps[o].def));
                let h = [d.hinge.x, d.hinge.y, d.hinge.z];
                let along = (0..3).find(|&k| h[k] != 0.0).unwrap();
                let c = &m.comps[o];
                let with = |bit: i32| live_sign(&Comp { mirror: c.mirror ^ bit, ..c.clone() }, p, d.hinge, true);
                assert_eq!(with(0), f.sign);
                assert_eq!(with(1 << along), f.sign, "{name}: mirror along the hinge");
                assert_eq!(with(1 << ((along + 1) % 3)), -f.sign, "{name}: mirror across the hinge");
                checked += 1;
            }
        }
        if checked > 0 {
            eprintln!("{checked} live control-surface panels");
        }
    }

    #[test]
    fn rocket_fin_deflection_matches_the_game_rotation() {
        // Saved AIM-9: each live rocket-fin panel, deflected by sign·α, must face where the game's
        // own rotation of the fin mesh (p·M about its axis, in the part frame) puts it.
        let home = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default());
        let game = home.join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = home.join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let Ok(text) = std::fs::read_to_string(appdata.join("data/vehicles/aim9.xml")) else {
            return;
        };
        let mut lib = crate::defs::Library::new(&game, &appdata);
        let m = Model::build(crate::vehicle::parse_vehicle_xml(&text), &mut lib, &Designations::default());
        let mut checked = 0;
        for (p, f) in m.panels.iter().zip(&m.fin_live) {
            let Some(f) = f else { continue };
            let c = &m.comps[f.comp];
            let d = lib.get(&c.def).unwrap();
            let alpha = 0.2;
            // The fin piece whose rest panel is this one (same normal and centroid).
            let fit = |rot: f64, ctrl: crate::defs::FinCtrl, piece: &[crate::defs::Tri]| {
                let tris: Vec<[V3; 3]> = piece.iter().map(|t| t.0.map(|q| c.part_to_aero(game_axis_angle(q, ctrl.axis, rot)))).collect();
                let pts: Vec<V3> = tris.iter().flatten().copied().collect();
                crate::wing::plate_panel(&pts, &tris, vec![0], Role::None, m.com).unwrap()
            };
            let (piece, ctrl) = d
                .plates
                .iter()
                .filter_map(|(t, k)| Some((t, (*k)?)))
                .find(|(t, k)| f.src == LiveSrc::Rocket { a: k.a } && (fit(0.0, *k, t).centroid - p.centroid).len() < 1e-6)
                .unwrap();
            let truth = fit(alpha, ctrl, piece).n;
            let ours = |t: f64| (p.n * t.cos() + p.u * t.sin()).norm();
            let (good, bad) = (ours(f.sign * alpha).dot(truth).abs(), ours(-f.sign * alpha).dot(truth).abs());
            assert!(good > 0.999 && good > bad, "fin {checked}: ours {good} vs flipped {bad}");
            checked += 1;
        }
        assert_eq!(checked, 4, "the AIM-9's one finned rocket has 4 live fins");
    }

    #[test]
    fn numbered_separate_wings_stay_apart_and_share_their_aerofoil() {
        // A flat slab: wing 1 across the middle (split into left and right panels at the
        // symmetry plane), wing 2 touching its tip, plain wing beyond. One aerofoil mark on
        // wing 1's left half sets its right half too.
        let home = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default());
        let game = home.join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = home.join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let mut lib = crate::defs::Library::new(&game, &appdata);
        if lib.get("01_block").is_none() {
            return;
        }
        let block = |x: i32, z: i32| Comp { def: "01_block".into(), pos: [x, 0, z], rot: crate::vehicle::IDENTITY, body: 0, mirror: 0, prop: None, size: None, wheel_size: None };
        let comps: Vec<Comp> = (-12..=12).flat_map(|x| (0..8).map(move |z| block(x, z))).collect();
        let mut des = Designations::default();
        des.wings.extend(comps.iter().map(|c| c.pos));
        for c in &comps {
            match c.pos[0] {
                -8..=8 => des.splits.insert(c.pos, 1),
                9..=10 => des.splits.insert(c.pos, 2),
                _ => None,
            };
        }
        des.foils.insert([-5, 0, 3], 3);
        let m = Model::build(comps, &mut lib, &des);
        let wing_no = |j: usize| m.separate[j];
        let of = |n: u32| (0..m.panels.len()).filter(|&j| wing_no(j) == n).collect::<Vec<_>>();
        let (one, two) = (of(1), of(2));
        assert_eq!(one.len(), 2, "wing 1 split at the symmetry plane");
        assert!(one.iter().all(|&j| m.panels[j].foil == 3 && m.wing_of[j] == m.wing_of[one[0]]));
        assert!(!two.is_empty() && two.iter().all(|&j| m.wing_of[j] != m.wing_of[one[0]] && m.panels[j].foil == des.default_foil));
    }

    #[test]
    fn detect_marks_free_slabs_but_no_hull_and_guesses_roles() {
        let home = std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default());
        let game = home.join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = home.join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let mut lib = crate::defs::Library::new(&game, &appdata);
        if lib.get("01_block").is_none() {
            return;
        }
        let block = |x: i32, y: i32, z: i32| Comp { def: "01_block".into(), pos: [x, y, z], rot: crate::vehicle::IDENTITY, body: 0, mirror: 0, prop: None, size: None, wheel_size: None };
        let mut comps = Vec::new();
        // A hollow open-ended hull 5 x 5 x 30 (walls one block thick), a thin 2 x 2 tail boom,
        // a wing through the hull, a tailplane on the boom and a fin on top of it.
        for z in 0..30 {
            for (x, y) in (-2..=2).flat_map(|x| (-2..=2).map(move |y| (x, y))).filter(|(x, y): &(i32, i32)| x.abs() == 2 || y.abs() == 2) {
                comps.push(block(x, y, z));
            }
        }
        comps.extend((-20..0).flat_map(|z| [(0, 0), (1, 0), (0, 1), (1, 1)].map(|(x, y)| block(x, y, z))));
        let wing: Vec<[i32; 3]> = (-16..=16).filter(|x: &i32| x.abs() > 2).flat_map(|x| (12..18).map(move |z| [x, 0, z])).collect();
        let tail: Vec<[i32; 3]> = (-6..=7).filter(|x| !(0..=1).contains(x)).flat_map(|x| (-19..-15).map(move |z| [x, 0, z])).collect();
        let fin: Vec<[i32; 3]> = (2..8).flat_map(|y| (-19..-15).map(move |z| [0, y, z])).collect();
        comps.extend(wing.iter().chain(&tail).chain(&fin).map(|p| block(p[0], p[1], p[2])));
        let m = Model::build(comps, &mut lib, &Designations::default());
        let d = m.detect(&Designations::default(), &mut lib);
        let want: HashSet<[i32; 3]> = wing.iter().chain(&tail).chain(&fin).copied().collect();
        let missed: Vec<&[i32; 3]> = want.difference(&d.wings).collect();
        // Nothing else but boom blocks where the tailplane and fin cross it (their centre section).
        let extra: Vec<&[i32; 3]> = d.wings.difference(&want).filter(|p| !(-19..-15).contains(&p[2])).collect();
        assert!(extra.is_empty() && missed.is_empty(), "extra {extra:?} missed {missed:?}");
        // Marks already there stay, and a second pass adds nothing.
        assert_eq!(m.detect(&d, &mut lib), d);
    }

    #[test]
    fn step_from_motion() {
        let v = v3(100.0, -5.0, 20.0);
        let mut dt = 1.0 / 60.0;
        for _ in 0..60 {
            dt = measure_step(dt, v * 0.025, v); // the game slowed to 40 steps per game second
        }
        assert!((dt - 0.025).abs() < 1e-4);
        assert_eq!(measure_step(dt, v3(0.01, 0.0, 0.0), v3(0.5, 0.0, 0.0)), dt);
        // too slow to measure
    }

    #[test]
    fn slipstream_tube_speeds_up_and_narrows_behind_the_disc() {
        let sl = Slipstream { hub: V3::ZERO, axis: v3(1.0, 0.0, 0.0), radius: 1.5, v0: 20.0, vi: 10.0 };
        assert!(sl.at(v3(1.0, 0.0, 0.0)).is_none(), "ahead of the disc");
        let (near, _) = sl.at(v3(-0.01, 1.45, 0.0)).unwrap();
        let (far, rv) = sl.at(v3(-30.0, 1.0, 0.0)).unwrap();
        assert!((near - 30.0).abs() < 0.2 && (far - 40.0).abs() < 0.2 && (rv - v3(0.0, 1.0, 0.0)).len() < 1e-9, "{near} {far}");
        // Far behind the tube is R·√(30/40) = 1.30 m.
        assert!(sl.at(v3(-30.0, 1.25, 0.0)).is_some() && sl.at(v3(-30.0, 1.35, 0.0)).is_none());
        // The drawing starts at the disc, fades out by 8 radii, and never leaves the tube.
        let lines = sl.wash_lines(10.0, 0.3);
        assert!(lines.iter().all(|l| l.2 >= 0.0 && l.2 <= 1.0 && l.0.x <= 1e-9 && l.0.x >= -12.0 - 1e-9 && (l.0.y * l.0.y + l.0.z * l.0.z).sqrt() <= 1.5 + 1e-9));
        let far = lines.iter().filter(|l| l.0.x < -11.0).map(|l| l.2).fold(0.0, f32::max);
        let near = lines.iter().filter(|l| l.0.x > -1.0).map(|l| l.2).fold(0.0, f32::max);
        assert!(far < 0.01 && near > 0.2, "near {near} far {far}");
    }

    #[test]
    fn pose_motion_recovers_velocity_and_spin() {
        // 0.02 rad about +y in 0.1 s while moving 3 m along x.
        let (c, sn) = (0.02f64.cos(), 0.02f64.sin());
        let r0 = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
        let r1 = [v3(c, 0.0, -sn), v3(0.0, 1.0, 0.0), v3(sn, 0.0, c)];
        let (v, w) = pose_motion(V3::ZERO, &r0, v3(3.0, 0.0, 0.0), &r1, 0.1);
        assert!((v.x - 30.0).abs() < 1e-9 && (w.y - 0.2).abs() < 1e-3 && w.x.abs() < 1e-9 && w.z.abs() < 1e-9, "{v:?} {w:?}");
    }

    #[test]
    fn touching_panels_form_one_wing() {
        let grid = Grid {
            min: [0, 0, 0],
            dims: [5, 1, 1],
            fill: vec![1.0; 5],
            owner: vec![0; 5],
            solids: Vec::new(),
            moving: Default::default(),
            missing: Vec::new(),
            blades: Default::default(),
        };
        let w = wing_groups(&grid, &[0, 1, usize::MAX, 2, 2], 3, &|_, _| true);
        assert_eq!(w[0], w[1]);
        assert_ne!(w[0], w[2]);
        // Panel 1 on another body (a static wingtip grid): its own wing, unless it is a control surface.
        let body = [0, 1, 0];
        let w = wing_groups(&grid, &[0, 1, usize::MAX, 2, 2], 3, &|a, b| body[a] == body[b]);
        assert_ne!(w[0], w[1]);
        let cs = [false, true, false];
        let w = wing_groups(&grid, &[0, 1, usize::MAX, 2, 2], 3, &|a, b| body[a] == body[b] || cs[a] || cs[b]);
        assert_eq!(w[0], w[1]);
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
    fn atmosphere_curve() {
        assert!((atmosphere(0.0).rho - 1.225).abs() < 1e-9);
        assert!((atmosphere(10_000.0).rho - 0.3143).abs() < 0.001, "Stormworks curve at 10 km");
        assert_eq!(atmosphere(41_000.0).rho, 0.0);
        assert!((atmosphere(0.0).a - 340.3).abs() < 0.5);
    }
}

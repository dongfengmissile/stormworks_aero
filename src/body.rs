//! Voxel (fuselage / body) aerodynamics: port of FAR's `VehicleAerodynamics`
//! (cross-sections, smoothing, area-rule wave drag, sonic pressure) and
//! `FARAeroSection` (per-section potential + viscous crossflow force model).
//! FAR source: KSPModStewards/Ferram-Aerospace-Research, GPLv3.

use std::f64::consts::PI;

use crate::math::{v3, Curve, V3};
use crate::voxel::{Grid, EMPTY, VOXEL_M};

/// FAR "Full Drag, Moderate Area Ruling" preset (FAR's default difficulty).
const GAUSSIAN_LENGTH_FRACTION: f64 = 0.015;
const AREA_SMOOTHING_PASSES: i32 = 2;
const DERIV_SMOOTHING_PASSES: i32 = 1;

/// Grid cell centre in the aero body frame (x fwd, y right, z down), metres.
pub fn aero_pos(c: [i32; 3]) -> V3 {
    v3(c[2] as f64 * VOXEL_M, c[0] as f64 * VOXEL_M, -c[1] as f64 * VOXEL_M)
}

/// One slice across the main axis (FAR `VoxelCrossSection`).
#[derive(Clone, Default)]
pub struct Xsec {
    pub area: f64,
    /// Area before smoothing, for the area-ruling plot.
    pub raw_area: f64,
    pub centroid: V3,
    pub second_deriv: f64,
    pub flatness: f64,
    pub flat_normal: V3,
    pub cp_sonic_fwd: f64,
    pub cp_sonic_back: f64,
    /// Exposed voxel face area in this slice.
    pub surface: f64,
    /// Aero-frame x of the slice centre.
    pub x: f64,
}

/// A run of merged slices with its force model (FAR `FARAeroSection`).
#[derive(Clone)]
pub struct Section {
    xforce_aoa0: Curve,
    xforce_aoa180: Curve,
    skin: Curve,
    potential_normal: f64,
    visc_crossflow: f64,
    flatness: f64,
    inv_flatness: f64,
    hyp_moment_fwd: f64,
    hyp_moment_back: f64,
    diameter: f64,
    merge_factor: f64,
    pub centroid: V3,
    x_ref: V3,
    n_ref: V3,
}

pub struct BodyAero {
    /// Every slice of the padded grid, nose first.
    pub xs: Vec<Xsec>,
    pub front: usize,
    pub back: usize,
    pub thickness: f64,
    pub length: f64,
    pub max_area: f64,
    pub fineness: f64,
    pub critical_mach: f64,
    /// Mach 1 wave drag area, m² (FAR `SonicDragArea`).
    pub sonic_drag_area: f64,
    pub sections: Vec<Section>,
}

/// Flow state shared by every section of one evaluation.
pub struct Flow {
    pub mach: f64,
    pub reynolds_per_m: f64,
    pub pseudo_knudsen: f64,
    /// Skin friction coefficient for the vehicle length.
    pub cf: f64,
}

impl BodyAero {
    /// `no_skin[i]`: grid cell i belongs to a wing panel, whose skin friction the wing
    /// model already counts (2·Cf in its Cd0), so its faces are left out of the wetted area.
    pub fn build(grid: &Grid, no_skin: &[bool]) -> BodyAero {
        let mut xs = cross_sections(grid, no_skin);
        let thickness = VOXEL_M;
        let filled: Vec<usize> = (0..xs.len()).filter(|&i| xs[i].area > 0.0).collect();
        let (front, back) = match (filled.first(), filled.last()) {
            (Some(&f), Some(&b)) if b > f + 1 => (f, b),
            _ => {
                return BodyAero {
                    xs,
                    front: 0,
                    back: 0,
                    thickness,
                    length: 0.0,
                    max_area: 0.0,
                    fineness: 0.0,
                    critical_mach: 1.0,
                    sonic_drag_area: 0.0,
                    sections: Vec::new(),
                }
            }
        };
        raw_second_derivs(&mut xs, front, back, thickness);
        let mut max_area = xs[front..=back].iter().map(|s| s.area).fold(0.0, f64::max);
        let mut body = BodyAero {
            xs,
            front,
            back,
            thickness,
            length: 0.0,
            max_area,
            fineness: 0.0,
            critical_mach: 1.0,
            sonic_drag_area: 0.0,
            sections: Vec::new(),
        };
        let inner = [0, 1, 2].map(|a| (grid.dims[a] - 2) as f64);
        let voxel_volume = inner[0] * inner[1] * inner[2] * VOXEL_M.powi(3);
        body.properties(voxel_volume, &mut max_area);
        body
    }

    /// FAR `CalculateVesselAeroProperties` (minus intake ducting).
    fn properties(&mut self, voxel_volume: f64, max_area: &mut f64) {
        let (front, back, t) = (self.front, self.back, self.thickness);
        let num_sections = back - front;
        let length = t * num_sections as f64;
        self.length = length;

        let filled_volume: f64 = self.xs[front..=back].iter().map(|s| s.area).sum::<f64>() * t;
        let grid_filledness = filled_volume / voxel_volume * 25.0;
        let std_dev_cutoff = (3.0 * grid_filledness).clamp(0.5, 3.0);

        let fineness = length * 0.5 / (*max_area / PI).sqrt();
        let extra_deriv = (((5.0 - fineness) * 0.5).round() as i32 * DERIV_SMOOTHING_PASSES).max(0);
        let extra_area = (((grid_filledness / 25.0 - 0.5) * 4.0).round() as i32).max(0);

        gaussian_smooth(
            &mut self.xs,
            std_dev_cutoff,
            GAUSSIAN_LENGTH_FRACTION,
            t,
            length,
            front,
            back,
            AREA_SMOOTHING_PASSES + extra_area,
            DERIV_SMOOTHING_PASSES + extra_deriv,
        );
        sonic_pressure(&mut self.xs, front, back, t);

        let fineness = length * 0.5 / (*max_area / PI).sqrt();
        self.fineness = fineness;
        // DATCOM 4.2.3.1 body form factor on each slice's own wetted area. FAR also divides
        // by the section count, which makes subsonic skin drag N times smaller than at Mach 2;
        // we keep the documented intent (see docs/DESIGN.md).
        let viscous_drag_factor = 60.0 / fineness.powi(3) + 0.0025 * fineness + 1.0;
        let crit = critical_mach_number(fineness) * self.crit_factor_unsmooth(fineness);
        self.critical_mach = crit;
        // FAR: `blendFactor = lowFinenessRatioFactor--` keeps the value BEFORE the decrement, so
        // `1 - blend` is negative. That contradicts its comments but is what keeps the Mach 1
        // pressure term a drag (using the decremented value gave negative wave drag on real
        // vehicles), so it is kept as written.
        let low_fineness = 1.0 / (1.0 + 0.5 * fineness);
        let blend = 1.0 + low_fineness;

        let main_axis = v3(1.0, 0.0, 0.0);
        let mut sections: Vec<Section> = Vec::new();
        for i in 0..=num_sections {
            let index = i + front;
            let xs = &self.xs;
            let cur = xs[index].area;
            let prev = if i == 0 { 0.0 } else { xs[index - 1].area };
            let next = if i == num_sections { 0.0 } else { xs[index + 1].area };

            let mut potential = if i == 0 {
                next - cur
            } else if i == num_sections {
                cur - prev
            } else {
                (next - prev) * 0.5
            };
            let area_change_max = (next.min(prev) * 0.1).min(length * 0.01);
            let mut sonic_base_drag = 0.21 * potential;
            if potential > area_change_max {
                potential = area_change_max;
            } else if potential < -area_change_max {
                potential = -area_change_max;
            } else if area_change_max != 0.0 {
                sonic_base_drag *= (potential / area_change_max).abs();
            }

            let flatness = xs[index].flatness;
            let flat2 = flatness * flatness;
            if flatness >= 1.0 {
                sonic_base_drag /= flat2;
            } else {
                sonic_base_drag *= flat2;
            }

            let hyp_fwd = hypersonic_drag(prev, cur, t);
            let hyp_back = hypersonic_drag(next, cur, t);
            let mut fwd_frac = if cur != prev { (hyp_fwd * 0.5 / (cur - prev)).abs() } else { 0.0 };
            let mut back_frac = if cur != next { (hyp_back * 0.5 / (cur - next)).abs() } else { 0.0 };
            fwd_frac = fwd_frac.powi(4);
            back_frac = back_frac.powi(4);
            if flatness >= 1.0 {
                fwd_frac /= flat2;
                back_frac /= flat2;
            } else {
                fwd_frac *= flat2;
                back_frac *= flat2;
            }
            let hyp_moment_fwd = hypersonic_moment(prev, cur, t);
            let hyp_moment_back = hypersonic_moment(next, cur, t);

            let cp_fwd = xs[index].cp_sonic_fwd;
            let cp_back = xs[index].cp_sonic_back;
            let area_for_forces = (cur + prev - (next + cur)) * 0.5;

            // Key 0 (critical Mach) of both pressure curves, then the sonic drag at Mach 1.
            let (k0_fwd, k0_back);
            let mut sonic0;
            let mut sonic180;
            let base_fwd = if sonic_base_drag < 0.0 { sonic_base_drag * 0.2 } else { 0.0 };
            let base_back = if sonic_base_drag > 0.0 { sonic_base_drag * 0.2 } else { 0.0 };
            k0_fwd = (base_fwd + 0.325 * hyp_fwd * fwd_frac) * low_fineness;
            k0_back = (base_back - 0.325 * hyp_back * back_frac) * low_fineness;
            let fwd_frac = (fwd_frac + 1.0) * 0.5;
            let back_frac = (back_frac + 1.0) * 0.5;
            let base_s0 = if sonic_base_drag < 0.0 { sonic_base_drag } else { 0.0 };
            let base_s180 = if sonic_base_drag > 0.0 { sonic_base_drag } else { 0.0 };
            sonic0 = -(cp_fwd * area_for_forces) + base_s0 + 0.3 * hyp_fwd * fwd_frac;
            sonic0 *= 1.0 - blend;
            sonic0 += (hyp_fwd * fwd_frac * 1.4 + base_s0) * blend;
            sonic180 = cp_back * -area_for_forces + base_s180 - 0.3 * hyp_back * back_frac;
            sonic180 *= 1.0 - blend;
            sonic180 += (-hyp_back * back_frac * 1.4 + base_s180) * blend;

            let d0 = sonic0.abs() - hyp_fwd.abs();
            let d180 = sonic180.abs() - hyp_back.abs();
            let aoa0 = Curve(vec![
                [crit, k0_fwd, 0.0],
                [1.0, sonic0, 0.0],
                [2.0, sonic0 * 0.5773503 + (1.0 - 0.5773503) * hyp_fwd, -0.2735292 * d0],
                [5.0, sonic0 * 0.2041242 + (1.0 - 0.2041242) * hyp_fwd, -0.04252587 * d0],
                [10.0, sonic0 * 0.1005038 + (1.0 - 0.1005038) * hyp_fwd, -0.0101519 * d0],
                [35.0, hyp_fwd, 0.0],
            ]);
            let aoa180 = Curve(vec![
                [crit, k0_back, 0.0],
                [1.0, sonic180, 0.0],
                [2.0, sonic180 * 0.5773503 - (1.0 - 0.5773503) * hyp_back, -0.2735292 * d180],
                [5.0, sonic180 * 0.2041242 - (1.0 - 0.2041242) * hyp_back, -0.04252587 * d180],
                [10.0, sonic180 * 0.1005038 - (1.0 - 0.1005038) * hyp_back, -0.0101519 * d180],
                [35.0, -hyp_back, 0.0],
            ]);

            // Local body axis: follows the centroid line, at most 10° off the main axis.
            let x_ref = if index == front || index == back {
                main_axis
            } else {
                let d = xs[index - 1].centroid - xs[index + 1].centroid;
                let off = d.reject(main_axis);
                let tan = off.len() / (2.0 * t);
                if tan > 0.176_326_980_708_464_97 {
                    (main_axis + off.norm() * 0.176_326_980_708_464_97).norm()
                } else {
                    d.norm()
                }
            };
            let n_ref = rotate_from_to(main_axis, x_ref, xs[index].flat_normal);

            let surface = xs[index].surface;
            let skin = Curve(vec![
                [0.0, surface * viscous_drag_factor, 0.0],
                [1.0, surface * viscous_drag_factor, 0.0],
                [2.0, surface, 0.0],
            ]);
            let visc_crossflow = (cur / PI).sqrt() * t * 2.0;
            let section = Section {
                xforce_aoa0: aoa0,
                xforce_aoa180: aoa180,
                skin,
                potential_normal: potential,
                visc_crossflow,
                flatness,
                inv_flatness: 1.0 / flatness,
                hyp_moment_fwd,
                hyp_moment_back,
                diameter: visc_crossflow / t,
                merge_factor: 0.0,
                centroid: xs[index].centroid,
                x_ref,
                n_ref,
            };
            match sections.last_mut() {
                Some(p) if p.can_merge(&section) => p.merge(&section),
                _ => sections.push(section),
            }
        }
        self.sections = sections;

        // Mach 1 wave drag area: FAR `UpdateSonicDragArea` (M 1, q 1, flow along the axis).
        let flow = Flow { mach: 1.0, reynolds_per_m: 50000.0, pseudo_knudsen: 0.0, cf: 0.005 };
        let (f, _) = self.forces(&flow, -main_axis, V3::ZERO, V3::ZERO, 2.0);
        self.sonic_drag_area = -f.x;
    }

    fn crit_factor_unsmooth(&self, fineness: f64) -> f64 {
        let inv_t = 1.0 / self.thickness;
        let mut max_rate = 0.0f64;
        let mut max_d2 = 0.0f64;
        let mut prev = 0.0;
        for s in &self.xs {
            max_rate = max_rate.max((s.area - prev).abs() * inv_t);
            prev = s.area;
            max_d2 = max_d2.max(s.second_deriv.abs());
        }
        let a = self.max_area;
        let k = 0.5 * max_rate + 0.3 * max_d2;
        let max_adj = 0.5 + (a - 0.5 * k) / (2.0 * a + 5.0 * k);
        (3.5 * (1.0 - max_adj) / (4.0 + fineness) + max_adj).min(1.0)
    }

    /// Sum of all section forces and torques (about `com`), body frame, newtons.
    /// `air_vel`: air velocity relative to the vehicle CoM (= −vehicle velocity), m/s.
    /// `omega`: body rotation rate, rad/s. `q_scale`: multiplies ½ρV² (ρ passed in through it).
    pub fn forces(&self, flow: &Flow, air_vel: V3, omega: V3, com: V3, rho: f64) -> (V3, V3) {
        let mut force = V3::ZERO;
        let mut torque = V3::ZERO;
        for s in &self.sections {
            let r = s.centroid - com;
            // Velocity of this section through the air.
            let vel = -air_vel + omega.cross(r);
            let (f, t) = s.force(flow, vel, omega, rho);
            force += f;
            torque += t + r.cross(f);
        }
        (force, torque)
    }
}

impl Section {
    fn can_merge(&self, o: &Section) -> bool {
        if self.merge_factor >= 10.0 {
            return false;
        }
        let flat_rel = (self.flatness - o.flatness) * self.inv_flatness;
        if flat_rel < 0.05 && self.flatness >= 1.05 && self.n_ref.dot(o.n_ref).abs() <= 0.999 {
            return false;
        }
        (self.diameter - o.diameter) / self.diameter < 0.05
    }

    fn merge(&mut self, o: &Section) {
        self.merge_factor += 1.0;
        let m = self.merge_factor;
        let inv = 1.0 / (m + 1.0);
        self.potential_normal += o.potential_normal;
        self.visc_crossflow += o.visc_crossflow;
        self.hyp_moment_fwd += o.hyp_moment_fwd;
        self.hyp_moment_back += o.hyp_moment_back;
        self.flatness = inv * (self.flatness * m + o.flatness);
        self.inv_flatness = inv * (self.inv_flatness * m + o.inv_flatness);
        self.diameter = inv * (self.diameter * m + o.diameter);
        self.xforce_aoa0.add(&o.xforce_aoa0);
        self.xforce_aoa180.add(&o.xforce_aoa180);
        self.skin.add(&o.skin);
        self.centroid = (self.centroid * m + o.centroid) * inv;
        self.x_ref = (self.x_ref * m + o.x_ref).norm();
        self.n_ref = (self.n_ref * m + o.n_ref).norm();
    }

    /// FAR `CalculateAeroForces` + `SimulatedForceContext.ApplyForce`, in newtons.
    /// `vel`: this section's velocity through the air.
    fn force(&self, flow: &Flow, vel: V3, omega: V3, rho: f64) -> (V3, V3) {
        let speed2 = vel.dot(vel);
        if speed2 < 1e-12 {
            return (V3::ZERO, V3::ZERO);
        }
        let mach = flow.mach;
        let skin_force = flow.cf * self.skin.eval(mach);
        let x0 = self.xforce_aoa0.eval(mach);
        let x180 = self.xforce_aoa180.eval(mach);
        let x_ref = self.x_ref;
        let vn = vel.norm();
        let normal_dir = (-vn).reject(x_ref).norm();

        let cos = x_ref.dot(vn);
        let cos2 = cos * cos;
        let sin2 = (1.0 - cos2).max(0.0);
        let sin = sin2.sqrt();
        let sin_2a = 2.0 * sin * cos.abs();
        let cos_half = (0.5 + 0.5 * cos.abs()).sqrt();

        let mut n_force = (self.potential_normal * cos.signum() * cos_half * sin_2a).max(0.0);
        let mut nff = normal_dir.dot(self.n_ref).abs();
        nff *= nff;
        let nff = self.inv_flatness * (1.0 - nff) + self.flatness * nff;
        let cf_mach = mach * sin;
        let cf_re = flow.reynolds_per_m * self.diameter * sin / nff;
        n_force += self.visc_crossflow * sin2 * crossflow_drag(cf_mach, cf_re);
        n_force *= nff;

        let mut x_force = -skin_force * cos.signum() * cos2;
        let mut local_vel_force = x_force * flow.pseudo_knudsen;
        x_force -= local_vel_force;
        local_vel_force = local_vel_force.abs();

        let mut moment = cos * sin;
        let mut damping = 4.0 * moment;
        let blend = |a: f64, b: f64| {
            if mach > 6.0 {
                a
            } else if mach < 0.6 {
                0.6 * b
            } else {
                let k = -0.185185185 * mach + 1.11111111111;
                k * b * 0.6 + (1.0 - k) * a
            }
        };
        let moment_factor = if cos > 0.0 {
            x_force += cos2 * x0;
            blend(self.hyp_moment_fwd, self.hyp_moment_back)
        } else {
            x_force += cos2 * x180;
            blend(self.hyp_moment_back, self.hyp_moment_fwd)
        };
        moment *= moment_factor;
        damping *= moment_factor;
        moment /= nff;
        let damping = damping.abs() * 0.1;
        let roll_damping = skin_force * 0.5 * self.diameter * (0.75 + self.flatness * 0.25);

        let mut f = x_ref * x_force + normal_dir * n_force - vn * local_vel_force;
        let mut t = x_ref.cross(normal_dir) * moment;
        let axial = x_ref * x_ref.dot(omega);
        let off_axis = omega - axial;
        t -= off_axis * damping + axial * (roll_damping * axial.len() / speed2.max(0.001));

        let q = 0.5 * rho * speed2;
        f = f * q;
        t = t * q;
        (f, t)
    }
}

fn crossflow_drag(mach: f64, re: f64) -> f64 {
    let mach_curve = Curve(vec![
        [0.0, 1.2, 0.0],
        [0.3, 1.2, 0.0],
        [0.7, 1.5, 0.0],
        [0.85, 1.41, 0.0],
        [0.95, 2.1, 0.0],
        [1.0, 2.0, -2.0],
        [1.3, 1.6, -0.5],
        [2.0, 1.4, -0.1],
        [5.0, 1.25, -0.02],
        [10.0, 1.2, 0.0],
    ]);
    if mach > 0.5 {
        return mach_curve.eval(mach);
    }
    // shortcut: the 250000 key's slope is a central difference (KSP's auto tangent is not in the repo).
    let re_curve = Curve(vec![
        [10000.0, 1.0, 0.0],
        [100000.0, 1.008_333_333_333_333_3, 0.0],
        [180000.0, 1.008_333_333_333_333_3, 0.0],
        [250000.0, 0.666_666_666_666_666_7, -6.32e-6],
        [300000.0, 0.25, -5e-7],
        [500000.0, 0.208_333_333_333_333_33, 0.0],
        [1000000.0, 0.333_333_333_333_333_3, 7e-8],
        [10000000.0, 0.583_333_333_333_333_4, 0.0],
    ]);
    let influence = if mach > 0.4 { 1.0 - (mach - 0.4) * 10.0 } else { 1.0 };
    ((re_curve.eval(re) - 1.0) * influence + 1.0) * mach_curve.eval(mach)
}

/// Rotate `v` by the shortest rotation taking unit `from` to unit `to`.
fn rotate_from_to(from: V3, to: V3, v: V3) -> V3 {
    let axis = from.cross(to);
    let s = axis.len();
    let c = from.dot(to);
    if s < 1e-12 {
        return v;
    }
    let k = axis * (1.0 / s);
    // Rodrigues
    v * c + k.cross(v) * s + k * (k.dot(v) * (1.0 - c))
}

/// Slices along the main axis (grid Z, nose first) with area, centroid, flatness, exposure.
fn cross_sections(g: &Grid, no_skin: &[bool]) -> Vec<Xsec> {
    let [nx, ny, nz] = g.dims;
    let cell_area = VOXEL_M * VOXEL_M;
    let mut out = Vec::with_capacity(nz);
    for zi in (0..nz).rev() {
        let z = g.min[2] + zi as i32;
        let mut s = Xsec { x: z as f64 * VOXEL_M, ..Default::default() };
        let (mut cnt, mut sy, mut sz) = (0.0, 0.0, 0.0);
        let mut pts: Vec<(f64, f64, f64)> = Vec::new();
        let mut exposed = 0.0;
        for yi in 0..ny {
            for xi in 0..nx {
                let i = xi + nx * (yi + ny * zi);
                if g.owner[i] == EMPTY {
                    continue;
                }
                let c = [g.min[0] + xi as i32, g.min[1] + yi as i32, z];
                let f = g.fill[i].min(1.0) as f64;
                let p = aero_pos(c);
                cnt += f;
                sy += p.y * f;
                sz += p.z * f;
                pts.push((p.y, p.z, f));
                if no_skin.get(i).copied().unwrap_or(false) {
                    continue;
                }
                exposed += g.exposed_area(i);
            }
        }
        s.surface = exposed * cell_area;
        s.flatness = 1.0;
        s.flat_normal = v3(0.0, 0.0, 1.0);
        if cnt > 0.0 {
            let (cy, cz) = (sy / cnt, sz / cnt);
            s.centroid = v3(s.x, cy, cz);
            // Second moments about the centroid, each voxel counted with its own extent.
            let own = cell_area / 12.0;
            let (mut iyy, mut izz, mut iyz) = (0.0, 0.0, 0.0);
            for (y, z, f) in pts {
                iyy += f * ((y - cy).powi(2) + own);
                izz += f * ((z - cz).powi(2) + own);
                iyz += f * (y - cy) * (z - cz);
            }
            // Eigen-decomposition of [[iyy, iyz], [iyz, izz]].
            let mean = 0.5 * (iyy + izz);
            let d = (0.25 * (iyy - izz).powi(2) + iyz * iyz).sqrt();
            let (l1, l2) = (mean + d, (mean - d).max(1e-12));
            // Eigenvector of the small eigenvalue = face normal of the flat shape.
            let (ey, ez) = if iyz.abs() > 1e-12 {
                (iyz, l2 - iyy)
            } else if iyy <= izz {
                (1.0, 0.0)
            } else {
                (0.0, 1.0)
            };
            s.flat_normal = v3(0.0, ey, ez).norm();
            s.flatness = (l1 / l2).sqrt().sqrt().min(10.0);
        } else {
            s.centroid = v3(s.x, 0.0, 0.0);
        }
        s.area = cnt * cell_area;
        s.raw_area = s.area;
        out.push(s);
    }
    out
}

fn raw_second_derivs(xs: &mut [Xsec], front: usize, back: usize, t: f64) {
    let denom = 1.0 / (t * t);
    for i in front..=back {
        let (m1, a0, p1) = if i == front {
            (xs[i].area, xs[i + 1].area, xs[i + 2].area)
        } else if i == back {
            (xs[i - 2].area, xs[i - 1].area, xs[i].area)
        } else {
            (xs[i - 1].area, xs[i].area, xs[i + 1].area)
        };
        xs[i].second_deriv = (m1 + p1 - 2.0 * a0) * denom;
    }
}

#[allow(clippy::too_many_arguments)]
fn gaussian_smooth(
    xs: &mut [Xsec],
    std_dev_cutoff: f64,
    length_fraction: f64,
    t: f64,
    length: f64,
    front: usize,
    back: usize,
    area_passes: i32,
    deriv_passes: i32,
) {
    let std_dev = length * length_fraction;
    let num_vals = (std_dev_cutoff * std_dev / t).ceil() as usize;
    if num_vals <= 1 {
        return;
    }
    let inv_var = 1.0 / (std_dev * std_dev);
    let mut g: Vec<f64> = (0..num_vals).map(|i| (-0.5 * (i as f64 * t).powi(2) * inv_var).exp()).collect();
    let sum: f64 = g[0] + 2.0 * g[1..].iter().sum::<f64>();
    g.iter_mut().for_each(|v| *v /= sum);

    let smooth = |xs: &mut [Xsec], get: fn(&Xsec) -> f64, set: fn(&mut Xsec, f64)| {
        let mut prev = vec![get(&xs[front]); num_vals];
        let mut future = vec![0.0; num_vals - 1];
        for i in front..=back {
            for k in (1..num_vals).rev() {
                prev[k] = prev[k - 1];
            }
            prev[0] = get(&xs[i]);
            for (k, fv) in future.iter_mut().enumerate() {
                *fv = if i + k < back { get(&xs[i + k + 1]) } else { get(&xs[back]) };
            }
            let mut cur = 0.0;
            let mut border = 1.0;
            for k in 0..num_vals {
                cur += g[k] * prev[k];
                if prev[k] == 0.0 {
                    border -= g[k];
                }
            }
            for k in 0..num_vals - 1 {
                cur += g[k + 1] * future[k];
                if future[k] == 0.0 {
                    border -= g[k + 1];
                }
            }
            if border > 0.0 {
                cur /= border;
            }
            set(&mut xs[i], cur);
        }
    };
    for _ in 0..area_passes {
        smooth(xs, |s| s.area, |s, v| s.area = v);
    }
    second_derivs(xs, num_vals, front, back, t);
    for _ in 0..deriv_passes {
        smooth(xs, |s| s.second_deriv, |s, v| s.second_deriv = v);
    }
}

/// Holoborodko noise-robust second derivative (FAR `CalculateCrossSectionSecondDerivs`).
fn second_derivs(xs: &mut [Xsec], one_sided: usize, front: usize, back: usize, t: f64) {
    let m = one_sided.clamp(2, 40);
    let n = 2 * m + 1;
    fn sk(k: i64, m: i64, n: i64) -> i64 {
        if k > m {
            return 0;
        }
        if k == m {
            return 1;
        }
        ((2 * n - 10) * sk(k + 1, m, n) - (n + 2 * k + 3) * sk(k + 2, m, n)) / (n - 2 * k - 1)
    }
    let s: Vec<f64> = (0..=m).map(|k| sk(k as i64, m as i64, n as i64) as f64).collect();
    let denom = 1.0 / (2f64.powi(n as i32 - 3) * t * t);
    let low = front.saturating_sub(1);
    let high = (back + 1).min(xs.len() - 1);
    let mut i = low + m;
    while i + m <= high {
        let mut d = if i >= front && i <= back { s[0] * xs[i].area } else { 0.0 };
        for k in 1..=m {
            let b = if i + k <= back { xs[i + k].area } else { 0.0 };
            let f = if i >= k && i - k >= front { xs[i - k].area } else { 0.0 };
            d += s[k] * (f + b);
        }
        xs[i].second_deriv = d * denom;
        i += 1;
    }
    for i in front..(low + m).min(back + 1) {
        let mut d = xs[i].area;
        if i + 2 <= back {
            d += xs[i + 2].area;
        }
        if i < back {
            d -= 2.0 * xs[i + 1].area;
        }
        xs[i].second_deriv = d / (t * t);
    }
    for i in (high + 1).saturating_sub(m).max(front)..=back {
        let mut d = xs[i].area;
        if i >= front + 2 {
            d += xs[i - 2].area;
        }
        if i > front {
            d -= 2.0 * xs[i - 1].area;
        }
        xs[i].second_deriv = d / (t * t);
    }
}

/// Linearised supersonic pressure at Mach 1.2 marching forward and backward (NASA TR R-213 App. A).
fn sonic_pressure(xs: &mut [Xsec], front: usize, back: usize, t: f64) {
    let mach = 1.2;
    let beta = (mach * mach - 1.0f64).sqrt();
    let cp90 = max_cp(mach);
    let sq = |n: i64| (n as f64).sqrt();
    let finish = |xs: &[Xsec], i: usize, cp: f64, sign: f64| -> f64 {
        let mut cp = -2.0 * adjust_vel_finite_pressure(cp * -0.5);
        if cp < 0.0 {
            let d = if i == front {
                xs[i].area - xs[i + 1].area
            } else if i == back {
                xs[i - 1].area - xs[i].area
            } else {
                (xs[i - 1].area - xs[i + 1].area) * 0.5
            } / (sign * t);
            cp = adjust_cp_nonlinear(cp, xs[i].area, d, beta, mach);
        }
        cp.min(cp90)
    };
    let scale = |area: f64, cp: f64| {
        let cp = cp * (0.5 * t / (beta * (PI * area).sqrt())).sqrt();
        if cp > 2.0 || !cp.is_finite() {
            2.0
        } else {
            cp
        }
    };
    for i in front..=back {
        let mut cp = 0.0;
        let mut t1 = sq(i as i64 - front as i64 + 2);
        for j in (front as i64 - 1)..=(i as i64) {
            let t2 = sq(i as i64 - j);
            let w = t1 - t2;
            t1 = t2;
            if j >= 0 {
                cp += w * xs[j as usize].second_deriv;
            }
        }
        let cp = scale(xs[i].area, cp);
        xs[i].cp_sonic_fwd = finish(xs, i, cp, 1.0);
    }
    for i in (front..=back).rev() {
        let mut cp = 0.0;
        let mut t1 = sq(back as i64 + 2 - i as i64);
        for j in (i..=back + 1).rev() {
            let t2 = sq(j as i64 - i as i64);
            let w = t1 - t2;
            t1 = t2;
            if j < xs.len() {
                cp += w * xs[j].second_deriv;
            }
        }
        let cp = scale(xs[i].area, cp);
        xs[i].cp_sonic_back = finish(xs, i, cp, -1.0);
    }
}

fn adjust_vel_finite_pressure(vel: f64) -> f64 {
    if vel > 0.0 {
        return vel;
    }
    let n = (1.0 - vel).powi(2);
    1.0 - 2.0 * n / (1.0 + n)
}

fn max_cp(mach: f64) -> f64 {
    let m2 = mach * mach;
    ((6.0 / (7.0 * m2 - 1.0)).powf(2.5) * (1.2 * m2).powf(3.5) - 1.0) / (0.7 * m2)
}

fn adjust_cp_nonlinear(cp: f64, area: f64, d_area: f64, beta: f64, mach: f64) -> f64 {
    let nu_free = pm_angle(mach, mach);
    let turn = (d_area / (2.0 * (area * PI).sqrt())).atan();
    let u_i = cp * -0.5;
    let u_o = adjust_vel_finite_pressure(-turn / beta);
    let nu_i = pm_angle(mach * (1.0 + u_i), mach);
    let nu_o = pm_angle(mach * (1.0 + u_o), mach);
    cp_pm_expansion(mach, nu_free, turn + nu_o - nu_i)
}

fn cp_pm_expansion(mach: f64, nu_free: f64, deflection: f64) -> f64 {
    let mut nu = nu_free - deflection;
    let cap = 180.0 / (130.45 * PI);
    if nu > cap {
        nu = cap;
    }
    let eff = (nu / 130.45 * 180.0 / PI).max(0.0);
    let exp = -0.42 * eff.sqrt() + 0.313 * eff + 0.56;
    let eff_mach = 1.0 / (1.0 - eff.powf(exp));
    (stag_pres_ratio(eff_mach, nu) / stag_pres_ratio(mach, nu_free) - 1.0) / (0.7 * mach * mach)
}

fn stag_pres_ratio(mach: f64, nu: f64) -> f64 {
    let r = (nu + (1.0 / mach).acos()) * 2.0 / 6f64.sqrt();
    ((1.0 + r.cos()) / 2.4).powf(3.5)
}

fn pm_angle(local: f64, free: f64) -> f64 {
    if local < 1.0 {
        let free_nu = pm_angle(free, free);
        (1.0 - local).powi(2) * (free_nu - PI * 0.5)
    } else {
        ((local * local - 1.0) / 6.0).sqrt().atan() * 6f64.sqrt() - (1.0 / local).acos()
    }
}

fn hypersonic_drag(low: f64, high: f64, t: f64) -> f64 {
    if low >= high {
        return 0.0;
    }
    let r1 = (low / PI).sqrt();
    let r2 = (high / PI).sqrt();
    let d = r2 - r1;
    -(2.0 * PI / (t * t + d * d)) * d * d * d * (r1 + r2)
}

fn hypersonic_moment(low: f64, high: f64, t: f64) -> f64 {
    if low >= high {
        return 0.0;
    }
    let r1 = (low / PI).sqrt();
    let r2 = (high / PI).sqrt();
    let d2 = (r2 - r1).powi(2);
    let m = (r2 * r2 + r1 * r1 + t * t * 0.5) * 2.0 * PI * d2 / (t * t + d2);
    -m * t
}

fn critical_mach_number(f: f64) -> f64 {
    if f > 10.0 {
        0.925
    } else if f > 6.0 {
        0.00625 * f + 0.8625
    } else if f > 4.0 {
        0.025 * f + 0.75
    } else if f > 3.0 {
        0.07 * f + 0.57
    } else if f > 1.5 {
        0.33 * f - 0.21
    } else {
        0.285
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A long round-ish body flying straight has drag but no lift; at AoA it lifts up.
    #[test]
    fn slender_body_forces() {
        let mut cells = Vec::new();
        for z in 0..80 {
            // Ogive-ish radius in voxels.
            let r = (4.0 * (1.0 - ((z as f64 - 40.0) / 40.0).powi(2))).sqrt() * 2.0;
            for x in -6..=6 {
                for y in -6..=6 {
                    if ((x * x + y * y) as f64) <= r * r {
                        cells.push(([x, y, z], 1.0, 0));
                    }
                }
            }
        }
        let g = Grid::from_cells(&cells);
        let b = BodyAero::build(&g, &[]);
        assert!(b.length > 19.0 && b.length < 21.0, "len {}", b.length);
        assert!(b.critical_mach > 0.5 && b.critical_mach < 1.0);
        assert!(b.sonic_drag_area > 0.0, "{}", b.sonic_drag_area);
        let flow = Flow { mach: 0.3, reynolds_per_m: 7e6, pseudo_knudsen: 0.0, cf: 0.003 };
        let com = v3(10.0, 0.0, 0.0);
        let (f0, _) = b.forces(&flow, v3(-100.0, 0.0, 0.0), V3::ZERO, com, 1.2);
        assert!(f0.x < 0.0 && f0.z.abs() < 1e-6 * f0.x.abs() + 1e-9, "{f0:?}");
        // 10° nose up: air comes from ahead and below (body frame z down => air vel has -z).
        let a = 10f64.to_radians();
        let (f, _) = b.forces(&flow, v3(-a.cos(), 0.0, -a.sin()) * 100.0, V3::ZERO, com, 1.2);
        assert!(f.z < 0.0, "lift should point up (-z): {f:?}");
        // Pitching rotation is damped.
        let (_, t) = b.forces(&flow, v3(-100.0, 0.0, 0.0), v3(0.0, 0.5, 0.0), com, 1.2);
        assert!(t.y < 0.0, "pitch damping {t:?}");
    }
}

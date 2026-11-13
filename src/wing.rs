//! Lifting panels: FAR's legacy wing model (`FARWingAerodynamicModel`,
//! `FARWingInteraction`, `FARControllableSurface`; GPLv3) on panels built from
//! voxels instead of KSP wing parts.
//!
//! A panel is a connected run of wing cells of one body: user-selected blocks,
//! vanilla wing parts, or one vanilla control surface / control-role sub-grid.

use std::collections::{HashMap, VecDeque};
use std::f64::consts::PI;

use crate::body::aero_pos;
use crate::math::{v3, V3};
use crate::voxel::{Grid, DIRS, EMPTY, INTERIOR, VOXEL_M};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub enum Role {
    #[default]
    None,
    Pitch,
    Roll,
    Yaw,
    Flap,
}

impl Role {
    pub const ALL: [Role; 5] = [Role::None, Role::Pitch, Role::Roll, Role::Yaw, Role::Flap];
    pub fn name(self) -> &'static str {
        match self {
            Role::None => "none",
            Role::Pitch => "pitch",
            Role::Roll => "roll",
            Role::Yaw => "yaw",
            Role::Flap => "flap",
        }
    }
    pub fn parse(s: &str) -> Option<Role> {
        Role::ALL.into_iter().find(|r| r.name() == s)
    }
}

/// Max control deflection (FAR `maxdeflect` / `maxdeflectFlap` defaults), radians.
pub const MAX_DEFLECT: f64 = 15.0 * PI / 180.0;

/// Section properties of a lifting surface. FAR has one built-in section (thin, symmetric);
/// these let a panel trade lift for drag like real aerofoils do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aerofoil {
    /// Stable id in the vehicle file's `aero` attribute.
    pub id: &'static str,
    pub name: &'static str,
    pub note: &'static str,
    /// Thickness / chord: profile drag form factor, drag-divergence Mach, wave drag.
    pub t_c: f64,
    /// Section Cl at zero angle of attack (camber); fades out by Mach 1.2.
    pub cl0: f64,
    /// Max section lift coefficient: sets the stall angle. FAR uses 1.6 for everything; a thin
    /// section reaches about 0.9 (NACA 64A006: 0.89 at Reynolds 5.8 million, NACA TN 1923).
    pub cl_max: f64,
    /// Korn technology factor: drag-divergence Mach = κ − t/c at zero lift (0.87 conventional, 0.95 supercritical).
    pub kappa: f64,
    /// Supersonic wave drag = k · (t/c)² / β: 16/3 biconvex, 4 double wedge, more for blunt noses.
    pub k_wave: f64,
    pub kind: FoilKind,
}

/// Which list a section is offered in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FoilKind {
    Wing,
    Fin,
}

impl Aerofoil {
    /// Raymer's form factor on skin friction (≈ FAR's ×1.1 for the default section).
    pub fn form_factor(&self) -> f64 {
        1.0 + 2.4 * self.t_c + 100.0 * self.t_c.powi(4)
    }
    /// Zero-lift angle offset (rad), thin-airfoil theory: Cl = 2π(α − α₀).
    pub fn camber_aoa(&self) -> f64 {
        self.cl0 / (2.0 * PI)
    }
}

/// Selectable sections. The first is FAR's: its drag rise at Mach 0.8 and supersonic wave drag
/// 0.009216/β are those of a ~4 % symmetric section.
pub const AEROFOILS: [Aerofoil; 14] = [
    Aerofoil { id: "far", name: "Default (thin, symmetric)", note: "The built-in shape: thin and symmetrical, no lift at zero angle. Low drag, but it stalls at a modest angle.", t_c: 0.0416, cl0: 0.0, cl_max: 0.9, kappa: 0.8416, k_wave: 16.0 / 3.0, kind: FoilKind::Wing },
    Aerofoil { id: "naca4415", name: "NACA 4415 (high lift, slow flight)", note: "Thick and strongly curved: lots of lift at low speed, but it stalls early and drags a lot when fast.", t_c: 0.15, cl0: 0.45, cl_max: 1.75, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Wing },
    Aerofoil { id: "naca2412", name: "NACA 2412 (light aircraft)", note: "Moderate curve and thickness: the usual light-aircraft wing, easy to fly.", t_c: 0.12, cl0: 0.24, cl_max: 1.6, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Wing },
    Aerofoil { id: "naca23015", name: "NACA 23015 (warbird)", note: "Old warbird wing: thick with little curve, so a high maximum lift without much lift at zero angle.", t_c: 0.15, cl0: 0.12, cl_max: 1.65, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Wing },
    Aerofoil { id: "naca0012", name: "NACA 0012 (symmetric, tails)", note: "No lift at zero angle and works the same upside down: tails, fins and aerobatic wings.", t_c: 0.12, cl0: 0.0, cl_max: 1.5, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Wing },
    Aerofoil { id: "naca64a010", name: "NACA 64A010 (fast jets)", note: "Thin and flat: keeps its drag rise later near the speed of sound, in exchange for less maximum lift.", t_c: 0.10, cl0: 0.0, cl_max: 1.3, kappa: 0.9, k_wave: 6.0, kind: FoilKind::Wing },
    Aerofoil { id: "supercritical", name: "Supercritical SC(2)-0712 (airliner)", note: "Flat on top, so a thick wing stays efficient near the speed of sound; poor above it.", t_c: 0.12, cl0: 0.5, cl_max: 1.7, kappa: 0.95, k_wave: 8.0, kind: FoilKind::Wing },
    Aerofoil { id: "slotted0012", name: "Slotted symmetric (very high angle)", note: "Symmetric, with a slot at the front edge that keeps it lifting to about twice the angle of a plain wing, at the cost of drag. For all-moving tails and flaps that swing far.", t_c: 0.14, cl0: 0.0, cl_max: 2.6, kappa: 0.86, k_wave: 9.0, kind: FoilKind::Wing },
    Aerofoil { id: "biconvex4", name: "Biconvex 4% (supersonic)", note: "Knife-edge shape: least drag above the speed of sound, but little lift and an early stall when slow.", t_c: 0.04, cl0: 0.0, cl_max: 0.9, kappa: 0.87, k_wave: 16.0 / 3.0, kind: FoilKind::Wing },
    // Missile fins: thin sharp-edged sections (modified double wedge or biconvex, t/c ≈ 3–5 %,
    // NSWC TR 81-156). Wave drag k: 4 diamond, 8 hexagonal with a flat middle half, more blunt.
    // High-angle: sharp edges on low aspect ratio fins keep lifting through a leading-edge vortex.
    Aerofoil { id: "fin_hex4", name: "Hexagonal 4% (all-round missile fin)", note: "The usual missile fin: a thin plate with sharpened front and back edges. Works well from slow to very fast.", t_c: 0.04, cl0: 0.0, cl_max: 1.2, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Fin },
    Aerofoil { id: "fin_diamond3", name: "Diamond 3% (long range, very fast)", note: "Very thin with knife edges front and back: the least drag at high speed, so the missile keeps its speed longer. Turns less hard when slow.", t_c: 0.03, cl0: 0.0, cl_max: 1.0, kappa: 0.88, k_wave: 4.0, kind: FoilKind::Fin },
    Aerofoil { id: "fin_dogfight", name: "Sharp high-angle (dogfight)", note: "Keeps lifting at steep angles, so the missile can pull very hard turns at short range. A little more drag.", t_c: 0.035, cl0: 0.0, cl_max: 1.8, kappa: 0.86, k_wave: 6.0, kind: FoilKind::Fin },
    Aerofoil { id: "fin_round10", name: "Rounded 10% (slow missiles, bombs)", note: "Thicker with a rounded front: smooth and forgiving below the speed of sound, but draggy above it. For anti-tank missiles, glide bombs and drones.", t_c: 0.10, cl0: 0.0, cl_max: 1.4, kappa: 0.87, k_wave: 8.0, kind: FoilKind::Fin },
    Aerofoil { id: "fin_flat", name: "Flat plate, square edges (rockets)", note: "A plain flat sheet with square edges: simple, but more drag at every speed and it stalls early. For unguided rockets and simple builds.", t_c: 0.05, cl0: 0.0, cl_max: 0.9, kappa: 0.80, k_wave: 12.0, kind: FoilKind::Fin },
];

/// Default section of missile fin panels.
pub const FIN_DEFAULT: usize = 9;

pub fn aerofoil_index(id: &str) -> Option<usize> {
    AEROFOILS.iter().position(|a| a.id == id)
}

#[derive(Clone, Debug)]
pub struct Panel {
    /// Grid cells of this panel.
    pub cells: Vec<usize>,
    pub role: Role,
    /// Unit normal, chord (towards the leading edge) and span (root to tip) axes, body frame.
    pub n: V3,
    pub u: V3,
    pub s: V3,
    /// Semispan (root to tip), MAC = area / b, taper, tan of mid-chord sweep (+ = tip aft).
    pub b: f64,
    pub mac: f64,
    pub area: f64,
    pub taper: f64,
    pub tan_sweep: f64,
    /// Area centroid, body frame.
    pub centroid: V3,
    /// FAR wing-group aspect ratio, from root/tip coverage.
    pub ar: f64,
    pub root_covered: f64,
    pub tip_covered: f64,
    /// Panel continuing this one past its tip (same wing, e.g. the outer panel of a cranked
    /// wing), with the number of tip-strip cells that run into it.
    pub tip_next: Option<(usize, f64)>,
    /// Neighbours ahead (+u) and behind (−u): (panel index, influence).
    pub fwd: Vec<(usize, f64)>,
    pub aft: Vec<(usize, f64)>,
    /// Biplane-style interference factor on Cl.
    pub interference: f64,
    /// +1/−1: deflection sign that makes a positive command give a positive moment/lift.
    pub ctrl_sign: f64,
    /// Section (index into `AEROFOILS`).
    pub foil: usize,
}

/// What makes a cell a wing cell, and the panel it groups into.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum Key {
    Wing(u32),
    /// A vanilla control surface component.
    Comp(u32),
    /// A sub-grid body with a control role.
    Body(u32),
    /// Wing blocks of this body marked as separate wing number .1 (their own panels and wing).
    Split(u32, u32),
}

/// Build panels. `key_of(component index)` says whether a component's cells are wing cells.
pub fn build_panels(grid: &Grid, key_of: &dyn Fn(u32) -> Option<(Key, Role)>, com: V3) -> Vec<Panel> {
    let n = grid.fill.len();
    let mut key: Vec<Option<(Key, Role)>> = vec![None; n];
    for i in 0..n {
        let o = grid.owner[i];
        if o != EMPTY && o != INTERIOR {
            key[i] = key_of(o);
        }
    }
    // Symmetry plane for splitting wings that run across the fuselage: the CoM's y.
    let mid_y = com.y;
    let mut panel_of = vec![usize::MAX; n];
    let mut panels = Vec::new();
    for start in 0..n {
        let Some((k, role)) = key[start] else {
            continue;
        };
        if panel_of[start] != usize::MAX {
            continue;
        }
        let mut cells = Vec::new();
        let mut queue = VecDeque::from([start]);
        panel_of[start] = panels.len();
        while let Some(i) = queue.pop_front() {
            cells.push(i);
            let c = grid.cell(i);
            for d in DIRS {
                let Some(j) = grid.index([c[0] + d[0], c[1] + d[1], c[2] + d[2]]) else {
                    continue;
                };
                if panel_of[j] == usize::MAX && key[j].map(|x| x.0) == Some(k) {
                    panel_of[j] = panels.len();
                    queue.push_back(j);
                }
            }
        }
        panels.push((k, role, cells));
    }
    // Split horizontal-ish wing panels that straddle the symmetry plane into left and right.
    let mut split = Vec::new();
    for (k, role, cells) in panels {
        let ys: Vec<f64> = cells.iter().map(|&i| aero_pos(grid.cell(i)).y - mid_y).collect();
        let left = ys.iter().filter(|y| **y < -0.01).count();
        let right = ys.iter().filter(|y| **y > 0.01).count();
        if matches!(k, Key::Wing(_) | Key::Split(..)) && left > 4 && right > 4 {
            let (l, r) = cells.iter().zip(&ys).fold((vec![], vec![]), |(mut l, mut r), (&c, &y)| {
                if y < 0.0 {
                    l.push(c)
                } else {
                    r.push(c)
                }
                (l, r)
            });
            split.push((k, role, l));
            split.push((k, role, r));
        } else {
            split.push((k, role, cells));
        }
    }
    let mut out: Vec<Panel> = Vec::new();
    let mut drops: Vec<(&'static str, usize)> = Vec::new();
    for (_, role, cells) in split {
        match panel_geometry(grid, cells, role, com) {
            Ok(p) => out.push(p),
            Err(why) => match drops.iter_mut().find(|d| d.0 == why) {
                Some(d) => d.1 += 1,
                None => drops.push((why, 1)),
            },
        }
    }
    #[cfg(windows)]
    if !drops.is_empty() {
        crate::logging::log_line(&format!(
            "panels: {} built, dropped {}",
            out.len(),
            drops.iter().map(|(w, n)| format!("{n} x {w}")).collect::<Vec<_>>().join(", ")
        ));
    }
    let mut owner = vec![usize::MAX; n];
    for (pi, p) in out.iter().enumerate() {
        for &c in &p.cells {
            owner[c] = pi;
        }
    }
    interactions(grid, &mut out, &owner);
    chain_aspect_ratios(&mut out);
    for p in &mut out {
        p.ctrl_sign = control_sign(p, com);
    }
    out
}

/// FAR wing-group aspect ratio, over whole wings: panels joined root-to-tip (when several
/// lead into one panel, the one touching it with the most cells continues it) share the span
/// and area of their chain, with the chain root's coverage (fuselage: a reflection plane,
/// AR doubles) and the chain tip's (end plate, +half).
fn chain_aspect_ratios(panels: &mut [Panel]) {
    let n = panels.len();
    let mut pred: Vec<Option<(usize, f64)>> = vec![None; n];
    for (i, p) in panels.iter().enumerate() {
        if let Some((j, share)) = p.tip_next.filter(|&(j, _)| j != i) {
            if pred[j].is_none_or(|(_, best)| share > best) {
                pred[j] = Some((i, share));
            }
        }
    }
    let next: Vec<Option<usize>> = (0..n).map(|i| panels[i].tip_next.map(|(j, _)| j).filter(|&j| pred[j].map(|(k, _)| k) == Some(i))).collect();
    let link = |i: usize| next[i];
    let mut has_prev = vec![false; n];
    for i in 0..n {
        if let Some(j) = link(i) {
            has_prev[j] = true;
        }
    }
    let mut done = vec![false; n];
    // Chain starts first, then whatever is left (cycles) as single panels.
    let starts: Vec<usize> = (0..n).filter(|&i| !has_prev[i]).chain(0..n).collect();
    for start in starts {
        if done[start] {
            continue;
        }
        let mut chain = vec![start];
        done[start] = true;
        while let Some(j) = link(*chain.last().unwrap()).filter(|&j| !done[j]) {
            done[j] = true;
            chain.push(j);
        }
        let b: f64 = chain.iter().map(|&i| panels[i].b).sum();
        let area: f64 = chain.iter().map(|&i| panels[i].area).sum();
        let root = panels[chain[0]].root_covered;
        let tip = panels[*chain.last().unwrap()].tip_covered;
        let ar = (b * b / area * (1.0 + root + 0.5 * tip)).clamp(0.25, 30.0);
        for &i in &chain {
            panels[i].ar = ar;
        }
    }
}

/// Symmetric 3x3 eigen-decomposition (Jacobi). Returns (values, column vectors).
pub(crate) fn eigen3(mut a: [[f64; 3]; 3]) -> ([f64; 3], [V3; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        let (mut p, mut q, mut max) = (0, 1, 0.0);
        for i in 0..3 {
            for j in i + 1..3 {
                if a[i][j].abs() > max {
                    max = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if max < 1e-15 {
            break;
        }
        let theta = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
        let (s, c) = theta.sin_cos();
        for k in 0..3 {
            let (akp, akq) = (a[k][p], a[k][q]);
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for k in 0..3 {
            let (apk, aqk) = (a[p][k], a[q][k]);
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
        for k in 0..3 {
            let (vkp, vkq) = (v[k][p], v[k][q]);
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let col = |j: usize| v3(v[0][j], v[1][j], v[2][j]);
    ([a[0][0], a[1][1], a[2][2]], [col(0), col(1), col(2)])
}

/// Principal axes of a point set: (axes sorted thinnest first, mean); None for under 3 points.
fn principal(pts: &[V3]) -> Option<([V3; 3], V3)> {
    if pts.len() < 3 {
        return None;
    }
    let mean = pts.iter().fold(V3::ZERO, |a, p| a + *p) * (1.0 / pts.len() as f64);
    let mut cov = [[0.0; 3]; 3];
    for p in pts {
        let d = *p - mean;
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += d.get(i) * d.get(j);
            }
        }
    }
    let (vals, vecs) = eigen3(cov);
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| vals[a].total_cmp(&vals[b]));
    Some((order.map(|k| vecs[k]), mean))
}

/// Extents of a point set along its principal axes, thinnest first.
pub fn extents(pts: &[V3]) -> Option<[f64; 3]> {
    let (axes, _) = principal(pts)?;
    Some(axes.map(|a| {
        let (lo, hi) = pts.iter().fold((f64::MAX, f64::MIN), |(l, h), p| (l.min(p.dot(a)), h.max(p.dot(a))));
        hi - lo
    }))
}

/// The plane between a thick fin's two sides: the area-weighted normal of the triangles facing
/// about `n0`, each turned to agree with it. A wedge fin (thick root, blunt trailing edge) tilts a
/// vertex fit by a fraction of a degree, a cant that rolls a missile with four of them.
pub fn side_normal(tris: &[[V3; 3]], n0: V3) -> Option<V3> {
    let sum = tris.iter().fold(V3::ZERO, |acc, t| {
        let c = (t[1] - t[0]).cross(t[2] - t[0]);
        let len = c.len();
        if len < 1e-12 || (c.dot(n0) / len).abs() < 0.95 {
            return acc;
        }
        acc + if c.dot(n0) < 0.0 { -c } else { c }
    });
    (sum.len() > 1e-12).then(|| sum.norm())
}

/// Lifting panel straight from a flat mesh piece (a missile fin, aero frame metres): plane fit
/// to its vertices (or to its sides, `tris`, when it has them), planform from their outline.
/// For fins far smaller than a voxel.
pub fn plate_panel(pts: &[V3], tris: &[[V3; 3]], cells: Vec<usize>, role: Role, com: V3) -> Option<Panel> {
    let (axes, mean) = principal(pts)?;
    let n = side_normal(tris, axes[0]).filter(|s| s.dot(axes[0]) > 0.98).unwrap_or(axes[0]);
    let fwd = v3(1.0, 0.0, 0.0);
    if n.x.abs() > 0.9 {
        return None; // faces the flow: no chord
    }
    let u = fwd.reject(n).norm();
    let mut s = n.cross(u).norm();
    let range = |s: V3| pts.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.dot(s)), b.max(p.dot(s))));
    let (lo, hi) = range(s);
    let b = hi - lo;
    if b < 1e-3 {
        return None;
    }
    // Span runs root to tip: the root is the end nearer the CoM line (the missile body).
    let off = |end: f64| {
        let sel: Vec<f64> = pts.iter().filter(|p| (p.dot(s) - end).abs() < 0.1 * b).map(|p| (*p - com).reject(fwd).len()).collect();
        sel.iter().sum::<f64>() / sel.len().max(1) as f64
    };
    if off(hi) < off(lo) {
        s = -s;
    }
    let (lo, _) = range(s);
    let outline = hull_2d(pts.iter().map(|p| [p.dot(s) - lo, p.dot(u)]).collect());
    // Spanwise strips across the outline: chord and mid-chord, least squares against span.
    const N: usize = 16;
    let ds = b / N as f64;
    let nmean = mean.dot(n);
    let (mut area, mut centroid) = (0.0, V3::ZERO);
    let (mut sx, mut sxx, mut sc, mut sxc, mut sm, mut sxm, mut cnt) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for k in 0..N {
        let x = (k as f64 + 0.5) * ds;
        let Some((cl, ch)) = slice_2d(&outline, x) else {
            continue;
        };
        let (chord, mid) = (ch - cl, 0.5 * (cl + ch));
        area += chord * ds;
        centroid += (u * mid + s * (x + lo) + n * nmean) * (chord * ds);
        sx += x;
        sxx += x * x;
        sc += chord;
        sxc += x * chord;
        sm += mid;
        sxm += x * mid;
        cnt += 1.0;
    }
    if area < 1e-6 || cnt < 2.0 {
        return None;
    }
    let den = cnt * sxx - sx * sx;
    let (chord_slope, mid_slope) = if den.abs() > 1e-12 { ((cnt * sxc - sx * sc) / den, (cnt * sxm - sx * sm) / den) } else { (0.0, 0.0) };
    let (mean_x, mean_c) = (sx / cnt, sc / cnt);
    let root_c = (mean_c - chord_slope * mean_x).max(1e-3);
    let tip_c = (mean_c + chord_slope * (b - mean_x)).max(0.0);
    let mut p = Panel {
        cells,
        role,
        n,
        u,
        s,
        b,
        mac: area / b,
        area,
        taper: (tip_c / root_c).clamp(0.0, 2.0),
        tan_sweep: -mid_slope,
        centroid: centroid * (1.0 / area),
        // Root on the body, which mirrors the fin (FAR: a root against a fuselage doubles AR).
        ar: (2.0 * b * b / area).clamp(0.25, 30.0),
        root_covered: 1.0,
        tip_covered: 0.0,
        tip_next: None,
        fwd: Vec::new(),
        aft: Vec::new(),
        interference: 1.0,
        ctrl_sign: 1.0,
        foil: 0,
    };
    p.ctrl_sign = control_sign(&p, com);
    Some(p)
}

/// Convex hull (counter-clockwise, monotone chain).
fn hull_2d(mut p: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    p.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut h: Vec<[f64; 2]> = Vec::new();
    for pass in 0..2 {
        let start = h.len();
        let it: Box<dyn Iterator<Item = &[f64; 2]>> = if pass == 0 { Box::new(p.iter()) } else { Box::new(p.iter().rev()) };
        for &q in it {
            while h.len() >= start + 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 {
                h.pop();
            }
            h.push(q);
        }
        h.pop();
    }
    h
}

/// Extent in y of a convex polygon along the line x = `x`.
fn slice_2d(poly: &[[f64; 2]], x: f64) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        if (a[0] - x) * (b[0] - x) <= 0.0 && a[0] != b[0] {
            let y = a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]);
            lo = lo.min(y);
            hi = hi.max(y);
        }
    }
    (hi > lo).then_some((lo, hi))
}

fn panel_geometry(grid: &Grid, cells: Vec<usize>, role: Role, com: V3) -> Result<Panel, &'static str> {
    if cells.len() < 2 {
        return Err("single-cell group");
    }
    let pts: Vec<V3> = cells.iter().map(|&i| aero_pos(grid.cell(i))).collect();
    let w: Vec<f64> = cells.iter().map(|&i| grid.fill[i] as f64).collect();
    let wsum: f64 = w.iter().sum();
    let mean = pts.iter().zip(&w).fold(V3::ZERO, |a, (p, w)| a + *p * *w) * (1.0 / wsum);
    let own = VOXEL_M * VOXEL_M / 12.0;
    let mut cov = [[0.0; 3]; 3];
    for (p, wi) in pts.iter().zip(&w) {
        let d = *p - mean;
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += wi * (d.get(i) * d.get(j) + if i == j { own } else { 0.0 });
            }
        }
    }
    let (vals, vecs) = eigen3(cov);
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| vals[a].total_cmp(&vals[b]));
    let mut n = vecs[order[0]];
    let mut short = vecs[order[1]]; // shorter in-plane axis
                                    // A strip one voxel thick and one voxel wide is ambiguous: prefer a horizontal surface.
    if vals[order[1]] < vals[order[0]] * 1.1 && vecs[order[1]].z.abs() > n.z.abs() {
        (n, short) = (vecs[order[1]], vecs[order[0]]);
    }
    let fwd = v3(1.0, 0.0, 0.0);
    // A plate facing the flow (Starship-style drag flaps) has no chord from the airflow: the chord
    // runs along its shorter side, the span along its longer one.
    let u = if n.x.abs() > 0.9 { short.reject(n).norm() } else { fwd.reject(n).norm() };
    let mut s = n.cross(u).norm();
    // Span points away from the root: the end nearer the CoM line is the root.
    let off = |p: V3| (p - com).reject(fwd).len();
    let (smin, smax) = pts.iter().fold((f64::MAX, f64::MIN), |(a, b), p| (a.min(p.dot(s)), b.max(p.dot(s))));
    let near_end = |end: f64| {
        let sel: Vec<&V3> = pts.iter().filter(|p| (p.dot(s) - end).abs() < 0.13).collect();
        sel.iter().map(|p| off(**p)).sum::<f64>() / sel.len().max(1) as f64
    };
    if near_end(smax) < near_end(smin) {
        s = -s;
    }
    // Spanwise strips: chord extent per strip of cells.
    let mut strips: HashMap<i64, (f64, f64, f64)> = HashMap::new(); // s index -> (umin, umax, fill)
    for (p, wi) in pts.iter().zip(&w) {
        let k = (p.dot(s) / VOXEL_M).round() as i64;
        let e = strips.entry(k).or_insert((f64::MAX, f64::MIN, 0.0));
        e.0 = e.0.min(p.dot(u));
        e.1 = e.1.max(p.dot(u));
        e.2 += wi;
    }
    let mut keys: Vec<i64> = strips.keys().copied().collect();
    keys.sort();
    let s0 = keys[0];
    let b = (keys[keys.len() - 1] - s0 + 1) as f64 * VOXEL_M;
    let mut area = 0.0;
    let mut centroid = V3::ZERO;
    // Least squares of chord and mid-chord against span position.
    let (mut sx, mut sxx, mut sc, mut sxc, mut sm, mut sxm, mut cnt) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    let nmean = mean.dot(n);
    for k in &keys {
        let (umin, umax, _) = strips[k];
        let chord = umax - umin + VOXEL_M;
        let mid = 0.5 * (umin + umax);
        let x = (*k - s0) as f64 * VOXEL_M;
        area += chord * VOXEL_M;
        centroid += (u * mid + s * (*k as f64 * VOXEL_M) + n * nmean) * (chord * VOXEL_M);
        sx += x;
        sxx += x * x;
        sc += chord;
        sxc += x * chord;
        sm += mid;
        sxm += x * mid;
        cnt += 1.0;
    }
    centroid = centroid * (1.0 / area);
    let den = cnt * sxx - sx * sx;
    let (chord_slope, mid_slope) = if den.abs() > 1e-9 { ((cnt * sxc - sx * sc) / den, (cnt * sxm - sx * sm) / den) } else { (0.0, 0.0) };
    let mean_x = sx / cnt;
    let mean_c = sc / cnt;
    let root_c = (mean_c - chord_slope * mean_x).max(VOXEL_M * 0.5);
    let tip_c = (mean_c + chord_slope * (b - VOXEL_M - mean_x)).max(VOXEL_M * 0.5);
    Ok(Panel {
        cells,
        role,
        n,
        u,
        s,
        b,
        mac: area / b,
        area,
        taper: (tip_c / root_c).clamp(0.0, 2.0),
        tan_sweep: -mid_slope,
        centroid,
        ar: 1.0,
        root_covered: 0.0,
        tip_covered: 0.0,
        tip_next: None,
        fwd: Vec::new(),
        aft: Vec::new(),
        interference: 1.0,
        ctrl_sign: 1.0,
        foil: 0,
    })
}

/// Grid direction closest to a body-frame unit vector.
fn grid_dir(v: V3) -> [i32; 3] {
    // body (x fwd, y right, z down) -> grid (X right, Y up, Z fwd)
    let g = [v.y, -v.z, v.x];
    let a = (0..3).max_by(|&i, &j| g[i].abs().total_cmp(&g[j].abs())).unwrap();
    let mut d = [0; 3];
    d[a] = if g[a] > 0.0 { 1 } else { -1 };
    d
}

/// Root/tip coverage, fore/aft neighbours and stacked-wing interference.
fn interactions(grid: &Grid, panels: &mut [Panel], owner: &[usize]) {
    for pi in 0..panels.len() {
        let p = panels[pi].clone();
        let in_p = |i: usize| owner[i] == pi;
        let solid_other = |i: usize| grid.owner[i] != EMPTY && !in_p(i);
        // Root / tip: share of the end strip's cells with something solid just beyond the end.
        let sd = grid_dir(p.s);
        let span_pos = |i: usize| aero_pos(grid.cell(i)).dot(p.s);
        let (lo, hi) = p.cells.iter().fold((f64::MAX, f64::MIN), |(a, b), &i| (a.min(span_pos(i)), b.max(span_pos(i))));
        let cover = |end: f64, dir: [i32; 3]| {
            let ends: Vec<usize> = p.cells.iter().copied().filter(|&i| (span_pos(i) - end).abs() < 0.13).collect();
            let hit = ends
                .iter()
                .filter(|&&i| {
                    let c = grid.cell(i);
                    grid.index([c[0] + dir[0], c[1] + dir[1], c[2] + dir[2]]).is_some_and(solid_other)
                })
                .count();
            hit as f64 / ends.len().max(1) as f64
        };
        panels[pi].root_covered = cover(lo, sd.map(|x| -x));
        panels[pi].tip_covered = cover(hi, sd);
        // The panel most of the tip strip runs into, if it is roughly coplanar (dihedral and
        // cranks up to ~30°; a winglet or fin is an end plate, not more span).
        let mut next: HashMap<usize, usize> = HashMap::new();
        let tips: Vec<usize> = p.cells.iter().copied().filter(|&i| (span_pos(i) - hi).abs() < 0.13).collect();
        for &i in &tips {
            let c = grid.cell(i);
            if let Some(j) = grid.index([c[0] + sd[0], c[1] + sd[1], c[2] + sd[2]]) {
                if owner[j] != usize::MAX && owner[j] != pi {
                    *next.entry(owner[j]).or_default() += 1;
                }
            }
        }
        panels[pi].tip_next = next
            .into_iter()
            .filter(|&(j, n)| 4 * n >= tips.len().max(1) && panels[j].n.dot(p.n).abs() > 0.85)
            .max_by_key(|&(_, n)| n)
            .map(|(j, n)| (j, n as f64));

        // Chord rays from 5 span stations, one MAC long, ahead and behind.
        let ud = grid_dir(p.u);
        let steps = (p.mac / VOXEL_M).ceil().max(1.0) as i32;
        for (dir, aft) in [(ud, false), (ud.map(|x| -x), true)] {
            let mut infl: HashMap<usize, f64> = HashMap::new();
            for k in 0..5 {
                let target = lo + (0.2 * k as f64 + 0.1) * (hi - lo + VOXEL_M) - VOXEL_M * 0.5;
                // Start from the panel cell of this strip furthest along the ray direction.
                let Some(&start) = p.cells.iter().filter(|&&i| (span_pos(i) - target).abs() < 0.13).max_by_key(|&&i| {
                    let c = grid.cell(i);
                    c[0] * dir[0] + c[1] * dir[1] + c[2] * dir[2]
                }) else {
                    continue;
                };
                let c = grid.cell(start);
                for t in 1..=steps {
                    let Some(j) = grid.index([c[0] + dir[0] * t, c[1] + dir[1] * t, c[2] + dir[2] * t]) else {
                        break;
                    };
                    if !solid_other(j) {
                        continue;
                    }
                    if owner[j] != usize::MAX {
                        *infl.entry(owner[j]).or_default() += 0.2 * p.n.dot(panels[owner[j]].n).abs();
                    }
                    break;
                }
            }
            let list: Vec<(usize, f64)> = infl.into_iter().collect();
            if aft {
                panels[pi].aft = list;
            } else {
                panels[pi].fwd = list;
            }
        }

        // Interference: nearest other panel along ±normal within one semispan.
        let nd = grid_dir(p.n);
        let c = grid.cell(p.cells[p.cells.len() / 2]);
        let mut f = [1.0; 2];
        for (side, sign) in [1, -1].into_iter().enumerate() {
            let max = (p.b / VOXEL_M).ceil() as i32;
            for t in 1..=max {
                let Some(j) = grid.index([c[0] + nd[0] * t * sign, c[1] + nd[1] * t * sign, c[2] + nd[2] * t * sign]) else {
                    break;
                };
                if owner[j] != usize::MAX && owner[j] != pi {
                    let k = (t as f64 * VOXEL_M / p.b).clamp(0.0, 1.0);
                    f[side] = 1.0 - (1.0 - k) * p.n.dot(panels[owner[j]].n).abs();
                    break;
                }
            }
        }
        panels[pi].interference = 0.5 * (f[0] + f[1]);
    }
}

/// Sign of deflection that makes a +1 command produce the role's positive response.
/// A +θ deflection tilts the normal toward the leading edge, i.e. moves the trailing
/// edge to +n, so the extra force points along −n.
fn control_sign(p: &Panel, com: V3) -> f64 {
    let df = -p.n;
    let m = (p.centroid - com).cross(df);
    let v = match p.role {
        Role::None => return 1.0,
        Role::Pitch => m.y,  // nose up
        Role::Roll => m.x,   // right wing down
        Role::Yaw => m.z,    // nose right
        Role::Flap => -df.z, // more lift (up = −z)
    };
    if v < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Per-evaluation inputs.
pub struct WingFlow {
    pub mach: f64,
    pub rho: f64,
    /// Skin friction per panel is from its own MAC Reynolds number: Re = re_per_m · MAC.
    pub re_per_m: f64,
    pub gamma: f64,
}

/// Per-panel results of the first (raw) pass, read by neighbours.
#[derive(Clone, Copy, Default)]
struct Raw {
    aoa: f64,
    lift_slope: f64,
    aoa_max: f64,
    cos_sweep: f64,
    cd0_inc: f64,
    stall: f64,
}

pub struct PanelForce {
    pub force: V3,
    pub at: V3,
    pub cl: f64,
    pub cd: f64,
    pub cd_induced: f64,
}

/// Forces on every panel. `vel(panel, at)`: velocity of a point of that panel through the air.
/// `deflect[i]`: control deflection of panel i in radians. `stall`: persistent stall state.
pub fn panel_forces(panels: &[Panel], flow: &WingFlow, vel: &dyn Fn(usize, V3) -> V3, deflect: &[f64], stall: &mut [f64]) -> Vec<PanelForce> {
    let mut raw = vec![Raw::default(); panels.len()];
    let mut normals = vec![V3::ZERO; panels.len()];
    for (i, p) in panels.iter().enumerate() {
        let th = deflect[i];
        normals[i] = (p.n * th.cos() + p.u * th.sin()).norm();
        let v = vel(i, p.centroid);
        let vn = v.norm();
        let pl = match v.reject(normals[i]).norm() {
            V3::ZERO => p.u,
            d => d,
        };
        let (ls, cos_sweep) = subsonic_lift_slope(p, flow.mach, pl);
        let af = &AEROFOILS[p.foil.min(AEROFOILS.len() - 1)];
        raw[i] = Raw {
            // Camber acts as built-in incidence toward body-up (none on fins: no "up" side).
            aoa: normals[i].dot(vn).clamp(-1.0, 1.0).asin() - camber_sign(normals[i]) * af.camber_aoa() * camber_fade(flow.mach),
            lift_slope: ls,
            aoa_max: aoa_max(flow.mach, ls, af.cl_max),
            cos_sweep,
            cd0_inc: cd_compressibility(flow.mach, cos_sweep, af),
            stall: stall[i],
        };
    }
    // Flap camber (FAR's thin-airfoil flap term, computed on the trailing panel from its upstream
    // wing): the share over the wing's chord acts on the wing ahead, not on the flap. Kept on the
    // flap, the whole section's extra lift sat at the trailing edge: a big nose-down moment the
    // tail had to fight (flaps felt useless) and induced drag on a small, overloaded panel.
    let mach_coeff = (1.0 - flow.mach * flow.mach).clamp(0.0, 1.0);
    let mut flap_own = vec![0.0; panels.len()];
    let mut wing_add = vec![0.0; panels.len()]; // Cl·area handed to a wing by the flaps behind it
    for (i, p) in panels.iter().enumerate() {
        let v = vel(i, p.centroid);
        let pl = v.reject(normals[i]).norm();
        if pl == V3::ZERO || mach_coeff <= 0.0 {
            continue;
        }
        let fwd_w = pl.dot(p.u);
        let list = if fwd_w > 0.0 { &p.fwd } else { &p.aft };
        let wdir = fwd_w * fwd_w;
        let (mut up_mac, mut up_b, mut up_area, mut up_ls, mut up_infl, mut up_aoa) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for &(j, infl) in list {
            let f = infl * wdir;
            let o = &panels[j];
            up_mac += o.mac * f;
            up_b += o.b * f;
            up_area += o.area * f;
            up_ls += raw[j].lift_slope * f;
            up_infl += f;
            up_aoa += (raw[i].aoa - raw[j].aoa * o.n.dot(p.n).signum()) * f;
        }
        if up_infl <= 0.0 || up_b <= 0.0 {
            continue;
        }
        let flap_ratio = (p.mac / (p.mac + up_mac)).clamp(0.0, 1.0);
        let th = (2.0 * flap_ratio - 1.0).clamp(-1.0, 1.0).acos();
        let flap_factor = 1.0 - (th - th.sin()) / PI;
        let wing_frac = (p.b / up_b).clamp(0.0, 1.0);
        let flap_frac = (up_b / p.b).clamp(0.0, 1.0);
        // Past ~45° a flap stops adding lift (separation off its upper surface).
        let defl = up_aoa.clamp(-up_infl * PI / 4.0, up_infl * PI / 4.0);
        // shortcut: added to Cl directly; FAR multiplies it by |sin α| again, which makes small
        // flap deflections do almost nothing.
        let on_flap = flap_factor * up_ls * defl * p.area * flap_frac - flap_frac * up_ls * defl * p.area;
        let on_wing = flap_factor * up_ls * defl * up_area * wing_frac;
        flap_own[i] = on_flap / p.area * mach_coeff;
        for &(j, infl) in list {
            // In the wing's own AoA sense (normals may point opposite ways).
            wing_add[j] += on_wing * mach_coeff * (infl * wdir / up_infl) * panels[j].n.dot(p.n).signum();
        }
    }

    let mut out = Vec::with_capacity(panels.len());
    for (i, p) in panels.iter().enumerate() {
        let n = normals[i];
        let v = vel(i, p.centroid);
        let speed = v.len();
        if speed < 0.1 {
            stall[i] = 0.0;
            out.push(PanelForce { force: V3::ZERO, at: p.centroid, cl: 0.0, cd: 0.0, cd_induced: 0.0 });
            continue;
        }
        let vn = v * (1.0 / speed);
        // Flow straight onto the face has no in-plane direction: any one gives the same pure drag.
        let pl = match v.reject(n).norm() {
            V3::ZERO => p.u,
            d => d,
        };
        let perp = n.cross(pl).norm();
        let lift_dir = perp.cross(v).norm();
        let r = raw[i];
        let alpha = r.aoa;
        let mach = flow.mach;

        // Upstream neighbours: those ahead along the in-plane flow.
        let fwd_w = pl.dot(p.u);
        let list = if fwd_w > 0.0 { &p.fwd } else { &p.aft };
        let wdir = fwd_w * fwd_w;
        let (mut up_ls, mut up_stall, mut up_cos, mut up_amax, mut up_cd0, mut up_infl, mut up_aoa) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for &(j, infl) in list {
            let f = infl * wdir;
            let o = &panels[j];
            let rj = raw[j];
            up_ls += rj.lift_slope * f;
            up_stall += rj.stall * f;
            up_cos += rj.cos_sweep * f;
            up_amax += rj.aoa_max * f;
            up_cd0 += rj.cd0_inc * f;
            up_infl += f;
            up_aoa += (alpha - rj.aoa * o.n.dot(p.n).signum()) * f;
        }
        let mut lift_slope = r.lift_slope;
        let mut cos_sweep = r.cos_sweep;
        let mut aoa_max = r.aoa_max;
        // Flap camber: this panel's own share as a flap, plus what flaps behind it hand it.
        let cl_inc = flap_own[i] + wing_add[i] / p.area.max(1e-9);
        if up_infl > 0.0 {
            aoa_max += up_amax;
            lift_slope = lift_slope * (1.0 - up_infl) + up_ls;
            cos_sweep = (cos_sweep * (1.0 - up_infl) + up_cos).clamp(0.0, 1.0);
        }

        // Stall with hysteresis. A trailing surface (flap) works in its upstream wing's flow: its stall
        // angle is measured from that flow, not from its own deflection (else deflected flaps stall).
        let last = stall[i];
        let a = (alpha - up_aoa).abs();
        let mut st = if a > aoa_max {
            ((a - aoa_max) * 10.0).clamp(0.0, 1.0).max(last)
        } else if a < aoa_max {
            (1.0 - ((aoa_max - a) * 25.0).clamp(0.0, 1.0)).min(last)
        } else {
            last
        };
        st = (st + up_stall).clamp(0.0, 1.0);
        if st < 1e-5 {
            st = 0.0;
        }
        stall[i] = st;

        // Skin friction from this panel's Reynolds number (×1.1 for thickness, FAR).
        let af = &AEROFOILS[p.foil.min(AEROFOILS.len() - 1)];
        let cf = skin_friction(flow.re_per_m * p.mac, mach) * af.form_factor();
        let beta = {
            let b = (mach * mach - 1.0).sqrt();
            if b.is_nan() || b < 0.663_324_958_07 {
                0.663_324_958_07
            } else {
                b
            }
        };
        let tan_sweep = (1.0 - cos_sweep * cos_sweep).clamp(0.0, 1.0).sqrt() / cos_sweep.max(1e-6);
        let cd0_own = cd_compressibility(mach, cos_sweep, af);
        let cd0 = if up_infl > 0.99 { up_cd0 } else { cd0_own * (1.0 - up_infl.min(1.0)) + up_cd0 } + 2.0 * cf;
        let cd_max = flat_plate_cd(mach, p.ar);
        let e = oswald(p.ar, cos_sweep, cd0, p.taper);
        let pi_ar_e = p.ar * e * PI;
        let cos_a = alpha.cos();
        let sin_a = (1.0 - cos_a * cos_a).clamp(0.0, 1.0).sqrt();
        let sgn = alpha.signum();

        let lift_slope = lift_slope * rounded(a, aoa_max);
        let (mut cl, mut cd, cdi);
        if mach <= 0.8 {
            cl = lift_slope * cos_a * sgn * sin_a + cl_inc;
            cdi = cl * cl / pi_ar_e;
            cd = cdi + cd0;
        } else {
            // Supersonic normal force: Ackeret linear theory with FAR's leading-edge factor.
            // shortcut: FAR uses shock-expansion on a 0.05 rad diamond aerofoil; upgrade if
            // supersonic Stormworks builds need it.
            let sup = |m: f64| {
                let bt = (m * m - 1.0).max(0.44).sqrt();
                // Tip Mach cones of a finite wing (linear theory, rectangular: 1 − 1/(2βA) for βA ≥ 1),
                // by how much tip there is. shortcut: below βA 1 a straight run to the slender wing's πA/2.
                let ba = bt * p.ar;
                let tips = if ba >= 1.0 { 1.0 - 0.5 / ba } else { ba * (PI / 2.0 + (2.0 - PI / 2.0) * ba) / 4.0 };
                4.0 / bt * sin_a * le_factor(p.ar, bt, tan_sweep) * (1.0 - (1.0 - tips) * p.taper.clamp(0.0, 1.0))
            };
            if mach <= 1.4 {
                let sc = -(125.0 / 27.0) * (((2.0 * mach - 6.6) * mach + 6.72) * mach - 2.176);
                cl = lift_slope * cos_a * sin_a * sgn;
                if mach <= 1.0 {
                    cl += cl_inc;
                }
                cl *= 1.0 - sc;
                cl += cos_a * sgn * sup(mach.max(1.2)) * sc;
                let eff_beta = beta * sc + (1.0 - sc);
                cdi = eff_beta * cl * cl / pi_ar_e;
            } else {
                cl = sup(mach) * cos_a * sgn;
                cdi = beta * cl * cl / pi_ar_e;
            }
            cd = cdi + cd0;
        }
        // Vortex lift of slender wings (Polhamus' leading-edge suction analogy, NASA TN D-3767 eq. 15):
        // the lost edge suction turns into a normal force Kv·sin²α, Kv ≈ π, so lift Kv·cos α·sin²α and
        // drag Kv·sin³α (his CD ≈ CL·tan α). Subsonic, like camber.
        // Lamar (NASA TR R-428): with the side edges' share the total stays near π, and predicts the
        // lift of round-edged rectangular wings of aspect ratio 0.5 to 2 well. The vortices burst
        // near 30° (Wentz & Kohlman: 29° on a 70° delta).
        // shortcut: the fades from aspect ratio 2 to 3 and from 25° to 35° are guesses.
        let vortex = PI * sin_a * sin_a * (3.0 - p.ar).clamp(0.0, 1.0) * ((0.61 - alpha.abs()) / 0.175).clamp(0.0, 1.0) * camber_fade(mach);
        cl += vortex * cos_a * sgn;
        cd += vortex * sin_a;
        let mut at = p.centroid + pl * ac_offset(mach, p.mac, cos_sweep, cos_a);
        if st > 0.0 {
            at -= pl * (0.75 / 1.6 * p.mac * cl.abs() * st * cos_a);
        }
        // Stalled: a flat plate's normal force, CN = Cd90·sin α / (0.56 + 0.44 sin α). With Cd90 = 1.8
        // it is within 0.06 of the NACA 0012 CL and CD measured from 30° to 90° (Sheldahl & Klimas,
        // SAND80-2114, table 2). FAR instead scaled the unstalled lift and drag (×0.23, ×4): about half
        // the lift and twice the drag at 30°.
        let cn = cd_max * sin_a / (0.56 + 0.44 * sin_a);
        cl += (cn * cos_a * sgn - cl) * st;
        cd += (cn * sin_a + cd0 - cd) * st;
        cl *= p.interference;
        let q = 0.5 * flow.rho * speed * speed;
        let force = (lift_dir * cl - vn * cd) * (q * p.area);
        out.push(PanelForce { force, at, cl, cd, cd_induced: cdi });
    }
    out
}

/// FAR `CalculateSubsonicLiftSlope` + `SetSweepAngle`: (lift slope per rad, cos LE sweep).
fn subsonic_lift_slope(p: &Panel, mach: f64, pl: V3) -> (f64, f64) {
    let lam = p.taper;
    let k = (1.0 - lam) / (1.0 + lam);
    let geom_ar = p.b / p.mac;
    let sweep = p.tan_sweep.atan();
    // In-plane flow direction in panel coordinates: x along span, y along chord.
    let px = pl.dot(p.s);
    let py = pl.dot(p.u);
    let mss = (PI / 2.0 - (p.tan_sweep + k * 4.0 / geom_ar).atan()) * k * 0.5;
    let c1 = (sweep.sin() * -px + sweep.cos() * py).clamp(-1.0, 1.0);
    let c2 = (mss.cos() * px + mss.sin() * py).clamp(-1.0, 1.0);
    let x = if c1.abs() > c2.abs() { c1 } else { c2 };
    let cy2 = py.clamp(-1.0, 1.0).powi(2);
    let sy2 = (1.0 - cy2).clamp(0.0, 1.0);
    let b_eff = (p.b * cy2).max(p.mac * sy2);
    let mac_eff = p.mac * cy2 + p.b * sy2;
    let ar_t = b_eff / mac_eff;
    let t = (1.0 - x * x).max(0.0).sqrt() / x.abs().max(1e-9);
    let tt = t + k * 2.0 / ar_t;
    let cos_sweep = (1.0 / (1.0 + tt * tt).sqrt()).min(1.0);
    let beta2 = if mach < 0.9 { 1.0 - mach * mach } else { 0.19 };
    let ar = p.ar;
    let ls = 2.0 * PI / (2.0 + (4.0 + ar * ar * (beta2 + t * t)).sqrt()) * ar;
    (ls, cos_sweep)
}

/// Camber lifts toward body-up (−z): +1 when the panel normal points up, −1 down, 0 on fins.
fn camber_sign(n: V3) -> f64 {
    if n.z.abs() < 0.5 {
        0.0
    } else {
        -n.z.signum()
    }
}

/// Camber lift is a subsonic effect (linear supersonic theory: camber gives no lift).
fn camber_fade(mach: f64) -> f64 {
    ((1.2 - mach) / 0.4).clamp(0.0, 1.0)
}

/// Share of the stall angle from which the lift curve bends over. Measured: 0.55 on a 6 % section
/// (NACA 64A006: bends at 5°, peak 0.89 at 9°, NACA TN 1923), 0.71 at 18 % (63-018: 10° and 14°,
/// NACA TN 2502). shortcut: one figure for every section; a 12 % one stays straight and breaks sharply.
const ROUND_FROM: f64 = 0.65;

/// What is left of the straight-line lift at angle `a`: a parabola from `ROUND_FROM` of the stall
/// angle, level at the stall angle.
fn rounded(a: f64, aoa_max: f64) -> f64 {
    let from = ROUND_FROM * aoa_max;
    let over = a.min(aoa_max) - from;
    if over <= 0.0 || a <= 0.0 {
        return 1.0;
    }
    (a.min(aoa_max) - over * over / (2.0 * (aoa_max - from))) / a
}

fn aoa_max(mach: f64, lift_slope: f64, cl_max: f64) -> f64 {
    // Slender wings: cl_max / slope runs past 60°, but their vortex lift breaks down near 30°
    // (Polhamus: the analogy holds to 20-25° at AR 0.5-2). shortcut: 30° is a round figure.
    // ×2/(1+ROUND_FROM): the rounded top (`rounded`) reaches cl_max later than a straight line would.
    let tmp = (cl_max / lift_slope * 2.0 / (1.0 + ROUND_FROM)).min(0.52 * (cl_max / 1.6).max(1.0));
    if mach < 0.8 {
        tmp
    } else if mach > 1.4 {
        PI / 3.0
    } else {
        (mach - 0.8) * (PI / 3.0 - tmp) / 0.6 + tmp
    }
}

fn ac_offset(mach: f64, mac: f64, cos_sweep: f64, cos_a: f64) -> f64 {
    let tmp = cos_a * cos_a;
    let off = if mach < 0.85 {
        0.25 * mac
    } else if mach > 1.4 {
        0.10 * mac
    } else if mach >= 1.0 {
        mac * (-0.375 * mach + 0.625)
    } else {
        let sf = cos_sweep * cos_sweep * tmp;
        if mach < 0.9 {
            mac * ((mach - 0.85) * 2.0 * sf + 0.25)
        } else {
            mac * ((1.0 - mach) * sf + 0.25)
        }
    };
    off * tmp
}

fn le_factor(ar: f64, beta: f64, tan_sweep: f64) -> f64 {
    let art = ar * tan_sweep;
    let bt = beta / tan_sweep.max(1e-9);
    if bt < 1.0 {
        let f = if bt < 0.5 { 1.57 * ar / (art + 0.5) } else { (1.57 - 0.28 * (bt - 0.5)) * ar / (art + 0.5 - 0.25 * (bt - 0.5)) };
        f * beta
    } else {
        1.0 + (1.0 / bt) * (1.43 * art / (art + 0.375) - 1.0)
    }
}

/// Zero-lift drag rise from compressibility (FAR `CdCompressibilityZeroLiftIncrement`, own part),
/// scaled to the section: drag divergence at Korn's κ − t/c (FAR: 0.8), transonic peak with
/// (t/c)^1.5 and supersonic wave drag k·(t/c)²/β (FAR: 0.009216/β, a 4 % biconvex).
fn cd_compressibility(mach: f64, cos_sweep: f64, af: &Aerofoil) -> f64 {
    let cos_sweep = cos_sweep.max(1e-6);
    let beta = (mach * mach - 1.0).sqrt().max(0.663_324_96);
    let beta = if beta.is_nan() { 0.663_324_96 } else { beta };
    let tan_sweep = (1.0 - cos_sweep * cos_sweep).max(0.0).sqrt() / cos_sweep;
    let bt = beta / tan_sweep.max(1e-9);
    let wave = af.k_wave * af.t_c * af.t_c;
    let sup = if bt < 1.0 { wave / tan_sweep } else { wave / beta };
    if mach > 1.4 {
        return sup;
    }
    let dd = (af.kappa - af.t_c) / cos_sweep.sqrt();
    let pk = dd + 0.3 / cos_sweep.sqrt();
    let pk_inc = 0.025 * cos_sweep.powf(2.5) * (af.t_c / 0.0416).powf(1.5);
    let mut inc = if mach < dd {
        0.0
    } else if mach > pk {
        pk_inc
    } else {
        pk_inc * (((2.0 * mach - 3.0 * (dd + pk)) * mach + 6.0 * dd * pk) * mach + dd * dd * (dd - 3.0 * pk)) / (dd - pk).powi(3)
    };
    let sc = pk.min(1.2);
    if mach >= sc {
        let s = (mach - 1.4) / (sc - 1.4);
        inc = inc * s + (1.0 - s) * sup;
    }
    inc
}

/// Drag of a flat plate square to the flow: FAR's Mach curve (2.0 when slow, an endless plate),
/// scaled to the plate's aspect ratio by Spera's fit to Hoerner's and Binder's plates,
/// Cd90 = 1.98·(0.60 + 0.40·e^(−20/AR)) (NASA/CR-2008-215434 eq. 4a): 1.19 square, 1.98 endless.
pub fn flat_plate_cd(mach: f64, ar: f64) -> f64 {
    let beta = (mach * mach - 1.0).sqrt().max(0.663_324_958_07);
    let ar = ar.max(1e-3);
    let ar = ar.max(1.0 / ar);
    cd_max_flat_plate(mach, if beta.is_nan() { 0.663_324_958_07 } else { beta }) * (0.60 + 0.40 * (-20.0 / ar).exp())
}

fn cd_max_flat_plate(mach: f64, beta: f64) -> f64 {
    if mach < 0.5 {
        2.0
    } else if mach < 1.0 {
        2.0 + 2.0 * (mach - 0.5).powi(2)
    } else if mach <= 1.2 {
        3.39 - 0.609091 * mach
    } else {
        0.4 / (beta * beta) + 1.75
    }
}

/// Nita–Scholz Oswald efficiency (FAR `CalculateOswaldsEfficiencyNitaScholz`, exact exp/pow).
fn oswald(ar: f64, cos_sweep: f64, cd0: f64, taper: f64) -> f64 {
    let sweep_deg = cos_sweep.clamp(-1.0, 1.0).acos().to_degrees();
    let d_taper = 0.45 * (-0.0375 * sweep_deg).exp() - 0.357;
    let l = taper - d_taper;
    let e_str = (((0.0524 * l - 0.15) * l + 0.1659) * l - 0.0706) * l + 0.0119;
    let e_theo = 1.0 / (e_str * ar + 1.0);
    let e_wi = 0.974008 * e_theo;
    e_wi / (1.0 + 0.38 * cd0 * ar * PI * e_wi)
}

/// FAR `SkinFrictionDrag(Re, M)` (with its 8 % roughness factor).
pub fn skin_friction(re: f64, mach: f64) -> f64 {
    const TR: f64 = 5e5;
    if !(re > 0.0) {
        return 0.005;
    }
    if re < TR {
        let lam = 1.328 / re.sqrt();
        let rare = mach / re;
        if rare > 0.01 {
            return (lam + (0.075 - lam) * (rare - 0.01) / (0.99 + rare)) * 1.08;
        }
        return lam * 1.08;
    }
    let frac = TR / re;
    let lam = 1.328 / TR.sqrt();
    let turb_lam = 0.074 / TR.powf(0.2);
    (0.074 / re.powf(0.2) - frac * (turb_lam - lam)) * 1.08
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_wing() -> (Grid, V3) {
        // Rectangular wing 1 voxel thick: chord 8 (z), span x in -24..=24, plus a fuselage box.
        let mut cells = Vec::new();
        for x in -24i32..=24 {
            for z in 0..8 {
                cells.push(([x, 0, z], 1.0, if x.abs() <= 2 { 1 } else { 0 }));
            }
        }
        for x in -2..=2 {
            for y in -2..=2 {
                for z in -16..16 {
                    if !(y == 0 && (0..8).contains(&z)) {
                        cells.push(([x, y, z], 1.0, 1));
                    }
                }
            }
        }
        (Grid::from_cells(&cells), v3(0.75, 0.0, 0.0))
    }

    #[test]
    fn deflected_trailing_flap_adds_lift() {
        // flat_wing with its rear quarter chord (z 0..2) as a separate flap surface.
        let (g0, com) = flat_wing();
        let cells: Vec<([i32; 3], f32, u32)> = (0..g0.fill.len())
            .filter(|&i| g0.owner[i] != crate::voxel::EMPTY && g0.owner[i] != crate::voxel::INTERIOR)
            .map(|i| {
                let c = g0.cell(i);
                let o = g0.owner[i];
                (c, 1.0, if o == 0 && c[1] == 0 && c[2] < 2 { 2 } else { o })
            })
            .collect();
        let g = Grid::from_cells(&cells);
        let panels = build_panels(
            &g,
            &|o| match o {
                0 => Some((Key::Wing(0), Role::None)),
                2 => Some((Key::Comp(2), Role::Flap)),
                _ => None,
            },
            com,
        );
        assert!(panels.iter().any(|p| p.role == Role::Flap) && panels.iter().any(|p| !p.aft.is_empty() || !p.fwd.is_empty()));
        let flow = WingFlow { mach: 0.2, rho: 1.225, re_per_m: 4.6e6, gamma: 1.4 };
        let lift = |flap_deg: f64| {
            let a = 8f64.to_radians();
            let v = v3(a.cos(), 0.0, a.sin()) * 68.0;
            let defl: Vec<f64> = panels.iter().map(|p| if p.role == Role::Flap { p.ctrl_sign * flap_deg.to_radians() } else { 0.0 }).collect();
            let mut stall = vec![0.0; panels.len()];
            let f = panel_forces(&panels, &flow, &|_, _| v, &defl, &mut stall);
            let t = f.iter().fold(V3::ZERO, |acc, p| acc + p.force);
            -(t.z * a.cos() - t.x * a.sin())
        };
        let (clean, flaps) = (lift(0.0), lift(35.0));
        assert!(flaps > clean * 1.15 && flaps >= lift(20.0) * 0.95, "35° flaps add lift at takeoff AoA: {flaps} vs {clean}");
        // The camber lift over the wing's chord acts on the wing ahead of the flap, not all on the flap.
        let wing_cl = |flap_deg: f64| {
            let a = 8f64.to_radians();
            let v = v3(a.cos(), 0.0, a.sin()) * 68.0;
            let defl: Vec<f64> = panels.iter().map(|p| if p.role == Role::Flap { p.ctrl_sign * flap_deg.to_radians() } else { 0.0 }).collect();
            let mut stall = vec![0.0; panels.len()];
            let f = panel_forces(&panels, &flow, &|_, _| v, &defl, &mut stall);
            panels.iter().zip(&f).filter(|(p, _)| p.role == Role::None).map(|(_, f)| f.cl).sum::<f64>()
        };
        assert!(wing_cl(35.0) > wing_cl(0.0) + 0.1, "wing gains lift from its flap: {} vs {}", wing_cl(35.0), wing_cl(0.0));
    }

    #[test]
    fn rectangular_wing_panels() {
        let (g, com) = flat_wing();
        let panels = build_panels(&g, &|o| (o == 0).then_some((Key::Wing(0), Role::None)), com);
        assert_eq!(panels.len(), 2, "split at the fuselage into two halves");
        let p = &panels[0];
        assert!((p.b - 5.5).abs() < 1e-9, "semispan {}", p.b);
        assert!((p.mac - 2.0).abs() < 1e-9, "chord {}", p.mac);
        assert!(p.tan_sweep.abs() < 1e-9 && (p.taper - 1.0).abs() < 1e-9);
        assert!(p.n.z.abs() > 0.99, "normal {:?}", p.n);
        assert!(p.root_covered > 0.99 && p.tip_covered < 0.01);
        assert!((p.ar - 2.0 * 5.5 / 2.0).abs() < 1e-9, "AR {}", p.ar);
        // Tips point outward.
        for p in &panels {
            assert!(p.s.y * p.centroid.y > 0.0);
        }
    }

    #[test]
    fn lift_up_at_positive_aoa_and_stall_hysteresis() {
        let (g, com) = flat_wing();
        let panels = build_panels(&g, &|o| (o == 0).then_some((Key::Wing(0), Role::None)), com);
        let flow = WingFlow { mach: 0.2, rho: 1.225, re_per_m: 4.6e6, gamma: 1.4 };
        let mut stall = vec![0.0; panels.len()];
        let defl = vec![0.0; panels.len()];
        let at = |deg: f64, stall: &mut Vec<f64>| {
            let a = deg.to_radians();
            // Vehicle moves forward, nose up => velocity in body frame has +z (down) component.
            let v = v3(a.cos(), 0.0, a.sin()) * 68.0;
            let f = panel_forces(&panels, &flow, &|_, _| v, &defl, stall);
            let q = 0.5 * 1.225 * 68.0 * 68.0;
            let s: f64 = panels.iter().map(|p| p.area).sum();
            let lift = f.iter().fold(V3::ZERO, |a, p| a + p.force);
            // Lift up = −z (rotated by α, but at small α −z dominates).
            (-(lift.z * a.cos() - lift.x * a.sin()) / (q * s), f[0].cl)
        };
        let (cl5, _) = at(5.0, &mut stall);
        let cla = cl5 / 5f64.to_radians();
        // Finite wing AR 5.5: Helmbold gives ~3.9 /rad.
        assert!(cla > 3.0 && cla < 4.6, "CLα {cla}");
        let (cl30, _) = at(30.0, &mut stall);
        assert!(stall[0] > 0.0 && cl30 < cl5 * 6.0, "stalled at 30°");
        at(23.0, &mut stall);
        assert!(stall[0] > 0.0, "hysteresis keeps it stalled on the way down");
        assert!(skin_friction(1e7, 0.2) > 0.002 && skin_friction(1e7, 0.2) < 0.004);
    }

    #[test]
    fn aerofoils_trade_lift_for_drag() {
        let (g, com) = flat_wing();
        let base = build_panels(&g, &|o| (o == 0).then_some((Key::Wing(0), Role::None)), com);
        // (CL, CD) of the whole wing with section `foil` at α (deg) and Mach.
        let coeffs = |foil: &str, deg: f64, mach: f64| {
            let mut panels = base.clone();
            panels.iter_mut().for_each(|p| p.foil = aerofoil_index(foil).unwrap());
            let speed = mach * 340.0;
            let flow = WingFlow { mach, rho: 1.225, re_per_m: 4.6e6, gamma: 1.4 };
            let a = f64::to_radians(deg);
            let v = v3(a.cos(), 0.0, a.sin()) * speed;
            let mut stall = vec![0.0; panels.len()];
            let f = panel_forces(&panels, &flow, &|_, _| v, &vec![0.0; panels.len()], &mut stall);
            let q = 0.5 * 1.225 * speed * speed;
            let s: f64 = panels.iter().map(|p| p.area).sum();
            let t = f.iter().fold(V3::ZERO, |acc, p| acc + p.force) * (1.0 / (q * s));
            (-(t.z * a.cos() - t.x * a.sin()), -(t.x * a.cos() + t.z * a.sin()))
        };
        // Slotted section: still lifting where the plain symmetric one has stalled, and draggier when not.
        let (cl_plain, _) = coeffs("naca0012", 24.0, 0.2);
        let (cl_slot, _) = coeffs("slotted0012", 24.0, 0.2);
        assert!(cl_slot > cl_plain * 1.3, "slotted keeps lift at 24°: {cl_slot} vs {cl_plain}");
        assert!(coeffs("slotted0012", 2.0, 0.2).1 > coeffs("naca0012", 2.0, 0.2).1);
        // The default section is FAR's: same results as before the aerofoil table existed.
        assert_eq!(AEROFOILS[0].id, "far");
        let (cl0_far, _) = coeffs("far", 0.0, 0.2);
        let (cl0_gal, _) = coeffs("naca4415", 0.0, 0.2);
        assert!(cl0_far.abs() < 1e-9 && cl0_gal > 0.15, "camber lifts at zero AoA, upward: {cl0_gal}");
        // High-lift section: more lift at low speed, more drag everywhere, terrible supersonic.
        let (cl_gal, cd_gal) = coeffs("naca4415", 5.0, 0.2);
        let (cl_bic, cd_bic) = coeffs("biconvex4", 5.0, 0.2);
        assert!(cl_gal > cl_bic + 0.15 && cd_gal > cd_bic, "GA {cl_gal}/{cd_gal} vs biconvex {cl_bic}/{cd_bic}");
        let (_, cd_gal_s) = coeffs("naca4415", 2.0, 1.6);
        let (_, cd_bic_s) = coeffs("biconvex4", 2.0, 1.6);
        assert!(cd_gal_s > 3.0 * cd_bic_s, "supersonic wave drag {cd_gal_s} vs {cd_bic_s}");
        // Camber gives no lift supersonic.
        let (cls_gal, _) = coeffs("naca4415", 0.0, 1.6);
        assert!(cls_gal.abs() < 1e-6, "{cls_gal}");
        // Supercritical delays the drag rise (zero-lift compressibility drag, straight wing).
        let foil = |id| &AEROFOILS[aerofoil_index(id).unwrap()];
        let (sc, n0012) = (cd_compressibility(0.8, 1.0, foil("supercritical")), cd_compressibility(0.8, 1.0, foil("naca0012")));
        assert!(sc < 1e-9 && n0012 > 1e-3, "M 0.8: supercritical {sc} vs 0012 {n0012}");
    }

    #[test]
    fn thick_body_marked_whole_makes_a_panel() {
        // A flying wing: the body is the wing, 3 voxels thick (grid z = aero x), 8 chord, 8 span.
        let cells: Vec<([i32; 3], f32, u32)> = (0..8).flat_map(|y| (0..8).flat_map(move |x| (0..3).map(move |z| ([y + 3, z, x], 1.0, 0u32)))).collect();
        let g = Grid::from_cells(&cells);
        let panels = build_panels(&g, &|o| (o == 0).then_some((Key::Wing(0), Role::None)), v3(0.0, 0.0, 0.0));
        assert_eq!(panels.len(), 1);
        let p = &panels[0];
        // Planform is chord x span of whole voxels, whatever the thickness.
        assert!((p.area / (VOXEL_M * VOXEL_M) - 64.0).abs() < 1e-6, "area {} voxels", p.area / (VOXEL_M * VOXEL_M));
    }

    #[test]
    fn flap_facing_the_flow_still_makes_a_panel() {
        // A plate across the flow (thin along x) like a Starship drag flap: 12 wide, 6 tall, one
        // side of the symmetry plane so it is not split into two.
        // Cells are [grid x, grid y, grid z] = [aero y, -aero z, aero x].
        let cells: Vec<([i32; 3], f32, u32)> = (3..15).flat_map(|y| (0..6).map(move |z| ([y, z, 0], 1.0, 0u32))).collect();
        let g = Grid::from_cells(&cells);
        let panels = build_panels(&g, &|o| (o == 0).then_some((Key::Wing(0), Role::None)), v3(0.0, 0.0, 0.0));
        assert_eq!(panels.len(), 1, "a plate facing the flow is still a wing");
        let p = &panels[0];
        let v = VOXEL_M * VOXEL_M;
        assert!((p.area / v - 72.0).abs() < 1e-6, "area {} voxels", p.area / v);
        assert!((p.b / VOXEL_M - 12.0).abs() < 1e-6, "span {}", p.b / VOXEL_M);
        assert!(p.n.x.abs() > 0.9, "normal {:?} should face the flow", p.n);
    }

    #[test]
    fn split_wing_shares_one_aspect_ratio() {
        // The flat wing with its outer half (|x| > 13) as a separate part: inner + outer panels
        // per side must get the aspect ratio of the whole wing.
        let (g, com) = flat_wing();
        let cells: Vec<([i32; 3], f32, u32)> = (0..g.fill.len())
            .filter(|&i| g.owner[i] != EMPTY)
            .map(|i| {
                let c = g.cell(i);
                (c, g.fill[i], if g.owner[i] == 0 && c[0].abs() > 13 { 2 } else { g.owner[i] })
            })
            .collect();
        let g = Grid::from_cells(&cells);
        let key = |o: u32| match o {
            0 => Some((Key::Wing(0), Role::None)),
            2 => Some((Key::Wing(1), Role::None)),
            _ => None,
        };
        let panels = build_panels(&g, &key, com);
        assert_eq!(panels.len(), 4);
        for p in &panels {
            assert!((p.ar - 5.5).abs() < 1e-9, "whole-wing AR 5.5, got {} (b {})", p.ar, p.b);
        }
    }
}

#[cfg(test)]
mod side_normal_tests {
    use super::*;

    #[test]
    fn wedge_fin_mid_plane_has_no_cant() {
        // A fin in the x-z plane (flow along x), 10 mm thick at the root (z = 0) and 2 mm at the
        // tip (z = 0.2): its two sides lean in opposite ways; their mid-plane is y = 0.
        let side = |y: f64, flip: bool| {
            let (a, b, c, d) = (v3(0.0, y * 1.0, 0.0), v3(0.3, y * 1.0, 0.0), v3(0.3, y * 0.2, 0.2), v3(0.0, y * 0.2, 0.2));
            if flip { [[a, c, b], [a, d, c]] } else { [[a, b, c], [a, c, d]] }
        };
        let tris: Vec<[V3; 3]> = side(0.005, false).into_iter().chain(side(-0.005, true)).collect();
        let n = side_normal(&tris, v3(0.0, 1.0, 0.0)).unwrap();
        assert!((n.y - 1.0).abs() < 1e-9 && n.x.abs() < 1e-9, "{n:?}");
    }
}

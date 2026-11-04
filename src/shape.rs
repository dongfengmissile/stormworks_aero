//! True voxel shapes. Every Stormworks physics shape is the unit cube cut by one plane:
//! wedges, corner pyramids and inverse pyramids, optionally stretched over several voxels
//! (shapes 4..41 are the per-voxel pieces of the 2/4-long and 2x2.. blocks).
//!
//! The canonical plane of each shape id is derived from the game's own block definitions
//! (`02_wedge` .. `16_invpyramid_4x4`, all at identity rotation), whose surfaces show:
//! wedge = bottom (−y) and front (+z) faces full; pyramid = right-angle corner at
//! (+x, −y, +z); inverse pyramid = cube minus the corner at (−x, +y, −z).

use std::collections::HashMap;
use std::path::Path;

use crate::math::{v3, V3};
use crate::voxel::DIRS;
use crate::xml;

/// Solid part of a voxel: points p (voxel units, centre origin) with `n·p <= d`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub n: V3,
    pub d: f64,
}

/// Plane rotated by a voxel rotation applied as a row vector (`p' = p·m`).
pub fn rotate(p: Plane, m: &[i32; 9]) -> Plane {
    let n = p.n;
    let m = m.map(|v| v as f64);
    Plane { n: v3(n.x * m[0] + n.y * m[3] + n.z * m[6], n.x * m[1] + n.y * m[4] + n.z * m[7], n.x * m[2] + n.y * m[5] + n.z * m[8]), d: p.d }
}

pub fn transpose(m: &[i32; 9]) -> [i32; 9] {
    [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]
}

#[derive(Default)]
pub struct ShapeTable {
    planes: HashMap<i32, Plane>,
}

impl ShapeTable {
    /// Read the stock block definitions in `rom/data/definitions`.
    pub fn from_rom(dir: &Path) -> ShapeTable {
        const BLOCKS: [(&str, u8); 15] = [
            ("02_wedge", b'w'),
            ("05_wedge_2", b'w'),
            ("08_wedge_4", b'w'),
            ("03_pyramid", b'p'),
            ("06_pyramid_2", b'p'),
            ("09_pyramid_4", b'p'),
            ("11_pyramid_2x2", b'p'),
            ("12_pyramid_2x4", b'p'),
            ("13_pyramid_4x4", b'p'),
            ("04_invpyramid", b'i'),
            ("07_invpyramid_2", b'i'),
            ("10_invpyramid_4", b'i'),
            ("14_invpyramid_2x2", b'i'),
            ("15_invpyramid_2x4", b'i'),
            ("16_invpyramid_4x4", b'i'),
        ];
        let mut t = ShapeTable::default();
        for (file, family) in BLOCKS {
            if let Ok(text) = std::fs::read_to_string(dir.join(format!("{file}.xml"))) {
                t.add_block(&text, family);
            }
        }
        t
    }

    fn add_block(&mut self, text: &str, family: u8) {
        let mut vox: Vec<([i32; 3], i32)> = Vec::new();
        let mut open = false;
        for tag in xml::tags(text) {
            match (tag.name, tag.close) {
                ("voxel", false) => {
                    vox.push(([0; 3], tag.int0("physics_shape")));
                    open = !tag.empty;
                }
                ("voxel", true) => open = false,
                ("position", false) if open => {
                    if let Some(v) = vox.last_mut() {
                        v.0 = [tag.int0("x"), tag.int0("y"), tag.int0("z")];
                    }
                }
                _ => {}
            }
        }
        if vox.is_empty() {
            return;
        }
        let lo = [0, 1, 2].map(|a| vox.iter().map(|v| v.0[a]).min().unwrap() as f64 - 0.5);
        let hi = [0, 1, 2].map(|a| vox.iter().map(|v| v.0[a]).max().unwrap() as f64 + 0.5);
        let l = [0, 1, 2].map(|a| hi[a] - lo[a]);
        // Block-space plane, then shifted so it is relative to each voxel's centre.
        let (n, d) = match family {
            // (y − ylo)/Ly <= (z − zlo)/Lz
            b'w' => (v3(0.0, 1.0 / l[1], -1.0 / l[2]), lo[1] / l[1] - lo[2] / l[2]),
            // (xhi − x)/Lx + (y − ylo)/Ly + (zhi − z)/Lz <= 1
            b'p' => (v3(-1.0 / l[0], 1.0 / l[1], -1.0 / l[2]), 1.0 - hi[0] / l[0] + lo[1] / l[1] - hi[2] / l[2]),
            // (x − xlo)/Lx + (yhi − y)/Ly + (z − zlo)/Lz >= 1
            _ => (v3(-1.0 / l[0], 1.0 / l[1], -1.0 / l[2]), -1.0 - lo[0] / l[0] + hi[1] / l[1] - lo[2] / l[2]),
        };
        for (p, shape) in vox {
            let c = v3(p[0] as f64, p[1] as f64, p[2] as f64);
            self.planes.entry(shape).or_insert(Plane { n, d: d - n.dot(c) });
        }
    }

    /// Canonical plane of a shape id; None = full cube (shape 0 or unknown).
    pub fn plane(&self, shape: i32) -> Option<Plane> {
        if shape == 0 {
            return None;
        }
        self.planes.get(&shape).copied()
    }

    pub fn len(&self) -> usize {
        self.planes.len()
    }
}

/// The cube face polygons (DIRS order) and cap of one voxel, voxel units, centre origin.
pub struct CellGeom {
    pub faces: [Vec<V3>; 6],
    pub cap: Vec<V3>,
    pub volume: f64,
}

fn cube_face(k: usize) -> Vec<V3> {
    let d = DIRS[k];
    let n = v3(d[0] as f64, d[1] as f64, d[2] as f64);
    let (a, b) = if d[0] != 0 { (v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)) } else if d[1] != 0 { (v3(0.0, 0.0, 1.0), v3(1.0, 0.0, 0.0)) } else { (v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0)) };
    let o = n * 0.5;
    vec![o - a * 0.5 - b * 0.5, o + a * 0.5 - b * 0.5, o + a * 0.5 + b * 0.5, o - a * 0.5 + b * 0.5]
}

pub fn clip(poly: &[V3], p: &Plane) -> Vec<V3> {
    let mut out = Vec::new();
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (da, db) = (p.n.dot(a) - p.d, p.n.dot(b) - p.d);
        if da <= 1e-12 {
            out.push(a);
        }
        if (da < -1e-12 && db > 1e-12) || (da > 1e-12 && db < -1e-12) {
            out.push(a + (b - a) * (da / (da - db)));
        }
    }
    out
}

pub fn area(poly: &[V3]) -> f64 {
    let mut s = V3::ZERO;
    for i in 1..poly.len().saturating_sub(1) {
        s += (poly[i] - poly[0]).cross(poly[i + 1] - poly[0]);
    }
    s.len() * 0.5
}

pub fn geometry(plane: Option<&Plane>) -> CellGeom {
    let Some(p) = plane else {
        return CellGeom { faces: std::array::from_fn(cube_face), cap: Vec::new(), volume: 1.0 };
    };
    let faces: [Vec<V3>; 6] = std::array::from_fn(|k| {
        let f = clip(&cube_face(k), p);
        if area(&f) < 1e-9 {
            Vec::new()
        } else {
            f
        }
    });
    // Cap: where the plane crosses the cube's edges, ordered around its centre.
    let mut pts: Vec<V3> = Vec::new();
    let corners: Vec<V3> = (0..8).map(|i| v3((i & 1) as f64 - 0.5, ((i >> 1) & 1) as f64 - 0.5, ((i >> 2) & 1) as f64 - 0.5)).collect();
    for i in 0..8 {
        for bit in [1, 2, 4] {
            let j = i | bit;
            if j == i {
                continue;
            }
            let (a, b) = (corners[i], corners[j]);
            let (da, db) = (p.n.dot(a) - p.d, p.n.dot(b) - p.d);
            if da.abs() < 1e-9 {
                pts.push(a);
            }
            if db.abs() < 1e-9 {
                pts.push(b);
            }
            if da * db < 0.0 && da.abs() > 1e-9 && db.abs() > 1e-9 {
                pts.push(a + (b - a) * (da / (da - db)));
            }
        }
    }
    pts.dedup_by(|a, b| (*a - *b).len() < 1e-7);
    let mut uniq: Vec<V3> = Vec::new();
    for q in pts {
        if !uniq.iter().any(|u| (*u - q).len() < 1e-7) {
            uniq.push(q);
        }
    }
    let nn = p.n.norm();
    let mut cap = Vec::new();
    if uniq.len() >= 3 {
        let c = uniq.iter().fold(V3::ZERO, |a, b| a + *b) * (1.0 / uniq.len() as f64);
        let e1 = (uniq[0] - c).norm();
        let e2 = nn.cross(e1);
        uniq.sort_by(|a, b| {
            let (ta, tb) = ((*a - c).dot(e2).atan2((*a - c).dot(e1)), (*b - c).dot(e2).atan2((*b - c).dot(e1)));
            ta.total_cmp(&tb)
        });
        cap = uniq;
    }
    let face_area: f64 = faces.iter().map(|f| area(f)).sum();
    let cap_area = area(&cap);
    let volume = ((0.5 * face_area + p.d / p.n.len() * cap_area) / 3.0).clamp(0.0, 1.0);
    CellGeom { faces, cap, volume }
}

/// A convex solid as its face polygons.
pub type Solid = Vec<Vec<V3>>;

/// The voxel's solid (unit cube at the origin, optionally cut), as faces.
pub fn voxel_solid(plane: Option<&Plane>) -> Solid {
    let g = geometry(plane);
    let mut f: Solid = g.faces.into_iter().filter(|f| f.len() >= 3).collect();
    if g.cap.len() >= 3 {
        f.push(g.cap);
    }
    f
}

/// Keep the part of a convex solid with `n·p <= d`.
pub fn clip_solid(s: &Solid, p: &Plane) -> Solid {
    let mut out: Solid = Vec::new();
    let mut on: Vec<V3> = Vec::new();
    for f in s {
        let c = clip(f, p);
        on.extend(c.iter().filter(|q| (p.n.dot(**q) - p.d).abs() < 1e-9));
        if c.len() >= 3 && area(&c) > 1e-12 {
            out.push(c);
        }
    }
    let mut uniq: Vec<V3> = Vec::new();
    for q in on {
        if !uniq.iter().any(|u| (*u - q).len() < 1e-9) {
            uniq.push(q);
        }
    }
    // A face already lying in the plane is the cap (clipping at an existing face).
    let has_cap = out.iter().any(|f| f.iter().all(|q| (p.n.dot(*q) - p.d).abs() < 1e-9));
    if uniq.len() >= 3 && !has_cap {
        let c = uniq.iter().fold(V3::ZERO, |a, b| a + *b) * (1.0 / uniq.len() as f64);
        let nn = p.n.norm();
        let e1 = (uniq[0] - c).norm();
        let e2 = nn.cross(e1);
        uniq.sort_by(|a, b| ((*a - c).dot(e2).atan2((*a - c).dot(e1))).total_cmp(&(*b - c).dot(e2).atan2((*b - c).dot(e1))));
        if area(&uniq) > 1e-12 {
            out.push(uniq);
        }
    }
    out
}

/// Volume of a convex solid: tetrahedra from an interior point.
pub fn solid_volume(s: &Solid) -> f64 {
    let pts: Vec<V3> = s.iter().flatten().copied().collect();
    if pts.is_empty() {
        return 0.0;
    }
    let c = pts.iter().fold(V3::ZERO, |a, b| a + *b) * (1.0 / pts.len() as f64);
    let mut v = 0.0;
    for f in s {
        for i in 1..f.len().saturating_sub(1) {
            v += ((f[0] - c).dot((f[i] - c).cross(f[i + 1] - c))).abs() / 6.0;
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_volumes() {
        let w = Plane { n: v3(0.0, 1.0, -1.0), d: 0.0 }; // y <= z
        assert!((geometry(Some(&w)).volume - 0.5).abs() < 1e-9);
        let p = Plane { n: v3(-1.0, 1.0, -1.0), d: -0.5 };
        assert!((geometry(Some(&p)).volume - 1.0 / 6.0).abs() < 1e-9);
        let i = Plane { n: v3(-1.0, 1.0, -1.0), d: 0.5 };
        assert!((geometry(Some(&i)).volume - 5.0 / 6.0).abs() < 1e-9);
        assert_eq!(geometry(Some(&w)).cap.len(), 4);
        assert!((area(&geometry(Some(&w)).cap) - 2f64.sqrt()).abs() < 1e-9);
        // Clip the wedge with x <= 0: a quarter of the cube.
        let s = clip_solid(&voxel_solid(Some(&w)), &Plane { n: v3(1.0, 0.0, 0.0), d: 0.0 });
        assert!((solid_volume(&s) - 0.25).abs() < 1e-9, "{}", solid_volume(&s));
        assert!((solid_volume(&voxel_solid(None)) - 1.0).abs() < 1e-9);
        // Clipping at an existing face must not add a second copy of it.
        let s = clip_solid(&clip_solid(&voxel_solid(None), &Plane { n: v3(1.0, 0.0, 0.0), d: -0.46 }), &Plane { n: v3(-1.0, 0.0, 0.0), d: 0.5 });
        assert!(s.len() == 6 && (solid_volume(&s) - 0.04).abs() < 1e-9);
    }

    #[test]
    fn rom_blocks() {
        let Ok(home) = std::env::var("HOME") else { return };
        let dir = Path::new(&home).join(".local/share/Steam/steamapps/common/Stormworks/rom/data/definitions");
        if !dir.is_dir() {
            return;
        }
        let t = ShapeTable::from_rom(&dir);
        assert!(t.len() >= 41, "{}", t.len());
        let vol = |s| geometry(t.plane(s).as_ref()).volume;
        assert!((vol(1) - 0.5).abs() < 1e-9 && (vol(2) - 1.0 / 6.0).abs() < 1e-9 && (vol(3) - 5.0 / 6.0).abs() < 1e-9);
        // 2-long wedge: front voxel (z=0) 3/4, back voxel 1/4.
        assert!((vol(4) - 0.75).abs() < 1e-9 && (vol(5) - 0.25).abs() < 1e-9, "{} {}", vol(4), vol(5));
        // Every stretched block keeps its total volume.
        assert!((vol(6) + vol(7) + vol(8) + vol(9) - 2.0).abs() < 1e-9);
    }
}

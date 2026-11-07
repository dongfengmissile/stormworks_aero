//! Vehicle voxel grid (FAR's VehicleVoxel, but Stormworks parts already are voxels).
//!
//! One cell per 0.25 m game voxel, each holding the share of the cube its part
//! fills. Enclosed air (cabins, hollow hulls) is solidified like FAR does, so a
//! cross-section is the outer shape the flow sees.

use std::collections::HashMap;

use crate::defs::Library;
use crate::math::{v3, V3};
use crate::shape::{clip_solid, rotate, solid_volume, voxel_solid, Plane, Solid};
use crate::vehicle::{Comp, IDENTITY};

pub const VOXEL_M: f64 = 0.25;
pub const EMPTY: u32 = u32::MAX;
/// Enclosed air filled in by `solidify`.
pub const INTERIOR: u32 = u32::MAX - 1;

pub struct Grid {
    /// Grid cell of index (0,0,0). One empty cell of padding surrounds the vehicle.
    pub min: [i32; 3],
    pub dims: [usize; 3],
    /// Share of each cell that is solid (0..1).
    pub fill: Vec<f32>,
    /// Component index with the largest share of the cell, EMPTY or INTERIOR.
    pub owner: Vec<u32>,
    /// Every part voxel's true solid in grid units (cell c spans c ± 0.5), with its component.
    pub solids: Vec<(u32, Solid)>,
    /// Definitions that could not be found (drawn as one full voxel).
    pub missing: Vec<String>,
}

/// `p·R + pos`: the component transform, stretched or not.
fn place(s: &Solid, c: &Comp) -> Solid {
    let r = c.rot.map(|v| v as f64);
    let t = |p: V3| {
        v3(
            c.pos[0] as f64 + p.x * r[0] + p.y * r[3] + p.z * r[6],
            c.pos[1] as f64 + p.x * r[1] + p.y * r[4] + p.z * r[7],
            c.pos[2] as f64 + p.x * r[2] + p.y * r[5] + p.z * r[8],
        )
    };
    s.iter().map(|f| f.iter().map(|p| t(*p)).collect()).collect()
}

impl Grid {
    pub fn build(comps: &[Comp], lib: &mut Library) -> Grid {
        let mut solids: Vec<(u32, Solid)> = Vec::new();
        let mut missing = Vec::new();
        for (i, c) in comps.iter().enumerate() {
            match lib.get(&c.def) {
                Some(def) => {
                    for v in &def.voxels {
                        let plane = lib.shapes.plane(v.shape).map(|p| rotate(p, &v.rot));
                        let o = v3(v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64);
                        let mut cell = voxel_solid(plane.as_ref());
                        for p in v.bounds.iter().flatten() {
                            cell = clip_solid(&cell, p);
                        }
                        let local: Solid = cell.into_iter().map(|f| f.into_iter().map(|p| p + o).collect()).collect();
                        let placed = place(&local, c);
                        // XML-flattened parts (a zero row in the transform) take no space.
                        if solid_volume(&placed) > 1e-9 {
                            solids.push((i as u32, placed));
                        }
                    }
                }
                None => {
                    if !missing.contains(&c.def) {
                        missing.push(c.def.clone());
                    }
                    solids.push((i as u32, place(&voxel_solid(None), &Comp { rot: IDENTITY, ..c.clone() })));
                }
            }
        }
        // Voxelize: each solid's volume inside every cell it touches (FAR voxelizes meshes
        // the same way, with partial cells).
        let mut acc: HashMap<[i32; 3], (f32, u32, f32)> = HashMap::new();
        for (o, s) in &solids {
            let pts = s.iter().flatten();
            let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
            for p in pts {
                for a in 0..3 {
                    lo[a] = lo[a].min(p.get(a));
                    hi[a] = hi[a].max(p.get(a));
                }
            }
            let c0 = lo.map(|v| (v + 0.5 + 1e-6).floor() as i32);
            let c1 = hi.map(|v| (v + 0.5 - 1e-6).floor() as i32);
            let mut add = |c: [i32; 3], vol: f64| {
                if vol < 1e-6 {
                    return;
                }
                let e = acc.entry(c).or_insert((0.0, *o, 0.0));
                e.0 += vol as f32;
                if vol as f32 > e.2 {
                    e.1 = *o;
                    e.2 = vol as f32;
                }
            };
            if c0 == c1 {
                add(c0, solid_volume(s));
                continue;
            }
            for x in c0[0]..=c1[0] {
                for y in c0[1]..=c1[1] {
                    for z in c0[2]..=c1[2] {
                        let mut part = s.clone();
                        for (a, k) in [(0, x), (1, y), (2, z)] {
                            let mut n = [0.0; 3];
                            n[a] = 1.0;
                            part = clip_solid(&part, &Plane { n: v3(n[0], n[1], n[2]), d: k as f64 + 0.5 });
                            n[a] = -1.0;
                            part = clip_solid(&part, &Plane { n: v3(n[0], n[1], n[2]), d: -(k as f64 - 0.5) });
                        }
                        add([x, y, z], solid_volume(&part));
                    }
                }
            }
        }
        let cells: Vec<([i32; 3], f32, u32)> = acc.into_iter().map(|(c, (f, o, _))| (c, f.min(1.0), o)).collect();
        let mut grid = Grid::from_cells(&cells);
        grid.solids = solids;
        grid.missing = missing;
        grid.solidify();
        grid
    }

    /// Grid from (cell, fill, owner); solids are plain cubes (tests).
    pub fn from_cells(cells: &[([i32; 3], f32, u32)]) -> Grid {
        let mut lo = [i32::MAX; 3];
        let mut hi = [i32::MIN; 3];
        for (p, _, _) in cells {
            for a in 0..3 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
        if cells.is_empty() {
            lo = [0; 3];
            hi = [0; 3];
        }
        let min = [lo[0] - 1, lo[1] - 1, lo[2] - 1];
        let dims = [0, 1, 2].map(|a| (hi[a] - lo[a] + 3) as usize);
        let n = dims[0] * dims[1] * dims[2];
        let mut g = Grid { min, dims, fill: vec![0.0; n], owner: vec![EMPTY; n], solids: Vec::new(), missing: Vec::new() };
        for &(p, f, o) in cells {
            let i = g.index(p).unwrap();
            if f > g.fill[i] {
                g.fill[i] = f;
                g.owner[i] = o;
            }
            let c = Comp { def: String::new(), pos: p, rot: IDENTITY, body: 0 };
            g.solids.push((o, place(&voxel_solid(None), &c)));
        }
        g
    }

    pub fn index(&self, p: [i32; 3]) -> Option<usize> {
        let mut i = 0;
        for a in (0..3).rev() {
            let d = p[a] - self.min[a];
            if d < 0 || d as usize >= self.dims[a] {
                return None;
            }
            i = i * self.dims[a] + d as usize;
        }
        Some(i)
    }

    pub fn cell(&self, i: usize) -> [i32; 3] {
        let x = i % self.dims[0];
        let y = (i / self.dims[0]) % self.dims[1];
        let z = i / (self.dims[0] * self.dims[1]);
        [self.min[0] + x as i32, self.min[1] + y as i32, self.min[2] + z as i32]
    }

    pub fn filled(&self, p: [i32; 3]) -> bool {
        self.index(p).is_some_and(|i| self.owner[i] != EMPTY)
    }

    /// Flood the outside air from the padding; whatever empty cell it cannot reach is enclosed.
    fn solidify(&mut self) {
        let mut outside = vec![false; self.fill.len()];
        let mut stack = vec![0usize];
        outside[0] = true;
        while let Some(i) = stack.pop() {
            let p = self.cell(i);
            for d in DIRS {
                if let Some(j) = self.index([p[0] + d[0], p[1] + d[1], p[2] + d[2]]) {
                    if !outside[j] && self.owner[j] == EMPTY {
                        outside[j] = true;
                        stack.push(j);
                    }
                }
            }
        }
        for i in 0..self.fill.len() {
            if self.owner[i] == EMPTY && !outside[i] {
                self.owner[i] = INTERIOR;
                self.fill[i] = 1.0;
            }
        }
    }

    /// Exposed area of cell i in voxel-face units: how much more solid it is than each
    /// neighbour (a full cell against air counts 1 per face, a wedge about half).
    pub fn exposed_area(&self, i: usize) -> f64 {
        let c = self.cell(i);
        let f = self.fill[i] as f64;
        DIRS.iter()
            .map(|d| {
                let nf = self.index([c[0] + d[0], c[1] + d[1], c[2] + d[2]]).map_or(0.0, |j| self.fill[j] as f64);
                (f - nf).max(0.0)
            })
            .sum()
    }

    pub fn count_filled(&self) -> usize {
        self.owner.iter().filter(|&&o| o != EMPTY).count()
    }
}

pub const DIRS: [[i32; 3]; 6] = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hollow_box_is_solidified() {
        let mut cells = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                for z in 0..3 {
                    if [x, y, z] != [1, 1, 1] {
                        cells.push(([x, y, z], 1.0, 0));
                    }
                }
            }
        }
        let mut g = Grid::from_cells(&cells);
        g.solidify();
        assert_eq!(g.owner[g.index([1, 1, 1]).unwrap()], INTERIOR);
        assert_eq!(g.count_filled(), 27);
        assert_eq!(g.cell(g.index([2, 0, 1]).unwrap()), [2, 0, 1]);
        assert!(!g.filled([3, 0, 0]));
    }
}

/// A wing slab around `start`: spread only within the plane across `axis` (the clicked face's
/// normal), taking each column of `member` cells along `axis` whole. With `thin_only`, a column
/// counts only if it is at most 4 cells thick with open air (EMPTY or outside the grid) at both ends,
/// so the spread stops where a wing meets a fuselage (whose skin has the inside behind it).
pub fn slab_cells(g: &Grid, start: usize, axis: usize, member: &dyn Fn(usize) -> bool, thin_only: bool) -> Vec<usize> {
    let step = |i: usize, s: i32| {
        let mut c = g.cell(i);
        c[axis] += s;
        g.index(c)
    };
    let air = |j: Option<usize>| j.is_none_or(|j| g.owner[j] == EMPTY);
    let mut seen = vec![false; g.owner.len()];
    let mut out = Vec::new();
    let mut queue = std::collections::VecDeque::from([start]);
    while let Some(i) = queue.pop_front() {
        if seen[i] || !member(i) {
            continue;
        }
        // The whole column through i along the axis.
        let mut col = vec![i];
        let mut ends = [None; 2];
        for (k, s) in [1, -1].into_iter().enumerate() {
            let mut j = step(i, s);
            while let Some(x) = j.filter(|&x| member(x)) {
                col.push(x);
                j = step(x, s);
            }
            ends[k] = j;
        }
        for &c in &col {
            seen[c] = true;
        }
        if thin_only && (col.len() > 4 || !air(ends[0]) || !air(ends[1])) {
            continue;
        }
        for &c in &col {
            out.push(c);
            let at = g.cell(c);
            for d in DIRS.iter().filter(|d| d[axis] == 0) {
                if let Some(j) = g.index([at[0] + d[0], at[1] + d[1], at[2] + d[2]]) {
                    if !seen[j] {
                        queue.push_back(j);
                    }
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod slab_tests {
    use super::*;

    #[test]
    fn wing_slab_stops_at_the_fuselage() {
        // 12 x 7 x 3 grid (x span, y up, z chord): a hollow box "fuselage" at x 0..4 (walls of
        // owner 1, inside INTERIOR) and a 1-thick wing plate (owner 2) at y 3 from x 5 to 11 that
        // touches the box wall; the box's top skin is 1 thick too.
        let dims = [12usize, 7, 3];
        let n = dims[0] * dims[1] * dims[2];
        let mut g = Grid { min: [0, 0, 0], dims, fill: vec![0.0; n], owner: vec![EMPTY; n], solids: Vec::new(), missing: Vec::new() };
        for z in 0..3 {
            for y in 0..7 {
                for x in 0..5 {
                    let i = g.index([x, y, z]).unwrap();
                    let wall = x == 0 || x == 4 || y == 0 || y == 6;
                    g.owner[i] = if wall { 1 } else { INTERIOR };
                }
            }
            for x in 5..12 {
                let i = g.index([x, 3, z]).unwrap();
                g.owner[i] = 2;
            }
        }
        let solid = |i: usize| g.owner[i] != EMPTY && g.owner[i] != INTERIOR;
        // Click the wing's top (normal along y): the whole plate, none of the box.
        let start = g.index([8, 3, 1]).unwrap();
        let cells = slab_cells(&g, start, 1, &solid, true);
        assert_eq!(cells.len(), 7 * 3);
        assert!(cells.iter().all(|&c| g.owner[c] == 2));
        // Click the box's top skin: inside behind it, so nothing is a wing slab.
        let top = g.index([2, 6, 1]).unwrap();
        assert!(slab_cells(&g, top, 1, &solid, true).is_empty());
        // Removal (any thickness, marked cells only) clears the whole top skin plane.
        let marked = |i: usize| g.owner[i] == 1 && g.cell(i)[1] == 6;
        assert_eq!(slab_cells(&g, top, 1, &marked, false).len(), 5 * 3);
    }
}

// Chunk-mesh triangles -> voxels, for painting stalled voxels in the game render (paint.rs).

/// Voxel of each triangle: its centroid moved 1/16 m into the block along the face normal, then
/// rounded to the voxel·0.25 grid. Pure so it can be tested.
pub fn triangle_voxels(verts: &[u8], stride: usize, indices: &[u16]) -> Vec<[i32; 3]> {
    let vcount = verts.len() / stride;
    let f = |v: usize, o: usize| f32::from_le_bytes(verts[v * stride + o..v * stride + o + 4].try_into().unwrap()) as f64;
    indices
        .chunks_exact(3)
        .map(|t| {
            if t.iter().any(|&i| i as usize >= vcount) {
                return [i32::MIN; 3];
            }
            let mut c = [0.0; 3];
            let mut n = [0.0; 3];
            for &i in t {
                for k in 0..3 {
                    c[k] += f(i as usize, 4 * k) / 3.0;
                    n[k] += f(i as usize, 0x10 + 4 * k);
                }
            }
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-9);
            std::array::from_fn(|k| ((c[k] - n[k] / len * 0.0625) / 0.25).round() as i32)
        })
        .collect()
}

/// Runs (first index, index count, colour) of consecutive triangles in `[first, first+count)` whose
/// voxel has a colour in `colours`, split where the colour changes.
pub fn colour_runs(tri: &[[i32; 3]], first: u32, count: u32, colours: &std::collections::HashMap<[i32; 3], u32>) -> Vec<(u32, u32, u32)> {
    let (t0, t1) = ((first / 3) as usize, (((first + count) / 3) as usize).min(tri.len()));
    let mut runs: Vec<(u32, u32, u32)> = Vec::new();
    for t in t0..t1 {
        let Some(&col) = colours.get(&tri[t]) else { continue };
        match runs.last_mut() {
            Some(r) if r.0 + r.1 == t as u32 * 3 && r.2 == col => r.1 += 3,
            _ => runs.push((t as u32 * 3, 3, col)),
        }
    }
    runs
}

#[cfg(test)]
mod paint_tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn faces_map_to_their_voxel_and_runs_merge() {
        // Two unit faces: +y face of voxel (1,0,0) and -x face of voxel (-2,3,4).
        let mut verts = Vec::new();
        let mut push = |p: [f32; 3], n: [f32; 3]| {
            for x in p {
                verts.extend(x.to_le_bytes());
            }
            verts.extend([0u8; 4]);
            for x in n {
                verts.extend(x.to_le_bytes());
            }
        };
        for (dx, dz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0)] {
            push([0.25 + dx * 0.125, 0.125, dz * 0.125], [0.0, 1.0, 0.0]);
        }
        for (dy, dz) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0)] {
            push([-0.5 - 0.125, 0.75 + dy * 0.125, 1.0 + dz * 0.125], [-1.0, 0.0, 0.0]);
        }
        let tri = triangle_voxels(&verts, 28, &[0, 1, 2, 3, 4, 5, 0, 1, 2]);
        assert_eq!(tri, vec![[1, 0, 0], [-2, 3, 4], [1, 0, 0]]);
        let set: HashMap<[i32; 3], u32> = [([1, 0, 0], 7)].into();
        assert_eq!(colour_runs(&tri, 0, 9, &set), vec![(0, 3, 7), (6, 3, 7)]);
        let set: HashMap<[i32; 3], u32> = [([1, 0, 0], 7), ([-2, 3, 4], 7)].into();
        assert_eq!(colour_runs(&tri, 0, 9, &set), vec![(0, 9, 7)]);
        assert_eq!(colour_runs(&tri, 3, 6, &set), vec![(3, 6, 7)]);
        let set: HashMap<[i32; 3], u32> = [([1, 0, 0], 7), ([-2, 3, 4], 9)].into();
        assert_eq!(colour_runs(&tri, 0, 9, &set), vec![(0, 3, 7), (3, 3, 9), (6, 3, 7)]);
    }
}

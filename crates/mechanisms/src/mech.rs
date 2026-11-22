//! Mechanism check, as a CAD constraint check: which joints can move at the workbench pose, and
//! which are locked because other joints on the same sub-grids fight them (two pivots whose axes
//! do not line up, a loop of joints that cannot close as it moves).

use std::collections::HashMap;

use crate::defs::Library;
use crate::math::{v3, V3};
use crate::joints::JointLink;
use crate::pose::{body_poses, Pose};
use crate::shape::{voxel_solid, Solid};
use crate::vehicle::Comp;

#[derive(Clone, Debug, PartialEq)]
pub enum Mobility {
    /// Can turn or slide.
    Free,
    /// Cannot move: the other joints of its loop (`by`, indices into the joints) hold it.
    Locked { by: Vec<usize> },
    /// Has no travel by design (a slider with no range: the modded half block offset).
    Fixed,
}

/// Each joint's mobility (`joints`: every joint, loops included, `model::all_joint_links`) with
/// the bodies at `poses` (empty: as built). The first body of each group of jointed bodies stays
/// put; the check is on velocities (instantaneous), so it holds at this pose only.
pub fn mobility(joints: &[JointLink], n_bodies: usize, poses: &[Pose]) -> Vec<Mobility> {
    mobility_basis(joints, n_bodies, poses, None).0
}

/// `mobility`, with the motions the joints allow (a basis, 6 twist values per moving body) and
/// each body's first column in them (None: a fixed body). `fixed`: the bodies that stay put, else
/// the first of each group.
fn mobility_basis(joints: &[JointLink], n_bodies: usize, poses: &[Pose], held: Option<&[bool]>) -> (Vec<Mobility>, Vec<Vec<f64>>, Vec<Option<usize>>) {
    let pose = |b: usize| poses.get(b).copied().unwrap_or(Pose::IDENTITY);
    let fixed = |j: &JointLink| j.slide && j.travel.1 - j.travel.0 < 1e-9;
    // One fixed body per group of jointed bodies: the lowest-numbered one no joint carries.
    let mut group: Vec<usize> = (0..n_bodies).collect();
    fn root(g: &mut [usize], b: usize) -> usize {
        if g[b] != b {
            g[b] = root(g, g[b]);
        }
        g[b]
    }
    for j in joints.iter().filter(|j| j.parent < n_bodies && j.child < n_bodies) {
        let (a, b) = (root(&mut group, j.parent), root(&mut group, j.child));
        group[a.max(b)] = a.min(b);
    }
    let carried = |b: usize| joints.iter().any(|j| j.child == b);
    let mut anchor = vec![usize::MAX; n_bodies];
    for b in 0..n_bodies {
        let r = root(&mut group, b);
        let better = anchor[r] == usize::MAX || (carried(anchor[r]) && !carried(b));
        if better {
            anchor[r] = b;
        }
    }
    let mut col = vec![None; n_bodies];
    let mut n = 0;
    for (b, c) in col.iter_mut().enumerate() {
        let held = held.map_or(anchor[root(&mut group, b)] == b, |f| f.get(b).copied().unwrap_or(false));
        if !held {
            *c = Some(n);
            n += 6;
        }
    }
    // Rows on the body twists (ω, v at the origin): the child's minus the parent's.
    let row = |j: &JointLink, w: V3, v: V3| -> Vec<f64> {
        let mut r = vec![0.0; n];
        for (b, s) in [(j.child, 1.0), (j.parent, -1.0)] {
            if let Some(c) = col.get(b).copied().flatten() {
                for k in 0..3 {
                    r[c + k] += s * w.get(k);
                    r[c + 3 + k] += s * v.get(k);
                }
            }
        }
        r
    };
    let frame = |j: &JointLink| {
        let t = pose(j.parent);
        let (q, a) = (t.apply(j.point), t.dir(j.axis).norm());
        let e1 = if a.x.abs() < 0.9 { a.cross(v3(1.0, 0.0, 0.0)) } else { a.cross(v3(0.0, 1.0, 0.0)) }.norm();
        (q, a, e1, a.cross(e1))
    };
    let turn = |j: &JointLink, e: V3| row(j, e, V3::ZERO);
    let shift = |j: &JointLink, q: V3, e: V3| row(j, q.cross(e), e);
    let axes = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    let mut rows = Vec::new();
    for j in joints {
        let (q, _, e1, e2) = frame(j);
        if fixed(j) {
            rows.extend(axes.iter().map(|&e| turn(j, e)).chain(axes.iter().map(|&e| shift(j, q, e))));
        } else if j.slide {
            rows.extend(axes.iter().map(|&e| turn(j, e)).chain([shift(j, q, e1), shift(j, q, e2)]));
        } else {
            rows.extend([turn(j, e1), turn(j, e2)].into_iter().chain(axes.iter().map(|&e| shift(j, q, e))));
        }
    }
    let free = null_space(rows, n);
    let m = joints
        .iter()
        .enumerate()
        .map(|(i, j)| {
            if fixed(j) {
                return Mobility::Fixed;
            }
            // Between two fixed bodies (both chosen fixed).
            if j.parent != j.child && [j.parent, j.child].iter().all(|&b| col.get(b).is_some_and(|c| c.is_none())) {
                return Mobility::Locked { by: loop_mates(joints, i) };
            }
            let (q, a, _, _) = frame(j);
            let rate = if j.slide { shift(j, q, a) } else { turn(j, a) };
            let size = rate.iter().map(|v| v * v).sum::<f64>().sqrt();
            let moves = free.iter().any(|x| {
                let d: f64 = rate.iter().zip(x).map(|(r, x)| r * x).sum();
                d.abs() > 1e-7 * size * x.iter().map(|v| v * v).sum::<f64>().sqrt()
            });
            if size == 0.0 || moves {
                Mobility::Free
            } else {
                Mobility::Locked { by: loop_mates(joints, i) }
            }
        })
        .collect();
    (m, free, col)
}

/// Each body's mechanism (`usize::MAX` for a fixed body): the bodies joined to each other, not
/// through a fixed one (each door, each landing gear, a crank with its rod and piston). Fixed
/// bodies are the ones `tree` places nothing on, and the ones welded (`joints::welds`).
pub fn mechanisms(tree: &[JointLink], all: &[JointLink], n_bodies: usize) -> Vec<usize> {
    let moving: Vec<bool> = fixed_bodies(tree, all, n_bodies).into_iter().map(|f| !f).collect();
    let mut up: Vec<usize> = (0..n_bodies).collect();
    fn root(g: &mut [usize], b: usize) -> usize {
        if g[b] != b {
            g[b] = root(g, g[b]);
        }
        g[b]
    }
    for j in all.iter().filter(|j| j.parent < n_bodies && j.child < n_bodies && moving[j.parent] && moving[j.child]) {
        let (a, b) = (root(&mut up, j.parent), root(&mut up, j.child));
        up[a.max(b)] = a.min(b);
    }
    let mut ids = HashMap::new();
    (0..n_bodies)
        .map(|b| if moving[b] { let r = root(&mut up, b); let n = ids.len(); *ids.entry(r).or_insert(n) } else { usize::MAX })
        .collect()
}

/// The bodies that stay put: those `tree` places nothing on and those `all` welds.
pub fn fixed_bodies(tree: &[JointLink], all: &[JointLink], n_bodies: usize) -> Vec<bool> {
    (0..n_bodies).map(|b| !tree.iter().any(|j| j.child == b) || all.iter().any(|j| j.comp == crate::joints::WELD && j.child == b)).collect()
}

/// Degrees of freedom of each mechanism (`mech_of`, `mechanisms`) in a basis of allowed motions.
fn dofs(basis: &[Vec<f64>], col: &[Option<usize>], mech_of: &[usize], n_mech: usize) -> Vec<usize> {
    (0..n_mech)
        .map(|m| {
            let cols: Vec<usize> = (0..mech_of.len()).filter(|&b| mech_of[b] == m).filter_map(|b| col.get(b).copied().flatten()).flat_map(|c| c..c + 6).collect();
            let rows: Vec<Vec<f64>> = basis.iter().map(|x| cols.iter().map(|&c| x[c]).collect()).collect();
            // The rank of the motions seen on this mechanism's bodies.
            cols.len() - null_space(rows, cols.len()).len()
        })
        .collect()
}

/// `mobility` at the pose `settings` of `tree`, and also a little way along each joint's travel:
/// at a dead centre (a crank in line with its rod) the first-order check holds a joint that
/// moves as soon as the rest does. A joint free at any of those poses is free.
/// Also each mechanism's degrees of freedom (`mechanisms`), the fewest seen at those poses (a
/// dead centre can show more).
pub fn mobility_near(tree: &[JointLink], all: &[JointLink], n_bodies: usize, settings: &[f64]) -> (Vec<Mobility>, Vec<usize>) {
    let mech_of = mechanisms(tree, all, n_bodies);
    let n_mech = mech_of.iter().filter(|&&m| m != usize::MAX).max().map_or(0, |m| m + 1);
    let fixed = fixed_bodies(tree, all, n_bodies);
    let (mut m, basis, col) = mobility_basis(all, n_bodies, &body_poses(tree, n_bodies, &|i, _| settings[i]), Some(&fixed));
    let mut dof = dofs(&basis, &col, &mech_of, n_mech);
    let loops: Vec<usize> = all.iter().filter(|l| !tree.iter().any(|t| t.comp == l.comp)).flat_map(|l| [l.parent, l.child]).collect();
    let on_loop = crate::pose::above(tree, loops.iter().copied());
    // Whether each joint on a loop really moves: a step each way, the rest of its loops following.
    // A linkage stretched straight (two links as long as the gap between their pivots) passes the
    // first-order check but cannot move at all.
    let mut moves = vec![true; tree.len()];
    for &t in on_loop.iter().take(24) {
        moves[t] = false;
        let step = if tree[t].slide { 0.02 } else { 0.05 };
        for d in [step, -step] {
            if !(tree[t].travel.0..=tree[t].travel.1).contains(&(settings[t] + d)) {
                continue;
            }
            let mut s = settings.to_vec();
            s[t] += d;
            let rest: Vec<usize> = on_loop.iter().copied().filter(|&i| i != t).collect();
            if crate::pose::settle(tree, all, n_bodies, &mut s, &rest, &|_, _| {}) > crate::pose::JAM {
                continue; // cannot move that way: the loop does not close
            }
            moves[t] = true;
            let (there, basis, col) = mobility_basis(all, n_bodies, &body_poses(tree, n_bodies, &|i, _| s[i]), Some(&fixed));
            for (a, b) in m.iter_mut().zip(there) {
                if b == Mobility::Free {
                    *a = Mobility::Free;
                }
            }
            for (a, b) in dof.iter_mut().zip(dofs(&basis, &col, &mech_of, n_mech)) {
                *a = (*a).min(b);
            }
        }
        // Held both ways (two tracks side by side whose travels point opposite ways).
        if let Some(a) = all.iter().position(|l| l.comp == tree[t].comp).filter(|_| !moves[t]) {
            m[a] = Mobility::Locked { by: loop_mates(all, a) };
        }
    }
    // A mechanism none of whose looped joints can move is rigid.
    for (k, d) in dof.iter_mut().enumerate() {
        let looped: Vec<usize> = on_loop.iter().copied().filter(|&t| mech_of.get(tree[t].child) == Some(&k)).collect();
        if !looped.is_empty() && looped.iter().all(|&t| !moves[t]) {
            *d = 0;
            for (i, j) in all.iter().enumerate() {
                if [j.parent, j.child].iter().any(|&b| mech_of.get(b) == Some(&k)) {
                    m[i] = Mobility::Locked { by: loop_mates(all, i) };
                }
            }
        }
    }
    (m, dof)
}

/// The joints that share a loop with joint `i`: its bodies stay joined without it, and so do theirs.
fn loop_mates(joints: &[JointLink], i: usize) -> Vec<usize> {
    let joined = |skip: usize, a: usize, b: usize| {
        let mut seen = vec![a];
        let mut k = 0;
        while k < seen.len() {
            let x = seen[k];
            k += 1;
            for (_, j) in joints.iter().enumerate().filter(|(e, _)| *e != skip) {
                for (p, q) in [(j.parent, j.child), (j.child, j.parent)] {
                    if p == x && !seen.contains(&q) {
                        seen.push(q);
                    }
                }
            }
        }
        seen.contains(&b)
    };
    // Bodies of joint i's two-edge-connected group: reached from its parent without any bridge.
    let bridge = |e: usize| !joined(e, joints[e].parent, joints[e].child);
    let mut group = vec![joints[i].parent];
    let mut k = 0;
    while k < group.len() {
        let x = group[k];
        k += 1;
        for (e, j) in joints.iter().enumerate() {
            for (p, q) in [(j.parent, j.child), (j.child, j.parent)] {
                if p == x && !group.contains(&q) && !bridge(e) {
                    group.push(q);
                }
            }
        }
    }
    (0..joints.len()).filter(|&e| e != i && group.contains(&joints[e].parent) && group.contains(&joints[e].child) && !bridge(e)).collect()
}

/// A basis of the null space of `rows` (each `n` long): the motions every row allows.
fn null_space(mut a: Vec<Vec<f64>>, n: usize) -> Vec<Vec<f64>> {
    let tol = 1e-9 * a.iter().flatten().fold(1.0f64, |m, v| m.max(v.abs()));
    let mut pivots = Vec::new();
    let mut r = 0;
    for c in 0..n {
        if r == a.len() {
            break;
        }
        let Some(p) = (r..a.len()).max_by(|&x, &y| a[x][c].abs().total_cmp(&a[y][c].abs())) else { break };
        if a[p][c].abs() < tol {
            continue;
        }
        a.swap(r, p);
        let d = a[r][c];
        a[r].iter_mut().for_each(|v| *v /= d);
        let pivot = a[r].clone();
        for (i, row) in a.iter_mut().enumerate() {
            let f = row[c];
            if i != r && f != 0.0 {
                row.iter_mut().zip(&pivot).for_each(|(v, p)| *v -= f * p);
            }
        }
        pivots.push(c);
        r += 1;
    }
    (0..n)
        .filter(|c| !pivots.contains(c))
        .map(|f| {
            let mut x = vec![0.0; n];
            x[f] = 1.0;
            for (i, &pc) in pivots.iter().enumerate() {
                x[pc] = -a[i][f];
            }
            x
        })
        .collect()
}

// --- Collision (knowledge/vehicle-model.md "Body collision shapes and filtering"): every body of
// a vehicle collides with every other, the two bodies of a joint included, through the physics
// shapes of the voxels with `flags & 1`. Each is Bullet's shape with a 0.04 m margin: the voxel
// shrunk by 0.042 m, then rounded out by 0.04 m. So each surface sits 2 mm inside its voxel's
// faces (voxels whose faces touch are 4 mm apart) and the edges are rounded. The game's solver
// lets shapes sink `SLOP` into each other before a contact pushes back (contact RE).

/// Bullet's collision margin, and how far inside the voxel the shape's core sits, metres.
const MARGIN: f64 = 0.04;
const CORE: f64 = 0.042;
/// Bullet's `linearSlop`, which the game sets to 0.05 (`physics_world_create` `0x1A36A0`): a contact
/// shallower than this adds no push-out, so a joint turns on until its parts are 5 cm deep.
const SLOP: f64 = 0.05;

/// One convex piece of a part's collision shape, aero frame metres, as built: its core, which
/// the game rounds with `MARGIN`.
#[derive(Clone, Debug)]
pub struct Hull {
    pub comp: u32,
    pub body: usize,
    pts: Vec<V3>,
    normals: Vec<V3>,
    edges: Vec<V3>,
}

fn push_dir(v: &mut Vec<V3>, d: V3) {
    let l = d.len();
    if l > 1e-9 && !v.iter().any(|e| e.dot(d).abs() > l * (1.0 - 1e-9)) {
        v.push(d * (1.0 / l));
    }
}

impl Hull {
    /// From a placed solid in grid units (`voxel::place`).
    pub fn from_solid(comp: u32, body: usize, s: &Solid) -> Hull {
        let to_aero = |p: V3| v3(p.z * 0.25, p.x * 0.25, -p.y * 0.25);
        let mut h = Hull { comp, body, pts: Vec::new(), normals: Vec::new(), edges: Vec::new() };
        for f in s.iter().filter(|f| f.len() >= 3) {
            let f: Vec<V3> = f.iter().map(|p| to_aero(*p)).collect();
            push_dir(&mut h.normals, (f[1] - f[0]).cross(f[2] - f[0]));
            for (k, p) in f.iter().enumerate() {
                push_dir(&mut h.edges, f[(k + 1) % f.len()] - *p);
                if !h.pts.iter().any(|q| (*q - *p).len() < 1e-9) {
                    h.pts.push(*p);
                }
            }
        }
        h
    }

    fn moved(&self, t: &Pose) -> Hull {
        Hull { comp: self.comp, body: self.body, pts: self.pts.iter().map(|p| t.apply(*p)).collect(), normals: self.normals.iter().map(|n| t.dir(*n)).collect(), edges: self.edges.iter().map(|e| t.dir(*e)).collect() }
    }

    fn bounds(&self) -> (V3, V3) {
        let lo = self.pts.iter().fold(v3(f64::MAX, f64::MAX, f64::MAX), |a, p| v3(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z)));
        let hi = self.pts.iter().fold(v3(f64::MIN, f64::MIN, f64::MIN), |a, p| v3(a.x.max(p.x), a.y.max(p.y), a.z.max(p.z)));
        (lo, hi)
    }

    /// Whether the rounded shapes are at least `SLOP` deep in each other. Separating axis test on
    /// the cores, plus the margin. Conservative where two rounded corners meet (the true
    /// separating direction is not among the axes).
    fn hits(&self, o: &Hull) -> bool {
        self.depth(o, SLOP) >= SLOP
    }

    /// How deep the rounded shapes are in each other, metres: the least overlap over the
    /// separating axes (below 0: apart). Stops early once below `enough`.
    fn depth(&self, o: &Hull, enough: f64) -> f64 {
        let over = |ax: V3| {
            let l = ax.len();
            if l < 1e-9 {
                return f64::INFINITY;
            }
            let ax = ax * (1.0 / l);
            let span = |pts: &[V3]| pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.dot(ax)), hi.max(p.dot(ax))));
            let ((a0, a1), (b0, b1)) = (span(&self.pts), span(&o.pts));
            // How far to push either way to part them: a thin part right through a block is deep.
            (a1 - b0).min(b1 - a0) + 2.0 * MARGIN
        };
        let mut d = f64::INFINITY;
        for ax in self.normals.iter().chain(&o.normals).copied().chain(self.edges.iter().flat_map(|&e| o.edges.iter().map(move |&f| e.cross(f)))) {
            d = d.min(over(ax));
            if d < enough {
                break;
            }
        }
        d
    }
}

/// The game's cut plane of each physics shape id 1..=41 (`physics_cell_ctor` 0x473D00): normal
/// and a point on it, in a lattice where the cell is [-1, 1]^3 (knowledge/vehicle-model.md "Cut
/// plane").
const CUTS: [([i32; 3], [i32; 3]); 41] = [
    ([0, 1, -1], [-1, -1, -1]),
    ([-1, 1, -1], [1, -1, -1]),
    ([-1, 1, -1], [-1, -1, -1]),
    ([0, 2, -1], [-1, -1, -3]),
    ([0, 2, -1], [-1, -1, -1]),
    ([0, 4, -1], [-1, -1, -7]),
    ([0, 4, -1], [-1, -1, -5]),
    ([0, 4, -1], [-1, -1, -3]),
    ([0, 4, -1], [-1, -1, -1]),
    ([-2, 2, -1], [1, -1, -3]),
    ([-2, 2, -1], [1, -1, -1]),
    ([-4, 4, -1], [1, -1, -7]),
    ([-4, 4, -1], [1, -1, -5]),
    ([-4, 4, -1], [1, -1, -3]),
    ([-4, 4, -1], [1, -1, -1]),
    ([-1, 2, -1], [1, -1, -3]),
    ([-1, 2, -1], [1, -1, -1]),
    ([-2, 4, -1], [1, -1, -7]),
    ([-2, 4, -1], [1, -1, -5]),
    ([-2, 4, -1], [1, -1, -3]),
    ([-2, 4, -1], [1, -1, -1]),
    ([-1, 4, -1], [1, -1, -7]),
    ([-1, 4, -1], [1, -1, -5]),
    ([-1, 4, -1], [1, -1, -3]),
    ([-1, 4, -1], [1, -1, -1]),
    ([-2, 2, -1], [-1, -1, -3]),
    ([-2, 2, -1], [-1, -1, -1]),
    ([-4, 4, -1], [-1, -1, -7]),
    ([-4, 4, -1], [-1, -1, -5]),
    ([-4, 4, -1], [-1, -1, -3]),
    ([-4, 4, -1], [-1, -1, -1]),
    ([-1, 2, -1], [-1, -1, -3]),
    ([-1, 2, -1], [-1, -1, -1]),
    ([-2, 4, -1], [-1, -1, -7]),
    ([-2, 4, -1], [-1, -1, -5]),
    ([-2, 4, -1], [-1, -1, -3]),
    ([-2, 4, -1], [-1, -1, -1]),
    ([-1, 4, -1], [-1, -1, -7]),
    ([-1, 4, -1], [-1, -1, -5]),
    ([-1, 4, -1], [-1, -1, -3]),
    ([-1, 4, -1], [-1, -1, -1]),
];

/// The cut plane of shape `id` in the vehicle grid (`n·p <= d`, voxel units, cell centre origin),
/// as the game makes it: through the voxel's matrix `a6` then the part's `rot` (mirror folded in),
/// raw integers, so a stretched part skews the plane but never the cell.
fn cut_plane(id: i32, a6: &[i32; 9], rot: &[i32; 9]) -> Option<crate::shape::Plane> {
    let (n, p) = CUTS.get(usize::try_from(id).ok()?.checked_sub(1)?)?;
    let mul = |v: [i32; 3], m: &[i32; 9]| [0, 1, 2].map(|k| v[0] * m[k] + v[1] * m[3 + k] + v[2] * m[6 + k]);
    let (n, p) = (mul(mul(*n, a6), rot), mul(mul(*p, a6), rot));
    // The point from the cell's low corner, (P + 1) / 2 as the game's C division rounds it.
    let q = p.map(|v| ((v + 1) / 2) as f64 - 0.5);
    let n = v3(n[0] as f64, n[1] as f64, n[2] as f64);
    Some(crate::shape::Plane { n, d: n.dot(v3(q[0], q[1], q[2])) })
}

/// The 1:4 slopes, whose cut plane the game does not move in (`physics_cells_build_records`).
fn flush_cut(id: i32) -> bool {
    matches!(id, 6..=9 | 12..=15 | 18..=25 | 28..=31)
}

/// Every part's collision pieces as the game builds them when the vehicle spawns
/// (knowledge/vehicle-model.md "Body collision shapes"): each `flags & 1` voxel a unit cell at
/// its grid cell (the part's matrix moves it but never stretches it), cut by its plane; as the
/// core inside the `MARGIN` (planes moved in by `CORE`), the way `Hull::depth` takes them.
/// shortcut: cells are not merged into records as the game does; that only drops the 4 mm seams
/// and rounded edges between cells of one body, which never decide a 5 cm contact.
pub fn hulls(comps: &[Comp], lib: &mut Library) -> Vec<Hull> {
    const IN: f64 = CORE / crate::voxel::VOXEL_M;
    let core = |plane: Option<crate::shape::Plane>, flush: bool| {
        let k = 1.0 - 2.0 * IN;
        let cube: Solid = voxel_solid(None).into_iter().map(|f| f.into_iter().map(|p| p * k).collect()).collect();
        match plane {
            Some(p) => crate::shape::clip_solid(&cube, &crate::shape::Plane { n: p.n, d: p.d - if flush { 0.0 } else { IN * p.n.len() } }),
            None => cube,
        }
    };
    let mut out = Vec::new();
    for (i, c) in comps.iter().enumerate() {
        let (comp, body) = (i as u32, c.body as usize);
        let at = |g: [i32; 3]| Comp { pos: g, rot: crate::vehicle::IDENTITY, ..c.clone() };
        let Some(def) = lib.get(&c.def) else {
            out.push(Hull::from_solid(comp, body, &crate::voxel::place(&core(None, false), &at(c.pos))));
            continue;
        };
        // A resized microcontroller repeats its voxel over width x length cells.
        let [w, l] = c.size.map_or([1, 1], |s| s.map(|n| n.clamp(1, 16)));
        for v in &def.collision {
            let plane = cut_plane(v.shape, &v.rot, &c.rot);
            let solid = core(plane, flush_cut(v.shape));
            if solid.len() < 4 {
                continue; // the cut leaves no core: the game's hull has no points either
            }
            for (x, z) in (0..w).flat_map(|x| (0..l).map(move |z| (x, z))) {
                let g = c.cell([v.pos[0] + x, v.pos[1], v.pos[2] + z]);
                out.push(Hull::from_solid(comp, body, &crate::voxel::place(&solid, &at(g))));
            }
        }
        if let Some((lo, hi)) = def.collision_box {
            // Part frame metres -> voxel units; Bullet's box keeps its margin inside the extents.
            let m = MARGIN / crate::voxel::VOXEL_M;
            let (a, b) = (lo * 4.0 + v3(m, m, m), hi * 4.0 - v3(m, m, m));
            if a.x < b.x && a.y < b.y && a.z < b.z {
                let corner = |k: usize| v3(if k & 1 == 0 { a.x } else { b.x }, if k & 2 == 0 { a.y } else { b.y }, if k & 4 == 0 { a.z } else { b.z });
                let faces = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
                let local: Solid = faces.iter().map(|f| f.iter().map(|&k| corner(k)).collect()).collect();
                // The game turns it by the part's raw matrix, stretched or not.
                out.push(Hull::from_solid(comp, body, &crate::voxel::place(&local, c)));
            }
        }
    }
    out
}

/// Two parts in contact: one on the moving sub-grids, one elsewhere.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Contact {
    pub moving: u32,
    pub other: u32,
}

/// How far a joint moves one way from its setting: to the setting `to` (as `body_poses` takes
/// it), where it first touches `hit`, or its end of travel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reach {
    pub to: f64,
    pub hit: Option<Contact>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Sweep {
    /// Contacts at the current setting (left out of the sweep: they stay in contact).
    pub now: Vec<Contact>,
    pub down: Reach,
    pub up: Reach,
    /// Turns without limit and touches nothing all the way round.
    pub all_round: bool,
}

/// Sweeps posing joint `j` (`tree`, `model::joint_links`) from its setting both ways with the
/// other joints held at theirs (`settings`), against the parts it does not carry.
pub fn sweep(tree: &[JointLink], j: usize, settings: &[f64], n_bodies: usize, hulls: &[Hull]) -> Sweep {
    let joint = &tree[j];
    let mut moving = vec![false; n_bodies];
    moving[joint.child] = true;
    for l in tree {
        if moving[l.parent] {
            moving[l.child] = true;
        }
    }
    let at = |v: f64| body_poses(tree, n_bodies, &|i, _| if i == j { v } else { settings[i] });
    let cur = settings[j];
    let poses = at(cur);
    // The rest, posed, in a hash of 0.5 m cells.
    const CELL: f64 = 0.5;
    let cell = |p: V3| [p.x, p.y, p.z].map(|v| (v / CELL).floor() as i32);
    let rest: Vec<Hull> = hulls.iter().filter(|h| !moving[h.body]).map(|h| h.moved(&poses[h.body])).collect();
    let mut grid: HashMap<[i32; 3], Vec<u32>> = HashMap::new();
    for (k, h) in rest.iter().enumerate() {
        let (lo, hi) = h.bounds();
        let (a, b) = (cell(lo), cell(hi));
        for x in a[0]..=b[0] {
            for y in a[1]..=b[1] {
                for z in a[2]..=b[2] {
                    grid.entry([x, y, z]).or_default().push(k as u32);
                }
            }
        }
    }
    let carried: Vec<&Hull> = hulls.iter().filter(|h| moving[h.body]).collect();
    let mut stamp = vec![usize::MAX; rest.len()];
    let mut pass = 0;
    let mut contacts = |v: f64, skip: &[Contact], first: bool| -> Vec<Contact> {
        let poses = at(v);
        let mut out = Vec::new();
        for h in &carried {
            let m = h.moved(&poses[h.body]);
            let (lo, hi) = m.bounds();
            let (a, b) = (cell(lo), cell(hi));
            pass += 1;
            for x in a[0]..=b[0] {
                for y in a[1]..=b[1] {
                    for z in a[2]..=b[2] {
                        for &k in grid.get(&[x, y, z]).map(Vec::as_slice).unwrap_or_default() {
                            if stamp[k as usize] == pass {
                                continue;
                            }
                            stamp[k as usize] = pass;
                            let c = Contact { moving: h.comp, other: rest[k as usize].comp };
                            if !skip.contains(&c) && !out.contains(&c) && m.hits(&rest[k as usize]) {
                                out.push(c);
                                if first {
                                    return out;
                                }
                            }
                        }
                    }
                }
            }
        }
        out
    };
    let now = contacts(cur, &[], false);
    // Steps that move no carried point more than 3 cm.
    let parent = poses[joint.parent];
    let (q, a) = (parent.apply(joint.point), parent.dir(joint.axis).norm());
    let reach = carried.iter().flat_map(|h| h.moved(&poses[h.body]).pts).map(|p| (p - q).reject(a).len()).fold(0.1f64, f64::max);
    let step = if joint.slide { 0.03 } else { 0.03 / reach };
    let all_round = !joint.slide && joint.travel.1 - joint.travel.0 >= 2.0 * std::f64::consts::PI - 1e-6;
    let mut go = |end: f64| -> Reach {
        let dir = (end - cur).signum();
        let mut prev = cur;
        while (end - prev) * dir > 1e-12 {
            let next = if (end - prev).abs() <= step { end } else { prev + dir * step };
            if let Some(&c) = contacts(next, &now, true).first() {
                // Narrow down the first contact.
                let (mut free, mut hit, mut what) = (prev, next, c);
                for _ in 0..12 {
                    let mid = 0.5 * (free + hit);
                    match contacts(mid, &now, true).first() {
                        Some(&c) => (hit, what) = (mid, c),
                        None => free = mid,
                    }
                }
                return Reach { to: free, hit: Some(what) };
            }
            prev = next;
        }
        Reach { to: end, hit: None }
    };
    let (lo, hi) = if all_round { (cur - std::f64::consts::PI, cur + std::f64::consts::PI) } else { (joint.travel.0.min(cur), joint.travel.1.max(cur)) };
    let (down, up) = (go(lo), go(hi));
    let all_round = all_round && down.hit.is_none() && up.hit.is_none();
    Sweep { now, down, up, all_round }
}

// --- Sequence frames: free joints held by loops and by contact.

/// Poses sequence frames as the game would: driven joints at their values, free ones moved so
/// loops stay joined and their parts stay out of the rest (a piston between its cylinder walls).
/// Goes in small steps from the last frame, so parts cannot jump through each other, and stops
/// where a loop can go no further or a moving part is as deep in another as the game lets it
/// sink (`SLOP`, the depth Check uses).
#[derive(Default)]
pub struct Settler {
    last: Vec<f64>,
    /// Hull pairs already in each other at the workbench pose: built that way, left alone.
    built: std::collections::HashSet<(usize, usize)>,
    /// Driven joints (`tree`) held short of the last frame: their mechanism can move no further.
    /// shortcut: one stop holds every driven joint, other mechanisms included; settle each
    /// mechanism on its own if sequences moving several at once need it.
    pub stuck: Vec<usize>,
    /// How far the last frame got before it stopped, 0..1 of the way from the frame before.
    pub stop: Option<f64>,
    /// The parts (components) that stopped it, the moving one first; none: a loop did.
    pub hit: Option<(u32, u32)>,
    /// Parts pass through each other: no contacts at all.
    pub ignore: bool,
}

impl Settler {
    /// Start again from the workbench pose (a new analysis, the sequence back at its start).
    pub fn reset(&mut self) {
        self.last.clear();
    }

    /// The last frame's joint settings (`tree`), if any since `reset`.
    pub fn pose(&self) -> Option<&[f64]> {
        (!self.last.is_empty()).then_some(&self.last[..])
    }

    pub fn frame(&mut self, r: &Report, target: &[f64]) -> Vec<f64> {
        let tree = &r.tree;
        (self.stop, self.hit) = (None, None);
        self.stuck.clear();
        // The bodies carried by the joints `own` picks.
        let carried = |own: &dyn Fn(usize) -> bool| {
            let mut m = vec![false; r.n_bodies];
            for (i, j) in tree.iter().enumerate() {
                if j.child < r.n_bodies {
                    m[j.child] = own(i) || m.get(j.parent).copied().unwrap_or(false);
                }
            }
            m
        };
        let free_moved = carried(&|i| tree[i].free);
        // Pairs of bodies joined to each other: a free joint lets those sink into each other.
        let joined: std::collections::HashSet<(usize, usize)> = r.links.iter().flat_map(|j| [(j.parent, j.child), (j.child, j.parent)]).collect();
        // Hull pairs near each other, one of them on a `moved` body.
        let pairs = |s: &[f64], moved: &[bool]| -> Vec<(usize, usize)> {
            let poses = body_poses(tree, r.n_bodies, &|i, _| s[i]);
            let at = |b: usize| poses.get(b).copied().unwrap_or(Pose::IDENTITY);
            const CELL: f64 = 0.5;
            let cell = |p: V3| [p.x, p.y, p.z].map(|v| (v / CELL).floor() as i32);
            let bounds: Vec<(V3, V3)> = r.hulls.iter().map(|h| h.moved(&at(h.body)).bounds()).collect();
            let mut grid: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
            for (k, (lo, hi)) in bounds.iter().enumerate() {
                let (a, b) = (cell(*lo), cell(*hi));
                for x in a[0]..=b[0] {
                    for y in a[1]..=b[1] {
                        for z in a[2]..=b[2] {
                            grid.entry([x, y, z]).or_default().push(k);
                        }
                    }
                }
            }
            let mut out = std::collections::BTreeSet::new();
            for (i, h) in r.hulls.iter().enumerate().filter(|(_, h)| moved.get(h.body).copied().unwrap_or(false)) {
                let pad = v3(0.1, 0.1, 0.1);
                let (lo, hi) = (bounds[i].0 - pad, bounds[i].1 + pad);
                let (a, b) = (cell(lo), cell(hi));
                for x in a[0]..=b[0] {
                    for y in a[1]..=b[1] {
                        for z in a[2]..=b[2] {
                            for &k in grid.get(&[x, y, z]).map(Vec::as_slice).unwrap_or_default() {
                                let o = &r.hulls[k];
                                let near = (0..3).all(|c| bounds[k].0.get(c) <= hi.get(c) && bounds[k].1.get(c) >= lo.get(c));
                                if o.body != h.body && near && (!moved[o.body] || i < k) {
                                    out.insert((i, k));
                                }
                            }
                        }
                    }
                }
            }
            out.into_iter().collect()
        };
        let depth = |p: &[Pose], (i, k): (usize, usize)| {
            let (a, b) = (&r.hulls[i], &r.hulls[k]);
            a.moved(&p[a.body]).depth(&b.moved(&p[b.body]), 0.0)
        };
        if self.last.len() != tree.len() {
            self.last = r.settings.clone();
            let poses = body_poses(tree, r.n_bodies, &|i, _| r.settings[i]);
            self.built = pairs(&r.settings, &carried(&|_| true)).into_iter().filter(|&pk| depth(&poses, pk) > 0.0).collect();
        }
        // A joint without stops looping round (360° back to 0°): the same pose, so no turn back.
        for i in (0..tree.len()).filter(|&i| !tree[i].slide && tree[i].travel.1.is_infinite()) {
            let tau = 2.0 * std::f64::consts::PI;
            self.last[i] += tau * ((target[i] - self.last[i]) / tau).round();
        }
        if self.last[..] == target[..] {
            return self.last.clone();
        }
        let (built, ignore) = (&self.built, self.ignore);
        let mut hit = None;
        let (s, stop) = crate::pose::march(tree, &self.last, target, &mut |s| {
            let moved = carried(&|i| tree[i].free || (s[i] - r.settings[i]).abs() > 1e-12);
            let near: Vec<(usize, usize)> = if ignore { Vec::new() } else { pairs(s, &moved).into_iter().filter(|pk| !built.contains(pk)).collect() };
            // Free joints push their parts out of anything but the body they are joined to.
            let push: Vec<(usize, usize)> = (near.iter().copied())
                .filter(|&(i, k)| {
                    let (a, b) = (r.hulls[i].body, r.hulls[k].body);
                    (free_moved[a] || free_moved[b]) && !joined.contains(&(a, b))
                })
                .collect();
            let loops = r.links.iter().filter(|l| !tree.iter().any(|t| t.comp == l.comp)).flat_map(|l| [l.parent, l.child]);
            let free = crate::pose::free_above(tree, loops.chain(push.iter().flat_map(|&(i, k)| [r.hulls[i].body, r.hulls[k].body])));
            let e = crate::pose::settle(tree, &r.links, r.n_bodies, s, &free, &|p, out| out.extend(push.iter().map(|&pk| depth(p, pk).max(0.0))));
            if e > crate::pose::JAM {
                hit = None;
                return e;
            }
            let p = body_poses(tree, r.n_bodies, &|i, _| s[i]);
            let deep = |&&(i, k): &&(usize, usize)| r.hulls[i].moved(&p[r.hulls[i].body]).hits(&r.hulls[k].moved(&p[r.hulls[k].body]));
            match near.iter().find(deep) {
                Some(&(i, k)) => {
                    hit = Some((r.hulls[i].comp, r.hulls[k].comp));
                    f64::INFINITY
                }
                None => e,
            }
        });
        // The driven joints a stop holds short of the sequence.
        self.stuck = if stop.is_some() { (0..tree.len()).filter(|&i| !tree[i].free && (s[i] - target[i]).abs() > 1e-9).collect() } else { Vec::new() };
        (self.stop, self.hit) = (stop, hit);
        self.last = s;
        self.last.clone()
    }
}

// --- Report: what the Mechanisms mode shows for one state of the vehicle.

pub struct Report {
    /// Posing joints (`model::joint_links`) and every joint (`model::all_joint_links`).
    pub tree: Vec<JointLink>,
    pub all: Vec<JointLink>,
    /// `all` and the welds holding a group's other fixed bodies (`joints::welds`): what the
    /// solver closes. Per body: it stays put (`fixed_bodies`).
    pub links: Vec<JointLink>,
    pub fixed: Vec<bool>,
    pub n_bodies: usize,
    /// Each posing joint's workbench setting, where its loops let it get to it (`settings`), and
    /// the bodies posed by them.
    pub wanted: Vec<f64>,
    pub settings: Vec<f64>,
    pub poses: Vec<Pose>,
    /// Per joint of `all`, at the workbench pose.
    pub mobility: Vec<Mobility>,
    /// Each body's mechanism (`mechanisms`), and each mechanism's degrees of freedom.
    pub mech_of: Vec<usize>,
    pub dof: Vec<usize>,
    pub hulls: Vec<Hull>,
    /// Per posing joint, once checked: None for a joint that is locked or in a loop.
    pub sweeps: Option<Vec<Option<Sweep>>>,
}

/// How a joint's result reads at a glance (the tooltip title and the paint).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Free,
    /// Touches something now, or stops before its end of travel.
    Hits,
    Locked,
}

impl Report {
    pub fn new(comps: &[Comp], lib: &mut Library, des: &crate::marks::Designations, check: bool) -> Report {
        let all = crate::joints::all_joint_links(comps, lib);
        // The sub-grids the user fixed (a part of each marked), lowest first.
        let mut chosen: Vec<usize> = comps.iter().filter(|c| des.fixed.contains(&c.pos)).map(|c| c.body as usize).collect();
        chosen.sort_unstable();
        chosen.dedup();
        let tree = crate::joints::tree_links(all.clone(), comps, &chosen);
        let links: Vec<JointLink> = all.iter().cloned().chain(crate::joints::welds(&tree, &chosen)).collect();
        let n_bodies = crate::pose::body_count(comps);
        let fixed = fixed_bodies(&tree, &links, n_bodies);
        let wanted = crate::pose::joint_settings(&tree, comps, &des.joint_angles, &des.joint_offsets);
        let (settings, _) = crate::pose::workbench_settings(&tree, &links, n_bodies, &wanted);
        let poses = body_poses(&tree, n_bodies, &|i, _| settings[i]);
        let (mut mobility, dof) = mobility_near(&tree, &links, n_bodies, &settings);
        mobility.truncate(all.len());
        let mech_of = mechanisms(&tree, &links, n_bodies);
        let hulls = hulls(comps, lib);
        let mut r = Report { tree, all, links, fixed, n_bodies, wanted, settings, poses, mobility, mech_of, dof, hulls, sweeps: None };
        if check {
            r.sweeps = Some((0..r.tree.len()).map(|t| r.sweepable(t).then(|| sweep(&r.tree, t, &r.settings, r.n_bodies, &r.hulls))).collect());
        }
        r
    }

    /// `all` index of posing joint `t`.
    fn joint_of(&self, t: usize) -> Option<usize> {
        self.all.iter().position(|j| j.comp == self.tree[t].comp)
    }

    /// Free and not in a loop (the tree pose would pull a loop apart).
    fn sweepable(&self, t: usize) -> bool {
        self.joint_of(t).is_some_and(|a| self.mobility[a] == Mobility::Free && loop_mates(&self.all, a).is_empty())
    }

    /// Posing joint `t` is held short of its workbench setting by its loop.
    fn short(&self, t: usize) -> bool {
        (self.settings[t] - self.wanted[t]).abs() > 1e-6
    }

    pub fn status(&self, a: usize) -> Status {
        match &self.mobility[a] {
            Mobility::Locked { .. } => Status::Locked,
            Mobility::Fixed => Status::Free,
            Mobility::Free => {
                let t = self.tree.iter().position(|l| l.comp == self.all[a].comp);
                if t.is_some_and(|t| self.short(t)) {
                    return Status::Hits;
                }
                let s = t.and_then(|t| self.sweeps.as_ref()?.get(t)?.as_ref());
                if s.is_some_and(|s| !s.now.is_empty() || s.down.hit.is_some() || s.up.hit.is_some()) {
                    Status::Hits
                } else {
                    Status::Free
                }
            }
        }
    }

    /// The vehicle's box as built, aero frame (zero without collision pieces).
    pub fn extent(&self) -> (V3, V3) {
        let all = self.hulls.iter().map(Hull::bounds);
        let lo = all.clone().map(|b| b.0).reduce(|a, p| v3(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z))).unwrap_or(V3::ZERO);
        let hi = all.map(|b| b.1).reduce(|a, p| v3(a.x.max(p.x), a.y.max(p.y), a.z.max(p.z))).unwrap_or(V3::ZERO);
        (lo, hi)
    }

    /// Where part `c` shows: the middle of its collision pieces, posed.
    pub fn anchor(&self, c: usize) -> Option<V3> {
        let hs: Vec<&Hull> = self.hulls.iter().filter(|h| h.comp as usize == c).collect();
        let pts: Vec<V3> = hs.iter().flat_map(|h| h.pts.iter().copied()).collect();
        let mid = pts.iter().fold(V3::ZERO, |a, p| a + *p) * (1.0 / pts.len().max(1) as f64);
        Some(self.poses.get(hs.first()?.body).map_or(mid, |t| t.apply(mid)))
    }

    /// Where joint `j` shows: its part, or its other half when the part has no collision pieces
    /// (a micro pivot's base).
    pub fn joint_anchor(&self, j: &JointLink) -> Option<V3> {
        self.anchor(j.comp).or_else(|| self.anchor(j.half))
    }

    /// The joint a part belongs to: one of its halves, or a part its sub-grid carries.
    pub fn joint_at(&self, comps: &[Comp], c: usize) -> Option<usize> {
        let body = comps.get(c)?.body as usize;
        self.all.iter().position(|j| j.comp == c || j.half == c).or_else(|| {
            let t = self.tree.iter().position(|l| l.child == body)?;
            self.joint_of(t)
        })
    }

    /// Parts to paint: in contact or stopping a joint (`Hits`), or the halves of a locked joint.
    pub fn paint(&self) -> Vec<(u32, Status)> {
        let mut out = Vec::new();
        for (a, j) in self.all.iter().enumerate() {
            if matches!(self.mobility[a], Mobility::Locked { .. }) {
                out.extend([(j.comp as u32, Status::Locked), (j.half as u32, Status::Locked)]);
            }
        }
        for s in self.sweeps.iter().flatten().flatten() {
            for c in s.now.iter().chain(s.down.hit.iter()).chain(s.up.hit.iter()) {
                out.extend([(c.moving, Status::Hits), (c.other, Status::Hits)]);
            }
        }
        out
    }

    /// Tooltip title and text for part `c`: its joint's result, or what it touches.
    pub fn describe(&self, comps: &[Comp], name: &dyn Fn(usize) -> String, c: usize) -> Option<(String, String, Status)> {
        let Some(a) = self.joint_at(comps, c) else {
            // A part on no sub-grid: what touches it.
            let touching: Vec<&Contact> = self.sweeps.iter().flatten().flatten().flat_map(|s| s.now.iter()).filter(|k| k.other as usize == c).collect();
            let k = touching.first()?;
            return Some((name(c), format!("Touches {}", name(k.moving as usize)), Status::Hits));
        };
        let j = &self.all[a];
        let title = name(j.comp);
        let text = match &self.mobility[a] {
            Mobility::Fixed => "Fixed".to_string(),
            Mobility::Locked { .. } => "Locked".to_string(),
            Mobility::Free => match self.tree.iter().position(|l| l.comp == j.comp) {
                Some(t) if self.short(t) => {
                    let l = &self.tree[t];
                    let v = l.sign * self.settings[t];
                    format!("Stops at {}", if l.slide { format!("{v:+.2} m") } else { format!("{:+.0}°", v.to_degrees()) })
                }
                Some(t) => match self.sweeps.as_ref().and_then(|s| s.get(t)?.as_ref()) {
                    Some(s) => self.sweep_text(t, s, name),
                    None => "Free".to_string(),
                },
                None => "Free".to_string(),
            },
        };
        Some((title, text, self.status(a)))
    }

    fn sweep_text(&self, t: usize, s: &Sweep, name: &dyn Fn(usize) -> String) -> String {
        let j = &self.tree[t];
        let fmt = |v: f64| if j.slide { format!("{:+.2} m", j.sign * v) } else { format!("{:+.0}°", (j.sign * v).to_degrees()) };
        let mut stops: Vec<(f64, Contact)> = [s.down, s.up].iter().filter_map(|r| Some((r.to, r.hit?))).collect();
        stops.sort_by(|x, y| (j.sign * x.0).total_cmp(&(j.sign * y.0)));
        let mut lines: Vec<String> = stops.iter().map(|(at, c)| format!("Hits {} at {}", name(c.other as usize), fmt(*at))).collect();
        lines.extend(s.now.first().map(|c| format!("Touches {}", name(c.other as usize))));
        if lines.is_empty() {
            lines.push("Free".to_string());
        }
        lines.join("\n")
    }
}

/// The part under the mouse: `at` is the view position (x right, y down, −1..1), `clip` the
/// camera's aero frame → clip space matrix (row vectors, `editor::WORLD_CLIP`), the hulls posed
/// by `poses`. Only parts with a collision shape can be picked.
pub fn pick(hulls: &[Hull], poses: &[Pose], clip: &[f64; 16], at: [f64; 2]) -> Option<u32> {
    let inv = invert4(clip)?;
    let un = |z: f64| {
        let v = [at[0], -at[1], z, 1.0];
        let r: [f64; 4] = std::array::from_fn(|k| (0..4).map(|i| v[i] * inv[i * 4 + k]).sum());
        (r[3].abs() > 1e-12).then(|| v3(r[0] / r[3], r[1] / r[3], r[2] / r[3]))
    };
    let (o, far) = (un(-1.0)?, un(1.0)?);
    let d = far - o;
    let mut best: Option<(f64, u32)> = None;
    for h in hulls {
        let h = h.moved(&poses.get(h.body).copied().unwrap_or(Pose::IDENTITY));
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        for n in h.normals.iter() {
            let (lo, hi) = h.pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.dot(*n)), hi.max(p.dot(*n))));
            let (on, dn) = (o.dot(*n), d.dot(*n));
            if dn.abs() < 1e-12 {
                if on < lo || on > hi {
                    t1 = -1.0;
                    break;
                }
                continue;
            }
            let (a, b) = ((lo - on) / dn, (hi - on) / dn);
            t0 = t0.max(a.min(b));
            t1 = t1.min(a.max(b));
            if t0 > t1 {
                break;
            }
        }
        if t0 <= t1 && best.is_none_or(|(t, _)| t0 < t) {
            best = Some((t0, h.comp));
        }
    }
    best.map(|b| b.1)
}

fn invert4(m: &[f64; 16]) -> Option<[f64; 16]> {
    let mut a = *m;
    let mut inv = [0.0; 16];
    for i in 0..4 {
        inv[i * 5] = 1.0;
    }
    for c in 0..4 {
        let p = (c..4).max_by(|&x, &y| a[x * 4 + c].abs().total_cmp(&a[y * 4 + c].abs()))?;
        if a[p * 4 + c].abs() < 1e-15 {
            return None;
        }
        for k in 0..4 {
            a.swap(c * 4 + k, p * 4 + k);
            inv.swap(c * 4 + k, p * 4 + k);
        }
        let d = a[c * 4 + c];
        for k in 0..4 {
            a[c * 4 + k] /= d;
            inv[c * 4 + k] /= d;
        }
        for r in (0..4).filter(|&r| r != c) {
            let f = a[r * 4 + c];
            for k in 0..4 {
                a[r * 4 + k] -= f * a[c * 4 + k];
                inv[r * 4 + k] -= f * inv[c * 4 + k];
            }
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn hinge(parent: usize, child: usize, point: V3, axis: V3) -> JointLink {
        JointLink { comp: 0, half: 0, parent, child, point, axis, sign: 1.0, slide: false, travel: (-3.0, 3.0), free: false, settle: V3::ZERO }
    }

    /// A voxel's collision core (`hulls`) in grid units, centred on aero point `p`.
    fn cube(p: V3) -> Solid {
        let o = v3(p.y / 0.25, -p.z / 0.25, p.x / 0.25);
        let k = 1.0 - 2.0 * CORE / 0.25;
        voxel_solid(None).into_iter().map(|f| f.into_iter().map(|q| q * k + o).collect()).collect()
    }

    #[test]
    #[ignore = "needs the user's saved vehicles"]
    fn dump_named_vehicle() {
        let (mut lib, appdata) = crate::defs::test_library();
        let name = std::env::var("MECH_VEHICLE").unwrap_or_default();
        let Ok(text) = std::fs::read_to_string(appdata.join(format!("data/vehicles/{name}.xml"))) else { return };
        let comps = crate::vehicle::parse_vehicle_xml(&text);
        let r = Report::new(&comps, &mut lib, &crate::marks::Designations::default(), false);
        for j in &r.tree {
            println!("tree {} {} -> {} free {} slide {} travel {:?}", comps[j.comp].def, j.parent, j.child, j.free, j.slide, j.travel);
        }
        for (a, (j, m)) in r.all.iter().zip(&r.mobility).enumerate() {
            println!("all {a}: {} {} -> {} slide {} free {} travel {:?} axis {:?} point {:?} settle {:?} {m:?}", comps[j.comp].def, j.parent, j.child, j.slide, j.free, j.travel, j.axis, j.point, j.settle);
        }
        println!("mech_of {:?} dof {:?}", r.mech_of, r.dof);
        let mut st = Settler::default();
        let target: Vec<f64> = r.tree.iter().map(|j| if j.free { 0.0 } else { 1.5 }).collect();
        let out = st.frame(&r, &target);
        println!("driven to 86°: {:?} deg, stuck {:?}", out.iter().map(|v| (v.to_degrees() * 10.0).round() / 10.0).collect::<Vec<_>>(), st.stuck);
    }

    #[test]
    #[ignore = "needs the user's saved vehicle"]
    fn piston_crankshaft_follows_the_rod() {
        let (mut lib, appdata) = crate::defs::test_library();
        let Ok(text) = std::fs::read_to_string(appdata.join("data/vehicles/piston crankshaft.xml")) else { return };
        let comps = crate::vehicle::parse_vehicle_xml(&text);
        let r = Report::new(&comps, &mut lib, &crate::marks::Designations::default(), false);
        for j in &r.tree {
            println!("{} -> {} free {} sign {}", j.parent, j.child, j.free, j.sign);
        }
        for (a, (j, m)) in r.all.iter().zip(&r.mobility).enumerate() {
            println!("all {a}: {} {} -> {} slide {} free {} travel {:?} axis {:?} point {:?} {m:?}", comps[j.comp].def, j.parent, j.child, j.slide, j.free, j.travel, j.axis, j.point);
        }
        // The rod's top pivot: the piston's parent half, on the piston body.
        let top = r.all.iter().find(|j| r.tree.iter().all(|t| t.comp != j.comp) || r.tree.iter().any(|t| t.comp == j.comp && t.parent != j.parent)).unwrap();
        let mut st = Settler::default();
        let crank: Vec<f64> = r.tree.iter().map(|j| if j.free { 0.0 } else { 1.0 }).collect();
        let s = st.frame(&r, &crank);
        let p = body_poses(&r.tree, r.n_bodies, &|i, _| s[i]);
        let (on_piston, on_rod) = (p[top.parent].apply(top.point), p[top.child].apply(top.point - top.settle));
        println!("pin on piston {on_piston:?}, on rod {on_rod:?}, as built {:?}", top.point);
        assert!((on_piston - on_rod).len() < 1e-6);
        assert!((on_rod - top.point).len() > 0.05, "the rod's top moved with the crank");
    }

    #[test]
    #[ignore]
    fn m8a1_collides_as_spawned() {
        let (mut lib, appdata) = crate::defs::test_library();
        let Ok(text) = std::fs::read_to_string(appdata.join("data/vehicles/m8a1.xml")) else { return };
        let comps = crate::vehicle::parse_vehicle_xml(&text);
        let r = Report::new(&comps, &mut lib, &crate::marks::Designations::default(), false);
        // The game's slope table and the one the aero model infers from the stock blocks agree.
        for id in 1..=41 {
            let (g, a) = (cut_plane(id, &crate::vehicle::IDENTITY, &crate::vehicle::IDENTITY).unwrap(), lib.shapes.plane(id).unwrap());
            assert!((g.n.norm() - a.n.norm()).len() < 1e-9 && (g.d / g.n.len() - a.d / a.n.len()).abs() < 1e-9, "id {id}");
        }
        // The mantlet, a one-voxel part stretched in the file (`r="0,2,0,-2,0,0,0,-1,1"`), is one
        // unit cell, whose core is 0.166 m.
        let mantlet = comps.iter().position(|c| c.def == "solid_rocket_small" && c.pos == [-1, -5, -4]).unwrap();
        let (lo, hi) = r.hulls.iter().find(|h| h.comp as usize == mantlet).unwrap().bounds();
        assert!(((hi - lo).len() - 0.166 * 3f64.sqrt()).abs() < 1e-9);
        // The user's "Turret" (turret +360° over 10 s, gun +10° in the first second): the gun's
        // barrel (10 voxels) meets a wedge of the hull's deck at about 4.2 s.
        let value = |t: f64| {
            let mut s = r.settings.clone();
            s[1] = r.tree[1].sign * crate::pose::seq_value(0.0, &[[0.0, 10.0, 360.0]], t, None).to_radians();
            s[2] = r.tree[2].sign * crate::pose::seq_value(0.0, &[[0.0, 1.0, 10.0]], t, None).to_radians();
            s
        };
        let mut st = Settler::default();
        let stop = (1..=300).map(|k| k as f64 / 30.0).find(|&t| {
            st.frame(&r, &value(t));
            st.stop.is_some()
        });
        let (a, b) = st.hit.unwrap();
        println!("turret stops at {stop:?} s: {} hits {}", comps[a as usize].def, comps[b as usize].def);
        assert!(stop.is_some_and(|t| (4.1..4.4).contains(&t)) && comps[a as usize].def == "gun_m" && comps[b as usize].body == 0);
        // The gun down: the mantlet first touches the turret top's 1:2 wedge tips at 60° (where the
        // user sees it stop in game) and is 5 cm in at about 77°.
        let s = sweep(&r.tree, 2, &r.settings, r.n_bodies, &r.hulls);
        let deg = (s.down.to * r.tree[2].sign).to_degrees();
        println!("gun stops at {deg:.1}° on {:?}", s.down.hit.map(|c| (&comps[c.moving as usize].def, &comps[c.other as usize].def)));
        assert!((-78.0..-75.0).contains(&deg));
    }

    #[test]
    fn chosen_fixed_body_holds_still() {
        // A swing wing: the wing is body 0, the fuselage body 1 on a driven pivot.
        let z = v3(0.0, 0.0, 1.0);
        let all = vec![hinge(0, 1, v3(1.0, 0.0, 0.0), z)];
        let tree = crate::joints::tree_links(all.clone(), &[], &[1]);
        assert_eq!((tree[0].parent, tree[0].child), (1, 0), "grown from the fuselage");
        // An input of 0.5 (`joint_settings`: its sign times the input) turns the wing −0.5 about the fuselage.
        let p = body_poses(&tree, 2, &|_, l| l.sign * all[0].sign * 0.5);
        assert_eq!(p[1], Pose::IDENTITY);
        let wing = Pose::turn(z, v3(1.0, 0.0, 0.0), -0.5);
        assert!((p[0].apply(v3(3.0, 2.0, 0.0)) - wing.apply(v3(3.0, 2.0, 0.0))).len() < 1e-9, "the same input turns the wing the other way");
        assert!(crate::joints::welds(&tree, &[1]).is_empty());
        let (m, dof) = mobility_near(&tree, &all, 2, &[0.0]);
        assert_eq!((m, dof), (vec![Mobility::Free], vec![1]));
        // Both fixed: the pivot cannot move.
        let tree = crate::joints::tree_links(all.clone(), &[], &[0, 1]);
        let links = [all.clone(), crate::joints::welds(&tree, &[0, 1])].concat();
        assert_eq!(links.len(), 2);
        assert!(fixed_bodies(&tree, &links, 2).iter().all(|&f| f));
        let (m, _) = mobility_near(&tree, &links, 2, &[0.0]);
        assert!(matches!(m[0], Mobility::Locked { .. }), "{m:?}");
        let (s, stuck) = crate::pose::workbench_settings(&tree, &links, 2, &[0.5]);
        assert!(stuck && s[0].abs() < 0.01, "{s:?}");
    }

    #[test]
    fn stretched_linkage_is_rigid() {
        // "2 link mechanism": links of 1 m and 0.75 m between pivots 1.75 m apart, in line.
        let x = v3(1.0, 0.0, 0.0);
        let tree = vec![hinge(0, 1, V3::ZERO, x), JointLink { comp: 1, free: true, ..hinge(0, 2, v3(0.0, 0.0, -1.75), x) }];
        let all = [tree.clone(), vec![JointLink { comp: 2, free: true, ..hinge(1, 2, v3(0.0, 0.0, -1.0), x) }]].concat();
        let (m, dof) = mobility_near(&tree, &all, 3, &[0.0; 2]);
        assert_eq!(dof, vec![0]);
        assert!(m.iter().all(|m| matches!(m, Mobility::Locked { .. })), "{m:?}");
        let (s, stuck) = crate::pose::workbench_settings(&tree, &all, 3, &[0.5, 0.0]);
        assert!(stuck && s[0].abs() < 0.01, "{s:?}");
        let (lo, hi) = crate::pose::reach(&tree, &all, 3, &[0.0; 2], 0);
        assert!(lo.abs() < 0.01 && hi.abs() < 0.01, "reach {lo} {hi}");
    }

    #[test]
    fn piston_travel_limits_the_linkage() {
        // "2 link mechanism works": the second link carries a piston (±0.25 m) whose head is
        // pinned 0.75 m out on the first link. It reaches 1 m + 0.25 m: about 38° each way.
        let x = v3(1.0, 0.0, 0.0);
        let tree = vec![
            JointLink { travel: (-1.57, 1.57), ..hinge(0, 1, V3::ZERO, x) },
            JointLink { comp: 1, free: true, ..hinge(0, 2, v3(0.0, 0.0, -1.75), x) },
            JointLink { comp: 2, free: true, ..hinge(1, 3, v3(0.0, 0.0, -0.75), x) },
        ];
        let slide = JointLink { comp: 3, free: true, slide: true, travel: (-0.25, 0.25), ..hinge(2, 3, v3(0.0, 0.0, -1.75), v3(0.0, 0.0, -1.0)) };
        let all = [tree.clone(), vec![slide]].concat();
        assert_eq!(mobility_near(&tree, &all, 4, &[0.0; 3]).1, vec![1]);
        let reach = (((1.75f64.powi(2) + 0.75f64.powi(2) - 1.25f64.powi(2)) / (2.0 * 1.75 * 0.75)).acos()).to_degrees();
        for to in [1.5, -1.5] {
            let (s, stuck) = crate::pose::workbench_settings(&tree, &all, 4, &[to, 0.0, 0.0]);
            assert!(stuck && (s[0].abs().to_degrees() - reach).abs() < 0.5, "stops at {} not {reach}", s[0].to_degrees());
        }
        let (s, stuck) = crate::pose::workbench_settings(&tree, &all, 4, &[0.3, 0.0, 0.0]);
        assert!(!stuck && (s[0] - 0.3).abs() < 1e-12, "within reach: goes all the way");
        let (lo, hi) = crate::pose::reach(&tree, &all, 4, &[0.0; 3], 0);
        assert!((lo.to_degrees() + reach).abs() < 0.5 && (hi.to_degrees() - reach).abs() < 0.5, "reach {lo} {hi}");
    }

    #[test]
    fn crank_slider_at_dead_centre_moves() {
        // Crank 1 m, rod 2 m, slider along x at 3 m: all in line, as built.
        let z = v3(0.0, 0.0, 1.0);
        let tree = vec![
            JointLink { comp: 1, ..hinge(0, 1, V3::ZERO, z) },
            JointLink { comp: 2, free: true, ..hinge(1, 2, v3(1.0, 0.0, 0.0), z) },
            JointLink { comp: 3, free: true, slide: true, travel: (-2.0, 2.0), ..hinge(0, 3, v3(3.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)) },
        ];
        let all = [tree.clone(), vec![JointLink { comp: 4, free: true, ..hinge(2, 3, v3(3.0, 0.0, 0.0), z) }]].concat();
        let first = mobility(&all, 4, &[]);
        println!("{first:?}");
        let (m, dof) = mobility_near(&tree, &all, 4, &[0.0; 3]);
        assert!(m.iter().all(|m| *m == Mobility::Free), "{m:?}");
        assert_eq!(dof, vec![1], "crank, rod and slider: one mechanism, one degree of freedom");
        // Driven by the slider instead: from dead centre it goes in (to 1 m out), not out.
        let mut by_slide = tree.clone();
        (by_slide[0].free, by_slide[2].free) = (true, false);
        let (lo, hi) = crate::pose::reach(&by_slide, &all, 4, &[0.0; 3], 2);
        assert!((lo + 2.0).abs() < 0.05 && hi.abs() < 1e-3, "reach {lo} {hi}");
        // A triangle stays locked: no step of one joint lets the others close it.
        let tri = vec![hinge(0, 1, V3::ZERO, z), JointLink { comp: 1, ..hinge(1, 2, v3(0.0, 1.0, 0.0), z) }];
        let all = [tri.clone(), vec![JointLink { comp: 2, ..hinge(2, 0, v3(1.0, 0.0, 0.0), z) }]].concat();
        let (m, dof) = mobility_near(&tri, &all, 3, &[0.0; 2]);
        assert!(m.iter().all(|m| matches!(m, Mobility::Locked { .. })));
        assert_eq!(dof, vec![0]);
        // Two doors on the main body: two mechanisms of one each.
        let doors = vec![hinge(0, 1, V3::ZERO, z), JointLink { comp: 1, ..hinge(0, 2, v3(5.0, 0.0, 0.0), z) }];
        assert_eq!(mobility_near(&doors, &doors, 3, &[0.0; 2]).1, vec![1, 1]);
    }

    #[test]
    fn walls_guide_a_piston_on_a_crank() {
        // Crank 0.5 m about z (driven), rod 1.25 m (free), a two-block piston on the rod's far pin
        // (free), sliding between two rows of wall blocks with faces touching.
        let z = v3(0.0, 0.0, 1.0);
        let tree = vec![
            JointLink { comp: 10, ..hinge(0, 1, V3::ZERO, z) },
            JointLink { comp: 11, free: true, ..hinge(1, 2, v3(0.5, 0.0, 0.0), z) },
            JointLink { comp: 12, free: true, ..hinge(2, 3, v3(1.75, 0.0, 0.0), z) },
        ];
        let mut hulls = vec![Hull::from_solid(0, 3, &cube(v3(1.75, 0.0, 0.0))), Hull::from_solid(1, 3, &cube(v3(2.0, 0.0, 0.0)))];
        for k in 0..7 {
            for y in [-0.25, 0.25] {
                hulls.push(Hull::from_solid(2, 0, &cube(v3(1.0 + 0.25 * k as f64, y, 0.0))));
            }
        }
        let r = Report { all: tree.clone(), links: tree.clone(), fixed: vec![true, false, false, false], tree, n_bodies: 4, wanted: vec![0.0; 3], settings: vec![0.0; 3], poses: Vec::new(), mobility: Vec::new(), mech_of: Vec::new(), dof: Vec::new(), hulls, sweeps: None };
        let mut st = Settler::default();
        let s = st.frame(&r, &[FRAC_PI_2, 0.0, 0.0]);
        let p = body_poses(&r.tree, 4, &|i, _| s[i]);
        let pin = p[3].apply(v3(1.75, 0.0, 0.0));
        let x = (1.25f64.powi(2) - 0.25).sqrt();
        assert!((s[0] - FRAC_PI_2).abs() < 1e-9);
        assert!(pin.y.abs() < 0.02 && (pin.x - x).abs() < 0.02, "pin at {pin:?}, settings {s:?}");
        assert!(p[3].dir(v3(1.0, 0.0, 0.0)).x > 0.999, "the piston stays in line");
        // And back down to the start.
        let s = st.frame(&r, &[0.0, 0.0, 0.0]);
        assert!(s.iter().all(|v| v.abs() < 0.02), "{s:?}");
    }

    #[test]
    fn swept_arm_stops_at_the_wall() {
        // An arm cube 1 m out on a pivot about aero z (down), a wall cube 1 m to the right.
        let arm = Hull::from_solid(1, 1, &cube(v3(1.0, 0.0, 0.0)));
        let wall = Hull::from_solid(0, 0, &cube(v3(0.0, 1.0, 0.0)));
        let touching = Hull::from_solid(2, 0, &cube(v3(1.0, 0.0, -0.25)));
        let tree = [JointLink { travel: (-1.5, 1.5), ..hinge(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0)) }];
        let s = sweep(&tree, 0, &[0.0], 2, &[arm, wall, touching]);
        assert!(s.now.is_empty(), "face to face is not a contact: {:?}", s.now);
        assert_eq!(s.up.hit, Some(Contact { moving: 1, other: 0 }));
        // The cubes' corners meet first, about 90° less two half widths, then sink 5 cm (~3°).
        assert!((1.1..1.4).contains(&s.up.to), "{}", s.up.to.to_degrees());
        assert_eq!(s.down, Reach { to: -1.5, hit: None }, "free the other way to the end of travel");
        assert!(!s.all_round);
    }

    #[test]
    fn picks_the_nearest_part_under_the_mouse() {
        // Camera on the aero x axis 10 m behind, looking forward (+x); clip = view·projection
        // with view mapping aero (x fwd, y right, z down) to GL (x right, y up, −z fwd).
        let view = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -10.0, 1.0];
        let (n, f) = (0.1, 100.0);
        let proj = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -(f + n) / (f - n), -1.0, 0.0, 0.0, -2.0 * f * n / (f - n), 0.0];
        let clip: [f64; 16] = std::array::from_fn(|k| (0..4).map(|j| view[k / 4 * 4 + j] * proj[j * 4 + k % 4]).sum());
        let near = Hull::from_solid(1, 0, &cube(v3(-2.0, 0.0, 0.0)));
        let far = Hull::from_solid(2, 0, &cube(v3(2.0, 0.0, 0.0)));
        let right = Hull::from_solid(3, 0, &cube(v3(0.0, 1.0, 0.0)));
        let hs = [far, near, right];
        assert_eq!(pick(&hs, &[], &clip, [0.0, 0.0]), Some(1), "the nearer of two in line");
        // 1 m right at 10 m: x = 0.1 in NDC (90° view).
        assert_eq!(pick(&hs, &[], &clip, [0.1, 0.0]), Some(3));
        assert_eq!(pick(&hs, &[], &clip, [0.1, 0.5]), None);
    }

    #[test]
    fn one_pivot_moves() {
        assert_eq!(mobility(&[hinge(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0))], 2, &[]), vec![Mobility::Free]);
    }

    #[test]
    fn two_pivots_on_one_axis_move_together() {
        let z = v3(0.0, 0.0, 1.0);
        let j = [hinge(0, 1, v3(1.0, 2.0, 0.0), z), hinge(0, 1, v3(1.0, 2.0, 3.0), -z)];
        assert_eq!(mobility(&j, 2, &[]), vec![Mobility::Free; 2]);
    }

    #[test]
    fn two_pivots_off_axis_lock_each_other() {
        let j = [hinge(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0)), hinge(0, 1, V3::ZERO, v3(1.0, 0.0, 0.0)), hinge(1, 2, V3::ZERO, v3(0.0, 1.0, 0.0))];
        let m = mobility(&j, 3, &[]);
        assert_eq!(m[0], Mobility::Locked { by: vec![1] });
        assert_eq!(m[1], Mobility::Locked { by: vec![0] });
        assert_eq!(m[2], Mobility::Free, "a pivot on the locked sub-grid still turns");
        // Parallel but apart: also locked.
        let j = [hinge(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0)), hinge(0, 1, v3(1.0, 0.0, 0.0), v3(0.0, 0.0, 1.0))];
        assert!(mobility(&j, 2, &[]).iter().all(|m| matches!(m, Mobility::Locked { .. })));
    }

    #[test]
    fn four_bar_linkage_moves() {
        // Ground 0, crank 1, coupler 2, rocker 3, all pivots on parallel axes: one way to move.
        let z = v3(0.0, 0.0, 1.0);
        let j = [hinge(0, 1, v3(0.0, 0.0, 0.0), z), hinge(1, 2, v3(0.0, 1.0, 0.0), z), hinge(2, 3, v3(2.0, 1.5, 0.0), z), hinge(3, 0, v3(2.5, 0.0, 0.0), z)];
        assert_eq!(mobility(&j, 4, &[]), vec![Mobility::Free; 4]);
        // A triangle cannot move.
        let j = [hinge(0, 1, v3(0.0, 0.0, 0.0), z), hinge(1, 2, v3(0.0, 1.0, 0.0), z), hinge(2, 0, v3(1.0, 0.0, 0.0), z)];
        assert!(mobility(&j, 3, &[]).iter().all(|m| matches!(m, Mobility::Locked { by } if by.len() == 2)));
    }

    #[test]
    #[ignore = "survey of the saved vehicles"]
    fn saved_vehicles() {
        let (mut lib, appdata) = crate::defs::test_library();
        let mut files: Vec<_> = std::fs::read_dir(appdata.join("data/vehicles")).unwrap().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|e| e == "xml")).collect();
        files.sort();
        let only = std::env::var("MECH_ONLY").ok();
        for f in files {
            let name = f.file_stem().unwrap().to_string_lossy().to_string();
            if only.as_ref().is_some_and(|o| !name.contains(o.as_str())) {
                continue;
            }
            let comps = crate::vehicle::parse_vehicle_xml(&std::fs::read_to_string(&f).unwrap_or_default());
            let joints = crate::joints::all_joint_links(&comps, &mut lib);
            if joints.is_empty() {
                continue;
            }
            let n = crate::pose::body_count(&comps);
            {
                // Loops close at the build pose already: closing them must not move anything.
                let tree = crate::joints::joint_links(&comps, &mut lib);
                let mut s = vec![0.0; tree.len()];
                let t = std::time::Instant::now();
                crate::pose::close_loops(&tree, &joints, n, &mut s);
                let moved = s.iter().fold(0.0f64, |a, v| a.max(v.abs()));
                let free = tree.iter().filter(|j| j.free).count();
                println!("loops {name}: {} loop joints, {free} free, moved {moved:.2e} in {} us", joints.len() - tree.len(), t.elapsed().as_micros());
                assert!(moved < 1e-6, "{name}: closing its loops moved the build pose");
                // Every joint gets its label.
                let r = Report::new(&comps, &mut lib, &crate::marks::Designations::default(), false);
                let lost: Vec<&str> = r.all.iter().filter(|j| r.joint_anchor(j).is_none()).map(|j| comps[j.comp].def.as_str()).collect();
                assert!(lost.is_empty(), "{name}: no label place for {lost:?}");
                let t = std::time::Instant::now();
                let reach: Vec<_> = (0..r.tree.len()).map(|i| crate::pose::reach(&r.tree, &r.all, n, &r.settings, i)).collect();
                let short = reach.iter().zip(&r.tree).filter(|(a, j)| **a != j.travel).count();
                println!("reach {name}: {short} of {} joints held short by loops, {} ms", r.tree.len(), t.elapsed().as_millis());
                if free > 0 {
                    let mut st = Settler::default();
                    let target: Vec<f64> = r.tree.iter().map(|j| if j.free { 0.0 } else { 0.05 }).collect();
                    let t = std::time::Instant::now();
                    let out = st.frame(&r, &target);
                    let drift = out.iter().zip(&target).filter(|_| true).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
                    println!("settle {name}: {} hulls, one frame {} ms, free joints moved up to {drift:.3}", r.hulls.len(), t.elapsed().as_millis());
                }
            }
            let m = mobility(&joints, n, &[]);
            let locked = m.iter().filter(|m| matches!(m, Mobility::Locked { .. })).count();
            let t = std::time::Instant::now();
            let hulls = hulls(&comps, &mut lib);
            let tree = crate::joints::joint_links(&comps, &mut lib);
            let zero = vec![0.0; tree.len()];
            let th = t.elapsed().as_millis();
            let name_of = |c: u32| format!("{}@{:?}", comps[c as usize].def, comps[c as usize].pos);
            let mut lines = Vec::new();
            for j in 0..tree.len() {
                let s = sweep(&tree, j, &zero, n, &hulls);
                let unit = |v: f64| if tree[j].slide { format!("{:.2} m", v * tree[j].sign) } else { format!("{:.0}°", (v * tree[j].sign).to_degrees()) };
                let hit = |r: &Reach| r.hit.map_or("end".to_string(), |c| format!("{} vs {}", name_of(c.moving), name_of(c.other)));
                lines.push(format!("    {} now {:?} | down {} ({}) | up {} ({}){}", name_of(tree[j].comp as u32), s.now.iter().map(|c| format!("{} vs {}", name_of(c.moving), name_of(c.other))).collect::<Vec<_>>(), unit(s.down.to), hit(&s.down), unit(s.up.to), hit(&s.up), if s.all_round { " all round" } else { "" }));
            }
            if only.is_some() {
                let r = Report::new(&comps, &mut lib, &crate::marks::Designations::default(), true);
                let named: Vec<String> = comps.iter().map(|c| lib.get(&c.def).map(|d| d.name.clone()).unwrap_or_default()).collect();
                let names = |c: usize| named[c].clone();
                for j in r.tree.iter().take(3) {
                    eprintln!("{:?}", r.describe(&comps, &names, j.half));
                }
            }
            eprintln!("{name}: {} joints, {locked} locked, {} hulls in {th} ms, sweeps {} ms", joints.len(), hulls.len(), t.elapsed().as_millis());
            if only.is_some() {
                lines.iter().for_each(|l| eprintln!("{l}"));
            }
        }
    }

    #[test]
    fn slider_and_fixed_offset() {
        let y = v3(0.0, 1.0, 0.0);
        let slide = JointLink { slide: true, travel: (-0.5, 0.5), ..hinge(0, 1, V3::ZERO, y) };
        let offset = JointLink { slide: true, travel: (0.0, 0.0), ..hinge(0, 2, V3::ZERO, y) };
        assert_eq!(mobility(&[slide.clone(), offset], 3, &[]), vec![Mobility::Free, Mobility::Fixed]);
        // A pivot and a slide along its axis on one sub-grid: neither can move.
        let m = mobility(&[slide, hinge(0, 1, V3::ZERO, y)], 2, &[]);
        assert!(m.iter().all(|m| matches!(m, Mobility::Locked { .. })), "{m:?}");
    }
}



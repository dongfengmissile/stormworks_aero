//! Sub-grid poses from joint settings, without physics: each body carried by a joint turns
//! about (or slides along) the joint's axis, after the body carrying it has been placed.

use crate::math::{v3, V3};
use std::collections::HashMap;

use crate::joints::JointLink;

/// A rigid move in the aero frame: `p' = rot·p + trans` (`rot` as matrix rows).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub rot: [V3; 3],
    pub trans: V3,
}

impl Pose {
    pub const IDENTITY: Pose = Pose { rot: [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)], trans: V3::ZERO };

    pub fn dir(&self, d: V3) -> V3 {
        v3(self.rot[0].dot(d), self.rot[1].dot(d), self.rot[2].dot(d))
    }

    pub fn apply(&self, p: V3) -> V3 {
        self.dir(p) + self.trans
    }

    /// `inner`, then this pose.
    pub fn after(&self, inner: &Pose) -> Pose {
        let rot = self.rot.map(|r| inner.rot[0] * r.x + inner.rot[1] * r.y + inner.rot[2] * r.z);
        Pose { rot, trans: self.apply(inner.trans) }
    }

    /// A turn by `angle` (rad, right hand about the unit `axis`) about the line through `point`.
    pub fn turn(axis: V3, point: V3, angle: f64) -> Pose {
        let (s, c) = angle.sin_cos();
        let k = [axis.x, axis.y, axis.z];
        // Rodrigues: R = c·I + s·[k]x + (1 − c)·k·kᵀ.
        let cross = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]];
        let r = |i: usize, j: usize| if i == j { c } else { 0.0 } + s * cross[i][j] + (1.0 - c) * k[i] * k[j];
        let rot = [0, 1, 2].map(|i| v3(r(i, 0), r(i, 1), r(i, 2)));
        let p = Pose { rot, trans: V3::ZERO };
        Pose { rot, trans: point - p.dir(point) }
    }
}

/// Each body's pose (index = body) with joint `i` set to `setting(i, joint)`: radians for a
/// turning joint, metres along its axis for a sliding one. Bodies no joint carries stay as built.
/// `joints` must list parents first (`model::joint_links`).
pub fn body_poses(joints: &[JointLink], n_bodies: usize, setting: &dyn Fn(usize, &JointLink) -> f64) -> Vec<Pose> {
    let mut out = vec![Pose::IDENTITY; n_bodies];
    for (i, j) in joints.iter().enumerate() {
        let v = setting(i, j);
        let settle = Pose { trans: j.settle, ..Pose::IDENTITY };
        let local = if j.slide { Pose { trans: j.axis * v + j.settle, ..Pose::IDENTITY } } else { Pose::turn(j.axis, j.point, v).after(&settle) };
        if let (Some(&parent), true) = (out.get(j.parent), j.child < n_bodies) {
            out[j.child] = parent.after(&local);
        }
    }
    out
}

/// Moves the free joints of `tree` (`JointLink::free`) so that each joint closing a loop (in
/// `all`, not in `tree`: a crank's rod pinned to its piston) joins its two bodies again. Least
/// squares from `settings`; driven joints keep theirs.
pub fn close_loops(tree: &[JointLink], all: &[JointLink], n_bodies: usize, settings: &mut [f64]) {
    let loops = all.iter().filter(|l| !tree.iter().any(|t| t.comp == l.comp));
    let free = free_above(tree, loops.flat_map(|l| [l.parent, l.child]));
    settle(tree, all, n_bodies, settings, &free, &|_, _| {});
}

/// The workbench pose for joint settings `target` (`joint_settings`): the joints in loops start
/// as built (where every loop is joined) and the driven ones move to their settings in steps,
/// the free ones following (`march`). True: a loop stopped them short of `target`.
pub fn workbench_settings(tree: &[JointLink], all: &[JointLink], n_bodies: usize, target: &[f64]) -> (Vec<f64>, bool) {
    let loop_bodies = loop_bodies(tree, all);
    let mut from = target.to_vec();
    for i in above(tree, loop_bodies.iter().copied()) {
        from[i] = 0.0;
    }
    let free = free_above(tree, loop_bodies);
    let (s, stop) = march(tree, &from, target, &mut |s| settle(tree, all, n_bodies, s, &free, &|_, _| {}));
    (s, stop.is_some())
}

/// The bodies the joints closing loops (in `all`, not in `tree`) join.
fn loop_bodies(tree: &[JointLink], all: &[JointLink]) -> Vec<usize> {
    all.iter().filter(|l| !tree.iter().any(|t| t.comp == l.comp)).flat_map(|l| [l.parent, l.child]).collect()
}

/// How far driven joint `t` can move from `settings` each way, the loops kept joined and every
/// other joint on them following (two rams lifting one boom move together): its travel where no
/// loop stops it.
/// shortcut: loops only, not contacts.
pub fn reach(tree: &[JointLink], all: &[JointLink], n_bodies: usize, settings: &[f64], t: usize) -> (f64, f64) {
    let (lo, hi) = tree[t].travel;
    let on_loop = above(tree, loop_bodies(tree, all));
    if tree[t].free || !on_loop.contains(&t) {
        return (lo, hi);
    }
    let free: Vec<usize> = on_loop.into_iter().filter(|&i| i != t).collect();
    let mut follow = tree.to_vec();
    for &i in &free {
        follow[i].free = true;
    }
    let tree = &follow[..];
    let end = |to: f64| {
        // A joint with no end: a full turn that way shows whether a loop stops it.
        let far = if to.is_finite() { to } else { settings[t] + to.signum() * std::f64::consts::TAU };
        let mut target = settings.to_vec();
        target[t] = far;
        let (s, stop) = march(tree, settings, &target, &mut |s| settle(tree, all, n_bodies, s, &free, &|_, _| {}));
        if stop.is_some() { s[t] } else { to }
    };
    (end(lo), end(hi))
}

/// How far loops and contacts may stay apart (squared residual of `settle`, m²): 0.1 mm.
pub const JAM: f64 = 1e-8;
/// Steps of the driven joints between two settles: radians, or metres for a slide.
pub const TURN_STEP: f64 = 0.05;
pub const SLIDE_STEP: f64 = 0.03;

/// Moves the driven joints of `tree` from `from` to `to` in small steps, `settle` placing the
/// free ones after each (it returns the squared residual left). Stops where a step cannot be
/// settled (a linkage at the end of its reach, a piston at the end of its cylinder), to within
/// 1/256 of a step: those settings, and how far it got (0..1 of the way).
pub fn march(tree: &[JointLink], from: &[f64], to: &[f64], settle: &mut dyn FnMut(&mut Vec<f64>) -> f64) -> (Vec<f64>, Option<f64>) {
    let driven: Vec<usize> = (0..tree.len()).filter(|&i| !tree[i].free).collect();
    let span = driven.iter().map(|&i| (to[i] - from[i]).abs() / if tree[i].slide { SLIDE_STEP } else { TURN_STEP }).fold(0.0, f64::max);
    let n = (span.ceil() as usize).clamp(1, 400);
    let at = |last: &[f64], k: f64| {
        let mut s = last.to_vec();
        for &i in &driven {
            s[i] = from[i] + (to[i] - from[i]) * k;
        }
        s
    };
    let mut last = from.to_vec();
    for k in 1..=n {
        let mut s = at(&last, k as f64 / n as f64);
        if settle(&mut s) <= JAM {
            last = s;
            continue;
        }
        let (mut lo, mut hi) = ((k - 1) as f64 / n as f64, k as f64 / n as f64);
        for _ in 0..8 {
            let mid = 0.5 * (lo + hi);
            let mut s = at(&last, mid);
            if settle(&mut s) <= JAM {
                (last, lo) = (s, mid);
            } else {
                hi = mid;
            }
        }
        return (last, Some(lo));
    }
    (last, None)
}

/// The free joints between `bodies` and the fixed body.
pub fn free_above(tree: &[JointLink], bodies: impl IntoIterator<Item = usize>) -> Vec<usize> {
    above(tree, bodies).into_iter().filter(|&i| tree[i].free).collect()
}

/// The joints between `bodies` and the fixed body.
pub fn above(tree: &[JointLink], bodies: impl IntoIterator<Item = usize>) -> Vec<usize> {
    let up: std::collections::HashMap<usize, usize> = tree.iter().enumerate().map(|(i, j)| (j.child, i)).collect();
    let mut above = vec![false; tree.len()];
    for mut b in bodies {
        while let Some(&i) = up.get(&b) {
            if std::mem::replace(&mut above[i], true) {
                break;
            }
            b = tree[i].parent;
        }
    }
    (0..tree.len()).filter(|&i| above[i]).collect()
}

/// Least squares on the joints `free` of `tree` from `settings`: every loop of `all` joined, and
/// `extra`'s residuals for the posed bodies (contacts) at zero. Returns the squared residual left.
pub fn settle(tree: &[JointLink], all: &[JointLink], n_bodies: usize, settings: &mut [f64], free: &[usize], extra: &dyn Fn(&[Pose], &mut Vec<f64>)) -> f64 {
    let loops: Vec<&JointLink> = all.iter().filter(|l| !tree.iter().any(|t| t.comp == l.comp)).collect();
    let residual = |s: &[f64]| -> Vec<f64> {
        let p = body_poses(tree, n_bodies, &|i, _| s[i]);
        let at = |b: usize| p.get(b).copied().unwrap_or(Pose::IDENTITY);
        let mut r = Vec::new();
        for l in &loops {
            let (a, b) = (at(l.parent), at(l.child));
            let d = a.apply(l.point) - b.apply(l.point - l.settle);
            let (ea, eb) = (a.dir(l.axis), b.dir(l.axis));
            let e = if l.axis.x.abs() < 0.9 { l.axis.cross(v3(1.0, 0.0, 0.0)) } else { l.axis.cross(v3(0.0, 1.0, 0.0)) }.norm();
            // Its own setting, as `body_poses` would take it, kept within its travel.
            let v = if l.slide {
                // Free along the axis only; no turn.
                r.extend([d - ea * ea.dot(d), ea - eb, a.dir(e) - b.dir(e)]);
                -ea.dot(d)
            } else {
                r.extend([d, ea.cross(eb)]);
                let (pe, ce) = (a.dir(e), b.dir(e));
                ea.dot(pe.cross(ce)).atan2(pe.dot(ce))
            };
            r.push(v3((v - l.travel.1).max(0.0) + (l.travel.0 - v).max(0.0), 0.0, 0.0));
        }
        let mut out: Vec<f64> = r.iter().flat_map(|v| [v.x, v.y, v.z]).collect();
        extra(&p, &mut out);
        out
    };
    let size = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>();
    let solve = |mut s: Vec<f64>| -> (Vec<f64>, f64) {
        let mut r = residual(&s);
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
        'solve: for _ in 0..40 {
            if size(&r) < 1e-14 || free.is_empty() {
                break;
            }
            let cols: Vec<Vec<f64>> = (free.iter())
                .map(|&i| {
                    let mut t = s.clone();
                    t[i] += 1e-6;
                    residual(&t).iter().zip(&r).map(|(a, b)| (a - b) / 1e-6).collect()
                })
                .collect();
            // Levenberg-Marquardt: (JᵀJ + damp·diag) step = −Jᵀr.
            let mut damp = 1e-3;
            loop {
                let m: Vec<Vec<f64>> = (0..free.len())
                    .map(|i| (0..free.len()).map(|k| dot(&cols[i], &cols[k]) * if i == k { 1.0 + damp } else { 1.0 } + if i == k { 1e-12 } else { 0.0 }).chain([-dot(&cols[i], &r)]).collect())
                    .collect();
                let Some(step) = gauss(m) else { break 'solve };
                let mut t = s.clone();
                for (k, &i) in free.iter().enumerate() {
                    t[i] = (t[i] + step[k]).clamp(tree[i].travel.0, tree[i].travel.1);
                }
                let rt = residual(&t);
                if size(&rt) < size(&r) {
                    (s, r) = (t, rt);
                    break;
                }
                damp *= 10.0;
                if damp > 1e6 {
                    break 'solve;
                }
            }
        }
        let e = size(&r);
        (s, e)
    };
    let (mut s, mut e) = solve(settings.to_vec());
    // From a dead centre (a crank and its rod in line) no small move closes the gap at first
    // order and the solve stalls: start again a little way off, each way.
    for kick in [1.0, -1.0] {
        if e <= JAM || free.is_empty() {
            break;
        }
        let mut k = settings.to_vec();
        for &i in free {
            k[i] = (k[i] + kick * if tree[i].slide { 0.02 } else { 0.1 }).clamp(tree[i].travel.0, tree[i].travel.1);
        }
        let (t, et) = solve(k);
        if et < e {
            (s, e) = (t, et);
        }
    }
    settings.copy_from_slice(&s);
    e
}

/// Solves `a` (n rows of n coefficients and the right-hand side) by elimination.
fn gauss(mut a: Vec<Vec<f64>>) -> Option<Vec<f64>> {
    let n = a.len();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &k| a[i][c].abs().total_cmp(&a[k][c].abs()))?;
        a.swap(c, p);
        if a[c][c].abs() < 1e-300 {
            return None;
        }
        for i in 0..n {
            if i != c {
                let f = a[i][c] / a[c][c];
                for k in c..=n {
                    a[i][k] -= f * a[c][k];
                }
            }
        }
    }
    Some((0..n).map(|i| a[i][n] / a[i][i]).collect())
}

/// A joint's value at time `t` (s) of its sequence: from `base`, each move (start, duration,
/// target) eases to its target in start order; later moves start from where earlier ones left it.
/// A move carries on without easing into a move the same way that starts as it ends, and, when
/// the sequence loops every `period` s, from its end round to its start (a crank turning on).
pub fn seq_value(base: f64, moves: &[[f64; 3]], t: f64, period: Option<f64>) -> f64 {
    let mut m = moves.to_vec();
    m.sort_by(|a, b| a[0].total_cmp(&b[0]));
    // Each move's start value and way.
    let mut from = Vec::with_capacity(m.len());
    let mut v = base;
    for [_, _, to] in &m {
        from.push(v);
        v = *to;
    }
    let way = |k: usize| (m[k][2] - from[k]).signum();
    let flows = |a: usize, b: usize, gap: f64| gap.abs() < 1e-6 && way(a) != 0.0 && way(a) == way(b);
    let n = m.len();
    m.iter().enumerate().take_while(|(_, [s, ..])| t > *s).fold(base, |v, (k, [s, d, to])| {
        let k_in = if k > 0 { flows(k - 1, k, m[k - 1][0] + m[k - 1][1] - s) } else { period.is_some_and(|p| flows(n - 1, 0, m[n - 1][0] + m[n - 1][1] - p) && s.abs() < 1e-6) };
        let k_out = if k + 1 < n { flows(k, k + 1, s + d - m[k + 1][0]) } else { period.is_some_and(|p| flows(k, 0, s + d - p) && m[0][0].abs() < 1e-6) };
        let x = if *d <= 0.0 { 1.0 } else { ((t - s) / d).clamp(0.0, 1.0) };
        // Cubic from 0 to 1 with end slopes 0 (eased) or 1 (carrying on).
        let (a, b) = (if k_in { 1.0 } else { 0.0 }, if k_out { 1.0 } else { 0.0 });
        let f = x.powi(3) * (a + b - 2.0) + x * x * (3.0 - 2.0 * a - b) + a * x;
        v + (to - v) * f
    })
}

/// When the last move of any sequence ends, s.
pub fn seq_end<'a>(moves: impl IntoIterator<Item = &'a [f64; 3]>) -> f64 {
    moves.into_iter().map(|[s, d, _]| s + d).fold(0.0, f64::max)
}

pub fn body_count(comps: &[crate::vehicle::Comp]) -> usize {
    comps.iter().map(|c| c.body as usize + 1).max().unwrap_or(1)
}

/// Each joint's workbench setting as `body_poses` takes it (the game's sign, within its travel).
/// `angles` (degrees) and `offsets` (metres) by the joint's parent half position.
pub fn joint_settings(joints: &[JointLink], comps: &[crate::vehicle::Comp], angles: &HashMap<[i32; 3], f64>, offsets: &HashMap<[i32; 3], f64>) -> Vec<f64> {
    joints
        .iter()
        .map(|j| {
            let pos = &comps[j.comp].pos;
            let v = if j.slide { offsets.get(pos).copied().unwrap_or(0.0) } else { angles.get(pos).copied().unwrap_or(0.0).to_radians() };
            (j.sign * v).clamp(j.travel.0, j.travel.1)
        })
        .collect()
}

/// The pose shown in the game's workbench, eased toward its target over `SECONDS` (entering the
/// mode, leaving it, moving a slider), so the sub-grids are seen moving.
#[derive(Default)]
pub struct PoseAnim {
    links: Vec<JointLink>,
    /// Joint settings and how far the settle offsets are in (0 as built, 1 posed).
    from: (Vec<f64>, f64),
    to: (Vec<f64>, f64),
    t0: f64,
}

impl PoseAnim {
    pub const SECONDS: f64 = 1.0;

    fn shown(&self, now: f64) -> (Vec<f64>, f64) {
        let t = ((now - self.t0) / Self::SECONDS).clamp(0.0, 1.0);
        let k = t * t * (3.0 - 2.0 * t);
        let lerp = |a: f64, b: f64| a + (b - a) * k;
        (self.from.0.iter().zip(&self.to.0).map(|(a, b)| lerp(*a, *b)).collect(), lerp(self.from.1, self.to.1))
    }

    /// The body poses to show at `now` (s) for target `settings` of `links`, posed if `on`, else
    /// back to as built; None once everything is as built. `snap`: show the target at once (a
    /// sequence playing frame by frame).
    pub fn poses(&mut self, links: &[JointLink], n_bodies: usize, settings: &[f64], on: bool, snap: bool, now: f64) -> Option<Vec<Pose>> {
        let want = (if on { settings.to_vec() } else { vec![0.0; settings.len()] }, if on { 1.0 } else { 0.0 });
        if snap && on {
            (self.links, self.from, self.to, self.t0) = (links.to_vec(), want.clone(), want, now);
        } else if self.links != links || self.to.0.len() != want.0.len() {
            // Other joints (the vehicle changed): ease in from as built if nothing showed, else snap.
            let hidden = self.links.is_empty() || self.shown(now).1 == 0.0;
            self.links = links.to_vec();
            self.from = if hidden { (vec![0.0; want.0.len()], 0.0) } else { want.clone() };
            self.to = want;
            self.t0 = now;
        } else if want != self.to {
            self.from = self.shown(now);
            self.to = want;
            self.t0 = now;
        }
        let (s, f) = self.shown(now);
        if f == 0.0 && s.iter().all(|v| *v == 0.0) {
            return None;
        }
        let links: Vec<JointLink> = self.links.iter().map(|l| JointLink { settle: l.settle * f, ..l.clone() }).collect();
        Some(body_poses(&links, n_bodies, &|i, _| s[i]))
    }
}

/// A pose as a move of the game's grid metres (x right, y up, z forward), `g' = Q·g + shift`:
/// the rows of Qᵀ (images of the grid axes), then the shift.
pub fn grid_move(t: &Pose) -> [[f64; 3]; 4] {
    let to_grid = |a: V3| [a.y, -a.z, a.x];
    let to_aero = |g: V3| v3(g.z, g.x, -g.y);
    let axis = |i: usize| to_grid(t.dir(to_aero([v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)][i])));
    [axis(0), axis(1), axis(2), to_grid(t.trans)]
}

/// A body's draw matrix moved by `mv` (`grid_move`). The game draws a grid point at
/// `(g − origin)·M` (row vectors); the result draws it where the move puts it.
pub fn draw_matrix(mv: &[[f64; 3]; 4], origin: [f64; 3], m: &[f64; 16]) -> [f64; 16] {
    let [r0, r1, r2, shift] = mv;
    let o = origin;
    // (g − o)·T·M = (g' − o)·M with T = [Qᵀ; o·Qᵀ + shift − o].
    let row3: [f64; 3] = std::array::from_fn(|j| o[0] * r0[j] + o[1] * r1[j] + o[2] * r2[j] + shift[j] - o[j]);
    let t = [r0[0], r0[1], r0[2], 0.0, r1[0], r1[1], r1[2], 0.0, r2[0], r2[1], r2[2], 0.0, row3[0], row3[1], row3[2], 1.0];
    std::array::from_fn(|k| (0..4).map(|j| t[k / 4 * 4 + j] * m[j * 4 + k % 4]).sum())
}

#[cfg(test)]
mod tests {
    #[test]
    fn free_joints_close_a_crank() {
        // Crank 1 m about z at the origin (driven), rod 2 m on its pin (free), its far pin on a
        // free slider along x at 3 m: the pin that closes the loop.
        let z = v3(0.0, 0.0, 1.0);
        let tree = [
            JointLink { comp: 1, ..link(0, 1, V3::ZERO, z, false) },
            JointLink { comp: 2, free: true, ..link(1, 2, v3(1.0, 0.0, 0.0), z, false) },
            JointLink { comp: 3, free: true, ..link(0, 3, v3(3.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), true) },
        ];
        let all = [tree.to_vec(), vec![JointLink { comp: 4, ..link(2, 3, v3(3.0, 0.0, 0.0), z, false) }]].concat();
        let mut s = [FRAC_PI_2, 0.0, 0.0];
        close_loops(&tree, &all, 4, &mut s);
        let p = body_poses(&tree, 4, &|i, _| s[i]);
        let pin = v3(3.0, 0.0, 0.0);
        assert!((p[2].apply(pin) - p[3].apply(pin)).dot(p[2].apply(pin) - p[3].apply(pin)) < 1e-10, "{s:?}");
        assert!((s[0] - FRAC_PI_2).abs() < 1e-12, "the crank stays where it is driven");
        assert!((s[2] - (3f64.sqrt() - 3.0)).abs() < 1e-6, "slider at {}", s[2]);
    }

    #[test]
    fn sequence_moves_in_order_from_the_workbench_pose() {
        use super::seq_value;
        // Door opens 0..1 s, gear comes down 1..3 s, door closes 3..4 s.
        let door = [[0.0, 1.0, 90.0], [3.0, 1.0, 0.0]];
        assert_eq!(seq_value(0.0, &door, 0.0, None), 0.0);
        assert_eq!(seq_value(0.0, &door, 0.5, None), 45.0, "eased: half way at half time");
        assert_eq!(seq_value(0.0, &door, 2.0, None), 90.0);
        assert_eq!(seq_value(0.0, &door, 9.0, None), 0.0);
        assert_eq!(seq_value(10.0, &[], 5.0, None), 10.0, "no moves: the workbench pose");
        assert_eq!(seq_value(0.0, &door, 0.5, Some(4.0)), 45.0, "a door going back does not carry on");
        // A crank: one turn a loop runs at one speed; eased if it does not loop.
        let crank = [[0.0, 2.0, 360.0]];
        assert!((seq_value(0.0, &crank, 0.2, Some(2.0)) - 36.0).abs() < 1e-9);
        assert!(seq_value(0.0, &crank, 0.2, None) < 20.0);
        // Two moves back to back, the same way: no stop between them.
        let two = [[0.0, 1.0, 90.0], [1.0, 1.0, 180.0]];
        let speed = |t: f64| (seq_value(0.0, &two, t + 1e-4, None) - seq_value(0.0, &two, t - 1e-4, None)) / 2e-4;
        assert!(speed(1.0) > 80.0, "{}", speed(1.0));
        assert_eq!(super::seq_end(&door), 4.0);
    }

    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn close(a: V3, b: V3) -> bool {
        (a - b).len() < 1e-9
    }

    fn link(parent: usize, child: usize, point: V3, axis: V3, slide: bool) -> JointLink {
        JointLink { comp: 0, half: 0, parent, child, point, axis, sign: 1.0, slide, travel: (-9.0, 9.0), free: false, settle: V3::ZERO }
    }

    #[test]
    fn draw_matrix_draws_points_where_the_pose_puts_them() {
        let t = Pose::turn(v3(0.3, -0.5, 0.8).norm(), v3(1.0, 2.0, -0.5), 0.7);
        let m = [0.0, 0.0, 1.0, 0.0, 0.6, 0.8, 0.0, 0.0, -0.8, 0.6, 0.0, 0.0, 10.0, -3.0, 4.0, 1.0];
        let o = [0.25, -1.0, 2.0];
        let draw = |m: &[f64; 16], g: [f64; 3]| -> [f64; 3] { std::array::from_fn(|k| (0..3).map(|j| (g[j] - o[j]) * m[j * 4 + k]).sum::<f64>() + m[12 + k]) };
        let posed = draw_matrix(&grid_move(&t), o, &m);
        let g = [1.5, -0.25, 3.0];
        let a = t.apply(v3(g[2], g[0], -g[1]));
        let want = draw(&m, [a.y, -a.z, a.x]);
        let got = draw(&posed, g);
        assert!((0..3).all(|k| (want[k] - got[k]).abs() < 1e-9), "{want:?} {got:?}");
    }

    #[test]
    fn pose_eases_in_and_back_out() {
        let joints = [link(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0), false)];
        let mut a = PoseAnim::default();
        let angle = |p: &[Pose]| p[1].rot[1].x.atan2(p[1].rot[0].x);
        assert!(a.poses(&joints, 2, &[1.0], true, false, 10.0).is_none(), "starts as built");
        let half = angle(&a.poses(&joints, 2, &[1.0], true, false, 10.5).unwrap());
        assert!((half - 0.5).abs() < 1e-9, "half way at half time: {half}");
        assert!((angle(&a.poses(&joints, 2, &[1.0], true, false, 11.0).unwrap()) - 1.0).abs() < 1e-9);
        // A new target eases on from where it is.
        let p = a.poses(&joints, 2, &[-1.0], true, false, 12.0).unwrap();
        assert!((angle(&p) - 1.0).abs() < 1e-9 && (angle(&a.poses(&joints, 2, &[-1.0], true, false, 13.0).unwrap()) + 1.0).abs() < 1e-9);
        assert!(a.poses(&joints, 2, &[-1.0], false, false, 13.5).is_some(), "leaving eases out");
        assert!(a.poses(&joints, 2, &[-1.0], false, false, 14.5).is_none(), "then shows as built");
    }

    #[test]
    fn turn_is_right_handed_about_its_point() {
        let t = Pose::turn(v3(0.0, 0.0, 1.0), v3(1.0, 0.0, 0.0), FRAC_PI_2);
        assert!(close(t.apply(v3(2.0, 0.0, 0.0)), v3(1.0, 1.0, 0.0)));
        assert!(close(t.apply(v3(1.0, 0.0, 5.0)), v3(1.0, 0.0, 5.0)), "points on the axis stay");
    }

    #[test]
    fn a_sub_grid_on_a_sub_grid_moves_with_its_parent() {
        // Body 1 turns 90° about z at the origin; body 2 slides 1 m along x on body 1.
        let joints = [link(0, 1, V3::ZERO, v3(0.0, 0.0, 1.0), false), link(1, 2, v3(5.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), true)];
        let poses = body_poses(&joints, 3, &|_, j| if j.slide { 1.0 } else { FRAC_PI_2 });
        assert_eq!(poses[0], Pose::IDENTITY);
        assert!(close(poses[1].apply(v3(1.0, 0.0, 0.0)), v3(0.0, 1.0, 0.0)));
        assert!(close(poses[2].apply(V3::ZERO), v3(0.0, 1.0, 0.0)));
        assert!(close(poses[2].dir(v3(1.0, 0.0, 0.0)), v3(0.0, 1.0, 0.0)));
        let zero = body_poses(&joints, 3, &|_, _| 0.0);
        assert!(zero.iter().all(|p| p.rot.iter().zip(Pose::IDENTITY.rot).all(|(a, b)| close(*a, b)) && close(p.trans, V3::ZERO)));
    }
}

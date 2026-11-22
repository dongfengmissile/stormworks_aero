//! Joints that carry sub-grids (pivots, robotic pivots, sliders, pistons, tracks) as a tree,
//! parent before child: what `pose` turns and the mechanisms check moves.

use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use crate::defs::Library;
use crate::math::{v3, V3};
use crate::vehicle::Comp;

/// A joint that carries a sub-grid: its parent half on `parent`, the child half on `child`.
#[derive(Clone, Debug, PartialEq)]
pub struct JointLink {
    /// The parent half's component, and the child half's.
    pub comp: usize,
    pub half: usize,
    pub parent: usize,
    pub child: usize,
    /// The joint's point and axis as built, aero frame (metres, unit): the game's hinge axis.
    pub point: V3,
    pub axis: V3,
    /// A positive setting turns the sub-grid by `sign`·setting about `axis` (right hand, aero
    /// frame): the game's own input direction, so mirrored joints given the same angle move mirrored.
    pub sign: f64,
    /// Slides along `axis` instead of turning about it.
    pub slide: bool,
    /// Where the game stops it, from the build pose along `axis` (rad, or metres for a slide);
    /// infinite for a joint whose input is a speed.
    pub travel: (f64, f64),
    /// Nothing drives it (a plain pivot, a suspension, a max power of 0): in a loop of
    /// joints it follows the others (`pose::close_loops`).
    pub free: bool,
    /// How far the game pulls the sub-grid as it settles, aero frame, metres: the parent half's
    /// joint point less the child half's (zero on stock joints; half a block on the modded
    /// half block offset).
    pub settle: V3,
}

/// A linear track's travel, metres (knowledge/vehicle-model.md "Sliders and linear tracks"): the
/// run of extension parts in line with the base along its part z (`z`, unmirrored), `fwd` on +z and
/// `back` on −z, gives −fwd·0.25 ..= back·0.25 along the slide axis (part −z).
pub fn track_travel(pos: [i32; 3], z: [i32; 3], is_ext: impl Fn([i32; 3]) -> bool) -> (f64, f64) {
    let run = |s: i32| (1..4096).take_while(|&k| is_ext(std::array::from_fn(|i| pos[i] + s * k * z[i]))).count() as f64;
    (-run(1) * 0.25, run(-1) * 0.25)
}

/// The vehicle's joints: each parent half (`Def::joint`) paired with the child half in the cell
/// its definition names, on another body. A body reached twice (a sub-grid held by two pivots)
/// keeps its first joint. Sorted so every parent body is placed before the bodies it carries.
pub fn joint_links(comps: &[Comp], lib: &mut Library) -> Vec<JointLink> {
    tree_links(all_joint_links(comps, lib), comps)
}

/// Every joint of the vehicle in part order, loops and second joints on one sub-grid included
/// (the game makes a constraint for each).
pub fn all_joint_links(comps: &[Comp], lib: &mut Library) -> Vec<JointLink> {
    let mut at: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for (i, c) in comps.iter().enumerate() {
        at.entry(c.pos).or_default().push(i);
    }
    let mut links: Vec<JointLink> = Vec::new();
    for (i, c) in comps.iter().enumerate() {
        let Some(j) = lib.get(&c.def).and_then(|d| d.joint.clone()) else { continue };
        let here = at.get(&c.cell(j.child_at)).map(Vec::as_slice).unwrap_or_default();
        let other = |k: &&usize| comps[**k].body != c.body;
        let Some(&k) = here.iter().filter(other).find(|&&k| comps[k].def == j.child).or_else(|| here.iter().find(other)) else { continue };
        let (parent, child) = (c.body as usize, comps[k].body as usize);
        // The game's hinge axis is the part frame's z after `M(axis)`: x, −y or z; a slider moves
        // along its x: x, y or −z (knowledge/vehicle-model.md "Type-7 joints").
        // It turns with the part's rotation before mirroring (`comp+0x30`), like the input sign below.
        let a = j.axis.min(2);
        let e = if j.slide { [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, -1.0)] } else { [v3(1.0, 0.0, 0.0), v3(0.0, -1.0, 0.0), v3(0.0, 0.0, 1.0)] }[a];
        let raw = Comp { rot: crate::vehicle::flipped(c.rot, c.mirror), def: String::new(), ..c.clone() };
        let axis = raw.part_to_aero(e) - raw.part_to_aero(V3::ZERO);
        // The game negates the input of a joint whose mirror flips its move (`comp+0x418`).
        let (mx, my, mz) = (c.mirror & 1 != 0, c.mirror & 2 != 0, c.mirror & 4 != 0);
        let invert = if j.slide { [mx, my, mz][a] } else { [my ^ mz, !(mx ^ mz), my ^ mx][a] };
        // A positive input turns the sub-grid right-handed about `axis` (aero frame) and slides it
        // along +axis, unless inverted. The hinge sign was the other way round until the labels
        // were compared with pivot inputs in game.
        let sign = if invert { -1.0 } else { 1.0 };
        let travel = match (j.slide, &j.extender) {
            (true, Some(ext)) if j.range <= 0.0 => {
                let z = [raw.rot[6], raw.rot[7], raw.rot[8]];
                let is_ext = |p: [i32; 3]| {
                    at.get(&p).is_some_and(|ks| ks.iter().any(|&k| {
                        let o = &comps[k];
                        // The parts' up rows (unmirrored, `comp+0x3C`) must match; extensions may face
                        // either way or be mirrored (symmetric builds, e.g. the "reach stacker").
                        o.body == c.body && (o.def == c.def || o.def == *ext) && crate::vehicle::flipped(o.rot, o.mirror)[3..6] == raw.rot[3..6]
                    }))
                };
                track_travel(c.pos, z, is_ext)
            }
            (true, _) => (-j.range / 2.0, j.range / 2.0),
            (false, _) if j.endless() => (f64::NEG_INFINITY, f64::INFINITY),
            (false, _) if j.range > 0.0 => (-j.range * PI, j.range * PI),
            (false, _) => (-PI, PI),
        };
        let point = c.part_to_aero(j.point);
        let settle = point - comps[k].part_to_aero(j.child_point);
        let free = j.passive() || (j.has_power() && c.power.is_some_and(|p| p.abs() < 1e-6));
        links.push(JointLink { comp: i, half: k, parent, child, point, axis: axis.norm(), sign, slide: j.slide, travel, free, settle });
    }
    links
}

/// The joints that place each sub-grid, parents first, grown out from the fixed body of each
/// group of jointed bodies (the lowest-numbered one no joint carries). A joint is taken either
/// way round: a sub-grid that only carries the other half (a piston whose pivot's parent half
/// sits on it) hangs from the sub-grid it is joined to. Driven joints are taken first, so free
/// joints are the ones left to close loops.
pub fn tree_links(all: Vec<JointLink>, comps: &[Comp]) -> Vec<JointLink> {
    let n = comps.iter().map(|c| c.body as usize + 1).max().unwrap_or(1).max(all.iter().map(|l| l.parent.max(l.child) + 1).max().unwrap_or(0));
    let carried: HashSet<usize> = all.iter().map(|l| l.child).collect();
    let roots: Vec<usize> = (0..n).filter(|b| !carried.contains(b)).chain(0..n).collect();
    let mut placed = vec![false; n];
    let mut used = vec![false; all.len()];
    let mut out = Vec::with_capacity(all.len());
    for r in roots {
        if placed[r] {
            continue;
        }
        placed[r] = true;
        // Driven before free, as built before turned round.
        while let Some((_, k, back)) = (0..all.len())
            .filter(|&k| !used[k])
            .filter_map(|k| match (placed[all[k].parent], placed[all[k].child]) {
                (true, false) => Some((2 * all[k].free as u8, k, false)),
                (false, true) => Some((2 * all[k].free as u8 + 1, k, true)),
                _ => None,
            })
            .min()
        {
            used[k] = true;
            let l = if back { turned_round(&all[k]) } else { all[k].clone() };
            placed[l.child] = true;
            out.push(l);
        }
    }
    out
}

/// A joint that carries its parent half's body from its child half's: the same move undone
/// (`child = parent·turn(point, v)·shift(settle)`, so `parent = child·turn(point − settle, −v)·shift(−settle)`).
/// Settings, travel and sign are negated together, so the same input gives the same pose.
pub fn turned_round(l: &JointLink) -> JointLink {
    JointLink { parent: l.child, child: l.parent, point: l.point - l.settle, settle: l.settle * -1.0, sign: -l.sign, travel: (-l.travel.1, -l.travel.0), ..l.clone() }
}


//! Logging builds, in game: the Bullet contacts between the bodies of each vehicle that has joint
//! loops, to learn when a shallow rub pushes (knowledge/vehicle-model.md "Contacts between a
//! vehicle's bodies"). Four times a second, while a loop joint moves or a contact pushes, writes
//! the loop joints' positions and every contact between the vehicle's bodies to
//! `UserData/logs/mechanisms_contacts.log`.

use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::sync::{LazyLock, Mutex};
use std::time::Instant;

use crate::editor::{body_comp_count, ptr_ok, ring, vehicle_bodies};
use crate::memory::{read_u32_at, read_usize_at};
use crate::offsets::*;

/// Physics ticks per log window (the sync runs once per 1/60 s tick, assumed; the log says).
const WINDOW: u32 = 15;
const BUDGET: usize = 64 << 20;

struct Joint {
    comp: usize,
    def: usize,
    pos: [i32; 3],
    bodies: (usize, usize),
    first: f32,
    last: f32,
    /// Largest change in one tick.
    step: f32,
}

#[derive(Default)]
struct Contact {
    bodies: (usize, usize),
    ticks: u32,
    pushed: u32,
    deepest: f64,
    /// Deepest point on each body, in its voxel grid.
    at: ([f64; 3], [f64; 3]),
    impulse_max: f64,
    impulse_sum: f64,
}

struct Vehicle {
    key: u64,
    bodies: Vec<usize>,
    joints: Vec<Joint>,
    /// Per manifold this window.
    contacts: HashMap<usize, Contact>,
}

#[derive(Default)]
struct Log {
    scene: usize,
    vehicles: HashMap<u32, Vehicle>,
    ticks: u32,
    start: Option<Instant>,
    path: Option<PathBuf>,
    bytes: usize,
}

static LOG: LazyLock<Mutex<Log>> = LazyLock::new(Mutex::default);

pub(crate) fn set_path(path: PathBuf) {
    let _ = std::fs::write(&path, b"");
    LOG.lock().unwrap_or_else(|p| p.into_inner()).path = Some(path);
}

unsafe fn f64_at(p: usize) -> f64 {
    std::ptr::read_unaligned(p as *const f64)
}

unsafe fn f32_at(p: usize) -> f32 {
    std::ptr::read_unaligned(p as *const f32)
}

/// The body's rigid body, checked by its pointer back to the physics object.
unsafe fn rigid_body(body: usize) -> Option<usize> {
    let obj = read_usize_at(body + OFF_BODY_PHYS_OBJ);
    if !ptr_ok(obj, OFF_OBJ_RIGID_BODY + 8) {
        return None;
    }
    let held = read_usize_at(obj + OFF_OBJ_RIGID_BODY);
    [held, obj + OFF_OBJ_RIGID_BODY].into_iter().find(|&rb| ptr_ok(rb, OFF_RB_USER + 8) && read_usize_at(rb + OFF_RB_USER) == obj)
}

// Compound children (knowledge/vehicle-model.md "Live shapes"), to check the plugin's collision
// records against the game's.
const OFF_OBJ_SHAPES: usize = 0x148; // ring of child shape pointers, one per record
const OFF_OBJ_COMPOUND: usize = 0x160;
const OFF_COMPOUND_COUNT: usize = 0x24;
const OFF_COMPOUND_CHILDREN: usize = 0x30;
const COMPOUND_CHILD_SIZE: usize = 160;
const OFF_CHILD_ORIGIN: usize = 0x60;
const OFF_CHILD_SHAPE: usize = 0x80;
const OFF_SHAPE_TYPE: usize = 0x08;
const OFF_BOX_HALF: usize = 0x40;
const OFF_BOX_MARGIN: usize = 0x60; // btConvexInternalShape's margin: hulls too
const OFF_SHAPE_POLYHEDRON: usize = 0x70;

/// Each child shape of the body, in child order: a box as its voxel range, else its type, centre
/// and whether it has a polyhedron.
unsafe fn shapes(body: usize) -> String {
    let obj = read_usize_at(body + OFF_BODY_PHYS_OBJ);
    if !ptr_ok(obj, OFF_OBJ_COMPOUND + 8) {
        return "no physics object".into();
    }
    let Some(list) = ring(obj + OFF_OBJ_SHAPES, 1 << 20) else { return "no shape ring".into() };
    // The compound is either inline or behind a pointer; take the one whose children match the ring.
    let compound = [obj + OFF_OBJ_COMPOUND, read_usize_at(obj + OFF_OBJ_COMPOUND)].into_iter().find(|&c| {
        ptr_ok(c, OFF_COMPOUND_CHILDREN + 8) && read_u32_at(c + OFF_COMPOUND_COUNT) as usize == list.len() && {
            let a = read_usize_at(c + OFF_COMPOUND_CHILDREN);
            ptr_ok(a, list.len() * COMPOUND_CHILD_SIZE) && list.iter().enumerate().all(|(k, &sh)| read_usize_at(a + k * COMPOUND_CHILD_SIZE + OFF_CHILD_SHAPE) == sh)
        }
    });
    let Some(c) = compound else { return format!("{} shapes, compound not found", list.len()) };
    let a = read_usize_at(c + OFF_COMPOUND_CHILDREN);
    let items: Vec<String> = list
        .iter()
        .enumerate()
        .map(|(k, &sh)| {
            let centre = voxel(body, a + k * COMPOUND_CHILD_SIZE + OFF_CHILD_ORIGIN);
            if !ptr_ok(sh, OFF_BOX_MARGIN + 8) {
                return format!("[{k}] bad");
            }
            let ty = read_u32_at(sh + OFF_SHAPE_TYPE);
            if ty == 0 {
                // Outer half extents are 2 mm short of the nominal voxel faces.
                let half = [0, 1, 2].map(|i| (f64_at(sh + OFF_BOX_HALF + 8 * i) + f64_at(sh + OFF_BOX_MARGIN) + 0.002) / 0.25);
                let r = |i: usize| format!("{:.0}..{:.0}", centre[i] - half[i] + 0.5, centre[i] + half[i] - 0.5);
                format!("[{k}] box {} {} {} margin {:.4}", r(0), r(1), r(2), f64_at(sh + OFF_BOX_MARGIN))
            } else {
                let poly = read_usize_at(sh + OFF_SHAPE_POLYHEDRON) != 0;
                format!("[{k}] type {ty} at ({:.2}, {:.2}, {:.2}) poly {poly} margin {:.4}", centre[0], centre[1], centre[2], f64_at(sh + OFF_BOX_MARGIN))
            }
        })
        .collect();
    items.join(" | ")
}

/// A local point (metres, body frame) in the body's voxel grid.
unsafe fn voxel(body: usize, p: usize) -> [f64; 3] {
    let o = body + OFF_BODY_ORIGIN;
    [0, 1, 2].map(|k| (f64_at(p + 8 * k) + f32_at(o + 4 * k) as f64) / 0.25)
}

/// Physics post-step sync (update thread, physics idle).
pub(crate) unsafe fn on_post_step_sync(physics_scene: usize) {
    let mut l = LOG.lock().unwrap_or_else(|p| p.into_inner());
    let l = &mut *l;
    let scene = physics_scene.wrapping_sub(OFF_SCENE_PHYSICS);
    if l.scene != scene {
        l.vehicles.clear(); // a world load swaps the scene: every pointer is stale
        l.scene = scene;
    }
    let start = *l.start.get_or_insert_with(Instant::now);
    let Some(records) = ring(scene + OFF_SCENE_VEHICLES, 1 << 16) else { return };
    let mut header = Vec::new();
    let mut seen = Vec::new();
    for rec in records {
        if !ptr_ok(rec, 0x18) || *(rec as *const u8) != 0 || read_u32_at(rec + 0x10) != 5 {
            continue;
        }
        let id = read_u32_at(rec + 4);
        let Some(bodies) = vehicle_bodies(read_usize_at(rec + 8)) else { continue };
        seen.push(id);
        let key = bodies.iter().flat_map(|&b| [b as u64, body_comp_count(b) as u64]).fold(0xcbf29ce484222325u64, |k, v| (k ^ v).wrapping_mul(0x100000001b3));
        if l.vehicles.get(&id).is_some_and(|v| v.key == key) {
            continue;
        }
        let joints = loop_joints(&bodies);
        if joints.is_empty() {
            l.vehicles.remove(&id);
            continue;
        }
        let parts: Vec<String> = bodies.iter().enumerate().map(|(i, &b)| format!("b{i} {} parts", body_comp_count(b))).collect();
        let js: Vec<String> = joints.iter().map(|j| format!("{:?} b{}-b{}", j.pos, j.bodies.0, j.bodies.1)).collect();
        header.push(format!("vehicle {id}: {}; loop joints at {}", parts.join(", "), js.join(", ")));
        for (i, &b) in bodies.iter().enumerate() {
            header.push(format!("  b{i} shapes: {}", shapes(b)));
        }
        l.vehicles.insert(id, Vehicle { key, bodies, joints, contacts: HashMap::new() });
    }
    l.vehicles.retain(|id, _| seen.contains(id));
    // Rigid body -> (vehicle, body index).
    let mut owner: HashMap<usize, (u32, usize)> = HashMap::new();
    for (&id, v) in &l.vehicles {
        for (i, &b) in v.bodies.iter().enumerate() {
            if let Some(rb) = rigid_body(b) {
                owner.insert(rb, (id, i));
            }
        }
    }
    for v in l.vehicles.values_mut() {
        for j in &mut v.joints {
            if !ptr_ok(j.comp, OFF_JOINT_INFO + 8) || read_usize_at(j.comp + OFF_COMP_DEF) != j.def {
                continue;
            }
            let info = read_usize_at(j.comp + OFF_JOINT_INFO);
            if ptr_ok(info, OFF_INFO_POSITION + 8) {
                let p = f32_at(info + OFF_INFO_POSITION);
                if l.ticks == 0 {
                    j.first = p;
                } else {
                    j.step = j.step.max((p - j.last).abs());
                }
                j.last = p;
            }
        }
    }
    let disp = read_usize_at(physics_scene + OFF_PHYS_DISPATCHER);
    if ptr_ok(disp, OFF_DISP_MANIFOLDS + 8) && !owner.is_empty() {
        let n = (read_u32_at(disp + OFF_DISP_MANIFOLD_COUNT) as usize).min(1 << 20);
        let arr = read_usize_at(disp + OFF_DISP_MANIFOLDS);
        if n > 0 && ptr_ok(arr, n * 8) {
            for i in 0..n {
                let m = read_usize_at(arr + 8 * i);
                if !ptr_ok(m, OFF_MANIFOLD_COUNT + 4) {
                    continue;
                }
                let (Some(&(v0, b0)), Some(&(v1, b1))) = (owner.get(&read_usize_at(m + OFF_MANIFOLD_BODY0)), owner.get(&read_usize_at(m + OFF_MANIFOLD_BODY1))) else { continue };
                let count = read_u32_at(m + OFF_MANIFOLD_COUNT).min(4) as usize;
                if v0 != v1 || count == 0 {
                    continue;
                }
                let v = l.vehicles.get_mut(&v0).expect("owner");
                let c = v.contacts.entry(m).or_default();
                if c.bodies != (b0, b1) {
                    *c = Contact { bodies: (b0, b1), deepest: f64::MIN, ..Default::default() };
                }
                c.ticks += 1;
                let mut pushed = false;
                for k in 0..count {
                    let p = m + OFF_MANIFOLD_POINTS + k * MANIFOLD_POINT_SIZE;
                    let (d, imp) = (-f64_at(p + OFF_POINT_DISTANCE), f64_at(p + OFF_POINT_IMPULSE));
                    if d > c.deepest {
                        c.deepest = d;
                        c.at = (voxel(v.bodies[b0], p + OFF_POINT_LOCAL_A), voxel(v.bodies[b1], p + OFF_POINT_LOCAL_B));
                    }
                    if imp > 0.0 {
                        pushed = true;
                        c.impulse_max = c.impulse_max.max(imp);
                        c.impulse_sum += imp;
                    }
                }
                c.pushed += pushed as u32;
            }
        }
    }
    l.ticks += 1;
    let mut out = header;
    if l.ticks >= WINDOW {
        let t = start.elapsed().as_secs_f64();
        for (id, v) in &mut l.vehicles {
            let moved = v.joints.iter().any(|j| (j.last - j.first).abs() > 1e-4);
            if moved || v.contacts.values().any(|c| c.pushed > 0) {
                let js: Vec<String> = v.joints.iter().map(|j| format!("{:?} {:.4}..{:.4} (max {:.4}/tick)", j.pos, j.first, j.last, j.step)).collect();
                out.push(format!("{t:.2} s, {} ticks, vehicle {id}: joints {}", l.ticks, js.join(", ")));
                let mut cs: Vec<&Contact> = v.contacts.values().collect();
                cs.sort_by(|a, b| b.deepest.total_cmp(&a.deepest));
                for c in cs {
                    let f = |p: [f64; 3]| format!("({:.2}, {:.2}, {:.2})", p[0], p[1], p[2]);
                    out.push(format!(
                        "  b{} {} / b{} {}: {:.1} mm deep, {} ticks, pushed {} ticks, impulse max {:.3} sum {:.3}",
                        c.bodies.0, f(c.at.0), c.bodies.1, f(c.at.1), c.deepest * 1000.0, c.ticks, c.pushed, c.impulse_max, c.impulse_sum
                    ));
                }
            }
            v.contacts.clear();
            for j in &mut v.joints {
                (j.first, j.step) = (j.last, 0.0);
            }
        }
        l.ticks = 0;
    }
    if !out.is_empty() && l.bytes < BUDGET {
        if let Some(path) = &l.path {
            if let Ok(mut f) = std::fs::OpenOptions::new().append(true).open(path) {
                let text = out.join("\n") + "\n";
                let _ = f.write_all(text.as_bytes());
                l.bytes += text.len();
            }
        }
    }
}

/// The vehicle's sub-grid joints that are on closed loops.
unsafe fn loop_joints(bodies: &[usize]) -> Vec<Joint> {
    let mut joints = Vec::new();
    for &body in bodies {
        for comp in ring(body + OFF_BODY_COMPONENTS, 1 << 20).unwrap_or_default() {
            if !ptr_ok(comp, OFF_JOINT_INFO + 8) {
                continue;
            }
            let def = read_usize_at(comp + OFF_COMP_DEF);
            // Type-7 parent halves only (the ones that carry sub-grids).
            if !ptr_ok(def, OFF_DEF_PROP_FLAGS + 4) || read_u32_at(def + OFF_DEF_CLASS) != 7 || read_u32_at(def + OFF_DEF_PROP_FLAGS) & 0x40 == 0 {
                continue;
            }
            let child = read_usize_at(comp + OFF_JOINT_CHILD);
            if !ptr_ok(child, OFF_COMP_BODY + 8) {
                continue;
            }
            let at = |b: usize| bodies.iter().position(|&x| x == b);
            let (Some(p), Some(c)) = (at(read_usize_at(comp + OFF_COMP_BODY)), at(read_usize_at(child + OFF_COMP_BODY))) else { continue };
            let pos = [0, 1, 2].map(|k| read_u32_at(comp + OFF_COMP_POS + 4 * k) as i32);
            joints.push(Joint { comp, def, pos, bodies: (p, c), first: 0.0, last: 0.0, step: 0.0 });
        }
    }
    let edges: Vec<(usize, usize)> = joints.iter().map(|j| j.bodies).collect();
    crate::mech::on_loops(&edges).into_iter().zip(joints).filter_map(|(on, j)| on.then_some(j)).collect()
}

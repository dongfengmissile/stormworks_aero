//! Component definitions: which voxels a part fills and how much.
//!
//! Read from the same files the game loads: `rom/data/definitions/<name>.xml`
//! and component-mod `.bin` files (u32 length + XML) from mods, the workshop and
//! the per-vehicle folders the game embeds them in.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::math::{v3, V3};
use crate::shape::{area, clip, Plane};
use crate::xml;

pub struct DefVoxel {
    pub pos: [i32; 3],
    /// `physics_shape` id (0 = full cube; see `shape.rs`).
    pub shape: i32,
    /// `physics_shape_rotation`, row-major.
    pub rot: [i32; 9],
    /// Planes around the part's mesh in this voxel (voxel units, voxel centre origin, solid
    /// where `n·p <= d`); None = no usable mesh, the physics shape alone.
    pub bounds: Option<Vec<Plane>>,
    /// Bounds a moving piece (a fin), not the part's static mesh.
    pub moving: bool,
}

pub struct Def {
    pub type_: i32,
    pub mass: f32,
    /// `rudder_type` of type-18 parts: 0 rudder/fin/control surface, 1 wing, 2 keel.
    pub rudder_type: i32,
    pub rudder_area: f32,
    /// Solid voxels only (sweep / reserved space of props, wheels, seats left out).
    pub voxels: Vec<DefVoxel>,
    /// `mesh_data_name`, empty when the part's moving pieces have no known rest pose.
    pub mesh: String,
    /// Moving pieces (`mesh_N_name`) and where the game draws them at rest.
    pub moving: Vec<(String, Pose)>,
    /// Flat mesh pieces (part frame, metres) that can lift on their own: fins too small for the
    /// voxel grid. `fins`: they are the part's aerodynamic fins (rocket fins), lifting unmarked.
    pub plates: Vec<(Vec<Tri>, Option<FinCtrl>)>,
    pub fins: bool,
    /// Voxels the game draws block faces on (`<surface shape>` ≠ 0), with the sides drawn as bits
    /// `1 << orientation` (0 +x, 1 −x, 2 +y, 3 −y, 4 +z, 5 −z): solid between them, whatever the mesh.
    pub faced: HashMap<[i32; 3], u8>,
    /// `block_type` > 0: a block the game draws from its surfaces, a mesh only adds to it.
    pub block: bool,
    /// Has logic nodes (a control surface's "Rotation" input).
    pub logic: bool,
    /// `dynamic_rotation_axes`: the hinge of a type-18 fin or control surface, part frame.
    pub hinge: V3,
    /// Air propeller or rotor with its own blade mesh (the plugin's blade-element model).
    pub prop: Option<PropDef>,
    /// A wheel's tyre (`Grid::build` adds `tyre_voxels` per part, for its size setting).
    pub wheel: Option<Wheel>,
}

/// A wheel's tyre as a cylinder in the part's frame. The game marks the cells a tyre sweeps as
/// reserved, not solid, and draws the wheel mesh with its own transform, so neither puts the
/// tyre in the air: this does, from the reserved cells (where) and the wheel's size
/// (knowledge/vehicle-model.md "Wheel draw").
#[derive(Clone, Debug)]
pub struct Wheel {
    /// Metres. Advanced wheels (type 41): `wheel_radius` / `wheel_width`. Simple wheels (type 4):
    /// their wheel mesh's size (their `wheel_radius` is about 1.5x the drawn wheel).
    pub radius: f64,
    pub width: f64,
    /// Middle of the reserved cells, voxel units.
    pub centre: V3,
    /// Part axis the axle lies along (`constraint_axis`).
    pub axle: usize,
    /// Advanced wheel with suspension travel (not a fixed or track wheel).
    pub sprung: bool,
    mesh: String,
}

impl Wheel {
    /// The tyres of one part: an advanced wheel's `wheel_size` setting adds 0.125 m of radius per
    /// step above 1 and scales the tyre with it; off the ground a sprung wheel hangs half a
    /// radius down its part +z; a double wheel has a second tyre one width further out the axle.
    pub fn tyres(&self, set: Option<(f64, bool)>) -> Vec<Wheel> {
        let mut w = self.clone();
        if let Some((s, _)) = set {
            w.radius = (self.radius + 0.125 * (s.clamp(0.0, 4.0) - 1.0)).max(0.05);
            w.width *= w.radius / self.radius;
        }
        if self.sprung {
            w.centre.z += 2.0 * w.radius;
        }
        let mut outer = w.clone();
        match w.axle {
            0 => outer.centre.x += 4.0 * w.width,
            1 => outer.centre.y += 4.0 * w.width,
            _ => outer.centre.z += 4.0 * w.width,
        }
        if set.is_some_and(|s| s.1) { vec![w, outer] } else { vec![w] }
    }
}
const TYPE_WHEEL: i32 = 4;
const TYPE_WHEEL_ADVANCED: i32 = 41;

/// The tyre's cells: each bounded by the cylinder (an octagon of the circle's area), so the grid
/// gets a wheel-shaped solid. Cells in `taken` (the part's own solid voxels) stay as they are.
pub fn tyre_voxels(w: &Wheel, taken: &std::collections::HashSet<[i32; 3]>) -> Vec<DefVoxel> {
    let (a, b, c) = (w.axle, (w.axle + 1) % 3, (w.axle + 2) % 3);
    let unit = |i: usize, s: f64| v3(if i == 0 { s } else { 0.0 }, if i == 1 { s } else { 0.0 }, if i == 2 { s } else { 0.0 });
    let (r, half) = (w.radius * 4.0, w.width * 2.0);
    let apothem = r * (std::f64::consts::PI / (8.0 * (std::f64::consts::PI / 8.0).tan())).sqrt();
    let mut planes = vec![Plane { n: unit(a, 1.0), d: half }, Plane { n: unit(a, -1.0), d: half }];
    planes.extend((0..8).map(|k| {
        let (sin, cos) = (std::f64::consts::TAU * k as f64 / 8.0).sin_cos();
        Plane { n: unit(b, cos) + unit(c, sin), d: apothem }
    }));
    let reach = |i: usize| if i == a { half } else { r };
    let range = |i: usize| ((w.centre.get(i) - reach(i)).round() as i32)..=((w.centre.get(i) + reach(i)).round() as i32);
    let mut out = Vec::new();
    for x in range(0) {
        for y in range(1) {
            for z in range(2) {
                let pos = [x, y, z];
                if taken.contains(&pos) {
                    continue;
                }
                let rel = v3(x as f64, y as f64, z as f64) - w.centre;
                let local: Vec<Plane> = planes.iter().map(|p| Plane { n: p.n, d: p.d - p.n.dot(rel) }).collect();
                // The cell's share inside the tyre on a 4x4x4 lattice: under a tenth is no cell.
                let at = |i: usize| -0.375 + 0.25 * i as f64;
                let inside = (0..64).filter(|i| local.iter().all(|p| p.n.dot(v3(at(i % 4), at(i / 4 % 4), at(i / 16))) <= p.d)).count();
                if inside >= 6 {
                    out.push(DefVoxel { pos, shape: 0, rot: crate::vehicle::IDENTITY, bounds: Some(local), moving: false });
                }
            }
        }
    }
    out
}

/// Type-2 air force emitter with a blade mesh (`mesh_1`): aircraft propeller, tail rotor,
/// helicopter rotors (knowledge/vehicle-model.md "Type 2 force emitters").
#[derive(Clone, Debug, Default)]
pub struct PropDef {
    /// Definition `flags`: bit 4 single pitch input (else cyclic + collective), 0x40000000 "coaxial" (one disc with a power output on top; the game adds a torque-reaction couple).
    pub flags: u32,
    /// Shaped as an aeroplane propeller (cambered, helical twist, rigid), not a helicopter rotor:
    /// a single pitch input (flag 4), or "propeller" in the name (mod propellers and the coaxial
    /// "Rotor Propeller" parts take cyclic and collective like rotors).
    pub airscrew: bool,
    /// `force_dir`: spin and thrust axis, part frame.
    pub axis: V3,
    /// `force_emitter_rotation_speed`: blade revolutions per unit of shaft speed.
    pub rotation_speed: f64,
    pub blade_height: f64,
    pub default_pitch: i32,
    /// `force_emitter_max_force`: shaft slot mass ×1000.
    pub max_force: f64,
    /// `mesh_1_name`: one blade.
    pub mesh: String,
    /// One blade's mesh points at scale 1 (part frame, metres, blade along −x, spin about y).
    pub blade: Vec<V3>,
    /// Tip and root radius, chord at 0.75 R, metres at scale 1.
    pub radius: f64,
    pub root: f64,
    pub chord: f64,
}

impl PropDef {
    /// Tip and root radius and the 0.75 R chord from the blade mesh (spin axis y); false if it
    /// is no blade.
    fn measure(&mut self) -> bool {
        let radial = |p: &V3| (p.x * p.x + p.z * p.z).sqrt();
        let Some(tip) = self.blade.iter().map(radial).max_by(f64::total_cmp) else {
            return false;
        };
        self.radius = tip;
        self.root = self.blade.iter().map(radial).fold(f64::MAX, f64::min);
        let slab: Vec<&V3> = self.blade.iter().filter(|p| (radial(p) - 0.75 * tip).abs() < 0.1 * tip).collect();
        let d = slab.iter().fold(V3::ZERO, |a, p| a + v3(p.x, 0.0, p.z)).norm();
        let t = v3(-d.z, 0.0, d.x);
        let (lo, hi) = slab.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(t.dot(**p)), hi.max(t.dot(**p))));
        self.chord = if hi > lo { hi - lo } else { 0.1 * tip };
        tip > 0.1 && self.root < tip
    }

    /// Cyclic and collective (rotor), not one pitch input (propeller, tail rotor).
    pub fn rotor(&self) -> bool {
        self.flags & 4 == 0
    }
    /// The game's blade scale from the part's `blade_length`.
    pub fn scale(length: f64) -> f64 {
        length * 0.4 + 0.8
    }
}

/// Joint halves of a pivot, hinge, suspension or turret (`multibody_*`, both halves).
pub const TYPE_MULTIBODY: i32 = 7;
pub const TYPE_RUDDER: i32 = 18;
const TYPE_FORCE_EMITTER: i32 = 2;
pub const TYPE_ROCKET: i32 = 47;

/// How the game deflects a rocket fin (`solid_rocket_draw` 0x43ECC0 / `rocket_tick` 0x439DF0,
/// c38c03ff756b): about `axis` (part frame, after the fin's quarter turn) by the angle of fin
/// pair A (`a`) or C, see `flight::fin_angle`.
#[derive(Clone, Copy, Debug)]
pub struct FinCtrl {
    pub axis: V3,
    pub a: bool,
}

/// Rest pose of a moving mesh in the part frame (build c38c03ff756b, knowledge/vehicle-model.md).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pose {
    /// Drawn once, shifted by `dynamic_body_position`·0.25 m (rudder_draw: fins, rudders).
    Shift(V3),
    /// Drawn 4×, turned 90° each about the rocket's y axis (solid_rocket_draw: fins).
    Cross4,
}

impl Pose {
    pub fn instances(self, tris: &[Tri]) -> Vec<Vec<Tri>> {
        match self {
            Pose::Shift(d) => vec![tris.iter().map(|(t, n)| (t.map(|p| p + d), *n)).collect()],
            Pose::Cross4 => (0..4)
                .map(|k| {
                    // (x, z) -> (z, -x) per quarter turn.
                    let turn = |mut v: V3| {
                        for _ in 0..k {
                            v = v3(v.z, v.y, -v.x);
                        }
                        v
                    };
                    tris.iter().map(|(t, n)| (t.map(turn), turn(*n))).collect()
                })
                .collect(),
        }
    }
}

impl Def {
    /// Fixed lifting part: vanilla wings, and type-18 "fins" without a deflection input
    /// (mod wing pieces such as Thales' wing_middle_3x3 omit `rudder_type`).
    pub fn is_wing(&self) -> bool {
        self.type_ == TYPE_RUDDER && (self.rudder_type == 1 || (self.rudder_type == 0 && !self.logic))
    }
    /// Movable vanilla control surface (fins, rudders, control surfaces: a "Rotation" input).
    pub fn is_control_surface(&self) -> bool {
        self.type_ == TYPE_RUDDER && self.rudder_type == 0 && self.logic
    }
}

pub fn parse_def(text: &str) -> Def {
    let mut def = Def {
        type_: 0,
        mass: 0.0,
        rudder_type: 0,
        rudder_area: 0.0,
        voxels: Vec::new(),
        mesh: String::new(),
        moving: Vec::new(),
        plates: Vec::new(),
        fins: false,
        faced: Default::default(),
        block: false,
        logic: false,
        hinge: V3::ZERO,
        prop: None,
        wheel: None,
    };
    // Reserved (not solid) voxels, and the wheel's size attributes.
    let (mut reserved, mut open_reserved, mut wheel_size, mut axle) = (Vec::<[i32; 3]>::new(), false, (0.0f64, None::<f64>), 0);
    let mut prop = PropDef::default();
    let mut names: [String; 3] = Default::default();
    let mut editor_only = false;
    let (mut rest_angle, mut dyn_pos) = (0.0f64, V3::ZERO);
    let mut in_voxels = false;
    // Inside a drawn <surface> of <surfaces> (not <buoyancy_surfaces>): its <position>
    // (omitted when 0,0,0) may follow.
    let (mut in_surfaces, mut face) = (false, None::<([i32; 3], u8)>);
    let add_face = |def: &mut Def, f: Option<([i32; 3], u8)>| {
        if let Some((pos, side)) = f {
            *def.faced.entry(pos).or_default() |= side;
        }
    };
    // Inside a solid <voxel>: its <position> (omitted when 0,0,0) and rotation may follow.
    let mut open = false;
    for tag in xml::tags(text) {
        match (tag.name, tag.close) {
            ("definition", false) => {
                def.type_ = tag.int0("type");
                def.mass = tag.num("mass").unwrap_or(0.0);
                def.rudder_type = tag.int0("rudder_type");
                def.rudder_area = tag.num("rudder_surface_area").unwrap_or(0.0);
                def.block = tag.int0("block_type") > 0;
                def.mesh = tag.get("mesh_data_name").unwrap_or("").to_string();
                names = ["mesh_0_name", "mesh_1_name", "mesh_2_name"].map(|k| tag.get(k).unwrap_or("").to_string());
                editor_only = !tag.get("mesh_editor_only_name").unwrap_or("").is_empty();
                // Fins and rudders rest at the middle of their range.
                rest_angle = (tag.num::<f64>("dynamic_min_rotation").unwrap_or(0.0) + tag.num::<f64>("dynamic_max_rotation").unwrap_or(0.0)) * 0.5;
                prop.flags = tag.num::<i64>("flags").unwrap_or(0) as u32;
                prop.airscrew = prop.flags & 4 != 0 || tag.get("name").is_some_and(|n| n.to_ascii_lowercase().contains("propeller"));
                prop.rotation_speed = tag.num("force_emitter_rotation_speed").unwrap_or(1.0);
                prop.blade_height = tag.num("force_emitter_blade_height").unwrap_or(0.0);
                prop.default_pitch = tag.int0("force_emitter_default_pitch");
                prop.max_force = tag.num("force_emitter_max_force").unwrap_or(1000.0);
                wheel_size = (tag.num("wheel_radius").unwrap_or(0.0), tag.num("wheel_width"));
                axle = tag.int0("constraint_axis").clamp(0, 2) as usize;
            }
            ("force_dir", false) => prop.axis = v3(tag.num("x").unwrap_or(0.0), tag.num("y").unwrap_or(0.0), tag.num("z").unwrap_or(0.0)),
            ("dynamic_rotation_axes", false) => def.hinge = v3(tag.num("x").unwrap_or(0.0), tag.num("y").unwrap_or(0.0), tag.num("z").unwrap_or(0.0)),
            ("dynamic_body_position", false) => dyn_pos = v3(tag.num("x").unwrap_or(0.0), tag.num("y").unwrap_or(0.0), tag.num("z").unwrap_or(0.0)) * 0.25,
            ("voxels", close) => in_voxels = !close && !tag.empty,
            ("surfaces", close) => in_surfaces = !close && !tag.empty,
            ("surface", false) if in_surfaces => {
                // A connection port (`trans_type`: fluid, hardpoint, ...) is an opening the game
                // marks for joining, not a square of hull.
                face = (tag.int0("shape") != 0 && tag.int0("trans_type") == 0).then_some(([0; 3], 1 << tag.int0("orientation").clamp(0, 5)));
                if tag.empty {
                    add_face(&mut def, face.take());
                }
            }
            ("surface", true) => add_face(&mut def, face.take()),
            ("position", false) if face.is_some() => face = face.map(|f| ([tag.int0("x"), tag.int0("y"), tag.int0("z")], f.1)),
            ("logic_node", false) => def.logic = true,
            ("voxel", false) if in_voxels => {
                let flags = tag.int0("flags");
                // bit 0 = solid, 4 = moving panel (control surface, door). 0/2 = reserved space
                // (prop discs, tyre sweep, seat occupant); an omitted attribute means 0.
                let solid = flags & 1 != 0 || flags & 4 != 0;
                if solid {
                    def.voxels.push(DefVoxel { pos: [0; 3], shape: tag.int0("physics_shape"), rot: crate::vehicle::IDENTITY, bounds: None, moving: false });
                }
                open = solid && !tag.empty;
                if !solid {
                    reserved.push([0; 3]);
                }
                open_reserved = !solid && !tag.empty;
            }
            ("voxel", true) => (open, open_reserved) = (false, false),
            ("position", false) if open_reserved => {
                if let Some(v) = reserved.last_mut() {
                    *v = [tag.int0("x"), tag.int0("y"), tag.int0("z")];
                }
            }
            ("position", false) if open => {
                if let Some(v) = def.voxels.last_mut() {
                    v.pos = [tag.int0("x"), tag.int0("y"), tag.int0("z")];
                }
            }
            ("physics_shape_rotation", false) if open => {
                // Entries equal to identity are omitted (`00="0" 02="-1" 20="1" 22="0"` keeps 11 = 1).
                let r: [i32; 9] = std::array::from_fn(|k| tag.num(&format!("{}{}", k / 3, k % 3)).unwrap_or((k % 4 == 0) as i32));
                if let Some(v) = def.voxels.last_mut() {
                    if crate::vehicle::valid_rotation(&r) {
                        v.rot = r;
                    }
                }
            }
            _ => {}
        }
    }
    if def.voxels.is_empty() {
        def.voxels.push(DefVoxel { pos: [0; 3], shape: 0, rot: crate::vehicle::IDENTITY, bounds: None, moving: false });
    }
    // Invisible parts (sensors, hidden weights): nothing drawn in game, only an editor-only mesh.
    // They take no space in the air (their mass stays, at the part's position).
    if editor_only && def.mesh.is_empty() && names.iter().all(|n| n.is_empty()) && def.faced.is_empty() && !def.block {
        def.voxels.clear();
    }
    // Moving meshes are drawn per type (RE of each class's draw). Spinning props, rotors and
    // fans (mesh_0 hub/rotor, mesh_1 blades) sweep a disc, not solid: their static mount only.
    // shortcut: other animated types, and fins resting off-centre (none installed), keep the
    // physics voxels; add their pose from the type's draw function when one matters.
    let [m0, m1, m2] = names;
    if matches!(def.type_, TYPE_WHEEL | TYPE_WHEEL_ADVANCED) && !reserved.is_empty() && wheel_size.0 > 0.0 {
        let n = reserved.len() as f64;
        let centre = reserved.iter().fold(V3::ZERO, |a, p| a + v3(p[0] as f64, p[1] as f64, p[2] as f64)) * (1.0 / n);
        // No width given: the advanced wheels' own proportion (0.25 m wide at 0.35 m radius).
        let sprung = def.type_ == TYPE_WHEEL_ADVANCED && prop.flags & 0x800000 == 0;
        def.wheel = Some(Wheel { radius: wheel_size.0, width: wheel_size.1.filter(|w| *w > 0.0).unwrap_or(0.7 * wheel_size.0), centre, axle, sprung, mesh: m0.clone() });
    }
    if !(m0.is_empty() && m1.is_empty() && m2.is_empty()) {
        // Air props and rotors with a blade mesh: the hub is drawn at rest, the blades per part
        // (their count, length and pitch are the part's settings): `PropDef::blade`.
        let air_blades = def.type_ == TYPE_FORCE_EMITTER && prop.flags & 2 == 0 && !m1.is_empty();
        if air_blades && prop.axis.len() > 0.5 {
            def.prop = Some(PropDef { mesh: m1.clone(), ..prop.clone() });
        }
        let pose = match def.type_ {
            TYPE_FORCE_EMITTER if air_blades => Some(Some(Pose::Shift(V3::ZERO))),
            TYPE_FORCE_EMITTER => Some(None),
            TYPE_ROCKET if m1.is_empty() && m2.is_empty() && rest_angle.abs() < 1e-6 => Some(Some(Pose::Cross4)),
            TYPE_RUDDER if m1.is_empty() && m2.is_empty() && rest_angle.abs() < 1e-6 => Some(Some(Pose::Shift(dyn_pos))),
            _ => None,
        };
        match pose {
            Some(Some(p)) => def.moving.push((m0, p)),
            Some(None) => {}
            None => def.mesh.clear(),
        }
    }
    def
}

/// Component-mod `.bin`: little-endian u32 length, then the definition XML followed by
/// binary mesh data inside the same length.
pub fn bin_xml(bytes: &[u8]) -> Option<&str> {
    let len = u32::from_le_bytes(bytes.get(..4)?.try_into().ok()?) as usize;
    let body = bytes.get(4..4 + len)?;
    let end = match std::str::from_utf8(body) {
        Ok(s) => return Some(s),
        Err(e) => e.valid_up_to(),
    };
    std::str::from_utf8(&body[..end]).ok()
}

/// The definition name in a mod part's header (u32 length, u32 1, name, NUL, XML); None for the
/// older layout that has no name.
fn bin_name(head: &[u8]) -> Option<&str> {
    if head.get(4..8)? != 1u32.to_le_bytes() {
        return None;
    }
    let name = head[8..].split(|&b| b == 0).next()?;
    (name.len() < head.len() - 8).then(|| std::str::from_utf8(name).ok()).flatten().filter(|n| !n.is_empty())
}

/// A mesh triangle (metres, part origin) and its outward unit normal.
pub type Tri = ([V3; 3], V3);

/// Triangles of a Stormworks `.mesh`; None if not that format.
/// Layout: "mesh" 07 00 01 00, u16 vertex count, u16, u16, vertices of 28 bytes
/// (f32 pos[3], rgba, f32 normal[3]), u32 index count, u16 indices, submeshes.
/// The outward side comes from the stored vertex normals (winding is not relied on).
pub fn mesh_triangles(b: &[u8]) -> Option<Vec<Tri>> {
    if b.get(..8)? != b"mesh\x07\x00\x01\x00" {
        return None;
    }
    let u16_at = |o: usize| Some(u16::from_le_bytes(b.get(o..o + 2)?.try_into().ok()?) as usize);
    let f32_at = |o: usize| Some(f32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?) as f64);
    let v3_at = |o: usize| Some(v3(f32_at(o)?, f32_at(o + 4)?, f32_at(o + 8)?));
    let vc = u16_at(8)?;
    let verts = (0..vc).map(|i| 14 + 28 * i).map(|o| Some((v3_at(o)?, v3_at(o + 16)?))).collect::<Option<Vec<(V3, V3)>>>()?;
    let o = 14 + 28 * vc;
    let ic = u32::from_le_bytes(b.get(o..o + 4)?.try_into().ok()?) as usize;
    let vert = |k: usize| verts.get(u16_at(o + 4 + 2 * k)?).copied();
    (0..ic / 3)
        .map(|t| {
            let (a, b, c) = (vert(3 * t)?, vert(3 * t + 1)?, vert(3 * t + 2)?);
            let n = (b.0 - a.0).cross(c.0 - a.0).norm();
            let flip = n.dot(a.1 + b.1 + c.1) < 0.0;
            Some(([a.0, b.0, c.0], if flip { -n } else { n }))
        })
        .collect()
}

/// Mesh file embedded in a component `.bin`: name, NUL, u32 length, data.
fn bin_mesh<'a>(bytes: &'a [u8], name: &str) -> Option<&'a [u8]> {
    let needle: Vec<u8> = name.bytes().chain([0]).collect();
    let at = bytes.windows(needle.len()).position(|w| w == needle)? + needle.len();
    let len = u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?) as usize;
    bytes.get(at + 4..at + 4 + len)
}

/// Thinnest slab kept for a mesh in a voxel (10 mm): flat glass is one-sided quads, and a
/// zero-volume cell would let the air flood into the cabin behind it.
const MIN_THICK: f64 = 0.04;
/// Face directions kept per voxel. shortcut: curved meshes beyond this get a looser hull, raise if needed.
const MAX_DIRS: usize = 24;

/// Bound the mesh cell by cell: support planes of its points along the axes and along the
/// mesh's own face normals (a k-DOP, so a slanted pane becomes a slanted slab). FAR voxelizes
/// the mesh; this is its convex per-cell version. The static mesh is kept to the part's voxels;
/// voxels it misses are dropped. In a cell, connected pieces that interpenetrate another (fins
/// rooted in a body) keep their own hulls; the rest merge into one (a valve's two end caps
/// still fill the cell between them). Each `moving` piece (one instance of a mesh_N, e.g. one
/// fin) is bounded on its own, so it is not merged with the body into one hull, and may reach
/// outside the part's voxels (fins overhanging a 1-voxel rocket).
pub fn apply_mesh(def: &mut Def, tris: &[Tri], moving: &[Vec<Tri>], shapes: &crate::shape::ShapeTable) {
    if def.block {
        return block_cuts(def, tris, shapes);
    }
    let physics: HashMap<[i32; 3], (i32, [i32; 9])> = def.voxels.iter().map(|v| (v.pos, (v.shape, v.rot))).collect();
    let mut out: Vec<DefVoxel> = Vec::new();
    // Faces are drawn on reserved cells too (the floor under a passenger seat's legroom).
    let mut faced: Vec<(&[i32; 3], &u8)> = def.faced.iter().collect();
    faced.sort();
    // Cells closed by their drawn faces (the rest are bounded by the mesh alone, below).
    let mut closed: HashSet<[i32; 3]> = HashSet::new();
    let axes = || vec![v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    // shortcut: every face counts as a full square; use the surface shape if cut faces matter.
    let square = |o: usize| {
        let (a, h) = (o / 2, if o % 2 == 0 { 0.5 } else { -0.5 });
        [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)].map(|(u, w)| {
            let mut q = [0.0; 3];
            (q[a], q[(a + 1) % 3], q[(a + 2) % 3]) = (h, u, w);
            v3(q[0], q[1], q[2])
        })
    };
    for (pos, &sides) in faced {
        let (shape, rot) = physics.get(pos).copied().unwrap_or((0, crate::vehicle::IDENTITY));
        let mesh = cell_points(tris, *pos, true);
        let sides_on = (0..6).filter(|o| sides >> o & 1 != 0);
        // One or two faces do not close a cell: with a mesh in it they are plates of their own
        // (a hardpoint's hook on its base plate). One hull around both would fill the cell.
        if sides.count_ones() <= 2 && mesh.is_some() {
            out.extend(sides_on.map(|o| DefVoxel { pos: *pos, shape, rot, bounds: Some(hull(&square(o), &axes())), moving: false }));
            continue;
        }
        closed.insert(*pos);
        // A voxel the game draws faces on is closed by them: one hull around faces and mesh
        // (a sensor is five block faces and a mesh on the sixth side; a lone face is a thin plate).
        let (mut pts, dirs) = mesh.unwrap_or((Vec::new(), axes()));
        sides_on.for_each(|o| pts.extend(square(o)));
        out.push(DefVoxel { pos: *pos, shape, rot, bounds: Some(hull(&pts, &dirs)), moving: false });
    }
    if out.is_empty() && !tris.is_empty() && !def.voxels.iter().any(|v| cell_points(tris, v.pos, true).is_some()) {
        return; // mesh not in the voxels' frame: keep the physics shapes (drawn faces: it sits on them)
    }
    let static_pieces: Vec<Vec<Tri>> = pieces(tris).into_iter().map(|p| p.into_iter().map(|t| tris[t]).collect()).collect();
    for (k, piece) in moving.iter().chain(&static_pieces).enumerate() {
        let (mut lo, mut hi) = ([i32::MAX; 3], [i32::MIN; 3]);
        for p in piece.iter().flat_map(|t| t.0) {
            for a in 0..3 {
                // Voxel units; cells c with |u - c| <= 0.5 (a point on a face touches both).
                lo[a] = lo[a].min((p.get(a) * 4.0 - 0.5).ceil() as i32);
                hi[a] = hi[a].max((p.get(a) * 4.0 + 0.5).floor() as i32);
            }
        }
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    let pos = [x, y, z];
                    if closed.contains(&pos) {
                        continue;
                    }
                    let own = physics.contains_key(&pos);
                    if let Some(b) = kdop(piece, pos, own) {
                        let (shape, rot) = physics.get(&pos).copied().unwrap_or((0, crate::vehicle::IDENTITY));
                        out.push(DefVoxel { pos, shape, rot, bounds: Some(b), moving: k < moving.len() });
                    }
                }
            }
        }
    }
    if !out.is_empty() {
        def.voxels = out;
    }
}

/// k-DOP of the triangles inside cell `pos` (voxel units, cell centre origin); None if none
/// are. `own`: the cell is one of the part's voxels; otherwise a face lying on its boundary
/// belongs to the neighbour, not to it.
fn kdop(tris: &[Tri], pos: [i32; 3], own: bool) -> Option<Vec<Plane>> {
    let (pts, dirs) = cell_points(tris, pos, own)?;
    Some(hull(&pts, &dirs))
}

/// Points of the triangles inside cell `pos` (voxel units, cell centre origin) and the face
/// directions to bound them along (axes first).
fn cell_points(tris: &[Tri], pos: [i32; 3], own: bool) -> Option<(Vec<V3>, Vec<V3>)> {
    let c = v3(pos[0] as f64, pos[1] as f64, pos[2] as f64);
    let mut pts: Vec<V3> = Vec::new();
    let mut dirs: Vec<V3> = vec![v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    for (t, n) in tris {
        let Some(poly) = in_cell(t, c) else { continue };
        if !own && (0..3).any(|a| poly.iter().all(|q| q.get(a).abs() > 0.5 - 1e-6) && poly.iter().all(|q| q.get(a).signum() == poly[0].get(a).signum())) {
            continue;
        }
        let n = *n;
        if dirs.len() < MAX_DIRS && n.len() > 0.5 && !dirs.iter().any(|d| d.dot(n).abs() > 0.999) {
            dirs.push(n);
        }
        pts.extend(poly);
    }
    // Outside the part, a mesh reaching under 1/10 voxel into a cell (window frames lapping onto
    // the next block by 1.5 cm, a fin edge 2 mm over) is no new cell.
    // shortcut: fixed depth cut-off; make it volume-based if real overhangs this thin matter.
    // Along one axis, on one side: a rod running through the cell also has all its points on the faces.
    if pts.is_empty() || (!own && (0..3).any(|a| pts.iter().all(|q| q.get(a) > 0.4) || pts.iter().all(|q| q.get(a) < -0.4))) {
        return None;
    }
    Some((pts, dirs))
}

/// k-DOP of `pts` along `dirs`, flat spans thickened to `MIN_THICK`.
fn hull(pts: &[V3], dirs: &[V3]) -> Vec<Plane> {
    let planes = dirs.iter().flat_map(|&n| {
        let hi = pts.iter().map(|p| n.dot(*p)).fold(f64::MIN, f64::max);
        let lo = pts.iter().map(|p| n.dot(*p)).fold(f64::MAX, f64::min);
        // Thicken flat spans, staying inside the cell along the axes.
        let (lo, hi) = if hi - lo >= MIN_THICK {
            (lo, hi)
        } else {
            let lim = if n.x.abs().max(n.y.abs()).max(n.z.abs()) > 0.999 { 0.5 - MIN_THICK / 2.0 } else { f64::MAX };
            let m = ((lo + hi) * 0.5).clamp(-lim, lim);
            (m - MIN_THICK / 2.0, m + MIN_THICK / 2.0)
        };
        [Plane { n, d: hi }, Plane { n: -n, d: -lo }]
    });
    planes.collect()
}

/// Convex solid around `pts` (k-DOP along the axes and `dirs`), for display.
pub fn hull_solid(pts: &[V3], dirs: &[V3]) -> crate::shape::Solid {
    if pts.is_empty() {
        return Vec::new();
    }
    let all: Vec<V3> = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)].into_iter().chain(dirs.iter().map(|d| d.norm())).collect();
    let c = pts.iter().fold(V3::ZERO, |a, p| a + *p) * (1.0 / pts.len() as f64);
    let size = 2.0 * pts.iter().map(|p| (*p - c).len()).fold(0.0, f64::max) + 1.0;
    let mut s: crate::shape::Solid = crate::shape::voxel_solid(None).into_iter().map(|f| f.into_iter().map(|p| p * size + c).collect()).collect();
    for p in hull(pts, &all) {
        s = crate::shape::clip_solid(&s, &p);
    }
    s
}

/// A flat piece big enough to lift: thinner than a quarter of its width, over ~20 cm² of outline.
fn is_plate(piece: &[Tri]) -> bool {
    let pts: Vec<V3> = piece.iter().flat_map(|t| t.0).collect();
    let Some(e) = crate::wing::extents(&pts) else {
        return false;
    };
    e[0] < 0.25 * e[1] && e[1] * e[2] > 0.004
}

/// Triangle indices grouped into connected pieces (triangles sharing a vertex position).
fn pieces(tris: &[Tri]) -> Vec<Vec<usize>> {
    fn find(p: &mut [usize], mut i: usize) -> usize {
        while p[i] != i {
            p[i] = p[p[i]];
            i = p[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..tris.len()).collect();
    let mut first: HashMap<[i64; 3], usize> = HashMap::new();
    for (t, (pts, _)) in tris.iter().enumerate() {
        for p in pts {
            let key = [p.x, p.y, p.z].map(|v| (v * 1e4).round() as i64); // 0.1 mm
            let o = *first.entry(key).or_insert(t);
            let (a, b) = (find(&mut parent, o), find(&mut parent, t));
            parent[a] = b;
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut slot: HashMap<usize, usize> = HashMap::new();
    for t in 0..tris.len() {
        let r = find(&mut parent, t);
        let k = *slot.entry(r).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[k].push(t);
    }
    groups
}

/// Triangle in voxel units relative to cell centre `c`, clipped to that cell; None if it
/// only touches the cell along an edge.
fn in_cell(t: &[V3; 3], c: V3) -> Option<Vec<V3>> {
    let mut poly: Vec<V3> = t.iter().map(|p| *p * 4.0 - c).collect();
    for a in 0..3 {
        for s in [1.0, -1.0] {
            let mut n = [0.0; 3];
            n[a] = s;
            poly = clip(&poly, &Plane { n: v3(n[0], n[1], n[2]), d: 0.5 });
        }
    }
    (area(&poly) >= 1e-9).then_some(poly)
}

/// Blocks: the game draws their faces from the block definition, so a mesh is either trim
/// (all faces axis-aligned, e.g. the end cap of Thales' "Wedge 1x2 LW Tip") or a slope the
/// physics voxels can only approximate (Thales' 1x6 wedge: physics is 2 cubes + a 4-long
/// wedge, the mesh is the true 6-long slope). Keep each voxel's physics shape and cut it
/// under every slanted mesh face inside the voxel; never drop a voxel. Where the mesh pokes
/// out of the physics shape (Thales' "recpyramid" family: hip-roof pyramids no physics shape
/// can express), the physics is wrong there: start from the full cube instead.
fn block_cuts(def: &mut Def, tris: &[Tri], shapes: &crate::shape::ShapeTable) {
    for v in &mut def.voxels {
        let c = v3(v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64);
        let phys = shapes.plane(v.shape).map(|p| crate::shape::rotate(p, &v.rot));
        let mut cuts: Vec<Plane> = Vec::new();
        let mut outside = 0.0f64;
        for (t, n) in tris {
            let slanted = n.x.abs().max(n.y.abs()).max(n.z.abs()) < 0.999;
            if !slanted || n.len() < 0.5 {
                continue;
            }
            let Some(poly) = in_cell(t, c) else { continue };
            if let Some(pl) = phys {
                outside = poly.iter().map(|q| (pl.n.dot(*q) - pl.d) / pl.n.len()).fold(outside, f64::max);
            }
            let d = n.dot(poly[0]);
            if !cuts.iter().any(|q| q.n.dot(*n) > 0.9999 && (q.d - d).abs() < 1e-6) {
                cuts.push(Plane { n: *n, d });
            }
        }
        if !cuts.is_empty() {
            if outside > 0.02 {
                (v.shape, v.rot) = (0, crate::vehicle::IDENTITY);
            }
            v.bounds = Some(cuts);
        }
    }
}

/// Name -> definition, loaded on first use.
pub struct Library {
    rom: PathBuf,
    pub shapes: crate::shape::ShapeTable,
    bin_dirs: Vec<PathBuf>,
    /// Mod part files by the definition name in their header. The game keys a mod part by that
    /// name, not the file name, and it replaces a stock part of the same name.
    named: HashMap<String, PathBuf>,
    cache: HashMap<String, Option<Arc<Def>>>,
}

impl Library {
    /// `game`: the Stormworks install folder. `appdata`: `%APPDATA%/Stormworks`.
    pub fn new(game: &Path, appdata: &Path) -> Self {
        let mut bin_dirs = vec![appdata.join("data/vehicles/tmp/autosave")];
        let mut push_children = |root: PathBuf, sub: &str| {
            if let Ok(rd) = std::fs::read_dir(root) {
                let mut dirs: Vec<PathBuf> = rd.flatten().map(|e| e.path().join(sub)).filter(|p| p.is_dir()).collect();
                dirs.sort();
                bin_dirs.extend(dirs);
            }
        };
        push_children(appdata.join("data/mods"), "data/components");
        if let Some(steamapps) = game.parent().and_then(Path::parent) {
            push_children(steamapps.join("workshop/content/573090"), "data/components");
        }
        // Vehicles carry the definitions of every mod part they use.
        push_children(appdata.join("data/vehicles"), "");
        let rom = game.join("rom/data/definitions");
        let mut named = HashMap::new();
        for dir in &bin_dirs {
            let mut files: Vec<PathBuf> = std::fs::read_dir(dir).into_iter().flatten().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "bin")).collect();
            files.sort();
            for p in files {
                let mut head = [0u8; 136];
                let n = std::fs::File::open(&p).and_then(|mut f| std::io::Read::read(&mut f, &mut head)).unwrap_or(0);
                if let Some(name) = bin_name(&head[..n]) {
                    named.entry(name.to_string()).or_insert(p);
                }
            }
        }
        Library { shapes: crate::shape::ShapeTable::from_rom(&rom), rom, bin_dirs, named, cache: HashMap::new() }
    }

    /// Whether an installed mod part replaces the stock part of this name.
    pub fn replaced_by_mod(&self, name: &str) -> bool {
        self.named.contains_key(name)
    }

    pub fn get(&mut self, name: &str) -> Option<Arc<Def>> {
        if let Some(hit) = self.cache.get(name) {
            return hit.clone();
        }
        let def = self.load(name).map(Arc::new);
        self.cache.insert(name.to_string(), def.clone());
        def
    }

    fn load(&self, name: &str) -> Option<Def> {
        // Names come from game memory / vehicle files: never let one escape the folders.
        if name.is_empty() || name.contains(['/', '\\', ':']) || name.contains("..") {
            return None;
        }
        let rom_root = self.rom.parent().and_then(Path::parent).unwrap_or(&self.rom);
        let rom_mesh = |m: &str| (!m.contains("..") && !m.contains(':') && !m.starts_with(['/', '\\'])).then(|| std::fs::read(rom_root.join(m)).ok()).flatten();
        // Mesh by name: embedded in the part's .bin (mods), else from rom.
        let finish = |mut def: Def, bytes: &[u8]| {
            let get = |m: &str| {
                (!m.is_empty()).then(|| bin_mesh(bytes, m).and_then(mesh_triangles).or_else(|| rom_mesh(m).and_then(|b| mesh_triangles(&b)))).flatten()
            };
            let tris = get(&def.mesh);
            // Rocket fin k (k quarter turns from the authored fin): even k = pair C, axis (1,0,1)/√2;
            // odd = pair A, axis (−1,0,1)/√2.
            let fin = |k: usize| FinCtrl { axis: if k % 2 == 0 { v3(1.0, 0.0, 1.0) } else { v3(-1.0, 0.0, 1.0) }.norm(), a: k % 2 == 1 };
            let tagged: Vec<(Vec<Tri>, Option<FinCtrl>)> = def
                .moving
                .iter()
                .filter_map(|(m, p)| {
                    Some(p.instances(&get(m)?).into_iter().enumerate().map(|(k, t)| (t, (*p == Pose::Cross4).then(|| fin(k)))).collect::<Vec<_>>())
                })
                .flatten()
                .collect();
            let moving: Vec<Vec<Tri>> = tagged.iter().map(|t| t.0.clone()).collect();
            if let Some(p) = def.prop.as_mut() {
                let blade = get(&p.mesh).unwrap_or_default();
                p.blade = blade.iter().flat_map(|t| t.0).collect();
                if !p.measure() {
                    def.prop = None;
                }
            }
            def.fins = def.type_ == TYPE_ROCKET && !moving.is_empty();
            if !def.block {
                let pieces = tris.iter().flat_map(|t| pieces(t).into_iter().map(|p| p.into_iter().map(|i| t[i]).collect::<Vec<Tri>>()));
                def.plates = tagged.into_iter().filter(|p| def.fins || is_plate(&p.0)).chain(pieces.filter(|p| is_plate(p)).map(|p| (p, None))).collect();
            }
            if tris.is_some() || !moving.is_empty() {
                apply_mesh(&mut def, &tris.unwrap_or_default(), &moving, &self.shapes);
            }
            if let Some(mut w) = def.wheel.take() {
                // A simple wheel's drawn size: its mesh is the tyre alone, thinnest along the axle.
                if let Some(t) = get(&w.mesh).filter(|_| def.type_ == TYPE_WHEEL) {
                    let mut ext = [0, 1, 2].map(|a| {
                        let (lo, hi) = t.iter().flat_map(|t| t.0).fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.get(a)), hi.max(p.get(a))));
                        hi - lo
                    });
                    ext.sort_by(f64::total_cmp);
                    if ext[0] > 0.0 {
                        (w.width, w.radius) = (ext[0], 0.5 * ext[2]);
                    }
                }
                def.wheel = Some(w);
            }
            def
        };
        if let Some(bytes) = self.named.get(name).and_then(|p| std::fs::read(p).ok()) {
            if let Some(xml) = bin_xml(&bytes) {
                return Some(finish(parse_def(xml), &bytes));
            }
        }
        if let Ok(text) = std::fs::read_to_string(self.rom.join(format!("{name}.xml"))) {
            return Some(finish(parse_def(&text), &[]));
        }
        self.bin_dirs.iter().find_map(|dir| {
            let bytes = std::fs::read(dir.join(format!("{name}.bin"))).ok()?;
            Some(finish(parse_def(bin_xml(&bytes)?), &bytes))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn editor_only_parts_take_no_space() {
        // An invisible sensor: no drawn mesh or faces, only an editor arrow.
        let inv = parse_def(
            r#"<definition mesh_data_name="" mesh_editor_only_name="arrow.mesh"><surfaces><surface shape="0"/></surfaces><voxels><voxel flags="1"/></voxels></definition>"#,
        );
        assert!(inv.voxels.is_empty());
        // Drawn by its faces (a sign), or with no editor mesh at all (a microcontroller): kept.
        let sign = parse_def(
            r#"<definition mesh_data_name="" mesh_editor_only_name="arrow.mesh"><surfaces><surface shape="1"/></surfaces><voxels><voxel flags="1"/></voxels></definition>"#,
        );
        let mc = parse_def(r#"<definition mesh_data_name=""><voxels><voxel flags="1"/></voxels></definition>"#);
        assert_eq!((sign.voxels.len(), mc.voxels.len()), (1, 1));
    }

    #[test]
    fn wedge_and_reserved_voxels() {
        let def = parse_def(
            r#"<definition type="18" mass="8" rudder_type="1" rudder_surface_area="8"><voxels>
            <voxel flags="1" physics_shape="1"><position x="1"/><physics_shape_rotation 00="1" 11="1" 22="1"/></voxel>
            <voxel flags="2"><position y="1"/></voxel>
            <voxel flags="4"><position z="-1"/></voxel>
            <voxel flags="1"><physics_shape_rotation 00="0" 02="-1" 20="1" 22="0"/></voxel>
            </voxels></definition>"#,
        );
        assert!(def.is_wing());
        let fin = |extra: &str| parse_def(&format!(r#"<definition type="18" rudder_surface_area="9"><logic_nodes>{extra}</logic_nodes></definition>"#));
        assert!(fin(r#"<logic_node label="Rotation" mode="1" type="1"/>"#).is_control_surface());
        assert!(fin("").is_wing() && !fin("").is_control_surface(), "a fin without input is a fixed wing piece");
        assert_eq!(def.voxels.len(), 3);
        assert_eq!(def.voxels[2].pos, [0, 0, 0]);
        assert_eq!(def.voxels[0].pos, [1, 0, 0]);
        assert_eq!(def.voxels[0].shape, 1);
        assert_eq!(def.voxels[0].rot, crate::vehicle::IDENTITY);
        assert_eq!(def.voxels[2].rot, [0, 0, -1, 0, 1, 0, 1, 0, 0], "omitted 11 is 1");
        assert_eq!(def.voxels[1].pos, [0, 0, -1]);
    }

    #[test]
    fn flat_glass_mesh_is_a_thin_slab() {
        // One-sided quad on the -x face of voxel 0 (like the modded flat window), part of 2 voxels.
        let quad = [[-0.125f32, -0.125, -0.125], [-0.125, 0.125, -0.125], [-0.125, 0.125, 0.125], [-0.125, -0.125, 0.125]];
        let mut b = b"mesh\x07\x00\x01\x00".to_vec();
        b.extend(4u16.to_le_bytes());
        b.extend([0x13, 0, 0, 0]);
        for v in quad {
            v.iter().for_each(|f| b.extend(f.to_le_bytes()));
            b.extend([0u8; 16]);
        }
        b.extend(6u32.to_le_bytes());
        [0u16, 1, 2, 0, 2, 3].iter().for_each(|i| b.extend(i.to_le_bytes()));
        let tris = mesh_triangles(&b).unwrap();
        assert_eq!(tris.len(), 2);
        assert!(tris[0].1.x.abs() > 0.99, "zero vertex normals keep the winding normal");
        let mut def =
            parse_def(r#"<definition mesh_data_name="m.mesh"><voxels><voxel flags="1"/><voxel flags="1"><position z="1"/></voxel></voxels></definition>"#);
        assert_eq!(def.mesh, "m.mesh");
        apply_mesh(&mut def, &tris, &[], &Default::default());
        assert_eq!(def.voxels.len(), 1, "the voxel without mesh is dropped");
        let bnd = def.voxels[0].bounds.as_ref().unwrap();
        let s = bnd.iter().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p));
        assert!((crate::shape::solid_volume(&s) - MIN_THICK).abs() < 1e-9, "{}", crate::shape::solid_volume(&s));
        // A pane across the cell diagonal becomes a slanted slab, not the whole cell.
        let diag = [v3(-0.125, -0.125, -0.125), v3(-0.125, 0.125, -0.125), v3(0.125, 0.125, 0.125), v3(0.125, -0.125, 0.125)];
        let mut def = parse_def("<definition/>");
        let dn = (diag[1] - diag[0]).cross(diag[2] - diag[0]).norm();
        apply_mesh(&mut def, &[([diag[0], diag[1], diag[2]], dn), ([diag[0], diag[2], diag[3]], dn)], &[], &Default::default());
        let s = def.voxels[0].bounds.as_ref().unwrap().iter().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p));
        let v = crate::shape::solid_volume(&s);
        assert!(v > 0.04 && v < 0.07, "{v}");

        // Block: a 6-long slope mesh over physics "2 cubes + steeper wedge" cuts each voxel to
        // the true slope; an axis-aligned trim mesh leaves the physics shape alone.
        let (top, bot) = (v3(0.0, 0.125, 0.125), v3(0.0, -0.125, -0.625)); // metres: 3 voxels long
        let n = v3(0.0, 3.0, -1.0).norm(); // outward: up and back (rise 1 over 3)
        let quad = |x: f64| v3(x, 0.0, 0.0);
        let slope = [([top + quad(-0.125), top + quad(0.125), bot + quad(0.125)], n), ([bot + quad(0.125), bot + quad(-0.125), top + quad(-0.125)], n)];
        let mut def = parse_def(
            r#"<definition block_type="7" mesh_data_name="m"><voxels><voxel flags="1"/><voxel flags="1"><position z="-1"/></voxel><voxel flags="1"><position z="-2"/></voxel></voxels></definition>"#,
        );
        assert!(def.block);
        apply_mesh(&mut def, &slope, &[], &Default::default());
        let vol: f64 = def
            .voxels
            .iter()
            .map(|v| crate::shape::solid_volume(&v.bounds.iter().flatten().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p))))
            .sum();
        assert!((vol - 1.5).abs() < 1e-6, "3-long wedge over 3 cubes: {vol}");
        let mut def = parse_def(r#"<definition block_type="4" mesh_data_name="cap"/>"#);
        apply_mesh(&mut def, &[([v3(-0.125, -0.125, 0.125), v3(0.125, -0.125, 0.125), v3(0.125, 0.0, 0.125)], v3(0.0, 0.0, 1.0))], &[], &Default::default());
        assert!(def.voxels[0].bounds.is_none(), "axis-aligned trim keeps the block shape");
        let mut bin = b"x\0".to_vec();
        bin.extend(b"m.mesh\0");
        bin.extend((b.len() as u32).to_le_bytes());
        bin.extend(&b);
        assert_eq!(bin_mesh(&bin, "m.mesh"), Some(&b[..]));
    }

    /// Every installed definition: solid volume and cell count, for comparing voxelizer changes
    /// (`cargo test --release audit_defs -- --ignored --nocapture > out`).
    #[test]
    fn mod_part_replaces_stock_part_of_the_same_header_name() {
        let root = std::env::temp_dir().join(format!("aero_named_{}", std::process::id()));
        let (rom, mods) = (root.join("game/rom/data/definitions"), root.join("app/data/mods/m/data/components"));
        std::fs::create_dir_all(&rom).unwrap();
        std::fs::create_dir_all(&mods).unwrap();
        std::fs::write(rom.join("01_block_weight.xml"), r#"<definition mass="10"/>"#).unwrap();
        let body = [&1u32.to_le_bytes()[..], b"01_block_weight\0", br#"<definition mass="50"/>"#].concat();
        std::fs::write(mods.join("block_weight_50.bin"), [&(body.len() as u32).to_le_bytes()[..], &body].concat()).unwrap();
        let mut lib = Library::new(&root.join("game"), &root.join("app"));
        assert_eq!(lib.get("01_block_weight").unwrap().mass, 50.0);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    #[ignore]
    fn audit_defs() {
        let home = std::env::var("HOME").unwrap();
        let game = Path::new(&home).join(".local/share/Steam/steamapps/common/Stormworks");
        let appdata = Path::new(&home).join(".local/share/Steam/steamapps/compatdata/573090/pfx/drive_c/users/steamuser/AppData/Roaming/Stormworks");
        let mut lib = Library::new(&game, &appdata);
        let mut names: Vec<String> = Vec::new();
        for dir in std::iter::once(lib.rom.clone()).chain(lib.bin_dirs.clone()) {
            for e in std::fs::read_dir(&dir).into_iter().flatten().flatten() {
                let p = e.path();
                if matches!(p.extension().and_then(|x| x.to_str()), Some("xml" | "bin")) {
                    names.push(p.file_stem().unwrap().to_string_lossy().into_owned());
                }
            }
        }
        names.sort();
        names.dedup();
        for n in names {
            let Some(def) = lib.get(&n) else { continue };
            let vol: f64 = def
                .voxels
                .iter()
                .map(|v| {
                    let pl = lib.shapes.plane(v.shape).map(|p| crate::shape::rotate(p, &v.rot));
                    crate::shape::solid_volume(&v.bounds.iter().flatten().fold(crate::shape::voxel_solid(pl.as_ref()), |s, p| crate::shape::clip_solid(&s, p)))
                })
                .sum();
            let mut cells: Vec<[i32; 3]> = def.voxels.iter().map(|v| v.pos).collect();
            cells.sort();
            cells.dedup();
            println!("AUDIT {n} {vol:.4} {}", cells.len());
            if std::env::var("AUDIT_SHOW").is_ok_and(|w| w == n) {
                for v in &def.voxels {
                    let pl = lib.shapes.plane(v.shape).map(|p| crate::shape::rotate(p, &v.rot));
                    let vv = crate::shape::solid_volume(
                        &v.bounds.iter().flatten().fold(crate::shape::voxel_solid(pl.as_ref()), |s, p| crate::shape::clip_solid(&s, p)),
                    );
                    println!("  {:?} shape {} vol {vv:.3} planes {}", v.pos, v.shape, v.bounds.as_ref().map_or(0, |b| b.len()));
                }
            }
        }
    }

    #[test]
    fn bin_wrapper() {
        let mut b = 5u32.to_le_bytes().to_vec();
        b.extend_from_slice(b"<a/>x\xffjunk");
        assert_eq!(bin_xml(&b), Some("<a/>x"));
        b[0] = 4;
        assert_eq!(bin_xml(&b), Some("<a/>"));
    }

    #[test]
    fn wheel_tyre_fills_its_reserved_cells_as_a_disc() {
        // An advanced 3x3 wheel: one solid hub cell, eight reserved around it in the x-z plane.
        let ring: String = (-1..=1).flat_map(|x| (-1..=1).map(move |z| (x, z))).filter(|p| *p != (0, 0)).map(|(x, z)| format!(r#"<voxel flags="2"><position x="{x}" z="{z}"/></voxel>"#)).collect();
        let d = parse_def(&format!(r#"<definition type="41" flags="8388609" constraint_axis="1" wheel_radius="0.35" wheel_width="0.25" mesh_data_name=""><voxels><voxel flags="1"/>{ring}</voxels></definition>"#));
        let w = d.wheel.clone().unwrap();
        assert_eq!((w.axle, w.radius, w.width), (1, 0.35, 0.25));
        assert!(w.centre.len() < 1e-9);
        let cells = tyre_voxels(&w, &d.voxels.iter().map(|v| v.pos).collect());
        let mut at: Vec<[i32; 3]> = cells.iter().map(|v| v.pos).collect();
        at.sort();
        // The eight cells around the hub, one layer thick; the hub cell itself is left alone.
        assert_eq!(at.len(), 8);
        assert!(at.iter().all(|p| p[1] == 0 && p[0].abs() <= 1 && p[2].abs() <= 1 && *p != [0, 0, 0]));
        // A double wheel: the same disc again, one layer further out the axle.
        let two: Vec<[i32; 3]> = w.tyres(Some((1.0, true))).iter().flat_map(|t| tyre_voxels(t, &Default::default())).map(|v| v.pos).collect();
        assert_eq!((two.len(), two.iter().filter(|p| p[1] == 1).count()), (18, 9));
        // A block is not a wheel, whatever its wheel_radius attribute says.
        assert!(parse_def(r#"<definition type="0" wheel_radius="0.25"><voxels><voxel flags="1"/><voxel flags="2"><position x="1"/></voxel></voxels></definition>"#).wheel.is_none());
    }
}

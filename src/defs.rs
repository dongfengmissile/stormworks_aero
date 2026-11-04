//! Component definitions: which voxels a part fills and how much.
//!
//! Read from the same files the game loads: `rom/data/definitions/<name>.xml`
//! and component-mod `.bin` files (u32 length + XML) from mods, the workshop and
//! the per-vehicle folders the game embeds them in.

use std::collections::HashMap;
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
}

pub struct Def {
    pub type_: i32,
    pub mass: f32,
    /// `rudder_type` of type-18 parts: 0 rudder/fin/control surface, 1 wing, 2 keel.
    pub rudder_type: i32,
    pub rudder_area: f32,
    /// Solid voxels only (sweep / reserved space of props, wheels, seats left out).
    pub voxels: Vec<DefVoxel>,
    /// `mesh_data_name`, empty when the part is not static (has mesh_0/1/2 pieces that move).
    pub mesh: String,
    /// `block_type` > 0: a block the game draws from its surfaces, a mesh only adds to it.
    pub block: bool,
    /// Has logic nodes (a control surface's "Rotation" input).
    pub logic: bool,
}

pub const TYPE_RUDDER: i32 = 18;

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
    let mut def = Def { type_: 0, mass: 0.0, rudder_type: 0, rudder_area: 0.0, voxels: Vec::new(), mesh: String::new(), block: false, logic: false };
    let mut in_voxels = false;
    // Inside a solid <voxel>: its <position> (omitted when 0,0,0) and rotation may follow.
    let mut open = false;
    for tag in xml::tags(text) {
        match (tag.name, tag.close) {
            ("definition", false) => {
                def.type_ = tag.int0("type");
                def.mass = tag.num("mass").unwrap_or(0.0);
                def.rudder_type = tag.int0("rudder_type");
                def.rudder_area = tag.num("rudder_surface_area").unwrap_or(0.0);
                // Moving pieces are drawn in their own frames; only trust fully static meshes.
                let animated = ["mesh_0_name", "mesh_1_name", "mesh_2_name"].iter().any(|k| tag.get(k).is_some_and(|v| !v.is_empty()));
                def.block = tag.int0("block_type") > 0;
                if !animated {
                    def.mesh = tag.get("mesh_data_name").unwrap_or("").to_string();
                }
            }
            ("voxels", close) => in_voxels = !close && !tag.empty,
            ("logic_node", false) => def.logic = true,
            ("voxel", false) if in_voxels => {
                let flags = tag.int0("flags");
                // bit 0 = solid, 4 = moving panel (control surface, door). 0/2 = reserved space
                // (prop discs, tyre sweep, seat occupant); an omitted attribute means 0.
                let solid = flags & 1 != 0 || flags & 4 != 0;
                if solid {
                    def.voxels.push(DefVoxel { pos: [0; 3], shape: tag.int0("physics_shape"), rot: crate::vehicle::IDENTITY, bounds: None });
                }
                open = solid && !tag.empty;
            }
            ("voxel", true) => open = false,
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
        def.voxels.push(DefVoxel { pos: [0; 3], shape: 0, rot: crate::vehicle::IDENTITY, bounds: None });
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

/// Bound each voxel by the mesh inside it: support planes of its points along the axes and
/// along the mesh's own face normals (a k-DOP around the mesh, so a slanted pane becomes a
/// slanted slab). FAR voxelizes the mesh; this is its convex per-cell version.
/// Voxels the mesh misses are dropped.
pub fn apply_mesh(def: &mut Def, tris: &[Tri], shapes: &crate::shape::ShapeTable) {
    if def.block {
        return block_cuts(def, tris, shapes);
    }
    let axes = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    let bounds: Vec<Option<Vec<Plane>>> = def
        .voxels
        .iter()
        .map(|v| {
            let c = v3(v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64);
            let mut pts: Vec<V3> = Vec::new();
            let mut dirs: Vec<V3> = axes.to_vec();
            for (t, n) in tris {
                let Some(poly) = in_cell(t, c) else { continue };
                let n = *n;
                if dirs.len() < MAX_DIRS && n.len() > 0.5 && !dirs.iter().any(|d| d.dot(n).abs() > 0.999) {
                    dirs.push(n);
                }
                pts.extend(poly);
            }
            if pts.is_empty() {
                return None;
            }
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
            Some(planes.collect())
        })
        .collect();
    if bounds.iter().all(Option::is_none) {
        return; // mesh not in the voxels' frame: keep the physics shapes
    }
    let mut bounds = bounds.into_iter();
    def.voxels.retain_mut(|v| {
        v.bounds = bounds.next().flatten();
        v.bounds.is_some()
    });
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
        Library { shapes: crate::shape::ShapeTable::from_rom(&rom), rom, bin_dirs, cache: HashMap::new() }
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
        if let Ok(text) = std::fs::read_to_string(self.rom.join(format!("{name}.xml"))) {
            let mut def = parse_def(&text);
            if let Some(tris) = (!def.mesh.is_empty()).then(|| rom_mesh(&def.mesh)).flatten().and_then(|b| mesh_triangles(&b)) {
                apply_mesh(&mut def, &tris, &self.shapes);
            }
            return Some(def);
        }
        self.bin_dirs.iter().find_map(|dir| {
            let bytes = std::fs::read(dir.join(format!("{name}.bin"))).ok()?;
            let mut def = parse_def(bin_xml(&bytes)?);
            if !def.mesh.is_empty() {
                let tris = bin_mesh(&bytes, &def.mesh).and_then(mesh_triangles).or_else(|| rom_mesh(&def.mesh).and_then(|b| mesh_triangles(&b)));
                if let Some(tris) = tris {
                    apply_mesh(&mut def, &tris, &self.shapes);
                }
            }
            Some(def)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let mut def = parse_def(r#"<definition mesh_data_name="m.mesh"><voxels><voxel flags="1"/><voxel flags="1"><position z="1"/></voxel></voxels></definition>"#);
        assert_eq!(def.mesh, "m.mesh");
        apply_mesh(&mut def, &tris, &Default::default());
        assert_eq!(def.voxels.len(), 1, "the voxel without mesh is dropped");
        let bnd = def.voxels[0].bounds.as_ref().unwrap();
        let s = bnd.iter().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p));
        assert!((crate::shape::solid_volume(&s) - MIN_THICK).abs() < 1e-9, "{}", crate::shape::solid_volume(&s));
        // A pane across the cell diagonal becomes a slanted slab, not the whole cell.
        let diag = [v3(-0.125, -0.125, -0.125), v3(-0.125, 0.125, -0.125), v3(0.125, 0.125, 0.125), v3(0.125, -0.125, 0.125)];
        let mut def = parse_def("<definition/>");
        let dn = (diag[1] - diag[0]).cross(diag[2] - diag[0]).norm();
        apply_mesh(&mut def, &[([diag[0], diag[1], diag[2]], dn), ([diag[0], diag[2], diag[3]], dn)], &Default::default());
        let s = def.voxels[0].bounds.as_ref().unwrap().iter().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p));
        let v = crate::shape::solid_volume(&s);
        assert!(v > 0.04 && v < 0.07, "{v}");

        // Block: a 6-long slope mesh over physics "2 cubes + steeper wedge" cuts each voxel to
        // the true slope; an axis-aligned trim mesh leaves the physics shape alone.
        let (top, bot) = (v3(0.0, 0.125, 0.125), v3(0.0, -0.125, -0.625)); // metres: 3 voxels long
        let n = v3(0.0, 3.0, -1.0).norm(); // outward: up and back (rise 1 over 3)
        let quad = |x: f64| v3(x, 0.0, 0.0);
        let slope = [([top + quad(-0.125), top + quad(0.125), bot + quad(0.125)], n), ([bot + quad(0.125), bot + quad(-0.125), top + quad(-0.125)], n)];
        let mut def = parse_def(r#"<definition block_type="7" mesh_data_name="m"><voxels><voxel flags="1"/><voxel flags="1"><position z="-1"/></voxel><voxel flags="1"><position z="-2"/></voxel></voxels></definition>"#);
        assert!(def.block);
        apply_mesh(&mut def, &slope, &Default::default());
        let vol: f64 = def.voxels.iter().map(|v| crate::shape::solid_volume(&v.bounds.iter().flatten().fold(crate::shape::voxel_solid(None), |s, p| crate::shape::clip_solid(&s, p)))).sum();
        assert!((vol - 1.5).abs() < 1e-6, "3-long wedge over 3 cubes: {vol}");
        let mut def = parse_def(r#"<definition block_type="4" mesh_data_name="cap"/>"#);
        apply_mesh(&mut def, &[([v3(-0.125, -0.125, 0.125), v3(0.125, -0.125, 0.125), v3(0.125, 0.0, 0.125)], v3(0.0, 0.0, 1.0))], &Default::default());
        assert!(def.voxels[0].bounds.is_none(), "axis-aligned trim keeps the block shape");
        let mut bin = b"x\0".to_vec();
        bin.extend(b"m.mesh\0");
        bin.extend((b.len() as u32).to_le_bytes());
        bin.extend(&b);
        assert_eq!(bin_mesh(&bin, "m.mesh"), Some(&b[..]));
    }

    #[test]
    fn bin_wrapper() {
        let mut b = 5u32.to_le_bytes().to_vec();
        b.extend_from_slice(b"<a/>x\xffjunk");
        assert_eq!(bin_xml(&b), Some("<a/>x"));
        b[0] = 4;
        assert_eq!(bin_xml(&b), Some("<a/>"));
    }
}

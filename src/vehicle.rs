//! The vehicle as the analysis sees it: a flat list of placed components.
//! Filled either from the live workbench (see `editor.rs`) or a saved vehicle XML.

use crate::xml;

#[derive(Clone, Debug, PartialEq)]
pub struct Comp {
    /// Definition name (`d` attribute; file name of the definition).
    pub def: String,
    /// Voxel position in the vehicle grid.
    pub pos: [i32; 3],
    /// Voxel-space transform, row-major, as stored by the game (`r` attribute / comp+0x30):
    /// a rotation, possibly stretched by XML editing.
    pub rot: [i32; 9],
    /// Body (sub-grid) index within this vehicle; 0 is the first body.
    pub body: u32,
    /// Mirror flags (1 x, 2 y, 4 z), already folded into `rot`; kept for moving meshes the game
    /// turns after mirroring (`model::live_sign`).
    pub mirror: i32,
    /// Propeller / rotor settings (`<o blade_count blade_pitch blade_length>`); None for other
    /// parts, and fields the vehicle leaves at the part's default.
    pub prop: Option<PropSet>,
    /// A microcontroller's width and length in voxels (`<microprocessor_definition width length>`):
    /// it takes width x 1 x length cells, along the part's +x and +z from `pos`. None for other parts.
    pub size: Option<[i32; 2]>,
    /// An advanced wheel's size setting (`<o wheel_size>`, 1 when unset) and whether it is a
    /// double wheel (`<o double_wheel>`): `defs::Wheel::tyres`.
    pub wheel_size: Option<(f64, bool)>,
}

/// The transform of a part saved without an `r` attribute. Not the identity, which the game
/// writes out: every multi-voxel stock part saved without `r` fits its vehicle only this way.
pub const DEFAULT_ROT: [i32; 9] = [0, 0, 1, -1, 0, 0, 0, -1, 0];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PropSet {
    pub blades: Option<u32>,
    /// Blade pitch −1..1 (setting plus the live collective input).
    pub pitch: Option<f64>,
    pub length: Option<f64>,
}

pub const IDENTITY: [i32; 9] = [1, 0, 0, 0, 1, 0, 0, 0, 1];

impl Comp {
    /// Grid cell of a definition-local voxel offset (offset as a row vector times `rot`;
    /// the same convention the infrared plugin verified in game).
    /// Part-frame point in metres -> aero body frame (x fwd, y right, z down), metres.
    pub fn part_to_aero(&self, p: crate::math::V3) -> crate::math::V3 {
        let r = self.rot.map(|v| v as f64);
        let (x, y, z) = (p.x * 4.0, p.y * 4.0, p.z * 4.0); // voxel units
        let g = [
            self.pos[0] as f64 + r[0] * x + r[3] * y + r[6] * z,
            self.pos[1] as f64 + r[1] * x + r[4] * y + r[7] * z,
            self.pos[2] as f64 + r[2] * x + r[5] * y + r[8] * z,
        ];
        crate::math::v3(g[2] * 0.25, g[0] * 0.25, -g[1] * 0.25)
    }

    pub fn cell(&self, o: [i32; 3]) -> [i32; 3] {
        let r = &self.rot;
        [
            self.pos[0] + r[0] * o[0] + r[3] * o[1] + r[6] * o[2],
            self.pos[1] + r[1] * o[0] + r[4] * o[1] + r[7] * o[2],
            self.pos[2] + r[2] * o[0] + r[5] * o[1] + r[8] * o[2],
        ]
    }
}

/// A component transform: a rotation, or a stretched one from XML editing
/// (e.g. `0,0,-23,0,1,0,3,0,0`), or a flattened one that hides the part
/// (`0,0,0,0,0,0,0,0,1`: no volume, mass kept). Garbage reads are rejected: huge.
pub fn transform_ok(r: &[i32; 9]) -> bool {
    r.iter().all(|v| v.abs() <= 1024)
}

/// `(p ⊙ mirror)·R`: mirror flags (bit 0 x, 1 y, 2 z) negate rows of the transform.
pub fn flipped(mut r: [i32; 9], flags: i32) -> [i32; 9] {
    for a in 0..3 {
        if flags >> a & 1 != 0 {
            r[a * 3..a * 3 + 3].iter_mut().for_each(|v| *v = -*v);
        }
    }
    r
}

/// A rotation is a signed permutation; anything else is garbage.
pub fn valid_rotation(r: &[i32; 9]) -> bool {
    r.iter().all(|v| (-1..=1).contains(v))
        && (0..3).all(|i| (0..3).filter(|&j| r[i * 3 + j] != 0).count() == 1)
        && (0..3).all(|j| (0..3).filter(|&i| r[i * 3 + j] != 0).count() == 1)
}

/// Components of every body of a saved vehicle XML (microcontroller internals skipped).
pub fn parse_vehicle_xml(text: &str) -> Vec<Comp> {
    parse_vehicle_xml_marked(text).0
}

/// Components and the plugin's `aero` attribute of each (`""` when absent).
pub fn parse_vehicle_xml_marked(text: &str) -> (Vec<Comp>, Vec<String>) {
    let mut out = Vec::new();
    let mut aero = Vec::new();
    let mut stack: Vec<&str> = Vec::new();
    let mut body: i64 = -1;
    let mut cur: Option<Comp> = None;
    // `t` on <c>: mirror flags of the part (1 = x, 2 = y, 4 = z), applied before `r`.
    let mut flip = 0;
    let ends = |stack: &[&str], tail: &[&str]| stack.ends_with(tail);
    for tag in xml::tags(text) {
        if tag.close {
            if tag.name == "c" && ends(&stack, &["body", "components", "c"]) {
                out.extend(cur.take().map(|c| Comp { rot: flipped(c.rot, flip), mirror: flip & 7, ..c }));
            }
            if let Some(i) = stack.iter().rposition(|n| *n == tag.name) {
                stack.truncate(i);
            }
            continue;
        }
        match tag.name {
            "body" if ends(&stack, &["bodies"]) => body += 1,
            "c" if ends(&stack, &["body", "components"]) => {
                cur = Some(Comp {
                    def: tag.get("d").unwrap_or("01_block").to_string(),
                    pos: [0; 3],
                    rot: DEFAULT_ROT,
                    body: body.max(0) as u32,
                    mirror: 0,
                    prop: None, size: None, wheel_size: None,
                });
                flip = tag.int0("t");
                aero.push(tag.get("aero").unwrap_or("").to_string());
                if tag.empty {
                    out.extend(cur.take().map(|c| Comp { rot: flipped(c.rot, flip), mirror: flip & 7, ..c }));
                }
            }
            "o" if ends(&stack, &["components", "c"]) => {
                if let Some(c) = cur.as_mut() {
                    let p = PropSet { blades: tag.num("blade_count"), pitch: tag.num("blade_pitch"), length: tag.num("blade_length") };
                    c.prop = (p != PropSet::default()).then_some(p);
                    let double = tag.get("double_wheel") == Some("true");
                    c.wheel_size = (tag.get("wheel_size").is_some() || double).then(|| (tag.num("wheel_size").unwrap_or(1.0), double));
                }
                if let (Some(c), Some(r)) = (cur.as_mut(), tag.get("r")) {
                    let v: Vec<i32> = r.split(',').filter_map(|s| s.trim().parse().ok()).collect();
                    if let Ok(rot) = <[i32; 9]>::try_from(v) {
                        if transform_ok(&rot) {
                            c.rot = rot;
                        }
                    }
                }
            }
            "microprocessor_definition" if ends(&stack, &["body", "components", "c", "o"]) => {
                if let Some(c) = cur.as_mut() {
                    c.size = Some([tag.num("width").unwrap_or(1), tag.num("length").unwrap_or(1)]);
                }
            }
            "vp" if ends(&stack, &["c", "o"]) => {
                if let Some(c) = cur.as_mut() {
                    c.pos = [tag.int0("x"), tag.int0("y"), tag.int0("z")];
                }
            }
            _ => {}
        }
        if !tag.empty {
            stack.push(tag.name);
        }
    }
    (out, aero)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_bodies_and_skips_mc_internals() {
        let s = r#"<vehicle><bodies><body unique_id="1"><components>
            <c><o r="0,0,-1,-1,0,0,0,1,0"><vp x="-2" z="3"/></o></c>
            <c d="02_wedge" t="5"><o r="1,0,0,0,1,0,0,0,1"><vp x="1"/></o></c>
            <c d="microprocessor"><o><vp y="1"/><microprocessor_definition width="3" length="2"><group><components><c><object/></c></components></group></microprocessor_definition></o></c>
            </components></body><body unique_id="2"><components><c d="02_wedge"><o/></c></components></body></bodies></vehicle>"#;
        let mut v = parse_vehicle_xml(s);
        let w = v.remove(1);
        assert_eq!(w.rot, [-1, 0, 0, 0, 1, 0, 0, 0, -1], "t=5 mirrors x and z");
        assert_eq!(v.len(), 3);
        assert_eq!(v[0].def, "01_block");
        assert_eq!(v[0].pos, [-2, 0, 3]);
        assert_eq!(v[0].rot, [0, 0, -1, -1, 0, 0, 0, 1, 0]);
        assert_eq!(v[1].def, "microprocessor");
        assert_eq!(v[1].pos, [0, 1, 0]);
        assert_eq!((v[1].size, v[0].size), (Some([3, 2]), None));
        assert_eq!(v[1].rot, DEFAULT_ROT, "no r attribute");
        assert_eq!((v[2].def.as_str(), v[2].body, v[2].pos), ("02_wedge", 1, [0, 0, 0]));
        // row vector times rot: x offset 1 -> (0,0,-1)
        assert_eq!(v[0].cell([1, 0, 0]), [-2, 0, 2]);
    }
}

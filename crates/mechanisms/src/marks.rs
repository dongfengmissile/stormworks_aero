//! Joint settings stored on the joint parts themselves: the `mech` attribute on each `<c>` of the
//! vehicle XML (`stormworks_core::markhooks`). Older files kept them in the aero plugin's `aero`
//! attribute; those are read when a part has no `mech` attribute.

use std::collections::HashMap;

pub use stormworks_core::marks::GEN;
use stormworks_core::marks as raw;
use stormworks_core::vehicle::Comp;

/// One joint part's settings.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mark {
    /// A joint's workbench pose, degrees (`JOINT_ANGLE_RANGE`): the workbench shows its sub-grid
    /// turned this far.
    pub joint_angle: Option<f32>,
    /// A sliding joint's workbench pose, metres along its axis (`JOINT_OFFSET_RANGE`).
    pub joint_offset: Option<f32>,
    /// The sequences this joint is in, by name, with its moves in each: start (s), duration (s),
    /// target (degrees, or metres for a sliding joint), from the workbench pose.
    pub seq: Vec<(String, Vec<[f32; 3]>)>,
    /// The part's sub-grid stays put in the Mechanisms mode; the joints move the rest about it.
    pub fixed: bool,
    /// Tokens this version does not know (a newer plugin's), written back unchanged.
    pub other: Vec<String>,
}

/// Workbench joint angle range, degrees.
pub const JOINT_ANGLE_RANGE: (f32, f32) = (-180.0, 180.0);
/// Workbench sliding joint offset range, metres (long linear tracks).
pub const JOINT_OFFSET_RANGE: (f32, f32) = (-100.0, 100.0);
/// The largest sequence move target: velocity pivots and turret rings turn on (100 turns).
const SEQ_TARGET_MAX: f32 = 36000.0;

impl Mark {
    pub fn is_empty(&self) -> bool {
        *self == Mark::default()
    }

    /// Attribute value: space-separated `pa:<degrees>`, `po:<metres>`,
    /// `sq:<name>@<start>/<duration>/<target>;...` (one per sequence, `_` for spaces), `fx` (fixed).
    pub fn encode(&self) -> String {
        let mut t: Vec<String> = Vec::new();
        t.extend(self.joint_angle.map(|a| format!("pa:{a:.1}")));
        t.extend(self.joint_offset.map(|o| format!("po:{o:.3}")));
        for (name, moves) in &self.seq {
            let moves: Vec<String> = moves.iter().map(|[s, d, v]| format!("{s:.2}/{d:.2}/{v:.2}")).collect();
            t.push(format!("sq:{}@{}", seq_name(name).replace(' ', "_"), moves.join(";")));
        }
        if self.fixed {
            t.push("fx".into());
        }
        t.extend(self.other.iter().cloned());
        t.join(" ")
    }

    /// Unknown tokens are kept in `other`.
    pub fn decode(s: &str) -> Mark {
        let mut m = Mark::default();
        for tok in s.split_whitespace() {
            if tok == "fx" {
                m.fixed = true;
                continue;
            }
            match tok.split_once(':') {
                Some(("pa", v)) => m.joint_angle = v.parse::<f32>().ok().filter(|a| a.is_finite()).map(|a| a.clamp(JOINT_ANGLE_RANGE.0, JOINT_ANGLE_RANGE.1)),
                Some(("po", v)) => m.joint_offset = v.parse::<f32>().ok().filter(|o| o.is_finite()).map(|o| o.clamp(JOINT_OFFSET_RANGE.0, JOINT_OFFSET_RANGE.1)),
                Some(("sq", v)) => {
                    let (name, moves) = v.split_once('@').map_or(("Sequence 1".to_string(), v), |(n, mv)| (seq_name(&n.replace('_', " ")), mv));
                    let moves = (moves.split(';'))
                        .filter_map(|mv| {
                            let n: Vec<f32> = mv.split('/').filter_map(|x| x.parse().ok()).filter(|x: &f32| x.is_finite()).collect();
                            (n.len() == 3).then(|| [n[0].clamp(0.0, 600.0), n[1].clamp(0.0, 600.0), n[2].clamp(-SEQ_TARGET_MAX, SEQ_TARGET_MAX)])
                        })
                        .collect();
                    if !name.is_empty() && !m.seq.iter().any(|(n, _)| *n == name) {
                        m.seq.push((name, moves));
                    }
                }
                _ => m.other.push(tok.to_string()),
            }
        }
        m
    }
}

/// An older file's `aero` attribute, cut to the joint tokens (the aero plugin keeps its own).
pub fn from_aero(s: &str) -> String {
    Mark { other: Vec::new(), ..Mark::decode(s) }.encode()
}

/// A sequence name as stored: letters, digits, spaces and dashes.
pub fn seq_name(s: &str) -> String {
    s.chars().filter(|c| c.is_alphanumeric() || " -".contains(*c)).collect::<String>().trim().to_string()
}

/// A vehicle's joint settings by the joint's parent half position.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Designations {
    /// Workbench joint angle, degrees (`pose`).
    pub joint_angles: HashMap<[i32; 3], f64>,
    /// Workbench sliding joint offset, metres (`pose`).
    pub joint_offsets: HashMap<[i32; 3], f64>,
    /// Sequences and their moves (`Mark::seq`).
    pub joint_seq: HashMap<[i32; 3], Vec<(String, Vec<[f64; 3]>)>>,
    /// Parts whose sub-grid stays put (`Mark::fixed`).
    pub fixed: std::collections::HashSet<[i32; 3]>,
}

pub fn to_designations(comps: &[Comp], marks: &[Mark]) -> Designations {
    let mut d = Designations::default();
    for (c, m) in comps.iter().zip(marks) {
        if let Some(a) = m.joint_angle {
            d.joint_angles.insert(c.pos, a as f64);
        }
        if let Some(o) = m.joint_offset {
            d.joint_offsets.insert(c.pos, o as f64);
        }
        if m.fixed {
            d.fixed.insert(c.pos);
        }
        if !m.seq.is_empty() {
            d.joint_seq.insert(c.pos, m.seq.iter().map(|(n, mv)| (n.clone(), mv.iter().map(|s| s.map(|x| x as f64)).collect())).collect());
        }
    }
    d
}

pub fn get(comp: usize) -> Option<Mark> {
    raw::get(comp).map(|s| Mark::decode(&s))
}

/// Set (or with an empty/None mark, clear) a component's marks. `game`: the game moved them.
pub fn set(comp: usize, mark: Option<Mark>, game: bool) {
    raw::set(comp, mark.map(|m| m.encode()), game);
}

/// Joint settings of a live vehicle from its component pointers.
pub fn designations(comps: &[Comp], ptrs: &[usize]) -> Option<Designations> {
    let marks: Vec<Mark> = raw::read(ptrs).iter().map(|s| Mark::decode(s)).collect();
    Some(to_designations(comps, &marks))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_tokens_round_trip_and_clamp() {
        let m = Mark { joint_angle: Some(-32.5), ..Default::default() };
        assert_eq!(m.encode(), "pa:-32.5");
        assert_eq!(Mark::decode(&m.encode()), m);
        assert_eq!(Mark::decode("pa:400").joint_angle, Some(JOINT_ANGLE_RANGE.1));
        let m = Mark { seq: vec![("Landing gear".into(), vec![[0.0, 1.5, 45.0], [3.0, 1.0, -10.0]]), ("Canopy".into(), vec![])], ..Default::default() };
        assert_eq!(m.encode(), "sq:Landing_gear@0.00/1.50/45.00;3.00/1.00/-10.00 sq:Canopy@");
        assert_eq!(Mark::decode(&m.encode()), m);
        assert_eq!(Mark::decode("sq:a@1/2;x/1/2;0/1/9").seq, vec![("a".into(), vec![[0.0, 1.0, 9.0]])], "bad moves are skipped");
        assert_eq!(Mark::decode(&Mark { seq: vec![("Spin".into(), vec![[0.0, 2.0, 720.0]])], ..Default::default() }.encode()).seq[0].1, vec![[0.0, 2.0, 720.0]], "turns past 180 keep");
        assert_eq!(Mark::decode("sq:0/1/9").seq, vec![("Sequence 1".into(), vec![[0.0, 1.0, 9.0]])], "unnamed: the first sequence");
        let m = Mark { fixed: true, joint_angle: Some(5.0), ..Default::default() };
        assert_eq!(m.encode(), "pa:5.0 fx");
        assert_eq!(Mark::decode(&m.encode()), m);
        let m = Mark { joint_offset: Some(0.375), ..Default::default() };
        assert_eq!(m.encode(), "po:0.375");
        assert_eq!(Mark::decode(&m.encode()), m);
    }

    #[test]
    fn reads_joint_tokens_from_an_older_aero_attribute() {
        assert_eq!(from_aero("w r:pitch pa:30.0 fm:on sq:Gear@0.00/1.00/90.00"), "pa:30.0 sq:Gear@0.00/1.00/90.00");
        assert_eq!(Mark::decode("pa:5.0 zz:1").encode(), "pa:5.0 zz:1", "unknown tokens stay");
    }
}

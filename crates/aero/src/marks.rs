//! Designations stored on the components themselves: the `aero` attribute on each `<c>` of the
//! vehicle XML, and in memory a table keyed by component pointer. Marks follow their part through
//! save/load, copy/paste and undo, wherever it ends up.

pub use stormworks_core::marks::GEN;
use stormworks_core::marks as raw;

use crate::model::{Designations, Switches};
use crate::vehicle::Comp;
use crate::wing::{aerofoil_index, Role, AEROFOILS};

/// One component's marks. Vehicle-wide defaults ride on the vehicle's first component.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mark {
    pub wing: bool,
    /// Separate wing number (`s` is wing 1, `s:<n>` wing n).
    pub split: Option<u32>,
    pub role: Option<Role>,
    pub foil: Option<usize>,
    pub default_foil: Option<usize>,
    pub default_fin: Option<usize>,
    pub sw: Switches,
    /// Propeller/rotor blade width, x the part's own (`BLADE_WIDTH_RANGE`).
    pub blade_width: Option<f32>,
    /// Blade shape chosen for a propeller/rotor: true propeller, false rotor (None: auto).
    pub airscrew: Option<bool>,
    /// Tokens this version does not know (a newer plugin's, or the mechanisms plugin's joint
    /// settings in older files), written back unchanged.
    pub other: Vec<String>,
}

/// Blade width multiplier range.
pub const BLADE_WIDTH_RANGE: (f32, f32) = (0.25, 2.0);

fn foil_id(i: usize) -> &'static str {
    AEROFOILS[i.min(AEROFOILS.len() - 1)].id
}

impl Mark {
    pub fn is_empty(&self) -> bool {
        *self == Mark::default()
    }

    /// Attribute value: space-separated `w`, `s` / `s:<n>`, `r:<role>`, `f:<foil>`, `fd:<foil>`, `ff:<foil>`,
    /// `fm:on|off`, `vrs:on|off`, `pm:on|off`, `ge:on|off`, `js:on|off`, `bw:<width>`, `sh:prop|rotor`.
    pub fn encode(&self) -> String {
        let mut t: Vec<String> = Vec::new();
        if self.wing {
            t.push("w".into());
        }
        match self.split {
            Some(1) => t.push("s".into()),
            Some(n) => t.push(format!("s:{n}")),
            None => {}
        }
        t.extend(self.role.map(|r| format!("r:{}", r.name())));
        t.extend(self.foil.map(|f| format!("f:{}", foil_id(f))));
        t.extend(self.default_foil.map(|f| format!("fd:{}", foil_id(f))));
        t.extend(self.default_fin.map(|f| format!("ff:{}", foil_id(f))));
        let mut sw = self.sw;
        t.extend(sw.tokens().into_iter().filter_map(|(k, v)| v.map(|b| format!("{k}:{}", if b { "on" } else { "off" }))));
        t.extend(self.blade_width.map(|w| format!("bw:{w:.2}")));
        t.extend(self.airscrew.map(|a| format!("sh:{}", if a { "prop" } else { "rotor" })));
        t.extend(self.other.iter().cloned());
        t.join(" ")
    }

    /// Unknown tokens (a newer plugin's) are kept in `other`.
    pub fn decode(s: &str) -> Mark {
        let mut m = Mark::default();
        for tok in s.split_whitespace() {
            match tok.split_once(':') {
                None if tok == "w" => m.wing = true,
                None if tok == "s" => m.split = Some(1),
                Some(("s", v)) => m.split = v.parse().ok().filter(|n| *n > 0),
                Some(("r", v)) => m.role = Role::parse(v).filter(|r| *r != Role::None),
                Some(("f", v)) => m.foil = aerofoil_index(v),
                Some(("fd", v)) => m.default_foil = aerofoil_index(v),
                Some(("ff", v)) => m.default_fin = aerofoil_index(v),
                Some(("sh", "prop")) => m.airscrew = Some(true),
                Some(("sh", "rotor")) => m.airscrew = Some(false),
                Some(("bw", v)) => m.blade_width = v.parse::<f32>().ok().filter(|w| w.is_finite()).map(|w| w.clamp(BLADE_WIDTH_RANGE.0, BLADE_WIDTH_RANGE.1)),
                Some((k, v)) if m.sw.tokens().iter().any(|(t, _)| *t == k) => {
                    if let Some((_, slot)) = m.sw.tokens().into_iter().find(|(t, _)| *t == k) {
                        *slot = on_off(v);
                    }
                }
                _ => m.other.push(tok.to_string()),
            }
        }
        m
    }
}

fn on_off(v: &str) -> Option<bool> {
    match v {
        "on" => Some(true),
        "off" => Some(false),
        _ => None,
    }
}

/// Each component's marks from position-keyed designations (each body's first component carries the defaults).
pub fn from_designations(des: &Designations, comps: &[Comp]) -> Vec<Mark> {
    // Each body's first component: a body held only by a hardpoint (a missile) spawns as its own
    // vehicle and must bring the settings with it.
    let mut seen = std::collections::HashSet::new();
    comps
        .iter()
        .map(|c| (seen.insert(c.body), c))
        .map(|(first, c)| Mark {
            wing: des.wings.contains(&c.pos),
            split: des.splits.get(&c.pos).copied(),
            role: des.roles.get(&c.pos).copied().filter(|r| *r != Role::None),
            foil: des.foils.get(&c.pos).copied(),
            default_foil: (first && des.default_foil != 0).then_some(des.default_foil),
            default_fin: if first { des.default_fin } else { None },
            sw: if first { des.sw } else { Switches::default() },
            blade_width: des.widths.get(&c.pos).map(|&w| w as f32),
            airscrew: des.shapes.get(&c.pos).copied(),
            other: Vec::new(),
        })
        .collect()
}

/// Designations from the components' marks; None if no component carries any (an unmarked vehicle).
/// Defaults: the first component in order that has one (pasted parts come after the vehicle's own).
pub fn to_designations(comps: &[Comp], marks: &[Mark]) -> Option<Designations> {
    if marks.iter().all(Mark::is_empty) {
        return None;
    }
    let mut d = Designations::default();
    for (c, m) in comps.iter().zip(marks) {
        if m.wing {
            d.wings.insert(c.pos);
        }
        if let Some(n) = m.split {
            d.splits.insert(c.pos, n);
        }
        if let Some(r) = m.role {
            d.roles.insert(c.pos, r);
        }
        if let Some(f) = m.foil {
            d.foils.insert(c.pos, f);
        }
        if let Some(w) = m.blade_width {
            d.widths.insert(c.pos, w as f64);
        }
        if let Some(a) = m.airscrew {
            d.shapes.insert(c.pos, a);
        }
    }
    d.default_foil = marks.iter().find_map(|m| m.default_foil).unwrap_or(0);
    d.default_fin = marks.iter().find_map(|m| m.default_fin);
    d.sw = marks.iter().fold(Switches::default(), |a, m| a.or(m.sw));
    Some(d)
}

pub fn get(comp: usize) -> Option<Mark> {
    raw::get(comp).map(|s| Mark::decode(&s))
}

/// Set (or with an empty/None mark, clear) a component's marks. `game`: the game moved them.
pub fn set(comp: usize, mark: Option<Mark>, game: bool) {
    raw::set(comp, mark.map(|m| m.encode()), game);
}

pub fn read(ptrs: &[usize]) -> Vec<Mark> {
    raw::read(ptrs).iter().map(|s| Mark::decode(s)).collect()
}

/// Copy the vehicle settings (the first of each found, in part order) onto every body's first part.
/// `parts`: (component, body) in part order. A body held only by a hardpoint (a missile) spawns
/// as its own vehicle and has to bring them along.
pub fn spread_settings(parts: &[(usize, u32)]) {
    let mut t = raw::table();
    let mut d = Mark::default();
    for m in parts.iter().filter_map(|(p, _)| t.get(p)).map(|s| Mark::decode(s)) {
        d.default_foil = d.default_foil.or(m.default_foil);
        d.default_fin = d.default_fin.or(m.default_fin);
        d.sw = d.sw.or(m.sw);
    }
    if d.is_empty() {
        return;
    }
    let mut seen = std::collections::HashSet::new();
    for (p, _) in parts.iter().filter(|(_, b)| seen.insert(*b)) {
        let mut m = t.get(p).map_or_else(Mark::default, |s| Mark::decode(s));
        (m.default_foil, m.default_fin, m.sw) = (d.default_foil, d.default_fin, d.sw);
        t.insert(*p, m.encode());
    }
}

/// Designations of a live vehicle from its component pointers (None: no marks).
pub fn designations(comps: &[Comp], ptrs: &[usize]) -> Option<Designations> {
    to_designations(comps, &read(ptrs))
}

/// Store designations on a live vehicle's components.
pub fn store(des: &Designations, comps: &[Comp], ptrs: &[usize]) {
    let marks = from_designations(des, comps);
    let mut t = raw::table();
    for (p, mut m) in ptrs.iter().zip(marks) {
        // Tokens of a newer plugin version stay.
        m.other = t.get(p).map(|s| Mark::decode(s).other).unwrap_or_default();
        if m.is_empty() {
            t.remove(p);
        } else {
            t.insert(*p, m.encode());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn comp(pos: [i32; 3]) -> Comp {
        Comp { def: "x".into(), pos, rot: crate::vehicle::IDENTITY, body: 0, mirror: 0, prop: None, size: None, wheel_size: None, power: None }
    }

    #[test]
    fn blade_width_round_trips_and_clamps() {
        let m = Mark { blade_width: Some(1.5), airscrew: Some(false), ..Default::default() };
        assert_eq!(Mark::decode(&m.encode()), m);
        assert_eq!(Mark::decode("bw:9").blade_width, Some(BLADE_WIDTH_RANGE.1));
        assert_eq!(Mark::decode("bw:nan").blade_width, None);
        let comps = [comp([1, 2, 3])];
        let des = to_designations(&comps, &[m.clone()]).unwrap();
        assert_eq!(des.widths.get(&[1, 2, 3]), Some(&1.5));
        assert_eq!(from_designations(&des, &comps)[0], m);
    }

    #[test]
    fn spread_settings_reaches_every_body() {
        // Distinct fake pointers; body 1 (the missile) has a wing mark but no switch.
        let base = 0x7000_0000_1000usize;
        set(base, Some(Mark { sw: Switches { fly: Some(true), ..Default::default() }, ..Default::default() }), false);
        set(base + 16, Some(Mark { wing: true, ..Default::default() }), false);
        spread_settings(&[(base, 0), (base + 8, 0), (base + 16, 1)]);
        assert_eq!(get(base + 16), Some(Mark { wing: true, sw: Switches { fly: Some(true), ..Default::default() }, ..Default::default() }));
        assert_eq!(get(base + 8), None);
        [base, base + 16].into_iter().for_each(|p| set(p, None, false));
    }

    #[test]
    fn every_body_carries_the_vehicle_settings() {
        // A missile on a hardpoint is its own body and spawns as its own vehicle.
        let mut comps: Vec<Comp> = (0..4).map(|i| comp([i, 0, 0])).collect();
        comps[2].body = 1;
        comps[3].body = 1;
        let d = Designations { sw: Switches { fly: Some(true), ..Default::default() }, ..Default::default() };
        let marks = from_designations(&d, &comps);
        assert_eq!(marks.iter().map(|m| m.sw.fly).collect::<Vec<_>>(), [Some(true), None, Some(true), None]);
        assert_eq!(to_designations(&comps[2..], &marks[2..]).unwrap().sw.fly, Some(true));
    }

    #[test]
    fn round_trips() {
        let comps: Vec<Comp> = (0..4).map(|i| comp([i, 0, 0])).collect();
        let mut d = Designations::default();
        d.wings.extend([[1, 0, 0], [2, 0, 0]]);
        d.splits.insert([2, 0, 0], 1);
        d.splits.insert([1, 0, 0], 3);
        d.roles.insert([3, 0, 0], Role::Roll);
        d.foils.insert([1, 0, 0], 2);
        d.default_foil = 4;
        d.default_fin = Some(9);
        d.sw = Switches { fly: Some(false), vrs: Some(true), props: Some(false), ground: Some(false), jet: Some(true) };
        let marks = from_designations(&d, &comps);
        let text: Vec<String> = marks.iter().map(Mark::encode).collect();
        assert_eq!(text[0], "fd:naca0012 ff:fin_hex4 fm:off vrs:on pm:off ge:off js:on");
        assert_eq!(text[2], "w s");
        let back: Vec<Mark> = text.iter().map(|s| Mark::decode(s)).collect();
        assert_eq!(back, marks);
        assert_eq!(to_designations(&comps, &back), Some(d));
        assert_eq!(to_designations(&comps, &[Mark::default(), Mark::default()]), None);
        // A newer plugin's tokens (or the mechanisms plugin's, in older files) survive a re-save.
        let m = Mark::decode("w zz:1 r:none q");
        assert_eq!(m, Mark { wing: true, other: vec!["zz:1".into(), "q".into()], ..Default::default() });
        assert_eq!(m.encode(), "w zz:1 q");
    }
}

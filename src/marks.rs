//! Designations stored on the components themselves: the `aero` attribute on each `<c>` of the
//! vehicle XML, and in memory a table keyed by component pointer. Marks follow their part through
//! save/load, copy/paste and undo, wherever it ends up.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{LazyLock, Mutex};

use crate::model::Designations;
use crate::vehicle::Comp;
use crate::wing::{aerofoil_index, Role, AEROFOILS};

/// One component's marks. Vehicle-wide defaults ride on the vehicle's first component.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mark {
    pub wing: bool,
    pub split: bool,
    pub role: Option<Role>,
    pub foil: Option<usize>,
    pub default_foil: Option<usize>,
    pub default_fin: Option<usize>,
    pub fly: Option<bool>,
    pub vrs: Option<bool>,
    /// Propeller/rotor blade width, x the part's own (`BLADE_WIDTH_RANGE`).
    pub blade_width: Option<f32>,
    /// Blade shape chosen for a propeller/rotor: true propeller, false rotor (None: auto).
    pub airscrew: Option<bool>,
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

    /// Attribute value: space-separated `w`, `s`, `r:<role>`, `f:<foil>`, `fd:<foil>`, `ff:<foil>`,
    /// `fm:on|off`, `vrs:on|off`, `bw:<width>`, `sh:prop|rotor`.
    pub fn encode(&self) -> String {
        let mut t: Vec<String> = Vec::new();
        if self.wing {
            t.push("w".into());
        }
        if self.split {
            t.push("s".into());
        }
        t.extend(self.role.map(|r| format!("r:{}", r.name())));
        t.extend(self.foil.map(|f| format!("f:{}", foil_id(f))));
        t.extend(self.default_foil.map(|f| format!("fd:{}", foil_id(f))));
        t.extend(self.default_fin.map(|f| format!("ff:{}", foil_id(f))));
        let on_off = |b: bool| if b { "on" } else { "off" };
        t.extend(self.fly.map(|b| format!("fm:{}", on_off(b))));
        t.extend(self.vrs.map(|b| format!("vrs:{}", on_off(b))));
        t.extend(self.blade_width.map(|w| format!("bw:{w:.2}")));
        t.extend(self.airscrew.map(|a| format!("sh:{}", if a { "prop" } else { "rotor" })));
        t.join(" ")
    }

    /// Unknown tokens (a newer plugin's) are skipped.
    pub fn decode(s: &str) -> Mark {
        let mut m = Mark::default();
        for tok in s.split_whitespace() {
            match tok.split_once(':') {
                None if tok == "w" => m.wing = true,
                None if tok == "s" => m.split = true,
                Some(("r", v)) => m.role = Role::parse(v).filter(|r| *r != Role::None),
                Some(("f", v)) => m.foil = aerofoil_index(v),
                Some(("fd", v)) => m.default_foil = aerofoil_index(v),
                Some(("ff", v)) => m.default_fin = aerofoil_index(v),
                Some(("fm", v)) => m.fly = on_off(v),
                Some(("vrs", v)) => m.vrs = on_off(v),
                Some(("sh", "prop")) => m.airscrew = Some(true),
                Some(("sh", "rotor")) => m.airscrew = Some(false),
                Some(("bw", v)) => m.blade_width = v.parse::<f32>().ok().filter(|w| w.is_finite()).map(|w| w.clamp(BLADE_WIDTH_RANGE.0, BLADE_WIDTH_RANGE.1)),
                _ => {}
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

/// Each component's marks from position-keyed designations (`comps[0]` carries the defaults).
pub fn from_designations(des: &Designations, comps: &[Comp]) -> Vec<Mark> {
    comps
        .iter()
        .enumerate()
        .map(|(i, c)| Mark {
            wing: des.wings.contains(&c.pos),
            split: des.splits.contains(&c.pos),
            role: des.roles.get(&c.pos).copied().filter(|r| *r != Role::None),
            foil: des.foils.get(&c.pos).copied(),
            default_foil: (i == 0 && des.default_foil != 0).then_some(des.default_foil),
            default_fin: if i == 0 { des.default_fin } else { None },
            fly: if i == 0 { des.fly } else { None },
            vrs: if i == 0 { des.vrs } else { None },
            blade_width: des.widths.get(&c.pos).map(|&w| w as f32),
            airscrew: des.shapes.get(&c.pos).copied(),
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
        if m.split {
            d.splits.insert(c.pos);
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
    d.fly = marks.iter().find_map(|m| m.fly);
    d.vrs = marks.iter().find_map(|m| m.vrs);
    Some(d)
}

static TABLE: LazyLock<Mutex<HashMap<usize, Mark>>> = LazyLock::new(Default::default);
/// Bumped when the game moves marks (load, clone, undo): the workbench snapshot re-reads them.
pub static GEN: AtomicU64 = AtomicU64::new(0);

fn table() -> std::sync::MutexGuard<'static, HashMap<usize, Mark>> {
    TABLE.lock().unwrap_or_else(|p| p.into_inner())
}

pub fn get(comp: usize) -> Option<Mark> {
    table().get(&comp).cloned()
}

/// Set (or with an empty/None mark, clear) a component's marks. `game`: the game moved them.
pub fn set(comp: usize, mark: Option<Mark>, game: bool) {
    let mut t = table();
    let changed = match mark.filter(|m| !m.is_empty()) {
        Some(m) => t.insert(comp, m.clone()).as_ref() != Some(&m),
        None => t.remove(&comp).is_some(),
    };
    if changed && game {
        GEN.fetch_add(1, Ordering::Relaxed);
    }
}

/// Copy `src`'s marks onto `dst` (clone, undo).
pub fn copy(dst: usize, src: usize) {
    let m = get(src);
    set(dst, m, true);
}

pub fn read(ptrs: &[usize]) -> Vec<Mark> {
    let t = table();
    ptrs.iter().map(|p| t.get(p).cloned().unwrap_or_default()).collect()
}

/// Designations of a live vehicle from its component pointers (None: no marks).
pub fn designations(comps: &[Comp], ptrs: &[usize]) -> Option<Designations> {
    to_designations(comps, &read(ptrs))
}

/// Store designations on a live vehicle's components.
pub fn store(des: &Designations, comps: &[Comp], ptrs: &[usize]) {
    let marks = from_designations(des, comps);
    let mut t = table();
    for (p, m) in ptrs.iter().zip(marks) {
        if m.is_empty() {
            t.remove(p);
        } else {
            t.insert(*p, m);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn comp(pos: [i32; 3]) -> Comp {
        Comp { def: "x".into(), pos, rot: crate::vehicle::IDENTITY, body: 0, mirror: 0, prop: None }
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
    fn round_trips() {
        let comps: Vec<Comp> = (0..4).map(|i| comp([i, 0, 0])).collect();
        let mut d = Designations::default();
        d.wings.extend([[1, 0, 0], [2, 0, 0]]);
        d.splits.insert([2, 0, 0]);
        d.roles.insert([3, 0, 0], Role::Roll);
        d.foils.insert([1, 0, 0], 2);
        d.default_foil = 4;
        d.default_fin = Some(9);
        d.fly = Some(false);
        d.vrs = Some(true);
        let marks = from_designations(&d, &comps);
        let text: Vec<String> = marks.iter().map(Mark::encode).collect();
        assert_eq!(text[0], "fd:naca0012 ff:fin_hex4 fm:off vrs:on");
        assert_eq!(text[2], "w s");
        let back: Vec<Mark> = text.iter().map(|s| Mark::decode(s)).collect();
        assert_eq!(back, marks);
        assert_eq!(to_designations(&comps, &back), Some(d));
        assert_eq!(to_designations(&comps, &[Mark::default(), Mark::default()]), None);
        assert_eq!(Mark::decode("w zz:1 r:none q"), Mark { wing: true, ..Default::default() });
    }
}

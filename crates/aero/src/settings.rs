//! The "All vehicles" settings, kept between game starts in `aero.xml` next to the game's
//! `save.xml` (%APPDATA%\Stormworks). It is the plugin's own file: the game never reads or
//! writes it, so the game's settings cannot be damaged, and without the plugin it just stays.

/// The file's text for these (name, value) settings.
pub fn encode(flags: &[(&str, bool)]) -> String {
    let attrs: String = flags.iter().map(|(n, v)| format!(" {n}=\"{v}\"")).collect();
    format!("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<aero{attrs}/>\n")
}

/// One setting from the file's text. None when it is missing or not `true` / `false` (a hand edit,
/// an older or newer plugin's file): the setting keeps its default.
pub fn decode(text: &str, name: &str) -> Option<bool> {
    crate::xml::tags(text).find(|t| t.name == "aero" && !t.close)?.get(name)?.trim().parse().ok()
}

#[cfg(windows)]
mod file {
    use std::sync::atomic::{AtomicBool, Ordering};

    use std::ffi::{c_char, CString};
    use std::sync::OnceLock;

    use stormworks_core::swml::{self, Bind, Setting, Value, MANAGED};

    use crate::logging::log_line;

    pub(crate) static ANALYSIS_KEY: Bind = Bind::new("F7");
    pub(crate) static VEHICLES_KEY: Bind = Bind::new("F8");
    pub(crate) static FLIGHT_DATA_KEY: Bind = Bind::new("F9");
    pub(crate) static UNDO_KEY: Bind = Bind::new("Ctrl+Z");
    pub(crate) static WING_PICK_KEY: Bind = Bind::new("Ctrl");

    /// What the mod manager shows: the "All vehicles" settings and every key.
    fn list() -> [Setting; 14] {
        let labels = [
            ("Plugin flight model", "The plugin's aerodynamics instead of the game's. Water forces unchanged."),
            ("Propeller and rotor model", "Propellers and rotors simulated blade by blade, with propwash."),
            ("Vortex ring state", "Rotors descending into their own downwash lose lift."),
            ("Ground effect", "Less induced drag near the ground or water."),
            ("Rotor blur", "Fast blades drawn as a motion smear."),
            ("Jet exhaust smoke", "Exhaust smoke from jet engines."),
            ("Jet engine tweaks", "Jet thrust rises with airspeed (ram air)."),
            ("Aerodynamic effects", "Wingtip vortex trails and condensation over the wings in humid air."),
            ("Condensation in any weather", "Condensation effects at any humidity."),
        ];
        let f = flags();
        let flag = |i: usize| Setting { id: f[i].0, label: labels[i].0, tip: labels[i].1, value: Value::Flag(f[i].1) };
        [
            flag(0),
            flag(1),
            flag(2),
            flag(3),
            flag(6),
            flag(4),
            flag(5),
            flag(7),
            flag(8),
            Setting {
                id: "analysis_window",
                label: "Analysis window",
                tip: "Workbench analysis: sweeps, stability, voxel view.",
                value: Value::Key(&ANALYSIS_KEY),
            },
            Setting {
                id: "vehicle_list",
                label: "Vehicle list (in flight)",
                tip: "Simulated vehicles, nearest first, with a Watch camera.",
                value: Value::Key(&VEHICLES_KEY),
            },
            Setting {
                id: "flight_data",
                label: "Flight data (in flight)",
                tip: "Airspeed, AoA, forces, CG / CoL markers, stalled blocks.",
                value: Value::Key(&FLIGHT_DATA_KEY),
            },
            Setting { id: "undo_designation", label: "Undo (analysis window)", tip: "Undo the last designation change.", value: Value::Key(&UNDO_KEY) },
            Setting {
                id: "wing_pick",
                label: "Pick a wing (workbench click)",
                tip: "Hold and click a block to select its wing.",
                value: Value::Hold(&WING_PICK_KEY),
            },
        ]
    }

    #[no_mangle]
    pub extern "C" fn swml_describe() -> *const c_char {
        static TEXT: OnceLock<CString> = OnceLock::new();
        TEXT.get_or_init(|| {
            // Defaults: aero.xml as it was before the manager had the settings.
            let file = stormworks_core::plugin::appdata().join("aero.xml");
            if let Ok(text) = std::fs::read_to_string(file) {
                for (name, flag) in flags() {
                    if let Some(v) = super::decode(&text, name) {
                        flag.store(v, Ordering::Relaxed);
                    }
                }
            }
            MANAGED.store(true, Ordering::Relaxed);
            CString::new(swml::describe(&list())).unwrap_or_default()
        })
        .as_ptr()
    }

    /// # Safety
    /// NUL-terminated strings, from the mod loader.
    #[no_mangle]
    pub unsafe extern "C" fn swml_set(id: *const c_char, value: *const c_char) {
        swml::set(&list(), id, value)
    }

    /// # Safety
    /// A NUL-terminated string, from the mod loader.
    #[no_mangle]
    pub unsafe extern "C" fn swml_get(id: *const c_char) -> *const c_char {
        swml::get(&list(), id)
    }

    fn flags() -> [(&'static str, &'static AtomicBool); 9] {
        [
            ("flight_model", &crate::flight::ENABLED),
            ("propeller_model", &crate::flight::PROPS),
            ("vortex_ring_state", &crate::flight::VRS),
            ("ground_effect", &crate::flight::GROUND),
            ("rotor_blur", &crate::blur::ENABLED),
            ("jet_smoke", &crate::hooks::JET_SMOKE),
            ("jet_speed_thrust", &crate::flight::JET_RAM),
            ("aero_effects", &crate::fx::ENABLED),
            ("aero_effects_any_weather", &crate::fx::ANY_WEATHER),
        ]
    }

    fn path() -> Option<std::path::PathBuf> {
        let dir = &crate::flight::PATHS.get()?.1;
        dir.is_dir().then(|| dir.join("aero.xml"))
    }

    /// At start-up. No file (first run) or an unreadable one: the defaults stay. Under the mod
    /// manager it has the settings (and read this file once, for its defaults).
    pub(crate) fn load() {
        if MANAGED.load(Ordering::Relaxed) {
            return;
        }
        let Some(text) = path().and_then(|p| std::fs::read_to_string(p).ok()) else { return };
        for (name, flag) in flags() {
            if let Some(v) = super::decode(&text, name) {
                flag.store(v, Ordering::Relaxed);
            }
        }
        log_line(&format!("settings read from aero.xml: {}", text.lines().last().unwrap_or_default()));
    }

    /// After a change. Written whole to a temporary file and renamed over the old one, so a
    /// crash mid-write leaves the previous settings.
    pub(crate) fn save() {
        if MANAGED.load(Ordering::Relaxed) {
            return; // the manager reads the flags back through swml_get
        }
        let Some(p) = path() else { return };
        let text = super::encode(&flags().map(|(n, f)| (n, f.load(Ordering::Relaxed))));
        let tmp = p.with_extension("xml.tmp");
        if let Err(e) = std::fs::write(&tmp, text).and_then(|_| std::fs::rename(&tmp, &p)) {
            log_line(&format!("settings not saved to {}: {e}", p.display()));
        }
    }
}
#[cfg(windows)]
pub(crate) use file::{load, save, ANALYSIS_KEY, FLIGHT_DATA_KEY, UNDO_KEY, VEHICLES_KEY, WING_PICK_KEY};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn settings_file_reads_back_and_bad_values_keep_the_default() {
        let text = encode(&[("flight_model", false), ("rotor_blur", true)]);
        assert_eq!(decode(&text, "flight_model"), Some(false));
        assert_eq!(decode(&text, "rotor_blur"), Some(true));
        assert_eq!(decode(&text, "ground_effect"), None);
        assert_eq!(decode("<aero flight_model=\"maybe\"/>", "flight_model"), None);
        assert_eq!(decode("", "flight_model"), None);
        assert_eq!(decode("<aero flight_model=\"fal", "flight_model"), None);
    }
}

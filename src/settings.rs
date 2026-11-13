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

    use crate::logging::log_line;

    fn flags() -> [(&'static str, &'static AtomicBool); 7] {
        [
            ("flight_model", &crate::flight::ENABLED),
            ("propeller_model", &crate::flight::PROPS),
            ("vortex_ring_state", &crate::flight::VRS),
            ("ground_effect", &crate::flight::GROUND),
            ("rotor_blur", &crate::blur::ENABLED),
            ("jet_smoke", &crate::hooks::JET_SMOKE),
            ("jet_speed_thrust", &crate::flight::JET_RAM),
        ]
    }

    fn path() -> Option<std::path::PathBuf> {
        let dir = &crate::flight::PATHS.get()?.1;
        dir.is_dir().then(|| dir.join("aero.xml"))
    }

    /// At start-up. No file (first run) or an unreadable one: the defaults stay.
    pub(crate) fn load() {
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
        let Some(p) = path() else { return };
        let text = super::encode(&flags().map(|(n, f)| (n, f.load(Ordering::Relaxed))));
        let tmp = p.with_extension("xml.tmp");
        if let Err(e) = std::fs::write(&tmp, text).and_then(|_| std::fs::rename(&tmp, &p)) {
            log_line(&format!("settings not saved to {}: {e}", p.display()));
        }
    }
}
#[cfg(windows)]
pub(crate) use file::{load, save};

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

//! Settings and key bindings the proxy mod loader's manager shows, sets and saves
//! (`<game>/UserData/<plugin>.ini`). Each plugin exports `swml_describe`, `swml_set` and `swml_get`
//! over its own list of [`Setting`]s; this is what they share. Key names are imgui's
//! (`F7`, `Z`, `Space`), with `Ctrl+` / `Shift+` / `Alt+` in front for a key pressed with them.

use std::ffi::{c_char, CStr};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use imgui::{Key, Ui};

/// A key binding, changed by the manager while the game runs.
pub struct Bind {
    default: &'static str,
    /// `UNSET` until set or first read; 0 = no key; else key index + 1 and the modifier bits.
    cur: AtomicU32,
}

const UNSET: u32 = u32::MAX;
const CTRL: u32 = 1 << 16;
const SHIFT: u32 = 1 << 17;
const ALT: u32 = 1 << 18;

/// `Ctrl+Z` -> its code; `Ctrl` alone (a held modifier) has no key bits. None: unknown name.
pub fn parse(s: &str) -> Option<u32> {
    if s.is_empty() || s == "None" {
        return Some(0);
    }
    let (mut code, mut key) = (0, None);
    for part in s.split('+').map(str::trim) {
        match part {
            "Ctrl" => code |= CTRL,
            "Shift" => code |= SHIFT,
            "Alt" => code |= ALT,
            name => key = Some(Key::VARIANTS.iter().position(|k| format!("{k:?}") == name)? as u32 + 1),
        }
    }
    Some(code | key.unwrap_or(0))
}

impl Bind {
    pub const fn new(default: &'static str) -> Bind {
        Bind { default, cur: AtomicU32::new(UNSET) }
    }

    fn code(&self) -> u32 {
        match self.cur.load(Ordering::Relaxed) {
            UNSET => parse(self.default).unwrap_or(0),
            c => c,
        }
    }

    /// False for a name this imgui does not know: the binding stays as it was.
    pub fn set(&self, s: &str) -> bool {
        parse(s).map(|c| self.cur.store(c, Ordering::Relaxed)).is_some()
    }

    fn key(code: u32) -> Option<Key> {
        Key::VARIANTS.get(((code & 0xffff) as usize).checked_sub(1)?).copied()
    }

    /// Pressed this frame with exactly its modifiers (repeats while held, as `is_key_pressed`).
    pub fn pressed(&self, ui: &Ui) -> bool {
        let code = self.code();
        let io = ui.io();
        let held = (io.key_ctrl as u32 * CTRL) | (io.key_shift as u32 * SHIFT) | (io.key_alt as u32 * ALT);
        Self::key(code).is_some_and(|k| code & !0xffff == held && ui.is_key_pressed(k))
    }

    /// Held now. A modifier on its own (`Ctrl`): that modifier.
    pub fn down(&self, ui: &Ui) -> bool {
        let code = self.code();
        let io = ui.io();
        match Self::key(code) {
            Some(k) => ui.is_key_down(k),
            None => (code & CTRL != 0 && io.key_ctrl) || (code & SHIFT != 0 && io.key_shift) || (code & ALT != 0 && io.key_alt),
        }
    }
}

pub enum Value {
    Flag(&'static AtomicBool),
    /// Pressed: `F7`, `Ctrl+Z`.
    Key(&'static Bind),
    /// Held: `Ctrl`, `Shift`, a key.
    Hold(&'static Bind),
}

pub struct Setting {
    pub id: &'static str,
    pub label: &'static str,
    /// The manager's tooltip; one line.
    pub tip: &'static str,
    pub value: Value,
}

/// Set by the first `swml_describe`: the manager keeps the settings, the plugin's own file is not
/// read or written.
pub static MANAGED: AtomicBool = AtomicBool::new(false);

/// `<bool|key|hold> <id> <default> <label> | <tooltip>` per setting; a flag's default is its
/// current value.
pub fn describe(list: &[Setting]) -> String {
    let line = |s: &Setting| {
        let (kind, default) = match &s.value {
            Value::Flag(f) => ("bool", if f.load(Ordering::Relaxed) { "true" } else { "false" }),
            Value::Key(b) => ("key", b.default),
            Value::Hold(b) => ("hold", b.default),
        };
        let tip = if s.tip.is_empty() { String::new() } else { format!(" | {}", s.tip.replace('\n', " ")) };
        format!("{kind} {} {default} {}{tip}\n", s.id, s.label)
    };
    list.iter().map(line).collect()
}

/// # Safety
/// `id` and `value` are NUL-terminated (or null: ignored).
pub unsafe fn set(list: &[Setting], id: *const c_char, value: *const c_char) {
    if id.is_null() || value.is_null() {
        return;
    }
    let (id, value) = (CStr::from_ptr(id).to_string_lossy(), CStr::from_ptr(value).to_string_lossy());
    match list.iter().find(|s| s.id == id).map(|s| &s.value) {
        Some(Value::Flag(f)) => {
            if let Ok(v) = value.parse() {
                f.store(v, Ordering::Relaxed);
            }
        }
        Some(Value::Key(b) | Value::Hold(b)) => {
            b.set(&value);
        }
        None => {}
    }
}

/// A flag's current value (its plugin's own UI changes it too); null for the rest.
/// # Safety
/// `id` is NUL-terminated (or null).
pub unsafe fn get(list: &[Setting], id: *const c_char) -> *const c_char {
    if id.is_null() {
        return std::ptr::null();
    }
    let id = CStr::from_ptr(id).to_string_lossy();
    match list.iter().find(|s| s.id == id).map(|s| &s.value) {
        Some(Value::Flag(f)) if f.load(Ordering::Relaxed) => c"true".as_ptr(),
        Some(Value::Flag(_)) => c"false".as_ptr(),
        _ => std::ptr::null(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bindings_parse_and_switch() {
        assert_eq!(parse("None"), Some(0));
        assert_eq!(parse("Ctrl"), Some(CTRL));
        let z = parse("Ctrl+Z").unwrap();
        assert_eq!(z & !0xffff, CTRL);
        assert_eq!(Bind::key(z), Some(Key::Z));
        assert_eq!(Bind::key(parse("F7").unwrap()), Some(Key::F7));
        assert_eq!(parse("Ctrl+Nope"), None);
        let b = Bind::new("F7");
        assert_eq!(Bind::key(b.code()), Some(Key::F7));
        assert!(b.set("Shift+Delete") && Bind::key(b.code()) == Some(Key::Delete));
        assert!(!b.set("Bogus") && Bind::key(b.code()) == Some(Key::Delete));
    }

    #[test]
    fn describe_set_get() {
        static F: AtomicBool = AtomicBool::new(true);
        static K: Bind = Bind::new("F9");
        let list = [Setting { id: "fly", label: "Flight model", tip: "", value: Value::Flag(&F) }, Setting { id: "data", label: "Flight data", tip: "Shows it.", value: Value::Key(&K) }];
        assert_eq!(describe(&list), "bool fly true Flight model\nkey data F9 Flight data | Shows it.\n");
        unsafe {
            set(&list, c"fly".as_ptr(), c"false".as_ptr());
            set(&list, c"data".as_ptr(), c"Ctrl+F9".as_ptr());
            assert_eq!(CStr::from_ptr(get(&list, c"fly".as_ptr())), c"false");
            assert!(get(&list, c"data".as_ptr()).is_null());
        }
        assert_eq!(K.code() & !0xffff, CTRL);
    }
}

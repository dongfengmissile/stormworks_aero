//! Each plugin's per-part data, stored on the components themselves: its own attribute on each
//! `<c>` of the vehicle XML (`markhooks`), and in memory this table of the attribute's text, keyed by
//! component pointer. The plugin decodes its own tokens; the table only carries the text through
//! save/load, copy/paste and undo.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{LazyLock, Mutex, MutexGuard};

static TABLE: LazyLock<Mutex<HashMap<usize, String>>> = LazyLock::new(Default::default);
/// Bumped when the game moves marks (load, clone, undo): the workbench snapshot re-reads them.
pub static GEN: AtomicU64 = AtomicU64::new(0);

/// The whole table, for edits across many parts.
pub fn table() -> MutexGuard<'static, HashMap<usize, String>> {
    TABLE.lock().unwrap_or_else(|p| p.into_inner())
}

pub fn get(comp: usize) -> Option<String> {
    table().get(&comp).cloned()
}

/// Set (or with an empty/None text, clear) a component's marks. `game`: the game moved them.
pub fn set(comp: usize, text: Option<String>, game: bool) {
    let mut t = table();
    let changed = match text.filter(|s| !s.is_empty()) {
        Some(s) => t.insert(comp, s.clone()).as_ref() != Some(&s),
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

/// Each part's text ("" for none).
pub fn read(ptrs: &[usize]) -> Vec<String> {
    let t = table();
    ptrs.iter().map(|p| t.get(p).cloned().unwrap_or_default()).collect()
}

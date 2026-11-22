//! The workbench vehicle (`stormworks_core::editor`) plus what this plugin does on each editor tick.

pub use stormworks_core::editor::*;

use std::sync::atomic::Ordering;

use crate::memory::RegionCacheScope;

/// Called after every editor tick, on the game thread.
pub(crate) unsafe fn on_tick(editor: usize) {
    stormworks_core::editor::on_tick(editor);
    // Once a second, panel open or not: every body's first part carries the vehicle settings.
    if TICKS.load(Ordering::Relaxed) % 60 == 1 {
        let _fault = crate::fault::enter(crate::fault::stage::EDITOR_SNAPSHOT);
        let _cache = RegionCacheScope::new();
        if let Some((raw, ptrs)) = read_raw(editor) {
            let parts: Vec<(usize, u32)> = ptrs.iter().zip(&raw).map(|(&p, r)| (p, r.3)).collect();
            crate::marks::spread_settings(&parts);
        }
    }
}


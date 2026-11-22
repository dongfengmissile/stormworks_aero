//! The Mechanisms button in the workbench top toolbar (after the game's own buttons,
//! `stormworks_core::toolbar`), its side shelf (Check, Labels, Sequence), and the Workbench Angle /
//! Offset rows in a joint's configuration panel.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};

use stormworks_core::toolbar::*;

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable, PTR_MIN};

/// Mode button and its shelf's ids (the game's run to ~62; the aero plugin's are 200..=245, 291..).
const MECH_ID: i32 = 260;
const ID_CHECK: i32 = 261;
const ID_LABELS: i32 = 263;
const ID_SEQ: i32 = 264;

static MECH_BUTTON: AtomicUsize = AtomicUsize::new(0);
static LABELS_BTN: AtomicUsize = AtomicUsize::new(0);
static SEQ_BTN: AtomicUsize = AtomicUsize::new(0);
/// The side shelf and the toolbar it was built for (game thread).
static SHELF: Mutex<(usize, usize)> = Mutex::new((0, 0));
static MECH_ACTIVE: AtomicBool = AtomicBool::new(false);

pub(crate) fn mech_active() -> bool {
    MECH_ACTIVE.load(Ordering::Relaxed) && crate::editor::editor_active()
}

/// Game icons the labels draw: name, its atlas's GL texture and UV rect (l, r, b, t). Set when the
/// shelf is built.
pub(crate) static LABEL_ICONS: Mutex<Vec<(&'static str, u32, [f32; 4])>> = Mutex::new(Vec::new());

unsafe fn append(f: &Fns, toolbar: usize, shelf: usize, ctx: usize) {
    let desc = "Check the joints: which are locked, and how far each sub-grid moves before it hits something. Hover a part for its joint.";
    MECH_BUTTON.store(add_tool_toggle(f, toolbar, shelf, ctx, "mechanics", MECH_ID, "Mechanisms", desc), Ordering::SeqCst);
}

unsafe fn event(editor: usize, i: i32) {
    match i {
        ID_SEQ => _ = crate::mechmode::SEQ_OPEN.fetch_xor(true, Ordering::Relaxed),
        ID_LABELS => _ = crate::mechmode::LABELS_SHOWN.fetch_xor(true, Ordering::Relaxed),
        ID_CHECK => crate::mechmode::CHECK_REQ.store(true, Ordering::Relaxed),
        MECH_ID => mode_clicked(editor, MECH_BUTTON.load(Ordering::SeqCst)),
        _ => {}
    }
}

unsafe fn click(_editor: usize, _at: [f32; 2]) {}

/// Editor tick (game thread): the shelf and the mode.
unsafe fn tick(f: &Fns, editor: usize, bar: Option<(usize, usize)>) {
    let Some((toolbar, iface)) = bar else {
        MECH_ACTIVE.store(false, Ordering::Relaxed);
        return;
    };
    let on = selected(MECH_BUTTON.load(Ordering::SeqCst)) && idle(editor);
    let ctx = editor + OFF_UI_CTX;
    let mut sh = SHELF.lock().unwrap_or_else(|p| p.into_inner());
    // Built once per interface (a child of it, freed with it), shown with the mode.
    if sh.0 != toolbar {
        let game = crate::plugin::PATHS.get().map(|p| p.0.clone()).unwrap_or_default();
        let icons = ["warning", "action_tick"].into_iter().filter_map(|n| Some((n, atlas_of(f, ctx, n)?.1, icon_uv(&game, n)?))).collect();
        *LABEL_ICONS.lock().unwrap_or_else(|p| p.into_inner()) = icons;
        let actions = [
            ("cube_wrench", "Check", "Sweep every free joint through its travel and find what each sub-grid hits first. Results then follow every change while the mode is on.", ID_CHECK, false),
            ("warning", "Labels", "Show each joint's result beside it. Point at a label, or at a part the joint moves, for the details.", ID_LABELS, true),
            ("playlist", "Sequence", "Open the sequence timeline: move joints one after another (a door opens, then the gear comes down) and play it.", ID_SEQ, true),
        ];
        let (shelf, btns) = build_shelf(f, iface, ctx, 0.0, &[], 0, 0, &actions);
        LABELS_BTN.store(btns.get(1).copied().unwrap_or(0), Ordering::SeqCst);
        SEQ_BTN.store(btns.get(2).copied().unwrap_or(0), Ordering::SeqCst);
        *sh = (toolbar, shelf);
        log_line(&format!("toolbar: Mechanisms shelf built {shelf:#x}"));
    }
    let shelf = sh.1;
    drop(sh);
    if shelf >= PTR_MIN && readable(shelf + 0x81, 1) && (*((shelf + 0x81) as *const u8) != 0) != on {
        set_shown(shelf, on, false);
    }
    set_lit(LABELS_BTN.load(Ordering::SeqCst), crate::mechmode::LABELS_SHOWN.load(Ordering::Relaxed));
    set_lit(SEQ_BTN.load(Ordering::SeqCst), crate::mechmode::SEQ_OPEN.load(Ordering::Relaxed));
    if MECH_ACTIVE.swap(on, Ordering::Relaxed) != on {
        log_line(&format!("toolbar: Mechanisms mode {}", if on { "on" } else { "off" }));
    }
    let _ = std::panic::catch_unwind(|| crate::mechmode::on_tick(on));
}

/// The toolbar hooks with the Mechanisms button. Call under `crate::hook::Exclusive`.
pub(crate) unsafe fn install() {
    if let Err(e) = stormworks_core::toolbar::install(Plugin { append, event, click, tick }) {
        log_line(&format!("{e} (Mechanisms button)"));
    }
}

// --- Workbench angle row on joints (knowledge/editor-ui.md "Joint parts (type 7) config panel",
// `22e792753489`): the parent half of a turning joint; the workbench shows its sub-grid posed.

/// A joint's workbench angle, degrees.
static ANGLE: SliderRow = SliderRow::new();
/// A sliding joint's workbench offset, metres.
static OFFSET: SliderRow = SliderRow::new();

pub(crate) const RVA_JOINT_PANEL: usize = 0x0035_B0D0;
pub(crate) const JOINT_PANEL_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x68, 0xA1, 0x48, 0x81, 0xEC, 0xD0, 0x00, 0x00];
pub(crate) static ORIGINAL_JOINT_PANEL: OnceLock<PartPanel> = OnceLock::new();
/// Definition: joint flags (0x80 child half), `constraint_type`, `constraint_range_of_motion` (turns).
const JOINT_CHILD_HALF: u32 = 0x80;
const OFF_DEF_CONSTRAINT_TYPE: usize = 0x33C;
const OFF_DEF_RANGE_OF_MOTION: usize = 0x38C;
const OFF_DEF_CONSTRAINT_AXIS: usize = 0x388;

pub(crate) unsafe extern "system" fn joint_panel_detour(comp: usize, ctx: usize, panel: usize) {
    let Some(orig) = ORIGINAL_JOINT_PANEL.get() else { return };
    orig(comp, ctx, panel);
    let _ = std::panic::catch_unwind(|| add_angle_row(comp, ctx, panel));
}

unsafe fn add_angle_row(comp: usize, ctx: usize, panel: usize) {
    if !readable(comp + 0x58, 8) {
        return;
    }
    let def = read_usize_at(comp + 0x58);
    if def < PTR_MIN || !readable(def + crate::offsets::OFF_DEF_PROP_FLAGS, 4) || !readable(def + OFF_DEF_RANGE_OF_MOTION, 4) {
        return;
    }
    let flags = read_u32_at(def + crate::offsets::OFF_DEF_PROP_FLAGS);
    if flags & JOINT_CHILD_HALF != 0 {
        return;
    }
    let range = f32::from_bits(read_u32_at(def + OFF_DEF_RANGE_OF_MOTION));
    if read_u32_at(def + OFF_DEF_CONSTRAINT_TYPE) == 1 {
        // A slide: centred on the build pose, or a linear track's run of extensions
        // (knowledge/vehicle-model.md "Sliders and linear tracks"). The row shows the setting,
        // which the game negates on a joint mirrored along its axis (`model::joint_links`).
        let (lo, hi) = if range > 0.0 { (-range as f64 / 2.0, range as f64 / 2.0) } else { crate::editor::track_travel_of(comp).unwrap_or((0.0, 0.0)) };
        let axis = read_u32_at(def + OFF_DEF_CONSTRAINT_AXIS).min(2);
        let (lo, hi) = if read_u32_at(def + crate::offsets::OFF_DEF_MIRROR) >> axis & 1 != 0 { (-hi, -lo) } else { (lo, hi) };
        if hi - lo < 1e-3 {
            return;
        }
        let o = crate::marks::get(comp).and_then(|m| m.joint_offset).unwrap_or(0.0).clamp(lo as f32, hi as f32);
        let step = if hi - lo > 2.0 { 0.05 } else { 0.01 };
        OFFSET.add(comp, ctx, panel, "Workbench Offset", "%.2f m", o, (lo as f32, hi as f32, step));
        return;
    }
    // The input of a velocity pivot or turret ring is a speed: no stops (`model::JointLink::travel`).
    let max = joint_max_deg(if flags & 0x200 != 0 { 0.0 } else { range });
    let a = crate::marks::get(comp).and_then(|m| m.joint_angle).unwrap_or(0.0).clamp(-max, max);
    ANGLE.add(comp, ctx, panel, "Workbench Angle", "%.0f deg", a.round(), (-max, max, 1.0));
}

/// The largest workbench angle of a joint, degrees: its range of motion (total turns), else a half turn.
fn joint_max_deg(range_turns: f32) -> f32 {
    let (_, hi) = crate::marks::JOINT_ANGLE_RANGE;
    if range_turns.is_finite() && range_turns > 0.0 { (range_turns * 180.0).min(hi) } else { hi }
}

/// Editor tick: a moved slider sets the selected part's mark (the workbench re-reads it).
pub(crate) unsafe fn panel_rows_tick(editor: usize) {
    if let Some((comp, v)) = OFFSET.moved(editor) {
        let mut m = crate::marks::get(comp).unwrap_or_default();
        let v = (v * 100.0).round() / 100.0;
        m.joint_offset = (v != 0.0).then_some(v);
        crate::marks::set(comp, Some(m), true);
    }
    if let Some((comp, v)) = ANGLE.moved(editor) {
        let mut m = crate::marks::get(comp).unwrap_or_default();
        m.joint_angle = (v.round() != 0.0).then_some(v.round());
        crate::marks::set(comp, Some(m), true);
    }
}

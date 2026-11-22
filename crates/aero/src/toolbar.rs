//! Native "Aero wings" button in the workbench top toolbar (after the game's own buttons,
//! `stormworks_core::toolbar`).
//!
//! The toggle switches the editor to the inert tool mode and the plugin owns left clicks in the 3D
//! view (picking on the vehicle, applied by the overlay). Its side shelves pick the tool, role and
//! aerofoil. See re-workbench round 13.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Mutex;

use stormworks_core::toolbar::*;

use crate::logging::log_line;
use crate::memory::{read_u32_at, read_usize_at, readable, PTR_MIN};

/// Our action id (the game's run to ~62; unknown ids only play the click sound).
const MY_ID: i32 = 200;
const ICON: &str = "propulsion";

/// Tool picked on the side shelf (0 wing blocks, 1 control role, 2 aerofoil), the role (index into
/// `Role::ALL`) and the aerofoil (index into `AEROFOILS`); set from the shelf clicks.
pub(crate) static TOOL: AtomicUsize = AtomicUsize::new(0);
pub(crate) static ROLE: AtomicUsize = AtomicUsize::new(1);
/// Aerofoil shelf category (0 lifting surfaces, 1 missile fins) and the pick in each.
pub(crate) static FOIL_CAT: AtomicUsize = AtomicUsize::new(0);
pub(crate) static FOIL: [AtomicUsize; 2] = [AtomicUsize::new(0), AtomicUsize::new(crate::wing::FIN_DEFAULT)];

/// The aerofoil picked on the shelves (index into `AEROFOILS`).
pub(crate) fn foil() -> usize {
    FOIL[FOIL_CAT.load(Ordering::Relaxed).min(1)].load(Ordering::Relaxed)
}

/// `AEROFOILS` range offered in each aerofoil category (the grid section is never offered).
fn foil_range(cat: usize) -> std::ops::Range<usize> {
    use crate::wing::{FoilKind, AEROFOILS};
    let kind = if cat == 0 { FoilKind::Wing } else { FoilKind::Fin };
    let first = AEROFOILS.iter().position(|a| a.kind == kind).unwrap_or(0);
    first..first + AEROFOILS[first..].iter().take_while(|a| a.kind == kind).count()
}
/// Wing blocks tool: 0 mark blocks, 1 fill body, 2 merge wings.
pub(crate) static WING_MODE: AtomicUsize = AtomicUsize::new(0);
const ID_WING: i32 = 231;
const ID_UNDO: i32 = 241;
const ID_REDO: i32 = 242;
const ID_RESET: i32 = 243;
const ID_RESET_YES: i32 = 244;
const ID_RESET_NO: i32 = 245;
const ID_NEW_WING: i32 = 246;
const ID_DETECT: i32 = 247;
/// Clicks on New wing / Detect wings the overlay has not applied yet.
pub(crate) static NEW_WING_REQ: AtomicBool = AtomicBool::new(false);
/// New wing is a standalone toggle (no group): lit while the marks go to a separate wing.
pub(crate) static NEW_WING_ON: AtomicBool = AtomicBool::new(false);
static NEW_BTN: AtomicUsize = AtomicUsize::new(0);
pub(crate) static DETECT_REQ: AtomicBool = AtomicBool::new(false);
/// Mouse over the 3D view (no UI element under it) while the mode is on: normalised view
/// position as `CLICK`. For the hover preview and drag painting.
static POINTER: Mutex<Option<[f32; 2]>> = Mutex::new(None);

pub(crate) fn pointer() -> Option<[f32; 2]> {
    *POINTER.lock().ok()?
}
/// A confirmed Reset the overlay has not applied yet.
pub(crate) static RESET_REQ: AtomicBool = AtomicBool::new(false);
/// The game's confirm dialog with our Reset text (built with the shelves, child of the interface),
/// and whether it is up (clicks then belong to it, not to the vehicle).
static RESET_DIALOG: AtomicUsize = AtomicUsize::new(0);
pub(crate) static RESET_ASKING: AtomicBool = AtomicBool::new(false);
/// Undo / redo clicks not yet applied by the overlay.
pub(crate) static UNDO_REQ: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
/// The Undo / Redo buttons of the first shelf, so their enabled state can follow the history.
static UNDO_BTN: AtomicUsize = AtomicUsize::new(0);
static REDO_BTN: AtomicUsize = AtomicUsize::new(0);
/// Whether the overlay has anything to undo / redo; a click with nothing to do is ignored.
pub(crate) static CAN_UNDO: AtomicBool = AtomicBool::new(false);
pub(crate) static CAN_REDO: AtomicBool = AtomicBool::new(false);
pub(crate) static REDO_REQ: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
const ID_TOOL: i32 = 201;
const ID_ROLE: i32 = 211;
const ID_FOIL_CAT: i32 = 291;
/// Aerofoil buttons: `ID_FOIL` + index into `AEROFOILS`.
const ID_FOIL: i32 = 301;

/// Side shelves (game thread): tools, wing modes, roles, aerofoil categories, lifting-surface
/// aerofoils, missile-fin aerofoils, and the toolbar they were built for.
static SHELVES: Mutex<(usize, [usize; 6])> = Mutex::new((0, [0; 6]));

/// Our button.
static BUTTON: AtomicUsize = AtomicUsize::new(0);
/// The plugin's wing mode is on (button selected and the editor idle).
static ACTIVE: AtomicBool = AtomicBool::new(false);
/// A left press in the 3D view while active: normalised view position (x right, y down, -1..1).
static CLICK: Mutex<Option<[f32; 2]>> = Mutex::new(None);

pub(crate) fn active() -> bool {
    ACTIVE.load(Ordering::Relaxed) && crate::editor::editor_active()
}

pub(crate) fn take_click() -> Option<[f32; 2]> {
    CLICK.lock().ok()?.take()
}

/// Our icons (tools/icons.py): DXT5 mips 64..4 px, written into the free lower half of the game's
/// icon atlas (1024², DXT5, 11 mips) on a 64-aligned grid so every mip stays block aligned.
const ICONS: [(&str, &[u8]); 24] = [
    ("aero_foil", include_bytes!("../assets/icons/aero_foil.bc3")),
    ("aero_x1", include_bytes!("../assets/icons/aero_x1.bc3")),
    ("aero_twinotter", include_bytes!("../assets/icons/aero_twinotter.bc3")),
    ("aero_cessna", include_bytes!("../assets/icons/aero_cessna.bc3")),
    ("aero_spitfire", include_bytes!("../assets/icons/aero_spitfire.bc3")),
    ("aero_extra", include_bytes!("../assets/icons/aero_extra.bc3")),
    ("aero_f86", include_bytes!("../assets/icons/aero_f86.bc3")),
    ("aero_a320", include_bytes!("../assets/icons/aero_a320.bc3")),
    ("aero_f4", include_bytes!("../assets/icons/aero_f4.bc3")),
    ("aero_f104", include_bytes!("../assets/icons/aero_f104.bc3")),
    ("aero_role_none", include_bytes!("../assets/icons/aero_role_none.bc3")),
    ("aero_pitch", include_bytes!("../assets/icons/aero_pitch.bc3")),
    ("aero_roll", include_bytes!("../assets/icons/aero_roll.bc3")),
    ("aero_yaw", include_bytes!("../assets/icons/aero_yaw.bc3")),
    ("aero_flap", include_bytes!("../assets/icons/aero_flap.bc3")),
    ("aero_wingblocks", include_bytes!("../assets/icons/aero_wingblocks.bc3")),
    ("aero_split", include_bytes!("../assets/icons/aero_split.bc3")),
    ("aero_detect", include_bytes!("../assets/icons/aero_detect.bc3")),
    ("aero_missile", include_bytes!("../assets/icons/aero_missile.bc3")),
    ("aero_m_sparrow", include_bytes!("../assets/icons/aero_m_sparrow.bc3")),
    ("aero_m_amraam", include_bytes!("../assets/icons/aero_m_amraam.bc3")),
    ("aero_m_sidewinder", include_bytes!("../assets/icons/aero_m_sidewinder.bc3")),
    ("aero_m_hellfire", include_bytes!("../assets/icons/aero_m_hellfire.bc3")),
    ("aero_m_rocket", include_bytes!("../assets/icons/aero_m_rocket.bc3")),
];
/// Aircraft / missile icon per aerofoil, in `AEROFOILS` order (the grid section is not offered).
const FOIL_ICONS: [&str; 15] = [
    "aero_x1",
    "aero_twinotter",
    "aero_cessna",
    "aero_spitfire",
    "aero_extra",
    "aero_f86",
    "aero_a320",
    "aero_f4",
    "aero_f104",
    "aero_m_sparrow",
    "aero_m_amraam",
    "aero_m_sidewinder",
    "aero_m_hellfire",
    "aero_m_rocket",
    "mechanics",
];

/// Atlas pixel origin of icon `i` (8 per row, 128 px apart, from (64, 576)).
fn icon_origin(i: usize) -> (u32, u32) {
    (64 + 128 * (i as u32 % 8), 576 + 128 * (i as u32 / 8))
}

/// GL texture of the icon atlas our regions point into (render thread uploads when it changes).
static ICON_TEX: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static ICONS_UPLOADED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Register our icon regions in the UI context (game thread, before buttons use them). False if
/// the atlas could not be found (buttons then fall back to stock icons).
unsafe fn register_icons(f: &Fns, ctx: usize) -> bool {
    let Some((tex, gl, size)) = atlas_of(f, ctx, "propulsion") else { return false };
    if size != [1024, 1024] {
        log_line("toolbar: icon atlas is not the expected 1024² texture; stock icons");
        return false;
    }
    for (i, (name, _)) in ICONS.iter().enumerate() {
        if (f.icon_find)(ctx, &mut gstr(f, name)) == 0 {
            let (x, y) = icon_origin(i);
            (f.icon_add_region)(ctx, &mut gstr(f, name), tex, x, x + 64, y, y + 64);
        }
    }
    ICON_TEX.store(gl, Ordering::SeqCst);
    true
}

/// Render thread, GL context current: write our icon pixels into the atlas texture once per texture.
pub(crate) fn upload_icons() {
    let tex = ICON_TEX.load(Ordering::SeqCst);
    if tex == 0 || ICONS_UPLOADED.load(Ordering::Relaxed) == tex {
        return;
    }
    ICONS_UPLOADED.store(tex, Ordering::Relaxed);
    let Some(gl) = crate::paint::gl() else { return };
    unsafe {
        let (mut prev, mut pbo) = (0i32, 0i32);
        (gl.get_integerv)(0x8069, &mut prev); // GL_TEXTURE_BINDING_2D
        (gl.get_integerv)(0x88EF, &mut pbo); // GL_PIXEL_UNPACK_BUFFER_BINDING: a bound PBO turns our pointer into an offset
        (gl.bind_buffer)(0x88EC, 0);
        (gl.bind_texture)(0x0DE1, tex);
        for (i, (_, data)) in ICONS.iter().enumerate() {
            let (x0, y0) = icon_origin(i);
            let mut off = 0usize;
            for k in 0..5 {
                let side = 64u32 >> k;
                let len = (side as usize / 4).pow(2) * 16;
                let Some(level) = data.get(off..off + len) else {
                    break;
                };
                (gl.compressed_tex_sub_image_2d)(
                    0x0DE1,
                    k as i32,
                    (x0 >> k) as i32,
                    (y0 >> k) as i32,
                    side as i32,
                    side as i32,
                    0x83F3,
                    len as i32,
                    level.as_ptr().cast(),
                );
                off += len;
            }
        }
        (gl.bind_texture)(0x0DE1, prev as u32);
        (gl.bind_buffer)(0x88EC, pbo as u32);
    }
    log_line(&format!("toolbar: {} icons written into atlas texture {tex}", ICONS.len()));
}

/// The three side shelves for the current interface (game thread, after it finished building).
unsafe fn build_shelves(f: &Fns, iface: usize, ctx: usize) -> [usize; 6] {
    use crate::wing::{Role, AEROFOILS};
    // Old buttons die with the old interface: never call into them if this build fails.
    UNDO_BTN.store(0, Ordering::SeqCst);
    REDO_BTN.store(0, Ordering::SeqCst);
    NEW_BTN.store(0, Ordering::SeqCst);
    let own = register_icons(f, ctx);
    let tools = [
        (if own { "aero_wingblocks" } else { "blocks" }, "Wing blocks", "Mark the blocks that form wings."),
        ("vehicle_control", "Control role", "Click a control surface or a hinged sub-grid to give it the role picked on the next shelf."),
        (
            if own { "aero_foil" } else { "mechanics" },
            "Aerofoil",
            "Click a wing to give it the aerofoil picked on the next shelf.",
        ),
    ];
    let wing_items = [
        ("paint_mode", "Mark blocks", "Click or drag over blocks to add them to the selected wing. Start on a block of it to unmark. Select a wing first: New wing starts one, Ctrl + click a wing selects it."),
        ("paint_fill", "Fill body", "Click a block to mark every block of its body as wing, however thick. Click a marked block to unmark them."),
        ("component", "Merge wings", "Click a wing, then the wing that should join it."),
    ];
    let wing_actions = [
        (if own { "aero_split" } else { "new" }, "New wing", "Start a separate wing and select it: Mark blocks adds to it. Off: no wing selected.", ID_NEW_WING, true),
        (if own { "aero_detect" } else { "mechanics" }, "Detect wings", "Mark the vehicle's thin slabs as wings and give its control surfaces a role.", ID_DETECT, false),
    ];
    let icons =
        if own { ["aero_role_none", "aero_pitch", "aero_roll", "aero_yaw", "aero_flap"] } else { ["action_cross", "arrows", "rotate", "orbit", "resize"] };
    let roles: Vec<(String, String)> = Role::ALL
        .iter()
        .map(|r| {
            (
                format!("Role: {}", r.name()),
                match r {
                    Role::None => "Plain wing: not a control surface.".to_string(),
                    Role::Pitch => "Elevator / stabilator: pitch.".to_string(),
                    Role::Roll => "Aileron: roll.".to_string(),
                    Role::Yaw => "Rudder: yaw.".to_string(),
                    Role::Flap => "Flap: lift and drag when lowered.".to_string(),
                },
            )
        })
        .collect();
    let role_items: Vec<(&str, &str, &str)> = roles.iter().zip(icons).map(|((t, d), i)| (i, t.as_str(), d.as_str())).collect();
    let foil_items = |cat: usize| -> Vec<(&str, &str, &str)> {
        foil_range(cat).map(|i| (if own { FOIL_ICONS[i] } else { "mechanics" }, AEROFOILS[i].name, AEROFOILS[i].note)).collect()
    };
    let cats = [
        (if own { "aero_foil" } else { "mechanics" }, "Lifting surfaces", "Wings, tails and control surfaces built from blocks."),
        (
            if own { "aero_missile" } else { "mechanics" },
            "Missile fins",
            "Fins on rockets and missile parts, shaped from the part's mesh.",
        ),
    ];
    let pick = |cat: usize| FOIL[cat].load(Ordering::Relaxed).saturating_sub(foil_range(cat).start);
    let tool_actions = [
        ("undo", "Undo", "Undo the last wing / role / aerofoil change.", ID_UNDO, false),
        ("redo", "Redo", "Redo the last undone change.", ID_REDO, false),
        ("action_cross", "Reset", "Remove every wing block, control role, aerofoil and separate wing from this vehicle.", ID_RESET, false),
    ];
    let (tools_shelf, b) = build_shelf(f, iface, ctx, 0.0, &tools, ID_TOOL, TOOL.load(Ordering::Relaxed), &tool_actions);
    UNDO_BTN.store(b.first().copied().unwrap_or(0), Ordering::SeqCst);
    REDO_BTN.store(b.get(1).copied().unwrap_or(0), Ordering::SeqCst);
    let (wing_shelf, b) = build_shelf(f, iface, ctx, 0.17, &wing_items, ID_WING, WING_MODE.load(Ordering::Relaxed).min(2), &wing_actions);
    NEW_BTN.store(b.first().copied().unwrap_or(0), Ordering::SeqCst);
    [
        tools_shelf,
        wing_shelf,
        build_shelf(f, iface, ctx, 0.17, &role_items, ID_ROLE, ROLE.load(Ordering::Relaxed), &[]).0,
        build_shelf(f, iface, ctx, 0.17, &cats, ID_FOIL_CAT, FOIL_CAT.load(Ordering::Relaxed), &[]).0,
        build_shelf(f, iface, ctx, 0.34, &foil_items(0), ID_FOIL + foil_range(0).start as i32, pick(0), &[]).0,
        build_shelf(f, iface, ctx, 0.34, &foil_items(1), ID_FOIL + foil_range(1).start as i32, pick(1), &[]).0,
    ]
}

unsafe fn append(f: &Fns, toolbar: usize, shelf: usize, ctx: usize) {
    BUTTON.store(add_tool_toggle(f, toolbar, shelf, ctx, ICON, MY_ID, "Aero wings", "Paint wings by role or aerofoil and designate them by clicking the vehicle."), Ordering::SeqCst);
}

unsafe fn event(editor: usize, i: i32) {
    if (ID_TOOL..ID_TOOL + 3).contains(&i) {
        TOOL.store((i - ID_TOOL) as usize, Ordering::Relaxed);
    } else if (ID_ROLE..ID_ROLE + 5).contains(&i) {
        ROLE.store((i - ID_ROLE) as usize, Ordering::Relaxed);
    } else if i == ID_UNDO {
        if CAN_UNDO.load(Ordering::Relaxed) {
            UNDO_REQ.fetch_add(1, Ordering::Relaxed);
        }
    } else if i == ID_REDO {
        if CAN_REDO.load(Ordering::Relaxed) {
            REDO_REQ.fetch_add(1, Ordering::Relaxed);
        }
    } else if i == ID_RESET {
        set_shown(RESET_DIALOG.load(Ordering::SeqCst), true, false);
    } else if i == ID_RESET_YES || i == ID_RESET_NO {
        set_shown(RESET_DIALOG.load(Ordering::SeqCst), false, false);
        RESET_REQ.store(i == ID_RESET_YES, Ordering::Relaxed);
    } else if i == ID_NEW_WING {
        NEW_WING_REQ.store(true, Ordering::Relaxed);
    } else if i == ID_DETECT {
        DETECT_REQ.store(true, Ordering::Relaxed);
    } else if (ID_WING..ID_WING + 3).contains(&i) {
        WING_MODE.store((i - ID_WING) as usize, Ordering::Relaxed);
    } else if (ID_FOIL_CAT..ID_FOIL_CAT + 2).contains(&i) {
        FOIL_CAT.store((i - ID_FOIL_CAT) as usize, Ordering::Relaxed);
    } else if (ID_FOIL..ID_FOIL + crate::wing::AEROFOILS.len() as i32).contains(&i) {
        let a = (i - ID_FOIL) as usize;
        if let Some(cat) = (0..2).find(|&c| foil_range(c).contains(&a)) {
            FOIL[cat].store(a, Ordering::Relaxed);
        }
    } else if i == MY_ID {
        mode_clicked(editor, BUTTON.load(Ordering::SeqCst));
    }
}

unsafe fn click(_editor: usize, at: [f32; 2]) {
    if ACTIVE.load(Ordering::Relaxed) {
        *CLICK.lock().unwrap_or_else(|p| p.into_inner()) = Some(at);
    }
}

/// Editor tick (game thread): the shelves, the reset dialog, the mode.
unsafe fn tick(f: &Fns, editor: usize, bar: Option<(usize, usize)>) {
    let Some((toolbar, iface)) = bar else {
        ACTIVE.store(false, Ordering::Relaxed);
        return;
    };
    let on = selected(BUTTON.load(Ordering::SeqCst)) && idle(editor);
    // Side shelves: built once per interface (children of it, freed with it), shown with the mode.
    let mut sh = SHELVES.lock().unwrap_or_else(|p| p.into_inner());
    if sh.0 != toolbar {
        sh.1 = build_shelves(f, iface, editor + OFF_UI_CTX);
        sh.0 = toolbar;
        // The game's own confirm screen (as Clear vehicle's), with our text and button ids.
        let (mut title, mut body) = (gstr(f, "RESET WINGS"), gstr(f, "Are you sure you want to remove every wing block, control role and aerofoil from this vehicle?"));
        let dlg = (f.op_new)(0xE8);
        if dlg >= PTR_MIN {
            (f.dialog_ctor)(dlg, &mut title, &mut body, 1, 1, ID_RESET_YES, ID_RESET_NO);
            (f.add_child)(iface, editor + OFF_UI_CTX, dlg);
        }
        RESET_DIALOG.store(dlg, Ordering::SeqCst);
        log_line(&format!("toolbar: side shelves built {:x?}", sh.1));
    }
    let dlg = RESET_DIALOG.load(Ordering::SeqCst);
    let mut asking = dlg >= PTR_MIN && readable(dlg + 0x81, 1) && *((dlg + 0x81) as *const u8) != 0;
    if asking && !on {
        set_shown(dlg, false, true);
        asking = false;
    }
    // The dark screen behind a dialog is the interface's own background element (the game fades
    // it in for its dialog states): held on while ours is up, as any state change turns it off.
    let bg = if readable(iface + OFF_IFACE_BACKGROUND, 8) { read_usize_at(iface + OFF_IFACE_BACKGROUND) } else { 0 };
    let was = RESET_ASKING.swap(asking, Ordering::Relaxed);
    if bg >= PTR_MIN && readable(bg, 0x70) {
        if asking {
            *((bg + 0x1C) as *mut u8) = 1; // visible, so its fade runs
            *((bg + 0x6C) as *mut u8) = 1; // fade in
        } else if was {
            *((bg + 0x6C) as *mut u8) = 0; // fades out and hides itself
        }
    }
    let tool = TOOL.load(Ordering::Relaxed);
    let cat = FOIL_CAT.load(Ordering::Relaxed);
    for (k, &shelf) in sh.1.iter().enumerate() {
        // Aerofoil: its category shelf, then the picked category's sections beside it. The
        // shelves draw over the dialog's background, so they go while it asks.
        let want = on && !asking && (k == 0 || k == tool + 1 || (tool == 2 && k == 4 + cat));
        if shelf >= PTR_MIN && readable(shelf + 0x81, 1) && (*((shelf + 0x81) as *const u8) != 0) != want {
            set_shown(shelf, want, false);
        }
    }
    drop(sh);
    *POINTER.lock().unwrap_or_else(|p| p.into_inner()) = if on && !asking { pointer_over_view(editor) } else { None };
    set_enabled(UNDO_BTN.load(Ordering::SeqCst), editor + OFF_UI_CTX, CAN_UNDO.load(Ordering::Relaxed));
    set_enabled(REDO_BTN.load(Ordering::SeqCst), editor + OFF_UI_CTX, CAN_REDO.load(Ordering::Relaxed));
    // The lit state follows the overlay.
    set_lit(NEW_BTN.load(Ordering::SeqCst), NEW_WING_ON.load(Ordering::Relaxed));
    if ACTIVE.swap(on, Ordering::Relaxed) != on {
        log_line(&format!("toolbar: Aero wings mode {}", if on { "on" } else { "off" }));
    }
}

/// The toolbar hooks with the Aero wings button. Call under `crate::hook::Exclusive`.
pub(crate) unsafe fn install() {
    if let Err(e) = stormworks_core::toolbar::install(Plugin { append, event, click, tick }) {
        log_line(&format!("{e} (Aero wings button)"));
    }
}

// --- Blade width row in the part's configuration panel (knowledge/editor-ui.md "Component
// configuration panel").

pub(crate) const RVA_PROP_PANEL: usize = 0x0034_5070;
pub(crate) const PROP_PANEL_PROLOGUE: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x55, 0x57, 0x41, 0x54, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x6C, 0x24, 0xC9, 0x48];
pub(crate) static ORIGINAL_PROP_PANEL: std::sync::OnceLock<PartPanel> = std::sync::OnceLock::new();

/// Blade width, % of the part's own.
static WIDTH: SliderRow = SliderRow::new();

pub(crate) unsafe extern "system" fn prop_panel_detour(comp: usize, ctx: usize, panel: usize) {
    let Some(orig) = ORIGINAL_PROP_PANEL.get() else { return };
    orig(comp, ctx, panel);
    let _ = std::panic::catch_unwind(|| add_width_row(comp, ctx, panel));
}

unsafe fn add_width_row(comp: usize, ctx: usize, panel: usize) {
    if !readable(comp + 0x58, 8) {
        return;
    }
    // Air propellers and rotors with blades, the parts the plugin flies (`defs`: air_blades).
    let def = read_usize_at(comp + 0x58);
    if def < PTR_MIN || !readable(def + crate::offsets::OFF_DEF_PROP_FLAGS, 4) || read_u32_at(def + crate::offsets::OFF_DEF_PROP_FLAGS) & 2 != 0 {
        return;
    }
    let mesh = read_usize_at(def);
    if mesh < PTR_MIN || !readable(mesh + crate::offsets::OFF_DEF_MESH1, 8) || read_usize_at(mesh + crate::offsets::OFF_DEF_MESH1) == 0 {
        return;
    }
    let w = crate::marks::get(comp).and_then(|m| m.blade_width).unwrap_or(1.0);
    let (lo, hi) = crate::marks::BLADE_WIDTH_RANGE;
    WIDTH.add(comp, ctx, panel, "Blade Width", "%.0f%%", (w * 100.0).round(), (lo * 100.0, hi * 100.0, 5.0));
}

/// Editor tick: a moved slider sets the selected part's mark (the workbench re-reads it).
pub(crate) unsafe fn panel_rows_tick(editor: usize) {
    if let Some((comp, v)) = WIDTH.moved(editor) {
        let (lo, hi) = crate::marks::BLADE_WIDTH_RANGE;
        let w = ((v).round() / 100.0).clamp(lo, hi);
        let mut m = crate::marks::get(comp).unwrap_or_default();
        m.blade_width = (w != 1.0).then_some(w);
        crate::marks::set(comp, Some(m), true);
    }
}

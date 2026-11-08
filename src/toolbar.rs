//! Native "Aero wings" button in the workbench top toolbar (after the microcontroller button).
//!
//! The toolbar is a retained UI tree built once by `ui_top_toolbar_init`; we append one more tool
//! toggle with the game's own constructors (game allocator, game strings), in the same tool group,
//! so it looks and toggles like the others. Its action id is handled in the editor's event hook:
//! it switches the editor to tool mode 11 (no tool acts on clicks) and the plugin owns left clicks
//! in the 3D view (picking on the vehicle, applied by the overlay). See re-workbench round 13.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};

use crate::logging::log_line;
use crate::memory::{bytes_match, read_u32_at, read_usize_at, readable, PTR_MIN};
use crate::state::MODULE_BASE;

/// Our action id (the game's run to ~62; unknown ids only play the click sound).
const MY_ID: i32 = 200;
/// Inert tool mode: past the game's tools (0..=10), so every per-mode switch takes its default
/// and no tool acts on hover, clicks or keys. Not 6 or 7: those poll a save / upload task that only
/// the save / upload action creates (null otherwise: crash at 0x7DA6C8).
const MODE_IDLE: i32 = 11;
const OFF_TOOL_MODE: usize = 0xE7C;
const OFF_MOUSE_X: usize = 0xB20;
const OFF_UI_CTX: usize = 0x448;
const OFF_UI_PRESSED: usize = 0x5D8;
const OFF_INTERFACE: usize = 0x8A8;
const OFF_TOP_TOOLBAR: usize = 0x60;
const RVA_TOOLBAR_VTABLE: usize = 0x00b6_bc58;
const TOOLBAR_SHELF: usize = 0x130;
const TOOLBAR_GROUP: usize = 0x88;
const BTN_SELECTED: usize = 0x128;
const ICON: &str = "propulsion";

pub(crate) const RVA_TOOLBAR_INIT: usize = 0x0061_e670;
pub(crate) const RVA_EDITOR_EVENT: usize = 0x007f_df70;
pub(crate) const RVA_EDITOR_MOUSE_BUTTON: usize = 0x007f_6c10;

#[repr(C)]
struct GStr {
    data: *mut u8,
    len: u32,
    cap: u32,
}

struct Fns {
    op_new: unsafe extern "system" fn(usize) -> usize,
    gstr_reserve: unsafe extern "system" fn(*mut GStr, u32),
    make_toggle: unsafe extern "system" fn(usize, f32, f32, u32, *mut GStr, i32, usize, usize) -> usize,
    add_child: unsafe extern "system" fn(usize, usize, usize),
    separator: unsafe extern "system" fn(usize, usize, f32),
    capsule_tooltip: unsafe extern "system" fn(usize, f32, f32, f32, f32, u32, u32) -> usize,
    set_tooltip_holder: unsafe extern "system" fn(usize, usize, usize) -> usize,
    basic_tooltip: unsafe extern "system" fn(usize, *mut GStr, *mut GStr, f32) -> usize,
    set_tooltip_content: unsafe extern "system" fn(usize, usize, usize),
    set_tool_mode: unsafe extern "system" fn(usize, i32, usize),
    slide_shelf_ctor: unsafe extern "system" fn(usize, f32, f32, f32, f32, i32, usize, usize) -> usize,
    slide_shelf_init_bg: unsafe extern "system" fn(usize, usize),
    rounded_rect_ctor: unsafe extern "system" fn(usize, f32, f32, f32, f32, i32, u32, *mut GStr) -> usize,
    rounded_rect_set_size: unsafe extern "system" fn(usize, usize, f32, f32),
    group_select: unsafe extern "system" fn(usize, usize, usize),
    icon_find: unsafe extern "system" fn(usize, *mut GStr) -> usize,
    make_action: unsafe extern "system" fn(usize, f32, f32, usize, *mut GStr, i32) -> usize,
    icon_add_region: unsafe extern "system" fn(usize, *mut GStr, i32, u32, u32, u32, u32),
}

/// Game functions, each gated on the prologue recorded for this build.
fn fns() -> Option<&'static Fns> {
    static F: OnceLock<Option<Fns>> = OnceLock::new();
    F.get_or_init(|| unsafe {
        let base = MODULE_BASE.load(Ordering::SeqCst);
        let at = |rva: usize, pro: &[u8]| -> Option<usize> {
            let a = base + rva;
            if bytes_match(a, pro) {
                Some(a)
            } else {
                log_line(&format!("toolbar: prologue mismatch at {a:#x}; no Aero wings button"));
                None
            }
        };
        Some(Fns {
            op_new: std::mem::transmute::<usize, _>(at(0x95_c6f8, &[0x40, 0x53, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xd9, 0xeb, 0x0f])?),
            gstr_reserve: std::mem::transmute::<usize, _>(at(0x17_ca10, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83, 0xec, 0x20, 0xbb, 0x20, 0x00, 0x00, 0x00])?),
            make_toggle: std::mem::transmute::<usize, _>(at(0x26_3a20, &[0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68, 0x10, 0x48, 0x89, 0x70, 0x18, 0x57, 0x48, 0x81, 0xec, 0x90])?),
            add_child: std::mem::transmute::<usize, _>(at(0x8f_aa70, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xf1, 0x49])?),
            separator: std::mem::transmute::<usize, _>(at(0x25_0e50, &[0x40, 0x53, 0x56, 0x57, 0x48, 0x83, 0xec, 0x40, 0x0f, 0x29, 0x74, 0x24, 0x30, 0x0f, 0x28, 0xf2])?),
            capsule_tooltip: std::mem::transmute::<usize, _>(at(0x24_7fa0, &[0x48, 0x89, 0x4c, 0x24, 0x08, 0x48, 0x83, 0xec, 0x18, 0x0f, 0x28, 0xe3, 0x48, 0x8b, 0xd1])?),
            set_tooltip_holder: std::mem::transmute::<usize, _>(at(0x24_a6b0, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xd9, 0x49, 0x8b, 0xf0])?),
            basic_tooltip: std::mem::transmute::<usize, _>(at(0x61_4260, &[0x4c, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4c, 0x24, 0x08, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56])?),
            set_tooltip_content: std::mem::transmute::<usize, _>(at(0x24_8890, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xfa, 0x48, 0x8b, 0xf1])?),
            set_tool_mode: std::mem::transmute::<usize, _>(at(0x80_2e50, &[0x48, 0x89, 0x5c, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0x6c, 0x24, 0xd9])?),
            slide_shelf_ctor: std::mem::transmute::<usize, _>(at(0x25_10c0, &[0x48, 0x89, 0x4c, 0x24, 0x08, 0x48, 0x83, 0xec, 0x18, 0xc7, 0x44, 0x24, 0x28, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8d, 0x05])?),
            slide_shelf_init_bg: std::mem::transmute::<usize, _>(at(0x25_11c0, &[0x40, 0x53, 0x56, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8b, 0xf2, 0x48, 0x8b, 0xf9, 0xb9, 0x60, 0x00, 0x00, 0x00, 0xe8])?),
            rounded_rect_ctor: std::mem::transmute::<usize, _>(at(0x24_58e0, &[0x4c, 0x8b, 0xdc, 0x49, 0x89, 0x5b, 0x18, 0x49, 0x89, 0x4b, 0x08, 0x55, 0x56, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8b, 0xf9, 0xc7])?),
            rounded_rect_set_size: std::mem::transmute::<usize, _>(at(0x24_5fc0, &[0x48, 0x83, 0xec, 0x28, 0xf3, 0x0f, 0x10, 0x61, 0x10, 0x4c, 0x8b, 0xd2, 0x0f, 0x2e, 0xe2, 0x4c, 0x8b, 0xc9, 0x7a, 0x0e])?),
            make_action: std::mem::transmute::<usize, _>(at(0x26_3730, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x81, 0xec, 0x80, 0x00, 0x00, 0x00, 0x0f, 0x29, 0x74, 0x24, 0x70])?),
            icon_find: std::mem::transmute::<usize, _>(at(0x1d_f300, &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x48, 0x89, 0x54, 0x24, 0x10, 0x57, 0x41, 0x54, 0x41])?),
            icon_add_region: std::mem::transmute::<usize, _>(at(0x1d_f190, &[0x48, 0x89, 0x5c, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xec, 0x20, 0x45, 0x8b, 0xf1])?),
            group_select: std::mem::transmute::<usize, _>(at(0x24_d2c0, &[0x48, 0x89, 0x5c, 0x24, 0x20, 0x41, 0x56, 0x48, 0x83, 0xec, 0x20, 0x48, 0x89, 0x6c, 0x24, 0x30, 0x4d, 0x8b, 0xf0, 0x8b, 0x69, 0x10])?),
        })
    })
    .as_ref()
}

/// Tool picked on the side shelf (0 wing blocks, 1 control role, 2 aerofoil), the role (index into
/// `Role::ALL`) and the aerofoil (index into `AEROFOILS`); set from the shelf clicks.
pub(crate) static TOOL: AtomicUsize = AtomicUsize::new(0);
pub(crate) static ROLE: AtomicUsize = AtomicUsize::new(1);
pub(crate) static FOIL: AtomicUsize = AtomicUsize::new(0);
/// Wing blocks tool: 0 toggle one block, 1 fill slab, 2 fill body, 3 remove slab, 4 new wing.
pub(crate) static WING_MODE: AtomicUsize = AtomicUsize::new(0);
const ID_WING: i32 = 231;
const ID_UNDO: i32 = 241;
const ID_REDO: i32 = 242;
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
const ID_FOIL: i32 = 221;

/// Side shelves (tools, roles, aerofoils) and the toolbar they were built for (game thread only).
static SHELVES: Mutex<(usize, [usize; 4])> = Mutex::new((0, [0; 4]));

/// Our button and the toolbar it was appended to.
static BUTTON: AtomicUsize = AtomicUsize::new(0);
static APPENDED_TOOLBAR: AtomicUsize = AtomicUsize::new(0);
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

/// A game string allocated by the game (the UI takes ownership and frees it).
unsafe fn gstr(f: &Fns, s: &str) -> GStr {
    let mut g = GStr { data: std::ptr::null_mut(), len: 0, cap: 0 };
    (f.gstr_reserve)(&mut g, s.len() as u32 + 1);
    if !g.data.is_null() {
        std::ptr::copy_nonoverlapping(s.as_ptr(), g.data, s.len());
        *g.data.add(s.len()) = 0;
        g.len = s.len() as u32;
    }
    g
}

/// Append our toggle (with a separator and a tooltip) to the toolbar's tool shelf. Game thread.
unsafe fn append(toolbar: usize, ctx: usize) {
    let Some(f) = fns() else { return };
    let shelf = read_usize_at(toolbar + TOOLBAR_SHELF);
    if shelf < PTR_MIN || !readable(shelf, 0x60) {
        log_line("toolbar: tool shelf not found; no Aero wings button");
        return;
    }
    (f.separator)(shelf, ctx, 0.05);
    let mut name = gstr(f, ICON);
    let btn = (f.make_toggle)(0, 0.0, 0.0, 0, &mut name, MY_ID, 0, toolbar + TOOLBAR_GROUP);
    if btn < PTR_MIN {
        log_line("toolbar: button constructor returned null");
        return;
    }
    (f.add_child)(shelf, ctx, btn);
    // Layout as the game does for its own buttons: next x on the shelf cursor, then grow the shelf.
    let f32_at = |a: usize| (a as *const f32).read_unaligned();
    let set = |a: usize, v: f32| (a as *mut f32).write_unaligned(v);
    let x = f32_at(shelf + 0x58);
    set(btn + 8, x);
    set(btn + 0xC, 0.0);
    let end = x + f32_at(btn + 0x10);
    set(shelf + 0x58, end);
    set(shelf + 0x10, end);
    set(shelf + 0x14, f32_at(shelf + 0x14).max(f32_at(btn + 0x14)));
    tooltip(f, btn, ctx, "Aero wings", "Paint wings by role or aerofoil and designate them by clicking the vehicle (aero plugin).", false);
    BUTTON.store(btn, Ordering::SeqCst);
    APPENDED_TOOLBAR.store(toolbar, Ordering::SeqCst);
    log_line(&format!("toolbar: Aero wings button added ({btn:#x}, toolbar {toolbar:#x})"));
}

/// Our icons (tools/icons.py): DXT5 mips 64..4 px, written into the free lower half of the game's
/// icon atlas (1024², DXT5, 11 mips) on a 64-aligned grid so every mip stays block aligned.
const ICONS: [(&str, &[u8]); 18] = [
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
    ("aero_fill_body", include_bytes!("../assets/icons/aero_fill_body.bc3")),
];
/// Aircraft icon per aerofoil, in `AEROFOILS` order.
const FOIL_ICONS: [&str; 9] = ["aero_x1", "aero_twinotter", "aero_cessna", "aero_spitfire", "aero_extra", "aero_f86", "aero_a320", "aero_f4", "aero_f104"];

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
    let reg = (f.icon_find)(ctx, &mut gstr(f, "propulsion"));
    if reg < PTR_MIN || !readable(reg, 40) {
        return false;
    }
    let tex = *((reg + 16) as *const i32);
    let (items, cap, offset) = (read_usize_at(ctx + 0x148), read_u32_at(ctx + 0x150), read_u32_at(ctx + 0x154));
    if items < PTR_MIN || cap == 0 || tex < 0 {
        return false;
    }
    let entry = items + 32 * ((tex as u32 + offset) % cap) as usize;
    let texobj = if readable(entry + 24, 8) { read_usize_at(entry + 24) } else { 0 };
    if texobj < PTR_MIN || !readable(texobj + 0x28, 4) || read_u32_at(entry + 16) != 1024 || read_u32_at(entry + 20) != 1024 {
        log_line("toolbar: icon atlas is not the expected 1024² texture; stock icons");
        return false;
    }
    for (i, (name, _)) in ICONS.iter().enumerate() {
        if (f.icon_find)(ctx, &mut gstr(f, name)) == 0 {
            let (x, y) = icon_origin(i);
            (f.icon_add_region)(ctx, &mut gstr(f, name), tex, x, x + 64, y, y + 64);
        }
    }
    ICON_TEX.store(read_u32_at(texobj + 0x28), Ordering::SeqCst);
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
                let Some(level) = data.get(off..off + len) else { break };
                (gl.compressed_tex_sub_image_2d)(0x0DE1, k as i32, (x0 >> k) as i32, (y0 >> k) as i32, side as i32, side as i32, 0x83F3, len as i32, level.as_ptr().cast());
                off += len;
            }
        }
        (gl.bind_texture)(0x0DE1, prev as u32);
        (gl.bind_buffer)(0x88EC, pbo as u32);
    }
    log_line(&format!("toolbar: {} icons written into atlas texture {tex}", ICONS.len()));
}

/// A game tooltip (title, description) on a shelf button.
unsafe fn tooltip(f: &Fns, btn: usize, ctx: usize, title: &str, desc: &str, side: bool) {
    let tt = (f.op_new)(0xA8);
    let cont = (f.op_new)(0xA0);
    if tt < PTR_MIN || cont < PTR_MIN {
        return;
    }
    if side {
        (f.capsule_tooltip)(tt, 0.119, -0.07, 0.0, 0.0, 0xFF27_292A, 3);
    } else {
        (f.capsule_tooltip)(tt, 0.07, -0.119, 0.0, 0.0, 0xFF27_292A, 1);
    }
    let holder = (f.set_tooltip_holder)(btn, ctx, tt);
    let (mut t, mut d) = (gstr(f, title), gstr(f, desc));
    (f.basic_tooltip)(cont, &mut t, &mut d, 0.56);
    if holder >= PTR_MIN {
        (f.set_tooltip_content)(holder, ctx, cont);
    }
}

/// Slide shelf's virtual `set_shown(show, instant)` (vtable +0x128).
unsafe fn set_shown(shelf: usize, show: bool, instant: bool) {
    if shelf < PTR_MIN || !readable(shelf, 0x88) {
        return;
    }
    let vt = read_usize_at(shelf);
    if vt < PTR_MIN || !readable(vt + 0x128, 8) {
        return;
    }
    let f: unsafe extern "system" fn(usize, u8, u8) = std::mem::transmute(read_usize_at(vt + 0x128));
    f(shelf, show as u8, instant as u8);
}

/// One vertical slide shelf at the left edge (`x` from it), like the game's selection shelf:
/// exclusive toggles (left orange bar) with ids `id0..`, `selected` picked. Starts hidden.
unsafe fn build_shelf(f: &Fns, iface: usize, ctx: usize, x: f32, items: &[(&str, &str, &str)], id0: i32, selected: usize, actions: &[(&str, &str, &str, i32)]) -> usize {
    let s = (f.op_new)(0xF0);
    if s < PTR_MIN {
        return 0;
    }
    let n = (items.len() + actions.len()) as f32;
    let h = 0.14 * n;
    (f.slide_shelf_ctor)(s, x, 0.0, 0.14, 0.0, 3, 0, 0);
    std::ptr::write_bytes((s + 0x88) as *mut u8, 0, 0xF0 - 0x88); // derived part: panel ptr, toggle group
    *((s + 0x90 + 0x18) as *mut u8) = 1; // group: one always selected
    (f.add_child)(iface, ctx, s);
    let w = ((s + 0x10) as *const f32).read_unaligned();
    ((s + 0x6C) as *mut f32).write_unaligned(-(w + x));
    ((s + 0x70) as *mut f32).write_unaligned(0.0);
    (f.slide_shelf_init_bg)(s, ctx);
    set_shown(s, false, true);
    // Dark rounded panel; beside the first shelf only as wide as this one (it would cover it).
    let pw = if x > 0.0 { 0.16 } else { 0.28 };
    let bg = (f.op_new)(0xC8);
    if bg >= PTR_MIN {
        let mut name = gstr(f, "main_ui_circle");
        (f.rounded_rect_ctor)(bg, 0.01, 0.0, pw, 0.0, 5, 0xEB14_1616, &mut name);
        (f.add_child)(s, ctx, bg);
    }
    let mut chosen = 0;
    for (i, (icon, title, desc)) in items.iter().enumerate() {
        let mut name = gstr(f, icon);
        let btn = (f.make_toggle)(0, 0.0, -0.14 * i as f32, 0, &mut name, id0 + i as i32, 2, s + 0x90);
        if btn < PTR_MIN {
            continue;
        }
        (f.add_child)(s, ctx, btn);
        tooltip(f, btn, ctx, title, desc, true);
        if i == selected || chosen == 0 {
            chosen = btn;
        }
    }
    for (j, (icon, title, desc, id)) in actions.iter().enumerate() {
        let mut name = gstr(f, icon);
        let btn = (f.make_action)(0, 0.0, -0.14 * (items.len() + j) as f32, 0, &mut name, *id);
        if btn >= PTR_MIN {
            (f.add_child)(s, ctx, btn);
            tooltip(f, btn, ctx, title, desc, true);
            match *id {
                ID_UNDO => UNDO_BTN.store(btn, Ordering::SeqCst),
                ID_REDO => REDO_BTN.store(btn, Ordering::SeqCst),
                _ => {}
            };
        }
    }
    ((s + 0x14) as *mut f32).write_unaligned(h);
    if bg >= PTR_MIN {
        (f.rounded_rect_set_size)(bg, ctx, pw, h + 0.05);
    }
    if chosen != 0 {
        (f.group_select)(s + 0x90, 0, chosen + 0x100);
    }
    s
}

/// The three side shelves for the current interface (game thread, after it finished building).
unsafe fn build_shelves(f: &Fns, iface: usize, ctx: usize) -> [usize; 4] {
    use crate::wing::{Role, AEROFOILS};
    let own = register_icons(f, ctx);
    let tools = [
        (if own { "aero_wingblocks" } else { "blocks" }, "Wing blocks", "Mark the blocks that form wings; pick how a click works on the next shelf."),
        ("vehicle_control", "Control role", "Click a control surface or a hinged sub-grid to give it the role picked on the next shelf."),
        (if own { "aero_foil" } else { "mechanics" }, "Aerofoil", "Click a wing to give it the aerofoil picked on the next shelf. One click sets the whole wing; use New wing to split it first."),
    ];
    let wing_items = [
        ("paint_mode", "Toggle a block", "Click a block to make it a wing block, or click it again to unmark it."),
        ("paint_add", "Fill slab", "Click a block to mark the whole thin slab around it as wing (a wing panel in one click)."),
        (if own { "aero_fill_body" } else { "paint_add" }, "Fill body", "Click a block to mark every block of its body as wing, however thick (a flying wing)."),
        ("erase", "Remove a slab", "Click a block to unmark the whole thin slab around it."),
        (if own { "aero_split" } else { "new" }, "New wing", "Click a wing panel to make it a wing of its own (lighter colour), e.g. to give part of a wing another aerofoil. Click it again to join it back."),
    ];
    let icons = if own { ["aero_role_none", "aero_pitch", "aero_roll", "aero_yaw", "aero_flap"] } else { ["action_cross", "arrows", "rotate", "orbit", "resize"] };
    let roles: Vec<(String, String)> = Role::ALL.iter().map(|r| (format!("Role: {}", r.name()), match r {
        Role::None => "Plain wing: not a control surface.".to_string(),
        Role::Pitch => "Elevator / stabilator: pitch.".to_string(),
        Role::Roll => "Aileron: roll.".to_string(),
        Role::Yaw => "Rudder: yaw.".to_string(),
        Role::Flap => "Flap: lift and drag when lowered.".to_string(),
    })).collect();
    let role_items: Vec<(&str, &str, &str)> = roles.iter().zip(icons).map(|((t, d), i)| (i, t.as_str(), d.as_str())).collect();
    let foil_items: Vec<(&str, &str, &str)> = AEROFOILS.iter().zip(FOIL_ICONS).map(|(a, icon)| (if own { icon } else { "mechanics" }, a.name, a.note)).collect();
    [
        build_shelf(f, iface, ctx, 0.0, &tools, ID_TOOL, TOOL.load(Ordering::Relaxed), &[("undo", "Undo", "Undo the last wing / role / aerofoil change.", ID_UNDO), ("redo", "Redo", "Redo the last undone change.", ID_REDO)]),
        build_shelf(f, iface, ctx, 0.17, &wing_items, ID_WING, WING_MODE.load(Ordering::Relaxed), &[]),
        build_shelf(f, iface, ctx, 0.17, &role_items, ID_ROLE, ROLE.load(Ordering::Relaxed), &[]),
        build_shelf(f, iface, ctx, 0.17, &foil_items, ID_FOIL, FOIL.load(Ordering::Relaxed), &[]),
    ]
}

fn selected(btn: usize) -> bool {
    btn >= PTR_MIN && readable(btn + BTN_SELECTED, 1) && unsafe { *((btn + BTN_SELECTED) as *const u8) } != 0
}

// ---------------------------------------------------------------------------------------------
// Hooks
// ---------------------------------------------------------------------------------------------

pub(crate) type ToolbarInit = unsafe extern "system" fn(usize, usize);
pub(crate) static ORIGINAL_INIT: OnceLock<ToolbarInit> = OnceLock::new();

pub(crate) unsafe extern "system" fn toolbar_init_detour(toolbar: usize, ctx: usize) {
    let Some(orig) = ORIGINAL_INIT.get() else { return };
    orig(toolbar, ctx);
    let _ = std::panic::catch_unwind(|| append(toolbar, ctx));
}

pub(crate) type EditorEvent = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize) -> u8;
pub(crate) static ORIGINAL_EVENT: OnceLock<EditorEvent> = OnceLock::new();

pub(crate) unsafe extern "system" fn editor_event_detour(editor: usize, a2: usize, a3: usize, a4: usize, a5: usize, id: usize) -> u8 {
    let Some(orig) = ORIGINAL_EVENT.get() else { return 0 };
    let r = orig(editor, a2, a3, a4, a5, id);
    let i = id as i32;
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
    } else if (ID_WING..ID_WING + 5).contains(&i) {
        WING_MODE.store((i - ID_WING) as usize, Ordering::Relaxed);
    } else if (ID_FOIL..ID_FOIL + crate::wing::AEROFOILS.len() as i32).contains(&i) {
        FOIL.store((i - ID_FOIL) as usize, Ordering::Relaxed);
    }
    if i == MY_ID {
        let _ = std::panic::catch_unwind(|| {
            let btn = BUTTON.load(Ordering::SeqCst);
            let Some(f) = fns() else { return };
            if selected(btn) {
                (f.set_tool_mode)(editor, MODE_IDLE, 0); // leaves the old tool; toggles untouched
            } else if readable(editor + OFF_TOOL_MODE, 4) && read_u32_at(editor + OFF_TOOL_MODE) == MODE_IDLE as u32 {
                (f.set_tool_mode)(editor, 0, 0); // untoggled: back to the default tool
            }
        });
    }
    r
}

pub(crate) type MouseButton = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize) -> u8;
pub(crate) static ORIGINAL_MOUSE: OnceLock<MouseButton> = OnceLock::new();

/// Left press the game's UI did not take, while our mode is on: queue a pick for the overlay.
pub(crate) unsafe extern "system" fn mouse_button_detour(editor: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, button: usize, action: usize) -> u8 {
    let Some(orig) = ORIGINAL_MOUSE.get() else { return 0 };
    let r = orig(editor, a2, a3, a4, a5, a6, button, action);
    // The return is not a "UI took it" bool (tool paths leave junk in al: 11 was seen). The UI
    // root's pressed element (root +0x1E0 = editor+0x5D8) is set by a press a UI element takes and
    // cleared by one none does.
    let ui_took = !readable(editor + OFF_UI_PRESSED, 8) || read_usize_at(editor + OFF_UI_PRESSED) != 0;
    static N: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    if ACTIVE.load(Ordering::Relaxed) && button as u32 == 0 && action as u32 == 1 && N.fetch_add(1, Ordering::Relaxed) < 30 {
        let m = if readable(editor + OFF_MOUSE_X, 8) { ((editor + OFF_MOUSE_X) as *const [f32; 2]).read_unaligned() } else { [f32::NAN; 2] };
        let ui = |o: usize| if readable(editor + OFF_UI_CTX + o, 8) { read_usize_at(editor + OFF_UI_CTX + o) } else { 0 };
        log_line(&format!("toolbar: left press ret={r} at {m:?} ui hovered {:#x} pressed {:#x}", ui(0x188), ui(0x190)));
    }
    if !ui_took && button as u32 == 0 && action as u32 == 1 && ACTIVE.load(Ordering::Relaxed) && readable(editor + OFF_MOUSE_X, 8) {
        let x = ((editor + OFF_MOUSE_X) as *const f32).read_unaligned();
        let y = ((editor + OFF_MOUSE_X + 4) as *const f32).read_unaligned();
        if x.is_finite() && y.is_finite() {
            *CLICK.lock().unwrap_or_else(|p| p.into_inner()) = Some([x, y]);
        }
    }
    r
}

/// Grey a button out the way the game does (vtable +0x128: enabled flag + dimmed children).
unsafe fn set_enabled(btn: usize, ctx: usize, enabled: bool) {
    if btn < PTR_MIN || ctx < PTR_MIN || !readable(btn, 0xA4) {
        return;
    }
    let vt = read_usize_at(btn);
    if vt < PTR_MIN || !readable(vt + 0x128, 8) {
        return;
    }
    let f: unsafe extern "system" fn(usize, usize, u8) = std::mem::transmute(read_usize_at(vt + 0x128));
    f(btn, ctx, enabled as u8);
}

/// Editor tick (game thread): append to a toolbar built before the plugin loaded, track the mode.
pub(crate) unsafe fn on_tick(editor: usize) {
    if fns().is_none() || !readable(editor + OFF_INTERFACE, 8) {
        return;
    }
    let iface = read_usize_at(editor + OFF_INTERFACE);
    if iface < PTR_MIN || !readable(iface + OFF_TOP_TOOLBAR, 8) {
        return;
    }
    let toolbar = read_usize_at(iface + OFF_TOP_TOOLBAR);
    let vt = MODULE_BASE.load(Ordering::Relaxed) + RVA_TOOLBAR_VTABLE;
    if toolbar < PTR_MIN || !readable(toolbar, 0x138) || read_usize_at(toolbar) != vt {
        ACTIVE.store(false, Ordering::Relaxed);
        return;
    }
    if APPENDED_TOOLBAR.load(Ordering::SeqCst) != toolbar {
        append(toolbar, editor + OFF_UI_CTX);
    }
    let on = selected(BUTTON.load(Ordering::SeqCst)) && read_u32_at(editor + OFF_TOOL_MODE) == MODE_IDLE as u32;
    // Side shelves: built once per interface (children of it, freed with it), shown with the mode.
    let mut sh = SHELVES.lock().unwrap_or_else(|p| p.into_inner());
    if sh.0 != toolbar {
        if let Some(f) = fns() {
            sh.1 = build_shelves(f, iface, editor + OFF_UI_CTX);
            sh.0 = toolbar;
            log_line(&format!("toolbar: side shelves built {:x?}", sh.1));
        }
    }
    let tool = TOOL.load(Ordering::Relaxed);
    for (k, &shelf) in sh.1.iter().enumerate() {
        let want = on && (k == 0 || k == tool + 1);
        if shelf >= PTR_MIN && readable(shelf + 0x81, 1) && (*((shelf + 0x81) as *const u8) != 0) != want {
            set_shown(shelf, want, false);
        }
    }
    drop(sh);
    set_enabled(UNDO_BTN.load(Ordering::SeqCst), editor + OFF_UI_CTX, CAN_UNDO.load(Ordering::Relaxed));
    set_enabled(REDO_BTN.load(Ordering::SeqCst), editor + OFF_UI_CTX, CAN_REDO.load(Ordering::Relaxed));
    if ACTIVE.swap(on, Ordering::Relaxed) != on {
        log_line(&format!("toolbar: Aero wings mode {}", if on { "on" } else { "off" }));
    }
}

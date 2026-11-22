//! The workbench top toolbar, shared by the plugins (knowledge/editor-ui.md): each plugin appends
//! its own tool toggles after the game's, with the game's own constructors (game allocator, game
//! strings) and in the same tool group, and its own side shelves. A plugin hands its callbacks to
//! `install`; the toolbar init, editor event and mouse button hooks call them (both plugins hook
//! them, chained through `crate::hook`).

use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::OnceLock;

use crate::logging::log_line;
use crate::memory::{bytes_match, read_u32_at, read_usize_at, readable, PTR_MIN};
use crate::state::MODULE_BASE;

/// Inert tool mode: past the game's tools (0..=10), so every per-mode switch takes its default
/// and no tool acts on hover, clicks or keys. Not 6 or 7: those poll a save / upload task that only
/// the save / upload action creates (null otherwise: crash at 0x7DA6C8).
pub const MODE_IDLE: i32 = 11;
pub const OFF_TOOL_MODE: usize = 0xE7C;
pub const OFF_MOUSE_X: usize = 0xB20;
pub const OFF_UI_CTX: usize = 0x448;
pub const OFF_UI_PRESSED: usize = 0x5D8;
pub const OFF_INTERFACE: usize = 0x8A8;
pub const OFF_TOP_TOOLBAR: usize = 0x60;
/// `c_ui_element_background` of the interface: the dark screen behind its dialogs.
pub const OFF_IFACE_BACKGROUND: usize = 0x98;
pub const RVA_TOOLBAR_VTABLE: usize = 0x00b6_cbd0;
pub const TOOLBAR_SHELF: usize = 0x130;
pub const TOOLBAR_GROUP: usize = 0x88;
pub const BTN_SELECTED: usize = 0x128;

pub const RVA_TOOLBAR_INIT: usize = 0x0061_e670;
pub const RVA_EDITOR_EVENT: usize = 0x007f_df70;
pub const RVA_EDITOR_MOUSE_BUTTON: usize = 0x007f_6c10;
pub const TOOLBAR_INIT_PROLOGUE: [u8; 32] = [0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x08, 0xfe, 0xff, 0xff, 0x48, 0x81, 0xec, 0xc0, 0x02, 0x00, 0x00,];
pub const EDITOR_EVENT_PROLOGUE: [u8; 32] = [0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x78, 0xfc, 0xff, 0xff, 0x48, 0x81, 0xec, 0x48, 0x04, 0x00, 0x00, 0x0f, 0x29, 0x70,];
pub const EDITOR_MOUSE_BUTTON_PROLOGUE: [u8; 32] = [0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0xb8, 0xfb, 0xff, 0xff, 0x48, 0x81, 0xec, 0x08, 0x05, 0x00, 0x00, 0x0f, 0x29, 0x70,];

#[repr(C)]
pub struct GStr {
    pub data: *mut u8,
    pub len: u32,
    pub cap: u32,
}

pub struct Fns {
    pub op_new: unsafe extern "system" fn(usize) -> usize,
    pub gstr_reserve: unsafe extern "system" fn(*mut GStr, u32),
    pub make_toggle: unsafe extern "system" fn(usize, f32, f32, u32, *mut GStr, i32, usize, usize) -> usize,
    pub add_child: unsafe extern "system" fn(usize, usize, usize),
    pub separator: unsafe extern "system" fn(usize, usize, f32),
    pub capsule_tooltip: unsafe extern "system" fn(usize, f32, f32, f32, f32, u32, u32) -> usize,
    pub set_tooltip_holder: unsafe extern "system" fn(usize, usize, usize) -> usize,
    pub basic_tooltip: unsafe extern "system" fn(usize, *mut GStr, *mut GStr, f32) -> usize,
    pub set_tooltip_content: unsafe extern "system" fn(usize, usize, usize),
    pub set_tool_mode: unsafe extern "system" fn(usize, i32, usize),
    pub slide_shelf_ctor: unsafe extern "system" fn(usize, f32, f32, f32, f32, i32, usize, usize) -> usize,
    pub slide_shelf_init_bg: unsafe extern "system" fn(usize, usize),
    pub rounded_rect_ctor: unsafe extern "system" fn(usize, f32, f32, f32, f32, i32, u32, *mut GStr) -> usize,
    pub rounded_rect_set_size: unsafe extern "system" fn(usize, usize, f32, f32),
    pub group_select: unsafe extern "system" fn(usize, usize, usize),
    pub icon_find: unsafe extern "system" fn(usize, *mut GStr) -> usize,
    pub make_action: unsafe extern "system" fn(usize, f32, f32, usize, *mut GStr, i32) -> usize,
    pub icon_add_region: unsafe extern "system" fn(usize, *mut GStr, i32, u32, u32, u32, u32),
    /// `ui_confirm_cancel_dialog_basic_ctor(obj, title, body, show_cancel, show_confirm, confirm_id, cancel_id)`.
    pub dialog_ctor: unsafe extern "system" fn(usize, *mut GStr, *mut GStr, u8, u8, i32, i32) -> usize,
}

/// Game functions, each gated on the prologue recorded for this build.
pub fn fns() -> Option<&'static Fns> {
    static F: OnceLock<Option<Fns>> = OnceLock::new();
    F.get_or_init(|| unsafe {
        let base = MODULE_BASE.load(Ordering::SeqCst);
        let at = |rva: usize, pro: &[u8]| -> Option<usize> {
            let a = base + rva;
            if bytes_match(a, pro) {
                Some(a)
            } else {
                log_line(&format!("toolbar: prologue mismatch at {a:#x}; no toolbar buttons"));
                None
            }
        };
        Some(Fns {
            op_new: std::mem::transmute::<usize, _>(at(0x95_ca68, &[0x40, 0x53, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xd9, 0xeb, 0x0f])?),
            gstr_reserve: std::mem::transmute::<usize, _>(at(
                0x17_ca10,
                &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83, 0xec, 0x20, 0xbb, 0x20, 0x00, 0x00, 0x00],
            )?),
            make_toggle: std::mem::transmute::<usize, _>(at(
                0x26_3a20,
                &[0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68, 0x10, 0x48, 0x89, 0x70, 0x18, 0x57, 0x48, 0x81, 0xec, 0x90],
            )?),
            add_child: std::mem::transmute::<usize, _>(at(
                0x8f_ade0,
                &[
                    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xf1,
                    0x49,
                ],
            )?),
            separator: std::mem::transmute::<usize, _>(at(
                0x25_0e50,
                &[0x40, 0x53, 0x56, 0x57, 0x48, 0x83, 0xec, 0x40, 0x0f, 0x29, 0x74, 0x24, 0x30, 0x0f, 0x28, 0xf2],
            )?),
            capsule_tooltip: std::mem::transmute::<usize, _>(at(
                0x24_7fa0,
                &[0x48, 0x89, 0x4c, 0x24, 0x08, 0x48, 0x83, 0xec, 0x18, 0x0f, 0x28, 0xe3, 0x48, 0x8b, 0xd1],
            )?),
            set_tooltip_holder: std::mem::transmute::<usize, _>(at(
                0x24_a6b0,
                &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xd9, 0x49, 0x8b, 0xf0],
            )?),
            basic_tooltip: std::mem::transmute::<usize, _>(at(
                0x61_4260,
                &[0x4c, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4c, 0x24, 0x08, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56],
            )?),
            set_tooltip_content: std::mem::transmute::<usize, _>(at(
                0x24_8890,
                &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xfa, 0x48, 0x8b, 0xf1],
            )?),
            set_tool_mode: std::mem::transmute::<usize, _>(at(
                0x80_2e50,
                &[0x48, 0x89, 0x5c, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0x6c, 0x24, 0xd9],
            )?),
            slide_shelf_ctor: std::mem::transmute::<usize, _>(at(
                0x25_10c0,
                &[0x48, 0x89, 0x4c, 0x24, 0x08, 0x48, 0x83, 0xec, 0x18, 0xc7, 0x44, 0x24, 0x28, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8d, 0x05],
            )?),
            slide_shelf_init_bg: std::mem::transmute::<usize, _>(at(
                0x25_11c0,
                &[0x40, 0x53, 0x56, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8b, 0xf2, 0x48, 0x8b, 0xf9, 0xb9, 0x60, 0x00, 0x00, 0x00, 0xe8],
            )?),
            rounded_rect_ctor: std::mem::transmute::<usize, _>(at(
                0x24_58e0,
                &[0x4c, 0x8b, 0xdc, 0x49, 0x89, 0x5b, 0x18, 0x49, 0x89, 0x4b, 0x08, 0x55, 0x56, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8b, 0xf9, 0xc7],
            )?),
            rounded_rect_set_size: std::mem::transmute::<usize, _>(at(
                0x24_5fc0,
                &[0x48, 0x83, 0xec, 0x28, 0xf3, 0x0f, 0x10, 0x61, 0x10, 0x4c, 0x8b, 0xd2, 0x0f, 0x2e, 0xe2, 0x4c, 0x8b, 0xc9, 0x7a, 0x0e],
            )?),
            make_action: std::mem::transmute::<usize, _>(at(
                0x26_3730,
                &[0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x81, 0xec, 0x80, 0x00, 0x00, 0x00, 0x0f, 0x29, 0x74, 0x24, 0x70],
            )?),
            icon_find: std::mem::transmute::<usize, _>(at(
                0x1d_f300,
                &[
                    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x48, 0x89, 0x54, 0x24, 0x10, 0x57, 0x41, 0x54,
                    0x41,
                ],
            )?),
            icon_add_region: std::mem::transmute::<usize, _>(at(
                0x1d_f190,
                &[
                    0x48, 0x89, 0x5c, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xec, 0x20, 0x45, 0x8b,
                    0xf1,
                ],
            )?),
            dialog_ctor: std::mem::transmute::<usize, _>(at(
                0x25_b1c0,
                &[0x48, 0x89, 0x5c, 0x24, 0x18, 0x48, 0x89, 0x4c, 0x24, 0x08, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xec, 0x20, 0x41, 0x0f, 0xb6, 0xe9],
            )?),
            group_select: std::mem::transmute::<usize, _>(at(
                0x24_d2c0,
                &[0x48, 0x89, 0x5c, 0x24, 0x20, 0x41, 0x56, 0x48, 0x83, 0xec, 0x20, 0x48, 0x89, 0x6c, 0x24, 0x30, 0x4d, 0x8b, 0xf0, 0x8b, 0x69, 0x10],
            )?),
        })
    })
    .as_ref()
}


/// A game string allocated by the game (the UI takes ownership and frees it).
pub unsafe fn gstr(f: &Fns, s: &str) -> GStr {
    let mut g = GStr { data: std::ptr::null_mut(), len: 0, cap: 0 };
    (f.gstr_reserve)(&mut g, s.len() as u32 + 1);
    if !g.data.is_null() {
        std::ptr::copy_nonoverlapping(s.as_ptr(), g.data, s.len());
        *g.data.add(s.len()) = 0;
        g.len = s.len() as u32;
    }
    g
}


/// The UI context's texture index, GL texture and size of the atlas holding icon `name`.
pub unsafe fn atlas_of(f: &Fns, ctx: usize, name: &str) -> Option<(i32, u32, [u32; 2])> {
    let reg = (f.icon_find)(ctx, &mut gstr(f, name));
    if reg < PTR_MIN || !readable(reg, 40) {
        return None;
    }
    let tex = *((reg + 16) as *const i32);
    let (items, cap, offset) = (read_usize_at(ctx + 0x148), read_u32_at(ctx + 0x150), read_u32_at(ctx + 0x154));
    if items < PTR_MIN || cap == 0 || tex < 0 {
        return None;
    }
    let entry = items + 32 * ((tex as u32 + offset) % cap) as usize;
    let texobj = if readable(entry + 24, 8) { read_usize_at(entry + 24) } else { 0 };
    if texobj < PTR_MIN || !readable(texobj + 0x28, 4) {
        return None;
    }
    Some((tex, read_u32_at(texobj + 0x28), [read_u32_at(entry + 16), read_u32_at(entry + 20)]))
}


/// UV rect (l, r, b, t) of a game icon, from the atlas XMLs.
pub fn icon_uv(game: &std::path::Path, name: &str) -> Option<[f32; 4]> {
    let key = format!("name=\"{name}\"");
    ["gameicons", "gameicons2", "gameicons3"].iter().find_map(|d| {
        let s = std::fs::read_to_string(game.join(format!("rom/graphics/ui/{d}/atlas_{d}.xml"))).ok()?;
        let line = s.lines().find(|l| l.contains(&key))?;
        let num = |k: &str| line.split(&format!(" {k}=\"")).nth(1)?.split('"').next()?.parse::<f32>().ok();
        Some([num("l")?, num("r")?, num("b")?, num("t")?])
    })
}

/// An atlas region's pixels, RGBA rows top first (render thread, GL context current).
pub fn read_region(tex: u32, uv: [f32; 4]) -> Option<(Vec<u8>, u32, u32)> {
    let gl = crate::paint::gl()?;
    unsafe {
        let q = |p: u32| {
            let mut v = 0i32;
            (gl.get_integerv)(p, &mut v);
            v
        };
        // Pack state the game may have left set would make GL write past our buffer.
        if q(0x0D02) != 0 || q(0x0D03) != 0 || q(0x0D04) != 0 {
            return None; // GL_PACK_ROW_LENGTH, _SKIP_ROWS, _SKIP_PIXELS
        }
        let (prev, pbo) = (q(0x8069), q(0x88ED)); // texture 2D, pixel pack buffer bindings
        (gl.bind_buffer)(0x88EB, 0);
        (gl.bind_texture)(0x0DE1, tex);
        let (mut w, mut h) = (0i32, 0i32);
        (gl.get_tex_level_parameteriv)(0x0DE1, 0, 0x1000, &mut w);
        (gl.get_tex_level_parameteriv)(0x0DE1, 0, 0x1001, &mut h);
        let ok = (1..=8192).contains(&w) && (1..=8192).contains(&h);
        let mut all = vec![0u8; if ok { (w * h * 4) as usize } else { 0 }];
        if ok {
            (gl.get_tex_image)(0x0DE1, 0, 0x1908, 0x1401, all.as_mut_ptr().cast()); // RGBA, unsigned byte
        }
        (gl.bind_texture)(0x0DE1, prev as u32);
        (gl.bind_buffer)(0x88EB, pbo as u32);
        if !ok {
            return None;
        }
        let [l, r, b, t] = uv;
        let (w, h) = (w as u32, h as u32);
        let (x0, x1) = ((l * w as f32).round() as u32, (r * w as f32).round() as u32);
        let (y0, y1) = ((b * h as f32).round() as u32, (t * h as f32).round() as u32);
        if x1 <= x0 || y1 <= y0 || x1 > w || y1 > h {
            return None;
        }
        // The atlas is stored bottom row first (tools/icons.py).
        let px = (y0..y1).rev().flat_map(|y| {
            let row = ((y * w + x0) * 4) as usize;
            all[row..row + ((x1 - x0) * 4) as usize].to_vec()
        });
        Some((px.collect(), x1 - x0, y1 - y0))
    }
}

/// A game tooltip (title, description) on a shelf button.
pub unsafe fn tooltip(f: &Fns, btn: usize, ctx: usize, title: &str, desc: &str, side: bool) {
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
pub unsafe fn set_shown(shelf: usize, show: bool, instant: bool) {
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
pub unsafe fn build_shelf(
    f: &Fns,
    iface: usize,
    ctx: usize,
    x: f32,
    items: &[(&str, &str, &str)],
    id0: i32,
    selected: usize,
    actions: &[(&str, &str, &str, i32, bool)],
) -> (usize, Vec<usize>) {
    let s = (f.op_new)(0xF0);
    if s < PTR_MIN {
        return (0, Vec::new());
    }
    let mut action_btns = vec![0; actions.len()];
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
    for (j, (icon, title, desc, id, toggle)) in actions.iter().enumerate() {
        let mut name = gstr(f, icon);
        let y = -0.14 * (items.len() + j) as f32;
        // Group 0: the game flips the toggle itself on a click, then sends the id.
        let btn = if *toggle { (f.make_toggle)(0, 0.0, y, 0, &mut name, *id, 2, 0) } else { (f.make_action)(0, 0.0, y, 0, &mut name, *id) };
        if btn >= PTR_MIN {
            (f.add_child)(s, ctx, btn);
            tooltip(f, btn, ctx, title, desc, true);
            action_btns[j] = btn;
        }
    }
    ((s + 0x14) as *mut f32).write_unaligned(h);
    if bg >= PTR_MIN {
        (f.rounded_rect_set_size)(bg, ctx, pw, h + 0.05);
    }
    if chosen != 0 {
        (f.group_select)(s + 0x90, 0, chosen + 0x100);
    }
    (s, action_btns)
}

pub fn selected(btn: usize) -> bool {
    btn >= PTR_MIN && readable(btn + BTN_SELECTED, 1) && unsafe { *((btn + BTN_SELECTED) as *const u8) } != 0
}


/// Grey a button out the way the game does: `ui_point_button_set_enabled` (vtable +0x128, RVA
/// 0x24ac40 on c38c03ff756b). A no-op when the enabled byte (+0xA0) already matches.
pub unsafe fn set_enabled(btn: usize, ctx: usize, enabled: bool) {
    if btn < PTR_MIN || ctx < PTR_MIN || !readable(btn, 0xA8) {
        return;
    }
    let vt = read_usize_at(btn);
    if vt < PTR_MIN || !readable(vt + 0x128, 8) {
        return;
    }
    let f: unsafe extern "system" fn(usize, usize, u8) = std::mem::transmute(read_usize_at(vt + 0x128));
    f(btn, ctx, enabled as u8);
}


/// Sets a toggle's lit state (knowledge/editor-ui.md: the byte alone drives the draw).
pub unsafe fn set_lit(btn: usize, on: bool) {
    if btn >= PTR_MIN && readable(btn + BTN_SELECTED, 1) {
        *((btn + BTN_SELECTED) as *mut u8) = on as u8;
    }
}

/// Appends a tool-mode toggle to the toolbar's tool shelf, laid out as the game lays out its own.
pub unsafe fn add_tool_toggle(f: &Fns, toolbar: usize, shelf: usize, ctx: usize, icon: &str, id: i32, title: &str, desc: &str) -> usize {
    let mut name = gstr(f, icon);
    let btn = (f.make_toggle)(0, 0.0, 0.0, 0, &mut name, id, 0, toolbar + TOOLBAR_GROUP);
    if btn < PTR_MIN {
        log_line("toolbar: button constructor returned null");
        return 0;
    }
    (f.add_child)(shelf, ctx, btn);
    // Next x on the shelf cursor, then grow the shelf.
    let f32_at = |a: usize| (a as *const f32).read_unaligned();
    let set = |a: usize, v: f32| (a as *mut f32).write_unaligned(v);
    let x = f32_at(shelf + 0x58);
    set(btn + 8, x);
    set(btn + 0xC, 0.0);
    let end = x + f32_at(btn + 0x10);
    set(shelf + 0x58, end);
    set(shelf + 0x10, end);
    set(shelf + 0x14, f32_at(shelf + 0x14).max(f32_at(btn + 0x14)));
    tooltip(f, btn, ctx, title, desc, false);
    log_line(&format!("toolbar: {title} button added ({btn:#x}, toolbar {toolbar:#x})"));
    btn
}

/// A mode toggle was clicked: on, the editor leaves its tool for the inert mode (toggles untouched);
/// off, back to the default tool.
pub unsafe fn mode_clicked(editor: usize, btn: usize) {
    let Some(f) = fns() else { return };
    if selected(btn) {
        (f.set_tool_mode)(editor, MODE_IDLE, 0);
    } else if readable(editor + OFF_TOOL_MODE, 4) && read_u32_at(editor + OFF_TOOL_MODE) == MODE_IDLE as u32 {
        (f.set_tool_mode)(editor, 0, 0);
    }
}

/// The editor is in the inert mode a plugin mode puts it in.
pub unsafe fn idle(editor: usize) -> bool {
    readable(editor + OFF_TOOL_MODE, 4) && read_u32_at(editor + OFF_TOOL_MODE) == MODE_IDLE as u32
}

/// The mouse over the 3D view (no UI element under it): normalised view position (x right, y down,
/// -1..1).
pub unsafe fn pointer_over_view(editor: usize) -> Option<[f32; 2]> {
    let hovered = editor + OFF_UI_CTX + 0x188; // the UI element under the mouse
    if !readable(hovered, 8) || read_usize_at(hovered) != 0 || !readable(editor + OFF_MOUSE_X, 8) {
        return None;
    }
    let at = ((editor + OFF_MOUSE_X) as *const [f32; 2]).read_unaligned();
    (at[0].is_finite() && at[1].is_finite()).then_some(at)
}

// ---------------------------------------------------------------------------------------------
// Hooks
// ---------------------------------------------------------------------------------------------

/// What a plugin does with the toolbar.
pub struct Plugin {
    /// The toolbar was built: add the plugin's toggles to `shelf` (the tool shelf).
    pub append: unsafe fn(f: &Fns, toolbar: usize, shelf: usize, ctx: usize),
    /// A button id was sent (any id: each plugin keeps to its own).
    pub event: unsafe fn(editor: usize, id: i32),
    /// A left press in the 3D view that no UI element took: normalised view position.
    pub click: unsafe fn(editor: usize, at: [f32; 2]),
    /// Every editor tick, with the top toolbar and its interface (None while there is none).
    pub tick: unsafe fn(f: &Fns, editor: usize, bar: Option<(usize, usize)>),
}

static PLUGIN: OnceLock<Plugin> = OnceLock::new();
static APPENDED_TOOLBAR: AtomicUsize = AtomicUsize::new(0);

/// Adds the separator and the plugin's toggles. Game thread.
unsafe fn append(toolbar: usize, ctx: usize) {
    let (Some(f), Some(p)) = (fns(), PLUGIN.get()) else { return };
    let shelf = read_usize_at(toolbar + TOOLBAR_SHELF);
    if shelf < PTR_MIN || !readable(shelf, 0x60) {
        log_line("toolbar: tool shelf not found; no toolbar buttons");
        return;
    }
    (f.separator)(shelf, ctx, 0.05);
    (p.append)(f, toolbar, shelf, ctx);
    APPENDED_TOOLBAR.store(toolbar, Ordering::SeqCst);
}

type ToolbarInit = unsafe extern "system" fn(usize, usize);
static ORIGINAL_INIT: OnceLock<ToolbarInit> = OnceLock::new();

unsafe extern "system" fn toolbar_init_detour(toolbar: usize, ctx: usize) {
    let Some(orig) = ORIGINAL_INIT.get() else {
        return;
    };
    orig(toolbar, ctx);
    let _ = std::panic::catch_unwind(|| append(toolbar, ctx));
}

type EditorEvent = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize) -> u8;
static ORIGINAL_EVENT: OnceLock<EditorEvent> = OnceLock::new();

unsafe extern "system" fn editor_event_detour(editor: usize, a2: usize, a3: usize, a4: usize, a5: usize, id: usize) -> u8 {
    let Some(orig) = ORIGINAL_EVENT.get() else {
        return 0;
    };
    let r = orig(editor, a2, a3, a4, a5, id);
    if let Some(p) = PLUGIN.get() {
        let _ = std::panic::catch_unwind(|| (p.event)(editor, id as i32));
    }
    r
}

type MouseButton = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize, usize, usize) -> u8;
static ORIGINAL_MOUSE: OnceLock<MouseButton> = OnceLock::new();

#[allow(clippy::too_many_arguments)]
unsafe extern "system" fn mouse_button_detour(editor: usize, a2: usize, a3: usize, a4: usize, a5: usize, a6: usize, button: usize, action: usize) -> u8 {
    let Some(orig) = ORIGINAL_MOUSE.get() else {
        return 0;
    };
    let r = orig(editor, a2, a3, a4, a5, a6, button, action);
    // The return is not a "UI took it" bool (tool paths leave junk in al: 11 was seen). The UI
    // root's pressed element (root +0x1E0 = editor+0x5D8) is set by a press a UI element takes and
    // cleared by one none does.
    let ui_took = !readable(editor + OFF_UI_PRESSED, 8) || read_usize_at(editor + OFF_UI_PRESSED) != 0;
    if !ui_took && button as u32 == 0 && action as u32 == 1 && readable(editor + OFF_MOUSE_X, 8) {
        let at = ((editor + OFF_MOUSE_X) as *const [f32; 2]).read_unaligned();
        if let (true, Some(p)) = (at[0].is_finite() && at[1].is_finite(), PLUGIN.get()) {
            let _ = std::panic::catch_unwind(|| (p.click)(editor, at));
        }
    }
    r
}

/// Editor tick (game thread): appends to a toolbar built before the plugin loaded, then the
/// plugin's own tick.
pub unsafe fn on_tick(editor: usize) {
    let (Some(f), Some(p)) = (fns(), PLUGIN.get()) else { return };
    let bar = (|| {
        if !readable(editor + OFF_INTERFACE, 8) {
            return None;
        }
        let iface = read_usize_at(editor + OFF_INTERFACE);
        if iface < PTR_MIN || !readable(iface + OFF_TOP_TOOLBAR, 8) {
            return None;
        }
        let toolbar = read_usize_at(iface + OFF_TOP_TOOLBAR);
        let vt = MODULE_BASE.load(Ordering::Relaxed) + RVA_TOOLBAR_VTABLE;
        (toolbar >= PTR_MIN && readable(toolbar, 0x138) && read_usize_at(toolbar) == vt).then_some((toolbar, iface))
    })();
    if let Some((toolbar, _)) = bar {
        if APPENDED_TOOLBAR.load(Ordering::SeqCst) != toolbar {
            append(toolbar, editor + OFF_UI_CTX);
        }
    }
    (p.tick)(f, editor, bar);
}

/// The toolbar init, editor event and mouse button hooks: all three or none (a button that does
/// nothing, or a mode nobody leaves, is worse than none). Call under `crate::hook::Exclusive`.
pub unsafe fn install(plugin: Plugin) -> Result<(), String> {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    let targets: [(&str, usize, &[u8], *mut std::ffi::c_void); 3] = [
        ("toolbar init", RVA_TOOLBAR_INIT, &TOOLBAR_INIT_PROLOGUE, toolbar_init_detour as *mut std::ffi::c_void),
        ("editor event", RVA_EDITOR_EVENT, &EDITOR_EVENT_PROLOGUE, editor_event_detour as *mut std::ffi::c_void),
        ("editor mouse button", RVA_EDITOR_MOUSE_BUTTON, &EDITOR_MOUSE_BUTTON_PROLOGUE, mouse_button_detour as *mut std::ffi::c_void),
    ];
    for (name, rva, pro, _) in &targets {
        if !crate::hook::verify(base + rva, pro) {
            return Err(format!("{name} prologue mismatch at {:#x}; no toolbar buttons", base + rva));
        }
    }
    let mut origs = [std::ptr::null_mut(); 3];
    for (i, (name, rva, _, detour)) in targets.iter().enumerate() {
        origs[i] = minhook::MinHook::create_hook((base + rva) as *mut std::ffi::c_void, *detour).map_err(|e| format!("create {name} hook: {e:?}; no toolbar buttons"))?;
    }
    let _ = ORIGINAL_INIT.set(std::mem::transmute::<*mut std::ffi::c_void, ToolbarInit>(origs[0]));
    let _ = ORIGINAL_EVENT.set(std::mem::transmute::<*mut std::ffi::c_void, EditorEvent>(origs[1]));
    let _ = ORIGINAL_MOUSE.set(std::mem::transmute::<*mut std::ffi::c_void, MouseButton>(origs[2]));
    for (name, rva, _, _) in &targets {
        if let Err(e) = minhook::MinHook::enable_hook((base + rva) as *mut std::ffi::c_void) {
            log_line(&format!("enable {name} hook: {e:?}"));
        }
    }
    let _ = PLUGIN.set(plugin);
    log_line("toolbar hooks installed");
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Slider rows in a part's configuration panel (knowledge/editor-ui.md "Component configuration
// panel"). The panel shows one part at a time and its slider writes `*target` every frame, so one
// plugin-owned cell serves it; the editor tick copies a moved value into the part's marks.
// ---------------------------------------------------------------------------------------------

/// A part panel builder (`prop panel`, `joint panel`): (component, ctx, panel).
pub type PartPanel = unsafe extern "system" fn(usize, usize, usize);
const RVA_ADD_SLIDER_ROW: usize = 0x008B_8230;
const ADD_SLIDER_ROW_PROLOGUE: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x4C, 0x89, 0x4C, 0x24, 0x20, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x70, 0x4D, 0x8B, 0xF1];
const OFF_SELECTED_COMP: usize = 0x1550;
/// `(panel, ctx, label, fmt, min, max, step, float* target, scale)`, floats as raw bits.
type AddSliderRow = unsafe extern "system" fn(usize, usize, *mut GStr, *mut GStr, u32, u32, u32, *mut f32, u32);

/// A plugin-owned slider row: the panel shows one part at a time and its slider writes `cell`
/// every frame, so one cell per row kind serves it; `comp` is the part it was added for and
/// `seen` the value last synced into the part's marks.
pub struct SliderRow {
    cell: AtomicU32,
    comp: AtomicUsize,
    seen: AtomicU32,
    label: OnceLock<(usize, u32, u32)>,
}

impl SliderRow {
    pub const fn new() -> SliderRow {
        SliderRow { cell: AtomicU32::new(0), comp: AtomicUsize::new(0), seen: AtomicU32::new(0), label: OnceLock::new() }
    }

    /// Adds the row to `panel` for `comp`, showing `value` (`min..max` in `step`s, shown as `fmt`).
    pub unsafe fn add(&self, comp: usize, ctx: usize, panel: usize, label: &str, fmt: &str, value: f32, range: (f32, f32, f32)) {
        let base = MODULE_BASE.load(Ordering::Relaxed);
        let Some(f) = fns() else { return };
        if !bytes_match(base + RVA_ADD_SLIDER_ROW, &ADD_SLIDER_ROW_PROLOGUE) {
            return;
        }
        self.cell.store(value.to_bits(), Ordering::SeqCst);
        self.seen.store(value.to_bits(), Ordering::SeqCst);
        self.comp.store(comp, Ordering::SeqCst);
        // The callee copies the label (and leaves it to us) and frees the format.
        let &(data, len, cap) = self.label.get_or_init(|| {
            let g = gstr(f, label);
            (g.data as usize, g.len, g.cap)
        });
        if data == 0 {
            return;
        }
        let mut label = GStr { data: data as *mut u8, len, cap };
        let mut fmt = gstr(f, fmt);
        let add: AddSliderRow = std::mem::transmute(base + RVA_ADD_SLIDER_ROW);
        let (lo, hi, step) = range;
        add(panel, ctx, &mut label, &mut fmt, lo.to_bits(), hi.to_bits(), step.to_bits(), self.cell.as_ptr() as *mut f32, 1.0f32.to_bits());
    }

    /// The part and its new value when the slider moved while that part is still selected.
    pub unsafe fn moved(&self, editor: usize) -> Option<(usize, f32)> {
        let comp = self.comp.load(Ordering::Relaxed);
        let bits = self.cell.load(Ordering::Relaxed);
        if comp == 0 || bits == self.seen.load(Ordering::Relaxed) || !readable(editor + OFF_SELECTED_COMP, 8) {
            return None;
        }
        self.seen.store(bits, Ordering::Relaxed);
        (read_usize_at(editor + OFF_SELECTED_COMP) == comp).then_some((comp, f32::from_bits(bits)))
    }
}


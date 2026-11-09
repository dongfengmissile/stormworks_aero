//! Game hooks that keep `marks` attached to their components (build c38c03ff756b,
//! knowledge/vehicle-model.md "Component XML serialize / clone flows"):
//! - XML: `aero` attribute on each `<c>` (written in `body_serialize_components`, read when
//!   `body_create_component_by_class` creates the part right after `component_def_ref_serialize`).
//! - Binary copies (spawning, the editor's restore): there is no room in the stream, so the marks a
//!   body had when written are kept by a signature of its parts and put back on an identical body read.
//! - Copy/paste and toolbox parts: `component_clone_state(dst, src)`. Undo: by undo record.
//! - Every new component starts unmarked (factory / create-by-class): a freed pointer's marks never
//!   reach the part that reuses it.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, VecDeque};
use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{LazyLock, Mutex, OnceLock};

use minhook::MinHook;

use crate::editor::{ptr_ok, ring};
use crate::logging::log_line;
use crate::marks::{self, Mark};
use crate::memory::{bytes_match, read_u32_at, read_usize_at, readable};
use crate::offsets::*;
use crate::state::MODULE_BASE;

const RVA_BODY_SER_XML: usize = 0x4B_AB50;
const RVA_DEF_REF_SER: usize = 0x4B_B180;
const RVA_CREATE_BY_CLASS: usize = 0x4B_B890;
const RVA_BODY_SER_BIN: usize = 0x4B_B330;
const RVA_FACTORY: usize = 0x46_05A0;
const RVA_CLONE_STATE: usize = 0x32_AE20;
const RVA_SER_STRING_FIELD: usize = 0x8F_70A0;
const RVA_ATTR_FIND: usize = 0x8D_A620;
const RVA_UNDO_FROM_COMP: usize = 0x46_02E0;
const RVA_UNDO_COPY: usize = 0x46_00E0;
const RVA_UNDO_APPLY: usize = 0x80_D520;

const P_BODY_SER_XML: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x20, 0x4C, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x56, 0x57, 0x41];
const P_DEF_REF_SER: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x40, 0x49, 0x8B, 0xE9, 0x49];
const P_CREATE_BY_CLASS: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC, 0x20, 0x83, 0xFA, 0x42, 0x0F, 0x87, 0x75];
const P_BODY_SER_BIN: [u8; 24] =
    [0x4C, 0x89, 0x4C, 0x24, 0x20, 0x48, 0x89, 0x54, 0x24, 0x10, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41];
const P_FACTORY: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC, 0x20, 0x45, 0x8B, 0xF8, 0x4C, 0x8B];
const P_CLONE_STATE: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x6C, 0x24, 0x18, 0x56, 0x57, 0x41, 0x56, 0x48, 0x83, 0xEC, 0x30, 0xF2, 0x0F, 0x10, 0x41, 0x18, 0x49];
const P_SER_STRING_FIELD: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x6C, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x41, 0x56, 0x48, 0x83, 0xEC, 0x30, 0x8B, 0x41, 0x20];
const P_ATTR_FIND: [u8; 24] =
    [0x48, 0x83, 0xEC, 0x08, 0x4C, 0x8B, 0xDA, 0x48, 0x85, 0xD2, 0x0F, 0x84, 0x88, 0x00, 0x00, 0x00, 0x80, 0x3A, 0x00, 0x4C, 0x8B, 0xD2, 0x74, 0x11];
// undo_record_from_component and undo_record_copy share these bytes.
const P_UNDO_RECORD: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x18, 0x48, 0x89, 0x4C, 0x24, 0x08, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC];
const P_UNDO_APPLY: [u8; 24] =
    [0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x48, 0x89, 0x7C, 0x24, 0x18, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57];

const OFF_SER_MODE: usize = 0x20; // 0 XML, 1 binary
const OFF_SER_WRITE: usize = 0x24; // 1 write, 0 read
const OFF_DEF_NAME_LEN: usize = 0x290;
const OFF_DEF_MIRROR_BYTE: usize = 0x40;
const ATTR: &[u8] = b"aero\0";

#[repr(C)]
struct GStr {
    data: *const u8,
    len: u32,
    cap: u32,
}

type Fn2 = unsafe extern "system" fn(usize, usize) -> usize;
type Fn3 = unsafe extern "system" fn(usize, usize, usize) -> usize;
type Fn4 = unsafe extern "system" fn(usize, usize, usize, usize) -> usize;
type Fn5 = unsafe extern "system" fn(usize, usize, usize, usize, usize) -> usize;
type Fn6 = unsafe extern "system" fn(usize, usize, usize, usize, usize, usize) -> usize;
type SerStringField = unsafe extern "system" fn(usize, usize, *const u8, *const GStr, *const GStr) -> usize;
type AttrFind = unsafe extern "system" fn(usize, *const u8) -> usize;

static O_BODY_SER_XML: OnceLock<Fn5> = OnceLock::new();
static O_DEF_REF_SER: OnceLock<Fn5> = OnceLock::new();
static O_CREATE_BY_CLASS: OnceLock<Fn2> = OnceLock::new();
static O_BODY_SER_BIN: OnceLock<Fn5> = OnceLock::new();
static O_FACTORY: OnceLock<Fn3> = OnceLock::new();
static O_CLONE_STATE: OnceLock<Fn4> = OnceLock::new();
static O_UNDO_FROM_COMP: OnceLock<Fn3> = OnceLock::new();
static O_UNDO_COPY: OnceLock<Fn2> = OnceLock::new();
static O_UNDO_APPLY: OnceLock<Fn6> = OnceLock::new();
static SER_STRING_FIELD: OnceLock<SerStringField> = OnceLock::new();
static ATTR_FIND: OnceLock<AttrFind> = OnceLock::new();

thread_local! {
    /// XML write of one body: its components in set order (the `<c>` order) and the next index.
    static XML_WRITE: RefCell<Option<(Vec<usize>, usize)>> = const { RefCell::new(None) };
    /// XML read: the `<c>` node whose part `body_create_component_by_class` creates next.
    static XML_NODE: Cell<usize> = const { Cell::new(0) };
    /// Inside an undo apply: (record, parts the factory made).
    static UNDO: RefCell<Option<(usize, Vec<usize>)>> = const { RefCell::new(None) };
}

/// Marks of bodies written to a binary blob, by part signature (newest last).
static BINARY: LazyLock<Mutex<VecDeque<(u64, Vec<Mark>)>>> = LazyLock::new(Default::default);
const BINARY_KEEP: usize = 256;
/// Marks by undo record.
static RECORDS: LazyLock<Mutex<HashMap<usize, Mark>>> = LazyLock::new(Default::default);
pub(crate) static HOOKED: AtomicBool = AtomicBool::new(false);

unsafe fn ser_is(ser: usize, mode: u8, write: u8) -> bool {
    ptr_ok(ser, OFF_SER_WRITE + 1) && *((ser + OFF_SER_MODE) as *const u8) == mode && *((ser + OFF_SER_WRITE) as *const u8) == write
}

unsafe fn body_parts(body: usize) -> Vec<usize> {
    ring(body + OFF_BODY_COMPONENTS, 1 << 20).unwrap_or_default()
}

/// Order-sensitive hash of the parts (def*, position, transform); None if one is unreadable.
unsafe fn signature(parts: &[usize]) -> Option<u64> {
    let mut h: u64 = 0xcbf29ce484222325;
    let mut mix = |v: u64| h = (h ^ v).wrapping_mul(0x100000001b3);
    mix(parts.len() as u64);
    for &c in parts {
        if !ptr_ok(c, OFF_COMP_DEF + 8) {
            return None;
        }
        mix(read_usize_at(c + OFF_COMP_DEF) as u64);
        (0..12).for_each(|k| mix(read_u32_at(c + if k < 3 { OFF_COMP_POS + 4 * k } else { OFF_COMP_ROT + 4 * (k - 3) }) as u64));
    }
    Some(h)
}

/// Evidence for the log: the first few of each event.
fn note(what: &'static str) {
    static SEEN: Mutex<Vec<(&str, u32)>> = Mutex::new(Vec::new());
    let mut s = SEEN.lock().unwrap_or_else(|p| p.into_inner());
    let n = match s.iter_mut().find(|(w, _)| *w == what) {
        Some(e) => {
            e.1 += 1;
            e.1
        }
        None => {
            s.push((what, 1));
            1
        }
    };
    if n <= 3 || n.is_power_of_two() {
        log_line(&format!("marks: {what} (#{n})"));
    }
}

fn once(flag: &AtomicBool, msg: impl FnOnce() -> String) {
    if !flag.swap(true, Ordering::Relaxed) {
        log_line(&msg());
    }
}

unsafe extern "system" fn body_ser_xml_detour(body: usize, ctx: usize, ser: usize, node: usize, compact: usize) -> usize {
    let Some(o) = O_BODY_SER_XML.get() else {
        return 0;
    };
    let write = ser_is(ser, 0, 1);
    XML_WRITE.with(|w| *w.borrow_mut() = write.then(|| (body_parts(body), 0)));
    XML_NODE.with(|n| n.set(0));
    let r = o(body, ctx, ser, node, compact);
    XML_WRITE.with(|w| *w.borrow_mut() = None);
    XML_NODE.with(|n| n.set(0));
    r
}

/// The `<c>` written here belongs to the part whose def name and mirror `src` holds.
unsafe fn same_def(src: usize, comp: usize) -> bool {
    if !ptr_ok(src, 0x18) || !ptr_ok(comp, OFF_COMP_DEF + 8) {
        return false;
    }
    let def = read_usize_at(comp + OFF_COMP_DEF);
    if !ptr_ok(def, OFF_DEF_NAME_LEN + 4) {
        return false;
    }
    let (a, alen) = (read_usize_at(src), read_u32_at(src + 8) as usize);
    let (b, blen) = (read_usize_at(def + OFF_DEF_NAME), read_u32_at(def + OFF_DEF_NAME_LEN) as usize);
    alen == blen
        && alen < 512
        && readable(a, alen)
        && readable(b, blen)
        && std::slice::from_raw_parts(a as *const u8, alen) == std::slice::from_raw_parts(b as *const u8, blen)
        && *((src + 0x10) as *const u8) == *((def + OFF_DEF_MIRROR_BYTE) as *const u8)
}

unsafe extern "system" fn def_ref_ser_detour(src: usize, a2: usize, ser: usize, node: usize, compact: usize) -> usize {
    let Some(o) = O_DEF_REF_SER.get() else {
        return 0;
    };
    let r = o(src, a2, ser, node, compact);
    let write = ser_write_flag(ser);
    if node == 0 || !ser_is(ser, 0, write) {
        return r;
    }
    if write == 0 {
        XML_NODE.with(|n| n.set(node));
        return r;
    }
    let comp = XML_WRITE.with(|w| {
        let mut w = w.borrow_mut();
        let (parts, i) = w.as_mut()?;
        let c = parts.get(*i).copied();
        *i += 1;
        if c.is_none_or(|c| !same_def(src, c)) {
            static WARNED: AtomicBool = AtomicBool::new(false);
            once(&WARNED, || "marks: XML write order does not match the body's parts; marks not written for that body".into());
            *w = None;
            return None;
        }
        c
    });
    let (Some(comp), Some(f)) = (comp, SER_STRING_FIELD.get()) else {
        return r;
    };
    if let Some(m) = marks::get(comp) {
        let text = m.encode() + "\0";
        let val = GStr { data: text.as_ptr(), len: text.len() as u32 - 1, cap: text.len() as u32 };
        let none = GStr { data: std::ptr::null(), len: 0, cap: 0 };
        f(ser, node, ATTR.as_ptr(), &val, &none);
        note("written to XML");
    }
    r
}

unsafe fn ser_write_flag(ser: usize) -> u8 {
    if ptr_ok(ser, OFF_SER_WRITE + 1) {
        *((ser + OFF_SER_WRITE) as *const u8)
    } else {
        0xFF
    }
}

/// The `aero` value on an XML node, if any.
unsafe fn node_attr(node: usize) -> Option<String> {
    let attr = (ATTR_FIND.get()?)(node, ATTR.as_ptr());
    if attr == 0 || !ptr_ok(attr, 0x10) {
        return None;
    }
    let p = read_usize_at(attr + 8);
    let mut bytes = Vec::new();
    while bytes.len() < 0x2000 && readable(p + bytes.len(), 1) {
        let b = *((p + bytes.len()) as *const u8);
        if b == 0 {
            break;
        }
        bytes.push(b);
    }
    String::from_utf8(bytes).ok()
}

unsafe extern "system" fn create_by_class_detour(body: usize, class: usize) -> usize {
    let Some(o) = O_CREATE_BY_CLASS.get() else {
        return 0;
    };
    let comp = o(body, class);
    if comp != 0 {
        let node = XML_NODE.with(|n| n.replace(0));
        let mark = if node != 0 { node_attr(node).map(|s| Mark::decode(&s)) } else { None };
        if mark.is_some() {
            note("read from XML");
        }
        marks::set(comp, mark, true);
    }
    comp
}

unsafe extern "system" fn body_ser_bin_detour(body: usize, ctx: usize, ser: usize, node: usize, compact: usize) -> usize {
    let Some(o) = O_BODY_SER_BIN.get() else {
        return 0;
    };
    let write = ser_write_flag(ser);
    let before = if ptr_ok(body, OFF_BODY_COMPONENTS + 0x14) { crate::editor::body_comp_count(body) as usize } else { 0 };
    let r = o(body, ctx, ser, node, compact);
    let parts = body_parts(body);
    let mut bin = BINARY.lock().unwrap_or_else(|p| p.into_inner());
    if write == 1 {
        let Some(sig) = signature(&parts) else {
            return r;
        };
        bin.retain(|(s, _)| *s != sig);
        let m = marks::read(&parts);
        if m.iter().any(|m| !m.is_empty()) {
            if bin.len() >= BINARY_KEEP {
                bin.pop_front();
            }
            bin.push_back((sig, m));
        }
    } else if write == 0 && r & 0xFF != 0 && parts.len() > before {
        let new = &parts[before..];
        let Some(sig) = signature(new) else { return r };
        if let Some((_, m)) = bin.iter().rev().find(|(s, m)| *s == sig && m.len() == new.len()) {
            new.iter().zip(m).for_each(|(&c, m)| marks::set(c, Some(m.clone()), true));
            note("restored on a binary copy");
        }
    }
    r
}

unsafe extern "system" fn factory_detour(def: usize, placement: usize, uid: usize) -> usize {
    let Some(o) = O_FACTORY.get() else { return 0 };
    let comp = o(def, placement, uid);
    if comp != 0 {
        marks::set(comp, None, true);
        UNDO.with(|u| {
            if let Some((_, made)) = u.borrow_mut().as_mut() {
                made.push(comp);
            }
        });
    }
    comp
}

unsafe extern "system" fn clone_state_detour(dst: usize, src: usize, ser: usize, ctx: usize) -> usize {
    let Some(o) = O_CLONE_STATE.get() else {
        return 0;
    };
    let r = o(dst, src, ser, ctx);
    marks::copy(dst, src);
    r
}

fn records() -> std::sync::MutexGuard<'static, HashMap<usize, Mark>> {
    RECORDS.lock().unwrap_or_else(|p| p.into_inner())
}

unsafe extern "system" fn undo_from_comp_detour(rec: usize, src: usize, ser: usize) -> usize {
    let Some(o) = O_UNDO_FROM_COMP.get() else {
        return 0;
    };
    let r = o(rec, src, ser);
    match marks::get(src) {
        Some(m) => records().insert(rec, m),
        None => records().remove(&rec),
    };
    r
}

unsafe extern "system" fn undo_copy_detour(dst: usize, src: usize) -> usize {
    let Some(o) = O_UNDO_COPY.get() else { return 0 };
    let r = o(dst, src);
    let mut t = records();
    match t.get(&src).cloned() {
        Some(m) => t.insert(dst, m),
        None => t.remove(&dst),
    };
    r
}

unsafe extern "system" fn undo_apply_detour(a1: usize, a2: usize, a3: usize, a4: usize, rec: usize, editor: usize) -> usize {
    let Some(o) = O_UNDO_APPLY.get() else {
        return 0;
    };
    let outer = UNDO.with(|u| u.replace(Some((rec, Vec::new()))));
    let r = o(a1, a2, a3, a4, rec, editor);
    let made = UNDO.with(|u| u.replace(outer)).map(|(_, m)| m).unwrap_or_default();
    let mark = records().get(&rec).cloned();
    if mark.is_some() {
        made.iter().for_each(|&c| marks::set(c, mark.clone(), true));
    }
    r
}

unsafe fn hook(base: usize, name: &str, rva: usize, prologue: &[u8], detour: *mut c_void) -> Result<*mut c_void, String> {
    if !bytes_match(base + rva, prologue) {
        return Err(format!("{name} prologue mismatch at {:#x}", base + rva));
    }
    MinHook::create_hook((base + rva) as *mut c_void, detour).map_err(|e| format!("create {name} hook: {e:?}"))
}

unsafe fn enable(base: usize, rvas: &[usize]) {
    for &rva in rvas {
        if let Err(e) = MinHook::enable_hook((base + rva) as *mut c_void) {
            log_line(&format!("enable mark hook {:#x}: {e:?}", base + rva));
        }
    }
}

/// All of the save/load/copy hooks or none: writing without reading (or the reverse) would drop
/// marks from vehicle files. Undo is a separate group (without it, undoing a delete loses marks).
pub(crate) unsafe fn install() {
    let base = MODULE_BASE.load(Ordering::SeqCst);
    for (name, rva, p) in [("ser_string_field", RVA_SER_STRING_FIELD, &P_SER_STRING_FIELD), ("xml attr find", RVA_ATTR_FIND, &P_ATTR_FIND)] {
        if !bytes_match(base + rva, p) {
            log_line(&format!("{name} prologue mismatch at {:#x}; designations are not stored in vehicle files", base + rva));
            return;
        }
    }
    let core: [(&str, usize, &[u8], *mut c_void); 6] = [
        ("body_serialize_components", RVA_BODY_SER_XML, &P_BODY_SER_XML, body_ser_xml_detour as *mut c_void),
        ("component_def_ref_serialize", RVA_DEF_REF_SER, &P_DEF_REF_SER, def_ref_ser_detour as *mut c_void),
        ("body_create_component_by_class", RVA_CREATE_BY_CLASS, &P_CREATE_BY_CLASS, create_by_class_detour as *mut c_void),
        ("body_serialize_components_binary", RVA_BODY_SER_BIN, &P_BODY_SER_BIN, body_ser_bin_detour as *mut c_void),
        ("veh_component_factory", RVA_FACTORY, &P_FACTORY, factory_detour as *mut c_void),
        ("component_clone_state", RVA_CLONE_STATE, &P_CLONE_STATE, clone_state_detour as *mut c_void),
    ];
    let mut o = Vec::new();
    for (name, rva, p, d) in core {
        match hook(base, name, rva, p, d) {
            Ok(x) => o.push(x),
            Err(e) => {
                for (_, rva, _, _) in &core[..o.len()] {
                    let _ = MinHook::remove_hook((base + rva) as *mut c_void);
                }
                log_line(&format!("{e}; designations are not stored in vehicle files"));
                return;
            }
        }
    }
    let _ = SER_STRING_FIELD.set(std::mem::transmute::<usize, SerStringField>(base + RVA_SER_STRING_FIELD));
    let _ = ATTR_FIND.set(std::mem::transmute::<usize, AttrFind>(base + RVA_ATTR_FIND));
    let _ = O_BODY_SER_XML.set(std::mem::transmute::<*mut c_void, Fn5>(o[0]));
    let _ = O_DEF_REF_SER.set(std::mem::transmute::<*mut c_void, Fn5>(o[1]));
    let _ = O_CREATE_BY_CLASS.set(std::mem::transmute::<*mut c_void, Fn2>(o[2]));
    let _ = O_BODY_SER_BIN.set(std::mem::transmute::<*mut c_void, Fn5>(o[3]));
    let _ = O_FACTORY.set(std::mem::transmute::<*mut c_void, Fn3>(o[4]));
    let _ = O_CLONE_STATE.set(std::mem::transmute::<*mut c_void, Fn4>(o[5]));
    enable(base, &core.map(|c| c.1));
    HOOKED.store(true, Ordering::Relaxed);
    log_line("mark hooks installed (designations stored in vehicle files)");

    let undo: [(&str, usize, &[u8], *mut c_void); 3] = [
        ("undo_record_from_component", RVA_UNDO_FROM_COMP, &P_UNDO_RECORD, undo_from_comp_detour as *mut c_void),
        ("undo_record_copy", RVA_UNDO_COPY, &P_UNDO_RECORD, undo_copy_detour as *mut c_void),
        ("undo_record_apply_component", RVA_UNDO_APPLY, &P_UNDO_APPLY, undo_apply_detour as *mut c_void),
    ];
    let mut o = Vec::new();
    for (name, rva, p, d) in undo {
        match hook(base, name, rva, p, d) {
            Ok(x) => o.push(x),
            Err(e) => {
                for (_, rva, _, _) in &undo[..o.len()] {
                    let _ = MinHook::remove_hook((base + rva) as *mut c_void);
                }
                log_line(&format!("{e}; undoing a delete loses the part's designations"));
                return;
            }
        }
    }
    let _ = O_UNDO_FROM_COMP.set(std::mem::transmute::<*mut c_void, Fn3>(o[0]));
    let _ = O_UNDO_COPY.set(std::mem::transmute::<*mut c_void, Fn2>(o[1]));
    let _ = O_UNDO_APPLY.set(std::mem::transmute::<*mut c_void, Fn6>(o[2]));
    enable(base, &undo.map(|c| c.1));
}

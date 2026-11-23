//! Mechanisms mode (workbench toolbar, beside Aero wings): the vehicle's joints checked like a CAD
//! assembly (`mech`). Locked joints and, after Check, the parts each sub-grid stops against are
//! painted on the vehicle, and each joint gets a label (`draw_labels`). The pose shows eased in
//! (`pose::PoseAnim`). Game thread, a worker for the analysis, the overlay for the labels.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock, Mutex};

use crate::defs::Library;
use crate::logging::log_line;
use crate::mech::{Report, Status};
use crate::vehicle::Comp;

/// A click on Check not yet taken by the tick.
pub(crate) static CHECK_REQ: AtomicBool = AtomicBool::new(false);
/// The joint labels are shown (Labels toggle beside Check).
pub(crate) static LABELS_SHOWN: AtomicBool = AtomicBool::new(true);
/// The analysis the labels show while the mode is on.
static SHOWN: Mutex<Option<Arc<Built>>> = Mutex::new(None);
/// The sequence window is open (Sequence toggle beside Check).
pub(crate) static SEQ_OPEN: AtomicBool = AtomicBool::new(false);
/// The Fixed tool is on: a click fixes the sub-grid under the mouse (`fix_click`).
pub(crate) static FIXING: AtomicBool = AtomicBool::new(false);
/// Shift is held (overlay frame), for the click on the game thread.
pub(crate) static SHIFT: AtomicBool = AtomicBool::new(false);
/// A sequence frame is shown instead of the workbench pose.
static SEQ_POSED: AtomicBool = AtomicBool::new(false);
/// The game's label icons (`toolbar::LABEL_ICONS`) as overlay textures, by name.
pub(crate) static ICONS: Mutex<Vec<(&'static str, imgui::TextureId)>> = Mutex::new(Vec::new());

const RED: [u8; 3] = [230, 32, 32];
const ORANGE: [u8; 3] = [245, 150, 20];
const BLUE: [u8; 3] = [50, 110, 230];
const MAGENTA: [u8; 3] = [215, 60, 215];

struct Built {
    key: (u64, bool),
    comps: Vec<Comp>,
    /// Each part's voxel cells (vehicle grid), for the paint.
    cells: Vec<Vec<[i32; 3]>>,
    report: Report,
    labels: Vec<Label>,
    /// One per posing joint (`Report::tree`), with its parent half's component pointer (its
    /// marks hold the moves) and where it shows on the vehicle.
    lanes: Vec<crate::seqwin::Lane>,
    lane_ptr: Vec<usize>,
    lane_at: Vec<Option<crate::math::V3>>,
    /// `marks::GEN` when the marks were read.
    marks_gen: u64,
    /// Each part's name, and its component pointer (marks).
    names: Vec<String>,
    ptrs: Vec<usize>,
}


struct Label {
    /// Opens with the mouse over a part of this joint (`Report::all`), or over one of `parts`.
    joint: Option<usize>,
    parts: Vec<u32>,
    at: crate::math::V3,
    title: String,
    text: String,
    status: Status,
}

#[derive(Default)]
struct Mode {
    on: bool,
    lib: Option<Arc<Mutex<Library>>>,
    /// Check was pressed since the mode came on: results follow every change after it.
    checked: bool,
    /// Snapshot generation and `checked` of the build started last.
    started: Option<(u64, bool)>,
    building: Arc<AtomicBool>,
    done: Arc<Mutex<Option<Arc<Built>>>>,
    built: Option<Arc<Built>>,
    /// The build last painted with the Fixed tool on or off, and whether it painted anything.
    painted: Option<(((u64, bool), bool), bool)>,
}

static MODE: LazyLock<Mutex<Mode>> = LazyLock::new(Mutex::default);

/// Editor tick (game thread). `on`: the mode's button is selected and the editor idle.
pub(crate) fn on_tick(on: bool) {
    let mut m = MODE.lock().unwrap_or_else(|p| p.into_inner());
    if !on {
        if m.on {
            m.on = false;
            m.checked = false;
            m.started = None;
            if m.painted.take().is_some() {
                *crate::editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()) = None;
            }
            *SHOWN.lock().unwrap_or_else(|p| p.into_inner()) = None;
            SEQ_POSED.store(false, Ordering::Relaxed);
        }
        return;
    }
    if !m.on {
        m.on = true;
        if let Some(b) = m.built.clone() {
            publish_pose(&b);
        }
    }
    if CHECK_REQ.swap(false, Ordering::Relaxed) {
        m.checked = true;
    }
    let snap = crate::editor::LATEST.lock().unwrap_or_else(|p| p.into_inner()).clone();
    if let Some(snap) = snap {
        let key = (snap.gen, m.checked);
        if m.started != Some(key) && !m.building.load(Ordering::Acquire) {
            m.started = Some(key);
            start(&mut m, &snap, key);
        }
    }
    let done = m.done.lock().unwrap_or_else(|p| p.into_inner()).take();
    if let Some(b) = done {
        if !SEQ_POSED.load(Ordering::Relaxed) {
            publish_pose(&b);
        }
        m.built = Some(b);
    }
    let Some(b) = m.built.clone() else { return };
    // Leaving Aero wings clears the paint after us (overlay frame): paint again then.
    let cleared = m.painted.is_some_and(|p| p.1) && crate::editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()).is_none();
    let fixing = FIXING.load(Ordering::Relaxed);
    if m.painted.map(|p| p.0) != Some((b.key, fixing)) || cleared {
        m.painted = Some(((b.key, fixing), paint(&b, fixing)));
    }
    let mut shown = SHOWN.lock().unwrap_or_else(|p| p.into_inner());
    if shown.as_ref().is_none_or(|s| !Arc::ptr_eq(s, &b)) {
        *shown = Some(b);
    }
}

fn start(m: &mut Mode, snap: &crate::editor::Snapshot, key: (u64, bool)) {
    let comps = snap.comps.clone();
    let marks_gen = crate::marks::GEN.load(Ordering::Relaxed);
    let des = crate::marks::designations(&snap.comps, &snap.ptrs).unwrap_or_default();
    let lib = m
        .lib
        .get_or_insert_with(|| {
            let (game, appdata) = crate::plugin::PATHS.get().cloned().unwrap_or_default();
            Arc::new(Mutex::new(Library::new(&game, &appdata)))
        })
        .clone();
    let ptrs = snap.ptrs.clone();
    let all_ptrs = snap.ptrs.clone();
    let (building, done) = (m.building.clone(), m.done.clone());
    building.store(true, Ordering::Release);
    let _ = std::thread::Builder::new().name("mechanisms".into()).spawn(move || {
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _fault = crate::fault::enter(crate::fault::stage::ANALYSIS);
            let mut lib = lib.lock().unwrap_or_else(|p| p.into_inner());
            let t = std::time::Instant::now();
            let report = Report::new(&comps, &mut lib, &des, key.1);
            let defs: Vec<_> = comps.iter().map(|c| lib.get(&c.def)).collect();
            let names: Vec<String> = comps.iter().zip(&defs).map(|(c, d)| d.as_ref().map_or(c.def.clone(), |d| if d.name.is_empty() { c.def.clone() } else { d.name.clone() })).collect();
            let cells = comps.iter().zip(&defs).map(|(c, d)| d.as_ref().map_or(vec![c.pos], |d| d.voxels.iter().map(|v| c.cell(v.pos)).collect())).collect();
            // Free joints say nothing until Check; locked ones show at once.
            let labels = (report.all.iter().enumerate())
                .filter_map(|(a, j)| {
                    let (title, text, status) = report.describe(&comps, &|i| names[i].clone(), j.comp)?;
                    (status != Status::Free || report.sweeps.is_some()).then_some(())?;
                    Some(Label { joint: Some(a), parts: Vec::new(), at: report.joint_anchor(j)?, title, text, status })
                })
                // Rubbing: one label on each part rubbed on, after the joints so it opens over them.
                .chain(report.rub_notes().into_iter().filter_map(|(on, parts, text, status)| {
                    Some(Label { joint: None, parts, at: report.anchor(on as usize)?, title: names[on as usize].clone(), text, status })
                }))
                .collect();
            // Lane names say where the joint is ("left rear"), numbered only if still alike.
            let lane_at: Vec<_> = report.tree.iter().map(|j| report.joint_anchor(j)).collect();
            let (lo, hi) = report.extent();
            let place = |at: Option<crate::math::V3>| {
                let Some(p) = at else { return String::new() };
                let (mid, half) = ((lo + hi) * 0.5, ((hi.x - lo.x) * 0.5).max(0.5));
                let side = if p.y - mid.y > 0.3 { "right" } else if p.y - mid.y < -0.3 { "left" } else { "centre" };
                let along = if p.x - mid.x > half / 3.0 { " front" } else if p.x - mid.x < -half / 3.0 { " rear" } else { "" };
                format!(" ({side}{along})")
            };
            let base_names: Vec<String> = report.tree.iter().zip(&lane_at).map(|(j, at)| format!("{}{}", names[j.comp], place(*at))).collect();
            let lanes = (report.tree.iter().enumerate())
                .map(|(i, j)| {
                    let user = |g: f64| if j.slide { j.sign * g } else { (j.sign * g).to_degrees() };
                    // Where its loops let it go, so a move cannot ask for more.
                    let reach = crate::pose::reach(&report.tree, &report.links, report.n_bodies, &report.settings, i);
                    let (a, z) = (user(reach.0), user(reach.1));
                    let n = &base_names[i];
                    let k = base_names[..i].iter().filter(|o| *o == n).count() + 1;
                    crate::seqwin::Lane {
                        name: if base_names.iter().filter(|o| *o == n).count() > 1 { format!("{n} {k}") } else { n.clone() },
                        base: user(report.settings[i]),
                        travel: (a.min(z), a.max(z)),
                        slide: j.slide,
                        free: j.free,
                        mech: report.mech_of.get(j.child).copied().unwrap_or(usize::MAX),
                        dof: report.mech_of.get(j.child).and_then(|&m| report.dof.get(m)).copied().unwrap_or(0),
                        seqs: des.joint_seq.get(&comps[j.comp].pos).cloned().unwrap_or_default(),
                    }
                })
                .collect();
            let lane_ptr = report.tree.iter().map(|j| ptrs.get(j.comp).copied().unwrap_or(0)).collect();
            log_line(&format!("mechanisms: {} joints, {} hulls, checked {}, {} ms", report.all.len(), report.hulls.len(), key.1, t.elapsed().as_millis()));
            Built { key, comps, cells, report, labels, lanes, lane_ptr, lane_at, marks_gen, names, ptrs: all_ptrs }
        }));
        match r {
            Ok(b) => *done.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(b)),
            Err(_) => log_line("mechanisms: analysis panicked"),
        }
        building.store(false, Ordering::Release);
    });
}

fn publish_pose(b: &Built) {
    crate::posing::publish_pose_target(&b.report.tree, b.report.n_bodies, &b.report.settings);
}

/// Red: parts in contact or stopping a joint; magenta: parts sinking 4 cm or more into each other
/// inside a loop; orange: the
/// halves of a locked joint; with the Fixed tool on, blue: the fixed sub-grids. Whether anything is painted.
fn paint(b: &Built, fixing: bool) -> bool {
    let mut per_body: Vec<crate::paint::VoxelColours> = Vec::new();
    let fixed = (0..b.comps.len()).filter(|&c| fixing && b.report.fixed.get(b.comps[c].body as usize) == Some(&true)).map(|c| (c as u32, None));
    let status = b.report.paint().into_iter().map(|(c, s)| (c, Some(s)));
    for (c, status) in fixed.chain(status).collect::<Vec<_>>() {
        let col = crate::paint::rgba(match status {
            None => BLUE,
            Some(Status::Locked) => ORANGE,
            Some(Status::Rubs) => MAGENTA,
            Some(_) => RED,
        });
        let body = b.comps[c as usize].body as usize;
        if per_body.len() <= body {
            per_body.resize_with(body + 1, Default::default);
        }
        for &cell in &b.cells[c as usize] {
            per_body[body].insert(cell, col);
        }
    }
    if per_body.iter().all(|v| v.is_empty()) {
        *crate::editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()) = None;
        return false;
    }
    // The other parts' cells of a painted body, as colour 0 (`voxel::colour_runs`).
    for (c, cells) in b.comps.iter().zip(&b.cells) {
        if let Some(v) = per_body.get_mut(c.body as usize).filter(|v| !v.is_empty()) {
            for &cell in cells {
                v.entry(cell).or_insert(0);
            }
        }
    }
    let key = b.key.0 ^ (b.key.1 as u64) << 63 ^ (fixing as u64) << 62 ^ 0x6d65_6368;
    *crate::editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()) = Some((key, per_body.into_iter().map(Arc::new).collect(), Arc::default()));
    true
}

/// The sub-grid under the mouse with the Fixed tool on, if any: its body and the part.
fn fix_target(b: &Built, at: [f64; 2]) -> Option<(usize, usize)> {
    let clip = (*crate::editor::WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner()))?;
    let c = crate::mech::pick(&b.report.hulls, &b.report.poses, &clip, at)? as usize;
    Some((b.comps.get(c)?.body as usize, c))
}

/// The bodies joined to `body` through joints (its group), itself included.
fn group(r: &Report, body: usize) -> Vec<usize> {
    let mut g = vec![body];
    let mut k = 0;
    while let Some(&b) = g.get(k) {
        for l in &r.all {
            for (x, y) in [(l.parent, l.child), (l.child, l.parent)] {
                if x == b && !g.contains(&y) {
                    g.push(y);
                }
            }
        }
        k += 1;
    }
    g
}

/// A click with the Fixed tool (game thread, view position −1..1): the sub-grid under it becomes
/// its group's only fixed one; with Shift, it is fixed as well, or freed if another stays fixed.
pub(crate) fn fix_click(at: [f32; 2]) {
    let Some(b) = SHOWN.lock().unwrap_or_else(|p| p.into_inner()).clone() else { return };
    let Some((body, part)) = fix_target(&b, [at[0] as f64, at[1] as f64]) else { return };
    let r = &b.report;
    let g = group(r, body);
    if g.len() < 2 {
        return crate::sound::error(); // nothing jointed to it: it stays put anyway
    }
    let ptr = |c: usize| b.ptrs.get(c).copied().unwrap_or(0);
    let set = |c: usize, on: bool| {
        let mut m = crate::marks::get(ptr(c)).unwrap_or_default();
        if m.fixed != on && ptr(c) != 0 {
            m.fixed = on;
            crate::marks::set(ptr(c), Some(m), true);
        }
    };
    let on_body = |o: usize| (0..b.comps.len()).filter(|&c| b.comps[c].body as usize == o).collect::<Vec<_>>();
    let fixed = |o: usize| r.fixed.get(o) == Some(&true);
    if SHIFT.load(Ordering::Relaxed) {
        if fixed(body) && !g.iter().any(|&o| o != body && fixed(o)) {
            return crate::sound::error(); // one sub-grid of each group stays fixed
        }
        // The group's fixed sub-grids marked, so a default one stays fixed beside the new one.
        for &o in g.iter().filter(|&&o| o != body && fixed(o)) {
            let cs = on_body(o);
            if !cs.iter().any(|&c| crate::marks::get(ptr(c)).is_some_and(|m| m.fixed)) {
                cs.first().map(|&c| set(c, true));
            }
        }
        if fixed(body) {
            on_body(body).into_iter().for_each(|c| set(c, false));
        } else {
            set(part, true);
        }
    } else {
        for &o in &g {
            on_body(o).into_iter().for_each(|c| set(c, c == part));
        }
    }
}

/// With the Fixed tool on: what a click on the sub-grid under the mouse would do (overlay frame).
pub(crate) fn draw_fix_hint(ui: &imgui::Ui) {
    if !FIXING.load(Ordering::Relaxed) {
        return;
    }
    let Some(b) = SHOWN.lock().unwrap_or_else(|p| p.into_inner()).clone() else { return };
    let (size, mouse) = (ui.io().display_size, ui.io().mouse_pos);
    let ndc = [(mouse[0] / size[0] * 2.0 - 1.0) as f64, (mouse[1] / size[1] * 2.0 - 1.0) as f64];
    let Some((body, _)) = fix_target(&b, ndc) else { return };
    let text = if group(&b.report, body).len() < 2 {
        "No joints: stays put"
    } else if b.report.fixed.get(body) == Some(&true) {
        "Fixed: stays put\nShift-click to free it"
    } else {
        "Click: hold this still instead\nShift-click: hold it still as well"
    };
    let (pad, at) = (5.0, [mouse[0] + 18.0, mouse[1] + 18.0]);
    let t = ui.calc_text_size(text);
    let dl = ui.get_foreground_draw_list();
    dl.add_rect(at, [at[0] + t[0] + 2.0 * pad, at[1] + t[1] + 2.0 * pad], [0.08, 0.08, 0.09, 0.9]).filled(true).rounding(4.0).build();
    dl.add_text([at[0] + pad, at[1] + pad], [0.95, 0.95, 0.95, 1.0], text);
}

/// Each label: an icon at the joint (or the part rubbed on); with the mouse on the icon, or on a part
/// the label is about, the text beside it.
/// Drawn on top of the vehicle, so joints inside the fuselage show too. Overlay frame.
pub(crate) fn draw_labels(ui: &imgui::Ui) {
    // Labels sit where the analysis put the joints, not where a sequence moved them.
    if !LABELS_SHOWN.load(Ordering::Relaxed) || SEQ_POSED.load(Ordering::Relaxed) {
        return;
    }
    let icons = ICONS.lock().unwrap_or_else(|p| p.into_inner()).clone();
    let icon = |n: &str| icons.iter().find(|i| i.0 == n).map(|i| i.1);
    let Some(b) = SHOWN.lock().unwrap_or_else(|p| p.into_inner()).clone() else { return };
    let Some(clip) = *crate::editor::WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner()) else { return };
    let size = ui.io().display_size;
    let mouse = ui.io().mouse_pos;
    let ndc = [(mouse[0] / size[0] * 2.0 - 1.0) as f64, (mouse[1] / size[1] * 2.0 - 1.0) as f64];
    let pick = crate::mech::pick(&b.report.hulls, &b.report.poses, &clip, ndc);
    let under = pick.and_then(|c| b.report.joint_at(&b.comps, c as usize));
    let dl = ui.get_background_draw_list();
    let (pad, line) = (5.0, ui.text_line_height());
    // The label whose icon the mouse is on, else the last one about the part under it.
    let mut open: Option<(&Label, [f32; 2], bool)> = None;
    for l in &b.labels {
        let (x, y, depth) = crate::ui::project(&clip, size[0], size[1], l.at);
        if depth <= 0.0 {
            continue;
        }
        let col = colour(l.status);
        // The game's warning or tick icon on a disc of the status colour, like its round buttons.
        dl.add_circle([x, y], 11.0, col).filled(true).build();
        let (name, mark) = if l.status == Status::Free { ("action_tick", "v") } else { ("warning", "!") };
        match icon(name) {
            Some(tex) => dl.add_image(tex, [x - 8.0, y - 8.0], [x + 8.0, y + 8.0]).build(),
            None => dl.add_text([x - 3.0, y - line * 0.5], [1.0; 4], mark),
        }
        let on_icon = (mouse[0] - x).hypot(mouse[1] - y) <= 11.0;
        let on_part = (l.joint.is_some() && l.joint == under) || pick.is_some_and(|c| l.parts.contains(&c));
        if on_icon || (on_part && !open.is_some_and(|o| o.2)) {
            open = Some((l, [x + 14.0, y - line * 0.5 - pad], on_icon));
        }
    }
    // The open one last, so it sits on top.
    if let Some((l, at, _)) = open {
        let w = ui.calc_text_size(&l.title)[0].max(ui.calc_text_size(&l.text)[0]);
        let n = 1 + l.text.lines().count();
        let end = [at[0] + w + 2.0 * pad, at[1] + n as f32 * line + 2.0 * pad];
        let col = colour(l.status);
        dl.add_rect(at, end, [0.08, 0.08, 0.09, 0.95]).filled(true).rounding(4.0).build();
        dl.add_rect(at, end, col).rounding(4.0).build();
        dl.add_text([at[0] + pad, at[1] + pad], col, &l.title);
        dl.add_text([at[0] + pad, at[1] + pad + line], [0.95, 0.95, 0.95, 1.0], &l.text);
    }
}

// --- Sequences window (`seqwin`): this side saves the moves on the joints and poses the vehicle.

#[derive(Default)]
struct Seq {
    ui: crate::seqwin::State,
    key: Option<(u64, bool)>,
    /// Marks generation after our last write: older builds would undo it.
    committed: u64,
    last: Option<std::time::Instant>,
    settler: crate::mech::Settler,
    /// The sequence time the settler's pose is at (`follow`).
    shown_t: f64,
    /// Where the settler stopped, and which way the timeline was going then.
    held: Option<(f64, f64)>,
}

impl Seq {
    /// Back to the workbench pose: the settler starts again from the sequence's start.
    fn restart(&mut self) {
        self.settler.reset();
        (self.shown_t, self.held) = (0.0, None);
        self.ui.stuck.clear();
    }
}

static SEQ: LazyLock<Mutex<Seq>> = LazyLock::new(Mutex::default);

/// The Sequences window (overlay frame, Mechanisms mode).
pub(crate) fn draw_sequence(ui: &imgui::Ui) {
    let Some(b) = SHOWN.lock().unwrap_or_else(|p| p.into_inner()).clone() else { return };
    let mut s = SEQ.lock().unwrap_or_else(|p| p.into_inner());
    let mut open = SEQ_OPEN.load(Ordering::Relaxed);
    if s.key != Some(b.key) && !s.ui.dragging() && b.marks_gen >= s.committed {
        s.key = Some(b.key);
        s.restart();
        s.ui.load(&b.lanes);
    }
    let now = std::time::Instant::now();
    let dt = s.last.map_or(0.0, |l| (now - l).as_secs_f64().min(0.1));
    s.last = Some(now);
    s.ui.step(dt);
    let mut acts = Vec::new();
    if open {
        let size = ui.io().display_size;
        let w = crate::seqwin::WINDOW_W;
        ui.window("Sequences##mech")
            .position([size[0] * 0.5 - w * 0.5, size[1] * 0.62], imgui::Condition::FirstUseEver)
            .size_constraints([w, 0.0], [w, size[1]])
            .flags(imgui::WindowFlags::ALWAYS_AUTO_RESIZE)
            .opened(&mut open)
            .build(|| acts = crate::seqwin::contents(ui, &b.lanes, &mut s.ui));
    }
    SEQ_OPEN.store(open, Ordering::Relaxed);
    acts.dedup();
    for a in acts {
        match a {
            crate::seqwin::Act::Save(i) => {
                let ptr = b.lane_ptr[i];
                if ptr >= crate::memory::PTR_MIN {
                    let mut m = crate::marks::get(ptr).unwrap_or_default();
                    m.seq = s.ui.lane_seqs(i).into_iter().map(|(n, mv)| (n, mv.iter().map(|v| v.map(|x| x as f32)).collect())).collect();
                    crate::marks::set(ptr, Some(m), true);
                    s.committed = crate::marks::GEN.load(Ordering::Relaxed);
                }
                s.restart();
            }
            crate::seqwin::Act::Show(i) => highlight(ui, &b.lanes[i].name, b.lane_at[i]),
            crate::seqwin::Act::Error => crate::sound::error(),
        }
    }
    // Show the sequence while it plays or sits past its start; else the workbench pose.
    let posed = open && s.ui.posed();
    if s.settler.ignore != s.ui.no_collide {
        s.settler.ignore = s.ui.no_collide;
        s.restart();
    }
    if posed {
        let settings = follow(&b, &mut s);
        crate::posing::publish_pose_now(&b.report.tree, b.report.n_bodies, &settings);
    } else {
        s.restart();
        if SEQ_POSED.load(Ordering::Relaxed) {
            publish_pose(&b);
        }
    }
    SEQ_POSED.store(posed, Ordering::Relaxed);
}

/// Moves the settler along the timeline from where its pose is (`Seq::shown_t`) to the play
/// head, in steps of at most `H` and `STEPS` a frame, so a jump (a scrub, a start mid-sequence)
/// takes the sequence's own path, not a short cut. Where a part hits another or a mechanism can
/// go no further it stops there, pauses playing, marks the timeline and plays the error sound.
/// The joint settings to show.
fn follow(b: &Built, s: &mut Seq) -> Vec<f64> {
    const H: f64 = 1.0 / 30.0;
    const STEPS: usize = 8;
    let pose = |s: &Seq| s.settler.pose().map_or_else(|| b.report.settings.clone(), <[f64]>::to_vec);
    let t = s.ui.t;
    if let Some((at, dir)) = s.held {
        if (t - at) * dir >= -1e-9 {
            // At or past the stop: the pose stays there, and playing on stops again.
            if s.ui.playing {
                s.ui.halt(at);
                crate::sound::error();
            }
            return pose(s);
        }
        s.held = None;
        s.ui.stuck.clear();
    }
    let end = s.ui.end();
    let fwd = !s.ui.back;
    // A loop playing on round its end: on to the end first, then on from the start.
    let mut wrap = s.ui.looping && s.ui.playing && end > 0.0 && if fwd { t < s.shown_t } else { t > s.shown_t };
    let mut goal = if wrap { if fwd { end } else { 0.0 } } else { t };
    let mut t0 = s.shown_t;
    for _ in 0..STEPS {
        if (goal - t0).abs() < 1e-9 {
            if !wrap {
                break;
            }
            (t0, goal, wrap) = (if fwd { 0.0 } else { end }, t, false);
        }
        let t1 = t0 + (goal - t0).clamp(-H, H);
        let target: Vec<f64> = (b.report.tree.iter().enumerate()).map(|(i, j)| setting(j, b.lanes[i].slide, s.ui.value_at(&b.lanes, i, t1))).collect();
        s.settler.frame(&b.report, &target);
        if let Some(f) = s.settler.stop {
            let at = t0 + (t1 - t0) * f;
            let why = match s.settler.hit {
                Some((a, k)) => format!("{} hits {}", b.names[a as usize], b.names[k as usize]),
                None => "can't move further".to_string(),
            };
            (s.shown_t, s.held) = (at, Some((at, (t1 - t0).signum())));
            s.ui.stuck = s.settler.stuck.clone();
            s.ui.stop = Some((at, why));
            s.ui.halt(at);
            crate::sound::error();
            break;
        }
        (t0, s.shown_t) = (t1, t1);
    }
    pose(s)
}

/// The joint setting (`pose::body_poses`) for a value in the Workbench Angle's units.
fn setting(j: &crate::joints::JointLink, slide: bool, v: f64) -> f64 {
    (j.sign * if slide { v } else { v.to_radians() }).clamp(j.travel.0, j.travel.1)
}

/// Rings the joint of a lane the mouse is over, on the vehicle.
fn highlight(ui: &imgui::Ui, name: &str, at: Option<crate::math::V3>) {
    let (Some(at), Some(clip)) = (at, *crate::editor::WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner())) else { return };
    let size = ui.io().display_size;
    let (x, y, depth) = crate::ui::project(&clip, size[0], size[1], at);
    if depth > 0.0 {
        let dl = ui.get_foreground_draw_list();
        dl.add_circle([x, y], 16.0, [1.0, 0.85, 0.2, 1.0]).thickness(3.0).build();
        dl.add_text([x + 20.0, y - 8.0], [1.0, 0.85, 0.2, 1.0], name);
    }
}

fn colour(s: Status) -> [f32; 4] {
    match s {
        Status::Free => [0.35, 0.8, 0.35, 1.0],
        Status::Hits => [0.95, 0.25, 0.25, 1.0],
        Status::Locked => [0.96, 0.6, 0.1, 1.0],
        Status::Rubs => [0.84, 0.24, 0.84, 1.0],
    }
}

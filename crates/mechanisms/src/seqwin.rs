//! The Mechanisms Sequences window: named timelines, each moving the joints added to it (a door
//! opens, then the gear comes down), played in the workbench. Animation only. Needs only Dear
//! ImGui, `pose` and `marks`, so the UI mock (`tools/uimock`) renders it too; the host
//! (`mechmode`) saves the moves and poses the vehicle.

use imgui::{MouseButton, MouseCursor, Ui};

use crate::ui::{hover_tip, BAD, CL_COLOR, DIM, GOOD, WARN};

/// A joint that can join a sequence. Values are in the Workbench Angle's units and sign (the
/// game's input): degrees, or metres when `slide`.
pub(crate) struct Lane {
    pub name: String,
    pub base: f64,
    pub travel: (f64, f64),
    pub slide: bool,
    /// Nothing drives it: sequences leave it out (`model::JointLink::free`).
    pub free: bool,
    /// Its mechanism (`mech::mechanisms`) and that mechanism's degrees of freedom.
    pub mech: usize,
    pub dof: usize,
    /// Its saved sequences and its moves in each (`marks::Mark::seq`).
    pub seqs: Vec<(String, Vec<[f64; 3]>)>,
}

/// What the host does after a frame.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(crate) enum Act {
    /// This lane's sequences changed: save `State::lane_seqs`.
    Save(usize),
    /// Point this joint out on the vehicle.
    Show(usize),
    /// Something the user tried cannot be done: the game's error sound.
    Error,
}

#[derive(Clone, Copy, PartialEq)]
enum Drag {
    Scrub,
    /// Row, move, and where in the move it was grabbed (s).
    Shift(usize, usize, f64),
    Start(usize, usize),
    End(usize, usize),
}

/// A move: start (s), length (s), target.
type Move = [f64; 3];

#[derive(Default)]
pub(crate) struct State {
    /// Each sequence: its name and rows (a lane, and that joint's moves, by start).
    pub seqs: Vec<(String, Vec<(usize, Vec<Move>)>)>,
    pub cur: usize,
    pub t: f64,
    pub playing: bool,
    /// Playing backwards (the gear going up).
    pub back: bool,
    pub looping: bool,
    pub sel: Option<(usize, usize)>,
    /// A mechanism has more joints driven than it can move: no playing.
    blocked: bool,
    /// Lanes the host holds short of the sequence: their mechanism can move no further.
    pub stuck: Vec<usize>,
    /// Where playing stopped (a part hit another, a mechanism at the end of its reach) and why:
    /// marked on the timeline until the sequence or the vehicle changes.
    pub stop: Option<(f64, String)>,
    /// Collisions unticked: parts pass through each other.
    pub no_collide: bool,
    drag: Option<Drag>,
    rename: String,
    filter: String,
}

const NAME_W: f32 = 250.0;
const ROW_H: f32 = 24.0;
const RULER_H: f32 = 22.0;
const WIDTH: f32 = 780.0;
/// The window's width (the host fixes it; the height follows the rows).
pub(crate) const WINDOW_W: f32 = WIDTH + 18.0;
const MAX_ROWS: usize = 10;
const BAR: [f32; 4] = [0.22, 0.44, 0.64, 1.0];
const BAR_HOT: [f32; 4] = [0.28, 0.54, 0.76, 1.0];
const INK: [f32; 4] = [0.93, 0.94, 0.95, 1.0];
const DARK_INK: [f32; 4] = [0.05, 0.08, 0.1, 1.0];

impl State {
    /// The sequences as the joints have them; sequences with no joint yet (not saved) stay.
    pub fn load(&mut self, lanes: &[Lane]) {
        let cur = self.seqs.get(self.cur).map(|q| q.0.clone());
        let mut seqs: Vec<(String, Vec<(usize, Vec<Move>)>)> = self.seqs.iter().filter(|q| q.1.is_empty()).map(|q| (q.0.clone(), Vec::new())).collect();
        for (i, l) in lanes.iter().enumerate() {
            for (name, moves) in &l.seqs {
                let mut moves = moves.clone();
                moves.sort_by(|a, b| a[0].total_cmp(&b[0]));
                match seqs.iter_mut().find(|q| q.0 == *name) {
                    Some(q) => q.1.push((i, moves)),
                    None => seqs.push((name.clone(), vec![(i, moves)])),
                }
            }
        }
        seqs.sort_by(|a, b| a.0.cmp(&b.0));
        self.seqs = seqs;
        self.cur = cur.and_then(|c| self.seqs.iter().position(|q| q.0 == c)).unwrap_or(0);
        self.stop = None;
        self.sel = self.sel.filter(|&(r, k)| self.rows().get(r).is_some_and(|row| k < row.1.len()));
    }

    pub fn dragging(&self) -> bool {
        self.drag.is_some()
    }

    /// Lane `lane`'s sequences and moves, to save on its joint.
    pub fn lane_seqs(&self, lane: usize) -> Vec<(String, Vec<Move>)> {
        self.seqs.iter().filter_map(|(name, rows)| rows.iter().find(|r| r.0 == lane).map(|r| (name.clone(), r.1.clone()))).collect()
    }

    fn rows(&self) -> &[(usize, Vec<Move>)] {
        self.seqs.get(self.cur).map_or(&[], |q| &q.1)
    }

    /// When the current sequence ends, s.
    pub fn end(&self) -> f64 {
        crate::pose::seq_end(self.rows().iter().flat_map(|r| &r.1))
    }

    /// Advances playback by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        if !self.playing {
            return;
        }
        let end = self.end();
        self.t += if self.back { -dt } else { dt };
        if (0.0..end).contains(&self.t) {
            return;
        }
        if self.looping && end > 0.0 {
            // Keep the time past the end, so a loop runs on at an even pace.
            self.t = self.t.rem_euclid(end);
        } else {
            (self.t, self.playing) = (self.t.clamp(0.0, end), false);
        }
    }

    /// Lane `i`'s value at time `t`: its workbench value unless the current sequence moves it.
    pub fn value_at(&self, lanes: &[Lane], i: usize, t: f64) -> f64 {
        self.rows().iter().find(|r| r.0 == i).map_or(lanes[i].base, |r| crate::pose::seq_value(lanes[i].base, &r.1, t, self.looping.then(|| self.end())))
    }

    /// Playing stops at `at` (the host found a stop there).
    pub fn halt(&mut self, at: f64) {
        if self.playing {
            (self.t, self.playing) = (at, false);
        }
    }

    /// A sequence pose is to be shown: playing, or the play head past the start.
    pub fn posed(&self) -> bool {
        !self.rows().is_empty() && !self.blocked && (self.playing || self.t > 0.0)
    }

    fn select(&mut self, i: usize) {
        if self.cur != i {
            (self.t, self.playing, self.sel, self.stop) = (0.0, false, None, None);
        }
        self.cur = i;
    }

    /// Sorts row `r`'s moves by start, keeping the selection on the same move.
    fn sort_row(&mut self, r: usize) {
        let cur = self.cur;
        let row = &mut self.seqs[cur].1[r].1;
        let picked = self.sel.filter(|s| s.0 == r).map(|s| row[s.1]);
        row.sort_by(|a, b| a[0].total_cmp(&b[0]));
        if let Some(m) = picked {
            self.sel = row.iter().position(|x| *x == m).map(|k| (r, k));
        }
    }

    fn unique(&self, base: &str) -> String {
        std::iter::once(base.to_string()).chain((2..).map(|n| format!("{base} {n}"))).find(|n| !self.seqs.iter().any(|q| q.0 == *n)).unwrap_or_default()
    }
}

/// The window's contents (the host opens the window).
pub(crate) fn contents(ui: &Ui, lanes: &[Lane], s: &mut State) -> Vec<Act> {
    let mut acts = Vec::new();
    if lanes.is_empty() {
        ui.text_colored(DIM, "No pivots or sliders.");
        return acts;
    }
    if s.seqs.is_empty() {
        if ui.button_with_size("New sequence", [160.0, 26.0]) {
            new_seq(s, "Sequence");
            s.rename = s.seqs[s.cur].0.clone();
            ui.open_popup("Rename sequence");
        }
        rename_popup(ui, s, &mut acts);
        return acts;
    }
    header(ui, s, &mut acts);
    // Each mechanism it moves: its degrees of freedom and how many of them the sequence drives.
    let mut mechs: Vec<Mech> = Vec::new();
    for &(i, _) in s.rows() {
        let stuck = s.stuck.contains(&i);
        match mechs.iter_mut().find(|m| m.id == lanes[i].mech) {
            Some(m) => (m.driven, m.stuck) = (m.driven + 1, m.stuck || stuck),
            None => mechs.push(Mech { id: lanes[i].mech, name: &lanes[i].name, dof: lanes[i].dof, driven: 1, stuck }),
        }
    }
    let was = s.blocked;
    s.blocked = mechs.iter().any(|m| m.driven > m.dof);
    if s.blocked {
        s.playing = false;
        if !was {
            acts.push(Act::Error);
        }
    }
    let top = ui.cursor_screen_pos()[1];
    {
        let _off = ui.begin_disabled(s.blocked);
        transport(ui, lanes, s, &mut acts);
    }
    // A click on the greyed controls: the error sound.
    let (m, wx) = (ui.io().mouse_pos, ui.window_pos()[0]);
    if s.blocked && ui.is_mouse_clicked(imgui::MouseButton::Left) && ui.is_window_hovered() && (top..ui.cursor_screen_pos()[1]).contains(&m[1]) && (wx..wx + WINDOW_W).contains(&m[0]) {
        acts.push(Act::Error);
    }
    ui.dummy([0.0, 2.0]);
    if s.rows().is_empty() {
        return acts;
    }
    mech_chips(ui, &mechs, s.stop.as_ref().map_or("can't move further", |x| x.1.as_str()));
    timeline(ui, lanes, s, &mut acts);
    inspector(ui, lanes, s, &mut acts);
    if ui.is_window_focused_with_flags(imgui::WindowFocusedFlags::ROOT_AND_CHILD_WINDOWS) && !ui.io().want_text_input {
        if crate::plugin::PLAY_KEY.pressed(ui) {
            if s.blocked {
                acts.push(Act::Error);
            }
            play(s, s.back);
        }
        if let Some((r, k)) = s.sel.filter(|_| crate::plugin::DELETE_KEY.pressed(ui)) {
            delete_move(s, r, k, &mut acts);
        }
    }
    // An edit: the stop found may be gone.
    if acts.iter().any(|a| matches!(a, Act::Save(_))) {
        s.stop = None;
    }
    acts
}

fn new_seq(s: &mut State, base: &str) {
    let name = s.unique(base);
    s.seqs.push((name, Vec::new()));
    s.select(s.seqs.len() - 1);
}

fn delete_move(s: &mut State, r: usize, k: usize, acts: &mut Vec<Act>) {
    let cur = s.cur;
    s.seqs[cur].1[r].1.remove(k);
    s.sel = None;
    acts.push(Act::Save(s.seqs[cur].1[r].0));
}

/// A mechanism the current sequence moves (`Lane::mech`).
struct Mech<'a> {
    id: usize,
    /// Its first lane's name.
    name: &'a str,
    dof: usize,
    driven: usize,
    stuck: bool,
}

/// One chip per mechanism the sequence moves: how many ways it can move (degrees of freedom)
/// and how many of them the sequence drives. Green when all are driven, orange with too many.
/// `why`: what stopped a stuck mechanism.
fn mech_chips(ui: &Ui, mechs: &[Mech], why: &str) {
    let dl = ui.get_window_draw_list();
    let (pad, gap, h) = (8.0, 6.0, ROW_H);
    let mut x = 0.0;
    for (k, &Mech { name, dof, driven, stuck, .. }) in mechs.iter().enumerate() {
        let motions = |n: usize| if n == 1 { "motion" } else { "motions" };
        let (col, detail) = if dof == 0 {
            (BAD, "can't move".to_string())
        } else if stuck {
            (BAD, why.to_string())
        } else if driven > dof {
            (BAD, format!("{driven} driven, {dof} {} only", motions(dof)))
        } else if driven == dof {
            (GOOD, format!("{driven} of {dof} {} driven", motions(dof)))
        } else {
            (WARN, format!("{driven} of {dof} {} driven", motions(dof)))
        };
        let (wn, wd) = (ui.calc_text_size(name)[0], ui.calc_text_size(&detail)[0]);
        let w = pad * 2.0 + 14.0 + wn + 10.0 + wd;
        if k > 0 && x + gap + w <= WIDTH {
            ui.same_line_with_spacing(0.0, gap);
            x += gap;
        } else {
            x = 0.0;
        }
        x += w;
        let p = ui.cursor_screen_pos();
        ui.invisible_button(format!("##mech{k}"), [w, h]);
        let hov = ui.is_item_hovered();
        dl.add_rect(p, [p[0] + w, p[1] + h], [col[0], col[1], col[2], if hov { 0.30 } else { 0.18 }]).filled(true).rounding(4.0).build();
        dl.add_rect(p, [p[0] + w, p[1] + h], [col[0], col[1], col[2], 0.7]).rounding(4.0).build();
        let ty = p[1] + (h - ui.current_font_size()) * 0.5;
        dl.add_circle([p[0] + pad + 4.0, p[1] + h * 0.5], 4.0, col).filled(true).build();
        dl.add_text([p[0] + pad + 14.0, ty], INK, name);
        dl.add_text([p[0] + pad + 14.0 + wn + 10.0, ty], col, &detail);
        if hov {
            hover_tip(ui, format!("This mechanism can move {dof} {} (degrees of freedom). Each needs one joint driven by the sequence.", if dof == 1 { "way" } else { "ways" }));
        }
    }
    ui.dummy([0.0, 2.0]);
}

/// Plays forwards, or backwards with `back`; the same press again pauses.
fn play(s: &mut State, back: bool) {
    if s.blocked {
        return;
    }
    if s.playing && s.back == back {
        s.playing = false;
        return;
    }
    let end = s.end();
    if back && s.t <= 0.0 {
        s.t = end;
    } else if !back && s.t >= end {
        s.t = 0.0;
    }
    (s.playing, s.back) = (true, back);
}

/// Sequence picker and its commands; Delete stands apart on the right.
fn header(ui: &Ui, s: &mut State, acts: &mut Vec<Act>) {
    ui.set_next_item_width(240.0);
    let preview = s.seqs.get(s.cur).map_or("No sequences yet".to_string(), |q| q.0.clone());
    if let Some(_c) = ui.begin_combo("##seq", preview) {
        for i in 0..s.seqs.len() {
            let n = s.seqs[i].1.len();
            if ui.selectable_config(format!("{}##{i}", s.seqs[i].0)).selected(i == s.cur).build() {
                s.select(i);
            }
            ui.same_line_with_pos(200.0);
            ui.text_colored(DIM, format!("{n} joint{}", if n == 1 { "" } else { "s" }));
        }
    }
    ui.same_line();
    if ui.button("New") {
        new_seq(s, "Sequence");
        ui.open_popup("Rename sequence");
        s.rename = s.seqs[s.cur].0.clone();
    }
    if ui.is_item_hovered() {
        hover_tip(ui, "A new, empty sequence.");
    }
    ui.same_line();
    if ui.button("Rename") {
        s.rename = s.seqs[s.cur].0.clone();
        ui.open_popup("Rename sequence");
    }
    ui.same_line();
    if ui.button("Duplicate") {
        let copy = (s.unique(&format!("{} copy", s.seqs[s.cur].0)), s.seqs[s.cur].1.clone());
        acts.extend(copy.1.iter().map(|r| Act::Save(r.0)));
        s.seqs.push(copy);
        s.select(s.seqs.len() - 1);
    }
    if ui.is_item_hovered() {
        hover_tip(ui, "A copy to try changes on.");
    }
    let w = ui.calc_text_size("Delete")[0] + 2.0 * ui.clone_style().frame_padding[0];
    ui.same_line_with_pos(ui.window_content_region_max()[0] - w);
    if ui.button("Delete") {
        ui.open_popup("Delete sequence");
    }
    rename_popup(ui, s, acts);
    ui.popup("Delete sequence", || {
        ui.text(format!("Delete \"{}\"?", s.seqs[s.cur].0));
        ui.dummy([0.0, 4.0]);
        if ui.button_with_size("Delete", [90.0, 0.0]) {
            let gone = s.seqs.remove(s.cur);
            acts.extend(gone.1.iter().map(|r| Act::Save(r.0)));
            s.cur = 0;
            (s.t, s.playing, s.sel) = (0.0, false, None);
            ui.close_current_popup();
        }
        ui.same_line();
        if ui.button_with_size("Cancel", [90.0, 0.0]) || crate::plugin::CANCEL_KEY.pressed(ui) {
            ui.close_current_popup();
        }
    });
}

fn rename_popup(ui: &Ui, s: &mut State, acts: &mut Vec<Act>) {
    ui.popup("Rename sequence", || {
        if ui.is_window_appearing() {
            ui.set_keyboard_focus_here();
        }
        ui.set_next_item_width(220.0);
        let enter = ui.input_text("##rename", &mut s.rename).enter_returns_true(true).build();
        let name = crate::marks::seq_name(&s.rename);
        let taken = s.seqs.iter().enumerate().any(|(i, q)| i != s.cur && q.0 == name);
        if taken {
            ui.text_colored(WARN, "Another sequence has this name.");
        }
        if (enter || ui.button_with_size("OK", [90.0, 0.0])) && !name.is_empty() && !taken {
            s.seqs[s.cur].0 = name;
            acts.extend(s.seqs[s.cur].1.iter().map(|r| Act::Save(r.0)));
            ui.close_current_popup();
        }
        ui.same_line();
        if ui.button_with_size("Cancel", [90.0, 0.0]) || crate::plugin::CANCEL_KEY.pressed(ui) {
            ui.close_current_popup();
        }
    });
}

#[derive(Clone, Copy)]
enum Glyph {
    ToStart,
    Back,
    Play,
    Pause,
    ToEnd,
}

/// A small button with a drawn transport glyph.
fn glyph_button(ui: &Ui, id: &str, g: Glyph, tip: &str) -> bool {
    let clicked = ui.button_with_size(id, [30.0, 24.0]);
    let (a, b) = (ui.item_rect_min(), ui.item_rect_max());
    let (cx, cy) = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5);
    let dl = ui.get_window_draw_list();
    let (r, col) = (6.0, INK);
    let tri = |dir: f32, x: f32| dl.add_triangle([x - r * 0.8 * dir, cy - r], [x - r * 0.8 * dir, cy + r], [x + r * 0.8 * dir, cy], col).filled(true).build();
    let bar = |x: f32| dl.add_rect([x - 1.5, cy - r], [x + 1.5, cy + r], col).filled(true).build();
    match g {
        Glyph::Play => tri(1.0, cx + 1.0),
        Glyph::Back => tri(-1.0, cx - 1.0),
        Glyph::Pause => {
            bar(cx - 3.5);
            bar(cx + 3.5);
        }
        Glyph::ToStart => {
            bar(cx - 6.0);
            tri(-1.0, cx + 1.5);
        }
        Glyph::ToEnd => {
            tri(1.0, cx - 1.5);
            bar(cx + 6.0);
        }
    }
    if ui.is_item_hovered() {
        hover_tip(ui, tip);
    }
    clicked
}

/// Play controls, the time, and the joint picker on the right.
fn transport(ui: &Ui, lanes: &[Lane], s: &mut State, acts: &mut Vec<Act>) {
    let end = s.end();
    if glyph_button(ui, "##start", Glyph::ToStart, "To the start") {
        (s.t, s.playing) = (0.0, false);
    }
    ui.same_line_with_spacing(0.0, 3.0);
    let paused_back = !(s.playing && s.back);
    if glyph_button(ui, "##back", if paused_back { Glyph::Back } else { Glyph::Pause }, "Play backwards (Space plays or pauses)") {
        play(s, true);
    }
    ui.same_line_with_spacing(0.0, 3.0);
    let paused = !(s.playing && !s.back);
    if glyph_button(ui, "##play", if paused { Glyph::Play } else { Glyph::Pause }, "Play (Space plays or pauses)") {
        play(s, false);
    }
    ui.same_line_with_spacing(0.0, 3.0);
    if glyph_button(ui, "##end", Glyph::ToEnd, "To the end") {
        (s.t, s.playing) = (end, false);
    }
    ui.same_line_with_spacing(0.0, 12.0);
    ui.checkbox("Loop", &mut s.looping);
    ui.same_line_with_spacing(0.0, 12.0);
    let mut collide = !s.no_collide;
    if ui.checkbox("Collisions", &mut collide) {
        (s.no_collide, s.stop) = (!collide, None);
    }
    if ui.is_item_hovered() {
        hover_tip(ui, "Stop where a part hits another. Untick to let parts pass through.");
    }
    ui.same_line_with_spacing(0.0, 16.0);
    let mut t = s.t as f32;
    ui.set_next_item_width(64.0);
    if ui.input_float("##time", &mut t).display_format("%.1f s").build() {
        (s.t, s.playing) = ((t as f64).clamp(0.0, end), false);
    }
    ui.same_line();
    ui.align_text_to_frame_padding();
    ui.text_colored(DIM, format!("of {end:.1} s"));
    // Joints not in it yet, searchable; pointing at one rings it on the vehicle.
    let w = 220.0;
    ui.same_line_with_pos(ui.window_content_region_max()[0] - w);
    ui.set_next_item_width(w);
    if let Some(_c) = ui.begin_combo("##add", "Add a joint") {
        if ui.is_window_appearing() {
            ui.set_keyboard_focus_here();
        }
        ui.set_next_item_width(-1.0);
        ui.input_text("##find", &mut s.filter).hint("Search").build();
        let find = s.filter.to_lowercase();
        let mut any = false;
        for (i, l) in lanes.iter().enumerate() {
            if l.free || s.rows().iter().any(|r| r.0 == i) || !l.name.to_lowercase().contains(&find) {
                continue;
            }
            any = true;
            if ui.selectable(format!("{}##{i}", l.name)) {
                let cur = s.cur;
                s.seqs[cur].1.push((i, Vec::new()));
                s.seqs[cur].1.sort_by_key(|r| r.0);
                acts.push(Act::Save(i));
            }
            if ui.is_item_hovered() {
                acts.push(Act::Show(i));
            }
        }
        if !any {
            ui.text_colored(DIM, "No other joints match.");
        }
    }
}

/// Rows of moves against time: ruler and play head on top, joint names on the left.
fn timeline(ui: &Ui, lanes: &[Lane], s: &mut State, acts: &mut Vec<Act>) {
    let cur = s.cur;
    let n = s.rows().len();
    let end = s.end();
    let shown = n.min(MAX_ROWS);
    // Rows beyond MAX_ROWS scroll in a child; the ruler stays on top.
    let origin = ui.cursor_screen_pos();
    let span = (end * 1.1 + 0.5).max(4.0);
    let x0 = origin[0] + NAME_W;
    let px = ((WIDTH - NAME_W - 10.0) as f64 / span).max(1.0);
    let x_of = |t: f64| x0 + (t * px) as f32;
    let t_of = |x: f32| ((x - x0) as f64 / px).max(0.0);
    let mouse = ui.io().mouse_pos;

    // Ruler (scrubs the play head).
    ui.invisible_button("##ruler", [WIDTH, RULER_H]);
    let ruler_hot = ui.is_item_hovered();
    if ui.is_item_activated() {
        s.drag = Some(Drag::Scrub);
    }
    if ruler_hot {
        ui.set_mouse_cursor(Some(MouseCursor::ResizeEW));
    }
    let ph = x_of(s.t);
    let ry = origin[1] + RULER_H;
    {
    let dl = ui.get_window_draw_list();
    let (major, minor) = if span > 24.0 { (5.0, 1.0) } else if span > 8.0 { (2.0, 0.5) } else { (1.0, 0.5) };
    dl.add_rect([x0 - 4.0, origin[1]], [origin[0] + WIDTH, ry], [1.0, 1.0, 1.0, if ruler_hot { 0.06 } else { 0.03 }]).filled(true).rounding(3.0).build();
    let mut t = 0.0;
    while t <= span + 1e-9 {
        let big = (t / major - (t / major).round()).abs() < 1e-6;
        dl.add_line([x_of(t), ry - if big { 7.0 } else { 4.0 }], [x_of(t), ry], DIM).build();
        if big {
            dl.add_text([x_of(t) + 3.0, origin[1] + 2.0], DIM, format!("{t:.0}s"));
        }
        t += minor;
    }
    dl.add_text([origin[0] + 4.0, origin[1] + 3.0], DIM, "Joint");
    // The stop (`State::stop`) over the play head: a head stopped there shows red.
    for (x, col) in [(ph, WARN)].into_iter().chain(s.stop.as_ref().map(|st| (x_of(st.0), BAD))) {
        dl.add_triangle([x - 6.0, origin[1] + 9.0], [x + 6.0, origin[1] + 9.0], [x, ry - 1.0], col).filled(true).build();
    }
    }

    // Rows.
    let rows_h = ROW_H * shown as f32;
    let child = n > MAX_ROWS;
    let _c = child.then(|| ui.child_window("##rows").size([WIDTH + 14.0, rows_h]).begin()).flatten();
    let top = ui.cursor_screen_pos()[1];
    ui.invisible_button("##rows", [WIDTH, ROW_H * n as f32]);
    let (hot, active) = (ui.is_item_hovered(), ui.is_item_active());
    let row = ((mouse[1] - top) / ROW_H).floor();
    let row = (hot && row >= 0.0 && (row as usize) < n).then_some(row as usize);
    let in_names = mouse[0] < x0 - 4.0;
    let remove_x = origin[0] + NAME_W - 22.0;
    // A move's body, or its start or end edge, under the mouse.
    let hit = row.filter(|_| !in_names).and_then(|r| {
        let moves = &s.rows()[r].1;
        let k = moves.iter().position(|[st, d, _]| (x_of(*st) - 4.0..=x_of(st + d).max(x_of(*st) + 6.0) + 4.0).contains(&mouse[0]))?;
        let [st, d, _] = moves[k];
        let (a, b) = (x_of(st), x_of(st + d).max(x_of(st) + 6.0));
        let edge = if (mouse[0] - b).abs() <= 5.0 { 2 } else if (mouse[0] - a).abs() <= 4.0 { 1 } else { 0 };
        Some((r, k, edge, t_of(mouse[0]) - st))
    });
    if let Some(r) = row {
        acts.push(Act::Show(s.rows()[r].0));
    }
    if hit.is_some_and(|h| h.2 > 0) || matches!(s.drag, Some(Drag::Start(..) | Drag::End(..))) {
        ui.set_mouse_cursor(Some(MouseCursor::ResizeEW));
    }
    if hot && ui.is_mouse_clicked(MouseButton::Left) {
        if let Some((r, k, edge, grab)) = hit {
            s.sel = Some((r, k));
            s.drag = Some(match edge {
                1 => Drag::Start(r, k),
                2 => Drag::End(r, k),
                _ => Drag::Shift(r, k, grab),
            });
        } else if let Some(r) = row {
            let lane = s.rows()[r].0;
            if in_names && mouse[0] >= remove_x {
                s.seqs[cur].1.remove(r);
                s.sel = None;
                acts.push(Act::Save(lane));
                return;
            } else if !in_names {
                // A new move from where the joint is then, to its farther end of travel.
                let l = &lanes[lane];
                let st = (t_of(mouse[0]) * 10.0).round() / 10.0;
                let now = crate::pose::seq_value(l.base, &s.rows()[r].1, st, None);
                let far = if l.travel.1 - now >= now - l.travel.0 { l.travel.1 } else { l.travel.0 };
                let to = if l.travel.1.is_infinite() { now + 90.0 } else if l.slide { far } else { far.clamp(-90.0, 90.0) };
                s.seqs[cur].1[r].1.push([st, 1.0, to]);
                s.sel = Some((r, s.seqs[cur].1[r].1.len() - 1));
                s.sort_row(r);
                acts.push(Act::Save(lane));
            }
        }
    }
    if hot && ui.is_mouse_clicked(MouseButton::Right) {
        if let Some((r, k, ..)) = hit {
            delete_move(s, r, k, acts);
        }
    }
    let down = ui.is_mouse_down(MouseButton::Left);
    // Snap to a tenth of a second, or to another move's start or end within a few pixels.
    let snap = |t: f64, own: (usize, usize)| {
        let near = (s.rows().iter().enumerate())
            .flat_map(|(r, row)| row.1.iter().enumerate().filter(move |&(k, _)| (r, k) != own).flat_map(|(_, m)| [m[0], m[0] + m[1]]))
            .chain([0.0])
            .min_by(|a, b| (a - t).abs().total_cmp(&(b - t).abs()));
        match near {
            Some(e) if ((e - t) * px).abs() < 6.0 => e,
            _ => (t * 10.0).round() / 10.0,
        }
    };
    match s.drag {
        Some(Drag::Scrub) if down => (s.t, s.playing) = (t_of(mouse[0]).min(end), false),
        Some(d) if down && active => {
            let (r, k) = match d {
                Drag::Shift(r, k, _) | Drag::Start(r, k) | Drag::End(r, k) => (r, k),
                Drag::Scrub => unreachable!(),
            };
            let [st, len, to] = s.rows()[r].1[k];
            let new = match d {
                Drag::Shift(.., grab) => {
                    // Either end may snap; the nearer snap wins.
                    let a = t_of(mouse[0]) - grab;
                    let (sa, sb) = (snap(a, (r, k)), snap(a + len, (r, k)) - len);
                    [if (sa - a).abs() <= (sb - a).abs() { sa } else { sb }.max(0.0), len, to]
                }
                Drag::Start(..) => {
                    let start = snap(t_of(mouse[0]), (r, k)).min(st + len - 0.1).max(0.0);
                    [start, st + len - start, to]
                }
                _ => [st, (snap(t_of(mouse[0]), (r, k)) - st).max(0.1), to],
            };
            s.seqs[cur].1[r].1[k] = new;
        }
        Some(d) if !down => {
            if let Drag::Shift(r, ..) | Drag::Start(r, _) | Drag::End(r, _) = d {
                s.sort_row(r);
                acts.push(Act::Save(s.rows()[r].0));
            }
            s.drag = None;
        }
        _ => {}
    }

    // Draw rows: names, moves, the sequence's end.
    let dl = ui.get_window_draw_list();
    for (r, (lane, moves)) in s.rows().iter().enumerate() {
        let l = &lanes[*lane];
        let y = top + ROW_H * r as f32;
        let over = row == Some(r);
        let bg = if over { 0.075 } else if r % 2 == 0 { 0.035 } else { 0.0 };
        dl.add_rect([origin[0], y], [origin[0] + WIDTH, y + ROW_H], [1.0, 1.0, 1.0, bg]).filled(true).build();
        let clip = if over && in_names { remove_x - 4.0 } else { x0 - 8.0 };
        dl.with_clip_rect_intersect([origin[0], y], [clip, y + ROW_H], || {
            dl.add_text([origin[0] + 6.0, y + 4.0], INK, &l.name);
        });
        if over && in_names {
            let col = if mouse[0] >= remove_x { [0.95, 0.4, 0.35, 1.0] } else { DIM };
            dl.add_text([remove_x + 6.0, y + 4.0], col, "x");
        }
        for (k, &[st, d, to]) in moves.iter().enumerate() {
            let (a, b) = ([x_of(st), y + 4.0], [x_of(st + d).max(x_of(st) + 6.0), y + ROW_H - 4.0]);
            let sel = s.sel == Some((r, k));
            let hov = hit.is_some_and(|h| h.0 == r && h.1 == k);
            let fill = if sel { CL_COLOR } else if hov { BAR_HOT } else { BAR };
            dl.add_rect(a, b, fill).filled(true).rounding(3.0).build();
            if sel || hov {
                // Grips at both ends.
                let grip = [1.0, 1.0, 1.0, 0.55];
                dl.add_rect([a[0] + 2.0, a[1] + 4.0], [a[0] + 4.0, b[1] - 4.0], grip).filled(true).build();
                dl.add_rect([b[0] - 4.0, a[1] + 4.0], [b[0] - 2.0, b[1] - 4.0], grip).filled(true).build();
            }
            let label = value(l, to);
            let lw = ui.calc_text_size(&label)[0];
            if b[0] - a[0] > lw + 12.0 {
                dl.add_text([a[0] + 7.0, a[1] + 1.0], if sel { DARK_INK } else { INK }, &label);
            } else {
                dl.add_text([b[0] + 4.0, a[1] + 1.0], DIM, &label);
            }
        }
    }
    dl.add_line([x0 - 4.0, top], [x0 - 4.0, top + ROW_H * n as f32], [1.0, 1.0, 1.0, 0.08]).build();
    // The sequence's end, and the play head over the moves.
    let bottom = top + ROW_H * n as f32;
    if end > 0.0 {
        dl.add_line([x_of(end), top], [x_of(end), bottom], [1.0, 1.0, 1.0, 0.18]).build();
    }
    dl.add_line([ph, top], [ph, bottom], WARN).thickness(2.0).build();
    if let Some((at, why)) = &s.stop {
        let x = x_of(*at);
        dl.add_line([x, top], [x, bottom], BAD).thickness(2.0).build();
        if (mouse[0] - x).abs() <= 5.0 && (origin[1]..bottom).contains(&mouse[1]) {
            hover_tip(ui, format!("Stopped at {at:.2} s: {why}"));
        }
    }
    let _ = (ry, rows_h);
}

/// A target as the game's input reads: degrees, or metres for a slider.
fn value(l: &Lane, v: f64) -> String {
    match (l.slide, v.abs() < 0.005) {
        (true, true) => "0 m".into(),
        (true, false) => format!("{v:+.2} m"),
        (false, _) if v.abs() < 0.5 => "0°".into(),
        (false, _) => format!("{v:+.0}°"),
    }
}

/// The selected move's numbers. End and length follow each other (the start stays).
fn inspector(ui: &Ui, lanes: &[Lane], s: &mut State, acts: &mut Vec<Act>) {
    ui.dummy([0.0, 6.0]);
    let cur = s.cur;
    let Some((r, k)) = s.sel.filter(|&(r, k)| s.rows().get(r).is_some_and(|row| k < row.1.len())) else {
        return;
    };
    let lane = s.rows()[r].0;
    let l = &lanes[lane];
    let n = s.rows()[r].1.len();
    let [st, len, to] = s.rows()[r].1[k];
    ui.align_text_to_frame_padding();
    ui.text(&l.name);
    ui.same_line();
    ui.text_colored(DIM, if n > 1 { format!("move {} of {n}, to {}", k + 1, value(l, to)) } else { format!("moves to {}", value(l, to)) });
    let w = ui.calc_text_size("Delete move")[0] + 2.0 * ui.clone_style().frame_padding[0];
    ui.same_line_with_pos(ui.window_content_region_max()[0] - w);
    if ui.button("Delete move") {
        delete_move(s, r, k, acts);
        return;
    }
    let mut v = [st as f32, (st + len) as f32, len as f32, to as f32];
    let mut changed = [false; 4];
    let unit = if l.slide { "%+.2f m" } else { "%+.0f°" };
    for (i, (label, fmt)) in [("Start", "%.1f s"), ("End", "%.1f s"), ("Length", "%.1f s"), (if l.slide { "Offset" } else { "Angle" }, unit)].into_iter().enumerate() {
        if i > 0 {
            ui.same_line_with_spacing(0.0, 18.0);
        }
        ui.align_text_to_frame_padding();
        ui.text_colored(DIM, label);
        ui.same_line();
        ui.set_next_item_width(70.0);
        changed[i] = ui.input_float(format!("##{label}"), &mut v[i]).display_format(fmt).build();
        if i == 3 && l.travel.1.is_finite() && ui.is_item_hovered() {
            hover_tip(ui, if l.slide { format!("Travel {:+.2} to {:+.2} m", l.travel.0, l.travel.1) } else { format!("Travel {:+.0}° to {:+.0}°", l.travel.0, l.travel.1) });
        }
    }
    if changed.iter().any(|c| *c) {
        let start = (v[0] as f64).max(0.0);
        let len = if changed[1] { v[1] as f64 - start } else { v[2] as f64 }.max(0.1);
        s.seqs[cur].1[r].1[k] = [start, len, (v[3] as f64).clamp(l.travel.0, l.travel.1)];
        s.sort_row(r);
        acts.push(Act::Save(lane));
    }
}

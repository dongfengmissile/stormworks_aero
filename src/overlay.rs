//! The workbench analysis window (F7), modelled on FAR's editor GUI: Sweep, Stability,
//! Simulation, Area rule, plus a voxel view where wings and control roles are designated,
//! propeller and rotor peaks, and settings.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock, Mutex};

use hudhook::imgui::{self, Condition, Context, Io, Key, MouseButton, TableColumnFlags, TableColumnSetup, TableFlags, TableRowFlags, TextureId, Ui};
use hudhook::{ImguiRenderLoop, MessageFilter, RenderContext};

use crate::defs::Library;
use crate::editor::{self, Snapshot};
use crate::logging::log_line;
use crate::math::V3;
use crate::model::{Coeffs, Cond, Designations, Model, Stability};
use crate::vehicle::Comp;
use crate::view::{self, Camera, Face, Image};
use crate::voxel::{EMPTY, INTERIOR};
use crate::wing::{FoilKind, Role, AEROFOILS};

const VIEW_W: usize = 760;
const VIEW_H: usize = 460;

const CL_COLOR: [f32; 4] = [0.30, 0.80, 0.95, 1.0];
const CD_COLOR: [f32; 4] = [0.95, 0.40, 0.35, 1.0];
const CM_COLOR: [f32; 4] = [0.95, 0.80, 0.30, 1.0];
const LD_COLOR: [f32; 4] = [0.45, 0.85, 0.45, 1.0];
const DIM: [f32; 4] = [0.60, 0.62, 0.66, 1.0];
const GOOD: [f32; 4] = [0.45, 0.85, 0.45, 1.0];
const BAD: [f32; 4] = [0.95, 0.40, 0.35, 1.0];
const WARN: [f32; 4] = [0.95, 0.70, 0.30, 1.0];
/// Controls column width, and where its fields start.
const LEFT_W: f32 = 210.0;
const FIELD_X: f32 = 92.0;
/// Plot area inset from the plot's own left and right edges (tick labels).
const PLOT_PAD_L: f32 = 44.0;
const PLOT_PAD_R: f32 = 14.0;
const FLAPS: [&str; 4] = ["Up", "Climb", "Takeoff", "Landing"];

#[derive(Clone, Copy, PartialEq, Debug)]
enum Tool {
    Inspect,
    Wing,
    Role,
    Foil,
    /// Mark wing blocks as a separate wing (own panels and wing, e.g. for another aerofoil).
    Split,
}

struct Built {
    model: Model,
    faces: Vec<Face>,
}

struct Sweep {
    mach_sweep: bool,
    x: Vec<f64>,
    up: Vec<Coeffs>,
    down: Vec<Coeffs>,
}

pub(crate) struct Overlay {
    open: bool,
    lib: Arc<Mutex<Library>>,
    status: String,

    // Vehicle source.
    snapshot: Option<Arc<Snapshot>>,
    des: Designations,

    // Model, built off the render thread.
    model: Option<Arc<Model>>,
    faces: Arc<Vec<Face>>,
    model_dirty: bool,
    building: Arc<AtomicBool>,
    built: Arc<Mutex<Option<Built>>>,

    // Static analysis.
    sweep_mach: bool,
    aoa_lo: f32,
    aoa_hi: f32,
    mach_lo: f32,
    mach_hi: f32,
    npts: i32,
    other_aoa: f32,
    other_mach: f32,
    alt: f32,
    flap: usize,
    pitch: f32,
    sweep: Option<Sweep>,

    // Stability.
    st_alt: f32,
    st_mach: f32,
    st_flap: usize,
    stab: Option<Stability>,

    // Simulation.
    sim_lateral: bool,
    sim_x0: [f32; 4],
    sim_end: f32,
    sim_dt: f32,
    sim: Option<(Vec<f64>, Vec<[f64; 4]>)>,

    // Transonic.
    show_area: bool,
    show_curv: bool,
    show_cp: bool,

    /// Propellers tab, per `Model::props`: blade RPS asked for, and its peak (rps, result).
    prop_rps: Vec<f32>,
    prop_peaks: Vec<Option<(f32, Option<(f64, f64, f64, f64)>)>>,

    // Voxel view.
    cam: Camera,
    tool: Tool,
    role_pick: usize,
    foil_pick: usize,
    tex: Option<TextureId>,
    img: Option<Image>,
    upload: bool,
    view_dirty: bool,
    hover_face: Option<usize>,
    dragged: bool,
    /// Designations before each wing / role edit (Ctrl+Z).
    undo: Vec<Designations>,
    /// Designations undone, for redo (cleared by a new edit).
    redo: Vec<Designations>,

    /// Workbench vehicle painted by wing role / aerofoil (while in the Aero wings mode).
    world: bool,
    /// The game's "Aero wings" toolbar mode is on (picking on the real vehicle).
    bench: bool,
    /// What the published paint was computed from: (model, aerofoil colours, designated wing count).
    paint_key: Option<(usize, bool, usize)>,
    /// Flight data readout (F9).
    flight_data: bool,
}

impl Overlay {
    pub(crate) fn new(game: PathBuf, appdata: PathBuf) -> Self {
        Overlay {
            open: false,
            lib: Arc::new(Mutex::new(Library::new(&game, &appdata))),
            status: String::new(),
            snapshot: None,
            des: Designations::default(),
            model: None,
            faces: Arc::new(Vec::new()),
            model_dirty: false,
            building: Arc::new(AtomicBool::new(false)),
            built: Arc::new(Mutex::new(None)),
            sweep_mach: false,
            aoa_lo: 0.0,
            aoa_hi: 25.0,
            mach_lo: 0.0,
            mach_hi: 1.5,
            npts: 100,
            other_aoa: 2.0,
            other_mach: 0.2,
            alt: 0.0,
            flap: 0,
            pitch: 0.0,
            sweep: None,
            st_alt: 500.0,
            st_mach: 0.2,
            st_flap: 0,
            stab: None,
            sim_lateral: false,
            sim_x0: [0.0, 0.0, 5.0, 0.0],
            sim_end: 10.0,
            sim_dt: 0.01,
            sim: None,
            show_area: true,
            show_curv: true,
            show_cp: false,
            prop_rps: Vec::new(),
            prop_peaks: Vec::new(),
            cam: Camera { yaw: 0.6, pitch: 0.45, dist: 25.0, target: V3::ZERO },
            tool: Tool::Inspect,
            role_pick: 1,
            foil_pick: 1,
            tex: None,
            img: None,
            upload: false,
            view_dirty: true,
            hover_face: None,
            dragged: false,
            undo: Vec::new(),
            redo: Vec::new(),
            world: false,
            paint_key: None,
            bench: false,
            flight_data: false,
        }
    }

    /// Paint the workbench vehicle by wing role / aerofoil, or stop.
    fn set_world(&mut self, on: bool) {
        self.world = on;
        self.paint_key = None;
        if !on {
            *editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()) = None;
        }
    }

    /// FAR-style flight data for the vehicles the plugin flies (click-through: never takes keys).
    fn draw_flight_data(&mut self, ui: &Ui) {
        let mut data = crate::flight::DEBUG.lock().map(|d| d.clone()).unwrap_or_default();
        data.sort_by(|a, b| b.airspeed.total_cmp(&a.airspeed));
        let flags =
            imgui::WindowFlags::NO_INPUTS | imgui::WindowFlags::NO_FOCUS_ON_APPEARING | imgui::WindowFlags::ALWAYS_AUTO_RESIZE | imgui::WindowFlags::NO_NAV;
        ui.window("Flight data##flight").position([12.0, 120.0], Condition::FirstUseEver).bg_alpha(0.75).flags(flags).build(|| {
            if data.is_empty() {
                if crate::flight::ENABLED.load(Ordering::Relaxed) {
                    ui.text_colored(DIM, "No vehicle in flight");
                } else {
                    ui.text_colored(WARN, "Plugin flight model is off (F7, Settings)");
                }
            }
            for (n, v) in data.iter().take(3).enumerate() {
                if n > 0 {
                    ui.dummy([0.0, 6.0]);
                }
                ui.text(format!("Vehicle {}", v.id));
                if !v.designations.is_empty() {
                    ui.same_line_with_spacing(0.0, 10.0);
                    ui.text_colored(DIM, &v.designations);
                }
                flight_vehicle(ui, n, v);
            }
        });
    }

    /// The game's "Aero wings" toolbar mode: paint on, a small tool window, clicks on the vehicle
    /// applied with the current tool.
    fn bench_mode(&mut self, ui: &Ui) {
        let on = crate::toolbar::active();
        editor::PICK_WANTED.store(on, Ordering::Relaxed);
        if on != self.bench {
            self.bench = on;
            // The workbench paint is on exactly while the Aero wings mode is.
            self.set_world(on);
            if on {
                let _ = crate::toolbar::take_click(); // a press from before the mode
            }
        }
        if !on {
            return;
        }
        // Tool, role and aerofoil come from the game-style side shelves.
        let mut tool = [Tool::Wing, Tool::Role, Tool::Foil][crate::toolbar::TOOL.load(Ordering::Relaxed).min(2)];
        if tool == Tool::Wing && crate::toolbar::WING_MODE.load(Ordering::Relaxed) == 4 {
            tool = Tool::Split; // the wing shelf's "New wing" button
        }
        if tool != self.tool {
            self.tool = tool;
            self.view_dirty = true;
        }
        self.role_pick = crate::toolbar::ROLE.load(Ordering::Relaxed).min(4);
        self.foil_pick = crate::toolbar::foil().min(AEROFOILS.len() - 1);
        crate::toolbar::CAN_UNDO.store(!self.undo.is_empty(), Ordering::Relaxed);
        crate::toolbar::CAN_REDO.store(!self.redo.is_empty(), Ordering::Relaxed);
        for _ in 0..crate::toolbar::UNDO_REQ.swap(0, Ordering::Relaxed) {
            self.undo_designation();
        }
        for _ in 0..crate::toolbar::REDO_REQ.swap(0, Ordering::Relaxed) {
            self.redo_designation();
        }
        if let (Some(at), Some(m)) = (crate::toolbar::take_click(), self.model.clone()) {
            let io = ui.io();
            let clip = editor::WORLD_CLIP.lock().map(|c| c.is_some()).unwrap_or(false);
            let pick = self.pick_world(at, io.display_size);
            let (cell, comp) = pick.and_then(|f| self.faces.get(f)).map_or((usize::MAX, u32::MAX), |f| (f.cell, f.comp));
            log_line(&format!(
                "bench pick at {at:?}: camera {} face {pick:?} cell {cell} comp {comp} tool {:?}",
                if clip { "ok" } else { "MISSING" },
                self.tool
            ));
            match pick {
                Some(face) => self.apply_tool(&m, face),
                None => self.status = "no block under the cursor".into(),
            }
        }
    }

    /// Publish the workbench paint (voxel colours per body) when the model or colouring changed.
    fn publish_paint(&mut self) {
        let Some(m) = self.model.clone() else { return };
        let key = (Arc::as_ptr(&m) as usize, self.tool == Tool::Foil, self.des.wings.len() + self.des.splits.len());
        if self.paint_key == Some(key) {
            return;
        }
        self.paint_key = Some(key);
        let g = &m.grid;
        let mut per_body: Vec<crate::paint::VoxelColours> = Vec::new();
        for cell in 0..g.owner.len() {
            let o = g.owner[cell];
            let Some(col) = self.cell_color(&m, cell) else {
                continue;
            };
            let b = if o != EMPTY && o != INTERIOR {
                m.comps[o as usize].body as usize
            } else {
                continue;
            };
            if per_body.len() <= b {
                per_body.resize_with(b + 1, Default::default);
            }
            per_body[b].insert(g.cell(cell), crate::paint::rgba(col));
        }
        let mut h = std::collections::hash_map::DefaultHasher::new();
        std::hash::Hash::hash(&key, &mut h);
        let key = std::hash::Hasher::finish(&h);
        // Moving meshes drawn outside the voxel paint (missile fins, control surfaces): by part.
        let tints: Vec<([i32; 3], u32)> = m
            .comps
            .iter()
            .enumerate()
            .filter_map(|(i, c)| {
                let col = match m.fin_of.get(&(i as u32)) {
                    Some(&(p, _)) => self.panel_color(&m, p),
                    None => g.index(c.pos).filter(|&k| g.owner[k] == i as u32).and_then(|k| self.cell_color(&m, k))?,
                };
                Some((c.pos, crate::paint::rgba(col)))
            })
            .collect();
        *editor::PAINT.lock().unwrap_or_else(|p| p.into_inner()) = Some((key, per_body.into_iter().map(Arc::new).collect(), Arc::new(tints)));
    }

    /// Workbench: the designations live on the parts (saved in the vehicle file, carried by copy/paste).
    fn store_marks(&self) {
        if let Some(s) = &self.snapshot {
            crate::marks::store(&self.des, &s.comps, &s.ptrs);
        }
    }

    /// New workbench snapshot: the parts' marks are the designations.
    fn sync_marks(&mut self) {
        let Some(snap) = self.snapshot.clone() else {
            return;
        };
        let d = crate::marks::designations(&snap.comps, &snap.ptrs).unwrap_or_default();
        if d != self.des {
            self.des = d;
            self.model_dirty = true;
        }
    }

    fn designations_changed(&mut self) {
        self.model_dirty = true;
        self.store_marks();
        self.status = if crate::markhooks::HOOKED.load(Ordering::Relaxed) {
            "marks are stored on the parts: save the vehicle to keep them".into()
        } else {
            "marks are kept for this session only (the game's save could not be hooked, see aero.log)".into()
        };
    }

    fn current_comps(&self) -> Option<Vec<Comp>> {
        self.snapshot.as_ref().map(|s| s.comps.clone())
    }

    /// Pick up finished builds and start a new one when the inputs changed.
    fn pump_model(&mut self) {
        let done = self.built.lock().unwrap_or_else(|p| p.into_inner()).take();
        if let Some(b) = done {
            let first = self.model.is_none();
            if first || self.cam.dist <= 0.0 {
                self.frame_camera(&b.model);
            }
            self.model = Some(Arc::new(b.model));
            self.prop_peaks.clear();
            self.faces = Arc::new(b.faces);
            self.view_dirty = true;
            self.sweep = None;
            self.stab = None;
            self.sim = None;
        }
        let latest = editor::LATEST.lock().unwrap_or_else(|p| p.into_inner()).clone();
        if latest.as_ref().map(|s| s.gen) != self.snapshot.as_ref().map(|s| s.gen) {
            self.snapshot = latest;
            self.model_dirty = true;
            self.sync_marks();
        }
        if !self.model_dirty || self.building.load(Ordering::Acquire) {
            return;
        }
        let Some(comps) = self.current_comps() else {
            return;
        };
        self.model_dirty = false;
        self.building.store(true, Ordering::Release);
        let (lib, des, building, built) = (self.lib.clone(), self.des.clone(), self.building.clone(), self.built.clone());
        let _ = std::thread::Builder::new().name("aero-build".into()).spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _fault = crate::fault::enter(crate::fault::stage::ANALYSIS);
                let mut lib = lib.lock().unwrap_or_else(|p| p.into_inner());
                let t = std::time::Instant::now();
                let model = Model::build(comps, &mut lib, &des);
                let faces = view::faces(&model.grid);
                log_line(&format!(
                    "model: {} comps, {} voxels, {} panels, {} faces, missing defs {:?}, {} ms",
                    model.comps.len(),
                    model.grid.count_filled(),
                    model.panels.len(),
                    faces.len(),
                    model.grid.missing,
                    t.elapsed().as_millis()
                ));
                Built { model, faces }
            }));
            match result {
                Ok(b) => *built.lock().unwrap_or_else(|p| p.into_inner()) = Some(b),
                Err(_) => log_line("model build panicked"),
            }
            building.store(false, Ordering::Release);
        });
    }

    fn frame_camera(&mut self, m: &Model) {
        let g = &m.grid;
        let lo = crate::body::aero_pos(g.min);
        let hi = crate::body::aero_pos([g.min[0] + g.dims[0] as i32, g.min[1] + g.dims[1] as i32, g.min[2] + g.dims[2] as i32]);
        let size = (hi - lo).len();
        self.cam.target = (lo + hi) * 0.5;
        self.cam.dist = size * 1.4;
    }

    /// Vehicle stats on one line, then anything that needs attention.
    fn header(&mut self, ui: &Ui) {
        if !editor::editor_active() {
            ui.text_colored(WARN, "Open a vehicle in the workbench to analyse it.");
        }
        match &self.model {
            Some(m) => {
                let items = [
                    ("Mass", format!("{:.0} kg", m.mass)),
                    ("Wing area", format!("{:.2} m²", m.s_ref)),
                    ("MAC", format!("{:.2} m", m.mac_ref)),
                    ("Span", format!("{:.2} m", m.b_ref)),
                    ("Panels", m.panels.len().to_string()),
                    ("Parts", m.comps.len().to_string()),
                ];
                for (i, (k, v)) in items.iter().enumerate() {
                    if i > 0 {
                        ui.same_line_with_spacing(0.0, 22.0);
                    }
                    ui.text_colored(DIM, k);
                    ui.same_line_with_spacing(0.0, 6.0);
                    ui.text(v);
                }
                if m.panels.is_empty() {
                    ui.text_colored(WARN, "No wings marked");
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Mark wing blocks on the Wings tab. Without wings, the reference area and length come from the fuselage.");
                    }
                }
                if !m.grid.missing.is_empty() {
                    ui.text_colored(WARN, format!("Unknown parts, counted as one block each: {}", m.grid.missing.join(", ")));
                }
            }
            None if self.building.load(Ordering::Relaxed) => ui.text_colored(DIM, "Building model..."),
            None => ui.text_colored(DIM, "No vehicle"),
        }
        if !self.status.is_empty() {
            ui.text_colored(DIM, &self.status);
        }
        ui.dummy([0.0, 2.0]);
    }

    fn tab_static(&mut self, ui: &Ui, m: &Arc<Model>) {
        if let Some(_c) = left_pane(ui) {
            heading(ui, "Sweep");
            if ui.radio_button_bool("Angle of attack", !self.sweep_mach) {
                self.sweep_mach = false;
            }
            ui.same_line();
            if ui.radio_button_bool("Mach", self.sweep_mach) {
                self.sweep_mach = true;
            }
            if self.sweep_mach {
                input_f(ui, "From", &mut self.mach_lo, "%.2f");
                input_f(ui, "To", &mut self.mach_hi, "%.2f");
            } else {
                input_f(ui, "From", &mut self.aoa_lo, "%.1f°");
                input_f(ui, "To", &mut self.aoa_hi, "%.1f°");
            }
            field(ui, "Points");
            ui.input_int("##pts", &mut self.npts).build();
            heading(ui, "Condition");
            if self.sweep_mach {
                input_f(ui, "AoA", &mut self.other_aoa, "%.1f°");
            } else {
                input_f(ui, "Mach", &mut self.other_mach, "%.2f");
            }
            input_f(ui, "Altitude", &mut self.alt, "%.0f m");
            field(ui, "Flaps");
            ui.combo_simple_string("##flaps", &mut self.flap, &FLAPS);
            input_f(ui, "Pitch input", &mut self.pitch, "%.2f");
            self.pitch = self.pitch.clamp(-1.0, 1.0);
            self.npts = self.npts.clamp(2, 500);
            if primary(ui, "Run sweep") {
                let base = Cond {
                    alpha: (self.other_aoa as f64).to_radians(),
                    mach: self.other_mach as f64,
                    alt: self.alt as f64,
                    pitch: self.pitch as f64,
                    flap: self.flap as f64 / 3.0,
                    ..Default::default()
                };
                self.sweep = Some(if self.sweep_mach {
                    let lo = self.mach_lo.max(0.0) as f64;
                    let (x, up) = m.mach_sweep(base, lo, (self.mach_hi as f64).max(lo + 0.01), self.npts as usize);
                    Sweep { mach_sweep: true, x, up, down: Vec::new() }
                } else {
                    let lo = (self.aoa_lo as f64).clamp(-90.0, 90.0);
                    let (x, up, down) = m.aoa_sweep(base, lo, (self.aoa_hi as f64).clamp(lo + 0.1, 90.0), self.npts as usize);
                    Sweep { mach_sweep: false, x, up, down }
                });
            }
        }
        let Some(_c) = right_pane(ui) else { return };
        let Some(s) = &self.sweep else {
            ui.text_colored(DIM, "Not run yet");
            return;
        };
        let mut series = vec![
            ("CL", CL_COLOR, s.up.iter().map(|c| c.cl).collect::<Vec<_>>()),
            ("CD", CD_COLOR, s.up.iter().map(|c| c.cd).collect()),
            ("Cm", CM_COLOR, s.up.iter().map(|c| c.cm).collect()),
            ("L/D / 10", LD_COLOR, s.up.iter().map(|c| c.cl * 0.1 / c.cd).collect()),
        ];
        // Return pass of the AoA sweep (stall hysteresis), dimmed.
        if !s.down.is_empty() {
            series.push(("CL return", CL_COLOR.map(|v| v * 0.5), s.down.iter().map(|c| c.cl).collect()));
        }
        let avail = ui.content_region_avail();
        let h = if s.mach_sweep { avail[1] - 6.0 } else { avail[1] - 70.0 };
        plot(ui, "static", [avail[0], h], &s.x, &series, 4, if s.mach_sweep { "Mach" } else { "Angle of attack (°)" });
        if !s.mach_sweep {
            sweep_readout(ui, &s.x, &s.up);
        }
    }

    fn tab_stability(&mut self, ui: &Ui, m: &Arc<Model>) {
        if let Some(_c) = left_pane(ui) {
            heading(ui, "Condition");
            input_f(ui, "Mach", &mut self.st_mach, "%.2f");
            self.st_mach = self.st_mach.max(0.001);
            input_f(ui, "Altitude", &mut self.st_alt, "%.0f m");
            field(ui, "Flaps");
            ui.combo_simple_string("##flaps", &mut self.st_flap, &FLAPS);
            if primary(ui, "Calculate") {
                self.stab = Some(m.stability(self.st_mach as f64, self.st_alt as f64, self.st_flap as f64 / 3.0));
            }
            if let Some(s) = &self.stab {
                heading(ui, "Inertia (kg m²)");
                kv(ui, "Ixx", &sci(s.inertia[0]));
                kv(ui, "Iyy", &sci(s.inertia[1]));
                kv(ui, "Izz", &sci(s.inertia[2]));
                kv(ui, "Ixz", &sci(s.inertia[5]));
            }
        }
        let Some(_c) = right_pane(ui) else { return };
        let Some(s) = &self.stab else {
            ui.text_colored(DIM, "Not calculated");
            if ui.is_item_hovered() {
                ui.tooltip_text("Trims for level flight (lift = weight) at this speed, then perturbs each state.");
            }
            return;
        };
        heading(ui, "Level flight");
        let cols = [("Speed", 0.0), ("AoA", 0.0), ("CL", 0.0), ("CL needed", 0.0), ("CD", 0.0), ("L/D", 0.0)];
        if let Some(_t) = table(ui, "##trim", &cols) {
            headers(ui, &cols, 0);
            ui.table_next_row();
            let vals = [
                format!("{:.1} m/s", s.u0),
                format!("{:.2}°", s.alpha.to_degrees()),
                format!("{:.4}", s.cl),
                format!("{:.4}", s.cl_req),
                format!("{:.4}", s.cd),
                format!("{:.1}", s.cl / s.cd),
            ];
            for (i, v) in vals.iter().enumerate() {
                ui.table_set_column_index(i);
                right(ui, v, if i == 1 && !s.trim_note.is_empty() { Some(WARN) } else { None });
            }
        }
        if !s.trim_note.is_empty() {
            ui.text_colored(WARN, "Can't trim: not enough lift at this speed. Shown at the closest AoA.");
        }
        heading(ui, "Longitudinal");
        deriv_table(
            ui,
            "##lon",
            &["", "Down speed w", "Fwd speed u", "Pitch rate q", "Elevator"],
            &[
                (
                    "Z  heave",
                    [
                        ("Zw", s.zw, -1, "1/s", "Change in Z-direction acceleration with respect to Z-direction velocity; should be negative"),
                        ("Zu", s.zu, -1, "1/s", "Change in Z-direction acceleration with respect to X-direction velocity; should be negative"),
                        ("Zq", s.zq, 0, "m/s", "Change in Z-direction acceleration with respect to pitch-up rate"),
                        ("Zde", s.zde, 0, "m/s²", "Change in Z-direction acceleration with respect to full pitch control input"),
                    ],
                ),
                (
                    "X  surge",
                    [
                        ("Xw", s.xw, 0, "1/s", "Change in X-direction acceleration with respect to Z-direction velocity"),
                        ("Xu", s.xu, -1, "1/s", "Change in X-direction acceleration with respect to X-direction velocity; should be negative"),
                        ("Xq", s.xq, 0, "m/s", "Change in X-direction acceleration with respect to pitch-up rate"),
                        ("Xde", s.xde, 0, "m/s²", "Change in X-direction acceleration with respect to full pitch control input"),
                    ],
                ),
                (
                    "M  pitch",
                    [
                        ("Mw", s.mw, -1, "1/(m s)", "Change in pitch-up angular acceleration with respect to Z-direction velocity; should be negative (statically stable)"),
                        ("Mu", s.mu, 0, "1/(m s)", "Change in pitch-up angular acceleration with respect to X-direction velocity"),
                        ("Mq", s.mq, -1, "1/s", "Change in pitch-up angular acceleration with respect to pitch-up rate; should be negative (pitch damping)"),
                        ("Mde", s.mde, 1, "1/s²", "Change in pitch-up angular acceleration with respect to full pitch control input; should be positive"),
                    ],
                ),
            ],
        );
        heading(ui, "Lateral");
        deriv_table(
            ui,
            "##lat",
            &["", "Sideslip b", "Roll rate p", "Yaw rate r"],
            &[
                (
                    "Y  side",
                    [
                        ("Yb", s.yb, -1, "m/s²", "Change in Y-direction acceleration with respect to sideslip angle (beta); should be negative"),
                        ("Yp", s.yp, 0, "m/s", "Change in Y-direction acceleration with respect to roll-right rate"),
                        ("Yr", s.yr, 1, "m/s", "Change in Y-direction acceleration with respect to yaw-right rate; should be positive"),
                    ],
                ),
                (
                    "L  roll",
                    [
                        ("Lb", s.lb, -1, "1/s²", "Change in roll-right angular acceleration with respect to sideslip (beta); should be negative (dihedral effect)"),
                        ("Lp", s.lp, -1, "1/s", "Change in roll-right angular acceleration with respect to roll-right rate; should be negative (roll damping)"),
                        ("Lr", s.lr, 1, "1/s", "Change in roll-right angular acceleration with respect to yaw-right rate; should be positive"),
                    ],
                ),
                (
                    "N  yaw",
                    [
                        ("Nb", s.nb, 1, "1/s²", "Change in yaw-right angular acceleration with respect to sideslip (beta); should be positive (weathervane stability)"),
                        ("Np", s.np, 0, "1/s", "Change in yaw-right angular acceleration with respect to roll-right rate"),
                        ("Nr", s.nr, -1, "1/s", "Change in yaw-right angular acceleration with respect to yaw-right rate; should be negative (yaw damping)"),
                    ],
                ),
            ],
        );
        ui.dummy([0.0, 4.0]);
        swatch(ui, GOOD, "Stabilising");
        ui.same_line_with_spacing(0.0, 18.0);
        swatch(ui, BAD, "Destabilising");
    }

    fn tab_sim(&mut self, ui: &Ui) {
        let names: [(&str, &str); 4] = if self.sim_lateral {
            [("Sideslip", "%.1f°"), ("Roll rate", "%.1f °/s"), ("Yaw rate", "%.1f °/s"), ("Bank", "%.1f°")]
        } else {
            [("Down speed", "%.1f m/s"), ("Fwd speed", "%.1f m/s"), ("Pitch rate", "%.1f °/s"), ("Pitch", "%.1f°")]
        };
        if let Some(_c) = left_pane(ui) {
            heading(ui, "Mode");
            if ui.radio_button_bool("Longitudinal", !self.sim_lateral) {
                self.sim_lateral = false;
                self.sim = None;
            }
            ui.same_line();
            if ui.radio_button_bool("Lateral", self.sim_lateral) {
                self.sim_lateral = true;
                self.sim = None;
            }
            heading(ui, "Start from");
            for (i, (n, f)) in names.iter().enumerate() {
                input_f(ui, n, &mut self.sim_x0[i], f);
            }
            heading(ui, "Run");
            input_f(ui, "Duration", &mut self.sim_end, "%.0f s");
            input_f(ui, "Time step", &mut self.sim_dt, "%.3f s");
            let _d = ui.begin_disabled(self.stab.is_none());
            if primary(ui, "Simulate") {
                if let Some(s) = &self.stab {
                    let d = std::f64::consts::PI / 180.0;
                    let k = if self.sim_lateral { [d, d, d, d] } else { [1.0, 1.0, d, d] };
                    let x0 = [0, 1, 2, 3].map(|i| self.sim_x0[i] as f64 * k[i]);
                    let dt = (self.sim_dt as f64).clamp(0.001, 1.0);
                    let end = (self.sim_end as f64).clamp(dt, 600.0);
                    self.sim = Some(s.simulate(self.sim_lateral, x0, end, dt));
                }
            }
        }
        let Some(_c) = right_pane(ui) else { return };
        if self.stab.is_none() {
            ui.text_colored(DIM, "Calculate on the Stability tab first");
            return;
        }
        let Some((t, xs)) = &self.sim else {
            ui.text_colored(DIM, "Not run yet");
            return;
        };
        let d = 180.0 / std::f64::consts::PI;
        let k = if self.sim_lateral { [d, d, d, d] } else { [1.0, 1.0, d, d] };
        let units = if self.sim_lateral { ["°", "°/s", "°/s", "°"] } else { ["m/s", "m/s", "°/s", "°"] };
        let labels: Vec<String> = (0..4).map(|i| format!("{} ({})", names[i].0, units[i])).collect();
        let cols = [LD_COLOR, CM_COLOR, CD_COLOR, CL_COLOR];
        let series: Vec<(&str, [f32; 4], Vec<f64>)> =
            (0..4).map(|i| (labels[i].as_str(), cols[i], xs.iter().map(|x| (x[i] * k[i]).clamp(-50.0, 50.0)).collect())).collect();
        let avail = ui.content_region_avail();
        plot(ui, "sim", [avail[0], avail[1] - 6.0], t, &series, 4, "Time (s)");
    }

    fn tab_transonic(&mut self, ui: &Ui, m: &Arc<Model>) {
        let b = &m.body;
        if let Some(_c) = left_pane(ui) {
            heading(ui, "Fuselage");
            kv(ui, "Length", &format!("{:.2} m", b.length));
            kv(ui, "Fineness", &format!("{:.2}", b.fineness));
            kv(ui, "Max section", &format!("{:.3} m²", b.max_area));
            kv(ui, "Wave drag area", &format!("{:.3} m²", b.sonic_drag_area));
            if ui.is_item_hovered() {
                ui.tooltip_text("Drag area from shock waves at Mach 1. A smooth cross-section curve with little curvature keeps it low.");
            }
            kv(ui, "Critical Mach", &format!("{:.3}", b.critical_mach));
            if 0.75 * b.sonic_drag_area >= b.max_area && b.max_area > 0.0 {
                ui.text_colored(WARN, "High wave drag");
                if ui.is_item_hovered() {
                    ui.tooltip_text("Wave drag is high for this cross-section: smooth the area curve.");
                }
            }
            heading(ui, "Curves");
            for (i, (name, col, on)) in
                [("Cross-section", LD_COLOR, &mut self.show_area), ("Curvature", CM_COLOR, &mut self.show_curv), ("Pressure (Mach 1)", CL_COLOR, &mut self.show_cp)]
                    .into_iter()
                    .enumerate()
            {
                ui.checkbox(format!("##curve{i}"), on);
                ui.same_line();
                swatch(ui, col, name);
            }
        }
        let Some(_c) = right_pane(ui) else { return };
        if b.back <= b.front {
            ui.text_colored(DIM, "No fuselage");
            return;
        }
        // Slices are stored nose first; plot tail to nose so x increases to the right.
        let slices: Vec<&crate::body::Xsec> = b.xs[b.front.saturating_sub(1)..(b.back + 2).min(b.xs.len())].iter().rev().collect();
        let xs: Vec<f64> = slices.iter().map(|s| s.x).collect();
        // Curvature and pressure scaled to the area curve's range, like FAR's overlay.
        let amax = b.max_area.max(1e-6);
        let cmax = slices.iter().map(|s| s.second_deriv.abs()).fold(1e-9, f64::max);
        let mut series = Vec::new();
        if self.show_area {
            series.push(("Cross-section (m²)", LD_COLOR, slices.iter().map(|s| s.area).collect::<Vec<_>>()));
        }
        if self.show_curv {
            series.push(("Curvature", CM_COLOR, slices.iter().map(|s| s.second_deriv / cmax * amax).collect()));
        }
        if self.show_cp {
            series.push(("Pressure (Mach 1)", CL_COLOR, slices.iter().map(|s| s.cp_sonic_fwd * amax).collect()));
        }
        let n = series.len();
        if self.show_area {
            // Unsmoothed voxel area, dimmed.
            series.push(("Raw area (m²)", [0.25, 0.45, 0.25, 1.0], slices.iter().map(|s| s.raw_area).collect()));
        }
        let avail = ui.content_region_avail();
        plot(ui, "area", [avail[0], avail[1] - 6.0], &xs, &series, n, "Position, tail to nose (m)");
    }

    /// Tool choice with its role / aerofoil pickers.
    fn tool_controls(&mut self, ui: &Ui) {
        const TOOLS: [(Tool, &str, &str); 5] = [
            (Tool::Inspect, "Inspect", "Hover a block for its details. Left-drag rotates, right-drag pans, the wheel zooms."),
            (Tool::Wing, "Wing blocks", "Click a block to apply what the wing shelf picked: toggle, fill slab, fill body or remove slab."),
            (Tool::Role, "Control role", "Click a vanilla control surface, or any block of a hinged sub-grid, to give it the role."),
            (Tool::Foil, "Aerofoil", "Click a wing to give it, and the surfaces on it, the aerofoil picked on the side shelf."),
            (Tool::Split, "Separate wing", "Click a wing panel to make it a separate wing (lighter colour), or back."),
        ];
        let tool_before = self.tool;
        heading(ui, "Tool");
        for (t, name, tip) in TOOLS {
            if ui.radio_button_bool(name, self.tool == t) {
                self.tool = t;
            }
            if ui.is_item_hovered() {
                ui.tooltip_text(tip);
            }
        }
        if (tool_before == Tool::Foil) != (self.tool == Tool::Foil) {
            self.view_dirty = true; // panel colours switch between roles and aerofoils
        }
        if self.tool == Tool::Role {
            field(ui, "Role");
            ui.combo_simple_string("##role", &mut self.role_pick, &["None", "Pitch", "Roll", "Yaw", "Flap"]);
        }
        if self.tool == Tool::Foil {
            // Defaults per category; the pick itself comes from the side shelves.
            let of = |k: FoilKind| -> Vec<usize> { (0..AEROFOILS.len()).filter(|&i| AEROFOILS[i].kind == k).collect() };
            let (wings, fins) = (of(FoilKind::Wing), of(FoilKind::Fin));
            let names = |l: &[usize]| -> Vec<&str> { l.iter().map(|&i| AEROFOILS[i].name).collect() };
            heading(ui, "Unmarked wings");
            field(ui, "Wings");
            let mut d = wings.iter().position(|&i| i == self.des.default_foil).unwrap_or(0);
            if ui.combo_simple_string("##foil_default", &mut d, &names(&wings)) && wings[d] != self.des.default_foil {
                self.push_undo();
                self.des.default_foil = wings[d];
                self.designations_changed();
            }
            field(ui, "Fins");
            let fin_now = self.des.default_fin.unwrap_or(crate::wing::FIN_DEFAULT);
            let mut d = fins.iter().position(|&i| i == fin_now).unwrap_or(0);
            if ui.combo_simple_string("##fin_default", &mut d, &names(&fins)) && fins[d] != fin_now {
                self.push_undo();
                self.des.default_fin = Some(fins[d]);
                self.designations_changed();
            }
            let a = &AEROFOILS[self.foil_pick];
            heading(ui, "Picked");
            ui.text(a.name);
            if ui.is_item_hovered() {
                ui.tooltip_text(a.note);
            }
            kv(ui, "Thickness", &format!("{:.0}%", a.t_c * 100.0));
            kv(ui, "CL at 0°", &format!("{:.2}", a.cl0));
            kv(ui, "CL max", &format!("{:.2}", a.cl_max));
            kv(ui, "Drag rise", &format!("Mach {:.2}", a.kappa - a.t_c));
        }
    }

    /// Face of the workbench vehicle under a view point (normalised, x right, y down, -1..1),
    /// through the editor camera (CPU id render at half resolution; only run on a click).
    fn pick_world(&self, at: [f32; 2], display: [f32; 2]) -> Option<usize> {
        let clip = (*editor::WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner()))?;
        let (w, h) = (((display[0] / 2.0) as usize).max(16), ((display[1] / 2.0) as usize).max(16));
        let (fx, fy) = ((at[0] + 1.0) * 0.5 * w as f32, (at[1] + 1.0) * 0.5 * h as f32);
        if !(0.0..w as f32).contains(&fx) || !(0.0..h as f32).contains(&fy) {
            return None;
        }
        let (x, y) = (fx as usize, fy as usize);
        let img = view::render_with(&self.faces, &|_| [0, 0, 0], &view::Projection::from_clip(clip, w, h), w, h, [0; 4]);
        img.id.get(y * w + x).filter(|&&id| id != 0).map(|&id| id as usize - 1)
    }

    fn tab_voxels(&mut self, ui: &Ui, m: &Arc<Model>) {
        if let Some(_c) = left_pane(ui) {
            self.tool_controls(ui);
            heading(ui, "Colours");
            if self.tool == Tool::Foil {
                for (i, a) in AEROFOILS.iter().enumerate() {
                    swatch(ui, rgb(FOIL_COLORS[i % FOIL_COLORS.len()]), a.name);
                }
            } else {
                for (name, c) in [
                    ("Body", [150u8, 150, 150]),
                    ("Wing", [90, 150, 230]),
                    ("Pitch", [240, 140, 40]),
                    ("Roll", [220, 70, 220]),
                    ("Yaw", [70, 210, 90]),
                    ("Flap", [230, 220, 60]),
                    ("Marked, no panel", [40, 60, 150]),
                ] {
                    swatch(ui, rgb(c), name);
                }
            }
            ui.dummy([0.0, 6.0]);
            if ui.button_with_size("Reset view", [-1.0, 0.0]) {
                self.frame_camera(m);
                self.view_dirty = true;
            }
        }
        let Some(_c) = right_pane(ui) else { return };
        let avail = ui.content_region_avail();
        let aspect = VIEW_H as f32 / VIEW_W as f32;
        let w = avail[0].min((avail[1] - 40.0) / aspect).max(200.0);
        let size = [w, w * aspect];
        let origin = ui.cursor_screen_pos();
        if let Some(tex) = self.tex {
            imgui::Image::new(tex, size).build(ui);
            ui.set_cursor_screen_pos(origin);
        }
        // An Image is not an item that takes the mouse, so a drag on it moved the window:
        // an invisible button over it claims clicks and drags.
        ui.invisible_button("##view_input", size);
        let hovered = ui.is_item_hovered();
        let io = ui.io();
        let to_px = |p: [f32; 2]| {
            let x = (p[0] - origin[0]) / size[0] * VIEW_W as f32;
            let y = (p[1] - origin[1]) / size[1] * VIEW_H as f32;
            (x >= 0.0 && y >= 0.0 && x < VIEW_W as f32 && y < VIEW_H as f32).then_some((x as usize, y as usize))
        };
        if hovered {
            let d = io.mouse_delta;
            if ui.is_mouse_dragging(MouseButton::Left) && (d[0] != 0.0 || d[1] != 0.0) {
                self.cam.yaw += d[0] as f64 * 0.01;
                self.cam.pitch = (self.cam.pitch + d[1] as f64 * 0.01).clamp(-1.55, 1.55);
                self.dragged = true;
                self.view_dirty = true;
            }
            if ui.is_mouse_dragging(MouseButton::Right) && (d[0] != 0.0 || d[1] != 0.0) {
                let pr = self.cam.projection(VIEW_W, VIEW_H);
                let k = self.cam.dist * 0.0016;
                let (r, u) = pr.axes();
                self.cam.target = self.cam.target - r * (d[0] as f64 * k) + u * (d[1] as f64 * k);
                self.view_dirty = true;
            }
            if io.mouse_wheel != 0.0 {
                self.cam.dist = (self.cam.dist * (1.0 - io.mouse_wheel as f64 * 0.1)).clamp(1.0, 2000.0);
                self.view_dirty = true;
            }
            let hover = to_px(io.mouse_pos).and_then(|(x, y)| self.img.as_ref().map(|i| i.id[y * i.w + x])).filter(|&id| id != 0).map(|id| id as usize - 1);
            if hover != self.hover_face {
                self.hover_face = hover;
                self.view_dirty = true;
            }
            if ui.is_mouse_released(MouseButton::Left) {
                if !self.dragged {
                    if let Some(f) = self.hover_face {
                        self.apply_tool(m, f);
                    }
                }
                self.dragged = false;
            }
        } else if self.hover_face.is_some() {
            self.hover_face = None;
            self.view_dirty = true;
        }

        // Markers: CoM (yellow) and aerodynamic centre (cyan).
        let pr = self.cam.projection(VIEW_W, VIEW_H);
        let dl = ui.get_window_draw_list();
        let marker = |p: V3, col: [f32; 4], label: &str| {
            let (x, y, z) = pr.project(p);
            if z > 0.0 {
                let s = [origin[0] + x as f32 / VIEW_W as f32 * size[0], origin[1] + y as f32 / VIEW_H as f32 * size[1]];
                dl.add_circle(s, 6.0, col).filled(true).build();
                dl.add_circle(s, 6.0, [0.0, 0.0, 0.0, 1.0]).build();
                dl.add_text([s[0] + 8.0, s[1] - 7.0], col, label);
            }
        };
        marker(m.com, [1.0, 0.9, 0.1, 1.0], "CoM");
        if let Some(np) = m.neutral_point(0.2, 0.0) {
            marker(np, [0.2, 0.9, 1.0, 1.0], "CoL");
        }
        drop(dl);

        // Hovered part.
        if let Some(f) = self.hover_face.and_then(|f| self.faces.get(f)) {
            let c = &m.comps[f.comp as usize];
            ui.text(format!("{}  {}, {}, {}  body {}", c.def, c.pos[0], c.pos[1], c.pos[2], c.body));
            if let Some(p) = face_panel(m, f).and_then(|p| m.panels.get(p)) {
                ui.text_colored(
                    DIM,
                    format!(
                        "Panel  area {:.2} m²  semispan {:.2} m  MAC {:.2} m  AR {:.2}  taper {:.2}  sweep {:.0}°  {}  {}",
                        p.area,
                        p.b,
                        p.mac,
                        p.ar,
                        p.taper,
                        p.tan_sweep.atan().to_degrees(),
                        p.role.name(),
                        AEROFOILS[p.foil.min(AEROFOILS.len() - 1)].name
                    ),
                );
            }
        }
    }

    fn apply_tool(&mut self, m: &Arc<Model>, face: usize) {
        let Some(f) = self.faces.get(face) else {
            return;
        };
        let comp = &m.comps[f.comp as usize];
        let before = self.des.clone();
        match self.tool {
            Tool::Inspect => return,
            Tool::Wing => match crate::toolbar::WING_MODE.load(Ordering::Relaxed) {
                // Fill body: every block of the clicked body, however thick (a flying wing).
                2 => self.des.wings.extend(m.comps.iter().filter(|c| c.body == comp.body).map(|c| c.pos)),
                3 => {
                    let marked = self.des.wings.clone();
                    for pos in slab(m, f, Some(&marked)) {
                        self.des.wings.remove(&pos);
                    }
                }
                1 => {
                    let add = slab(m, f, None);
                    if add.is_empty() {
                        self.status = "nothing to fill there (too thick, or inside a solid body): use Fill body".into();
                    }
                    self.des.wings.extend(add);
                }
                _ => {
                    if !self.des.wings.remove(&comp.pos) {
                        self.des.wings.insert(comp.pos);
                    }
                }
            },
            Tool::Foil => {
                let Some(pi) = face_panel(m, f) else {
                    self.status = "that block is not part of a lifting panel".into();
                    return;
                };
                let p = &m.panels[pi];
                if p.foil == crate::wing::GRID {
                    self.status = "grid fins always use the grid fin section".into();
                    return;
                }
                // The whole wing: its panels and the surfaces on it.
                let targets: Vec<usize> = (0..m.panels.len()).filter(|&j| m.wing_of[j] == m.wing_of[pi]).collect();
                let _ = p;
                for j in targets {
                    // One mark per panel: drop the old ones, then mark one of its blocks.
                    let owners: Vec<[i32; 3]> = match m.fin_comp[j] {
                        Some(c) => vec![m.comps[c as usize].pos],
                        None => m.panels[j].cells.iter().filter_map(|&c| m.comps.get(m.grid.owner[c] as usize)).map(|o| o.pos).collect(),
                    };
                    for pos in &owners {
                        self.des.foils.remove(pos);
                    }
                    let default = if m.fin_comp[j].is_some() { self.des.default_fin.unwrap_or(crate::wing::FIN_DEFAULT) } else { self.des.default_foil };
                    if self.foil_pick != default {
                        let at = if j == pi { comp.pos } else { owners[0] };
                        self.des.foils.insert(at, self.foil_pick);
                    }
                }
            }
            Tool::Split => {
                let Some(&pi) = m.panel_of.get(f.cell).filter(|&&p| p != usize::MAX) else {
                    self.status = "that block is not part of a wing panel".into();
                    return;
                };
                // The whole panel: unmark it if it is marked, else mark all its blocks.
                let owners: Vec<[i32; 3]> = m.panels[pi].cells.iter().filter_map(|&c| m.comps.get(m.grid.owner[c] as usize)).map(|o| o.pos).collect();
                if m.separate.get(pi).copied().unwrap_or(false) {
                    for p in &owners {
                        self.des.splits.remove(p);
                    }
                } else {
                    self.des.splits.extend(owners);
                }
            }
            Tool::Role => {
                let role = Role::ALL[self.role_pick.min(4)];
                // One entry per body (or per vanilla control surface): drop older ones first.
                let body = comp.body;
                let is_cs = self.lib.lock().map(|mut l| l.get(&comp.def).is_some_and(|d| d.is_control_surface())).unwrap_or(false);
                if is_cs {
                    self.des.roles.remove(&comp.pos);
                } else {
                    let same_body: Vec<[i32; 3]> = m.comps.iter().filter(|c| c.body == body).map(|c| c.pos).collect();
                    for p in same_body {
                        self.des.roles.remove(&p);
                    }
                }
                if role != Role::None {
                    self.des.roles.insert(comp.pos, role);
                }
            }
        }
        if self.des != before {
            self.redo.clear();
            self.undo.push(before);
            if self.undo.len() > 200 {
                self.undo.remove(0);
            }
            self.designations_changed();
        }
    }

    fn tab_props(&mut self, ui: &Ui, m: &Arc<Model>) {
        let n = m.props.len();
        if self.prop_rps.len() != n {
            // Default: a 200 m/s blade tip, a typical real one.
            self.prop_rps = (0..n).map(|k| ((200.0 / (std::f64::consts::TAU * m.prop_geom(k, 2, 0.5).radius)) * 10.0).round() as f32 / 10.0).collect();
        }
        self.prop_peaks.resize(n, None);
        let cols = [
            ("Part", 0.0),
            ("Blades", 52.0),
            ("Radius", 60.0),
            ("Blade speed", 96.0),
            ("Best pitch", 78.0),
            ("Blade angle", 82.0),
            ("Thrust", 72.0),
            ("Power", 72.0),
        ];
        let Some(_t) = table(ui, "##rotors", &cols) else { return };
        headers(ui, &cols, 1);
        for (k, u) in m.props.iter().enumerate() {
            let c = &m.comps[u.comp];
            let set = c.prop.as_ref();
            let blades = set.and_then(|p| p.blades).unwrap_or(4).clamp(2, 8);
            let length = set.and_then(|p| p.length).unwrap_or(0.5);
            let setting = set.and_then(|p| p.pitch).unwrap_or(u.def.default_pitch as f64);
            let input = if u.def.rotor() { "collective" } else { "pitch input" };
            ui.table_next_row();
            ui.table_set_column_index(0);
            ui.align_text_to_frame_padding();
            ui.text(&c.def);
            ui.same_line();
            ui.text_colored(DIM, format!("{}, {}, {}", c.pos[0], c.pos[1], c.pos[2]));
            ui.table_set_column_index(1);
            right(ui, &blades.to_string(), None);
            ui.table_set_column_index(2);
            right(ui, &format!("{:.2} m", m.prop_geom(k, blades, length).radius), None);
            ui.table_set_column_index(3);
            ui.set_next_item_width(-1.0);
            ui.input_float(format!("##rps{k}"), &mut self.prop_rps[k]).display_format("%.1f RPS").build();
            let rps = self.prop_rps[k];
            if self.prop_peaks[k].is_none_or(|(r, _)| r != rps) {
                self.prop_peaks[k] = Some((rps, m.prop_peak(k, blades, length, rps as f64)));
            }
            let Some((p, thrust, power, _)) = self.prop_peaks[k].and_then(|p| p.1) else {
                ui.table_set_column_index(4);
                right(ui, "-", Some(DIM));
                if ui.is_item_hovered() {
                    ui.tooltip_text("No thrust peak at this blade speed.");
                }
                continue;
            };
            let need = p - setting;
            let out = !(-1.0..=1.0).contains(&need);
            let vals = [format!("{need:.2}"), format!("{:.1}°", (0.75 * p).to_degrees()), format!("{:.1} kN", thrust / 1000.0), format!("{:.0} kW", power / 1000.0)];
            for (i, v) in vals.iter().enumerate() {
                ui.table_set_column_index(4 + i);
                right(ui, v, if i > 0 { None } else if out { Some(WARN) } else { Some(CL_COLOR) });
                if i == 0 && ui.is_item_hovered() {
                    let offset = if setting != 0.0 { format!(" with blade_pitch {setting:+.0}") } else { String::new() };
                    let range = if out { "\nOutside the input's -1 to 1 range with this blade_pitch setting." } else { "" };
                    ui.tooltip_text(format!(
                        "The {input}{offset} that gives the most static thrust (standing still, sea level). More than this stalls the blades: less thrust for more power.{range}"
                    ));
                }
            }
        }
    }

    /// Global settings, and the workbench vehicle's own (stored on its parts like the marks).
    fn tab_settings(&mut self, ui: &Ui) {
        const FLY_TIP: &str = "Replaces the game's air forces with this plugin's model (wings with their aerofoils and designations, fuselage drag and lift, propellers and rotors blade by blade). Water forces are unchanged. Off: the game's own aerodynamics.";
        const VRS_TIP: &str = "Rotors sinking slowly and almost straight down into their own downwash lose lift: the descent speeds up, and more collective makes it worse. Fly out forwards or sideways to recover. Off: rotors stay steady in any descent.";
        const BLUR_TIP: &str = "Fast propeller and rotor blades are drawn as a faint stippled smear over the arc they sweep in about 1/20 s, like a camera or your eye sees them. Off: sharp blades.";
        let hooked = crate::flight::HOOKED.load(Ordering::Relaxed);
        let vehicle = self.snapshot.is_some() && editor::editor_active();
        let cols = [("", 180.0), ("All vehicles", 100.0), ("This vehicle", 150.0)];
        let Some(_t) = table(ui, "##settings", &cols) else { return };
        headers(ui, &cols, 3);
        if ui.is_item_hovered() {
            ui.tooltip_text(if vehicle { "Saved in the vehicle's file; travels with copy and paste." } else { "Open a vehicle in the workbench to give it its own settings." });
        }
        let row = |ui: &Ui, name: &str, tip: &str| {
            ui.table_next_row();
            ui.table_set_column_index(0);
            ui.align_text_to_frame_padding();
            ui.text(name);
            if ui.is_item_hovered() {
                ui.tooltip_text(tip);
            }
            ui.table_set_column_index(1);
        };
        // Per-vehicle override: Global / On / Off.
        let pick = |ui: &Ui, id: &str, cur: Option<bool>, global: bool| -> Option<Option<bool>> {
            ui.table_set_column_index(2);
            let _d = ui.begin_disabled(!vehicle);
            let names = [if global { "Global (on)" } else { "Global (off)" }, "On", "Off"];
            let mut i = match cur {
                None => 0,
                Some(true) => 1,
                Some(false) => 2,
            };
            ui.set_next_item_width(-1.0);
            ui.combo_simple_string(id, &mut i, &names).then_some([None, Some(true), Some(false)][i])
        };

        row(ui, "Plugin flight model", if hooked { FLY_TIP } else { "Unavailable: the game's flight code could not be hooked, see aero.log." });
        let mut fly = crate::flight::ENABLED.load(Ordering::Relaxed);
        {
            let _d = ui.begin_disabled(!hooked);
            if ui.checkbox("##fly", &mut fly) {
                crate::flight::ENABLED.store(fly, Ordering::Relaxed);
                log_line(&format!("flight model {}", if fly { "on" } else { "off" }));
            }
        }
        let new_fly = pick(ui, "##fly_vehicle", self.des.fly, fly);

        row(ui, "Vortex ring state", VRS_TIP);
        let mut vrs = crate::flight::VRS.load(Ordering::Relaxed);
        if ui.checkbox("##vrs", &mut vrs) {
            crate::flight::VRS.store(vrs, Ordering::Relaxed);
            log_line(&format!("vortex ring state {}", if vrs { "on" } else { "off" }));
        }
        let new_vrs = pick(ui, "##vrs_vehicle", self.des.vrs, vrs);

        row(ui, "Rotor blur", BLUR_TIP);
        let mut blur = crate::blur::ENABLED.load(Ordering::Relaxed);
        if ui.checkbox("##blur", &mut blur) {
            crate::blur::ENABLED.store(blur, Ordering::Relaxed);
        }

        if new_fly.is_some_and(|f| f != self.des.fly) || new_vrs.is_some_and(|v| v != self.des.vrs) {
            self.push_undo();
            self.des.fly = new_fly.unwrap_or(self.des.fly);
            self.des.vrs = new_vrs.unwrap_or(self.des.vrs);
            self.designations_changed();
        }
    }

    fn push_undo(&mut self) {
        self.redo.clear();
        self.undo.push(self.des.clone());
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
    }

    fn undo_designation(&mut self) {
        match self.undo.pop() {
            Some(d) => {
                self.redo.push(std::mem::replace(&mut self.des, d));
                self.designations_changed();
            }
            None => self.status = "nothing to undo".into(),
        }
    }

    fn redo_designation(&mut self) {
        match self.redo.pop() {
            Some(d) => {
                self.undo.push(std::mem::replace(&mut self.des, d));
                self.designations_changed();
            }
            None => self.status = "nothing to redo".into(),
        }
    }

    /// Wing colour of a grid cell (role, or aerofoil in the aerofoil tool); None for non-wing cells.
    fn cell_color(&self, m: &Model, cell: usize) -> Option<[u8; 3]> {
        let o = m.grid.owner[cell];
        match m.panel_of[cell] {
            // A missile part: its body is no wing; it stands for its fins only when they are not
            // separate solids (fins in its static mesh).
            usize::MAX if o != EMPTY && o != INTERIOR && m.fin_of.contains_key(&o) => m.fin_of.get(&o).filter(|f| !f.1).map(|f| self.panel_color(m, f.0)),
            usize::MAX if o != EMPTY && o != INTERIOR && self.des.wings.contains(&m.comps[o as usize].pos) => Some([40, 60, 150]),
            usize::MAX => None,
            p => Some(self.panel_color(m, p)),
        }
    }

    fn panel_color(&self, m: &Model, p: usize) -> [u8; 3] {
        let c = match p {
            p if self.tool == Tool::Foil => FOIL_COLORS[m.panels[p].foil % FOIL_COLORS.len()],
            p => match m.panels[p].role {
                Role::None => [90, 150, 230],
                Role::Pitch => [240, 140, 40],
                Role::Roll => [220, 70, 220],
                Role::Yaw => [70, 210, 90],
                Role::Flap => [230, 220, 60],
            },
        };
        // Separate wings in a lighter shade, so the split shows.
        if m.separate.get(p).copied().unwrap_or(false) {
            c.map(|v| v + (255 - v) / 2)
        } else {
            c
        }
    }

    fn face_color(&self, m: &Model, face: usize, hover_comp: Option<u32>) -> [u8; 3] {
        let cell = self.faces[face].cell;
        let o = self.faces[face].comp;
        let fin = self.faces.get(face).and_then(|f| m.fin_of.get(&f.comp).filter(|&&(_, moving)| f.moving || !moving)).map(|&(p, _)| self.panel_color(m, p));
        let mut c = fin.or_else(|| self.cell_color(m, cell)).unwrap_or_else(|| {
            let b = if o != EMPTY && o != INTERIOR { m.comps[o as usize].body } else { 0 };
            if b == 0 {
                [150, 150, 150]
            } else {
                [135, 145, 160]
            }
        });
        if hover_comp == Some(o) {
            c = c.map(|v: u8| v.saturating_add(70));
        }
        c
    }

    fn refresh_view(&mut self) {
        let Some(m) = self.model.clone() else { return };
        if !self.view_dirty || self.upload {
            return;
        }
        self.view_dirty = false;
        let hover_comp = self.hover_face.and_then(|f| self.faces.get(f)).map(|f| f.comp);
        let color = |face: usize| self.face_color(&m, face, hover_comp);
        let faces = self.faces.clone();
        // Face index as the id, so picking knows which side was clicked.
        let img = view::render(&faces, &|i| color(i), &self.cam, VIEW_W, VIEW_H);
        self.img = Some(img);
        self.upload = true;
    }
}

/// Grid positions of the components in the wing slab around a clicked face (see
/// `voxel::slab_cells`). Adding (`marked` None): thin columns of the clicked body, not buried
/// inside a solid one.
/// Removing: the marked blocks of that body in the clicked plane, however thick.
fn slab(m: &Model, f: &Face, marked: Option<&std::collections::HashSet<[i32; 3]>>) -> Vec<[i32; 3]> {
    let g = &m.grid;
    let n = f.normal;
    // Body-frame normal -> its dominant grid axis.
    let gnf = [n.y, -n.z, n.x];
    let axis = (0..3).max_by(|&i, &j| gnf[i].abs().total_cmp(&gnf[j].abs())).unwrap_or(1);
    let owner_ok = |i: usize| g.owner[i] != EMPTY && g.owner[i] != INTERIOR;
    if !owner_ok(f.cell) {
        return Vec::new();
    }
    let body = m.comps[f.comp as usize].body;
    let member = |i: usize| owner_ok(i) && m.comps[g.owner[i] as usize].body == body && marked.is_none_or(|s| s.contains(&m.comps[g.owner[i] as usize].pos));
    let cells = crate::voxel::slab_cells(g, f.cell, axis, &member, marked.is_none());
    let out: std::collections::HashSet<[i32; 3]> = cells.iter().map(|&i| m.comps[g.owner[i] as usize].pos).collect();
    out.into_iter().collect()
}

/// F9 table: every table the same width, columns split by weight.
fn f9_table<'a>(ui: &'a Ui, id: &str, cols: &[(&str, f32)]) -> Option<imgui::TableToken<'a>> {
    let flags = TableFlags::ROW_BG | TableFlags::BORDERS_INNER_H | TableFlags::PAD_OUTER_X | TableFlags::SIZING_STRETCH_PROP;
    let t = ui.begin_table_with_sizing(id, cols.len(), flags, [560.0, 0.0], 0.0)?;
    for &(name, w) in cols {
        ui.table_setup_column_with(TableColumnSetup { flags: TableColumnFlags::WIDTH_STRETCH, init_width_or_weight: w, ..TableColumnSetup::new(name) });
    }
    Some(t)
}

/// One vehicle's block in the F9 flight data: label / value pairs in two columns, then its
/// propellers and rotors.
fn flight_vehicle(ui: &Ui, n: usize, v: &crate::flight::VehicleDebug) {
    type Cell = (&'static str, String, Option<[f32; 4]>);
    let rows = |id: &str, rows: &[[Cell; 2]]| {
        let cols = [("", 1.0), ("", 1.0), ("", 1.15), ("", 0.85)];
        let Some(_t) = f9_table(ui, &format!("{id}{n}"), &cols) else { return };
        for row in rows {
            ui.table_next_row();
            for (j, (k, val, col)) in row.iter().enumerate() {
                ui.table_set_column_index(2 * j);
                ui.text_colored(DIM, k);
                ui.table_set_column_index(2 * j + 1);
                right(ui, val, *col);
            }
        }
    };
    // kN, and the weight of that many kg.
    let kn = |f: f64| format!("{:.1} kN", f / 1000.0);
    let sea = f64::from_bits(crate::model::SEA_LEVEL_RHO.load(Ordering::Relaxed)).max(1e-6);
    heading(ui, "Air");
    rows(
        "##air",
        &[
            [("Airspeed", format!("{:.1} m/s", v.airspeed), None), ("Mach", format!("{:.2}", v.mach), None)],
            [("", format!("{:.0} km/h", v.airspeed * 3.6), Some(DIM)), ("", format!("{:.0} kt", v.airspeed * 1.943_84), Some(DIM))],
            [("Altitude", format!("{:.0} m", v.alt), None), ("Dyn. pressure", format!("{:.2} kPa", v.q / 1000.0), None)],
            [("Density", format!("{:.3} kg/m³", v.rho), None), ("", format!("{:.0}% sea level", v.rho / sea * 100.0), Some(DIM))],
            [("AoA", format!("{:.1}°", v.aoa.to_degrees()), None), ("Sideslip", format!("{:.1}°", v.beta.to_degrees()), None)],
        ],
    );
    // Coefficients are lift and drag over q·S: meaningless near standstill (hover, taxi).
    let slow = v.airspeed < 5.0;
    let coef = |c: f64, d: usize| if slow { "-".to_string() } else { format!("{c:.d$}") };
    let ld = if v.drag > 1e-6 && !slow { format!("{:.1}", v.lift / v.drag) } else { "-".into() };
    let w = v.weight.max(1e-9);
    let thrust = v.thrust_exact.unwrap_or(v.thrust);
    heading(ui, "Wings and body");
    rows(
        "##aero",
        &[
            [("CL", coef(v.cl, 3), None), ("CD", coef(v.cd, 4), None)],
            [("Lift", kn(v.lift), None), ("L/D", ld, None)],
            [("Drag", kn(v.drag), None), ("Lift / weight", format!("{:.2}", v.lift / w), None)],
        ],
    );
    heading(ui, "Vehicle");
    let mut vehicle = vec![
        [("Mass", format!("{:.2} t", v.mass / 1000.0), None), ("Physics step", format!("{:.1} ms", v.step * 1000.0), Some(DIM))],
        [("Thrust", kn(thrust), None), ("Thrust / weight", format!("{:.2}", thrust / w), None)],
    ];
    if let Some(sm) = v.margin {
        let pct = sm / v.mac.max(1e-6) * 100.0;
        let col = if sm >= 0.0 { GOOD } else { BAD };
        vehicle.push([("Static margin", format!("{pct:.0}% MAC"), Some(col)), ("", if sm >= 0.0 { "stable".into() } else { "unstable".into() }, Some(col))]);
    }
    if let Some((h, phi)) = v.ground.filter(|g| g.1 < 0.99) {
        vehicle.push([("Ground effect", format!("{h:.1} m"), None), ("Induced drag", format!("×{phi:.2}"), None)]);
    }
    rows("##veh", &vehicle);
    ui.text_colored(DIM, format!("Thrust from {}", if v.thrust_source.is_empty() { "nothing" } else { &v.thrust_source }));
    if v.stalled > 0 {
        swatch(ui, BAD, &format!("Stall: {} of {} panels", v.stalled, v.panels));
    }
    if v.props.is_empty() {
        return;
    }
    heading(ui, "Propellers and rotors");
    let cols = [("", 1.5), ("Shaft RPS", 0.8), ("Blade RPS", 0.8), ("Angle", 0.55), ("Thrust", 0.7), ("Power", 0.7), ("", 0.85)];
    let Some(_t) = f9_table(ui, &format!("##props{n}"), &cols) else { return };
    headers(ui, &cols, 1);
    for p in &v.props {
        ui.table_next_row();
        ui.table_set_column_index(0);
        ui.text(&p.name);
        ui.table_set_column_index(1);
        right(ui, &format!("{:.1}", p.shaft), None);
        ui.table_set_column_index(2);
        right(ui, &format!("{:.1}", p.rps), None);
        ui.table_set_column_index(3);
        right(ui, &format!("{:.1}°", p.blade_deg), if p.stalled > 0.05 { Some(BAD) } else { None });
        ui.table_set_column_index(4);
        right(ui, &kn(p.thrust), None);
        ui.table_set_column_index(5);
        right(ui, &format!("{:.0} kW", p.power / 1000.0), None);
        ui.table_set_column_index(6);
        // A few % is one azimuth step of the outer blade; more is a real stall.
        if p.stalled > 0.05 {
            right(ui, &format!("Stall {:.0}%", p.stalled * 100.0), Some(BAD));
        } else if p.vrs > 0.05 {
            right(ui, &format!("Vortex ring {:.0}%", p.vrs * 100.0), Some(WARN));
        }
    }
}

// ---------- layout helpers ----------

/// Section title with a rule to the right edge.
fn heading(ui: &Ui, title: &str) {
    ui.dummy([0.0, 2.0]);
    ui.text(title);
    let (l, r) = (ui.item_rect_min(), ui.item_rect_max());
    let right = ui.window_pos()[0] + ui.window_content_region_max()[0];
    let y = (l[1] + r[1]) * 0.5;
    ui.get_window_draw_list().add_line([r[0] + 8.0, y], [right, y], [0.35, 0.37, 0.42, 1.0]).build();
}

/// Small colour square, then the label.
fn swatch(ui: &Ui, col: [f32; 4], label: &str) {
    let p = ui.cursor_screen_pos();
    let h = ui.text_line_height();
    ui.get_window_draw_list().add_rect([p[0], p[1] + 3.0], [p[0] + h - 6.0, p[1] + h - 3.0], col).filled(true).build();
    ui.dummy([h - 6.0, h]);
    ui.same_line_with_spacing(0.0, 6.0);
    ui.text(label);
}

fn rgb(c: [u8; 3]) -> [f32; 4] {
    [c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, 1.0]
}

/// Label on the left, the next item from a fixed column to the right edge.
fn field(ui: &Ui, label: &str) {
    ui.align_text_to_frame_padding();
    ui.text_colored(DIM, label);
    ui.same_line_with_pos(FIELD_X);
    ui.set_next_item_width(-1.0);
}

fn input_f(ui: &Ui, label: &str, v: &mut f32, fmt: &str) -> bool {
    field(ui, label);
    ui.input_float(format!("##{label}"), v).display_format(fmt).build()
}

/// Label on the left, value right-aligned.
fn kv(ui: &Ui, k: &str, v: &str) {
    ui.text_colored(DIM, k);
    ui.same_line_with_pos(ui.window_content_region_max()[0] - ui.calc_text_size(v)[0]);
    ui.text(v);
}

fn sci(v: f64) -> String {
    if v.abs() >= 1e5 {
        let e = v.abs().log10().floor() as i32;
        format!("{:.3}e{e}", v / 10f64.powi(e))
    } else {
        format!("{v:.0}")
    }
}

/// The tab's one action, full width.
fn primary(ui: &Ui, label: &str) -> bool {
    ui.dummy([0.0, 6.0]);
    ui.button_with_size(label, [-1.0, 26.0])
}

/// Controls column on the left; `right_pane` after it takes the rest of the row.
fn left_pane(ui: &Ui) -> Option<imgui::ChildWindowToken<'_>> {
    let h = ui.content_region_avail()[1];
    ui.child_window("##controls").size([LEFT_W, h]).begin()
}

fn right_pane(ui: &Ui) -> Option<imgui::ChildWindowToken<'_>> {
    ui.same_line_with_spacing(0.0, 14.0);
    let h = ui.content_region_avail()[1];
    ui.child_window("##results").size([0.0, h]).begin()
}

/// Text right-aligned in the current table cell.
fn right(ui: &Ui, s: &str, col: Option<[f32; 4]>) {
    let w = ui.calc_text_size(s)[0];
    ui.set_cursor_pos([ui.cursor_pos()[0] + ui.content_region_avail()[0] - w, ui.cursor_pos()[1]]);
    match col {
        Some(c) => ui.text_colored(c, s),
        None => ui.text(s),
    }
}

/// Table with the given columns: a width > 0 is fixed, 0 stretches.
fn table<'a>(ui: &'a Ui, id: &str, cols: &[(&str, f32)]) -> Option<imgui::TableToken<'a>> {
    let t = ui.begin_table_with_flags(id, cols.len(), TableFlags::ROW_BG | TableFlags::BORDERS_INNER_H | TableFlags::SIZING_FIXED_FIT | TableFlags::PAD_OUTER_X)?;
    for &(name, w) in cols {
        let mut c = TableColumnSetup::new(name);
        if w > 0.0 {
            c.flags = TableColumnFlags::WIDTH_FIXED;
            c.init_width_or_weight = w;
        } else {
            c.flags = TableColumnFlags::WIDTH_STRETCH;
        }
        ui.table_setup_column_with(c);
    }
    Some(t)
}

/// Header row: the first `left_cols` titles left-aligned, the rest right-aligned.
fn headers(ui: &Ui, cols: &[(&str, f32)], left_cols: usize) {
    ui.table_next_row_with_flags(TableRowFlags::HEADERS);
    for (i, (name, _)) in cols.iter().enumerate() {
        ui.table_set_column_index(i);
        if i < left_cols {
            ui.text_colored(DIM, name);
        } else {
            right(ui, name, Some(DIM));
        }
    }
}

/// Key points of an AoA sweep, under its plot and as wide as the plot area.
fn sweep_readout(ui: &Ui, x: &[f64], up: &[Coeffs]) {
    let best = |f: &dyn Fn(&Coeffs) -> f64| up.iter().enumerate().map(|(i, c)| (i, f(c))).filter(|p| p.1.is_finite()).fold((0, f64::MIN), |a, b| if b.1 > a.1 { b } else { a });
    let (imax, clmax) = best(&|c| c.cl);
    let (ild, ldmax) = best(&|c| c.cl / c.cd);
    // First upward zero crossing of CL, interpolated.
    let zero = (1..x.len()).find(|&i| up[i - 1].cl <= 0.0 && up[i].cl > 0.0).map(|i| x[i - 1] + (x[i] - x[i - 1]) * up[i - 1].cl / (up[i - 1].cl - up[i].cl));
    let cols = ["CL max", "Stall", "Best L/D", "Zero lift"];
    let vals = [
        format!("{clmax:.3}"),
        format!("{:.1}°", x[imax]),
        format!("{ldmax:.1} at {:.1}°", x[ild]),
        zero.map_or("-".into(), |z| format!("{z:.1}°")),
    ];
    ui.indent_by(PLOT_PAD_L);
    let t = ui.begin_table_with_sizing(
        "##keys",
        cols.len(),
        TableFlags::ROW_BG | TableFlags::BORDERS_INNER_H | TableFlags::PAD_OUTER_X,
        [ui.content_region_avail()[0] - PLOT_PAD_R, 0.0],
        0.0,
    );
    ui.unindent_by(PLOT_PAD_L);
    let Some(_t) = t else { return };
    for name in cols {
        ui.table_setup_column_with(TableColumnSetup { flags: TableColumnFlags::WIDTH_STRETCH, ..TableColumnSetup::new(name) });
    }
    ui.table_next_row_with_flags(TableRowFlags::HEADERS);
    for (i, name) in cols.iter().enumerate() {
        ui.table_set_column_index(i);
        right(ui, name, Some(DIM));
    }
    ui.table_next_row();
    for (i, v) in vals.iter().enumerate() {
        ui.table_set_column_index(i);
        right(ui, v, None);
    }
}

type Deriv = (&'static str, f64, i32, &'static str, &'static str);

/// Stability derivatives: rows are the force or moment, columns the cause. Green when the
/// sign is the stabilising one, red when not; the tooltip names and explains each.
fn deriv_table<const N: usize>(ui: &Ui, id: &str, heads: &[&str], rows: &[(&str, [Deriv; N])]) {
    let cols: Vec<(&str, f32)> = heads.iter().enumerate().map(|(i, h)| (*h, if i == 0 { 80.0 } else { 0.0 })).collect();
    let Some(_t) = ui.begin_table_with_flags(id, cols.len(), TableFlags::ROW_BG | TableFlags::BORDERS_INNER_H | TableFlags::PAD_OUTER_X) else {
        return;
    };
    for (i, (h, w)) in cols.iter().enumerate() {
        let mut c = TableColumnSetup::new(*h);
        c.flags = if i == 0 { TableColumnFlags::WIDTH_FIXED } else { TableColumnFlags::WIDTH_STRETCH };
        c.init_width_or_weight = *w;
        ui.table_setup_column_with(c);
    }
    headers(ui, &cols, 1);
    for (label, vals) in rows {
        ui.table_next_row();
        ui.table_set_column_index(0);
        ui.text(label);
        for (j, &(name, v, sign, unit, tip)) in vals.iter().enumerate() {
            ui.table_set_column_index(j + 1);
            let col = if !v.is_finite() || (sign != 0 && v.signum() as i32 != sign) {
                Some(BAD)
            } else if sign != 0 {
                Some(GOOD)
            } else {
                None
            };
            right(ui, &format!("{v:+.4}"), col);
            if ui.is_item_hovered() {
                ui.tooltip_text(format!("{name} = {v:.4} {unit}\n{tip}"));
            }
        }
    }
}

/// Line plot: grid, zero line, legend along the top, hover readout. Series past `legend`
/// are drawn but not named. Non-finite values break the line.
fn plot(ui: &Ui, id: &str, size: [f32; 2], x: &[f64], series: &[(&str, [f32; 4], Vec<f64>)], legend: usize, xlabel: &str) {
    if x.len() < 2 {
        return;
    }
    let p0 = ui.cursor_screen_pos();
    ui.invisible_button(format!("##plot_{id}"), size);
    let hovered = ui.is_item_hovered();
    let dl = ui.get_window_draw_list();
    let (pad_b, pad_t) = (30.0, 6.0);
    let (x0, x1) = (x[0], x[x.len() - 1]);
    let ys = series.iter().flat_map(|s| s.2.iter().copied()).filter(|v| v.is_finite());
    let (mut y0, mut y1) = ys.fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    if y0 > y1 {
        y0 = -1.0;
        y1 = 1.0;
    }
    if y1 - y0 < 1e-9 {
        y0 -= 1.0;
        y1 += 1.0;
    }
    let step = |lo: f64, hi: f64| {
        let raw = (hi - lo) / 6.0;
        let mag = 10f64.powf(raw.log10().floor());
        [1.0, 2.0, 5.0, 10.0].iter().map(|k| k * mag).find(|s| *s >= raw).unwrap_or(raw)
    };
    let ysx = step(y0, y1);
    y0 = (y0 / ysx).floor() * ysx;
    y1 = (y1 / ysx).ceil() * ysx;
    let lo = [p0[0] + PLOT_PAD_L, p0[1] + pad_t];
    let hi = [p0[0] + size[0] - PLOT_PAD_R, p0[1] + size[1] - pad_b];
    let sx = |v: f64| lo[0] + ((v - x0) / (x1 - x0)) as f32 * (hi[0] - lo[0]);
    let sy = |v: f64| hi[1] - ((v - y0) / (y1 - y0)) as f32 * (hi[1] - lo[1]);
    dl.add_rect(lo, hi, [0.10, 0.11, 0.13, 1.0]).filled(true).build();
    let grid = [0.22, 0.24, 0.28, 1.0];
    let tick = [0.62, 0.64, 0.68, 1.0];
    let xs = step(x0, x1);
    let mut v = (x0 / xs).ceil() * xs;
    while v <= x1 + 1e-9 {
        dl.add_line([sx(v), lo[1]], [sx(v), hi[1]], grid).build();
        let t = fmt_tick(v);
        let w = ui.calc_text_size(&t)[0];
        dl.add_text([sx(v) - w * 0.5, hi[1] + 4.0], tick, &t);
        v += xs;
    }
    let mut v = y0;
    while v <= y1 + 1e-9 {
        let c = if v.abs() < 1e-9 { [0.45, 0.47, 0.52, 1.0] } else { grid };
        dl.add_line([lo[0], sy(v)], [hi[0], sy(v)], c).build();
        let t = fmt_tick(v);
        let w = ui.calc_text_size(&t)[0];
        dl.add_text([lo[0] - 6.0 - w, sy(v) - 7.0], tick, &t);
        v += ysx;
    }
    let w = ui.calc_text_size(xlabel)[0];
    dl.add_text([(lo[0] + hi[0]) * 0.5 - w * 0.5, hi[1] + 17.0], DIM, xlabel);
    // Back to front, so the named series sit on top of the dimmed ones.
    for (_, col, ys) in series.iter().rev() {
        let mut run: Vec<[f32; 2]> = Vec::new();
        for (xv, yv) in x.iter().zip(ys) {
            if yv.is_finite() {
                run.push([sx(*xv), sy(yv.clamp(y0, y1))]);
            } else if run.len() > 1 {
                dl.add_polyline(std::mem::take(&mut run), *col).thickness(1.5).build();
            } else {
                run.clear();
            }
        }
        if run.len() > 1 {
            dl.add_polyline(run, *col).thickness(1.5).build();
        }
    }
    let mut lx = lo[0] + 10.0;
    for (name, col, _) in series.iter().take(legend) {
        dl.add_line([lx, lo[1] + 12.0], [lx + 14.0, lo[1] + 12.0], *col).thickness(2.0).build();
        dl.add_text([lx + 19.0, lo[1] + 5.0], [0.85, 0.86, 0.88, 1.0], name);
        lx += 19.0 + ui.calc_text_size(name)[0] + 18.0;
    }
    if hovered {
        let mx = ui.io().mouse_pos[0];
        let t = ((mx - lo[0]) / (hi[0] - lo[0])).clamp(0.0, 1.0) as f64;
        let i = ((t * (x.len() - 1) as f64).round() as usize).min(x.len() - 1);
        dl.add_line([sx(x[i]), lo[1]], [sx(x[i]), hi[1]], [1.0, 1.0, 1.0, 0.4]).build();
        drop(dl);
        let mut text = format!("{xlabel}: {:.3}", x[i]);
        for (name, _, ys) in series.iter().take(legend) {
            text += &format!("\n{name}: {:.4}", ys[i]);
        }
        ui.tooltip_text(text);
    }
}

fn fmt_tick(v: f64) -> String {
    if v.abs() >= 100.0 || (v - v.round()).abs() < 1e-9 {
        format!("{v:.0}")
    } else if (v * 10.0 - (v * 10.0).round()).abs() < 1e-9 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

impl ImguiRenderLoop for Overlay {
    fn initialize<'a>(&'a mut self, ctx: &mut Context, render_context: &'a mut dyn RenderContext) {
        ctx.set_ini_filename(None);
        let blank = vec![0u8; VIEW_W * VIEW_H * 4];
        match render_context.load_texture(&blank, VIEW_W as u32, VIEW_H as u32) {
            Ok(t) => self.tex = Some(t),
            Err(e) => log_line(&format!("voxel view texture failed: {e:?}")),
        }
        log_line("overlay initialised (F7 toggles the aero analysis window)");
    }

    fn before_render<'a>(&'a mut self, _ctx: &mut Context, render_context: &'a mut dyn RenderContext) {
        if self.upload {
            self.upload = false;
            if let (Some(tex), Some(img)) = (self.tex, self.img.as_ref()) {
                if let Err(e) = render_context.replace_texture(tex, &img.rgba, img.w as u32, img.h as u32) {
                    log_line(&format!("voxel view upload failed: {e:?}"));
                }
            }
        }
    }

    fn render(&mut self, ui: &mut Ui) {
        // A panic here would unwind into the game's wglSwapBuffers and abort it.
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.frame(ui))).is_ok();
        if !ok {
            self.open = false;
            self.set_world(false);
            editor::WANTED.store(false, Ordering::Relaxed);
            log_line("overlay frame panicked; window closed (see the PANIC line above)");
        }
    }

    fn message_filter(&self, io: &Io) -> MessageFilter {
        self.filter(io)
    }
}

impl Overlay {
    fn frame(&mut self, ui: &Ui) {
        let _fault = crate::fault::enter(crate::fault::stage::OVERLAY_RENDER);
        crate::paint::new_frame();
        crate::toolbar::upload_icons();
        // imgui-rs builds the draw-list slice from a null pointer when a frame draws nothing,
        // which panics under debug assertions (and is UB anyway): always emit one invisible quad.
        ui.get_foreground_draw_list().add_rect([-4.0, -4.0], [-3.0, -3.0], [0.0, 0.0, 0.0, 0.01]).filled(true).build();
        if ui.is_key_pressed(Key::F7) {
            self.open = !self.open;
        }
        if ui.is_key_pressed(Key::F9) {
            self.flight_data = !self.flight_data;
        }
        crate::flight::DEBUG_WANTED.store(self.flight_data, Ordering::Relaxed);
        if self.flight_data {
            self.draw_flight_data(ui);
            draw_force_arrows(ui);
        }
        editor::WANTED.store(self.open || self.world, Ordering::Relaxed);
        if self.open || self.world {
            self.pump_model();
        }
        self.bench_mode(ui);
        if self.world {
            self.publish_paint();
        }
        if !self.open {
            return;
        }
        let mut open = self.open;
        ui.window("Aero analysis##aero").size([840.0, 700.0], Condition::FirstUseEver).opened(&mut open).build(|| {
            let io = ui.io();
            if io.key_ctrl && ui.is_key_pressed(Key::Z) && ui.is_window_focused_with_flags(imgui::WindowFocusedFlags::ROOT_AND_CHILD_WINDOWS) {
                self.undo_designation();
            }
            self.header(ui);
            if let Some(_bar) = ui.tab_bar("##aero_tabs") {
                if let Some(m) = self.model.clone() {
                    if let Some(_t) = ui.tab_item("Sweep") {
                        ui.dummy([0.0, 4.0]);
                        self.tab_static(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Stability") {
                        ui.dummy([0.0, 4.0]);
                        self.tab_stability(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Simulation") {
                        ui.dummy([0.0, 4.0]);
                        self.tab_sim(ui);
                    }
                    if let Some(_t) = ui.tab_item("Area rule") {
                        ui.dummy([0.0, 4.0]);
                        self.tab_transonic(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Wings") {
                        ui.dummy([0.0, 4.0]);
                        self.tab_voxels(ui, &m);
                    }
                    if !m.props.is_empty() {
                        if let Some(_t) = ui.tab_item("Rotors") {
                            ui.dummy([0.0, 4.0]);
                            self.tab_props(ui, &m);
                        }
                    }
                }
                if let Some(_t) = ui.tab_item("Settings") {
                    ui.dummy([0.0, 4.0]);
                    self.tab_settings(ui);
                }
            }
        });
        self.open = open;
        self.refresh_view();
    }

    fn filter(&self, io: &Io) -> MessageFilter {
        if !self.open && !self.bench {
            return MessageFilter::empty();
        }
        let mut f = MessageFilter::empty();
        if io.want_capture_mouse {
            f |= MessageFilter::InputMouse | MessageFilter::InputRaw;
        }
        // While the window has focus the workbench gets no keys (Ctrl+Z here must not undo a
        // build step too); click outside the window to give the keyboard back to the game.
        if io.want_text_input || io.want_capture_keyboard {
            f |= MessageFilter::InputKeyboard | MessageFilter::InputRaw;
        }
        f
    }
}

/// Lifting panel a view face belongs to: a missile fin's own solids (or, when its fins are not
/// separate solids, the whole part) pick the fin panel; other faces their cell's panel.
fn face_panel(m: &Model, f: &view::Face) -> Option<usize> {
    if let Some(&(p, moving)) = m.fin_of.get(&f.comp) {
        if f.moving || !moving {
            return Some(p);
        }
    }
    m.panel_of.get(f.cell).copied().filter(|&p| p != usize::MAX)
}

/// Panel colour per aerofoil (index into `AEROFOILS`) in the aerofoil tool.
const FOIL_COLORS: [[u8; 3]; 9] =
    [[90, 150, 230], [230, 120, 40], [80, 200, 90], [200, 80, 200], [230, 220, 60], [60, 210, 210], [255, 255, 255], [240, 90, 110], [170, 170, 255]];

/// FAR-style lift (green, cyan on the fuselage) and drag (red) arrows, one per surface, totals at the CoM.
fn draw_force_arrows(ui: &Ui) {
    // Camera and alpha as the game drew this frame (taken around its draw, not now).
    let frame = crate::flight::frame_snap();
    let Some(vp) = frame.map(|f| f.vp).or_else(crate::flight::main_viewproj) else {
        return;
    };
    let data = crate::flight::DEBUG.lock().map(|d| d.clone()).unwrap_or_default();
    publish_track(&data);
    let size = ui.io().display_size;
    let dl = ui.get_background_draw_list();
    let arrow = |from: V3, vec: V3, col: [f32; 4], thick: f32| {
        let (Some(a), Some(b)) = (view::project(&vp, from, size), view::project(&vp, from + vec, size)) else {
            return;
        };
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 2.0 {
            return;
        }
        let (ux, uy, h) = (dx / len, dy / len, (len * 0.3).min(8.0 + 3.0 * thick));
        let l = [b[0] - ux * h - uy * h * 0.5, b[1] - uy * h + ux * h * 0.5];
        let r = [b[0] - ux * h + uy * h * 0.5, b[1] - uy * h - ux * h * 0.5];
        let base = [b[0] - ux * h * 0.8, b[1] - uy * h * 0.8];
        let outline = [0.0, 0.0, 0.0, 0.6];
        dl.add_line(a, base, outline).thickness(thick + 3.0).build();
        dl.add_triangle(b, l, r, outline).thickness(3.0).build();
        dl.add_line(a, base, col).thickness(thick).build();
        dl.add_triangle(b, l, r, col).filled(true).build();
        dl.add_circle(a, thick * 0.9, col).filled(true).build();
    };
    for v in &data {
        // Logarithmic length: small loads stay visible, high-g supersonic loads don't fill the screen.
        // A surface arrow is 6.4 m at 1× weight, 10.3 m at 3×, 14.9 m at 10×; totals 1.2× those.
        let w = v.weight.max(1.0);
        let scaled = |f: V3, len: f64| {
            let n = f.len();
            if n < 1e-9 {
                V3::ZERO
            } else {
                f * (len * (1.0 + 4.0 * n / w).ln() / n)
            }
        };
        // Grid-local point on `body`, placed with the matrix the game drew it with (world fallback).
        let place_on = |body: usize, local: V3, world: V3| {
            smoothed_matrix(body, frame)
                .map(|m| V3 {
                    x: local.x * m[0] + local.y * m[4] + local.z * m[8] + m[12],
                    y: local.x * m[1] + local.y * m[5] + local.z * m[9] + m[13],
                    z: local.x * m[2] + local.y * m[6] + local.z * m[10] + m[14],
                })
                .filter(|p| (*p - world).len() < 100.0)
                .unwrap_or(world)
        };
        // The CoM rides on the main body (the fuselage arrow's), shifted by the same render offset.
        let mut com_shift = V3::ZERO;
        for a in &v.arrows {
            let placed = smoothed_matrix(a.body, frame).map(|m| {
                let row = |r: usize| V3 { x: m[4 * r], y: m[4 * r + 1], z: m[4 * r + 2] };
                let dir = |d: V3| row(0) * d.x + row(1) * d.y + row(2) * d.z;
                (dir(a.local_at) + row(3), dir(a.local_lift), dir(a.local_drag))
            });
            // Fall back to the physics-tick placement if the render matrix disagrees wildly (wrong frame).
            let (at, lift, drag) = match placed {
                Some(p) if (p.0 - a.at).len() < 100.0 => p, // the drawn pose trails physics by up to one tick
                _ => (a.at, a.lift, a.drag),
            };
            if !a.wing {
                com_shift = at - a.at;
                note_render_motion(at, (at - a.at).len());
            }
            arrow(at, scaled(lift, 4.0), if a.wing { [0.2, 1.0, 0.2, 1.0] } else { [0.2, 0.9, 1.0, 1.0] }, 4.0);
            arrow(at, scaled(drag, 4.0), [1.0, 0.25, 0.2, 1.0], 4.0);
        }
        // CG (yellow), live centre of lift (blue), centre of thrust (magenta) on the main body; the neutral
        // point (stability reference, hollow ring) too.
        let place = |(world, local): (V3, V3)| place_on(v.main_body, local, world);
        let cot = v.thrust_at.map(|(p, _)| p);
        for (pt, fill, label) in [(v.cg, [1.0, 0.85, 0.1, 1.0], "CG"), (v.lift_at, [0.25, 0.6, 1.0, 1.0], "CoL"), (cot, [1.0, 0.3, 1.0, 1.0], "CoT")] {
            let Some(s) = pt.and_then(|p| view::project(&vp, place(p), size)) else {
                continue;
            };
            dl.add_circle(s, 9.0, [0.0, 0.0, 0.0, 0.8]).filled(true).build();
            dl.add_circle(s, 7.0, fill).filled(true).build();
            dl.add_text([s[0] + 11.0, s[1] - 8.0], fill, label);
        }
        if let Some(s) = v.col.and_then(|p| view::project(&vp, place(p), size)) {
            dl.add_circle(s, 7.0, [0.0, 0.0, 0.0, 0.8]).thickness(4.0).build();
            dl.add_circle(s, 7.0, [0.5, 0.9, 1.0, 1.0]).thickness(2.0).build();
            dl.add_text([s[0] + 11.0, s[1] + 2.0], [0.5, 0.9, 1.0, 1.0], "NP");
        }
        if let Some((p, t)) = v.thrust_at {
            arrow(place(p), scaled(t, 4.8), [1.0, 0.3, 1.0, 1.0], 6.0);
        }
        let lift = v.arrows.iter().fold(V3::ZERO, |s, a| s + a.lift);
        let drag = v.arrows.iter().fold(V3::ZERO, |s, a| s + a.drag);
        arrow(v.com + com_shift, scaled(lift, 4.8), [1.0, 1.0, 0.2, 1.0], 6.0);
        arrow(v.com + com_shift, scaled(drag, 4.8), [1.0, 0.6, 0.1, 1.0], 6.0);
    }
}

/// One log line per 600 frames: how often the fuselage arrow's render placement moved between frames
/// (every frame = interpolated, smooth; ~tick rate = the matrix is per physics tick) and its offset from physics.
fn note_render_motion(at: V3, off: f64) {
    static S: Mutex<(V3, u32, u32, f64)> = Mutex::new((V3::ZERO, 0, 0, 0.0));
    let mut s = S.lock().unwrap_or_else(|p| p.into_inner());
    if at != s.0 {
        s.2 += 1;
    }
    s.0 = at;
    s.1 += 1;
    s.3 = s.3.max(off);
    if s.1 >= 600 {
        log_line(&format!("arrows: render placement moved in {} of {} frames, max {:.2} m from tick position", s.2, s.1, s.3));
        *s = (at, 0, 0, 0.0);
    }
}

/// The exact matrix the game drew this body with in the frame `f`: its interpolation between the two
/// builds ending at `f.ts`, by `f.alpha`.
fn smoothed_matrix(body: usize, f: Option<crate::flight::FrameSnap>) -> Option<[f64; 16]> {
    // Exact: the matrix the game drew this body with (draw capture), when it was matched.
    if let Some(m) = crate::drawcap::drawn(body) {
        diag(|d| d.cases[3] += 1);
        return Some(m);
    }
    let Some(f) = f else {
        return crate::flight::body_render_matrix(body);
    };
    let h = crate::flight::BUILT_HISTORY.lock().ok()?.get(&body).cloned().unwrap_or_default();
    let (prev, cur, case) = match h.iter().position(|(t, _)| *t == f.ts) {
        Some(i) => (h[i.saturating_sub(1)].1, h[i].1, 0),
        // The game built again after our last record (update thread hasn't reached our hook yet):
        // that build's pose is already in body+0x308.
        None if h.last().is_some_and(|(t, _)| *t < f.ts) => (h.last()?.1, crate::flight::body_built_matrix(body)?, 1),
        None => {
            let m = h.first().map(|x| x.1).or_else(|| crate::flight::body_built_matrix(body))?;
            (m, m, 2)
        }
    };
    diag(|d| {
        d.frames += 1;
        d.cases[case] += 1;
        d.alpha_min = d.alpha_min.min(f.alpha);
        d.alpha_max = d.alpha_max.max(f.alpha);
    });
    Some(interp(&prev, &cur, f.alpha))
}

#[derive(Default)]
struct Diag {
    frames: u32,
    cases: [u32; 4],
    alpha_min: f32,
    alpha_max: f32,
}

/// Smoothing diagnostics, one log line per 4000 samples.
fn diag(f: impl FnOnce(&mut Diag)) {
    static D: LazyLock<Mutex<Diag>> = LazyLock::new(|| Mutex::new(Diag { alpha_min: 9.0, ..Default::default() }));
    let mut d = D.lock().unwrap_or_else(|p| p.into_inner());
    f(&mut d);
    if d.frames + d.cases[3] >= 4000 {
        log_line(&format!(
            "arrows smoothing: {} exact from the draw, else {} matched a recorded build, {} one build newer, {} no match; alpha {:.2}..{:.2}",
            d.cases[3], d.cases[0], d.cases[1], d.cases[2], d.alpha_min, d.alpha_max
        ));
        *d = Diag { alpha_min: 9.0, ..Default::default() };
    }
}

/// The game's `matrix_interp_prev_cur` on f32 copies, translations taken relative to `cur` (exact: the
/// translation part is a linear lerp), so f32 keeps millimetres far from the origin.
fn interp(prev: &[f64; 16], cur: &[f64; 16], alpha: f32) -> [f64; 16] {
    let t = [cur[12], cur[13], cur[14]];
    let rel = |m: &[f64; 16]| -> [f32; 16] { std::array::from_fn(|k| (if (12..15).contains(&k) { m[k] - t[k - 12] } else { m[k] }) as f32) };
    let (p, c) = (rel(prev), rel(cur));
    let mut out = [0f32; 16];
    match crate::flight::MATRIX_INTERP.get() {
        Some(f) => unsafe { f(p.as_ptr(), c.as_ptr(), alpha, out.as_mut_ptr()) },
        None => out = std::array::from_fn(|k| p[k] + (c[k] - p[k]) * alpha),
    }
    std::array::from_fn(|k| out[k] as f64 + if (12..15).contains(&k) { t[k - 12] } else { 0.0 })
}

/// Bodies the next frame's draw capture should look for, with their recorded poses [build n-1, build n].
fn publish_track(data: &[crate::flight::VehicleDebug]) {
    let hist = crate::flight::BUILT_HISTORY.lock().map(|h| h.clone()).unwrap_or_default();
    let mut track: Vec<(usize, [[f64; 16]; 2])> = Vec::new();
    for a in data.iter().flat_map(|v| v.arrows.iter()) {
        if track.iter().any(|(b, _)| *b == a.body) {
            continue;
        }
        let Some(live) = crate::flight::body_render_matrix(a.body) else {
            continue;
        };
        let h = hist.get(&a.body);
        let n = h.and_then(|h| h.last()).map_or(live, |x| x.1);
        let n1 = h.and_then(|h| h.len().checked_sub(2).map(|i| h[i].1)).unwrap_or(n);
        track.push((a.body, [n1, n]));
    }
    *crate::drawcap::TRACK.lock().unwrap_or_else(|p| p.into_inner()) = track;
}

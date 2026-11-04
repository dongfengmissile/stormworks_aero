//! The workbench analysis window (F7), modelled on FAR's editor GUI: Static
//! Analysis, Data + Stability Derivatives, Stability Deriv Simulation, Transonic
//! Design, plus a voxel view where wings and control roles are designated.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock, Mutex};

use hudhook::imgui::{self, Condition, Context, Io, Key, MouseButton, TextureId, Ui};
use hudhook::{ImguiRenderLoop, MessageFilter, RenderContext};

use crate::defs::Library;
use crate::editor::{self, Snapshot};
use crate::logging::log_line;
use crate::math::V3;
use crate::model::{Coeffs, Cond, Designations, Model, Stability};
use crate::vehicle::{parse_vehicle_xml, Comp};
use crate::view::{self, Camera, Face, Image};
use crate::voxel::{EMPTY, INTERIOR};
use crate::wing::{Role, AEROFOILS};

const VIEW_W: usize = 760;
const VIEW_H: usize = 460;
/// In-world model texture, stretched over the whole screen (projection is NDC-based).
const WORLD_W: usize = 960;
const WORLD_H: usize = 540;

const CL_COLOR: [f32; 4] = [0.0, 1.0, 1.0, 1.0];
const CD_COLOR: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const CM_COLOR: [f32; 4] = [1.0, 0.92, 0.016, 1.0];
const LD_COLOR: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
const FLAPS: [&str; 4] = ["0 (up)", "1 (init climb)", "2 (takeoff)", "3 (landing)"];

#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Inspect,
    Wing,
    Role,
    Foil,
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
    vehicles_dir: PathBuf,
    status: String,

    // Vehicle source.
    snapshot: Option<Arc<Snapshot>>,
    file_comps: Option<Arc<Vec<Comp>>>,
    use_file: bool,
    vehicle_name: String,
    /// Last automatic `.aero` lookup (ms).
    auto_ms: u64,
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

    // Plugin model drawn in the workbench view instead of the game's (F8).
    world: bool,
    world_tex: Option<TextureId>,
    world_img: Option<Image>,
    world_upload: bool,
    /// Camera and face list the current world image was rendered with.
    world_key: Option<([f64; 16], usize)>,
    /// The game's show/hide vehicle key is due (sent once the game gets keys again).
    press_hide: bool,
    /// Flight data readout (F9).
    flight_data: bool,
}

impl Overlay {
    pub(crate) fn new(game: PathBuf, appdata: PathBuf) -> Self {
        Overlay {
            open: false,
            lib: Arc::new(Mutex::new(Library::new(&game, &appdata))),
            vehicles_dir: appdata.join("data/vehicles"),
            status: String::new(),
            snapshot: None,
            file_comps: None,
            use_file: false,
            vehicle_name: String::new(),
            auto_ms: 0,
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
            world: false,
            world_tex: None,
            world_img: None,
            world_upload: false,
            world_key: None,
            press_hide: false,
            flight_data: false,
        }
    }

    /// Swap the workbench view between the game's vehicle and the plugin's voxel model.
    fn set_world(&mut self, on: bool) {
        if on != self.world {
            self.world = on;
            self.world_key = None;
            self.press_hide = !self.press_hide;
        }
    }

    /// FAR-style flight data for the vehicles the plugin flies (click-through: never takes keys).
    fn draw_flight_data(&mut self, ui: &Ui) {
        let mut data = crate::flight::DEBUG.lock().map(|d| d.clone()).unwrap_or_default();
        data.sort_by(|a, b| b.airspeed.total_cmp(&a.airspeed));
        let flags = imgui::WindowFlags::NO_INPUTS | imgui::WindowFlags::NO_FOCUS_ON_APPEARING | imgui::WindowFlags::ALWAYS_AUTO_RESIZE | imgui::WindowFlags::NO_NAV;
        ui.window("Aero flight data (F9)##flight").position([12.0, 120.0], Condition::FirstUseEver).bg_alpha(0.6).flags(flags).build(|| {
            if !crate::flight::ENABLED.load(Ordering::Relaxed) {
                ui.text_colored([1.0, 0.7, 0.3, 1.0], "Plugin flight model is off (F7 window): the game's own aerodynamics are flying.");
                return;
            }
            if data.is_empty() {
                ui.text("No vehicle in flight yet (models build a moment after spawning).");
            }
            for v in data.iter().take(3) {
                ui.text(format!("Vehicle {}  {}", v.id, if v.designations.is_empty() { "(no designations)".to_string() } else { v.designations.clone() }));
                ui.text(format!("  Airspeed {:6.1} m/s ({:5.0} km/h, {:4.0} kt)  Mach {:.2}  Alt {:6.0} m  q {:6.1} kPa", v.airspeed, v.airspeed * 3.6, v.airspeed * 1.943_84, v.mach, v.alt, v.q / 1000.0));
                ui.text(format!("  Air density {:.3} kg/m³ ({:.0}% of sea level, Stormworks' curve)", v.rho, v.rho / f64::from_bits(crate::model::SEA_LEVEL_RHO.load(Ordering::Relaxed)).max(1e-6) * 100.0));
                ui.text(format!("  AoA {:5.1}°  Sideslip {:5.1}°", v.aoa.to_degrees(), v.beta.to_degrees()));
                let ld = if v.drag > 1e-6 { v.lift / v.drag } else { 0.0 };
                ui.text(format!("  Cl {:6.3}  Cd {:6.4}  L/D {:5.1}", v.cl, v.cd, ld));
                // kN and kgf (the weight of that many kg) instead of raw newtons.
                let force = |n: f64| format!("{:.1} kN ({:.0} kgf)", n / 1000.0, n / 9.806_65);
                ui.text(format!("  Lift {} = {:4.2} × weight  Drag {}", force(v.lift), v.lift / v.weight.max(1e-9), force(v.drag)));
                let (thrust, how) = (v.thrust_exact.unwrap_or(v.thrust), v.thrust_source.as_str());
                ui.text(format!("  Mass {:.2} t  Thrust {} ({how})  T/W {:4.2}  Physics step {:4.1} ms", v.mass / 1000.0, force(thrust), thrust / v.weight.max(1e-9), v.step * 1000.0));
                if let Some(sm) = v.margin {
                    let pct = sm / v.mac.max(1e-6) * 100.0;
                    let (col, word) = if sm >= 0.0 { ([0.6, 1.0, 0.6, 1.0], "stable") } else { ([1.0, 0.35, 0.3, 1.0], "UNSTABLE in pitch") };
                    ui.text_colored(col, format!("  CG {:.2} m {} the neutral point NP ({:.0}% MAC static margin): {word}", sm.abs(), if sm >= 0.0 { "ahead of" } else { "behind" }, pct.abs()));
                }
                if let Some((h, phi)) = v.ground.filter(|g| g.1 < 0.99) {
                    ui.text(format!("  Ground effect: wing {h:4.1} m above the surface, induced drag ×{phi:.2}"));
                }
                if v.stalled > 0 {
                    ui.text_colored([1.0, 0.3, 0.3, 1.0], format!("  STALL: {} of {} panels", v.stalled, v.panels));
                }
            }
        });
    }

    fn draw_world(&mut self, ui: &Ui) {
        if !editor::editor_active() {
            return;
        }
        let clip = *editor::WORLD_CLIP.lock().unwrap_or_else(|p| p.into_inner());
        let (Some(clip), Some(m)) = (clip, self.model.clone()) else { return };
        let key = (clip, Arc::as_ptr(&self.faces) as usize);
        if !self.world_upload && self.world_key != Some(key) {
            // shortcut: CPU render on the render thread (~20 ms while the camera moves), move to a worker if it stutters.
            let faces = self.faces.clone();
            let pr = view::Projection::from_clip(clip, WORLD_W, WORLD_H);
            self.world_img = Some(view::render_with(&faces, &|i| self.face_color(&m, i, None), &pr, WORLD_W, WORLD_H, [0; 4]));
            self.world_upload = true;
            self.world_key = Some(key);
        }
        if let Some(tex) = self.world_tex {
            ui.get_background_draw_list().add_image(tex, [0.0, 0.0], ui.io().display_size).build();
        }
    }

    fn sidecar(&self) -> Option<PathBuf> {
        let n = self.vehicle_name.trim();
        (!n.is_empty() && !n.contains(['/', '\\', ':']) && !n.contains("..")).then(|| self.vehicles_dir.join(format!("{n}.aero")))
    }

    fn load_designations(&mut self) {
        let Some(path) = self.sidecar() else {
            self.status = "enter a vehicle name (no / \\ : or ..)".into();
            return;
        };
        match std::fs::read_to_string(&path) {
            Ok(t) => {
                self.undo.clear();
                self.des = Designations::parse(&t);
                self.status = format!("designations: {} wing blocks, {} roles ({})", self.des.wings.len(), self.des.roles.len(), path.display());
                self.model_dirty = true;
                self.prune_stale();
            }
            // No file for this name yet: keep the marks made so far and start the file with them.
            Err(_) => {
                self.designations_changed();
                self.status = format!("new designation file {} ({} wing blocks, {} roles kept)", path.display(), self.des.wings.len(), self.des.roles.len());
            }
        }
        log_line(&format!("designations: {}", self.status));
    }

    fn designations_changed(&mut self) {
        self.model_dirty = true;
        if let Some(path) = self.sidecar() {
            match std::fs::write(&path, self.des.to_text()) {
                Ok(()) => self.status = format!("saved {}", path.display()),
                Err(e) => {
                    self.status = format!("could not save {}: {e}", path.display());
                    log_line(&format!("designations: {}", self.status));
                }
            }
        } else {
            self.status = "enter the vehicle name to keep these designations".into();
        }
    }

    /// Workbench vehicle changed: if the loaded designations don't fit it, load the best-matching `.aero`
    /// (so a vehicle's marks are never saved into another vehicle's file).
    fn auto_designations(&mut self) {
        let Some(snap) = self.snapshot.clone() else { return };
        if self.des.match_score(&snap.comps).is_some_and(|x| x.0 >= 0.5) {
            self.prune_stale();
            return;
        }
        let now = editor::now_ms();
        if now.saturating_sub(self.auto_ms) < 2000 {
            return; // every edit changes the snapshot: don't rescan the folder each time
        }
        self.auto_ms = now;
        match Designations::best_match(&self.vehicles_dir, &snap.comps, 0.5) {
            Some((d, name, score, missing)) => {
                self.undo.clear();
                self.des = d;
                self.vehicle_name = name.clone();
                self.status = if missing == 0 {
                    format!("loaded {name}.aero (matches this vehicle)")
                } else {
                    format!("loaded {name}.aero: {:.0}% of its marks match, {missing} are on blocks that are gone", score * 100.0)
                };
                log_line(&format!("designations: {}", self.status));
                self.model_dirty = true;
                self.prune_stale();
            }
            None if self.sidecar().is_some() => {
                // A different vehicle with no designations yet: stop editing the previous vehicle's file.
                self.undo.clear();
                self.des = Designations::default();
                self.vehicle_name.clear();
                self.status = "new vehicle (no matching .aero): enter its name to save designations".into();
                self.model_dirty = true;
            }
            None => {}
        }
    }

    /// Same vehicle in the workbench: marks on blocks deleted (or moved onto a new sub-grid) since designating go
    /// with them, and the file is saved. Ctrl+Z brings them back.
    fn prune_stale(&mut self) {
        let Some(snap) = self.snapshot.clone().filter(|_| !self.use_file) else { return };
        let Some((score, missing)) = self.des.match_score(&snap.comps).filter(|x| x.0 >= 0.5 && x.1 > 0) else { return };
        let before = self.des.clone();
        let n = self.des.retain_present(&snap.comps);
        self.undo.push(before);
        self.designations_changed();
        self.status = format!("removed {n} marks on deleted blocks (Ctrl+Z restores them); {:.0}% had matched", score * 100.0);
        log_line(&format!("designations: {} ({missing} stale)", self.status));
    }

    fn current_comps(&self) -> Option<Vec<Comp>> {
        if self.use_file {
            self.file_comps.as_ref().map(|c| c.as_ref().clone())
        } else {
            self.snapshot.as_ref().map(|s| s.comps.clone())
        }
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
            self.faces = Arc::new(b.faces);
            self.view_dirty = true;
            self.sweep = None;
            self.stab = None;
            self.sim = None;
        }
        if !self.use_file {
            let latest = editor::LATEST.lock().unwrap_or_else(|p| p.into_inner()).clone();
            if latest.as_ref().map(|s| s.gen) != self.snapshot.as_ref().map(|s| s.gen) {
                self.snapshot = latest;
                self.model_dirty = true;
                self.auto_designations();
            }
        }
        if !self.model_dirty || self.building.load(Ordering::Acquire) {
            return;
        }
        let Some(comps) = self.current_comps() else { return };
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

    fn header(&mut self, ui: &Ui) {
        let in_editor = editor::editor_active();
        let mut world = self.world;
        if ui.checkbox("Plugin model in the workbench view (F8)", &mut world) {
            self.set_world(world);
        }
        ui.same_line();
        if crate::flight::HOOKED.load(Ordering::Relaxed) {
            let mut fly = crate::flight::ENABLED.load(Ordering::Relaxed);
            if ui.checkbox("Plugin flight model in game", &mut fly) {
                crate::flight::ENABLED.store(fly, Ordering::Relaxed);
                log_line(&format!("flight model {}", if fly { "on" } else { "off" }));
            }
            if ui.is_item_hovered() {
                ui.tooltip_text("Replaces the game's air forces on every vehicle with this plugin's model (wings with their aerofoils and designations, fuselage drag and lift). Water forces are unchanged. Off restores the game's.");
            }
        } else {
            ui.text_disabled("(in-game flight unavailable: see aero.log)");
        }
        if ui.is_item_hovered() {
            ui.tooltip_text("Swaps the game's vehicle (hidden with its show/hide key, H) for the voxel model the analysis uses.");
        }
        if ui.radio_button_bool("Workbench", !self.use_file) && self.use_file {
            self.use_file = false;
            self.model_dirty = true;
        }
        ui.same_line();
        if ui.radio_button_bool("Saved file", self.use_file) && !self.use_file {
            self.use_file = true;
            self.model_dirty = true;
        }
        ui.same_line();
        ui.set_next_item_width(220.0);
        if ui.input_text("Vehicle name", &mut self.vehicle_name).enter_returns_true(true).build() {
            self.load_designations();
        }
        ui.same_line();
        if ui.button("Load designations") {
            self.load_designations();
        }
        if self.use_file {
            ui.same_line();
            if ui.button("Load file") {
                let path = self.vehicles_dir.join(format!("{}.xml", self.vehicle_name.trim()));
                match std::fs::read_to_string(&path) {
                    Ok(t) if self.sidecar().is_some() => {
                        self.file_comps = Some(Arc::new(parse_vehicle_xml(&t)));
                        self.load_designations();
                    }
                    _ => self.status = format!("cannot read {}", path.display()),
                }
            }
        } else if !in_editor {
            ui.text_colored([1.0, 0.6, 0.2, 1.0], "Not in the workbench: open it, or analyse a saved file.");
        }
        match &self.model {
            Some(m) => {
                ui.text(format!(
                    "{} parts, {} voxels | mass {:.0} kg | wing area {:.2} m², MAC {:.2} m, span {:.2} m | {} lifting panels",
                    m.comps.len(),
                    m.grid.count_filled(),
                    m.mass,
                    m.s_ref,
                    m.mac_ref,
                    m.b_ref,
                    m.panels.len()
                ));
                if m.panels.is_empty() {
                    ui.text_colored([1.0, 0.8, 0.3, 1.0], "No wings: mark wing blocks in the Voxels tab (FAR then uses max cross-section and length as reference).");
                }
                if !m.grid.missing.is_empty() {
                    ui.text_colored([1.0, 0.5, 0.5, 1.0], format!("Unknown parts drawn as single voxels: {}", m.grid.missing.join(", ")));
                }
                if let Some((_, missing)) = self.des.match_score(&m.comps).filter(|x| x.1 > 0) {
                    ui.text_colored(
                        [1.0, 0.8, 0.3, 1.0],
                        format!("{missing} marks are on blocks that no longer exist (vehicle edited since designating): re-mark those parts; the rest still apply."),
                    );
                }
            }
            None if self.building.load(Ordering::Relaxed) => ui.text("Voxelizing..."),
            None => ui.text("No vehicle yet."),
        }
        if !self.status.is_empty() {
            ui.text_disabled(&self.status);
        }
    }

    fn tab_static(&mut self, ui: &Ui, m: &Arc<Model>) {
        ui.text(if self.sweep_mach { "Mach Number Sweep Analysis" } else { "Angle of Attack Sweep Analysis" });
        ui.same_line();
        if ui.button(if self.sweep_mach { "Switch To AoA Sweep" } else { "Switch To Mach Sweep" }) {
            self.sweep_mach = !self.sweep_mach;
        }
        let w = 90.0;
        ui.set_next_item_width(w);
        if self.sweep_mach {
            ui.input_float("Lower Mach", &mut self.mach_lo).build();
            ui.same_line();
            ui.set_next_item_width(w);
            ui.input_float("Upper Mach", &mut self.mach_hi).build();
            ui.same_line();
            ui.set_next_item_width(w);
            ui.input_float("AoA (deg)", &mut self.other_aoa).build();
        } else {
            ui.input_float("Lower AoA", &mut self.aoa_lo).build();
            ui.same_line();
            ui.set_next_item_width(w);
            ui.input_float("Upper AoA", &mut self.aoa_hi).build();
            ui.same_line();
            ui.set_next_item_width(w);
            ui.input_float("Mach", &mut self.other_mach).build();
        }
        ui.same_line();
        ui.set_next_item_width(w);
        ui.input_int("Num Pts", &mut self.npts).build();
        ui.set_next_item_width(w);
        ui.input_float("Altitude (m)", &mut self.alt).build();
        ui.same_line();
        ui.set_next_item_width(140.0);
        ui.combo_simple_string("Flap Setting", &mut self.flap, &FLAPS);
        ui.same_line();
        ui.set_next_item_width(w);
        ui.input_float("Pitch Setting", &mut self.pitch).build();
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.npts = self.npts.clamp(2, 500);
        ui.same_line();
        if ui.button(if self.sweep_mach { "Sweep Mach" } else { "Sweep AoA" }) {
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
        if let Some(s) = &self.sweep {
            let series = |c: &[Coeffs], dim: f32| {
                let k = |col: [f32; 4]| [col[0] * dim, col[1] * dim, col[2] * dim, 1.0];
                vec![
                    ("Cl", k(CL_COLOR), c.iter().map(|c| c.cl).collect::<Vec<_>>()),
                    ("Cd", k(CD_COLOR), c.iter().map(|c| c.cd).collect()),
                    ("Cm", k(CM_COLOR), c.iter().map(|c| c.cm).collect()),
                    ("L/D / 10", k(LD_COLOR), c.iter().map(|c| c.cl * 0.1 / c.cd).collect()),
                ]
            };
            let mut all = series(&s.up, 1.0);
            let legend = all.len();
            if !s.down.is_empty() {
                all.extend(series(&s.down, 0.5));
            }
            let xl = if s.mach_sweep { "Mach Number" } else { "Angle of Attack, degrees" };
            plot(ui, "static", [ui.content_region_avail()[0], 360.0], &s.x, &all, legend, xl);
            ui.text_disabled("Dim lines: return pass of the AoA sweep (stall hysteresis).");
        }
    }

    fn tab_stability(&mut self, ui: &Ui, m: &Arc<Model>) {
        let w = 90.0;
        ui.set_next_item_width(w);
        ui.input_float("Altitude (m)", &mut self.st_alt).build();
        ui.same_line();
        ui.set_next_item_width(w);
        ui.input_float("Mach Number", &mut self.st_mach).build();
        self.st_mach = self.st_mach.max(0.001);
        ui.same_line();
        ui.set_next_item_width(140.0);
        ui.combo_simple_string("Flap Setting", &mut self.st_flap, &FLAPS);
        ui.same_line();
        if ui.button("Calculate Stability Derivatives") {
            self.stab = Some(m.stability(self.st_mach as f64, self.st_alt as f64, self.st_flap as f64 / 3.0));
        }
        let Some(s) = &self.stab else {
            ui.text_disabled("Trims for level flight (lift = weight) at this speed, then perturbs each state.");
            return;
        };
        ui.columns(3, "props", false);
        ui.text("Aircraft Properties");
        ui.text(format!("Ref Area: {:.3} m²", s.s));
        ui.text(format!("Scaled Chord: {:.3} m", s.mac));
        ui.text(format!("Scaled Span: {:.3} m", s.b));
        ui.text(format!("Mass: {:.0} kg", s.mass));
        ui.next_column();
        ui.text("Moments of Inertia");
        ui.text(format!("Ixx: {:.4e} kg m^2", s.inertia[0]));
        ui.text(format!("Iyy: {:.4e} kg m^2", s.inertia[1]));
        ui.text(format!("Izz: {:.4e} kg m^2", s.inertia[2]));
        ui.text(format!("Ixz: {:.4e} kg m^2", s.inertia[5]));
        ui.next_column();
        ui.text("Level Flight");
        ui.text(format!("u0: {:.2} m/s", s.u0));
        ui.text(format!("Cl: {:.4} (required {:.4})", s.cl, s.cl_req));
        ui.text(format!("Cd: {:.4}", s.cd));
        ui.text(format!("AoA: {}{:.3}°", s.trim_note, s.alpha.to_degrees()));
        ui.columns(1, "props_end", false);
        if !s.trim_note.is_empty() {
            ui.text_colored([1.0, 0.6, 0.3, 1.0], "Could not trim: the vehicle cannot make enough lift here (shown at the closest AoA).");
        }
        ui.separator();
        ui.text("Longitudinal Derivatives");
        deriv_grid(
            ui,
            "lon",
            &["Down Vel", "Fwd Vel", "Pitch Rate", "Pitch Ctrl"],
            &[
                [("Zw", s.zw, -1, "1/s", "Change in Z-direction acceleration with respect to Z-direction velocity; should be negative"),
                 ("Zu", s.zu, -1, "1/s", "Change in Z-direction acceleration with respect to X-direction velocity; should be negative"),
                 ("Zq", s.zq, 0, "m/s", "Change in Z-direction acceleration with respect to pitch-up rate"),
                 ("Zde", s.zde, 0, "m/s²", "Change in Z-direction acceleration with respect to full pitch control input")],
                [("Xw", s.xw, 0, "1/s", "Change in X-direction acceleration with respect to Z-direction velocity"),
                 ("Xu", s.xu, -1, "1/s", "Change in X-direction acceleration with respect to X-direction velocity; should be negative"),
                 ("Xq", s.xq, 0, "m/s", "Change in X-direction acceleration with respect to pitch-up rate"),
                 ("Xde", s.xde, 0, "m/s²", "Change in X-direction acceleration with respect to full pitch control input")],
                [("Mw", s.mw, -1, "1/(m s)", "Change in pitch-up angular acceleration with respect to Z-direction velocity; should be negative (statically stable)"),
                 ("Mu", s.mu, 0, "1/(m s)", "Change in pitch-up angular acceleration with respect to X-direction velocity"),
                 ("Mq", s.mq, -1, "1/s", "Change in pitch-up angular acceleration with respect to pitch-up rate; should be negative (pitch damping)"),
                 ("Mde", s.mde, 1, "1/s^2", "Change in pitch-up angular acceleration with respect to full pitch control input; should be positive")],
            ],
        );
        ui.text("Lateral Derivatives");
        deriv_grid(
            ui,
            "lat",
            &["Sideslip", "Roll Rate", "Yaw Rate"],
            &[
                [("Yb", s.yb, -1, "m/s²", "Change in Y-direction acceleration with respect to sideslip angle (beta); should be negative"),
                 ("Yp", s.yp, 0, "m/s", "Change in Y-direction acceleration with respect to roll-right rate"),
                 ("Yr", s.yr, 1, "m/s", "Change in Y-direction acceleration with respect to yaw-right rate; should be positive"),
                 ("", 0.0, 0, "", "")],
                [("Lb", s.lb, -1, "1/s^2", "Change in roll-right angular acceleration with respect to sideslip (beta); should be negative (dihedral effect)"),
                 ("Lp", s.lp, -1, "1/s", "Change in roll-right angular acceleration with respect to roll-right rate; should be negative (roll damping)"),
                 ("Lr", s.lr, 1, "1/s", "Change in roll-right angular acceleration with respect to yaw-right rate; should be positive"),
                 ("", 0.0, 0, "", "")],
                [("Nb", s.nb, 1, "1/s^2", "Change in yaw-right angular acceleration with respect to sideslip (beta); should be positive (weathervane stability)"),
                 ("Np", s.np, 0, "1/s", "Change in yaw-right angular acceleration with respect to roll-right rate"),
                 ("Nr", s.nr, -1, "1/s", "Change in yaw-right angular acceleration with respect to yaw-right rate; should be negative (yaw damping)"),
                 ("", 0.0, 0, "", "")],
            ],
        );
    }

    fn tab_sim(&mut self, ui: &Ui) {
        let Some(s) = self.stab.clone() else {
            ui.text_disabled("Calculate stability derivatives first (Data + Stability Derivatives tab).");
            return;
        };
        if ui.radio_button_bool("Longitudinal Sim", !self.sim_lateral) {
            self.sim_lateral = false;
            self.sim = None;
        }
        ui.same_line();
        if ui.radio_button_bool("Lateral Sim", self.sim_lateral) {
            self.sim_lateral = true;
            self.sim = None;
        }
        let names: [&str; 4] =
            if self.sim_lateral { ["beta (deg)", "p (deg/s)", "r (deg/s)", "phi (deg)"] } else { ["w (m/s)", "u (m/s)", "q (deg/s)", "theta (deg)"] };
        for (i, n) in names.iter().enumerate() {
            ui.set_next_item_width(80.0);
            ui.input_float(*n, &mut self.sim_x0[i]).build();
            ui.same_line();
        }
        ui.set_next_item_width(70.0);
        ui.input_float("End Time", &mut self.sim_end).build();
        ui.same_line();
        ui.set_next_item_width(70.0);
        ui.input_float("dt", &mut self.sim_dt).build();
        ui.same_line();
        if ui.button("Run Simulation") {
            let d = std::f64::consts::PI / 180.0;
            let k = if self.sim_lateral { [d, d, d, d] } else { [1.0, 1.0, d, d] };
            let x0 = [0, 1, 2, 3].map(|i| self.sim_x0[i] as f64 * k[i]);
            let dt = (self.sim_dt as f64).clamp(0.001, 1.0);
            let end = (self.sim_end as f64).clamp(dt, 600.0);
            self.sim = Some(s.simulate(self.sim_lateral, x0, end, dt));
        }
        if let Some((t, xs)) = &self.sim {
            let d = 180.0 / std::f64::consts::PI;
            let k = if self.sim_lateral { [d, d, d, d] } else { [1.0, 1.0, d, d] };
            let cols = [LD_COLOR, CM_COLOR, CD_COLOR, CL_COLOR];
            let series: Vec<(&str, [f32; 4], Vec<f64>)> =
                (0..4).map(|i| (names[i], cols[i], xs.iter().map(|x| (x[i] * k[i]).clamp(-50.0, 50.0)).collect())).collect();
            plot(ui, "sim", [ui.content_region_avail()[0], 300.0], t, &series, 4, "Time (s)");
        }
    }

    fn tab_transonic(&mut self, ui: &Ui, m: &Arc<Model>) {
        let b = &m.body;
        ui.text("Transonic Area Ruling Analysis");
        ui.text(format!("Max Cross-Section Area: {:.4} m²", b.max_area));
        ui.text(format!("Mach 1 Wave Drag-Area: {:.4} m²", b.sonic_drag_area));
        ui.text(format!("Critical Mach Number: {:.3}", b.critical_mach));
        ui.text(format!("Length: {:.2} m, fineness ratio {:.2}", b.length, b.fineness));
        if 0.75 * b.sonic_drag_area >= b.max_area && b.max_area > 0.0 {
            ui.text_colored([1.0, 0.5, 0.3, 1.0], "Area ruling concern: wave drag is high for this cross-section; smooth the area curve.");
        }
        ui.text_wrapped("Minimal wave drag is achieved by maintaining a smooth, minimal curvature cross-section curve. Minimize curvature to minimize wave drag.");
        ui.checkbox("Show cross-sectional area curve (green)", &mut self.show_area);
        ui.checkbox("Show curvature cross-sectional area curve (yellow)", &mut self.show_curv);
        ui.checkbox("Show avg pressure coefficient curve (cyan)", &mut self.show_cp);
        if b.back <= b.front {
            return;
        }
        // Slices are stored nose first; plot tail to nose so x increases to the right.
        let slices: Vec<&crate::body::Xsec> = b.xs[b.front.saturating_sub(1)..(b.back + 2).min(b.xs.len())].iter().rev().collect();
        let xs: Vec<f64> = slices.iter().map(|s| s.x).collect();
        // Curvature scaled to the area curve's range, like FAR's overlay.
        let amax = b.max_area.max(1e-6);
        let cmax = slices.iter().map(|s| s.second_deriv.abs()).fold(1e-9, f64::max);
        let mut series = Vec::new();
        if self.show_area {
            series.push(("Area (m²)", LD_COLOR, slices.iter().map(|s| s.area).collect::<Vec<_>>()));
            series.push(("Raw area (m²)", [0.0, 0.5, 0.0, 1.0], slices.iter().map(|s| s.raw_area).collect()));
        }
        if self.show_curv {
            series.push(("Curvature (scaled)", CM_COLOR, slices.iter().map(|s| s.second_deriv / cmax * amax).collect()));
        }
        if self.show_cp {
            series.push(("Cp sonic (x area max)", CL_COLOR, slices.iter().map(|s| s.cp_sonic_fwd * amax).collect()));
        }
        let n = series.len();
        plot(ui, "area", [ui.content_region_avail()[0], 280.0], &xs, &series, n, "Position along the vehicle, m (nose right)");
    }

    fn tab_voxels(&mut self, ui: &Ui, m: &Arc<Model>) {
        let tool_before = self.tool;
        if ui.radio_button_bool("Inspect", self.tool == Tool::Inspect) {
            self.tool = Tool::Inspect;
        }
        ui.same_line();
        if ui.radio_button_bool("Wing blocks", self.tool == Tool::Wing) {
            self.tool = Tool::Wing;
        }
        ui.same_line();
        if ui.radio_button_bool("Control role", self.tool == Tool::Role) {
            self.tool = Tool::Role;
        }
        ui.same_line();
        if ui.radio_button_bool("Aerofoil", self.tool == Tool::Foil) {
            self.tool = Tool::Foil;
        }
        if (tool_before == Tool::Foil) != (self.tool == Tool::Foil) {
            self.view_dirty = true; // panel colours switch between roles and aerofoils
        }
        if self.tool == Tool::Role {
            ui.same_line();
            ui.set_next_item_width(100.0);
            ui.combo_simple_string("##role", &mut self.role_pick, &["none", "pitch", "roll", "yaw", "flap"]);
        }
        let names: Vec<&str> = AEROFOILS.iter().map(|a| a.name).collect();
        if self.tool == Tool::Foil {
            ui.set_next_item_width(260.0);
            ui.combo_simple_string("Click a wing to give it##foil", &mut self.foil_pick, &names);
            ui.same_line();
            ui.set_next_item_width(260.0);
            let mut d = self.des.default_foil;
            if ui.combo_simple_string("Other panels##foil_default", &mut d, &names) && d != self.des.default_foil {
                self.push_undo();
                self.des.default_foil = d;
                self.designations_changed();
            }
            let a = &AEROFOILS[self.foil_pick];
            ui.text_disabled(format!(
                "{}  t/c {:.0}%, Cl at 0° {:.2}, Clmax {:.2}, drag rise from Mach {:.2} (straight wing)",
                a.note,
                a.t_c * 100.0,
                a.cl0,
                a.cl_max,
                a.kappa - a.t_c
            ));
        }
        ui.same_line();
        if ui.button("Reset view") {
            self.frame_camera(m);
            self.view_dirty = true;
        }
        ui.text_disabled(match self.tool {
            Tool::Inspect => "Left-drag rotate, right-drag pan, wheel zoom. Hover a block for details.",
            Tool::Wing => "Click: toggle a wing block. Shift+click: add the whole thin slab. Ctrl+click: remove the slab. Ctrl+Z: undo.",
            Tool::Role => "Click a vanilla control surface, or any block of a hinged sub-grid, to give it the selected role. Ctrl+Z: undo.",
            Tool::Foil => "Click a wing to give the whole wing (and the surfaces on it) the selected aerofoil. Shift+click: one panel only. Ctrl+Z: undo.",
        });

        let avail = ui.content_region_avail();
        let size = [avail[0].min(VIEW_W as f32 * 1.4).max(200.0), (avail[0] * VIEW_H as f32 / VIEW_W as f32).min(avail[1] - 120.0).max(150.0)];
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
                        self.apply_tool(m, f, io.key_shift, io.key_ctrl);
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

        // Hover details.
        if let Some(f) = self.hover_face.and_then(|f| self.faces.get(f)) {
            let o = f.comp;
            {
                let c = &m.comps[o as usize];
                ui.text(format!("{}  at ({}, {}, {})  body {}", c.def, c.pos[0], c.pos[1], c.pos[2], c.body));
                if let Some(p) = m.panel_of.get(f.cell).and_then(|&p| m.panels.get(p)) {
                    ui.text(format!(
                        "Panel: area {:.2} m², semispan {:.2} m, MAC {:.2} m, AR {:.2}, taper {:.2}, sweep {:.1}°, role {}, aerofoil {}",
                        p.area,
                        p.b,
                        p.mac,
                        p.ar,
                        p.taper,
                        p.tan_sweep.atan().to_degrees(),
                        p.role.name(),
                        AEROFOILS[p.foil.min(AEROFOILS.len() - 1)].name
                    ));
                }
            }
        }
        if self.tool == Tool::Foil {
            ui.text_disabled("Panel colour = aerofoil:");
            for (i, a) in AEROFOILS.iter().enumerate() {
                let c = FOIL_COLORS[i % FOIL_COLORS.len()].map(|v| v as f32 / 255.0);
                if i % 4 != 0 {
                    ui.same_line();
                }
                ui.text_colored([c[0], c[1], c[2], 1.0], a.name);
            }
        } else {
            ui.text_disabled("Grey: body. Blue: wing panel. Orange: pitch, magenta: roll, green: yaw, yellow: flap. Dark blue: marked block not part of a panel.");
        }
    }

    fn apply_tool(&mut self, m: &Arc<Model>, face: usize, shift: bool, ctrl: bool) {
        let Some(f) = self.faces.get(face) else { return };
        let comp = &m.comps[f.comp as usize];
        let before = self.des.clone();
        match self.tool {
            Tool::Inspect => return,
            Tool::Wing => {
                if shift || ctrl {
                    for pos in slab(m, f) {
                        if ctrl {
                            self.des.wings.remove(&pos);
                        } else {
                            self.des.wings.insert(pos);
                        }
                    }
                } else if !self.des.wings.remove(&comp.pos) {
                    self.des.wings.insert(comp.pos);
                }
            }
            Tool::Foil => {
                let Some(p) = m.panel_of.get(f.cell).and_then(|&p| m.panels.get(p)) else {
                    self.status = "that block is not part of a lifting panel".into();
                    return;
                };
                // Click: the whole wing (its panels and the surfaces on it). Shift+click: this panel only.
                let pi = m.panel_of[f.cell];
                let targets: Vec<usize> = if shift { vec![pi] } else { (0..m.panels.len()).filter(|&j| m.wing_of[j] == m.wing_of[pi]).collect() };
                let _ = p;
                for j in targets {
                    // One mark per panel: drop the old ones, then mark one of its blocks.
                    let owners: Vec<[i32; 3]> = m.panels[j].cells.iter().filter_map(|&c| m.comps.get(m.grid.owner[c] as usize)).map(|o| o.pos).collect();
                    for pos in &owners {
                        self.des.foils.remove(pos);
                    }
                    if self.foil_pick != self.des.default_foil {
                        let at = if j == pi { comp.pos } else { owners[0] };
                        self.des.foils.insert(at, self.foil_pick);
                    }
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
            self.undo.push(before);
            if self.undo.len() > 200 {
                self.undo.remove(0);
            }
            self.designations_changed();
        }
    }

    fn push_undo(&mut self) {
        self.undo.push(self.des.clone());
        if self.undo.len() > 200 {
            self.undo.remove(0);
        }
    }

    fn undo_designation(&mut self) {
        match self.undo.pop() {
            Some(d) => {
                self.des = d;
                self.designations_changed();
            }
            None => self.status = "nothing to undo".into(),
        }
    }

    fn face_color(&self, m: &Model, face: usize, hover_comp: Option<u32>) -> [u8; 3] {
        let des = &self.des;
        let cell = self.faces[face].cell;
        let o = self.faces[face].comp;
        let mut c: [u8; 3] = match m.panel_of[cell] {
            usize::MAX if o != EMPTY && o != INTERIOR && des.wings.contains(&m.comps[o as usize].pos) => [40, 60, 150],
            usize::MAX => {
                let b = if o != EMPTY && o != INTERIOR { m.comps[o as usize].body } else { 0 };
                if b == 0 { [150, 150, 150] } else { [135, 145, 160] }
            }
            p if self.tool == Tool::Foil => FOIL_COLORS[m.panels[p].foil % FOIL_COLORS.len()],
            p => match m.panels[p].role {
                Role::None => [90, 150, 230],
                Role::Pitch => [240, 140, 40],
                Role::Roll => [220, 70, 220],
                Role::Yaw => [70, 210, 90],
                Role::Flap => [230, 220, 60],
            },
        };
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

/// Grid positions of the components in the thin slab around a clicked face: cells of the
/// same body reachable through cells no more than 4 voxels thick along the face normal.
fn slab(m: &Model, f: &Face) -> Vec<[i32; 3]> {
    let g = &m.grid;
    let n = f.normal;
    // Body-frame normal -> its dominant grid axis.
    let gnf = [n.y, -n.z, n.x];
    let a = (0..3).max_by(|&i, &j| gnf[i].abs().total_cmp(&gnf[j].abs())).unwrap_or(1);
    let mut gn = [0; 3];
    gn[a] = 1;
    let owner_ok = |i: usize| g.owner[i] != EMPTY && g.owner[i] != INTERIOR;
    if !owner_ok(f.cell) {
        return Vec::new();
    }
    let body = m.comps[f.comp as usize].body;
    let thickness = |c: [i32; 3]| {
        let mut t = 1;
        for s in [1, -1] {
            let mut k = 1;
            while g.index([c[0] + gn[0] * k * s, c[1] + gn[1] * k * s, c[2] + gn[2] * k * s]).is_some_and(|j| g.owner[j] != EMPTY) {
                t += 1;
                k += 1;
                if t > 8 {
                    return t;
                }
            }
        }
        t
    };
    let mut seen = vec![false; g.owner.len()];
    let mut stack = vec![f.cell];
    seen[f.cell] = true;
    let mut out = std::collections::HashSet::new();
    while let Some(i) = stack.pop() {
        let c = g.cell(i);
        out.insert(m.comps[g.owner[i] as usize].pos);
        for d in crate::voxel::DIRS {
            let Some(j) = g.index([c[0] + d[0], c[1] + d[1], c[2] + d[2]]) else { continue };
            if !seen[j] && owner_ok(j) && m.comps[g.owner[j] as usize].body == body && thickness(g.cell(j)) <= 4 {
                seen[j] = true;
                stack.push(j);
            }
        }
    }
    out.into_iter().collect()
}

/// Line plot with grid, zero line, legend and a hover readout.
fn plot(ui: &Ui, id: &str, size: [f32; 2], x: &[f64], series: &[(&str, [f32; 4], Vec<f64>)], legend: usize, xlabel: &str) {
    if x.len() < 2 {
        return;
    }
    let p0 = ui.cursor_screen_pos();
    ui.invisible_button(format!("##plot_{id}"), size);
    let hovered = ui.is_item_hovered();
    let dl = ui.get_window_draw_list();
    let (pad_l, pad_b, pad_t) = (52.0, 34.0, 8.0);
    let (x0, x1) = (x[0], x[x.len() - 1]);
    let ys = series.iter().flat_map(|s| s.2.iter().copied()).filter(|v| v.is_finite());
    let (mut y0, mut y1) = ys.fold((f64::MAX, f64::MIN), |(a, b), v| (a.min(v), b.max(v)));
    if y0 > y1 {
        y0 = -1.0;
        y1 = 1.0;
    }
    y0 = y0.min(-0.25).floor().max(-1e6);
    y1 = y1.max(0.25).ceil().min(1e6);
    let plot_lo = [p0[0] + pad_l, p0[1] + pad_t];
    let plot_hi = [p0[0] + size[0] - 8.0, p0[1] + size[1] - pad_b];
    let sx = |v: f64| plot_lo[0] + ((v - x0) / (x1 - x0)) as f32 * (plot_hi[0] - plot_lo[0]);
    let sy = |v: f64| plot_hi[1] - ((v - y0) / (y1 - y0)) as f32 * (plot_hi[1] - plot_lo[1]);
    dl.add_rect(plot_lo, plot_hi, [0.08, 0.08, 0.1, 1.0]).filled(true).build();
    let grid = [0.3, 0.3, 0.35, 1.0];
    let step = |lo: f64, hi: f64| {
        let raw = (hi - lo) / 6.0;
        let mag = 10f64.powf(raw.log10().floor());
        [1.0, 2.0, 5.0, 10.0].iter().map(|k| k * mag).find(|s| *s >= raw).unwrap_or(raw)
    };
    let xs = step(x0, x1);
    let mut v = (x0 / xs).ceil() * xs;
    while v <= x1 + 1e-9 {
        dl.add_line([sx(v), plot_lo[1]], [sx(v), plot_hi[1]], grid).build();
        dl.add_text([sx(v) - 10.0, plot_hi[1] + 3.0], [0.8, 0.8, 0.8, 1.0], fmt_tick(v));
        v += xs;
    }
    let ysx = step(y0, y1);
    let mut v = (y0 / ysx).ceil() * ysx;
    while v <= y1 + 1e-9 {
        let c = if v.abs() < 1e-9 { [0.7, 0.7, 0.7, 1.0] } else { grid };
        dl.add_line([plot_lo[0], sy(v)], [plot_hi[0], sy(v)], c).build();
        dl.add_text([p0[0] + 2.0, sy(v) - 7.0], [0.8, 0.8, 0.8, 1.0], fmt_tick(v));
        v += ysx;
    }
    dl.add_text([plot_lo[0] + 4.0, plot_hi[1] + 17.0], [0.9, 0.9, 0.9, 1.0], xlabel);
    for (_, col, ys) in series {
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
    let mut ly = plot_lo[1] + 4.0;
    for (name, col, _) in series.iter().take(legend) {
        dl.add_rect([plot_hi[0] - 120.0, ly + 4.0], [plot_hi[0] - 108.0, ly + 10.0], *col).filled(true).build();
        dl.add_text([plot_hi[0] - 104.0, ly], [0.9, 0.9, 0.9, 1.0], name);
        ly += 15.0;
    }
    if hovered {
        let mx = ui.io().mouse_pos[0];
        let t = ((mx - plot_lo[0]) / (plot_hi[0] - plot_lo[0])).clamp(0.0, 1.0) as f64;
        let i = ((t * (x.len() - 1) as f64).round() as usize).min(x.len() - 1);
        dl.add_line([sx(x[i]), plot_lo[1]], [sx(x[i]), plot_hi[1]], [1.0, 1.0, 1.0, 0.4]).build();
        drop(dl);
        let mut text = format!("x = {:.3}", x[i]);
        for (name, _, ys) in series.iter().take(legend) {
            text += &format!("\n{name}: {:.4}", ys[i]);
        }
        ui.tooltip_text(text);
    }
}

fn fmt_tick(v: f64) -> String {
    if v.abs() >= 100.0 || v == v.round() {
        format!("{v:.0}")
    } else {
        format!("{v:.2}")
    }
}

type Deriv = (&'static str, f64, i32, &'static str, &'static str);

/// FAR-style derivative table: green when the sign is the expected one, red when not.
fn deriv_grid<const N: usize>(ui: &Ui, id: &str, heads: &[&str], rows: &[[Deriv; N]]) {
    ui.columns(heads.len() as i32, id, false);
    for h in heads {
        ui.text_disabled(*h);
        ui.next_column();
    }
    for row in rows {
        for &(name, v, sign, unit, tip) in row.iter().take(heads.len()) {
            if !name.is_empty() {
                let col = if !v.is_finite() || (sign != 0 && v.signum() as i32 != sign) {
                    [1.0, 0.35, 0.35, 1.0]
                } else if sign != 0 {
                    [0.4, 1.0, 0.4, 1.0]
                } else {
                    [1.0, 1.0, 1.0, 1.0]
                };
                ui.text_colored(col, format!("{name}: {v:.4} {unit}"));
                if ui.is_item_hovered() {
                    ui.tooltip_text(tip);
                }
            }
            ui.next_column();
        }
    }
    ui.columns(1, format!("{id}_end"), false);
}

impl ImguiRenderLoop for Overlay {
    fn initialize<'a>(&'a mut self, ctx: &mut Context, render_context: &'a mut dyn RenderContext) {
        ctx.set_ini_filename(None);
        let blank = vec![0u8; VIEW_W * VIEW_H * 4];
        match render_context.load_texture(&blank, VIEW_W as u32, VIEW_H as u32) {
            Ok(t) => self.tex = Some(t),
            Err(e) => log_line(&format!("voxel view texture failed: {e:?}")),
        }
        match render_context.load_texture(&vec![0u8; WORLD_W * WORLD_H * 4], WORLD_W as u32, WORLD_H as u32) {
            Ok(t) => self.world_tex = Some(t),
            Err(e) => log_line(&format!("world view texture failed: {e:?}")),
        }
        log_line("overlay initialised (F7 toggles the aero analysis window)");
    }

    fn before_render<'a>(&'a mut self, _ctx: &mut Context, render_context: &'a mut dyn RenderContext) {
        if self.world_upload {
            self.world_upload = false;
            if let (Some(tex), Some(img)) = (self.world_tex, self.world_img.as_ref()) {
                if let Err(e) = render_context.replace_texture(tex, &img.rgba, img.w as u32, img.h as u32) {
                    log_line(&format!("world view upload failed: {e:?}"));
                }
            }
        }
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
            self.world = false;
            editor::WANTED.store(false, Ordering::Relaxed);
            editor::WORLD_WANTED.store(false, Ordering::Relaxed);
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
        // imgui-rs builds the draw-list slice from a null pointer when a frame draws nothing,
        // which panics under debug assertions (and is UB anyway): always emit one invisible quad.
        ui.get_foreground_draw_list().add_rect([-4.0, -4.0], [-3.0, -3.0], [0.0, 0.0, 0.0, 0.01]).filled(true).build();
        if ui.is_key_pressed(Key::F7) {
            self.open = !self.open;
        }
        if ui.is_key_pressed(Key::F8) {
            self.set_world(!self.world);
        }
        if self.press_hide {
            if ui.io().want_capture_keyboard {
                // Our window holds the keyboard, so the game would not see the key: drop focus first.
                unsafe { imgui::sys::igSetWindowFocus_Str(std::ptr::null()) };
            } else {
                press_game_key(b'H');
                self.press_hide = false;
            }
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
        editor::WORLD_WANTED.store(self.world, Ordering::Relaxed);
        if self.open || self.world {
            self.pump_model();
        }
        if self.world {
            self.draw_world(ui);
        }
        if !self.open {
            return;
        }
        let mut open = self.open;
        ui.window("Aero Analysis (FAR-style)##aero")
            .size([840.0, 700.0], Condition::FirstUseEver)
            .opened(&mut open)
            .build(|| {
                let io = ui.io();
                if io.key_ctrl && ui.is_key_pressed(Key::Z) && ui.is_window_focused_with_flags(imgui::WindowFocusedFlags::ROOT_AND_CHILD_WINDOWS) {
                    self.undo_designation();
                }
                self.header(ui);
                ui.separator();
                let Some(m) = self.model.clone() else { return };
                if let Some(_bar) = ui.tab_bar("##aero_tabs") {
                    if let Some(_t) = ui.tab_item("Static Analysis") {
                        self.tab_static(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Data + Stability Derivatives") {
                        self.tab_stability(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Stability Deriv Simulation") {
                        self.tab_sim(ui);
                    }
                    if let Some(_t) = ui.tab_item("Transonic Design") {
                        self.tab_transonic(ui, &m);
                    }
                    if let Some(_t) = ui.tab_item("Voxels & Wings") {
                        self.tab_voxels(ui, &m);
                    }
                }
            });
        self.open = open;
        self.refresh_view();
    }

    fn filter(&self, io: &Io) -> MessageFilter {
        if !self.open {
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

/// Tap a key in the game window, as if the user pressed it (the editor reads keys from
/// WM_KEYDOWN/UP through GLFW's window procedure, scancode in lParam).
/// shortcut: assumes the default show/hide vehicle binding (H); read the binding table
/// (get_input_settings 0x21abb0, current key at +0x2C of each 0x58 record) if users rebind it.
fn press_game_key(vk: u8) {
    use windows_sys::Win32::UI::Input::KeyboardAndMouse::{MapVirtualKeyW, MAPVK_VK_TO_VSC};
    use windows_sys::Win32::UI::WindowsAndMessaging::{GetForegroundWindow, PostMessageW, WM_KEYDOWN, WM_KEYUP};
    unsafe {
        let hwnd = GetForegroundWindow();
        if hwnd.is_null() {
            return;
        }
        let sc = (MapVirtualKeyW(vk as u32, MAPVK_VK_TO_VSC) as isize) << 16;
        PostMessageW(hwnd, WM_KEYDOWN, vk as usize, 1 | sc);
        PostMessageW(hwnd, WM_KEYUP, vk as usize, 1 | sc | (3 << 30));
    }
}

/// Panel colour per aerofoil (index into `AEROFOILS`) in the aerofoil tool.
const FOIL_COLORS: [[u8; 3]; 9] = [[90, 150, 230], [230, 120, 40], [80, 200, 90], [200, 80, 200], [230, 220, 60], [60, 210, 210], [255, 255, 255], [240, 90, 110], [170, 170, 255]];

/// FAR-style lift (green, cyan on the fuselage) and drag (red) arrows, one per surface, totals at the CoM.
fn draw_force_arrows(ui: &Ui) {
    // Camera and alpha as the game drew this frame (taken around its draw, not now).
    let frame = crate::flight::frame_snap();
    let Some(vp) = frame.map(|f| f.vp).or_else(crate::flight::main_viewproj) else { return };
    let data = crate::flight::DEBUG.lock().map(|d| d.clone()).unwrap_or_default();
    publish_track(&data);
    let size = ui.io().display_size;
    let dl = ui.get_background_draw_list();
    let arrow = |from: V3, vec: V3, col: [f32; 4], thick: f32| {
        let (Some(a), Some(b)) = (view::project(&vp, from, size), view::project(&vp, from + vec, size)) else { return };
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
            if n < 1e-9 { V3::ZERO } else { f * (len * (1.0 + 4.0 * n / w).ln() / n) }
        };
        // Grid-local point on `body`, placed with the matrix the game drew it with (world fallback).
        let place_on = |body: usize, local: V3, world: V3| {
            smoothed_matrix(body, frame)
                .map(|m| V3 { x: local.x * m[0] + local.y * m[4] + local.z * m[8] + m[12], y: local.x * m[1] + local.y * m[5] + local.z * m[9] + m[13], z: local.x * m[2] + local.y * m[6] + local.z * m[10] + m[14] })
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
            let Some(s) = pt.and_then(|p| view::project(&vp, place(p), size)) else { continue };
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
    let Some(f) = f else { return crate::flight::body_render_matrix(body) };
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
        let Some(live) = crate::flight::body_render_matrix(a.body) else { continue };
        let h = hist.get(&a.body);
        let n = h.and_then(|h| h.last()).map_or(live, |x| x.1);
        let n1 = h.and_then(|h| h.len().checked_sub(2).map(|i| h[i].1)).unwrap_or(n);
        track.push((a.body, [n1, n]));
    }
    *crate::drawcap::TRACK.lock().unwrap_or_else(|p| p.into_inner()) = track;
}

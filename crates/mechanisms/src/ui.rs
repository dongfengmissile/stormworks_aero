//! Dear ImGui helpers and colours (the names `seqwin` expects; the UI mock gives it the aero
//! plugin's `ui.rs` instead).

use imgui::Ui;

use crate::math::V3;

pub(crate) const CL_COLOR: [f32; 4] = [0.30, 0.80, 0.95, 1.0];
pub(crate) const DIM: [f32; 4] = [0.60, 0.62, 0.66, 1.0];
pub(crate) const GOOD: [f32; 4] = [0.45, 0.85, 0.45, 1.0];
pub(crate) const BAD: [f32; 4] = [0.95, 0.40, 0.35, 1.0];
pub(crate) const WARN: [f32; 4] = [0.95, 0.70, 0.30, 1.0];

/// A tooltip wrapped to a readable width (imgui tooltips are one line per paragraph otherwise).
pub(crate) fn hover_tip(ui: &Ui, text: impl AsRef<str>) {
    ui.tooltip(|| {
        let _wrap = ui.push_text_wrap_pos_with_pos(ui.current_font_size() * 30.0);
        ui.text(text.as_ref());
    });
}

/// Screen position (pixels, `w` x `h`) and view depth of `p` through the game camera's clip
/// matrix (`clip = [p, 1]·m`); depth <= 0 is behind the camera.
pub(crate) fn project(m: &[f64; 16], w: f32, h: f32, p: V3) -> (f32, f32, f64) {
    let c = |k: usize| p.x * m[k] + p.y * m[4 + k] + p.z * m[8 + k] + m[12 + k];
    let cw = c(3);
    if cw <= 1e-6 {
        return (0.0, 0.0, -1.0);
    }
    ((w as f64 * (0.5 + 0.5 * c(0) / cw)) as f32, (h as f64 * (0.5 - 0.5 * c(1) / cw)) as f32, cw)
}

//! Widgets of the F7 / F8 / F9 windows that need only Dear ImGui. The UI mock (`tools/uimock`)
//! compiles this file too, so nothing here may name the rest of the crate.

use std::sync::atomic::{AtomicBool, Ordering};

use imgui::{TableColumnFlags, TableColumnSetup, TableFlags, TableRowFlags, Ui};

pub(crate) const CL_COLOR: [f32; 4] = [0.30, 0.80, 0.95, 1.0];
pub(crate) const CD_COLOR: [f32; 4] = [0.95, 0.40, 0.35, 1.0];
pub(crate) const CM_COLOR: [f32; 4] = [0.95, 0.80, 0.30, 1.0];
pub(crate) const LD_COLOR: [f32; 4] = [0.45, 0.85, 0.45, 1.0];
pub(crate) const DIM: [f32; 4] = [0.60, 0.62, 0.66, 1.0];
pub(crate) const GOOD: [f32; 4] = [0.45, 0.85, 0.45, 1.0];
pub(crate) const BAD: [f32; 4] = [0.95, 0.40, 0.35, 1.0];
pub(crate) const WARN: [f32; 4] = [0.95, 0.70, 0.30, 1.0];
/// Controls column width, and where its fields start.
pub(crate) const LEFT_W: f32 = 210.0;
pub(crate) const FIELD_X: f32 = 92.0;
/// Plot area inset from the plot's own left and right edges (tick labels).
pub(crate) const PLOT_PAD_L: f32 = 44.0;
pub(crate) const PLOT_PAD_R: f32 = 14.0;
/// Settings tab columns: the setting, its value for all vehicles, the workbench vehicle's own.
pub(crate) const SETTING_COLS: [(&str, f32); 3] = [("", 360.0), ("All vehicles", 100.0), ("This vehicle", 150.0)];
pub(crate) type Deriv = (&'static str, f64, i32, &'static str, &'static str);

/// A tooltip wrapped to a readable width (imgui tooltips are one line per paragraph otherwise).
pub(crate) fn hover_tip(ui: &Ui, text: impl AsRef<str>) {
    ui.tooltip(|| {
        let _wrap = ui.push_text_wrap_pos_with_pos(ui.current_font_size() * 30.0);
        ui.text(text.as_ref());
    });
}

/// F9 table: every table the same width, columns split by weight.
pub(crate) fn f9_table<'a>(ui: &'a Ui, id: &str, cols: &[(&str, f32)]) -> Option<imgui::TableToken<'a>> {
    let flags = TableFlags::ROW_BG | TableFlags::BORDERS_INNER_H | TableFlags::PAD_OUTER_X | TableFlags::SIZING_STRETCH_PROP;
    let t = ui.begin_table_with_sizing(id, cols.len(), flags, [560.0, 0.0], 0.0)?;
    for &(name, w) in cols {
        ui.table_setup_column_with(TableColumnSetup { flags: TableColumnFlags::WIDTH_STRETCH, init_width_or_weight: w, ..TableColumnSetup::new(name) });
    }
    Some(t)
}

/// Section title with a rule to the right edge.
pub(crate) fn heading(ui: &Ui, title: &str) {
    ui.dummy([0.0, 2.0]);
    ui.text(title);
    rule(ui);
}

/// One setting: its name (what it does on hover), the box for all
/// vehicles and, with `own`, the vehicle's Global / On / Off choice. `depth` indents a setting
/// under the one it needs; `enabled`: (the box, the vehicle's choice). True when the box changed.
#[allow(clippy::too_many_arguments)]
pub(crate) fn setting_row(ui: &Ui, depth: u8, name: &str, tip: &str, flag: &AtomicBool, enabled: (bool, bool), own: Option<&mut Option<bool>>) -> bool {
    ui.table_next_row();
    ui.table_set_column_index(0);
    let x = ui.cursor_pos()[0] + 14.0 * depth as f32;
    ui.set_cursor_pos([x, ui.cursor_pos()[1]]);
    ui.align_text_to_frame_padding();
    if enabled.0 || enabled.1 {
        ui.text(name);
    } else {
        ui.text_colored(DIM, name);
    }
    if ui.is_item_hovered() {
        hover_tip(ui, tip);
    }
    ui.table_set_column_index(1);
    let mut v = flag.load(Ordering::Relaxed);
    let changed = {
        let _d = ui.begin_disabled(!enabled.0);
        ui.checkbox(format!("##{name}"), &mut v)
    };
    if changed {
        flag.store(v, Ordering::Relaxed);
    }
    if let Some(own) = own {
        ui.table_set_column_index(2);
        let _d = ui.begin_disabled(!enabled.1);
        let names = [if v { "Global (on)" } else { "Global (off)" }, "On", "Off"];
        let mut i = match *own {
            None => 0,
            Some(true) => 1,
            Some(false) => 2,
        };
        ui.set_next_item_width(-1.0);
        if ui.combo_simple_string(format!("##own {name}"), &mut i, &names) {
            *own = [None, Some(true), Some(false)][i];
        }
    }
    changed
}

/// Hairline from the last item to the pane's right edge.
pub(crate) fn rule(ui: &Ui) {
    let (l, r) = (ui.item_rect_min(), ui.item_rect_max());
    let right = ui.window_pos()[0] + ui.window_content_region_max()[0];
    let y = (l[1] + r[1]) * 0.5;
    ui.get_window_draw_list().add_line([r[0] + 8.0, y], [right, y], [0.35, 0.37, 0.42, 1.0]).build();
}

/// Label on the left, value right-aligned in a colour.
pub(crate) fn kv_col(ui: &Ui, k: &str, v: &str, col: Option<[f32; 4]>) {
    ui.text_colored(DIM, k);
    ui.same_line_with_pos(ui.window_content_region_max()[0] - ui.calc_text_size(v)[0]);
    ui.text_colored(col.unwrap_or(ui.style_color(imgui::StyleColor::Text)), v);
}

/// Small colour square, then the label.
pub(crate) fn swatch(ui: &Ui, col: [f32; 4], label: &str) {
    let p = ui.cursor_screen_pos();
    let h = ui.text_line_height();
    ui.get_window_draw_list().add_rect([p[0], p[1] + 3.0], [p[0] + h - 6.0, p[1] + h - 3.0], col).filled(true).build();
    ui.dummy([h - 6.0, h]);
    ui.same_line_with_spacing(0.0, 6.0);
    ui.text(label);
}

pub(crate) fn rgb(c: [u8; 3]) -> [f32; 4] {
    [c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0, 1.0]
}

/// Label on the left, the next item from a fixed column to the right edge.
pub(crate) fn field(ui: &Ui, label: &str) {
    ui.align_text_to_frame_padding();
    ui.text_colored(DIM, label);
    ui.same_line_with_pos(FIELD_X);
    ui.set_next_item_width(-1.0);
}

pub(crate) fn input_f(ui: &Ui, label: &str, v: &mut f32, fmt: &str) -> bool {
    field(ui, label);
    ui.input_float(format!("##{label}"), v).display_format(fmt).build()
}

/// Label on the left, value right-aligned.
pub(crate) fn kv(ui: &Ui, k: &str, v: &str) {
    kv_col(ui, k, v, None);
}

pub(crate) fn sci(v: f64) -> String {
    if v.abs() >= 1e5 {
        let e = v.abs().log10().floor() as i32;
        format!("{:.3}e{e}", v / 10f64.powi(e))
    } else {
        format!("{v:.0}")
    }
}

/// The tab's one action, full width.
pub(crate) fn primary(ui: &Ui, label: &str) -> bool {
    ui.dummy([0.0, 6.0]);
    ui.button_with_size(label, [-1.0, 26.0])
}

/// Text right-aligned in the current table cell.
pub(crate) fn right(ui: &Ui, s: &str, col: Option<[f32; 4]>) {
    let w = ui.calc_text_size(s)[0];
    ui.set_cursor_pos([ui.cursor_pos()[0] + ui.content_region_avail()[0] - w, ui.cursor_pos()[1]]);
    match col {
        Some(c) => ui.text_colored(c, s),
        None => ui.text(s),
    }
}

/// Table with the given columns: a width > 0 is fixed, 0 stretches.
pub(crate) fn table<'a>(ui: &'a Ui, id: &str, cols: &[(&str, f32)]) -> Option<imgui::TableToken<'a>> {
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
pub(crate) fn headers(ui: &Ui, cols: &[(&str, f32)], left_cols: usize) {
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

/// Stability derivatives: rows are the force or moment, columns the cause. Green when the
/// sign is the stabilising one, red when not; the tooltip names and explains each.
pub(crate) fn deriv_table<const N: usize>(ui: &Ui, id: &str, heads: &[&str], rows: &[(&str, [Deriv; N])]) {
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
                hover_tip(ui, format!("{name} = {v:.4} {unit}\n{tip}"));
            }
        }
    }
}

/// Line plot: grid, zero line, legend along the top, hover readout. Series past `legend`
/// are drawn but not named. Non-finite values break the line.
pub(crate) fn plot(ui: &Ui, id: &str, size: [f32; 2], x: &[f64], series: &[(&str, [f32; 4], Vec<f64>)], legend: usize, xlabel: &str) {
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
        hover_tip(ui, text);
    }
}

pub(crate) fn fmt_tick(v: f64) -> String {
    if v.abs() >= 100.0 || (v - v.round()).abs() < 1e-9 {
        format!("{v:.0}")
    } else if (v * 10.0 - (v * 10.0).round()).abs() < 1e-9 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

/// A small button right-aligned in the current table cell.
pub(crate) fn right_button(ui: &Ui, label: &str) -> bool {
    let shown = label.split("##").next().unwrap_or(label);
    let w = ui.calc_text_size(shown)[0] + ui.clone_style().frame_padding[0] * 2.0;
    ui.set_cursor_pos([ui.cursor_pos()[0] + ui.content_region_avail()[0] - w, ui.cursor_pos()[1]]);
    ui.small_button(label)
}

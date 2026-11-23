//! The plugin's Dear ImGui overlay (hudhook, its own beside the aero plugin's): the joint labels and
//! the Sequences window while the Mechanisms mode is on.

use std::sync::atomic::Ordering;

use hudhook::imgui::{Context, Io};
use hudhook::{ImguiRenderLoop, MessageFilter, RenderContext};

use crate::logging::log_line;

#[derive(Default)]
pub(crate) struct Overlay {
    /// The label icons read back so far, and the atlas texture each came from.
    icons_tried: Vec<(&'static str, u32)>,
}

impl ImguiRenderLoop for Overlay {
    fn initialize<'a>(&'a mut self, ctx: &mut Context, _render_context: &'a mut dyn RenderContext) {
        ctx.set_ini_filename(None);
        log_line("overlay initialised");
    }

    fn before_render<'a>(&'a mut self, _ctx: &mut Context, render_context: &'a mut dyn RenderContext) {
        // The game's icons for the labels, read back from their atlases once.
        let icons = crate::toolbar::LABEL_ICONS.lock().unwrap_or_else(|p| p.into_inner()).clone();
        for (name, tex, uv) in icons {
            if self.icons_tried.contains(&(name, tex)) {
                continue;
            }
            self.icons_tried.push((name, tex));
            let id = stormworks_core::toolbar::read_region(tex, uv).and_then(|(px, w, h)| render_context.load_texture(&px, w, h).ok());
            log_line(&format!("mechanisms: icon {name} {}", if id.is_some() { "loaded" } else { "not readable; labels use a plain mark" }));
            let mut have = crate::mechmode::ICONS.lock().unwrap_or_else(|p| p.into_inner());
            have.retain(|(n, _)| *n != name);
            have.extend(id.map(|id| (name, id)));
        }
    }

    fn render(&mut self, ui: &mut hudhook::imgui::Ui) {
        #[cfg(feature = "logging")]
        crate::hang::FRAMES.fetch_add(1, Ordering::Relaxed);
        // A panic here would unwind into the game's wglSwapBuffers and abort it.
        let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _fault = crate::fault::enter(crate::fault::stage::OVERLAY_RENDER);
            crate::paint::new_frame();
            // imgui-rs builds the draw-list slice from a null pointer when a frame draws nothing,
            // which panics under debug assertions (and is UB anyway): always emit one invisible quad.
            ui.get_foreground_draw_list().add_rect([-4.0, -4.0], [-3.0, -3.0], [0.0, 0.0, 0.0, 0.01]).filled(true).build();
            let on = crate::toolbar::mech_active();
            crate::editor::WANTED.store(on, Ordering::Relaxed);
            crate::editor::PICK_WANTED.store(on, Ordering::Relaxed);
            crate::mechmode::SHIFT.store(crate::plugin::MECH_KEY.down(ui), Ordering::Relaxed);
            if on {
                crate::mechmode::draw_fix_hint(ui);
                crate::mechmode::draw_labels(ui);
                crate::mechmode::draw_sequence(ui);
            }
        }))
        .is_ok();
        if !ok {
            crate::editor::WANTED.store(false, Ordering::Relaxed);
            log_line("overlay frame panicked (see the PANIC line above)");
        }
    }

    fn message_filter(&self, io: &Io) -> MessageFilter {
        let mut f = MessageFilter::empty();
        if !crate::toolbar::mech_active() {
            return f;
        }
        if io.want_capture_mouse {
            f |= MessageFilter::InputMouse | MessageFilter::InputRaw;
        }
        // While a field has focus the workbench gets no keys (typing a name must not build).
        if io.want_text_input || io.want_capture_keyboard {
            f |= MessageFilter::InputKeyboard | MessageFilter::InputRaw;
        }
        f
    }
}

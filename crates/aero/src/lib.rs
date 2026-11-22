//! Stormworks aerodynamics plugin: a FAR-style (Ferram Aerospace Research) voxel
//! aerodynamics model and workbench analysis menu.
//!
//! Platform-independent core (tested on the host): the `pub mod`s below. Windows-only plugin
//! (`#[cfg(windows)]`): bootstrap, game hooks, ImGui overlay.

#![allow(non_snake_case)] // crate name matches the DLL name

pub use stormworks_core::{defs, joints, math, pose, shape, vehicle, voxel, xml};
pub mod body;
pub mod fx;
pub mod fxwing;
pub mod marks;
pub mod mech;
pub mod model;
pub mod prop;
pub mod settings;
pub mod spool;
pub mod view;
pub mod wing;

#[cfg(windows)]
mod blur;
#[cfg(windows)]
mod drawcap;
#[cfg(windows)]
mod editor;
#[cfg(windows)]
mod fault;
#[cfg(windows)]
mod flight;
#[cfg(windows)]
mod fxdraw;
/// The effects' shaders (public for tools/wingsim, which draws them outside the game).
pub mod fxglsl;
#[cfg(all(windows, feature = "logging"))]
mod hang;
#[cfg(windows)]
mod hooks;
#[cfg(windows)]
mod logging;
#[cfg(windows)]
mod markhooks;
#[cfg(windows)]
mod memory;
#[cfg(windows)]
mod offsets;
#[cfg(windows)]
mod overlay;
#[cfg(windows)]
mod paint;
#[cfg(windows)]
#[cfg(windows)]
mod plugin;
#[cfg(windows)]
mod watch;
#[cfg(windows)]
mod state;
#[cfg(windows)]
mod toolbar;
#[cfg(windows)]
mod mechmode;
#[cfg(windows)]
mod seqwin;
#[cfg(windows)]
mod sound;
#[cfg(windows)]
mod ui;

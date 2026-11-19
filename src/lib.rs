//! Stormworks aerodynamics plugin: a FAR-style (Ferram Aerospace Research) voxel
//! aerodynamics model and workbench analysis menu.
//!
//! Platform-independent core (tested on the host): the `pub mod`s below. Windows-only plugin
//! (`#[cfg(windows)]`): bootstrap, game hooks, ImGui overlay.

#![allow(non_snake_case)] // crate name matches the DLL name

pub mod body;
pub mod defs;
pub mod fx;
pub mod fxwing;
pub mod marks;
pub mod math;
pub mod model;
pub mod prop;
pub mod settings;
pub mod shape;
pub mod spool;
pub mod vehicle;
pub mod view;
pub mod voxel;
pub mod wing;
pub mod xml;

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
mod ui;

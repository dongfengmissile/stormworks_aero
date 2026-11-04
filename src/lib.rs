//! Stormworks aerodynamics plugin: a FAR-style (Ferram Aerospace Research) voxel
//! aerodynamics model and workbench analysis menu.
//!
//! Platform-independent core (tested on the host): [`defs`], [`vehicle`], [`voxel`],
//! [`body`], [`wing`], [`model`], [`view`]. Windows-only plugin: bootstrap, editor hook,
//! ImGui overlay.

#![allow(non_snake_case)] // crate name matches the DLL name

pub mod body;
pub mod defs;
pub mod math;
pub mod model;
pub mod shape;
pub mod spool;
pub mod vehicle;
pub mod view;
pub mod voxel;
pub mod wing;
pub mod xml;

#[cfg(windows)]
mod editor;
#[cfg(windows)]
mod fault;
#[cfg(windows)]
mod drawcap;
#[cfg(windows)]
mod flight;
#[cfg(windows)]
mod hooks;
#[cfg(windows)]
mod logging;
#[cfg(windows)]
mod memory;
#[cfg(windows)]
mod offsets;
#[cfg(windows)]
mod overlay;
#[cfg(windows)]
mod paint;
#[cfg(windows)]
mod plugin;
#[cfg(windows)]
mod state;
#[cfg(windows)]
mod toolbar;

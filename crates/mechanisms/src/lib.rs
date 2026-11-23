//! Stormworks mechanisms plugin: the workbench's joints checked like a CAD assembly (which are
//! locked, how far each sub-grid moves before it hits something), sub-grids shown at their joints'
//! workbench settings, and sequences that move the joints one after another.
//!
//! Platform-independent core (tested on the host): `mech`, `marks`. Windows-only plugin: bootstrap,
//! game hooks, ImGui overlay.
#![allow(non_snake_case)] // crate name matches the DLL name
pub use stormworks_core::{defs, joints, math, pose, shape, vehicle, voxel, xml};
pub mod marks;
pub mod mech;
#[cfg(windows)]
use stormworks_core::{editor, fault, hook, logging, markhooks, memory, offsets, paint, state};
#[cfg(all(windows, feature = "logging"))]
use stormworks_core::hang;
#[cfg(all(windows, feature = "logging"))]
mod contacts;
#[cfg(windows)]
mod hooks;
#[cfg(windows)]
mod mechmode;
#[cfg(windows)]
mod overlay;
#[cfg(windows)]
mod plugin;
#[cfg(windows)]
mod posing;
#[cfg(windows)]
mod seqwin;
#[cfg(windows)]
mod sound;
#[cfg(windows)]
mod toolbar;
#[cfg(windows)]
mod ui;


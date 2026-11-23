//! Shared by the Stormworks plugins (aero, mechanisms): the game's part definitions and vehicle
//! files, voxels and collision shapes, the joint tree and its poses. Platform independent, tested
//! on the host.
pub mod defs;
pub mod joints;
pub mod marks;
pub mod math;
pub mod pose;
pub mod shape;
pub mod vehicle;
pub mod voxel;
pub mod xml;
#[cfg(windows)]
pub mod fault;
#[cfg(all(windows, feature = "logging"))]
pub mod hang;
#[cfg(windows)]
pub mod hook;
#[cfg(windows)]
pub mod logging;
#[cfg(windows)]
pub mod memory;
#[cfg(windows)]
pub mod state;
#[cfg(windows)]
pub mod editor;
#[cfg(windows)]
pub mod markhooks;
#[cfg(windows)]
pub mod offsets;
#[cfg(windows)]
pub mod paint;
#[cfg(windows)]
pub mod toolbar;
#[cfg(windows)]
pub mod unload;
#[cfg(windows)]
pub mod plugin;
#[cfg(all(windows, feature = "swml"))]
pub mod swml;

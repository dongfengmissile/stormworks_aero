//! Shared by the Stormworks plugins (aero, mechanisms): the game's part definitions and vehicle
//! files, voxels and collision shapes, the joint tree and its poses. Platform independent, tested
//! on the host.
pub mod defs;
pub mod joints;
pub mod math;
pub mod pose;
pub mod shape;
pub mod vehicle;
pub mod voxel;
pub mod xml;

//! Every fact about `stormworks64.exe` lives here, so a game update is a one-file diff.
//! Source: re-workbench pass on a fresh IDA database (`stormworks_aero_idapro/`).

/// stormworks64.exe sha256 c38c03ff756b4f42bbc9f11cdbb034977ad0f968e45741c5cf410732f22a338d
/// (v1.15.25 hotfix).
pub(crate) const GAME_BUILD_LABEL: &str = "c38c03ff756b";

/// Compile-time stamp of THIS binary, logged at bootstrap: tells which DLL the game loaded.
pub(crate) const PLUGIN_BUILD_LABEL: &str = env!("AERO_BUILD_STAMP");

/// `game_state_vehicle_editor_update(editor, a2, a3, a4, a5, a6)`: the editor state's
/// per-tick update (vtable slot 6), on the thread that edits the vehicle. All args are
/// integer-class (call site 0x86ad4d: rcx, rdx, r8, r9 + two stack pointers).
pub(crate) const RVA_EDITOR_UPDATE: usize = 0x007d_6740;
/// No rip-relative or rel32 bytes in this window, so nothing is wildcarded.
pub(crate) const EDITOR_UPDATE_PROLOGUE: [u8; 32] = [
    0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48,
    0x8d, 0xa8, 0x68, 0xf5, 0xff, 0xff, 0x48, 0x81, 0xec, 0x58, 0x0b, 0x00, 0x00, 0x0f, 0x29, 0x70,
];

/// Editor state -> the vehicle being edited (0x408-byte vehicle; replaced on load/new).
pub(crate) const OFF_EDITOR_VEHICLE: usize = 0x13C8;
/// Ring buffers `{T** items; u32 mod; u32 offset; u32 count}`: element i = items[(i+offset)%mod].
pub(crate) const OFF_VEHICLE_BODIES: usize = 0x00;
pub(crate) const OFF_BODY_COMPONENTS: usize = 0x3F0;
pub(crate) const OFF_COMP_POS: usize = 0x18; // int3 voxel position
pub(crate) const OFF_COMP_ROT: usize = 0x30; // int[9] row-major voxel rotation
pub(crate) const OFF_COMP_DEF: usize = 0x58; // c_scene_tile_definition* (a mirror variant)
/// u8 mirror mask of the definition variant = the XML `t` (1 x, 2 y, 4 z); comp+0x30 is the
/// raw `r` without it (definition_init_variant 0x317f30, component_def_ref_serialize 0x4bb180).
pub(crate) const OFF_DEF_MIRROR: usize = 0x40;
/// Definition name as written to the vehicle XML `d` attribute: `{char* data; u32 len}`.
pub(crate) const OFF_DEF_NAME: usize = 0x288;

/// Editor camera (`mm_camera_perspective`) inside the editor state; view and projection are
/// f32 4x4 row-major, row vectors (`clip = p·V·P`, OpenGL NDC, clip.w = -z_eye).
/// Written each tick by editor_camera_update 0x7f39e0 (re-workbench round 2).
pub(crate) const OFF_EDITOR_CAMERA: usize = 0x78;
pub(crate) const OFF_CAM_VIEW: usize = 0x88;
pub(crate) const OFF_CAM_PROJ: usize = 0xC8;
/// Vehicle world transform, f64 4x4 row-major (copied from editor+0xCD8 on reset).
pub(crate) const OFF_VEHICLE_WORLD: usize = 0x190;
/// Body transform in its vehicle, f32 4x4 row-major: world = (voxel·0.25)·Body·Vehicle.
pub(crate) const OFF_BODY_MATRIX: usize = 0x2C8;

// ---- In-game flight forces (re-workbench round 4: vanilla air path + Bullet).

/// `physics_scene_post_step_sync(physics_scene)`: update thread, right after the physics wait
/// and before `scene_vehicle_update`. Physics is idle here, and forces put on a btRigidBody now
/// last through the next step (clearForces runs once at the end of stepSimulation).
/// Not hooked by the infrared plugin (it hooks scene_vehicle_update 0x5ebd20).
pub(crate) const RVA_POST_STEP_SYNC: usize = 0x001a_3a10;
/// Up to the first `call rel32` (at +0x16).
pub(crate) const POST_STEP_SYNC_PROLOGUE: [u8; 22] = [
    0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8d, 0x51, 0x48, 0x48, 0x89, 0x6c, 0x24, 0x38, 0x48, 0x8b, 0xf9,
];
/// The physics scene inside c_scene.
pub(crate) const OFF_SCENE_PHYSICS: usize = 0x64D0;
/// c_scene vehicle container ring (container 0x4990 + 0x320): records
/// `{u8 removed; u32 vehicle_id @+4; vehicle* @+8; u32 type @+0x10 (5 = vehicle)}`.
pub(crate) const OFF_SCENE_VEHICLES: usize = 0x4CB0;

/// `physics_scene_set_damping(scene, obj, f32 linear, f32 angular)`: Bullet velocity damping,
/// re-set on every vehicle body every tick (vanilla 0.1 = ~10 %/s speed loss).
pub(crate) const RVA_SET_DAMPING: usize = 0x001a_73e0;
pub(crate) const SET_DAMPING_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xfa, 0xf3, 0x0f, 0x10, 0x82, 0xb0, 0x00, 0x00, 0x00, 0x0f, 0x2e, 0xc2, 0x7a, 0x11, 0x75,
];

/// `btRigidBody::applyForce(rb, const double force[4], const double rel_pos[4])`, double precision;
/// rel_pos from the centre of mass, world axes.
pub(crate) const RVA_APPLY_FORCE: usize = 0x0019_a220;
pub(crate) const APPLY_FORCE_PROLOGUE: [u8; 32] = [
    0x48, 0x83, 0xec, 0x48, 0xf2, 0x0f, 0x10, 0x4a, 0x08, 0xf2, 0x0f, 0x10, 0x5a, 0x10, 0xf2, 0x0f, 0x10, 0xa1, 0xf0, 0x03, 0x00, 0x00, 0xf2, 0x0f, 0x10, 0x02, 0xf2, 0x0f, 0x59, 0x81, 0xe0, 0x03,
];

/// Game settings singleton (`get_game_settings` 0x296940): the vanilla air-force constants,
/// f32, re-read every tick by `air_face_force` 0x490b50: linear/quadratic pressure drag,
/// linear/quadratic suction drag, viscous air resistance, lift. Water uses other fields.
pub(crate) const RVA_GAME_SETTINGS: usize = 0x00d0_5830;
pub(crate) const AIR_CONSTANTS: [usize; 6] = [0x60, 0x64, 0x68, 0x6C, 0xE8, 0xEC];

pub(crate) const OFF_BODY_PHYS_OBJ: usize = 0x718; // mm_physics_scene_object*
pub(crate) const OFF_BODY_ORIGIN: usize = 0x720; // f32x3: world = (voxel·0.25 − origin)·M
pub(crate) const OFF_BODY_NO_FORCE: usize = 0x1478; // u8: vanilla skips force flush when set
pub(crate) const OFF_BODY_WIND: usize = 0x14D0; // f32x3 wind velocity, world, this tick
pub(crate) const OFF_OBJ_MASS: usize = 0x08; // f32
pub(crate) const OFF_OBJ_WORLD: usize = 0x18; // f64[16] row-major, row vectors
pub(crate) const OFF_OBJ_RIGID_BODY: usize = 0xB8; // btRigidBody*
pub(crate) const OFF_RB_COM: usize = 0x70; // f64x3 centre of mass (worldTransform origin)
pub(crate) const OFF_RB_FLAGS: usize = 0x1A0; // u32, 0x800 = externally driven
pub(crate) const OFF_RB_ACTIVATION: usize = 0x1B0; // i32, 2 = sleeping
pub(crate) const OFF_RB_LIN_VEL: usize = 0x390; // f64x3
pub(crate) const OFF_RB_ANG_VEL: usize = 0x3B0; // f64x3
pub(crate) const OFF_RB_TOTAL_TORQUE: usize = 0x480; // f64x3
pub(crate) const OFF_RB_ANGULAR_FACTOR: usize = 0x560; // f64x3

// Main in-game camera (render thread). c_game_state_simulation vtable slot 7 gets G in rcx;
// G+0x1010 is the main view's mm_camera_perspective, +0x1C8 its f64 viewproj (world space, row vectors p·M).
pub(crate) const RVA_SIM_RENDER_SLOT7: usize = 0x007b_1fc0;
pub(crate) const SIM_RENDER_SLOT7_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8d,
    0x9a, 0xa0, 0x01, 0x00, 0x00, 0x48, 0x8b, 0xf2, 0x48, 0x8b,
];
pub(crate) const RVA_SIM_STATE_VTABLE: usize = 0x00b1_a988;
pub(crate) const OFF_SIM_MAIN_CAMERA: usize = 0x1010;
pub(crate) const OFF_CAM_VIEWPROJ_F64: usize = 0x1C8;
pub(crate) const OFF_BODY_VEHICLE: usize = 0x250; // vehicle*
pub(crate) const OFF_BODY_RENDER: usize = 0x2C8; // f32[16] body -> vehicle (render); world = grid_m·this·vehicle+0x190
// Per-frame draw interpolation (render thread): mat_world = matrix_interp_prev_cur(prev, cur, *(frame+0x89C)),
// frame = *(G+0x5C0). Each frame build snapshots cur = vehicle+0x190 · body+0x2C8, then copies it to
// vehicle+0x210 / body+0x308, so those hold the latest built pose until the next build.
pub(crate) const RVA_MATRIX_INTERP: usize = 0x001e_7ec0; // (f32* prev, f32* cur, f32 alpha (xmm2), f32* out)
pub(crate) const MATRIX_INTERP_PROLOGUE: [u8; 18] = [0x4c, 0x8b, 0xdc, 0x53, 0x56, 0x57, 0x48, 0x81, 0xec, 0xf0, 0x00, 0x00, 0x00, 0x41, 0x0f, 0x29, 0x73, 0xd8];
pub(crate) const OFF_SIM_FRAME: usize = 0x5C0;
pub(crate) const OFF_FRAME_ALPHA: usize = 0x89C; // f32, written each render frame by slot 7
pub(crate) const OFF_FRAME_BUILD_NS: usize = 0xED0; // u64, time of the last frame build
pub(crate) const OFF_VEHICLE_WORLD_BUILT: usize = 0x210; // f64[16]
pub(crate) const OFF_BODY_RENDER_BUILT: usize = 0x308; // f32[16]

// Engine thrust (RE round 6): components queue world-frame forces through these physics-scene wrappers
// (scene, phys_obj, f32[3]* force, ptr, ptr, f32 max, f32 scale); return addresses identify the source.
pub(crate) const RVA_APPLY_FORCE_THRESHOLD: usize = 0x001a_7b20;
pub(crate) const APPLY_FORCE_THRESHOLD_PROLOGUE: [u8; 20] =
    [0x48, 0x89, 0x5c, 0x24, 0x10, 0x48, 0x89, 0x6c, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83, 0xec, 0x60];
pub(crate) const RVA_APPLY_FORCE_CLAMPED: usize = 0x001a_7980;
pub(crate) const APPLY_FORCE_CLAMPED_PROLOGUE: [u8; 20] =
    [0x48, 0x89, 0x5c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x60, 0x0f, 0x29, 0x74, 0x24, 0x50];
/// Thrust call sites by return address (each verified to follow a call to the named wrapper), with a source bit:
/// 1 jet engine tick (0x3534C0), 2 solid rocket tick (0x439DF0), 4 propeller/rotor main thrust (force_emitter
/// 0x341530; its two blade lift/drag forces at 0x342341/0x3423A4 are left out), 8 RCS and 16 water jet / fluid
/// reaction (water component 0x3F1FA0; fluid sub-types not mapped to part names, low confidence).
pub(crate) const THRESHOLD_THRUST_RETURNS: [(usize, u8); 4] = [(0x0035_451f, 1), (0x0035_6442, 1), (0x0034_1eb0, 4), (0x003f_6cbc, 16)];
pub(crate) const CLAMPED_THRUST_RETURNS: [(usize, u8); 6] = [(0x0043_a652, 2), (0x0043_b007, 2), (0x0043_cdaa, 2), (0x0043_cef8, 2), (0x003f_60d0, 8), (0x003f_3df9, 16)];
pub(crate) const THRUST_SOURCE_NAMES: [&str; 5] = ["jet", "rocket", "propeller/rotor", "RCS", "water jet"];

// Modular jet (RE round 8): the nozzle's thrust call passes `this + 0x510` in r9 (arg 4). Component def type at
// *(this+0x58)+0xED4 (u32: 0 fan, 1 compressor, 2 combustor, 3 turbine, 4 nozzle); shaft RPS f32 at +0x1BC on
// types 1-3; upstream neighbours ring at +0x408 ({items, mod, offset, count}).
pub(crate) const JET_NOZZLE_RETURN: usize = 0x0035_451f;
pub(crate) const JET_THIS_FROM_ARG4: usize = 0x510;
pub(crate) const OFF_JET_DEF_TYPE: usize = 0xED4;
pub(crate) const OFF_JET_RPS: usize = 0x1BC;
pub(crate) const OFF_JET_UPSTREAM: usize = 0x408;

// Ground effect (RE round 9): closest-hit ray on the physics scene, f64 world coordinates, update thread:
// u8 f(physics_scene, f64 ray[6] {from, to}, f64 hit[3] out, mm_physics_scene_object** out, i32 group, i32 mask).
pub(crate) const RVA_RAYCAST_CLOSEST: usize = 0x001a_4010;
pub(crate) const RAYCAST_CLOSEST_PROLOGUE: [u8; 21] = [0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x48, 0x8d, 0x68, 0xa8, 0x48, 0x81, 0xec, 0x40, 0x01, 0x00, 0x00];
// Water surface height: f32 f(c_scene + 0x68A0, const f64 pos[3]).
pub(crate) const RVA_WATER_HEIGHT: usize = 0x002b_ebf0;
pub(crate) const WATER_HEIGHT_PROLOGUE: [u8; 17] = [0x4c, 0x8b, 0xdc, 0x48, 0x83, 0xec, 0x68, 0x0f, 0x57, 0xc0, 0x49, 0x8d, 0x43, 0x18, 0x4c, 0x8b, 0xc2];
pub(crate) const OFF_SCENE_WATER_FIELD: usize = 0x68A0;
pub(crate) const OFF_SETTINGS_AIR_DENSITY: usize = 0xC4; // f32 sea-level air density (game constant "air_density", 1.225)

/// `opaque_chunk_draw_item(bindset, mesh, first, count, mat_world, prev, next, light, c1, c2, c3,
/// is_preview, is_force_color, is_disable_color_replace, _, wire)`: every vehicle chunk draw.
pub(crate) const RVA_CHUNK_DRAW: usize = 0x0068_0c00;
pub(crate) const CHUNK_DRAW_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x40, 0x48, 0x8b, 0x41, 0x40, 0x41, 0x8b, 0xf9, 0x4c, 0x8b, 0x4c, 0x24, 0x70,
];

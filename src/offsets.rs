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
    0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x68, 0xf5, 0xff, 0xff, 0x48, 0x81, 0xec, 0x58,
    0x0b, 0x00, 0x00, 0x0f, 0x29, 0x70,
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
pub(crate) const OFF_DEF_CLASS: usize = 0x2AC; // i32 component class (XML `type`)
                                               // Type-2 force emitter (propeller / rotor) state, knowledge/vehicle-model.md "Type 2 force emitters".
pub(crate) const OFF_PROP_SPEED: usize = 0x150; // f32 shaft speed w (game units; blades turn w·rotation_speed rev/s)
/// f32 drawn hub angle rad (`fmodf 2π`), only for drawing; the game turns it by
/// `w_prev·rotation_speed·0.5/60` per tick for air props: 4π slower than the shaft (c38c03ff756b) [high].
pub(crate) const OFF_PROP_ANGLE: usize = 0x148;
/// Type-2 def (`comp+0x58`): `force_dir` f32[3]; `*def + OFF_DEF_MESH1` = mesh_1 object, its
/// `+0x30` -> the mesh holder the render pushes get as `a2` [medium] (`[rotor-blur RE]`).
pub(crate) const OFF_DEF_FORCE_DIR: usize = 0x850;
/// Hinge constraint (`*(comp+0x2F0)`) angle f32 rad, drawn as hub angle `−angle·sy` for air
/// props (`force_emitter_draw` 0x343AFC; sy from `OFF_DEF_MIRROR` bit 2) [high].
pub(crate) const OFF_HINGE_ANGLE: usize = 0x120;
pub(crate) const OFF_DEF_MESH1: usize = 0x580;
/// Rotor blur draw path (c38c03ff756b, `[rotor-blur RE]`): `force_emitter_draw` (class draw ABI),
/// `push_render_item_232` (16 args, c1..c3 = [0x40..0x50], a14 cull flag), its two shadow pushes
/// right after it (matA = 5th arg, same pointer), and `push_item240_to_0x180_dither` (13 args,
/// [0x60] f32 opacity) into the view queue ring `+0x180` of 240-byte items (`+0xDC` cull flag).
pub(crate) const RVA_FORCE_EMITTER_DRAW: usize = 0x0034_3930;
pub(crate) const FORCE_EMITTER_DRAW_PROLOGUE: [u8; 24] =
    [0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0x08, 0xF9, 0xFF, 0xFF, 0x48, 0x81, 0xEC];
pub(crate) const RVA_PUSH_232: usize = 0x0067_2270;
pub(crate) const RVA_PUSH_SHADOW_A: usize = 0x0067_3080;
pub(crate) const RVA_PUSH_SHADOW_B: usize = 0x0067_3920;
/// The three pushes share these bytes but for byte 28 (the frame offset): f9 / f1 / e9.
pub(crate) const PUSH_PROLOGUE: [u8; 36] = [
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D,
    0x6C, 0x24, 0xF9, 0x48, 0x81, 0xEC, 0xF0, 0x00, 0x00, 0x00,
];
pub(crate) const RVA_PUSH_DITHER: usize = 0x0067_1dc0;
pub(crate) const PUSH_DITHER_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x6C, 0x24, 0xF9, 0x48,
    0x81, 0xEC, 0xF0, 0x00, 0x00, 0x00,
];
pub(crate) const OFF_QUEUE_DITHER_RING: usize = 0x180;
/// Type-2 def `flags` u32 (bit 2: marine prop) [high].
pub(crate) const OFF_DEF_PROP_FLAGS: usize = 0x2B8;
pub(crate) const OFF_PROP_CYCLIC: usize = 0x258; // f32 ×2 cyclic x / y input
pub(crate) const OFF_PROP_COLLECTIVE: usize = 0x260; // f32 live pitch input
pub(crate) const OFF_PROP_BLADES: usize = 0x264; // u32 blade_count
pub(crate) const OFF_PROP_PITCH: usize = 0x268; // i32 blade_pitch setting
pub(crate) const OFF_PROP_FOLDED: usize = 0x270; // u8
pub(crate) const OFF_PROP_LENGTH: usize = 0x294; // f32 blade_length
pub(crate) const OFF_PROP_HINGE: usize = 0x2F0; // ptr: hub spun by a hinge motor
pub(crate) const OFF_PROP_ACTIVE: usize = 0x304; // u32 active_rotors mask
pub(crate) const OFF_PROP_LINK: usize = 0x1D8; // torque slot link (0 = no shaft)
pub(crate) const OFF_PROP_SHAFT: usize = 0x1E8; // f32 shaft speed from the network
pub(crate) const OFF_DEF_ROTATION_SPEED: usize = 0x864; // f32 force_emitter_rotation_speed [high]
pub(crate) const OFF_PROP_REVERSE: usize = 0x758; // u8 thrust reversed
pub(crate) const OFF_PROP_FORCE_ACC: usize = 0x760; // the thrust call's 4th argument is comp + this
pub(crate) const RVA_PROP_TICK: usize = 0x0034_1530; // force_emitter_tick_propeller_rotor (vtable slot 108)
pub(crate) const PROP_TICK_PROLOGUE: [u8; 24] =
    [0x4C, 0x8B, 0xDC, 0x55, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x57, 0x49, 0x8D, 0xAB, 0x28, 0xFF, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xB0, 0x01, 0x00];
pub(crate) const PROP_THRUST_RETURN: usize = 0x0034_1eb0; // apply_force_threshold, thrust; arg 4 = comp + 0x760
pub(crate) const PROP_COUPLE_RETURNS: [usize; 2] = [0x0034_2341, 0x0034_23a4]; // apply_force_clamped, coaxial couple

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
pub(crate) const POST_STEP_SYNC_PROLOGUE: [u8; 22] =
    [0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8d, 0x51, 0x48, 0x48, 0x89, 0x6c, 0x24, 0x38, 0x48, 0x8b, 0xf9];
/// The physics scene inside c_scene.
pub(crate) const OFF_SCENE_PHYSICS: usize = 0x64D0;
/// c_scene vehicle container ring (container 0x4990 + 0x320): records
/// `{u8 removed; u32 vehicle_id @+4; vehicle* @+8; u32 type @+0x10 (5 = vehicle)}`.
pub(crate) const OFF_SCENE_VEHICLES: usize = 0x4CB0;

/// `physics_scene_set_damping(scene, obj, f32 linear, f32 angular)`: Bullet velocity damping,
/// re-set on every vehicle body every tick (vanilla 0.1 = ~10 %/s speed loss).
pub(crate) const RVA_SET_DAMPING: usize = 0x001a_73e0;
pub(crate) const SET_DAMPING_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xfa, 0xf3, 0x0f, 0x10, 0x82, 0xb0, 0x00, 0x00, 0x00,
    0x0f, 0x2e, 0xc2, 0x7a, 0x11, 0x75,
];

/// `fluid_air_node_forces(ctx, node, matrix, water, _)`, void: one octree node's hull-face air
/// forces (worker threads); `*ctx` = `body + OFF_BODY_AIR_RECORDS` (c38c03ff756b) [high].
pub(crate) const RVA_AIR_NODE_FORCES: usize = 0x0049_08b0;
pub(crate) const AIR_NODE_FORCES_PROLOGUE: [u8; 24] =
    [0x48, 0x8b, 0xc4, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xec, 0x00, 0x01, 0x00, 0x00, 0x80, 0xb9, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x8b];
pub(crate) const OFF_BODY_AIR_RECORDS: usize = 0x11f0;

/// `btRigidBody::applyForce(rb, const double force[4], const double rel_pos[4])`, double precision;
/// rel_pos from the centre of mass, world axes.
pub(crate) const RVA_APPLY_FORCE: usize = 0x0019_a220;
pub(crate) const APPLY_FORCE_PROLOGUE: [u8; 32] = [
    0x48, 0x83, 0xec, 0x48, 0xf2, 0x0f, 0x10, 0x4a, 0x08, 0xf2, 0x0f, 0x10, 0x5a, 0x10, 0xf2, 0x0f, 0x10, 0xa1, 0xf0, 0x03, 0x00, 0x00, 0xf2, 0x0f, 0x10, 0x02,
    0xf2, 0x0f, 0x59, 0x81, 0xe0, 0x03,
];

/// Game settings singleton (`get_game_settings` 0x296940): the vanilla air-force constants,
/// f32, re-read every tick by `air_face_force` 0x490b50: linear/quadratic pressure drag,
/// linear/quadratic suction drag, viscous air resistance, lift. Water uses other fields.
pub(crate) const RVA_GAME_SETTINGS: usize = 0x00d0_5830;
pub(crate) const AIR_CONSTANTS: [usize; 6] = [0x60, 0x64, 0x68, 0x6C, 0xE8, 0xEC];
/// Solid rocket (type 47) component: current fin angles f32 rad (pitch / yaw channel, clamped to
/// the def's min/max rotation) and their inversion bytes (`rocket_tick` 0x439DF0, written before
/// `physics_scene_post_step_sync` in the same tick) [high].
pub(crate) const OFF_ROCKET_FIN_ANGLE: usize = 0x800;
pub(crate) const OFF_ROCKET_FIN_INVERT: usize = 0x77C;
/// Type-18 fin / control surface: current angle f32 rad about `dynamic_rotation_axes` (`rudder_draw`
/// 0x3E38A0; written before `physics_scene_post_step_sync` in the same tick) [high].
pub(crate) const OFF_RUDDER_ANGLE: usize = 0x274;
/// The game's own fin and wing air forces, by return address of their
/// `physics_scene_apply_force_clamped` call: rudder tick AIR force (call 0x3E383B, the last of six;
/// the other five are water) and the solid rocket fin force (call 0x43CDA5). Ours replace them
/// while the plugin is on; neither reads the settings air constants [high, disassembly checked].
pub(crate) const GAME_AIR_FORCE_RETURNS: [usize; 2] = [0x003e_3840, 0x0043_cdaa];

pub(crate) const OFF_BODY_PHYS_OBJ: usize = 0x718; // mm_physics_scene_object*
pub(crate) const OFF_BODY_ORIGIN: usize = 0x720; // f32x3: world = (voxel·0.25 − origin)·M
pub(crate) const OFF_BODY_NO_FORCE: usize = 0x1478; // u8: vanilla skips force flush when set
pub(crate) const OFF_BODY_WIND: usize = 0x14D0; // f32x3 wind velocity, world, this tick
pub(crate) const OFF_OBJ_MASS: usize = 0x08; // f32, without tank fluid
/// Body compartment set `{items, mod, offset, count}` (tank fluid); per item f32 capacity and
/// f32[13] amounts by fluid type (`body_fluid_force_compute` 0x48D5B0, c38c03ff756b) [high/medium].
pub(crate) const OFF_BODY_COMPARTMENTS: usize = 0xF80;
pub(crate) const OFF_COMPARTMENT_CAPACITY: usize = 0x38;
pub(crate) const OFF_COMPARTMENT_AMOUNTS: usize = 0x3C;
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
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8d, 0x9a, 0xa0, 0x01, 0x00,
    0x00, 0x48, 0x8b, 0xf2, 0x48, 0x8b,
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
pub(crate) const MATRIX_INTERP_PROLOGUE: [u8; 18] =
    [0x4c, 0x8b, 0xdc, 0x53, 0x56, 0x57, 0x48, 0x81, 0xec, 0xf0, 0x00, 0x00, 0x00, 0x41, 0x0f, 0x29, 0x73, 0xd8];
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
// 0x43CDAA was listed here as rocket thrust; it is the rocket FIN force (`GAME_AIR_FORCE_RETURNS`).
pub(crate) const CLAMPED_THRUST_RETURNS: [(usize, u8); 5] = [(0x0043_a652, 2), (0x0043_b007, 2), (0x0043_cef8, 2), (0x003f_60d0, 8), (0x003f_3df9, 16)];
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
pub(crate) const RAYCAST_CLOSEST_PROLOGUE: [u8; 21] =
    [0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x48, 0x8d, 0x68, 0xa8, 0x48, 0x81, 0xec, 0x40, 0x01, 0x00, 0x00];
// Water surface height: f32 f(c_scene + 0x68A0, const f64 pos[3]).
pub(crate) const RVA_WATER_HEIGHT: usize = 0x002b_ebf0;
pub(crate) const WATER_HEIGHT_PROLOGUE: [u8; 17] = [0x4c, 0x8b, 0xdc, 0x48, 0x83, 0xec, 0x68, 0x0f, 0x57, 0xc0, 0x49, 0x8d, 0x43, 0x18, 0x4c, 0x8b, 0xc2];
pub(crate) const OFF_SCENE_WATER_FIELD: usize = 0x68A0;
pub(crate) const OFF_SETTINGS_AIR_DENSITY: usize = 0xC4; // f32 sea-level air density (game constant "air_density", 1.225)

/// `opaque_chunk_draw_item(bindset, mesh, first, count, mat_world, prev, next, light, c1, c2, c3,
/// is_preview, is_force_color, is_disable_color_replace, _, wire)`: every vehicle chunk draw.
pub(crate) const RVA_CHUNK_DRAW: usize = 0x0068_0c00;
/// Class draws of moving component meshes (vtable +0x470, game thread; `[fin-paint RE]`):
/// `(comp, queue, matA, matB, a5, a6, a7, a8 override flag, a9, a10 colour)`. With a8 != 0 the
/// mesh's three paint colours are a10 instead of `comp+0x60..0x68`; the items reach
/// `RVA_CHUNK_DRAW` as 232-byte items with force 0 [high data flow, medium look].
pub(crate) const RVA_SOLID_ROCKET_DRAW: usize = 0x0043_ecc0;
pub(crate) const SOLID_ROCKET_DRAW_PROLOGUE: [u8; 24] =
    [0x4C, 0x8B, 0xDC, 0x55, 0x53, 0x41, 0x55, 0x49, 0x8D, 0xAB, 0x48, 0xFB, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xA0, 0x05, 0x00, 0x00, 0x48, 0x8B, 0x05];
pub(crate) const RVA_RUDDER_DRAW: usize = 0x003e_38a0;
pub(crate) const RUDDER_DRAW_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xA8, 0xA8, 0xFD, 0xFF, 0xFF, 0x48, 0x81];
pub(crate) const CHUNK_DRAW_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x40, 0x48, 0x8b, 0x41, 0x40, 0x41, 0x8b,
    0xf9, 0x4c, 0x8b, 0x4c, 0x24, 0x70,
];

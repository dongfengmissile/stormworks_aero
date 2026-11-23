//! Facts about `stormworks64.exe` shared between modules. A hook's own addresses and prologues sit
//! beside it (`markhooks.rs`, `watch.rs`, `toolbar.rs`, `hooks.rs`).
//! Source: re-workbench pass on a fresh IDA database (`stormworks_aero_idapro/`).

/// stormworks64.exe sha256 c38c03ff756b4f42bbc9f11cdbb034977ad0f968e45741c5cf410732f22a338d
/// (v1.15.25 hotfix).
pub const GAME_BUILD_LABEL: &str = "22e792753489";


/// `game_state_vehicle_editor_update(editor, a2, a3, a4, a5, a6)`: the editor state's
/// per-tick update (vtable slot 6), on the thread that edits the vehicle. All args are
/// integer-class (call site 0x86ad4d: rcx, rdx, r8, r9 + two stack pointers).
pub const RVA_EDITOR_UPDATE: usize = 0x007d_6740;
/// `editor_render_scene(E, …)` (`22e792753489` [high]): draws the workbench scene; byte 28 starts
/// a call, so the check stops there (knowledge/rendering.md "Workbench body draw matrix").
pub const RVA_EDITOR_RENDER_SCENE: usize = 0x007e_0530;
pub const EDITOR_RENDER_SCENE_PROLOGUE: [u8; 28] =
    [0x48, 0x89, 0x5C, 0x24, 0x18, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0xD0, 0xE6, 0xFF, 0xFF, 0xB8, 0x40, 0x1A, 0x00];
/// No rip-relative or rel32 bytes in this window, so nothing is wildcarded.
pub const EDITOR_UPDATE_PROLOGUE: [u8; 32] = [
    0x48, 0x8b, 0xc4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8d, 0xa8, 0x68, 0xf5, 0xff, 0xff, 0x48, 0x81, 0xec, 0x58,
    0x0b, 0x00, 0x00, 0x0f, 0x29, 0x70,
];

/// Editor state -> the vehicle being edited (0x408-byte vehicle; replaced on load/new).
pub const OFF_EDITOR_VEHICLE: usize = 0x13C8;
/// Ring buffers `{T** items; u32 mod; u32 offset; u32 count}`: element i = items[(i+offset)%mod].
pub const OFF_VEHICLE_BODIES: usize = 0x00;
pub const OFF_BODY_COMPONENTS: usize = 0x3F0;
pub const OFF_COMP_POS: usize = 0x18; // int3 voxel position
pub const OFF_COMP_ROT: usize = 0x30; // int[9] row-major voxel rotation
pub const OFF_COMP_DEF: usize = 0x58; // c_scene_tile_definition* (a mirror variant)
/// u8 mirror mask of the definition variant = the XML `t` (1 x, 2 y, 4 z); comp+0x30 is the
/// raw `r` without it (definition_init_variant 0x317f30, component_def_ref_serialize 0x4bb180).
pub const OFF_DEF_MIRROR: usize = 0x40;
/// `extender_name` of a linear track base (string, as `OFF_DEF_NAME`) (`22e792753489` [high]).
pub const OFF_DEF_EXTENDER: usize = 0x378;
/// Definition name as written to the vehicle XML `d` attribute: `{char* data; u32 len}`.
pub const OFF_DEF_NAME: usize = 0x288;
pub const OFF_DEF_CLASS: usize = 0x2AC; // i32 component class (XML `type`)
                                               // Type-2 force emitter (propeller / rotor) state, knowledge/vehicle-model.md "Type 2 force emitters".
pub const OFF_PROP_SPEED: usize = 0x150; // f32 shaft speed w (game units; blades turn w·rotation_speed rev/s)
/// f32 drawn hub angle rad (`fmodf 2π`), only for drawing; the game turns it by
/// `w_prev·rotation_speed·0.5/60` per tick for air props: 4π slower than the shaft (c38c03ff756b) [high].
pub const OFF_PROP_ANGLE: usize = 0x148;
/// Type-2 def (`comp+0x58`): `force_dir` f32[3]; `*def + OFF_DEF_MESH1` = mesh_1 object, its
/// `+0x30` -> the mesh holder the render pushes get as `a2` [medium] (`[rotor-blur RE]`).
pub const OFF_DEF_FORCE_DIR: usize = 0x850;
/// Hinge constraint (`*(comp+0x2F0)`) angle f32 rad, drawn as hub angle `−angle·sy` for air
/// props (`force_emitter_draw` 0x343AFC; sy from `OFF_DEF_MIRROR` bit 2) [high].
pub const OFF_HINGE_ANGLE: usize = 0x120;
pub const OFF_DEF_MESH1: usize = 0x580;
/// Rotor blur draw path (c38c03ff756b, `[rotor-blur RE]`): `force_emitter_draw` (class draw ABI),
/// `push_render_item_232` (16 args, c1..c3 = [0x40..0x50], a14 cull flag), its two shadow pushes
/// right after it (matA = 5th arg, same pointer), and `push_item240_to_0x180_dither` (13 args,
/// [0x60] f32 opacity) into the view queue ring `+0x180` of 240-byte items (`+0xDC` cull flag).
pub const RVA_FORCE_EMITTER_DRAW: usize = 0x0034_3930;
/// `draw_def_mesh1(def, queue+8, f64 matA[16], f64 matB[16], &c1, &c2, &c3, 0, flag)`: draws one
/// blade for `force_emitter_draw` (call at 0x3443B2), matA = Pre_i·a3 (c38c03ff756b) [high].
pub const RVA_DRAW_DEF_MESH1: usize = 0x0032_3680;
/// `force_emitter_editor_draw` (vtable slot 87): the editor's highlight copy of the part (Select
/// tool), its own blade chain calling `draw_def_mesh1` at 0x344C53 (c38c03ff756b) [high].
pub const RVA_FORCE_EMITTER_EDITOR_DRAW: usize = 0x0034_4450;
pub const FORCE_EMITTER_EDITOR_DRAW_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xA8, 0x78, 0xFA, 0xFF, 0xFF, 0x48, 0x81];
pub const DRAW_DEF_MESH1_PROLOGUE: [u8; 24] =
    [0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0x08, 0xFF, 0xFF, 0xFF, 0x48, 0x81, 0xEC];
pub const FORCE_EMITTER_DRAW_PROLOGUE: [u8; 24] =
    [0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24, 0x08, 0xF9, 0xFF, 0xFF, 0x48, 0x81, 0xEC];
pub const RVA_PUSH_232: usize = 0x0067_2270;
pub const RVA_PUSH_SHADOW_A: usize = 0x0067_3080;
pub const RVA_PUSH_SHADOW_B: usize = 0x0067_3920;
/// The three pushes share these bytes but for byte 28 (the frame offset): f9 / f1 / e9.
pub const PUSH_PROLOGUE: [u8; 36] = [
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D,
    0x6C, 0x24, 0xF9, 0x48, 0x81, 0xEC, 0xF0, 0x00, 0x00, 0x00,
];
pub const RVA_PUSH_DITHER: usize = 0x0067_1dc0;
pub const PUSH_DITHER_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x48, 0x89, 0x7C, 0x24, 0x20, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x6C, 0x24, 0xF9, 0x48,
    0x81, 0xEC, 0xF0, 0x00, 0x00, 0x00,
];
pub const OFF_QUEUE_DITHER_RING: usize = 0x180;
/// Type-2 def `flags` u32 (bit 2: marine prop) [high].
pub const OFF_DEF_PROP_FLAGS: usize = 0x2B8;
pub const OFF_PROP_CYCLIC: usize = 0x258; // f32 ×2 cyclic x / y input
pub const OFF_PROP_COLLECTIVE: usize = 0x260; // f32 live pitch input
pub const OFF_MC_WIDTH: usize = 0x188; // i32 microcontroller width, then i32 length
pub const OFF_WHEEL_SIZE: usize = 0x6F4; // f32 advanced wheel wheel_size setting
pub const OFF_WHEEL_DOUBLE: usize = 0x6FC; // u8 advanced wheel double_wheel setting
pub const OFF_JOINT_POWER: usize = 0x420; // f32 type-7 driven joint max power setting (`max_force_scalar`)
pub const OFF_PROP_BLADES: usize = 0x264; // u32 blade_count
pub const OFF_PROP_PITCH: usize = 0x268; // i32 blade_pitch setting
pub const OFF_PROP_FOLDED: usize = 0x270; // u8
pub const OFF_PROP_LENGTH: usize = 0x294; // f32 blade_length
pub const OFF_PROP_HINGE: usize = 0x2F0; // ptr: hub spun by a hinge motor
pub const OFF_PROP_ACTIVE: usize = 0x304; // u32 active_rotors mask
pub const OFF_PROP_LINK: usize = 0x1D8; // torque slot link (0 = no shaft)
pub const OFF_PROP_SHAFT: usize = 0x1E8; // f32 shaft speed from the network
pub const OFF_DEF_ROTATION_SPEED: usize = 0x864; // f32 force_emitter_rotation_speed [high]
pub const OFF_PROP_REVERSE: usize = 0x758; // u8 thrust reversed
pub const OFF_PROP_FORCE_ACC: usize = 0x760; // the thrust call's 4th argument is comp + this
pub const RVA_PROP_TICK: usize = 0x0034_1530; // force_emitter_tick_propeller_rotor (vtable slot 108)
pub const PROP_TICK_PROLOGUE: [u8; 24] =
    [0x4C, 0x8B, 0xDC, 0x55, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x57, 0x49, 0x8D, 0xAB, 0x28, 0xFF, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xB0, 0x01, 0x00];
pub const PROP_THRUST_RETURN: usize = 0x0034_1eb0; // apply_force_threshold, thrust; arg 4 = comp + 0x760
pub const PROP_COUPLE_RETURNS: [usize; 2] = [0x0034_2341, 0x0034_23a4]; // apply_force_clamped, coaxial couple

/// Editor camera (`mm_camera_perspective`) inside the editor state; view and projection are
/// f32 4x4 row-major, row vectors (`clip = p·V·P`, OpenGL NDC, clip.w = -z_eye).
/// Written each tick by editor_camera_update 0x7f39e0 (re-workbench round 2).
pub const OFF_EDITOR_CAMERA: usize = 0x78;
pub const OFF_CAM_VIEW: usize = 0x88;
pub const OFF_CAM_PROJ: usize = 0xC8;
/// Vehicle world transform, f64 4x4 row-major (copied from editor+0xCD8 on reset).
pub const OFF_VEHICLE_WORLD: usize = 0x190;
/// Body transform in its vehicle, f32 4x4 row-major: world = (voxel·0.25)·Body·Vehicle.
pub const OFF_BODY_MATRIX: usize = 0x2C8;

// ---- In-game flight forces (re-workbench round 4: vanilla air path + Bullet).

/// `physics_scene_post_step_sync(physics_scene)`: update thread, right after the physics wait
/// and before `scene_vehicle_update`. Physics is idle here, and forces put on a btRigidBody now
/// last through the next step (clearForces runs once at the end of stepSimulation).
/// Not hooked by the infrared plugin (it hooks scene_vehicle_update 0x5ebd20).
pub const RVA_POST_STEP_SYNC: usize = 0x001a_3a10;
/// Up to the first `call rel32` (at +0x16).
pub const POST_STEP_SYNC_PROLOGUE: [u8; 22] =
    [0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8d, 0x51, 0x48, 0x48, 0x89, 0x6c, 0x24, 0x38, 0x48, 0x8b, 0xf9];
/// Joints in game (knowledge/vehicle-model.md, `22e792753489`): a joint's parent half
/// (`def+0x2B8 & 0x40`) holds its child half at `+0x148` and an info record at `+0x158`, whose
/// `+0x120` f32 is the slider position (m) or hinge angle (rad). A part's body is at `+0x28`.
pub const OFF_COMP_BODY: usize = 0x28;
pub const OFF_JOINT_CHILD: usize = 0x148;
pub const OFF_JOINT_INFO: usize = 0x158;
pub const OFF_INFO_POSITION: usize = 0x120;
/// Bullet contacts (knowledge/vehicle-model.md "Contacts between a vehicle's bodies", rub RE):
/// physics scene `+0x08` btCollisionDispatcher, its manifold array (`+0x14` i32 count, `+0x20`
/// pointers); a manifold's points at `+0x10` (368 B each), body0/body1 `+0x5D0`/`+0x5D8`, count
/// `+0x5E0`; a point's local points on A/B (own body's frame, f64x3) `+0x00`/`+0x20`, distance
/// `+0xA0` (f64, < 0 deep), applied impulse `+0xE8` (f64). A body's rigid body is reached from its
/// physics object (`body+0x718`) at `+0xB8`, and points back to it at `+0x1F8`.
pub const OFF_PHYS_DISPATCHER: usize = 0x08;
pub const OFF_DISP_MANIFOLD_COUNT: usize = 0x14;
pub const OFF_DISP_MANIFOLDS: usize = 0x20;
pub const OFF_MANIFOLD_POINTS: usize = 0x10;
pub const MANIFOLD_POINT_SIZE: usize = 368;
pub const OFF_MANIFOLD_BODY0: usize = 0x5D0;
pub const OFF_MANIFOLD_BODY1: usize = 0x5D8;
pub const OFF_MANIFOLD_COUNT: usize = 0x5E0;
pub const OFF_POINT_LOCAL_A: usize = 0x00;
pub const OFF_POINT_LOCAL_B: usize = 0x20;
pub const OFF_POINT_DISTANCE: usize = 0xA0;
pub const OFF_POINT_IMPULSE: usize = 0xE8;
pub const OFF_RB_USER: usize = 0x1F8;
/// The physics scene inside c_scene.
pub const OFF_SCENE_PHYSICS: usize = 0x64D0;
/// c_scene vehicle container ring (container 0x4990 + 0x320): records
/// `{u8 removed; u32 vehicle_id @+4; vehicle* @+8; u32 type @+0x10 (5 = vehicle)}`.
pub const OFF_SCENE_VEHICLES: usize = 0x4CB0;
/// Ring of every spawned vehicle, loaded or not (host / single player; knowledge/vehicle-model.md).
/// Record: +0 removed, +4 id, +0x70 f64x3 position, +0xD5 keep active, +0xE0 loaded record or 0.
pub const OFF_SCENE_SPAWNED: usize = 0x30;

/// `physics_scene_set_damping(scene, obj, f32 linear, f32 angular)`: Bullet velocity damping,
/// re-set on every vehicle body every tick (vanilla 0.1 = ~10 %/s speed loss).
pub const RVA_SET_DAMPING: usize = 0x001a_73e0;
pub const SET_DAMPING_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xfa, 0xf3, 0x0f, 0x10, 0x82, 0xb0, 0x00, 0x00, 0x00,
    0x0f, 0x2e, 0xc2, 0x7a, 0x11, 0x75,
];

/// `fluid_air_node_forces(ctx, node, matrix, water, _)`, void: one octree node's hull-face air
/// forces (worker threads); `*ctx` = `body + OFF_BODY_AIR_RECORDS` (c38c03ff756b) [high].
pub const RVA_AIR_NODE_FORCES: usize = 0x0049_08b0;
pub const AIR_NODE_FORCES_PROLOGUE: [u8; 24] =
    [0x48, 0x8b, 0xc4, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xec, 0x00, 0x01, 0x00, 0x00, 0x80, 0xb9, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x8b];
pub const OFF_BODY_AIR_RECORDS: usize = 0x11f0;

/// `btRigidBody::applyForce(rb, const double force[4], const double rel_pos[4])`, double precision;
/// rel_pos from the centre of mass, world axes.
pub const RVA_APPLY_FORCE: usize = 0x0019_a220;
pub const APPLY_FORCE_PROLOGUE: [u8; 32] = [
    0x48, 0x83, 0xec, 0x48, 0xf2, 0x0f, 0x10, 0x4a, 0x08, 0xf2, 0x0f, 0x10, 0x5a, 0x10, 0xf2, 0x0f, 0x10, 0xa1, 0xf0, 0x03, 0x00, 0x00, 0xf2, 0x0f, 0x10, 0x02,
    0xf2, 0x0f, 0x59, 0x81, 0xe0, 0x03,
];

/// Game settings singleton (`get_game_settings` 0x296940): the vanilla air-force constants,
/// f32, re-read every tick by `air_face_force` 0x490b50: linear/quadratic pressure drag,
/// linear/quadratic suction drag, viscous air resistance, lift. Water uses other fields.
pub const RVA_GAME_SETTINGS: usize = 0x00d0_6810;
pub const AIR_CONSTANTS: [usize; 6] = [0x60, 0x64, 0x68, 0x6C, 0xE8, 0xEC];
/// Solid rocket (type 47) component: current fin angles f32 rad (pitch / yaw channel, clamped to
/// the def's min/max rotation) and their inversion bytes (`rocket_tick` 0x439DF0, written before
/// `physics_scene_post_step_sync` in the same tick) [high].
pub const OFF_ROCKET_FIN_ANGLE: usize = 0x800;
pub const OFF_ROCKET_FIN_INVERT: usize = 0x77C;
/// Type-18 fin / control surface: current angle f32 rad about `dynamic_rotation_axes` (`rudder_draw`
/// 0x3E38A0; written before `physics_scene_post_step_sync` in the same tick) [high].
pub const OFF_RUDDER_ANGLE: usize = 0x274;
/// The game's own fin and wing air forces, by return address of their
/// `physics_scene_apply_force_clamped` call: rudder tick AIR force (call 0x3E383B, the last of six;
/// the other five are water) and the solid rocket fin force (call 0x43CDA5). Ours replace them
/// while the plugin is on; neither reads the settings air constants [high, disassembly checked].
pub const GAME_AIR_FORCE_RETURNS: [usize; 2] = [0x003e_3840, 0x0043_cdaa];

pub const OFF_BODY_PHYS_OBJ: usize = 0x718; // mm_physics_scene_object*
pub const OFF_BODY_ORIGIN: usize = 0x720; // f32x3: world = (voxel·0.25 − origin)·M
pub const OFF_BODY_NO_FORCE: usize = 0x1478; // u8: vanilla skips force flush when set
pub const OFF_BODY_WIND: usize = 0x14D0; // f32x3 wind velocity, world, this tick
pub const OFF_OBJ_MASS: usize = 0x08; // f32, without tank fluid
/// Body compartment set `{items, mod, offset, count}` (tank fluid); per item f32 capacity and
/// f32[13] amounts by fluid type (`body_fluid_force_compute` 0x48D5B0, c38c03ff756b) [high/medium].
pub const OFF_BODY_COMPARTMENTS: usize = 0xF80;
pub const OFF_COMPARTMENT_CAPACITY: usize = 0x38;
pub const OFF_COMPARTMENT_AMOUNTS: usize = 0x3C;
pub const OFF_OBJ_WORLD: usize = 0x18; // f64[16] row-major, row vectors
pub const OFF_OBJ_RIGID_BODY: usize = 0xB8; // btRigidBody*
pub const OFF_RB_COM: usize = 0x70; // f64x3 centre of mass (worldTransform origin)
pub const OFF_RB_FLAGS: usize = 0x1A0; // u32, 0x800 = externally driven
pub const OFF_RB_ACTIVATION: usize = 0x1B0; // i32, 2 = sleeping
pub const OFF_RB_LIN_VEL: usize = 0x390; // f64x3
pub const OFF_RB_ANG_VEL: usize = 0x3B0; // f64x3
pub const OFF_RB_TOTAL_TORQUE: usize = 0x480; // f64x3
pub const OFF_RB_ANGULAR_FACTOR: usize = 0x560; // f64x3

// Main in-game camera (render thread). c_game_state_simulation vtable slot 7 gets G in rcx;
// G+0x1010 is the main view's mm_camera_perspective, +0x1C8 its f64 viewproj (world space, row vectors p·M).
pub const RVA_SIM_RENDER_SLOT7: usize = 0x007b_1fc0;
pub const SIM_RENDER_SLOT7_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x30, 0x48, 0x8d, 0x9a, 0xa0, 0x01, 0x00,
    0x00, 0x48, 0x8b, 0xf2, 0x48, 0x8b,
];
pub const RVA_SIM_STATE_VTABLE: usize = 0x00b1_b778;
pub const OFF_SIM_MAIN_CAMERA: usize = 0x1010;
pub const OFF_CAM_VIEWPROJ_F64: usize = 0x1C8;
pub const OFF_BODY_VEHICLE: usize = 0x250; // vehicle*
pub const OFF_BODY_RENDER: usize = 0x2C8; // f32[16] body -> vehicle (render); world = grid_m·this·vehicle+0x190
                                                 // Per-frame draw interpolation (render thread): mat_world = matrix_interp_prev_cur(prev, cur, *(frame+0x89C)),
                                                 // frame = *(G+0x5C0). Each frame build snapshots cur = vehicle+0x190 · body+0x2C8, then copies it to
                                                 // vehicle+0x210 / body+0x308, so those hold the latest built pose until the next build.
pub const RVA_MATRIX_INTERP: usize = 0x001e_7ec0; // (f32* prev, f32* cur, f32 alpha (xmm2), f32* out)
pub const MATRIX_INTERP_PROLOGUE: [u8; 18] =
    [0x4c, 0x8b, 0xdc, 0x53, 0x56, 0x57, 0x48, 0x81, 0xec, 0xf0, 0x00, 0x00, 0x00, 0x41, 0x0f, 0x29, 0x73, 0xd8];
pub const OFF_SIM_FRAME: usize = 0x5C0;
pub const OFF_FRAME_ALPHA: usize = 0x89C; // f32, written each render frame by slot 7
pub const OFF_FRAME_BUILD_NS: usize = 0xED0; // u64, time of the last frame build
pub const OFF_VEHICLE_WORLD_BUILT: usize = 0x210; // f64[16]
pub const OFF_BODY_RENDER_BUILT: usize = 0x308; // f32[16]

// Engine thrust (RE round 6): components queue world-frame forces through these physics-scene wrappers
// (scene, phys_obj, f32[3]* force, ptr, ptr, f32 max, f32 scale); return addresses identify the source.
/// `jet_engine_tick` (vtable slot 107 of every jet part), 25 arguments, rcx = component.
pub const RVA_JET_TICK: usize = 0x0035_34c0;
/// The jet engine class's vtable entry for its tick (slot 107): the exe's only pointer to the tick.
pub const RVA_JET_TICK_SLOT: usize = 0x00B9_42C0;
pub const RVA_APPLY_FORCE_THRESHOLD: usize = 0x001a_7b20;
pub const APPLY_FORCE_THRESHOLD_PROLOGUE: [u8; 20] =
    [0x48, 0x89, 0x5c, 0x24, 0x10, 0x48, 0x89, 0x6c, 0x24, 0x18, 0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83, 0xec, 0x60];
pub const RVA_APPLY_FORCE_CLAMPED: usize = 0x001a_7980;
pub const APPLY_FORCE_CLAMPED_PROLOGUE: [u8; 20] =
    [0x48, 0x89, 0x5c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x60, 0x0f, 0x29, 0x74, 0x24, 0x50];
/// Thrust call sites by return address (each verified to follow a call to the named wrapper), with a source bit:
/// 1 jet engine tick (0x3534C0), 2 solid rocket tick (0x439DF0), 4 propeller/rotor main thrust (force_emitter
/// 0x341530; its two blade lift/drag forces at 0x342341/0x3423A4 are left out), 8 RCS and 16 water jet / fluid
/// reaction (water component 0x3F1FA0; fluid sub-types not mapped to part names, low confidence).
pub const THRESHOLD_THRUST_RETURNS: [(usize, u8); 4] = [(0x0035_451f, 1), (0x0035_6442, 1), (0x0034_1eb0, 4), (0x003f_6cbc, 16)];
// 0x43CDAA was listed here as rocket thrust; it is the rocket FIN force (`GAME_AIR_FORCE_RETURNS`).
pub const CLAMPED_THRUST_RETURNS: [(usize, u8); 5] = [(0x0043_a652, 2), (0x0043_b007, 2), (0x0043_cef8, 2), (0x003f_60d0, 8), (0x003f_3df9, 16)];
pub const THRUST_SOURCE_NAMES: [&str; 5] = ["jet", "rocket", "propeller/rotor", "RCS", "water jet"];

// Modular jet (RE round 8): the nozzle's thrust call passes `this + 0x510` in r9 (arg 4). Component def type at
// *(this+0x58)+0xED4 (u32: 0 fan, 1 compressor, 2 combustor, 3 turbine, 4 nozzle); shaft RPS f32 at +0x1BC on
// types 1-3; upstream neighbours ring at +0x408 ({items, mod, offset, count}).
pub const JET_NOZZLE_RETURN: usize = 0x0035_451f;
pub const JET_FAN_RETURN: usize = 0x356442; // large intake fan force, builds c38c03ff756b and 22e792753489
pub const JET_THIS_FROM_ARG4: usize = 0x510;
pub const OFF_JET_DEF_TYPE: usize = 0xED4;
pub const OFF_JET_RPS: usize = 0x1BC;
pub const OFF_JET_UPSTREAM: usize = 0x408;

// Ground effect (RE round 9): closest-hit ray on the physics scene, f64 world coordinates, update thread:
// u8 f(physics_scene, f64 ray[6] {from, to}, f64 hit[3] out, mm_physics_scene_object** out, i32 group, i32 mask).
pub const RVA_RAYCAST_CLOSEST: usize = 0x001a_4010;
pub const RAYCAST_CLOSEST_PROLOGUE: [u8; 21] =
    [0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x10, 0x55, 0x56, 0x57, 0x48, 0x8d, 0x68, 0xa8, 0x48, 0x81, 0xec, 0x40, 0x01, 0x00, 0x00];
// Water surface height: f32 f(c_scene + 0x68A0, const f64 pos[3]).
pub const RVA_WATER_HEIGHT: usize = 0x002b_ebf0;
pub const WATER_HEIGHT_PROLOGUE: [u8; 17] = [0x4c, 0x8b, 0xdc, 0x48, 0x83, 0xec, 0x68, 0x0f, 0x57, 0xc0, 0x49, 0x8d, 0x43, 0x18, 0x4c, 0x8b, 0xc2];
pub const OFF_SCENE_WATER_FIELD: usize = 0x68A0;
// Weather at a world point, as the humidity and rain sensors read it (knowledge/heat-model.md):
// void f(c_scene + 0xBADE8, f32 out[8], const f64 pos[3], u8 storm_cells). out[1] rain, out[2] fog, 0..1.
pub const RVA_WEATHER_SAMPLE: usize = 0x002c_3d40;
pub const WEATHER_SAMPLE_PROLOGUE: [u8; 32] = [
    0x48, 0x8b, 0xc4, 0x53, 0x56, 0x57, 0x41, 0x57, 0x48, 0x81, 0xec, 0xd8, 0x00, 0x00, 0x00, 0xf2, 0x41, 0x0f, 0x10, 0x00, 0x4d, 0x8b, 0xf8, 0xf2, 0x41, 0x0f,
    0x10, 0x48, 0x10, 0x48, 0x8b, 0xda,
];
pub const OFF_SCENE_WEATHER: usize = 0xBADE8;

// Effects pass (knowledge/rendering.md, "Main-view translucent insertion point"): the call
// `gl_set_depth_test_mask(glctx, 1, 0)` after the game's translucent passes, scene colour bound.
pub const RVA_GL_SET_DEPTH_TEST_MASK: usize = 0x001b_de00;
/// Up to the first rip-relative call.
pub const GL_SET_DEPTH_TEST_MASK_PROLOGUE: [u8; 19] = [0x40, 0x53, 0x48, 0x83, 0xec, 0x20, 0x41, 0x0f, 0xb6, 0xd8, 0xb9, 0x71, 0x0b, 0x00, 0x00, 0x84, 0xd2, 0x74, 0x08];
/// `xor r8d, r8d; mov dl, 1; mov rcx, r14; call` (rel32 follows) in `render_scene_with_cameras`.
pub const RVA_FX_CALL_SITE: usize = 0x006d_d362;
pub const FX_CALL_SITE_BYTES: [u8; 9] = [0x45, 0x33, 0xc0, 0xb2, 0x01, 0x49, 0x8b, 0xce, 0xe8];
/// Renderer: the main view's render target set (`[0]` G-buffer, `[1]` scene colour).
pub const OFF_RENDERER_MAIN_TARGETS: usize = 0x2628;
/// Render target: width, height (u32).
pub const OFF_TARGET_WIDTH: usize = 0x2C;
pub const OFF_TARGET_HEIGHT: usize = 0x30;
/// G-buffer target: depth texture id (u32), log depth as in depth_utils.glslh.
pub const OFF_GBUFFER_DEPTH_TEX: usize = 0x60;
/// View state: camera feed (u64, 0 = main view), editor preview (u8), fog density (f32) [medium].
pub const OFF_VIEW_FEED: usize = 0x828;
pub const OFF_VIEW_EDITOR: usize = 0x83B;
pub const OFF_VIEW_FOG_DENSITY: usize = 0x770;
/// View state lights as `light_pass_uniforms` uploads them (`22e792753489` [high], light-RE): sun
/// colour vec3, intensity (× 1 − (1 − f32[+0x8E4])·f32[+0x8E8]), moon colour vec3 and intensity,
/// ambient low/high (u8x4, /255).
pub const OFF_VIEW_SUN_COLOR: usize = 0x730;
pub const OFF_VIEW_SUN_INTENSITY: usize = 0x73C;
pub const OFF_VIEW_SUN_DIM_A: usize = 0x8E4;
pub const OFF_VIEW_SUN_DIM_B: usize = 0x8E8;
pub const OFF_VIEW_MOON_COLOR: usize = 0x74C;
pub const OFF_VIEW_MOON_INTENSITY: usize = 0x758;
pub const OFF_VIEW_AMBIENT_LOW: usize = 0x7B4;
pub const OFF_VIEW_AMBIENT_HIGH: usize = 0x7B8;

pub const OFF_SETTINGS_AIR_DENSITY: usize = 0xC4; // f32 sea-level air density (game constant "air_density", 1.225)

/// `opaque_chunk_draw_item(bindset, mesh, first, count, mat_world, prev, next, light, c1, c2, c3,
/// is_preview, is_force_color, is_disable_color_replace, _, wire)`: every vehicle chunk draw.
pub const RVA_CHUNK_DRAW: usize = 0x0068_0c00;
/// Class draws of moving component meshes (vtable +0x470, game thread; `[fin-paint RE]`):
/// `(comp, queue, matA, matB, a5, a6, a7, a8 override flag, a9, a10 colour)`. With a8 != 0 the
/// mesh's three paint colours are a10 instead of `comp+0x60..0x68`; the items reach
/// `RVA_CHUNK_DRAW` as 232-byte items with force 0 [high data flow, medium look].
pub const RVA_SOLID_ROCKET_DRAW: usize = 0x0043_ecc0;
pub const SOLID_ROCKET_DRAW_PROLOGUE: [u8; 24] =
    [0x4C, 0x8B, 0xDC, 0x55, 0x53, 0x41, 0x55, 0x49, 0x8D, 0xAB, 0x48, 0xFB, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xA0, 0x05, 0x00, 0x00, 0x48, 0x8B, 0x05];
pub const RVA_RUDDER_DRAW: usize = 0x003e_38a0;
pub const RUDDER_DRAW_PROLOGUE: [u8; 24] =
    [0x48, 0x8B, 0xC4, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xA8, 0xA8, 0xFD, 0xFF, 0xFF, 0x48, 0x81];
pub const CHUNK_DRAW_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57, 0x48, 0x83, 0xec, 0x40, 0x48, 0x8b, 0x41, 0x40, 0x41, 0x8b,
    0xf9, 0x4c, 0x8b, 0x4c, 0x24, 0x70,
];

// Net role (knowledge/multiplayer.md, c38c03ff756b): u8 globals. Net client = multiplayer != 0 && authoritative == 0.
pub const RVA_NET_IS_MULTIPLAYER: usize = 0x00ce_f64e;
pub const RVA_NET_IS_AUTHORITATIVE: usize = 0x00ce_f64f;
pub const OFF_BODY_NET_STAMP: usize = 0x218; // u32 tick of the last replicated pose (clients)

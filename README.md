# stormworks-aero

A plugin for [Stormworks: Build and Rescue](https://store.steampowered.com/app/573090/Stormworks_Build_and_Rescue/) that brings a voxel aerodynamics model in the style of [Ferram Aerospace Research Continued](https://github.com/KSPModStewards/Ferram-Aerospace-Research) (FAR) for KSP to the workbench.

**Status: first version.** It analyses the vehicle in the workbench, and can replace the game's air forces in flight with the same model (experimental, on by default; untick it in the F7 window for the game's own aerodynamics).

## What it does

Press **F7** in the workbench to open the analysis window, laid out like FAR's editor GUI:

- **Static Analysis**: angle-of-attack sweeps (with the stall return pass) or Mach sweeps of Cl, Cd, Cm and L/D, with flap and pitch settings.
- **Data + Stability Derivatives**: trims for level flight at a given altitude and Mach, then shows the longitudinal and lateral stability derivatives (Zw, Mw, Mq, Lβ, Nβ, Lp, Nr…). Green means the expected sign, red means not.
- **Stability Deriv Simulation**: the linearised free response (short period, phugoid, dutch roll, spiral).
- **Transonic Design**: area ruling. Cross-section area curve, its curvature, the Mach 1 pressure coefficient, max cross-section, Mach 1 wave drag area and critical Mach.
- **Voxels & Wings**: the vehicle's voxels in a 3D view, with the centre of mass and centre of lift.

Press **F8** in the workbench (or tick the box at the top of the window) to swap the game's vehicle for the plugin's voxel model, drawn in the workbench view itself, so you can fly the editor camera around what the analysis sees. F8 again swaps back. It hides the game's vehicle with the editor's show/hide key, so this assumes that key is still bound to H.

## In-game flight (experimental)

**Plugin flight model in game** (top of the F7 window) is on from the moment the plugin loads. While it is on:

- the game's own air forces (block drag and lift) and its velocity damping are switched off for every vehicle; water forces are unchanged;
- every movable vehicle gets the plugin's forces each physics tick: its wings and control surfaces (with their aerofoils), and fuselage lift and drag;
- hinged sub-grid control surfaces work by moving: the deflected sub-grid is simply a tilted wing;
- a spawned vehicle uses the `.aero` designations whose marked blocks it matches best (at least half of them; F9 shows the file, and how many marks sit on blocks that were removed or moved since).

**F9** shows flight data (airspeed, Mach, AoA, sideslip, Cl, Cd, L/D, lift vs weight, mass from the game's physics, thrust by source and T/W, air density, static margin, ground effect, stalled panels; forces in kN and kgf) and draws force arrows on the vehicle, one lift and one drag arrow per wing (with the surfaces hinged on it) and for the fuselage: green = wing lift, cyan = fuselage lift, red = drag, with yellow/orange totals at the centre of mass, plus live markers: CG (yellow, the game's own centre of mass), CoL (blue, where the lift acts right now, so it moves with flaps, AoA and stall), NP (hollow ring, the neutral point that sets pitch stability), and CoT (magenta, the centre of thrust with the thrust vector). Stalled surfaces have their blocks painted red in the game's own render (lit and hidden like the game's blocks; needs the plugin flight model on; `aero_disable_paint` in the game folder turns it off). Arrow length grows logarithmically with the load relative to the vehicle's weight (a surface carrying 1× weight is about 6 m, 10× about 15 m), so high-g supersonic loads stay readable.

Ground effect: each wing panel's height above the ground, buildings or water below it is measured with the game's own raycast every tick; within about a wingspan its induced drag drops and its lift slope rises (McCormick's factor, applied as a larger effective aspect ratio). Upright fins feel none. F9 shows the wing's height and the induced-drag factor near the ground.

All air loads are applied to the vehicle's main body at their true points, so hinges and pivots don't carry them and sub-grid surfaces don't flex.

Air: density follows Stormworks' own curve, `air_density · (1 − h·2.2413793e-5)^5.3595023` (zero above 40 km), with the sea-level value read from the game's settings (1.225 kg/m³ by default); the game has no temperature model, so the speed of sound (Mach) and viscosity use Earth's standard atmosphere temperature. Jet thrust follows the same density curve instead of the game's linear jet altitude factor.

Jet engines: nozzle thrust is smoothed over about half a second (a throttle blip to 0 for a tick or two barely dents it), and never exceeds what that engine makes while holding the current shaft RPS. That limit is learned from the engine's own steady running (kept across respawns), and above the highest RPS it has learned the engine is not held back, so a throttle step no longer gives a thrust spike once the engine has run at those RPS.

Untick it to get the game's aerodynamics back. Vanilla control fins do not deflect in the plugin's model yet (use sub-grid surfaces). `aero_disable_flight` in the game folder keeps the flight hooks out entirely.

## Wings and control surfaces

Stormworks wings are usually built from blocks, so the plugin has to be told which blocks are wings:

- In **Voxels & Wings**, pick **Wing blocks** and click blocks to mark them. Shift+click marks a whole thin slab at once, Ctrl+click unmarks it.
- Vanilla wing parts are wings automatically. Vanilla control surfaces are always lifting panels.
- Pick **Control role** to give a vanilla control surface, or a hinged sub-grid, a role: pitch, roll, yaw or flap.
- Pick **Aerofoil** to choose sections: click a wing to give the whole wing (and the surfaces hinged on it) the selected section, Shift+click for one panel only; "Other panels" sets the rest. In this tool panels are coloured by aerofoil, with a colour key below the view. Thick cambered sections (NACA 4415, 2412) give lots of lift at low speed but are draggy and hit the transonic drag rise early; symmetric ones (0012) suit tails; the slotted symmetric section keeps lifting to about twice the angle (all-moving tails, flaps) for extra drag; thin ones (64A010, biconvex) are for jets and supersonic flight; supercritical delays the drag rise for airliners. "FAR default" is FAR's own thin symmetric section.
- Static mod wing pieces (type-18 parts without a deflection input, e.g. Thales' wing_middle) are wings automatically.
- **Ctrl+Z** undoes the last wing or role change. While the analysis window has focus the workbench gets no keyboard input, so Ctrl+Z never undoes a build step; click outside the window to give the keys back to the game.
- Type the vehicle's name at the top so the marks are saved next to the vehicle, in `data/vehicles/<name>.aero`. When you load a vehicle into the workbench, its `.aero` is found and loaded automatically (by matching the marked blocks), and the name is filled in. Marks on blocks you delete are removed automatically (Ctrl+Z in the window restores them).

Everything that is not a wing still makes lift and drag through FAR's slender-body model (fuselage lift, area ruling, skin friction).

## Build

Requires the Rust toolchain and the GNU Windows cross target:

```
rustup target add x86_64-pc-windows-gnu
cargo build --target x86_64-pc-windows-gnu --features logging   # debug, verbose log
cargo build --release --target x86_64-pc-windows-gnu             # release
cargo test                                                       # aero model tests (host)
```

## Usage

1. Start Stormworks.
2. Inject `StormworksAero.dll` into `stormworks64.exe` with `aero_inject.exe` (next to the DLL) or any injector. On Linux run it inside the game's Proton prefix: `protontricks-launch --appid 573090 aero_inject.exe`.
3. Open the workbench and press F7.

The log is `<Stormworks>/logs/aero.log`. Kill switches (marker file next to the DLL, or environment variable):
`aero_disable_hook` / `AERO_NO_HOOK=1` (no workbench reading), `aero_disable_overlay` / `AERO_NO_OVERLAY=1` (no window).

## Credits and license

The aerodynamics are a port of FAR Continued (Ferram Aerospace Research, © ferram4 and the KSPModStewards contributors), which is GPLv3. This plugin is therefore GPLv3 too (see `LICENSE`). Where the port deviates from FAR, a comment at that spot in the code says so.

I am not affiliated with the developers or publishers of Stormworks: Build and Rescue, nor have they endorsed this project.

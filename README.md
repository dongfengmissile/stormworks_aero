# stormworks-aero

A plugin for [Stormworks: Build and Rescue](https://store.steampowered.com/app/573090/Stormworks_Build_and_Rescue/) that replaces the game's aerodynamics with a voxel model in the style of [Ferram Aerospace Research Continued](https://github.com/KSPModStewards/Ferram-Aerospace-Research) (FAR) for KSP, with FAR-style analysis tools in the workbench.

## READ THIS FIRST

stormworks-aero uses low-level techniques (DLL injection + function hooking) to extend the game, because there is no supported way to do it.

- No guarantees about stability. Every function the plugin hooks is verified against a known byte pattern first, and if it doesn't match (usually after a game update) that hook is skipped and the plugin says so in its log instead of patching blindly — but injection itself can still be a bad time.
- **Antivirus software might flag it.** Injecting into another process is exactly what malware does, so this is expected. If you want to be sure what it does, read the source and build it yourself.
- It does not modify any game files on disk. The only files it writes are the `.aero` designation files next to your vehicles.

## Build

Requires the Rust toolchain and the GNU Windows cross target:

```
rustup target add x86_64-pc-windows-gnu
cargo build --target x86_64-pc-windows-gnu --features logging
```

or for the minimal logging release:

```
rustup target add x86_64-pc-windows-gnu
cargo build --release --target x86_64-pc-windows-gnu
```

## Usage

1. Start Stormworks.
2. Inject `StormworksAero.dll` into `stormworks64.exe` — either with the bundled `aero_inject.exe` or any other DLL injector.

On Linux, the game runs under Proton, so the injector has to run inside the same Proton prefix Steam created for Stormworks (573090) rather than a bare WINE prefix.

Logs go to:

```
<Stormworks install>/logs/aero.log
```

## Wings

Wings in Stormworks are usually built from blocks, so the plugin has to be told which blocks are wings. Click the **Aero wings** button at the end of the workbench toolbar, then use the shelves on the left:

| tool | what a click on the vehicle does |
|---|---|
| Wing blocks | mark or unmark a block, or a whole thin slab, as wing |
| Control role | give a control surface or hinged sub-grid a role: pitch, roll, yaw, flap |
| Aerofoil | give a wing an aerofoil (hover the aircraft icons for their names) |

Wing blocks are painted on the vehicle by role, or by aerofoil in the aerofoil tool. Vanilla wings and control surfaces are wings automatically. Designations are saved to `data/vehicles/<name>.aero` and loaded automatically when the vehicle is. **Beware** that if you edited your vehicle while the plugin was not injected, it might not get assigned its aero file if the voxels differ too much, so you'll have to mark wing blocks again.

## Keybindings

| key | |
|---|---|
| F7 | analysis window: AoA / Mach sweeps, stability derivatives, response simulation, area ruling, voxel view |
| F8 | paint wing blocks in the workbench (legacy) |
| F9 | in flight: flight data (airspeed, Mach, AoA, Cl, Cd, L/D, thrust, air density, static margin…), lift/drag arrows, CG / CoL / CoT markers, stalled blocks painted red |

## In flight

The plugin flight model is on by default (toggle at the top of the F7 window). It switches off the game's aerodynamic forces for every vehicle and applies its own: wings with their aerofoils, control surfaces (hinged sub-grids deflect by moving), fuselage lift and drag, ground effect, and Stormworks' own air density curve. Water forces are unchanged.

## Credits

The aerodynamics are a port of FAR Continued (Ferram Aerospace Research, © ferram4 and the KSPModStewards contributors).

## Disclaimer

I am not personally affiliated with the developers or publishers of Stormworks: Build and Rescue, nor have they endorsed this project. Stormworks: Build and Rescue and any of its content or materials are the property of their respective owners.

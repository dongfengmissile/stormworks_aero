# stormworks-aero

A plugin for [Stormworks: Build and Rescue](https://store.steampowered.com/app/573090/Stormworks_Build_and_Rescue/) that replaces the game's aerodynamics with a voxel model in the style of [Ferram Aerospace Research Continued](https://github.com/KSPModStewards/Ferram-Aerospace-Research) (FAR) for KSP, with FAR-style analysis tools in the workbench.

## READ THIS FIRST

stormworks-aero uses low-level techniques (DLL injection + function hooking) to extend the game, because there is no supported way to do it.

- No guarantees about stability. Every function the plugin hooks is verified against a known byte pattern first, and if it doesn't match (usually after a game update) that hook is skipped and the plugin says so in its log instead of patching blindly — but injection itself can still be a bad time.
- **Antivirus software might flag it.** Injecting into another process is exactly what malware does, so this is expected. If you want to be sure what it does, read the source and build it yourself.
- It does not modify any game files on disk. Your designations are saved inside your vehicle files, as an extra `aero` attribute on each marked part, which the game without the plugin ignores. If you save a vehicle while the plugin is not loaded, its designations are lost.

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
| Wing blocks | click modes picked on the next shelf: toggle a block, fill a slab (one thin wing panel), fill body (every block of the body, for thick flying wings), remove a slab |
| Control role | give a control surface or hinged sub-grid a role: pitch, roll, yaw, flap |
| Aerofoil | give a wing or missile fin an aerofoil: pick Lifting surfaces or Missile fins, then a section (hover the icons for their names) |

Wing blocks are painted on the vehicle by role, or by aerofoil in the aerofoil tool. Vanilla wings and control surfaces are wings automatically. Designations belong to the parts themselves: they are saved with the vehicle, and copying and pasting parts (a missile onto another vehicle, say) or undoing an edit takes them along.

## Keybindings

| key | |
|---|---|
| F7 | analysis window: AoA / Mach sweeps, stability derivatives, response simulation, area ruling, voxel view, the pitch / collective for each propeller and rotor's maximum thrust |
| F9 | in flight: flight data (airspeed, Mach, AoA, Cl, Cd, L/D, thrust, air density, static margin…, and each propeller's RPS, thrust, power and vortex ring state), lift/drag arrows, CG / CoL / CoT markers, stalled blocks painted red |

## In flight

The plugin flight model is on by default (F7 window, Settings tab). It switches off the game's aerodynamic forces and applies its own: wings with their aerofoils, missile fins (shaped from the part's mesh; grid fins too), control surfaces (vanilla ones follow the game's own deflection, hinged sub-grids deflect by moving), fuselage lift and drag, ground effect, and Stormworks' own air density curve. Water forces are unchanged.

Propellers and helicopter rotors are flown blade by blade from their shaft speed, blade count, length and pitch: they windmill and drag when unpowered, a free rotor autorotates (gyrocopters), rotors flap and take cyclic, and a driven rotor twists the airframe the other way (single main rotors need a tail rotor). The air's load on the blades slows or drives the shaft. Blade speed is the shaft's RPS (× the part's built-in ratio: 1.5 heavy rotor, 3 aircraft propeller), so set real rotor speeds (a UH-1's 324 rpm main rotor is 5.4 RPS), and they are drawn at that speed (the game draws them 12.6× slower); vanilla builds that spin rotors fast make much more lift than in the base game. "Vortex ring state" in the Settings tab (off by default) makes rotors that sink slowly, nearly straight down, lose lift into their own wake. Fans and boat propellers keep the game's forces.

Blade pitch input 1 is 43° of blade angle, far past stall: rotors lift most at about 0.35 collective. The F7 Rotors tab gives each one's best pitch at the blade speed you enter, and F9 shows the blade angle and a warning when the blades stall.

"Rotor blur" (on by default) draws fast blades as a faint smear over the arc they sweep in about 1/20 s. The flight model and vortex ring settings apply to all vehicles, and the Settings tab can override them for the vehicle open in the workbench: that choice is saved in the vehicle file with its designations.

## Credits

The aerodynamics are a port of FAR Continued (Ferram Aerospace Research, © ferram4 and the KSPModStewards contributors).

## Disclaimer

I am not personally affiliated with the developers or publishers of Stormworks: Build and Rescue, nor have they endorsed this project. Stormworks: Build and Rescue and any of its content or materials are the property of their respective owners.

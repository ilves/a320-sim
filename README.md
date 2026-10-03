# A320 Sim (MVP)

Fly an Airbus A320 out of and back into Tallinn (EETN): take off, fly a circuit, intercept
the ILS and land, from a glass cockpit with a working PFD, ND and E/WD.

- **Flight model:** [JSBSim](https://github.com/JSBSim-Team/jsbsim) (open source, used by
  FlightGear) with its A320 model, plus a simplified Airbus **Normal Law** fly-by-wire.
  That includes load-factor and roll-rate control, flight-path and bank hold, auto-trim
  through the THS, protections (bank, pitch and alpha) and the flare law.
- **Airport:** EETN runway 08/26 at its real coordinates. It has ILS on both runways, PAPI,
  approach and edge lights and runway markings.
- **Cockpit:**
  - PFD: attitude, speed tape with VLS/VMAX, altitude, vertical speed, heading and ILS
    deviation.
  - ND: map in ARC mode with the runway and the ILS extended centreline.
  - E/WD: N1, flaps, gear, THS, warnings and memos.
  - A clickable button panel, plus radio-altimeter callouts ("FIFTY … RETARD").
- **Pause** (P), sim rate ×2/×4, and instant scenarios (lined up, 10 NM final, 4 NM final).
- Engine: **Unreal Engine 5** (Windows). The world is built from code, so the repo contains
  no binary assets.

## Run it on Windows

You need:

1. **Windows 10/11** and a DX12 GPU.
2. **Visual Studio 2022 or 2026**, with these selected in the Visual Studio Installer:
   - workloads: *Desktop development with C++* and *Game development with C++*;
   - individual components: a *Windows 11 SDK* and *MSVC v143 - VS 2022 C++ x64/x86 build
     tools (Latest)*. Unreal Engine 5 builds with v143, including inside VS 2026.

   For a fresh install:
   ```
   winget install Microsoft.VisualStudio.2022.Community --override "--add Microsoft.VisualStudio.Workload.NativeGame --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended --passive"
   ```
   `Setup.bat` lists what it finds and tells you what is missing.
3. **CMake 3.25 or newer**: `winget install Kitware.CMake`. Alternatively, add *C++ CMake
   tools for Windows* in the Visual Studio Installer.
4. **Unreal Engine 5.3 or newer**, from the Epic Games Launcher (Unreal Engine → Library → +).
5. **Git**: `winget install Git.Git`.

You also need about 100 GB of free disk space, mostly for the engine. `Setup.bat` downloads
JSBSim itself.

Then:

```
git clone https://github.com/ilves/a320-sim.git
cd a320-sim
Setup.bat          (once: builds the flight model, runs its tests, compiles the Unreal project)
Play.bat           (starts the simulator in a window)
```

Other scripts:

| Script | What it does |
| --- | --- |
| `OpenEditor.bat` | Opens the project in the Unreal Editor (press Play there). |
| `Package.bat` | Builds a standalone game and zips it as `dist\A320Sim-Win64.zip`. Unzip it on any Windows PC and run `A320Sim.exe`; no Unreal Engine is needed there. |
| `Setup.bat -EngineDir "D:\UE_5.5"` | Uses a specific engine install, if auto-detection picks the wrong one. |
| `Setup.bat -CoreOnly` | Builds and tests only the flight model (no Unreal needed). |
| `Play.bat -Fullscreen` | Starts in fullscreen. |

## Controls

| Action | Keys | Gamepad |
| --- | --- | --- |
| Sidestick | Arrow keys / numpad (Shift = full deflection) | Left stick |
| Rudder and nosewheel | Q / E (or Z / X) | Right stick X |
| Thrust levers | PgUp / PgDn; Home TOGA, Del FLX/MCT, Ins CL, End IDLE | Right stick Y |
| Reversers | R (on the ground), then PgUp | Y |
| Brakes / parking brake | B (hold) / N | B / left trigger |
| Flaps | F retract, V extend | LB / RB |
| Gear | G | A |
| Speedbrake | / | X |
| LS (ILS on the PFD) | L | |
| ND range | , and . | |
| Pause / sim rate | P / = | Start |
| Cockpit ↔ outside view | C; right-drag to look around, middle-click to reset | Back |
| Scenarios | F5 lined up, F6 10 NM final, F7 4 NM final, F9 swap runway 26/08 | |
| Help overlay | H or F1 | |
| Quit | Esc (standalone game) | |

Every action is also a button on the cockpit panel (bottom right).

## First flight

**Takeoff (F5).** You start lined up on runway 26 in CONF 1+F, with the parking brake set.
1. Press **N** to release the parking brake and **Home** for TOGA.
2. At about **150 kt**, hold **Down arrow** to rotate to about 10°. Then hold 15° pitch.
3. Once the vertical speed is positive, press **G** to raise the gear.
4. At 1500 ft, press **Ins** to set CL thrust, and **F** to retract the flaps as the speed
   builds.

**ILS landing (F7, then F6 when you're comfortable).** You start established on the ILS for
runway 26, gear down.
1. Keep both magenta diamonds centred:
   - Small stick inputs: in Normal Law the aircraft holds its flight path when you let go.
   - Thrust holds the speed at about **VLS + 5 kt**, just above the amber strip on the speed
     tape.
2. Around **30 ft**, ease the stick back to flare. When you hear "RETARD", press **End**
   (idle).
3. After touchdown:
   - Press **R** and **PgUp** for reverse thrust, and hold **B** for the brakes.
   - Press **/** for the ground spoilers.

The screen reports your touchdown rate, the distance past the threshold and how far you
were from the centreline.

## How it's built

```
core/                        engine-independent C++17, compiled into A320Core.dll
  include/a320/a320_api.h    plain C API, the only thing Unreal sees
  src/Simulation.cpp         JSBSim wrapper: scenarios, trim, controls, state
  src/FlyByWire.cpp          Normal Law (C*-like pitch, roll rate, protections, flare law)
  src/Ils.cpp                localizer and glideslope from the runway geometry (ICAO Annex 10 sectors), PAPI
  src/Airport.cpp            EETN runway data, runway-aligned frames
  src/Systems.cpp            flaps/1+F logic, thrust detents, VLS/VFE/VMAX, warnings, callouts
  tests/                     unit tests plus scripted JSBSim flights through the C API
unreal/A320Sim/              UE5 project (C++ only)
  Content/JSBSim/            JSBSim aircraft data (A320 model, CFM56 engine)
  Source/A320Sim/            aircraft pawn, cockpit HUD, airport/world builder, input
scripts/, *.bat              Windows setup, run and package scripts
```

The simulation runs at a fixed 120 Hz, decoupled from the frame rate. Pausing stops sim time
while rendering and the cockpit keep running.

### Tests

```
cmake -S core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The flight tests fly the real JSBSim A320 with a scripted pilot:
- a takeoff that rotates without a tail strike, lifts off before the runway end and climbs
  away;
- Normal Law flight-path and bank hold;
- ILS approaches from 4 NM and 10 NM, which must stay within 0.5 dot of the localizer and
  0.6 dot of the glideslope, touch down in the touchdown zone below 600 fpm and stop on the
  runway.

CI runs these tests on Linux and Windows, and runs `Setup.ps1 -CoreOnly` on Windows.

### Changes to JSBSim's A320 model

These are marked `a320-sim:` in `unreal/A320Sim/Content/JSBSim/aircraft/A320/A320.xml`:
- **Trimmable horizontal stabiliser (THS)** added. The original model trims with the
  elevator alone.
- **Cmα changed from -4.0 to -2.0.** The original value needed more nose-up trim on approach
  than the A320's THS range allows.
- **Nosewheel steering raised from 5° to 75°.** The core reduces it with speed (tiller vs
  pedals).

## Limitations and next steps

**Not yet built:**
- autopilot and autothrust (FCU);
- weather radar and TCAS;
- detailed terrain (Cesium for Unreal);
- a 3D clickable cockpit;
- sounds;
- wind and low visibility;
- failures.

**Approximations:**
- ILS frequencies and idents are not in the data yet. The glideslope uses standard values
  (3°, 50 ft threshold crossing height); verify them against the Estonian AIP.
- Magnetic variation is fixed at 9.5° E.

**Licences:**
- JSBSim is LGPL-2.1. Its A320 model is marked "for educational and entertainment purposes
  only … not to be sold".
- "Airbus" and "A320" are trademarks of Airbus. This project is not affiliated with Airbus.

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
  - ND: map in ARC or ROSE NAV with the runway and the ILS extended centreline, and ROSE LS
    with the ILS course pointer, deviation bar and glideslope scale.
  - E/WD: N1, flaps, gear, THS, warnings, memos and the LDG MEMO.
  - FCU and EFIS controls on the glareshield.
  - Pedestal: draggable thrust levers (detents, reverse), speedbrake lever with ARM, flaps
    lever, engine masters, ENG MODE selector and parking brake.
  - Centre panel: gear lever and the AUTO/BRK pushbuttons.
  - A pop-up overhead panel: APU, APU bleed, exterior lights and signs.
- **Autopilot and autothrust** through the FCU: HDG, LOC, APPR with **autoland** (LOC*, LOC,
  G/S*, G/S, LAND, FLARE, ROLL OUT), OPEN CLIMB/DESCENT, V/S, ALT capture and hold, and SPEED
  autothrust with RETARD.
  - AP1 and AP2: both engage only with LOC or APPR armed (CAT 3 DUAL), as on the aircraft.
  - The PFD's flight mode annunciator boxes a new mode for 10 s and shows CAT 3 SINGLE/DUAL.
- **Lessons (F3)** that guide you step by step, highlight the control or display to use, tick
  each step off when you've done it, and say what the same thing is in MSFS 2024. The first
  one is an ILS approach and autoland, from a 20 NM intercept to a stop on the runway.
- **Sim tutor:** when the aircraft refuses a press (AP on the ground, flaps above VFE, APPR
  below 400 ft, an engine start without bleed air…), a message says why.
- **Sound:**
  - engines that follow N1/N2, plus airflow, gear, runway and speedbrake rumble;
  - touchdown thump and gear clunks;
  - master-warning chimes and the autopilot-disconnect "cavalry charge";
  - spoken radio-altimeter callouts ("FIFTY … RETARD") and GPWS ("GLIDE SLOPE", "SINK RATE",
    "STALL").
- **Systems behind the switches:**
  - APU start;
  - engine start from cold and dark (starter, fuel at about 20% N2, spool to idle);
  - ground spoilers that extend at touchdown;
  - autobrake LO/MED/MAX holding 1.7 m/s², 3 m/s² or full braking;
  - exterior lights on the model, and cabin signs with the cabin chime.
- **Pause** (P), sim rate ×2/×4, and instant scenarios (lined up, cold and dark, 20 NM
  intercept, 10 NM final, 4 NM final). These are in the bar at the top right.
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

The first start compiles the engine's shaders, which takes 5-15 minutes; later starts take seconds.
While it starts, the `Play.bat` window shows a progress bar with the current stage (engine, flight
model, shaders left to compile) and closes itself when the simulator is ready, and the game window
shows a "Preparing graphics" panel with a progress bar instead of a black screen. If the game
closes during start-up, `Play.bat` prints the last lines of its log
(`unreal\A320Sim\Saved\Logs\A320Sim.log`) and stays open. `Play.bat -NoWait` skips the progress
window.

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
| Sidestick | Arrow keys / numpad (Shift = full deflection), or a USB joystick | Left stick |
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
| Scenarios | F5 lined up, F4 20 NM intercept, F6 10 NM final, F7 4 NM final, F9 swap runway 26/08 | |
| Lessons | F3 | |
| Autopilot / autothrust | A (AP1), Shift+A (AP2) / T (A/THR; levers in CL: Ins) | |
| Approach / localizer | K (APPR, autoland) / J (LOC) | |
| FCU targets | 1/2 SPD, 3/4 HDG, 5/6 ALT, 7/8 V/S (Shift = ×10) | |
| FCU modes | U fly the HDG, 9 climb/descend to ALT (LVL/CH), 0 hold V/S | |
| Sound on/off, silence master warning | - (minus), M or click MASTER WARN | |
| Overhead panel | O | |
| Joystick setup | F2 | |
| Cold and dark | Shift + F5 | |
| Help overlay | H or F1 | |
| Quit | Esc (standalone game) | |

Every action is also on the cockpit panels, where you click switches and drag levers. The
simulator functions (pause, views, scenarios, sound) are in the bar at the top right.

**USB joystick, throttle and rudder pedals.** Any Windows game controller works: the sim
reads it through the Windows joystick API, so it needs no drivers or plugins. Plug it in
before or while the sim runs; it's picked up within 5 seconds.
- **Defaults:**
  - stick X/Y for roll and pitch, twist (R) for rudder, and the slider (Z) for the thrust
    levers, which snap into the TOGA, FLX/MCT, CL and IDLE detents;
  - trigger for autopilot disconnect, button 2 for brakes, 3/4 for flaps up/down, 5 for
    gear and 6 for reverse;
  - the hat switch looks around.
- **Setup panel:** open it with **F2** or **JOYSTICK** in the top bar. It shows each axis
  live.
  - Click **LEARN** next to a function, then move the axis you want for it.
  - Use **INV** if an axis moves the wrong way.
  - Rudder pedals with toe brakes work too: learn RUDDER, BRAKE L and BRAKE R on the pedals'
    axes.
- **Settings file:** `unreal\A320Sim\Saved\A320Joystick.ini`. Edit `buttonN=` lines to remap
  buttons. Available commands: `AP_DISCONNECT`, `BRAKES`, `FLAPS_UP`, `FLAPS_DOWN`, `GEAR`,
  `REVERSE`, `SPEEDBRAKE`, `VIEW`, `PAUSE`, `TOGA`, `IDLE`, `AP1`, `AP2`, `ATHR`.

**USB throttle quadrant** (e.g. Thrustmaster TCA Quadrant Airbus Edition, Saitek/Logitech
Throttle Quadrant, or any separate throttle):
- **Automatic set-up:** the first time the sim sees a device named like a throttle or
  quadrant, it binds **THRUST 1** to it. On a two-lever quadrant it also binds **THRUST 2**,
  so each lever drives its own engine. Stick functions that were on it move to your stick.
  Devices are remembered by name, so plugging them in a different order doesn't matter.
- **Calibrate the detents:** open the setup panel (**F2**), press **CALIBRATE THRUST** and
  follow the steps. Put the levers in IDLE, CL, FLX/MCT and TOGA, pressing **SET** each
  time, then in full reverse (or press **NO REVERSE** if your throttle has no reverse range).
  - After that the levers click into the sim's detents exactly where your hardware's are.
  - On the ground, pulling the levers behind IDLE deploys the reversers. In flight that
    gives idle thrust, as in the real aircraft.
  - A lever that runs backwards is detected and inverted automatically.
- **Two levers:** the pedestal and E/WD show both levers. A/THR works below each engine's own
  lever, so a retarded lever keeps its engine back. With one lever, or THRUST 2 unbound, it
  moves both.
- **Quadrant buttons:** the setup panel shows the numbers of the throttle's buttons as you
  press them. Map them with `throttleButtonN=` lines in the settings file (`buttonN=` is for
  the stick). Besides the commands above there are:
  - `ATHR_DISCONNECT`, the instinctive disconnect on the levers;
  - switches that are on while held: `ENG1_MASTER`, `ENG2_MASTER`, and `ENG_MODE_CRANK` /
    `ENG_MODE_IGN` (released = NORM).

**Cold and dark start (Shift + F5).**
1. Press **O** to open the overhead panel. Press APU **MASTER SW**, then **START**.
2. After about 30 s **AVAIL** lights up. Press **APU BLEED**.
3. On the pedestal, set **ENG MODE IGN/START**, then **ENG 1 ON**. N2 spools on the starter,
   fuel comes on at about 20%, and the engine settles at idle after about 30 s.
4. Start **ENG 2** the same way, then set **ENG MODE NORM** and switch APU BLEED off.
5. Before takeoff: set the flaps to 1, **ARM** the spoilers, set **AUTO/BRK MAX**, and switch
   the lights on (strobe, landing, nose T.O).

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

**Lessons (F3).** Pick a lesson and press START. The aircraft resets to the lesson's start, and
a panel on the left takes you through it:
- **DO** says what to do. The control is outlined in yellow, with a line from the panel.
- **LOOK FOR** says what changes when it worked, usually on the FMA.
- **HOW IT WORKS** explains why, and **IN MSFS 2024** gives the same action in Microsoft Flight
  Simulator.
- A step ticks itself off when the cockpit shows it is done. Check steps need **CHECKED:
  NEXT**, and **SKIP** / **BACK** move between steps.
- An amber box warns when something needs attention now, for example the autopilot
  disconnected or the gear still up.

*ILS approach and autoland* starts 20 NM from runway 26, at 3000 ft and 220 kt, with AP1 and
A/THR flying HDG and ALT. It covers:
- autobrake, LS and ND ROSE LS, and checking the ILS;
- slowing down with flaps 1 and 2, arming APPR and engaging AP2 (CAT 3 DUAL);
- LOC and G/S capture, gear down, flaps FULL, VAPP and spoilers armed, and the LDG MEMO;
- LAND, FLARE and RETARD (thrust levers to idle);
- touchdown, full reverse, stowing the reversers at 70 kt, braking to a stop and the parking
  brake.

The lesson ends once you have stopped on the runway.

**Using the autopilot.** The FCU strip above the displays works like the real one:
- the selected values are in the amber windows;
- −/+ change them, and Shift-click changes them ten times faster;
- the HDG, LVL/CH and V/S buttons "pull" the knob, which tells the autopilot to fly the
  selected value.

1. **After takeoff.** The FCU is preset to 200 kt and 5000 ft.
   1. Above 100 ft, press **Ins** to put the thrust levers in CL, **T** for A/THR and **A**
      for AP1.
   2. Press **9** (LVL/CH): the autopilot climbs in OP CLB and levels off at the selected
      altitude.
   3. To turn, set a heading with **3/4** and press **U**.
2. **Descent.** Set a lower altitude with **5/6**, then press **9** for an idle descent, or
   **0** for a fixed vertical speed (set it with **7/8**).
3. **Autoland.** Start from F6, or head towards the localizer within about 30°.
   1. With AP1 and A/THR on, press **K** (APPR). LOC and G/S show armed in cyan, then capture
      in green.
   2. Set FLAPS FULL around 7 NM, and the speed to VLS + 5 with **1/2**.
   3. The aircraft flies LAND, FLARE and RETARD, then ROLL OUT on the centreline. Press
      **End** at "RETARD".
   4. After touchdown, press **R**/**PgUp** for reverse thrust and hold **B** to brake.
   5. Press **A** to disconnect the autopilot. You'll hear the cavalry charge.

A firm sidestick input also disconnects the autopilot.

## How it's built

```
core/                        engine-independent C++17, compiled into A320Core.dll
  include/a320/a320_api.h    plain C API, the only thing Unreal sees
  src/Simulation.cpp         JSBSim wrapper: scenarios, trim, controls, state
  src/FlyByWire.cpp          Normal Law (C*-like pitch, roll rate, protections, flare law)
  src/Ils.cpp                localizer and glideslope from the runway geometry (ICAO Annex 10 sectors), PAPI
  src/Airport.cpp            EETN runway data, runway-aligned frames
  src/Systems.cpp            flaps/1+F logic, thrust detents, VLS/VFE/VMAX, warnings, callouts
  src/Autopilot.cpp          FCU modes, autopilot guidance, autothrust, autoland
  src/Audio.cpp              cockpit sound synthesis and mixing (engines, airflow, chimes, callouts)
  tests/                     unit tests plus scripted JSBSim flights through the C API
unreal/A320Sim/              UE5 project (C++ only)
  Content/JSBSim/            JSBSim aircraft data (A320 model, CFM56 engine)
  Content/Sounds/            spoken callouts (generated by tools/make_callouts.py with eSpeak NG)
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
  runway;
- the autopilot: OP CLB with ALT capture, a HDG turn, a V/S descent with capture, LOC
  intercept from a heading, instinctive disconnect, and a full autoland to a stop;
- the audio engine: WAV loading, engine level following thrust, pause, callouts, and the
  master warning and its acknowledgment;
- joystick mapping: axis scaling, deadzone, throttle detents, learn mode and the settings
  file;
- throttle quadrants: detent calibration (including a backwards axis and reverse), devices
  kept by name, and automatic quadrant set-up;
- cockpit systems: a cold-and-dark start (APU, starter, light-off, crossbleed, shutdown),
  ground spoilers, autobrake LO/MED deceleration and stopping distance, and the autobrake
  disarming when the pilot brakes;
- split thrust levers: each engine follows its own lever, reverse on one engine, and a lever
  in reverse in flight giving idle;
- the ILS autoland lesson, flown by a test pilot who does exactly what each step says from 20
  NM out to a stop: every step must tick off, the FMA must show LOC*, G/S* and G/S, and the
  touchdown must be in the touchdown zone below 600 fpm near VAPP;
- the 20 NM intercept scenario holding 3000 ft and 220 kt, the AP1/AP2 rules, and the sim
  tutor messages.

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
- managed (FMS) modes, a flight plan, alpha floor and the flight directors;
- go-around (TOGA during an approach: SRS and GA TRK);
- weather radar and TCAS;
- detailed terrain (Cesium for Unreal);
- a 3D clickable cockpit;
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

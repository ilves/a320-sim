# A320 Sim (MVP)

Fly an Airbus A320 out of and back into Tallinn (EETN), or between Tallinn and Kuressaare (EEKE):
take off, fly a circuit or the route, intercept the ILS and land, from a glass cockpit with a
working PFD, ND and E/WD. A FLIGHT menu at start picks the airports and how the flight starts.

- **Flight model:** [JSBSim](https://github.com/JSBSim-Team/jsbsim) (open source, used by
  FlightGear) with its A320 model, plus a simplified Airbus **Normal Law** fly-by-wire.
  That includes load-factor and roll-rate control, flight-path and bank hold, auto-trim
  through the THS, protections (bank, pitch and alpha) and the flare law.
- **Airports:** Estonia's seven public airports from the eAIP, at their real coordinates and
  elevations, with PAPI, approach and edge lights, runway markings, and taxiways and aprons from
  OpenStreetMap: Tallinn EETN (ILS 08 and 26), Tartu EETU (ILS 26), Kuressaare EEKE (ILS 17),
  Pärnu EEPU, Kärdla EEKA, and the 600 m grass strips of Ruhnu EERU and Kihnu EEKU.
- **World map** (F12): all of Estonia, zoom, move, search towns and airports, pick the departure
  and the destination (an airport, a town or any point), the route and the aircraft. See
  [World map](#world-map).
- **Scenery:** real terrain from open data, satellite imagery on real elevation:
  - **All of Estonia:** about 20 m per pixel, islands included.
  - **Around Tallinn:** 80 × 80 km at about 10 m per pixel.
  - **Buildings:** about 380,000 OpenStreetMap buildings: Tallinn, around every airport, and the
    towns of over about 4,000 people.
  - **Performance:** only what is near the aircraft is loaded, streamed in the background. See
    [Scenery](#scenery).
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
  - A pop-up **MCDU** (Tab), see [MCDU](#mcdu-flight-computer).
  - A **RADIO** window (F10): VHF 1 radio panel, transponder and the ATC radio, see
    [Radio and ATC](#radio-and-atc).
- **Autopilot and autothrust** through the FCU: HDG, LOC, APPR with **autoland** (LOC*, LOC,
  G/S*, G/S, LAND, FLARE, ROLL OUT), OPEN CLIMB/DESCENT, V/S, ALT capture and hold, and SPEED
  autothrust with RETARD.
  - HDG-V/S / TRK-FPA pushbutton: TRK holds the ground track (the wind correction comes by
    itself) and FPA a flight path angle (−9.9° to +9.9°). The windows, the FMA and your
    hardware FCU switch with it; the PFD shows the flight path vector ("bird").
  - **NAV** along the flight plan's route (see [Route and NAV](#route-and-nav)): armed on the
    ground, it engages at 30 ft; in the air HDG push engages it. LOC and G/S capture from it.
  - Knob pushes: HDG push is NAV (without a route it holds the present heading, wings level);
    ALT push levels off at the present altitude, since managed climb/descent is not built; V/S
    push levels off with V/S 0 (FPA 0).
  - AP1 and AP2: both engage only with LOC or APPR armed (CAT 3 DUAL), as on the aircraft.
  - The PFD's flight mode annunciator boxes a new mode for 10 s and shows CAT 3 SINGLE/DUAL.
  - Speed protection: A/THR and the autopilot never fly slower than VLS or faster than VMAX,
    whatever speed is selected.
  - **Alpha floor:** close to the stall, A/THR engages by itself with TOGA thrust (A.FLOOR,
    amber box). It then stays in TOGA LK until you press A/THR, as on the aircraft. Auto-trim
    stops in alpha protection.
- **Lessons (F3)** that guide you step by step, highlight the control or display to use, tick
  each step off when you've done it, and say what the same thing is in MSFS 2024:
  - **Takeoff from cold and dark** (runway 26): APU and engine start, MCDU setup (INIT, F-PLN,
    PERF TAKE OFF), ATIS, IFR clearance and squawk, takeoff clearance, takeoff, thrust reduction,
    flaps up and radar contact.
  - **ILS approach and autoland:** from a 20 NM intercept to a stop on the runway.
  - **Radio: a full flight with ATC:** clearance to landing clearance, with the radar vectors.
- **Sim tutor:** when the aircraft refuses a press (AP on the ground, flaps above VFE, APPR
  below 400 ft, an engine start without bleed air…), a message says why.
- **Sound:**
  - engines that follow N1/N2, plus airflow, gear, runway and speedbrake rumble;
  - touchdown thump and gear clunks;
  - master-warning chimes and the autopilot-disconnect "cavalry charge";
  - spoken radio-altimeter callouts ("FIFTY … RETARD") and GPWS ("GLIDE SLOPE", "SINK RATE",
    "STALL");
  - takeoff calls: "ONE HUNDRED KNOTS", "V ONE", "ROTATE" and "POSITIVE CLIMB". V1, VR and V2
    come from the weight and flaps, and show under the PFD speed tape before takeoff, with V1
    as a cyan "1" on the tape. Pulling the levers back before V1 (a rejected takeoff) stops
    the calls.
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
| Flight menu | F11: departure and arrival, cold and dark / lined up / in the air, distance, lessons | |
| World map | F12: wheel to zoom, drag to move, click to select, type to search, Enter to fly | |
| Scenarios | F5 lined up, Shift+F5 cold and dark, F4 in the air, F6 10 NM final, F7 4 NM final, F9 other runway direction | |
| Lessons | F3 | |
| Autopilot / autothrust | A (AP1), Shift+A (AP2) / T (A/THR; levers in CL: Ins) | |
| Approach / localizer | K (APPR, autoland) / J (LOC) | |
| FCU targets | 1/2 SPD, 3/4 HDG, 5/6 ALT, 7/8 V/S (Shift = ×10) | |
| FCU modes | U fly the HDG, 9 climb/descend to ALT (LVL/CH), 0 hold V/S | |
| FCU pushes, TRK-FPA | Shift+U hold heading, Shift+9 level off, Shift+0 V/S 0; \ HDG-V/S / TRK-FPA | |
| Sound on/off, silence master warning | - (minus), M or click MASTER WARN | |
| Overhead panel | O | |
| MCDU | Tab (type on the keyboard while it's open, Backspace = CLR, Esc closes) | |
| Radio / ATC | F10 (keys 1-6 pick a reply while it's open) | |
| Joystick setup | F2 | |
| Cold and dark | Shift + F5 | |
| Help overlay | H or F1 | |
| Quit | Esc (standalone game) | |

Every action is also on the cockpit panels, where you click switches and drag levers. The
simulator functions (pause, views, scenarios, sound) are in the bar at the top right.

**USB joystick, throttle, rudder pedals and cockpit panels.** Any Windows game controller
works: the sim reads it through DirectInput (up to 8 axes and 128 buttons per device), so it
needs no drivers or plugins. Plug it in before or while the sim runs; it's picked up within
5 seconds.
- **Defaults:**
  - stick X/Y for roll and pitch, twist (R) for rudder, and the slider (Z) for the thrust
    levers, which snap into the TOGA, FLX/MCT, CL and IDLE detents;
  - trigger for autopilot disconnect, button 2 for brakes, 3/4 for flaps up/down, 5 for
    gear and 6 for reverse;
  - the hat switch looks around.
- **Setup panel, AXES page:** open it with **F2** or **JOYSTICK** in the top bar. It shows each
  axis live.
  - Click **LEARN** next to a function, then move the axis you want for it. The functions are
    PITCH, ROLL, RUDDER, THRUST 1 and 2, FLAPS, SPEEDBRAKE and the toe brakes.
  - Use **INV** if an axis moves the wrong way.
  - **CAL** next to THRUST 1, FLAPS and SPEEDBRAKE teaches the sim where your lever's detents
    are (see below).
- **Setup panel, BUTTONS page:** every cockpit command a button, switch position or knob
  click can drive. Click **SET** next to one, then press the button, flip the switch or turn
  the knob one click. **X** clears it. Each knob click counts, even fast ones, so FCU
  encoders step one by one.
  - Switches are on while held: ENG 1/2 MASTER, ENG MODE CRANK and IGN/START (released =
    NORM), PARKING BRAKE, SPOILERS ARM, ALT 100/1000.
  - Selector positions set on press: ND mode ARC/NAV/LS, ND range 10…320, gear UP/DOWN,
    autobrake OFF/LO/MED/MAX.
  - Knobs and buttons act once per click: AP1, AP2, A/THR, LOC, APPR, SPD/HDG/ALT/V/S +/−,
    HDG/ALT/V/S pull and push, HDG-V/S / TRK-FPA, LS, ND range +/−, the A/THR instinctive disconnect
    and more.
- **Settings file:** `unreal\A320Sim\Saved\A320Joystick.ini` (`bind=` lines). Older
  `buttonN=` / `throttleButtonN=` lines still work.

**USB throttle quadrant** (e.g. Thrustmaster TCA Quadrant Airbus Edition, Saitek/Logitech
Throttle Quadrant, or any separate throttle):
- **Automatic set-up:** the first time the sim sees a device named like a throttle or
  quadrant, it binds **THRUST 1** to it. On a two-lever quadrant it also binds **THRUST 2**,
  so each lever drives its own engine. Stick functions that were on it move to your stick.
  Devices are remembered by name, so plugging them in a different order doesn't matter.
- **Calibrate the detents:** open the setup panel (**F2**), press **CAL** next to THRUST 1 and
  follow the steps. Put the levers in IDLE, CL, FLX/MCT and TOGA, pressing **SET** each
  time, then in full reverse (or press **NO REVERSE** if your throttle has no reverse range).
  - After that the levers click into the sim's detents exactly where your hardware's are.
  - On the ground, pulling the levers behind IDLE deploys the reversers. In flight that
    gives idle thrust, as in the real aircraft.
  - A lever that runs backwards is detected and inverted automatically.
- **Two levers:** the pedestal and E/WD show both levers. A/THR works below each engine's own
  lever, so a retarded lever keeps its engine back. With one lever, or THRUST 2 unbound, it
  moves both.
- **Quadrant buttons:** assign them on the BUTTONS page: the A/THR disconnect buttons on the
  levers, the ENG 1/2 MASTER switches and the ENG MODE selector.

**Quadrant add-on (flaps, speedbrake, gear, autobrake, parking brake),** e.g. the Thrustmaster
TCA Quadrant Add-On Airbus Edition:
1. On the AXES page, **LEARN** FLAPS and move the flaps lever, then **LEARN** SPEEDBRAKE and
   move the speedbrake lever.
2. Press **CAL** next to FLAPS and set the lever in 0, 1, 2, 3 and FULL.
3. Press **CAL** next to SPEEDBRAKE and set it in RET, FULL and ARM (or **NO ARM** if your lever
   has no ARM position: then assign SPOILERS ARM to its button).
4. On the BUTTONS page, assign:
   - the gear lever. Move it and watch **Last pressed**. If each position presses its own
     button, assign GEAR UP and GEAR DOWN. If only one position presses a button (and the
     other just releases it), assign that button to **GEAR DOWN switch** (pressed when the
     lever is down) or **GEAR UP switch** (pressed when up);
   - each autobrake knob position to AUTOBRAKE OFF/LO/MED/MAX;
   - the parking brake switch to PARKING BRAKE.

The levers then move the sim's flaps and speedbrake levers with them. Rudder trim is not
simulated yet.

**WingFlex FCU Cube and EFIS Cube:** the sim talks to them directly over USB, so nothing
needs assigning:
- **Buttons and knobs:** AP1, AP2, A/THR, LOC, APPR and HDG-V/S / TRK-FPA. The SPD, HDG, ALT
  and V/S knobs turn the targets; HDG, ALT and V/S pull and push. ALT steps 100 or 1000 ft with the
  100/1000 switch. On the EFIS: LS, the ND mode and range knobs, and MASTER WARN/CAUT.
- **Lights and displays:** the AP1, AP2, A/THR, LOC and APPR lights. The SPD, HDG, ALT and V/S
  windows show the sim's targets, with dashes where the real FCU shows them (HDG on the
  localizer, V/S outside V/S or FPA mode) and TRK-FPA lit when it is selected. The EFIS shows LS and flashing MASTER WARN/CAUT, and the
  baro window shows STD.
- The panels' own brightness knobs set the brightness.
- **Close WingFlex Bridge while flying this sim,** or both programs drive the displays. The
  setup panel (F2) shows "WingFlex panels: FCU Cube EFIS Cube" when they are found.
- The protocol comes from MobiFlight's open-source WingFlex drivers (MIT licence). It is for
  the FCU Cube (USB ID A316:C787) and EFIS Cube (A516:C987).

**Other FCU and EFIS panels** (e.g. Winwing): their buttons and knobs are game controller
buttons, so assign them on the BUTTONS page. Their displays and lights are driven by their
maker's software, so they don't show this sim's values; the FCU strip on screen does.

**Cold and dark start (Shift + F5).**
1. Press **O** to open the overhead panel. Press APU **MASTER SW**, then **START**.
2. After about 30 s **AVAIL** lights up. Press **APU BLEED**.
3. On the pedestal, set **ENG MODE IGN/START**, then **ENG 1 ON**. N2 spools on the starter,
   fuel comes on at about 20%, and the engine settles at idle after about 30 s.
4. Start **ENG 2** the same way, then set **ENG MODE NORM** and switch APU BLEED off.
5. Before takeoff: set the flaps to 1, **ARM** the spoilers, set **AUTO/BRK MAX**, and switch
   the lights on (strobe, landing, nose T.O).

## MCDU (flight computer)

Press **Tab**, or MCDU at the top right. It works like the A320's MCDU:
- Type into the scratchpad, on the keyboard or the MCDU's keypad.
- Put the entry into a field with the line select key next to it (the `-` buttons beside the
  screen).
- Error messages appear in the scratchpad, for example NOT IN DATA BASE or FORMAT ERROR.
  CLR removes the message, then deletes characters.
- CLR on an empty scratchpad, then a line select key, deletes that field.

**Land on another runway (switch the ILS):**
1. Press **F-PLN**, then the destination line (LSK 6L) to open LAT REV FROM EETN.
2. Press **ARRIVAL>** and pick the approach, for example `<ILS08`. It shows in yellow
   (temporary).
3. Press **INSERT\*** (LSK 6R).

The FMGC tunes the ILS by itself:
- **On the ground:** the departure runway's ILS.
- **After takeoff:** the arrival's ILS, so a takeoff on 26 with ILS08 inserted retunes to IIB
  108.30 once airborne.
- **On the displays:** the PFD (bottom left), the ND and the FMA follow the change.

**Pages:**

| Key | Page |
|---|---|
| F-PLN | The route's waypoints with each leg's track and distance (BRG and distance to the active one), constraints in magenta, the destination with the distance to go along the route. The slew keys (↑ ↓) scroll it. The origin's line opens DEPARTURE, the destination's (LSK 6L) ARRIVAL |
| RAD NAV | The tuned ILS (small font = auto-tuned). Type `ILK`, `109.30` or `ILK/109.30` at LSK 3L to tune it yourself, CLR to go back to auto. LSK 4L sets the course |
| PERF | On the ground: TAKE OFF, with V1/VR/V2, FLAPS/THS, FLEX TO TEMP and THR RED/ACC. In the air: APPR, with QNH, temperature, wind, minimums (BARO or RADIO) and LDG CONF |
| INIT | FROM/TO (e.g. `EETN/EEKE`, on the ground), flight number, cost index and cruise level |
| PROG | Flight phase, bearing and distance to the runway, tuned ILS and DME |

**What the entries do:**
- **V-speeds:** V1/VR/V2 drive the PFD markers and the V1 and ROTATE calls. Empty fields fall
  back to computed speeds, shown small next to the labels; LSK with an empty scratchpad copies
  one in.
- **Minimums:** RADIO (DH) or BARO (MDA) adds the HUNDRED ABOVE and MINIMUM calls, and shows DH
  or BARO on the FMA.
- **VAPP:** computed as VLS + 5 kt, or more with a headwind from the entered wind. You can
  overwrite it.
- **Display only:** FLEX TO TEMP, FLAPS/THS and QNH are shown but don't change thrust or the
  altimeter yet.
- **Not in this sim:** VORs, ADFs, SIDs/STARs and DIR TO.

The ILS data is the published one (Estonian eAIP, EETN AD 2.19, AIRAC 2026-10-01):

| Runway | ILS | Frequency | Course | Glide path |
|---|---|---|---|---|
| 08 | IIB | 108.30 | 080° | 3°, RDH 54 ft |
| 26 | ILK | 109.30 | 260° | 3°, RDH 54 ft |
| EEKE 17 | IWA | 109.90 | 171° | 3°, RDH 52 ft (EEKE AD 2.19) |
| EETU 26 | IUM | 108.50 | 257° | 3°, RDH 47 ft (EETU AD 2.19) |

EEKE runway 35 has no ILS: the ARRIVAL page lists it as NO ILS, for a visual approach.

The DME is co-located with the glide path antenna, as at EETN, so it reads the distance to
the touchdown zone.

## Radio and ATC

Tallinn's real ATC units and frequencies (Estonian eAIP, EETN AD 2.18) talk to you with ICAO
phraseology:

| Station | Frequency | What it does here |
|---|---|---|
| Tallinn Information (ATIS) | 124.880 | Runway in use, weather, QNH, information letter (changes at hh20/hh50 UTC) |
| Tallinn Tower | 135.905 | IFR clearance, takeoff clearance, landing clearance |
| Tallinn Radar | 127.905 | Radar contact, vectors to the ILS, approach clearance |
| Tallinn Handling | 131.905 | After landing |
| Kuressaare Information (AFIS) | 118.055 | EEKE: relays the IFR clearance, reports the runway free |
| Tartu Information (AFIS; ATIS 123.130) | 133.905 | EETU, as Kuressaare |
| Pärnu Information (AFIS) | 135.305 | EEPU, as Kuressaare |
| Kärdla Information (AFIS) | 133.405 | EEKA, as Kuressaare |
| Ruhnu Radio | 118.055 | EERU, as Kuressaare |
| Kihnu Traffic | 135.305 | EEKU has no ATS: you announce yourself, nobody answers |

At Kihnu, Tallinn Radar gives the IFR clearance itself ("call me airborne"); you announce
"departing runway 22" on Kihnu Traffic and call Radar once airborne. Arriving there, Radar ends
its service ("frequency change approved") and you announce your final yourself.

Tallinn Radar works the whole route. Kuressaare has no tower: its AFIS officer gives
information, not clearances. You hear "runway 17 free, take off at your discretion" and read back
"taking off runway 17"; on arrival, "runway 17 free, wind calm, QNH 1013" and "landing runway
17". "Cleared for takeoff" or "cleared to land" there is a wrong readback, and the tutor says why.

**The radio window (F10, or RADIO at the top right):**
- **VHF 1:** set the next frequency in STBY with the MHz and kHz knobs (8.33 kHz channels), then
  swap it to ACTIVE with `<->`. You only hear, and are only heard on, the ACTIVE frequency.
- **Transponder:** type the four-digit squawk on the keypad and select AUTO. Radar only gives
  "radar contact" when it sees your code.
- **Radio log:** what was said on your frequency. ATC is in white, you are in blue, the ATIS in
  grey.
- **Replies:** what you can say now, by click or keys 1–6 (requests, check-ins and readbacks).
  - For a readback you get the correct one and wrong ones, as a test. A wrong one gets
    "negative, I say again", and the sim tutor explains what was wrong.

**How ATC behaves:**
- **Voices:** ATC speaks with the Windows text-to-speech voices, through a radio filter with
  static and squelch.
  - English voices are used if installed. Add them in Windows Settings > Time & language >
    Speech.
  - Your own transmissions use a second voice.
  - Without voices, everything still shows as text.
- **Unanswered instructions:** they are repeated, then ATC asks "how do you read?". If you're on
  the wrong frequency, the tutor tells you who is calling and where.
- **Compliance:** headings or altitudes you don't fly are queried ("check heading", "check
  altitude").
- **Takeoff and landing:** taking off or landing without a clearance is pointed out.

**A flight:**
1. Listen to the ATIS.
2. Ask Tower for the IFR clearance. It is "cleared to Tallinn via radar vectors, runway heading,
   climb 4000 feet, squawk ….". Read it back.
3. Set the squawk and 4000 ft, then report "ready for departure".
4. After takeoff Tower hands you to Radar. Check in.
5. Radar vectors you around: downwind, descend 3000 ft, base, then a 30° intercept with
   "cleared ILS approach runway 26".
6. On the localizer, Radar hands you to Tower for the landing clearance. After landing, Tower
   sends you to Handling.
7. You can request the other runway's ILS from Radar. Select it in the MCDU too.

To the other airport, Radar climbs you to FL090 after radar contact (set STD passing 5000 ft),
vectors you there and descends you to 3000 ft in time for the approach.

Your callsign is the MCDU's flight number (INIT page), or SIM320.

**Lesson:** F3, *Radio: a full flight with ATC*, walks you through all of this step by step.

## Choosing a flight

The FLIGHT menu opens at start (F11, or FLIGHT at the top right). Everything in it is optional:
close it to fly the default, lined up on EETN 26.
- **FROM and TO:** an airport (by its ICAO code) and a runway. The same airport is a local
  flight; another is a route (Tallinn to Tartu is 90 NM). The MCDU flight plan, the ILS and ATC
  are set up for it. CHOOSE ON MAP opens the [world map](#world-map).
- **START:** cold and dark at the runway, lined up with engines running, in the air, or on final
  at 10 or 4 NM.
- **DISTANCE:** for a start in the air: 10, 20, 40 or 80 NM from the arrival runway. Beyond 20 NM
  you start higher (on a 3° profile, at most FL200) and Radar vectors and descends you.
- **MCDU:** FLIGHT PLAN ENTERED fills it all in: FROM/TO, flight number, cost index, cruise
  level, departure and arrival, PERF TAKE OFF (V-speeds, 1/UP0.0, FLEX 50) and PERF APPR (QNH,
  wind, a CAT I minimum). NOT ENTERED leaves it to you: INIT FROM/TO, F-PLN DEPARTURE and
  ARRIVAL, PERF. In the air FROM/TO is always set; without the approach inserted no ILS is tuned.
- **LESSONS:** start a lesson (they are set at Tallinn).

## World map

F12, or MAP at the top: Estonia from the satellite imagery, with the motorways and main roads,
rivers, lakes and railways from OpenStreetMap; towns, villages, islands and lakes are labelled,
more as you zoom in.
- **Move and zoom:** drag with the left mouse button, the mouse wheel zooms at the cursor (or
  `+` / `-`); AIRCRAFT centres on your position (the yellow arrow).
- **Search:** click the search box and type a town, village, island, lake, airport name or ICAO
  code (`parnu` finds Pärnu); Enter or a click on a result goes there.
- **Select:** click an airport, a label or any point on the map.
  - **DEPART FROM HERE** (airports): the flight starts there, on its ILS runway if it has one.
  - **FLY TO HERE:** an airport becomes the arrival, with ATC; a town or a map point makes a free
    flight without ATC (RADIO can switch it on) with the route on the map and on the ND.
- **FLY** (or Enter) starts the flight as the FLIGHT menu is set (cold and dark, lined up, in the
  air...); FLIGHT OPTIONS goes back to that menu.
- The magenta line is the route, with its distance and magnetic course; you can fly anywhere,
  there are no borders.

## Route and NAV

Choose a departure and an arrival airport (FLIGHT menu, map, or MCDU INIT FROM/TO with the
departure and arrival runways) and the flight plan gets a route the autopilot can fly:
- **Departure:** the runway, a climb on its track to 1500 ft above the field (`(1630)` at
  Tallinn), then direct to the airport's FRA departure point (Tallinn and Tartu have them, e.g.
  LONSA or GONOS).
- **En route:** real points of Estonia's free route airspace from the eAIP (ENR 4.4), the ones
  near the direct line, and the arrival airport's FRA arrival point (e.g. SULUN for Tallinn).
- **Approach:** generated as an Airbus database codes unnamed fixes: CF (course fix) and FF (final
  fix, where the glide path meets 2000 ft above the threshold) on the extended centreline, and
  RW at the threshold. Coming from beside or beyond the runway, a downwind (DW) and base (BS)
  point lead onto it. CF and FF carry the intercept altitude.
- **NAV:** armed for the takeoff (cyan on the FMA), it engages at 30 ft and flies the legs with
  fly-by turns. HDG pull leaves it, HDG push (Shift+U, Shift-click on HDG, or the panel's push)
  re-engages it, direct to the next fix when far from the route. With APPR armed, LOC and G/S
  capture from NAV.
- **On the displays:** the ND draws the legs still to fly in green with the TO waypoint in white,
  and its name, bearing and distance at the top right; the map draws the whole route. The HDG
  window shows dashes and the managed dot in NAV.
- **Vertical:** set the altitudes yourself (ALT, LVL/CH, V/S): managed climb and descent are not
  built. A route of 100 NM wants its descent from about 45 NM out.
- A circuit (the same airport) and a free flight to a town have no route.
- The points come from `tools/make_navdata.py`, which turns ENR 4.4 (AIRAC 2026-10-01) into
  `core/src/NavData.cpp`; run it again for a new AIRAC (`--airac`).

## Breakup and crash

- **Breakup:** in the air at 380 kt (indicated) or more, the airframe breaks in two just ahead
  of the wings. The nose section dives, the rest with the wings spins down; both trail fire and
  smoke and explode where they hit the ground. The flight model stops.
- **Crash:** a ground contact far too hard (1500 fpm or more), a wing tip or the nose first,
  or the fuselage on the ground (gear up): an explosion, fire, a smoke column and debris.
- The view stays as it was (C still switches). From the seat you fall with the nose section and
  the instruments keep showing how it falls: altitude, vertical speed, airspeed, attitude and
  heading, with the engines winding down. The outside camera follows without tumbling.
- A new flight (F11, or F5) puts everything back.

380 kt is the A320's design dive speed (VD), the highest it is shown to survive. Normal flight
stays below 350 kt (VMO, the overspeed warning); between the two the aircraft holds together. The
limit is `A320_BREAKUP_IAS_KT` in `core/include/a320/a320_api.h`.

## First flight

**Takeoff (F5).** You start lined up on runway 26 in CONF 1+F, with the parking brake set.
1. Press **N** to release the parking brake and **Home** for TOGA.
2. At **"ROTATE"** (VR, about 150 kt; V1, VR and V2 are under the speed tape), hold **Down
   arrow** to rotate to about 10°. Then hold 15° pitch.
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
- **DO** says which cockpit control to use and what to do with it. The control is outlined in
  yellow, with a line from the panel. Keyboard shortcuts are added in brackets, e.g.
  [Key: K].
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
  selected value, and Shift-click "pushes" it (hold the heading, level off, V/S 0);
- HDG V/S switches to TRK FPA: the same windows then select a track and a flight path angle.

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
      - G/S captures only when you reach the beam. At 3000 ft that is about 9 NM from the
        runway, so be on the localizer before then.
      - If you are higher, the glideslope diamond sits below the centre and ALT holds you
        above the beam. As in the real aircraft, keep G/S armed and descend with V/S −1500
        (**0**, then **7/8**) until G/S* captures. The tutor tells you when this happens.
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
  src/Airport.cpp            EETN and EEKE runway data, the world they share, runway-aligned frames
  src/Systems.cpp            flaps/1+F logic, thrust detents, VLS/VFE/VMAX, warnings, callouts
  src/Mcdu.cpp               MCDU pages and entries; the crew's data in Fms.h tunes the ILS
  src/Autopilot.cpp          FCU modes, autopilot guidance, autothrust, autoland
  src/Audio.cpp              cockpit sound synthesis and mixing (engines, airflow, chimes, callouts)
  tests/                     unit tests plus scripted JSBSim flights through the C API
unreal/A320Sim/              UE5 project (C++ only)
  Content/JSBSim/            JSBSim aircraft data (A320 model, CFM56 engine)
  Content/Sounds/            spoken callouts (generated by tools/make_callouts.py with eSpeak NG)
  Content/Terrain/           real scenery: imagery, heights, buildings (generated by tools/make_terrain.py)
  Source/A320Sim/            aircraft pawn, cockpit HUD, airport/world builder, input
scripts/, *.bat              Windows setup, run and package scripts
```

### Scenery

`tools/make_terrain.py` builds `Content/Terrain/` from open data. The output is in the
repository, so you only run the tool to change the area or the detail. It covers all of
Estonia, with the islands, in three layers (about 78 MB in all):
- **Detailed:** 80 × 80 km around EETN, 64 tiles of 10 km at about 10 m per pixel, and 65 more
  around Kuressaare (EEKE, 16), Tartu (EETU, 16), Pärnu (EEPU, 16), Kärdla (EEKA, 9), Ruhnu (EERU, 4)
  and Kihnu (EEKU, 4).
- **Region:** 841 tiles of 10 km at about 20 m per pixel, wherever there is land (57.45–59.85° N,
  21.6–28.3° E, files in `Region/`). Open sea is skipped.
- **Base:** one 570 km square at about 140 m per pixel, from southern Finland to Riga.
- **Buildings:** about 382,000 (`buildings.bin`, 11 MB): within 20 km of EETN, around the other
  airports and around every city and town of 4,000 people or more (23 of them, Narva to Valga).
- **Ground map:** `ground.txt` and `ground.i16` (3.7 MB), the terrain heights every 250 m over the
  region box, for the flight model (radio altimeter, crashes).
- **Airport layouts:** `Content/Airports/<ICAO>.txt`, the taxiways and aprons of all seven airports.

- **Imagery:** Sentinel-2 cloudless 2024 by EOX.
- **Elevation:** Mapzen Terrain Tiles on AWS.
- **Buildings and airport layouts:** OpenStreetMap, through the Overpass API.

To regenerate it:

```
pip install numpy pillow
python tools/make_terrain.py              # --inner-km, --tile-px, --buildings-km to change the area
python tools/make_terrain.py --region none                       # the airport areas only
python tools/make_terrain.py --airports none                     # only EETN gets detailed tiles
python tools/make_terrain.py --region 57.5,60,21.5,28.5 --region-px 1024   # another box, sharper
```

Other options: `--region-zoom`, `--region-grid`, `--region-quality`, `--outer-margin-km`,
`--outer-px` and `--outer-step-km` (see `--help`). The first run downloads about 9,000 tiles into
`build/terrain-cache`.
`--patch-km` and `--patch-buildings-km` override the per-airport radii of `AIRPORTS` in the tool;
`--town-population` and `--ground-m` set which towns get buildings and the ground map spacing.

The ground map and the airport layouts are plain files (the tool's docstring has the details):
```
ground.txt    version=1 / origin=<southM>|<westM> / spacing=<metres> / size=<rows>|<cols>, one a line
ground.i16    rows*cols int16 little-endian, decimetres above the EETN field (as the terrain),
              rows south to north, columns west to east, the first sample at the origin
Airports/<ICAO>.txt
              icao=<ICAO>
              level=<metres above the EETN field, the level the terrain is flattened to>
              taxiway=<ref or ->|<widthM>|n,e;n,e;...  centreline; OSM width, else a default
              apron=<ref or ->|n,e;n,e;...             simple closed outline, last point not repeated
```
Positions are metres north and east in the sim's frame, with one decimal; `#` starts a comment.

How the sim uses the data:
- **Placement:** every pixel and vertex goes through the same airport frame as the flight model,
  so the imaged runway lies under the modelled one.
- **Airport area:** the terrain around the runways of all seven airports is blended to their elevation.
- **Sea level:** the sea sits at sea level, about 40 m below the field.
- **Streaming:** only the base layer loads at start-up, with the tiles under the aircraft. In
  flight, detailed tiles load within 20 km (and unload beyond 28 km), region tiles within 45 km
  (beyond 60 km) and buildings within 15 km (beyond 20 km), nearest first. Images are decoded on
  worker threads, and at most two pieces are added per frame. A region tile stands in for a
  detailed one that is not loaded, so there is never a hole.
- **Layers:** the base layer sits at least 3 m below the finer tiles over it, so the two never
  flicker against each other. Where no finer tile is loaded, the base layer shows.
- **Buildings:** they are boxes fitted to their OpenStreetMap footprints. The height comes from the
  `height` or `building:levels` tags, or is estimated from the building type. Buildings mapped as
  multipolygons are skipped (for example the Ülemiste centre). In towns away from the detailed
  tiles, they stand on the region tiles' surface.
- **Material:** `Play.bat` runs the editor build, so the terrain gets a lit material that is
  built at start-up. A packaged build (`Package.bat`) shows the same imagery unlit.

The simulation runs at a fixed 120 Hz, decoupled from the frame rate. Pausing stops sim time
while rendering and the cockpit keep running.

### Map data

`tools/make_map.py` builds `Content/Map/`, the data of the world map, from the scenery imagery
(no new downloads) and OpenStreetMap. Run `make_terrain.py` first.
- **Tiles:** 512 px JPEGs in the same flat world as the terrain, north up, in three levels:
  `L0` 102.4 km (200 m per pixel, 3 × 5 tiles over 57.25–60.03° N, 20.6–29.2° E), `L1` 51.2 km
  and `L2` 25.6 km (50 m per pixel). Open sea is left out at `L1` and `L2`.
- **Style:** darkened satellite imagery and a dark blue sea, with the land border, coastline,
  lakes, rivers, roads and railways drawn in, more of them at the finer levels. No text: the game
  draws the names.
- **`map.txt`:** the grid origin, the levels, one line per tile file and the attribution.
- **`places.txt`:** cities, towns, villages (not hamlets), islands and lakes with their
  Estonian names, positions in metres and a rank for decluttering labels. No airports.

- **Map data:** © OpenStreetMap contributors (ODbL), through the Overpass API.

```
python tools/make_map.py                  # caches OpenStreetMap in build/terrain-cache/map
python tools/make_map.py --offline --levels 2 --quality 85
```

Other options: `--supersample`, `--no-places` and `--preview DIR` (see `--help`). A layer no
Overpass server delivers is left out and named in `map.txt`; a later run fills it in.

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
  runway, at Tallinn and at Kuressaare (its own ground elevation and ILS);
- ATC flights: a Tallinn circuit, Kuressaare to Tallinn (AFIS, FL090, descent, ILS 26) and a
  40 NM straight-in to Kuressaare, flown by a crew that reads back, tunes and squawks;
- the autopilot: OP CLB with ALT capture, a HDG turn, a V/S descent with capture, TRK in a
  40 kt crosswind, an FPA descent with capture, the HDG/ALT/V/S pushes, a route from the
  departure and arrival, NAV from 30 ft along Tallinn 08 to Tartu onto the ILS 26 (downwind and
  base), HDG push to NAV in the air, LOC
  intercept from a heading, instinctive disconnect, and a full autoland to a stop;
- the audio engine: WAV loading, engine level following thrust, pause, callouts, and the
  master warning and its acknowledgment;
- joystick mapping: axis scaling, deadzone, throttle detents, learn mode and the settings
  file (including older files);
- flaps and speedbrake levers: detent calibration (a backwards lever, the ARM position), and
  the button assignments and command list;
- the WingFlex FCU and EFIS Cube USB reports: buttons, knob clicks, lights, displays and
  negative V/S;
- throttle quadrants: detent calibration (including a backwards axis and reverse), devices
  kept by name, and automatic quadrant set-up;
- takeoff callouts: V-speeds from the weight and flaps, the four calls in order on a real
  takeoff, none after a rejected takeoff or while taxiing;
- cockpit systems: a cold-and-dark start (APU, starter, light-off, crossbleed, shutdown),
  ground spoilers, autobrake LO/MED deceleration and stopping distance, and the autobrake
  disarming when the pilot brakes;
- split thrust levers: each engine follows its own lever, reverse on one engine, and a lever
  in reverse in flight giving idle;
- the ILS autoland lesson, flown by a test pilot who does exactly what each step says from 20
  NM out to a stop: every step must tick off, the FMA must show LOC*, G/S* and G/S, and the
  touchdown must be in the touchdown zone below 600 fpm near VAPP;
- the 20 NM intercept scenario holding 3000 ft and 220 kt, the AP1/AP2 rules, and the sim
  tutor messages;
- speed protection: SPD 100 selected, A/THR holds VLS;
- alpha floor: full back stick at idle thrust triggers A.FLOOR (TOGA, alpha held below the
  stall), then TOGA LK until A/THR is disconnected.

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
- managed (FMS) speed and vertical modes, the published SIDs/STARs, DIR TO, and the flight
  directors;
- go-around (TOGA during an approach: SRS and GA TRK);
- weather radar and TCAS;
- trees as 3D objects (forests are in the imagery only), and buildings in the villages;
- a 3D clickable cockpit;
- wind and low visibility (the core takes a steady wind, `a320_set_wind`; only the tests use it);
- failures.

**Approximations:**
- ILS data is from the eAIP (see [MCDU](#mcdu-flight-computer)); the antenna positions are
  placed on the extended centreline.
- Magnetic variation is the nearest airport's (AIP, 2025: 7 to 12° E).
- The flight model's ground follows the terrain on a 250 m grid (runways at their exact level);
  hills and valleys smaller than that, buildings and trees are visual only.
- The world is flat (Tallinn's tangent plane, heights above sea level); at Kuressaare north is
  turned about 2° in it, which the aircraft, runways and map all follow.

**Licences:**
- JSBSim is LGPL-2.1. Its A320 model is marked "for educational and entertainment purposes
  only … not to be sold".
- Scenery data (`Content/Terrain/`):
  - Imagery: Sentinel-2 cloudless 2024 by EOX IT Services GmbH (https://s2maps.eu), which
    contains modified Copernicus Sentinel data 2024. Licence CC BY-NC-SA 4.0, non-commercial
    use only.
  - Elevation: Mapzen Terrain Tiles (AWS Open Data). Sources include SRTM and GMTED.
  - Buildings: © OpenStreetMap contributors, ODbL.
- "Airbus" and "A320" are trademarks of Airbus. This project is not affiliated with Airbus.

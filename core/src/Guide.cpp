#include "a320/Guide.h"

#include <cmath>

namespace a320 {
namespace {

bool onGlideslope(const A320State& s) {
  return s.vertMode == A320_VERT_GS_STAR || s.vertMode == A320_VERT_GS || s.vertMode == A320_VERT_LAND ||
         s.vertMode == A320_VERT_FLARE || s.latMode == A320_LAT_ROLLOUT;
}

bool locCaptured(const A320State& s) {
  return s.latMode == A320_LAT_LOC || s.latMode == A320_LAT_ROLLOUT || s.vertMode == A320_VERT_LAND ||
         s.vertMode == A320_VERT_FLARE;
}

bool landing(const A320State& s) {
  return s.vertMode == A320_VERT_LAND || s.vertMode == A320_VERT_FLARE || s.latMode == A320_LAT_ROLLOUT;
}

// ILS approach and autoland, from the intercept scenario on runway 26. Step indexes matter
// to ilsAlert below.
const GuideStep kIlsAutoland[] = {
    {"APPROACH PREPARATION", "Read the FMA",
     "Look at the top of the PFD: the Flight Mode Annunciator (FMA) shows what the autopilot is doing right now.",
     "SPEED (autothrust), ALT (altitude hold), HDG (heading), and AP1 and A/THR on the right.",
     "Always confirm a button press on the FMA. Green = engaged, blue = armed (waiting to engage), a white box = "
     "just changed.",
     "The MSFS 2024 A320neo has the same FMA in the same place.",
     {A320_GT_FMA}, true, nullptr},
    {"APPROACH PREPARATION", "Set the autobrake",
     "Click AUTO/BRK LO on the centre panel, below the gear lever (MED for a short or wet runway).",
     "The LO button lights, and AUTO BRK LO appears in blue in the E/WD memo.",
     "After touchdown the autobrake brakes to a fixed deceleration (LO 1.7 m/s2, MED 3 m/s2) as soon as the ground "
     "spoilers deploy.",
     "Same buttons in MSFS, to the right of the gear lever.",
     {A320_GT_AUTOBRAKE}, false,
     [](const A320State&, const A320Controls& c) { return c.autobrake == A320_AUTOBRAKE_LO || c.autobrake == A320_AUTOBRAKE_MED; }},
    {"APPROACH PREPARATION", "Show the ILS",
     "Press LS on the EFIS panel (left end of the FCU strip), and click the ND mode button until it shows ND LS "
     "(ROSE LS). [Key: L]",
     "PFD: magenta localizer diamond at the bottom, glideslope diamond on the right. ND: course pointer and "
     "deviation bar.",
     "LS only changes the display; the FMGC tunes the ILS of the arrival in the MCDU flight plan. A diamond shows "
     "where the beam is, so fly towards it.",
     "In MSFS check the ILS frequency and course on the MCDU RAD NAV page. LS and the ND mode knob are on the EFIS "
     "panel.",
     {A320_GT_EFIS_LS, A320_GT_EFIS_ND_MODE}, false,
     [](const A320State&, const A320Controls& c) { return c.efisLs != 0 && c.ndMode == A320_ND_ROSE_LS; }},
    {"APPROACH PREPARATION", "Check the ILS",
     "Open the MCDU (MCDU at the top, or Tab) and press RAD NAV: ILS ILK/109.30, CRS 260 for runway 26. Compare "
     "with the PFD (bottom left) and the ND, then press NEXT.",
     "The same ident ILK and course 260 on the MCDU, PFD and ND. The deviation bar sits to one side: you are not on "
     "the beam yet.",
     "Pilots verify the identifier, course and frequency before trusting an ILS. A different runway is chosen on the "
     "MCDU: F-PLN, the destination (LSK3L), ARRIVAL>, the approach, INSERT.",
     "Same pages in the MSFS A320neo MCDU (RAD NAV, F-PLN > ARRIVAL).",
     {A320_GT_MCDU, A320_GT_ND}, true, nullptr},
    {"INTERCEPT", "Slow down: SPD 180",
     "Turn the SPD knob on the FCU down to 180 kt: click - next to the SPD window. [Keys: 1, Shift = 10 kt]",
     "The blue speed target on the PFD speed tape moves to 180. A/THR reduces thrust, and the FMA still shows "
     "SPEED.",
     "This is selected speed: you set it with the knob. Managed speed (knob pushed) would follow the flight plan; "
     "this sim only has selected speed.",
     "In MSFS you usually activate the APPR phase on the MCDU PERF page and push the knob for managed speed.",
     {A320_GT_FCU_SPD, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) { return s.fcuSpdKt <= 185.0; }},
    {"INTERCEPT", "FLAPS 1",
     "Below 230 kt, drag the flaps lever on the pedestal to 1. [Key: V]",
     "The E/WD shows flaps 1, and the red-and-black VFE band on the speed tape moves down to 230 kt.",
     "Each flap position has a maximum speed (VFE). Extending early is not possible without overspeeding the "
     "flaps.",
     "Same lever on the pedestal in MSFS.",
     {A320_GT_FLAPS}, false,
     [](const A320State&, const A320Controls& c) { return c.flapsLever >= 1; }},
    {"INTERCEPT", "Arm the approach: APPR",
     "Press APPR on the FCU. [Key: K]",
     "FMA, second row: G/S and LOC in BLUE (armed). The APPR button lights up.",
     "Armed means waiting. The aircraft keeps HDG and ALT until the beams are reached: first LOC captures, then "
     "G/S. Arm it only on an intercept heading.",
     "Identical in MSFS. On a real flight you arm it once cleared for the ILS approach.",
     {A320_GT_FCU_APPR, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return (s.armed & A320_ARMED_GS) != 0 || onGlideslope(s); }},
    {"INTERCEPT", "Engage AP2",
     "Press AP2 on the FCU. With APPR armed both autopilots can be engaged. [Keys: Shift+A]",
     "FMA, right column: AP1+2 and CAT 3 DUAL.",
     "Two autopilots monitor each other, which a CAT 3 autoland in fog requires. Outside LOC or APPR only one "
     "autopilot can be engaged.",
     "Same in MSFS: press AP2 after APPR for a CAT 3 DUAL autoland.",
     {A320_GT_FCU_AP2, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return s.ap1Engaged && s.ap2Engaged; }},
    {"INTERCEPT", "Localizer capture",
     "Wait. The aircraft keeps its heading until the localizer diamond moves towards the centre.",
     "FMA: LOC* (green, boxed) while it turns onto the course, then LOC. On the ND the deviation bar centres.",
     "LOC* = capturing, LOC = tracking the runway centreline. The HDG mode ends by itself.",
     "Same annunciations in MSFS.",
     {A320_GT_FMA, A320_GT_ND}, false,
     [](const A320State& s, const A320Controls&) { return locCaptured(s); }},
    {"INTERCEPT", "FLAPS 2, SPD 160",
     "Below 200 kt drag the flaps lever to 2, then turn the SPD knob down to 160 kt. [Keys: V, 1]",
     "E/WD flaps 2. The speed target moves to 160.",
     "Slowing down in steps keeps the speed below each flap limit while the aircraft stays stable.",
     "In MSFS with managed speed the target follows the flaps (F and S speeds) by itself.",
     {A320_GT_FLAPS, A320_GT_FCU_SPD}, false,
     [](const A320State& s, const A320Controls& c) { return c.flapsLever >= 2 && s.fcuSpdKt <= 165.0; }},
    {"FINAL APPROACH", "Glideslope capture",
     "Wait for the glideslope diamond on the right of the PFD to come down to the centre. Level at 3000 ft you "
     "meet the beam from below, which is normal.",
     "FMA: G/S* (boxed), then G/S. The aircraft starts down at about 700-800 ft/min.",
     "G/S* = capturing, G/S = tracking the 3 degree glidepath. The ALT mode ends by itself. Intercept below the beam: "
     "at 3000 ft it is about 9 NM from the runway, so be on the localizer before that. If the diamond is already "
     "below the centre you are high: G/S stays armed, descend with V/S -1500 until it captures.",
     "Same in MSFS.",
     {A320_GT_FMA, A320_GT_PFD_ILS}, false,
     [](const A320State& s, const A320Controls&) { return onGlideslope(s); }},
    {"FINAL APPROACH", "Gear down",
     "Click the gear lever on the centre panel to put it DOWN. [Key: G]",
     "The gear indications turn green; LDG MEMO on the E/WD shows GEAR DN in green.",
     "The gear adds a lot of drag, so it goes down once you start descending on the glideslope.",
     "Same lever in MSFS, on the centre panel.",
     {A320_GT_GEAR}, false,
     [](const A320State&, const A320Controls& c) { return c.gearDown != 0; }},
    {"FINAL APPROACH", "FLAPS 3, then FULL",
     "Below 185 kt drag the flaps lever to 3, then below 177 kt to FULL. [Key: V]",
     "E/WD: FLAPS FULL. The VLS (amber band at the bottom of the speed tape) moves down.",
     "FULL is the normal landing setting: lowest approach speed and good view over the nose.",
     "Same in MSFS.",
     {A320_GT_FLAPS}, false,
     [](const A320State&, const A320Controls& c) { return c.flapsLever == 4; }},
    {"FINAL APPROACH", "Approach speed",
     "Once the flaps are fully out, turn the SPD knob to VAPP: the top of the amber VLS band plus 5 kt (about "
     "135-150 kt). [Key: 1]",
     "A/THR slows down and holds the speed just above the amber band.",
     "VLS is the lowest selectable speed (1.23 x stall speed). VAPP adds a margin for gusts and autothrust.",
     "In MSFS, managed speed flies VAPP from the MCDU PERF APPR page automatically.",
     {A320_GT_FCU_SPD, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) {
       // VLS keeps dropping until the flaps stop moving, so judge VAPP on the final value.
       return s.flapDeg > 34.0 && s.fcuSpdKt <= s.vlsKt + 8.0 && s.fcuSpdKt >= s.vlsKt;
     }},
    {"FINAL APPROACH", "Arm the ground spoilers",
     "Arm the speedbrake lever: click ARM on the speedbrake lever (pedestal).",
     "E/WD memo: GND SPLRS ARMED (blue).",
     "Armed spoilers deploy by themselves at touchdown. They dump lift so the weight goes on the wheels, which "
     "also starts the autobrake.",
     "In MSFS: speedbrake lever up to ARM (Shift+/ by default).",
     {A320_GT_SPOILERS}, false,
     [](const A320State&, const A320Controls& c) { return c.spoilersArmed != 0; }},
    {"FINAL APPROACH", "Landing memo",
     "Check the LDG MEMO on the E/WD: every line green (gear down, spoilers armed, flaps full). Then press NEXT.",
     "Blue lines are still to do; they turn green when done.",
     "This is the landing checklist the A320 shows by itself below 2000 ft.",
     "Same LDG MEMO in MSFS.",
     {A320_GT_EWD}, true, nullptr},
    {"LANDING", "LAND mode",
     "Watch the FMA at 400 ft radio altitude.",
     "FMA: LAND (green, across the vertical and lateral columns).",
     "From LAND the autoland can no longer be disarmed; only disconnecting the AP or a go-around ends it.",
     "Same in MSFS.",
     {A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return landing(s); }},
    {"LANDING", "Flare and RETARD",
     "At about 40 ft the aircraft flares by itself. When you hear \"RETARD\", drag the thrust levers on the pedestal back to the IDLE detent. [Key: End]",
     "FMA: FLARE, and RETARD in the thrust column.",
     "Airbus autothrust does not move the levers: you retard them. After touchdown, levers at IDLE also disconnect "
     "A/THR.",
     "Identical in MSFS: pull the levers to idle at RETARD.",
     {A320_GT_THRUST_LEVERS, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls& c) {
       return c.thrustLever < 0.05 && !c.reverse && (s.onGround || s.vertMode == A320_VERT_FLARE);
     }},
    {"LANDING", "Touchdown",
     "Hands off: the autopilot puts the main wheels down and keeps the centreline. Just watch.",
     "FMA: ROLL OUT. E/WD: GND SPLRS (the spoilers are up), and the autobrake starts braking (DECEL lights).",
     "The armed spoilers deploy at touchdown and dump lift; that puts the weight on the wheels and starts the "
     "autobrake. ROLL OUT steers along the localizer.",
     "Same in MSFS. In a manual landing you would fly the flare yourself from about 30 ft.",
     {A320_GT_FMA, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls&) { return s.onGround != 0; }},
    {"LANDING", "Full reverse",
     "Select full reverse: drag the thrust levers down past IDLE into the red REV zone on the pedestal (on a throttle "
     "quadrant: pull the levers behind IDLE, fully into reverse). [Keys: R, then hold PgUp]",
     "E/WD: REV in green on both engines, N1 rises. The deceleration increases.",
     "Reversers turn the engine exhaust forward. They deploy only on the ground, and only with the levers in the "
     "reverse range.",
     "In MSFS: hold F2 (or the reverse range of your throttle) for reverse thrust.",
     {A320_GT_THRUST_LEVERS, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls& c) {
       return (s.onGround && s.reverse && c.thrustLever > 0.6) || (s.onGround && s.groundSpeedKt < 70.0);
     }},
    {"LANDING", "70 kt: reverse idle, then stow",
     "At 70 kt bring the thrust levers up to reverse idle, then out of reverse to the IDLE detent (on a quadrant: "
     "levers forward to IDLE). [Keys: hold PgDn, then R]",
     "E/WD: REV disappears. The engines spool down to idle.",
     "Reversers lose effect at low speed and can blow debris into the engines, so they are back at idle by 70 kt.",
     "In MSFS: release F2 and bring the levers to IDLE.",
     {A320_GT_THRUST_LEVERS}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.onGround && !c.reverse && c.thrustLever < 0.05 && s.groundSpeedKt < 80.0;
     }},
    {"LANDING", "Brake to a stop",
     "Press the brake pedals (toe brakes) down to a stop on the centreline. Then set the PARK BRK switch on the "
     "pedestal to ON, and press AP1 / AP2 on the FCU to disconnect the autopilot. [Keys: hold B, then N]",
     "The speed reads 0, PARK BRK appears on the E/WD memo.",
     "Pressing the brakes disconnects the autobrake: from then on you brake yourself. That is normal and expected.",
     "In MSFS: brakes are the . key (or toe brakes), parking brake Ctrl+. ; the autobrake disconnects the same way.",
     {A320_GT_PARK_BRAKE, A320_GT_FCU_AP1, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.onGround && s.groundSpeedKt < 2.0 && c.parkBrake && !c.reverse;
     }},
};

constexpr int kIlsStepCount = static_cast<int>(sizeof(kIlsAutoland) / sizeof(kIlsAutoland[0]));
constexpr int kStepTouchdown = 18;  // "Reversers": before it the aircraft must be flying

const char* ilsAlert(const A320State& s, const A320Controls& c, int step) {
  const bool flying = !s.onGround;
  if (flying && !s.apEngaged && step < kStepTouchdown)
    return "The autopilot is off (a firm sidestick input disconnects it). Press AP1 on the FCU to engage it again.";
  if (flying && s.iasKt > s.vmaxKt + 3.0)
    return "Too fast for this flap setting: turn the SPD knob down.";
  if (flying && onGlideslope(s) && !c.gearDown && s.radioAltFt < 1500.0) return "The gear is still up: gear lever DOWN. [Key: G]";
  if (flying && s.athrEngaged && !s.athrActive && c.thrustLever > 0.05)
    return "A/THR is armed but not active: put the thrust levers in the CL detent. [Key: Ins]";
  if (s.onGround && s.reverse && s.groundSpeedKt < 60.0)
    return "Below 70 kt: thrust levers to reverse idle, then out of reverse to IDLE. [Keys: PgDn, R]";
  if (s.onGround && !s.reverse && c.thrustLever > 0.1 && step >= kStepTouchdown)
    return "Thrust levers to the IDLE detent: forward thrust on the runway works against the brakes. [Key: End]";
  return nullptr;
}

bool fcuAltIs(const A320State& s, int ft) { return std::fabs(s.fcuAltFt - ft) < 1.0; }

// A full IFR flight with ATC from runway 26: the radio panel, the ATIS, the clearance and its
// readback, the transponder, the handovers, radar vectors and the ILS clearance. Step indexes
// matter to radioAlert below.
const GuideStep kRadioFlight[] = {
    {"BEFORE START", "The radio panel",
     "Open the RADIO window (RADIO at the top, or F10). COM 1 shows ACTIVE 135.905 (Tallinn Tower) and STBY 124.880 "
     "(the ATIS). Then press NEXT.",
     "ACTIVE is what you hear and transmit on. STBY is the one you prepare with the knobs.",
     "On the A320's radio management panel (RMP) you set the next frequency in STBY and swap it in with the transfer "
     "key, so you can always swap back if nobody answers.",
     "Same RMP on the pedestal in MSFS; the ATC window there is the reply list here.",
     {A320_GT_RADIO}, true, nullptr},
    {"BEFORE START", "Listen to the ATIS",
     "Press the transfer key (<->) so 124.880 is ACTIVE, and listen to Tallinn Information to the end.",
     "The ATIS repeats: information letter, time, runway in use, wind, visibility, temperature, QNH.",
     "The ATIS saves the controller from saying the weather to every aircraft. You confirm you have it with its "
     "letter on first contact (\"with information K\").",
     "In MSFS: tune the ATIS frequency on COM 1 or pick ATIS in the ATC window.",
     {A320_GT_RADIO}, false,
     [](const A320State&, const A320Controls& c) { return c.com1ActiveKhz == 124880; }},
    {"BEFORE START", "Note the ATIS",
     "Note the information letter, runway in use 26 and QNH 1013, then press NEXT.",
     "The letter changes every half hour (time hh20 and hh50).",
     "QNH sets the altimeter to show altitude above sea level near the airport; below the transition altitude "
     "(5000 ft at Tallinn) all altitudes are on QNH.",
     "The same in MSFS (the ATIS text also shows in the ATC window).",
     {A320_GT_RADIO}, true, nullptr},
    {"CLEARANCE", "Back to Tower",
     "Press the transfer key again: 135.905 Tallinn Tower is ACTIVE.",
     "COM 1 ACTIVE 135.905, and the window shows TALLINN TOWER.",
     "At Tallinn the Tower also gives IFR clearances; there is no separate delivery frequency (AIP EETN AD 2.20).",
     "In MSFS the ATC window switches to Tower when you tune it.",
     {A320_GT_RADIO}, false,
     [](const A320State&, const A320Controls& c) { return c.com1ActiveKhz == 135905; }},
    {"CLEARANCE", "Request the IFR clearance",
     "In the reply list, pick \"request IFR clearance to Tallinn\" (click it, or its number key while the RADIO "
     "window is open).",
     "Your call is logged in blue; a few seconds later Tower answers with the clearance.",
     "Every call starts with who you call and who you are: \"Tallinn Tower, SIM320\". Set a flight number in the "
     "MCDU INIT page to fly as that callsign.",
     "MSFS: \"Request IFR clearance\" in the ATC window.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcAwaitingReadback || s.atcIfrCleared; }},
    {"CLEARANCE", "Read back the clearance",
     "Listen to the clearance, then pick the readback that repeats it exactly: route, altitude, squawk, callsign.",
     "Tower answers \"readback correct, report ready for departure\". A wrong readback gets \"negative, I say again\".",
     "The readback is how ATC knows you heard right; altitudes, headings, frequencies, squawks and runways are always "
     "read back.",
     "The MSFS ATC window always offers the correct readback; here you have to find it.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcIfrCleared != 0; }},
    {"CLEARANCE", "Squawk and altitude",
     "Type the squawk on the transponder keypad in the RADIO window and set the mode to AUTO. Set the cleared "
     "altitude, 4000, in the FCU ALT window. [Keys: 5/6, Shift = 1000 ft]",
     "XPDR shows your code; FCU ALT 4000.",
     "The squawk identifies you on radar. AUTO makes the transponder reply once airborne.",
     "In MSFS the transponder is on the pedestal (ATC/TCAS panel); the FCU is the same.",
     {A320_GT_RADIO, A320_GT_FCU_ALT}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.atcSquawk > 0 && c.xpdrCode == s.atcSquawk && c.xpdrMode != A320_XPDR_STBY && fcuAltIs(s, 4000);
     }},
    {"DEPARTURE", "Ready for departure",
     "Pick \"ready for departure runway 26\", then read back the takeoff clearance with the runway.",
     "Tower: \"wind calm, runway 26, cleared for takeoff\".",
     "Never take off without hearing \"cleared for takeoff\" and reading back the runway: runway confusion is a "
     "classic accident cause.",
     "MSFS: \"Ready for departure\" in the ATC window.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcTakeoffCleared != 0; }},
    {"DEPARTURE", "Take off",
     "Release the parking brake [N], thrust levers to FLX/MCT [Del] or TOGA [Home], rotate at VR [Down arrow], gear "
     "up at positive climb [G].",
     "ONE HUNDRED KNOTS, V ONE, ROTATE, POSITIVE CLIMB.",
     "Fly runway heading, as cleared, and climb to 4000 ft.",
     "Same in MSFS.",
     {A320_GT_PARK_BRAKE, A320_GT_THRUST_LEVERS, A320_GT_GEAR}, false,
     [](const A320State& s, const A320Controls& c) { return !s.onGround && s.radioAltFt > 300.0 && !c.gearDown; }},
    {"DEPARTURE", "Autopilot",
     "Engage AP1 [A]. At 1500 ft put the thrust levers in CL [Ins] and press A/THR if it isn't on [T].",
     "FMA: AP1 and A/THR; the aircraft climbs to 4000 ft.",
     "With the autopilot flying you can work the radio.",
     "Same in MSFS.",
     {A320_GT_FCU_AP1, A320_GT_THRUST_LEVERS}, false,
     [](const A320State& s, const A320Controls&) { return s.apEngaged != 0; }},
    {"DEPARTURE", "Contact Radar",
     "When Tower says \"contact Tallinn Radar 127.905\", read it back, then set 127.905 in STBY with the knobs and "
     "press the transfer key.",
     "COM 1 ACTIVE 127.905 TALLINN RADAR.",
     "A handover always includes the frequency, and you read it back so a wrong frequency is caught.",
     "MSFS tunes it for you if you let it; here you tune it yourself.",
     {A320_GT_RADIO, A320_GT_ATC_REPLY}, false,
     [](const A320State&, const A320Controls& c) { return c.com1ActiveKhz == 127905; }},
    {"DEPARTURE", "Check in with Radar",
     "Pick the check-in: \"Tallinn Radar, SIM320, passing ... climbing altitude 4000 feet, heading 260\".",
     "Radar: \"radar contact\" and your first heading. Without the right squawk in AUTO, Radar asks for it first.",
     "On first contact you tell the controller your level (passing and cleared) so the radar picture can be checked.",
     "MSFS checks you in automatically after you tune.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcRadarContact != 0; }},
    {"VECTORS", "Fly the heading",
     "Read back the heading (\"Right heading 070\"), set it in the FCU HDG window and pull the knob. [Keys: 3/4, "
     "Shift = 10 degrees, U]",
     "FMA: HDG; the ND heading bug on the assigned heading, the aircraft turning in the direction ATC said.",
     "Radar vectors take you around to the final approach. Turn the way ATC says, even if the other way looks shorter.",
     "Same FCU in MSFS.",
     {A320_GT_FCU_HDG, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) {
       return s.atcHeadingMag > 0 && std::fabs(std::remainder(s.fcuHdgMagDeg - s.atcHeadingMag, 360.0)) < 3.0 &&
              s.latMode == A320_LAT_HDG;
     }},
    {"VECTORS", "Descend to 3000",
     "When Radar clears you to 3000 ft, read it back with the QNH, set 3000 in the FCU ALT window and pull it "
     "(open descent) [Keys: 5, 9].",
     "FMA: OP DES, then ALT at 3000 ft.",
     "Descend only to the cleared altitude: going below it is a level bust.",
     "Same in MSFS.",
     {A320_GT_FCU_ALT, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcClearedAltFt == 3000 && fcuAltIs(s, 3000); }},
    {"VECTORS", "Cleared for the ILS",
     "Keep reading back and flying the headings. On \"cleared ILS approach runway 26\": read it back, set the "
     "heading, press APPR [K] and AP2 [Shift+A], and set SPD 180 if told.",
     "FMA: LOC and G/S in blue (armed), then LOC* and G/S*.",
     "The approach clearance lets the autopilot capture the localizer by itself; before it you may only fly the "
     "headings ATC gives.",
     "Same in MSFS.",
     {A320_GT_FCU_APPR, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) {
       return s.atcApproachCleared && ((s.armed & A320_ARMED_GS) || s.vertMode == A320_VERT_GS_STAR ||
                                       s.vertMode == A320_VERT_GS || s.latMode == A320_LAT_LOC);
     }},
    {"APPROACH", "Contact Tower",
     "On \"contact Tallinn Tower 135.905\": read it back, set 135.905 and transfer, then check in: \"established ILS "
     "runway 26\".",
     "Tower: \"wind calm, runway 26, cleared to land\".",
     "Radar hands you to Tower once you are on the localizer; Tower owns the runway.",
     "Same in MSFS.",
     {A320_GT_RADIO, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls& c) { return c.com1ActiveKhz == 135905 && s.atcPhase >= A320_ATC_PHASE_TOWER; }},
    {"APPROACH", "Landing clearance",
     "Read back \"cleared to land runway 26\".",
     "The landing clearance is logged; without it you go around at 500 ft at the latest.",
     "Like the takeoff clearance, the landing clearance is read back with the runway.",
     "Same in MSFS.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcLandingCleared != 0; }},
    {"APPROACH", "Configure and land",
     "Flaps 2 and SPD 160 after LOC capture, gear down and flaps 3 at G/S capture, flaps FULL and VAPP, spoilers "
     "armed, autobrake LO; the autoland flares, RETARD: levers to idle [End].",
     "LAND, FLARE, ROLL OUT; reverse after touchdown [R], brake below 70 kt.",
     "The same configuration as the ILS lesson (F3).",
     "Same in MSFS.",
     {A320_GT_FLAPS, A320_GT_GEAR, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return s.onGround && s.atcTakeoffCleared && s.groundSpeedKt < 60.0 &&
                                                           s.atcPhase >= A320_ATC_PHASE_TOWER; }},
    {"AFTER LANDING", "Vacate",
     "Slow down, then read back \"vacate the runway when able, contact Tallinn Handling 131.905\".",
     "The flight is closed with Tower.",
     "Vacating quickly frees the runway for the next aircraft; Handling then guides you to the stand.",
     "Same in MSFS.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcPhase == A320_ATC_PHASE_DONE && !s.atcAwaitingReadback; }},
};

constexpr int kRadioStepCount = static_cast<int>(sizeof(kRadioFlight) / sizeof(kRadioFlight[0]));

const char* radioAlert(const A320State& s, const A320Controls& c, int) {
  if (!s.atcEnabled) return "ATC is switched off: switch it on in the RADIO window.";
  if (s.atcAwaitingReadback) return "ATC is waiting for your readback: pick it in the RADIO window.";
  if (s.onGround && s.groundSpeedKt > 30.0 && !s.atcTakeoffCleared && c.thrustLever > 0.5)
    return "No takeoff clearance yet: thrust levers to IDLE and brake.";
  if (!s.onGround && s.atcApproachCleared && !s.atcLandingCleared && s.radioAltFt < 1000.0)
    return "No landing clearance yet: contact Tower on 135.905.";
  return nullptr;
}

// A takeoff from cold and dark on runway 26: cockpit preparation, engine start, the MCDU, the
// ATC clearance, the takeoff and the climb out to radar contact. Step indexes matter to
// takeoffAlert below.
bool lightsOn(const A320Controls& c, int bits) { return (c.lights & bits) == bits; }

const GuideStep kTakeoff[] = {
    {"COCKPIT PREPARATION", "Start the APU",
     "Open the overhead panel (OVERHEAD at the top, or O). Press APU MASTER SW, then START, and wait for AVAIL.",
     "APU N rises to 100 %; the START button shows AVAIL in green.",
     "The APU (auxiliary power unit) gives electrical power and bleed air for the engine start without ground "
     "equipment.",
     "MSFS: the same buttons on the overhead, APU section.",
     {A320_GT_OVERHEAD}, false,
     [](const A320State& s, const A320Controls&) { return s.apuAvail != 0; }},
    {"COCKPIT PREPARATION", "APU bleed, beacon and seat belts",
     "On the overhead: APU BLEED ON, BEACON ON, SEAT BELTS ON.",
     "APU BLEED shows ON in blue; the red beacon flashes on the outside view [C].",
     "Bleed air spins the engine starters. The beacon tells the ground crew the engines are about to start.",
     "Same in MSFS.",
     {A320_GT_OVERHEAD}, false,
     [](const A320State&, const A320Controls& c) {
       return c.apuBleed && lightsOn(c, A320_LT_BEACON) && (c.signs & A320_SIGN_SEATBELTS);
     }},
    {"MCDU", "INIT: flight number",
     "Open the MCDU (MCDU at the top, or Tab), press INIT, type a flight number (e.g. EST123) and press the key next "
     "to FLT NBR (LSK 3L).",
     "FLT NBR shows your number in blue. It is your radio callsign from now on.",
     "The FMS is set up in a fixed order: INIT (who and where), F-PLN (route), RAD NAV, PERF (speeds and thrust).",
     "MSFS: INIT A page, same LSK.",
     {A320_GT_MCDU}, false,
     [](const A320State& s, const A320Controls&) { return s.fmsFlightNumberSet != 0; }},
    {"MCDU", "F-PLN: departure runway",
     "Press F-PLN: the first line is EETN26, the departure runway. Then press NEXT.",
     "EETN26 in green at the top of the flight plan.",
     "The departure runway is what PERF TAKE OFF and the ILS tuning use. Another one: LSK 1L, <DEPARTURE.",
     "MSFS: F-PLN, LSK 1L > DEPARTURE to choose the runway and SID.",
     {A320_GT_MCDU}, true, nullptr},
    {"MCDU", "PERF TAKE OFF: V-speeds",
     "Press PERF. Enter V1, VR and V2: type each speed and press its LSK (1L, 2L, 3L), or press the LSK with an empty "
     "scratchpad to take the computed value shown next to the label.",
     "V1, VR, V2 in blue. On the PFD speed tape a blue 1 marks V1.",
     "V1 is the decision speed: below it you can stop on the runway, above it you continue the takeoff even with an "
     "engine failure. VR is where you rotate, V2 the safe climb speed with one engine.",
     "MSFS: the EFB or simBrief gives the speeds; you type them on the same page.",
     {A320_GT_MCDU, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) { return s.vSpeedsEntered != 0; }},
    {"MCDU", "PERF TAKE OFF: flaps and FLEX",
     "Type 1/UP0.0 and press LSK 3R (FLAPS/THS), then a FLEX temperature such as 50 and LSK 4R.",
     "FLAPS/THS 1/UP0.0 and FLEX TO TEMP 50 in blue.",
     "FLEX tells the engines to assume a hotter day so they give only the thrust needed, which saves wear. (In this "
     "sim the FLX detent always gives MCT.)",
     "Same in MSFS.",
     {A320_GT_MCDU}, false,
     [](const A320State& s, const A320Controls&) { return s.fmsFlapsThsSet && s.fmsFlexTempC > -100; }},
    {"CLEARANCE", "ATIS",
     "Open the RADIO window (RADIO at the top, or F10), press <-> so 124.880 is ACTIVE, and listen to the ATIS.",
     "Tallinn Information: runway 26, wind, QNH 1013, information letter.",
     "You'll quote the letter to Tower so it knows you have the current weather.",
     "Same frequency in MSFS.",
     {A320_GT_RADIO}, false,
     [](const A320State&, const A320Controls& c) { return c.com1ActiveKhz == 124880; }},
    {"CLEARANCE", "IFR clearance",
     "Press <-> back to 135.905 (Tower), pick \"request IFR clearance\" and then the correct readback.",
     "Tower: \"readback correct, report ready for departure\".",
     "The clearance is your route and limit: runway heading, climb 4000 ft, and a squawk code.",
     "MSFS: \"Request IFR clearance\" in the ATC window.",
     {A320_GT_RADIO, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcIfrCleared != 0; }},
    {"CLEARANCE", "Squawk and FCU altitude",
     "Type the squawk on the transponder keypad (RADIO window) and select AUTO. Set 4000 in the FCU ALT window "
     "[Keys: 5/6, Shift = 1000 ft].",
     "XPDR shows your code; FCU ALT 4000.",
     "The FCU altitude is the clearance limit: the aircraft will never climb through it by itself.",
     "Same in MSFS.",
     {A320_GT_RADIO, A320_GT_FCU_ALT}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.atcSquawk > 0 && c.xpdrCode == s.atcSquawk && c.xpdrMode != A320_XPDR_STBY && std::fabs(s.fcuAltFt - 4000.0) < 1.0;
     }},
    {"ENGINE START", "ENG MODE IGN/START",
     "On the pedestal, turn ENG MODE to IGN/START.",
     "The engine page shows the start valves and ignition ready.",
     "IGN/START arms the automatic start: the FADEC opens the start valve, adds fuel and ignition at the right N2.",
     "Same selector in MSFS.",
     {A320_GT_ENGINES}, false,
     [](const A320State&, const A320Controls& c) { return c.engMode == A320_ENG_MODE_IGN_START; }},
    {"ENGINE START", "Start engine 2",
     "ENG 2 MASTER ON. Wait until N1 and N2 settle at idle.",
     "N2 rises, fuel flow appears at about 20 % N2, then N1 about 20 % and N2 about 60 % at idle.",
     "Engine 2 is started first: it powers the yellow hydraulic system, which the parking brake uses.",
     "MSFS: ENG 2 master switch on the pedestal.",
     {A320_GT_ENGINES, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls&) { return s.engRunning[1] != 0; }},
    {"ENGINE START", "Start engine 1",
     "ENG 1 MASTER ON and wait for idle.",
     "Both engines at idle on the E/WD.",
     "One engine at a time: the APU has bleed air for one start.",
     "Same in MSFS.",
     {A320_GT_ENGINES, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls&) { return s.engRunning[0] != 0; }},
    {"ENGINE START", "After start",
     "ENG MODE back to NORM, APU BLEED OFF (overhead).",
     "ENG MODE NORM; the engines supply the bleed air now.",
     "Leaving IGN/START would keep the igniters on. The APU keeps running for electrical power until after takeoff.",
     "Same in MSFS.",
     {A320_GT_ENGINES, A320_GT_OVERHEAD}, false,
     [](const A320State&, const A320Controls& c) { return c.engMode == A320_ENG_MODE_NORM && !c.apuBleed; }},
    {"BEFORE TAKEOFF", "Flaps 1+F",
     "Flaps lever to 1 [V]. On the ground this gives CONF 1+F.",
     "E/WD: flaps 1+F.",
     "Takeoff flaps add lift for a shorter takeoff roll. CONF 1+F is the usual setting on long runways.",
     "Same lever in MSFS.",
     {A320_GT_FLAPS}, false,
     [](const A320State&, const A320Controls& c) { return c.flapsLever == 1; }},
    {"BEFORE TAKEOFF", "Spoilers and autobrake",
     "Pull the speedbrake lever up to ARM and press AUTO/BRK MAX.",
     "GND SPLRS ARM and AUTO BRK MAX in the E/WD memo.",
     "If you reject the takeoff, the ground spoilers deploy and the autobrake brakes at full power by themselves.",
     "Same in MSFS.",
     {A320_GT_SPOILERS, A320_GT_AUTOBRAKE}, false,
     [](const A320State&, const A320Controls& c) { return c.spoilersArmed && c.autobrake == A320_AUTOBRAKE_MAX; }},
    {"BEFORE TAKEOFF", "Takeoff lights",
     "Overhead: STROBE ON, LAND ON, NOSE T.O.",
     "The runway ahead lights up.",
     "Strobes and landing lights make you visible on the runway. Turn them on when entering the runway.",
     "Same in MSFS.",
     {A320_GT_OVERHEAD}, false,
     [](const A320State&, const A320Controls& c) { return lightsOn(c, A320_LT_STROBE | A320_LT_LANDING | A320_LT_TAKEOFF); }},
    {"BEFORE TAKEOFF", "Takeoff clearance",
     "In the RADIO window pick \"ready for departure runway 26\", then read back \"cleared for takeoff runway 26\".",
     "Tower: \"wind calm, runway 26, cleared for takeoff\".",
     "No takeoff without hearing and reading back the clearance with the runway.",
     "MSFS: \"Ready for departure\" in the ATC window.",
     {A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcTakeoffCleared != 0; }},
    {"TAKEOFF", "Brakes off, takeoff thrust",
     "Parking brake OFF [N], thrust levers to FLX/MCT [Del] (or TOGA [Home]).",
     "FMA: MAN FLX (or MAN TOGA); the aircraft accelerates.",
     "On the A320 you set takeoff thrust with the levers in a detent; A/THR takes over later in the CL detent.",
     "Same in MSFS.",
     {A320_GT_PARK_BRAKE, A320_GT_THRUST_LEVERS}, false,
     [](const A320State& s, const A320Controls& c) { return !c.parkBrake && s.thrustDetent >= 2 && s.groundSpeedKt > 20.0; }},
    {"TAKEOFF", "V1, rotate",
     "Keep the centreline with the rudder [Q/E]. At ROTATE, pull back gently [Down arrow] to about 15 degrees nose up.",
     "Callouts: ONE HUNDRED KNOTS, V ONE (hands off the thrust levers: no stopping now), ROTATE.",
     "A smooth rotation of about 3 degrees per second avoids a tail strike.",
     "Same in MSFS.",
     {A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) { return !s.onGround && s.radioAltFt > 30.0; }},
    {"TAKEOFF", "Positive climb, gear up",
     "At POSITIVE CLIMB: gear lever UP [G].",
     "The gear doors close; the green gear indications go out.",
     "The gear makes a lot of drag: retract it as soon as the aircraft climbs for sure.",
     "Same in MSFS.",
     {A320_GT_GEAR}, false,
     [](const A320State& s, const A320Controls& c) { return !s.onGround && !c.gearDown; }},
    {"CLIMB", "Autopilot",
     "Above 100 ft press AP1 [A].",
     "FMA: AP1 on the right; the aircraft keeps climbing on runway heading.",
     "With the autopilot on you can work the radio and the systems.",
     "Same in MSFS.",
     {A320_GT_FCU_AP1, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return s.apEngaged != 0; }},
    {"CLIMB", "Thrust reduction",
     "At 1500 ft (THR RED on the PERF TAKE OFF page): thrust levers back to CL [Ins], press A/THR if it isn't on [T].",
     "FMA: THR CLB and A/THR in white (active).",
     "From here autothrust manages the thrust; the levers stay in CL until landing.",
     "Same in MSFS, where LVR CLB flashes on the FMA at the thrust reduction altitude.",
     {A320_GT_THRUST_LEVERS, A320_GT_FCU_ATHR}, false,
     [](const A320State& s, const A320Controls&) { return !s.onGround && s.athrActive && s.thrustDetent == 1; }},
    {"CLIMB", "Flaps up",
     "Above the F speed (F= on the MCDU PERF TAKE OFF page) and the S speed, flaps 0 [F].",
     "E/WD flaps 0; the amber VLS band on the speed tape moves up to the clean value.",
     "Retracting below F (or S) speed would get close to the stall; above VFE would overstress the flaps.",
     "Same in MSFS.",
     {A320_GT_FLAPS, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls& c) { return !s.onGround && c.flapsLever == 0; }},
    {"CLIMB", "Contact Radar",
     "When Tower hands you over, read back, tune 127.905 and check in. Radar answers \"radar contact\" if your squawk "
     "is right.",
     "Radar contact and a first heading. Fly it: FCU HDG and pull [3/4, U].",
     "Departure control now keeps you separated from other traffic; you fly its headings and altitudes.",
     "MSFS hands you over in the ATC window.",
     {A320_GT_RADIO, A320_GT_ATC_REPLY}, false,
     [](const A320State& s, const A320Controls&) { return s.atcRadarContact != 0; }},
    {"CLIMB", "After takeoff",
     "Spoilers DISARM (lever down), APU MASTER OFF, NOSE light OFF and LAND OFF above 10000 ft (here: once level).",
     "Clean aircraft climbing to 4000 ft with Radar. The takeoff is done; the Radio lesson continues to the landing.",
     "The after-takeoff checklist cleans up what the takeoff needed.",
     "Same in MSFS.",
     {A320_GT_SPOILERS, A320_GT_OVERHEAD}, false,
     [](const A320State& s, const A320Controls& c) { return !s.onGround && !c.spoilersArmed && !c.apuMaster; }},
};

constexpr int kTakeoffStepCount = static_cast<int>(sizeof(kTakeoff) / sizeof(kTakeoff[0]));
constexpr int kTakeoffRollStep = 17;  // "Brakes off, takeoff thrust"

const char* takeoffAlert(const A320State& s, const A320Controls& c, int step) {
  if (s.onGround && c.parkBrake && s.thrustDetent >= 2)
    return "Takeoff thrust with the parking brake set: release it [N], or levers to IDLE [End].";
  if (s.onGround && step >= kTakeoffRollStep && !s.vSpeedsEntered && s.thrustDetent >= 2)
    return "No V-speeds in the MCDU: PERF TAKE OFF should be filled before takeoff.";
  if (!s.onGround && s.iasKt > s.vmaxKt + 3.0) return "Too fast for the flaps: retract them or reduce speed.";
  if (s.onGround && step < kTakeoffRollStep && s.thrustDetent >= 2 && !s.atcTakeoffCleared)
    return "Not cleared for takeoff yet: levers to IDLE [End].";
  return nullptr;
}

const GuideDef kGuides[] = {
    {"ILS approach and autoland",
     "Runway 26 at Tallinn, from 20 NM out at 3000 ft: arm the approach, capture the localizer and glideslope, "
     "configure for landing and let both autopilots land.",
     A320_SCENARIO_APPROACH, "26", kIlsAutoland, kIlsStepCount, ilsAlert},
    {"Radio: a full flight with ATC",
     "From runway 26 with Tallinn Tower and Radar: ATIS, IFR clearance and readback, squawk, takeoff clearance, "
     "handovers, radar vectors, the ILS clearance and the landing clearance.",
     A320_SCENARIO_RUNWAY, "26", kRadioFlight, kRadioStepCount, radioAlert},
    {"Takeoff from cold and dark",
     "Runway 26 at Tallinn with everything off: APU and engine start, the MCDU (INIT, F-PLN, PERF TAKE OFF), the ATC "
     "clearance and the takeoff clearance, then takeoff, thrust reduction, flaps up and radar contact.",
     A320_SCENARIO_COLD_DARK, "26", kTakeoff, kTakeoffStepCount, takeoffAlert},
};

}  // namespace

int guideCount() { return static_cast<int>(sizeof(kGuides) / sizeof(kGuides[0])); }

const GuideDef* guideDef(int guide) { return guide >= 0 && guide < guideCount() ? &kGuides[guide] : nullptr; }

void GuideRunner::start(int guide) {
  guide_ = guideDef(guide) ? guide : -1;
  step_ = 0;
  done_ = 0;
  alert_ = "";
}

void GuideRunner::next() {
  const GuideDef* g = guideDef(guide_);
  if (!g || step_ >= g->stepCount) return;
  if (g->steps[step_].manual) done_ |= 1u << step_;
  ++step_;
}

void GuideRunner::back() {
  if (step_ > 0) --step_;
  done_ &= ~(1u << step_);
}

void GuideRunner::update(const A320State& s, const A320Controls& c) {
  const GuideDef* g = guideDef(guide_);
  if (!g) return;
  while (step_ < g->stepCount) {
    const GuideStep& st = g->steps[step_];
    if (st.manual || !st.done || !st.done(s, c)) break;
    done_ |= 1u << step_;
    ++step_;
  }
  const char* a = step_ < g->stepCount && g->alert ? g->alert(s, c, step_) : nullptr;
  alert_ = a ? a : "";
}

A320GuideStatus GuideRunner::status() const {
  A320GuideStatus st{};
  const GuideDef* g = guideDef(guide_);
  st.guide = guide_;
  if (!g) return st;
  st.active = 1;
  st.step = step_;
  st.stepCount = g->stepCount;
  st.complete = step_ >= g->stepCount ? 1 : 0;
  st.doneMask = done_;
  if (step_ < g->stepCount) {
    st.manual = g->steps[step_].manual ? 1 : 0;
    for (int i = 0; i < A320_GUIDE_MAX_TARGETS; ++i) st.targets[i] = g->steps[step_].targets[i];
  }
  return st;
}

}  // namespace a320

#include "a320/Guide.h"

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
     "Press AUTO BRK LO on the centre panel, next to the gear lever (MED for a short or wet runway).",
     "The LO button lights, and AUTO BRK LO appears in blue in the E/WD memo.",
     "After touchdown the autobrake brakes to a fixed deceleration (LO 1.7 m/s2, MED 3 m/s2) as soon as the ground "
     "spoilers deploy.",
     "Same buttons in MSFS, to the right of the gear lever.",
     {A320_GT_AUTOBRAKE}, false,
     [](const A320State&, const A320Controls& c) { return c.autobrake == A320_AUTOBRAKE_LO || c.autobrake == A320_AUTOBRAKE_MED; }},
    {"APPROACH PREPARATION", "Show the ILS",
     "Press LS on the EFIS panel, and turn the ND mode to ROSE LS.",
     "PFD: magenta localizer diamond at the bottom, glideslope diamond on the right. ND: course pointer and "
     "deviation bar.",
     "LS only changes the display; the ILS is tuned automatically for the runway. A diamond shows where the beam is, "
     "so fly towards it.",
     "In MSFS check the ILS frequency and course on the MCDU RAD NAV page. LS and the ND mode knob are on the EFIS "
     "panel.",
     {A320_GT_EFIS_LS, A320_GT_EFIS_ND_MODE}, false,
     [](const A320State&, const A320Controls& c) { return c.efisLs != 0 && c.ndMode == A320_ND_ROSE_LS; }},
    {"APPROACH PREPARATION", "Check the ILS",
     "On the ND, check the ILS (runway 26), its course and the distance (DME). Then press NEXT.",
     "The course arrow points along the runway, and the deviation bar sits to one side: you are not on the beam "
     "yet.",
     "Pilots verify the identifier, course and frequency before trusting an ILS: a wrong course would make LOC "
     "capture fail.",
     "In MSFS this is on the ND and the MCDU RAD NAV page.",
     {A320_GT_ND}, true, nullptr},
    {"INTERCEPT", "Slow down: SPD 180",
     "Turn the SPD knob on the FCU down to 180 kt.",
     "The blue speed target on the PFD speed tape moves to 180. A/THR reduces thrust, and the FMA still shows "
     "SPEED.",
     "This is selected speed: you set it with the knob. Managed speed (knob pushed) would follow the flight plan; "
     "this sim only has selected speed.",
     "In MSFS you usually activate the APPR phase on the MCDU PERF page and push the knob for managed speed.",
     {A320_GT_FCU_SPD, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) { return s.fcuSpdKt <= 185.0; }},
    {"INTERCEPT", "FLAPS 1",
     "Below 230 kt, move the flaps lever to 1 (V).",
     "The E/WD shows flaps 1, and the red-and-black VFE band on the speed tape moves down to 230 kt.",
     "Each flap position has a maximum speed (VFE). Extending early is not possible without overspeeding the "
     "flaps.",
     "Same lever on the pedestal in MSFS.",
     {A320_GT_FLAPS}, false,
     [](const A320State&, const A320Controls& c) { return c.flapsLever >= 1; }},
    {"INTERCEPT", "Arm the approach: APPR",
     "Press APPR on the FCU.",
     "FMA, second row: G/S and LOC in BLUE (armed). The APPR button lights up.",
     "Armed means waiting. The aircraft keeps HDG and ALT until the beams are reached: first LOC captures, then "
     "G/S. Arm it only on an intercept heading.",
     "Identical in MSFS. On a real flight you arm it once cleared for the ILS approach.",
     {A320_GT_FCU_APPR, A320_GT_FMA}, false,
     [](const A320State& s, const A320Controls&) { return (s.armed & A320_ARMED_GS) != 0 || onGlideslope(s); }},
    {"INTERCEPT", "Engage AP2",
     "Press AP2. With APPR armed both autopilots can be engaged.",
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
     "Below 200 kt set FLAPS 2, then turn the SPD knob to 160 kt.",
     "E/WD flaps 2. The speed target moves to 160.",
     "Slowing down in steps keeps the speed below each flap limit while the aircraft stays stable.",
     "In MSFS with managed speed the target follows the flaps (F and S speeds) by itself.",
     {A320_GT_FLAPS, A320_GT_FCU_SPD}, false,
     [](const A320State& s, const A320Controls& c) { return c.flapsLever >= 2 && s.fcuSpdKt <= 165.0; }},
    {"FINAL APPROACH", "Glideslope capture",
     "Wait for the glideslope diamond on the right of the PFD to come down to the centre. Level at 3000 ft you "
     "meet the beam from below, which is normal.",
     "FMA: G/S* (boxed), then G/S. The aircraft starts down at about 700-800 ft/min.",
     "G/S* = capturing, G/S = tracking the 3 degree glidepath. The ALT mode ends by itself.",
     "Same in MSFS.",
     {A320_GT_FMA, A320_GT_PFD_ILS}, false,
     [](const A320State& s, const A320Controls&) { return onGlideslope(s); }},
    {"FINAL APPROACH", "Gear down",
     "Put the gear lever down (G).",
     "The gear indications turn green; LDG MEMO on the E/WD shows GEAR DN in green.",
     "The gear adds a lot of drag, so it goes down once you start descending on the glideslope.",
     "Same lever in MSFS, on the centre panel.",
     {A320_GT_GEAR}, false,
     [](const A320State&, const A320Controls& c) { return c.gearDown != 0; }},
    {"FINAL APPROACH", "FLAPS 3, then FULL",
     "Below 185 kt FLAPS 3, then below 177 kt FLAPS FULL.",
     "E/WD: FLAPS FULL. The VLS (amber band at the bottom of the speed tape) moves down.",
     "FULL is the normal landing setting: lowest approach speed and good view over the nose.",
     "Same in MSFS.",
     {A320_GT_FLAPS}, false,
     [](const A320State&, const A320Controls& c) { return c.flapsLever == 4; }},
    {"FINAL APPROACH", "Approach speed",
     "Once the flaps are fully out, turn the SPD knob to VAPP: the top of the amber VLS band plus 5 kt (about "
     "135-140 kt).",
     "A/THR slows down and holds the speed just above the amber band.",
     "VLS is the lowest selectable speed (1.23 x stall speed). VAPP adds a margin for gusts and autothrust.",
     "In MSFS, managed speed flies VAPP from the MCDU PERF APPR page automatically.",
     {A320_GT_FCU_SPD, A320_GT_PFD_SPEED}, false,
     [](const A320State& s, const A320Controls&) {
       // VLS keeps dropping until the flaps stop moving, so judge VAPP on the final value.
       return s.flapDeg > 34.0 && s.fcuSpdKt <= s.vlsKt + 8.0 && s.fcuSpdKt >= s.vlsKt;
     }},
    {"FINAL APPROACH", "Arm the ground spoilers",
     "Pull the speedbrake lever up to ARM (on the pedestal).",
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
     "At about 40 ft the aircraft flares by itself. When you hear \"RETARD\", pull the thrust levers to IDLE (End).",
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
     "Select reverse thrust: press R (levers to reverse idle), then hold PgUp to MAX REV. On a throttle quadrant: "
     "pull the levers back behind IDLE, fully into reverse.",
     "E/WD: REV in green on both engines, N1 rises. The deceleration increases.",
     "Reversers turn the engine exhaust forward. They deploy only on the ground, and only with the levers in the "
     "reverse range.",
     "In MSFS: hold F2 (or the reverse range of your throttle) for reverse thrust.",
     {A320_GT_THRUST_LEVERS, A320_GT_EWD}, false,
     [](const A320State& s, const A320Controls& c) {
       return (s.onGround && s.reverse && c.thrustLever > 0.6) || (s.onGround && s.groundSpeedKt < 70.0);
     }},
    {"LANDING", "70 kt: reverse idle, then stow",
     "At 70 kt hold PgDn back to reverse idle, then press R to stow the reversers. The levers end at IDLE. On a "
     "quadrant: levers forward to the IDLE detent.",
     "E/WD: REV disappears. The engines spool down to idle.",
     "Reversers lose effect at low speed and can blow debris into the engines, so they are back at idle by 70 kt.",
     "In MSFS: release F2 and bring the levers to IDLE.",
     {A320_GT_THRUST_LEVERS}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.onGround && !c.reverse && c.thrustLever < 0.05 && s.groundSpeedKt < 80.0;
     }},
    {"LANDING", "Brake to a stop",
     "Hold B (or your toe brakes) to brake down to a stop on the centreline. Then set the parking brake (N) and "
     "disconnect the autopilot (AP1 / AP2).",
     "The speed reads 0, PARK BRK appears on the E/WD memo.",
     "Pressing the brakes disconnects the autobrake: from then on you brake yourself. That is normal and expected.",
     "In MSFS: brakes are the . key (or toe brakes), parking brake Ctrl+. ; the autobrake disconnects the same way.",
     {A320_GT_EWD, A320_GT_FCU_AP1}, false,
     [](const A320State& s, const A320Controls& c) {
       return s.onGround && s.groundSpeedKt < 2.0 && c.parkBrake && !c.reverse;
     }},
};

constexpr int kIlsStepCount = static_cast<int>(sizeof(kIlsAutoland) / sizeof(kIlsAutoland[0]));
constexpr int kStepTouchdown = 18;  // "Reversers": before it the aircraft must be flying

const char* ilsAlert(const A320State& s, const A320Controls& c, int step) {
  const bool flying = !s.onGround;
  if (flying && !s.apEngaged && step < kStepTouchdown)
    return "The autopilot is off (a firm sidestick input disconnects it). Press AP1 to engage it again.";
  if (flying && s.iasKt > s.vmaxKt + 3.0)
    return "Too fast for this flap setting: turn the SPD knob down.";
  if (flying && onGlideslope(s) && !c.gearDown && s.radioAltFt < 1500.0) return "The gear is still up: G.";
  if (flying && s.athrEngaged && !s.athrActive && c.thrustLever > 0.05)
    return "A/THR is armed but not active: put the thrust levers in the CL detent (Ins).";
  if (s.onGround && s.reverse && s.groundSpeedKt < 60.0)
    return "Below 70 kt: back to reverse idle (PgDn) and stow the reversers (R).";
  if (s.onGround && !s.reverse && c.thrustLever > 0.1 && step >= kStepTouchdown)
    return "Thrust levers to IDLE (End): forward thrust on the runway works against the brakes.";
  return nullptr;
}

const GuideDef kGuides[] = {
    {"ILS approach and autoland",
     "Runway 26 at Tallinn, from 20 NM out at 3000 ft: arm the approach, capture the localizer and glideslope, "
     "configure for landing and let both autopilots land.",
     A320_SCENARIO_APPROACH, kIlsAutoland, kIlsStepCount, ilsAlert},
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

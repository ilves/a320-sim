/* C API of the A320 core. Unreal (or any other front end) talks to the simulation only
 * through these POD structs and functions, so no C++ runtime or STL crosses the DLL. */
#ifndef A320_API_H
#define A320_API_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(A320CORE_BUILD)
#    define A320_API __declspec(dllexport)
#  elif defined(A320CORE_STATIC)
#    define A320_API
#  else
#    define A320_API __declspec(dllimport)
#  endif
#else
#  define A320_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define A320_API_VERSION 5

typedef enum A320Scenario {
  A320_SCENARIO_RUNWAY = 0,     /* lined up, engines idle, CONF 1+F, park brake set */
  A320_SCENARIO_FINAL_10NM = 1, /* established on the ILS, gear down, CONF 3 */
  A320_SCENARIO_FINAL_4NM = 2,  /* established on the ILS, gear down, CONF FULL */
  A320_SCENARIO_COLD_DARK = 3,  /* lined up, engines and APU off: start them yourself */
  A320_SCENARIO_APPROACH = 4    /* 20 NM out on a 30 degree intercept, 3000 ft, 220 kt clean, AP1 + A/THR */
} A320Scenario;

/* Autoflight (FCU) modes, as shown on the PFD's flight mode annunciator. */
typedef enum A320LatMode {
  A320_LAT_NONE = 0, A320_LAT_HDG, A320_LAT_LOC_STAR, A320_LAT_LOC, A320_LAT_ROLLOUT
} A320LatMode;
typedef enum A320VertMode {
  A320_VERT_NONE = 0, A320_VERT_ALT, A320_VERT_ALT_STAR, A320_VERT_VS, A320_VERT_OP_CLB, A320_VERT_OP_DES,
  A320_VERT_GS, A320_VERT_LAND, A320_VERT_FLARE,
  A320_VERT_GS_STAR /* glideslope capture, before G/S */
} A320VertMode;
typedef enum A320AthrMode {
  A320_ATHR_OFF = 0, A320_ATHR_SPEED, A320_ATHR_THR_CLB, A320_ATHR_THR_IDLE, A320_ATHR_RETARD
} A320AthrMode;
#define A320_ARMED_ALT 1
#define A320_ARMED_LOC 2
#define A320_ARMED_GS 4

/* FCU pushbuttons and knob pushes/pulls. */
typedef enum A320FcuCommand {
  A320_FCU_AP1 = 0,   /* engage / disengage the autopilot */
  A320_FCU_ATHR,      /* engage / disengage autothrust */
  A320_FCU_HDG_PULL,  /* fly the selected heading */
  A320_FCU_LOC,       /* arm (or disarm) localizer capture */
  A320_FCU_APPR,      /* arm (or disarm) localizer + glideslope capture, autoland */
  A320_FCU_ALT_PULL,  /* open climb / descent to the selected altitude */
  A320_FCU_VS_PULL,   /* hold the selected vertical speed */
  A320_FCU_AP2        /* second autopilot: both only with LOC or APPR (autoland CAT 3 DUAL) */
} A320FcuCommand;

/* Sounds the front end can trigger in addition to the ones the core raises itself. */
typedef enum A320SoundEvent {
  A320_SOUND_CLICK = 0,       /* cockpit button */
  A320_SOUND_ACK_WARNING      /* master warning pushed: silence the repetitive chime */
} A320SoundEvent;

typedef enum A320PitchLaw { A320_LAW_GROUND = 0, A320_LAW_FLIGHT = 1, A320_LAW_FLARE = 2 } A320PitchLaw;

/* Bits of A320State.warnings. Cautions (amber) are in A320_WARN_CAUTION_MASK, the rest are
 * red warnings. */
typedef enum A320WarningBit {
  A320_WARN_CONFIG_FLAPS = 1u << 0,
  A320_WARN_CONFIG_SPD_BRK = 1u << 1,
  A320_WARN_CONFIG_PARK_BRK = 1u << 2,
  A320_WARN_GEAR_NOT_DOWN = 1u << 3,
  A320_WARN_OVERSPEED = 1u << 4,
  A320_WARN_STALL = 1u << 5,
  A320_WARN_GLIDESLOPE = 1u << 6,
  A320_WARN_SINK_RATE = 1u << 7
} A320WarningBit;
#define A320_WARN_COUNT 8
#define A320_WARN_CAUTION_MASK (A320_WARN_GLIDESLOPE | A320_WARN_SINK_RATE)

/* Exterior lights (A320Controls.lights / A320State.lights) and cabin signs. */
#define A320_LT_BEACON 1
#define A320_LT_STROBE 2
#define A320_LT_NAV 4
#define A320_LT_LANDING 8
#define A320_LT_TAXI 16       /* nose light TAXI */
#define A320_LT_TAKEOFF 32    /* nose light T.O */
#define A320_LT_RWY_TURNOFF 64
#define A320_SIGN_SEATBELTS 1
#define A320_SIGN_NO_SMOKING 2

/* ND mode (EFIS selector). */
#define A320_ND_ARC 0
#define A320_ND_ROSE_NAV 1
#define A320_ND_ROSE_LS 2

/* ENG MODE selector. */
#define A320_ENG_MODE_CRANK 0
#define A320_ENG_MODE_NORM 1
#define A320_ENG_MODE_IGN_START 2

/* AUTO BRK. */
#define A320_AUTOBRAKE_OFF 0
#define A320_AUTOBRAKE_LO 1
#define A320_AUTOBRAKE_MED 2
#define A320_AUTOBRAKE_MAX 3

typedef struct A320Controls {
  double stickPitch;  /* -1..1, +1 = full back (nose up) */
  double stickRoll;   /* -1..1, +1 = full right */
  double pedals;      /* -1..1, +1 = right rudder / nosewheel right */
  double brakeLeft;   /* 0..1 */
  double brakeRight;  /* 0..1 */
  double thrustLever; /* 0..1, detents: IDLE 0, CL 0.75, FLX/MCT 0.88, TOGA 1 */
  int reverse;        /* 1 = reversers deployed (ground only), thrustLever then sets reverse thrust */
  int gearDown;       /* gear lever; retraction is blocked while on the ground */
  int flapsLever;     /* 0..4 = 0, 1, 2, 3, FULL */
  int parkBrake;
  double speedbrake;  /* 0..1 */
  int spoilersArmed;  /* speedbrake lever pulled up to ARM: ground spoilers extend at touchdown */
  int autobrake;      /* A320_AUTOBRAKE_* */
  int engMaster[2];   /* ENG MASTER 1 / 2 */
  int engMode;        /* A320_ENG_MODE_* */
  int apuMaster;      /* APU MASTER SW */
  int apuStart;       /* APU START pushbutton (held or latched: rising edge starts the APU) */
  int apuBleed;       /* APU BLEED */
  int lights;         /* A320_LT_* bits */
  int signs;          /* A320_SIGN_* bits */
  /* Two-lever hardware quadrant: with splitThrust set, engine 2 follows thrustLever2/reverse2
   * instead of thrustLever/reverse. */
  int splitThrust;
  double thrustLever2;
  int reverse2;
  /* EFIS control panel (display only; the guides check them). */
  int efisLs;  /* LS pushbutton: ILS scales on the PFD */
  int ndMode;  /* A320_ND_* */
} A320Controls;

typedef struct A320State {
  uint32_t structSize;
  double simTimeS;
  int paused;
  double simRate;

  /* Position. northM/eastM/heightAboveFieldM form the flat world the front end renders. */
  double latDeg, lonDeg, altitudeFt;
  double northM, eastM, heightAboveFieldM;

  double headingTrueDeg, pitchDeg, bankDeg, trackTrueDeg, flightPathDeg;
  double iasKt, tasKt, groundSpeedKt, mach, verticalSpeedFpm, radioAltFt, alphaDeg, loadFactor;
  double vlsKt, vsKt, vmaxKt, vfeNextKt;
  int onGround;
  int pitchLaw; /* A320PitchLaw */

  double n1[2], n2[2], fuelFlowKgH[2];
  double fuelKg, grossWeightKg;
  double thrustLever; /* most advanced forward lever (A/THR and detents), or the reverse amount */
  int thrustDetent;   /* 0 IDLE, 1 CL, 2 FLX/MCT, 3 TOGA, 4 MAN */
  int reverse;        /* any engine in reverse */

  int flapsLever, onePlusF;
  double flapDeg, gearPos, speedbrakePos;
  int gearLeverDown, parkBrake;
  double elevatorNorm, aileronNorm, rudderNorm;
  double thsDeg; /* trimmable horizontal stabiliser, negative = nose up */

  /* ILS auto-tuned to the active runway. */
  int ilsRunwayIndex;
  int locValid, gsValid;
  double locDots, gsDots, dmeNm, ilsCourseDeg;
  int papiWhite[4]; /* left to right as seen on approach */

  uint32_t warnings;    /* bitmask, see a320_warning_text */
  uint32_t calloutSeq;  /* increments whenever a new callout is announced */
  char callout[32];

  /* Last touchdown, for landing feedback. */
  uint32_t touchdownSeq;
  double touchdownFpm, touchdownDistanceM, touchdownCenterlineM;

  /* Autoflight. FCU heading is magnetic. */
  int apEngaged, athrEngaged, athrActive;
  int latMode, vertMode, athrMode, armed; /* A320LatMode, A320VertMode, A320AthrMode, A320_ARMED_* bits */
  double fcuSpdKt, fcuHdgMagDeg, fcuAltFt, fcuVsFpm;
  uint32_t apDisconnectSeq; /* increments on every autopilot disconnect (cavalry charge) */

  /* Systems. */
  double apuN;              /* APU speed % */
  int apuAvail, apuStarting, apuMaster, apuBleed;
  int bleedAvailable;       /* start air: APU bleed or a running engine */
  int engMaster[2], engMode, engRunning[2], engStarting[2];
  int autobrake, autobrakeActive, autobrakeDecel; /* DECEL light: target deceleration reached */
  int spoilersArmed, groundSpoilers;              /* ground spoilers extended */
  int lights, signs;

  /* Per-engine thrust levers (equal unless the controls split them); with reverseEng the
   * lever is the reverse amount. */
  double thrustLeverEng[2];
  int reverseEng[2];

  int ap1Engaged, ap2Engaged; /* apEngaged is either */
  /* Sim tutor: why a button press did nothing, or what to watch for (empty when none). */
  uint32_t hintSeq;
  char hint[160];
} A320State;

typedef struct A320RunwayInfo {
  char ident[8];
  double trueCourseDeg, widthM, landingDistanceM, displacementM;
  /* Flat-world positions (metres north/east of the airport reference point). */
  double startNorthM, startEastM;
  double thresholdNorthM, thresholdEastM;
  double endNorthM, endEastM;
  double localizerNorthM, localizerEastM;
  double gsOriginNorthM, gsOriginEastM;
  double glideslopeDeg;
} A320RunwayInfo;

typedef struct A320Sim A320Sim;

/* jsbsimRoot: folder containing aircraft/ and engine/. Returns NULL and fills error on failure. */
A320_API A320Sim* a320_create(const char* jsbsimRoot, char* error, int errorSize);
A320_API void a320_destroy(A320Sim* sim);
A320_API int a320_api_version(void);

A320_API int a320_reset(A320Sim* sim, A320Scenario scenario, int runwayIndex);
A320_API void a320_set_controls(A320Sim* sim, const A320Controls* controls);
/* The controls the core is using: after a reset, the scenario's switch and lever positions.
 * Start from these rather than a zeroed struct (zero means engine masters off). */
A320_API void a320_get_controls(const A320Sim* sim, A320Controls* controls);
/* Advances by real elapsed time; the core runs fixed 120 Hz steps, honouring pause and rate. */
A320_API void a320_update(A320Sim* sim, double realDtS);
A320_API void a320_get_state(const A320Sim* sim, A320State* state);

A320_API void a320_set_paused(A320Sim* sim, int paused);
A320_API void a320_set_sim_rate(A320Sim* sim, double rate);

A320_API const char* a320_airport_icao(const A320Sim* sim);
A320_API double a320_field_elevation_ft(const A320Sim* sim);
A320_API double a320_magnetic_variation_deg(const A320Sim* sim); /* east positive */
A320_API int a320_runway_count(const A320Sim* sim);
A320_API int a320_get_runway(const A320Sim* sim, int index, A320RunwayInfo* info);

A320_API void a320_fcu_command(A320Sim* sim, A320FcuCommand command);
/* Selected targets; the FCU rounds them like the real knobs (1 kt, 1 deg, 100 ft, 100 fpm). */
A320_API void a320_fcu_set_targets(A320Sim* sim, double spdKt, double hdgMagDeg, double altFt, double vsFpm);
A320_API const char* a320_lat_mode_name(int latMode);
A320_API const char* a320_vert_mode_name(int vertMode);
A320_API const char* a320_athr_mode_name(int athrMode);

/* Audio: one mono stream with engines, airflow, rumble, chimes and callouts. soundsDir holds
 * the callout WAVs (missing files are skipped). Render from the same thread as a320_update. */
A320_API int a320_audio_init(A320Sim* sim, int sampleRate, const char* soundsDir);
A320_API void a320_audio_render(A320Sim* sim, int16_t* out, int frames);
A320_API void a320_audio_event(A320Sim* sim, A320SoundEvent event);
A320_API void a320_audio_set_volume(A320Sim* sim, double volume);

/* Guides: step-by-step lessons that watch the cockpit and tick each step off when it is done.
 * The front end resets to a320_guide_scenario() before a320_guide_start(). */
typedef enum A320GuideTarget {
  A320_GT_NONE = 0,
  A320_GT_FMA, A320_GT_PFD_SPEED, A320_GT_PFD_ILS, A320_GT_ND, A320_GT_EWD,
  A320_GT_FCU_SPD, A320_GT_FCU_HDG, A320_GT_FCU_ALT, A320_GT_FCU_AP1, A320_GT_FCU_AP2, A320_GT_FCU_ATHR,
  A320_GT_FCU_LOC, A320_GT_FCU_APPR, A320_GT_EFIS_LS, A320_GT_EFIS_ND_MODE,
  A320_GT_THRUST_LEVERS, A320_GT_FLAPS, A320_GT_GEAR, A320_GT_SPOILERS, A320_GT_AUTOBRAKE,
  A320_GT_COUNT
} A320GuideTarget;
typedef enum A320GuideText {
  A320_GUIDE_PHASE = 0, /* e.g. "APPROACH PREPARATION" */
  A320_GUIDE_TITLE,     /* short step name */
  A320_GUIDE_ACTION,    /* what to do */
  A320_GUIDE_LOOK,      /* what to look for when it worked */
  A320_GUIDE_WHY,       /* how it works */
  A320_GUIDE_MSFS       /* the same in Microsoft Flight Simulator 2024 */
} A320GuideText;
#define A320_GUIDE_MAX_TARGETS 3
typedef struct A320GuideStatus {
  int active, guide, step, stepCount;
  int complete;         /* every step done */
  int manual;           /* the current step is a check: NEXT confirms it */
  uint32_t doneMask;    /* bit per step */
  int targets[A320_GUIDE_MAX_TARGETS]; /* A320GuideTarget of the current step, NONE-padded */
} A320GuideStatus;

A320_API int a320_guide_count(void);
A320_API const char* a320_guide_name(int guide);
A320_API const char* a320_guide_summary(int guide);
A320_API A320Scenario a320_guide_scenario(int guide);
A320_API const char* a320_guide_runway(int guide); /* runway ident the lesson is written for */
A320_API int a320_guide_step_count(int guide);
A320_API const char* a320_guide_step_text(int guide, int step, A320GuideText field);
A320_API void a320_guide_start(A320Sim* sim, int guide);
A320_API void a320_guide_stop(A320Sim* sim);
A320_API void a320_guide_next(A320Sim* sim); /* confirm a check step, or skip a step */
A320_API void a320_guide_back(A320Sim* sim);
A320_API void a320_guide_get_status(const A320Sim* sim, A320GuideStatus* status);
/* Something that needs attention now (e.g. "the autopilot is off"), or "". */
A320_API const char* a320_guide_alert(const A320Sim* sim);

A320_API const char* a320_warning_text(uint32_t warningBit);
A320_API const char* a320_flap_config_name(int flapsLever, int onePlusF);
A320_API const char* a320_thrust_detent_name(int detent);

#ifdef __cplusplus
}
#endif

#endif /* A320_API_H */

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

#define A320_API_VERSION 1

typedef enum A320Scenario {
  A320_SCENARIO_RUNWAY = 0,     /* lined up, engines idle, CONF 1+F, park brake set */
  A320_SCENARIO_FINAL_10NM = 1, /* established on the ILS, gear down, CONF 3 */
  A320_SCENARIO_FINAL_4NM = 2   /* established on the ILS, gear down, CONF FULL */
} A320Scenario;

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
  double thrustLever;
  int thrustDetent; /* 0 IDLE, 1 CL, 2 FLX/MCT, 3 TOGA, 4 MAN */
  int reverse;

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

A320_API const char* a320_warning_text(uint32_t warningBit);
A320_API const char* a320_flap_config_name(int flapsLever, int onePlusF);
A320_API const char* a320_thrust_detent_name(int detent);

#ifdef __cplusplus
}
#endif

#endif /* A320_API_H */

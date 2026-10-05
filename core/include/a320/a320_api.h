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

#define A320_API_VERSION 16

typedef enum A320Scenario {
  A320_SCENARIO_RUNWAY = 0,     /* lined up, engines idle, CONF 1+F, park brake set */
  A320_SCENARIO_FINAL_10NM = 1, /* established on the ILS, gear down, CONF 3 */
  A320_SCENARIO_FINAL_4NM = 2,  /* established on the ILS, gear down, CONF FULL */
  A320_SCENARIO_COLD_DARK = 3,  /* lined up, engines and APU off: start them yourself */
  A320_SCENARIO_APPROACH = 4    /* 20 NM out on a 30 degree intercept, 3000 ft, 220 kt clean, AP1 + A/THR */
} A320Scenario;

/* Autoflight (FCU) modes, as shown on the PFD's flight mode annunciator. */
typedef enum A320LatMode {
  A320_LAT_NONE = 0, A320_LAT_HDG, A320_LAT_LOC_STAR, A320_LAT_LOC, A320_LAT_ROLLOUT,
  A320_LAT_TRK, /* API 13: the selected track (TRK-FPA reference) */
  A320_LAT_NAV  /* API 14: the flight plan's route (managed lateral navigation) */
} A320LatMode;
typedef enum A320VertMode {
  A320_VERT_NONE = 0, A320_VERT_ALT, A320_VERT_ALT_STAR, A320_VERT_VS, A320_VERT_OP_CLB, A320_VERT_OP_DES,
  A320_VERT_GS, A320_VERT_LAND, A320_VERT_FLARE,
  A320_VERT_GS_STAR, /* glideslope capture, before G/S */
  A320_VERT_FPA,     /* API 13: the selected flight path angle (TRK-FPA reference) */
  /* API 15: managed (with a flight plan's route): CLB to the FCU altitude, DES on the computed
   * descent path to the next altitude constraint, and the capture and hold of that constraint. */
  A320_VERT_CLB, A320_VERT_DES, A320_VERT_ALT_CST_STAR, A320_VERT_ALT_CST
} A320VertMode;
typedef enum A320AthrMode {
  A320_ATHR_OFF = 0, A320_ATHR_SPEED, A320_ATHR_THR_CLB, A320_ATHR_THR_IDLE, A320_ATHR_RETARD,
  A320_ATHR_AFLOOR, /* alpha floor: TOGA thrust whatever the levers */
  A320_ATHR_TOGA_LK /* after alpha floor: TOGA kept until A/THR is disconnected */
} A320AthrMode;
#define A320_ARMED_ALT 1
#define A320_ARMED_LOC 2
#define A320_ARMED_GS 4
#define A320_ARMED_NAV 8 /* API 14: engages at 30 ft after takeoff, or when the route is reached */
#define A320_ARMED_CLB 16 /* API 15: engages at 1500 ft after takeoff */

/* FCU pushbuttons and knob pushes/pulls. */
typedef enum A320FcuCommand {
  A320_FCU_AP1 = 0,   /* engage / disengage the autopilot */
  A320_FCU_ATHR,      /* engage / disengage autothrust */
  A320_FCU_HDG_PULL,  /* fly the selected heading */
  A320_FCU_LOC,       /* arm (or disarm) localizer capture */
  A320_FCU_APPR,      /* arm (or disarm) localizer + glideslope capture, autoland */
  A320_FCU_ALT_PULL,  /* open climb / descent to the selected altitude */
  A320_FCU_VS_PULL,   /* hold the selected vertical speed */
  A320_FCU_AP2,       /* second autopilot: both only with LOC or APPR (autoland CAT 3 DUAL) */
  /* API 13. */
  A320_FCU_TRK_FPA,   /* HDG-V/S / TRK-FPA pushbutton: the heading and V/S windows become track and FPA */
  A320_FCU_HDG_PUSH,  /* NAV along the flight plan (armed on the ground); without one, hold the present heading */
  A320_FCU_ALT_PUSH,  /* CLB or DES towards the FCU altitude along the flight plan; without one, level off */
  A320_FCU_VS_PUSH    /* level off: V/S 0 (FPA 0) */
} A320FcuCommand;

/* Sounds the front end can trigger in addition to the ones the core raises itself. */
typedef enum A320SoundEvent {
  A320_SOUND_CLICK = 0,       /* cockpit button */
  A320_SOUND_ACK_WARNING      /* master warning pushed: silence the repetitive chime */
} A320SoundEvent;

/* The aircraft destroyed (API 12): a structural breakup in flight at or above
 * A320_BREAKUP_IAS_KT, or a crash: a ground impact at or beyond A320_CRASH_SINK_FPM, a wing
 * or the nose first, or the fuselage touching. The flight model stops; a new flight resets it. */
typedef enum A320Destroyed { A320_DESTROYED_NONE = 0, A320_DESTROYED_BREAKUP, A320_DESTROYED_CRASH } A320Destroyed;
/* 380 kt: the A320's design dive speed (VD), the most it is shown to survive; VMO is 350 kt. */
#define A320_BREAKUP_IAS_KT 380.0
#define A320_CRASH_SINK_FPM 1500.0
/* The fuselage breaks this far ahead of the reference point (m): the nose section ahead, the
 * wings, engines and tail behind. */
#define A320_BREAKUP_SPLIT_X_M 3.3

/* One piece of a destroyed aircraft. Its parts keep their places in the aircraft's frame, so the
 * pose is that frame's: the reference point (the CG before the breakup) and its attitude. */
typedef struct A320Section {
  double northM, eastM, heightAboveFieldM;
  double gridHeadingDeg, pitchDeg, bankDeg;
  double velNorthMps, velEastMps, velUpMps;
  int onGround;
  uint32_t impactSeq; /* +1 when it hits the ground */
} A320Section;

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
  /* Radio management panel (VHF 1, kHz, e.g. 135905) and the ATC transponder (API 9). */
  int com1ActiveKhz, com1StandbyKhz;
  int xpdrCode; /* four octal digits written as a decimal number, e.g. 2341 */
  int xpdrMode; /* A320_XPDR_* */
} A320Controls;

#define A320_XPDR_STBY 0
#define A320_XPDR_AUTO 1 /* replies once airborne */
#define A320_XPDR_ON 2

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

  /* ILS tuned by the FMGC (MCDU RAD NAV / arrival): runway index, -1 = none. */
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

  /* Autoflight. FCU heading is magnetic; in TRK-FPA (fcuTrkFpa) it is the selected track. */
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
  char hint[256];

  /* Takeoff speeds, frozen once the takeoff roll starts: the MCDU entries, else computed for the
   * current weight and flaps. */
  double v1Kt, vrKt, v2Kt;

  /* MCDU / FMGC (API 8). */
  int vSpeedsEntered;             /* V1/VR/V2 come from the MCDU PERF TAKE OFF page */
  int depRunwayIndex, arrRunwayIndex; /* -1 = none */
  char ilsIdent[8];
  double ilsFreqMHz;              /* 0 = no ILS tuned */
  double ilsCourseMagDeg;         /* RAD NAV CRS: entered, else the published course */
  int ilsManual;                  /* tuned on RAD NAV rather than automatically */
  int papiRunway[4][4];           /* PAPI of every runway direction (index < a320_runway_count) */
  int dhFt, mdaFt;                /* approach minimums from PERF APPR, -1 = none */
  double vappKt;                  /* approach speed for the landing configuration */

  /* ATC (API 9). */
  int atcEnabled;
  int atcPhase;                   /* A320AtcPhase */
  int atcClearedAltFt;            /* 0 = none yet */
  int atcHeadingMag;              /* assigned heading, -1 = own navigation */
  int atcSpeedKt;                 /* assigned speed, 0 = none */
  int atcSquawk;                  /* assigned code, -1 = none */
  int atcIfrCleared, atcTakeoffCleared, atcApproachCleared, atcLandingCleared, atcRadarContact;
  int atcAwaitingReadback;        /* an instruction waits for the crew's readback */
  int atcRunwayIndex;             /* the runway ATC expects you to land on */
  int com1ActiveKhz;              /* mirrors the control, for the audio engine */
  uint32_t atcMessageSeq;         /* number of the newest radio message (a320_atc_message) */

  /* MCDU preparation, for the lessons (API 10). */
  int fmsFlightNumberSet;         /* INIT: FLT NBR entered */
  int fmsFlapsThsSet;             /* PERF TAKE OFF: FLAPS/THS entered */
  int fmsFlexTempC;               /* PERF TAKE OFF: FLEX TO TEMP, -100 = none */

  /* Several airports (API 11). */
  int nearestAirport;             /* a320_get_airport index; the flight model's ground is its elevation */
  double magneticVariationDeg;    /* at the nearest airport, east positive */
  /* Heading in the flat world (northM/eastM axes): true heading plus the meridian convergence,
   * about 2 degrees at Kuressaare. Orient the aircraft and the map with this. */
  double gridHeadingDeg, gridTrackDeg;
  /* API 12. */
  double velNorthMps, velEastMps, velUpMps;
  int destroyed;             /* A320Destroyed */
  uint32_t destroyedSeq;     /* +1 each time */
  double impactFpm;          /* vertical speed at a crash */
  double groundHeightM;      /* the flight model's ground here, in heightAboveFieldM terms */
  /* After a breakup: [0] the nose section, [1] the rest (wings, engines, tail), falling. After
   * a crash both are the aircraft where it hit. */
  A320Section sections[2];
  /* API 13. */
  int fcuTrkFpa;       /* HDG-V/S / TRK-FPA pushbutton in TRK-FPA */
  double fcuFpaDeg;    /* selected flight path angle */
  double windFromTrueDeg, windKt;
  /* API 14: the flight plan's route (a320_get_waypoint) and where the FMS is on it. */
  int routeCount;
  int routeActive;          /* the TO waypoint, -1 = none */
  char toWaypoint[8];
  double toDistanceNm, toBearingMagDeg;
  double crossTrackNm;      /* right of the active leg positive */
  double routeRemainingNm;  /* along the route to its last point */
  /* API 15: the managed descent. */
  int descentPathValid;     /* a route with an altitude constraint ahead */
  double descentPathAltFt;  /* the 3 degree path's altitude here (ending at the constraint) */
  int descentConstraintFt;  /* the next altitude constraint ahead, 0 = none */
  int todValid;             /* the top of descent is ahead */
  double todDistanceNm;     /* along the route */
  double todNorthM, todEastM;
  int weather;              /* A320Weather (API 15) */
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
  /* API 11. */
  int airport;              /* a320_get_airport index */
  char icao[8];
  double elevationM;        /* threshold height in the flat world (above the first airport's field) */
  int hasIls;
  double gridCourseDeg;     /* course in the flat world: true course plus the meridian convergence */
} A320RunwayInfo;

typedef struct A320AirportInfo {
  char icao[8];
  char name[48];
  double northM, eastM;     /* reference point in the flat world */
  double elevationM;        /* above the first airport's field elevation */
  double magneticVariationDeg;
} A320AirportInfo;

typedef struct A320Sim A320Sim;

/* jsbsimRoot: folder containing aircraft/ and engine/. Returns NULL and fills error on failure. */
A320_API A320Sim* a320_create(const char* jsbsimRoot, char* error, int errorSize);
A320_API void a320_destroy(A320Sim* sim);
A320_API int a320_api_version(void);

/* How much of the MCDU flight plan a new flight starts with. */
typedef enum A320FlightPlan {
  A320_PLAN_EMPTY = 0, /* the crew enters it: INIT FROM/TO, runways, PERF (in the air FROM/TO is set) */
  A320_PLAN_ROUTE,     /* FROM/TO and the departure runway, the arrival for a trip elsewhere or in the air */
  A320_PLAN_FULL       /* all of it: flight number, cost index, cruise level, runways and arrival,
                          PERF TAKE OFF (V-speeds, flaps, FLEX) and PERF APPR (QNH, wind, minimum) */
} A320FlightPlan;

/* Same as a320_start_flight(sim, scenario, runwayIndex, runwayIndex, 20, A320_PLAN_ROUTE). */
A320_API int a320_reset(A320Sim* sim, A320Scenario scenario, int runwayIndex);
/* A flight between two runways (indices from a320_get_runway, any airport): ground scenarios start
 * at depRunway, airborne ones distanceNm from arrRunway (APPROACH; the finals at 10 and 4 NM). ATC
 * is set up for the trip, the MCDU flight plan as flightPlan (A320FlightPlan) says. */
A320_API int a320_start_flight(A320Sim* sim, A320Scenario scenario, int depRunway, int arrRunway, double distanceNm,
                               int flightPlan);
/* The terrain for the flight model: terrainDir holds ground.txt and ground.i16 (Content/Terrain).
 * a320_create already loads it from jsbsimRoot/../Terrain when it is there. Without it the
 * ground is the nearest airport's elevation everywhere. 1 if loaded. */
A320_API int a320_load_ground(A320Sim* sim, const char* terrainDir);
A320_API int a320_airport_count(const A320Sim* sim);
A320_API int a320_get_airport(const A320Sim* sim, int index, A320AirportInfo* info);
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
/* The selected flight path angle (API 13), rounded to 0.1 degree, at most 9.9 either way. */
A320_API void a320_fcu_set_fpa(A320Sim* sim, double fpaDeg);
/* The weather (API 15). The front end draws it (sky, clouds, fog, rain; day or night is its own
 * choice); the core reports it on the ATIS and plays the rain. Heights are above the airports. */
typedef enum A320Weather {
  A320_WEATHER_SUNNY = 0, /* no significant cloud, 10 km or more */
  A320_WEATHER_CLOUDS,    /* broken cloud 2500-4500 ft, 10 km or more */
  A320_WEATHER_RAIN,      /* moderate rain, 4000 m, overcast 1200-6000 ft */
  A320_WEATHER_FOG,       /* 300 m in fog, 200 ft deep: the sky above is clear */
  A320_WEATHER_COUNT
} A320Weather;
typedef struct A320WeatherInfo {
  double visibilityM;
  double cloudBaseFt, cloudTopFt; /* 0 = no layer */
  double cloudCover;              /* 0..1 */
  double fogTopFt;                /* 0 = no fog */
  int rain;
  char name[16];                  /* "SUNNY", "CLOUDS", "RAIN", "FOG" */
} A320WeatherInfo;
A320_API void a320_set_weather(A320Sim* sim, int weather); /* stays for later flights */
A320_API int a320_weather_info(int weather, A320WeatherInfo* out);
/* A steady wind (API 13), the direction it blows from (true). It stays for later flights. */
A320_API void a320_set_wind(A320Sim* sim, double fromTrueDeg, double kt);
/* The flight plan's waypoints (API 14), built from the MCDU's FROM/TO and runways: the departure
 * runway, a climb on its track to an altitude, real FRA points (eAIP ENR 4.4) and an ILS transition. */
typedef enum A320WaypointKind {
  A320_WPT_FIX = 0,      /* a named point */
  A320_WPT_ALTITUDE,     /* "(1630)": climb on the runway track to altFt, then direct to the next fix */
  A320_WPT_APPROACH,     /* a generated approach fix: CF (course fix), FF (final fix), DW/BS (downwind, base) */
  A320_WPT_RUNWAY,       /* the departure runway (first) or the landing threshold (last) */
  A320_WPT_POSITION      /* where a plan made in the air starts */
} A320WaypointKind;
typedef struct A320Waypoint {
  char ident[8];
  int kind;              /* A320WaypointKind */
  double northM, eastM;  /* flat world */
  double latDeg, lonDeg;
  int altFt;             /* constraint, or an A320_WPT_ALTITUDE leg's altitude; 0 = none */
  double legCourseMagDeg, legNm; /* the leg to it from the previous waypoint */
} A320Waypoint;
A320_API int a320_route_count(const A320Sim* sim);
A320_API int a320_get_waypoint(const A320Sim* sim, int index, A320Waypoint* out);
/* The route a flight would get (API 16), without starting it: the same arguments as
 * a320_start_flight. Fills up to maxPoints of out; returns the route's length (0: no route, e.g. a
 * circuit, or an empty flight plan on the ground). */
A320_API int a320_preview_route(const A320Sim* sim, A320Scenario scenario, int depRunway, int arrRunway, double distanceNm,
                                int flightPlan, A320Waypoint* out, int maxPoints);

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
  A320_GT_PARK_BRAKE,
  A320_GT_MCDU,
  A320_GT_RADIO,     /* the RADIO window: RMP and transponder */
  A320_GT_ATC_REPLY, /* the reply list in the RADIO window */
  A320_GT_OVERHEAD,  /* the overhead panel: APU, bleed, lights, signs */
  A320_GT_ENGINES,   /* ENG MASTER switches and the ENG MODE selector */
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

/* MCDU: the Airbus multipurpose control and display unit (flight plan, radio nav, performance).
 * Keys are ASCII characters ('A'-'Z', '0'-'9', '.', '/', ' ', '+', '-') or A320McduKey. */
#define A320_MCDU_ROWS 14
#define A320_MCDU_COLS 24

typedef enum A320McduKey {
  A320_MCDU_LSK1L = 256, /* LSK1L..LSK6L = 256..261 */
  A320_MCDU_LSK1R = 262, /* LSK1R..LSK6R = 262..267 */
  A320_MCDU_DIR = 268,
  A320_MCDU_PROG,
  A320_MCDU_PERF,
  A320_MCDU_INIT,
  A320_MCDU_DATA,
  A320_MCDU_FPLN,
  A320_MCDU_RADNAV,
  A320_MCDU_FUELPRED,
  A320_MCDU_SECFPLN,
  A320_MCDU_ATCCOMM,
  A320_MCDU_MENU,
  A320_MCDU_AIRPORT,
  A320_MCDU_NEXTPAGE,
  A320_MCDU_UP,
  A320_MCDU_DOWN,
  A320_MCDU_CLR,
  A320_MCDU_OVFY,
  A320_MCDU_PLUSMINUS
} A320McduKey;

typedef enum A320McduColor {
  A320_MCDU_WHITE = 0,
  A320_MCDU_CYAN,    /* crew entries and selectable values */
  A320_MCDU_GREEN,   /* active data */
  A320_MCDU_AMBER,   /* required entries, messages that need action */
  A320_MCDU_MAGENTA, /* constraints */
  A320_MCDU_YELLOW   /* temporary flight plan */
} A320McduColor;

/* Row 0 is the title, rows 1-12 alternate small labels and data lines (LSK n is row 2n), row 13
 * is the scratchpad. '#' is the amber entry box, '`' the degree sign. */
typedef struct A320McduDisplay {
  char text[A320_MCDU_ROWS][A320_MCDU_COLS + 1];
  uint8_t color[A320_MCDU_ROWS][A320_MCDU_COLS];
  uint8_t small[A320_MCDU_ROWS][A320_MCDU_COLS];
} A320McduDisplay;

A320_API void a320_mcdu_key(A320Sim* sim, int key);
A320_API void a320_mcdu_get_display(const A320Sim* sim, A320McduDisplay* display);

/* ATC: Tallinn Information (ATIS) 124.880, Tallinn Tower 135.905 (also IFR clearances), Tallinn
 * Radar 127.905 (the whole route), Tallinn Handling 131.905 (EETN AD 2.18); Kuressaare Information
 * 118.055, an AFIS (EEKE AD 2.18) that relays clearances and reports the runway. Every transmission on the radio, ATC's
 * and the crew's, is a message; the front end speaks it (speech) and logs it (text). */
typedef enum A320AtcPhase {
  A320_ATC_PHASE_CLEARANCE = 0, /* on the ground: IFR clearance from Tower */
  A320_ATC_PHASE_DEPARTURE,     /* cleared IFR: ready for departure, takeoff */
  A320_ATC_PHASE_RADAR,         /* with Tallinn Radar: vectors */
  A320_ATC_PHASE_APPROACH,      /* cleared for the ILS */
  A320_ATC_PHASE_TOWER,         /* with Tower on final */
  A320_ATC_PHASE_LANDED,
  A320_ATC_PHASE_DONE
} A320AtcPhase;

typedef enum A320AtcSpeaker {
  A320_ATC_SPEAKER_ATC = 0,
  A320_ATC_SPEAKER_PILOT,
  A320_ATC_SPEAKER_ATIS
} A320AtcSpeaker;

#define A320_ATC_MAX_OPTIONS 6

typedef struct A320AtcMessage {
  uint32_t seq;
  int speaker;       /* A320AtcSpeaker */
  int frequencyKhz;
  int heard;         /* on the frequency COM 1 was tuned to (otherwise nobody on board heard it) */
  double simTimeS;
  char station[24];  /* "TALLINN TOWER", or the callsign for the crew */
  char text[320];    /* as written, e.g. "SIM320, turn left heading 080" */
  char speech[480];  /* as spoken, e.g. "Sierra India Mike three two zero, turn left heading zero eight zero" */
} A320AtcMessage;

typedef struct A320AtcStatus {
  char station[24]; /* who is on COM 1 now, "" = nobody */
  int optionCount;  /* what the crew can say now (readbacks, requests) */
  char options[A320_ATC_MAX_OPTIONS][200];
  int awaitingReadback;
  char callsign[16];
} A320AtcStatus;

/* Message number seq (1 .. state.atcMessageSeq); 0 if it is too old to be kept. */
A320_API int a320_atc_message(const A320Sim* sim, uint32_t seq, A320AtcMessage* out);
A320_API void a320_atc_get_status(const A320Sim* sim, A320AtcStatus* status);
A320_API void a320_atc_choose(A320Sim* sim, int option);
A320_API void a320_atc_set_enabled(A320Sim* sim, int enabled);
/* A spoken transmission (16-bit mono PCM) for the radio: band-pass, static and squelch, heard only
 * while COM 1 stays on frequencyKhz. Clips play one after another. */
A320_API void a320_audio_radio_clip(A320Sim* sim, const int16_t* samples, int frames, int sampleRate, int frequencyKhz);

A320_API const char* a320_warning_text(uint32_t warningBit);
A320_API const char* a320_flap_config_name(int flapsLever, int onePlusF);
A320_API const char* a320_thrust_detent_name(int detent);

#ifdef __cplusplus
}
#endif

#endif /* A320_API_H */

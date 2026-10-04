#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <new>
#include <string>

#include "a320/Guide.h"
#include "a320/Simulation.h"
#include "a320/Systems.h"
#include "a320/Units.h"
#include "a320/a320_api.h"

struct A320Sim {
  a320::Simulation sim{a320::makeEstonia()};
  a320::GuideRunner guide;
};

namespace {

void copyError(const std::string& msg, char* error, int errorSize) {
  if (error && errorSize > 0) std::snprintf(error, static_cast<size_t>(errorSize), "%s", msg.c_str());
}

}  // namespace

extern "C" {

A320Sim* a320_create(const char* jsbsimRoot, char* error, int errorSize) {
  try {
    auto* handle = new A320Sim();
    std::string err;
    if (!handle->sim.init(jsbsimRoot ? jsbsimRoot : "", &err)) {
      copyError(err, error, errorSize);
      delete handle;
      return nullptr;
    }
    // The terrain next to the flight data (Content/Terrain beside Content/JSBSim), if it is there.
    if (jsbsimRoot) handle->sim.loadGround(std::string(jsbsimRoot) + "/../Terrain");
    return handle;
  } catch (const std::exception& e) {
    copyError(e.what(), error, errorSize);
  } catch (...) {
    copyError("unknown error", error, errorSize);
  }
  return nullptr;
}

void a320_destroy(A320Sim* sim) { delete sim; }

int a320_api_version(void) { return A320_API_VERSION; }

int a320_reset(A320Sim* sim, A320Scenario scenario, int runwayIndex) {
  if (!sim) return 0;
  try {
    return sim->sim.reset(scenario, runwayIndex) ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

int a320_start_flight(A320Sim* sim, A320Scenario scenario, int depRunway, int arrRunway, double distanceNm,
                      int flightPlan) {
  if (!sim) return 0;
  try {
    return sim->sim.startFlight(scenario, depRunway, arrRunway, distanceNm, flightPlan) ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

int a320_load_ground(A320Sim* sim, const char* terrainDir) {
  if (!sim || !terrainDir) return 0;
  return sim->sim.loadGround(terrainDir) ? 1 : 0;
}

int a320_airport_count(const A320Sim* sim) { return sim ? static_cast<int>(sim->sim.world().airports.size()) : 0; }

int a320_get_airport(const A320Sim* sim, int index, A320AirportInfo* info) {
  if (!sim || !info || index < 0 || index >= a320_airport_count(sim)) return 0;
  const a320::World& w = sim->sim.world();
  const a320::Airport& a = w.airports[static_cast<size_t>(index)];
  *info = A320AirportInfo{};
  std::snprintf(info->icao, sizeof(info->icao), "%s", a.icao.c_str());
  std::snprintf(info->name, sizeof(info->name), "%s", a.name.c_str());
  info->northM = w.airportNorthM[static_cast<size_t>(index)];
  info->eastM = w.airportEastM[static_cast<size_t>(index)];
  info->elevationM = a.reference.altM - w.reference.altM;
  info->magneticVariationDeg = a.magneticVariationDeg;
  return 1;
}

void a320_set_controls(A320Sim* sim, const A320Controls* controls) {
  if (sim && controls) sim->sim.setControls(*controls);
}

void a320_get_controls(const A320Sim* sim, A320Controls* controls) {
  if (sim && controls) *controls = sim->sim.controls();
}

void a320_update(A320Sim* sim, double realDtS) {
  if (!sim) return;
  try {
    sim->sim.update(realDtS);
    sim->guide.update(sim->sim.state(), sim->sim.controls());
  } catch (...) {
    // JSBSim throws on numerical blow-ups; freeze rather than take the host process down.
    sim->sim.clock().setPaused(true);
  }
}

void a320_get_state(const A320Sim* sim, A320State* state) {
  if (sim && state) *state = sim->sim.state();
}

void a320_set_paused(A320Sim* sim, int paused) {
  if (sim) sim->sim.clock().setPaused(paused != 0);
}

void a320_set_sim_rate(A320Sim* sim, double rate) {
  if (sim) sim->sim.clock().setRate(rate);
}

// The first airport's: the flat world's origin.
const char* a320_airport_icao(const A320Sim* sim) { return sim ? sim->sim.world().airports[0].icao.c_str() : ""; }

double a320_field_elevation_ft(const A320Sim* sim) {
  return sim ? sim->sim.world().reference.altM / 0.3048 : 0.0;
}

double a320_magnetic_variation_deg(const A320Sim* sim) {
  return sim ? sim->sim.world().airports[0].magneticVariationDeg : 0.0;
}

int a320_runway_count(const A320Sim* sim) {
  return sim ? static_cast<int>(sim->sim.world().runways.size()) : 0;
}

int a320_get_runway(const A320Sim* sim, int index, A320RunwayInfo* info) {
  if (!sim || !info || index < 0 || index >= a320_runway_count(sim)) return 0;
  const a320::Simulation& s = sim->sim;
  const a320::World& w = s.world();
  const a320::Runway& rw = w.runways[static_cast<size_t>(index)];
  const a320::Airport& airport = w.airports[static_cast<size_t>(rw.airport)];
  // Geometry in the runway's own airport frame, placed in the flat world through geodetic
  // coordinates like the aircraft (see Simulation::refreshState).
  const a320::LocalFrame& af = s.airportFrame(rw.airport);
  const a320::Ils ils(af, rw);
  const a320::RunwayAxes& axes = ils.axes();

  *info = A320RunwayInfo{};
  std::snprintf(info->ident, sizeof(info->ident), "%s", rw.ident.c_str());
  std::snprintf(info->icao, sizeof(info->icao), "%s", airport.icao.c_str());
  info->airport = rw.airport;
  info->elevationM = rw.threshold.altM - w.reference.altM;
  info->hasIls = rw.ils.ident.empty() ? 0 : 1;
  info->trueCourseDeg = rw.trueCourseDeg;
  info->widthM = rw.widthM;
  info->landingDistanceM = axes.landingDistanceM();
  info->displacementM = axes.displacementM();
  info->glideslopeDeg = ils.glideslopeDeg();

  auto put = [&](double x, double& north, double& east) {
    const a320::Enu p = s.frame().toEnu(af.toGeo(axes.toEnu({x, 0.0, 0.0})));
    north = p.n;
    east = p.e;
  };
  put(-axes.displacementM(), info->startNorthM, info->startEastM);
  put(0.0, info->thresholdNorthM, info->thresholdEastM);
  put(axes.landingDistanceM(), info->endNorthM, info->endEastM);
  put(ils.localizerX(), info->localizerNorthM, info->localizerEastM);
  put(ils.glideslopeOriginX(), info->gsOriginNorthM, info->gsOriginEastM);
  info->gridCourseDeg = std::atan2(info->endEastM - info->startEastM, info->endNorthM - info->startNorthM) * 57.29577951308232;
  if (info->gridCourseDeg < 0.0) info->gridCourseDeg += 360.0;
  return 1;
}

void a320_fcu_command(A320Sim* sim, A320FcuCommand command) {
  if (sim) sim->sim.fcuCommand(command);
}

int a320_atc_message(const A320Sim* sim, uint32_t seq, A320AtcMessage* out) {
  if (!sim || !out) return 0;
  return sim->sim.atc().message(seq, *out) ? 1 : 0;
}

void a320_atc_get_status(const A320Sim* sim, A320AtcStatus* status) {
  if (!status) return;
  *status = A320AtcStatus{};
  if (!sim) return;
  const a320::Simulation& s = sim->sim;
  const A320State& st = s.state();
  std::snprintf(status->station, sizeof(status->station), "%s", s.atc().stationName(st.com1ActiveKhz).c_str());
  const std::vector<std::string> options = s.atcOptions();
  status->optionCount = static_cast<int>(std::min<size_t>(options.size(), A320_ATC_MAX_OPTIONS));
  for (int i = 0; i < status->optionCount; ++i)
    std::snprintf(status->options[i], sizeof(status->options[i]), "%s", options[static_cast<size_t>(i)].c_str());
  status->awaitingReadback = st.atcAwaitingReadback;
  std::snprintf(status->callsign, sizeof(status->callsign), "%s", s.atc().callsign(s.fms()).c_str());
}

void a320_atc_choose(A320Sim* sim, int option) {
  if (sim) sim->sim.atcChoose(option);
}

void a320_atc_set_enabled(A320Sim* sim, int enabled) {
  if (sim) sim->sim.atc().setEnabled(enabled != 0);
}

void a320_audio_radio_clip(A320Sim* sim, const int16_t* samples, int frames, int sampleRate, int frequencyKhz) {
  if (sim && samples && frames > 0 && sampleRate > 0) sim->sim.audio().radioClip(samples, frames, sampleRate, frequencyKhz);
}

void a320_mcdu_key(A320Sim* sim, int key) {
  if (sim) sim->sim.mcduKey(key);
}

void a320_mcdu_get_display(const A320Sim* sim, A320McduDisplay* display) {
  if (sim && display) sim->sim.mcduDisplay(*display);
}

void a320_fcu_set_targets(A320Sim* sim, double spdKt, double hdgMagDeg, double altFt, double vsFpm) {
  if (sim) sim->sim.setFcuTargets(spdKt, hdgMagDeg, altFt, vsFpm);
}

int a320_route_count(const A320Sim* sim) { return sim ? static_cast<int>(sim->sim.route().points.size()) : 0; }

int a320_get_waypoint(const A320Sim* sim, int index, A320Waypoint* out) {
  if (!sim || !out || index < 0 || index >= a320_route_count(sim)) return 0;
  const a320::World& w = sim->sim.world();
  const std::vector<a320::Waypoint>& pts = sim->sim.route().points;
  const a320::Waypoint& p = pts[static_cast<size_t>(index)];
  *out = A320Waypoint{};
  std::snprintf(out->ident, sizeof(out->ident), "%s", p.ident.c_str());
  out->kind = p.kind;
  out->northM = p.n;
  out->eastM = p.e;
  out->latDeg = p.geo.latDeg;
  out->lonDeg = p.geo.lonDeg;
  out->altFt = p.altFt;
  if (index > 0) {
    // Initial great-circle course from the previous point, magnetic at the nearest airport.
    const a320::Waypoint& q = pts[static_cast<size_t>(index - 1)];
    const double la1 = q.geo.latDeg * a320::kDegToRad, la2 = p.geo.latDeg * a320::kDegToRad;
    const double dLon = (p.geo.lonDeg - q.geo.lonDeg) * a320::kDegToRad;
    const double trueDeg = std::atan2(std::sin(dLon) * std::cos(la2),
                                      std::cos(la1) * std::sin(la2) - std::sin(la1) * std::cos(la2) * std::cos(dLon)) *
                           a320::kRadToDeg;
    const double var = w.airports[static_cast<size_t>(w.nearestAirport(p.n, p.e))].magneticVariationDeg;
    out->legCourseMagDeg = std::fmod(trueDeg - var + 720.0, 360.0);
    out->legNm = std::hypot(p.n - q.n, p.e - q.e) / a320::kNmToM;
  }
  return 1;
}

void a320_fcu_set_fpa(A320Sim* sim, double fpaDeg) {
  if (sim) sim->sim.setFcuFpa(fpaDeg);
}

void a320_set_weather(A320Sim* sim, int weather) {
  if (sim) sim->sim.setWeather(weather);
}

int a320_weather_info(int weather, A320WeatherInfo* out) {
  if (!out || weather < 0 || weather >= A320_WEATHER_COUNT) return 0;
  *out = A320WeatherInfo{};
  switch (weather) {
    case A320_WEATHER_SUNNY:
      out->visibilityM = 30000.0;
      std::snprintf(out->name, sizeof(out->name), "SUNNY");
      break;
    case A320_WEATHER_CLOUDS:
      out->visibilityM = 15000.0;
      out->cloudBaseFt = 2500.0;
      out->cloudTopFt = 4500.0;
      out->cloudCover = 0.65;
      std::snprintf(out->name, sizeof(out->name), "CLOUDS");
      break;
    case A320_WEATHER_RAIN:
      out->visibilityM = 4000.0;
      out->cloudBaseFt = 1200.0;
      out->cloudTopFt = 6000.0;
      out->cloudCover = 1.0;
      out->rain = 1;
      std::snprintf(out->name, sizeof(out->name), "RAIN");
      break;
    default:
      out->visibilityM = 300.0;
      out->fogTopFt = 200.0;
      std::snprintf(out->name, sizeof(out->name), "FOG");
      break;
  }
  return 1;
}

void a320_set_wind(A320Sim* sim, double fromTrueDeg, double kt) {
  if (sim) sim->sim.setWind(fromTrueDeg, kt);
}

const char* a320_lat_mode_name(int latMode) { return a320::latModeName(latMode); }
const char* a320_vert_mode_name(int vertMode) { return a320::vertModeName(vertMode); }
const char* a320_athr_mode_name(int athrMode) { return a320::athrModeName(athrMode); }

int a320_audio_init(A320Sim* sim, int sampleRate, const char* soundsDir) {
  if (!sim) return 0;
  try {
    return sim->sim.audio().init(sampleRate, soundsDir ? soundsDir : "") ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

void a320_audio_render(A320Sim* sim, int16_t* out, int frames) {
  if (sim && out) sim->sim.audio().render(out, frames, sim->sim.state());
}

void a320_audio_event(A320Sim* sim, A320SoundEvent event) {
  if (sim) sim->sim.audio().event(event);
}

void a320_audio_set_volume(A320Sim* sim, double volume) {
  if (sim) sim->sim.audio().setVolume(volume);
}

const char* a320_warning_text(uint32_t warningBit) {
  return a320::warningText(static_cast<a320::Warning>(warningBit));
}

const char* a320_flap_config_name(int flapsLever, int onePlusF) {
  return a320::flapConfigName(flapsLever, onePlusF != 0);
}

const char* a320_thrust_detent_name(int detent) {
  return a320::thrustDetentName(static_cast<a320::ThrustDetent>(detent));
}

}  // extern "C"

int a320_guide_count(void) { return a320::guideCount(); }

const char* a320_guide_name(int guide) {
  const a320::GuideDef* g = a320::guideDef(guide);
  return g ? g->name : "";
}

const char* a320_guide_summary(int guide) {
  const a320::GuideDef* g = a320::guideDef(guide);
  return g ? g->summary : "";
}

A320Scenario a320_guide_scenario(int guide) {
  const a320::GuideDef* g = a320::guideDef(guide);
  return g ? g->scenario : A320_SCENARIO_RUNWAY;
}

const char* a320_guide_runway(int guide) {
  const a320::GuideDef* g = a320::guideDef(guide);
  return g ? g->runway : "";
}

int a320_guide_step_count(int guide) {
  const a320::GuideDef* g = a320::guideDef(guide);
  return g ? g->stepCount : 0;
}

const char* a320_guide_step_text(int guide, int step, A320GuideText field) {
  const a320::GuideDef* g = a320::guideDef(guide);
  if (!g || step < 0 || step >= g->stepCount) return "";
  const a320::GuideStep& st = g->steps[step];
  switch (field) {
    case A320_GUIDE_PHASE: return st.phase;
    case A320_GUIDE_TITLE: return st.title;
    case A320_GUIDE_ACTION: return st.action;
    case A320_GUIDE_LOOK: return st.look;
    case A320_GUIDE_WHY: return st.why;
    case A320_GUIDE_MSFS: return st.msfs;
  }
  return "";
}

void a320_guide_start(A320Sim* sim, int guide) {
  if (!sim) return;
  sim->guide.start(guide);
  sim->guide.update(sim->sim.state(), sim->sim.controls());
}

void a320_guide_stop(A320Sim* sim) {
  if (sim) sim->guide.stop();
}

void a320_guide_next(A320Sim* sim) {
  if (!sim) return;
  sim->guide.next();
  sim->guide.update(sim->sim.state(), sim->sim.controls());
}

void a320_guide_back(A320Sim* sim) {
  if (sim) sim->guide.back();
}

void a320_guide_get_status(const A320Sim* sim, A320GuideStatus* status) {
  if (!status) return;
  *status = sim ? sim->guide.status() : A320GuideStatus{};
}

const char* a320_guide_alert(const A320Sim* sim) { return sim ? sim->guide.alert() : ""; }

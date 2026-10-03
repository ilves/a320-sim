#include <cstdio>
#include <exception>
#include <new>
#include <string>

#include "a320/Simulation.h"
#include "a320/Systems.h"
#include "a320/a320_api.h"

struct A320Sim {
  a320::Simulation sim{a320::makeTallinn()};
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

const char* a320_airport_icao(const A320Sim* sim) { return sim ? sim->sim.airport().icao.c_str() : ""; }

double a320_field_elevation_ft(const A320Sim* sim) {
  return sim ? sim->sim.airport().reference.altM / 0.3048 : 0.0;
}

double a320_magnetic_variation_deg(const A320Sim* sim) {
  return sim ? sim->sim.airport().magneticVariationDeg : 0.0;
}

int a320_runway_count(const A320Sim* sim) {
  return sim ? static_cast<int>(sim->sim.airport().runways.size()) : 0;
}

int a320_get_runway(const A320Sim* sim, int index, A320RunwayInfo* info) {
  if (!sim || !info || index < 0 || index >= a320_runway_count(sim)) return 0;
  const a320::Simulation& s = sim->sim;
  const a320::Runway& rw = s.airport().runways[static_cast<size_t>(index)];
  const a320::Ils ils(s.frame(), rw);
  const a320::RunwayAxes& axes = ils.axes();

  *info = A320RunwayInfo{};
  std::snprintf(info->ident, sizeof(info->ident), "%s", rw.ident.c_str());
  info->trueCourseDeg = rw.trueCourseDeg;
  info->widthM = rw.widthM;
  info->landingDistanceM = axes.landingDistanceM();
  info->displacementM = axes.displacementM();
  info->glideslopeDeg = ils.glideslopeDeg();

  auto put = [&](double x, double& north, double& east) {
    const a320::Enu p = axes.toEnu({x, 0.0, 0.0});
    north = p.n;
    east = p.e;
  };
  put(-axes.displacementM(), info->startNorthM, info->startEastM);
  put(0.0, info->thresholdNorthM, info->thresholdEastM);
  put(axes.landingDistanceM(), info->endNorthM, info->endEastM);
  put(ils.localizerX(), info->localizerNorthM, info->localizerEastM);
  put(ils.glideslopeOriginX(), info->gsOriginNorthM, info->gsOriginEastM);
  return 1;
}

void a320_fcu_command(A320Sim* sim, A320FcuCommand command) {
  if (sim) sim->sim.fcuCommand(command);
}

void a320_fcu_set_targets(A320Sim* sim, double spdKt, double hdgMagDeg, double altFt, double vsFpm) {
  if (sim) sim->sim.setFcuTargets(spdKt, hdgMagDeg, altFt, vsFpm);
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

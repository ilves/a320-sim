// Cold-and-dark engine start, APU, ground spoilers and autobrake on the JSBSim A320.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>

#include "Check.h"
#include "a320/Systems.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

struct Sys {
  A320Sim* sim = nullptr;
  A320Controls c{};
  A320State s{};
  explicit Sys(A320Scenario scenario) {
    char err[256] = {0};
    sim = a320_create(A320_DATA_DIR, err, sizeof(err));
    if (!sim) return;
    a320_reset(sim, scenario, 1);
    a320_get_state(sim, &s);
    a320_get_controls(sim, &c);
  }
  ~Sys() { a320_destroy(sim); }
  void fly(double seconds, const std::function<bool()>& f = [] { return true; }) {
    for (double t = 0; t < seconds; t += kDt) {
      if (!f()) return;
      a320_set_controls(sim, &c);
      a320_update(sim, kDt);
      a320_get_state(sim, &s);
    }
  }
};

}  // namespace

TEST(apu_unit) {
  a320::Apu apu;
  apu.update(true, true, 0.1);
  for (int i = 0; i < 400; ++i) apu.update(true, false, 0.1);
  CHECK(apu.avail());
  for (int i = 0; i < 300; ++i) apu.update(false, false, 0.1);
  CHECK(apu.n() < 1.0);
  a320::Apu noMaster;
  for (int i = 0; i < 100; ++i) noMaster.update(false, true, 0.1);
  CHECK(noMaster.n() == 0.0);
}

TEST(cold_and_dark_engine_start) {
  Sys f(A320_SCENARIO_COLD_DARK);
  if (!f.sim) { CHECK(false); return; }
  f.fly(2.0);
  CHECK(f.s.engRunning[0] == 0 && f.s.engRunning[1] == 0);
  CHECK(f.s.n2[0] < 1.0);

  // Without bleed air the starter does nothing.
  f.c.engMode = A320_ENG_MODE_IGN_START;
  f.c.engMaster[0] = 1;
  f.fly(10.0);
  CHECK(f.s.n2[0] < 1.0);
  CHECK(f.s.bleedAvailable == 0);

  // APU: master, start, bleed.
  f.c.engMaster[0] = 0;
  f.c.apuMaster = 1;
  f.fly(3.0, [&] { f.c.apuStart = 1; return true; });
  f.c.apuStart = 0;
  double apuAvailAt = -1.0, t = 0.0;
  f.fly(60.0, [&] { t += kDt; if (apuAvailAt < 0 && f.s.apuAvail) apuAvailAt = t; return true; });
  std::printf("  APU available after %.0f s\n", apuAvailAt + 3.0);
  CHECK(apuAvailAt > 15.0 && apuAvailAt < 45.0);
  f.c.apuBleed = 1;
  f.fly(1.0);
  CHECK(f.s.bleedAvailable == 1);

  // Engine 1: master on with IGN/START. N2 spools on the starter, then lights at ~20 %.
  f.c.engMaster[0] = 1;
  double running1 = -1.0, maxN2Starter = 0.0;
  t = 0.0;
  f.fly(90.0, [&] {
    t += kDt;
    if (f.s.engStarting[0]) maxN2Starter = std::fmax(maxN2Starter, f.s.n2[0]);
    if (running1 < 0 && f.s.engRunning[0]) running1 = t;
    return true;
  });
  std::printf("  engine 1 running after %.0f s, N1 %.1f N2 %.1f FF %.0f kg/h\n", running1, f.s.n1[0], f.s.n2[0], f.s.fuelFlowKgH[0]);
  CHECK(running1 > 10.0 && running1 < 80.0);
  CHECK(f.s.n1[0] > 15.0 && f.s.n2[0] > 50.0);
  CHECK(f.s.fuelFlowKgH[0] > 50.0);
  CHECK(f.s.engRunning[1] == 0);

  // Engine 2 can now also start from engine 1's bleed.
  f.c.apuBleed = 0;
  f.c.engMaster[1] = 1;
  f.fly(90.0);
  CHECK(f.s.engRunning[1] == 1);

  // Master off: fuel cut, spool down.
  f.c.engMaster[0] = 0;
  f.fly(60.0);
  std::printf("  engine 1 after shutdown: N2 %.1f, running %d\n", f.s.n2[0], f.s.engRunning[0]);
  CHECK(f.s.engRunning[0] == 0);
  CHECK(f.s.n2[0] < 15.0);
}

static double landingRollM(int autobrake, bool armSpoilers, double* avgDecel, int* spoilersOut) {
  Sys f(A320_SCENARIO_FINAL_4NM);
  if (!f.sim) return -1.0;
  f.c.autobrake = autobrake;
  f.c.spoilersArmed = armSpoilers ? 1 : 0;
  A320RunwayInfo rw;
  a320_get_runway(f.sim, f.s.ilsRunwayIndex, &rw);
  double tdGs = 0.0, rollStartX = 0.0, decelSum = 0.0;
  int decelN = 0;
  bool touched = false;
  const double c = rw.trueCourseDeg * 3.14159265358979 / 180.0;
  auto along = [&] {
    const double dn = f.s.northM - rw.thresholdNorthM, de = f.s.eastM - rw.thresholdEastM;
    return de * std::sin(c) + dn * std::cos(c);
  };
  double lastGs = 0.0, stopX = 0.0;
  *spoilersOut = 0;
  a320_fcu_command(f.sim, A320_FCU_APPR);
  a320_fcu_command(f.sim, A320_FCU_AP1);
  f.fly(400.0, [&] {
    if (!f.s.onGround && f.s.radioAltFt < 25.0) f.c.thrustLever = 0.0;
    if (f.s.touchdownSeq > 0 && !touched) {
      touched = true;
      tdGs = f.s.groundSpeedKt;
      rollStartX = along();
    }
    if (touched) {
      if (f.s.groundSpoilers) *spoilersOut = 1;
      if (f.s.groundSpeedKt < tdGs - 15.0 && f.s.groundSpeedKt > 40.0) {
        decelSum += (lastGs - f.s.groundSpeedKt) * 0.514444 / kDt;
        ++decelN;
      }
      lastGs = f.s.groundSpeedKt;
      if (f.s.groundSpeedKt < 1.0) {
        stopX = along();
        return false;
      }
    } else {
      lastGs = f.s.groundSpeedKt;
    }
    return true;
  });
  *avgDecel = decelN ? decelSum / decelN : 0.0;
  return touched ? stopX - rollStartX : -1.0;
}

TEST(ground_spoilers_and_autobrake) {
  double decelLo = 0, decelMed = 0, decelNone = 0;
  int outLo = 0, outMed = 0, outNone = 0;
  const double rollLo = landingRollM(A320_AUTOBRAKE_LO, true, &decelLo, &outLo);
  const double rollMed = landingRollM(A320_AUTOBRAKE_MED, true, &decelMed, &outMed);
  const double rollNone = landingRollM(A320_AUTOBRAKE_OFF, false, &decelNone, &outNone);
  std::printf("  roll: LO %.0f m (%.2f m/s2), MED %.0f m (%.2f m/s2), unarmed no brakes %.0f m (spoilers %d)\n",
              rollLo, decelLo, rollMed, decelMed, rollNone, outNone);
  CHECK(outLo == 1 && outMed == 1);
  CHECK(outNone == 0);
  CHECK_NEAR(decelLo, 1.7, 0.5);
  CHECK_NEAR(decelMed, 3.0, 0.6);
  CHECK(rollMed < rollLo);
  CHECK(rollLo < rollNone);
}

TEST(autobrake_disarms_on_pilot_braking) {
  a320::GroundDecel d;
  a320::GroundDecelInput in;
  in.onGround = true;
  in.armed = true;
  in.autobrake = A320_AUTOBRAKE_MED;
  in.groundSpeedKt = 130.0;
  d.update(in);
  CHECK(d.spoilersOut());
  CHECK(d.autobrakeActive());
  in.pilotBrake = 0.9;
  d.update(in);
  CHECK(!d.autobrakeActive());
  in.pilotBrake = 0.0;
  d.update(in);  // the selector still says MED, but a disarm sticks
  CHECK(!d.autobrakeActive());
  in.thrustLever = 0.5;  // advancing thrust retracts the ground spoilers
  d.update(in);
  CHECK(!d.spoilersOut());
}

TEST(thrust_levers_unit) {
  A320Controls c{};
  c.thrustLever = 0.75;
  c.thrustLever2 = 0.2;
  a320::ThrustLevers t = a320::thrustLevers(c);
  CHECK(t.lever[1] == 0.75);  // not split: engine 2 follows lever 1
  c.splitThrust = 1;
  t = a320::thrustLevers(c);
  CHECK(t.lever[0] == 0.75 && t.lever[1] == 0.2);
  CHECK_NEAR(t.forward(), 0.75, 1e-9);
  c.reverse = 1;
  t = a320::thrustLevers(c);
  CHECK_NEAR(t.forward(), 0.2, 1e-9);  // a lever in reverse counts as idle
}

TEST(split_thrust_levers) {
  Sys f(A320_SCENARIO_RUNWAY);
  if (!f.sim) { CHECK(false); return; }
  f.c.splitThrust = 1;
  f.c.thrustLever = 0.75;
  f.c.thrustLever2 = 0.0;
  f.fly(15.0);
  CHECK(f.s.n1[0] > f.s.n1[1] + 20.0);
  CHECK(f.s.thrustLeverEng[0] == 0.75 && f.s.thrustLeverEng[1] == 0.0);
  CHECK(f.s.thrustDetent == 1);  // CL, from the most advanced lever

  // Reverse on engine 2 only, on the ground.
  f.c.thrustLever = 0.0;
  f.c.reverse2 = 1;
  f.c.thrustLever2 = 1.0;
  f.fly(10.0);
  CHECK(f.s.reverseEng[0] == 0 && f.s.reverseEng[1] == 1 && f.s.reverse == 1);
  CHECK(f.s.n1[1] > f.s.n1[0] + 20.0);
}

TEST(reverse_selected_in_flight_gives_idle) {
  Sys f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.c.reverse = 1;
  f.c.thrustLever = 1.0;
  f.fly(10.0);
  CHECK(f.s.reverse == 0);
  CHECK(f.s.thrustLeverEng[0] == 0.0 && f.s.thrustDetent == 0);
  CHECK(f.s.n1[0] < 40.0 && f.s.n1[1] < 40.0);
}

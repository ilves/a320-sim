#pragma once

#include <string>

#include "a320/Airport.h"
#include "a320/Fms.h"
#include "a320/Geo.h"
#include "a320/Route.h"
#include "a320/a320_api.h"

namespace a320 {

// Speeds the PERF pages show, from the same CLmax model as VLS.
struct ConfigSpeeds {
  double fKt = 0.0;         // F: flaps retraction / CONF 2-3 approach target
  double sKt = 0.0;         // S: slats retraction / CONF 1 target
  double greenDotKt = 0.0;  // O: best lift-to-drag, clean
  double vlsFullKt = 0.0, vls3Kt = 0.0;
};
ConfigSpeeds computeConfigSpeeds(double weightLbs);

// VAPP: the entered value, else VLS of the landing configuration plus a third of the headwind
// component (5 to 15 kt).
double computeVapp(const Fms& fms, const ConfigSpeeds& speeds, const World& world);

struct McduContext {
  const World& world;
  const LocalFrame& frame;
  const A320State& state;
  Fms& fms;
  double weightLbs;
  const Route& route;
  int routeActive;  // the TO waypoint, -1 = none
};

// The MCDU pages for one airport: INIT, F-PLN with lateral revisions, DEPARTURE and ARRIVAL,
// RAD NAV, PERF TAKE OFF / APPR, PROG and the MCDU MENU. Entries go through the scratchpad and
// the line select keys as on the aircraft; the crew's data lives in Fms.
class Mcdu {
 public:
  void reset();
  void press(int key, McduContext& ctx);
  void render(const McduContext& ctx, A320McduDisplay& out) const;
  const std::string& scratchpad() const { return scratch_; }

 private:
  enum class Page { Init, Fpln, LatRevOrigin, LatRevDest, Departure, Arrival, RadNav, PerfTakeoff, PerfAppr, Prog, Menu };

  void lineSelect(int line, bool right, McduContext& ctx);
  void lineSelectInit(int line, bool right, McduContext& ctx);
  void lineSelectRunways(int line, bool right, McduContext& ctx);
  void lineSelectRadNav(int line, bool right, McduContext& ctx);
  void lineSelectTakeoff(int line, bool right, McduContext& ctx);
  void lineSelectAppr(int line, bool right, McduContext& ctx);
  void typeChar(char c);
  void show(const char* message) { message_ = message; }
  void accepted() { scratch_.clear(); }

  Page page_ = Page::Fpln;
  std::string scratch_;
  std::string message_;
  int tmpyDep_ = -1, tmpyArr_ = -1;
  int fplnScroll_ = 0;  // F-PLN: waypoints scrolled past the FROM line (slew keys)
  // The route's waypoint on an F-PLN line (0-4), or -1.
  int fplnIndex(const McduContext& ctx, int line) const;
};

}  // namespace a320

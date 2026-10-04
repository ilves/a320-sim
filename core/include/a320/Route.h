#pragma once

#include <string>
#include <vector>

#include "a320/Airport.h"
#include "a320/Fms.h"
#include "a320/Geo.h"
#include "a320/a320_api.h"

namespace a320 {

struct Waypoint {
  std::string ident;
  int kind = A320_WPT_FIX;  // A320WaypointKind
  double n = 0.0, e = 0.0;  // the flat world (metres)
  GeoPos geo;
  int altFt = 0;            // constraint (approach), or the altitude an A320_WPT_ALTITUDE leg climbs to
};

// The lateral flight plan from the MCDU's FROM/TO and runways: the departure runway, a climb on
// its track, the airport's FRA departure point, FRA points along the way, its arrival point and a
// generated ILS transition (CF, FF, RW as an Airbus database codes unnamed approach fixes).
struct Route {
  std::vector<Waypoint> points;
  bool hasDeparture = false;  // starts on the departure runway (NAV is armed for the takeoff)
  bool empty() const { return points.size() < 2; }
};

struct RouteRequest {
  const World& world;
  const LocalFrame& frame;  // the flat world's
  const Fms& fms;
  bool onGround = true;
  double northM = 0.0, eastM = 0.0;  // the aircraft, for a plan made in the air
};

// Empty without an arrival runway, or for a circuit (the same airport, on the ground).
Route buildRoute(const RouteRequest& request);

// Follows a Route: the active leg, its sequencing (fly-by turns with anticipation, the climb
// leg ends at its altitude, then direct to the next fix), and the track to fly.
class Lnav {
 public:
  struct Aircraft {
    double northM = 0.0, eastM = 0.0, altitudeFt = 0.0;
    double gridTrackDeg = 0.0, groundSpeedKt = 0.0;
    bool onGround = true;
  };

  void reset(const Route& route);
  // Picks the leg to fly from where the aircraft is: the nearest leg not yet flown, or direct to
  // its fix when far from it (an engagement or a new plan in the air).
  void resync(const Route& route, const Aircraft& a);
  void update(const Route& route, const Aircraft& a);

  bool valid() const { return valid_; }
  int active() const { return active_; }        // the TO waypoint
  double desiredGridTrackDeg() const { return desiredDeg_; }
  double crossTrackM() const { return xtkM_; }  // right of the leg positive
  double toDistanceM() const { return toDistM_; }
  double toBearingGridDeg() const { return toBrgDeg_; }
  double remainingM(const Route& route) const;  // along the route to its last point

 private:
  void startLeg(const Route& route, int to, double fromN, double fromE, bool direct = false);
  void guide(const Route& route, const Aircraft& a);

  int active_ = -1;
  double fromN_ = 0.0, fromE_ = 0.0;
  bool valid_ = false;
  bool direct_ = false;  // a direct-to: straight at the fix until the track points there
  double desiredDeg_ = 0.0, xtkM_ = 0.0, toDistM_ = 0.0, toBrgDeg_ = 0.0;
};

}  // namespace a320

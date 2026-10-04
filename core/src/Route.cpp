#include "a320/Route.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "a320/NavData.h"
#include "a320/Units.h"

namespace a320 {
namespace {

struct P2 {
  double n = 0.0, e = 0.0;
};

double dist(const P2& a, const P2& b) { return std::hypot(b.n - a.n, b.e - a.e); }
double courseDeg(const P2& a, const P2& b) { return std::atan2(b.e - a.e, b.n - a.n) * kRadToDeg; }

double wrap180(double d) {
  d = std::fmod(d, 360.0);
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

double wrap360(double d) {
  d = std::fmod(d, 360.0);
  return d < 0.0 ? d + 360.0 : d;
}

bool listed(const char* airports, const std::string& icao) {
  // Space-separated ICAO codes.
  for (const char* p = std::strstr(airports, icao.c_str()); p; p = std::strstr(p + 1, icao.c_str()))
    if ((p == airports || p[-1] == ' ') && (p[icao.size()] == '\0' || p[icao.size()] == ' ')) return true;
  return false;
}

P2 flat(const LocalFrame& frame, const GeoPos& g) {
  const Enu p = frame.toEnu(g);
  return {p.n, p.e};
}

struct Builder {
  const RouteRequest& r;
  Route route;

  void add(const std::string& ident, int kind, const P2& p, int altFt = 0) {
    Waypoint w;
    w.ident = ident.substr(0, 7);
    w.kind = kind;
    w.n = p.n;
    w.e = p.e;
    w.geo = r.frame.toGeo(Enu{p.e, p.n, 0.0});
    w.altFt = altFt;
    route.points.push_back(w);
  }
  P2 last() const { return {route.points.back().n, route.points.back().e}; }

  // The cheapest transition point of an airport (from -> fix -> to), unless it is a detour.
  const NavFix* transition(const std::string& icao, bool departure, const P2& from, const P2& to, P2& at) const {
    const NavFix* best = nullptr;
    double bestCost = std::numeric_limits<double>::max();
    for (const NavFix& f : estonianFixes()) {
      if (!listed(departure ? f.depFor : f.arrFor, icao)) continue;
      const P2 p = flat(r.frame, GeoPos{f.latDeg, f.lonDeg, 0.0});
      const double cost = dist(from, p) + dist(p, to);
      if (cost < bestCost) {
        bestCost = cost;
        best = &f;
        at = p;
      }
    }
    // Some 20% longer than direct is what a transition may cost; more and the flight goes direct.
    if (!best || bestCost > 1.2 * dist(from, to) + 5.0 * kNmToM || dist(from, at) < 3.0 * kNmToM) return nullptr;
    return best;
  }

  // Free route points within 8 NM of the straight line from a to b, at least 25 NM apart, in order.
  void intermediates(const P2& a, const P2& b) {
    const double len = dist(a, b);
    if (len < 40.0 * kNmToM) return;
    const double un = (b.n - a.n) / len, ue = (b.e - a.e) / len;
    struct Cand {
      double along;
      const NavFix* fix;
      P2 p;
    };
    std::vector<Cand> cands;
    for (const NavFix& f : estonianFixes()) {
      if (!f.intermediate) continue;
      const P2 p = flat(r.frame, GeoPos{f.latDeg, f.lonDeg, 0.0});
      const double along = (p.n - a.n) * un + (p.e - a.e) * ue;
      const double across = (p.e - a.e) * un - (p.n - a.n) * ue;
      if (along < 20.0 * kNmToM || along > len - 20.0 * kNmToM || std::fabs(across) > 8.0 * kNmToM) continue;
      cands.push_back({along, &f, p});
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.along < y.along; });
    double lastAlong = 0.0;
    for (const Cand& c : cands) {
      if (c.along - lastAlong < 25.0 * kNmToM) continue;
      add(c.fix->ident, A320_WPT_FIX, c.p);
      lastAlong = c.along;
    }
  }
};

}  // namespace

Route buildRoute(const RouteRequest& r) {
  Builder b{r, Route{}};
  const World& world = r.world;
  const Fms& f = r.fms;
  if (f.arrRunway < 0 || f.arrRunway >= static_cast<int>(world.runways.size())) return {};
  const Runway& arr = world.runways[static_cast<size_t>(f.arrRunway)];
  const Airport& dest = world.airports[static_cast<size_t>(arr.airport)];
  const bool depart = r.onGround && f.depRunway >= 0 && f.depRunway < static_cast<int>(world.runways.size()) &&
                      world.runways[static_cast<size_t>(f.depRunway)].airport != arr.airport;
  if (r.onGround && !depart) return {};  // a circuit: no route

  // The arrival runway in the flat world: threshold, landing direction u, right of it rr.
  const P2 thr = flat(r.frame, arr.threshold);
  const P2 arrStart = flat(r.frame, arr.start), arrEnd = flat(r.frame, arr.end);
  const double arrLen = dist(arrStart, arrEnd);
  const P2 u{(arrEnd.n - arrStart.n) / arrLen, (arrEnd.e - arrStart.e) / arrLen};
  const P2 rr{-u.e, u.n};  // right of the landing direction, as (n, e)
  auto onFinal = [&](double x, double y) { return P2{thr.n + u.n * x + rr.n * y, thr.e + u.e * x + rr.e * y}; };
  // The intercept altitude 2000 ft above the threshold; FF where the glide path meets it, CF 4 NM before.
  const double gs = arr.ils.ident.empty() ? 3.0 : arr.ils.glideslopeDeg;
  const double thrFt = arr.threshold.altM * kMToFt;
  const int interceptFt = static_cast<int>(std::ceil((thrFt + 2000.0) / 100.0)) * 100;
  const double dFf = (interceptFt - thrFt) * kFtToM / std::tan(gs * kDegToRad);
  const double dCf = dFf + 4.0 * kNmToM;
  const P2 cf = onFinal(-dCf, 0.0);

  if (depart) {
    const Runway& dep = world.runways[static_cast<size_t>(f.depRunway)];
    const Airport& origin = world.airports[static_cast<size_t>(dep.airport)];
    const P2 s = flat(r.frame, dep.start), e = flat(r.frame, dep.end);
    const double len = dist(s, e);
    b.add(origin.icao + dep.ident, A320_WPT_RUNWAY, s, static_cast<int>(std::lround(origin.reference.altM * kMToFt)));
    // A climb on the runway track to 1500 ft above the field, then direct to the first fix.
    const int climbFt = static_cast<int>(std::lround((origin.reference.altM * kMToFt + 1500.0) / 10.0)) * 10;
    const double past = len + 1.5 * kNmToM;
    b.add("(" + std::to_string(climbFt) + ")", A320_WPT_ALTITUDE,
          P2{s.n + (e.n - s.n) / len * past, s.e + (e.e - s.e) / len * past}, climbFt);
    b.route.hasDeparture = true;
    P2 at;
    if (const NavFix* fix = b.transition(origin.icao, true, b.last(), cf, at)) b.add(fix->ident, A320_WPT_FIX, at);
  } else {
    b.add("PPOS", A320_WPT_POSITION, P2{r.northM, r.eastM});
  }

  // On the final course already (a final approach start): only what is still ahead of it.
  {
    const P2 p = b.last();
    const double x = (p.n - thr.n) * u.n + (p.e - thr.e) * u.e;
    const double y = (p.n - thr.n) * rr.n + (p.e - thr.e) * rr.e;
    if (!depart && std::fabs(y) < 1.5 * kNmToM && x > -(dCf + 2.0 * kNmToM) && x < 0.0) {
      if (x < -dCf - 1.0 * kNmToM) b.add("CF" + arr.ident, A320_WPT_APPROACH, cf, interceptFt);
      if (x < -dFf - 1.0 * kNmToM) b.add("FF" + arr.ident, A320_WPT_APPROACH, onFinal(-dFf, 0.0), interceptFt);
      b.add("RW" + arr.ident, A320_WPT_RUNWAY, thr, static_cast<int>(std::lround(thrFt)));
      return b.route;
    }
  }

  P2 arrivalAt;
  const NavFix* arrival = b.transition(dest.icao, false, b.last(), cf, arrivalAt);
  b.intermediates(b.last(), arrival ? arrivalAt : cf);
  if (arrival) b.add(arrival->ident, A320_WPT_FIX, arrivalAt);

  // Onto the final course: straight to CF from the approach side (a turn of 90 degrees at most),
  // else a downwind on the side the flight comes from and a base turn.
  const P2 p = b.last();
  const double x = (p.n - thr.n) * u.n + (p.e - thr.e) * u.e;
  const double y = (p.n - thr.n) * rr.n + (p.e - thr.e) * rr.e;
  if (x > -(dCf + 2.0 * kNmToM)) {
    const double side = y >= 0.0 ? 1.0 : -1.0;
    if (x > -(dCf - 2.0 * kNmToM)) b.add("DW" + arr.ident, A320_WPT_APPROACH, onFinal(std::min(x, 0.0), side * 5.0 * kNmToM));
    b.add("BS" + arr.ident, A320_WPT_APPROACH, onFinal(-(dCf + 3.0 * kNmToM), side * 5.0 * kNmToM));
  }
  b.add("CF" + arr.ident, A320_WPT_APPROACH, cf, interceptFt);
  b.add("FF" + arr.ident, A320_WPT_APPROACH, onFinal(-dFf, 0.0), interceptFt);
  b.add("RW" + arr.ident, A320_WPT_RUNWAY, thr, static_cast<int>(std::lround(thrFt)));
  return b.route;
}

void Lnav::reset(const Route& route) {
  valid_ = !route.empty();
  active_ = valid_ ? 1 : -1;
  if (valid_) startLeg(route, 1, route.points[0].n, route.points[0].e);
}

void Lnav::startLeg(const Route& route, int to, double fromN, double fromE, bool direct) {
  active_ = to;
  direct_ = direct;
  fromN_ = fromN;
  fromE_ = fromE;
  valid_ = to >= 1 && to < static_cast<int>(route.points.size());
}

void Lnav::resync(const Route& route, const Aircraft& a) {
  if (route.empty()) {
    valid_ = false;
    active_ = -1;
    return;
  }
  const P2 at{a.northM, a.eastM};
  int best = static_cast<int>(route.points.size()) - 1;
  double bestDist = std::numeric_limits<double>::max();
  for (size_t i = 1; i < route.points.size(); ++i) {
    const Waypoint& to = route.points[i];
    if (to.kind == A320_WPT_ALTITUDE && (!a.onGround && a.altitudeFt >= to.altFt)) continue;
    const P2 p0{route.points[i - 1].n, route.points[i - 1].e}, p1{to.n, to.e};
    const double len = std::max(dist(p0, p1), 1.0);
    const double t = ((at.n - p0.n) * (p1.n - p0.n) + (at.e - p0.e) * (p1.e - p0.e)) / (len * len);
    if (t >= 1.0) continue;  // flown
    const double d = t <= 0.0 ? dist(at, p0) : std::fabs(((at.e - p0.e) * (p1.n - p0.n) - (at.n - p0.n) * (p1.e - p0.e)) / len);
    if (d < bestDist) {
      bestDist = d;
      best = static_cast<int>(i);
    }
  }
  // Near the leg: fly it; far from it: direct to its fix.
  if (bestDist < 3.0 * kNmToM) startLeg(route, best, route.points[static_cast<size_t>(best - 1)].n, route.points[static_cast<size_t>(best - 1)].e);
  else startLeg(route, best, a.northM, a.eastM, true);
  update(route, a);
}

void Lnav::update(const Route& route, const Aircraft& a) {
  if (!valid_ || active_ >= static_cast<int>(route.points.size())) {
    valid_ = false;
    return;
  }
  const Waypoint& to = route.points[static_cast<size_t>(active_)];
  const bool hasNext = active_ + 1 < static_cast<int>(route.points.size());
  if (to.kind == A320_WPT_ALTITUDE) {
    // The climb leg ends at its altitude, wherever that is: then direct to the next fix.
    if (!a.onGround && a.altitudeFt >= to.altFt && hasNext) startLeg(route, active_ + 1, a.northM, a.eastM, true);
  } else if (!a.onGround && hasNext) {
    const P2 from{fromN_, fromE_}, p1{to.n, to.e}, next{route.points[static_cast<size_t>(active_ + 1)].n,
                                                        route.points[static_cast<size_t>(active_ + 1)].e};
    const double len = std::max(dist(from, p1), 1.0);
    const double along = ((a.northM - from.n) * (p1.n - from.n) + (a.eastM - from.e) * (p1.e - from.e)) / len;
    const double xtk = ((a.eastM - from.e) * (p1.n - from.n) - (a.northM - from.n) * (p1.e - from.e)) / len;
    // Fly-by: the turn starts its radius times tan(half the course change) before the fix.
    const double turn = std::min(std::fabs(wrap180(courseDeg(p1, next) - courseDeg(from, p1))), 150.0);
    const double gsMps = std::max(a.groundSpeedKt, 140.0) * kKtToMps;
    const double radius = gsMps * gsMps / (9.81 * std::tan(25.0 * kDegToRad));
    const double lead = std::min(radius * std::tan(turn / 2.0 * kDegToRad), 7.0 * kNmToM);
    if (len - along <= lead && std::fabs(xtk) < 7.0 * kNmToM) startLeg(route, active_ + 1, to.n, to.e);
  }
  guide(route, a);
}

void Lnav::guide(const Route& route, const Aircraft& a) {
  const Waypoint& to = route.points[static_cast<size_t>(active_)];
  const P2 from{fromN_, fromE_}, p1{to.n, to.e}, at{a.northM, a.eastM};
  const double len = dist(from, p1);
  toDistM_ = dist(at, p1);
  toBrgDeg_ = wrap360(courseDeg(at, p1));
  if (len < 1.0) {
    xtkM_ = 0.0;
    desiredDeg_ = toBrgDeg_;
    return;
  }
  // A direct-to becomes a leg from where the turn towards the fix ends.
  if (direct_) {
    xtkM_ = 0.0;
    desiredDeg_ = toBrgDeg_;
    if (std::fabs(wrap180(a.gridTrackDeg - toBrgDeg_)) < 3.0 || a.onGround) {
      direct_ = false;
      fromN_ = a.northM;
      fromE_ = a.eastM;
    }
    return;
  }
  const double course = courseDeg(from, p1);
  xtkM_ = ((at.e - from.e) * (p1.n - from.n) - (at.n - from.n) * (p1.e - from.e)) / len;
  // Back onto the leg: up to 45 degrees of intercept, 45 at 1.8 km off.
  desiredDeg_ = wrap360(course + clamp(-xtkM_ / 40.0, -45.0, 45.0));
}

double Lnav::alongToM(const Route& route, int index, double groundSpeedKt) const {
  if (!valid_ || index < active_) return 0.0;
  double m = toDistM_;
  const double gsMps = std::max(groundSpeedKt, 140.0) * kKtToMps;
  const double radius = gsMps * gsMps / (9.81 * std::tan(25.0 * kDegToRad));
  for (size_t i = static_cast<size_t>(active_) + 1; i <= static_cast<size_t>(index) && i < route.points.size(); ++i) {
    const P2 a{route.points[i - 1].n, route.points[i - 1].e}, b{route.points[i].n, route.points[i].e};
    m += dist(a, b);
    // The turn at the waypoint before this leg: two leads saved, the arc flown instead.
    const P2 before = i >= 2 && static_cast<int>(i) - 1 > active_ ? P2{route.points[i - 2].n, route.points[i - 2].e} : P2{fromN_, fromE_};
    const double turn = std::min(std::fabs(wrap180(courseDeg(a, b) - courseDeg(before, a))), 150.0) * kDegToRad;
    const double lead = std::min(radius * std::tan(turn / 2.0), 7.0 * kNmToM);
    m -= 2.0 * lead - lead / std::max(std::tan(turn / 2.0), 1e-6) * turn;
  }
  return m;
}

bool Lnav::pointAlong(const Route& route, const Aircraft& a, double distM, double& northM, double& eastM) const {
  if (!valid_) return false;
  P2 from{a.northM, a.eastM};
  for (size_t i = static_cast<size_t>(active_); i < route.points.size(); ++i) {
    const P2 to{route.points[i].n, route.points[i].e};
    const double len = dist(from, to);
    if (distM <= len && len > 0.0) {
      northM = from.n + (to.n - from.n) * distM / len;
      eastM = from.e + (to.e - from.e) * distM / len;
      return true;
    }
    distM -= len;
    from = to;
  }
  return false;
}

double Lnav::remainingM(const Route& route) const {
  if (!valid_) return 0.0;
  double m = toDistM_;
  for (size_t i = static_cast<size_t>(active_) + 1; i < route.points.size(); ++i)
    m += std::hypot(route.points[i].n - route.points[i - 1].n, route.points[i].e - route.points[i - 1].e);
  return m;
}

}  // namespace a320

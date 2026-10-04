#include "Check.h"
#include "a320/Airport.h"
#include "a320/Ils.h"
#include "a320/Units.h"

using namespace a320;

TEST(geo_roundtrip) {
  const LocalFrame f({59.4, 24.8, 40.0});
  const GeoPos p{59.45, 24.95, 900.0};
  const GeoPos back = f.toGeo(f.toEnu(p));
  CHECK_NEAR(back.latDeg, p.latDeg, 1e-9);
  CHECK_NEAR(back.lonDeg, p.lonDeg, 1e-9);
  CHECK_NEAR(back.altM, p.altM, 1e-4);
  // One minute of latitude is about one nautical mile.
  const Enu n = f.toEnu({59.4 + 1.0 / 60.0, 24.8, 40.0});
  CHECK_NEAR(n.n, 1855.0, 5.0);
  CHECK_NEAR(n.e, 0.0, 0.01);
}

TEST(tallinn_runway_geometry) {
  const Airport a = makeTallinn();
  CHECK(a.runways.size() == 2);
  const Runway* r26 = a.find("26");
  const Runway* r08 = a.find("08");
  CHECK(r26 && r08);
  if (!r26 || !r08) return;
  const LocalFrame f(a.reference);
  const RunwayAxes ax26(f, *r26), ax08(f, *r08);
  // 11417 ft paved; 08 threshold displaced 820 ft.
  CHECK_NEAR(ax26.landingDistanceM(), 11417 * kFtToM, 15.0);
  CHECK_NEAR(ax26.displacementM(), 0.0, 0.01);
  CHECK_NEAR(ax08.displacementM(), 820 * kFtToM, 0.5);
  CHECK_NEAR(ax08.landingDistanceM(), (11417 - 820) * kFtToM, 15.0);
  // Runway 08/26 is about 81 magnetic with ~9 degrees east variation: ~90 true.
  CHECK_NEAR(r08->trueCourseDeg, 90.2, 0.5);
  CHECK_NEAR(r26->trueCourseDeg, 270.2, 0.5);
}

TEST(runway_axes_roundtrip_and_sides) {
  const Airport a = makeTallinn();
  const LocalFrame f(a.reference);
  const RunwayAxes ax(f, *a.find("26"));  // landing west
  const RunwayPoint p{-500.0, 30.0, 12.0};
  const RunwayPoint back = ax.fromEnu(ax.toEnu(p));
  CHECK_NEAR(back.x, p.x, 1e-6);
  CHECK_NEAR(back.y, p.y, 1e-6);
  CHECK_NEAR(back.z, p.z, 1e-6);
  // Landing west, "right of course" is north.
  const Enu e = ax.toEnu({0.0, 100.0, 0.0});
  const Enu t = ax.toEnu({0.0, 0.0, 0.0});
  CHECK(e.n - t.n > 99.0);
}

TEST(ils_on_glidepath_is_centred) {
  const Airport a = makeTallinn();
  const LocalFrame f(a.reference);
  const Ils ils(f, *a.find("26"));
  for (double nm : {1.0, 4.0, 9.0}) {
    const IlsSignal s = ils.receive(ils.axes().toEnu(ils.glidepathPoint(-nm * kNmToM)));
    CHECK(s.locValid);
    CHECK(s.gsValid);
    CHECK_NEAR(s.locDots, 0.0, 1e-6);
    CHECK_NEAR(s.gsDots, 0.0, 1e-6);
  }
  // Glidepath crosses the threshold at the AIP's 54 ft RDH.
  CHECK_NEAR(ils.glidepathPoint(0.0).z, 54.0 * kFtToM, 1e-6);
}

TEST(ils_deviation_signs_and_scale) {
  const Airport a = makeTallinn();
  const LocalFrame f(a.reference);
  const Ils ils(f, *a.find("26"));
  const double x = -5.0 * kNmToM;
  const RunwayPoint gp = ils.glidepathPoint(x);

  // Right of the centreline: localizer is left, so negative dots (fly left).
  const IlsSignal right = ils.receive(ils.axes().toEnu({x, 200.0, gp.z}));
  CHECK(right.locDots < 0.0);
  // Full scale (2 dots) is +-105 m at the threshold.
  const IlsSignal atThr = ils.receive(ils.axes().toEnu({0.0, -105.0, 15.0}));
  CHECK_NEAR(atThr.locDots, 2.0, 0.01);

  // Below the glidepath: glideslope is above, positive dots (fly up). 1 dot = 0.36 deg.
  const IlsSignal low = ils.receive(ils.axes().toEnu({x, 0.0, gp.z - 30.0}));
  CHECK(low.gsDots > 0.0);
  const double horiz = ils.glideslopeOriginX() - x;
  const double z1dot = horiz * std::tan((3.0 - 0.36) * kDegToRad);
  CHECK_NEAR(ils.receive(ils.axes().toEnu({x, 0.0, z1dot})).gsDots, 1.0, 0.01);

  CHECK_NEAR(right.locRangeNm, (ils.localizerX() - x) / kNmToM, 0.05);
  // The DME sits with the glide path antenna, so it reads range to the touchdown zone.
  CHECK_NEAR(right.dmeNm, (ils.glideslopeOriginX() - x) / kNmToM, 0.05);
}

TEST(ils_coverage) {
  const Airport a = makeTallinn();
  const LocalFrame f(a.reference);
  const Ils ils(f, *a.find("26"));
  // Beyond 25 NM: no localizer.
  CHECK(!ils.receive(ils.axes().toEnu({-30.0 * kNmToM, 0.0, 3000.0})).locValid);
  // Abeam the runway (90 degrees off course): no localizer, no glideslope.
  const IlsSignal abeam = ils.receive(ils.axes().toEnu({1000.0, 5000.0, 300.0}));
  CHECK(!abeam.locValid);
  CHECK(!abeam.gsValid);
  // Glideslope only to 10 NM.
  CHECK(!ils.receive(ils.axes().toEnu(ils.glidepathPoint(-12.0 * kNmToM))).gsValid);
}

TEST(papi_two_white_two_red_on_slope) {
  const Airport a = makeTallinn();
  const LocalFrame f(a.reference);
  const Ils ils(f, *a.find("26"));
  auto whites = [&](double dz) {
    const RunwayPoint gp = ils.glidepathPoint(-2.0 * kNmToM);
    const PapiState s = computePapi(ils, ils.axes().toEnu({gp.x, 0.0, gp.z + dz}));
    int n = 0;
    for (bool w : s.white) n += w ? 1 : 0;
    return n;
  };
  CHECK(whites(0.0) == 2);
  CHECK(whites(120.0) == 4);
  CHECK(whites(-120.0) == 0);
  CHECK(whites(25.0) == 3);
}

#include "a320/Geometry2D.h"

TEST(geometry2d_ground_polygon) {
  using namespace a320::geom;
  const Rect r{0, 0, 100, 100};
  // Level, horizon through the middle: ground is the lower half.
  auto g = groundPolygon(r, {50, 50}, 0.0);
  double area = 0.0;
  for (size_t i = 0; i < g.size(); ++i) {
    const Vec2& p = g[i];
    const Vec2& q = g[(i + 1) % g.size()];
    area += p.x * q.y - q.x * p.y;
  }
  CHECK_NEAR(std::fabs(area) / 2.0, 5000.0, 1e-6);
  for (const Vec2& p : g) CHECK(p.y >= 50.0 - 1e-9);
  // Right bank: the world rolls left, so the horizon rises to the right. With 30 degrees
  // the horizon is at y = 35.6 at x = 75 and y = 64.4 at x = 25.
  g = groundPolygon(r, {50, 50}, 30.0);
  auto inside = [&](Vec2 p) {
    const double rad = 30.0 * 3.14159265358979 / 180.0;
    return std::sin(rad) * (p.x - 50) + std::cos(rad) * (p.y - 50) >= 0.0;
  };
  CHECK(inside({75, 40}));
  CHECK(!inside({25, 60}));
  CHECK(g.size() >= 4);
  // Pitched up so far the horizon is below the box: no ground.
  CHECK(groundPolygon(r, {50, 150}, 0.0).empty());
}

TEST(geometry2d_clip_segment) {
  using namespace a320::geom;
  const Rect r{0, 0, 10, 10};
  Vec2 a{-5, 5}, b{15, 5};
  CHECK(clipSegment(r, a, b));
  CHECK_NEAR(a.x, 0.0, 1e-9);
  CHECK_NEAR(b.x, 10.0, 1e-9);
  Vec2 c{-5, -5}, d{-1, -1};
  CHECK(!clipSegment(r, c, d));
}

/* Header-only 2D helpers for drawing instruments (shared by the Unreal HUD and the tests). */
#pragma once

#include <cmath>
#include <vector>

namespace a320 {
namespace geom {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Rect {
  double x0, y0, x1, y1;  // x0 < x1, y0 < y1
};

// Keeps the part of a convex polygon on the side where a*x + b*y + c >= 0
// (Sutherland-Hodgman against one edge).
inline std::vector<Vec2> clipHalfPlane(const std::vector<Vec2>& poly, double a, double b, double c) {
  std::vector<Vec2> out;
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const Vec2& p = poly[i];
    const Vec2& q = poly[(i + 1) % n];
    const double dp = a * p.x + b * p.y + c;
    const double dq = a * q.x + b * q.y + c;
    if (dp >= 0.0) out.push_back(p);
    if ((dp >= 0.0) != (dq >= 0.0)) {
      const double t = dp / (dp - dq);
      out.push_back({p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t});
    }
  }
  return out;
}

inline std::vector<Vec2> rectPolygon(const Rect& r) {
  return {{r.x0, r.y0}, {r.x1, r.y0}, {r.x1, r.y1}, {r.x0, r.y1}};
}

// Clips segment a-b to the rectangle (Liang-Barsky). Returns false when fully outside.
inline bool clipSegment(const Rect& r, Vec2& a, Vec2& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  double t0 = 0.0, t1 = 1.0;
  const double p[4] = {-dx, dx, -dy, dy};
  const double q[4] = {a.x - r.x0, r.x1 - a.x, a.y - r.y0, r.y1 - a.y};
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0) {
      if (q[i] < 0.0) return false;
      continue;
    }
    const double t = q[i] / p[i];
    if (p[i] < 0.0) {
      if (t > t1) return false;
      if (t > t0) t0 = t;
    } else {
      if (t < t0) return false;
      if (t < t1) t1 = t;
    }
  }
  const Vec2 a0 = a;
  a = {a0.x + t0 * dx, a0.y + t0 * dy};
  b = {a0.x + t1 * dx, a0.y + t1 * dy};
  return true;
}

// Ground part of an attitude indicator: the rectangle below a horizon line through
// `centre` rotated by bankDeg (screen y grows downwards, right wing down = positive bank).
inline std::vector<Vec2> groundPolygon(const Rect& r, Vec2 centre, double bankDeg) {
  const double rad = bankDeg * 3.14159265358979323846 / 180.0;
  // Normal pointing to the ground side. Level: ground is below (+y). In a right bank the
  // world rolls left, so the horizon rises towards the right of the screen.
  const double nx = std::sin(rad), ny = std::cos(rad);
  return clipHalfPlane(rectPolygon(r), nx, ny, -(nx * centre.x + ny * centre.y));
}

}  // namespace geom
}  // namespace a320

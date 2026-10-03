#pragma once

namespace a320 {

struct GeoPos {
  double latDeg = 0.0;
  double lonDeg = 0.0;
  double altM = 0.0;  // above the WGS84 ellipsoid
};

// East-North-Up metres in a tangent plane.
struct Enu {
  double e = 0.0;
  double n = 0.0;
  double u = 0.0;
};

// Exact WGS84 tangent-plane frame (geodetic -> ECEF -> ENU). Radio beams such as the ILS
// are straight lines in space, so their geometry is evaluated in this frame.
class LocalFrame {
 public:
  explicit LocalFrame(const GeoPos& origin);

  Enu toEnu(const GeoPos& p) const;
  GeoPos toGeo(const Enu& p) const;
  const GeoPos& origin() const { return origin_; }

 private:
  GeoPos origin_;
  double ox_, oy_, oz_;  // origin in ECEF
  double sinLat_, cosLat_, sinLon_, cosLon_;
};

}  // namespace a320

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace a320 {

// The terrain's height for the flight model: a grid over all of Estonia in the flat world
// (Content/Terrain/ground.txt + ground.i16, written by tools/make_terrain.py with the same
// flattened heights the scenery shows).
class GroundMap {
 public:
  // dir holds ground.txt and ground.i16. False (and the map stays empty) if they are missing or
  // malformed; error says why.
  bool load(const std::string& dir, std::string* error = nullptr);
  bool loaded() const { return !heights_.empty(); }
  // Metres above the first airport's field (the flat world's height reference), bilinear;
  // false outside the grid.
  bool height(double northM, double eastM, double& outM) const;

 private:
  double southM_ = 0.0, westM_ = 0.0, spacingM_ = 0.0;
  int rows_ = 0, cols_ = 0;
  std::vector<int16_t> heights_;  // decimetres, rows south to north
};

}  // namespace a320

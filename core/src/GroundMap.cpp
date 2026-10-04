#include "a320/GroundMap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace a320 {
namespace {

bool fail(std::string* error, const std::string& why) {
  if (error) *error = why;
  return false;
}

}  // namespace

bool GroundMap::load(const std::string& dir, std::string* error) {
  heights_.clear();
  std::ifstream manifest(dir + "/ground.txt");
  if (!manifest) return fail(error, "no ground.txt in " + dir);
  double south = 0.0, west = 0.0, spacing = 0.0;
  int rows = 0, cols = 0;
  std::string line;
  while (std::getline(manifest, line)) {
    if (line.empty() || line[0] == '#') continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = line.substr(0, eq), value = line.substr(eq + 1);
    if (key == "origin") std::sscanf(value.c_str(), "%lf|%lf", &south, &west);
    else if (key == "spacing") spacing = std::atof(value.c_str());
    else if (key == "size") std::sscanf(value.c_str(), "%d|%d", &rows, &cols);
  }
  if (rows < 2 || cols < 2 || spacing <= 0.0) return fail(error, "ground.txt: no origin, spacing or size");
  std::ifstream data(dir + "/ground.i16", std::ios::binary);
  if (!data) return fail(error, "no ground.i16 in " + dir);
  std::vector<int16_t> heights(static_cast<size_t>(rows) * static_cast<size_t>(cols));
  data.read(reinterpret_cast<char*>(heights.data()), static_cast<std::streamsize>(heights.size() * sizeof(int16_t)));
  if (static_cast<size_t>(data.gcount()) != heights.size() * sizeof(int16_t)) return fail(error, "ground.i16 is too short");
  // The file is little-endian, as is every platform the sim runs on.
  southM_ = south;
  westM_ = west;
  spacingM_ = spacing;
  rows_ = rows;
  cols_ = cols;
  heights_ = std::move(heights);
  return true;
}

bool GroundMap::height(double northM, double eastM, double& outM) const {
  if (heights_.empty()) return false;
  const double r = (northM - southM_) / spacingM_, c = (eastM - westM_) / spacingM_;
  if (r < 0.0 || c < 0.0 || r > rows_ - 1 || c > cols_ - 1) return false;
  const int r0 = std::min(static_cast<int>(r), rows_ - 2), c0 = std::min(static_cast<int>(c), cols_ - 2);
  const double fr = r - r0, fc = c - c0;
  auto at = [&](int rr, int cc) { return heights_[static_cast<size_t>(rr) * static_cast<size_t>(cols_) + static_cast<size_t>(cc)]; };
  const double h = (at(r0, c0) * (1.0 - fc) + at(r0, c0 + 1) * fc) * (1.0 - fr) +
                   (at(r0 + 1, c0) * (1.0 - fc) + at(r0 + 1, c0 + 1) * fc) * fr;
  outM = h / 10.0;
  return true;
}

}  // namespace a320

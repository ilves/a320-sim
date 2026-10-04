/* Header-only logic of the world map (shared by the Unreal HUD and the tests): its tile pyramid
 * (Content/Map/map.txt, written by tools/make_map.py), the places for labels and search
 * (Content/Map/places.txt), which tiles a view needs, label decluttering, and polygon
 * triangulation for airport aprons. Positions are the flat world's metres north/east. */
#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "a320/Geometry2D.h"

namespace a320 {
namespace worldmap {

struct Level {
  int level = 0;
  double tileM = 0.0;  // a tile's side
  int px = 512;        // a tile's pixels per side
  int rows = 0, cols = 0;
  double metresPerPixel() const { return tileM / px; }
};

struct Manifest {
  double southM = 0.0, westM = 0.0;  // the grid's origin, shared by all levels
  std::vector<Level> levels;         // coarsest first
  std::set<std::tuple<int, int, int>> tiles;  // level, row, col of the tiles that exist
  std::string attribution;

  bool has(int level, int row, int col) const { return tiles.count({level, row, col}) != 0; }
  const Level* find(int level) const {
    for (const Level& l : levels)
      if (l.level == level) return &l;
    return nullptr;
  }

  // map.txt: key=value lines, values separated by '|', '#' comments.
  bool parse(const std::string& text) {
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty() || line[0] == '#') continue;
      const size_t eq = line.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = line.substr(0, eq);
      const std::vector<std::string> v = split(line.substr(eq + 1));
      if (key == "origin" && v.size() >= 2) {
        southM = std::atof(v[0].c_str());
        westM = std::atof(v[1].c_str());
      } else if (key == "level" && v.size() >= 5) {
        Level l;
        l.level = std::atoi(v[0].c_str());
        l.tileM = std::atof(v[1].c_str());
        l.px = std::atoi(v[2].c_str());
        l.rows = std::atoi(v[3].c_str());
        l.cols = std::atoi(v[4].c_str());
        if (l.tileM > 0.0 && l.px > 0) levels.push_back(l);
      } else if (key == "tile" && v.size() >= 3) {
        tiles.insert({std::atoi(v[0].c_str()), std::atoi(v[1].c_str()), std::atoi(v[2].c_str())});
      } else if (key == "attribution") {
        attribution = line.substr(eq + 1);
      }
    }
    std::sort(levels.begin(), levels.end(), [](const Level& a, const Level& b) { return a.level < b.level; });
    return !levels.empty() && !tiles.empty();
  }

  // The finest level not much sharper than the view needs (a little magnification is fine).
  int levelFor(double viewMetresPerPixel) const {
    int best = levels.empty() ? 0 : levels.front().level;
    for (const Level& l : levels)
      if (l.metresPerPixel() >= viewMetresPerPixel * 0.6) best = l.level;
    return best;
  }

  struct TileRef {
    int level, row, col;
    double southM, westM, sizeM;
  };
  // Existing tiles of a level overlapping a view rectangle.
  std::vector<TileRef> visible(int level, double viewSouth, double viewWest, double viewNorth, double viewEast) const {
    std::vector<TileRef> out;
    const Level* l = find(level);
    if (!l) return out;
    const int r0 = std::max(0, static_cast<int>(std::floor((viewSouth - southM) / l->tileM)));
    const int r1 = std::min(l->rows - 1, static_cast<int>(std::floor((viewNorth - southM) / l->tileM)));
    const int c0 = std::max(0, static_cast<int>(std::floor((viewWest - westM) / l->tileM)));
    const int c1 = std::min(l->cols - 1, static_cast<int>(std::floor((viewEast - westM) / l->tileM)));
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c)
        if (has(level, r, c)) out.push_back({level, r, c, southM + r * l->tileM, westM + c * l->tileM, l->tileM});
    return out;
  }

  static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
      const size_t bar = s.find('|', start);
      out.push_back(s.substr(start, bar == std::string::npos ? std::string::npos : bar - start));
      if (bar == std::string::npos) break;
      start = bar + 1;
    }
    return out;
  }
};

// For search: lower case, and the Estonian letters as their base letters, so "parnu" finds
// "Pärnu" on any keyboard (UTF-8 in, ASCII out for those letters).
inline std::string fold(const std::string& utf8) {
  std::string out;
  for (size_t i = 0; i < utf8.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(utf8[i]);
    if (c < 0x80) {
      out += static_cast<char>(std::tolower(c));
      continue;
    }
    const unsigned char d = i + 1 < utf8.size() ? static_cast<unsigned char>(utf8[i + 1]) : 0;
    char base = 0;
    if (c == 0xC3) {
      switch (d) {
        case 0xA4: case 0x84: case 0xA5: case 0x85: base = 'a'; break;  // a-umlaut, a-ring
        case 0xB6: case 0x96: case 0xB5: case 0x95: base = 'o'; break;  // o-umlaut, o-tilde
        case 0xBC: case 0x9C: base = 'u'; break;
        default: break;
      }
    } else if (c == 0xC5) {
      if (d == 0xA1 || d == 0xA0) base = 's';
      if (d == 0xBE || d == 0xBD) base = 'z';
    }
    if (base) {
      out += base;
      ++i;
    } else {
      out += static_cast<char>(c);  // other characters as they are
    }
  }
  return out;
}

struct Place {
  std::string type;  // city, town, village, island, lake, airport
  std::string name;  // UTF-8
  std::string key;   // fold(name), for search
  double northM = 0.0, eastM = 0.0;
  long long rank = 0;  // importance: higher first
  int airport = -1;    // the sim's airport index for airports
};

// places.txt: type|name|northM|eastM|rank per line.
inline std::vector<Place> parsePlaces(const std::string& text) {
  std::vector<Place> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const std::vector<std::string> v = Manifest::split(line);
    if (v.size() < 5 || v[1].empty()) continue;
    Place p;
    p.type = v[0];
    p.name = v[1];
    p.key = fold(v[1]);
    p.northM = std::atof(v[2].c_str());
    p.eastM = std::atof(v[3].c_str());
    p.rank = std::atoll(v[4].c_str());
    out.push_back(p);
  }
  return out;
}

// Places matching a query: names starting with it first, then containing it; within each, by
// rank. Airports also match their ICAO code.
inline std::vector<int> search(const std::vector<Place>& places, const std::string& query, size_t maxResults) {
  const std::string q = fold(query);
  std::vector<std::pair<long long, int>> hits;  // (score, index)
  if (q.empty()) return {};
  for (size_t i = 0; i < places.size(); ++i) {
    const Place& p = places[i];
    const size_t at = p.key.find(q);
    if (at == std::string::npos) continue;
    // Whole word starts beat the middle of a word; airports and big places first.
    const bool start = at == 0 || p.key[at - 1] == ' ' || p.key[at - 1] == '-' || p.key[at - 1] == '(';
    const long long score = (start ? 4000000000LL : 0) + (p.type == "airport" ? 2000000000LL : 0) + p.rank;
    hits.push_back({score, static_cast<int>(i)});
  }
  std::stable_sort(hits.begin(), hits.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
  std::vector<int> out;
  for (size_t k = 0; k < hits.size() && k < maxResults; ++k) out.push_back(hits[k].second);
  return out;
}

// Greedy label placement: the most important first, skipping any that would overlap one already
// placed. boxes are screen rectangles in rank order; returns the indices to draw.
inline std::vector<int> declutter(const std::vector<geom::Rect>& boxes, size_t maxLabels) {
  std::vector<int> out;
  std::vector<geom::Rect> placed;
  for (size_t i = 0; i < boxes.size() && out.size() < maxLabels; ++i) {
    const geom::Rect& b = boxes[i];
    bool clear = true;
    for (const geom::Rect& p : placed) {
      if (b.x0 < p.x1 && b.x1 > p.x0 && b.y0 < p.y1 && b.y1 > p.y0) {
        clear = false;
        break;
      }
    }
    if (clear) {
      placed.push_back(b);
      out.push_back(static_cast<int>(i));
    }
  }
  return out;
}

// Ear clipping of a simple polygon (no holes, either winding): triangles as index triples.
inline std::vector<int> triangulate(const std::vector<geom::Vec2>& poly) {
  std::vector<int> tris;
  const int n = static_cast<int>(poly.size());
  if (n < 3) return tris;
  double area = 0.0;
  for (int i = 0; i < n; ++i) {
    const geom::Vec2& a = poly[static_cast<size_t>(i)];
    const geom::Vec2& b = poly[static_cast<size_t>((i + 1) % n)];
    area += a.x * b.y - b.x * a.y;
  }
  const double orient = area >= 0.0 ? 1.0 : -1.0;  // counter-clockwise positive
  std::vector<int> idx(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) idx[static_cast<size_t>(i)] = i;
  auto cross = [](const geom::Vec2& a, const geom::Vec2& b, const geom::Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };
  auto inside = [&](const geom::Vec2& p, const geom::Vec2& a, const geom::Vec2& b, const geom::Vec2& c) {
    const double d1 = cross(a, b, p) * orient, d2 = cross(b, c, p) * orient, d3 = cross(c, a, p) * orient;
    return d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0;
  };
  int guard = 0;
  while (idx.size() > 3 && guard < n * n) {
    ++guard;
    bool clipped = false;
    const size_t m = idx.size();
    for (size_t k = 0; k < m; ++k) {
      const int ia = idx[(k + m - 1) % m], ib = idx[k], ic = idx[(k + 1) % m];
      const geom::Vec2& a = poly[static_cast<size_t>(ia)];
      const geom::Vec2& b = poly[static_cast<size_t>(ib)];
      const geom::Vec2& c = poly[static_cast<size_t>(ic)];
      if (cross(a, b, c) * orient <= 1e-9) continue;  // reflex or flat: not an ear
      bool empty = true;
      for (size_t j = 0; j < m && empty; ++j) {
        const int ip = idx[j];
        if (ip == ia || ip == ib || ip == ic) continue;
        if (inside(poly[static_cast<size_t>(ip)], a, b, c)) empty = false;
      }
      if (!empty) continue;
      tris.insert(tris.end(), {ia, ib, ic});
      idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(k));
      clipped = true;
      break;
    }
    if (!clipped) break;  // not simple: keep what was triangulated
  }
  if (idx.size() == 3) tris.insert(tris.end(), {idx[0], idx[1], idx[2]});
  return tris;
}

}  // namespace worldmap
}  // namespace a320

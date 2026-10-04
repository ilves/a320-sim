// The world map's logic: manifest, places and search, tiles in view, labels, triangulation.
#include <string>
#include <vector>

#include "Check.h"
#include "a320/WorldMap.h"

using namespace a320;

TEST(worldmap_manifest_levels_and_tiles) {
  worldmap::Manifest m;
  CHECK(m.parse("# map\nversion=1\norigin=-200000|-300000\nlevel=0|102400|512|3|5\nlevel=1|51200|512|6|10\n"
                "tile=0|1|2\ntile=1|3|5\nattribution=EOX; OSM\n"));
  CHECK(m.levels.size() == 2 && m.has(0, 1, 2) && !m.has(0, 0, 0) && m.attribution == "EOX; OSM");
  CHECK(m.levelFor(300.0) == 0);   // 200 m/px is enough for a 300 m/px view
  CHECK(m.levelFor(100.0) == 1);   // finer view: the finest level
  CHECK(m.levelFor(250.0) == 0);
  // Tile (1, 3, 5) spans north -46400..4800, east -44000..7200.
  const auto v = m.visible(1, 0.0, 0.0, 1000.0, 1000.0);
  CHECK(v.size() == 1 && v[0].row == 3 && v[0].col == 5 && v[0].sizeM == 51200.0);
  CHECK(m.visible(1, 10000.0, 10000.0, 20000.0, 20000.0).empty());
}

TEST(worldmap_places_and_search) {
  const std::vector<worldmap::Place> places = worldmap::parsePlaces(
      "city|Tallinn|0|-5000|400000\n"
      "city|P\xC3\xA4rnu|-110000|-20000|50000\n"
      "village|Paikuse|-105000|-15000|3000\n"
      "town|Kohtla-J\xC3\xA4rve|10000|150000|33000\n");
  CHECK(places.size() == 4);
  CHECK(worldmap::fold("P\xC3\xA4rnu \xC3\x95ISMU \xC5\xA0") == "parnu oismu s");
  std::vector<int> r = worldmap::search(places, "parnu", 5);
  CHECK(r.size() == 1 && places[static_cast<size_t>(r[0])].name == "P\xC3\xA4rnu");
  r = worldmap::search(places, "PA", 5);  // both start with "pa": the bigger first
  CHECK(r.size() == 2 && r[0] == 1 && r[1] == 2);
  r = worldmap::search(places, "jarve", 5);  // a word start inside the name
  CHECK(r.size() == 1 && r[0] == 3);
  CHECK(worldmap::search(places, "", 5).empty());
}

TEST(worldmap_declutter_and_triangulate) {
  const std::vector<geom::Rect> boxes = {{0, 0, 10, 10}, {5, 5, 15, 15}, {20, 0, 30, 10}};
  const std::vector<int> shown = worldmap::declutter(boxes, 10);
  CHECK(shown.size() == 2 && shown[0] == 0 && shown[1] == 2);
  // An L-shaped apron: 6 corners, 4 triangles, covering its area (3 squares).
  const std::vector<geom::Vec2> l = {{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};
  const std::vector<int> t = worldmap::triangulate(l);
  CHECK(t.size() == 12);
  double area = 0.0;
  for (size_t i = 0; i + 2 < t.size(); i += 3) {
    const geom::Vec2 &a = l[static_cast<size_t>(t[i])], &b = l[static_cast<size_t>(t[i + 1])], &c = l[static_cast<size_t>(t[i + 2])];
    area += std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2.0;
  }
  CHECK_NEAR(area, 3.0, 1e-9);
  // Clockwise works too.
  const std::vector<geom::Vec2> cw(l.rbegin(), l.rend());
  CHECK(worldmap::triangulate(cw).size() == 12);
}

#!/usr/bin/env python3
"""Builds the in-game world map of Estonia: a satellite map pyramid with OpenStreetMap overlays.

  python tools/make_map.py                 (needs: pip install numpy pillow)
  python tools/make_map.py --offline       (only the cached OpenStreetMap data, no network)
  python tools/make_map.py --levels 2 --quality 85 --preview /tmp/map-preview

Run tools/make_terrain.py first: the satellite base is its output (Content/Terrain/outer.jpg and
the region tiles in Content/Terrain/Region/), so no imagery is downloaded here; the sea mask comes
from the elevation tiles it cached (build/terrain-cache/elevation, zoom 9). OpenStreetMap data
comes from the Overpass API once and is cached under build/terrain-cache/map/ (re-runs reuse it).
A layer that no Overpass mirror delivers (or, with --offline, that is not cached) is left out,
named in a comment at the top of map.txt; run again later to fill it in.

Writes unreal/A320Sim/Content/Map/:
  map.txt      manifest, plain text, '#' starts a comment:
                 version=1
                 origin=<southM>|<westM>                 south-west corner of tile (0, 0), every level
                 level=<L>|<tileM>|<px>|<rows>|<cols>    tile size in metres and pixels, grid size
                 tile=<L>|<row>|<col>                    one line per existing file L<L>/<row>_<col>.jpg
                 attribution=...
  L<L>/<row>_<col>.jpg
               512 x 512 px tiles, north up, row 0 the southernmost, column 0 the westernmost.
               L0 102.4 km (200 m/px), L1 51.2 km (100 m/px), L2 25.6 km (50 m/px). L0 is complete;
               at L1 and L2 tiles of open sea are left out (draw the coarser level there).
  places.txt   one line per place, sorted by rank (highest first):
                 type|name|northM|eastM|rank
               type is city, town, village, island or lake. name is the Estonian name (the `name`
               tag in Estonia; `name:et`, else `name:en`, else `name` for towns across the border).
               rank orders labels for decluttering: settlements 3,000,000 (city), 2,000,000 (town)
               or 1,000,000 (village) plus the population; islands and lakes 3,000,000 (100 km2 or
               more), 2,000,000 (10 km2 or more) or 1,000,000 plus the area in hectares.

Every position is in the sim's flat world, the EETN tangent plane of make_terrain.py (Frame, REF):
metres north and east of the airport reference.

Map style: the imagery a little darker and less saturated, the sea a calm dark blue (where the
elevation is below sea level and the imagery is dark), with antialiased lines (drawn at
--supersample times the resolution and box-filtered): the land border (admin_level=2, maritime
parts left out), the coastline, lakes (natural=water + water=lake or reservoir, over 0.5 km2) as a
see-through fill with an outline, rivers (L0 only those over 60 km, L1 over 20 km, counting all
ways of the name), roads (motorway and trunk, primary; secondary from L1,
tertiary at L2) and railways (dashed, from L1). No text: the game draws names itself.

places.txt holds the cities and towns in the map area, the villages of Estonia (place=village;
those with a population tag under 200 are left out, hamlets are not included), the named islands of
Estonia over 1 km2 and its named lakes over 2 km2.
Islands and lakes are labelled at the point of the polygon furthest from its edge (nearest the
centroid among equals). Airports are not in it: the game has its own airport data.

Sources (keep the attribution when sharing the output):
  Imagery   Sentinel-2 cloudless 2024 by EOX IT Services GmbH (https://s2maps.eu), contains
            modified Copernicus Sentinel data 2024. CC BY-NC-SA 4.0: non-commercial use.
  Map data  (c) OpenStreetMap contributors, ODbL (via the Overpass API).
"""
import argparse
import json
import math
import os
import re
import sys
import time
import urllib.parse
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_terrain as mt  # noqa: E402 - the shared frame, geo helpers and paths

np, Image = mt.np, mt.Image
from PIL import ImageDraw  # noqa: E402

MAP_DIR = os.path.join(mt.ROOT, "unreal", "A320Sim", "Content", "Map")
CACHE_DIR = os.path.join(mt.CACHE_DIR, "map")
TERRAIN_TXT = os.path.join(mt.OUT_DIR, "terrain.txt")
# make_terrain.py's mirrors, tried in turn; a copy, reordered to ask the last one that answered first.
OVERPASS_URLS = list(mt.OVERPASS_URLS)
ATTRIBUTION = ("Imagery: Sentinel-2 cloudless 2024 by EOX IT Services GmbH (s2maps.eu), modified Copernicus "
               "Sentinel data 2024, CC BY-NC-SA 4.0. Map data: (c) OpenStreetMap contributors, ODbL.")

# South-west corner of every level's tile grid (flat-world metres). 3 x 5 tiles of 102.4 km cover
# about 57.25-60.03 N, 20.6-29.2 E: Estonia with its islands and 25-70 km around it. A multiple of
# the region tile size, so region tiles land on whole pixels at every level.
ORIGIN = (-236000.0, -252000.0)
L0_ROWS, L0_COLS = 3, 5
TILE_PX = 512
LEVELS = {0: 102400.0, 1: 51200.0, 2: 25600.0}  # tile size in metres

# Base imagery: keep this much of the colour, then scale the brightness.
SATURATION = 0.7
BRIGHTNESS = 0.9
SEA_RGB = (16, 30, 50)  # dark navy, as on a flight simulator's world map
SEA_OPACITY = 0.85

LAKE_MIN_M2 = 0.5e6
RIVER_MIN_KM = (60.0, 20.0, 0.0)  # rivers drawn at L0, L1, L2: at least this long (all ways of the name)
LAKE_LABEL_M2 = 2.0e6
ISLAND_LABEL_M2 = 1.0e6
VILLAGE_MIN_POP = 200

# Line styles: colour, opacity and width in output pixels at L0, L1, L2 (None: not drawn there).
# Road casings are the road's width plus CASING px, drawn under all road fills.
STYLE = {
    "lake_fill": ((45, 110, 190), 0.45, None),
    "lake_edge": ((25, 75, 160), 0.9, (0.7, 0.9, 1.1)),
    "coast": ((140, 190, 225), 0.55, (0.7, 0.9, 1.1)),
    "river": ((70, 140, 225), 0.95, (0.8, 1.1, 1.5)),
    "rail": ((200, 200, 200), 0.95, (None, 1.1, 1.5)),
    "tertiary": ((225, 222, 210), 0.8, (None, None, 1.2)),
    "secondary": ((245, 238, 190), 0.95, (None, 1.2, 1.7)),
    "primary": ((250, 215, 70), 1.0, (1.2, 1.7, 2.3)),
    "motorway": ((245, 145, 40), 1.0, (1.8, 2.4, 3.1)),
    "border": ((240, 240, 240), 0.9, (1.3, 1.5, 1.8)),
}
ROADS = ("tertiary", "secondary", "primary", "motorway")  # drawing order, least important first
CASING = ((25, 25, 25), 0.55, 1.0)
RAIL_DASH = (5.0, 4.0)  # dash and gap in output pixels


# ---------------------------------------------------------------------------------------------
# OpenStreetMap


def ask_overpass(query, label):
    """POSTs one query to the mirrors in turn (three rounds); returns the parsed answer or raises.

    Raises TooBig when the server says the query ran out of time or memory, so the caller can ask
    for a smaller area. A busy mirror (504) or one that does not answer is just skipped.
    """
    body = urllib.parse.urlencode({"data": query}).encode()
    last = None
    for attempt in range(4):
        for url in list(OVERPASS_URLS):
            try:
                req = urllib.request.Request(url, data=body, headers={"User-Agent": mt.USER_AGENT})
                with urllib.request.urlopen(req, timeout=200) as r:
                    doc = json.loads(r.read())
                remark = doc.get("remark", "")
                if "timed out" in remark or "out of memory" in remark:
                    raise TooBig(remark)
                if "error" in remark.lower():
                    raise RuntimeError(remark)  # partial data: never keep it
                OVERPASS_URLS.remove(url)  # ask the mirror that answered first next time
                OVERPASS_URLS.insert(0, url)
                return doc
            except TooBig:
                raise
            except Exception as e:  # noqa: BLE001 - any mirror failure: try the next one
                last = e
                print(f"    {url.split('/')[2]}: {e}", flush=True)
        time.sleep(30 * (attempt + 1))
    raise RuntimeError(f"{label}: no Overpass server answered ({last})")


class TooBig(Exception):
    pass


def overpass(name, query, bbox, offline, cells=(1, 1)):
    """One query over bbox (south, west, north, east), returns its elements.

    query has a {bbox} placeholder. The box is asked for in cells (split in four again when a cell
    is too much for the server), each cached as build/terrain-cache/map/<name>/<cell>.json; the
    merged answer is cached as build/terrain-cache/map/<name>.json, which is all a re-run reads.
    """
    path = os.path.join(CACHE_DIR, name + ".json")
    if not os.path.exists(path):
        if offline:
            raise RuntimeError(f"--offline: {path} is not cached; run once without --offline")
        s, w, n, e = bbox
        rows, cols = cells
        todo = [(s + (n - s) * r / rows, w + (e - w) * c / cols, s + (n - s) * (r + 1) / rows,
                 w + (e - w) * (c + 1) / cols, 0) for r in range(rows) for c in range(cols)]
        seen, elements = set(), []
        while todo:
            cs, cw, cn, ce, depth = todo.pop(0)
            key = f"{cs:.4f}_{cw:.4f}_{cn:.4f}_{ce:.4f}"
            cell = os.path.join(CACHE_DIR, name, key + ".json")
            if os.path.exists(cell):
                with open(cell, "rb") as f:
                    doc = json.load(f)
            else:
                try:
                    print(f"  {name}: cell {key}", flush=True)
                    doc = ask_overpass(query.format(bbox=f"({cs:.4f},{cw:.4f},{cn:.4f},{ce:.4f})"), name)
                except TooBig as err:
                    if depth >= 3:
                        sys.exit(f"{name}: still too big after splitting: {err}")
                    print(f"    too big, splitting ({err})", flush=True)
                    ms, mw = (cs + cn) / 2.0, (cw + ce) / 2.0
                    todo += [(cs, cw, ms, mw, depth + 1), (cs, mw, ms, ce, depth + 1),
                             (ms, cw, cn, mw, depth + 1), (ms, mw, cn, ce, depth + 1)]
                    continue
                os.makedirs(os.path.dirname(cell), exist_ok=True)
                with open(cell + ".part", "w", encoding="utf-8") as f:
                    json.dump(doc, f)
                os.replace(cell + ".part", cell)
            for el in doc.get("elements", []):
                if (el["type"], el["id"]) not in seen:
                    seen.add((el["type"], el["id"]))
                    elements.append(el)
        elements.sort(key=lambda el: (el["type"], el["id"]))  # the same order however it was split
        with open(path + ".part", "w", encoding="utf-8") as f:
            json.dump({"elements": elements}, f)
        os.replace(path + ".part", path)
    with open(path, "rb") as f:
        return json.load(f).get("elements", [])


def fetch_osm(bbox, offline, only=None, missing=None):
    """All OpenStreetMap layers over bbox: {name: elements}. A layer no server delivers is left
    empty and named in `missing` (when given) instead of stopping the build."""
    # Modest declared limits: a server only starts a query once that much time and memory is free,
    # so a large [maxsize] waits in the queue until the mirror's gateway gives up (504).
    head = "[out:json][timeout:120][maxsize:268435456];"

    def ways(sel):
        return f"{head}way{sel}{{bbox}};out tags geom;"

    def areas(sel):
        return f"{head}(way{sel}{{bbox}};)->.w;(relation{sel}{{bbox}};)->.r;.w out tags geom;.r out geom;"

    small, large = (1, 1), (1, 2)  # cells (rows, columns) over the box
    queries = {
        "roads_major": (ways('["highway"~"^(motorway|trunk|primary)$"]'), small),
        "roads_secondary": (ways('["highway"="secondary"]'), small),
        "roads_tertiary": (ways('["highway"="tertiary"]'), large),
        "railways": (ways('["railway"="rail"][!"service"]'), small),
        "rivers": (ways('["waterway"="river"]'), small),
        "coastline": (ways('["natural"="coastline"]'), large),
        "borders": (ways('["boundary"="administrative"]["admin_level"="2"]'), small),
        "lakes": (areas('["natural"="water"]["water"~"^(lake|reservoir)$"]'), large),
        "islands": (f'{head}(node["place"~"^(island|islet)$"]["name"]{{bbox}};)->.n;'
                    f'(way["place"~"^(island|islet)$"]["name"]{{bbox}};)->.w;'
                    f'(relation["place"~"^(island|islet)$"]["name"]{{bbox}};)->.r;.n out;.w out tags geom;.r out geom;',
                    small),
        "places": (f'{head}node["place"~"^(city|town|village)$"]{{bbox}};out;', (1, 1)),
        "estonia": (f'{head}relation["boundary"="administrative"]["admin_level"="2"]["ISO3166-1"="EE"]{{bbox}};out geom;',
                    (1, 1)),
    }
    out = {}
    for k in only or list(queries):
        try:
            out[k] = overpass(k, queries[k][0], bbox, offline, queries[k][1])
        except RuntimeError as err:
            if missing is None:
                raise
            print(f"  {k}: left out ({err})", flush=True)
            missing.append(k)
            out[k] = []
    return out


def to_enu(frame, geom):
    """[{lat, lon}, ...] -> (N, 2) array of east, north metres."""
    lat = np.array([g["lat"] for g in geom], dtype=np.float64)
    lon = np.array([g["lon"] for g in geom], dtype=np.float64)
    e, n = frame.to_enu(lat, lon)
    return np.stack([e, n], axis=1)


def way_lines(frame, elements, keep=lambda tags: True):
    out = []
    for el in elements:
        if el["type"] == "way" and len(el.get("geometry") or ()) >= 2 and keep(el.get("tags", {})):
            out.append(to_enu(frame, el["geometry"]))
    return out


def join_rings(parts):
    """Closes member ways (lists of (lat, lon)) into rings by matching end points; drops open ones."""
    parts = [list(p) for p in parts if len(p) >= 2]
    rings = []
    while parts:
        ring = parts.pop(0)
        while ring[0] != ring[-1]:
            for i, p in enumerate(parts):
                if p[0] == ring[-1]:
                    ring += p[1:]
                elif p[-1] == ring[-1]:
                    ring += p[-2::-1]
                elif p[-1] == ring[0]:
                    ring = p[:-1] + ring
                elif p[0] == ring[0]:
                    ring = p[:0:-1] + ring
                else:
                    continue
                parts.pop(i)
                break
            else:
                break
        if ring[0] == ring[-1] and len(ring) >= 4:
            rings.append(ring)
    return rings


def ring_area(r):
    x, y = r[:, 0], r[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def area_polygons(frame, elements):
    """Closed ways and multipolygon relations -> [(tags, outers, inners, area m2)], rings as (N, 2)."""
    out = []
    for el in elements:
        tags = el.get("tags", {})
        if el["type"] == "way":
            g = el.get("geometry") or []
            if len(g) < 4 or (g[0]["lat"], g[0]["lon"]) != (g[-1]["lat"], g[-1]["lon"]):
                continue
            outers, inners = [[(p["lat"], p["lon"]) for p in g]], []
        elif el["type"] == "relation":
            parts = {"outer": [], "inner": []}
            for m in el.get("members", []):
                if m.get("type") == "way" and m.get("geometry") and None not in m["geometry"]:
                    parts["inner" if m.get("role") == "inner" else "outer"].append(
                        [(p["lat"], p["lon"]) for p in m["geometry"]])
            outers, inners = join_rings(parts["outer"]), join_rings(parts["inner"])
        else:
            continue
        if not outers:
            continue
        conv = [[to_enu(frame, [{"lat": a, "lon": b} for a, b in r]) for r in rs] for rs in (outers, inners)]
        area = sum(ring_area(r) for r in conv[0]) - sum(ring_area(r) for r in conv[1])
        out.append((tags, conv[0], conv[1], area))
    return out


def osm_layers(frame, osm):
    """Everything drawn on the map, as flat-world (N, 2) polylines and polygons."""
    roads = {"motorway": [], "primary": [], "secondary": [], "tertiary": []}
    for key in ("roads_major", "roads_secondary", "roads_tertiary"):
        for el in osm[key]:
            hw = el.get("tags", {}).get("highway")
            cls = "motorway" if hw in ("motorway", "trunk") else hw
            if cls in roads and len(el.get("geometry") or ()) >= 2:
                roads[cls].append(to_enu(frame, el["geometry"]))
    maritime = ("maritime", "territorial", "baseline", "eez", "contiguous")

    def land_border(t):
        return t.get("maritime") != "yes" and t.get("border_type") not in maritime

    lakes = drop_composites([p for p in area_polygons(frame, osm["lakes"]) if p[3] >= LAKE_MIN_M2])
    rivers = [el for el in osm["rivers"] if el["type"] == "way" and len(el.get("geometry") or ()) >= 2]
    river_lines = [to_enu(frame, el["geometry"]) for el in rivers]
    # A river's length is the sum of its ways that share its name: what decides if a level shows it.
    total = {}
    for el, p in zip(rivers, river_lines):
        name = el.get("tags", {}).get("name") or f"#{el['id']}"
        total[name] = total.get(name, 0.0) + float(np.hypot(*np.diff(p, axis=0).T).sum())
    river_km = [total[el.get("tags", {}).get("name") or f"#{el['id']}"] / 1000.0 for el in rivers]
    lines = {
        "coast": way_lines(frame, osm["coastline"]),
        "river": river_lines,
        "rail": way_lines(frame, osm["railways"]),
        "border": way_lines(frame, osm["borders"], land_border),
    }
    lines.update(roads)
    return lines, lakes, river_km


def inside(pt, rings):
    """Even-odd point in polygon over all rings (each (N, 2), closed)."""
    hit = False
    for r in rings:
        a, b = r[:-1], r[1:]
        cross = (a[:, 1] > pt[1]) != (b[:, 1] > pt[1])
        with np.errstate(divide="ignore", invalid="ignore"):
            x = a[:, 0] + (pt[1] - a[:, 1]) * (b[:, 0] - a[:, 0]) / (b[:, 1] - a[:, 1])
        hit ^= bool(np.count_nonzero(cross & (x > pt[0])) % 2)
    return hit


def drop_composites(polys):
    """Leaves out a named lake that is mapped again as named parts (Peipsi-Pihkva jarv, over Peipsi,
    Lammijarv and Pihkva jarv), so it is neither filled twice nor labelled twice."""
    named = [(i, p) for i, p in enumerate(polys) if p[0].get("name")]
    boxes = {i: (*np.concatenate(p[1]).min(0), *np.concatenate(p[1]).max(0)) for i, p in named}
    drop = set()
    for i, big in named:
        bi = boxes[i]
        for j, part in named:
            bj = boxes[j]
            if j == i or not 0.02 * big[3] <= part[3] < big[3]:
                continue
            if bj[0] < bi[0] or bj[1] < bi[1] or bj[2] > bi[2] or bj[3] > bi[3]:
                continue
            if inside(label_point(part[1], part[2]), big[1] + big[2]):
                drop.add(i)
                break
    return [p for i, p in enumerate(polys) if i not in drop]


# ---------------------------------------------------------------------------------------------
# Places


def population(tags):
    m = re.match(r"\d+", re.sub(r"[\s,. ']", "", tags.get("population", "")))
    return int(m.group()) if m else None


def clean_name(s):
    return " ".join(s.replace("|", "/").split())


class Mask:
    """A polygon rasterised over the map at `step` metres, for point-in-polygon tests."""

    def __init__(self, outers, inners, west, south, east, north, step):
        self.west, self.south, self.east, self.north, self.step = west, south, east, north, step
        w, h = int(math.ceil((east - west) / step)), int(math.ceil((north - south) / step))
        img = Image.new("L", (w, h), 0)
        d = ImageDraw.Draw(img)
        for rings, v in ((outers, 255), (inners, 0)):
            for r in rings:
                d.polygon(self._px(r), fill=v)
        self.m = np.asarray(img) > 0

    def _px(self, r):
        return np.stack([(r[:, 0] - self.west) / self.step, (self.north - r[:, 1]) / self.step], 1).ravel().tolist()

    def contains(self, pts):
        pts = np.atleast_2d(pts)
        c = np.floor((pts[:, 0] - self.west) / self.step).astype(np.int64)
        r = np.floor((self.north - pts[:, 1]) / self.step).astype(np.int64)
        ok = (r >= 0) & (r < self.m.shape[0]) & (c >= 0) & (c < self.m.shape[1])
        out = np.zeros(len(pts), dtype=bool)
        out[ok] = self.m[r[ok], c[ok]]
        return out

    def in_grid(self, p):
        return self.west <= p[0] < self.east and self.south <= p[1] < self.north


def label_point(outers, inners):
    """The point furthest inside the polygon (on a ~160 px raster), nearest the centroid among equals."""
    allp = np.concatenate(outers)
    lo, hi = allp.min(0), allp.max(0)
    step = max(hi - lo) / 160.0
    w, h = int((hi[0] - lo[0]) / step) + 3, int((hi[1] - lo[1]) / step) + 3
    img = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(img)
    for rings, v in ((outers, 255), (inners, 0)):
        for r in rings:
            d.polygon(np.stack([(r[:, 0] - lo[0]) / step + 1, (hi[1] - r[:, 1]) / step + 1], 1).ravel().tolist(), fill=v)
    m = np.asarray(img) > 0
    if not m.any():
        return allp.mean(0)
    while True:  # erode until nothing is left; the last survivors are the deepest pixels
        p = np.pad(m, 1)
        e = m & p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:]
        if not e.any():
            break
        m = e
    rr, cc = np.nonzero(m)
    xs, ys = lo[0] + (cc - 0.5) * step, hi[1] - (rr - 0.5) * step
    a = sum(ring_area(r) for r in outers) or 1.0
    cen = sum(r.mean(0) * ring_area(r) for r in outers) / a
    i = int(np.argmin((xs - cen[0]) ** 2 + (ys - cen[1]) ** 2))
    return np.array([xs[i], ys[i]])


def area_rank(area_m2):
    tier = 3 if area_m2 >= 100e6 else 2 if area_m2 >= 10e6 else 1
    return tier * 1_000_000 + int(round(area_m2 / 1e4))


def foreign_name(tags):
    return tags.get("name:et") or tags.get("name:en") or tags.get("name")


def build_places(frame, osm, estonia):
    rows = []
    tier = {"city": 3, "town": 2, "village": 1}
    for el in osm["places"]:
        tags = el.get("tags", {})
        kind = tags.get("place")
        if kind not in tier or not tags.get("name"):
            continue
        p = np.array(frame.to_enu(el["lat"], el["lon"]), dtype=np.float64)
        if not estonia.in_grid(p):
            continue
        inside = bool(estonia.contains(p[None])[0])
        pop = population(tags)
        # Estonia maps its small villages (kula) as hamlets, so place=village is only a few hundred
        # larger ones, most without a population tag: keep those, drop the ones tagged as small.
        if kind == "village" and (not inside or (pop is not None and pop < VILLAGE_MIN_POP)):
            continue
        name = tags["name"] if inside else foreign_name(tags)
        rows.append((kind, clean_name(name), p, tier[kind] * 1_000_000 + min(pop or 0, 999_999)))

    for key, kind, min_area in (("islands", "island", ISLAND_LABEL_M2), ("lakes", "lake", LAKE_LABEL_M2)):
        polys = area_polygons(frame, osm[key])
        for tags, outers, inners, area in (drop_composites(polys) if kind == "lake" else polys):
            name = tags.get("name:et") or tags.get("name")
            if not name or area < min_area:
                continue
            p = label_point(outers, inners)
            # Lakes on the border (Peipus) count when any of their shore is Estonian.
            if not estonia.contains(p[None])[0] and not (kind == "lake" and estonia.contains(np.concatenate(outers)).any()):
                continue
            rows.append((kind, clean_name(name), p, area_rank(area)))
    rows.sort(key=lambda r: (-r[3], r[0], r[1], round(r[2][0]), round(r[2][1])))
    return rows


# ---------------------------------------------------------------------------------------------
# Imagery


def read_terrain():
    """Base and region tiles of the terrain manifest: [(file, south, west, size)]."""
    base, region = None, []
    with open(TERRAIN_TXT, encoding="utf-8") as f:
        for line in f:
            key, _, val = line.strip().partition("=")
            if key in ("tile", "region"):
                name, south, west, size = val.split("|")[:4]
                path = os.path.join(mt.OUT_DIR if key == "tile" else mt.REGION_DIR, name + ".jpg")
                item = (path, float(south), float(west), float(size))
                if key == "tile":
                    base = item
                else:
                    region.append(item)
    if base is None:
        sys.exit(f"{TERRAIN_TXT} has no base tile: run tools/make_terrain.py first")
    return base, region


def finest_imagery(base, region, mpp):
    """The satellite mosaic over the whole grid at mpp metres per pixel, north up (uint8 RGB)."""
    south, west = ORIGIN
    w, h = int(round(L0_COLS * LEVELS[0] / mpp)), int(round(L0_ROWS * LEVELS[0] / mpp))
    north = south + h * mpp
    path, b_south, b_west, b_size = base
    img = Image.open(path).convert("RGB")
    bm = b_size / img.size[0]
    box = ((west - b_west) / bm, (b_south + b_size - north) / bm, (west + w * mpp - b_west) / bm,
           (b_south + b_size - south) / bm)
    canvas = img.resize((w, h), Image.BICUBIC, box=box)
    for path, s0, w0, size in region:
        x0, y0 = (w0 - west) / mpp, (north - (s0 + size)) / mpp
        n = size / mpp
        if abs(x0 - round(x0)) > 1e-6 or abs(y0 - round(y0)) > 1e-6 or abs(n - round(n)) > 1e-6:
            sys.exit(f"{path}: not aligned with the map grid; ORIGIN must be a multiple of the region tile size")
        if x0 + n <= 0 or y0 + n <= 0 or x0 >= w or y0 >= h:
            continue
        tile = Image.open(path).convert("RGB").resize((int(round(n)),) * 2, Image.LANCZOS)
        canvas.paste(tile, (int(round(x0)), int(round(y0))))
    return canvas


def sea_mask(frame, mpp, w, h):
    """Sea weight 0..1 over the grid (h x w pixels of mpp metres), from the elevation tiles that
    make_terrain.py cached for its base layer (zoom 9, about 160 m): below sea level is sea."""
    south, west = ORIGIN
    step = max(mpp, 100.0)
    gw, gh = int(math.ceil(w * mpp / step)), int(math.ceil(h * mpp / step))
    lat_r, lon_r = mt.geo_bounds(frame, west, south, west + gw * step, south + gh * step)
    elevation = mt.Source("elevation", mt.ELEVATION_URL, 9, "png", mt.decode_terrarium).area(lat_r, lon_r)
    e = west + (np.arange(gw) + 0.5) * step
    out = np.zeros((gh, gw), dtype=np.float32)
    north = south + gh * step
    for r0 in range(0, gh, 256):
        n = north - (np.arange(r0, min(r0 + 256, gh)) + 0.5) * step
        ee, nn = np.meshgrid(e, n)
        height = elevation.sample(*frame.to_geo(ee, nn))
        out[r0:r0 + 256] = np.clip((1.0 - height) / 2.0, 0.0, 1.0)
    img = Image.fromarray((out * 255.0 + 0.5).astype(np.uint8)).resize((w, h), Image.BILINEAR)
    return img


def map_style(img, sea):
    """Darker, less saturated imagery so the overlays read well, and one calm colour for the sea.

    The sea is only recoloured where the imagery is dark, so a beach or a harbour the coarse sea
    mask overlaps keeps its own colour; the OpenStreetMap coastline drawn later gives the sharp edge.
    """
    a = np.asarray(img)
    m = np.asarray(sea)
    out = np.empty_like(a)
    sea_rgb = np.array(SEA_RGB, dtype=np.float32)
    for r0 in range(0, a.shape[0], 512):
        s = a[r0:r0 + 512].astype(np.float32)
        lum = s @ np.array([0.299, 0.587, 0.114], dtype=np.float32)
        s = (lum[..., None] + (s - lum[..., None]) * SATURATION) * BRIGHTNESS
        w = m[r0:r0 + 512].astype(np.float32) / 255.0 * np.clip((45.0 - lum) / 20.0, 0.0, 1.0) * SEA_OPACITY
        s += (sea_rgb - s) * w[..., None]
        out[r0:r0 + 512] = np.clip(s + 0.5, 0, 255).astype(np.uint8)
    return Image.fromarray(out)


# ---------------------------------------------------------------------------------------------
# Overlays


class Layer:
    """Polylines (or polygons) of one style, with bounding boxes to pick those over a tile."""

    def __init__(self, shapes, step):
        # Drop vertices that fall in the same drawing cell as the previous one.
        self.shapes = [decimate(s, step) for s in shapes]
        self.shapes = [s for s in self.shapes if len(s) >= 2]
        self.box = (np.array([[*s.min(0), *s.max(0)] for s in self.shapes]) if self.shapes
                    else np.zeros((0, 4)))

    def over(self, west, south, east, north, pad):
        b = self.box
        hit = (b[:, 0] <= east + pad) & (b[:, 2] >= west - pad) & (b[:, 1] <= north + pad) & (b[:, 3] >= south - pad)
        return [self.shapes[i] for i in np.nonzero(hit)[0]]


def decimate(pts, step):
    q = np.round(pts / step).astype(np.int64)
    keep = np.ones(len(pts), dtype=bool)
    keep[1:] = np.any(q[1:] != q[:-1], axis=1)
    keep[-1] = True
    return pts[keep]


def dashes(lines, dash, gap):
    """Splits polylines into dashes (lengths in metres), phased along each line from its start."""
    out = []
    period = dash + gap
    for p in lines:
        seg = np.hypot(*np.diff(p, axis=0).T)
        dist = np.concatenate([[0.0], np.cumsum(seg)])
        for a in np.arange(0.0, dist[-1], period):
            b = min(a + dash, dist[-1])
            inner = (dist > a) & (dist < b)
            pts = np.vstack([[np.interp(a, dist, p[:, 0]), np.interp(a, dist, p[:, 1])], p[inner],
                             [np.interp(b, dist, p[:, 0]), np.interp(b, dist, p[:, 1])]])
            out.append(pts)
    return out


class Overlay:
    """The vector layers of one level, in metres, ready to draw into its tiles."""

    def __init__(self, lines, lakes, river_km, level, mpp, ss):
        self.level, self.mpp, self.ss = level, mpp, ss
        step = mpp / ss
        self.layers = {}
        for key, shapes in lines.items():
            width = STYLE[key][2][level]
            if width is None:
                continue
            if key == "river":
                shapes = [s for s, km in zip(shapes, river_km) if km >= RIVER_MIN_KM[level]]
            if key == "rail":
                shapes = dashes(shapes, RAIL_DASH[0] * mpp, RAIL_DASH[1] * mpp)
            self.layers[key] = Layer(shapes, step)
        self.lakes = []
        for _, outers, inners, _ in lakes:
            o = [decimate(r, step) for r in outers]
            i = [decimate(r, step) for r in inners]
            allp = np.concatenate(o)
            self.lakes.append((o, i, (*allp.min(0), *allp.max(0))))

    def render(self, base, west, south, size):
        """Draws every layer over one tile; base is (px, px, 3) uint8, returns the same."""
        px = base.shape[0]
        ss, mpp = self.ss, self.mpp
        east, north = west + size, south + size
        out = base.astype(np.float32)

        def to_px(p):
            return np.stack([(p[:, 0] - west) / mpp * ss, (north - p[:, 1]) / mpp * ss], 1).ravel().tolist()

        def blend(mask, colour, alpha):
            nonlocal out
            m = np.asarray(mask.reduce(ss), dtype=np.float32)[..., None] * (alpha / 255.0)
            if m.any():
                out = out * (1.0 - m) + np.array(colour, dtype=np.float32) * m

        def lines(shapes, width_px, caps=True):
            img = Image.new("L", (px * ss, px * ss), 0)
            d = ImageDraw.Draw(img)
            w = max(1, int(round(width_px * ss)))
            for p in shapes:
                xy = to_px(p)
                d.line(xy, fill=255, width=w, joint="curve")
                if caps and w > 2:
                    r = w / 2.0
                    for x, y in (xy[:2], xy[-2:]):
                        d.ellipse((x - r, y - r, x + r, y + r), fill=255)
            return img

        def picked(key, extra=0.0):
            layer = self.layers.get(key)
            if layer is None:
                return []
            return layer.over(west, south, east, north, (STYLE[key][2][self.level] + extra) * mpp)

        lakes = [lk for lk in self.lakes if lk[2][0] <= east and lk[2][2] >= west and lk[2][1] <= north and lk[2][3] >= south]
        if lakes:
            fill = Image.new("L", (px * ss, px * ss), 0)
            d = ImageDraw.Draw(fill)
            for outers, inners, _ in lakes:
                for r in outers:
                    d.polygon(to_px(r), fill=255)
                for r in inners:
                    d.polygon(to_px(r), fill=0)
            blend(fill, *STYLE["lake_fill"][:2])
            edge = lines([r for o, i, _ in lakes for r in o + i], STYLE["lake_edge"][2][self.level], caps=False)
            blend(edge, *STYLE["lake_edge"][:2])
        for key in ("coast", "river", "rail"):
            shapes = picked(key)
            if shapes:
                blend(lines(shapes, STYLE[key][2][self.level], caps=key != "rail"), *STYLE[key][:2])
        casing = None
        for key in ROADS:
            shapes = picked(key, CASING[2])
            if shapes:
                img = lines(shapes, STYLE[key][2][self.level] + CASING[2])
                casing = img if casing is None else Image.fromarray(np.maximum(np.asarray(casing), np.asarray(img)))
        if casing is not None:
            blend(casing, *CASING[:2])
        for key in ROADS + ("border",):
            shapes = picked(key)
            if shapes:
                blend(lines(shapes, STYLE[key][2][self.level]), *STYLE[key][:2])
        return np.clip(out + 0.5, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------------------------


def grid_bbox(frame):
    """Lat/lon box (south, west, north, east) around the whole tile grid."""
    south, west = ORIGIN
    (lat0, lat1), (lon0, lon1) = mt.geo_bounds(frame, west, south, west + L0_COLS * LEVELS[0],
                                               south + L0_ROWS * LEVELS[0])
    return lat0, lon0, lat1, lon1


def land_tiles(level, region, lines):
    """(row, col) of the tiles at this level that are not entirely open sea.

    A tile has land when a region tile (made only where there is land) overlaps it, or when a
    coastline, road or railway vertex lies in it.
    """
    size = LEVELS[level]
    south, west = ORIGIN
    rows, cols = L0_ROWS * int(LEVELS[0] / size), L0_COLS * int(LEVELS[0] / size)
    hit = np.zeros((rows, cols), dtype=bool)
    for _, s0, w0, sz in region:
        r0, r1 = int((s0 - south) // size), int((s0 + sz - 1e-6 - south) // size)
        c0, c1 = int((w0 - west) // size), int((w0 + sz - 1e-6 - west) // size)
        for r in range(max(r0, 0), min(r1, rows - 1) + 1):
            for c in range(max(c0, 0), min(c1, cols - 1) + 1):
                hit[r, c] = True
    pts = np.concatenate([np.concatenate(lines[k]) for k in ("coast", "rail") + ROADS if lines[k]])
    r = np.floor((pts[:, 1] - south) / size).astype(np.int64)
    c = np.floor((pts[:, 0] - west) / size).astype(np.int64)
    ok = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    hit[r[ok], c[ok]] = True
    return {(int(a), int(b)) for a, b in zip(*np.nonzero(hit))}


def write_manifest(missing=()):
    south, west = ORIGIN
    lines = ["# Generated by tools/make_map.py; see that file for the format and the sources.",
             "version=1", f"origin={south:.1f}|{west:.1f}"]
    if missing:
        lines.insert(1, "# OpenStreetMap layers left out (no server answered): " + ", ".join(missing))
    tiles = []
    for level, size in LEVELS.items():
        k = int(LEVELS[0] / size)
        lines.append(f"level={level}|{size:.1f}|{TILE_PX}|{L0_ROWS * k}|{L0_COLS * k}")
        d = os.path.join(MAP_DIR, f"L{level}")
        if os.path.isdir(d):
            for f in os.listdir(d):
                m = re.fullmatch(r"(\d+)_(\d+)\.jpg", f)
                if m:
                    tiles.append((level, int(m.group(1)), int(m.group(2))))
    lines += [f"tile={lv}|{r}|{c}" for lv, r, c in sorted(tiles)]
    lines.append(f"attribution={ATTRIBUTION}")
    with open(os.path.join(MAP_DIR, "map.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return tiles


def parse_levels(text):
    try:
        levels = sorted({int(v) for v in text.split(",") if v.strip()})
    except ValueError:
        levels = []
    if not levels or any(v not in LEVELS for v in levels):
        raise argparse.ArgumentTypeError(f"expected comma-separated levels from {', '.join(map(str, LEVELS))}")
    return levels


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--levels", type=parse_levels, default=list(LEVELS),
                    help="levels to (re)build, comma-separated (default 0,1,2); the manifest lists all on disk")
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality (default 80)")
    ap.add_argument("--supersample", type=int, default=4, help="overlay antialiasing factor (default 4)")
    ap.add_argument("--offline", action="store_true", help="use only cached OpenStreetMap data")
    ap.add_argument("--no-places", action="store_true", help="leave places.txt as it is")
    ap.add_argument("--preview", metavar="DIR", help="also write preview PNGs (L0 mosaic, L2 around Tallinn)")
    args = ap.parse_args()

    frame = mt.Frame(*mt.REF)
    south, west = ORIGIN
    east, north = west + L0_COLS * LEVELS[0], south + L0_ROWS * LEVELS[0]
    os.makedirs(MAP_DIR, exist_ok=True)

    print("OpenStreetMap", flush=True)
    missing = []
    osm = fetch_osm(grid_bbox(frame), args.offline, missing=missing)
    lines, lakes, river_km = osm_layers(frame, osm)
    print("  " + ", ".join(f"{k} {len(v)}" for k, v in lines.items()) + f", lakes {len(lakes)}", flush=True)

    if not args.no_places:
        print("Places", flush=True)
        poly = area_polygons(frame, osm["estonia"])
        if poly:
            estonia = Mask(poly[0][1], poly[0][2], west, south, east, north, 50.0)
        else:  # no country outline: count the whole grid as Estonia
            box = np.array([[west, south], [east, south], [east, north], [west, north]])
            estonia = Mask([box], [], west, south, east, north, 50.0)
        places = build_places(frame, osm, estonia)
        with open(os.path.join(MAP_DIR, "places.txt"), "w", encoding="utf-8") as f:
            for kind, name, p, rank in places:
                f.write(f"{kind}|{name}|{p[1]:.1f}|{p[0]:.1f}|{rank}\n")
        counts = {}
        for kind, *_ in places:
            counts[kind] = counts.get(kind, 0) + 1
        print(f"  {len(places)} places: " + ", ".join(f"{k} {v}" for k, v in counts.items()), flush=True)

    base, region = read_terrain()
    finest = max(args.levels)
    mpp = LEVELS[finest] / TILE_PX
    print(f"Imagery at {mpp:.0f} m/px", flush=True)
    canvas = finest_imagery(base, region, mpp)
    canvas = map_style(canvas, sea_mask(frame, mpp, *canvas.size))
    for level in sorted(args.levels, reverse=True):
        size = LEVELS[level]
        lmpp = size / TILE_PX
        k = int(LEVELS[0] / size)
        rows, cols = L0_ROWS * k, L0_COLS * k
        img = canvas.reduce(int(round(lmpp / mpp))) if lmpp != mpp else canvas
        pix = np.asarray(img)
        keep = {(r, c) for r in range(rows) for c in range(cols)} if level == 0 else land_tiles(level, region, lines)
        out_dir = os.path.join(MAP_DIR, f"L{level}")
        os.makedirs(out_dir, exist_ok=True)
        for f in os.listdir(out_dir):
            if f.endswith(".jpg"):
                os.remove(os.path.join(out_dir, f))
        overlay = Overlay(lines, lakes, river_km, level, lmpp, args.supersample)
        for i, (r, c) in enumerate(sorted(keep)):
            y0 = (rows - 1 - r) * TILE_PX
            tile = pix[y0:y0 + TILE_PX, c * TILE_PX:(c + 1) * TILE_PX]
            tile = overlay.render(tile, west + c * size, south + r * size, size)
            Image.fromarray(tile).save(os.path.join(out_dir, f"{r}_{c}.jpg"), quality=args.quality,
                                       optimize=True, subsampling=0)
            if (i + 1) % 50 == 0:
                print(f"    {i + 1}/{len(keep)}", flush=True)
        print(f"  L{level}: {len(keep)} of {rows}x{cols} tiles", flush=True)

    tiles = write_manifest(missing)
    if missing:
        print("Left out (re-run to fill in): " + ", ".join(missing))
    total = sum(os.path.getsize(os.path.join(dp, f)) for dp, _, fs in os.walk(MAP_DIR) for f in fs)
    print(f"Done: {MAP_DIR} ({len(tiles)} tiles, {total / 1e6:.1f} MB)")
    if args.preview:
        write_previews(frame, args.preview)


def stitch(level):
    """All existing tiles of a level in one image (missing ones black), north up."""
    k = int(LEVELS[0] / LEVELS[level])
    rows, cols = L0_ROWS * k, L0_COLS * k
    img = Image.new("RGB", (cols * TILE_PX, rows * TILE_PX))
    for r in range(rows):
        for c in range(cols):
            p = os.path.join(MAP_DIR, f"L{level}", f"{r}_{c}.jpg")
            if os.path.exists(p):
                img.paste(Image.open(p), (c * TILE_PX, (rows - 1 - r) * TILE_PX))
    return img


def write_previews(frame, out):
    os.makedirs(out, exist_ok=True)
    stitch(0).reduce(2).save(os.path.join(out, "map_L0_estonia.png"))
    # 2 x 2 L2 tiles around Tallinn (the reference point is at EETN).
    size = LEVELS[2]
    south, west = ORIGIN
    k = int(LEVELS[0] / size)
    rows = L0_ROWS * k
    r = int((0.0 - south) // size)
    c = int((0.0 - west) // size)
    img = stitch(2)
    x0, y0 = (c - 1) * TILE_PX, (rows - 1 - (r + 1)) * TILE_PX
    img.crop((x0 + TILE_PX // 2, y0 + TILE_PX // 2, x0 + TILE_PX * 2 + TILE_PX // 2,
              y0 + TILE_PX * 2 + TILE_PX // 2)).save(os.path.join(out, "map_L2_tallinn.png"))
    print(f"Previews in {out}")


if __name__ == "__main__":
    main()

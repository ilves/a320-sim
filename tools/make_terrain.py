#!/usr/bin/env python3
"""Builds the real-world scenery of Estonia, centred on Tallinn (EETN), from open data.

  python tools/make_terrain.py            (needs: pip install numpy pillow)
  python tools/make_terrain.py --airports none        (no detailed patches away from EETN)
  python tools/make_terrain.py --airports EEKE --patch-km 20 --patch-buildings-km 10

Downloads once into build/terrain-cache (re-runs reuse it) and writes
unreal/A320Sim/Content/Terrain/, which the simulator loads at start-up and streams in flight:
  terrain.txt    manifest: reference point, attribution, one line per tile and the buildings file.
                 Tile lines are name|southM|westM|sizeM|grid|holeM, keyed by layer:
                   tile=    always loaded (the low-detail base under everything)
                   detail=  10 m/px tiles around the airport (tile_R_C) and around the other
                            airports of --airports (eeke_R_C, eetu_R_C, ...), streamed close to the aircraft
                   region=  20 m/px tiles over all of Estonia (files in Region/), streamed
  *.jpg / *.f32  per tile: satellite image (north up) and a float32 height grid (metres above
                 the airfield, rows south to north, columns west to east)
  buildings.bin  OpenStreetMap buildings as boxes: 7 float32 each (north, east, ground height, length,
                 width, yaw from north towards east in degrees, height), metres. Rows: around EETN,
                 around each airport of --airports, then the towns of --town-population and more
                 (each OSM building once; a town's ground is the surface the sim draws there)
  ground.txt     the ground height map for the flight model (radio altimeter, crashes), the same
  ground.i16     flattened heights as the terrain on a regular grid over the region box:
                   version=1
                   origin=<southM>|<westM>        (the first sample, south-west corner)
                   spacing=<metres>
                   size=<rows>|<cols>
                 ground.i16 holds rows*cols int16 little-endian, decimetres above the EETN field,
                 rows south to north, columns west to east (sample r,c at north = south + r*spacing,
                 east = west + c*spacing); sea level outside the elevation data
and unreal/A320Sim/Content/Airports/<ICAO>.txt for every airport in AIRPORTS: its taxiways and aprons
from OpenStreetMap (runways come from the AIP data in the sim), plain text, '#' starts a comment:
  icao=<ICAO>
  level=<metres above the EETN field: the level the terrain is flattened to there>
  taxiway=<ref or ->|<widthM>|n,e;n,e;...   centreline polyline, north/east metres in the same frame
  apron=<ref or ->|n,e;n,e;...              simple closed outline: last point not repeated, no holes
Numbers have one decimal. Taxiway width is the OSM width tag, else the airport's default (AIRPORTS).

Sources (keep the attribution when sharing the output):
  Imagery   Sentinel-2 cloudless 2024 by EOX IT Services GmbH (https://s2maps.eu), contains
            modified Copernicus Sentinel data 2024. CC BY-NC-SA 4.0: non-commercial use.
  Elevation Terrain Tiles (Mapzen, AWS Open Data): SRTM, GMTED, ETOPO1 and others.
  Buildings OpenStreetMap contributors, ODbL (via the Overpass API), also the airport layouts.

The sim's world is the airport's local east-north-up tangent plane (core/src/Geo.cpp), with
heights above the field; every vertex and pixel is placed through the same transform.

Every runway in AIRPORTS is blended to its elevation in all layers, under the buildings and in the
ground map. Each airport named by --airports (default: all but EETN) also gets detailed tiles over
the 10 km squares within its patch radius (AIRPORTS, or --patch-km for all) of its reference point,
each with a region tile of the same square, and OpenStreetMap buildings within its buildings radius
(or --patch-buildings-km), cached as buildings_<code>_<m>.json.
"""
import argparse
import collections
import concurrent.futures
import io
import json
import math
import os
import sys
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request

try:
    import numpy as np
    from PIL import Image
except ImportError:
    sys.exit("make_terrain.py needs numpy and Pillow: pip install numpy pillow")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "unreal", "A320Sim", "Content", "Terrain")
REGION_DIR = os.path.join(OUT_DIR, "Region")  # the sim looks for region= tiles here
CACHE_DIR = os.path.join(ROOT, "build", "terrain-cache")
USER_AGENT = "a320-sim-terrain/1.0 (personal flight simulator; one-time download)"

IMAGERY_URL = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2024_3857/default/g/{z}/{y}/{x}.jpg"
ELEVATION_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
OVERPASS_URLS = [
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
    "https://overpass.openstreetmap.fr/api/interpreter",
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
AIRPORTS_DIR = os.path.join(ROOT, "unreal", "A320Sim", "Content", "Airports")
ATTRIBUTION = ("Imagery: Sentinel-2 cloudless 2024 by EOX IT Services GmbH (s2maps.eu), modified Copernicus "
               "Sentinel data 2024, CC BY-NC-SA 4.0. Elevation: Mapzen Terrain Tiles (AWS Open Data). "
               "Buildings: (c) OpenStreetMap contributors, ODbL.")

# EETN runway ends, as in core/src/Airport.cpp makeTallinn(): (lat, lon, elevation ft).
RWY08 = (59.413338, 24.805908, 129.0)
RWY26 = (59.413174, 24.867208, 131.0)
REF = ((RWY08[0] + RWY26[0]) / 2.0, (RWY08[1] + RWY26[1]) / 2.0, 131.0 * 0.3048)

# Runways the terrain is flattened around: both ends (lat, lon), the level to blend to (ft MSL), the
# reference point a detailed patch is centred on, the half widths (km) of that patch and of its
# buildings area (None: the --patch-km / --patch-buildings-km defaults) and the taxiway width (m) the
# layout uses where OSM has none. Data from the Estonian eAIP AD 2.2/2.12 (AIRAC 2026-10-01).
Airport = collections.namedtuple("Airport", "ends elev_ft ref patch_km buildings_km taxiway_m",
                                 defaults=(None, None, 18.0))
AIRPORTS = {
    "EETN": Airport((RWY08[:2], RWY26[:2]), 131.0, REF[:2], taxiway_m=23.0),
    # 17/35 thresholds 58°14'27.69"N 22°30'33.61"E (10 ft), 58°13'23.05"N 22°30'34.52"E (8 ft).
    "EEKE": Airport(((58.2410250, 22.5093361), (58.2230694, 22.5095889)), 9.0, (58.2300000, 22.5094444), 15.0, 8.0),
    # Tartu 08/26: 58°18'25.87"N 26°40'17.26"E, 58°18'27.47"N 26°42'07.76"E; ARP 58°18'27"N 26°41'13"E.
    # Its buildings reach Tartu city, 9 km NNE.
    "EETU": Airport(((58.3071861, 26.6714611), (58.3076306, 26.7021556)), 202.0, (58.3075000, 26.6869444),
                    15.0, 12.0),
    # Pärnu 03/21: 58°24'48.66"N 24°27'56.45"E, 58°25'40.63"N 24°29'06.58"E; ARP 58°25'08"N 24°28'22"E.
    "EEPU": Airport(((58.4135167, 24.4656806), (58.4279528, 24.4851611)), 38.0, (58.4188889, 24.4727778),
                    12.0, 8.0),
    # Kärdla 14/32: 58°59'47.53"N 22°49'25.47"E, 58°59'06.16"N 22°50'16.78"E; ARP 58°59'27"N 22°49'51"E.
    "EEKA": Airport(((58.9965361, 22.8237417), (58.9850444, 22.8379944)), 14.0, (58.9908333, 22.8308333),
                    10.0, 5.0),
    # Ruhnu 13/31 (600 m grass): 57°47'16.1"N 23°15'32.9"E, 57°47'00.8"N 23°15'55.6"E; ARP 57°47'08"N 23°15'44"E.
    "EERU": Airport(((57.7878056, 23.2591389), (57.7835556, 23.2654444)), 10.0, (57.7855556, 23.2622222),
                    4.0, 4.0, 10.0),
    # Kihnu 04/22 (600 m grass): 58°08'46.87"N 23°59'41.22"E, 58°08'59.44"N 24°00'09.15"E;
    # ARP 58°08'54"N 24°00'09"E.
    "EEKU": Airport(((58.1463528, 23.9947833), (58.1498444, 24.0025417)), 10.0, (58.1483333, 24.0025000),
                    5.0, 4.0, 10.0),
}

# Estonia with the islands and the south-east: lat min, lat max, lon min, lon max.
ESTONIA = (57.45, 59.85, 21.6, 28.3)
LAND_M = 0.5  # elevation above which a sample counts as land (the sea is at or below 0)
# The base layer sits this far below every finer tile over it, so the two never z-fight.
BASE_DROP_M = 3.0
# Open water as the imagery shows it near the coast. Further out EOX paints a flat blue fill that
# reaches closer to shore at low zoom, so coarse tiles next to fine ones would show blue against dark.
WATER_RGB = (6.0, 16.0, 12.0)

# WGS84, as in core/src/Geo.cpp.
A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


def geo_to_ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    sl = np.sin(lat)
    n = A / np.sqrt(1.0 - E2 * sl * sl)
    return ((n + alt) * np.cos(lat) * np.cos(lon), (n + alt) * np.cos(lat) * np.sin(lon),
            (n * (1.0 - E2) + alt) * sl)


class Frame:
    """The sim's LocalFrame: geodetic <-> east/north/up metres at the airport reference."""

    def __init__(self, lat, lon, alt):
        self.o = [float(v) for v in geo_to_ecef(lat, lon, alt)]
        self.sl, self.cl = math.sin(math.radians(lat)), math.cos(math.radians(lat))
        self.so, self.co = math.sin(math.radians(lon)), math.cos(math.radians(lon))

    def to_enu(self, lat, lon, alt=0.0):
        x, y, z = geo_to_ecef(lat, lon, alt)
        dx, dy, dz = x - self.o[0], y - self.o[1], z - self.o[2]
        e = -self.so * dx + self.co * dy
        n = -self.sl * self.co * dx - self.sl * self.so * dy + self.cl * dz
        return e, n

    def to_geo(self, e, n):
        # Points on the ground: the tangent plane drops away from the earth by d^2 / 2R.
        u = -(e * e + n * n) / (2.0 * 6371000.0)
        dx = -self.so * e - self.sl * self.co * n + self.cl * self.co * u
        dy = self.co * e - self.sl * self.so * n + self.cl * self.so * u
        dz = self.cl * n + self.sl * u
        x, y, z = self.o[0] + dx, self.o[1] + dy, self.o[2] + dz
        lon = np.arctan2(y, x)
        p = np.hypot(x, y)
        lat = np.arctan2(z, p * (1.0 - E2))
        for _ in range(4):
            sl = np.sin(lat)
            nn = A / np.sqrt(1.0 - E2 * sl * sl)
            h = p / np.cos(lat) - nn
            lat = np.arctan2(z, p * (1.0 - E2 * nn / (nn + h)))
        return np.degrees(lat), np.degrees(lon)


def mercator_px(lat, lon, z):
    """Global Web Mercator pixel coordinates (256 px tiles) at zoom z."""
    s = 256.0 * 2 ** z
    lat = np.clip(lat, -85.0, 85.0)
    x = (np.asarray(lon) + 180.0) / 360.0 * s
    y = (1.0 - np.log(np.tan(np.radians(lat)) + 1.0 / np.cos(np.radians(lat))) / math.pi) / 2.0 * s
    return x, y


def fetch(url, path, retries=4):
    if os.path.exists(path):
        with open(path, "rb") as f:
            return f.read()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=60) as r:
                data = r.read()
            with open(path + ".part", "wb") as f:
                f.write(data)
            os.replace(path + ".part", path)
            return data
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None
            err = e
        except (urllib.error.URLError, OSError) as e:
            err = e
        time.sleep(2 ** attempt)
    raise RuntimeError(f"download failed: {url}: {err}")


def tile_range(lat, lon, z, pad_px=0.0):
    """Web Mercator tiles (x0, y0, x1, y1, inclusive) covering the given points."""
    x, y = mercator_px(np.asarray(lat), np.asarray(lon), z)
    return (int((np.min(x) - pad_px) // 256), int((np.min(y) - pad_px) // 256),
            int((np.max(x) + pad_px) // 256), int((np.max(y) + pad_px) // 256))


class Source:
    """One tile server at one zoom: downloads into the cache, decodes, keeps the last `keep` tiles."""

    def __init__(self, name, url, z, ext, decode, keep=0):
        self.name, self.url, self.z, self.ext, self.decode, self.keep = name, url, z, ext, decode, keep
        self.recent = collections.OrderedDict()
        self.missing = set()  # tiles the server doesn't have (404): not cached on disk, so remember them

    def _get(self, t):
        if t in self.missing:
            return None
        x, y = t
        raw = fetch(self.url.format(z=self.z, x=x, y=y),
                    os.path.join(CACHE_DIR, self.name, str(self.z), str(x), f"{y}.{self.ext}"))
        if raw is None:
            self.missing.add(t)
        return raw

    def prefetch(self, tiles):
        tiles = sorted(set(tiles))
        print(f"  {self.name}: zoom {self.z}, {len(tiles)} tiles", flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
            for done, _ in enumerate(pool.map(self._get, tiles), 1):
                if done % 500 == 0:
                    print(f"    {done}/{len(tiles)}", flush=True)

    def tile(self, x, y):
        t = self.recent.get((x, y))
        if t is None:
            t = self.decode(self._get((x, y)))
            if self.keep:
                self.recent[(x, y)] = t
                if len(self.recent) > self.keep:
                    self.recent.popitem(last=False)
        else:
            self.recent.move_to_end((x, y))
        return t

    def mosaic(self, x0, y0, x1, y1, prefetch=True):
        if prefetch:
            self.prefetch((x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1))
        data = None
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                t = self.tile(x, y)
                if data is None:
                    data = np.zeros(((y1 - y0 + 1) * 256, (x1 - x0 + 1) * 256) + t.shape[2:], dtype=t.dtype)
                data[(y - y0) * 256:(y - y0 + 1) * 256, (x - x0) * 256:(x - x0 + 1) * 256] = t
        return Mosaic(self.z, x0, y0, data)

    def area(self, lat_range, lon_range):
        """The whole lat/lon box stitched in memory: for areas a few thousand pixels across."""
        return self.mosaic(*tile_range(lat_range, lon_range, self.z))

    def sample(self, lat, lon):
        # Only the tiles under these points (plus a pixel for the bilinear neighbours), from the cache.
        return self.mosaic(*tile_range(lat, lon, self.z, pad_px=1.0), prefetch=False).sample(lat, lon)


class Mosaic:
    """Web Mercator tiles of one zoom stitched into one array, sampled bilinearly."""

    def __init__(self, z, tx0, ty0, data):
        self.z, self.tx0, self.ty0, self.data = z, tx0, ty0, data

    def sample(self, lat, lon):
        x, y = mercator_px(lat, lon, self.z)
        x = x - self.tx0 * 256 - 0.5
        y = y - self.ty0 * 256 - 0.5
        h, w = self.data.shape[:2]
        x = np.clip(x, 0, w - 1.001)
        y = np.clip(y, 0, h - 1.001)
        x0, y0 = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
        fx, fy = x - x0, y - y0
        if self.data.ndim == 3:
            fx, fy = fx[..., None], fy[..., None]
        d = self.data
        f32 = np.float32
        top = d[y0, x0].astype(f32) * (1 - fx) + d[y0, x0 + 1].astype(f32) * fx
        bottom = d[y0 + 1, x0].astype(f32) * (1 - fx) + d[y0 + 1, x0 + 1].astype(f32) * fx
        return top * (1 - fy) + bottom * fy


def decode_jpeg(raw):
    if raw is None:
        return np.zeros((256, 256, 3), dtype=np.uint8)
    return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"), dtype=np.uint8)


def decode_terrarium(raw):
    if raw is None:
        return np.zeros((256, 256), dtype=np.float32)
    rgb = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"), dtype=np.float32)
    return rgb[..., 0] * 256.0 + rgb[..., 1] + rgb[..., 2] / 256.0 - 32768.0


def geo_box(frame, half_m):
    return geo_bounds(frame, -half_m, -half_m, half_m, half_m)


def geo_bounds(frame, west, south, east, north):
    """Lat/lon box around a plane rectangle; edges are sampled since meridians converge across it."""
    t = np.linspace(0.0, 1.0, 9)
    e = np.concatenate([west + (east - west) * t, np.full(9, east), west + (east - west) * t, np.full(9, west)])
    n = np.concatenate([np.full(9, south), south + (north - south) * t, np.full(9, north), south + (north - south) * t])
    lat, lon = frame.to_geo(e, n)
    pad = 0.02
    return (float(lat.min()) - pad, float(lat.max()) + pad), (float(lon.min()) - pad * 2, float(lon.max()) + pad * 2)


def airport_flatten(frame, e, n, h):
    """Blends the terrain to runway level around each runway, so the modelled runway sits on it."""
    for airport in AIRPORTS.values():
        e0, e1 = (frame.to_enu(lat, lon) for lat, lon in airport.ends)
        de, dn = e1[0] - e0[0], e1[1] - e0[1]
        length = math.hypot(de, dn)
        ue, un = de / length, dn / length
        me, mn = (e0[0] + e1[0]) / 2.0, (e0[1] + e1[1]) / 2.0
        along = np.abs((e - me) * ue + (n - mn) * un) - (length / 2.0 + 1000.0)
        across = np.abs(-(e - me) * un + (n - mn) * ue) - 600.0
        dist = np.hypot(np.maximum(along, 0.0), np.maximum(across, 0.0))
        w = np.clip(dist / 700.0, 0.0, 1.0)
        w = w * w * (3.0 - 2.0 * w)
        level = airport_level(airport)
        h = np.where(w < 1.0, level + (h - level) * w, h)  # exactly unchanged away from the airport
    return h


def airport_level(airport):
    return airport.elev_ft * 0.3048 - REF[2]  # metres above the reference: 0 at EETN


def ground_heights(frame, elevation, ee, nn):
    lat, lon = frame.to_geo(ee, nn)
    h = np.maximum(elevation.sample(lat, lon), 0.0) - REF[2]  # sea level, not the sea floor
    return airport_flatten(frame, ee, nn, h)


def tile_heights(frame, elevation, south, west, size, grid):
    # grid x grid samples, rows south to north, columns west to east.
    t = np.linspace(0.0, size, grid)
    ee, nn = np.meshgrid(west + t, south + t)
    return ground_heights(frame, elevation, ee, nn).astype("<f4")


def calm_sea(rgb, sea):
    """Replaces the imagery's blue sea fill (blue well above green) with real water's colour, at sea only."""
    w = np.clip((rgb[..., 2] - rgb[..., 1]) / 12.0, 0.0, 1.0) * sea
    return rgb + (np.array(WATER_RGB, dtype=np.float32) - rgb) * w[..., None]


def write_tile(frame, out_dir, name, south, west, size, heights, px, imagery, quality=87, hole=0.0, sea_from=None):
    heights.astype("<f4").tofile(os.path.join(out_dir, name + ".f32"))
    # Image: pixel centres, north at the top; in strips so a 4096 px image needs little memory.
    c = (np.arange(px) + 0.5) / px * size
    rgb = np.zeros((px, px, 3), dtype=np.uint8)
    for r0 in range(0, px, 512):
        ee, nn = np.meshgrid(west + c, south + size - c[r0:r0 + 512])
        lat, lon = frame.to_geo(ee, nn)
        strip = imagery.sample(lat, lon)
        if sea_from is not None:
            strip = calm_sea(strip, (sea_from.sample(lat, lon) <= 0.0).astype(np.float32))
        rgb[r0:r0 + 512] = np.clip(strip + 0.5, 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(os.path.join(out_dir, name + ".jpg"), quality=quality, optimize=True)
    return f"{name}|{south:.1f}|{west:.1f}|{size:.1f}|{heights.shape[0]}|{hole:.1f}"


def min_filter(a, k):
    """Lowest value within k samples (a square window) of each sample; inf counts as nothing."""
    for axis in (0, 1):
        pad = [(0, 0), (0, 0)]
        pad[axis] = (k, k)
        p = np.pad(a, pad, constant_values=np.inf)
        out = np.full(a.shape, np.inf, dtype=a.dtype)
        n = a.shape[axis]
        for d in range(2 * k + 1):
            out = np.minimum(out, p[d:d + n] if axis == 0 else p[:, d:d + n])
        a = out
    return a


class Coverage:
    """Heights of a finer layer on a regular grid, inf where that layer has no tile."""

    def __init__(self, west, south, spacing, heights):
        self.west, self.south, self.spacing, self.h = west, south, spacing, heights

    def lowest_near(self, ee, nn, reach):
        """Per point: the lowest covered height within `reach` metres (each axis), inf if none."""
        k = int(math.ceil(reach / self.spacing - 1e-6))
        low = min_filter(self.h.astype(np.float32), k)
        c = np.rint((ee - self.west) / self.spacing).astype(np.int64)
        r = np.rint((nn - self.south) / self.spacing).astype(np.int64)
        inside = (r >= 0) & (r < low.shape[0]) & (c >= 0) & (c < low.shape[1])
        out = np.full(ee.shape, np.inf, dtype=np.float32)
        out[inside] = low[r[inside], c[inside]]
        return out

    def surface(self, ee, nn):
        """Heights on the mesh the sim builds from this grid (A320Terrain.cpp: each cell split along
        its south-west to north-east diagonal), inf outside the grid or where no tile is."""
        x, y = (np.asarray(ee) - self.west) / self.spacing, (np.asarray(nn) - self.south) / self.spacing
        rows, cols = self.h.shape
        c = np.clip(np.floor(x).astype(np.int64), 0, cols - 2)
        r = np.clip(np.floor(y).astype(np.int64), 0, rows - 2)
        fc, fr = x - c, y - r
        a, b, cc, d = self.h[r, c], self.h[r, c + 1], self.h[r + 1, c], self.h[r + 1, c + 1]
        with np.errstate(invalid="ignore"):
            out = np.where(fr >= fc, a + fc * (d - cc) + fr * (cc - a), a + fc * (b - a) + fr * (d - b))
        inside = (x >= 0) & (x <= cols - 1) & (y >= 0) & (y <= rows - 1)
        return np.where(inside & np.isfinite(out), out, np.inf)


def base_heights(frame, elevation, south, west, size, grid, coverages):
    """The always-loaded base layer: true heights, but under any finer tile it is pushed below it.

    A base vertex is lowered to the lowest finer sample within one base cell plus one fine cell,
    minus BASE_DROP_M. Every point of a base triangle lies within that reach of all three of its
    vertices, so the base surface stays at least BASE_DROP_M under the finer mesh everywhere.
    """
    h = tile_heights(frame, elevation, south, west, size, grid).astype(np.float32)
    t = np.linspace(0.0, size, grid)
    ee, nn = np.meshgrid(west + t, south + t)
    step = size / (grid - 1)
    low = np.full(h.shape, np.inf, dtype=np.float32)
    for cov in coverages:
        low = np.minimum(low, cov.lowest_near(ee, nn, step + cov.spacing))
    covered = np.isfinite(low)
    h[covered] = np.minimum(h[covered], low[covered]) - BASE_DROP_M
    return h.astype("<f4"), int(covered.sum())


def region_tiles(frame, bounds, tile, elevation_zoom):
    """The tile grid (aligned with the detailed area) over a lat/lon box, and its elevation."""
    west, south, rows, cols = region_grid(frame, bounds, tile)
    lat_r, lon_r = geo_bounds(frame, west, south, west + cols * tile, south + rows * tile)
    elevation = Source("elevation", ELEVATION_URL, elevation_zoom, "png", decode_terrarium).area(lat_r, lon_r)
    return west, south, rows, cols, elevation


def region_grid(frame, bounds, tile):
    """West, south, rows and columns of the tile grid (aligned with the detailed area) over a lat/lon box."""
    lat0, lat1, lon0, lon1 = bounds
    t = np.linspace(0.0, 1.0, 33)
    lat = np.concatenate([lat0 + (lat1 - lat0) * t, np.full(33, lat1), lat0 + (lat1 - lat0) * t, np.full(33, lat0)])
    lon = np.concatenate([np.full(33, lon0), lon0 + (lon1 - lon0) * t, np.full(33, lon1), lon0 + (lon1 - lon0) * t])
    e, n = frame.to_enu(lat, lon)
    west, east = math.floor(e.min() / tile) * tile, math.ceil(e.max() / tile) * tile
    south, north = math.floor(n.min() / tile) * tile, math.ceil(n.max() / tile) * tile
    return west, south, int(round((north - south) / tile)), int(round((east - west) / tile))


def overpass(query, path, what):
    """An Overpass API answer (parsed JSON), cached at path; None if no server gives a complete one."""
    if not os.path.exists(path):
        body = urllib.parse.urlencode({"data": query}).encode()
        last = None
        for attempt in range(3):  # the public servers are often busy: try them all, a few times
            for url in OVERPASS_URLS:
                try:
                    print(f"  {what}: asking {url}", flush=True)
                    req = urllib.request.Request(url, data=body, headers={"User-Agent": USER_AGENT})
                    with urllib.request.urlopen(req, timeout=400) as r:
                        data = r.read()
                    remark = json.loads(data).get("remark", "")
                    if "error" in remark.lower():  # a timed-out query still answers 200, with partial data
                        raise RuntimeError(remark)
                    os.makedirs(os.path.dirname(path), exist_ok=True)
                    with open(path + ".part", "wb") as f:
                        f.write(data)
                    os.replace(path + ".part", path)
                    break
                except Exception as e:  # noqa: BLE001 - any mirror failure: try the next one
                    last = e
            else:
                time.sleep(30 * (attempt + 1))
                continue
            break
        else:
            print(f"  {what}: no Overpass server answered ({last})")
            return None
    with open(path, "rb") as f:
        return json.load(f)


def fetch_buildings(frame, half_m, centre=(0.0, 0.0), cache_name=None, seen=None):
    """OpenStreetMap buildings in the square of half width half_m around centre (east, north).

    seen: OSM way ids already taken from other areas, skipped here; updated with the new ones.
    """
    ce, cn = centre
    (lat0, lat1), (lon0, lon1) = geo_bounds(frame, ce - half_m, cn - half_m, ce + half_m, cn + half_m)
    query = (f'[out:json][timeout:300];(way["building"]({lat0:.5f},{lon0:.5f},{lat1:.5f},{lon1:.5f}););'
             "out tags geom;")
    data = overpass(query, os.path.join(CACHE_DIR, cache_name or f"buildings_{int(half_m)}.json"), "buildings")
    if data is None:
        print("  buildings: skipped")
        return None
    elements = data.get("elements", [])

    boxes = []
    for el in elements:
        if seen is not None:
            if el.get("id") in seen:
                continue
            seen.add(el.get("id"))
        geom = el.get("geometry")
        if not geom or len(geom) < 3:
            continue
        tags = el.get("tags", {})
        e, n = frame.to_enu(np.array([g["lat"] for g in geom]), np.array([g["lon"] for g in geom]))
        pts = np.stack([e, n], axis=1)[:-1] if len(geom) > 3 else np.stack([e, n], axis=1)
        box = oriented_box(pts)
        if box is None:
            continue
        boxes.append(box + (building_height(tags, box[2] * box[3]),))
    return boxes


def oriented_box(pts):
    """Minimum-area rectangle over the footprint's edge directions: (north, east, length, width, yaw)."""
    best = None
    for i in range(len(pts)):
        d = pts[(i + 1) % len(pts)] - pts[i]
        if np.hypot(*d) < 0.5:
            continue
        a = math.atan2(d[1], d[0])  # angle from east towards north
        ca, sa = math.cos(a), math.sin(a)
        u = pts[:, 0] * ca + pts[:, 1] * sa
        v = -pts[:, 0] * sa + pts[:, 1] * ca
        area = (u.max() - u.min()) * (v.max() - v.min())
        if best is None or area < best[0]:
            cu, cv = (u.max() + u.min()) / 2.0, (v.max() + v.min()) / 2.0
            best = (area, cu * ca - cv * sa, cu * sa + cv * ca, u.max() - u.min(), v.max() - v.min(), a)
    if best is None or best[3] < 2.0 or best[4] < 2.0:
        return None
    _, ce, cn, length, width, a = best
    yaw = 90.0 - math.degrees(a)  # compass-style: from north towards east, as the sim's yaw
    return (cn, ce, length, width, yaw)


def building_height(tags, area):
    for key in ("height", "building:height"):
        try:
            return max(3.0, min(float(tags[key].split()[0].replace(",", ".")), 350.0))
        except (KeyError, ValueError, IndexError):
            pass
    try:
        return max(3.0, float(tags["building:levels"]) * 3.2 + 1.0)
    except (KeyError, ValueError):
        pass
    kind = tags.get("building", "yes")
    if kind in ("house", "detached", "garage", "garages", "shed", "hut", "cabin", "barn", "farm_auxiliary"):
        return 5.5
    if kind in ("apartments", "residential"):
        return 15.0
    if kind in ("hangar", "industrial", "warehouse", "commercial", "retail"):
        return 10.0
    return 6.0 if area < 200.0 else 9.0


def building_rows(frame, elevation, half_m, centre=(0.0, 0.0), cache_name=None, seen=None, surface=None):
    """buildings.bin rows (north, east, ground, length, width, yaw, height) for one area, or None.

    surface: optional (east, north) -> heights of the mesh the sim draws there, inf where it has none;
    elsewhere the ground is the true (flattened) height under each building.
    """
    boxes = fetch_buildings(frame, half_m, centre, cache_name, seen)
    if not boxes:
        return None
    b = np.array(boxes, dtype=np.float64)
    ground = ground_heights(frame, elevation, b[:, 1], b[:, 0])
    if surface is not None:
        drawn = surface(b[:, 1], b[:, 0])
        ground = np.where(np.isfinite(drawn), drawn, ground)
    return np.column_stack([b[:, 0], b[:, 1], ground, b[:, 2], b[:, 3], b[:, 4], b[:, 5]]).astype("<f4")


def write_patch(frame, args, code, tile, taken, seen=None):
    """Detailed tiles over the grid squares within the patch radius of an airport, and its buildings.

    taken: (south, west) of squares that already have a detailed tile; updated with the new ones.
    seen: OSM building ids already written (see fetch_buildings).
    """
    ce, cn = frame.to_enu(*AIRPORTS[code].ref)
    patch_km, buildings_km = patch_radii_km(args, code)
    half = patch_km * 1000.0
    west, south = math.floor((ce - half) / tile) * tile, math.floor((cn - half) / tile) * tile
    cols = int(math.ceil((ce + half) / tile)) - int(west / tile)
    rows = int(math.ceil((cn + half) / tile)) - int(south / tile)
    lat_r, lon_r = geo_bounds(frame, west, south, west + cols * tile, south + rows * tile)
    imagery = Source("imagery", IMAGERY_URL, 13, "jpg", decode_jpeg).area(lat_r, lon_r)
    elevation = Source("elevation", ELEVATION_URL, 11, "png", decode_terrarium).area(lat_r, lon_r)
    grid = int(tile / 100.0) + 1
    heights = np.full((rows * (grid - 1) + 1, cols * (grid - 1) + 1), np.inf, dtype=np.float32)
    lines = []
    for r in range(rows):
        for c in range(cols):
            s0, w0 = south + r * tile, west + c * tile
            if square_key(s0, w0) in taken:
                continue
            taken.add(square_key(s0, w0))
            h = tile_heights(frame, elevation, s0, w0, tile, grid)
            heights[r * (grid - 1):r * (grid - 1) + grid, c * (grid - 1):c * (grid - 1) + grid] = h
            lines.append("detail=" + write_tile(frame, OUT_DIR, f"{code.lower()}_{r}_{c}", s0, w0, tile, h,
                                                args.tile_px, imagery))
    print(f"  {len(lines)} tiles of {rows}x{cols} squares written", flush=True)
    rows_b = None
    if buildings_km > 0:
        half_b = buildings_km * 1000.0
        rows_b = building_rows(frame, elevation, half_b, (ce, cn), f"buildings_{code.lower()}_{int(half_b)}.json",
                               seen)
        print(f"  {0 if rows_b is None else len(rows_b)} buildings", flush=True)
    extent = (west, south, west + cols * tile, south + rows * tile)
    return lines, Coverage(west, south, tile / (grid - 1), heights), rows_b, extent


def patch_radii_km(args, code):
    """Half widths (km) of an airport's detailed patch and of its buildings area."""
    airport = AIRPORTS[code]
    patch = args.patch_km if args.patch_km is not None else airport.patch_km or 15.0
    buildings = args.patch_buildings_km if args.patch_buildings_km is not None else airport.buildings_km or 8.0
    return patch, buildings


TOWNS_QUERY = ('[out:json][timeout:180];area["ISO3166-1"="EE"][admin_level=2]->.ee;'
               'node["place"~"^(city|town)$"](area.ee);out;')


def fetch_towns(min_population):
    """Estonia's cities and towns (OSM place nodes) of at least min_population: (name, population, lat, lon)."""
    data = overpass(TOWNS_QUERY, os.path.join(CACHE_DIR, "towns_ee.json"), "towns")
    towns = []
    for el in (data or {}).get("elements", []):
        tags = el.get("tags", {})
        digits = "".join(ch for ch in tags.get("population", "").split(";")[0] if ch.isdigit())
        if digits and int(digits) >= min_population and "lat" in el:
            towns.append((tags.get("name", str(el["id"])), int(digits), el["lat"], el["lon"]))
    return sorted(towns, key=lambda t: (-t[1], t[0]))


def town_half_km(population):
    """Half width of a town's buildings area: 2.5 km at 4,000 people, up to 6 km at 50,000 and more."""
    t = (math.log10(population) - math.log10(4000.0)) / (math.log10(50000.0) - math.log10(4000.0))
    return 2.5 + 3.5 * min(max(t, 0.0), 1.0)


def town_buildings(frame, args, tile, areas, seen, taken, region):
    """buildings.bin rows for each town of --town-population not inside an area done before.

    areas: (east, north, half width) of the buildings areas so far, updated; region: the region
    layer's Coverage, whose mesh is what the sim draws where no detailed tile is (taken: those squares).
    """
    out = []
    for name, population, lat, lon in fetch_towns(args.town_population):
        ce, cn = frame.to_enu(lat, lon)
        half = town_half_km(population) * 1000.0
        if any(abs(ce - e) + half <= h and abs(cn - n) + half <= h for e, n, h in areas):
            print(f"  {name}: inside an area done before", flush=True)
            continue
        areas.append((ce, cn, half))

        def surface(ee, nn):
            if region is None:
                return np.full(np.shape(ee), np.inf)
            detailed = np.array([square_key(math.floor(n / tile) * tile, math.floor(e / tile) * tile) in taken
                                 for e, n in zip(ee, nn)], dtype=bool)
            return np.where(detailed, np.inf, region.surface(ee, nn))

        lat_r, lon_r = geo_bounds(frame, ce - half, cn - half, ce + half, cn + half)
        elevation = Source("elevation", ELEVATION_URL, 11, "png", decode_terrarium).area(lat_r, lon_r)
        slug = "".join(c if c.isalnum() else "-" for c in
                       unicodedata.normalize("NFKD", name).encode("ascii", "ignore").decode().lower())
        rows = building_rows(frame, elevation, half, (ce, cn), f"buildings_town_{slug}_{int(half)}.json", seen,
                             surface)
        print(f"  {name} ({population} people, {half / 1000:.1f} km): {0 if rows is None else len(rows)} buildings",
              flush=True)
        if rows is not None:
            out.append(rows)
    return out


def write_ground(frame, bounds, tile, spacing):
    """ground.txt / ground.i16: the flattened terrain heights on a regular grid over the region box."""
    west, south, rows, cols = region_grid(frame, bounds, tile)
    nr, nc = int(round(rows * tile / spacing)) + 1, int(round(cols * tile / spacing)) + 1
    e = west + np.arange(nc) * spacing
    elevation = Source("elevation", ELEVATION_URL, 11, "png", decode_terrarium, keep=400)
    bands = [np.arange(r0, min(r0 + 40, nr)) for r0 in range(0, nr, 40)]
    needed = set()
    for band in bands:
        ee, nn = np.meshgrid(e, south + band * spacing)
        x0, y0, x1, y1 = tile_range(*frame.to_geo(ee, nn), elevation.z, pad_px=1.0)
        needed.update((x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1))
    elevation.prefetch(needed)
    heights = np.zeros((nr, nc), dtype="<i2")
    for band in bands:
        ee, nn = np.meshgrid(e, south + band * spacing)
        h = ground_heights(frame, elevation, ee, nn)
        heights[band] = np.clip(np.rint(h * 10.0), -32768, 32767).astype("<i2")
    heights.tofile(os.path.join(OUT_DIR, "ground.i16"))
    with open(os.path.join(OUT_DIR, "ground.txt"), "w", encoding="utf-8") as f:
        f.write(f"version=1\norigin={south:.1f}|{west:.1f}\nspacing={spacing:.1f}\nsize={nr}|{nc}\n"
                "# int16 little-endian, decimetres above the EETN field (same reference as the terrain heights), "
                "rows south to north, columns west to east\n")
    worst = 0.0
    for airport in AIRPORTS.values():
        (e0, n0), (e1, n1) = (frame.to_enu(lat, lon) for lat, lon in airport.ends)
        x, y = ((e0 + e1) / 2.0 - west) / spacing, ((n0 + n1) / 2.0 - south) / spacing
        c, r = int(x), int(y)
        fx, fy = x - c, y - r
        g = heights[r:r + 2, c:c + 2].astype(np.float64) / 10.0
        at = (g[0, 0] * (1 - fx) + g[0, 1] * fx) * (1 - fy) + (g[1, 0] * (1 - fx) + g[1, 1] * fx) * fy
        worst = max(worst, abs(at - airport_level(airport)))
    print(f"  {nr}x{nc} heights every {spacing:.0f} m, {heights.nbytes / 1e6:.1f} MB; "
          f"runway midpoints within {worst:.2f} m of their level", flush=True)


LAYOUT_RADIUS_M = 4000.0
LAYOUT_MAX_POINTS = 200


def write_layouts(frame):
    """Content/Airports/<ICAO>.txt: taxiway centrelines and apron outlines from OpenStreetMap."""
    os.makedirs(AIRPORTS_DIR, exist_ok=True)
    for code, airport in AIRPORTS.items():
        ce, cn = frame.to_enu(*airport.ref)
        r = LAYOUT_RADIUS_M
        t = np.linspace(-r, r, 9)
        lat, lon = frame.to_geo(np.concatenate([t, t, np.full(9, -r), np.full(9, r)]) + ce,
                                np.concatenate([np.full(9, -r), np.full(9, r), t, t]) + cn)
        bbox = f"{lat.min():.5f},{lon.min():.5f},{lat.max():.5f},{lon.max():.5f}"
        query = (f'[out:json][timeout:120];(way["aeroway"~"^(taxiway|apron)$"]({bbox});'
                 f'relation["aeroway"="apron"]({bbox}););out tags geom;')
        data = overpass(query, os.path.join(CACHE_DIR, f"airport_{code.lower()}.json"), f"{code} layout")
        if data is None:
            print(f"  {code}: no layout (kept the old file, if any)")
            continue

        def enu(geom):
            e, n = frame.to_enu(np.array([g[0] for g in geom]), np.array([g[1] for g in geom]))
            return [(round(float(nv), 1), round(float(ev), 1)) for nv, ev in zip(n, e)]

        def near(p):
            return math.hypot(p[0] - cn, p[1] - ce) <= r

        elements = data.get("elements", [])
        in_relation = {m["ref"] for el in elements if el["type"] == "relation"
                       for m in el.get("members", []) if m.get("type") == "way"}
        taxiways, aprons, dropped = [], [], 0
        for el in elements:
            tags = el.get("tags", {})
            ref = layout_ref(tags)
            if el["type"] == "way" and tags.get("aeroway") == "taxiway":
                width = osm_width(tags) or airport.taxiway_m
                run = []
                for p in dedupe(enu([(g["lat"], g["lon"]) for g in el.get("geometry", [])])) + [None]:
                    if p is not None and near(p):
                        run.append(p)
                        continue
                    if len(run) >= 2:  # the part within reach of the airport
                        taxiways.append((ref, width, run))
                    run = []
            elif tags.get("aeroway") == "apron":
                if el["type"] == "way":
                    geom = [(g["lat"], g["lon"]) for g in el.get("geometry", [])]
                    closed = len(geom) >= 4 and geom[0] == geom[-1]
                    rings = [geom[:-1]] if closed and el["id"] not in in_relation else []
                else:
                    rings = join_rings([[(g["lat"], g["lon"]) for g in m.get("geometry", [])]
                                        for m in el.get("members", []) if m.get("role") == "outer"])
                for ring in rings:
                    pts = simple_ring(dedupe(enu(ring), closed=True))
                    if pts is None or not all(near(p) for p in pts):
                        dropped += 1
                        continue
                    aprons.append((ref, pts))

        lines = [f"# {code} taxiways and aprons, generated by tools/make_terrain.py (see there for the format).",
                 "# From OpenStreetMap: (c) OpenStreetMap contributors, ODbL. North,east metres in the sim's frame.",
                 f"icao={code}", f"level={airport_level(airport):.1f}"]
        lines += [f"taxiway={ref}|{w:.1f}|" + ";".join(f"{n:.1f},{e:.1f}" for n, e in pts) for ref, w, pts in taxiways]
        lines += [f"apron={ref}|" + ";".join(f"{n:.1f},{e:.1f}" for n, e in pts) for ref, pts in aprons]
        with open(os.path.join(AIRPORTS_DIR, f"{code}.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"  {code}: {len(taxiways)} taxiways, {len(aprons)} aprons"
              + (f", {dropped} aprons skipped (not simple or beyond {r / 1000:.0f} km)" if dropped else ""), flush=True)


def layout_ref(tags):
    ref = tags.get("ref", "").strip()
    return "".join("/" if c in "|;=\n" else c for c in ref) or "-"


def osm_width(tags):
    """The OSM width tag in metres (plain metres, or feet marked ' or ft), None if absent or odd."""
    text = tags.get("width", "").strip().lower()
    try:
        value = float(text.replace(",", ".").rstrip("m'ft ").split()[0])
    except (ValueError, IndexError):
        return None
    value *= 0.3048 if ("'" in text or "ft" in text) else 1.0
    return value if 3.0 <= value <= 100.0 else None


def dedupe(pts, closed=False):
    """Drops points equal to the one before (after rounding), and for a ring a last point equal to the first."""
    out = [p for i, p in enumerate(pts) if i == 0 or p != pts[i - 1]]
    while closed and len(out) > 1 and out[-1] == out[0]:
        out.pop()
    return out


def join_rings(parts):
    """Closed rings (last point not repeated) from multipolygon member ways joined end to end."""
    parts = [list(p) for p in parts if len(p) >= 2]
    rings = []
    while parts:
        ring = parts.pop(0)
        while ring[0] != ring[-1]:
            for i, p in enumerate(parts):
                if p[0] == ring[-1] or p[-1] == ring[-1]:
                    ring += (p if p[0] == ring[-1] else p[::-1])[1:]
                    parts.pop(i)
                    break
            else:
                break  # an unclosed ring: broken data, left out
        if len(ring) >= 4 and ring[0] == ring[-1]:
            rings.append(ring[:-1])
    return rings


def simple_ring(pts):
    """The outline with at most LAYOUT_MAX_POINTS points (Douglas-Peucker), or None if it can't be simple."""
    if len(pts) < 3:
        return None
    tol = 0.0
    for _ in range(40):
        out = pts if tol == 0.0 else simplify_ring(pts, tol)
        if 3 <= len(out) <= LAYOUT_MAX_POINTS and is_simple(out):
            return out
        tol = 0.25 if tol == 0.0 else tol * 1.4
    return None


def simplify_ring(pts, tol):
    a = np.array(pts, dtype=np.float64)
    far = int(np.argmax(np.hypot(*(a - a[0]).T)))
    keep = np.zeros(len(a), dtype=bool)
    keep[[0, far]] = True
    for lo, hi in ((0, far), (far, len(a))):
        stack = [(lo, hi)]
        while stack:
            i, j = stack.pop()
            if j - i < 2:
                continue
            p, q = a[i], a[j % len(a)]
            seg = q - p
            mid = a[i + 1:j]
            d = np.abs(seg[0] * (mid[:, 1] - p[1]) - seg[1] * (mid[:, 0] - p[0])) / max(np.hypot(*seg), 1e-9)
            k = int(np.argmax(d))
            if d[k] > tol:
                keep[i + 1 + k] = True
                stack += [(i, i + 1 + k), (i + 1 + k, j)]
    return [pts[i] for i in np.flatnonzero(keep)]


def is_simple(pts):
    """True if no two edges of the closed outline meet except neighbours at their shared corner."""
    a = np.array(pts, dtype=np.float64)
    n = len(a)
    if len(set(pts)) != n:
        return False
    p, q = a, np.roll(a, -1, axis=0)

    def orient(o, u, v):
        return np.sign((u[..., 0] - o[..., 0]) * (v[..., 1] - o[..., 1])
                       - (u[..., 1] - o[..., 1]) * (v[..., 0] - o[..., 0]))

    for i in range(n):
        j = np.arange(i + 2, n)
        if i == 0:
            j = j[j != n - 1]
        if not len(j):
            continue
        d1, d2 = orient(p[i], q[i], p[j]), orient(p[i], q[i], q[j])
        d3, d4 = orient(p[j], q[j], p[i]), orient(p[j], q[j], q[i])
        if np.any((d1 * d2 < 0) & (d3 * d4 < 0)):
            return False
        for k in np.flatnonzero((d1 == 0) | (d2 == 0) | (d3 == 0) | (d4 == 0)):  # touching or collinear
            jj = j[k]
            if segments_touch(p[i], q[i], p[jj], q[jj]):
                return False
    return True


def segments_touch(p1, q1, p2, q2):
    def on(o, u, v):  # v on segment o-u, given collinear
        return min(o[0], u[0]) - 1e-9 <= v[0] <= max(o[0], u[0]) + 1e-9 and \
            min(o[1], u[1]) - 1e-9 <= v[1] <= max(o[1], u[1]) + 1e-9

    def cross(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    for o, u, v in ((p1, q1, p2), (p1, q1, q2), (p2, q2, p1), (p2, q2, q1)):
        if cross(o, u, v) == 0 and on(o, u, v):
            return True
    return False


def union(a, b):
    return min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])


def square_key(south, west):
    return int(round(south)), int(round(west))


def parse_airports(text):
    codes = [] if text.lower() == "none" else [c.strip().upper() for c in text.split(",") if c.strip()]
    for c in codes:
        if c not in AIRPORTS or c == "EETN":
            raise argparse.ArgumentTypeError(f"{c}: expected none or codes from {', '.join(AIRPORTS)} but EETN")
    return codes


def parse_box(text):
    if text.lower() == "none":
        return None
    v = [float(x) for x in text.split(",")]
    if len(v) != 4 or v[0] >= v[1] or v[2] >= v[3]:
        raise argparse.ArgumentTypeError("expected LAT_MIN,LAT_MAX,LON_MIN,LON_MAX or none")
    return tuple(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inner-km", type=float, default=40.0, help="half width of the detailed area (km)")
    ap.add_argument("--tile-km", type=float, default=10.0, help="tile size of the detailed and region layers (km)")
    ap.add_argument("--tile-px", type=int, default=1024, help="detailed tile image size (px)")
    ap.add_argument("--region", type=parse_box, default=ESTONIA,
                    help="region layer box LAT_MIN,LAT_MAX,LON_MIN,LON_MAX (default: Estonia), or none")
    ap.add_argument("--region-px", type=int, default=512, help="region tile image size (px)")
    ap.add_argument("--region-grid", type=int, default=51, help="region tile height samples per side")
    ap.add_argument("--region-zoom", type=int, default=12, help="region imagery zoom (12: about 20 m/px)")
    ap.add_argument("--region-quality", type=int, default=80, help="region JPEG quality")
    ap.add_argument("--outer-margin-km", type=float, default=80.0, help="low-detail base beyond the region (km)")
    ap.add_argument("--outer-px", type=int, default=4096, help="low-detail base image size (px)")
    ap.add_argument("--outer-step-km", type=float, default=2.0, help="low-detail base height spacing (km)")
    ap.add_argument("--buildings-km", type=float, default=20.0, help="half width of the buildings area (km), 0 = none")
    ap.add_argument("--airports", type=parse_airports, default=[c for c in AIRPORTS if c != "EETN"],
                    help="other airports (codes from AIRPORTS) that get a detailed patch, comma-separated, or none "
                         "(default: all but EETN)")
    ap.add_argument("--patch-km", type=float, default=None,
                    help="half width of every airport's detailed patch (km); default: per airport in AIRPORTS")
    ap.add_argument("--patch-buildings-km", type=float, default=None,
                    help="half width of every airport patch's buildings area (km), 0 = none; default: per airport")
    ap.add_argument("--town-population", type=int, default=4000,
                    help="buildings also around every city and town of at least this many people, 0 = none")
    ap.add_argument("--ground-m", type=float, default=250.0,
                    help="spacing of the flight model's ground height map (m), 0 = none")
    args = ap.parse_args()

    frame = Frame(*REF)
    os.makedirs(REGION_DIR, exist_ok=True)
    for d in (OUT_DIR, REGION_DIR):
        for f in os.listdir(d):
            if f.endswith((".jpg", ".f32", ".bin", ".i16")) or f in ("terrain.txt", "ground.txt"):
                os.remove(os.path.join(d, f))
    lines = ["# Generated by tools/make_terrain.py; see that file for the format and the sources.",
             "version=2", f"reference={REF[0]:.7f},{REF[1]:.7f},{REF[2]:.2f}", f"attribution={ATTRIBUTION}"]

    inner = args.inner_km * 1000.0
    tile = args.tile_km * 1000.0
    count = int(round(2 * inner / tile))
    coverages = []

    print("Detailed area", flush=True)
    lat_r, lon_r = geo_box(frame, inner)
    imagery = Source("imagery", IMAGERY_URL, 13, "jpg", decode_jpeg).area(lat_r, lon_r)
    elevation = Source("elevation", ELEVATION_URL, 11, "png", decode_terrarium).area(lat_r, lon_r)
    grid = int(tile / 100.0) + 1
    detail = np.zeros((count * (grid - 1) + 1,) * 2, dtype=np.float32)
    for r in range(count):
        for c in range(count):
            south, west = -inner + r * tile, -inner + c * tile
            h = tile_heights(frame, elevation, south, west, tile, grid)
            detail[r * (grid - 1):r * (grid - 1) + grid, c * (grid - 1):c * (grid - 1) + grid] = h
            lines.append("detail=" + write_tile(frame, OUT_DIR, f"tile_{r}_{c}", south, west, tile, h,
                                                args.tile_px, imagery))
        print(f"  row {r + 1}/{count} written", flush=True)
    coverages.append(Coverage(-inner, -inner, tile / (grid - 1), detail))
    taken = {square_key(-inner + r * tile, -inner + c * tile) for r in range(count) for c in range(count)}

    buildings = []
    seen = set()  # OSM ids of the buildings written: each once, where areas overlap
    areas = []  # (east, north, half width) of the buildings areas
    if args.buildings_km > 0:
        print("Buildings", flush=True)
        half_b = min(args.buildings_km * 1000.0, inner)
        out = building_rows(frame, elevation, half_b, seen=seen)
        areas.append((0.0, 0.0, half_b))
        if out is not None:
            buildings.append(out)
            print(f"  {len(out)} buildings")
    del imagery, elevation

    extent = (-inner, -inner, inner, inner)  # west, south, east, north
    for code in args.airports:
        print(f"Detailed area around {code}", flush=True)
        patch_lines, cov, out, box = write_patch(frame, args, code, tile, taken, seen)
        lines += patch_lines
        coverages.append(cov)
        extent = union(extent, box)
        if patch_radii_km(args, code)[1] > 0:
            areas.append(frame.to_enu(*AIRPORTS[code].ref) + (patch_radii_km(args, code)[1] * 1000.0,))
        if out is not None:
            buildings.append(out)

    region = None
    if args.region:
        print("Region", flush=True)
        box, region_lines, region = write_region(frame, args, tile, taken)
        extent = union(extent, box)
        lines += region_lines
        coverages.append(region)

    if args.town_population > 0:
        print("Towns", flush=True)
        buildings += town_buildings(frame, args, tile, areas, seen, taken, region)
    if buildings:
        rows = np.concatenate(buildings)
        rows.tofile(os.path.join(OUT_DIR, "buildings.bin"))
        lines.append("buildings=buildings.bin")
        print(f"  buildings.bin: {len(rows)} buildings, {rows.nbytes / 1e6:.1f} MB", flush=True)

    if args.ground_m > 0:
        print("Ground height map", flush=True)
        write_ground(frame, args.region or ESTONIA, tile, args.ground_m)

    print("Airport layouts", flush=True)
    write_layouts(frame)

    print("Low-detail base", flush=True)
    margin = args.outer_margin_km * 1000.0
    side = math.ceil((max(extent[2] - extent[0], extent[3] - extent[1]) + 2.0 * margin) / tile) * tile
    west = math.floor(((extent[0] + extent[2]) / 2.0 - side / 2.0) / tile) * tile
    south = math.floor(((extent[1] + extent[3]) / 2.0 - side / 2.0) / tile) * tile
    lat_r, lon_r = geo_bounds(frame, west, south, west + side, south + side)
    imagery = Source("imagery", IMAGERY_URL, 10, "jpg", decode_jpeg).area(lat_r, lon_r)
    elevation = Source("elevation", ELEVATION_URL, 9, "png", decode_terrarium).area(lat_r, lon_r)
    h, lowered = base_heights(frame, elevation, south, west, side, int(round(side / (args.outer_step_km * 1000.0))) + 1,
                              coverages)
    print(f"  {side / 1000:.0f} km square, {h.shape[0]}x{h.shape[0]} heights, {lowered} under finer tiles")
    lines.append("tile=" + write_tile(frame, OUT_DIR, "outer", south, west, side, h, args.outer_px, imagery, quality=80,
                                       sea_from=elevation))
    del imagery, elevation

    with open(os.path.join(OUT_DIR, "terrain.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    total = 0
    for d in (OUT_DIR, REGION_DIR):
        total += sum(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d) if os.path.isfile(os.path.join(d, f)))
    print(f"Done: {OUT_DIR} ({total / 1e6:.1f} MB)")


def write_region(frame, args, tile, detailed):
    """10 km tiles over the region box that touch land (and all under detailed tiles, `detailed`)."""
    west, south, rows, cols, elevation = region_tiles(frame, args.region, tile, 10)
    grid = args.region_grid
    spacing = tile / (grid - 1)
    # One height grid for the whole box: neighbouring tiles share their edge samples exactly.
    e = west + np.arange(cols * (grid - 1) + 1) * spacing
    n = south + np.arange(rows * (grid - 1) + 1) * spacing
    heights = np.zeros((len(n), len(e)), dtype=np.float32)
    for r0 in range(0, len(n), 200):
        ee, nn = np.meshgrid(e, n[r0:r0 + 200])
        heights[r0:r0 + 200] = ground_heights(frame, elevation, ee, nn)

    keep = []
    t = np.linspace(0.0, tile, int(tile / 100.0) + 1)
    for r in range(rows):
        for c in range(cols):
            s0, w0 = south + r * tile, west + c * tile
            if square_key(s0, w0) not in detailed:
                ee, nn = np.meshgrid(w0 + t, s0 + t)
                if not (elevation.sample(*frame.to_geo(ee, nn)) > LAND_M).any():
                    continue  # open sea: the base layer shows the same
            keep.append((r, c))
    print(f"  {rows}x{cols} grid, {len(keep)} tiles touch land or lie under the detailed area", flush=True)

    imagery = Source("imagery", IMAGERY_URL, args.region_zoom, "jpg", decode_jpeg, keep=600)
    needed = set()
    k = np.linspace(0.0, tile, 5)
    for r, c in keep:
        ee, nn = np.meshgrid(west + c * tile + k, south + r * tile + k)
        x0, y0, x1, y1 = tile_range(*frame.to_geo(ee, nn), args.region_zoom, pad_px=1.0)
        needed.update((x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1))
    imagery.prefetch(needed)

    lines = []
    covered = np.full(heights.shape, np.inf, dtype=np.float32)
    for i, (r, c) in enumerate(keep):
        s0, w0 = south + r * tile, west + c * tile
        block = (slice(r * (grid - 1), r * (grid - 1) + grid), slice(c * (grid - 1), c * (grid - 1) + grid))
        covered[block] = heights[block]
        name = f"n{s0 / 1000:+04.0f}_e{w0 / 1000:+04.0f}"
        lines.append("region=" + write_tile(frame, REGION_DIR, name, s0, w0, tile, heights[block], args.region_px,
                                            imagery, quality=args.region_quality, sea_from=elevation))
        if (i + 1) % 100 == 0:
            print(f"    {i + 1}/{len(keep)} written", flush=True)
    extent = (west, south, west + cols * tile, south + rows * tile)
    return extent, lines, Coverage(west, south, spacing, covered)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Builds the real-world scenery of Estonia, centred on Tallinn (EETN), from open data.

  python tools/make_terrain.py            (needs: pip install numpy pillow)

Downloads once into build/terrain-cache (re-runs reuse it) and writes
unreal/A320Sim/Content/Terrain/, which the simulator loads at start-up and streams in flight:
  terrain.txt    manifest: reference point, attribution, one line per tile and the buildings file.
                 Tile lines are name|southM|westM|sizeM|grid|holeM, keyed by layer:
                   tile=    always loaded (the low-detail base under everything)
                   detail=  10 m/px tiles around the airport, streamed close to the aircraft
                   region=  20 m/px tiles over all of Estonia (files in Region/), streamed
  *.jpg / *.f32  per tile: satellite image (north up) and a float32 height grid (metres above
                 the airfield, rows south to north, columns west to east)
  buildings.bin  OpenStreetMap buildings as boxes: 7 float32 each (north, east, ground height, length,
                 width, yaw from north towards east in degrees, height), metres

Sources (keep the attribution when sharing the output):
  Imagery   Sentinel-2 cloudless 2024 by EOX IT Services GmbH (https://s2maps.eu), contains
            modified Copernicus Sentinel data 2024. CC BY-NC-SA 4.0: non-commercial use.
  Elevation Terrain Tiles (Mapzen, AWS Open Data): SRTM, GMTED, ETOPO1 and others.
  Buildings OpenStreetMap contributors, ODbL (via the Overpass API).

The sim's world is the airport's local east-north-up tangent plane (core/src/Geo.cpp), with
heights above the field; every vertex and pixel is placed through the same transform.
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
    "https://overpass-api.de/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
ATTRIBUTION = ("Imagery: Sentinel-2 cloudless 2024 by EOX IT Services GmbH (s2maps.eu), modified Copernicus "
               "Sentinel data 2024, CC BY-NC-SA 4.0. Elevation: Mapzen Terrain Tiles (AWS Open Data). "
               "Buildings: (c) OpenStreetMap contributors, ODbL.")

# EETN runway ends, as in core/src/Airport.cpp makeTallinn(): (lat, lon, elevation ft).
RWY08 = (59.413338, 24.805908, 129.0)
RWY26 = (59.413174, 24.867208, 131.0)
REF = ((RWY08[0] + RWY26[0]) / 2.0, (RWY08[1] + RWY26[1]) / 2.0, 131.0 * 0.3048)

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
    """Blends the terrain to field level around the runway, so the modelled runway sits on it."""
    e08 = frame.to_enu(RWY08[0], RWY08[1])
    e26 = frame.to_enu(RWY26[0], RWY26[1])
    de, dn = e26[0] - e08[0], e26[1] - e08[1]
    length = math.hypot(de, dn)
    ue, un = de / length, dn / length
    me, mn = (e08[0] + e26[0]) / 2.0, (e08[1] + e26[1]) / 2.0
    along = np.abs((e - me) * ue + (n - mn) * un) - (length / 2.0 + 1000.0)
    across = np.abs(-(e - me) * un + (n - mn) * ue) - 600.0
    dist = np.hypot(np.maximum(along, 0.0), np.maximum(across, 0.0))
    w = np.clip(dist / 700.0, 0.0, 1.0)
    w = w * w * (3.0 - 2.0 * w)
    return h * w


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
    lat0, lat1, lon0, lon1 = bounds
    t = np.linspace(0.0, 1.0, 33)
    lat = np.concatenate([lat0 + (lat1 - lat0) * t, np.full(33, lat1), lat0 + (lat1 - lat0) * t, np.full(33, lat0)])
    lon = np.concatenate([np.full(33, lon0), lon0 + (lon1 - lon0) * t, np.full(33, lon1), lon0 + (lon1 - lon0) * t])
    e, n = frame.to_enu(lat, lon)
    west, east = math.floor(e.min() / tile) * tile, math.ceil(e.max() / tile) * tile
    south, north = math.floor(n.min() / tile) * tile, math.ceil(n.max() / tile) * tile
    rows, cols = int(round((north - south) / tile)), int(round((east - west) / tile))
    lat_r, lon_r = geo_bounds(frame, west, south, east, north)
    elevation = Source("elevation", ELEVATION_URL, elevation_zoom, "png", decode_terrarium).area(lat_r, lon_r)
    return west, south, rows, cols, elevation


def fetch_buildings(frame, half_m):
    (lat0, lat1), (lon0, lon1) = geo_box(frame, half_m)
    query = (f'[out:json][timeout:300];(way["building"]({lat0:.5f},{lon0:.5f},{lat1:.5f},{lon1:.5f}););'
             "out tags geom;")
    path = os.path.join(CACHE_DIR, f"buildings_{int(half_m)}.json")
    if not os.path.exists(path):
        body = urllib.parse.urlencode({"data": query}).encode()
        last = None
        for url in OVERPASS_URLS:
            try:
                print(f"  buildings: asking {url}", flush=True)
                req = urllib.request.Request(url, data=body, headers={"User-Agent": USER_AGENT})
                with urllib.request.urlopen(req, timeout=400) as r:
                    data = r.read()
                json.loads(data)
                os.makedirs(CACHE_DIR, exist_ok=True)
                with open(path, "wb") as f:
                    f.write(data)
                break
            except Exception as e:  # noqa: BLE001 - any mirror failure: try the next one
                last = e
        else:
            print(f"  buildings: no Overpass server answered ({last}); skipping buildings")
            return None
    with open(path, "rb") as f:
        elements = json.load(f).get("elements", [])

    boxes = []
    for el in elements:
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
    args = ap.parse_args()

    frame = Frame(*REF)
    os.makedirs(REGION_DIR, exist_ok=True)
    for d in (OUT_DIR, REGION_DIR):
        for f in os.listdir(d):
            if f.endswith((".jpg", ".f32", ".bin")) or f == "terrain.txt":
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

    if args.buildings_km > 0:
        print("Buildings", flush=True)
        boxes = fetch_buildings(frame, min(args.buildings_km * 1000.0, inner))
        if boxes:
            b = np.array(boxes, dtype=np.float64)
            ground = ground_heights(frame, elevation, b[:, 1], b[:, 0])
            out = np.column_stack([b[:, 0], b[:, 1], ground, b[:, 2], b[:, 3], b[:, 4], b[:, 5]]).astype("<f4")
            out.tofile(os.path.join(OUT_DIR, "buildings.bin"))
            lines.append("buildings=buildings.bin")
            print(f"  {len(out)} buildings")
    del imagery, elevation

    extent = (-inner, -inner, inner, inner)  # west, south, east, north
    if args.region:
        print("Region", flush=True)
        extent, region_lines, cov = write_region(frame, args, tile, inner)
        lines += region_lines
        coverages.append(cov)

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


def write_region(frame, args, tile, inner):
    """10 km tiles over the region box that touch land (and all under the detailed area)."""
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
            under_detail = -inner <= s0 and s0 + tile <= inner and -inner <= w0 and w0 + tile <= inner
            if not under_detail:
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

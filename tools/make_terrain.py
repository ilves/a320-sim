#!/usr/bin/env python3
"""Builds the real-world scenery around Tallinn (EETN) from open data.

  python tools/make_terrain.py            (needs: pip install numpy pillow)

Downloads once into build/terrain-cache (re-runs reuse it) and writes
unreal/A320Sim/Content/Terrain/, which the simulator loads at start-up:
  terrain.txt    manifest: reference point, attribution, one line per tile and the buildings file
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


class Mosaic:
    """Web Mercator tiles of one source and zoom, stitched over a lat/lon box, sampled bilinearly."""

    def __init__(self, name, url, z, lat_range, lon_range, ext, decode):
        self.z = z
        x0, y1 = mercator_px(lat_range[0], lon_range[0], z)
        x1, y0 = mercator_px(lat_range[1], lon_range[1], z)
        self.tx0, self.ty0 = int(x0 // 256), int(y0 // 256)
        tx1, ty1 = int(x1 // 256), int(y1 // 256)
        tiles = [(x, y) for y in range(self.ty0, ty1 + 1) for x in range(self.tx0, tx1 + 1)]
        print(f"  {name}: zoom {z}, {len(tiles)} tiles", flush=True)
        self.data = None
        done = 0

        def get(t):
            x, y = t
            return t, fetch(url.format(z=z, x=x, y=y), os.path.join(CACHE_DIR, name, str(z), str(x), f"{y}.{ext}"))

        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
            for (x, y), raw in pool.map(get, tiles):
                tile = decode(raw)
                if self.data is None:
                    shape = ((ty1 - self.ty0 + 1) * 256, (tx1 - self.tx0 + 1) * 256) + tile.shape[2:]
                    self.data = np.zeros(shape, dtype=tile.dtype)
                r, c = (y - self.ty0) * 256, (x - self.tx0) * 256
                self.data[r:r + 256, c:c + 256] = tile
                done += 1
                if done % 200 == 0:
                    print(f"    {done}/{len(tiles)}", flush=True)

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
    corners = [frame.to_geo(np.array(e), np.array(n)) for e in (-half_m, half_m) for n in (-half_m, half_m)]
    lats = [float(c[0]) for c in corners]
    lons = [float(c[1]) for c in corners]
    pad = 0.02
    return (min(lats) - pad, max(lats) + pad), (min(lons) - pad * 2, max(lons) + pad * 2)


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


def write_tile(frame, name, south, west, size, grid, px, imagery, elevation, hole=0.0):
    # Heights: grid x grid samples, rows south to north, columns west to east.
    t = np.linspace(0.0, size, grid)
    ee, nn = np.meshgrid(west + t, south + t)
    lat, lon = frame.to_geo(ee, nn)
    h = np.maximum(elevation.sample(lat, lon), 0.0) - REF[2]  # sea level, not the sea floor
    h = airport_flatten(frame, ee, nn, h).astype("<f4")
    h.tofile(os.path.join(OUT_DIR, name + ".f32"))

    # Image: pixel centres, north at the top.
    c = (np.arange(px) + 0.5) / px * size
    ee, nn = np.meshgrid(west + c, south + size - c)
    lat, lon = frame.to_geo(ee, nn)
    rgb = np.clip(imagery.sample(lat, lon) + 0.5, 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(os.path.join(OUT_DIR, name + ".jpg"), quality=87, optimize=True)
    return f"tile={name}|{south:.1f}|{west:.1f}|{size:.1f}|{grid}|{hole:.1f}"


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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inner-km", type=float, default=40.0, help="half width of the detailed area (km)")
    ap.add_argument("--tile-km", type=float, default=10.0, help="detailed tile size (km)")
    ap.add_argument("--tile-px", type=int, default=1024, help="detailed tile image size (px)")
    ap.add_argument("--outer-km", type=float, default=150.0, help="half width of the low-detail area (km)")
    ap.add_argument("--buildings-km", type=float, default=25.0, help="half width of the buildings area (km), 0 = none")
    args = ap.parse_args()

    frame = Frame(*REF)
    os.makedirs(OUT_DIR, exist_ok=True)
    for f in os.listdir(OUT_DIR):
        if f.endswith((".jpg", ".f32", ".bin")) or f == "terrain.txt":
            os.remove(os.path.join(OUT_DIR, f))
    lines = ["# Generated by tools/make_terrain.py; see that file for the format and the sources.",
             "version=1", f"reference={REF[0]:.7f},{REF[1]:.7f},{REF[2]:.2f}", f"attribution={ATTRIBUTION}"]

    inner = args.inner_km * 1000.0
    outer = args.outer_km * 1000.0
    tile = args.tile_km * 1000.0
    count = int(round(2 * inner / tile))

    print("Low-detail surroundings", flush=True)
    lat_r, lon_r = geo_box(frame, outer)
    imagery = Mosaic("imagery", IMAGERY_URL, 10, lat_r, lon_r, "jpg", decode_jpeg)
    elevation = Mosaic("elevation", ELEVATION_URL, 9, lat_r, lon_r, "png", decode_terrarium)
    # The outer grid steps by a whole fraction of the inner half width, so the hole edge lies on grid lines.
    steps = int(round(outer / (inner / 16.0)))
    lines.append(write_tile(frame, "outer", -outer, -outer, 2 * outer, steps + 1, 2048, imagery, elevation, hole=inner))
    del imagery, elevation

    print("Detailed area", flush=True)
    lat_r, lon_r = geo_box(frame, inner)
    imagery = Mosaic("imagery", IMAGERY_URL, 13, lat_r, lon_r, "jpg", decode_jpeg)
    elevation = Mosaic("elevation", ELEVATION_URL, 11, lat_r, lon_r, "png", decode_terrarium)
    for r in range(count):
        for c in range(count):
            south, west = -inner + r * tile, -inner + c * tile
            lines.append(write_tile(frame, f"tile_{r}_{c}", south, west, tile, int(tile / 100.0) + 1, args.tile_px,
                                    imagery, elevation))
        print(f"  row {r + 1}/{count} written", flush=True)

    if args.buildings_km > 0:
        print("Buildings", flush=True)
        boxes = fetch_buildings(frame, min(args.buildings_km * 1000.0, inner))
        if boxes:
            b = np.array(boxes, dtype=np.float64)
            lat, lon = frame.to_geo(b[:, 1], b[:, 0])
            ground = np.maximum(elevation.sample(lat, lon), 0.0) - REF[2]
            ground = airport_flatten(frame, b[:, 1], b[:, 0], ground)
            out = np.column_stack([b[:, 0], b[:, 1], ground, b[:, 2], b[:, 3], b[:, 4], b[:, 5]]).astype("<f4")
            out.tofile(os.path.join(OUT_DIR, "buildings.bin"))
            lines.append("buildings=buildings.bin")
            print(f"  {len(out)} buildings")
    del imagery, elevation

    with open(os.path.join(OUT_DIR, "terrain.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    total = sum(os.path.getsize(os.path.join(OUT_DIR, f)) for f in os.listdir(OUT_DIR))
    print(f"Done: {OUT_DIR} ({total / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

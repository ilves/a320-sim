#!/usr/bin/env python3
"""Draws the start-up splash (Content/Splash/Splash.bmp, EdSplash.bmp). Requires Pillow."""
import os

from PIL import Image, ImageDraw, ImageFont

W, H = 640, 320
OUT = os.path.join(os.path.dirname(__file__), "..", "unreal", "A320Sim", "Content", "Splash")
FONTS = "/usr/share/fonts/truetype/dejavu/"


def font(name, size):
    try:
        return ImageFont.truetype(os.path.join(FONTS, name), size)
    except OSError:
        return ImageFont.load_default()


def main():
    img = Image.new("RGB", (W, H))
    d = ImageDraw.Draw(img)
    # Dusk sky gradient over a dark runway.
    for y in range(H):
        t = y / H
        d.line([(0, y), (W, y)], fill=(int(12 + 30 * t), int(24 + 40 * t), int(48 + 60 * t)))
    # Runway on the right, receding to the horizon.
    rx = 520
    d.polygon([(rx - 90, H), (rx + 90, H), (rx + 12, 190), (rx - 12, 190)], fill=(28, 30, 34))
    for i in range(8):
        y0 = 195 + i * i * 1.8
        d.line([(rx, y0), (rx, y0 + 3 + i)], fill=(220, 220, 210), width=2)
    for x in range(0, 6):
        d.ellipse([(rx - 22 - x * 9, 186 + x * 2), (rx - 19 - x * 9, 189 + x * 2)], fill=(255, 255, 230))
        d.ellipse([(rx + 19 + x * 9, 186 + x * 2), (rx + 22 + x * 9, 189 + x * 2)], fill=(255, 255, 230))

    # A320 seen from the front-left, simple shapes.
    cx, cy = 480, 100
    d.ellipse([(cx - 70, cy - 12), (cx + 60, cy + 12)], fill=(225, 228, 232))   # fuselage
    d.polygon([(cx - 10, cy + 2), (cx - 120, cy + 34), (cx - 105, cy + 38), (cx + 20, cy + 6)], fill=(170, 175, 182))
    d.polygon([(cx + 10, cy + 2), (cx + 95, cy + 26), (cx + 88, cy + 30), (cx + 25, cy + 8)], fill=(150, 155, 162))
    d.polygon([(cx - 62, cy - 6), (cx - 80, cy - 46), (cx - 66, cy - 46), (cx - 44, cy - 8)], fill=(20, 60, 140))
    d.ellipse([(cx - 52, cy + 14), (cx - 30, cy + 30)], fill=(120, 125, 132))   # engine
    d.ellipse([(cx + 40, cy - 6), (cx + 64, cy + 6)], fill=(30, 40, 55))        # cockpit windows

    d.text((32, 36), "A320 SIM", font=font("DejaVuSans-Bold.ttf", 48), fill=(245, 245, 250))
    d.text((34, 96), "Tallinn  EETN  -  JSBSim flight model", font=font("DejaVuSans.ttf", 17), fill=(170, 200, 235))
    small = font("DejaVuSans.ttf", 13)
    # The bottom ~40 px stay free for the engine's own status line.
    d.text((34, 150), "Loading Unreal Engine...", font=font("DejaVuSans-Bold.ttf", 15), fill=(255, 210, 120))
    d.text((34, 176), "First start: graphics are prepared", font=small, fill=(210, 215, 225))
    d.text((34, 194), "(shaders compiled), 5-15 minutes.", font=small, fill=(210, 215, 225))
    d.text((34, 216), "Progress: launcher window and in game.", font=small, fill=(210, 215, 225))
    d.text((34, 234), "Later starts take seconds.", font=small, fill=(210, 215, 225))
    os.makedirs(OUT, exist_ok=True)
    for name in ("Splash.bmp", "EdSplash.bmp"):
        img.save(os.path.join(OUT, name), format="BMP")
        print(os.path.join(OUT, name))


if __name__ == "__main__":
    main()

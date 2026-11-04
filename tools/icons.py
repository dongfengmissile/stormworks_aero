#!/usr/bin/env python3
"""Aero plugin icons for the game's icon atlas: 64x64 white-on-transparent RGBA.

Top-view silhouettes (nose up) of an aircraft typical of each aerofoil, plus an aerofoil profile
for the Aerofoil tool. Drawn at 8x on a 100x100 unit canvas and downsampled for anti-aliasing.
Writes assets/icons/<name>.bc3 (DXT5 mips 64..4 px, top row first like the game's atlas, which
the plugin writes into the free half of atlas_gameicons at runtime) and assets/icons/preview.png.
Run: python3 tools/icons.py
"""
import math
import os

from PIL import Image, ImageDraw

S = 8  # supersampling
N = 64
OUT = os.path.join(os.path.dirname(__file__), "..", "assets", "icons")


def canvas():
    im = Image.new("L", (100 * S, 100 * S), 0)
    return im, ImageDraw.Draw(im)


def poly(d, pts):
    d.polygon([(50 * S + x * S, y * S) for x, y in pts], fill=255)


def sym(d, half):
    """Polygon from its right half (x >= 0), mirrored."""
    poly(d, half + [(-x, y) for x, y in reversed(half)])


def fuselage(d, prof):
    """prof: (y, half width) from nose to tail."""
    sym(d, [(w, y) for y, w in prof])


def wing(d, half):
    """A wing panel on the right (x >= 0), mirrored to the left."""
    poly(d, half)
    poly(d, [(-x, y) for x, y in half])


def ellipse_wing(d, x0, span, c0, y_qc, le_frac=0.3, n=40):
    pts_le, pts_te = [], []
    for i in range(n + 1):
        x = x0 + (span - x0) * i / n
        c = c0 * math.sqrt(max(0.0, 1 - (x / span) ** 2))
        pts_le.append((x, y_qc - le_frac * c))
        pts_te.append((x, y_qc + (1 - le_frac) * c))
    wing(d, pts_le + list(reversed(pts_te)))


def ellipse(d, cx, cy, rx, ry):
    for sx in (1, -1):
        d.ellipse([(50 + sx * cx - rx) * S, (cy - ry) * S, (50 + sx * cx + rx) * S, (cy + ry) * S], fill=255)


def cessna172(d):
    fuselage(d, [(14, 0), (15, 2.5), (18, 4.5), (26, 5.5), (42, 5.5), (55, 3.5), (75, 2.2), (86, 1)])
    wing(d, [(4, 30), (30, 30), (46, 31), (47.5, 33), (47.5, 39), (46, 40.5), (30, 41.5), (4, 42)])
    wing(d, [(1, 75), (10, 75.5), (16, 77.5), (17, 80), (16.5, 83), (1, 84)])


def twin_otter(d):
    fuselage(d, [(11, 0), (12, 3.5), (16, 5), (60, 5), (80, 2.5), (88, 1.2)])
    wing(d, [(4, 30), (46, 30), (48, 31.5), (48, 37.5), (46, 39), (4, 39)])
    wing(d, [(13.5, 25), (15, 20), (19, 20), (20.5, 25), (20, 45), (14, 45)])  # nacelles
    wing(d, [(1, 80), (16, 81), (17, 83), (16, 87), (1, 88)])


def spitfire(d):
    fuselage(d, [(10, 0), (11, 2.5), (15, 4.5), (24, 5.2), (45, 5.2), (60, 4), (80, 2), (89, 1)])
    ellipse_wing(d, 0, 48, 24, 38)
    ellipse_wing(d, 0, 16, 10, 83)


def extra300(d):
    fuselage(d, [(10, 0), (11, 3), (14, 5.5), (30, 6), (45, 5), (60, 3), (85, 1.5), (89, 0.8)])
    wing(d, [(4, 27), (44, 35), (45.5, 36.5), (45.5, 41.5), (44, 43), (4, 46)])
    wing(d, [(1, 73), (17, 79), (18.5, 81), (18.5, 86), (1, 88)])


def bell_x1(d):
    fuselage(d, [(4, 0), (8, 3.5), (16, 6), (30, 7.5), (60, 7.5), (85, 5), (96, 3.5)])
    wing(d, [(6, 44), (42, 47), (43.5, 48.5), (43.5, 51.5), (42, 53), (6, 55)])
    wing(d, [(3, 84), (18, 88), (19, 90), (18, 93), (3, 94)])


def f86(d):
    fuselage(d, [(6, 3), (8, 5), (20, 5.8), (60, 5.8), (85, 4), (94, 3)])
    wing(d, [(5, 35), (43, 61), (44, 63), (43, 67), (5, 56)])
    wing(d, [(3, 80), (20, 90), (20.5, 92), (20, 93.5), (3, 92)])


def a320(d):
    fuselage(d, [(2, 0), (3, 2.5), (7, 4.5), (80, 4.5), (92, 2.5), (98, 1)])
    wing(d, [(4, 40), (43, 58), (43.5, 60), (43, 61.5), (14, 59), (4, 58)])
    wing(d, [(12.5, 37), (13.5, 35), (16.5, 35), (17.5, 37), (17.5, 49), (12.5, 49)])  # engines
    wing(d, [(3, 84), (16, 93), (16.5, 95), (3, 94)])


def f4(d):
    fuselage(d, [(2, 0), (6, 2), (20, 3.5), (27, 4), (28, 6.5), (33, 8.5), (60, 7.5), (80, 5), (92, 3.8), (98, 3.2)])
    wing(d, [(8, 42), (21, 55), (21, 53.5), (29, 61.5), (29.5, 66), (21, 68), (8, 72)])
    wing(d, [(3, 86), (15, 94), (15.5, 97), (3, 96)])


def f104(d):
    fuselage(d, [(2, 0), (10, 1.5), (25, 3), (38, 4.5), (42, 6.5), (55, 6.5), (62, 5), (85, 4.5), (98, 3.5)])
    wing(d, [(4, 55), (18, 61), (18.5, 63), (18, 64), (4, 68)])
    ellipse(d, 18.5, 62, 1.6, 8)  # tip tanks
    wing(d, [(1, 88), (11, 92), (11, 95), (1, 96)])


def aerofoil(d):
    """NACA 2412 profile, thickness exaggerated for legibility, slight nose-up."""
    m, p, t = 0.02, 0.4, 0.17
    up, lo = [], []
    for i in range(61):
        x = (1 - math.cos(math.pi * i / 60)) / 2
        yt = 5 * t * (0.2969 * math.sqrt(x) - 0.126 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
        yc = m / p**2 * (2 * p * x - x * x) if x < p else m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x)
        up.append((x, yc + yt))
        lo.append((x, yc - yt))
    a = math.radians(8)
    pts = []
    for x, y in up + list(reversed(lo)):
        x, y = x - 0.35, y
        xr, yr = x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)
        pts.append((xr * 88, 50 - yr * 88))
    d.polygon([(50 * S + x * S, y * S) for x, y in pts], fill=255)


ICONS = {
    "aero_foil": aerofoil,
    "aero_x1": bell_x1,
    "aero_twinotter": twin_otter,
    "aero_cessna": cessna172,
    "aero_spitfire": spitfire,
    "aero_extra": extra300,
    "aero_f86": f86,
    "aero_a320": a320,
    "aero_f4": f4,
    "aero_f104": f104,
}


def render(fn):
    im, d = canvas()
    fn(d)
    # Fit into the icon with a margin like the game's (glyph about 80% of the cell).
    box = im.getbbox()
    w, h = box[2] - box[0], box[3] - box[1]
    side = max(w, h)
    sq = Image.new("L", (side, side), 0)
    sq.paste(im.crop(box), ((side - w) // 2, (side - h) // 2))
    inner = int(N * 0.8)
    a = sq.resize((inner, inner), Image.LANCZOS)
    out = Image.new("L", (N, N), 0)
    out.paste(a, ((N - inner) // 2, (N - inner) // 2))
    rgba = Image.merge("RGBA", (Image.new("L", (N, N), 255),) * 3 + (out,))
    return rgba


def bc3_alpha(vals):
    a0, a1 = max(vals), min(vals)
    if a0 == a1:
        return bytes([a0, a1, 0, 0, 0, 0, 0, 0])
    pal = [a0, a1] + [((7 - i) * a0 + i * a1) // 7 for i in range(1, 7)]
    bits = 0
    for j, v in enumerate(vals):
        k = min(range(8), key=lambda i: abs(pal[i] - v))
        bits |= k << (3 * j)
    return bytes([a0, a1]) + bits.to_bytes(6, "little")


def bc3(alpha):
    """DXT5 blocks of a white image with this alpha (L image, side multiple of 4), rows top first."""
    w, h = alpha.size
    px = alpha.load()
    out = bytearray()
    for by in range(0, h, 4):
        for bx in range(0, w, 4):
            vals = [px[bx + j % 4, by + j // 4] for j in range(16)]
            out += bc3_alpha(vals)
            out += bytes([0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0])  # colour white (index 0 everywhere)
    return bytes(out)


def mips(alpha):
    """Levels 64, 32, 16, 8, 4 px (box filter), each DXT5."""
    out, a = b"", alpha
    while a.size[0] >= 4:
        out += bc3(a)
        a = a.resize((a.size[0] // 2, a.size[1] // 2), Image.BOX)
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    sheet = Image.new("RGBA", (N * len(ICONS) * 2, N * 2 + 8), (61, 140, 180, 255))
    for i, (name, fn) in enumerate(ICONS.items()):
        im = render(fn)
        with open(os.path.join(OUT, name + ".bc3"), "wb") as f:
            f.write(mips(im.getchannel("A")))
        big = im.resize((N * 2, N * 2), Image.NEAREST)
        sheet.alpha_composite(big, (i * N * 2, 0))
    sheet.save(os.path.join(OUT, "preview.png"))
    print("wrote", len(ICONS), "icons to", os.path.normpath(OUT))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Write the Dex1-1 finger-pad knurl normal map: dex1_d405_hand/textures/pad_knurl_normal.png.

The real pads are matte black rubber with a diamond knurl: grooves at +/-45 deg, ~1 mm apart.
The texture is one seamless tile of that pattern (raised diamonds between narrow grooves),
stored as a tangent-space normal map (OpenGL convention, +Y up). One tile spans TILE_M metres,
so the builder sets the pad UVs to (position / TILE_M) to reproduce the real pitch.

Requires numpy, Pillow.   Usage: python tools/make_pad_knurl_texture.py
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

PITCH_M = 0.001                 # distance between parallel grooves (measured on the real pad)
TILE_M = PITCH_M * np.sqrt(2)   # grooves along u+v = k and u-v = k repeat every tile
GROOVE_M = 0.00012              # groove half-width
OUT = Path(__file__).resolve().parent.parent / "dex1_d405_hand" / "textures" / "pad_knurl_normal.png"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size", type=int, default=512, help="texture size in pixels (one tile)")
    ap.add_argument("--strength", type=float, default=4.0, help="bump strength of the groove walls")
    ap.add_argument("--out", type=Path, default=OUT)
    a = ap.parse_args()

    n = a.size
    u, v = np.meshgrid((np.arange(n) + 0.5) / n, (np.arange(n) + 0.5) / n)
    # Distance (in metres) to the nearest groove of each family; the lines u +/- v = k are
    # PITCH_M apart because a tile is PITCH_M * sqrt(2) wide.
    dist = lambda s: np.abs(s - np.round(s)) * PITCH_M
    d = np.minimum(dist(u + v), dist(u - v))
    height = np.clip(d / GROOVE_M, 0.0, 1.0) ** 0.5   # flat diamond tops, sloped groove walls

    # Periodic central differences keep the tile seamless.
    du = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * n / 2
    dv = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * n / 2
    scale = a.strength / n
    normal = np.dstack([-du * scale, dv * scale, np.ones_like(height)])  # image rows run down, +v runs up
    normal /= np.linalg.norm(normal, axis=2, keepdims=True)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.round((normal * 0.5 + 0.5) * 255).astype(np.uint8)).save(a.out)
    print(f"wrote {a.out} ({n}x{n}, one tile = {TILE_M * 1000:.3f} mm, groove pitch {PITCH_M * 1000:.1f} mm)")


if __name__ == "__main__":
    main()

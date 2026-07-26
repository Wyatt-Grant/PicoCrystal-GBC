#!/usr/bin/env python3
"""Nearest-neighbour PPM (P6) -> PNG upscaler, stdlib only.

The UI renders at the panel's native 240x240; GitHub scales README images to
the column width, and a browser's smooth downscale of a 2x nearest-neighbour
copy keeps the pixel edges crisp where a 1x image would go soft. Integer
scaling only, so no pixel is ever resampled.

    upscale_png.py <scale> <out_dir> <in.ppm>...
"""
import os
import sys

sys.dont_write_bytecode = True  # no __pycache__/ dropped in host_test/
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'host_test'))
from ppm2png import read_ppm, write_png  # noqa: E402


def upscale(w, h, pixels, n):
    if n == 1:
        return w, h, pixels
    out = bytearray()
    stride = w * 3
    for y in range(h):
        row = pixels[y * stride:(y + 1) * stride]
        big = bytearray()
        for x in range(w):
            big += row[x * 3:x * 3 + 3] * n
        out += big * n
    return w * n, h * n, bytes(out)


if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    scale = int(sys.argv[1])
    out_dir = sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    for path in sys.argv[3:]:
        w, h, pixels = read_ppm(path)
        w, h, pixels = upscale(w, h, pixels, scale)
        name = os.path.basename(path).rsplit('.', 1)[0] + '.png'
        out = os.path.join(out_dir, name)
        write_png(out, w, h, pixels)
        print(f"{path} -> {out} ({w}x{h})")

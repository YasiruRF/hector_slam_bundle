#!/usr/bin/env python3
"""Room size from a saved map: the two strongest wall lines along x and along y.

  measure_room.py MAP.yaml [MAP2.yaml ...]

For a roughly axis-aligned rectangular room, occupied cells pile up at the four walls.
The strongest peaks of the occupied-cell histograms (5 cm bins) at least 2 m apart are taken
as opposite walls; the room size is the distance between the peak centres. Also prints the
wall thickness (spread of the cells within +-0.3 m of each peak), a sign of a doubled wall.
"""
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from plot_map import load  # noqa: E402


def walls(coords, res):
    lo, hi = coords.min(), coords.max()
    bins = np.arange(lo, hi + res, res)
    h, edges = np.histogram(coords, bins=bins)
    centres = (edges[:-1] + edges[1:]) / 2
    order = np.argsort(h)[::-1]
    a = centres[order[0]]
    b = next(centres[i] for i in order[1:] if abs(centres[i] - a) > 2.0)
    out = []
    for p in sorted((a, b)):
        near = coords[np.abs(coords - p) < 0.3]
        out.append((near.mean(), near.std()))
    return out


def main():
    for path in sys.argv[1:]:
        img, res, ox, oy = load(path)
        h = img.shape[0]
        rows, cols = np.nonzero(img < 50)
        xs = ox + (cols + 0.5) * res
        ys = oy + (h - rows - 0.5) * res
        (x1, sx1), (x2, sx2) = walls(xs, res)
        (y1, sy1), (y2, sy2) = walls(ys, res)
        print('%s: x walls %.2f / %.2f m -> %.2f m wide (wall spread %.0f / %.0f cm); '
              'y walls %.2f / %.2f m -> %.2f m long (spread %.0f / %.0f cm)' % (
                  path, x1, x2, x2 - x1, sx1 * 100, sx2 * 100, y1, y2, y2 - y1,
                  sy1 * 100, sy2 * 100))


if __name__ == '__main__':
    main()

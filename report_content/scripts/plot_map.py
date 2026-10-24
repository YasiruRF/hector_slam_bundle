#!/usr/bin/env python3
"""Render saved occupancy maps (map_server .yaml/.pgm) as figures.

  plot_map.py OUT.png MAP.yaml [--traj CSV:XCOL:YCOL:LABEL ...] [--title T] [--crop]
  plot_map.py OUT.png --grid MAP1.yaml MAP2.yaml ... [--labels a,b,...] [--title T]

--traj draws a trajectory from a CSV (columns given by name) in map coordinates.
--crop trims unknown space around the explored area.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load(yaml_path):
    meta = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(img_path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    return img, res, ox, oy


def extent(img, res, ox, oy):
    h, w = img.shape
    return [ox, ox + w * res, oy, oy + h * res]


def crop_box(img, res, ox, oy, margin=0.5):
    known = np.argwhere(img != 205)
    if not len(known):
        return None
    h = img.shape[0]
    r0, c0 = known.min(0)
    r1, c1 = known.max(0)
    return (ox + c0 * res - margin, ox + (c1 + 1) * res + margin,
            oy + (h - r1 - 1) * res - margin, oy + (h - r0) * res + margin)


def draw(ax, yaml_path, crop=True):
    img, res, ox, oy = load(yaml_path)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=extent(img, res, ox, oy),
              interpolation='nearest')
    if crop:
        box = crop_box(img, res, ox, oy)
        if box:
            ax.set_xlim(box[0], box[1])
            ax.set_ylim(box[2], box[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('map', nargs='?')
    ap.add_argument('--grid', nargs='+')
    ap.add_argument('--labels')
    ap.add_argument('--traj', action='append', default=[])
    ap.add_argument('--title')
    ap.add_argument('--no-crop', action='store_true')
    a = ap.parse_args()

    if a.grid:
        n = len(a.grid)
        cols = min(3, n)
        rows = (n + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 4.0 * rows), squeeze=False)
        labels = a.labels.split(',') if a.labels else [os.path.basename(p) for p in a.grid]
        for ax, p, lab in zip(axes.flat, a.grid, labels):
            draw(ax, p, not a.no_crop)
            ax.set_title(lab)
        for ax in list(axes.flat)[n:]:
            ax.axis('off')
    else:
        fig, ax = plt.subplots(figsize=(7, 6))
        draw(ax, a.map, not a.no_crop)
        colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
        for i, spec in enumerate(a.traj):
            path, xc, yc, label = spec.split(':')
            rows = [r for r in csv.DictReader(open(path)) if r[xc] and r[yc]]
            xs = [float(r[xc]) for r in rows]
            ys = [float(r[yc]) for r in rows]
            ax.plot(xs, ys, '-', color=colors[i % 4], lw=1.5, label=label)
            ax.plot(xs[0], ys[0], 'o', color=colors[i % 4], ms=6)
            ax.plot(xs[-1], ys[-1], 's', color=colors[i % 4], ms=6)
        if a.traj:
            ax.legend(loc='best', fontsize=8)
    if a.title:
        fig.suptitle(a.title)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    print('wrote', a.out)


if __name__ == '__main__':
    main()

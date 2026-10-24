#!/usr/bin/env python3
"""Error-over-time figures.

  plot_errors.py hector OUT.png POSE.csv:LABEL [POSE.csv:LABEL ...] [--mark T:TEXT]
      pose_logger.py logs (Hector vs ground truth): position and heading error
  plot_errors.py amcl OUT.png KIDNAP.csv [EVENTS.txt]
      nav_tests.py kidnap log: AMCL error and particle spread, phases shaded
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math


def hector(out, specs, marks):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    for spec in specs:
        path, label = spec.rsplit(':', 1)
        rows = list(csv.DictReader(open(path)))
        t = [float(r['t']) for r in rows]
        a1.plot(t, [float(r['err_xy']) * 100 for r in rows], label=label, lw=1.2)
        a2.plot(t, [math.degrees(float(r['err_yaw'])) for r in rows], label=label, lw=1.2)
    for m in marks:
        tt, text = m.split(':', 1)
        for ax in (a1, a2):
            ax.axvline(float(tt), color='tab:red', ls='--', lw=1)
        a1.text(float(tt), a1.get_ylim()[1] * 0.9, ' ' + text, color='tab:red', fontsize=8)
    a1.set_ylabel('position error [cm]')
    a2.set_ylabel('heading error [deg]')
    a2.set_xlabel('sim time [s]')
    a1.set_yscale('symlog', linthresh=10)
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)


def amcl(out, path, events=None):
    rows = list(csv.DictReader(open(path)))
    t = [float(r['t_s']) for r in rows]
    e = [float(r['err_m']) for r in rows]
    s = [float(r['spread_m']) for r in rows]
    ph = [r['phase'] for r in rows]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    colors = {'no_help': '#f8d7d7', 'global': '#fff0c2', 'kidnapped': '#f8d7d7',
              'kidnapped2': '#f8d7d7', 'manual': '#d9f2d9', 'after': '#e3ecfb', 'converge': '#e3ecfb'}
    names = {'converge': 'localised, short goal', 'kidnapped': 'kidnap 1',
             'no_help': 'turning, AMCL alone', 'global': 'global relocalisation',
             'kidnapped2': 'kidnap 2', 'manual': 'manual /initialpose', 'after': 'goal after recovery'}
    i = 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and ph[j + 1] == ph[i]:
            j += 1
        if ph[i] in colors:
            ax.axvspan(t[i], t[j], color=colors[ph[i]], lw=0)
            ax.text((t[i] + t[j]) / 2, 0.97, names[ph[i]], transform=ax.get_xaxis_transform(),
                    ha='center', va='top', fontsize=7, rotation=90 if t[j] - t[i] < 25 else 0)
        i = j + 1
    ax.plot(t, e, color='tab:red', label='AMCL position error (vs ground truth)')
    ax.plot(t, s, color='tab:blue', ls='--', label='particle spread (RMS distance to mean)')
    ax.set_yscale('symlog', linthresh=0.1)
    ax.set_ylabel('[m]')
    ax.set_xlabel('sim time [s]')
    ax.legend(loc='center left', fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)


if __name__ == '__main__':
    kind, out = sys.argv[1], sys.argv[2]
    rest = sys.argv[3:]
    if kind == 'hector':
        marks = [rest[i + 1] for i, a in enumerate(rest) if a == '--mark']
        specs = [a for i, a in enumerate(rest) if a != '--mark' and (i == 0 or rest[i - 1] != '--mark')]
        hector(out, specs, marks)
    else:
        amcl(out, *rest)
    print('wrote', out)

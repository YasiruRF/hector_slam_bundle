#!/usr/bin/env python3
"""Real-robot run without ground truth: compare Hector's heading with the Create 3 odometry.

  analyze_real.py TRAJ.csv ODOM.csv OUT_PREFIX [--label L] [TRAJ2.csv:LABEL ...]

TRAJ.csv has t_s, map_yaw, odom_yaw (dashboard trajectory.csv or run_logger.py output).
The Create 3 odometry heading is gyro-based and drifts only slowly, so a sudden jump in
(Hector yaw - odometry yaw) is a scan-matching slip. ODOM.csv (bag_tools.py odom) gives the
turn rate. Writes OUT_PREFIX.png and OUT_PREFIX.txt (slips > 3 deg, path lengths).
"""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_traj(path):
    rows = [r for r in csv.DictReader(open(path)) if r.get('map_yaw') and r.get('odom_yaw')]
    t = [float(r['t_s']) for r in rows]
    off = [wrap(float(r['map_yaw']) - float(r['odom_yaw'])) for r in rows]
    base = off[0]
    off = [math.degrees(wrap(o - base)) for o in off]
    path_m = sum(math.hypot(float(b['map_x']) - float(a['map_x']), float(b['map_y']) - float(a['map_y']))
                 for a, b in zip(rows, rows[1:]))
    odom_m = sum(math.hypot(float(b['odom_x']) - float(a['odom_x']), float(b['odom_y']) - float(a['odom_y']))
                 for a, b in zip(rows, rows[1:]))
    return t, off, path_m, odom_m


def main():
    traj, odom, out = sys.argv[1:4]
    extra = [a for a in sys.argv[4:] if ':' in a]
    o = list(csv.DictReader(open(odom)))
    ot = [float(r['t_s']) for r in o]
    oy = [float(r['yaw']) for r in o]
    w_t, w = [], []
    for i in range(5, len(o), 5):
        dt = ot[i] - ot[i - 5]
        if dt > 0:
            w_t.append(ot[i])
            w.append(math.degrees(abs(wrap(oy[i] - oy[i - 5])) / dt))

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    lines = []
    for spec in [traj + ':live run'] + extra:
        p, label = spec.rsplit(':', 1)
        t, off, pm, om = load_traj(p)
        # the dashboard trajectory starts ~33 s after the bag; align on the bag clock
        shift = 0.0
        a1.plot([x + shift for x in t], off, label=label)
        slips = [(t[i], off[i] - off[i - 1]) for i in range(1, len(t))
                 if abs(off[i] - off[i - 1]) > 3.0]
        lines.append('%s: final heading offset %.1f deg, max |offset| %.1f deg, '
                     'Hector path %.1f m, odometry path %.1f m' % (
                         label, off[-1], max(abs(x) for x in off), pm, om))
        lines += ['  slip at t=%.1f s: %+.1f deg' % s for s in slips]
    a1.set_ylabel('Hector yaw - odom yaw [deg]')
    a1.axhline(0, color='k', lw=0.5)
    a1.legend(fontsize=8)
    a1.set_title('Real robot: Hector heading vs Create 3 odometry heading')
    a2.plot(w_t, w, color='tab:gray')
    a2.set_ylabel('turn rate |w| [deg/s]')
    a2.set_xlabel('time [s] (own clock of each log)')
    fig.tight_layout()
    fig.savefig(out + '.png', dpi=150)
    lines.append('max turn rate (0.27 s window): %.0f deg/s' % max(w))
    open(out + '.txt', 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()

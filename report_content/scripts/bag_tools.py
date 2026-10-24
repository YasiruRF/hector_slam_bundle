#!/usr/bin/env python3
"""Offline tools for the real-robot rosbag (run inside the jazzy container).

  bag_tools.py info  BAG                 topic rates, TF frame pairs
  bag_tools.py maps  BAG OUTDIR [N]      last /map as map.pgm/.yaml + N snapshots (snap_XX.pgm)
  bag_tools.py strip BAG OUT             copy BAG without the map -> odom TF that the live
                                         Hector published, so the bag can be replayed into a
                                         fresh Hector with other parameters
  bag_tools.py odom  BAG CSV             /odom pose as CSV (t, x, y, yaw)
"""
import math
import os
import sys
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    return r, types


def messages(path, topics=None):
    r, types = reader(path)
    while r.has_next():
        topic, data, t = r.read_next()
        if topics and topic not in topics:
            continue
        yield topic, deserialize_message(data, get_message(types[topic])), t


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def cmd_info(bag):
    stamps = defaultdict(list)
    pairs = Counter()
    for topic, msg, t in messages(bag):
        stamps[topic].append(t * 1e-9)
        if topic in ('/tf', '/tf_static'):
            for tr in msg.transforms:
                pairs[(topic, tr.header.frame_id, tr.child_frame_id)] += 1
    for topic, ts in sorted(stamps.items()):
        dur = ts[-1] - ts[0]
        print('%-12s %5d msgs  %6.1f s  %5.2f Hz' % (topic, len(ts), dur,
                                                     (len(ts) - 1) / dur if dur else 0))
    for (topic, a, b), n in sorted(pairs.items()):
        print('%-11s %s -> %s: %d' % (topic, a, b, n))


def write_pgm(grid, path_base):
    w, h = grid.info.width, grid.info.height
    px = bytearray(w * h)
    for row in range(h):
        src = (h - 1 - row) * w  # image row 0 is the top of the map
        for col in range(w):
            v = grid.data[src + col]
            px[row * w + col] = 205 if v < 0 else (0 if v >= 65 else (254 if v <= 19 else 205))
    with open(path_base + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(bytes(px))
    o = grid.info.origin
    with open(path_base + '.yaml', 'w') as f:
        f.write('image: %s.pgm\nmode: trinary\nresolution: %g\n'
                'origin: [%g, %g, %g]\nnegate: 0\noccupied_thresh: 0.65\n'
                'free_thresh: 0.196\n' % (os.path.basename(path_base), grid.info.resolution,
                                           o.position.x, o.position.y, yaw_of(o.orientation)))


def cmd_maps(bag, outdir, n=6):
    os.makedirs(outdir, exist_ok=True)
    grids = [(t, m) for _, m, t in messages(bag, {'/map'})]
    if not grids:
        sys.exit('no /map in ' + bag)
    t0 = grids[0][0]
    picks = [grids[round(i * (len(grids) - 1) / max(1, n - 1))] for i in range(n)]
    for i, (t, g) in enumerate(picks):
        write_pgm(g, os.path.join(outdir, 'snap_%02d' % i))
        print('snap_%02d  t=%.0f s' % (i, (t - t0) * 1e-9))
    write_pgm(grids[-1][1], os.path.join(outdir, 'map'))
    print('map: %d snapshots, last at %.0f s' % (len(grids), (grids[-1][0] - t0) * 1e-9))


def cmd_strip(bag, out):
    r, types = reader(bag)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=out, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in r.get_all_topics_and_types():
        if meta.name != '/map':
            w.create_topic(meta)
    dropped = 0
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/map':
            continue
        if topic == '/tf':
            msg = deserialize_message(data, get_message(types[topic]))
            keep = [tr for tr in msg.transforms if tr.header.frame_id != 'map']
            dropped += len(msg.transforms) - len(keep)
            if not keep:
                continue
            msg.transforms = keep
            data = serialize_message(msg)
        w.write(topic, data, t)
    print('dropped %d map->* transforms and all /map messages' % dropped)


def cmd_odom(bag, csv):
    with open(csv, 'w') as f:
        f.write('t_s,x,y,yaw\n')
        t0 = None
        for _, m, t in messages(bag, {'/odom'}):
            t0 = t if t0 is None else t0
            p = m.pose.pose
            f.write('%.3f,%.4f,%.4f,%.4f\n' % ((t - t0) * 1e-9, p.position.x, p.position.y,
                                               yaw_of(p.orientation)))


if __name__ == '__main__':
    cmd, args = sys.argv[1], sys.argv[2:]
    {'info': cmd_info, 'maps': cmd_maps, 'strip': cmd_strip, 'odom': cmd_odom}[cmd](
        *[int(a) if a.isdigit() else a for a in args])

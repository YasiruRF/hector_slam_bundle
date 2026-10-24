#!/usr/bin/env python3
"""Log a run: map -> base_link, odom -> base_link and /ground_truth/pose (if any) at 2 Hz,
/scan and /slam_out_pose rates, and the last /map. Ctrl-C (SIGINT/SIGTERM) writes
OUT/trajectory.csv, OUT/rates.txt and OUT/map.pgm/.yaml.

  run_logger.py OUTDIR [--ros-args -p use_sim_time:=true]
"""
import math
import os
import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_tools import write_pgm, yaw_of  # noqa: E402


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('run_logger')
    buf = Buffer()
    TransformListener(buf, node)
    state = {'map': None, 'gt': None, 'rows': [], 't0': None}
    counts = {'scan': [], 'slam': []}

    def stamp_s():
        return node.get_clock().now().nanoseconds * 1e-9

    node.create_subscription(LaserScan, 'scan', lambda m: counts['scan'].append(stamp_s()),
                             qos_profile_sensor_data)
    node.create_subscription(PoseStamped, 'slam_out_pose',
                             lambda m: counts['slam'].append(stamp_s()), 10)
    node.create_subscription(PoseStamped, 'ground_truth/pose',
                             lambda m: state.__setitem__('gt', m), 10)
    node.create_subscription(OccupancyGrid, 'map', lambda m: state.__setitem__('map', m),
                             QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def pose(parent):
        try:
            t = buf.lookup_transform(parent, 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def tick():
        now = stamp_s()
        if now <= 0:
            return
        m, o = pose('map'), pose('odom')
        if m is None and o is None:
            return
        state['t0'] = now if state['t0'] is None else state['t0']
        g = state['gt']
        gt = (g.pose.position.x, g.pose.position.y, yaw_of(g.pose.orientation)) if g else None
        state['rows'].append((now - state['t0'], m, o, gt))

    node.create_timer(0.5, tick)

    def finish(*_):
        f = lambda p: ('%.4f,%.4f,%.4f' % p) if p else ',,'
        with open(os.path.join(out, 'trajectory.csv'), 'w') as fh:
            fh.write('t_s,map_x,map_y,map_yaw,odom_x,odom_y,odom_yaw,gt_x,gt_y,gt_yaw\n')
            for t, m, o, g in state['rows']:
                fh.write('%.2f,%s,%s,%s\n' % (t, f(m), f(o), f(g)))
        with open(os.path.join(out, 'rates.txt'), 'w') as fh:
            for k, ts in counts.items():
                d = ts[-1] - ts[0] if len(ts) > 1 else 0
                fh.write('%s: %d msgs, %.2f Hz\n' % (k, len(ts), (len(ts) - 1) / d if d else 0))
        if state['map'] is not None:
            write_pgm(state['map'], os.path.join(out, 'map'))
        print('run_logger: %d rows -> %s' % (len(state['rows']), out), flush=True)
        os._exit(0)

    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    rclpy.spin(node)


if __name__ == '__main__':
    main()

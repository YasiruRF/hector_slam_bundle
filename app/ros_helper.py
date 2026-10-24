#!/usr/bin/env python3
"""ROS side of the dashboard: one rclpy node, talks JSON lines on stdin/stdout.

  ros_helper.py [--namespace /tb4] [--sim-time]

Out (stdout, one JSON object per line):
  status   rates, battery, dock state, nav state (1 Hz)
  pose     map->base_link and odom->base_link (5 Hz)
  scan     decimated ranges for the map overlay (2 Hz)
  map      PNG (base64) + geometry + cell stats, when /map changes (max 0.5 Hz)
  nav      NavigateToPose state changes
  probe_result, traj_done
In (stdin): vel, goal, cancel, estop, probe, traj_start, traj_stop.
Everything else the dashboard needs (launching, logs) is done by server.py.
"""
import argparse
import base64
import csv
import json
import math
import struct
import sys
import threading
import time
import zlib
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:  # only needed for dock state and the dock/undock buttons
    DockStatus = Dock = Undock = None
from std_srvs.srv import Empty

OUT_LOCK = threading.Lock()


def emit(obj):
    with OUT_LOCK:
        sys.stdout.write(json.dumps(obj) + '\n')
        sys.stdout.flush()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def png_gray(img):
    """uint8 [h, w] -> PNG bytes (stdlib only)."""
    h, w = img.shape
    raw = np.hstack([np.zeros((h, 1), np.uint8), img]).tobytes()

    def chunk(tag, data):
        body = tag + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


class Rate:
    def __init__(self):
        self.t = deque(maxlen=60)

    def tick(self):
        self.t.append(time.monotonic())

    def hz(self):
        now = time.monotonic()
        recent = [x for x in self.t if now - x < 3.0]
        return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 and recent[-1] > recent[0] else 0.0

    def age(self):
        return time.monotonic() - self.t[-1] if self.t else None


class Helper(Node):
    def __init__(self, ns, sim_time):
        super().__init__('tb4_gui_helper', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, sim_time)])
        self.ns = ns
        self.rates = {k: Rate() for k in ('scan', 'odom', 'map')}
        self.scan_frame = None
        self.scan_msg = None
        self.scan_delay = deque(maxlen=30)   # this computer's clock minus scan stamp: latency + clock offset
        self.map_msg = None
        self.map_sent = None
        self.map_stamp = 0.0
        self.battery = None
        self.docked = None
        self.nav_state = 'idle'
        self.nav_remaining = None
        self.goal_handle = None
        self.last_vel = 0.0
        self.moving = False
        self.vel_pubs = {}
        self.traj = None
        self.traj_len = 0.0
        self.traj_last = None
        self.traj_t0 = 0.0

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, f'{ns}/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'{ns}/odom', lambda m: self.rates['odom'].tick(),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, f'{ns}/map', self.on_map, qos_profile_sensor_data)
        self.create_subscription(BatteryState, f'{ns}/battery_state', self.on_battery,
                                 qos_profile_sensor_data)
        if DockStatus:
            self.create_subscription(DockStatus, f'{ns}/dock_status', self.on_dock,
                                     qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, f'{ns}/navigate_to_pose')
        self.dock_clients = {}
        if Dock:
            self.dock_clients = {'dock': ActionClient(self, Dock, f'{ns}/dock'),
                                 'undock': ActionClient(self, Undock, f'{ns}/undock')}
        self.motor = {'start': self.create_client(Empty, f'{ns}/start_motor'),
                      'stop': self.create_client(Empty, f'{ns}/stop_motor')}
        self.robot_action = None
        self.wall_timer(0.2, self.tick_pose)
        self.wall_timer(0.5, self.tick_scan)
        self.wall_timer(1.0, self.tick_status)
        self.wall_timer(2.0, self.tick_map)
        self.wall_timer(0.1, self.tick_deadman)

    def wall_timer(self, period, cb):
        # Wall-clock timers: heartbeat and e-stop deadman must not slow down with a simulator clock.
        return self.create_timer(period, cb, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ---- subscriptions ----
    def on_scan(self, m):
        self.rates['scan'].tick()
        self.scan_frame = m.header.frame_id
        self.scan_msg = m
        now = self.get_clock().now()
        stamp = Time.from_msg(m.header.stamp, clock_type=now.clock_type)
        self.scan_delay.append((now - stamp).nanoseconds * 1e-9)

    def on_map(self, m):
        self.rates['map'].tick()
        self.map_msg = m

    def on_battery(self, m):
        self.battery = m.percentage

    def on_dock(self, m):
        self.docked = bool(m.is_docked)

    # ---- periodic output ----
    def lookup(self, target, source):
        try:
            t = self.tf.lookup_transform(target, source, Time())
        except Exception:
            return None
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def tf_fresh(self, target, source, max_age=2.0):
        """True only if the transform is still being published (the buffer keeps the last one forever)."""
        try:
            t = self.tf.get_latest_common_time(target, source)
        except Exception:
            return False
        now = self.get_clock().now()   # tf2 hands back a Time of another clock type; rclpy refuses to mix them
        return (now - Time(nanoseconds=t.nanoseconds, clock_type=now.clock_type)).nanoseconds * 1e-9 < max_age

    def tick_pose(self):
        mp = self.lookup('map', 'base_link')
        od = self.lookup('odom', 'base_link')
        emit({'type': 'pose', 'map': mp, 'odom': od})
        if self.traj and mp and time.monotonic() - self.traj_t0 >= 1.0 * self.traj_n:
            self.traj_n += 1
            if self.traj_last:
                self.traj_len += math.hypot(mp[0] - self.traj_last[0], mp[1] - self.traj_last[1])
            self.traj_last = mp
            self.traj.writerow([round(time.monotonic() - self.traj_t0, 1), *map(lambda v: round(v, 4), mp),
                                *(map(lambda v: round(v, 4), od) if od else ('', '', ''))])

    def tick_scan(self):
        m = self.scan_msg
        if not m:
            return
        step = max(1, len(m.ranges) // 180)
        rs = [round(r, 2) if m.range_min < r < m.range_max else None for r in m.ranges[::step]]
        emit({'type': 'scan', 'a0': m.angle_min, 'da': m.angle_increment * step, 'r': rs})

    def tick_status(self):
        emit({'type': 'status', 'scan_hz': round(self.rates['scan'].hz(), 1),
              'odom_hz': round(self.rates['odom'].hz(), 1), 'map_hz': round(self.rates['map'].hz(), 2),
              'scan_age': self.rates['scan'].age(), 'odom_age': self.rates['odom'].age(),
              'battery': self.battery, 'docked': self.docked,
              'nav': self.nav_state, 'nav_remaining': self.nav_remaining,
              'robot_action': self.robot_action,
              'tracking': bool(self.traj)})

    def tick_map(self):
        m = self.map_msg
        if m is None or m is self.map_sent:
            return
        self.map_sent = m
        w, h = m.info.width, m.info.height
        a = np.array(m.data, dtype=np.int8).reshape(h, w)
        img = np.full((h, w), 205, np.uint8)
        img[a == 0] = 254
        img[a >= 65] = 0
        free = int((a == 0).sum())
        occ = int((a >= 65).sum())
        res = m.info.resolution
        emit({'type': 'map', 'w': w, 'h': h, 'res': res,
              'ox': m.info.origin.position.x, 'oy': m.info.origin.position.y,
              'free_cells': free, 'occ_cells': occ, 'explored_m2': round((free + occ) * res * res, 2),
              'png': base64.b64encode(png_gray(np.flipud(img))).decode()})

    def tick_deadman(self):
        if self.moving and time.monotonic() - self.last_vel > 0.5:
            self.publish_vel(0.0, 0.0)
            self.moving = False

    # ---- commands ----
    def vel_pub(self, stamped):
        key = 'stamped' if stamped else 'plain'
        if key not in self.vel_pubs:
            self.vel_pubs[key] = self.create_publisher(
                TwistStamped if stamped else Twist,
                f'{self.ns}/cmd_vel' if stamped else f'{self.ns}/cmd_vel_unstamped', 10)
        return self.vel_pubs[key]

    def publish_vel(self, lin, ang, mode='auto'):
        # Plain Twist on cmd_vel_unstamped when something takes it, else stamped on cmd_vel.
        plain = mode == 'plain' or (mode == 'auto' and self.count_subscribers(f'{self.ns}/cmd_vel_unstamped') > 0)
        if plain:
            m = Twist()
            m.linear.x, m.angular.z = float(lin), float(ang)
            self.vel_pub(False).publish(m)
        else:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x, m.twist.angular.z = float(lin), float(ang)
            self.vel_pub(True).publish(m)

    def send_goal(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.nav_state = 'nav2 not running'
            return
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.nav_state = 'sending'
        fut = self.nav.send_goal_async(g, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.nav_state = 'rejected'
            return
        self.goal_handle = gh
        self.nav_state = 'navigating'
        gh.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, fb):
        self.nav_remaining = round(fb.feedback.distance_remaining, 2)

    def on_result(self, fut):
        status = {4: 'succeeded', 5: 'cancelled', 6: 'failed'}.get(fut.result().status, 'finished')
        self.nav_state, self.nav_remaining, self.goal_handle = status, None, None

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def estop(self):
        self.cancel_goal()
        for stamped in (False, True):
            self.publish_vel_to(stamped)
        self.moving = False

    def publish_vel_to(self, stamped):
        if stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub(True).publish(m)
        else:
            self.vel_pub(False).publish(Twist())

    def toast(self, text, level='info'):
        emit({'type': 'toast', 'text': text, 'level': level})

    def dock_action(self, which):
        """Undock / Dock the Create 3 (irobot_create_msgs actions)."""
        client = self.dock_clients.get(which)
        if client is None:
            return self.toast('irobot_create_msgs is not installed in this image', 'error')
        if self.robot_action:
            return self.toast(f'{self.robot_action} is still running', 'warn')
        if not client.wait_for_server(timeout_sec=3.0):
            return self.toast(f'{self.ns}/{which} action not available: is the robot (or the simulation) connected?', 'error')
        self.robot_action = f'{which}ing'
        self.toast(f'{which.capitalize()}ing…')
        fut = client.send_goal_async((Undock if which == 'undock' else Dock).Goal())

        def accepted(f):
            gh = f.result()
            if not gh.accepted:
                self.robot_action = None
                return self.toast(f'{which} goal rejected', 'error')
            gh.get_result_async().add_done_callback(done)

        def done(f):
            self.robot_action = None
            ok = f.result().status == 4
            res = f.result().result
            self.toast(f'{which.capitalize()} ' + ('succeeded' if ok else 'failed'), 'ok' if ok else 'error')
            if which == 'dock' and ok and getattr(res, 'is_docked', True) is False:
                self.toast('Dock finished but the robot is not docked: try again closer to the dock', 'warn')
        fut.add_done_callback(accepted)

    def lidar(self, which):
        cli = self.motor[which]
        if not cli.wait_for_service(timeout_sec=3.0):
            return self.toast(f'{self.ns}/{which}_motor service not available (the TurtleBot 4 lidar driver provides it)', 'error')
        cli.call_async(Empty.Request()).add_done_callback(
            lambda f: self.toast(f'Lidar motor {which} sent', 'ok'))

    def traj_start(self, path):
        f = open(path, 'w', newline='')
        self.traj_file, self.traj = f, csv.writer(f)
        self.traj.writerow(['t_s', 'map_x', 'map_y', 'map_yaw', 'odom_x', 'odom_y', 'odom_yaw'])
        self.traj_n, self.traj_len, self.traj_last, self.traj_t0 = 0, 0.0, None, time.monotonic()
        self.traj_path = path

    def traj_stop(self):
        if self.traj:
            self.traj_file.close()
            emit({'type': 'traj_done', 'path': self.traj_path, 'length_m': round(self.traj_len, 2),
                  'duration_s': round(time.monotonic() - self.traj_t0, 1)})
            self.traj = None

    def probe(self, req_id):
        """Everything the preflight pipeline needs, after a short settle time for discovery."""
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and not (self.rates['scan'].t and self.rates['odom'].t):
            time.sleep(0.2)
        time.sleep(0.5)
        ns = self.ns
        topics = dict(self.get_topic_names_and_types())

        def qos_of(topic):
            infos = self.get_publishers_info_by_topic(topic)
            return str(infos[0].qos_profile.reliability.name) if infos else None

        nodes = sorted({(n if not s or s == '/' else f'{s}/{n}').lstrip('/')
                        for n, s in self.get_node_names_and_namespaces()} - {'tb4_gui_helper'})
        scan_frame = self.scan_frame
        scan_valid, scan_span = None, None
        m = self.scan_msg
        if m is not None:      # a working lidar sees a spread of distances; a stuck or broken one returns one value
            fin = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
            if fin:
                scan_span = [round(min(fin), 2), round(max(fin), 2)]
                scan_valid = len(fin) > 20 and max(fin) - min(fin) > 0.05
            else:
                scan_valid = False
        emit({'type': 'probe_result', 'id': req_id, 'scan_valid': scan_valid, 'scan_span': scan_span,
              'nodes': nodes, 'n_topics': len(topics),
              'scan_hz': round(self.rates['scan'].hz(), 1), 'scan_frame': scan_frame,
              'scan_qos': qos_of(f'{ns}/scan'), 'odom_hz': round(self.rates['odom'].hz(), 1),
              'scan_delay': round(float(np.median(list(self.scan_delay))), 2) if self.scan_delay else None,
              'odom_base': self.lookup('odom', 'base_link') is not None,
              'scan_tf': bool(scan_frame) and self.tf.can_transform('base_link', scan_frame, Time()),
              'map_odom': self.tf_fresh('map', 'odom'),
              'cmd_vel_types': topics.get(f'{ns}/cmd_vel', []),
              'cmd_vel_subs': self.count_subscribers(f'{ns}/cmd_vel'),
              'unstamped_subs': self.count_subscribers(f'{ns}/cmd_vel_unstamped'),
              'battery': self.battery, 'docked': self.docked})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--namespace', default='')
    ap.add_argument('--sim-time', action='store_true')
    args = ap.parse_args()
    ns = '/' + args.namespace.strip('/') if args.namespace.strip('/') else ''
    ros_args = ['--ros-args'] + (['-r', f'/tf:={ns}/tf', '-r', f'/tf_static:={ns}/tf_static'] if ns else [])
    rclpy.init(args=ros_args if ns else None)
    node = Helper(ns, args.sim_time)
    ex =MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    emit({'type': 'ready', 'namespace': ns})
    for line in sys.stdin:
        try:
            c = json.loads(line)
            cmd = c.get('cmd')
            if cmd == 'vel':
                node.last_vel, node.moving = time.monotonic(), True
                node.publish_vel(c['lin'], c['ang'], c.get('mode', 'auto'))
            elif cmd == 'goal':
                node.send_goal(c['x'], c['y'], c.get('yaw', 0.0))
            elif cmd == 'cancel':
                node.cancel_goal()
            elif cmd in ('undock', 'dock'):
                node.dock_action(cmd)
            elif cmd in ('lidar_start', 'lidar_stop'):
                node.lidar(cmd.split('_')[1])
            elif cmd == 'goto_start':   # Hector's map frame starts at the robot's start pose
                node.send_goal(0.0, 0.0, 0.0)
            elif cmd == 'estop':
                node.estop()
            elif cmd == 'probe':
                node.probe(c.get('id'))
            elif cmd == 'traj_start':
                node.traj_start(c['path'])
            elif cmd == 'traj_stop':
                node.traj_stop()
        except Exception as e:  # keep the helper alive; the server shows this in the log
            print(f'helper error: {e!r}', file=sys.stderr, flush=True)
    node.traj_stop()


if __name__ == '__main__':
    main()

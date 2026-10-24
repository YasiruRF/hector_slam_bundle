#!/usr/bin/env python3
"""Phase-2 tests in Webots (Nav2 + AMCL on a saved map), scored against /ground_truth/pose.

  nav_tests.py goals OUTDIR x,y x,y ...        NavigateToPose to each goal, one after another
  nav_tests.py kidnap OUTDIR X Y YAW_DEG       kidnapped-robot test (see below)

Both start by giving AMCL the start pose (0, 0, 0), the origin of the saved Hector map.
goals: per goal the result, time, where the robot really stopped, AMCL's estimate (TF
  map -> base_link) and the error between them -> goals.csv, goals.md.
kidnap: 1) a short goal so AMCL converges, 2) /kidnap teleports the robot to (X, Y, YAW),
  3) the robot turns in place for 30 s (AMCL as configured, no help),
  4) /reinitialize_global_localization + turning for up to 60 s,
  5) kidnap again and set the true pose with /initialpose (manual relocalisation),
  6) a goal to check the robot navigates again.
  AMCL error and particle spread are logged at 2 Hz -> kidnap.csv, kidnap_events.txt.
Run with --ros-args -p use_sim_time:=true.
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Tester:
    def __init__(self, out):
        self.out = out
        os.makedirs(out, exist_ok=True)
        self.n = rclpy.create_node('nav_tests')
        self.gt = self.amcl_msg = None
        self.tf = Buffer()
        TransformListener(self.tf, self.n)
        self.spread = float('nan')
        self.n.create_subscription(PoseStamped, 'ground_truth/pose', self.on_gt, 10)
        self.n.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl,
                                   QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.n.create_subscription(ParticleCloud, 'particle_cloud', self.on_cloud,
                                   qos_profile_sensor_data)
        self.init_pub = self.n.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.kidnap_pub = self.n.create_publisher(Pose2D, 'kidnap', 10)
        self.cmd_pub = self.n.create_publisher(Twist, 'cmd_vel_unstamped', 10)
        self.nav = ActionClient(self.n, NavigateToPose, 'navigate_to_pose')
        self.global_loc = self.n.create_client(Empty, 'reinitialize_global_localization')
        self.rows = []
        self.t0 = None
        self.events = []

    # -- callbacks / helpers ---------------------------------------------
    def on_gt(self, m):
        p = m.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_amcl(self, m):
        p = m.pose.pose
        self.amcl_msg = (p.position.x, p.position.y, yaw_of(p.orientation))

    @property
    def amcl(self):
        # The localisation Nav2 uses: map -> odom from AMCL composed with the latest
        # odometry. /amcl_pose itself only updates after update_min_d / update_min_a of
        # motion, so it lags behind a robot that has just stopped.
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return self.amcl_msg
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def on_cloud(self, m):
        xs = [p.pose.position.x for p in m.particles]
        ys = [p.pose.position.y for p in m.particles]
        if xs:
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            self.spread = math.sqrt(sum((x - mx) ** 2 + (y - my) ** 2
                                        for x, y in zip(xs, ys)) / len(xs))

    def now(self):
        return self.n.get_clock().now().nanoseconds * 1e-9

    def spin(self, seconds, log=False, phase=''):
        # seconds of sim time: Webots runs slower than real time under load
        end = self.now() + seconds
        nxt = 0.0
        while self.now() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if log and self.now() >= nxt:
                nxt = self.now() + 0.5
                self.log(phase)

    def err(self):
        if not (self.gt and self.amcl):
            return float('nan'), float('nan')
        return (math.hypot(self.gt[0] - self.amcl[0], self.gt[1] - self.amcl[1]),
                abs(math.degrees(wrap(self.gt[2] - self.amcl[2]))))

    def log(self, phase):
        t = self.now()
        self.t0 = t if self.t0 is None else self.t0
        e, eyaw = self.err()
        self.rows.append((t - self.t0, phase, self.gt, self.amcl, e, eyaw, self.spread))

    def event(self, text):
        t = self.now() - (self.t0 or self.now())
        self.events.append('%7.1f s  %s' % (t, text))
        print(self.events[-1], flush=True)

    def wait_ready(self):
        while self.gt is None:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if not self.nav.wait_for_server(timeout_sec=120):
            sys.exit('navigate_to_pose not available')

    def set_pose(self, x, y, yaw, cov=0.05):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = cov
        m.pose.covariance[35] = 0.03
        for _ in range(3):
            self.init_pub.publish(m)
            self.spin(0.3)

    def goto(self, x, y, yaw=0.0, timeout=150.0, phase='goal'):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t_start = self.now()
        fut = self.nav.send_goal_async(g)
        while not fut.done():
            self.spin(0.1, True, phase)
        h = fut.result()
        if not h.accepted:
            return 'REJECTED', 0.0
        res = h.get_result_async()
        end = self.now() + timeout
        while not res.done() and self.now() < end:
            self.spin(0.2, True, phase)
        if not res.done():
            h.cancel_goal_async()
            self.spin(1.0)
            return 'TIMEOUT', self.now() - t_start
        return STATUS.get(res.result().status, str(res.result().status)), self.now() - t_start

    def turn(self, seconds, w=0.5, phase='turn', stop_when=None):
        t = Twist()
        t.angular.z = w
        end = self.now() + seconds
        while self.now() < end:
            self.cmd_pub.publish(t)
            self.spin(0.1, True, phase)
            if stop_when and stop_when():
                break
        self.cmd_pub.publish(Twist())
        self.spin(0.5)

    def kidnap(self, x, y, yaw_deg):
        m = Pose2D()
        m.x, m.y, m.theta = float(x), float(y), math.radians(float(yaw_deg))
        self.kidnap_pub.publish(m)
        self.spin(1.0)

    def write_csv(self, name):
        f = lambda p: ('%.3f,%.3f,%.3f' % p) if p else ',,'
        with open(os.path.join(self.out, name), 'w') as fh:
            fh.write('t_s,phase,gt_x,gt_y,gt_yaw,amcl_x,amcl_y,amcl_yaw,err_m,err_deg,spread_m\n')
            for t, ph, g, a, e, ey, s in self.rows:
                fh.write('%.2f,%s,%s,%s,%.3f,%.2f,%.3f\n' % (t, ph, f(g), f(a), e, ey, s))

    # -- tests -----------------------------------------------------------
    def goals(self, goals):
        self.wait_ready()
        self.set_pose(0.0, 0.0, 0.0)
        self.spin(3, True, 'init')
        lines = ['| # | Goal (map) | Result | Time [s] | Stopped at (ground truth) | AMCL estimate '
                 '| AMCL error [cm] | Heading error [deg] | Distance to goal [cm] |',
                 '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
        with open(os.path.join(self.out, 'goals.csv'), 'w') as fh:
            fh.write('goal_x,goal_y,result,time_s,gt_x,gt_y,gt_yaw,amcl_x,amcl_y,amcl_yaw,'
                     'amcl_err_m,amcl_err_deg,dist_to_goal_m\n')
            for i, (x, y) in enumerate(goals, 1):
                self.event('goal %d -> (%.2f, %.2f)' % (i, x, y))
                status, dt = self.goto(x, y, phase='goal%d' % i)
                self.spin(2.0, True, 'goal%d' % i)
                e, ey = self.err()
                g, a = self.gt, self.amcl
                d = math.hypot(g[0] - x, g[1] - y)
                self.event('goal %d %s in %.1f s, AMCL error %.1f cm' % (i, status, dt, e * 100))
                fh.write('%.2f,%.2f,%s,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.2f,%.3f\n' % (
                    x, y, status, dt, g[0], g[1], g[2], a[0], a[1], a[2], e, ey, d))
                lines.append('| %d | (%.2f, %.2f) | %s | %.0f | (%.3f, %.3f) | (%.3f, %.3f) | %.1f | %.1f | %.0f |' % (
                    i, x, y, status, dt, g[0], g[1], a[0], a[1], e * 100, ey, d * 100))
        open(os.path.join(self.out, 'goals.md'), 'w').write('\n'.join(lines) + '\n')
        self.write_csv('amcl_track.csv')

    def kidnap_test(self, kx, ky, kyaw):
        self.wait_ready()
        self.set_pose(0.0, 0.0, 0.0)
        self.spin(2, True, 'init')
        self.event('converge: short goal to (1.5, -1.0)')
        st, _ = self.goto(1.5, -1.0, phase='converge')
        self.spin(2, True, 'converge')
        self.event('converged (%s), AMCL error %.1f cm' % (st, self.err()[0] * 100))
        open(os.path.join(self.out, 'marker_before_kidnap'), 'w').close()
        time.sleep(1.5)  # the driver script takes a screenshot here

        self.event('KIDNAP 1 -> (%s, %s, %s deg)' % (kx, ky, kyaw))
        self.kidnap(kx, ky, kyaw)
        self.spin(2, True, 'kidnapped')
        self.event('after kidnap: AMCL error %.2f m (AMCL still at the old pose)' % self.err()[0])
        self.turn(30, phase='no_help')
        self.event('after 30 s turning, no help: AMCL error %.2f m' % self.err()[0])
        open(os.path.join(self.out, 'marker_lost'), 'w').close()
        time.sleep(1.5)

        self.global_loc.wait_for_service(timeout_sec=10)
        self.global_loc.call_async(Empty.Request())
        self.spin(1, True, 'global')
        self.event('reinitialize_global_localization called (spread %.2f m)' % self.spread)
        open(os.path.join(self.out, 'marker_global_spread'), 'w').close()
        time.sleep(1.5)
        t_g = self.now()
        conv = lambda: self.spread < 0.25 and self.err()[0] < 0.25
        self.turn(60, phase='global', stop_when=None)
        e = self.err()[0]
        conv_t = next((r[0] for r in self.rows if r[1] == 'global' and r[6] < 0.25 and r[4] < 0.25), None)
        self.event('global localisation after 60 s turning: AMCL error %.2f m, spread %.2f m, '
                   'first converged at %s' % (e, self.spread,
                                              '%.1f s' % (conv_t - (t_g - self.t0)) if conv_t else 'never'))
        open(os.path.join(self.out, 'marker_global_end'), 'w').close()
        time.sleep(1.5)
        if e > 0.25:
            self.event('global localisation did not converge: drive a short goal and keep turning')
            self.goto(self.amcl[0] + 0.0, self.amcl[1], timeout=20, phase='global')
            self.turn(30, phase='global')
            self.event('after extra motion: AMCL error %.2f m' % self.err()[0])

        self.event('KIDNAP 2 -> (0.5, -5.5, 90 deg)')
        self.kidnap(0.5, -5.5, 90)
        self.spin(2, True, 'kidnapped2')
        self.event('after kidnap 2: AMCL error %.2f m' % self.err()[0])
        g = self.gt
        self.set_pose(g[0], g[1], g[2], cov=0.1)
        self.event('manual relocalisation: /initialpose at the true pose')
        self.turn(10, phase='manual')
        self.event('after manual pose + 10 s turning: AMCL error %.1f cm' % (self.err()[0] * 100))
        st, dt = self.goto(1.8, -1.3, phase='after')
        self.spin(2, True, 'after')
        self.event('goal after recovery (1.8, -1.3): %s in %.0f s, AMCL error %.1f cm' % (
            st, dt, self.err()[0] * 100))
        self.write_csv('kidnap.csv')
        open(os.path.join(self.out, 'kidnap_events.txt'), 'w').write('\n'.join(self.events) + '\n')


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    rclpy.init(args=sys.argv)
    cmd, out = args[1], args[2]
    t = Tester(out)
    if cmd == 'goals':
        t.goals([tuple(map(float, g.split(','))) for g in args[3:]])
    elif cmd == 'kidnap':
        t.kidnap_test(*args[3:6])
    open(os.path.join(out, 'marker_done'), 'w').close()
    t.n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

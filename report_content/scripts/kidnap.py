#!/usr/bin/env python3
"""Teleport the Webots robot (bridge /kidnap) and print the true pose before and after.

  kidnap.py abs X Y YAW_DEG      start-relative pose (the /ground_truth/pose frame)
  kidnap.py rel DX DY DYAW_DEG   offset in the robot's own frame
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    rclpy.init(args=sys.argv)
    n = rclpy.create_node('kidnap')
    gt = []
    n.create_subscription(PoseStamped, 'ground_truth/pose', lambda m: gt.append(m), 10)
    pub = n.create_publisher(Pose2D, 'kidnap', 10)
    while not gt:
        rclpy.spin_once(n, timeout_sec=0.1)
    p = gt[-1].pose
    x, y = p.position.x, p.position.y
    yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    a, b, c = map(float, args[2:5])
    if args[1] == 'rel':
        tx = x + math.cos(yaw) * a - math.sin(yaw) * b
        ty = y + math.sin(yaw) * a + math.cos(yaw) * b
        tyaw = yaw + math.radians(c)
    else:
        tx, ty, tyaw = a, b, math.radians(c)
    m = Pose2D()
    m.x, m.y, m.theta = tx, ty, tyaw
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.5)
    print('kidnap t=%.1f s: from (%.3f, %.3f, %.1f deg) to (%.3f, %.3f, %.1f deg): '
          'jump %.2f m, %.1f deg' % (
              gt[-1].header.stamp.sec + gt[-1].header.stamp.nanosec * 1e-9,
              x, y, math.degrees(yaw), tx, ty, math.degrees(tyaw), math.hypot(tx - x, ty - y),
              math.degrees(math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw)))), flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

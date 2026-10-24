#!/usr/bin/env bash
# Checks before running slam_nav_launch.py on a real TurtleBot 4 (Jazzy).
#   ./preflight_real_robot.sh [namespace]      e.g. /tb4  (default: none)
# Run it in the same shell/container, with the same ROS_DOMAIN_ID and RMW as
# the robot. Read-only: it only lists and echoes topics.
NS="${1:-}"
ok()   { echo "  ok    $*"; }
warn() { echo "  WARN  $*"; FAIL=1; }
FAIL=0
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-default}"

TOPICS=$(timeout 15 ros2 topic list -t 2>/dev/null)
[ -n "$TOPICS" ] || { echo "no topics visible: check ROS_DOMAIN_ID/RMW and the network"; exit 1; }

echo "topics"
for t in scan odom tf tf_static; do
  grep -q "^$NS/$t " <<<"$TOPICS" && ok "$NS/$t" || warn "$NS/$t missing"
done

echo "scan"
QOS=$(ros2 topic info "$NS/scan" -v 2>/dev/null | grep -m1 -i reliability | awk '{print $2}')
ok "publisher reliability: ${QOS:-unknown} (Hector subscribes with sensor-data QoS)"
FRAME=$(timeout 10 ros2 topic echo "$NS/scan" --once --field header.frame_id 2>/dev/null | head -1)
[ -n "$FRAME" ] && ok "frame_id: $FRAME (must exist in TF under base_link)" || warn "no /scan message in 10 s"

echo "clock offset (informational: nav2.yaml is set up to work without clock sync)"
DELAY=$(timeout 8 ros2 topic delay "$NS/scan" 2>/dev/null | grep -m1 "average delay" | awk '{print $3}')
if [ -z "$DELAY" ]; then echo "        could not measure /scan delay"
elif awk -v d="$DELAY" 'BEGIN { exit !(d < 0.5 && d > -0.5) }'; then ok "scans arrive ${DELAY} s after their stamp (clocks agree)"
else ok "scans arrive ${DELAY} s after their stamp: robot and laptop clocks differ (fine for this setup)"; fi

echo "cmd_vel (Nav2 here outputs Twist on ${NS}/cmd_vel_unstamped)"
for t in cmd_vel cmd_vel_unstamped; do
  TYPE=$(grep "^$NS/$t " <<<"$TOPICS" | sed 's/.*\[\(.*\)\]/\1/')
  SUBS=$(ros2 topic info "$NS/$t" 2>/dev/null | grep -i 'subscription count' | awk '{print $3}')
  echo "        $NS/$t  type=${TYPE:-none}  subscribers=${SUBS:-0}"
done
U=$(ros2 topic info "$NS/cmd_vel_unstamped" 2>/dev/null | grep -i 'subscription count' | awk '{print $3}')
if [ "${U:-0}" -ge 1 ]; then ok "something takes Twist on cmd_vel_unstamped: use the launch as is"
else warn "nothing takes cmd_vel_unstamped: run  ros2 run tb4_hector_slam cmd_vel_stamper.py --in cmd_vel_unstamped --out cmd_vel  alongside the launch"; fi

echo "map -> odom publishers (must be none before Hector starts)"
if timeout 8 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q Translation; then
  warn "something already publishes map frames (stock slam_toolbox / localization?)"
else ok "no map frame in TF"; fi

echo "nodes that conflict"
ros2 node list 2>/dev/null | grep -E "slam_toolbox|amcl|map_server|bt_navigator|controller_server" \
  && warn "stop the stock navigation/localization nodes above" || ok "none running"

[ $FAIL = 0 ] && echo "PREFLIGHT OK" || echo "PREFLIGHT: fix the WARN items first"
exit $FAIL

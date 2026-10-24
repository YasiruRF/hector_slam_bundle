#!/usr/bin/env bash
# Replay the real-robot bag (no /map, no map->odom) into a fresh Hector with
# parameter overrides, inside the jazzy container. Each variant uses its own
# ROS_DOMAIN_ID, so several can run at once.
#   replay_real.sh BAG OUTDIR DOMAIN [param:=value ...]
# e.g. replay_real.sh real_stripped out/R1 51 use_tf_pose_start_estimate:=true
set -eo pipefail
BAG="$1"; OUT="$2"; export ROS_DOMAIN_ID="$3"; shift 3
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
PARAMS="$OUT/hector_slam.yaml"
cp "$(ros2 pkg prefix tb4_hector_slam)/share/tb4_hector_slam/config/hector_slam.yaml" "$PARAMS"
for kv in "$@"; do
  k="${kv%%:=*}"; v="${kv#*:=}"
  sed -i "s/^\(\s*$k:\).*/\1 $v/" "$PARAMS"
  grep -q "^\s*$k: $v" "$PARAMS" || { echo "param $k not in yaml" >&2; exit 1; }
done
echo "$*" > "$OUT/overrides.txt"

ros2 launch tb4_hector_slam hector_slam_launch.py use_sim_time:=true use_rviz:=false \
  params_file:="$PARAMS" > "$OUT/hector.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/run_logger.py" "$OUT" --ros-args -p use_sim_time:=true > "$OUT/logger.log" 2>&1 &
LOGGER=$!
sleep 4
ros2 bag play "$BAG" --clock 100 > "$OUT/play.log" 2>&1
sleep 3   # last map publish (map_pub_period 2 s)
kill -INT $LOGGER; wait $LOGGER || true
kill -INT $LAUNCH; wait $LAUNCH || true
cat "$OUT/logger.log" | tail -1

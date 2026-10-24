#!/usr/bin/env bash
# Full autonomous run in Webots: Hector SLAM + Nav2 + frontier explorer in one launch
# (slam_nav_launch.py explore:=true), as runs B-D in webots/results/2026-10-24.
#   run_w7_explore.sh OUTDIR
OUT="$(realpath -m "$1")"
source "$(dirname "$0")/sim_lib.sh"
TOOLS="$REPO_CONT/turtlebot_hector_slam/webots/tools"
trap 'rec_stop; ros_stop_all; sim_stop' EXIT

ros slam "ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true explore:=true use_rviz:=false"
sleep 5
sim_start
ros rviz "rviz2 -d $SCRIPTS_CONT/rviz/phase2.rviz --ros-args -p use_sim_time:=true"
wait_for "$OUT/slam.log" 'map lvl 1' 60
sleep 6
win_move "RViz" 1000 0 920 1080
ros pose "python3 $TOOLS/pose_logger.py pose.csv --ros-args -p use_sim_time:=true"
rec_start video
T0=$(date +%s); last=0; graph=0
until grep -q 'exploration complete\|Exploration complete' "$OUT/slam.log"; do
  sleep 1
  el=$(( $(date +%s) - T0 ))
  if [ $((el - last)) -ge 45 ]; then shot "t$(printf '%03d' "$el")s"; last=$el; fi
  if [ "$graph" = 0 ] && [ "$el" -ge 60 ]; then
    graph=1
    ros_fg "python3 $SCRIPTS_CONT/ros_graph.py ros_graph_slam_nav --skip pose_logger,rviz2" > "$OUT/graph.log" 2>&1 &
  fi
  [ "$el" -gt 1200 ] && { echo "exploration did not finish in 1200 s" >&2; break; }
done
echo "exploration time $(( $(date +%s) - T0 )) s (wall)" > "$OUT/explore_time.txt"
sleep 3
shot final
ros_fg "ros2 run nav2_map_server map_saver_cli -f map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true" > "$OUT/map_saver.log" 2>&1 || true
rec_stop
ros_stop_all
ros_fg "python3 $TOOLS/evaluate_map.py map.yaml --pose-log pose.csv --plot eval.png > eval.txt 2>&1; chown -R $(id -u):$(id -g) ." || true
grep -c 'Exploring towards' "$OUT/slam.log" | sed 's/^/goal attempts: /' >> "$OUT/explore_time.txt"
grep -c 'Goal timed out' "$OUT/slam.log" | sed 's/^/timed out: /' >> "$OUT/explore_time.txt"
grep -c 'Goal not reached' "$OUT/slam.log" | sed 's/^/not reached: /' >> "$OUT/explore_time.txt"
grep -c 'Goal rejected' "$OUT/slam.log" | sed 's/^/rejected: /' >> "$OUT/explore_time.txt"
cat "$OUT/eval.txt" "$OUT/explore_time.txt"

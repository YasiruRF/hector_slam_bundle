#!/usr/bin/env bash
# W1 (and variants): phase 1 in Webots, Hector SLAM only, the bridge drives the patrol loop.
#   run_w1_mapping.sh OUTDIR [--patrol-speed K] [--kidnap-at S abs|rel X Y YAW_DEG] [--no-graph]
# Writes map.*, pose.csv (pose_logger), trajectory.csv (run_logger), eval.*, screenshots,
# video.mp4, ros_graph.*, frames.pdf, rates.txt. --kidnap-at teleports the robot S seconds
# after the patrol starts (kidnap.py: abs = start-relative pose, rel = robot frame).
OUT="$(realpath -m "$1")"; shift
SPEED=1.0; KIDNAP=""; GRAPH=1
while [ $# -gt 0 ]; do
  case "$1" in
    --patrol-speed) SPEED="$2"; shift ;;
    --kidnap-at) KIDNAP="$2 $3 $4 $5 $6"; shift 5 ;;
    --no-graph) GRAPH=0 ;;
  esac
  shift
done
source "$(dirname "$0")/sim_lib.sh"
TOOLS="$REPO_CONT/turtlebot_hector_slam/webots/tools"
trap 'rec_stop; ros_stop_all; sim_stop' EXIT

ros hector "ros2 launch tb4_hector_slam hector_slam_launch.py use_sim_time:=true use_rviz:=true rviz_config:=$SCRIPTS_CONT/rviz/phase1.rviz"
sleep 6
sim_start --patrol --patrol-speed "$SPEED"
wait_for "$OUT/hector.log" 'map lvl 1' 60
sleep 4
win_move "RViz" 1000 0 920 1080
ros pose "python3 $TOOLS/pose_logger.py pose.csv --ros-args -p use_sim_time:=true"
ros runlog "python3 $SCRIPTS_CONT/run_logger.py . --ros-args -p use_sim_time:=true"
sleep 2
shot 00_start
rec_start video
T0=$(date +%s)
wait_for "$OUT/sim.log" 'Patrol waits|Patrol' 30 || true

i=1; kidnapped=0; last=0; graph=0
until grep -q 'Patrol finished' "$OUT/sim.log"; do
  sleep 1
  el=$(( $(date +%s) - T0 ))
  if [ -n "$KIDNAP" ] && [ "$kidnapped" = 0 ]; then
    read -r ks kmode kx ky kyaw <<< "$KIDNAP"
    if [ "$el" -ge "$ks" ]; then
      shot kidnap_before
      ros_fg "python3 $SCRIPTS_CONT/kidnap.py $kmode $kx $ky $kyaw --ros-args -p use_sim_time:=true" > "$OUT/kidnap.log" 2>&1
      kidnapped=1; sleep 4; shot kidnap_after_4s; sleep 20; shot kidnap_after_24s
    fi
  fi
  if [ $((el - last)) -ge 45 ]; then shot "$(printf '%02d' $i)_t${el}s"; i=$((i + 1)); last=$el; fi
  if [ "$GRAPH" = 1 ] && [ "$graph" = 0 ] && [ "$el" -ge 40 ]; then
    graph=1
    ros_fg "python3 $SCRIPTS_CONT/ros_graph.py ros_graph_phase1 && ros2 run tf2_tools view_frames -o frames_phase1 >/dev/null 2>&1 && timeout 12 ros2 topic hz /scan > hz_scan.txt 2>&1; timeout 12 ros2 topic hz /slam_out_pose > hz_slam.txt 2>&1; timeout 12 ros2 topic hz /map > hz_map.txt 2>&1; true" &
  fi
  [ "$el" -gt 900 ] && { echo "patrol did not finish in 900 s" >&2; break; }
done
echo "patrol time $(( $(date +%s) - T0 )) s (wall)" > "$OUT/patrol_time.txt"
sleep 3
shot 99_final
ros_fg "ros2 run nav2_map_server map_saver_cli -f map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true" > "$OUT/map_saver.log" 2>&1 || true
rec_stop
ros_stop_all
ros_fg "python3 $TOOLS/evaluate_map.py map.yaml --pose-log pose.csv --plot eval.png > eval.txt 2>&1; chown -R $(id -u):$(id -g) ." || true
cat "$OUT/eval.txt"

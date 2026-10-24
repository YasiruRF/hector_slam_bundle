#!/usr/bin/env bash
# Run on a computer with ROS 2 Jazzy installed natively (after native/setup.sh). No Docker.
#   ./native/run.sh                      dashboard on http://localhost:8080 (real robot target)
#   ./native/run.sh preflight [/ns]      robot connection checks in the terminal
#   ./native/run.sh launch [args...]     SLAM + Nav2 from the terminal, e.g. explore:=true use_rviz:=true
# Two-phase run (map first, then autonomous; Hector and Nav2 never run together):
#   ./native/run.sh map                  phase 1: Hector SLAM + RViz only, drive by hand
#   ./native/run.sh savemap [name]       while phase 1 runs: save to ~/tb4_data/maps/<name>.yaml
#   ./native/run.sh nav [name|map.yaml]  phase 2 (stop phase 1 first): Nav2 + AMCL on the saved map
# The dashboard sets ROS_DOMAIN_ID / RMW / discovery server from its Settings page.
# For preflight and launch, export the robot's values first, e.g.
#   export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Terminal output is also written to ~/tb4_data/logs/ (bring that file back if something fails).
set -eo pipefail      # no -u: the ROS setup scripts read unset variables
REPO="$(cd "$(dirname "$0")/.." && pwd)"
if [ -n "${TB4_WS:-}" ]; then WS="$TB4_WS"
elif [ "$(basename "$(dirname "$REPO")")" = src ]; then WS="$(dirname "$(dirname "$REPO")")"
else WS="$HOME/tb4_ws"; fi
[ -f "$WS/install/setup.bash" ] || { echo "No build in $WS: run $REPO/native/setup.sh first" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

DATA="${DATA_DIR:-$HOME/tb4_data}"
mkdir -p "$DATA/logs"
MODE="${1:-dashboard}"; [ $# -gt 0 ] && shift
LOG="$DATA/logs/$(date +%Y%m%d_%H%M%S)_$MODE.log"

case "$MODE" in
  dashboard)
    echo "Dashboard: http://localhost:${PORT:-8080}   (Ctrl+C to stop; results in $DATA/results)"
    DATA_DIR="$DATA" exec python3 "$REPO/app/server.py" ;;
  preflight)
    ros2 run tb4_hector_slam preflight_real_robot.sh "$@" 2>&1 | tee -i "$LOG" ;;
  launch)
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}   log: $LOG"
    ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=false use_rviz:=false "$@" 2>&1 | tee -i "$LOG" ;;
  map)
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}   log: $LOG"
    echo "Drive slowly by hand (teleop in another terminal), then: ./native/run.sh savemap <name>"
    ros2 launch tb4_hector_slam hector_slam_launch.py use_sim_time:=false use_rviz:=true "$@" 2>&1 | tee -i "$LOG" ;;
  savemap)
    mkdir -p "$DATA/maps"
    NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
    ros2 run nav2_map_server map_saver_cli -f "$DATA/maps/$NAME" 2>&1 | tee -i "$LOG"
    [ -f "$DATA/maps/$NAME.yaml" ] && echo "Saved $DATA/maps/$NAME.yaml  ->  next: stop phase 1, then ./native/run.sh nav $NAME" ;;
  nav)
    MAP="${1:-$(ls -t "$DATA"/maps/*.yaml 2>/dev/null | head -1)}"; [ $# -gt 0 ] && shift
    case "$MAP" in */*) ;; *) MAP="$DATA/maps/${MAP%.yaml}.yaml" ;; esac
    [ -f "$MAP" ] || { echo "No map '$MAP': run ./native/run.sh savemap <name> during phase 1 first" >&2; exit 1; }
    if ros2 node list 2>/dev/null | grep -q hector_mapping; then
      echo "hector_mapping is still running: stop phase 1 first (it would fight AMCL over map->odom)" >&2; exit 1
    fi
    echo "Map: $MAP   log: $LOG"
    echo "Start the robot where mapping started (or use '2D Pose Estimate' in RViz), then send 'Nav2 Goal's"
    ros2 launch tb4_hector_slam nav_on_map_launch.py map:="$MAP" use_sim_time:=false use_rviz:=true "$@" 2>&1 | tee -i "$LOG" ;;
  *)
    sed -n '2,13p' "$0"; exit 1 ;;
esac

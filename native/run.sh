#!/usr/bin/env bash
# Run on a computer with ROS 2 Jazzy installed natively (after native/setup.sh). No Docker.
#   ./native/run.sh                      dashboard on http://localhost:8080 (real robot target)
#   ./native/run.sh preflight [/ns]      robot connection checks in the terminal
#   ./native/run.sh launch [args...]     SLAM + Nav2 from the terminal, e.g. explore:=true use_rviz:=true
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
  *)
    sed -n '2,9p' "$0"; exit 1 ;;
esac

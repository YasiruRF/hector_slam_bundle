#!/usr/bin/env bash
# Phase 2 in Webots: Nav2 + map_server + AMCL on a saved Hector map, scored with nav_tests.py.
#   run_w2_nav.sh OUTDIR MAP.yaml goals "x,y x,y ..." [NAV2_PARAMS.yaml]
#   run_w2_nav.sh OUTDIR MAP.yaml kidnap "X Y YAW_DEG" [NAV2_PARAMS.yaml]
# MAP.yaml and NAV2_PARAMS.yaml are container paths.
OUT="$(realpath -m "$1")"; MAP="$2"; MODE="$3"; ARGS="$4"; PARAMS="${5:-}"
source "$(dirname "$0")/sim_lib.sh"
trap 'rec_stop; ros_stop_all; sim_stop' EXIT

EXTRA=""; [ -n "$PARAMS" ] && EXTRA="params_file:=$PARAMS"
ros nav "ros2 launch tb4_hector_slam nav_on_map_launch.py map:=$MAP use_sim_time:=true use_rviz:=false $EXTRA"
sleep 5
sim_start
ros rviz "rviz2 -d $SCRIPTS_CONT/rviz/phase2.rviz --ros-args -p use_sim_time:=true"
# localization + navigation lifecycle managers both report active
for i in $(seq 1 120); do
  [ "$(grep -c 'Managed nodes are active' "$OUT/nav.log" 2>/dev/null)" -ge 2 ] && break
  sleep 1
done
grep -c 'Managed nodes are active' "$OUT/nav.log"
win_move "RViz" 1000 0 920 1080
sleep 3
rec_start video
ros tests "python3 $SCRIPTS_CONT/nav_tests.py $MODE . $ARGS --ros-args -p use_sim_time:=true"
T0=$(date +%s); graph=0; last=0
declare -A seen
until [ -e "$OUT/marker_done" ]; do
  sleep 1
  el=$(( $(date +%s) - T0 ))
  for m in "$OUT"/marker_*; do
    [ -e "$m" ] || continue
    k=$(basename "$m"); [ -n "${seen[$k]:-}" ] && continue
    seen[$k]=1; [ "$k" = marker_done ] || shot "${k#marker_}"
  done
  if [ $((el - last)) -ge 15 ]; then shot "t$(printf '%03d' "$el")s"; last=$el; fi
  if [ "$graph" = 0 ] && [ "$el" -ge 20 ]; then
    graph=1
    ros_fg "python3 $SCRIPTS_CONT/ros_graph.py ros_graph_phase2 --skip nav_tests,rviz2" > "$OUT/graph.log" 2>&1 &
  fi
  [ "$el" -gt 1500 ] && { echo "tests did not finish" >&2; break; }
  kill -0 "$PID_tests" 2>/dev/null || [ -e "$OUT/marker_done" ] || { echo "nav_tests exited" >&2; break; }
done
shot final
rec_stop
ros_stop_all
ros_fg "chown -R $(id -u):$(id -g) ." || true
cat "$OUT/goals.md" "$OUT/kidnap_events.txt" 2>/dev/null

#!/usr/bin/env bash
# All Webots scenarios for the report, one after another (W1 must have produced its map).
#   run_all.sh [first scenario number, default 2]
set -o pipefail
D="$(cd "$(dirname "$0")" && pwd)"
W="${WS_HOST:-$HOME/ros2_jazzy_ws}/work/webots"
WC=/root/ros2_ws/work/webots
MAP="$WC/W1_mapping/map.yaml"
FROM="${1:-2}"
run() { echo "=== $1 $(date +%T)"; shift; "$@" || echo "!!! failed: $*"; }

[ "$FROM" -le 2 ] && run W2 "$D/run_w2_nav.sh" "$W/W2_nav_goals" "$MAP" goals \
  "1.8,-1.3 0.0,-6.0 4.8,-5.5 8.2,-6.2 8.2,0.4 0.0,0.0"
[ "$FROM" -le 3 ] && run W3 "$D/run_w2_nav.sh" "$W/W3_amcl_kidnap" "$MAP" kidnap "6.5 -6.0 120"
[ "$FROM" -le 4 ] && run W4 "$D/run_w1_mapping.sh" "$W/W4_hector_kidnap_large" --no-graph \
  --kidnap-at 60 abs -0.2 -3.0 180
[ "$FROM" -le 5 ] && run W5 "$D/run_w1_mapping.sh" "$W/W5_hector_kidnap_small" --no-graph \
  --kidnap-at 60 rel 0.2 0.1 10
[ "$FROM" -le 6 ] && run W6 "$D/run_w1_mapping.sh" "$W/W6_fast_turns" --no-graph --patrol-speed 3
if [ "$FROM" -le 7 ]; then
  # AMCL with random-particle injection on (recovery_alpha_*), the same kidnap as W3
  jazzy bash -c "sed -e 's/recovery_alpha_fast: 0.0/recovery_alpha_fast: 0.1/' \
    -e 's/recovery_alpha_slow: 0.0/recovery_alpha_slow: 0.001/' \
    \$(ros2 pkg prefix tb4_hector_slam)/share/tb4_hector_slam/config/nav2.yaml > /root/ros2_ws/work/nav2_recovery.yaml"
  run W7 "$D/run_w2_nav.sh" "$W/W7_amcl_kidnap_recovery_alphas" "$MAP" kidnap "6.5 -6.0 120" \
    /root/ros2_ws/work/nav2_recovery.yaml
fi
[ "$FROM" -le 8 ] && run W8 "$D/run_w7_explore.sh" "$W/W8_full_exploration"
echo "=== done $(date +%T)"

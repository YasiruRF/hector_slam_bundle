#!/usr/bin/env bash
# Copy run outputs from the work directory into report_content/ and build the figures.
#   collect.sh [WORK_DIR]   (default ~/ros2_jazzy_ws/work)
set -eo pipefail
WORK="${1:-$HOME/ros2_jazzy_ws/work}"
RC="$(cd "$(dirname "$0")/.." && pwd)"
S="$RC/scripts"
F="$RC/figures"
mkdir -p "$F" "$RC/webots" "$RC/real_robot/replays"

# Webots runs: everything except the big intermediate files
for d in "$WORK"/webots/W*; do
  [ -d "$d" ] || continue
  rsync -a --exclude '*.gv' "$d/" "$RC/webots/$(basename "$d")/"
done
# Real-robot replays: maps, trajectories, logs, parameters
for d in "$WORK"/replay/R*/; do d="${d%/}"
  rsync -a "$d/" "$RC/real_robot/replays/$(basename "$d")/"
done
cp "$WORK"/real_maps/snap_0*.{pgm,yaml} "$RC/real_robot/" 2>/dev/null || true
cp "$WORK/real_odom.csv" "$RC/real_robot/odom_from_bag.csv"

W="$RC/webots"; R="$RC/real_robot"
fig() { echo "fig: $1"; }

# --- Webots ------------------------------------------------------------
cp "$W/W1_mapping/00_start.png" "$F/fig01_webots_world_and_rviz.png"; fig 01
cp "$W/W1_mapping/eval.png" "$F/fig02_w1_map_vs_ground_truth.png"; fig 02
ls "$W"/W1_mapping/0[1-6]_t*.png >/dev/null 2>&1 && montage_src=$(ls "$W"/W1_mapping/0[2-5]_t*.png | head -4)
if [ -n "${montage_src:-}" ] && command -v montage >/dev/null; then
  montage $montage_src -tile 2x2 -geometry 960x540+4+4 "$F/fig03_w1_mapping_progress.png"; fig 03
fi
cp "$W/W1_mapping/99_final.png" "$F/fig04_w1_final_rviz.png"; fig 04
python3 "$S/plot_errors.py" hector "$F/fig05_w1_pose_error.png" "$W/W1_mapping/pose.csv:W1 patrol 0.25 m/s" >/dev/null; fig 05
grep -v -E 'run_logger|pose_logger' "$W/W1_mapping/ros_graph_phase1.dot" > "$F/fig06_ros_graph_phase1.dot"
dot -Tpng -Gdpi=130 "$F/fig06_ros_graph_phase1.dot" -o "$F/fig06_ros_graph_phase1.png"; rm "$F/fig06_ros_graph_phase1.dot"; fig 06
command -v pdftoppm >/dev/null && pdftoppm -png -r 80 -singlefile "$W/W1_mapping/frames_phase1.pdf" "$F/fig07_tf_tree_phase1" && fig 07

if [ -d "$W/W2_nav_goals" ]; then
  cp "$W/W2_nav_goals/ros_graph_phase2.png" "$F/fig08_ros_graph_phase2.png" 2>/dev/null && fig 08
  # the W2 rerun's RViz window was not placed (screens unusable); W3 shows Nav2 driving to a goal
  cp "$W/W3_amcl_kidnap/t015s.png" "$F/fig09_nav2_goal_rviz.png" && fig 09
fi
for k in W3_amcl_kidnap W7_amcl_kidnap_recovery_alphas; do
  [ -f "$W/$k/kidnap.csv" ] || continue
  n=$([ "$k" = W3_amcl_kidnap ] && echo 11 || echo 12)
  python3 "$S/plot_errors.py" amcl "$F/fig${n}_${k}_error.png" "$W/$k/kidnap.csv" >/dev/null; fig $n
  if command -v montage >/dev/null; then
    imgs=""; for m in before_kidnap lost global_spread global_end; do [ -f "$W/$k/$m.png" ] && imgs="$imgs $W/$k/$m.png"; done
    [ -n "$imgs" ] && montage $imgs -tile 2x2 -geometry 960x540+4+4 "$F/fig${n}b_${k}_screens.png"
  fi
done
specs=""; marks=""
for k in W4_hector_kidnap_large W5_hector_kidnap_small W6_fast_turns; do
  [ -f "$W/$k/pose.csv" ] && specs="$specs $W/$k/pose.csv:$k"
done
[ -n "$specs" ] && python3 "$S/plot_errors.py" hector "$F/fig13_hector_kidnap_fast_pose_error.png" \
  "$W/W1_mapping/pose.csv:W1 nominal" $specs >/dev/null && fig 13
for k in W4_hector_kidnap_large W5_hector_kidnap_small W6_fast_turns W8_full_exploration; do
  [ -f "$W/$k/eval.png" ] && cp "$W/$k/eval.png" "$F/fig14_${k}_eval.png"
done
for k in W4_hector_kidnap_large W5_hector_kidnap_small; do
  [ -f "$W/$k/kidnap_after_24s.png" ] && command -v montage >/dev/null && \
    montage "$W/$k/kidnap_before.png" "$W/$k/kidnap_after_24s.png" "$W/$k/99_final.png" \
      -tile 3x1 -geometry 960x540+4+4 "$F/fig15_${k}_screens.png"
done
[ -f "$W/W8_full_exploration/final.png" ] && cp "$W/W8_full_exploration/final.png" "$F/fig16_w8_exploration_final_rviz.png"

# --- Real robot ----------------------------------------------------------
python3 "$S/plot_map.py" "$F/fig20_real_live_map.png" "$R/map.yaml" \
  --traj "$RC/../real_robot/trajectory.csv:map_x:map_y:Hector pose (live)" \
  --title "Real TurtleBot 4, live Hector run (scan matching only)" >/dev/null; fig 20
(cd "$R" && python3 "$S/plot_map.py" "$F/fig21_real_map_growth.png" --grid snap_01.yaml snap_02.yaml snap_03.yaml snap_04.yaml snap_05.yaml map.yaml \
  --labels "t=35 s,t=73 s,t=109 s,t=147 s,t=183 s,final" --title "Real robot, live run: map over time" >/dev/null); fig 21
python3 "$S/analyze_real.py" "$RC/../real_robot/trajectory.csv" "$R/odom_from_bag.csv" "$F/fig22_real_heading_slips" \
  "$R/replays/R0b_live_config_repeat/trajectory.csv:replay R0b, live config" \
  "$R/replays/R4_odom_prior_fixed/trajectory.csv:replay R4, odometry prior" \
  "$R/replays/R5_odom_prior_fixed_3_levels/trajectory.csv:replay R5, odometry prior + 3 levels" >/dev/null; fig 22
cp "$F/fig22_real_heading_slips.txt" "$R/heading_slips.txt"; rm "$F/fig22_real_heading_slips.txt"
grid=""; labels=""
for spec in "R0_live_config:R0 live config" "R0b_live_config_repeat:R0b live config, repeat" \
            "R2_3_levels:R2 3 map levels" "R1_odom_prior:R1 odom prior (original port)" \
            "R4_odom_prior_fixed:R4 odom prior (fixed)" "R5_odom_prior_fixed_3_levels:R5 odom prior fixed + 3 levels"; do
  d="${spec%%:*}"; [ -f "$R/replays/$d/map.yaml" ] || continue
  grid="$grid $R/replays/$d/map.yaml"; labels="$labels,${spec#*:}"
done
python3 "$S/plot_map.py" "$F/fig23_real_replays.png" --grid $grid --labels "${labels#,}" \
  --title "Real-robot bag replayed into Hector with different settings" >/dev/null; fig 23
python3 "$S/plot_map.py" "$F/fig24_real_odom_prior_map.png" \
  "$R/replays/R5_odom_prior_fixed_3_levels/map.yaml" \
  --traj "$R/replays/R5_odom_prior_fixed_3_levels/trajectory.csv:map_x:map_y:Hector pose (odometry prior + 3 levels)" \
  --title "Real robot bag, Hector with wheel-odometry prior (R5)" >/dev/null; fig 24
cp "$RC/../real_robot/screenshot_165736.png" "$F/fig25_real_dashboard_screenshot.png" 2>/dev/null || true
ffmpeg -loglevel error -y -ss 150 -i "$RC/../real_robot/video_170103.webm" -frames:v 1 "$F/fig26_real_dashboard_video_frame.png" && fig 26
echo "figures in $F"

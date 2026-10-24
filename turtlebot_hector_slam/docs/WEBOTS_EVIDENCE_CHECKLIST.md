# Webots evidence checklist (report + presentation)

The real robot gave us data, but the evidence for the report comes from Webots
(`hector_slam_test.wbt`), where we have ground truth. Assignment sections this covers:
3.5 (screenshots, maps, video of successful runs, limitations), report "Application and
Performance Evaluation" (3 marks: SLAM + Nav2 integration, map quality, localisation
accuracy, critical discussion) and the presentation demo (1 mark).

Save everything under `webots/results/2026-10-25/<run>/` and use the same file names as the
runs B to E (`map.pgm/.yaml`, `pose.csv`, `eval.txt/.png`, logs).

## 0. Already done (just reuse)

- [x] Full-exploration scores, runs B to D: precision ~98 %, recall ~97–99 %, pose RMSE 1.0–1.4 cm (`webots/results/2026-10-24/README.md`)
- [x] Two-phase run E: map 98.9 / 100 / 97.6 %, 3/3 Nav2 goals, AMCL error 1.4–3.2 cm
- [x] Map/ground-truth overlays: `eval.png` in each run folder
- [x] Clock-offset failure and fix logs (run C)

What's missing is mostly visual: screenshots, videos, graphs, and the kidnap and failure cases.

## 1. Setup screenshots

- [ ] Webots world, top-down view of the whole arena (robot at the start pose)
- [ ] Webots close-up of the TurtleBot 4 with the lidar rays shown (View → Optional Rendering → Show Lidar Ray Paths)
- [ ] Terminal with the launch command running (shows it's reproducible)

## 2. Phase 1: Hector mapping

Run: `webots/run_sim_docker.sh`, then `native/run.sh map` (add `--patrol` to the bridge to drive automatically, or teleop slowly).

- [ ] **Video A (Webots + RViz side by side)**: map growing while the robot drives a full loop, 1–3 min
- [ ] RViz screenshot at about 25 %, 50 % and 100 % of the run (map building over time)
- [ ] RViz screenshot of the final map with the trajectory (`/trajectory` from hector_trajectory_server) and the scan on the walls
- [ ] `rqt_graph` screenshot (Nodes/Topics, all), phase 1: `/scan` → `hector_mapping` → `/map`, `/slam_out_pose`
- [ ] TF tree: `ros2 run rqt_tf_tree rqt_tf_tree` (or `ros2 run tf2_tools view_frames` → PDF) showing `map → odom → base_link → laser`
- [ ] `ros2 topic hz /scan` and `ros2 topic hz /map` (scan rate vs map update rate for the report)
- [ ] Saved map: `native/run.sh savemap webots_final`
- [ ] Scores: `pose_logger.py` running during the run → `evaluate_map.py map.yaml --pose-log pose.csv --plot eval.png`
- [ ] rosbag (30 s or more): `ros2 bag record -o phase1 /scan /odom /tf /tf_static /map /slam_out_pose /ground_truth/pose`

## 3. Phase 2: Nav2 + AMCL on the saved map

Run: stop phase 1, restart the sim (robot back at the start), then `native/run.sh nav webots_final`.

- [ ] RViz screenshot right after start: particle cloud spread, scan not quite on the walls yet
- [ ] RViz screenshot after AMCL converges: particle cloud tight, scan on the walls
- [ ] RViz screenshot of a goal: global path (planner), local costmap, global costmap inflation
- [ ] **Video B**: 3–5 Nav2 goals sent from RViz, including one behind an obstacle (has to plan around it); Webots and RViz both on screen
- [ ] `rqt_graph` screenshot, phase 2 (map_server, amcl, planner, controller, bt_navigator, collision_monitor)
- [ ] Goal table, as in run E: goal, result, ground-truth stop position, AMCL estimate, AMCL error. 5 goals this time

## 4. Kidnapped robot problem

The assignment doesn't name it, but it's the clearest way to show the "limitations" and
"localisation" points. Do it in both phases, because the two behave differently.
Kidnap in Webots: pause the sim, select the robot, change `translation`/`rotation` in the
scene tree (or Shift + drag it), then resume. Write down the old and new ground-truth poses.

### 4a. Kidnap during Hector mapping (expected: fails)

Hector has no odometry, no loop closure and no global relocalisation. It only matches each
scan against the map near the previous pose.

- [ ] Map about half the room, screenshot of the clean map
- [ ] Teleport the robot about 2 m and 90° away
- [ ] **Video C**: the moment of the kidnap; Hector "locks on" in the wrong place and starts drawing walls over the existing map
- [ ] Screenshot of the corrupted map (duplicated/rotated walls)
- [ ] `pose.csv` + `evaluate_map.py`: pose error jumps at the kidnap and stays wrong (plot the error over time)
- [ ] Small kidnap too (about 0.2 m, 10°): Hector probably recovers, because the scan matcher's search range still covers it. Shows where the limit is

### 4b. Kidnap during Nav2 + AMCL (expected: recovers with help)

- [ ] AMCL localised; teleport the robot to another part of the map
- [ ] Screenshot: scan no longer on the walls, AMCL pose still at the old spot
- [ ] Recovery 1, global relocalisation: `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`, then rotate the robot slowly / send a short goal until the cloud converges. Screenshots of the spread cloud and the converged cloud, time to converge
- [ ] Recovery 2, manual: **2D Pose Estimate** in RViz. Screenshot after
- [ ] Optional: set `recovery_alpha_slow: 0.001`, `recovery_alpha_fast: 0.1` in `config/nav2.yaml` (both are 0.0 now) and repeat, to show AMCL injecting random particles by itself. Note in the report which setting was used
- [ ] **Video D**: kidnap → lost → recovery → a goal reached afterwards
- [ ] Table: kidnap distance/angle, recovery method, time to recover, AMCL error after

## 5. Failure cases and limitations (for the critical discussion)

- [ ] Fast rotation in phase 1 (teleop turn above 1 rad/s): screenshot of the rotated/doubled map. Matches what we saw on the real robot (`pictures/` WhatsApp image), so the two can go side by side
- [ ] Featureless area: drive along a long, plain wall or into open space; screenshot of the drift along the wall
- [ ] Drift: distance between start and end of the trajectory after a loop, with no loop closure
- [ ] A Nav2 goal that fails (goal inside an obstacle, or next to a wall: "Failed to make progress"), screenshot of the log line
- [ ] Optional parameter comparison: `map_multi_res_levels` 2 vs 3, or `map_resolution` 0.05 vs 0.025, scores for both

## 6. For the report and slides

- [ ] Results table: all runs (B to E and the new ones), the same columns as `webots/results/2026-10-24/README.md`
- [ ] Figures: final map + overlay, trajectory error over time, kidnap before/after, AMCL convergence
- [ ] Each figure numbered and captioned (writing marks)
- [ ] Videos: 30–60 s cut for the slides (A + B + D), full versions linked or attached
- [ ] Real-robot map next to the Webots map, with one sentence on why they differ

## Recording tips

- Screen video: `ffmpeg -f x11grab -framerate 30 -i :0 -c:v libx264 -preset ultrafast out.mp4` (Ctrl-C stops it), or OBS / SimpleScreenRecorder
- Webots can also record by itself: File → Make Movie (only the 3D view)
- Screenshots: `gnome-screenshot -w` (current window) or the dashboard's Evidence panel
- Show the sim clock / RTF in the Webots window, so viewers can see the video isn't sped up

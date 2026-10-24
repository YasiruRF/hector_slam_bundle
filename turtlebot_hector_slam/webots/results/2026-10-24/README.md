# Webots runs, 2026-10-24

Full autonomous exploration (Hector SLAM + Nav2 + frontier explorer) of `hector_slam_test.wbt`,
scored against the ground truth with `../../tools/evaluate_map.py` (tolerance 0.10 m).
All three use the final `config/nav2.yaml` (commit `1703f2b` or later). Metric definitions and
the earlier baseline run are in `/metrics.md`.

| Run | Setup | Precision | Recall | Coverage | Wall err | Pose RMSE (max) | Heading mean (max) | Goals reached / sent | Path | Time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| [B](B_clock_synced/) | terminal launch, clocks synced | 98.1 % | 99.0 % | 97.3 % | 3.1 cm | 1.0 cm (4.3) | 0.76° (4.3) | 5 / 8 | 33.2 m | ~387 s |
| [C](C_clock_offset_-5s/) | terminal launch, robot stamps 5 s behind the laptop clock | 98.5 % | 98.2 % | 97.1 % | 2.7 cm | 1.1 cm (4.5) | 0.74° (2.0) | 4 / 8 | 25.7 m | ~351 s |
| [D](D_dashboard_real_mode_clock_1970/) | native dashboard, target "Real robot" (no sim time), robot stamps from 0 (a Pi clock at 1970) | 98.3 % | 97.7 % | 96.6 % | 2.7 cm | 1.4 cm (2.6) | 0.59° (3.3) | 5 / 8 | 28.2 m | ~336 s |

## Files per run
- `map.pgm`, `map.yaml`: the saved Hector map (`map_saver_cli`; D via the dashboard's Save map).
- `pose.csv`: `pose_logger.py` output, Hector `map → base_link` vs `/ground_truth/pose`, ~1 Hz.
- `eval.txt`, `eval.png`: evaluator output and the map/trajectory overlay on the ground truth.
- `launch.log` (B, C) / `log_slam.txt` (D): full SLAM + Nav2 + explorer output.
- D also: `trajectory.csv` (map and odom pose, 1 Hz), `summary.json` (map stats), `preflight.json` (connection checks, incl. the clock-offset warning).

## Clock-offset reproduction (C)
The first hardware test (2026-10-23) had working teleop but Nav2 never moved the robot. Run C
reproduces a robot whose clock disagrees with the laptop's (`run_sim_docker.sh --clock-offset -5`):
- `before_fix_1_old_config.log`: old `nav2.yaml`. `collision_monitor` "timestamps differ on 5.08 seconds. Ignoring the source" → "Robot to stop due to invalid source"; the robot never moved.
- `before_fix_2_collision_monitor_only.log`: collision monitor fixed (`source_timeout: 0.0`). The controller then failed with "Transform data too old when converting from map to odom" on every path; the robot still never moved and the explorer gave up.
- `launch.log`: plus local costmap `transform_tolerance`; full exploration, scores above.

## Notes
- Goals that time out are frontiers next to walls where DWB finds no valid trajectory ("Failed to make progress"; the robot stalls 50–90 s, the explorer blacklists the goal). Exploration completes in every run.
- D's `pose.csv` is cut at the end of the exploration run (591 samples). The logger kept running into a deliberate Nav2-restart test, which resets Hector's map frame.
- Time is from Start to "exploration complete" (wall clock; the sim runs at about real time).

## Two-phase run (E): map first, then Nav2 on the saved map

The workflow now recommended for the real robot (`docs/ASSIGNMENT_GUIDE.md`); Hector and Nav2
never run at the same time.
1. `native/run.sh map` (Hector only) while the bridge's `--patrol` drove the robot around the room, then
   `native/run.sh savemap webots_test`. Map score: precision 98.9 %, recall 100.0 %,
   coverage 97.6 %, mean wall error 2.6 cm (`eval.txt`, `eval.png`).
2. Hector stopped and the sim restarted, so the robot is back at the mapping start pose. Then
   `native/run.sh nav webots_test` (map_server + AMCL + Nav2, AMCL initial pose 0, 0, 0)
   and three `NavigateToPose` goals:

| Goal (map) | Result | Robot stopped at (ground truth) | AMCL estimate | AMCL error |
| --- | --- | --- | --- | --- |
| (2.0, 0.0) | SUCCEEDED | (1.799, 0.105) | (1.796, 0.127) | 2.3 cm |
| (4.5, -1.2) | SUCCEEDED | (4.299, -1.151) | (4.268, -1.148) | 3.2 cm |
| (0.5, 0.5) | SUCCEEDED | (0.695, 0.406) | (0.709, 0.409) | 1.4 cm |

The robot stops up to ~0.2 m short of each goal: that is the goal checker's 0.25 m
`xy_goal_tolerance`, not a localisation error.
Files: `map.*`, `eval.*`, `phase1_hector.log`, `phase2_nav.log`.

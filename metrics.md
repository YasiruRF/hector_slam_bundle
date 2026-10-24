# Metrics: what we measure, what we have, what to capture

For the ICTE4001 report ("Application Results and Performance Evaluation", 3 marks) and the
presentation ("Application Results and Performance Evaluation"). The rubric asks for a working
SLAM + Nav2 demonstration, map quality, localisation accuracy, and limitations discussed
critically, with maps, screenshots and video.

Last updated 2026-10-24.

> **Risk:** the brief says to apply SLAM to "the simulated robots given in our tutorials
> (ICTE4001Robot or virtual TurtleBot4), or the physical TurtleBot4 … otherwise, you will receive
> 0 mark for this part". Webots is our own TurtleBot 4 simulation, not the tutorial one. Get the
> physical-robot evidence in section 2, even if partial.

---

## 1. Simulation (Webots): done

Webots has ground truth, so these are measured, not estimated.

### Metric definitions
Computed by `turtlebot_hector_slam/webots/tools/evaluate_map.py` (map vs
`webots/maps/ground_truth.yaml`) and `pose_logger.py` (Hector pose vs `/ground_truth/pose`).

| Metric | Definition | Measures |
| --- | --- | --- |
| Wall precision | % of SLAM occupied cells within 0.10 m of a true wall/obstacle | phantom walls, smearing |
| Wall recall | % of true wall cells (visible from inside the room) within 0.10 m of a SLAM occupied cell | missing walls |
| Free coverage | % of true free cells inside the room that the SLAM map marks free | completeness of exploration |
| Mean wall error | mean distance from each SLAM occupied cell to the nearest true obstacle | geometric accuracy |
| Pose RMSE / max | Euclidean error between Hector's `map → base_link` and ground truth, sampled at 1 Hz | localisation accuracy |
| Heading error | absolute yaw difference, mean / max | localisation accuracy |
| Goals reached | explorer goals sent − timed out − failed (see 2.4) | Nav2 integration |
| Path length / time | distance driven (B, C: ground truth; D: Hector trajectory from the dashboard export); time from start to "exploration complete" | efficiency |

### Results

| Run | Precision | Recall | Coverage | Wall err | Pose RMSE (max) | Heading mean (max) | Goals reached / sent | Path | Time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A: baseline, 2026-10-22 | 98.3 % | 96.2 % | 94.2 % | 3.0 cm | 1.2 cm (6.6) | – | – | – | 890 s |
| B: clocks synced | 98.1 % | 99.0 % | 97.3 % | 3.1 cm | 1.0 cm (4.3) | 0.76° (4.3) | 5 / 8 | 33.2 m | ~387 s |
| C: robot clock −5 s | 98.5 % | 98.2 % | 97.1 % | 2.7 cm | 1.1 cm (4.5) | 0.74° (2.0) | 4 / 8 | 25.7 m | ~351 s |
| D: dashboard, "Real robot" mode, robot clock at 1970 | 98.3 % | 97.7 % | 96.6 % | 2.7 cm | 1.4 cm (2.6) | 0.59° (3.3) | 5 / 8 | 28.2 m | ~336 s |
| **Mean (min–max), A–D** | **98.3 % (98.1–98.5)** | **97.8 % (96.2–99.0)** | **96.3 % (94.2–97.3)** | **2.9 cm (2.7–3.1)** | **1.2 cm (1.0–1.4)** | | | | |

- Runs B–D use the final `nav2.yaml` (2026-10-24); A used the earlier one.
- Goals that time out are frontiers near walls that Nav2 cannot reach ("Failed to make progress",
  robot stalls 50–90 s, then the explorer blacklists the goal and moves on). Exploration still
  completes, so treat it as a known limitation of the frontier + DWB setup, not a SLAM error.
- Files for B–D (maps, pose logs, evaluator output, overlay figures, launch logs, and the two failing
  clock-offset logs): `turtlebot_hector_slam/webots/results/2026-10-24/` (see its README).
- Figures: `webots/results/2026-10-24/*/eval.png` (map and trajectory overlaid on ground truth),
  `webots/results/webots_eval.png` (run A), `docs/images/rviz_explored.png`,
  `docs/images/map_vs_ground_truth.png`, `docs/images/webots_world.png`, `docs/images/dashboard.png`.

### Reproduce
```bash
turtlebot_hector_slam/webots/run_sim_docker.sh [--clock-offset -5]      # Webots + bridge
jazzy ros2 launch tb4_hector_slam slam_nav_launch.py use_sim_time:=true explore:=true use_rviz:=false
jazzy python3 .../webots/tools/pose_logger.py pose.csv
jazzy ros2 run nav2_map_server map_saver_cli -f run --ros-args -p use_sim_time:=true   # after "exploration complete"
jazzy python3 .../webots/tools/evaluate_map.py run.yaml --pose-log pose.csv --plot run_eval.png
```

---

## 2. Physical TurtleBot 4: to capture

There is no ground truth, so these use a tape measure and the robot's own logs. Everything runs
from the dashboard (`native/run.sh`).

### 2.0 Before you start
- [ ] Put tape on the floor at the start position, with an arrow for the heading.
- [ ] Measure the room with a tape: length, width, one door opening, one more distance (e.g. a
      pillar or table leg to a wall). Write them in the table in 2.7.
- [ ] Phone ready to film the robot; screen recording for the dashboard (or its Video button).
- [ ] Dashboard: Run checks → all pass. Note the "Clock sync" value (it is informational).
- [ ] Press **Record rosbag** before driving. It lets the run be replayed and analysed later.

### 2.1 Run order
1. **Hand-driven mapping:** Start SLAM + Nav2 with exploration **off**, drive slowly
   (≤ 0.15 m/s, gentle turns) around the room. Watch `slam` Hz ≈ `scan` Hz in the status bar.
   **Save map**, then **Export** (zip). This is the minimum evidence that SLAM works on the robot.
2. **Clicked goals:** send 5 goals by click-dragging on the map. Count how many are reached.
3. **Autonomous exploration:** Stop, Start with exploration on, let it finish
   ("exploration complete"), then **Go to start**. Save map, Export.

Nothing in a session is overwritten: each Start writes a new trajectory (`trajectory.csv`,
`trajectory_2.csv`, …), and saved maps and rosbags are numbered the same way (`map`, `map_2`;
`rosbag`, `rosbag_2`). An automatic Nav2 restart also starts a new trajectory file. Use the one
that covers the run you are reporting. `log_slam.txt` in the session folder keeps every line
of the run (the dashboard's Logs panel only shows the tail).

### 2.2 Map quality: tape measure vs map
For each distance from 2.0, measure it on the saved map. Either:
- RViz: open the saved map, use the **Measure** tool between the two wall faces, or
- any image editor: pixel distance between the two walls × 0.05 m (map resolution).

Report error = map − tape (cm) and % of the tape value. Also note qualitatively: straight walls?
double walls (smearing)? phantom obstacles?

### 2.3 Localisation: start/end drift
After exploration, press **Go to start**. When the robot stops:
- **Physical error:** measure the robot centre's offset from the floor mark (cm) and its heading
  offset (degrees).
- **What Hector thinks:** its last `map_x, map_y, map_yaw` (Hector's map frame starts at the start pose).
- **What wheel odometry thinks:** the difference between the last and first `odom_*` values in the run's trajectory file.

Compare Hector's error with odometry's: SLAM correcting odometry drift is the point of the algorithm.

```python
# python3 drift.py trajectory_2.csv   (the run's trajectory file from the export zip)
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else 'trajectory.csv')))
f = lambda r, k: float(r[k])
s, e = rows[0], rows[-1]
wrap = lambda a: math.degrees(math.atan2(math.sin(a), math.cos(a)))
print('Hector end pose vs start: %.3f m, %.1f deg' % (
    math.hypot(f(e, 'map_x') - f(s, 'map_x'), f(e, 'map_y') - f(s, 'map_y')), wrap(f(e, 'map_yaw') - f(s, 'map_yaw'))))
print('Odometry end vs start:    %.3f m, %.1f deg' % (
    math.hypot(f(e, 'odom_x') - f(s, 'odom_x'), f(e, 'odom_y') - f(s, 'odom_y')), wrap(f(e, 'odom_yaw') - f(s, 'odom_yaw'))))
print('Path length (Hector): %.1f m over %.0f s' % (
    sum(math.hypot(f(b, 'map_x') - f(a, 'map_x'), f(b, 'map_y') - f(a, 'map_y')) for a, b in zip(rows, rows[1:])), f(e, 't_s')))
```
If the robot is physically back on the mark, the true displacement is 0. Each tool's printed
displacement is then its own error.

### 2.4 Navigation (Nav2) performance
From the export zip, `log_slam.txt`:
```bash
grep -c "Exploring towards" log_slam.txt   # goal attempts
grep -c "Goal rejected"     log_slam.txt   # attempts before Nav2 was active (subtract these)
grep -c "Goal timed out"    log_slam.txt   # given up after 60 s
grep -c "Goal not reached"  log_slam.txt   # Nav2 aborted
grep -c "Failed to make progress" log_slam.txt
grep "exploration complete" log_slam.txt   # did it finish?
```
Reached = attempts − rejected − timed out − not reached. Also record: exploration time, explored
area (`summary.json`, `explored_area_m2`), and any E-STOP or manual interventions (count them honestly).

### 2.5 Processing rate
From the dashboard status bar or `summary.json` → `last_status`: `scan_hz` and `slam_hz`.
`slam_hz ≈ scan_hz` (~7–10 Hz on the RPLIDAR A1) means Hector keeps up and processes every scan.
Optional: set `output_timing: true` in `config/hector_slam.yaml` to log milliseconds per scan.

### 2.6 Evidence checklist
- [ ] Map image(s): hand-driven and autonomous (`*.pgm` in the zip; screenshot of the dashboard map)
- [ ] Photo of the room from the robot's start for side-by-side comparison
- [ ] Video: robot exploring + dashboard/RViz map growing (30–60 s cut for the presentation)
- [ ] Export zips (map, trajectory, logs, summary.json) and the rosbag
- [ ] Tape measurements (2.2) and end drift (2.3) written down on the spot

### 2.7 Results table (fill in)

| Metric | Value | Notes |
| --- | --- | --- |
| Room length: tape / map / error | | |
| Room width: tape / map / error | | |
| Door width: tape / map / error | | |
| Extra distance: tape / map / error | | |
| End drift, physical (cm, deg) | | robot vs floor mark after Go to start |
| End drift, Hector (m, deg) | | from drift.py |
| End drift, odometry (m, deg) | | from drift.py |
| Clicked goals reached / sent | | |
| Exploration goals reached / sent | | 2.4 |
| Exploration completed? time | | |
| Explored area (m²) | | summary.json |
| scan Hz / slam Hz | | |
| Clock offset shown by checks (s) | | informational |
| Manual interventions / E-STOPs | | |

---

## 3. Limitations and insights (with evidence)

| Limitation / finding | Evidence | Why (theory) |
| --- | --- | --- |
| No loop closure | seam or offset when the robot returns along a loop; end drift (2.3) | Hector is scan-to-map matching with no pose graph; errors are never corrected retroactively |
| Sensitive to fast rotation | smeared walls if turned fast; Nav2 capped at 0.6 rad/s | the Gauss–Newton matcher starts from the previous pose with no odometry prior; large inter-scan motion leaves its convergence basin |
| Featureless corridors, glass | sliding along a corridor, holes at glass | the scan-matching Hessian is degenerate along a corridor; glass returns no lidar points |
| Nav2 stalls near walls | "Failed to make progress", 3–4 timed-out goals per run (table 1) | frontiers right at walls + inflation radius; DWB cannot find a valid trajectory |
| Robot/laptop clock offset blocked Nav2 on hardware (found and fixed) | first hardware test; reproduced in Webots with `--clock-offset -5`: robot never moved, then explored fully after the fix (run C; logs in `webots/results/2026-10-24/C_clock_offset_-5s/`) | Jazzy collision monitor drops "stale" scans and the controller rejects mismatched TF stamps; robot stamps use the Pi's clock |
| Fast DDS startup race | Nav2 never activated once (`failed to send response to /controller_server/change_state`); dashboard now restarts it | lifecycle service reply lost during discovery |
| Gazebo GPU lidar returns constant 0.16 m | `gazebo/` notes; software render works but runs 0.15–0.2× real time | rendering-based lidar on the GPU path in this setup |

These map onto the rubric's "critically reflective, discussing strengths, weaknesses, and
practical implications". The clock-offset story shows the full engineering loop: symptom on
hardware → hypothesis from Nav2 source → reproduction in sim → fix → before/after measurement.

---

## 4. What goes where

| Item | Report | Presentation |
| --- | --- | --- |
| Simulation results table (section 1) + map overlay figure | yes, with metric definitions | one slide, the mean row + overlay figure |
| Real-robot table (2.7) + map + room photo | yes | one slide + short video |
| Pose error over time plot (from `pose*.csv`) | yes (figure) | optional |
| Limitations table (section 3) | yes, discussed in prose | 1 slide, 3–4 points |
| Clock-offset debugging story | short paragraph in "Insights" | good 1-slide story for Q&A |
| Dashboard screenshot | in the reproduction / tooling paragraph | demo or one screenshot |

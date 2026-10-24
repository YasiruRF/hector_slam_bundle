# Assignment guide: real TurtleBot 4 run + what to hand in

Report due **25 Oct, 23:59**. This page is the short path: run the robot in two
phases, collect the evidence, then write the report against the rubric.
Details and background: [`REAL_ROBOT.md`](REAL_ROBOT.md), [`../../metrics.md`](../../metrics.md).

## Why two phases

Running Hector and Nav2 in one launch gave an empty map on the real robot,
while Hector on its own mapped fine. So, as the lecturer suggested:

| Phase | Runs | Who provides `/map` | Who provides `map → odom` |
| --- | --- | --- | --- |
| 1. Mapping | Hector SLAM + RViz, you drive by hand | Hector | Hector |
| 2. Autonomous | Nav2 + map_server + AMCL on the saved map | map_server | AMCL |

Never run both at once: they would fight over `/map` and `map → odom`.
This is still "SLAM + Nav2 integration": Nav2 navigates on the map Hector built.

## Fastest: the dashboard (`./native/run.sh`, then http://localhost:8080)

1. **2 · Run → 1 · Map**, then **Quick start**. It runs the checks, undocks, and starts Hector only.
2. Drive with the Drive pad at a slow speed (the slider sets the turn rate too). Come back to the start, then click **Save map**.
3. **Stop**, then **Dock** (or drive back to the start spot).
4. **2 · Run → 2 · Navigate**. The map you just saved is pre-selected. Then **Quick start**. It undocks to the same spot and starts Nav2 + AMCL.
5. If the red scan dots don't sit on the walls, use **Set pose** (click and drag on the map). Then use **Send goal**.
6. Evidence panel: Screenshot / Record video / Record rosbag / Export results.

The terminal way below does the same thing.

## 0. Every terminal on the laptop

Use plain terminals, not the dashboard (the dashboard's ROS settings were the
other suspect in the failed run).

```bash
source /opt/ros/jazzy/setup.bash
source ~/tb4_ws/install/setup.bash
export ROS_DOMAIN_ID=0            # same as the robot
cd ~/yasiru/hector_slam_bundle    # wherever the repo is on the laptop
git pull                          # get this guide + nav_on_map_launch.py
SKIP_APT=1 ./native/setup.sh      # rebuilds into ~/tb4_ws (once, after the pull)
source ~/tb4_ws/install/setup.bash
```

Never run `colcon build` inside a package folder: that makes a second,
incomplete workspace (`package 'hector_mapping' not found`).

## 1. Phase 1: build the map

Put the robot **on the dock** (or mark its start spot on the floor with tape),
then undock it. Its start pose becomes the map origin.

```bash
# terminal 1
./native/run.sh map            # = ros2 launch tb4_hector_slam hector_slam_launch.py
# terminal 2: drive SLOWLY
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
```

If the robot does not move, add `-p stamped:=true`.

**How to drive (this is what decides the map quality):**
- Turn slowly: 0.3 rad/s or less. Stop for a second after each turn.
  Hector uses only the lidar, with no wheel odometry. A fast turn makes it lose track, and the map
  then shows the room drawn several times, rotated (doubled, crossing walls).
- Keep walls and furniture in view. Avoid pointing the lidar at big empty
  space or glass.
- Drive one loop around the room and finish back at the start.
- If the map breaks (walls double), stop with Ctrl-C and start again from the dock.
  Don't keep driving on a broken map.

When the map looks clean, save it **while phase 1 is still running**:

```bash
# terminal 3
./native/run.sh savemap lab_map     # -> ~/tb4_data/maps/lab_map.pgm + .yaml
```

Take an RViz screenshot of the map and trajectory now (evidence, see section 3).
Then Ctrl-C terminal 1 and terminal 2.

## 2. Phase 2: autonomous navigation on the saved map

Put the robot back at the **same start spot** (the dock, or the tape mark). Then:

```bash
./native/run.sh nav lab_map         # Nav2 + map_server + AMCL + RViz
```

1. In RViz check that the laser scan lines up with the walls of the map.
   If not, click **2D Pose Estimate** and drag an arrow where the robot really is.
2. Give AMCL a moment: the scan should settle onto the map's walls. If it stays
   off, set the pose again with **2D Pose Estimate**.
3. Click **Nav2 Goal** on a free spot about 1 m away. Then try one 3–4 m away and one
   behind an obstacle, so it has to plan around it.
4. Watch the robot the whole time. Emergency stop: Ctrl-C, or lift it.

Do not run teleop while Nav2 is driving.

## 3. Evidence to collect (for "Application & Performance", 3 + 1 marks)

| What | How |
| --- | --- |
| Phase 1 map | `~/tb4_data/maps/lab_map.pgm/.yaml` + RViz screenshot |
| Map accuracy | Tape-measure 3+ distances (room length/width, a doorway). Compare with the map in RViz (**Measure** tool, or pixels × 0.05 m). Put them in a table: real, map, error. |
| Drift | Start and end at the dock in phase 1: how far apart are the start and end of the trajectory in the map? |
| Localisation accuracy | Phase 2: put tape crosses on the floor, send Nav2 goals to them, measure how far the robot stops from each cross. 3–5 goals. |
| Nav2 working | Phone video of the robot driving to goals + RViz screen recording (path, costmaps) |
| Architecture | Run `rqt_graph` during each phase and screenshot it (nodes and topics) |
| Data | `ros2 bag record -o phase1 /scan /odom /tf /tf_static /map /slam_out_pose` (30 s or more) |
| Failure case | Keep a screenshot of a broken map (like the doubled-room one) and explain it in limitations |

The simulation results are already done (Webots, ground truth): precision ~98 %,
recall ~97 %, pose RMSE ~1–1.4 cm. See `../../metrics.md` and `../webots/results/`.
This exact two-phase workflow was tested in Webots (run E in
`../webots/results/2026-10-24/`): map 98.9 % precision / 100 % recall, and 3/3 Nav2 goals
reached with AMCL within 1.4–3.2 cm of ground truth.

## 4. Report checklist (15 marks), mapped to the rubric

- [ ] **Title** with "Hector SLAM", group number, names and IDs
- [ ] **Abstract** (150–200 words): purpose, theory, implementation, results
- [ ] **Introduction (1)**: role of SLAM. Hector = 2D, occupancy grid, scan-matching
      (Gauss–Newton on a smoothed grid map), no particle filter, no loop closure, no odometry needed
- [ ] **Theory (4)**: define every variable.
  - Map: log-odds occupancy grid with bilinear interpolation, and its gradient.
  - Scan matching: minimise Σ(1 − M(Sᵢ(ξ)))² over the pose ξ = (x, y, θ) with Gauss–Newton.
  - Multi-resolution map pyramid (coarse to fine).
  - Covariance estimate. Map update with update_factor_free / occupied.
  - Workflow diagram with the feedback loop: scan → transform → match on coarse → fine map → pose → map update → back to the matcher.
  - Cite Kohlbrecher et al., "A Flexible and Scalable SLAM System with Full 3D Motion Estimation", SSRR 2011 (IEEE).
- [ ] **Implementation (4)**: architecture diagram of the nodes.
  - `hector_mapping` (`HectorMappingRos` → `HectorSlamProcessor` → `ScanMatcher`, `MapRepMultiMap`, `OccGridMapUtil`).
  - Topics: in `/scan`, `/tf`; out `/map`, `/slam_out_pose`, `map → odom`.
  - Pseudocode for the scan matcher and the map update.
  - Parameter table from `tb4_hector_slam/config/hector_slam.yaml`.
  - Port fixes from [`hector_ros2_port_fixes.md`](hector_ros2_port_fixes.md).
  - Dependencies: hector_mapping, Nav2 (planner, controller, bt_navigator, AMCL, map_server), Eigen.
- [ ] **Application & evaluation (3)**: the two-phase SLAM → Nav2 pipeline above.
  - Webots numbers, then real-robot map, measurements and goal errors.
  - Critical discussion: rotation slips without odometry, no loop closure → drift,
    lidar rate, why the combined launch failed and the phased setup was used.
- [ ] **Writing (2)**: IEEE references, numbered figures and tables

Presentation (5 marks): the same structure in 15–25 slides. Show the real-robot video.

## 5. Quick troubleshooting

| Symptom | Fix |
| --- | --- |
| `package 'hector_mapping' not found` | wrong workspace sourced: `source ~/tb4_ws/install/setup.bash` |
| Map empty, Hector says `lookupTransform ... timed out` | robot TF not arriving: check `ROS_DOMAIN_ID`, `ros2 topic hz /tf` |
| Map doubles or rotates | turning too fast. Restart phase 1 and drive slower. If it still happens, try `map_multi_res_levels: 3` in `hector_slam.yaml` |
| Phase 2: robot in the wrong place in RViz | **2D Pose Estimate** |
| `./native/run.sh nav` says Hector is still running | Ctrl-C phase 1 first |
| Nav2 goal accepted but robot doesn't move | `ros2 topic echo /cmd_vel_unstamped`; see REAL_ROBOT.md 5.6 |

# turtlebot-hector-slam-jazzy

Bundle for running Hector SLAM on a TurtleBot 4 under **ROS 2 Jazzy**, across three targets:

1. Webots
2. Gazebo
3. The physical TurtleBot 4 (runs Jazzy, not Humble)

## Layout

- `hector_slam/` – Hector SLAM packages (ported from the ROS 2 Humble bag)
- `turtlebot_hector_slam/` – TurtleBot 4 integration, Webots world, docs and handover notes

## Environment

Everything is built and run inside the Jazzy docker container (`jazzy` on the host drops you into it).

## Dashboard (container + web UI)

For running on another computer there is a self-contained image with everything built (Jazzy, patched Hector SLAM, Nav2, this package) and a browser dashboard. The other computer only needs Docker.

```bash
docker/build_export.sh     # on the build machine -> dist/tb4-hector-slam.tar.gz + dist/run.sh
./run.sh                   # on the other computer, then open http://localhost:8080
```

The dashboard runs a connection-check pipeline against the robot (network, ROS 2 discovery, `/scan`, `/odom`, TF, velocity interface, competing nodes, dock/battery) and keeps Start locked until it passes. It is the single control hub: pick Real robot, Gazebo or Webots (the simulators run inside the container on their own ROS domain), then Quick start runs everything needed and Stop all stops it. It can also undock/dock the robot and switch the lidar motor. It then starts SLAM + Nav2 (optionally the explorer), shows the live map, drives the robot, sends goals, has an E-STOP, and saves the evidence the case study asks for (map, trajectory, screenshots, video, rosbag, zipped results with map statistics). See `docker/README.md`.

![dashboard](turtlebot_hector_slam/docs/images/dashboard.png)

## Run natively (no Docker)

On a computer with ROS 2 Jazzy installed in `/opt/ros/jazzy` (the laptop that drives the real robot):

```bash
native/setup.sh            # once: apt packages + build (into ~/tb4_ws, or the workspace this repo sits in)
native/run.sh              # dashboard, http://localhost:8080
native/run.sh preflight    # robot checks in the terminal
native/run.sh launch explore:=true use_rviz:=true   # SLAM + Nav2 without the dashboard
```

In the `jazzy` dev container the same scripts work: `jazzy ./src/turtlebot-hector-slam-jazzy/native/run.sh launch`.
Real-robot notes, including the clock-sync requirement: `turtlebot_hector_slam/docs/REAL_ROBOT.md`.

## Run in Webots

Webots (snap) runs on the host, the ROS 2 bridge, SLAM and Nav2 run in the `jazzy` container. Run from inside `~/ros2_jazzy_ws` (the directory the container mounts). Headless mode does not work with the snap (Qt xcb fails under xvfb), so use a real display.

```bash
turtlebot_hector_slam/webots/run_sim_docker.sh            # terminal 1: Webots + bridge
jazzy ros2 launch tb4_hector_slam slam_nav_launch.py \
    use_sim_time:=true explore:=true use_rviz:=false      # terminal 2: SLAM + Nav2 + explorer
jazzy python3 .../webots/tools/pose_logger.py pose.csv    # terminal 3 (optional)
jazzy ros2 run nav2_map_server map_saver_cli -f run       # when the explorer reports it is done
jazzy python3 .../webots/tools/evaluate_map.py run.yaml --pose-log pose.csv
```

Result of a full autonomous exploration run (`turtlebot_hector_slam/webots/results/`): wall precision 98.3 %, wall recall 96.2 %, free coverage 94.2 %, mean wall error 3.0 cm, pose RMSE 1.2 cm (max 6.6 cm) over 890 s of sim time.

## Status

- [x] Webots (SLAM + Nav2 + frontier exploration, scored against ground truth)
- [~] Gazebo: the TB4 maze sim runs headless in the container (`turtlebot_hector_slam/gazebo/run_gz_headless.sh`) and the SLAM + Nav2 + explorer stack drives it, but without a GPU the lidar renders on the CPU at about 0.15x real time and falling, so mapping is impractical. A first run gave a partly corrupted map (`gazebo/maze_partial_map.png`). Not scored. With a GPU the sim is fast enough, but the dashboard-started Gazebo run still gave an empty Hector map (see Known issues in `docker/README.md`).
- [~] Real TurtleBot 4: first test 2026-10-23, teleop/undock/lidar worked, Nav2 goals and exploration did not (most likely the collision monitor stopping the robot; fix and clock check in `turtlebot_hector_slam/docs/REAL_ROBOT.md` 5.6, not yet re-tested)

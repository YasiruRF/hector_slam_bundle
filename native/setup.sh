#!/usr/bin/env bash
# One-time setup on a computer with ROS 2 Jazzy installed in /opt/ros/jazzy (no Docker).
#   ./native/setup.sh              install the apt packages (sudo) and build
#   SKIP_APT=1 ./native/setup.sh   only rebuild (after git pull)
# Builds Hector SLAM + tb4_hector_slam into the colcon workspace this repo sits in
# (<ws>/src/<repo>), otherwise into ~/tb4_ws (override with TB4_WS=...).
set -eo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
if [ -n "${TB4_WS:-}" ]; then WS="$TB4_WS"
elif [ "$(basename "$(dirname "$REPO")")" = src ]; then WS="$(dirname "$(dirname "$REPO")")"
else WS="$HOME/tb4_ws"; fi
[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found in /opt/ros/jazzy" >&2; exit 1; }

SUDO=sudo; [ "$(id -u)" = 0 ] && SUDO=
if [ "${SKIP_APT:-0}" != 1 ]; then
  $SUDO apt-get update
  $SUDO apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot4-msgs ros-jazzy-turtlebot4-description ros-jazzy-irobot-create-msgs \
    ros-jazzy-laser-geometry ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-teleop-twist-keyboard \
    python3-colcon-common-extensions python3-aiohttp python3-numpy libeigen3-dev iputils-ping
fi

source /opt/ros/jazzy/setup.bash
mkdir -p "$WS"
cd "$WS"
# Only the packages the robot run needs (skips hector_geotiff and the other extras).
colcon build --symlink-install \
  --base-paths "$REPO/hector_slam" "$REPO/turtlebot_hector_slam/tb4_hector_slam" \
  --packages-up-to tb4_hector_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo
echo "Built into $WS. Start with:  $REPO/native/run.sh"

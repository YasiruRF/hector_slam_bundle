# Helpers for scripted Webots runs on a private Xvfb display (source this file).
# Webots (snap) runs on the host, ROS 2 in the `jazzy` container, as in
# turtlebot_hector_slam/webots/run_sim_docker.sh. Screenshots and videos come
# from the Xvfb display, so nothing needs the desktop session.
#   OUT=<dir> source sim_lib.sh
set -eo pipefail
WS_HOST="${WS_HOST:-$HOME/ros2_jazzy_ws}"
REPO_HOST="$WS_HOST/src/turtlebot-hector-slam-jazzy"
REPO_CONT=/root/ros2_ws/src/turtlebot-hector-slam-jazzy
SCRIPTS_HOST="$REPO_HOST/report_content/scripts"
SCRIPTS_CONT="$REPO_CONT/report_content/scripts"
XDISP="${XDISP:-:99}"
: "${OUT:?set OUT to a directory under $WS_HOST}"
OUT_CONT="/root/ros2_ws/${OUT#"$WS_HOST"/}"
mkdir -p "$OUT"

xvfb_up() {
  [ -e "/tmp/.X11-unix/X${XDISP#:}" ] && return
  Xvfb "$XDISP" -screen 0 1920x1080x24 +extension GLX -ac -nolisten tcp >/dev/null 2>&1 &
  sleep 2
}

# sim_start [bridge args]: Webots + bridge in the background, log in $OUT/sim.log
sim_start() {
  xvfb_up
  (cd "$REPO_HOST/turtlebot_hector_slam/webots" &&
   exec env -u XAUTHORITY -u WAYLAND_DISPLAY DISPLAY="$XDISP" QT_QPA_PLATFORM=xcb \
     ./run_sim_docker.sh "$@") > "$OUT/sim.log" 2>&1 &
  SIM_PID=$!
  wait_for "$OUT/sim.log" 'extern controller: connected|tb4_webots_bridge' 90 || true
  sleep 3
  win_move "Webots R2025a" 0 0 1000 1080
}

sim_stop() {
  kill "$SIM_PID" 2>/dev/null || true
  wait "$SIM_PID" 2>/dev/null || true
  jazzy pkill -INT -f tb4_ros2_bridge.py >/dev/null 2>&1 || true
  pgrep -x webots-bin >/dev/null && pkill -x webots-bin || true
  sleep 2
}

# ros NAME CMD...: run CMD in the container in the background, log $OUT/NAME.log
ros() {
  local name="$1"; shift
  jazzy bash -c "cd $OUT_CONT && export DISPLAY=$XDISP && $*" > "$OUT/$name.log" 2>&1 &
  eval "PID_$name=$!"
}

# ros_fg CMD...: run CMD in the container and wait
ros_fg() { jazzy bash -c "cd $OUT_CONT && export DISPLAY=$XDISP && $*"; }

ros_stop_all() {
  # SIGINT to everything we launched in the container (loggers flush on SIGINT)
  # (the [x] in each pattern keeps pkill from matching this bash -c command line)
  jazzy bash -c 'pkill -INT -f run_logge[r].py; pkill -INT -f pose_logge[r].py; pkill -INT -f nav_test[s].py; sleep 2;
                 pkill -INT -f "ros2 launc[h]"; sleep 6;
                 pkill -9 -f "ros2 launc[h]|/opt/ros/jazzy/li[b]/|/root/ros2_ws/instal[l]/|rviz[2]"; true' >/dev/null 2>&1 || true
  sleep 2
}

# wait_for FILE REGEX TIMEOUT_S
wait_for() {
  local i=0
  until grep -Eq "$2" "$1" 2>/dev/null; do
    i=$((i + 1)); [ "$i" -gt "$3" ] && { echo "timeout waiting for '$2' in $1" >&2; return 1; }
    sleep 1
  done
}

shot() { import -display "$XDISP" -window root "$OUT/$1.png"; echo "shot $1"; }

win_move() {
  DISPLAY="$XDISP" PYTHONPATH="${XLIB_PATH:-}" python3 "$SCRIPTS_HOST/xwin.py" move "$@" || true
}

rec_start() {
  ffmpeg -loglevel error -y -f x11grab -framerate 10 -video_size 1920x1080 -i "$XDISP" \
    -vf scale=1280:-2 -c:v libx264 -preset ultrafast -crf 28 -pix_fmt yuv420p "$OUT/$1.mp4" &
  REC_PID=$!
}
rec_stop() { kill -INT "$REC_PID" 2>/dev/null || true; wait "$REC_PID" 2>/dev/null || true; }

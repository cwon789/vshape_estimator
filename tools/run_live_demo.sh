#!/bin/bash
set -e
vshape_package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
vshape_workspace_dir=$(cd -- "$vshape_package_dir/../.." && pwd)
source /opt/ros/foxy/setup.bash
if [ -f "$vshape_workspace_dir/install/setup.bash" ]; then
  source "$vshape_workspace_dir/install/setup.bash"
fi
source "$vshape_workspace_dir/install_vshape/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOG_DIR=/tmp/vshape-live-demo/logs
export XDG_CACHE_HOME=/tmp/vshape-live-demo/cache
export QT_X11_NO_MITSHM=1
export SIMPLE_SIM_NO_GPU=1
mkdir -p /tmp/vshape-live-demo/logs
# Keep one demo owner. The tracked estimator PID is the actual C++ process,
# so cleanup cannot leave an orphaned child of ros2 launch publishing old poses.
exec 9>/tmp/vshape-live-demo/session.lock
if ! flock -n 9; then
  echo "V-shape demo is already running; close its window before starting another." >&2
  exit 1
fi
printf '%s\n' "$$" > /tmp/vshape-live-demo/launcher.pid
"$vshape_workspace_dir/install_vshape/ros2_bot_estimator_lidar_vshape/lib/ros2_bot_estimator_lidar_vshape/vshape_node" \
  --ros-args -r __node:=vshape_estimator --params-file "$vshape_package_dir/config/vshape.yaml" \
  -p use_sim_time:=true > /tmp/vshape-live-demo/estimator.log 2>&1 &
vshape_estimator_pid=$!
vshape_gui_pid=
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$vshape_gui_pid" ]; then kill -TERM "$vshape_gui_pid" 2>/dev/null || true; fi
  kill -INT "$vshape_estimator_pid" 2>/dev/null || true
  for vshape_wait_step in {1..80}; do
    if ! kill -0 "$vshape_estimator_pid" 2>/dev/null &&
       { [ -z "$vshape_gui_pid" ] || ! kill -0 "$vshape_gui_pid" 2>/dev/null; }; then break; fi
    sleep 0.1
  done
  if [ -n "$vshape_gui_pid" ]; then
    kill -KILL "$vshape_gui_pid" 2>/dev/null || true
    wait "$vshape_gui_pid" 2>/dev/null || true
  fi
  kill -KILL "$vshape_estimator_pid" 2>/dev/null || true
  wait "$vshape_estimator_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$vshape_package_dir/tools/show_live_demo.py" &
vshape_gui_pid=$!
wait "$vshape_gui_pid"

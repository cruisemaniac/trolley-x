#!/usr/bin/env bash
# Pull the latest code and rebuild the ROS workspace on the Pi.
set -euo pipefail
REPO="$HOME/dissertation-trolley-x"
cd "$REPO"
git pull --ff-only
git submodule update --init --recursive
cd "$REPO/ros2_ws"
colcon build --symlink-install
echo "Done. Run: source $REPO/ros2_ws/install/setup.bash"

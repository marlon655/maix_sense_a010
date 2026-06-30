#!/usr/bin/env bash
set -eo pipefail

DEVICE="${1:-/dev/tof}"

cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args -p "device:=${DEVICE}"

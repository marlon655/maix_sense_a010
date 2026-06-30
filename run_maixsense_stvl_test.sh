#!/usr/bin/env bash
set -eo pipefail
DEVICE="${1:-/dev/tof}"
cd "$(dirname "$0")"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch tof_stvl_test stvl_maixsense_test.launch.py device:="$DEVICE"

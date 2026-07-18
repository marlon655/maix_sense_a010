#!/usr/bin/env bash

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    source "$setup_file"
  fi
}

source /opt/ros/jazzy/setup.bash
source_if_exists /home/marlon/sim_ws/nav_hub/install/setup.bash
source_if_exists /home/marlon/sim_ws/sim_bot/install/setup.bash
source_if_exists /home/marlon/sim_ws/maix_sense_a010/install/setup.bash

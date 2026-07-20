#!/usr/bin/env bash

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    source "$setup_file"
  fi
}

TOF_SIM_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source_if_exists "${TOF_SIM_WS_ROOT}/nav_hub/install/setup.bash"
source_if_exists "${TOF_SIM_WS_ROOT}/sim_bot/install/setup.bash"
source_if_exists "${TOF_SIM_WS_ROOT}/maix_sense_a010/install/setup.bash"

unset TOF_SIM_WS_ROOT

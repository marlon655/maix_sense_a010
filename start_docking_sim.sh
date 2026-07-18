#!/usr/bin/env bash
set -euo pipefail

SIM_WS="${SIM_WS:-$HOME/sim_ws}"
LOG_DIR="${LOG_DIR:-/tmp/docking_sim_logs}"

mkdir -p "$LOG_DIR"

set +u
source "$SIM_WS/setup_sim.bash"
set -u

# Avoid stale FastDDS shared-memory locks after interrupted Gazebo/RViz runs.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

pids=()
pgids=()

cleanup() {
  echo
  echo "Encerrando processos da simulacao..."
  for pgid in "${pgids[@]}"; do
    kill -- "-$pgid" 2>/dev/null || true
  done
  sleep 2
  for pgid in "${pgids[@]}"; do
    kill -9 -- "-$pgid" 2>/dev/null || true
  done

  pkill -u "$USER" -f "sim_bot/.*/worlds/aceleradora.world" 2>/dev/null || true
  pkill -u "$USER" -f "rviz2 .*sim_bot/.*/rviz/bot.rviz" 2>/dev/null || true
  pkill -u "$USER" -f "parameter_bridge .*sim_bot/.*/config/gz_bridge.yaml" 2>/dev/null || true
  pkill -u "$USER" -f "sim_bot/.*/lib/sim_bot/graph_visualizer" 2>/dev/null || true
  pkill -u "$USER" -f "ros2 launch sim_bot sim_manager.launch.py" 2>/dev/null || true
  pkill -u "$USER" -f "ros2 launch nav_hub sim_main_route_graph.launch.py" 2>/dev/null || true

  wait "${pids[@]}" 2>/dev/null || true
}

start_process() {
  local name="$1"
  shift

  echo "Iniciando $name..."
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  pids+=("$!")
  pgids+=("$!")
  echo "  pid=${pids[-1]} log=$LOG_DIR/$name.log"
}

trap cleanup EXIT INT TERM

start_process sim_manager ros2 launch sim_bot sim_manager.launch.py
sleep 3

start_process route_graph ros2 launch nav_hub sim_main_route_graph.launch.py
sleep 1

start_process graph_visualizer ros2 run sim_bot graph_visualizer

echo
echo "Simulacao rodando. Logs em: $LOG_DIR"
echo "Envie destinos manualmente, por exemplo:"
echo '  ros2 topic pub --once /destination std_msgs/msg/Int32 "{data: 99}" --qos-reliability reliable --qos-durability transient_local'
echo "Use Ctrl+C para encerrar tudo."

wait

cmake_minimum_required(VERSION 3.8)
project(sipeed_tof_ms_a010)

# Garante uso de C++17 (padrão no ROS 2 Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Ativa warnings úteis durante a compilação
add_compile_options(-Wall -Wextra -Wpedantic)

# =========================
# Dependências ROS 2
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV é usado diretamente (cv::Mat)
find_package(OpenCV REQUIRED)

# =========================
# Diretórios de include
# =========================
# Como seus headers estão em src/, precisamos expor esse caminho
include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
)

# =========================
# Executável principal do nó
# =========================
add_executable(sipeed_tof_node
  src/main.cc
  src/serial.cc
  src/frame_handle.cc
  src/cJSON.c
)

# =========================
# Dependências ROS linkadas ao alvo
# =========================
ament_target_dependencies(
  sipeed_tof_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# =========================
# Linkagem com OpenCV
# =========================
target_link_libraries(
  sipeed_tof_node
  ${OpenCV_LIBRARIES}
)

target_compile_options(sipeed_tof_node PRIVATE -Wno-pedantic)


# =========================
# Instalação do executável
# =========================
install(TARGETS
  sipeed_tof_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Finalização do pacote
# =========================
ament_package()

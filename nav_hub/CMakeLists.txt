cmake_minimum_required(VERSION 3.8)
project(nav_hub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Include directories
include_directories(include)

add_library(nav_hub_lib src/log_manager.cpp)
ament_target_dependencies(nav_hub_lib rclcpp)
target_include_directories(nav_hub_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add executable
add_executable(tf_listener src/tf_listener.cpp)
add_executable(botoeira_ponto_ponto_linear src/botoeira_ponto_linear.cpp src/log_manager.cpp)
add_executable(main_route_graph src/main_route_graph.cpp src/log_manager.cpp src/costmap_cleaner.cpp)
add_executable(sim_main_route_graph src/sim_main_route_graph.cpp)
add_executable(stop_obstacle src/stop_obstacle.cpp)
# Target dependencies

ament_target_dependencies(tf_listener
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(botoeira_ponto_ponto_linear
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(main_route_graph
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(sim_main_route_graph
  rclcpp
  geometry_msgs
  std_msgs
  action_msgs
)

ament_target_dependencies(stop_obstacle
  rclcpp
  geometry_msgs
  tf2
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  tf_listener
  botoeira_ponto_ponto_linear
  main_route_graph
  sim_main_route_graph
  stop_obstacle
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyright
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch maps route include/ graphs btree
  DESTINATION share/${PROJECT_NAME}
)

add_compile_options(-g)

install(
  DIRECTORY include/nav_hub
  DESTINATION include
)

ament_export_targets(nav_hub_lib_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

install(TARGETS nav_hub_lib
  EXPORT nav_hub_lib_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()

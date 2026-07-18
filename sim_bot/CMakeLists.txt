cmake_minimum_required(VERSION 3.8) # min cmake version
project(sim_bot)                    # package name

# Compiler Options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
#find_package(<dependency> REQUIRED) add more here

# Install files
install(
  DIRECTORY config description launch meshes rviz worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/graph_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME graph_visualizer
)

install(
  PROGRAMS scripts/route_to_poses.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_to_poses
)

ament_package()

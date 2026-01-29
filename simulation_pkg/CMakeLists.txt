cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

# This package is primarily Python, but we also build a small C++ node.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -------------------------
# C++ node
# -------------------------
add_executable(scan_fixer_node src/scan_fixer.cpp)
ament_target_dependencies(scan_fixer_node rclcpp sensor_msgs)

add_executable(odom_fixer_node src/odom_fixer.cpp)
ament_target_dependencies(odom_fixer_node rclcpp nav_msgs)

install(TARGETS
  scan_fixer_node
  odom_fixer_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# Install Python package (simulation_pkg/ directory)
# -------------------------
ament_python_install_package(${PROJECT_NAME})

# -------------------------
# Install non-code resources (folders under share/simulation_pkg)
# -------------------------
install(DIRECTORY
  launch
  params
  config
  description
  worlds
  models
  maps
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

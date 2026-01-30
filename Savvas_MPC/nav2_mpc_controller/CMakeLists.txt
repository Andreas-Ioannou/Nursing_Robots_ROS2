cmake_minimum_required(VERSION 3.8)
project(nav2_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)


add_library(nav2_mpc_controller SHARED
  src/mpc_controller.cpp
)

target_include_directories(nav2_mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_mpc_controller
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

target_link_libraries(nav2_mpc_controller Eigen3::Eigen)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS nav2_mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

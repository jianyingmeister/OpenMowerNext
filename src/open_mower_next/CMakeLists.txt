cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 找到Robot依赖包
find_package(sensor_msgs REQUIRED)  # ✅ 必须加
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DockRobot.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

### --- sim_node: GPS/IMU + PTY simulator --- ###
# add_executable(sim_node src/sim_node.cpp)
# ament_target_dependencies(sim_node rclcpp std_msgs geometry_msgs)

add_executable(sim_node
  sim/sim_node.cpp
)

ament_target_dependencies(sim_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  sim_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
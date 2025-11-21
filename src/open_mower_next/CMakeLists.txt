cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

# 在 CMakeLists.txt 中添加或检查以下内容 lxb20251119
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# 如果有其他目录也需要安装，例如 config, maps, worlds 等 lxb20251119
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#郭的
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rclcpp_action REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 找到Robot依赖包 郭的
find_package(sensor_msgs REQUIRED)  # ✅ 必须加
#find_package(nav_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

#原版
find_package(ament_cmake REQUIRED)           # ROS2 构建系统
find_package(rclcpp REQUIRED)                # ROS2 C++ 客户端库
find_package(rclcpp_action REQUIRED)         # ROS2 动作支持
find_package(nlohmann_json REQUIRED)         # JSON 库
find_package(rosidl_default_generators REQUIRED) # 接口生成
find_package(geometry_msgs REQUIRED)         # 几何消息
find_package(nav_msgs REQUIRED)              # 导航消息
find_package(nav2_msgs REQUIRED)             # Nav2 消息
find_package(std_msgs REQUIRED)              # 标准消息
find_package(std_srvs REQUIRED)              # 标准服务
find_package(robot_localization REQUIRED)    # 机器人定位
find_package(tf2 REQUIRED)                   # TF2 变换库
find_package(tf2_ros REQUIRED)               # TF2 ROS 包装
find_package(tf2_geometry_msgs REQUIRED)     # TF2 几何消息
#find_package(behaviortree_cpp REQUIRED)      # 行为树库   （编译出错、注释掉）
find_package(ament_index_cpp REQUIRED)       # ROS2 包索引
find_package(foxglove_msgs REQUIRED)         # Foxglove 消息 （编译出错、注释掉）
find_package(visualization_msgs REQUIRED)    # 可视化消息
find_package(unique_identifier_msgs REQUIRED) # 唯一标识符
find_package(pluginlib REQUIRED)             # 插件库
#find_package(opennav_docking REQUIRED)       # 开源导航对接 （编译出错、注释掉）
# 地理库特殊配置
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)



#郭的
#rosidl_generate_interfaces(${PROJECT_NAME}
# "action/DockRobot.action"
# DEPENDENCIES builtin_interfaces
#)

# 生成自定义的 ROS2 接口（消息、服务、动作） 原版
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/RemoveArea.srv"              # 删除区域服务
        "srv/RemoveDockingStation.srv"    # 删除对接站服务
        "srv/SaveArea.srv"                # 保存区域服务
        "srv/SaveDockingStation.srv"      # 保存对接站服务
        "srv/FindNearestDockingStation.srv" # 查找最近对接站服务
        "msg/Area.msg"                    # 区域消息
        "msg/Map.msg"                     # 地图消息
        "msg/DockingStation.msg"          # 对接站消息
        "action/DockRobotTo.action"       # 对接机器人到指定站动作
        "action/DockRobotNearest.action"  # 对接机器人到最近站动作
        "action/RecordDockingStation.action" # 录制对接站动作
        "action/RecordAreaBoundary.action" # 录制区域边界动作
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_geometry_msgs
        unique_identifier_msgs

        #郭的
        rclcpp
        rclcpp_action
        nav_msgs
        tf2_ros
)

ament_export_dependencies(rosidl_default_runtime)

#原版
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# 包含各个模块的构建配置 #原版
#include(cmake/map_server.cmake)      # 地图服务器模块
#include(cmake/sim.cmake)             # 仿真模块
#include(cmake/docking_helper.cmake)  # 对接助手模块
#include(cmake/map_recorder.cmake)    # 地图录制模块



#测试配置 原版
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # 跳过版权检查
    set(ament_cmake_copyright_FOUND TRUE)
    # 跳过 cpplint 检查
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

#安装配置 原版
# 安装配置文件、启动文件、描述文件等到共享目录
INSTALL(
        DIRECTORY config launch description worlds maps
        DESTINATION share/${PROJECT_NAME}
)



### --- sim_node: GPS/IMU + PTY simulator --- ###
# add_executable(sim_node src/sim_node.cpp)
# ament_target_dependencies(sim_node rclcpp std_msgs geometry_msgs)

## --- 郭的 --- ###
add_executable(sim_node
 sim/sim_node.cpp
)

## --- 郭的 --- ###
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

#install(TARGETS
#  sim_node
# DESTINATION lib/${PROJECT_NAME}
#)

#============ 删除模块包含，直接在这里构建 ============

# 构建 map_server_node
add_executable(map_server_node 
    map_server/node_main.cpp
    map_server/map_server_node.cpp
    map_server/geo_json_map.cpp 
)

target_include_directories(map_server_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/map_server
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

ament_target_dependencies(
    map_server_node
    rclcpp rclcpp_action nlohmann_json geometry_msgs nav_msgs
    visualization_msgs tf2 tf2_ros tf2_geometry_msgs
    robot_localization nav2_msgs std_msgs std_srvs
    foxglove_msgs unique_identifier_msgs
    GeographicLib
)

# 添加这一行：链接生成的接口库
# 关键修复：链接生成的接口库
target_link_libraries(map_server_node 
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_generator_c
) 

# 构建 map_recorder_node
add_executable(map_recorder_node 
    #map_recorder_node.cpp
    map_recorder/main.cpp
    map_recorder/map_recorder_node.cpp
)

target_include_directories(map_recorder_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/map_recorder
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

ament_target_dependencies(map_recorder_node
    rclcpp rclcpp_action geometry_msgs nav_msgs
    tf2 tf2_ros tf2_geometry_msgs std_msgs std_srvs
    nav2_msgs unique_identifier_msgs
)

# 关键修复：链接生成的接口库
target_link_libraries(map_recorder_node 
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_generator_c
)

install(TARGETS
    map_server_node
    map_recorder_node
    sim_node
    DESTINATION lib/${PROJECT_NAME}
)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

INSTALL(
        DIRECTORY config launch description worlds maps
        DESTINATION share/${PROJECT_NAME}
)

  
# 生成 ROS2 包配置
ament_package()
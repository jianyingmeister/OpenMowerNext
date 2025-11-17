#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('open_mower_next')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'solar_farm.world')
    robot_xacro = os.path.join(pkg_share, 'description', 'mower_robot.urdf.xacro')
    ekf_config = os.path.join(pkg_share, 'config', 'robot_localization.yaml')
    nav2_launch = os.path.join(pkg_share, 'launch', 'nav2.launch.py')  # existing nav2 launch

    # Launch config args (can be overridden from CLI)
    create_pty = LaunchConfiguration('create_pty', default='true')
    serial_device = LaunchConfiguration('serial_device', default='/tmp/ublox_sim_write')
    sim_freq = LaunchConfiguration('sim_freq', default='10.0')
    init_lat = LaunchConfiguration('init_lat', default='37.0')
    init_lon = LaunchConfiguration('init_lon', default='-122.0')
    init_alt = LaunchConfiguration('init_alt', default='10.0')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')

    # Render robot_description from xacro
    robot_description_cmd = Command(['xacro ', robot_xacro])

    # Gazebo (classic) start
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # robot_state_publisher publishes robot_description param (used by spawn_entity and TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_cmd}]
    )

    # spawn_entity: will read robot_description param published by robot_state_publisher
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'openmower_sim'],
        output='screen'
    )

    # sim_node: publishes /gps/fix, /imu/data, /odom, broadcasts TF and creates PTY (optional)
    sim_node = Node(
        package='open_mower_next',
        executable='sim_node',
        name='sim_node',
        output='screen',
        parameters=[{
            'create_pty': create_pty,
            'serial_device': serial_device,
            'freq': sim_freq,
            'init_lat': init_lat,
            'init_lon': init_lon,
            'init_alt': init_alt
        }]
    )

    # ekf (robot_localization) - ensure your config uses imu0: imu/data, gps0: gps/fix, odom0: odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # Nav2 (optional include) - only include if nav2.launch.py exists and use_nav2==true
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        condition=IfCondition(use_nav2)
    )

    ld = LaunchDescription()

    # Declare arguments so user can override via CLI
    ld.add_action(DeclareLaunchArgument('create_pty', default_value='true',
                                        description='Let sim_node create PTY pair when true.'))
    ld.add_action(DeclareLaunchArgument('serial_device', default_value='/tmp/ublox_sim_write',
                                        description='If create_pty==false, write NMEA to this device.'))
    ld.add_action(DeclareLaunchArgument('sim_freq', default_value='10.0',
                                        description='Sim sensor publish frequency (Hz)'))
    ld.add_action(DeclareLaunchArgument('init_lat', default_value='37.0', description='Initial latitude'))
    ld.add_action(DeclareLaunchArgument('init_lon', default_value='-122.0', description='Initial longitude'))
    ld.add_action(DeclareLaunchArgument('init_alt', default_value='10.0', description='Initial altitude'))
    ld.add_action(DeclareLaunchArgument('use_nav2', default_value='true', description='Include Nav2 launch'))

    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(sim_node)
    ld.add_action(ekf_node)
    ld.add_action(nav2_include)

    return ld

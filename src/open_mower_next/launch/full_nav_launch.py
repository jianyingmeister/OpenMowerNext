#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(
            os.path.expanduser('~/OpenMowerNext/maps'),
            'map.yaml'
        )
    )

    # sim_node
    sim_node = Node(
        package='open_mower_next',
        executable='sim_node',
        name='sim_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM toolbox (online async)
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Nav2
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    nav2_node = Node(
        package='nav2_bringup',
        executable='navigation_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'map': map_yaml_file}]
    )

    return LaunchDescription([
        sim_node,
        slam_node,
        nav2_node
    ])


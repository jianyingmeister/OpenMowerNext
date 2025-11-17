import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
pkg_share = get_package_share_directory('open_mower_next')
world_file = os.path.join(pkg_share, 'worlds', 'solar_farm.world')
robot_description = os.path.join(pkg_share, 'description', 'mower_robot.urdf.xacro')


# Spawn robot using spawn_entity.py from gazebo_ros
spawn_entity = Node(
package='gazebo_ros',
executable='spawn_entity.py',
arguments=['-topic', 'robot_description', '-entity', 'openmower_sim'],
output='screen'
)


# Launch Gazebo (classic) with given world
gazebo = ExecuteProcess(
cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
output='screen'
)


# Start sim_node (which publishes sensors + writes to a serial pty)
sim_node = Node(
package='open_mower_next',
executable='sim_node',
name='sim_node',
output='screen',
parameters=[{
'serial_device': '/tmp/ublox_sim_write',
'gps_frame': 'gps_link',
'publish_topics': True
}]
)


# Start robot_localization (assumes config file exists)
ekf_node = Node(
package='robot_localization',
executable='ekf_node',
name='ekf_filter_node',
output='screen',
parameters=[os.path.join(pkg_share, 'config', 'robot_localization.yaml')]
)


# Optionally start full_nav launch (Nav2) if available
nav2_launch = IncludeLaunchDescription(
PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav2.launch.py'))
)


return LaunchDescription([
gazebo,
spawn_entity,
sim_node,
ekf_node,
nav2_launch
])
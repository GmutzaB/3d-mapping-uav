"""Everything for the aircraft: L2 driver, MAVROS, and LiDAR obstacle sectors.

Arguments from lidar.launch.py and mavros.launch.py can be passed here too,
e.g. `ros2 launch uav_lidar_bridge uav.launch.py fcu_url:=serial:///dev/ttyAMA0:921600`.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('uav_lidar_bridge')
    include = lambda name: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
        condition=IfCondition(LaunchConfiguration(name.split('.')[0])))
    sectors = Node(
        package='uav_lidar_bridge',
        executable='obstacle_sectors',
        name='lidar_obstacle_sectors',
        output='screen',
        parameters=[os.path.join(share, 'config', 'obstacle_sectors.yaml')],
        condition=IfCondition(LaunchConfiguration('obstacles')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Start the L2 driver'),
        DeclareLaunchArgument('mavros', default_value='true', description='Start MAVROS'),
        DeclareLaunchArgument('obstacles', default_value='true',
                              description='Send LiDAR obstacle sectors to ArduPilot'),
        include('lidar.launch.py'),
        include('mavros.launch.py'),
        sectors,
    ])

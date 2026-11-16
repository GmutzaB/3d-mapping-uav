"""Everything for the aircraft: L2 driver, MAVROS, LiDAR obstacle sectors, scan recording.

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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('uav_lidar_bridge')
    text = lambda name: ParameterValue(LaunchConfiguration(name), value_type=str)
    config = lambda name: os.path.join(share, 'config', name)
    include = lambda name: IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
        condition=IfCondition(LaunchConfiguration(name.split('.')[0])))
    sectors = Node(
        package='uav_lidar_bridge',
        executable='obstacle_sectors',
        name='lidar_obstacle_sectors',
        output='screen',
        parameters=[config('mount.yaml'), config('obstacle_sectors.yaml')],
        condition=IfCondition(LaunchConfiguration('obstacles')),
    )
    recorder = Node(
        package='uav_lidar_bridge',
        executable='scan_recorder',
        name='scan_recorder',
        output='screen',
        parameters=[config('mount.yaml'), config('scan_recorder.yaml'), {
            'scan_dir': text('scan_dir'),
            'trigger': text('trigger'),
            'pose_topic': text('pose_topic'),
        }],
        condition=IfCondition(LaunchConfiguration('record')),
    )
    server = Node(
        package='uav_lidar_bridge',
        executable='scan_server',
        output='screen',
        arguments=['--dir', LaunchConfiguration('scan_dir'),
                   '--port', LaunchConfiguration('server_port')],
        condition=IfCondition(LaunchConfiguration('server')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Start the L2 driver'),
        DeclareLaunchArgument('mavros', default_value='true', description='Start MAVROS'),
        DeclareLaunchArgument('obstacles', default_value='true',
                              description='Send LiDAR obstacle sectors to ArduPilot'),
        DeclareLaunchArgument('record', default_value='true',
                              description='Record a 3D scan (see config/scan_recorder.yaml)'),
        DeclareLaunchArgument('trigger', default_value='arm',
                              description='arm | manual | always'),
        DeclareLaunchArgument('pose_topic', default_value='mavros/local_position/pose',
                              description='Aircraft pose topic, or none for a stationary bench scan'),
        DeclareLaunchArgument('scan_dir', default_value=os.path.expanduser('~/scans')),
        DeclareLaunchArgument('server', default_value='true',
                              description='Serve scans to the phone app over HTTP'),
        DeclareLaunchArgument('server_port', default_value='8080'),
        include('lidar.launch.py'),
        include('mavros.launch.py'),
        sectors,
        recorder,
        server,
    ])

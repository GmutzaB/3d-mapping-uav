"""Unitree L2 driver over Ethernet. No RViz, since the Pi runs headless."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = lambda name, value_type: ParameterValue(LaunchConfiguration(name), value_type=value_type)
    driver = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar',
        output='screen',
        parameters=[{
            'initialize_type': 2,  # 1 = serial, 2 = Ethernet (UDP)
            'work_mode': 0,        # standard FOV, 3D, IMU on, Ethernet, auto-start
            'use_system_timestamp': True,
            'range_min': 0.0,
            'range_max': arg('range_max', float),
            # Each UDP packet is one line of up to 300 returns. 18 lines was a thin
            # slice, so a sweep threw away most of the spin before the map saw it.
            'cloud_scan_num': arg('cloud_scan_num', int),
            'lidar_ip': arg('lidar_ip', str),
            'lidar_port': 6101,
            'local_ip': arg('local_ip', str),
            'local_port': 6201,
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.62',
                              description='L2 address (factory default 192.168.1.62)'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.2',
                              description="Pi Ethernet address the L2 sends to"),
        DeclareLaunchArgument('range_max', default_value='30.0'),
        DeclareLaunchArgument(
            'cloud_scan_num',
            default_value='72',
            description='UDP packets merged into each cloud. Higher keeps more of each spin.',
        ),
        driver,
    ])

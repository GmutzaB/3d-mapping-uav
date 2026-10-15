"""MAVROS connected to the ArduCopter flight controller.

Same setup as `ros2 launch mavros apm.launch`, plus config/mavros_overrides.yaml.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mavros_launch = os.path.join(get_package_share_directory('mavros'), 'launch')
    overrides = os.path.join(get_package_share_directory('uav_lidar_bridge'),
                             'config', 'mavros_overrides.yaml')
    arg = lambda name, value_type: ParameterValue(LaunchConfiguration(name), value_type=value_type)
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        parameters=[
            os.path.join(mavros_launch, 'apm_pluginlists.yaml'),
            os.path.join(mavros_launch, 'apm_config.yaml'),
            overrides,
            {
                'fcu_url': arg('fcu_url', str),
                'gcs_url': arg('gcs_url', str),
                'tgt_system': arg('tgt_system', int),
                'tgt_component': 1,
                'fcu_protocol': 'v2.0',
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='serial:///dev/ttyACM0:115200',
                              description='USB: serial:///dev/ttyACM0:115200, '
                                          'Pi GPIO UART: serial:///dev/ttyAMA0:921600'),
        DeclareLaunchArgument('gcs_url', default_value='',
                              description='Optional ground station relay, e.g. udp-b://@'),
        DeclareLaunchArgument('tgt_system', default_value='1',
                              description='Flight controller SYSID_THISMAV'),
        mavros,
    ])

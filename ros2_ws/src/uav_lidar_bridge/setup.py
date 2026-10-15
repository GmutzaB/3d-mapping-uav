from glob import glob
from setuptools import setup

package_name = 'uav_lidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GmutzaB',
    maintainer_email='gmutzab@gmail.com',
    description='Unitree L2 and MAVROS launch files plus a LiDAR obstacle sector node.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_sectors = uav_lidar_bridge.obstacle_node:main',
        ],
    },
)

"""Unitree L2 point cloud -> 72-sector LaserScan for the MAVROS obstacle plugin.

MAVROS forwards the scan to ArduPilot as OBSTACLE_DISTANCE. This node only reports
distances; it never arms, changes mode, or commands motion.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from .sectors import (SECTOR_COUNT, SECTOR_WIDTH_DEG, SectorMemory,
                      nearest_per_sector, to_body_flu)


class ObstacleSectors(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_sectors')
        param = lambda name, default: self.declare_parameter(name, default).value
        cloud_topic = param('cloud_topic', 'unilidar/cloud')
        scan_topic = param('scan_topic', 'mavros/obstacle/send')
        self.mount_xyz = [param('mount_x', 0.0), param('mount_y', 0.0), param('mount_z', 0.0)]
        self.mount_rpy = [math.radians(param(f'mount_{a}_deg', 0.0)) for a in ('roll', 'pitch', 'yaw')]
        self.z_min = param('z_min', -0.3)
        self.z_max = param('z_max', 0.3)
        self.range_min = param('range_min', 0.35)
        self.range_max = param('range_max', 30.0)
        self.frame_id = param('frame_id', 'base_link')
        self.memory = SectorMemory(param('hold_s', 0.3))

        self.pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f'{cloud_topic} -> {scan_topic}, {SECTOR_COUNT} sectors')

    def on_cloud(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        nearest = nearest_per_sector(to_body_flu(xyz, self.mount_xyz, self.mount_rpy),
                                     z_min=self.z_min, z_max=self.z_max,
                                     range_min=self.range_min, range_max=self.range_max)
        ranges = self.memory.update(nearest, time.monotonic())

        # Angles run clockwise from forward (not the usual ROS counter-clockwise),
        # because MAVROS passes them to ArduPilot unchanged with mav_frame BODY_FRD.
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = 0.0
        scan.angle_increment = math.radians(SECTOR_WIDTH_DEG)
        scan.angle_max = scan.angle_increment * (SECTOR_COUNT - 1)
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        scan.ranges = [float(r) for r in ranges]
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ObstacleSectors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

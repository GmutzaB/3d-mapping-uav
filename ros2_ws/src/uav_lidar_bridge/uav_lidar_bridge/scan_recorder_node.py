"""Record a 3D scan on the aircraft and save it to disk when the flight ends.

Default behaviour: start recording when the flight controller arms, save
~/scans/scan_<date>_<time>/map.ply + scan.json when it disarms. The scan server
(scan_server.py) then hands the files to the phone app after landing.

Pose comes from a PoseStamped topic (default: MAVROS local position, ENU). Set
pose_topic to none for a stationary bench scan with the sensor sitting still.
This node only listens; it never arms, changes mode, or commands motion.
"""
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger

from .scan_map import CloudIntegrator, PoseBuffer, VoxelMap, save_scan

TRIGGERS = ('arm', 'manual', 'always')


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ScanRecorder(Node):
    def __init__(self):
        super().__init__('scan_recorder')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.cloud_topic = param('cloud_topic', 'unilidar/cloud')
        self.pose_topic = param('pose_topic', 'mavros/local_position/pose')
        state_topic = param('state_topic', 'mavros/state')
        self.trigger = param('trigger', 'arm')
        self.scan_dir = os.path.expanduser(param('scan_dir', '~/scans'))
        self.voxel_size = param('voxel_size', 0.05)
        self.max_voxels = param('max_voxels', 4_000_000)
        self.mount_xyz = [param('mount_x', 0.0), param('mount_y', 0.0), param('mount_z', 0.0)]
        self.mount_rpy_deg = [param(f'mount_{a}_deg', 0.0) for a in ('roll', 'pitch', 'yaw')]
        self.range_min = param('range_min', 0.35)
        self.range_max = param('range_max', 30.0)
        self.deskew_slices = param('deskew_slices', 8)
        self.max_pose_gap_s = param('max_pose_gap_s', 0.2)
        self.autosave_s = param('autosave_s', 60.0)
        preview_s = param('preview_period_s', 2.0)
        self.preview_max_points = param('preview_max_points', 150_000)
        if self.trigger not in TRIGGERS:
            raise ValueError(f"trigger must be one of {TRIGGERS}, got '{self.trigger}'")

        self.stationary = self.pose_topic.strip().lower() in ('', 'none')
        self.poses = None if self.stationary else PoseBuffer(self.max_pose_gap_s)
        self.integrator = None   # set while recording
        self.folder = None
        self.started = None
        self.last_save = 0.0
        self.armed = None

        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud,
                                 qos_profile_sensor_data)
        if not self.stationary:
            self.create_subscription(PoseStamped, self.pose_topic, self.on_pose,
                                     qos_profile_sensor_data)
        if self.trigger == 'arm':
            self.create_subscription(State, state_topic, self.on_state, 10)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.preview_pub = self.create_publisher(PointCloud2, '~/map', 1)
        if preview_s > 0:
            self.create_timer(preview_s, self.publish_preview)
        self.create_timer(10.0, self.report)

        mode = 'stationary (no pose)' if self.stationary else f'pose from {self.pose_topic}'
        self.get_logger().info(f'{self.cloud_topic}, {mode}, trigger={self.trigger}, '
                               f'saving to {self.scan_dir}')
        if self.trigger == 'always':
            self.start()

    # --- start / stop ----------------------------------------------------------

    def start(self):
        if self.integrator is not None:
            return False, f'already recording {self.folder}'
        name = datetime.now().strftime('scan_%Y%m%d_%H%M%S')
        self.folder = os.path.join(self.scan_dir, name)
        self.integrator = CloudIntegrator(
            VoxelMap(self.voxel_size, self.max_voxels),
            mount_xyz=self.mount_xyz, mount_rpy=[math.radians(a) for a in self.mount_rpy_deg],
            range_min=self.range_min, range_max=self.range_max,
            deskew_slices=self.deskew_slices, pose_buffer=self.poses)
        self.started = datetime.now().astimezone()
        self.last_save = time.monotonic()
        self.get_logger().info(f'Recording started: {self.folder}')
        return True, self.folder

    def stop(self):
        if self.integrator is None:
            return False, 'not recording'
        folder, points = self.folder, len(self.integrator.map)
        if points:
            self.save(complete=True)
            message = f'saved {points} points to {folder}'
        else:
            message = 'nothing captured; no scan saved (check pose and cloud topics)'
        self.integrator = None
        self.folder = None
        self.get_logger().info(f'Recording stopped: {message}')
        return True, message

    def save(self, complete):
        meta = dict(
            name=os.path.basename(self.folder), complete=complete,
            started=self.started.isoformat(timespec='seconds'),
            ended=datetime.now().astimezone().isoformat(timespec='seconds'),
            trigger=self.trigger, pose_source='stationary' if self.stationary else self.pose_topic,
            cloud_topic=self.cloud_topic, mount_xyz_m=self.mount_xyz,
            mount_rpy_deg=self.mount_rpy_deg, range_m=[self.range_min, self.range_max],
            stats=dict(self.integrator.stats))
        try:
            save_scan(self.folder, self.integrator.map, meta)
        except OSError as e:  # full SD card etc.: keep flying, keep the map in memory
            self.get_logger().error(f'Could not save scan to {self.folder}: {e}')
        self.last_save = time.monotonic()

    def srv_start(self, _request, response):
        response.success, response.message = self.start()
        return response

    def srv_stop(self, _request, response):
        response.success, response.message = self.stop()
        return response

    # --- inputs ---------------------------------------------------------------

    def on_state(self, msg):
        # While the FC link is down MAVROS reports armed=false; that is not a landing,
        # so ignore it instead of cutting the scan in two.
        if not msg.connected:
            return
        if msg.armed and not self.armed:
            self.start()
        elif not msg.armed and self.armed:
            self.stop()
        self.armed = msg.armed

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.poses.add(stamp_s(msg.header.stamp), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_cloud(self, msg):
        if self.integrator is None:
            return
        names = {f.name for f in msg.fields}
        fields = ['x', 'y', 'z'] + [n for n in ('intensity', 'time') if n in names]
        data = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=True)
        data = np.asarray(data, dtype=float).reshape(-1, len(fields))
        col = {n: data[:, i] for i, n in enumerate(fields)}
        self.integrator.add_cloud(stamp_s(msg.header.stamp), data[:, :3],
                                  intensity=col.get('intensity'), point_time=col.get('time'))
        if self.autosave_s > 0 and time.monotonic() - self.last_save > self.autosave_s:
            self.save(complete=False)

    # --- feedback ---------------------------------------------------------------

    def report(self):
        if self.integrator is None:
            return
        s = self.integrator.stats
        self.get_logger().info(
            f"{len(self.integrator.map)} map points, clouds used {s['used']}/{s['clouds']}"
            + (f", {s['no_pose']} skipped without pose" if s['no_pose'] else '')
            + (' -- MAP FULL, raise max_voxels or voxel_size' if self.integrator.map.full else ''))
        if s['clouds'] >= 20 and s['used'] == 0:
            self.get_logger().warn(f'No clouds placed: is {self.pose_topic} publishing '
                                   'with timestamps close to the LiDAR clock?')

    def publish_preview(self):
        if self.integrator is None or self.preview_pub.get_subscription_count() == 0:
            return
        xyz, _, _ = self.integrator.map.points()
        step = max(1, len(xyz) // self.preview_max_points)
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        self.preview_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz[::step]))


def main():
    rclpy.init()
    node = ScanRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.integrator is not None:  # Ctrl-C or shutdown mid-flight: keep what we have
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

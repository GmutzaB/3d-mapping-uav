"""Run on any machine: cd ros2_ws/src/uav_lidar_bridge && python -m unittest discover -s test"""
import json
import math
import os
import tempfile
import unittest

import numpy as np

from uav_lidar_bridge.scan_map import (CloudIntegrator, PoseBuffer, VoxelMap, quat_to_matrix,
                                       read_ply, save_scan, slerp)


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class RotationTests(unittest.TestCase):
    def test_yaw_90_turns_x_into_y(self):
        np.testing.assert_allclose(quat_to_matrix(yaw_quat(math.pi / 2)) @ [1, 0, 0],
                                   [0, 1, 0], atol=1e-12)

    def test_slerp_halfway(self):
        q = slerp(yaw_quat(0), yaw_quat(math.pi / 2), 0.5)
        np.testing.assert_allclose(q, yaw_quat(math.pi / 4), atol=1e-12)

    def test_slerp_takes_short_way_round(self):
        q = slerp(yaw_quat(math.radians(170)), yaw_quat(math.radians(-170)), 0.5)
        np.testing.assert_allclose(quat_to_matrix(q) @ [1, 0, 0], [-1, 0, 0], atol=1e-9)


class PoseBufferTests(unittest.TestCase):
    def test_interpolates_position_and_heading(self):
        b = PoseBuffer(max_gap_s=0.5)
        b.add(1.0, (0, 0, 1), yaw_quat(0))
        b.add(1.2, (2, 0, 1), yaw_quat(math.pi / 2))
        R, p = b.at(1.1)
        np.testing.assert_allclose(p, [1, 0, 1])
        np.testing.assert_allclose(R @ [1, 0, 0], [math.sqrt(.5), math.sqrt(.5), 0], atol=1e-12)

    def test_refuses_to_guess(self):
        b = PoseBuffer(max_gap_s=0.2)
        b.add(1.0, (0, 0, 0), yaw_quat(0))
        b.add(2.0, (1, 0, 0), yaw_quat(0))           # 1 s gap: lost link, not interpolable
        self.assertIsNone(b.at(1.5))
        self.assertIsNone(b.at(0.9))                 # before history
        self.assertIsNone(b.at(2.1))                 # no extrapolation
        self.assertIsNotNone(b.at(2.0))

    def test_rejects_bad_samples(self):
        b = PoseBuffer()
        self.assertFalse(b.add(1.0, (np.nan, 0, 0), yaw_quat(0)))
        self.assertFalse(b.add(1.0, (0, 0, 0), (0, 0, 0, 0)))
        self.assertTrue(b.add(1.0, (0, 0, 0), yaw_quat(0)))
        self.assertFalse(b.add(1.0, (0, 0, 0), yaw_quat(0)))  # duplicate time


class VoxelMapTests(unittest.TestCase):
    def test_same_voxel_is_averaged(self):
        m = VoxelMap(0.1)
        m.add([(0.01, 0.01, 0.01), (0.03, 0.05, 0.07), (1, 1, 1)], [10, 30, 5])
        xyz, inten, counts = m.points()
        order = np.argsort(xyz[:, 0])
        np.testing.assert_allclose(xyz[order[0]], [0.02, 0.03, 0.04])
        self.assertEqual((inten[order[0]], counts[order[0]]), (20, 2))
        self.assertEqual(len(m), 2)

    def test_negative_coordinates_and_repeats(self):
        m = VoxelMap(0.5)
        for _ in range(3):
            m.add([(-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)])
        self.assertEqual(len(m), 2)                  # -0.1 and +0.1 are different voxels
        self.assertEqual(sorted(m.points()[2]), [3, 3])

    def test_cap_stops_growth_but_keeps_updating(self):
        m = VoxelMap(1.0, max_voxels=2)
        m.add([(0, 0, 0), (5, 0, 0), (9, 0, 0)])
        self.assertTrue(m.full)
        self.assertEqual(len(m), 2)
        self.assertEqual(m.add([(0.5, 0, 0)]), 1)   # existing voxel still accepted

    def test_grows_past_initial_capacity(self):
        m = VoxelMap(0.01)
        m.add(np.arange(5000)[:, None] * [0.02, 0, 0])
        self.assertEqual(len(m), 5000)


def wall_cloud(aircraft_xyz, times=None):
    """Points on the wall y=5 (north), x in 0..4, z in 0..2, as seen from the aircraft."""
    gx, gz = np.meshgrid(np.linspace(0, 4, 41), np.linspace(0, 2, 21))
    world = np.column_stack([gx.ravel(), np.full(gx.size, 5.0), gz.ravel()])
    return world - aircraft_xyz if times is None else world - aircraft_xyz(times)[:, None] * [1, 0, 0]


class IntegratorTests(unittest.TestCase):
    def test_wall_stays_put_while_aircraft_moves(self):
        poses = PoseBuffer()
        for k in range(11):
            poses.add(k * 0.1, (k * 0.1, 0, 1), yaw_quat(0))
        integ = CloudIntegrator(VoxelMap(0.05), pose_buffer=poses, range_min=0.1)
        for k in range(10):
            integ.add_cloud(k * 0.1, wall_cloud(np.array([k * 0.1, 0, 1])))
        xyz = integ.map.points()[0]
        self.assertLess(np.abs(xyz[:, 1] - 5).max(), 1e-9)
        self.assertEqual(integ.stats['used'], 10)

    def test_heading_is_applied(self):
        poses = PoseBuffer(max_gap_s=2.0)
        poses.add(0.0, (0, 0, 0), yaw_quat(math.pi / 2))   # nose pointing north
        poses.add(1.0, (0, 0, 0), yaw_quat(math.pi / 2))
        integ = CloudIntegrator(VoxelMap(0.05), pose_buffer=poses)
        integ.add_cloud(0.5, [(3.0, 0, 0)])                # 3 m straight ahead
        np.testing.assert_allclose(integ.map.points()[0], [[0, 3, 0]], atol=1e-9)

    def test_deskew_fixes_motion_within_one_message(self):
        speed = 10.0                                         # m/s east, 0.1 s per message
        poses = PoseBuffer()
        for k in range(21):
            poses.add(k * 0.01, (speed * k * 0.01, 0, 0), yaw_quat(0))
        n = 41 * 21
        offsets = np.linspace(0, 0.099, n)
        pts = wall_cloud(lambda t: speed * t, offsets)

        def overshoot(slices):
            """How far the mapped wall spills past its true ends (x = 0..4 m)."""
            integ = CloudIntegrator(VoxelMap(0.01), pose_buffer=poses, range_min=0.1,
                                    deskew_slices=slices)
            integ.add_cloud(0.0, pts, point_time=offsets)
            x = integ.map.points()[0][:, 0]
            return max(0 - x.min(), x.max() - 4)

        self.assertGreater(overshoot(1), 0.5)                # smeared by ~1 m without deskew
        self.assertLess(overshoot(16), 0.05)

    def test_missing_pose_is_skipped_not_guessed(self):
        poses = PoseBuffer()
        poses.add(10.0, (0, 0, 0), yaw_quat(0))
        poses.add(10.1, (0, 0, 0), yaw_quat(0))
        integ = CloudIntegrator(VoxelMap(0.05), pose_buffer=poses)
        self.assertFalse(integ.add_cloud(3.0, wall_cloud(np.zeros(3))))
        self.assertEqual((integ.stats['no_pose'], len(integ.map)), (1, 0))

    def test_stationary_mode_with_upside_down_mount(self):
        integ = CloudIntegrator(VoxelMap(0.01), mount_xyz=(0.1, 0, -0.05),
                                mount_rpy=(math.pi, 0, 0))
        integ.add_cloud(0.0, [(2.0, 1.0, 0.5), (0.1, 0, 0)])  # second is a prop: too close
        np.testing.assert_allclose(integ.map.points()[0], [[2.1, -1.0, -0.55]], atol=1e-9)


class SaveTests(unittest.TestCase):
    def test_ply_and_json_round_trip(self):
        m = VoxelMap(0.1)
        m.add([(1, 2, 3), (-4, 5, 0.5)], [7, 9])
        with tempfile.TemporaryDirectory() as d:
            meta = save_scan(os.path.join(d, 'scan_x'), m, {'name': 'scan_x', 'complete': True})
            xyz, rec = read_ply(os.path.join(d, 'scan_x', 'map.ply'))
            with open(os.path.join(d, 'scan_x', 'scan.json')) as f:
                on_disk = json.load(f)
            self.assertEqual(sorted(os.listdir(os.path.join(d, 'scan_x'))), ['map.ply', 'scan.json'])
        self.assertEqual(on_disk, meta)
        self.assertEqual(meta['points'], 2)
        self.assertEqual(meta['bounds_min'], [-4.0, 2.0, 0.5])
        np.testing.assert_allclose(sorted(map(tuple, xyz)), [(-4, 5, 0.5), (1, 2, 3)], atol=1e-6)
        self.assertEqual(sorted(rec['intensity']), [7, 9])

    def test_demo_scan_looks_like_the_room(self):
        from uav_lidar_bridge.demo_scan import ROOM, make_demo
        with tempfile.TemporaryDirectory() as d:
            meta = make_demo(os.path.join(d, 'demo'), seconds=3.0)
            xyz, _ = read_ply(os.path.join(d, 'demo', 'map.ply'))
        self.assertGreater(meta['points'], 5000)
        self.assertTrue(np.all(xyz > -0.1) and np.all(xyz < ROOM + 0.1))


if __name__ == '__main__':
    unittest.main()

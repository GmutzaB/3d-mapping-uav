"""python3 -m unittest test_stable_map.py from ros2_ws/pi. No ROS required."""

import unittest

import numpy as np

from stable_map import ImuTrack, StableMap, Tracker, apply_pose


def corner(seed=0):
    rng = np.random.default_rng(seed)
    count = 80
    wall_x = np.column_stack((np.full(count, 2.0), rng.uniform(-1, 1, count), rng.uniform(0, 1.5, count)))
    wall_y = np.column_stack((rng.uniform(0, 2, count), np.full(count, -1.0), rng.uniform(0, 1.5, count)))
    floor = np.column_stack((rng.uniform(0, 2, count), rng.uniform(-1, 1, count), np.zeros(count)))
    return np.vstack((wall_x, wall_y, floor))


def wall(seed=0, count=500):
    rng = np.random.default_rng(seed)
    return np.column_stack((
        np.full(count, 2.0),
        rng.uniform(-2, 2, count),
        rng.uniform(0, 1.5, count),
    ))


def yaw(radians):
    cosine, sine = np.cos(radians), np.sin(radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


class StableMapTests(unittest.TestCase):
    def test_first_point_in_a_voxel_is_kept(self):
        grid = StableMap(0.1)
        added = grid.insert([[0.01, 0.01, 0.01], [0.04, 0.02, 0.0]], [1.0, 9.0])
        self.assertEqual(added, 1)
        self.assertEqual(grid.insert([[0.02, 0.02, 0.02]], [3.0]), 0)
        np.testing.assert_allclose(grid.cloud()[0], [0.01, 0.01, 0.01, 1.0])

    def test_new_scans_do_not_move_points_already_in_the_map(self):
        tracker = Tracker(voxel=0.05, max_correspondence=0.5)
        original = corner()
        self.assertEqual(tracker.add_scan(original), "seed")
        before = tracker.map.cloud()
        rotation = yaw(np.deg2rad(12))
        translation = np.array([0.12, -0.08, 0.03])
        moved = apply_pose(rotation.T, -rotation.T @ translation, original)
        extra = np.array([[4.0, 4.0, 0.4], [4.1, 4.2, 0.5], [3.8, 4.3, 0.2]])
        self.assertEqual(tracker.add_scan(np.vstack((moved, extra))), "aligned")
        after = tracker.map.cloud()
        np.testing.assert_allclose(after[: len(before), :3], before[:, :3])
        self.assertGreater(len(after), len(before))

    def test_a_sweep_with_no_overlap_is_dropped(self):
        tracker = Tracker(voxel=0.05, max_correspondence=0.4)
        tracker.add_scan(corner())
        before = tracker.map.cloud()
        far = corner(seed=1) + np.array([30.0, 0.0, 0.0])
        self.assertEqual(tracker.add_scan(far), "skipped")
        np.testing.assert_array_equal(tracker.map.cloud(), before)
        np.testing.assert_allclose(tracker.translation, 0)

    def test_pose_recovers_a_small_move(self):
        rotation = yaw(np.deg2rad(12))
        translation = np.array([0.12, -0.08, 0.03])
        original = corner()
        moved = apply_pose(rotation.T, -rotation.T @ translation, original)
        tracker = Tracker(voxel=0.05, max_correspondence=0.6)
        tracker.add_scan(original)
        self.assertEqual(tracker.add_scan(moved), "aligned")
        recovered = apply_pose(tracker.rotation, tracker.translation, moved)
        error = np.linalg.norm(recovered - original, axis=1).mean()
        self.assertLess(error, 0.05)
        np.testing.assert_allclose(tracker.rotation, rotation, atol=0.05)
        np.testing.assert_allclose(tracker.translation, translation, atol=0.05)

    def test_a_wall_stays_one_surface_when_the_next_sweep_is_offset(self):
        tracker = Tracker(voxel=0.03, max_correspondence=0.4)
        tracker.add_scan(wall())
        before = len(tracker.map)
        shifted = wall(seed=1) + np.array([0.04, 0.3, 0.0])
        self.assertEqual(tracker.add_scan(shifted), "aligned")
        cloud = tracker.map.cloud()
        self.assertLess(np.max(np.abs(cloud[:, 0] - 2.0)), 0.04)
        self.assertLess(len(cloud), before * 1.5)
        # The sweep was copied 4 cm further from the origin, which is the sensor
        # moving 4 cm toward the wall. Motion along the wall stays small.
        np.testing.assert_allclose(tracker.translation[0], -0.04, atol=0.03)
        self.assertLess(abs(tracker.translation[1]), 0.08)

    def test_repeating_the_same_wall_does_not_thicken_it(self):
        rng = np.random.default_rng(2)
        tracker = Tracker(voxel=0.03, max_correspondence=0.4)
        base = wall(count=600)
        tracker.add_scan(base)
        before = len(tracker.map)
        for draw in range(6):
            noisy = base + rng.normal(0, 0.008, base.shape)
            self.assertEqual(tracker.add_scan(noisy), "aligned")
        cloud = tracker.map.cloud()
        self.assertLessEqual(len(cloud), int(before * 1.15) + 5)
        self.assertLess(np.max(np.abs(cloud[:, 0] - 2.0)), 0.04)

    def test_imu_hint_recovers_a_turn_that_is_too_large_to_guess(self):
        rotation = yaw(np.deg2rad(50))
        translation = np.array([0.1, -0.05, 0.0])
        original = corner()
        moved = apply_pose(rotation.T, -rotation.T @ translation, original)
        blind = Tracker(voxel=0.05, max_correspondence=0.45)
        blind.add_scan(original)
        self.assertEqual(blind.add_scan(moved), "skipped")
        guided = Tracker(voxel=0.05, max_correspondence=0.45)
        guided.add_scan(original)
        self.assertEqual(guided.add_scan(moved, rotation_hint=rotation), "aligned")
        np.testing.assert_allclose(guided.rotation, rotation, atol=0.08)
        np.testing.assert_allclose(guided.translation, translation, atol=0.08)

    def test_gyro_integration_tracks_a_steady_turn(self):
        imu = ImuTrack()
        rate = np.array([0.0, 0.0, np.deg2rad(90)])
        for step in range(11):
            imu.push(step * 0.1, rate)
        np.testing.assert_allclose(imu.rotation_since(0.0, 1.0), yaw(np.deg2rad(90)), atol=1e-6)


if __name__ == "__main__":
    unittest.main()

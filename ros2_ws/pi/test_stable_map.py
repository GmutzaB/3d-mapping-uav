"""python3 -m unittest test_stable_map.py from ros2_ws/pi. No ROS required."""

import unittest

import numpy as np

from stable_map import StableMap, Tracker, apply_pose, predict_pose, rigid_transform


def corner(seed=0):
    rng = np.random.default_rng(seed)
    count = 80
    wall_x = np.column_stack((np.full(count, 2.0), rng.uniform(-1, 1, count), rng.uniform(0, 1.5, count)))
    wall_y = np.column_stack((rng.uniform(0, 2, count), np.full(count, -1.0), rng.uniform(0, 1.5, count)))
    floor = np.column_stack((rng.uniform(0, 2, count), rng.uniform(-1, 1, count), np.zeros(count)))
    return np.vstack((wall_x, wall_y, floor))


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

    def test_constant_velocity_guess(self):
        rotation, translation = predict_pose(np.eye(3), np.zeros(3), np.eye(3), np.array([0.1, 0.0, 0.0]))
        np.testing.assert_allclose(translation, [0.2, 0.0, 0.0])
        np.testing.assert_allclose(rotation, np.eye(3))

    def test_rigid_transform_round_trip(self):
        src = corner()
        rotation = yaw(np.deg2rad(40))
        translation = np.array([1.0, -2.0, 0.5])
        found_rotation, found_translation = rigid_transform(src, apply_pose(rotation, translation, src))
        np.testing.assert_allclose(found_rotation, rotation, atol=1e-9)
        np.testing.assert_allclose(found_translation, translation, atol=1e-9)


if __name__ == "__main__":
    unittest.main()

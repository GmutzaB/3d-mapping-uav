"""python3 -m unittest test_stable_map.py from ros2_ws/pi. No ROS required."""

import unittest

import numpy as np

from stable_map import (
    DEFAULT_VOXEL,
    ImuTrack,
    StableMap,
    Tracker,
    apply_pose,
    heading_deg,
    novelty_gate,
    pose_guess,
    quat_xyzw_to_rot,
)


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


def _room(count=100):
    rng = np.random.default_rng(4)
    span = rng.uniform(-1.6, 1.6, count)
    height = rng.uniform(0.0, 1.2, count)
    depth = rng.uniform(-1.6, 1.6, count)
    return np.vstack((
        np.column_stack((np.full(count, 2.0), span, height)),
        np.column_stack((np.full(count, -2.0), span, height)),
        np.column_stack((depth, np.full(count, 2.0), height)),
        np.column_stack((depth, np.full(count, -2.0), height)),
    ))


def _wall_gap(points):
    to_x = np.minimum(np.abs(points[:, 0] - 2.0), np.abs(points[:, 0] + 2.0))
    to_y = np.minimum(np.abs(points[:, 1] - 2.0), np.abs(points[:, 1] + 2.0))
    return np.minimum(to_x, to_y)


def _room_with_floor():
    """Walls plus a dense floor and a blob at the sensor. Those match at any heading."""
    rng = np.random.default_rng(1)
    floor = np.column_stack((
        rng.uniform(-1.8, 1.8, 1200),
        rng.uniform(-1.8, 1.8, 1200),
        np.zeros(1200),
    ))
    near = rng.normal(0, 0.04, (400, 3))
    near[:, 2] *= 0.2
    return np.vstack((_room(count=80), floor, near))


def yaw(radians):
    cosine, sine = np.cos(radians), np.sin(radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def pitch(radians):
    cosine, sine = np.cos(radians), np.sin(radians)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def _rotation_error(left, right):
    cosine = np.clip((np.trace(left.T @ right) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


class StableMapTests(unittest.TestCase):
    def test_default_spacing_is_a_few_millimetres(self):
        tracker = Tracker()
        self.assertAlmostEqual(tracker.voxel, 0.0025)
        self.assertAlmostEqual(tracker.voxel, DEFAULT_VOXEL)
        # New points stay 2.5 mm apart. A repeat of the same surface has to
        # clear the centimetre of range noise before it is stored again.
        self.assertAlmostEqual(novelty_gate(0.0025), 0.02)
        self.assertAlmostEqual(tracker.map.gate, 0.02)
        self.assertGreater(novelty_gate(0.05), 0.05)

    def test_points_a_few_millimetres_apart_are_both_kept(self):
        grid = StableMap(0.0025)
        added = grid.insert([[0.0, 0.0, 0.0], [0.003, 0.0, 0.0]], [1.0, 1.0])
        self.assertEqual(added, 2)
        self.assertEqual(grid.insert([[0.001, 0.0, 0.0]], [1.0]), 0)
        # 5 mm shares the stored cell, so it is the same surface.
        # 8 cm is past the cells that count as range scatter, so it is new space.
        mask = grid.novel_mask([[0.08, 0.0, 0.0], [0.005, 0.0, 0.0]])
        self.assertTrue(bool(mask[0]))
        self.assertFalse(bool(mask[1]))

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

    def test_imu_hint_seats_a_large_turn(self):
        rotation = yaw(np.deg2rad(50))
        translation = np.array([0.1, -0.05, 0.0])
        original = corner()
        moved = apply_pose(rotation.T, -rotation.T @ translation, original)
        guided = Tracker(voxel=0.05, max_correspondence=0.45)
        guided.add_scan(original)
        self.assertEqual(guided.add_scan(moved, rotation_hint=rotation), "aligned")
        np.testing.assert_allclose(guided.rotation, rotation, atol=0.08)
        np.testing.assert_allclose(guided.translation, translation, atol=0.08)

    def test_imu_turn_is_applied_on_top_of_the_last_icp_yaw(self):
        previous = yaw(np.deg2rad(20))
        imu_turn = yaw(np.deg2rad(15))
        guess = pose_guess(previous, imu_turn)
        self.assertAlmostEqual(heading_deg(guess), 35, delta=0.5)
        self.assertGreater(abs(heading_deg(guess) - heading_deg(imu_turn)), 10)

    def test_icp_yaw_sticks_when_the_imu_does_not_track_the_turn(self):
        # Same room, sensor yaw only. rotation_hint is the IMU change since the
        # last good spin; identity means the gyro missed the turn. Resetting the
        # heading to zero would draw the walls again at a new angle.
        original = _room()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        self.assertEqual(tracker.add_scan(original), "seed")
        seed_count = len(tracker.map)
        for degrees in (20, 40, 60, 80, 90):
            rotation = yaw(np.deg2rad(degrees))
            scan = apply_pose(rotation.T, np.zeros(3), original)
            self.assertEqual(tracker.add_scan(scan, rotation_hint=np.eye(3)), "aligned")
            self.assertAlmostEqual(heading_deg(tracker.rotation), degrees, delta=6)
        cloud = tracker.map.cloud()
        self.assertLess(len(cloud), seed_count * 2)
        self.assertLess(np.percentile(_wall_gap(cloud[:, :3]), 90), 0.08)

    def test_a_quarter_turn_with_a_silent_gyro_stays_on_one_room(self):
        original = _room()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        scan = apply_pose(yaw(np.deg2rad(90)).T, np.zeros(3), original)
        self.assertEqual(tracker.add_scan(scan, rotation_hint=np.eye(3)), "aligned")
        self.assertAlmostEqual(heading_deg(tracker.rotation), 90, delta=6)
        self.assertLess(len(tracker.map), seed_count * 1.15)

    def test_floor_points_do_not_spin_the_walls_into_rays(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        self.assertEqual(tracker.add_scan(original), "seed")
        seed_count = len(tracker.map)
        for degrees in (20, 45, 90):
            scan = apply_pose(yaw(np.deg2rad(degrees)).T, np.zeros(3), original)
            self.assertEqual(tracker.add_scan(scan, rotation_hint=np.eye(3)), "aligned")
            self.assertAlmostEqual(heading_deg(tracker.rotation), degrees, delta=6)
        cloud = tracker.map.xyz()
        self.assertLess(len(cloud), seed_count * 1.25)
        walls = cloud[(cloud[:, 2] > 0.05) & (np.linalg.norm(cloud[:, :2], axis=1) > 1.2)]
        self.assertGreater(len(walls), 40)
        self.assertLess(np.percentile(_wall_gap(walls), 90), 0.08)

    def test_a_wrong_gyro_turn_does_not_override_the_walls(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        scan = apply_pose(yaw(np.deg2rad(15)).T, np.zeros(3), original)
        self.assertEqual(tracker.add_scan(scan, rotation_hint=yaw(np.deg2rad(80))), "aligned")
        self.assertAlmostEqual(heading_deg(tracker.rotation), 15, delta=8)

    def _moved(self, original, rotation, translation):
        return apply_pose(rotation.T, -rotation.T @ translation, original)

    def test_a_short_first_cloud_does_not_freeze_the_map(self):
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        handful = np.array([
            [1.0, 0.0, 0.5],
            [1.1, 0.2, 0.6],
            [0.9, -0.2, 0.4],
            [1.2, 0.1, 0.7],
        ])
        self.assertEqual(tracker.add_scan(handful), "seed")
        self.assertEqual(len(tracker.map), 4)
        original = _room_with_floor()
        self.assertEqual(tracker.add_scan(original), "seed")
        self.assertGreater(len(tracker.map), 40)
        translation = np.array([0.0, 0.0, 0.15])
        scan = self._moved(original, np.eye(3), translation)
        self.assertEqual(tracker.add_scan(scan, facing=np.eye(3)), "aligned")
        np.testing.assert_allclose(tracker.translation, translation, atol=0.05)

    def test_a_lift_is_not_pulled_back_to_the_origin(self):
        rng = np.random.default_rng(3)
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        translation = np.array([0.0, 0.0, 0.18])
        scan = self._moved(original, np.eye(3), translation)
        self.assertEqual(tracker.add_scan(scan, facing=np.eye(3)), "aligned")
        for _ in range(3):
            noisy = scan + rng.normal(0, 0.008, scan.shape)
            self.assertEqual(
                tracker.add_scan(
                    noisy,
                    facing=np.eye(3),
                    translation_hint=np.array([0.0, 0.0, -0.8]),
                ),
                "aligned",
            )
            np.testing.assert_allclose(tracker.translation, translation, atol=0.05)
        self.assertLess(len(tracker.map), seed_count * 1.2)

    def test_a_lift_stays_on_one_room(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        self.assertEqual(tracker.add_scan(original), "seed")
        seed_count = len(tracker.map)
        translation = np.array([0.0, 0.0, 0.22])
        scan = self._moved(original, np.eye(3), translation)
        self.assertEqual(tracker.add_scan(scan, facing=np.eye(3)), "aligned")
        np.testing.assert_allclose(tracker.translation, translation, atol=0.04)
        self.assertLess(len(tracker.map), seed_count * 1.15)
        floor = tracker.map.xyz()
        floor = floor[floor[:, 2] < 0.05]
        self.assertLess(np.median(np.abs(floor[:, 2])), 0.02)

    def test_walking_and_lifting_follow_the_lidar(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        poses = (
            (np.eye(3), np.array([0.00, 0.00, 0.15])),
            (np.eye(3), np.array([0.16, 0.00, 0.30])),
            (yaw(np.deg2rad(25)), np.array([0.28, -0.08, 0.42])),
            (pitch(np.deg2rad(12)), np.array([0.28, -0.08, 0.20])),
        )
        for rotation, translation in poses:
            scan = self._moved(original, rotation, translation)
            self.assertEqual(
                tracker.add_scan(scan, facing=rotation),
                "aligned",
            )
            np.testing.assert_allclose(tracker.translation, translation, atol=0.05)
            self.assertLess(_rotation_error(tracker.rotation, rotation), 8.0)
        cloud = tracker.map.xyz()
        self.assertLess(len(cloud), seed_count * 1.25)
        walls = cloud[(cloud[:, 2] > 0.05) & (np.linalg.norm(cloud[:, :2], axis=1) > 1.2)]
        self.assertLess(np.percentile(_wall_gap(walls), 90), 0.08)
        floor = cloud[np.abs(cloud[:, 2]) < 0.08]
        self.assertGreater(len(floor), 40)

    def test_a_turn_while_lifting_does_not_need_a_perfect_guess(self):
        # This used to be dropped: the guess stayed at the origin, and the
        # combined turn and step was too far for the match gate.
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        rotation = yaw(np.deg2rad(40))
        translation = np.array([0.20, 0.10, 0.25])
        scan = self._moved(original, rotation, translation)
        self.assertEqual(tracker.add_scan(scan, facing=rotation), "aligned")
        np.testing.assert_allclose(tracker.translation, translation, atol=0.05)
        self.assertLess(len(tracker.map), seed_count * 1.15)

    def test_steady_motion_is_carried_to_the_next_sweep(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.2)
        tracker.add_scan(original)
        first = np.array([0.0, 0.0, 0.12])
        self.assertEqual(
            tracker.add_scan(self._moved(original, np.eye(3), first), facing=np.eye(3)),
            "aligned",
        )
        # The next lift is 0.25 m, past the 0.2 m match distance from the last
        # pose. The carried step puts the guess close enough to seat it.
        second = np.array([0.0, 0.0, 0.37])
        self.assertEqual(
            tracker.add_scan(self._moved(original, np.eye(3), second), facing=np.eye(3)),
            "aligned",
        )
        np.testing.assert_allclose(tracker.translation, second, atol=0.05)

    def test_a_wrong_lift_hint_does_not_raise_the_floor(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        translation = np.array([0.0, 0.0, 0.05])
        scan = self._moved(original, np.eye(3), translation)
        self.assertEqual(
            tracker.add_scan(scan, facing=np.eye(3), translation_hint=np.array([0.0, 0.0, 0.8])),
            "aligned",
        )
        np.testing.assert_allclose(tracker.translation, translation, atol=0.05)

    def test_accel_integration_removes_gravity(self):
        imu = ImuTrack()
        imu.push_orientation((0.0, 0.0, 0.0, 1.0))
        for step in range(10):
            imu.push_accel(step * 0.05, (0.0, 0.0, 9.81))
        imu.lock_origin()
        imu.lock_gravity()
        np.testing.assert_allclose(imu.gravity, (0.0, 0.0, 9.81), atol=1e-6)
        for step in range(11):
            imu.push_accel(1.0 + step * 0.05, (0.0, 0.0, 11.81))
        shift = imu.shift_since(1.0, 1.5)
        np.testing.assert_allclose(shift, (0.0, 0.0, 0.25), atol=0.02)

    def test_gyro_integration_tracks_a_steady_turn(self):
        imu = ImuTrack()
        rate = np.array([0.0, 0.0, np.deg2rad(90)])
        for step in range(11):
            imu.push(step * 0.1, rate)
        np.testing.assert_allclose(imu.rotation_since(0.0, 1.0), yaw(np.deg2rad(90)), atol=1e-6)

    def test_quaternion_yaw_is_the_heading(self):
        half = np.sqrt(0.5)
        rotation = quat_xyzw_to_rot((0.0, 0.0, half, half))
        self.assertAlmostEqual(heading_deg(rotation), 90.0, delta=1e-6)
        np.testing.assert_allclose(rotation, yaw(np.deg2rad(90)), atol=1e-6)
        self.assertIsNone(quat_xyzw_to_rot((0.0, 0.0, 0.0, 0.0)))

    def test_facing_is_the_turn_since_the_map_started(self):
        imu = ImuTrack()
        imu.push_orientation((0.0, 0.0, 0.0, 1.0))
        self.assertIsNone(imu.facing_in_map())
        imu.lock_origin()
        self.assertAlmostEqual(heading_deg(imu.facing_in_map()), 0.0, delta=1e-6)
        half = np.deg2rad(20.0)
        imu.push_orientation((0.0, 0.0, np.sin(half), np.cos(half)))
        self.assertAlmostEqual(heading_deg(imu.facing_in_map()), 40.0, delta=1e-4)
        imu.push_orientation((0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(heading_deg(imu.facing_in_map()), 40.0, delta=1e-4)

    def test_reported_facing_places_a_turned_room_once(self):
        # The gyro reports no turn. The quaternion says the lidar has faced 40
        # degrees since the first cloud. The walls must land once, at that heading.
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        self.assertEqual(tracker.add_scan(original), "seed")
        seed_count = len(tracker.map)
        scan = apply_pose(yaw(np.deg2rad(40)).T, np.zeros(3), original)
        self.assertEqual(
            tracker.add_scan(scan, rotation_hint=np.eye(3), facing=yaw(np.deg2rad(40))),
            "aligned",
        )
        self.assertAlmostEqual(heading_deg(tracker.rotation), 40, delta=6)
        self.assertLess(len(tracker.map), seed_count * 1.15)
        cloud = tracker.map.xyz()
        walls = cloud[(cloud[:, 2] > 0.05) & (np.linalg.norm(cloud[:, :2], axis=1) > 1.2)]
        self.assertLess(np.percentile(_wall_gap(walls), 90), 0.08)

    def test_facing_is_not_stacked_on_the_previous_yaw(self):
        original = _room()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        first = apply_pose(yaw(np.deg2rad(20)).T, np.zeros(3), original)
        self.assertEqual(tracker.add_scan(first, facing=yaw(np.deg2rad(20))), "aligned")
        self.assertAlmostEqual(heading_deg(tracker.rotation), 20, delta=6)
        # Absolute facing is 55, and the gyro missed the extra turn. Adding the
        # 55 on top of the stored 20 would draw a second room.
        second = apply_pose(yaw(np.deg2rad(55)).T, np.zeros(3), original)
        self.assertEqual(
            tracker.add_scan(second, rotation_hint=np.eye(3), facing=yaw(np.deg2rad(55))),
            "aligned",
        )
        self.assertAlmostEqual(heading_deg(tracker.rotation), 55, delta=6)
        self.assertLess(len(tracker.map), seed_count * 1.25)

    def test_quaternion_facing_keeps_one_room_through_a_half_turn(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        imu = ImuTrack()
        imu.push_orientation((0.0, 0.0, 0.0, 1.0))
        self.assertEqual(tracker.add_scan(original, facing=None), "seed")
        imu.lock_origin()
        seed_count = len(tracker.map)
        for degrees in (40, 90, 180):
            half = np.deg2rad(degrees / 2.0)
            imu.push_orientation((0.0, 0.0, np.sin(half), np.cos(half)))
            scan = apply_pose(yaw(np.deg2rad(degrees)).T, np.zeros(3), original)
            status = tracker.add_scan(
                scan, rotation_hint=np.eye(3), facing=imu.facing_in_map()
            )
            self.assertEqual(status, "aligned")
            wrapped = (heading_deg(tracker.rotation) - degrees + 180.0) % 360.0 - 180.0
            self.assertLess(abs(wrapped), 6.0)
        cloud = tracker.map.xyz()
        self.assertLess(len(cloud), seed_count * 1.2)
        walls = cloud[(cloud[:, 2] > 0.05) & (np.linalg.norm(cloud[:, :2], axis=1) > 1.2)]
        self.assertLess(np.percentile(_wall_gap(walls), 90), 0.08)

    def test_a_wrong_facing_does_not_override_the_walls(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        scan = apply_pose(yaw(np.deg2rad(15)).T, np.zeros(3), original)
        self.assertEqual(
            tracker.add_scan(scan, facing=yaw(np.deg2rad(80))),
            "aligned",
        )
        self.assertAlmostEqual(heading_deg(tracker.rotation), 15, delta=8)

    def test_sensor_noise_does_not_lay_a_second_sheet(self):
        rng = np.random.default_rng(9)
        ys = np.linspace(-0.5, 0.5, 70)
        zs = np.linspace(0.2, 1.0, 36)
        yy, zz = np.meshgrid(ys, zs)
        base = np.column_stack((np.full(yy.size, 2.0), yy.ravel(), zz.ravel()))
        tracker = Tracker()
        self.assertEqual(tracker.add_scan(base), "seed")
        before = len(tracker.map)
        self.assertGreater(before, 1000)
        noisy = base + rng.normal(0.0, 0.01, base.shape)
        self.assertEqual(tracker.add_scan(noisy), "aligned")
        self.assertLess(len(tracker.map), before + before // 5)
        opened = base + np.array([0.08, 0.0, 0.0])
        self.assertGreater(float(tracker.map.novel_mask(opened).mean()), 0.9)

    def test_a_walk_without_an_imu_shift_keeps_aligning(self):
        # No translation hint: the cloud clock did not match the IMU, which is
        # what the Pi does. The sweep still has to be found and then carried.
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.45)
        tracker.add_scan(original)
        seed_count = len(tracker.map)
        translation = np.zeros(3)
        for step in (
            np.array([0.55, 0.00, 0.00]),
            np.array([0.50, 0.35, 0.15]),
            np.array([0.40, -0.25, 0.20]),
        ):
            translation = translation + step
            scan = self._moved(original, np.eye(3), translation)
            self.assertEqual(tracker.add_scan(scan, facing=np.eye(3)), "aligned")
            np.testing.assert_allclose(tracker.translation, translation, atol=0.08)
        self.assertLess(len(tracker.map), int(seed_count * 1.35) + 5)

    def test_kept_speed_reaches_a_sweep_past_the_match_gate(self):
        original = _room_with_floor()
        tracker = Tracker(voxel=0.05, max_correspondence=0.2)
        tracker.add_scan(original)
        first = np.array([0.15, 0.0, 0.0])
        self.assertEqual(
            tracker.add_scan(self._moved(original, np.eye(3), first), facing=np.eye(3)),
            "aligned",
        )
        tracker.note_elapsed(0.5)
        second = np.array([1.45, 0.0, 0.0])
        missed = Tracker(voxel=0.05, max_correspondence=0.2)
        missed.add_scan(original)
        missed.add_scan(self._moved(original, np.eye(3), first), facing=np.eye(3))
        self.assertEqual(
            missed.add_scan(self._moved(original, np.eye(3), second), facing=np.eye(3)),
            "skipped",
        )
        carried = tracker.speed * 4.0
        self.assertEqual(
            tracker.add_scan(
                self._moved(original, np.eye(3), second),
                facing=np.eye(3),
                carried=carried,
            ),
            "aligned",
        )
        np.testing.assert_allclose(tracker.translation, second, atol=0.08)

    def test_imu_motion_uses_the_imu_clock_not_the_cloud_stamp(self):
        imu = ImuTrack()
        imu.push_orientation((0.0, 0.0, 0.0, 1.0))
        for step in range(8):
            imu.push_accel(step * 0.05, (0.0, 0.0, 9.81))
        imu.lock_origin()
        imu.lock_gravity()
        imu.advance()
        for step in range(11):
            imu.push_accel(0.40 + step * 0.05, (0.0, 0.0, 11.81))
        shift = imu.preview_shift()
        self.assertIsNotNone(shift)
        self.assertGreater(shift[2], 0.15)
        self.assertLess(abs(shift[0]) + abs(shift[1]), 0.05)
        # A cloud stamp in another epoch used to be the window, and it missed.
        self.assertIsNone(imu.shift_since(100.0, 100.5))


if __name__ == "__main__":
    unittest.main()

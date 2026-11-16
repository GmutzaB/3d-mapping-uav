"""python3 -m unittest test_drift.py from ros2_ws/pi. No ROS or lidar required.

Checks the three things that stop the map drifting once the lidar moves: the
IMU heading offset is re-measured from the walls, sweeps are straightened for
the motion during them, and a lost pose does not coast out of the room.
"""

import unittest

import numpy as np

import stable_map
from sim_l2 import Path, rot_to_quat_xyzw, rot_x, rot_z, run
from stable_map import ImuTrack, deskew, sweep_offsets


def _still_imu(seconds=1.0, rate=200.0, heading=0.0):
    imu = ImuTrack()
    for stamp in np.arange(0.0, seconds, 1.0 / rate):
        imu.push(stamp, np.zeros(3))
        imu.push_orientation(rot_to_quat_xyzw(rot_z(heading)), stamp, stamp)
        imu.push_accel(stamp, [0.0, 0.0, 9.81])
    return imu


class HeadingCorrection(unittest.TestCase):
    def test_drifted_heading_is_pulled_back_to_what_the_walls_say(self):
        imu = _still_imu()
        imu.lock_origin()
        imu.lock_gravity()
        # The IMU now reports 5 degrees of turn that never happened.
        imu.push_orientation(rot_to_quat_xyzw(rot_z(np.radians(5))), 2.0, 2.0)
        self.assertAlmostEqual(stable_map.heading_deg(imu.facing_in_map()), 5.0, places=6)
        fixed = imu.correct(np.eye(3), 2.0)
        self.assertAlmostEqual(fixed, -5.0, places=5)
        np.testing.assert_allclose(imu.facing_in_map(), np.eye(3), atol=1e-9)
        # The offset stays: a real 10 degree turn on top of the drift reads as 10.
        imu.push_orientation(rot_to_quat_xyzw(rot_z(np.radians(15))), 3.0, 3.0)
        self.assertAlmostEqual(stable_map.heading_deg(imu.facing_in_map()), 10.0, places=5)

    def test_a_tilted_match_does_not_lean_the_imu(self):
        imu = _still_imu()
        imu.lock_origin()
        imu.lock_gravity()
        leaning = rot_z(np.radians(4)) @ rot_x(np.radians(3))
        self.assertAlmostEqual(imu.correct(leaning, 0.9), 4.0, delta=0.2)
        facing = imu.facing_in_map()
        self.assertAlmostEqual(stable_map.heading_deg(facing), 4.0, delta=0.2)
        self.assertLess(abs(facing[2, 1]), 1e-6)     # still level

    def test_orientation_is_read_at_the_time_asked_for(self):
        imu = ImuTrack()
        for step in range(100):
            stamp = step * 0.01
            imu.push_orientation(rot_to_quat_xyzw(rot_z(stamp)), stamp, stamp + 5.0)
        self.assertAlmostEqual(stable_map.heading_deg(imu.orientation_at(0.50)), np.degrees(0.50), places=4)
        self.assertAlmostEqual(stable_map.heading_deg(imu.orientation_at(9.0)), np.degrees(0.99), places=4)

    def test_cloud_arrival_is_placed_on_the_imu_clock(self):
        imu = ImuTrack()
        rng = np.random.default_rng(0)
        for step in range(200):
            stamp = 1000.0 + step * 0.005
            late = 0.0 if step % 7 == 0 else rng.uniform(0, 0.2)   # most messages queue up
            imu.push_orientation((0, 0, 0, 1), stamp, 50.0 + step * 0.005 + late)
        self.assertAlmostEqual(imu.imu_time(50.6), 1000.6, places=6)


class Deskew(unittest.TestCase):
    def test_packet_restarts_become_time_across_the_sweep(self):
        one = np.arange(300) * (0.0046875 / 300)
        offsets = sweep_offsets(np.tile(one, 72))
        self.assertAlmostEqual(offsets[-1], 0.0, delta=2e-5)
        self.assertAlmostEqual(offsets[0], -72 * 0.0046875, delta=1e-3)
        self.assertTrue(np.all(np.diff(offsets) > 0))
        self.assertIsNone(sweep_offsets(None))
        self.assertIsNone(sweep_offsets(np.zeros(50)))

    def test_a_wall_seen_while_turning_is_straight_again(self):
        rate = np.radians(90.0)                      # a quarter turn a second
        imu = ImuTrack()
        for stamp in np.arange(0.0, 1.0, 0.004):
            imu.push_orientation(rot_to_quat_xyzw(rot_z(rate * stamp)), stamp, stamp)
        packets, per = 72, 40
        offsets = np.repeat((np.arange(packets) - packets + 1) * 0.0047, per)
        end = 0.6
        rng = np.random.default_rng(1)
        wall = np.column_stack((np.full(packets * per, 3.0), rng.uniform(-1, 1, packets * per),
                                rng.uniform(0, 2, packets * per)))
        # Each point as the turning lidar saw it at its own moment.
        seen = np.vstack([rot_z(rate * (end + t)).T @ point for point, t in zip(wall, offsets)])
        at_end = wall @ rot_z(rate * end)            # the same wall in the frame at the end of the sweep
        self.assertGreater(np.abs(seen - at_end).max(), 1.0)
        np.testing.assert_allclose(deskew(seen, offsets, imu, end), at_end, atol=0.02)

    def test_travel_during_the_sweep_is_taken_out(self):
        imu = ImuTrack()
        for stamp in np.arange(0.0, 1.0, 0.004):
            imu.push_orientation((0, 0, 0, 1), stamp, stamp)
        offsets = np.linspace(-0.3, 0.0, 200)
        velocity = np.array([2.0, 0.0, 0.0])
        wall = np.column_stack((np.full(200, 4.0), np.linspace(-1, 1, 200), np.ones(200)))
        seen = wall - velocity * (0.3 + offsets)[:, None]      # the lidar is flying at the wall
        fixed = deskew(seen, offsets, imu, 0.5, velocity=velocity, facing=np.eye(3))
        np.testing.assert_allclose(fixed[:, 0], 4.0 - 0.6, atol=0.01)


class Coasting(unittest.TestCase):
    def test_a_measured_stop_ends_the_carried_speed(self):
        imu = _still_imu(seconds=0.5)
        imu.lock_origin()
        imu.lock_gravity()
        for stamp in np.arange(0.5, 1.0, 0.005):               # brake at 2 m/s^2 for half a second
            imu.push_orientation((0, 0, 0, 1), stamp, stamp)
            imu.push_accel(stamp, [-2.0, 0.0, 9.81])
        moved, slowed = imu.motion_since(0.5, 1.0)
        self.assertAlmostEqual(slowed[0], -1.0, delta=0.03)
        self.assertAlmostEqual(moved[0], -0.25, delta=0.02)


class CarriedLoop(unittest.TestCase):
    """The simulated L2 carried quickly round a room, worked on at every other sweep like a busy Pi."""

    def test_room_stays_one_room_with_a_drifting_imu(self):
        result = run(Path(hold=2.0, loop=10.0), yaw_drift_deg_s=0.5, every=2)
        self.assertEqual(result["counts"]["tracking"] + result["counts"]["skipped"], 0)
        self.assertLess(result["heading_error_max_deg"], 2.5)
        self.assertLess(result["position_error_max_cm"], 12.0)
        self.assertLess(result["map_error_p95_cm"], 4.0)
        # The walls found the drift the IMU could not see.
        self.assertAlmostEqual(result["imu_heading_fix_deg"], -result["true_imu_drift_deg"], delta=1.5)

    def test_same_carry_without_the_fixes_smears_the_room(self):
        result = run(Path(hold=2.0, loop=10.0), yaw_drift_deg_s=0.5, every=2,
                     deskew_sweeps=False, correct_heading=False)
        self.assertGreater(result["map_error_p95_cm"], 8.0)

    def test_a_square_room_is_not_accepted_a_quarter_turn_round(self):
        tracker = stable_map.Tracker(voxel=0.05, max_correspondence=0.45)
        rng = np.random.default_rng(3)
        span, height = rng.uniform(-2, 2, 600), rng.uniform(0, 1.5, 600)
        room = np.vstack((
            np.column_stack((np.full(600, 2.0), span, height)), np.column_stack((np.full(600, -2.0), span, height)),
            np.column_stack((span, np.full(600, 2.0), height)), np.column_stack((span, np.full(600, -2.0), height)),
        ))
        tracker.add_scan(room)
        turned = room @ rot_z(np.radians(90))          # looks identical, but claims a quarter turn
        status = tracker.add_scan(turned, facing=rot_z(np.radians(90)), facing_tolerance=np.radians(8))
        self.assertEqual(status, "aligned")           # the IMU agrees with the turn: fine
        status = tracker.add_scan(turned, facing=np.eye(3), facing_tolerance=np.radians(8))
        if status == "aligned":                        # only acceptable close to the IMU facing
            self.assertLess(abs(stable_map.heading_deg(tracker.rotation)), 8.5)


if __name__ == "__main__":
    unittest.main()

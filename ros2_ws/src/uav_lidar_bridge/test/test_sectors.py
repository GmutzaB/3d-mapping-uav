"""Run on any machine: cd ros2_ws/src/uav_lidar_bridge && python -m unittest discover -s test"""
import math
import unittest
import numpy as np
from uav_lidar_bridge.sectors import SectorMemory, nearest_per_sector, to_body_flu

LIMITS = dict(z_min=-0.3, z_max=0.3, range_min=0.35, range_max=30.0)


class SectorTests(unittest.TestCase):
    def test_directions_are_clockwise_from_forward(self):
        points = [(2, 0, 0), (0, -3, 0), (-4, 0, 0), (0, 5, 0)]  # front, right, back, left
        nearest = nearest_per_sector(points, **LIMITS)
        self.assertEqual(nearest[0], 2)
        self.assertEqual(nearest[18], 3)
        self.assertEqual(nearest[36], 4)
        self.assertEqual(nearest[54], 5)
        self.assertEqual(np.isfinite(nearest).sum(), 4)

    def test_closest_point_wins_and_filters_apply(self):
        points = [(5, 0, 0), (2, 0.05, 0.1), (1, 0, 1.0), (0.2, 0, 0), (40, 0, 0), (np.nan, 0, 0)]
        nearest = nearest_per_sector(points, **LIMITS)
        self.assertAlmostEqual(nearest[0], math.hypot(2, 0.05))
        self.assertEqual(np.isfinite(nearest).sum(), 1)

    def test_empty_cloud_is_unknown_everywhere(self):
        self.assertTrue(np.isnan(nearest_per_sector(np.empty((0, 3)), **LIMITS)).all())

    def test_mounting(self):
        yawed_left = to_body_flu([(2, 0, 0)], (0, 0, 0), (0, 0, math.radians(90)))
        self.assertTrue(np.isfinite(nearest_per_sector(yawed_left, **LIMITS)[54]))
        upside_down = to_body_flu([(0, 2, 0.1)], (0.1, 0, 0), (math.pi, 0, 0))
        np.testing.assert_allclose(upside_down, [[0.1, -2, -0.1]], atol=1e-12)

    def test_memory_keeps_closest_then_expires(self):
        memory = SectorMemory(0.3)
        first, second = np.full(72, np.nan), np.full(72, np.nan)
        first[0], second[0], second[10] = 2.0, 5.0, 7.0
        memory.update(first, 10.0)
        merged = memory.update(second, 10.1)
        self.assertEqual((merged[0], merged[10]), (2.0, 7.0))
        later = memory.update(np.full(72, np.nan), 10.5)
        self.assertTrue(np.isnan(later).all())


if __name__ == '__main__':
    unittest.main()

import unittest
import numpy as np

from battery_endurance import Battery, level_flight, Airframe
from survey_flight import Survey, coverage, fly, metrics, reference, speed_sweep, trapezoid, waypoints

SMALL = Survey(width_m=30, length_m=20, line_spacing_m=10, swath_m=12, altitude_m=6)


class PlanTests(unittest.TestCase):
    def test_lawnmower_covers_every_line(self):
        wps = waypoints(SMALL)
        easts = sorted(set(np.round(wps[2:-2, 1], 6)))
        self.assertEqual(easts, [0, 10, 20, 30])
        np.testing.assert_allclose(wps[0, :3], wps[-1, :3] + [0, 0, 0.3])   # lands where it started
        self.assertTrue(np.all(wps[1:-1, 2] == -SMALL.altitude_m))

    def test_trapezoid_respects_limits(self):
        s, v, a = trapezoid(20.0, 5.0, 1.5, 0.01)
        self.assertAlmostEqual(s[-1], 20.0, delta=0.06)
        self.assertLessEqual(v.max(), 5.0 + 1e-9)
        self.assertLessEqual(abs(a).max(), 1.5)
        s, v, _ = trapezoid(2.0, 5.0, 1.5, 0.01)        # too short to reach full speed
        self.assertLess(v.max(), 5.0)

    def test_reference_is_continuous(self):
        ref = reference(SMALL, 0.02)
        jumps = np.linalg.norm(np.diff(ref['pos'], axis=0), axis=1)
        self.assertLess(jumps.max(), SMALL.transit_speed_m_s * 0.02 + 1e-6)
        self.assertTrue(ref['surveying'].any())

    def test_coverage(self):
        s = Survey(width_m=10, length_m=10, swath_m=4, line_spacing_m=4)
        line = np.column_stack([np.linspace(0, 10, 50), np.full(50, 5.0)])
        frac, _, _ = coverage(s, line)
        self.assertAlmostEqual(frac, 0.4, delta=0.05)    # 4 m strip of a 10 m square


class FlightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calm = fly(SMALL, 0.0)
        cls.windy = fly(SMALL, 6.0)

    def test_calm_flight_follows_plan_and_covers_area(self):
        m = metrics(self.calm, SMALL, Battery())
        self.assertLess(m['max_line_error_m'], 0.5)
        self.assertLess(m['max_altitude_error_m'], 0.3)
        self.assertEqual(m['saturated_samples'], 0)
        self.assertGreater(m['coverage'], 0.99)
        final = self.calm['state'][-1, :3]
        np.testing.assert_allclose(final, self.calm['waypoints'][-1, :3], atol=0.3)

    def test_battery_bookkeeping(self):
        d, b = self.calm, Battery()
        dt = d['time'][1] - d['time'][0]
        used_ah = d['current_a'][:-1].sum() * dt / 3600
        self.assertAlmostEqual(1 - d['soc'][-1], used_ah / b.capacity_ah, places=6)
        hover_w = float(level_flight(Airframe(), b, 0.0)['power_w'])
        hold = d['time'] < 1.0                              # sitting on the pad, hover thrust
        self.assertAlmostEqual(d['power_w'][hold].mean(), hover_w, delta=0.02 * hover_w)

    def test_wind_is_handled(self):
        m = metrics(self.windy, SMALL, Battery())
        self.assertLess(m['max_line_error_m'], 1.0)
        self.assertGreater(m['max_tilt_deg'], metrics(self.calm, SMALL, Battery())['max_tilt_deg'])
        self.assertTrue(m['within_reserve'])

    def test_speed_sweep_has_a_best_speed(self):
        sweep = speed_sweep(SMALL, Airframe(), Battery())
        best = sweep[np.argmin(sweep[:, 3]), 0]
        self.assertTrue(sweep[0, 0] < best < sweep[-1, 0])


if __name__ == '__main__':
    unittest.main()

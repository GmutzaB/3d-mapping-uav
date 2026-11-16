import unittest
import numpy as np
from gust_rejection import (dryden_parameters, dryden_filter, discretize, dryden_series, lyapunov,
                            NOISE_INTENSITY, one_minus_cosine, WindQuad, WindSettings, wind_field,
                            run, metrics, PowerModel, _expm, G)


class DrydenTests(unittest.TestCase):
    def test_spec_values_at_reference_height(self):
        p = dryden_parameters(20 * 0.3048, 10.0)
        self.assertAlmostEqual(p['sigma'][2], 1.0)
        # MIL-HDBK-1797: sigma_u/sigma_w = 1/(0.177+0.000823h)^0.4, h in ft
        self.assertAlmostEqual(p['sigma'][0], 1 / (0.177 + 0.000823 * 20) ** 0.4)
        self.assertAlmostEqual(p['length'][2], 20 * 0.3048)

    def test_altitude_clamped_to_ten_feet(self):
        self.assertEqual(dryden_parameters(0.5, 5)['altitude_ft'], 10.0)

    def test_filters_have_exact_stationary_sigma(self):
        for transverse in (False, True):
            A, B, C = dryden_filter(1.3, 25.0, 8.0, transverse)
            P = lyapunov(A, NOISE_INTENSITY * B @ B.T)
            self.assertAlmostEqual((C @ P @ C.T).item(), 1.3**2, places=9)

    def test_discrete_noise_preserves_stationary_covariance(self):
        A, B, _ = dryden_filter(1.0, 25.0, 8.0, True)
        Ad, Qd = discretize(A, B, .005)
        P = lyapunov(A, NOISE_INTENSITY * B @ B.T)
        np.testing.assert_allclose(Ad @ P @ Ad.T + Qd, P, rtol=1e-8, atol=1e-12)

    def test_expm_matches_scalar_and_rotation(self):
        self.assertAlmostEqual(_expm(np.array([[-2.0]]))[0, 0], np.exp(-2))
        r = _expm(np.array([[0, -1.0], [1.0, 0]]))
        np.testing.assert_allclose(r, [[np.cos(1), -np.sin(1)], [np.sin(1), np.cos(1)]], atol=1e-12)

    def test_sampled_sigma_is_close_to_spec(self):
        out, p = dryden_series(60000, .05, 5.0, 8.0, 8.0, np.random.default_rng(1))
        np.testing.assert_allclose(out.std(axis=0), p['sigma'], rtol=.15)

    def test_discrete_gust_shape(self):
        g = one_minus_cosine([0, 1, 2.5, 4, 5], 1, 3, 6)
        np.testing.assert_allclose(g, [0, 0, 6, 0, 0], atol=1e-12)

    def test_wind_is_reproducible(self):
        a, _ = wind_field(WindSettings(seconds=2), .01)
        b, _ = wind_field(WindSettings(seconds=2), .01)
        np.testing.assert_array_equal(a['total'], b['total'])


class AircraftTests(unittest.TestCase):
    def test_drag_opposes_relative_velocity(self):
        q = WindQuad(); q.wind = np.array([0, 5.0, 0])
        f = q.drag_force(np.zeros(3))
        self.assertGreater(f[1], 0)  # wind toward east pushes the aircraft east
        self.assertAlmostEqual(f[0], 0); self.assertAlmostEqual(f[2], 0)
        np.testing.assert_allclose(q.drag_force([0, 5.0, 0]), 0)

    def test_still_air_hold_stays_put(self):
        s = WindSettings(mean_speed=1, seconds=3)
        wind, _ = wind_field(s, .005)
        for k in wind: wind[k] = np.zeros_like(wind[k]) if k != 'time' else wind[k]
        d = run('position_hold', s, .005, wind)
        self.assertLess(abs(d['error']).max(), 1e-8)

    def test_steady_wind_integral_removes_offset(self):
        s = WindSettings(mean_speed=6, seconds=25)
        wind, _ = wind_field(s, .005)
        wind['total'] = wind['mean'].copy()
        d = run('position_hold', s, .005, wind)
        self.assertLess(np.linalg.norm(d['error'][-1]), .005)
        self.assertGreater(d['tilt_deg'][-1], .5)  # leans into the wind

    def test_default_scenario(self):
        s = WindSettings()
        wind, _ = wind_field(s, .005)
        hold = metrics(run('position_hold', s, .005, wind), s)
        drift = metrics(run('attitude_only', s, .005, wind), s)
        self.assertLess(hold['max_horizontal_error_m'], .5)
        self.assertLess(hold['max_altitude_error_m'], .2)
        self.assertEqual(hold['saturated_samples'], 0)
        self.assertGreater(drift['final_horizontal_error_m'], 50)

    def test_power_matches_team_spreadsheet(self):
        # Prop_and_Motor_Calcs.xlsx, 9 in prop, 18.92 N row: 279.86 W total
        self.assertAlmostEqual(float(PowerModel().power(np.full(4, 18.92 / 4))), 279.86, places=1)
        self.assertAlmostEqual(PowerModel().flight_minutes(374.27315678608954), 28.471, places=2)


if __name__ == '__main__':
    unittest.main()

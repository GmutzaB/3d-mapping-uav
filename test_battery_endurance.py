import unittest
from dataclasses import replace
import numpy as np

from battery_endurance import (Airframe, Battery, RHO, G, discharge, induced_velocity, level_flight,
                               rotor_power, study)
from gust_rejection import PowerModel


class BatteryTests(unittest.TestCase):
    def test_selected_pack(self):
        b = Battery()
        self.assertEqual((b.cells, b.capacity_ah, b.c_rating), (6, 9.0, 80.0))
        self.assertAlmostEqual(b.nominal_wh, 199.8)
        self.assertEqual(b.max_current_a, 720)

    def test_voltage_falls_with_charge_and_load(self):
        b = Battery()
        self.assertAlmostEqual(float(b.ocv(1.0)), 25.2)
        self.assertTrue(np.all(np.diff(b.ocv(np.linspace(0, 1, 50))) > 0))
        i = float(b.current_for_power(300, 0.5))
        self.assertAlmostEqual((float(b.ocv(0.5)) - i * b.resistance) * i, 300, places=6)
        self.assertTrue(np.isnan(b.current_for_power(1e6, 0.5)))   # beyond what the pack can deliver

    def test_constant_power_energy_matches_charge_used(self):
        b = Battery(reserve_soc=0.2)
        run = discharge(b, 300.0, dt=1.0)
        self.assertEqual(run['end'], 'landing reserve reached')
        charge_ah = run['current_a'][:-1].sum() / 3600
        self.assertAlmostEqual(charge_ah, 0.8 * b.capacity_ah, delta=0.01)

    def test_voltage_cutoff_can_end_flight_first(self):
        run = discharge(Battery(reserve_soc=0.0, cutoff_cell_v=3.7), 300.0)
        self.assertEqual(run['end'], 'voltage cutoff under load')


class PowerTests(unittest.TestCase):
    def test_hover_matches_existing_power_model(self):
        a, b = Airframe(), Battery()
        mass = a.dry_mass_kg + b.mass_kg
        ours = float(level_flight(a, b, 0.0)['power_w'])
        theirs = float(PowerModel().power(np.full(4, mass * G / 4)))
        self.assertAlmostEqual(ours, theirs, places=6)

    def test_inflow_limits(self):
        area = Airframe().disk_area
        vh = np.sqrt(5.0 / (2 * RHO * area))
        self.assertAlmostEqual(float(induced_velocity(5.0, 0, 0, area)), vh, places=9)
        # fast edgewise flight: v -> vh^2 / V
        self.assertAlmostEqual(float(induced_velocity(5.0, 0, 40, area)), vh ** 2 / 40, delta=0.01 * vh ** 2 / 40)
        climb = float(rotor_power(5.0, 2.0, 0, Airframe()))
        self.assertGreater(climb, float(rotor_power(5.0, 0, 0, Airframe())))

    def test_power_curve_is_bucket_shaped(self):
        r = study()
        self.assertGreater(r['best_endurance']['speed_m_s'], 3)
        self.assertGreater(r['best_range']['speed_m_s'], r['best_endurance']['speed_m_s'])
        self.assertLess(r['best_endurance']['power_w'], r['hover']['power_w'])
        self.assertGreater(r['curves']['power_w'][-1], r['hover']['power_w'])

    def test_heavier_means_shorter(self):
        sweep = study()['mass_sweep']['hover_min']
        self.assertTrue(np.all(np.diff(sweep, axis=0) < 0))   # heavier airframe
        self.assertTrue(np.all(np.diff(sweep, axis=1) < 0))   # heavier battery, same capacity

    def test_hover_time_is_plausible(self):
        r = study()
        energy_min = Battery().nominal_wh * 0.8 / r['hover']['power_w'] * 60
        self.assertAlmostEqual(r['hover']['flight_min'], energy_min, delta=0.1 * energy_min)
        self.assertLess(r['margins']['pack_limit_used_at_full_throttle'], 1)


if __name__ == '__main__':
    unittest.main()

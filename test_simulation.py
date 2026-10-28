"""Numerical checks against an exact periodic Navier–Stokes solution."""
import unittest
import numpy as np
from hardware import (
    BATTERY_CELLS,
    BATTERY_MAH_OPTIONS,
    MOTOR_KV,
    NOMINAL_CELL_VOLTAGE_V,
    PROP_DIAMETER_IN,
    nominal_energy_wh,
)
from simulation import Flow, propeller
from flight_dynamics import Parameters


class PhysicsTests(unittest.TestCase):
    def test_taylor_green_vortex(self):
        f = Flow(32, length=2*np.pi, viscosity=0.01)
        f.u = np.sin(f.X)*np.cos(f.Y)
        f.v = -np.cos(f.X)*np.sin(f.Y)
        u0, v0 = f.u.copy(), f.v.copy()
        for _ in range(100):
            f.step(0.001)
        decay = np.exp(-2*f.nu*0.1)
        np.testing.assert_allclose(f.u, u0*decay, atol=1e-9)
        np.testing.assert_allclose(f.v, v0*decay, atol=1e-9)
        self.assertLess(np.max(abs(f.divergence())), 1e-11)
        pressure = f.rhs(f.u, f.v)[2]
        expected = 1.225/4*(np.cos(2*f.X)+np.cos(2*f.Y))*decay**2
        np.testing.assert_allclose(pressure, expected, atol=1e-9)

    def test_hardware_scaling(self):
        base = propeller(0.5, motor_kv=1500, battery_cells=6)
        self.assertAlmostEqual(base['rpm'], 1500 * 22.2 * 0.8 * 0.5)
        doubled_kv = propeller(0.5, motor_kv=3000, battery_cells=6)
        self.assertAlmostEqual(doubled_kv['thrust_per_motor_N'], 4 * base['thrust_per_motor_N'])
        half_voltage = propeller(0.5, motor_kv=1500, battery_cells=3)
        self.assertAlmostEqual(half_voltage['thrust_per_motor_N'], base['thrust_per_motor_N'] / 4)

    def test_propeller_diameter_scaling(self):
        small = propeller(0.5, prop_diameter_in=5)
        large = propeller(0.5, prop_diameter_in=7)
        self.assertAlmostEqual(large['thrust_per_motor_N'] / small['thrust_per_motor_N'], (7/5)**4)
        self.assertAlmostEqual(large['ideal_disk_pressure_jump_Pa'] / small['ideal_disk_pressure_jump_Pa'], (7/5)**2)

    def test_selected_aircraft_defaults(self):
        full = propeller(1)
        self.assertEqual(MOTOR_KV, 900)
        self.assertEqual(PROP_DIAMETER_IN, 9)
        self.assertEqual(BATTERY_MAH_OPTIONS, (7000, 10000))
        self.assertAlmostEqual(full["rpm"], 900 * NOMINAL_CELL_VOLTAGE_V * BATTERY_CELLS * 0.8)
        self.assertAlmostEqual(Parameters().max_thrust, full["thrust_per_motor_N"])
        self.assertAlmostEqual(nominal_energy_wh(7000), 3.7 * 6 * 7)
        self.assertAlmostEqual(nominal_energy_wh(10000), 3.7 * 6 * 10)

    def test_throttle_squared(self):
        self.assertEqual(propeller(0)["thrust_per_motor_N"], 0)
        self.assertAlmostEqual(propeller(.5)["thrust_per_motor_N"] / propeller(1)["thrust_per_motor_N"], .25)


if __name__ == "__main__":
    unittest.main()

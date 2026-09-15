"""Numerical checks against an exact periodic Navier–Stokes solution."""
import unittest
import numpy as np
from simulation import Flow, propeller


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

    def test_throttle_squared(self):
        self.assertEqual(propeller(0)["thrust_per_motor_N"], 0)
        self.assertAlmostEqual(propeller(.5)["thrust_per_motor_N"] / propeller(1)["thrust_per_motor_N"], .25)


if __name__ == "__main__":
    unittest.main()

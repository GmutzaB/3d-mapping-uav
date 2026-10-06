import unittest
import subprocess
import sys
import tempfile
from pathlib import Path
import numpy as np
from flight_dynamics import Quad, Parameters, rotation


class DynamicsTests(unittest.TestCase):
    def test_freefall_analytic(self):
        q = Quad()
        for _ in range(500):
            q.step([0]*4,0.002)
        np.testing.assert_allclose(q.state[:6],[0,0,4.905,0,0,9.81],atol=1e-10)

    def test_hover_balance(self):
        q = Quad()
        q.state[13:] = q.p.mass*9.81/4
        for _ in range(500):
            q.step([q.hover_throttle()]*4,0.002)
        np.testing.assert_allclose(q.state[:6],0,atol=1e-10)

    def test_motor_step_analytic(self):
        q = Quad()
        for _ in range(100):
            q.step([0.5]*4,0.001)
        expected = q.p.max_thrust*0.25*(1-np.exp(-0.1/q.p.motor_tau))
        np.testing.assert_allclose(q.state[13:],expected,atol=1e-8)

    def test_rotor_moment_signs(self):
        q = Quad()
        _, torque = q.wrench([1,0,0,0])
        self.assertLess(torque[0],0)
        self.assertGreater(torque[1],0)
        self.assertGreater(torque[2],0)
        np.testing.assert_allclose(q.wrench([1]*4)[1],0,atol=1e-15)

    def test_tilted_thrust_direction(self):
        q = Quad()
        angle = np.pi/6
        q.state[6:10] = [np.cos(angle/2),0,np.sin(angle/2),0]
        q.state[13:] = q.p.mass*9.81/4
        d = q.derivative(q.state,np.full(4,q.hover_throttle()))
        np.testing.assert_allclose(d[3:6],[-9.81*np.sin(angle),0,9.81*(1-np.cos(angle))],atol=1e-12)

    def test_torque_free_rotation(self):
        q = Quad()
        q.state[12] = 1
        for _ in range(500):
            q.step([0]*4,0.002)
        np.testing.assert_allclose(q.state[6:10],[np.cos(.5),0,0,np.sin(.5)],atol=1e-11)
        np.testing.assert_allclose(rotation(q.state[6:10]).T@rotation(q.state[6:10]),np.eye(3),atol=1e-12)

    def test_cli_scenario_durations(self):
        script = Path(__file__).with_name('flight_dynamics.py')
        with tempfile.TemporaryDirectory() as folder:
            for scenario, extra, expected in [('hover', [], 2.), ('roll', [], .5),
                                              ('roll', ['--seconds', '.1'], .1)]:
                subprocess.run([sys.executable, str(script), '--scenario', scenario,
                                '--no-visuals', '--output', folder, *extra],
                               check=True, capture_output=True, text=True)
                rows = np.loadtxt(Path(folder)/f'{scenario}.csv', delimiter=',', skiprows=1)
                self.assertAlmostEqual(rows[-1, 0], expected)
                if scenario == 'hover':
                    np.testing.assert_allclose(rows[:, 1:7], 0, atol=1e-10)
                else:
                    R = rotation(rows[-1, 7:11])
                    self.assertGreater(np.arctan2(R[2, 1], R[2, 2]), 0)
                    self.assertLess(np.arctan2(R[2, 1], R[2, 2]), np.pi/4)

    def test_reject_invalid(self):
        q = Quad()
        for u,dt in [([np.nan]*4,.001),([1.1]*4,.001),([0]*3,.001),([0]*4,-1),([0]*4,1)]:
            with self.assertRaises(ValueError): q.step(u,dt)
        with self.assertRaises(ValueError): Parameters(mass=-1)
        with self.assertRaises(ValueError): Quad(Parameters(max_thrust=1)).hover_throttle()


if __name__ == '__main__':
    unittest.main()

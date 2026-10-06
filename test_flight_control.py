import unittest
import numpy as np
from flight_dynamics import Quad
from flight_control import run, metrics, angles, Controller, settling_time


class ControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = run()
        cls.report = metrics(cls.data)

    def test_ten_second_hover(self):
        self.assertLess(self.report['hover_max_height_error_m'],1e-8)
        self.assertLess(self.report['hover_max_angle_deg'],1e-8)
        self.assertEqual(self.data['time'][-1],22)

    def test_roll_tracks_and_holds(self):
        self.assertIsNotNone(self.report['roll_settling_s'])
        self.assertLess(self.report['roll_settling_s'],2)
        self.assertLess(self.report['roll_overshoot_deg'],1)
        t = self.data['time']; e = angles(self.data['state'])
        np.testing.assert_allclose(e[(t>=12)&(t<14),0],10,atol=.5)
        self.assertLess(abs(e[:,1:]).max(),.01)

    def test_returns_level(self):
        self.assertIsNotNone(self.report['return_settling_s'])
        self.assertLess(self.report['return_settling_s'],2)
        self.assertLess(self.report['return_overshoot_deg'],1)

    def test_disturbance_is_applied_and_recovers(self):
        self.assertGreater(self.report['disturbance_peak_roll_deg'],.5)
        self.assertLess(self.report['disturbance_peak_roll_deg'],10)
        self.assertIsNotNone(self.report['recovery_settling_s'])
        self.assertLess(self.report['recovery_settling_s'],2)
        np.testing.assert_allclose(angles(self.data['state'])[-1],0,atol=.05)
        self.assertAlmostEqual(np.sum(self.data['disturbance'][:-1,0]*np.diff(self.data['time'])),.12*.15)

    def test_height_and_actuator_limits(self):
        self.assertLess(self.report['max_height_excursion_m'],.05)
        self.assertLess(abs(self.report['final_height_change_m']),.005)
        self.assertEqual(self.report['saturated_samples'],0)
        self.assertTrue(np.all((self.data['throttle']>=0)&(self.data['throttle']<=1)))
        self.assertGreater(self.report['final_horizontal_displacement_m'],1)

    def test_uncontrolled_has_same_disturbance_and_no_recovery(self):
        other = run(False)
        np.testing.assert_array_equal(other['disturbance'],self.data['disturbance'])
        np.testing.assert_array_equal(other['state'][0],self.data['state'][0])
        np.testing.assert_allclose(other['throttle'],Quad().hover_throttle())
        self.assertIsNone(metrics(other)['recovery_settling_s'])
        self.assertGreater(abs(other['state'][-1,10]),.5)

    def test_step_refinement(self):
        fine = run(dt=.0025)
        np.testing.assert_allclose(fine['state'][-1,:6],self.data['state'][-1,:6],atol=.02)
        self.assertLess(abs(metrics(fine)['roll_settling_s']-self.report['roll_settling_s']),.03)

    def test_invalid_commands_and_disturbance(self):
        q = Quad(); c = Controller(q)
        with self.assertRaises(ValueError): c.command(q.state,np.nan)
        with self.assertRaises(ValueError): c.command(q.state,np.pi)
        with self.assertRaises(ValueError): q.step([.5]*4,.005,[np.nan,0,0])

    def test_unsettled_metrics_do_not_report_false_success(self):
        self.assertIsNone(settling_time(np.arange(4.),np.array([0,0,0,2]),0,0,4))
        self.assertIsNone(settling_time(np.arange(4.),np.array([2,2,2,0]),0,0,4))


if __name__ == '__main__': unittest.main()

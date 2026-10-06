import unittest
from dataclasses import replace
import numpy as np
from src.companion.frames import enu_to_ned, flu_to_frd, attitude_enu_flu_to_ned_frd, covariance_enu_to_ned
from src.companion.supervisor import Supervisor, Health, VelocityPlan, State


class FrameTests(unittest.TestCase):
    def test_axes(self):
        np.testing.assert_array_equal(enu_to_ned([1,2,3]),[2,1,-3])
        np.testing.assert_array_equal(flu_to_frd([1,2,3]),[1,-2,-3])

    def test_east_facing_ros_identity(self):
        matrix = attitude_enu_flu_to_ned_frd([1,0,0,0])
        # ROS identity points forward east: aircraft forward must become NED east.
        np.testing.assert_array_equal(matrix@[1,0,0],[0,1,0])
        np.testing.assert_array_equal(matrix@[0,0,1],[0,0,1])
        self.assertAlmostEqual(np.linalg.det(matrix),1)

    def test_covariance(self):
        np.testing.assert_array_equal(covariance_enu_to_ned(np.diag([1,4,9])),np.diag([4,1,9]))
        with self.assertRaises(ValueError): covariance_enu_to_ned(np.diag([-1,1,1]))


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.supervisor = Supervisor()
        self.health = Health(10,10,True,True,True)
        self.plan = VelocityPlan(10,10.3,(0.5,0,0),collision_checked=True)

    def start(self):
        return self.supervisor.tick(10,self.health,self.plan,enable=True)

    def test_explicit_enable(self):
        self.assertEqual(self.supervisor.tick(10,self.health,self.plan).state,State.DISABLED)
        self.assertEqual(self.start().velocity,(0.5,0,0))

    def test_stale_plan_holds_and_latches(self):
        self.start()
        h = replace(self.health,heartbeat_at=10.4,odometry_at=10.4)
        self.assertEqual(self.supervisor.tick(10.4,h,self.plan).velocity,(0,0,0))
        p = replace(self.plan,created_at=10.4,expires_at=10.7)
        self.assertEqual(self.supervisor.tick(10.4,h,p).state,State.HOLD)
        self.assertEqual(self.supervisor.tick(10.4,h,p,enable=True).state,State.ACTIVE)

    def test_localization_loss_never_claims_hold(self):
        self.start()
        self.assertIsNone(self.supervisor.tick(10.21,self.health,self.plan).velocity)
        self.assertEqual(self.supervisor.state,State.FAULT)
        h = replace(self.health,odometry_at=10.22)
        self.assertEqual(self.supervisor.tick(10.22,h,self.plan).state,State.FAULT)

    def test_bad_commands(self):
        for p in [replace(self.plan,velocity=(float('nan'),0,0)),
                  replace(self.plan,velocity=(.8,.8,0)),replace(self.plan,frame='ENU'),
                  replace(self.plan,collision_checked=False),replace(self.plan,created_at=11),
                  replace(self.plan,expires_at=10),replace(self.plan,estimator_epoch=1)]:
            with self.subTest(plan=p):
                self.assertEqual(Supervisor().tick(10,self.health,p,enable=True).state,State.HOLD)

    def test_link_mode_and_clock_failures(self):
        for h in [replace(self.health,heartbeat_at=8),replace(self.health,heartbeat_at=11),
                  replace(self.health,guided=False),replace(self.health,armed=False)]:
            self.assertEqual(Supervisor().tick(10,h,self.plan,enable=True).state,State.FAULT)
        self.start()
        self.assertEqual(self.supervisor.tick(9,self.health,self.plan).state,State.FAULT)

    def test_reset_requires_operator_and_new_plan(self):
        self.start()
        h = replace(self.health,estimator_epoch=1)
        self.assertEqual(self.supervisor.tick(10,h,self.plan).state,State.HOLD)
        self.assertEqual(self.supervisor.tick(10,h,self.plan,enable=True).state,State.HOLD)
        p = replace(self.plan,estimator_epoch=1)
        self.assertEqual(self.supervisor.tick(10,h,p,enable=True).state,State.ACTIVE)

    def test_hold_escalates_when_localization_lost(self):
        self.start()
        self.assertEqual(self.supervisor.tick(10,self.health,None).state,State.HOLD)
        result = self.supervisor.tick(10,replace(self.health,localization_valid=False),None)
        self.assertEqual(result.state,State.FAULT)
        self.assertIsNone(result.velocity)


class WireTests(unittest.TestCase):
    def setUp(self):
        try:
            from pymavlink.dialects.v20 import ardupilotmega
        except ImportError:
            self.skipTest('Install requirements-companion.txt for MAVLink wire verification')
        from src.companion.mavlink import Encoder
        self.mav = ardupilotmega
        self.encoder = Encoder()
        self.decoder = ardupilotmega.MAVLink(None)

    def decode(self, packet):
        self.assertEqual(packet[0],0xFD)  # MAVLink 2 framing
        messages = self.decoder.parse_buffer(packet)
        self.assertEqual(len(messages),1)
        return messages[0]

    def test_velocity_wire_contract(self):
        for n in range(2):
            message = self.decode(self.encoder.velocity((1,-.5,.25),1234,7))
            self.assertEqual(message.get_type(),'SET_POSITION_TARGET_LOCAL_NED')
            self.assertEqual(message.get_seq(),n)
            self.assertEqual(message.target_system,7)
            self.assertEqual(message.coordinate_frame,1)
            self.assertEqual(message.type_mask,3527)
            self.assertEqual((message.vx,message.vy,message.vz),(1,-.5,.25))
            self.assertEqual(message.time_boot_ms,1234)

    def test_odometry_wire_contract(self):
        cov = np.eye(6)[np.triu_indices(6)].tolist()
        message = self.decode(self.encoder.odometry(time_usec=123456,position_local_frd=(1,2,-3),
            q_body_to_local_wxyz=(1,0,0,0),velocity_body_frd=(.1,.2,.3),
            angular_velocity_body_frd=(0,0,0),pose_covariance=cov,reset_counter=4,quality=80))
        self.assertEqual(message.get_type(),'ODOMETRY')
        self.assertEqual((message.frame_id,message.child_frame_id),(20,12))
        self.assertEqual(message.reset_counter,4)
        self.assertEqual(message.quality,80)
        self.assertEqual(message.z,-3)
        self.assertTrue(np.isnan(message.velocity_covariance[0]))

    def test_replay_fault_recovery_and_wire_stream(self):
        import contextlib
        import io
        import json
        import tempfile
        from pathlib import Path
        from src.companion.replay import run
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with contextlib.redirect_stdout(io.StringIO()):
                run(output)
            events = [json.loads(line) for line in (output/'events.jsonl').read_text().splitlines()]
            messages = self.decoder.parse_buffer((output/'commands.mavlink').read_bytes())
        self.assertEqual(len(events),61)
        sent = [event for event in events if event['packet_hex']]
        self.assertEqual(len(messages),len(sent))
        for event, message in zip(sent,messages):
            self.assertEqual(message.get_type(),'SET_POSITION_TARGET_LOCAL_NED')
            self.assertEqual(message.time_boot_ms,round(event['time']*1000))
            np.testing.assert_allclose([message.vx,message.vy,message.vz],event['velocity'])
        # Planner recovery at t=2 and localization recovery at t=4 cannot resume motion.
        for event in events:
            t = event['time']
            if 1 <= t < 2.5:
                self.assertEqual(event['state'],'hold_requested')
                self.assertEqual(event['velocity'],[0,0,0])
            if 3.2 <= t < 4.5:
                self.assertEqual(event['state'],'fault_latched')
                self.assertEqual(event['packet_hex'],'')
        self.assertEqual(events[-1]['state'],'disabled')
        self.assertEqual(events[-1]['packet_hex'],'')

    def test_broadcast_and_nonfinite_rejected(self):
        with self.assertRaises(ValueError): self.encoder.velocity((0,0,0),0,0)
        with self.assertRaises(ValueError): self.encoder.velocity((float('inf'),0,0),0,1)


if __name__ == '__main__':
    unittest.main()

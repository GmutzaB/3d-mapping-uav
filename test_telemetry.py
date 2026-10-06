import unittest
from src.companion.supervisor import Supervisor, VelocityPlan, State
try:
    from pymavlink.dialects.v20 import ardupilotmega as mav
    from src.companion.telemetry import Receiver
except ImportError:
    mav = None


@unittest.skipIf(mav is None,'Install requirements-companion.txt for telemetry tests')
class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.receiver = Receiver()
        self.codec = mav.MAVLink(None,srcSystem=1,srcComponent=1)

    def heartbeat(self, mode=4, armed=True, status=4):
        flags = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
        return mav.MAVLink_heartbeat_message(mav.MAV_TYPE_QUADROTOR,mav.MAV_AUTOPILOT_ARDUPILOTMEGA,flags,mode,status,3).pack(self.codec)

    def position(self, stamp=1000, x=1):
        return mav.MAVLink_local_position_ned_message(stamp,x,2,-3,.1,.2,.3).pack(self.codec)

    def test_fragmentation_and_source_filter(self):
        packet = self.heartbeat()
        self.receiver.feed(packet[:9],1)
        self.assertIsNone(self.receiver.state.heartbeat_at)
        self.receiver.feed(packet[9:]+self.position(),1.1)
        self.assertTrue(self.receiver.state.guided)
        self.assertEqual(self.receiver.state.position_ned,(1,2,-3))
        self.codec.srcSystem = 2
        self.receiver.feed(self.heartbeat(mode=0),1.2)
        self.assertTrue(self.receiver.state.guided)
        self.assertEqual(self.receiver.state.rejected,1)

    def test_missing_localization_and_stale_heartbeat_gate(self):
        self.receiver.feed(self.heartbeat(),10)
        plan = VelocityPlan(10,10.3,(.1,0,0),collision_checked=True)
        supervisor = Supervisor()
        self.assertEqual(supervisor.tick(10,self.receiver.health(),plan,enable=True).state,State.FAULT)
        health = self.receiver.health(odometry_at=10,localization_valid=True)
        self.assertEqual(supervisor.tick(10,health,plan,enable=True).state,State.ACTIVE)
        health = self.receiver.health(odometry_at=12,localization_valid=True)
        self.assertEqual(supervisor.tick(12,health,plan).state,State.FAULT)
        self.assertFalse(self.receiver.snapshot(12)['heartbeat_fresh'])

    def test_mode_and_critical_state_remove_authority(self):
        for packet in [self.heartbeat(mode=0),self.heartbeat(status=mav.MAV_STATE_CRITICAL)]:
            self.receiver.feed(packet,1)
            health = self.receiver.health(odometry_at=1,localization_valid=True)
            self.assertFalse(health.guided)

    def test_duplicates_and_discontinuity(self):
        self.receiver.feed(self.position(),1)
        self.receiver.feed(self.position(),2)
        self.assertEqual(self.receiver.state.position_at,1)
        self.receiver.feed(self.position(stamp=100),3)
        self.assertTrue(self.receiver.state.discontinuity)
        self.assertIsNone(self.receiver.state.position_at)
        self.receiver.feed(self.position(stamp=2000),4)
        self.assertIsNone(self.receiver.state.position_at)

    def test_boot_counter_wrap(self):
        self.receiver.feed(self.position(stamp=2**32-10),1)
        self.receiver.feed(self.position(stamp=10),1.02)
        self.assertFalse(self.receiver.state.discontinuity)
        self.assertEqual(self.receiver.state.position_boot_ms,10)

    def test_receive_only_udp_monitor(self):
        import contextlib
        import io
        import json
        import socket
        from src.companion.monitor import monitor
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as inbound, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            inbound.bind(('127.0.0.1',0))
            sender.bind(('127.0.0.1',0))
            sender.sendto(self.heartbeat()+self.position(),inbound.getsockname())
            log, capture = io.StringIO(), io.StringIO()
            with contextlib.redirect_stdout(io.StringIO()):
                summary = monitor(inbound,self.receiver,.05,log,capture)
            self.assertEqual(summary['accepted'],2)
            self.assertTrue(summary['heartbeat_fresh'])
            self.assertEqual(len(capture.getvalue().splitlines()),1)
            recorded = json.loads(capture.getvalue())
            replay = Receiver()
            replay.feed(bytes.fromhex(recorded['hex']),recorded['received_at'])
            self.assertEqual(replay.state.position_ned,(1,2,-3))
            sender.settimeout(.02)
            with self.assertRaises(socket.timeout): sender.recvfrom(65535)

    def test_invalid_payload_crc_and_clock(self):
        self.receiver.feed(self.position(x=float('nan')),1)
        self.assertIsNone(self.receiver.state.position_at)
        packet = bytearray(self.heartbeat()); packet[-1] ^= 0xFF
        self.receiver.feed(packet,2)
        self.assertIsNone(self.receiver.state.heartbeat_at)
        self.assertGreater(self.receiver.state.bad_packets,0)
        with self.assertRaises(ValueError): self.receiver.feed(b'',1)
        self.receiver.feed(self.heartbeat(),3)
        self.assertEqual(self.receiver.state.heartbeat_at,3)


if __name__ == '__main__':
    unittest.main()

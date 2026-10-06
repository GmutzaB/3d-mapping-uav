"""Protocol and lifecycle regressions; fake transport tests are not SITL evidence."""
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import importlib.util
if importlib.util.find_spec('pymavlink') is None:
    raise unittest.SkipTest('Install requirements-companion.txt for mission tests')

from pymavlink.dialects.v20 import ardupilotmega as mav
from src.companion.mission import Mission, MissionError, MissionPlan, Session, healthy
from src.companion.telemetry import Telemetry


class Clock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now


class FakeSocket:
    def __init__(self, clock, results=(mav.MAV_RESULT_ACCEPTED,), wrong_source=False):
        self.clock, self.results, self.wrong_source = clock, results, wrong_source
        self.pending = deque()
        self.sent = []
        self.parser = mav.MAVLink(None)
        self.codec = mav.MAVLink(None,srcSystem=2 if wrong_source else 1,srcComponent=1)

    def settimeout(self, timeout): pass
    def close(self): pass

    def sendall(self, data):
        message = self.parser.parse_char(data)
        self.sent.append(message)
        if message.get_type() == 'COMMAND_LONG':
            for result in self.results:
                self.pending.append(mav.MAVLink_command_ack_message(message.command,result,
                                    0,0,255,190).pack(self.codec))

    def recv(self, size):
        import socket
        self.clock.now += .1
        if self.pending: return self.pending.popleft()
        raise socket.timeout()


class FakeSession:
    def __init__(self):
        self.time = 0.
        self.started = 0.
        self.phase = 'CONNECT'
        self.position = [10.,20.,-2.]
        self.home = tuple(self.position)
        self.target = tuple(self.position)
        self.receiver = SimpleNamespace(state=Telemetry(position_ned=tuple(self.position),velocity_ned=(0.,0.,0.)))
        self.prearm_ok = True
        self.prearm_at = self.landed_at = 0.
        self.landed = mav.MAV_LANDED_STATE_ON_GROUND
        self.events, self.commands, self.setpoints = [], [], []
        self.landing = False
        self.inject = None

    def clock(self): return self.time
    def event(self, kind, **fields): self.events.append((kind,fields))

    def poll(self, timeout=.1):
        self.time += .1
        state = self.receiver.state
        distance = math.dist(self.position,self.target)
        velocity = [0.,0.,0.]
        if distance > .001:
            step = min(.05,distance)
            velocity = [(b-a)/distance*step/.1 for a,b in zip(self.position,self.target)]
            self.position = [a+v*.1 for a,v in zip(self.position,velocity)]
        state.position_ned, state.velocity_ned = tuple(self.position),tuple(velocity)
        state.heartbeat_at = state.position_at = self.time
        state.system_status = mav.MAV_STATE_ACTIVE if state.armed else mav.MAV_STATE_STANDBY
        self.prearm_at = self.landed_at = self.time
        if self.landing and math.dist(self.position,self.home) < .001:
            state.armed = False
            self.landed = mav.MAV_LANDED_STATE_ON_GROUND
        if self.inject: self.inject(self)

    def command(self, command, *params, **kwargs):
        self.commands.append((command,params))
        if command == mav.MAV_CMD_DO_SET_MODE: self.receiver.state.guided = True
        if command == mav.MAV_CMD_COMPONENT_ARM_DISARM: self.receiver.state.armed = True
        if command == mav.MAV_CMD_NAV_TAKEOFF:
            self.target = (*self.home[:2], self.home[2]-params[6])
            self.landed = mav.MAV_LANDED_STATE_IN_AIR
        if command == mav.MAV_CMD_NAV_LAND:
            self.home = (*self.position[:2],self.home[2])
            self.target = self.home
            self.landing = True
            self.receiver.state.guided = False

    def position_target(self, target):
        self.setpoints.append((self.phase,tuple(target)))
        self.target = tuple(target)


class MissionTests(unittest.TestCase):
    def test_invalid_plan(self):
        for kwargs in [dict(altitude=math.nan),dict(altitude=0),dict(north=11),
                       dict(east=math.inf),dict(speed=2),dict(hold_seconds=0),dict(phase_timeout=0)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError): MissionPlan(**kwargs)

    def test_full_lifecycle_and_relative_ned_target(self):
        session = FakeSession()
        result = Mission(session,MissionPlan(hold_seconds=1,phase_timeout=20)).run()
        self.assertTrue(result['passed'])
        self.assertEqual(result['targets_ned']['HOVER'], [10.,20.,-5.])
        self.assertEqual(result['targets_ned']['WAYPOINT'], [13.,22.,-5.])
        self.assertTrue(all(p['settled_hold_s'] >= 1 for p in result['phases']))
        self.assertEqual(session.commands[-1][0],mav.MAV_CMD_NAV_LAND)
        arm = next(p for c,p in session.commands if c == mav.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual(arm,(1,0))  # Never force arm.
        self.assertFalse(session.receiver.state.armed)
        self.assertEqual(session.landed,mav.MAV_LANDED_STATE_ON_GROUND)
        self.assertFalse(any(phase == 'TAKEOFF' for phase,_ in session.setpoints))

    def test_stale_position_stops_setpoints(self):
        session = FakeSession()
        session.inject = lambda s: setattr(s.receiver.state,'position_at',0.) if s.phase=='HOVER' else None
        with self.assertRaisesRegex(MissionError,'position stale'):
            Mission(session,MissionPlan(hold_seconds=1)).run()
        self.assertEqual(session.setpoints,[])

    def test_mode_change_stops_setpoints_and_abort_does_not_override(self):
        session = FakeSession()
        session.inject = lambda s: setattr(s.receiver.state,'guided',False) if s.phase=='HOVER' else None
        mission = Mission(session,MissionPlan(hold_seconds=1))
        with self.assertRaisesRegex(MissionError,'left GUIDED'): mission.run()
        previous = len(session.commands)
        mission.abort('mode change')
        self.assertEqual(previous,len(session.commands))
        self.assertEqual(session.setpoints,[])

    def test_hold_timer_restarts_after_excursion(self):
        session = FakeSession()
        disturbed_at = []
        hover_started = []
        def disturb(s):
            if s.phase != 'HOVER':
                return
            if not hover_started:
                hover_started.append(s.time)
            if not disturbed_at and s.time-hover_started[0] >= .5:
                s.receiver.state.velocity_ned = (.4,0.,0.)
                disturbed_at.append(s.time)
        session.inject = disturb
        result = Mission(session,MissionPlan(hold_seconds=1,phase_timeout=20)).run()
        hover = next(p for p in result['phases'] if p['phase'] == 'HOVER')
        self.assertTrue(disturbed_at)
        self.assertGreaterEqual(hover['duration_s'],1.5)

    def test_never_settles_times_out(self):
        session = FakeSession()
        session.inject = lambda s: setattr(s.receiver.state,'velocity_ned',(1.,0.,0.))
        with self.assertRaisesRegex(MissionError,'did not settle'):
            Mission(session,MissionPlan(hold_seconds=1,phase_timeout=10)).run()

    def test_disarmed_without_ground_confirmation_is_not_success(self):
        session = FakeSession()
        session.inject = lambda s: setattr(s,'landed',mav.MAV_LANDED_STATE_IN_AIR) if s.landing else None
        with self.assertRaisesRegex(MissionError,'landing and automatic disarm'):
            Mission(session,MissionPlan(hold_seconds=1,phase_timeout=20)).run()

    def test_bound_violation(self):
        session = FakeSession()
        mission = Mission(session,MissionPlan())
        mission.origin = (0,0,0)
        with self.assertRaisesRegex(MissionError,'boundary'): mission.in_bounds()

    def test_stale_heartbeat_and_clock_reset(self):
        state = Telemetry(heartbeat_at=0,position_at=5,armed=True,guided=True)
        with self.assertRaisesRegex(MissionError,'heartbeat stale'): healthy(state,5)
        state.heartbeat_at = 5
        state.discontinuity = True
        with self.assertRaisesRegex(MissionError,'clock changed'): healthy(state,5)

    def test_protocol_ack_transactions(self):
        cases = [((mav.MAV_RESULT_ACCEPTED,),False,None),
                 ((mav.MAV_RESULT_IN_PROGRESS,mav.MAV_RESULT_ACCEPTED),False,None),
                 ((mav.MAV_RESULT_DENIED,),False,'rejected'),
                 ((),False,'timed out'),
                 ((mav.MAV_RESULT_ACCEPTED,),True,'timed out')]
        for results, wrong_source, error in cases:
            with self.subTest(results=results,wrong_source=wrong_source), tempfile.TemporaryDirectory() as folder:
                clock = Clock()
                sock = FakeSocket(clock,results,wrong_source)
                session = Session(sock,Path(folder),clock)
                try:
                    if error:
                        with self.assertRaisesRegex(MissionError,error):
                            session.command(mav.MAV_CMD_COMPONENT_ARM_DISARM,1,0,timeout=.5)
                    else:
                        session.command(mav.MAV_CMD_COMPONENT_ARM_DISARM,1,0,timeout=.5)
                    sent = next(m for m in sock.sent if m.get_type() == 'COMMAND_LONG')
                    self.assertEqual((sent.target_system,sent.target_component),(1,1))
                finally: session.close()

    def test_position_packet_frame_and_mask(self):
        with tempfile.TemporaryDirectory() as folder:
            clock = Clock()
            sock = FakeSocket(clock)
            session = Session(sock,Path(folder),clock)
            try:
                session.position_target((13,22,-5))
                message = sock.sent[-1]
                self.assertEqual(message.coordinate_frame,mav.MAV_FRAME_LOCAL_NED)
                self.assertEqual(message.type_mask,3576)
                self.assertEqual((message.x,message.y,message.z),(13,22,-5))
            finally: session.close()


if __name__ == '__main__': unittest.main()

"""Bounded takeoff/hold/waypoint/land experiment for an owned ArduCopter SITL process.

Uses the autopilot's native position estimate and controller. This is not the
external-odometry supervisor, a collision planner, or a hardware flight interface.
"""
from collections import deque
from dataclasses import asdict, dataclass
import csv
import json
import math
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mav
from .mavlink import Encoder
from .telemetry import Receiver
from .supervisor import fresh


class MissionError(RuntimeError):
    pass


@dataclass(frozen=True)
class MissionPlan:
    altitude: float = 3.0
    north: float = 3.0
    east: float = 2.0
    speed: float = 0.75
    hold_seconds: float = 3.0
    tolerance: float = 0.35
    settle_speed: float = 0.20
    phase_timeout: float = 60.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError('Mission settings must be finite')
        if not 1 <= self.altitude <= 5 or not 0.1 <= math.hypot(self.north, self.east) <= 10:
            raise ValueError('Use altitude 1–5 m and horizontal distance 0.1–10 m')
        if not 0.1 <= self.speed <= 1 or not 1 <= self.hold_seconds <= 20:
            raise ValueError('Use speed 0.1–1 m/s and hold time 1–20 s')
        if not 0 < self.tolerance <= .5 or not 0 < self.settle_speed <= .3:
            raise ValueError('Invalid settling thresholds')
        if not 10 <= self.phase_timeout <= 120:
            raise ValueError('Phase timeout must be 10–120 seconds')


class Session:
    """One connected socket, explicit IDs, bounded waits, recorded wire traffic."""
    def __init__(self, sock, output, clock=time.monotonic):
        self.sock, self.clock = sock, clock
        self.started = clock()
        self.encoder = Encoder(source_system=255, source_component=190)
        self.receiver = Receiver(1, 1)
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True
        self.acks = deque(maxlen=100)
        self.last_heartbeat_sent = -math.inf
        self.landed = None
        self.landed_at = None
        self.prearm_ok = False
        self.prearm_at = None
        self.attitude = (0., 0., 0.)
        self.phase = 'CONNECT'
        self.events = (output/'events.jsonl').open('x')
        self.capture = (output/'wire.jsonl').open('x')
        self.csv_file = (output/'telemetry.csv').open('x', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['time_s','phase','north_m','east_m','down_m','vn_mps','ve_mps','vd_mps','roll_deg','pitch_deg','yaw_deg'])
        self.samples = []

    def event(self, kind, **fields):
        self.events.write(json.dumps(dict(time_s=self.clock()-self.started, event=kind, **fields), allow_nan=False)+'\n')
        self.events.flush()
        if kind in ('phase', 'failure', 'complete'):
            print(f'{self.clock()-self.started:6.1f}s  {kind}: {fields}', flush=True)

    def send(self, message):
        data = self.encoder.packet(message)
        self.sock.settimeout(1.)
        self.sock.sendall(data)
        self.capture.write(json.dumps(dict(time_s=self.clock()-self.started, direction='tx', hex=data.hex()))+'\n')
        self.capture.flush()

    def ingest(self, data, now):
        self.capture.write(json.dumps(dict(time_s=now-self.started, direction='rx', hex=data.hex()))+'\n')
        previous = self.receiver.state.position_boot_ms
        self.receiver.feed(data, now)
        for message in self.parser.parse_buffer(data) or []:
            if (message.get_srcSystem(), message.get_srcComponent()) != (1, 1):
                continue
            kind = message.get_type()
            if kind == 'COMMAND_ACK':
                if message.target_system in (0,255) and message.target_component in (0,190):
                    self.acks.append((now, message.command, message.result))
                    self.event('ack', command=message.command, result=message.result)
            elif kind == 'EXTENDED_SYS_STATE':
                self.landed, self.landed_at = message.landed_state, now
            elif kind == 'SYS_STATUS':
                bit = mav.MAV_SYS_STATUS_PREARM_CHECK
                self.prearm_ok = bool(message.onboard_control_sensors_health & bit)
                self.prearm_at = now
            elif kind == 'ATTITUDE':
                values = (message.roll, message.pitch, message.yaw)
                if all(math.isfinite(v) for v in values):
                    self.attitude = tuple(math.degrees(v) for v in values)
            elif kind == 'STATUSTEXT':
                self.event('status', severity=message.severity, text=message.text)
        state = self.receiver.state
        if state.position_boot_ms != previous and state.position_ned is not None:
            row = [now-self.started, self.phase, *state.position_ned, *state.velocity_ned, *self.attitude]
            self.csv.writerow(row)
            self.csv_file.flush()
            self.samples.append(row)

    def poll(self, timeout=.1):
        now = self.clock()
        if now-self.last_heartbeat_sent >= 1:
            self.send(mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID,
                                                   0, 0, mav.MAV_STATE_ACTIVE, 3))
            self.last_heartbeat_sent = now
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(65535)
        except socket.timeout:
            return
        if not data:
            raise MissionError('Simulator connection closed')
        self.ingest(data, self.clock())

    def command(self, command, *parameters, timeout=8.):
        if len(parameters) > 7 or not all(math.isfinite(p) for p in parameters):
            raise ValueError('Expected up to seven finite command parameters')
        # Drain traffic before opening a command transaction, then discard old ACKs.
        self.poll(.001)
        self.acks.clear()
        sent = self.clock()
        self.event('command', command=command, parameters=parameters)
        self.send(mav.MAVLink_command_long_message(1, 1, command, 0,
                  *(list(parameters)+[0.]*(7-len(parameters)))))
        while self.clock()-sent < timeout:
            self.poll()
            while self.acks:
                received, ack_command, result = self.acks.popleft()
                if received < sent or ack_command != command:
                    continue
                if result == mav.MAV_RESULT_ACCEPTED:
                    return
                if result != mav.MAV_RESULT_IN_PROGRESS:
                    raise MissionError(f'Command {command} rejected: MAV_RESULT {result}')
        raise MissionError(f'Command {command} acknowledgment timed out')

    def position_target(self, target):
        self.send(mav.MAVLink_set_position_target_local_ned_message(
            int((self.clock()-self.started)*1000) % 2**32, 1, 1, mav.MAV_FRAME_LOCAL_NED,
            3576, *target, 0,0,0, 0,0,0, 0,0))

    def close(self):
        self.events.close()
        self.capture.close()
        self.csv_file.close()
        self.sock.close()


def healthy(state, now, *, guided=True, armed=True):
    if not fresh(state.heartbeat_at, now, 2.5):
        raise MissionError('Autopilot heartbeat stale')
    if state.discontinuity or not fresh(state.position_at, now, 1.):
        raise MissionError('Local position stale or its boot clock changed')
    if armed and not state.armed:
        raise MissionError('Autopilot unexpectedly disarmed')
    if guided and not state.guided:
        raise MissionError('Autopilot left GUIDED; no further position commands')
    if state.system_status in (mav.MAV_STATE_CRITICAL, mav.MAV_STATE_EMERGENCY):
        raise MissionError('Autopilot reported a critical or emergency state')


class Mission:
    def __init__(self, session, plan):
        self.s, self.plan = session, plan
        self.origin = None
        self.targets = {}
        self.completed = []

    def phase(self, name):
        self.s.phase = name
        self.s.event('phase', name=name)

    def wait(self, predicate, timeout, description):
        end = self.s.clock()+timeout
        while self.s.clock() < end:
            self.s.poll()
            if predicate():
                return
        raise MissionError(f'Timed out waiting for {description}')

    def in_bounds(self):
        p = self.s.receiver.state.position_ned
        if p is None:
            raise MissionError('Missing local position')
        n,e,d = (a-b for a,b in zip(p,self.origin))
        if math.hypot(n,e) > 15 or not -8 <= d <= 2:
            raise MissionError('Simulation exceeded the 15 m horizontal / 8 m height boundary')

    def hold(self, name, target, *, takeoff=False):
        self.phase(name)
        self.targets[name] = list(target)
        entered = None
        start = self.s.clock()
        next_target = start
        max_error = 0.
        while self.s.clock()-start < self.plan.phase_timeout:
            self.s.poll()
            now = self.s.clock()
            state = self.s.receiver.state
            healthy(state, now)
            self.in_bounds()
            if not takeoff and now >= next_target:
                self.s.position_target(target)
                next_target = now+.2
            error = math.dist(state.position_ned, target)
            speed = math.hypot(*state.velocity_ned)
            if error <= self.plan.tolerance and speed <= self.plan.settle_speed:
                if entered is None:
                    entered, max_error = now, error
                max_error = max(max_error,error)
                if now-entered >= self.plan.hold_seconds:
                    self.completed.append(dict(phase=name, duration_s=now-start,
                        settled_hold_s=now-entered, max_hold_error_m=max_error,
                        final_error_m=error, final_speed_mps=speed))
                    return
            else:
                entered = None
        raise MissionError(f'{name} did not settle within {self.plan.phase_timeout:g} s')

    def run(self):
        self.phase('CONNECT')
        self.wait(lambda: fresh(self.s.receiver.state.heartbeat_at,self.s.clock(),2.5), 30, 'ArduCopter heartbeat')
        if self.s.receiver.state.armed:
            raise MissionError('Expected a fresh, disarmed simulator')
        # Request rates explicitly instead of assuming a GCS has configured them.
        for message_id, hz in [(32,10),(30,10),(1,2),(245,2)]:
            self.s.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 1e6/hz)
        self.phase('PREARM')
        self.wait(lambda: (self.s.prearm_ok and fresh(self.s.prearm_at,self.s.clock(),2.)
                    and fresh(self.s.receiver.state.position_at,self.s.clock(),1.)
                    and self.s.landed == mav.MAV_LANDED_STATE_ON_GROUND
                    and fresh(self.s.landed_at,self.s.clock(),2.)), 90, 'position estimate and pre-arm checks')
        self.origin = tuple(self.s.receiver.state.position_ned)
        n,e,d = self.origin
        hover = (n,e,d-self.plan.altitude)
        waypoint = (n+self.plan.north,e+self.plan.east,d-self.plan.altitude)
        self.phase('GUIDED')
        self.s.command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
        self.wait(lambda: self.s.receiver.state.guided, 5, 'GUIDED heartbeat')
        self.s.command(mav.MAV_CMD_DO_CHANGE_SPEED, 1, self.plan.speed, -1)
        self.phase('ARM')
        self.s.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
        self.wait(lambda: self.s.receiver.state.armed, 5, 'armed heartbeat')
        self.s.command(mav.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,self.plan.altitude)
        self.hold('TAKEOFF',hover,takeoff=True)
        self.hold('HOVER',hover)
        self.hold('WAYPOINT',waypoint)
        self.phase('LAND')
        self.s.command(mav.MAV_CMD_NAV_LAND)
        self.wait(lambda: (fresh(self.s.receiver.state.heartbeat_at,self.s.clock(),2.5)
                   and not self.s.receiver.state.armed
                   and fresh(self.s.landed_at,self.s.clock(),2.)
                   and self.s.landed == mav.MAV_LANDED_STATE_ON_GROUND),
                   self.plan.phase_timeout, 'landing and automatic disarm')
        self.phase('COMPLETE')
        self.s.event('complete', landed=True, disarmed=True)
        return dict(passed=True, plan=asdict(self.plan), origin_ned=self.origin,
                    targets_ned=self.targets, phases=self.completed, landed=True, disarmed=True,
                    duration_s=self.s.clock()-self.s.started)

    def abort(self, reason):
        self.s.event('failure', reason=str(reason))
        state = self.s.receiver.state
        # Do not override an operator/autopilot mode change. The launcher always
        # stops its owned simulator after recording the failure.
        if state.armed and state.guided and fresh(state.heartbeat_at,self.s.clock(),2.5):
            try:
                self.phase('ABORT_LAND')
                self.s.command(mav.MAV_CMD_NAV_LAND, timeout=3)
                self.wait(lambda: not self.s.receiver.state.armed, 30, 'abort landing')
            except (MissionError, OSError) as error:
                self.s.event('abort_incomplete', reason=str(error))

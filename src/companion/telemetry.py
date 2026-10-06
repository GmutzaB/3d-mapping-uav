"""Receive-only ArduCopter telemetry. No connection discovery or vehicle writes."""
from dataclasses import dataclass, asdict
import math
from pymavlink.dialects.v20 import ardupilotmega as mav
from .supervisor import Health, fresh


@dataclass
class Telemetry:
    heartbeat_at: float | None = None
    armed: bool = False
    guided: bool = False
    system_status: int | None = None
    position_at: float | None = None
    position_ned: tuple | None = None
    velocity_ned: tuple | None = None
    position_boot_ms: int | None = None
    discontinuity: bool = False
    accepted: int = 0
    rejected: int = 0
    bad_packets: int = 0


class Receiver:
    def __init__(self, system_id=1, component_id=1):
        for value in (system_id,component_id):
            if isinstance(value,bool) or not isinstance(value,int) or not 1 <= value <= 255:
                raise ValueError('Explicit system and component IDs in [1,255] required')
        self.system_id, self.component_id = system_id, component_id
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True
        self.state = Telemetry()
        self.last_receive_time = None

    def feed(self, data, received_at):
        if not math.isfinite(received_at) or received_at < 0:
            raise ValueError('Receive time must be finite, nonnegative monotonic seconds')
        if self.last_receive_time is not None and received_at < self.last_receive_time:
            raise ValueError('Receive time moved backwards')
        self.last_receive_time = received_at
        for message in self.parser.parse_buffer(data) or []:
            self._accept(message, received_at)

    def _accept(self, message, now):
        s = self.state
        if message.get_type() == 'BAD_DATA':
            s.bad_packets += 1
            return
        if (message.get_srcSystem(),message.get_srcComponent()) != (self.system_id,self.component_id):
            s.rejected += 1
            return
        kind = message.get_type()
        if kind == 'HEARTBEAT':
            if message.autopilot != mav.MAV_AUTOPILOT_ARDUPILOTMEGA or message.type != mav.MAV_TYPE_QUADROTOR:
                s.rejected += 1
                return
            s.heartbeat_at = now
            s.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            s.guided = bool(message.base_mode & mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) and message.custom_mode == 4
            s.system_status = message.system_status
        elif kind == 'LOCAL_POSITION_NED':
            values = (message.x,message.y,message.z,message.vx,message.vy,message.vz)
            if not all(math.isfinite(v) for v in values):
                s.rejected += 1
                return
            if s.position_boot_ms is not None:
                delta = (message.time_boot_ms-s.position_boot_ms) % 2**32
                if delta == 0:
                    s.rejected += 1  # Repeats must not refresh freshness.
                    return
                if delta >= 2**31:
                    # Reboot and out-of-order delivery are indistinguishable here.
                    # Latch a discontinuity, requiring a new Receiver/session.
                    s.discontinuity = True
                    s.position_at = None
                    s.position_ned = s.velocity_ned = None
                    s.rejected += 1
                    return
            if s.discontinuity:
                s.rejected += 1
                return
            s.position_boot_ms = message.time_boot_ms
            s.position_at = now
            s.position_ned = values[:3]
            s.velocity_ned = values[3:]
        else:
            return
        s.accepted += 1

    def health(self, *, odometry_at=None, localization_valid=False, estimator_epoch=0):
        """Companion localization must be supplied separately, already time-aligned.

        Fresh FC local position does NOT prove valid external localization or EKF
        acceptance. Only active system status permits command authority.
        """
        s = self.state
        operational = s.system_status == mav.MAV_STATE_ACTIVE and not s.discontinuity
        return Health(s.heartbeat_at,odometry_at,localization_valid and operational,
                      s.guided and operational,s.armed,estimator_epoch)

    def snapshot(self, now, heartbeat_timeout=1.5, position_timeout=0.5):
        if not math.isfinite(now) or now < 0 or any(not math.isfinite(v) or v <= 0 for v in (heartbeat_timeout,position_timeout)):
            raise ValueError('Invalid snapshot time or timeout')
        result = asdict(self.state)
        result.update(heartbeat_fresh=fresh(self.state.heartbeat_at,now,heartbeat_timeout),
                      position_received_recently=fresh(self.state.position_at,now,position_timeout),
                      time=now)
        return result

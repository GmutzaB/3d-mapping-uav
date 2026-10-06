"""Deterministic command gate. Decisions are intents, not flight mode acknowledgments."""
from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    DISABLED = 'disabled'
    ACTIVE = 'active'
    HOLD = 'hold_requested'
    FAULT = 'fault_latched'


@dataclass(frozen=True)
class Limits:
    heartbeat_age: float = 1.5
    odometry_age: float = 0.2
    plan_age: float = 0.3
    max_speed: float = 1.0

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Limits must be positive and finite')


@dataclass(frozen=True)
class Health:
    heartbeat_at: float | None = None
    odometry_at: float | None = None
    localization_valid: bool = False
    guided: bool = False
    armed: bool = False
    estimator_epoch: int = 0


@dataclass(frozen=True)
class VelocityPlan:
    created_at: float
    expires_at: float
    velocity: tuple[float, float, float]
    frame: str = 'LOCAL_NED'
    collision_checked: bool = False
    estimator_epoch: int = 0


@dataclass(frozen=True)
class Decision:
    state: State
    reason: str
    velocity: tuple[float, float, float] | None = None


def fresh(stamp, now, limit):
    return stamp is not None and math.isfinite(stamp) and 0 <= now-stamp <= limit


class Supervisor:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.state = State.DISABLED
        self.epoch = None
        self.last_now = None

    def _check(self, now, health, plan):
        limits = self.limits
        if not fresh(health.heartbeat_at, now, limits.heartbeat_age):
            return State.FAULT, 'flight-controller heartbeat missing or stale'
        if not health.guided or not health.armed:
            return State.FAULT, 'flight controller is not armed in GUIDED'
        if not health.localization_valid or not fresh(health.odometry_at, now, limits.odometry_age):
            return State.FAULT, 'localization invalid or stale; position hold is not assured'
        if self.epoch is not None and health.estimator_epoch != self.epoch:
            return State.HOLD, 'estimator reset; require a new plan and operator enable'
        if plan is None:
            return State.HOLD, 'no trajectory'
        if not fresh(plan.created_at, now, limits.plan_age) or not math.isfinite(plan.expires_at) or now >= plan.expires_at:
            return State.HOLD, 'trajectory stale or expired'
        if plan.estimator_epoch != health.estimator_epoch:
            return State.HOLD, 'trajectory uses an old estimator origin'
        if plan.frame != 'LOCAL_NED' or not plan.collision_checked:
            return State.HOLD, 'trajectory frame or collision validation missing'
        if len(plan.velocity) != 3 or not all(math.isfinite(v) for v in plan.velocity):
            return State.HOLD, 'invalid velocity'
        if math.hypot(*plan.velocity) > limits.max_speed:
            return State.HOLD, 'velocity exceeds configured limit'
        return State.ACTIVE, 'accepted'

    def tick(self, now, health, plan=None, *, enable=False, disable=False):
        """Enable is an explicit operator event, not a continuously asserted boolean.

        HOLD and FAULT latch; fresh data alone cannot resume. FAULT never emits a
        velocity. A HOLD may request zero velocity only while navigation is healthy.
        Caller must supply odometry_at in local monotonic capture-time coordinates,
        including transport/processing age, not simply restamp old poses on receipt.
        """
        if not math.isfinite(now) or (self.last_now is not None and now < self.last_now):
            self.state = State.FAULT
            return Decision(self.state, 'invalid or regressing monotonic clock')
        self.last_now = now
        if disable:
            self.state = State.DISABLED
            self.epoch = None
            return Decision(self.state, 'operator disabled command authority')
        if self.state == State.DISABLED and not enable:
            return Decision(self.state, 'operator enable required')
        # Enable explicitly accepts the current origin, subject to a matching fresh plan.
        if enable:
            self.epoch = None
        checked, reason = self._check(now, health, plan)
        if enable or self.state == State.ACTIVE:
            self.state = checked
            if checked == State.ACTIVE:
                self.epoch = health.estimator_epoch
                return Decision(self.state, reason, tuple(plan.velocity))
        elif checked == State.FAULT:
            self.state = State.FAULT
        if self.state == State.HOLD:
            return Decision(self.state, reason if checked != State.ACTIVE else 'operator re-enable required', (0.,0.,0.))
        return Decision(self.state, reason if checked != State.ACTIVE else 'operator re-enable required')

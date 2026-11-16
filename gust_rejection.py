"""Dryden wind-gust rejection: position hold vs. attitude-only, simulation only.

Wind model (MIL-HDBK-1797 / MIL-F-8785C, low altitude):
  * a steady mean wind that ramps up at the start,
  * continuous Dryden turbulence on all three axes (frozen-turbulence assumption:
    the mean wind carries the gust field past the hovering aircraft),
  * one discrete "1-cosine" crosswind gust.

The wind acts on the existing 6-DOF quad (flight_dynamics.py) as quadratic drag on
the air-relative velocity. Two controllers fly the same wind sample:
  * position hold: new cascaded PID (position -> tilt/thrust) over the existing
    attitude law from flight_control.py,
  * attitude only: the existing flight_control.Controller (level + height hold,
    no horizontal position feedback).

Everything here is provisional: drag areas, mass, inertia and thrust are unmeasured.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse
import json
import numpy as np

from flight_dynamics import Quad, rotation
from flight_control import Controller, Gains, angles

FT = 0.3048
RHO = 1.225
G = 9.81


# ----------------------------------------------------------------------------- wind
def _expm(M):
    """Matrix exponential by scaling and squaring (small, well-scaled matrices)."""
    norm = np.abs(M).sum(axis=1).max()
    squarings = max(0, int(np.ceil(np.log2(norm))) + 1) if norm > 0 else 0
    A = M / 2**squarings
    result = term = np.eye(len(M))
    for k in range(1, 20):
        term = term @ A / k
        result = result + term
    for _ in range(squarings):
        result = result @ result
    return result


def lyapunov(A, Q):
    """Solve A P + P A^T + Q = 0 for P."""
    n = len(A)
    K = np.kron(np.eye(n), A) + np.kron(A, np.eye(n))
    return np.linalg.solve(K, -Q.reshape(-1)).reshape(n, n)


def dryden_parameters(altitude_m, wind20_m_s):
    """MIL-HDBK-1797 low-altitude (< 1000 ft) scales and intensities, SI out.

    wind20 is the mean wind speed at 20 ft (6.1 m). Altitude is clamped to >= 10 ft,
    the bottom of the published model.
    """
    h = max(altitude_m / FT, 10.0)
    sigma_w = 0.1 * wind20_m_s
    sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 * h) ** 0.4
    L_u = L_v = h / (0.177 + 0.000823 * h) ** 1.2 * FT
    L_w = h * FT
    return dict(sigma=np.array([sigma_u, sigma_v, sigma_w]),
                length=np.array([L_u, L_v, L_w]), altitude_ft=h)


def dryden_filter(sigma, length, airspeed, transverse):
    """State space (A, B, C) of one Dryden forming filter driven by unit white noise.

    Longitudinal: H = sigma*sqrt(2L/(pi V)) / (1 + (L/V)s)
    Transverse:   H = sigma*sqrt(L/(pi V)) (1 + sqrt(3)(L/V)s) / (1 + (L/V)s)^2
    The noise convention matches the spec: unit one-sided PSD in rad/s, which is a
    continuous white-noise intensity of pi. Output variance is exactly sigma^2.
    """
    T = length / airspeed
    if not transverse:
        K = sigma * np.sqrt(2 * length / (np.pi * airspeed))
        return np.array([[-1 / T]]), np.array([[1.0]]), np.array([[K / T]])
    K = sigma * np.sqrt(length / (np.pi * airspeed))
    A = np.array([[0.0, 1.0], [-1 / T**2, -2 / T]])
    B = np.array([[0.0], [1.0]])
    C = np.array([[K / T**2, K * np.sqrt(3) / T]])
    return A, B, C


NOISE_INTENSITY = np.pi


def discretize(A, B, dt, q=NOISE_INTENSITY):
    """Van Loan: exact discrete transition and process-noise covariance."""
    n = len(A)
    M = np.zeros((2 * n, 2 * n))
    M[:n, :n] = -A
    M[:n, n:] = q * B @ B.T
    M[n:, n:] = A.T
    F = _expm(M * dt)
    Ad = F[n:, n:].T
    Qd = Ad @ F[:n, n:]
    return Ad, 0.5 * (Qd + Qd.T)


def dryden_series(n, dt, altitude_m, wind20_m_s, airspeed_m_s, rng):
    """n samples of [u, v, w] turbulence (m/s), started from the stationary state."""
    params = dryden_parameters(altitude_m, wind20_m_s)
    out = np.zeros((n, 3))
    for axis in range(3):
        A, B, C = dryden_filter(params['sigma'][axis], params['length'][axis],
                                airspeed_m_s, transverse=axis > 0)
        Ad, Qd = discretize(A, B, dt)
        L = np.linalg.cholesky(Qd + 1e-15 * np.eye(len(A)))
        P = lyapunov(A, NOISE_INTENSITY * B @ B.T)
        x = np.linalg.cholesky(P + 1e-15 * np.eye(len(A))) @ rng.standard_normal(len(A))
        for k in range(n):
            out[k, axis] = (C @ x)[0]
            x = Ad @ x + L @ rng.standard_normal(len(A))
    return out, params


def one_minus_cosine(t, start, duration, amplitude):
    """MIL-F-8785C discrete gust shape in time: rises and falls over `duration`."""
    t = np.asarray(t, dtype=float)
    inside = (t >= start) & (t <= start + duration)
    return np.where(inside, amplitude / 2 * (1 - np.cos(2 * np.pi * (t - start) / duration)), 0.0)


@dataclass(frozen=True)
class WindSettings:
    mean_speed: float = 8.0        # m/s, steady wind toward east (from the west)
    ramp_s: float = 4.0            # mean wind and turbulence fade in over this time
    gust_amplitude: float = 6.0    # m/s, discrete crosswind gust toward north
    gust_start: float = 18.0
    gust_duration: float = 3.0
    altitude: float = 5.0          # m above ground for the Dryden scales
    seconds: float = 36.0
    seed: int = 7

    def __post_init__(self):
        if not (self.mean_speed > 0 and self.seconds > 0 and self.ramp_s >= 0 and self.gust_duration > 0
                and self.altitude > 0 and self.gust_amplitude >= 0):
            raise ValueError('Wind settings out of range')


def wind_field(settings, dt):
    """World NED wind samples: total plus its mean, turbulence and gust parts."""
    t = np.arange(0, settings.seconds + dt / 2, dt)
    ramp = np.clip(t / settings.ramp_s, 0, 1) if settings.ramp_s > 0 else np.ones_like(t)
    ramp = ramp * ramp * (3 - 2 * ramp)  # smoothstep
    rng = np.random.default_rng(settings.seed)
    turb, params = dryden_series(len(t), dt, settings.altitude, settings.mean_speed,
                                 settings.mean_speed, rng)
    # Wind axes -> NED: u along the mean wind (east), v to its left (north), w up.
    mean = np.column_stack((0 * t, settings.mean_speed * ramp, 0 * t))
    turbulence = np.column_stack((turb[:, 1], turb[:, 0], -turb[:, 2])) * ramp[:, None]
    gust = np.column_stack((one_minus_cosine(t, settings.gust_start, settings.gust_duration,
                                             settings.gust_amplitude), 0 * t, 0 * t))
    return dict(time=t, total=mean + turbulence + gust, mean=mean, turbulence=turbulence,
                gust=gust), params


# ------------------------------------------------------------------ aircraft + wind
@dataclass(frozen=True)
class Aero:
    cda_horizontal: float = 0.05   # m^2, side/front drag area, unmeasured estimate
    cda_vertical: float = 0.10     # m^2, top-down drag area, unmeasured estimate


class WindQuad(Quad):
    """The existing quad plus quadratic drag on velocity relative to the air."""

    def __init__(self, parameters=None, aero=None):
        super().__init__(parameters)
        self.aero = aero or Aero()
        self.wind = np.zeros(3)

    def drag_force(self, velocity, wind=None):
        relative = np.asarray(velocity, dtype=float) - (self.wind if wind is None else wind)
        cda = np.array([self.aero.cda_horizontal, self.aero.cda_horizontal, self.aero.cda_vertical])
        return -0.5 * RHO * cda * np.linalg.norm(relative) * relative

    def derivative(self, s, command, disturbance_torque=None):
        out = super().derivative(s, command, disturbance_torque)
        out[3:6] += self.drag_force(s[3:6]) / self.p.mass
        return out


# -------------------------------------------------------------------- controllers
@dataclass(frozen=True)
class PositionGains:
    # Triple closed-loop pole at 1.1 rad/s: Kd = 3w, Kp = 3w^2, Ki = w^3.
    # Several times slower than the 6 rad/s attitude loop it commands.
    kp: float = 3 * 1.1**2
    kd: float = 3 * 1.1
    ki: float = 1.1**3
    max_tilt_deg: float = 25.0
    integral_limit: float = 3.0   # m/s^2 per axis, anti-windup clamp

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Position gains must be positive and finite')


def attitude_torque(controller, state, desired):
    """Same attitude law as flight_control.Controller, with an arbitrary target R."""
    R = rotation(state[6:10])
    skew = .5 * (desired.T @ R - R.T @ desired)
    error = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    omega = state[10:13]
    g = controller.gains
    inertia = np.asarray(controller.p.inertia)
    return (inertia * (-g.attitude_frequency**2 * error - 2 * g.damping * g.attitude_frequency * omega)
            + np.cross(omega, inertia * omega))


class PositionHold:
    """Cascaded PID: position error -> desired acceleration -> tilt + collective."""

    def __init__(self, quad, target, gains=None, attitude_gains=None):
        self.inner = Controller(quad, attitude_gains or Gains())
        self.p = quad.p
        self.gains = gains or PositionGains()
        self.target = np.asarray(target, dtype=float)
        self.integral = np.zeros(3)

    def command(self, state, dt):
        g = self.gains
        error = state[:3] - self.target
        self.integral = np.clip(self.integral - g.ki * error * dt, -g.integral_limit, g.integral_limit)
        accel = -g.kp * error - g.kd * state[3:6] + self.integral
        # Required specific force (NED): thrust/mass = accel - gravity
        f = accel - np.array([0, 0, G])
        f[2] = min(f[2], -0.3 * G)  # never ask for less than 30% lift
        horizontal = np.linalg.norm(f[:2])
        limit = -f[2] * np.tan(np.radians(g.max_tilt_deg))
        if horizontal > limit:
            f[:2] *= limit / horizontal
        z_body = -f / np.linalg.norm(f)                      # body down axis, world NED
        y_body = np.cross(z_body, [1.0, 0, 0]); y_body /= np.linalg.norm(y_body)
        x_body = np.cross(y_body, z_body)
        desired = np.column_stack((x_body, y_body, z_body))  # yaw held at north
        R = rotation(state[6:10])
        collective = self.p.mass * max(-f @ R[:, 2], 0.0)
        torque = attitude_torque(self.inner, state, desired)
        requested = self.inner.inverse @ np.r_[collective, torque]
        thrust = np.clip(requested, 0, self.p.max_thrust)
        return np.sqrt(thrust / self.p.max_thrust), bool(np.any(abs(requested - thrust) > 1e-10))


class AttitudeOnly:
    """The existing controller, shifted so it holds the same start altitude."""

    def __init__(self, quad, target):
        self.inner = Controller(quad)
        self.target = np.asarray(target, dtype=float)

    def command(self, state, dt):
        shifted = state.copy()
        shifted[2] -= self.target[2]
        return self.inner.command(shifted)


# ----------------------------------------------------------------------- power
@dataclass(frozen=True)
class PowerModel:
    """Same assumptions as the team's Prop_and_Motor_Calcs.xlsx (hover momentum theory)."""
    figure_of_merit: float = 0.65
    esc_efficiency: float = 0.85
    electronics_w: float = 45.0
    voltage: float = 22.2
    capacity_ah: float = 10.0
    usable_fraction: float = 0.8
    prop_diameter_m: float = 9 * 0.0254

    def power(self, rotor_thrusts):
        area = np.pi * (self.prop_diameter_m / 2) ** 2
        T = np.clip(np.asarray(rotor_thrusts, dtype=float), 0, None)
        ideal = (T ** 1.5).sum(axis=-1) / np.sqrt(2 * RHO * area)
        return ideal / self.figure_of_merit / self.esc_efficiency + self.electronics_w

    def flight_minutes(self, watts):
        return self.voltage * self.capacity_ah * self.usable_fraction / watts * 60


# ------------------------------------------------------------------------- run
def run(controller_name, settings=WindSettings(), dt=0.005, wind=None, aero=None):
    if wind is None:
        wind, _ = wind_field(settings, dt)
    quad = WindQuad(aero=aero)
    target = np.array([0.0, 0.0, -settings.altitude])
    quad.state[:3] = target
    quad.state[13:] = quad.p.mass * G / 4
    controller = (PositionHold if controller_name == 'position_hold' else AttitudeOnly)(quad, target)
    t = wind['time']
    states, throttles, saturated = [], [], []
    for k in range(len(t)):
        quad.wind = wind['total'][k]
        command, clipped = controller.command(quad.state, dt)
        states.append(quad.state.copy()); throttles.append(command); saturated.append(clipped)
        if k + 1 < len(t):
            quad.step(command, t[k + 1] - t[k])
    states = np.array(states)
    error = states[:, :3] - target
    return dict(time=t, state=states, throttle=np.array(throttles), saturated=np.array(saturated),
                wind=wind['total'], wind_mean=wind['mean'], wind_turbulence=wind['turbulence'],
                wind_gust=wind['gust'], error=error, target=target,
                tilt_deg=np.degrees(np.arccos(np.clip(np.array([rotation(q)[2, 2] for q in states[:, 6:10]]), -1, 1))),
                power_w=PowerModel().power(states[:, 13:17]))


def metrics(data, settings):
    t, e = data['time'], data['error']
    horizontal = np.linalg.norm(e[:, :2], axis=1)
    settled = t >= settings.ramp_s + 4
    gust = (t >= settings.gust_start) & (t <= settings.gust_start + settings.gust_duration + 3)
    pm = PowerModel()
    quiet = pm.power(np.full(4, WindQuad().p.mass * G / 4))
    mean_power = float(data['power_w'][settled].mean())
    return dict(
        rms_horizontal_error_m=float(np.sqrt((horizontal[settled] ** 2).mean())),
        max_horizontal_error_m=float(horizontal.max()),
        max_horizontal_error_during_gust_m=float(horizontal[gust].max()),
        final_horizontal_error_m=float(horizontal[-1]),
        max_altitude_error_m=float(abs(e[:, 2]).max()),
        max_tilt_deg=float(data['tilt_deg'].max()),
        saturated_samples=int(data['saturated'].sum()),
        still_air_hover_power_w=float(quiet), mean_power_in_wind_w=mean_power,
        still_air_flight_min=float(pm.flight_minutes(quiet)),
        flight_min_at_mean_wind_power=float(pm.flight_minutes(mean_power)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--wind', type=float, default=WindSettings.mean_speed, help='Mean wind, m/s')
    parser.add_argument('--gust', type=float, default=WindSettings.gust_amplitude, help='Discrete gust peak, m/s')
    parser.add_argument('--seconds', type=float, default=WindSettings.seconds)
    parser.add_argument('--seed', type=int, default=WindSettings.seed)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'outputs/gust')
    parser.add_argument('--no-visuals', action='store_true')
    args = parser.parse_args()
    settings = WindSettings(mean_speed=args.wind, gust_amplitude=args.gust, seconds=args.seconds,
                            seed=args.seed, gust_start=min(WindSettings.gust_start, args.seconds / 2))
    dt = 0.005
    wind, params = wind_field(settings, dt)
    datasets = {name: run(name, settings, dt, wind) for name in ['position_hold', 'attitude_only']}
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(
        wind=asdict(settings), dryden=dict(sigma_m_s=params['sigma'].tolist(), length_m=params['length'].tolist(),
                                           altitude_ft_used=params['altitude_ft']),
        aero=asdict(Aero()), position_gains=asdict(PositionGains()), aircraft=asdict(WindQuad().p),
        power_model=asdict(PowerModel()), dt_s=dt,
        results={name: metrics(data, settings) for name, data in datasets.items()},
        limitation=('Simulation only. Drag areas, mass, inertia and thrust are unmeasured. Perfect state '
                    'feedback (no GPS/EKF noise). Wind is spatially uniform across the airframe. Power uses '
                    'hover momentum theory and ignores translational lift and rotor drag.'))
    for name, data in datasets.items():
        np.savez(args.output / f'{name}.npz', **{k: v for k, v in data.items()})
    (args.output / 'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report['results'], indent=2))
    if not args.no_visuals:
        from gust_visuals import export
        export(datasets, report, args.output)


if __name__ == '__main__':
    main()

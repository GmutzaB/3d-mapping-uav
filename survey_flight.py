"""Fly a complete mapping survey in simulation: takeoff, lawnmower lines, return, land.

Combines the existing pieces:
  * 6-DOF quad with wind drag (gust_rejection.WindQuad) at the full takeoff mass,
  * Dryden turbulence + steady wind (gust_rejection.wind_field),
  * the cascaded position controller (gust_rejection.PositionHold) following a
    moving reference with velocity/acceleration feed-forward,
  * forward-flight rotor power and the 9000 mAh LiPo model (battery_endurance).

Reports how long the survey takes, battery used and left at landing, minimum voltage,
how closely the path was followed, and what fraction of the area the LiDAR swath covered.
Three wind cases fly the same plan so they can be compared directly.

Simulation only: perfect state feedback (no GPS/EKF error), uniform wind, unmeasured
mass/inertia/drag/efficiencies, no ground contact (the flight ends 0.3 m above the pad).
"""
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import argparse
import json
import numpy as np

from battery_endurance import Airframe, Battery, rotor_power, level_flight, discharge
from flight_dynamics import Parameters, rotation
from gust_rejection import G, PositionHold, WindQuad, WindSettings, wind_field


@dataclass(frozen=True)
class Survey:
    width_m: float = 120.0         # east extent of the area to map
    length_m: float = 80.0         # north extent
    altitude_m: float = 12.0       # flight height above the pad
    line_spacing_m: float = 15.0   # distance between survey lines
    swath_m: float = 20.0          # ground width the LiDAR covers usefully (set from mount/range)
    speed_m_s: float = 5.0         # survey line speed
    transit_speed_m_s: float = 6.0
    vertical_speed_m_s: float = 1.5
    accel_m_s2: float = 1.5
    home_offset_m: float = 8.0     # pad is this far south-west of the area corner
    hold_s: float = 3.0            # hover after takeoff and before landing

    def __post_init__(self):
        values = asdict(self)
        if any(not np.isfinite(v) or v <= 0 for k, v in values.items() if k != 'home_offset_m'):
            raise ValueError('Survey dimensions and speeds must be positive')
        if self.line_spacing_m > self.swath_m:
            print('Warning: line spacing wider than the swath leaves gaps between lines')


# ------------------------------------------------------------------- the plan
def waypoints(s):
    """List of (north, east, down, speed) targets, starting on the pad. World NED."""
    home = np.array([-s.home_offset_m, -s.home_offset_m, 0.0])
    up = -s.altitude_m
    pts = [(*home, 0), (home[0], home[1], up, s.vertical_speed_m_s)]
    n_lines = int(np.ceil(s.width_m / s.line_spacing_m)) + 1
    easts = np.linspace(0, s.width_m, n_lines)
    for k, e in enumerate(easts):
        start, end = (0.0, s.length_m) if k % 2 == 0 else (s.length_m, 0.0)
        speed = s.transit_speed_m_s if k == 0 else s.speed_m_s   # move to line start / step over
        pts.append((start, e, up, speed))
        pts.append((end, e, up, s.speed_m_s))
    pts.append((home[0], home[1], up, s.transit_speed_m_s))
    pts.append((home[0], home[1], -0.3, s.vertical_speed_m_s))
    return np.array(pts, dtype=float)


def trapezoid(distance, vmax, amax, dt):
    """Arc length, speed, acceleration samples for a stop-to-stop move."""
    if distance <= 1e-9:
        return np.zeros(1), np.zeros(1), np.zeros(1)
    t_acc = vmax / amax
    if amax * t_acc ** 2 > distance:              # never reaches vmax: triangle profile
        t_acc = np.sqrt(distance / amax)
        vmax = amax * t_acc
    t_total = 2 * t_acc + (distance - amax * t_acc ** 2) / vmax
    t = np.arange(0, t_total, dt)
    v = np.minimum.reduce([amax * t, np.full_like(t, vmax), amax * (t_total - t)])
    a = np.where(t < t_acc, amax, np.where(t > t_total - t_acc, -amax, 0.0))
    s = np.concatenate([[0], np.cumsum((v[1:] + v[:-1]) / 2 * dt)])
    return np.minimum(s, distance), v, a


def reference(s, dt):
    """Time-sampled position/velocity/acceleration reference and a 'surveying' flag."""
    wps = waypoints(s)
    pos, vel, acc, surveying = [], [], [], []

    def hold(p, seconds):
        n = int(round(seconds / dt))
        pos.extend([p] * n); vel.extend([np.zeros(3)] * n); acc.extend([np.zeros(3)] * n)
        surveying.extend([False] * n)

    hold(wps[0, :3], 1.0)
    for k in range(1, len(wps)):
        a, b, speed = wps[k - 1, :3], wps[k, :3], wps[k, 3]
        d = np.linalg.norm(b - a)
        arc, v, ac = trapezoid(d, speed, s.accel_m_s2 if a[2] == b[2] else 0.8, dt)
        unit = (b - a) / d if d > 0 else np.zeros(3)
        pos.extend(a + arc[:, None] * unit); vel.extend(v[:, None] * unit); acc.extend(ac[:, None] * unit)
        on_line = 3 <= k <= len(wps) - 3 and k % 2 == 1   # the segments along survey lines
        surveying.extend([on_line] * len(arc))
        if k in (1, len(wps) - 2):
            hold(b, s.hold_s)
    hold(wps[-1, :3], 1.0)
    return dict(pos=np.array(pos), vel=np.array(vel), acc=np.array(acc),
                surveying=np.array(surveying), waypoints=wps)


# -------------------------------------------------------------- flying it
class PathFollower(PositionHold):
    """PositionHold with a moving target: feed-forward velocity and acceleration."""

    def follow(self, state, dt, ref_pos, ref_vel, ref_acc):
        g = self.gains
        error = state[:3] - ref_pos
        self.integral = np.clip(self.integral - g.ki * error * dt, -g.integral_limit, g.integral_limit)
        accel = -g.kp * error - g.kd * (state[3:6] - ref_vel) + self.integral + ref_acc
        return self.command_from_accel(state, accel)


def electrical_power(state, wind, airframe):
    """Battery power (W) from the four rotor thrusts and the air-relative velocity."""
    up = -rotation(state[6:10])[:, 2]                 # thrust direction, world NED
    v_rel = state[3:6] - wind
    axial = float(v_rel @ up)
    edgewise = float(np.linalg.norm(v_rel - axial * up))
    return float(rotor_power(state[13:17], axial, edgewise, airframe).sum() + airframe.avionics_w)


def calm_wind(n, dt):
    t = np.arange(n) * dt
    zero = np.zeros((n, 3))
    return dict(time=t, total=zero, mean=zero, turbulence=zero, gust=zero)


def fly(survey=Survey(), wind_speed=5.0, airframe=Airframe(), battery=Battery(), dt=0.02, seed=7):
    ref = reference(survey, dt)
    n = len(ref['pos'])
    if wind_speed > 0:
        wind, _ = wind_field(WindSettings(mean_speed=wind_speed, gust_amplitude=0.0, ramp_s=2.0,
                                          altitude=survey.altitude_m, seconds=(n - 1) * dt,
                                          gust_start=0.5 * (n - 1) * dt, seed=seed), dt)
        wind = {k: v[:n] for k, v in wind.items()}
    else:
        wind = calm_wind(n, dt)
    mass = airframe.dry_mass_kg + battery.mass_kg
    quad = WindQuad(Parameters(mass=mass, max_thrust=airframe.max_thrust_per_rotor_n))
    quad.state[:3] = ref['pos'][0]
    quad.state[13:] = mass * G / 4
    controller = PathFollower(quad, ref['pos'][0])

    states = np.zeros((n, 17)); power = np.zeros(n); soc = np.zeros(n); volts = np.zeros(n)
    current = np.zeros(n); saturated = np.zeros(n, bool)
    charge = 1.0
    for k in range(n):
        quad.wind = wind['total'][k]
        command, clipped = controller.follow(quad.state, dt, ref['pos'][k], ref['vel'][k], ref['acc'][k])
        states[k], saturated[k] = quad.state, clipped
        power[k] = electrical_power(quad.state, quad.wind, airframe)
        current[k] = float(battery.current_for_power(power[k], charge))
        soc[k] = charge
        volts[k] = float(battery.ocv(charge)) - current[k] * battery.resistance
        charge -= current[k] * dt / 3600 / battery.capacity_ah
        if k + 1 < n:
            quad.step(command, dt)
    t = np.arange(n) * dt
    return dict(time=t, state=states, ref_pos=ref['pos'], surveying=ref['surveying'],
                waypoints=ref['waypoints'], wind=wind['total'], power_w=power, soc=soc,
                voltage_v=volts, current_a=current, saturated=saturated)


# ---------------------------------------------------------------- results
def coverage(survey, track_ne, cell_m=1.0):
    """Fraction of the survey rectangle within swath/2 of the flown survey track."""
    north = np.arange(cell_m / 2, survey.length_m, cell_m)
    east = np.arange(cell_m / 2, survey.width_m, cell_m)
    cells = np.stack(np.meshgrid(north, east, indexing='ij'), -1).reshape(-1, 2)
    covered = np.zeros(len(cells), bool)
    for chunk in np.array_split(track_ne, max(1, len(track_ne) // 200)):
        d = np.linalg.norm(cells[:, None, :] - chunk[None, :, :], axis=2).min(axis=1)
        covered |= d <= survey.swath_m / 2
    return float(covered.mean()), cells, covered


def metrics(data, survey, battery):
    t, s = data['time'], data['state']
    error = s[:, :3] - data['ref_pos']
    horiz = np.linalg.norm(error[:, :2], axis=1)
    on = data['surveying']
    track = s[on][::max(1, int(round(0.25 / (t[1] - t[0])))), :2]   # every 0.25 s along lines
    cov, _, _ = coverage(survey, track)
    dt = t[1] - t[0]
    energy_wh = float(data['power_w'].sum() * dt / 3600)
    used = 1 - float(data['soc'][-1])
    tilt = np.degrees(np.arccos(np.clip([rotation(q)[2, 2] for q in s[:, 6:10]], -1, 1)))
    distance = float(np.linalg.norm(np.diff(s[:, :3], axis=0), axis=1).sum())
    return dict(
        duration_min=float(t[-1] / 60), distance_m=distance, energy_wh=energy_wh,
        battery_used=used, battery_left_at_landing=float(data['soc'][-1]),
        within_reserve=bool(data['soc'][-1] >= battery.reserve_soc),
        surveys_per_battery=float((1 - battery.reserve_soc) / used) if used > 0 else None,
        mean_power_w=float(data['power_w'].mean()), peak_power_w=float(data['power_w'].max()),
        peak_current_a=float(data['current_a'].max()), min_voltage_v=float(data['voltage_v'].min()),
        rms_line_error_m=float(np.sqrt((horiz[on] ** 2).mean())), max_line_error_m=float(horiz[on].max()),
        max_altitude_error_m=float(abs(error[:, 2]).max()), max_tilt_deg=float(tilt.max()),
        saturated_samples=int(data['saturated'].sum()), coverage=cov)


def speed_sweep(survey, airframe, battery, speeds=np.arange(2.0, 13.0, 1.0)):
    """Quick steady-flight estimate of survey time and battery use vs. line speed (no 6-DOF)."""
    n_lines = int(np.ceil(survey.width_m / survey.line_spacing_m)) + 1
    line_length = n_lines * survey.length_m + (n_lines - 1) * survey.line_spacing_m
    hover_w = float(level_flight(airframe, battery, 0.0)['power_w'])
    rows = []
    for v in speeds:
        p = float(level_flight(airframe, battery, v)['power_w'])
        t_lines = line_length / v + 2 * n_lines * v / survey.accel_m_s2   # stop at each turn
        energy = p * t_lines / 3600 + hover_w * (2 * survey.altitude_m / survey.vertical_speed_m_s) / 3600
        rows.append((v, t_lines / 60, energy, energy / battery.nominal_wh))
    return np.array(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--width', type=float, default=Survey.width_m, help='Area east extent, m')
    parser.add_argument('--length', type=float, default=Survey.length_m, help='Area north extent, m')
    parser.add_argument('--altitude', type=float, default=Survey.altitude_m)
    parser.add_argument('--spacing', type=float, default=Survey.line_spacing_m, help='Line spacing, m')
    parser.add_argument('--swath', type=float, default=Survey.swath_m, help='Useful LiDAR swath, m')
    parser.add_argument('--speed', type=float, default=Survey.speed_m_s, help='Survey speed, m/s')
    parser.add_argument('--winds', type=float, nargs='+', default=[0.0, 5.0, 8.0],
                        help='Mean wind speeds to compare, m/s (default 0 5 8)')
    parser.add_argument('--battery-kg', type=float, default=Battery.mass_kg)
    parser.add_argument('--dry-kg', type=float, default=Airframe.dry_mass_kg)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'outputs/survey')
    parser.add_argument('--no-visuals', action='store_true')
    args = parser.parse_args()
    survey = Survey(width_m=args.width, length_m=args.length, altitude_m=args.altitude,
                    line_spacing_m=args.spacing, swath_m=args.swath, speed_m_s=args.speed)
    airframe, battery = Airframe(dry_mass_kg=args.dry_kg), Battery(mass_kg=args.battery_kg)

    runs, results = {}, {}
    for w in args.winds:
        name = f'wind_{w:g}_m_s'
        print(f'Flying survey in {w:g} m/s wind...', flush=True)
        runs[name] = fly(survey, w, airframe, battery)
        results[name] = metrics(runs[name], survey, battery)
    sweep = speed_sweep(survey, airframe, battery)
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(survey=asdict(survey), battery=asdict(battery), airframe=asdict(airframe),
                  takeoff_mass_kg=airframe.dry_mass_kg + battery.mass_kg, results=results,
                  speed_sweep=dict(speed_m_s=sweep[:, 0].tolist(), line_minutes=sweep[:, 1].tolist(),
                                   energy_wh=sweep[:, 2].tolist(), fraction_of_pack=sweep[:, 3].tolist()),
                  limitation=__doc__.strip().split('\n\n')[-1])
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for name, data in runs.items():
        np.savez_compressed(args.output / f'{name}.npz', **{k: v for k, v in data.items()})
    print(json.dumps(results, indent=2))
    if not args.no_visuals:
        from survey_visuals import export
        print(f'Visual saved: {export(runs, results, survey, battery, sweep, args.output).resolve()}')


if __name__ == '__main__':
    main()

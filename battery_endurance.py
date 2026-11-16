"""Battery and flight-time model for the selected 6S 9000 mAh 80C LiPo. Simulation only.

Answers: how long can the aircraft hover or cruise, which speed flies longest or
farthest, how flight time changes with the (still unweighed) battery and airframe,
and whether the pack and motors have enough margin.

Power: momentum theory per rotor, including forward flight (Glauert inflow) and body
drag, divided by a figure of merit and drive (ESC + motor) efficiency, plus the
avionics load. Same FM, drive efficiency and 45 W avionics as gust_rejection.PowerModel
(the team's Prop_and_Motor_Calcs.xlsx), so hover numbers agree with that model.

Battery: open-circuit voltage vs state of charge for a LiPo cell plus internal
resistance, so voltage sags under load and falls as the pack empties. A flight ends at
the landing reserve (state of charge) or the under-load voltage cutoff, whichever first.

Every number marked "estimate" is unmeasured. Replace with scale/bench values.
"""
from dataclasses import dataclass, asdict, replace
from pathlib import Path
import argparse
import json
import numpy as np

from flight_dynamics import Parameters

RHO = 1.225
G = 9.81

# Typical LiPo resting cell voltage vs state of charge (0 = empty, 1 = full).
LIPO_SOC = np.array([0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
LIPO_OCV = np.array([3.27, 3.61, 3.69, 3.74, 3.77, 3.79, 3.82, 3.87, 3.92, 3.98, 4.06, 4.20])


@dataclass(frozen=True)
class Battery:
    cells: int = 6                     # 6S assumed (22.2 V nominal, as in hardware.py)
    capacity_ah: float = 9.0           # selected pack
    c_rating: float = 80.0             # selected pack, continuous
    mass_kg: float = 1.15              # ESTIMATE: comparable 6S ~9 Ah packs are ~1.06-1.18 kg
    cell_resistance_ohm: float = 0.0025  # ESTIMATE for a high-C ~9 Ah cell; measure with charger
    wiring_resistance_ohm: float = 0.004  # ESTIMATE: leads, connector, PDB
    cutoff_cell_v: float = 3.5         # land if under-load voltage falls below this
    reserve_soc: float = 0.20          # land with 20 % left (LiPo life + safety)

    @property
    def resistance(self):
        return self.cells * self.cell_resistance_ohm + self.wiring_resistance_ohm

    @property
    def max_current_a(self):
        return self.c_rating * self.capacity_ah

    @property
    def nominal_wh(self):
        return 3.7 * self.cells * self.capacity_ah

    def ocv(self, soc):
        return self.cells * np.interp(np.clip(soc, 0, 1), LIPO_SOC, LIPO_OCV)

    def current_for_power(self, watts, soc):
        """Current that delivers `watts` at the terminals: P = (OCV - I R) I.

        Returns nan if the pack cannot deliver that power at all at this charge.
        """
        v, r = self.ocv(soc), self.resistance
        disc = v * v - 4 * r * np.asarray(watts, dtype=float)
        return np.where(disc >= 0, (v - np.sqrt(np.maximum(disc, 0))) / (2 * r), np.nan)


@dataclass(frozen=True)
class Airframe:
    dry_mass_kg: float = 1.05          # everything except the battery: 2.2 kg placeholder - 1.15 kg
    rotors: int = 4
    prop_diameter_m: float = 9 * 0.0254
    figure_of_merit: float = 0.65      # same as gust_rejection.PowerModel
    drive_efficiency: float = 0.85     # ESC x motor, same as PowerModel's esc_efficiency
    avionics_w: float = 45.0           # FC, Pi 5, L2, telemetry; same as PowerModel
    cda_horizontal: float = 0.05       # m^2, same estimate as gust_rejection.Aero
    max_thrust_per_rotor_n: float = Parameters().max_thrust  # unvalidated static-thrust formula

    @property
    def disk_area(self):
        return np.pi * (self.prop_diameter_m / 2) ** 2


def induced_velocity(thrust, axial, edgewise, area):
    """Glauert inflow: v = vh^2 / sqrt(edge^2 + (axial + v)^2), solved by damped iteration.

    thrust per rotor (N); axial = airspeed component along the thrust axis (climb > 0);
    edgewise = airspeed component across the disk. Valid for hover, climb and forward flight;
    steep descent (vortex ring) is outside this model.
    """
    thrust, axial, edgewise = np.broadcast_arrays(*(np.asarray(x, float) for x in (thrust, axial, edgewise)))
    vh2 = np.maximum(thrust, 0) / (2 * RHO * area)
    v = np.sqrt(vh2)
    for _ in range(30):  # converges to ~1e-6 m/s in ~20 iterations
        v = 0.5 * v + 0.5 * vh2 / np.sqrt(edgewise ** 2 + (axial + v) ** 2 + 1e-12)
    return v


def rotor_power(thrust, axial, edgewise, airframe):
    """Electrical power for ONE rotor (W), before avionics."""
    vi = induced_velocity(thrust, axial, edgewise, airframe.disk_area)
    shaft = np.maximum(thrust * (vi + axial), 0) / airframe.figure_of_merit
    return shaft / airframe.drive_efficiency


def total_mass(airframe, battery):
    return airframe.dry_mass_kg + battery.mass_kg


def level_flight(airframe, battery, airspeed, climb=0.0):
    """Steady straight flight at `airspeed` (m/s, relative to the air): power and attitude."""
    V = np.asarray(airspeed, dtype=float)
    weight = total_mass(airframe, battery) * G
    drag = 0.5 * RHO * airframe.cda_horizontal * V ** 2
    thrust = np.hypot(weight, drag)
    tilt = np.arctan2(drag, weight)
    per_rotor = thrust / airframe.rotors
    axial = V * np.sin(tilt) + climb * np.cos(tilt)
    edgewise = V * np.cos(tilt)
    electric = airframe.rotors * rotor_power(per_rotor, axial, edgewise, airframe) + airframe.avionics_w
    return dict(power_w=electric, tilt_deg=np.degrees(tilt), thrust_per_rotor_n=per_rotor,
                throttle=np.sqrt(per_rotor / airframe.max_thrust_per_rotor_n))


def discharge(battery, power_w, dt=1.0, max_s=4 * 3600):
    """Fly a power demand until reserve or voltage cutoff.

    power_w: constant watts or a function of time (s) -> watts.
    """
    demand = power_w if callable(power_w) else (lambda _t: power_w)
    t, soc, rows = 0.0, 1.0, []
    reason = 'time limit'
    while t <= max_s:
        p = float(demand(t))
        i = float(battery.current_for_power(p, soc))
        if not np.isfinite(i):
            reason = 'pack cannot supply the power'
            break
        v = float(battery.ocv(soc)) - i * battery.resistance
        rows.append((t, soc, v, i, p))
        if soc <= battery.reserve_soc:
            reason = 'landing reserve reached'
            break
        if v / battery.cells < battery.cutoff_cell_v:
            reason = 'voltage cutoff under load'
            break
        soc -= i * dt / 3600 / battery.capacity_ah
        t += dt
    data = np.array(rows) if rows else np.zeros((0, 5))
    return dict(time_s=data[:, 0], soc=data[:, 1], voltage_v=data[:, 2], current_a=data[:, 3],
                power_w=data[:, 4], endurance_s=float(data[-1, 0]) if len(data) else 0.0, end=reason)


def flight_minutes(airframe, battery, airspeed=0.0, dt=2.0):
    power = float(level_flight(airframe, battery, airspeed)['power_w'])
    return discharge(battery, power, dt)['endurance_s'] / 60


def study(airframe=Airframe(), battery=Battery()):
    """Everything main() reports, as plain data (also used by the tests)."""
    mass = total_mass(airframe, battery)
    speeds = np.linspace(0, 16, 81)
    curve = level_flight(airframe, battery, speeds)
    power = curve['power_w']
    best_endurance = int(np.argmin(power))
    best_range = int(np.argmin(power[1:] / speeds[1:])) + 1
    hover = discharge(battery, float(power[0]))

    def at_speed(k):
        minutes = discharge(battery, float(power[k]), 2.0)['endurance_s'] / 60
        return dict(speed_m_s=float(speeds[k]), power_w=float(power[k]), flight_min=minutes,
                    range_km=speeds[k] * minutes * 60 / 1000, tilt_deg=float(curve['tilt_deg'][k]))

    # Margins
    full_power = (airframe.rotors * float(rotor_power(airframe.max_thrust_per_rotor_n, 0, 0, airframe))
                  + airframe.avionics_w)
    full_current = float(battery.current_for_power(full_power, 0.5))

    battery_masses = np.round(np.arange(0.95, 1.40, 0.05), 2)
    dry_masses = np.round(np.arange(0.85, 1.50, 0.05), 2)
    grid = np.array([[flight_minutes(replace(airframe, dry_mass_kg=d), replace(battery, mass_kg=b))
                      for b in battery_masses] for d in dry_masses])
    return dict(
        mass_kg=mass, hover=dict(power_w=float(power[0]), current_a=float(hover['current_a'][0]),
                                 throttle=float(curve['throttle'][0]),
                                 flight_min=hover['endurance_s'] / 60, ends_by=hover['end'],
                                 voltage_start_v=float(hover['voltage_v'][0]),
                                 voltage_end_v=float(hover['voltage_v'][-1])),
        best_endurance=at_speed(best_endurance), best_range=at_speed(best_range),
        margins=dict(thrust_to_weight=airframe.rotors * airframe.max_thrust_per_rotor_n / (mass * G),
                     full_throttle_power_w=full_power, full_throttle_current_a=full_current,
                     pack_continuous_limit_a=battery.max_current_a,
                     pack_limit_used_at_full_throttle=full_current / battery.max_current_a),
        curves=dict(speed_m_s=speeds, power_w=power, tilt_deg=curve['tilt_deg']),
        hover_trace=hover,
        mass_sweep=dict(battery_kg=battery_masses, dry_kg=dry_masses, hover_min=grid),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--battery-kg', type=float, default=Battery.mass_kg,
                        help='Weighed battery mass (default 1.15 kg estimate)')
    parser.add_argument('--dry-kg', type=float, default=Airframe.dry_mass_kg,
                        help='Aircraft mass without battery (default 1.05 kg placeholder)')
    parser.add_argument('--avionics-w', type=float, default=Airframe.avionics_w)
    parser.add_argument('--reserve', type=float, default=Battery.reserve_soc,
                        help='Landing reserve as a fraction of capacity (default 0.2)')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'outputs/endurance')
    parser.add_argument('--no-visuals', action='store_true')
    args = parser.parse_args()
    if not (0 < args.battery_kg and 0 < args.dry_kg and 0 <= args.reserve < 1 and args.avionics_w >= 0):
        parser.error('masses must be positive, reserve in [0, 1), avionics >= 0')
    airframe = Airframe(dry_mass_kg=args.dry_kg, avionics_w=args.avionics_w)
    battery = Battery(mass_kg=args.battery_kg, reserve_soc=args.reserve)
    result = study(airframe, battery)
    args.output.mkdir(parents=True, exist_ok=True)
    report = dict(battery=asdict(battery), airframe=asdict(airframe),
                  **{k: result[k] for k in ('mass_kg', 'hover', 'best_endurance', 'best_range', 'margins')},
                  limitation=('Simulation only. Masses, FM, drive efficiency, drag area, cell resistance and '
                              'avionics power are estimates. No temperature, ageing, rotor profile drag, '
                              'ground effect or vortex-ring descent. Flight times assume steady flight.'))
    (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    c = result['curves']
    np.savetxt(args.output / 'power_curve.csv', np.column_stack([c['speed_m_s'], c['power_w'], c['tilt_deg']]),
               delimiter=',', header='airspeed_m_s,power_w,tilt_deg', comments='', fmt='%.4f')
    sweep = result['mass_sweep']
    np.savetxt(args.output / 'mass_sweep_hover_min.csv',
               np.column_stack([sweep['dry_kg'], sweep['hover_min']]), delimiter=',', fmt='%.2f',
               header='dry_kg \\ battery_kg,' + ','.join(f'{b:.2f}' for b in sweep['battery_kg']), comments='')
    print(json.dumps({k: report[k] for k in ('mass_kg', 'hover', 'best_endurance', 'best_range', 'margins')},
                     indent=2))
    if not args.no_visuals:
        from endurance_visuals import export
        print(f"Visual saved: {export(result, battery, airframe, args.output).resolve()}")


if __name__ == '__main__':
    main()

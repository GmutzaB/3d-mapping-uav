"""Selected aircraft used by the airflow and rigid-body models.

Thrust uses an unvalidated static-thrust estimate. Battery capacity is recorded
as energy only; it does not change RPM or thrust. Pitch and C rating are omitted
until a specification is supplied.
"""

import math

MOTOR_KV = 900
PROP_DIAMETER_IN = 9.0
BATTERY_CELLS = 6
NOMINAL_CELL_VOLTAGE_V = 3.7
BATTERY_MAH_OPTIONS = (7000, 10000)
DEFAULT_BATTERY_MAH = 7000
MASS_KG = 2.2
THRUST_COEFFICIENT = 0.11
LOADED_RPM_FACTOR = 0.80
AIR_DENSITY_KG_M3 = 1.225


def nominal_energy_wh(capacity_mah, cells=BATTERY_CELLS):
    return NOMINAL_CELL_VOLTAGE_V * cells * capacity_mah / 1000


def propeller(throttle, motor_kv=MOTOR_KV, battery_cells=BATTERY_CELLS, prop_diameter_in=PROP_DIAMETER_IN):
    """Empirical static thrust estimate, independent of the flow solver."""
    diameter = prop_diameter_in * 0.0254
    rpm = motor_kv * (NOMINAL_CELL_VOLTAGE_V * battery_cells) * LOADED_RPM_FACTOR * throttle
    thrust = THRUST_COEFFICIENT * AIR_DENSITY_KG_M3 * (rpm / 60) ** 2 * diameter ** 4
    area = math.pi * diameter ** 2 / 4
    induced = 0.0 if thrust <= 0 else float((thrust / (2 * AIR_DENSITY_KG_M3 * area)) ** 0.5)
    return dict(
        rpm=rpm,
        thrust_per_motor_N=thrust,
        total_quad_thrust_N=4 * thrust,
        equivalent_supported_mass_kg=4 * thrust / 9.81,
        ideal_disk_pressure_jump_Pa=thrust / area,
        ideal_induced_velocity_m_s=induced,
    )


def hardware_caption(motor_kv, cells, capacity_mah, diameter_in, pitch_in=None, c_rating=None):
    """One-line description of the hardware a run actually used."""
    others = [pack for pack in BATTERY_MAH_OPTIONS if pack != capacity_mah]
    also = f" (also {others[0]:g} mAh)" if len(others) == 1 else ""
    rating = f" {c_rating:g}C" if c_rating else ""
    if pitch_in:
        prop = f"{diameter_in:g}×{pitch_in:g} in prop (pitch not modeled)"
    else:
        prop = f"{diameter_in:g} in prop (pitch not specified or modeled)"
    return (
        f"Provisional: {motor_kv:g} KV, {cells}S {capacity_mah:g} mAh{rating}{also} | "
        f"{prop}, Ct={THRUST_COEFFICIENT:g}"
    )

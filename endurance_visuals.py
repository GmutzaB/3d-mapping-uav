"""Summary chart for battery_endurance.py: power curve, discharge, mass sensitivity."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).parent / '.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']   # categorical slots 1-3, fixed order


def _style(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc='left', fontsize=11.5, color=INK, fontweight='bold', pad=8)
    ax.set_xlabel(xlabel, color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.grid(color=GRID, linewidth=0.8)
    ax.tick_params(colors=INK2, length=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)


def export(result, battery, airframe, output):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), facecolor=SURFACE)
    (a, b), (c, d) = axes

    curve = result['curves']
    _style(a, 'Power needed vs. airspeed (level flight)', 'Airspeed (m/s)', 'Battery power (W)')
    a.plot(curve['speed_m_s'], curve['power_w'], color=SERIES[0], linewidth=2)
    for key, label, dx, dy in (('best_endurance', 'Longest flight', 0, 32), ('best_range', 'Farthest flight', -78, 12)):
        p = result[key]
        a.plot(p['speed_m_s'], p['power_w'], 'o', color=SERIES[0], markersize=8,
               markeredgecolor=SURFACE, markeredgewidth=2)
        a.annotate(f"{label}: {p['speed_m_s']:.1f} m/s\n{p['flight_min']:.0f} min, {p['range_km']:.1f} km",
                   (p['speed_m_s'], p['power_w']), xytext=(dx, dy), textcoords='offset points',
                   ha='center', va='center', fontsize=9, color=INK)
    h = result['hover']
    a.annotate(f"Hover: {h['power_w']:.0f} W", (0, h['power_w']), xytext=(8, 10),
               textcoords='offset points', fontsize=9, color=INK)

    trace = result['hover_trace']
    minutes = trace['time_s'] / 60
    _style(b, 'Pack voltage while hovering', 'Time (min)', 'Voltage under load (V)')
    b.plot(minutes, trace['voltage_v'], color=SERIES[0], linewidth=2)
    cutoff = battery.cutoff_cell_v * battery.cells
    b.axhline(cutoff, color=INK2, linewidth=1, linestyle='--')
    b.text(minutes[-1], cutoff + 0.08, f'cutoff {cutoff:.1f} V', ha='right', va='bottom', fontsize=9, color=INK2)
    b.plot(minutes[-1], trace['voltage_v'][-1], 'o', color=SERIES[0], markersize=8,
           markeredgecolor=SURFACE, markeredgewidth=2)
    b.annotate(f"Land: {battery.reserve_soc:.0%} left at {minutes[-1]:.1f} min",
               (minutes[-1], trace['voltage_v'][-1]), xytext=(-6, -22), textcoords='offset points',
               ha='right', fontsize=9, color=INK)
    b.set_ylim(cutoff - 0.6, trace['voltage_v'].max() + 0.4)

    sweep = result['mass_sweep']
    _style(c, 'Hover flight time vs. battery weight', 'Battery mass (kg)', 'Hover time to reserve (min)')
    picks = [airframe.dry_mass_kg - 0.15, airframe.dry_mass_kg, airframe.dry_mass_kg + 0.25]
    for color, dry in zip(SERIES, picks):
        row = int(np.argmin(abs(sweep['dry_kg'] - dry)))
        y = sweep['hover_min'][row]
        c.plot(sweep['battery_kg'], y, color=color, linewidth=2, label=f'airframe {sweep["dry_kg"][row]:.2f} kg')
        c.text(sweep['battery_kg'][-1] + 0.01, y[-1], f'{sweep["dry_kg"][row]:.2f} kg', va='center',
               fontsize=9, color=INK)
    c.axvline(battery.mass_kg, color=INK2, linewidth=1, linestyle=':')
    c.text(battery.mass_kg, c.get_ylim()[1], ' estimate', va='top', fontsize=9, color=INK2)
    c.set_xlim(sweep['battery_kg'][0], sweep['battery_kg'][-1] + 0.09)
    c.legend(title='Airframe without battery', frameon=False, fontsize=9, title_fontsize=9, loc='upper right')

    m = result['margins']
    d.axis('off')
    d.set_facecolor(SURFACE)
    lines = [
        ('Selected pack', f'6S {battery.capacity_ah * 1000:.0f} mAh {battery.c_rating:.0f}C, '
                          f'{battery.nominal_wh:.0f} Wh'),
        ('Takeoff mass used', f"{result['mass_kg']:.2f} kg ({battery.mass_kg:.2f} battery est. "
                              f"+ {airframe.dry_mass_kg:.2f} airframe)"),
        ('Hover', f"{h['power_w']:.0f} W, {h['current_a']:.1f} A, throttle {h['throttle']:.0%}, "
                  f"{h['flight_min']:.1f} min"),
        ('Best-endurance cruise', f"{result['best_endurance']['speed_m_s']:.1f} m/s, "
                                  f"{result['best_endurance']['flight_min']:.1f} min"),
        ('Thrust / weight', f"{m['thrust_to_weight']:.1f} : 1 (unvalidated thrust formula)"),
        ('Full-throttle current', f"{m['full_throttle_current_a']:.0f} A of {m['pack_continuous_limit_a']:.0f} A "
                                  f"pack limit ({m['pack_limit_used_at_full_throttle']:.0%})"),
        ('Landing rule', f"{battery.reserve_soc:.0%} capacity left or "
                         f"{battery.cutoff_cell_v:.2f} V/cell under load"),
    ]
    d.text(0, 1, 'Key numbers', fontsize=11.5, fontweight='bold', color=INK, va='top')
    for k, (name, value) in enumerate(lines):
        y = 0.86 - k * 0.125
        d.text(0, y, name, fontsize=10, color=INK2, va='top')
        d.text(0, y - 0.05, value, fontsize=10.5, color=INK, va='top')

    fig.suptitle('Flight time and battery — simulation', x=0.06, ha='left', fontsize=15,
                 fontweight='bold', color=INK)
    fig.text(0.06, 0.925, 'Momentum-theory rotor power + LiPo voltage model. Masses, efficiencies, drag and '
             'cell resistance are estimates — not measured flight data.', fontsize=10, color=INK2)
    fig.tight_layout(rect=(0.03, 0, 1, 0.91), h_pad=2.5, w_pad=3)
    path = Path(output) / 'endurance.png'
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    return path

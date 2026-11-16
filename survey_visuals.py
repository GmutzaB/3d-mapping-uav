"""Summary chart for survey_flight.py: flown path and coverage, tracking, battery, comparison."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).parent / '.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import numpy as np

from endurance_visuals import GRID, INK, INK2, SERIES, SURFACE, _style
from survey_flight import coverage

COVER = '#cfe0f5'   # light step of series 1, for the covered area


def _label(name):
    return name.replace('wind_', '').replace('_m_s', ' m/s wind')


def export(runs, results, survey, battery, sweep, output):
    names = list(runs)
    focus = next((n for n in names if n == 'wind_5_m_s'), names[len(names) // 2])
    data, res = runs[focus], results[focus]
    colors = {n: SERIES[k % len(SERIES)] for k, n in enumerate(names)}   # fixed by entity

    fig = plt.figure(figsize=(16, 9.5), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.35, 1, 1], hspace=0.42, wspace=0.28)

    # --- top-down map
    ax = fig.add_subplot(gs[:, 0])
    _style(ax, f'Flown survey, top view ({_label(focus)})', 'East (m)', 'North (m)')
    _, cells, covered = coverage(survey, data['state'][data['surveying']][::12, :2])
    rows, cols = len(np.unique(cells[:, 0])), len(np.unique(cells[:, 1]))
    ax.imshow(np.where(covered.reshape(rows, cols), 1.0, np.nan), origin='lower',
              extent=(0, survey.width_m, 0, survey.length_m), cmap=ListedColormap([COVER]),
              interpolation='nearest', zorder=0)
    ax.add_patch(Rectangle((0, 0), survey.width_m, survey.length_m, fill=False, edgecolor=INK2,
                           linestyle='--', linewidth=1))
    s = data['state']
    ax.plot(s[:, 1], s[:, 0], color=SERIES[0], linewidth=1.6)
    home = data['waypoints'][0]
    ax.plot(home[1], home[0], 's', color=INK, markersize=8)
    ax.annotate('Takeoff / land', (home[1], home[0]), xytext=(8, -12), textcoords='offset points',
                fontsize=9, color=INK)
    ax.text(survey.width_m / 2, survey.length_m + 3, f'Area {survey.width_m:g} x {survey.length_m:g} m, '
            f'{res["coverage"]:.0%} inside the {survey.swath_m:g} m LiDAR swath', ha='center',
            fontsize=9.5, color=INK)
    mean_wind = data['wind'].mean(axis=0)
    if np.linalg.norm(mean_wind[:2]) > 0.1:
        x0, y0 = survey.width_m * 0.82, -survey.home_offset_m * 0.6
        ax.annotate('', (x0 + 2.2 * mean_wind[1], y0 + 2.2 * mean_wind[0]), (x0, y0),
                    arrowprops=dict(arrowstyle='-|>', color=INK2, linewidth=1.6))
        ax.text(x0, y0 - 2.5, f'wind {np.linalg.norm(mean_wind[:2]):.0f} m/s', fontsize=9,
                color=INK2, ha='left', va='top')
    ax.set_aspect('equal')
    ax.margins(0.06)

    t = data['time'] / 60
    # --- line tracking error
    ax = fig.add_subplot(gs[0, 1])
    _style(ax, 'How far off the planned path', 'Time (min)', 'Horizontal error (m)')
    err = np.linalg.norm(s[:, :2] - data['ref_pos'][:, :2], axis=1)
    ax.plot(t, err, color=SERIES[0], linewidth=1.2)
    ax.text(0.02, 0.95, f'on survey lines: RMS {res["rms_line_error_m"]:.2f} m, max {res["max_line_error_m"]:.2f} m',
            transform=ax.transAxes, va='top', fontsize=9, color=INK)

    # --- battery used per wind case (same plan)
    ax = fig.add_subplot(gs[0, 2])
    _style(ax, 'Battery used for this survey', 'Share of the pack (%)', '')
    used = [100 * results[n]['battery_used'] for n in names]
    y = np.arange(len(names))[::-1]
    ax.barh(y, used, height=0.55, color=SERIES[0], edgecolor=SURFACE, linewidth=2)
    for yy, n, u in zip(y, names, used):
        r = results[n]
        ax.text(u + 1, yy, f'{u:.0f}%  ({r["duration_min"]:.1f} min, min {r["min_voltage_v"]:.1f} V)',
                va='center', fontsize=9, color=INK)
    ax.set_yticks(y, [_label(n) for n in names])
    usable = 100 * (1 - battery.reserve_soc)
    ax.axvline(usable, color=INK2, linestyle='--', linewidth=1)
    ax.text(usable - 1, y.min() - 0.45, 'usable before reserve', ha='right', va='center', fontsize=9, color=INK2)
    ax.set_xlim(0, 100)
    ax.grid(axis='y', visible=False)

    # --- electrical power
    ax = fig.add_subplot(gs[1, 1])
    _style(ax, f'Battery power ({_label(focus)})', 'Time (min)', 'Power (W)')
    k = max(1, int(round(1.0 / (data['time'][1] - data['time'][0]))))   # 1 s average
    smooth = np.convolve(data['power_w'], np.ones(k) / k, mode='valid')
    ax.plot(t[k // 2: k // 2 + len(smooth)], smooth, color=SERIES[0], linewidth=1.4)
    ax.text(0.02, 0.95, f'mean {res["mean_power_w"]:.0f} W, peak {res["peak_power_w"]:.0f} W '
            f'({res["peak_current_a"]:.0f} A)', transform=ax.transAxes, va='top', fontsize=9, color=INK)

    # --- survey speed choice
    ax = fig.add_subplot(gs[1, 2])
    _style(ax, 'Pick a line speed (steady-flight estimate)', 'Survey speed (m/s)', 'Pack used per survey (%)')
    ax.plot(sweep[:, 0], 100 * sweep[:, 3], color=SERIES[0], linewidth=2, marker='o', markersize=5,
            markeredgecolor=SURFACE, markeredgewidth=1.5)
    best = int(np.argmin(sweep[:, 3]))
    area_ha = survey.width_m * survey.length_m / 1e4 * (1 - battery.reserve_soc) / sweep[best, 3]
    ax.annotate(f'least battery: {sweep[best, 0]:.0f} m/s\n~{area_ha:.0f} ha per pack at this spacing',
                (sweep[best, 0], 100 * sweep[best, 3]), xytext=(0, 26), textcoords='offset points',
                ha='center', fontsize=9, color=INK)
    ax.set_ylim(0, 100 * sweep[:, 3].max() * 1.25)

    summary = ', '.join(f'{_label(n)}: {results[n]["duration_min"]:.1f} min, '
                        f'{100 * results[n]["battery_used"]:.0f}% used' for n in names)
    fig.suptitle('Mapping survey flight — simulation', x=0.04, ha='left', fontsize=15,
                 fontweight='bold', color=INK)
    fig.text(0.04, 0.905, f'6-DOF quad + Dryden wind + forward-flight rotor power + 6S 9000 mAh model; '
             f'{survey.speed_m_s:g} m/s lines at {survey.altitude_m:g} m, {survey.line_spacing_m:g} m apart.\n'
             f'{summary}.  Unmeasured mass, drag and efficiencies; perfect position sensing.',
             fontsize=9.5, color=INK2, linespacing=1.5)
    fig.subplots_adjust(left=0.04, right=0.98, top=0.84, bottom=0.07)
    path = Path(output) / 'survey.png'
    fig.savefig(path, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    return path

"""Run a controlled 32/64/128 grid study; export shared-scale plots and CSV metrics."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

import simulation  # Sets the local Matplotlib cache directory.
from hardware import hardware_caption
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('outputs/grid_comparison'))
    args, simulation_args = parser.parse_known_args()
    if any(a.split('=')[0] in ('--grid', '--output') for a in simulation_args):
        parser.error('Grid sizes and per-run output folders are controlled by this script')
    args.output.mkdir(parents=True, exist_ok=True)
    summaries, fields = [], []
    for n in (32, 64, 128):
        folder = args.output / f'grid_{n}'
        print(f'Running {n} × {n}...', flush=True)
        subprocess.run([sys.executable, str(Path(simulation.__file__).resolve()),
                        *simulation_args, '--grid', str(n), '--output', str(folder)], check=True,
                       stdout=subprocess.DEVNULL)
        summaries.append(json.loads((folder/'summary.json').read_text()))
        with np.load(folder/'flow.npz') as data:
            fields.append({k: data[k].copy() for k in data.files})
    rows = []
    for i, (s, f) in enumerate(zip(summaries, fields)):
        row = {k: s[k] for k in ('grid', 'simulated_seconds', 'throttle',
               'effective_viscosity_m2_s', 'force_thickness_m', 'peak_speed_m_s',
               'rms_speed_m_s', 'pressure_min_Pa', 'pressure_max_Pa', 'max_divergence_per_s')}
        row['velocity_relative_L2_difference_to_next_grid'] = ''
        row['pressure_relative_L2_difference_to_next_grid'] = ''
        if i < 2:
            finer = fields[i+1]
            for label, keys in [('velocity', ('u', 'v')), ('pressure', ('pressure_Pa',))]:
                reference = np.stack([finer[k][::2, ::2] for k in keys])
                coarse = np.stack([f[k] for k in keys])
                denominator = np.linalg.norm(reference)
                row[f'{label}_relative_L2_difference_to_next_grid'] = float(
                    np.linalg.norm(coarse-reference)/denominator) if denominator else 0.0
        rows.append(row)
    with (args.output/'comparison.csv').open('w', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    speeds = [np.hypot(f['u'], f['v']) for f in fields]
    vmax = max(float(s.max()) for s in speeds) or 1
    pmax = max(float(np.abs(f['pressure_Pa']).max()) for f in fields) or 1
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), layout='constrained')
    for j, (s, f, speed) in enumerate(zip(summaries, fields, speeds)):
        for i, (field, cmap, lo, hi) in enumerate([
                (speed, 'viridis', 0, vmax), (f['pressure_Pa'], 'RdBu_r', -pmax, pmax)]):
            ax = axes[i, j]
            im = ax.pcolormesh(f['x'], f['y'], field, shading='auto', cmap=cmap, vmin=lo, vmax=hi)
            radius = s['propeller_diameter_m'] / 2
            ax.plot([-radius, radius], [0.12, 0.12], 'r-', lw=2)
            ax.set(title=f"{s['grid']} × {s['grid']} — " + ('speed' if i == 0 else 'pressure'),
                   xlabel='x [m]', ylabel='y [m]', aspect='equal')
            if j == 2:
                fig.colorbar(im, ax=list(axes[i]), label='m/s' if i == 0 else 'Pa')
    s = summaries[0]
    fig.suptitle(f"Grid comparison | t={s['simulated_seconds']:g} s | throttle={s['throttle']:.0%} | "
                 f"viscosity={s['effective_viscosity_m2_s']:g} m²/s | forcing width={s['force_thickness_m']:g} m\n"
                 + hardware_caption(s['motor_kv'], s['battery_cells'], s['battery_capacity_mAh'],
                                    s['propeller_diameter_in'], s.get('propeller_pitch_in'), s.get('battery_c_rating')))
    fig.savefig(args.output/'comparison.png', dpi=160)
    plt.close(fig)
    note = ['Grid study: differences are numerical comparisons, not validation against real hardware.',
            'All runs use the same physical forcing thickness and inputs.',
            'Relative L2 differences compare fields at coincident grid points; 128 is not an exact reference.',
            'Time steps also vary with resolution. A separate time-step study is needed to isolate spatial error.',
            'Battery capacity and C rating are recorded only; this model does not predict current or endurance.', '']
    for row in rows[:-1]:
        note.append(f"{row['grid']} to {2*row['grid']}: velocity difference "
                    f"{row['velocity_relative_L2_difference_to_next_grid']:.1%}; pressure difference "
                    f"{row['pressure_relative_L2_difference_to_next_grid']:.1%}")
    (args.output/'comparison.txt').write_text('\n'.join(note)+'\n')
    print('\n'.join(note))
    print(f'Results: {args.output.resolve()}')


if __name__ == '__main__':
    main()

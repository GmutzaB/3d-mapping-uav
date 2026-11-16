"""Animation and summary charts for gust_rejection.py; replays saved samples only."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).parent / '.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image

from flight_dynamics import Quad, rotation

NAVY, TEAL, ORANGE, PURPLE, ROSE, GREY = '#142d4e', '#087f8c', '#ed7e32', '#6c58a6', '#cf4770', '#4a5d73'
BG = '#f4f7fb'
PROP_RADIUS = 9 * 0.0254 / 2


def enu(ned):
    """NED vectors -> plot axes (east, north, up)."""
    ned = np.asarray(ned)
    return np.stack((ned[..., 1], ned[..., 0], -ned[..., 2]), axis=-1)


def phase(t, wind):
    s = wind['settings']
    if t < s['ramp_s']:
        return 'WIND BUILDING'
    if s['gust_start'] <= t <= s['gust_start'] + s['gust_duration']:
        return 'DISCRETE CROSSWIND GUST'
    if s['gust_start'] + s['gust_duration'] < t <= s['gust_start'] + s['gust_duration'] + 3:
        return 'GUST RECOVERY'
    return 'STEADY WIND + DRYDEN TURBULENCE'


def draw_drone(ax, state, offset, motors):
    R = rotation(state[6:10])
    pos = state[:3] - offset
    world = motors @ R.T + pos
    for pair, color in [([0, 2], TEAL), ([1, 3], ORANGE)]:
        p = enu(world[pair])
        ax.plot(p[:, 0], p[:, 1], p[:, 2], color=color, lw=4, solid_capstyle='round', zorder=5)
    angle = np.linspace(0, 2 * np.pi, 28)
    disks = []
    for m in motors:
        ring = m + PROP_RADIUS * np.column_stack((np.cos(angle), np.sin(angle), 0 * angle))
        disks.append(enu(ring @ R.T + pos))
    ax.add_collection3d(Poly3DCollection(disks, facecolor='#8fb8d6', edgecolor=NAVY, lw=.6, alpha=.45))
    c = enu(pos)
    ax.scatter(*c, color=NAVY, s=40, depthshade=False, zorder=6)
    return c


def export(datasets, report, output):
    output = Path(output)
    hold, drift = datasets['position_hold'], datasets['attitude_only']
    t = hold['time']
    settings = report['wind']
    wind = dict(settings=settings)
    w_enu = enu(hold['wind'])
    speed = np.linalg.norm(hold['wind'], axis=1)
    h_dist = np.linalg.norm(hold['error'][:, :2], axis=1)
    d_dist = np.linalg.norm(drift['error'][:, :2], axis=1)
    target = hold['target']
    motors = Quad().positions
    res = report['results']['position_hold']
    res_d = report['results']['attitude_only']
    gust_span = (settings['gust_start'], settings['gust_start'] + settings['gust_duration'])
    dry = report['dryden']

    # ---------------------------------------------------------------- summary PNG
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.patch.set_facecolor(BG)
    fig.subplots_adjust(left=.07, right=.93, top=.86, bottom=.12, hspace=.38, wspace=.28)
    fig.suptitle('DRONE WIND-GUST REJECTION / DRYDEN TURBULENCE', x=.07, ha='left', y=.975,
                 fontsize=19, fontweight='bold', color=NAVY)
    fig.text(.07, .925, f"Mean wind {settings['mean_speed']:g} m/s from the west  •  Dryden turbulence (MIL-HDBK-1797) "
             f"σ = {dry['sigma_m_s'][0]:.2f}/{dry['sigma_m_s'][1]:.2f}/{dry['sigma_m_s'][2]:.2f} m/s  •  "
             f"{settings['gust_amplitude']:g} m/s 1-cosine crosswind gust at {settings['gust_start']:g} s", fontsize=10.5)
    fig.text(.07, .898, f"9 in props • 900 KV • 6S • provisional {report['aircraft']['mass']:g} kg • "
             f"hold altitude {settings['altitude']:g} m • same wind sample for both controllers", fontsize=10.5)
    fig.text(.07, .03, 'SIMULATION ONLY • Unmeasured drag areas, mass and inertia • Perfect state feedback (no GPS/EKF noise) • '
             'Wind uniform across the airframe\nPower: hover momentum theory with the team spreadsheet\'s FM 0.65, ESC 0.85, 45 W '
             'electronics, 10 Ah, 80% usable; ignores translational lift.', fontsize=9.5, color=GREY)

    a = axes[0, 0]
    for k, (label, color) in enumerate([('East (mean wind + turbulence)', TEAL), ('North (turbulence + gust)', ORANGE),
                                        ('Up (turbulence)', PURPLE)]):
        a.plot(t, w_enu[:, k], color=color, lw=1.3, label=label)
    a.plot(t, speed, color=NAVY, lw=1, ls='--', label='Total speed')
    a.set_title('Wind at the aircraft', loc='left', fontweight='bold', color=NAVY)
    a.set_ylabel('m/s'); a.legend(fontsize=8, ncol=2, loc='upper left')

    a = axes[0, 1]
    pts = enu(hold['error'])
    sc = a.scatter(pts[:, 0] * 100, pts[:, 1] * 100, c=t, cmap='viridis', s=2)
    a.add_patch(plt.Circle((0, 0), 10, fill=False, ls='--', color=GREY))
    a.text(0, 10.8, '10 cm', ha='center', fontsize=8, color=GREY)
    a.plot(0, 0, '+', color=ROSE, ms=14, mew=2)
    a.set_aspect('equal'); a.set_xlabel('East of hold point (cm)'); a.set_ylabel('North of hold point (cm)')
    lim = max(15, np.abs(pts[:, :2]).max() * 100 * 1.2)
    a.set_xlim(-lim, lim); a.set_ylim(-lim, lim)
    a.set_title('Position hold: top-down track (color = time)', loc='left', fontweight='bold', color=NAVY)
    fig.colorbar(sc, ax=a, label='Time (s)', pad=.02)

    a = axes[1, 0]
    a.semilogy(t, np.maximum(d_dist, 1e-3), color=ROSE, lw=2, label='Attitude-only (current controller)')
    a.semilogy(t, np.maximum(h_dist, 1e-3), color=TEAL, lw=2, label='Position hold (new cascaded PID)')
    a.set_ylim(1e-3, max(d_dist.max() * 2, 1))
    a.set_title('Horizontal distance from hold point (log scale)', loc='left', fontweight='bold', color=NAVY)
    a.set_ylabel('m'); a.set_xlabel('Simulation time (s)'); a.legend(fontsize=9, loc='lower right')
    a.text(.02, .95, f"Attitude-only ends {res_d['final_horizontal_error_m']:.0f} m downwind\n"
           f"Position hold: RMS {res['rms_horizontal_error_m']*100:.1f} cm, max {res['max_horizontal_error_m']*100:.1f} cm",
           transform=a.transAxes, va='top', fontsize=9.5, color=NAVY,
           bbox=dict(boxstyle='round', fc='white', ec='#c9d4e2'))

    a = axes[1, 1]
    a.plot(t, hold['tilt_deg'], color=TEAL, lw=1.5, label='Tilt (position hold)')
    a.set_ylabel('Tilt (deg)', color=TEAL); a.set_xlabel('Simulation time (s)')
    b = a.twinx()
    b.plot(t, hold['power_w'], color=ORANGE, lw=1, alpha=.8, label='Electrical power')
    b.axhline(res['still_air_hover_power_w'], color=ORANGE, ls=':', lw=1)
    b.set_ylabel('Estimated power (W)', color=ORANGE)
    a.set_title('Tilt into the wind and power draw', loc='left', fontweight='bold', color=NAVY)
    a.text(.02, .95, f"Still-air hover {res['still_air_hover_power_w']:.0f} W → {res['still_air_flight_min']:.1f} min\n"
           f"Mean in wind {res['mean_power_in_wind_w']:.0f} W → {res['flight_min_at_mean_wind_power']:.1f} min",
           transform=a.transAxes, va='top', fontsize=9.5, color=NAVY,
           bbox=dict(boxstyle='round', fc='white', ec='#c9d4e2'))
    for a in [axes[0, 0], axes[1, 0], axes[1, 1]]:
        a.axvspan(*gust_span, color=ORANGE, alpha=.15); a.set_xlim(0, t[-1]); a.grid(alpha=.25)
    axes[0, 1].grid(alpha=.25)
    fig.savefig(output / 'gust_summary.png', dpi=150, facecolor=BG)
    plt.close(fig)

    # ----------------------------------------------------------------- animation
    fig = plt.figure(figsize=(14, 7.6), dpi=90)
    fig.patch.set_facecolor(BG)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.25, 1], left=.02, right=.965, top=.86, bottom=.1, hspace=.55, wspace=.12)
    ax3 = fig.add_subplot(gs[:, 0], projection='3d')
    axw, axd, axt = [fig.add_subplot(gs[i, 1]) for i in range(3)]
    fig.suptitle('WIND-GUST REJECTION  /  POSITION HOLD IN DRYDEN TURBULENCE', x=.03, ha='left', y=.975,
                 fontsize=17, fontweight='bold', color=NAVY)
    fig.text(.03, .925, f"{settings['mean_speed']:g} m/s mean wind from the west + Dryden turbulence + "
             f"{settings['gust_amplitude']:g} m/s crosswind gust  •  9 in, 900 KV, 6S, provisional "
             f"{report['aircraft']['mass']:g} kg  •  playback 2× speed", fontsize=10)
    fig.text(.03, .025, 'SIMULATION ONLY • unmeasured drag/mass/inertia • perfect state feedback • '
             'aircraft and tilt drawn to true scale; 3D view zoomed to ±60 cm', fontsize=9, color=GREY)

    span = 0.6
    norm = Normalize(0, max(speed.max(), 1))
    lines = {}
    for k, (label, color) in enumerate([('East', TEAL), ('North', ORANGE), ('Up', PURPLE)]):
        axw.plot(t, w_enu[:, k], color=color, lw=.6, alpha=.18)
        lines[f'w{k}'], = axw.plot([], [], color=color, lw=1.4, label=label)
    axw.set_ylim(min(-3, w_enu.min() - .5), w_enu.max() + 1)
    axw.set_title('Wind at the aircraft (m/s)', loc='left', fontsize=10.5, fontweight='bold', color=NAVY)
    axw.legend(fontsize=8, ncol=3, loc='upper left', frameon=False)
    axd.semilogy(t, np.maximum(d_dist, 1e-3), color=ROSE, lw=.6, alpha=.18)
    axd.semilogy(t, np.maximum(h_dist, 1e-3), color=TEAL, lw=.6, alpha=.18)
    lines['d'], = axd.semilogy([], [], color=ROSE, lw=1.8, label='Attitude-only (current)')
    lines['h'], = axd.semilogy([], [], color=TEAL, lw=1.8, label='Position hold (new)')
    axd.set_ylim(1e-3, max(d_dist.max() * 3, 1))
    axd.set_title('Distance from hold point (m, log)', loc='left', fontsize=10.5, fontweight='bold', color=NAVY)
    axd.legend(fontsize=8, loc='lower right', frameon=False)
    axt.plot(t, hold['tilt_deg'], color=TEAL, lw=.6, alpha=.18)
    lines['t'], = axt.plot([], [], color=TEAL, lw=1.5)
    axt.set_ylim(0, hold['tilt_deg'].max() * 1.25)
    axt.set_title('Tilt into the wind (deg)', loc='left', fontsize=10.5, fontweight='bold', color=NAVY)
    axt.set_xlabel('Simulation time (s)')
    cursors = []
    for a in [axw, axd, axt]:
        a.set_xlim(0, t[-1]); a.grid(alpha=.25); a.axvspan(*gust_span, color=ORANGE, alpha=.15)
        cursors.append(a.axvline(0, color=GREY, lw=1))
    status = fig.text(.03, .885, '', fontsize=11.5, fontweight='bold', color=NAVY)

    frame_times = np.arange(0, t[-1] + 1e-9, 0.2)
    frames = []
    for ft in frame_times:
        i = min(np.searchsorted(t, ft), len(t) - 1)
        for k in range(3):
            lines[f'w{k}'].set_data(t[:i + 1], w_enu[:i + 1, k])
        lines['d'].set_data(t[:i + 1], np.maximum(d_dist[:i + 1], 1e-3))
        lines['h'].set_data(t[:i + 1], np.maximum(h_dist[:i + 1], 1e-3))
        lines['t'].set_data(t[:i + 1], hold['tilt_deg'][:i + 1])
        for c in cursors:
            c.set_xdata([ft, ft])

        ax3.cla()
        ax3.set_facecolor(BG)
        ax3.set_xlim(-span, span); ax3.set_ylim(-span, span); ax3.set_zlim(-span * .7, span * .7)
        ax3.set_box_aspect((1, 1, .7))
        ax3.view_init(elev=22, azim=-62)
        ax3.set_xlabel('East (m)', fontsize=8, labelpad=-2); ax3.set_ylabel('North (m)', fontsize=8, labelpad=-2)
        ax3.set_zlabel('Up (m)', fontsize=8, labelpad=-4)
        ax3.tick_params(labelsize=7, pad=-1)
        for pane in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
            pane.set_pane_color((1, 1, 1, .6))
        floor = -span * .7
        # hold point
        ax3.plot([0, 0], [0, 0], [floor, 0], color=ROSE, lw=1, ls=':')
        ax3.scatter(0, 0, 0, marker='+', color=ROSE, s=120, depthshade=False)
        ax3.text(0.03, 0.03, 0.03, 'hold point', color=ROSE, fontsize=8)
        # trail (last 10 s) + floor shadow
        j0 = max(0, np.searchsorted(t, ft - 10))
        trail = enu(hold['error'][j0:i + 1:10])
        if len(trail) > 1:
            ax3.plot(trail[:, 0], trail[:, 1], trail[:, 2], color=PURPLE, lw=1.4, alpha=.8)
            ax3.plot(trail[:, 0], trail[:, 1], 0 * trail[:, 2] + floor, color=GREY, lw=1, alpha=.4)
        # wind arrows on the upwind (west) side, colored by speed
        wv = w_enu[i]
        color = cm.plasma(norm(speed[i]))
        scale = 0.035  # m of arrow per m/s
        for yy in (-.4, 0, .4):
            for zz in (-.25, .15):
                ax3.quiver(-span + .02, yy, zz, *(wv * scale), color=color, lw=1.6,
                           arrow_length_ratio=.25, normalize=False)
        c = draw_drone(ax3, hold['state'][i], target, motors)
        ax3.plot([c[0], c[0]], [c[1], c[1]], [floor, c[2]], color=GREY, lw=.8, ls='--', alpha=.5)
        ax3.text2D(.02, .95, f"wind {speed[i]:4.1f} m/s   offset {h_dist[i]*100:4.1f} cm   tilt {hold['tilt_deg'][i]:3.1f}°",
                   transform=ax3.transAxes, fontsize=10.5, color=NAVY, family='DejaVu Sans Mono')
        ax3.text2D(.02, .905, f"attitude-only drone: {d_dist[i]:5.1f} m downwind (off screen)",
                   transform=ax3.transAxes, fontsize=9.5, color=ROSE, family='DejaVu Sans Mono')
        status.set_text(f"t = {ft:04.1f} s   /   {phase(ft, wind)}")
        fig.canvas.draw()
        img = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
        frames.append(img.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
    frames[-1].convert('RGB').save(output / 'gust_rejection.png')
    frames[0].save(output / 'gust_rejection.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)
    plt.close(fig)
    print(f'Visuals saved to {output.resolve()}')

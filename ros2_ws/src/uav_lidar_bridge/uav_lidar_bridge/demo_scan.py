"""Make a fake scan by "flying" a simulated LiDAR through a box-shaped room.

Runs the same CloudIntegrator / save_scan code as the aircraft, so the output is a
realistic test file for the phone app and CloudCompare. No ROS or hardware needed.

    python -m uav_lidar_bridge.demo_scan --dir ~/scans
"""
import argparse
import math
import os
from datetime import datetime, timedelta

import numpy as np

from .scan_map import CloudIntegrator, PoseBuffer, VoxelMap, save_scan

ROOM = np.array([12.0, 8.0, 4.0])          # east, north, up extent (m), floor at z=0
CRATE_MIN, CRATE_MAX = np.array([7.0, 2.0, 0.0]), np.array([8.5, 3.5, 1.2])


def _hit_box(origin, dirs, lo, hi, inside):
    """Ray distance to an axis-aligned box (slab method)."""
    with np.errstate(divide='ignore', invalid='ignore'):
        t1, t2 = (lo - origin) / dirs, (hi - origin) / dirs
    near, far = np.nanmax(np.minimum(t1, t2), axis=1), np.nanmin(np.maximum(t1, t2), axis=1)
    if inside:
        return far
    return np.where((far >= near) & (near > 0), near, np.inf)


def simulated_cloud(position, yaw, rng, n=4000):
    """Points in the LiDAR (= body FLU) frame, hemisphere-ish field of view like the L2."""
    az = rng.uniform(-math.pi, math.pi, n)
    el = rng.uniform(-math.radians(80), math.radians(80), n)
    body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    world_dirs = body @ R.T
    d = np.minimum(_hit_box(position, world_dirs, np.zeros(3), ROOM, inside=True),
                   _hit_box(position, world_dirs, CRATE_MIN, CRATE_MAX, inside=False))
    d = d + rng.normal(0, 0.01, n)  # ~1 cm range noise
    return body * d[:, None], rng.uniform(20, 200, n)


def make_demo(folder, seconds=30.0, rate_hz=10.0, seed=1):
    rng = np.random.default_rng(seed)
    poses = PoseBuffer(max_gap_s=0.5)
    integrator = CloudIntegrator(VoxelMap(0.05), pose_buffer=poses, range_min=0.35)
    track = lambda t: (np.array([2 + 8 * t / seconds, 4 + 2 * math.sin(t / 4), 1.5]), t / 10)
    dt = 1 / rate_hz
    for k in range(int(seconds * rate_hz) + 2):
        position, yaw = track(k * dt)
        poses.add(k * dt, position, (0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
    for k in range(int(seconds * rate_hz)):
        position, yaw = track(k * dt)
        xyz, intensity = simulated_cloud(position, yaw, rng)
        integrator.add_cloud(k * dt, xyz, intensity)
    start = datetime.now().astimezone()
    return save_scan(folder, integrator.map, dict(
        name=os.path.basename(folder), complete=True, demo=True,
        started=start.isoformat(timespec='seconds'),
        ended=(start + timedelta(seconds=seconds)).isoformat(timespec='seconds'),
        trigger='demo', pose_source='simulated', stats=integrator.stats))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Write a simulated scan for app testing.')
    parser.add_argument('--dir', default='~/scans')
    parser.add_argument('--seconds', type=float, default=30.0)
    args, _ = parser.parse_known_args(argv)
    folder = os.path.join(os.path.expanduser(args.dir),
                          datetime.now().strftime('demo_%Y%m%d_%H%M%S'))
    meta = make_demo(folder, args.seconds)
    print(f"Wrote {meta['points']} points to {folder}/map.ply")


if __name__ == '__main__':
    main()

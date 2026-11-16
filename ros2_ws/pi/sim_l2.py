#!/usr/bin/env python3
"""Simulated Unitree L2 carried through a room, for testing stable_map.py without hardware.

Produces what the Pi receives: IMU messages (quaternion, gyro, accel) and point clouds
built from 72 packets, each point captured at its own moment while the sensor moves.
The IMU quaternion's heading drifts, as a 6-axis IMU with no compass does.

    python3 sim_l2.py            # prints pose and map error for a carried loop
"""
import numpy as np

G = 9.81
ROOM_LO = np.array([-3.0, -2.5, 0.0])
ROOM_HI = np.array([3.0, 2.5, 2.7])
# Furniture: (low corner, high corner). Gives the walls something to disagree with.
BOXES = [
    (np.array([1.8, -2.5, 0.0]), np.array([3.0, -1.2, 1.0])),
    (np.array([-3.0, 1.0, 0.0]), np.array([-2.2, 2.5, 2.0])),
    (np.array([0.6, 1.9, 0.0]), np.array([1.6, 2.5, 0.8])),
]
PACKET_POINTS = 300
PACKET_PERIOD = PACKET_POINTS / 64000.0      # one line of returns
SPIN_HZ = 5.55


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_to_quat_xyzw(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                         (R[1, 0] - R[0, 1]) / (4 * w), w])
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    return np.array([x, np.copysign(y, R[0, 1] + R[1, 0]), np.copysign(z, R[0, 2] + R[2, 0]), 0.0])


def surface_distance(points):
    """Distance from each world point to the nearest real surface (walls, floor, ceiling, boxes)."""
    points = np.asarray(points, dtype=np.float64)
    room = np.minimum(np.abs(points - ROOM_LO), np.abs(points - ROOM_HI)).min(axis=1)
    best = room
    for lo, hi in BOXES:
        outside = np.maximum(np.maximum(lo - points, points - hi), 0.0)
        out_d = np.linalg.norm(outside, axis=1)
        inside = np.minimum(points - lo, hi - points).min(axis=1)
        best = np.minimum(best, np.where(out_d > 0, out_d, np.maximum(inside, 0.0)))
    return best


def cast(origins, directions):
    """Range along each ray to the first surface."""
    with np.errstate(divide="ignore", invalid="ignore"):
        t_room = np.where(directions > 0, (ROOM_HI - origins) / directions,
                          np.where(directions < 0, (ROOM_LO - origins) / directions, np.inf)).min(axis=1)
        best = t_room
        for lo, hi in BOXES:
            t1 = (lo - origins) / directions
            t2 = (hi - origins) / directions
            near = np.nanmax(np.minimum(t1, t2), axis=1)
            far = np.nanmin(np.maximum(t1, t2), axis=1)
            best = np.where((far >= near) & (near > 0), np.minimum(best, near), best)
    return best


class Path:
    """Hold still, carry the sensor once around a loop while turning and bobbing, hold still."""

    def __init__(self, hold=4.0, loop=30.0, turn_deg=20.0, tilt_deg=3.0):
        self.hold, self.loop = hold, loop
        self.turn, self.tilt = np.radians(turn_deg), np.radians(tilt_deg)
        self.duration = 2 * hold + loop

    def _phase(self, t):
        u = np.clip((t - self.hold) / self.loop, 0.0, 1.0)
        return u * u * u * (10 - 15 * u + 6 * u * u)      # smooth start and stop

    def position(self, t):
        s = self._phase(t)
        a = 2 * np.pi * s
        return np.array([-1.3 + 1.3 * (1 - np.cos(a)) * 1.0 - 0.0, -0.9 * np.sin(a) * 1.0 - 0.2,
                         1.2 + 0.15 * np.sin(2 * a)])

    def rotation(self, t):
        s = self._phase(t)
        a = 2 * np.pi * s
        heading = a + self.turn * np.sin(3 * a)           # full turn plus looking side to side
        return rot_z(heading) @ rot_y(self.tilt * np.sin(5 * a)) @ rot_x(self.tilt * np.sin(4 * a + 1))


def imu_messages(path, rate=250.0, yaw_drift_deg_s=0.1, gyro_bias=(0.002, -0.001, 0.0017),
                 seed=0, accel_noise=0.05, tilt_error_deg=0.0, quat_order="xyzw"):
    """List of (time, quat_xyzw, gyro, accel). The quaternion's heading wanders off steadily."""
    rng = np.random.default_rng(seed)
    out = []
    h = 1e-3
    for t in np.arange(0.0, path.duration, 1.0 / rate):
        R = path.rotation(t)
        dR = (path.rotation(t + h) - path.rotation(t - h)) / (2 * h)
        W = R.T @ dR
        gyro = np.array([W[2, 1], W[0, 2], W[1, 0]]) + gyro_bias + rng.normal(0, 0.002, 3)
        acc = (path.position(t + h) - 2 * path.position(t) + path.position(t - h)) / h ** 2
        accel = R.T @ (acc + np.array([0.0, 0.0, G])) + rng.normal(0, accel_noise, 3)
        # tilt_error leans the reported "down" slowly back and forth, as vibration does.
        lean = np.radians(tilt_error_deg) * np.sin(0.7 * t)
        reported = rot_z(np.radians(yaw_drift_deg_s) * t) @ rot_x(lean) @ R
        quat = rot_to_quat_xyzw(reported)
        if quat_order == "wxyz":       # the same rotation with w in the first slot
            quat = quat[[3, 0, 1, 2]]
        out.append((float(t), quat, gyro, accel))
    return out


def cloud_messages(path, packets=72, seed=1, noise=0.01):
    """List of (arrival_time, xyz, intensity, point_time). point_time restarts in every packet, as the SDK does."""
    rng = np.random.default_rng(seed)
    alpha = np.linspace(-np.radians(93), np.radians(93), PACKET_POINTS)   # over the top of the dome
    in_packet = np.arange(PACKET_POINTS) * (PACKET_PERIOD / PACKET_POINTS)
    out = []
    start = 0.0
    while start + packets * PACKET_PERIOD <= path.duration:
        xyz, times = [], []
        for k in range(packets):
            t0 = start + k * PACKET_PERIOD
            t_mid = t0 + PACKET_PERIOD / 2
            theta = 2 * np.pi * SPIN_HZ * (t0 + in_packet)
            body = np.column_stack((np.sin(alpha) * np.cos(theta), np.sin(alpha) * np.sin(theta), np.cos(alpha)))
            R, p = path.rotation(t_mid), path.position(t_mid)
            world = body @ R.T
            ranges = cast(np.broadcast_to(p, world.shape), world) + rng.normal(0, noise, len(body))
            keep = (ranges > 0.05) & (ranges < 30.0)
            xyz.append(body[keep] * ranges[keep, None])
            times.append(in_packet[keep])
        xyz = np.vstack(xyz)
        arrival = start + packets * PACKET_PERIOD
        out.append((float(arrival), xyz, rng.uniform(20, 200, len(xyz)), np.concatenate(times)))
        start = arrival
    return out


def run(path=None, yaw_drift_deg_s=0.1, deskew_sweeps=True, correct_heading=True, sweeps=None,
        every=1, verbose=False, cloud_delay=0.0, accel_noise=0.05, tilt_error_deg=0.0, return_map=False,
        quat_order="xyzw", read_as="auto"):
    """Drive stable_map.Mapper with a simulated carry. Returns pose and map errors against the truth.

    ``every`` works on every Nth cloud only, like a Pi that cannot keep up.
    ``cloud_delay`` is how late each cloud reaches the mapper, in seconds.
    """
    import time
    from stable_map import Mapper

    path = path or Path()
    clouds = cloud_messages(path)[:sweeps]
    imus = imu_messages(path, yaw_drift_deg_s=yaw_drift_deg_s, accel_noise=accel_noise,
                        tilt_error_deg=tilt_error_deg, quat_order=quat_order)
    mapper = Mapper(deskew_sweeps=deskew_sweeps, correct_heading=correct_heading, quat_order=read_as)
    fed = 0
    start_rotation = start_position = None
    turn_errors, place_errors, seconds = [], [], []
    for number, (arrival, xyz, intensity, point_time) in enumerate(clouds):
        if number % every:
            continue
        while fed < len(imus) and imus[fed][0] <= arrival + cloud_delay:
            mapper.on_imu(*imus[fed])
            fed += 1
        began = time.perf_counter()
        status = mapper.on_cloud(arrival + cloud_delay, xyz, intensity, point_time)
        seconds.append(time.perf_counter() - began)
        if start_rotation is None:
            start_rotation, start_position = path.rotation(arrival), path.position(arrival)
        true_rotation = start_rotation.T @ path.rotation(arrival)
        true_position = start_rotation.T @ (path.position(arrival) - start_position)
        cosine = (np.trace(true_rotation.T @ mapper.tracker.rotation) - 1) / 2
        turn_errors.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
        place_errors.append(float(np.linalg.norm(mapper.tracker.translation - true_position)))
        if verbose:
            print(f"{arrival:5.1f}s {status:8s} heading off {turn_errors[-1]:4.1f}°, "
                  f"position off {place_errors[-1] * 100:5.1f} cm, map {len(mapper.tracker.map)}")
    world = mapper.tracker.map.xyz() @ start_rotation.T + start_position
    off = surface_distance(world)
    result = {
        "counts": dict(mapper.counts),
        "heading_error_max_deg": max(turn_errors), "heading_error_end_deg": turn_errors[-1],
        "position_error_max_cm": 100 * max(place_errors), "position_error_end_cm": 100 * place_errors[-1],
        "map_error_mean_cm": 100 * float(off.mean()), "map_error_p95_cm": 100 * float(np.percentile(off, 95)),
        "map_points": int(len(off)), "imu_heading_fix_deg": mapper.heading_fix_deg,
        "true_imu_drift_deg": yaw_drift_deg_s * path.duration,
        "seconds_per_sweep": float(np.median(seconds)),
        "quaternion_read_as": mapper.imu.layout[0], "quaternion_checked": mapper.imu.layout_checked,
    }
    return (result, world, off) if return_map else result


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--drift", type=float, default=0.1, help="IMU heading drift, degrees per second")
    parser.add_argument("--loop", type=float, default=30.0, help="seconds to carry the lidar round the room")
    parser.add_argument("--every", type=int, default=1, help="work on every Nth cloud (a slow computer)")
    parser.add_argument("--old", action="store_true", help="no heading correction and no deskew")
    parser.add_argument("--sensor-quat", choices=("xyzw", "wxyz"), default="xyzw",
                        help="layout the simulated IMU sends its quaternion in")
    parser.add_argument("--read-as", choices=("auto", "xyzw", "wxyz"), default="auto",
                        help="layout the mapper is told to read")
    args = parser.parse_args()
    result = run(Path(loop=args.loop), args.drift, not args.old, not args.old, every=args.every, verbose=True,
                 quat_order=args.sensor_quat, read_as=args.read_as)
    print(json.dumps(result, indent=2))

#!/usr/bin/env python3
"""Build a map that stays put while the lidar moves.

The first cloud is the map. Each later cloud is pulled onto those surfaces
(so a wall stays one wall) and only points that see new space are added.
The last spin's pose is kept. The lidar IMU supplies only the turn since that spin.

Leave the lidar launch running, then:

    python3 ros2_ws/pi/stable_map.py

Walk slowly. Ctrl-C stops. ~/map.pcd is rewritten every few seconds.
"""

import argparse
import time
from pathlib import Path

import numpy as np

SOURCE_CAP = 1500
TARGET_CAP = 3000
MIN_PAIRS = 40
MIN_RANGE = 0.4


def write_pcd(path: Path, pts: np.ndarray) -> None:
    """Write an Nx4 float array (x, y, z, intensity) as a binary PCD."""
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 4:
        raise ValueError(f"expected Nx4 points, got {pts.shape}")
    n = pts.shape[0]
    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as out:
        out.write(header.encode("ascii"))
        out.write(pts.tobytes())
    temporary.replace(path)


def voxel_downsample(xyz, intensity, voxel):
    """Keep the first point in each voxel. xyz is Nx3, intensity is N."""
    xyz = np.asarray(xyz, dtype=np.float64)
    intensity = np.asarray(intensity, dtype=np.float64)
    if len(xyz) == 0:
        return xyz.reshape(0, 3), intensity.reshape(0)
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    index.sort()
    return xyz[index], intensity[index]


def stride_index(count, limit):
    if count <= limit:
        return np.arange(count)
    step = int(np.ceil(count / limit))
    return np.arange(0, count, step)


def rodrigues(rotvec):
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return np.eye(3)
    axis = rotvec / angle
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def apply_pose(rotation, translation, xyz):
    return (rotation @ np.asarray(xyz, dtype=np.float64).T).T + translation


def estimate_normals(xyz, radius=0.3):
    """Outward-facing plane normals. Rows are zero where the neighborhood is not flat."""
    xyz = np.asarray(xyz, dtype=np.float64)
    normals = np.zeros((len(xyz), 3))
    if len(xyz) < 8:
        return normals
    voxel = radius / 2
    keys = np.floor(xyz / voxel).astype(np.int64)
    buckets = {}
    for index, key in enumerate(map(tuple, keys.tolist())):
        buckets.setdefault(key, []).append(index)
    offsets = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    for key, members in buckets.items():
        neighbors = []
        for offset in offsets:
            neighbors.extend(buckets.get((key[0] + offset[0], key[1] + offset[1], key[2] + offset[2]), ()))
        if len(neighbors) < 6:
            continue
        pts = xyz[neighbors]
        _center, cov = _covariance(pts)
        eigvals, eigvecs = np.linalg.eigh(cov)
        if eigvals[1] < 1e-8 or eigvals[0] / eigvals[1] > 0.35:
            continue
        normal = eigvecs[:, 0]
        if normal @ pts.mean(axis=0) > 0:
            normal = -normal
        normals[members] = normal
    return normals


def _covariance(pts):
    center = pts.mean(axis=0)
    shifted = pts - center
    return center, (shifted.T @ shifted) / len(pts)


def correspondences(source, target, max_distance):
    """Nearest target index for each source point inside max_distance. -1 if none."""
    matched = np.full(len(source), -1, dtype=np.int64)
    if len(source) == 0 or len(target) == 0:
        return matched
    limit = max_distance * max_distance
    chunk = 250
    for start in range(0, len(source), chunk):
        block = source[start:start + chunk]
        distance_sq = np.sum((block[:, None, :] - target[None, :, :]) ** 2, axis=2)
        nearest = np.argmin(distance_sq, axis=1)
        best = distance_sq[np.arange(len(block)), nearest]
        keep = best <= limit
        rows = np.arange(start, start + len(block))[keep]
        matched[rows] = nearest[keep]
    return matched


def heading_deg(rotation):
    """Yaw of the lidar's forward axis in the map, in degrees."""
    return float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))


def pose_guess(previous_rotation, imu_delta):
    """Last ICP pose, plus this spin's IMU turn. Never an absolute IMU heading."""
    if imu_delta is None:
        return np.array(previous_rotation, dtype=np.float64, copy=True)
    return previous_rotation @ imu_delta


def align_scan(source, source_normals, target, target_normals, rotation, translation, max_distance, iterations=8):
    """Pull source onto the map planes. Returns pose, acceptance, and mean plane error.

    The incoming rotation is only the starting guess. A strong match may replace
    its yaw, and the returned rotation is the one that must be kept for the next spin.
    Floor points match at every heading, so only upright surfaces are allowed to
    accept a pose. Accepting on the floor draws the walls again as rays.
    """
    rotation = np.array(rotation, dtype=np.float64, copy=True)
    translation = np.array(translation, dtype=np.float64, copy=True)
    source = np.asarray(source, dtype=np.float64)
    source_normals = np.asarray(source_normals, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    target_normals = np.asarray(target_normals, dtype=np.float64)
    if len(source) < MIN_PAIRS or len(target) < MIN_PAIRS:
        return rotation, translation, False, None
    chosen = stride_index(len(target), TARGET_CAP)
    target = target[chosen]
    target_normals = target_normals[chosen]
    chosen = stride_index(len(source), SOURCE_CAP)
    source = source[chosen]
    source_normals = source_normals[chosen]
    current_rotation, current_translation, yaw_ok = _yaw_search(
        source, source_normals, target, target_normals, rotation, translation, max_distance
    )
    if not yaw_ok:
        return rotation, translation, False, None
    tight = max(0.08, min(0.12, max_distance * 0.35))
    for step in range(iterations):
        limit = max_distance if step < iterations // 2 else tight
        moved = apply_pose(current_rotation, current_translation, source)
        paired = correspondences(moved, target, limit)
        use = _with_normals(paired, target_normals)
        if use.sum() < MIN_PAIRS:
            return rotation, translation, False, None
        delta_rotation, delta_translation, step_ok = _plane_step(
            moved[use], target[paired[use]], target_normals[paired[use]]
        )
        if not step_ok:
            return rotation, translation, False, None
        current_rotation = delta_rotation @ current_rotation
        current_translation = delta_rotation @ current_translation + delta_translation
        if np.linalg.norm(delta_translation) < 1e-3 and _rotation_angle(delta_rotation) < 1e-3:
            break
    fit = _wall_fit(
        source, source_normals, current_rotation, current_translation, target, target_normals, tight
    )
    if fit is None:
        return rotation, translation, False, None
    return current_rotation, current_translation, True, fit


def _with_normals(paired, normals):
    use = paired >= 0
    if not np.any(use):
        return use
    use = use.copy()
    use[use] = np.linalg.norm(normals[paired[use]], axis=1) > 0.5
    return use


def _upright_mask(points, normals, min_range):
    """Upright surfaces. The floor is flat in every heading, so it cannot vote."""
    points = np.asarray(points, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    horizontal = np.linalg.norm(normals[:, :2], axis=1)
    ranged = np.linalg.norm(points, axis=1) >= min_range
    return (horizontal > 0.7) & ranged


def _wall_points(points, normals, min_range):
    keep = _upright_mask(points, normals, min_range)
    if keep.sum() < MIN_PAIRS:
        return None
    return points[keep]


def _yaw_search(source, source_normals, target, target_normals, rotation, translation, max_distance):
    """Pick the body yaw, around the prior, that lands the walls on the walls.

    The gate is tight on purpose. A wide gate counts a point that merely reaches
    some wall, so a square room scores just as well a quarter-turn off, and the
    sweep is stored as another rotated copy.
    """
    gate = max(0.08, min(0.12, max_distance * 0.35))
    probe_source = _wall_points(source, source_normals, MIN_RANGE)
    probe_target = _wall_points(target, target_normals, 0.0)
    if probe_source is None or probe_target is None:
        return rotation, translation, False
    probe_source = probe_source[stride_index(len(probe_source), 400)]
    probe_target = probe_target[stride_index(len(probe_target), 800)]
    best_rotation = rotation
    best_score = _inlier_count(probe_source, probe_target, rotation, translation, gate)
    best_offset = 0
    for degrees in (-90, -60, -40, -25, -12, 12, 25, 40, 60, 90):
        candidate = rotation @ _yaw_matrix(np.deg2rad(degrees))
        score = _inlier_count(probe_source, probe_target, candidate, translation, gate)
        closer = score == best_score and abs(degrees) < abs(best_offset)
        if score > best_score or closer:
            best_score = score
            best_offset = degrees
            best_rotation = candidate
    return best_rotation, translation, True


def _wall_fit(source, source_normals, rotation, translation, target, target_normals, gate):
    """Share of upright points that land on an upright surface, and their plane error.

    Returns None when the walls miss. A high score from the floor alone is a miss.
    Source points are still in the lidar frame, so the near-field blob around the
    sensor is dropped before the pose is applied.
    """
    keep = _upright_mask(source, source_normals, MIN_RANGE)
    stored_keep = _upright_mask(target, target_normals, 0.0)
    if keep.sum() < MIN_PAIRS or stored_keep.sum() < MIN_PAIRS:
        return None
    walls = apply_pose(rotation, translation, source[keep])
    stored = target[stored_keep]
    paired = correspondences(walls, stored, gate)
    hits = paired >= 0
    if hits.sum() < MIN_PAIRS or hits.sum() / len(walls) < 0.55:
        return None
    stored_normals = target_normals[stored_keep]
    residual = np.sum(stored_normals[paired[hits]] * (walls[hits] - stored[paired[hits]]), axis=1)
    fit = float(np.median(np.abs(residual)))
    if fit > 0.03:
        return None
    return fit


def _yaw_matrix(radians):
    cosine, sine = np.cos(radians), np.sin(radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def _inlier_count(source, target, rotation, translation, max_distance):
    moved = apply_pose(rotation, translation, source)
    return int((correspondences(moved, target, max_distance) >= 0).sum())


def _plane_step(source, target, normals):
    """Small rotation and translation that seats source on the target planes.

    Rotation is lightly damped so a strong match can correct a large yaw.
    Translation is damped harder so a wall cannot slide along itself.
    """
    delta = target - source
    axes = np.cross(source, normals)
    design = np.concatenate((axes, normals), axis=1)
    slope = np.einsum("ni,ni->n", normals, delta)
    system = design.T @ design
    load = design.T @ slope
    diag = np.diag(system).copy()
    diag[diag < 1e-6] = 1e-6
    system.flat[::7] += np.array([0.01, 0.01, 0.01, 0.25, 0.25, 0.25]) * diag
    try:
        update = np.linalg.solve(system, load)
    except np.linalg.LinAlgError:
        return np.eye(3), np.zeros(3), False
    if not np.all(np.isfinite(update)):
        return np.eye(3), np.zeros(3), False
    turn = np.linalg.norm(update[:3])
    shift = np.linalg.norm(update[3:])
    if turn > 0.35:
        update[:3] *= 0.35 / turn
    if shift > 0.2:
        update[3:] *= 0.2 / shift
    return rodrigues(update[:3]), update[3:], True


def _rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1) * 0.5, -1, 1)
    return float(np.arccos(cosine))


class StableMap:
    """Voxel map. The first point to claim a voxel is kept, and nearby points are not layered on it."""

    def __init__(self, voxel):
        self.voxel = float(voxel)
        self.gate = max(self.voxel * 2.5, 0.05)
        self.occupied = set()
        self._bins = {}
        self._capacity = 0
        self._count = 0
        self._points = np.empty((0, 4), dtype=np.float32)
        self._normals = np.empty((0, 3), dtype=np.float32)

    def __len__(self):
        return self._count

    def cloud(self):
        return self._points[:self._count].copy()

    def xyz(self):
        return np.asarray(self._points[:self._count, :3], dtype=np.float64)

    def normals(self):
        return np.asarray(self._normals[:self._count], dtype=np.float64)

    def novel_mask(self, xyz):
        """True for points that are not already sitting on a stored surface."""
        xyz = np.asarray(xyz, dtype=np.float64)
        mask = np.ones(len(xyz), dtype=bool)
        if self._count == 0 or len(xyz) == 0:
            return mask
        limit = self.gate * self.gate
        keys = np.floor(xyz / self.gate).astype(np.int64)
        offsets = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
        stored = self._points[:self._count, :3]
        for index, key in enumerate(keys.tolist()):
            best = limit + 1
            for offset in offsets:
                for stored_index in self._bins.get(
                    (key[0] + offset[0], key[1] + offset[1], key[2] + offset[2]), ()
                ):
                    gap = xyz[index] - stored[stored_index]
                    dist = float(gap @ gap)
                    if dist < best:
                        best = dist
            mask[index] = best > limit
        return mask

    def insert(self, xyz, intensity, normals=None):
        xyz = np.asarray(xyz, dtype=np.float64)
        intensity = np.asarray(intensity, dtype=np.float64)
        if normals is None:
            normals = np.zeros((len(xyz), 3))
        normals = np.asarray(normals, dtype=np.float64)
        if len(xyz) == 0:
            return 0
        keys = np.floor(xyz / self.voxel).astype(np.int64)
        keep = []
        for index, key in enumerate(map(tuple, keys.tolist())):
            if key in self.occupied:
                continue
            self.occupied.add(key)
            keep.append(index)
        if not keep:
            return 0
        chosen = np.asarray(keep, dtype=np.int64)
        block = np.column_stack((xyz[chosen], intensity[chosen])).astype(np.float32)
        normal_block = normals[chosen].astype(np.float32)
        start = self._count
        self._append(block, normal_block)
        for offset, point in enumerate(block):
            bin_key = tuple(np.floor(point[:3] / self.gate).astype(int))
            self._bins.setdefault(bin_key, []).append(start + offset)
        return len(block)

    def _append(self, block, normals):
        count = self._count
        need = count + len(block)
        if need > self._capacity:
            self._capacity = max(need, self._capacity * 2, 1024)
            grown = np.empty((self._capacity, 4), dtype=np.float32)
            grown_normals = np.empty((self._capacity, 3), dtype=np.float32)
            if count:
                grown[:count] = self._points[:count]
                grown_normals[:count] = self._normals[:count]
            self._points = grown
            self._normals = grown_normals
        self._points[count:need] = block
        self._normals[count:need] = normals
        self._count = need


class ImuTrack:
    """Gyro samples used to guess how far the lidar turned between clouds."""

    def __init__(self):
        self.samples = []

    def push(self, stamp, gyro):
        gyro = np.asarray(gyro, dtype=np.float64)
        if gyro.shape != (3,) or not np.all(np.isfinite(gyro)):
            return
        self.samples.append((float(stamp), gyro))
        cutoff = float(stamp) - 2.0
        while len(self.samples) > 2 and self.samples[0][0] < cutoff:
            self.samples.pop(0)

    def rotation_since(self, start, end):
        """Rotation of the sensor body from start to end, or None if the gap has no gyro."""
        if not self.samples or end < start:
            return None
        window = [sample for sample in self.samples if start - 0.05 <= sample[0] <= end + 0.05]
        if len(window) < 2 or window[-1][0] - window[0][0] < 0.5 * (end - start):
            return None
        rotation = np.eye(3)
        previous_time = start
        rate = window[0][1]
        for stamp, gyro in window:
            if stamp < start:
                rate = gyro
                continue
            if stamp > end:
                break
            step = stamp - previous_time
            if step > 0:
                rotation = rotation @ rodrigues(rate * step)
            previous_time = stamp
            rate = gyro
        tail = end - previous_time
        if tail > 0:
            rotation = rotation @ rodrigues(rate * tail)
        return rotation


class Tracker:
    """Align each sweep to the map planes, then add only points that see new space."""

    def __init__(self, voxel=0.02, max_correspondence=0.4):
        self.voxel = float(voxel)
        self.max_correspondence = float(max_correspondence)
        self.map = StableMap(self.voxel)
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self.last_fit = None

    def add_scan(self, xyz, intensity=None, rotation_hint=None):
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        if intensity is None:
            intensity = np.ones(len(xyz), dtype=np.float64)
        else:
            intensity = np.asarray(intensity, dtype=np.float64).reshape(-1)
        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        xyz, intensity = voxel_downsample(xyz[finite], intensity[finite], self.voxel)
        if len(xyz) == 0:
            return "skipped"
        normals = estimate_normals(xyz)
        if len(self.map) == 0:
            self.map.insert(xyz, intensity, normals)
            self.last_fit = 0.0
            return "seed"
        # rotation_hint is the IMU turn since the last good spin, in the body
        # frame. It is applied on top of that spin. Substituting an absolute IMU
        # heading here would throw away the yaw just found and draw the room again.
        if rotation_hint is None:
            guess_rotation = np.array(self.rotation, dtype=np.float64, copy=True)
        else:
            guess_rotation = pose_guess(self.rotation, rotation_hint)
        rotation, translation, ok, fit = align_scan(
            xyz, normals, self.map.xyz(), self.map.normals(),
            guess_rotation, self.translation, self.max_correspondence,
        )
        self.last_fit = fit
        if not ok:
            return "skipped"
        moved = apply_pose(rotation, translation, xyz)
        moved_normals = (rotation @ normals.T).T
        fresh = self.map.novel_mask(moved)
        self.rotation = rotation
        self.translation = translation
        self.map.insert(moved[fresh], intensity[fresh], moved_normals[fresh])
        return "aligned"


def collect(seconds_between_saves, topic, imu_topic, output, voxel, max_correspondence):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("stable_map")
    tracker = Tracker(voxel, max_correspondence)
    imu = ImuTrack()
    pending = {}
    last_save = time.monotonic()
    last_stamp = None
    counts = {"seed": 0, "aligned": 0, "skipped": 0}

    def on_cloud(msg: PointCloud2) -> None:
        pending["msg"] = msg

    def on_imu(msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu.push(stamp, [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ])

    node.create_subscription(PointCloud2, topic, on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, imu_topic, on_imu, qos_profile_sensor_data)
    print(
        f"mapping {topic} into {output} at {voxel * 100:.0f} cm voxels — walk slowly",
        flush=True,
    )

    def save():
        cloud = tracker.map.cloud()
        if len(cloud) == 0:
            return
        write_pcd(output, cloud)
        print(f"wrote {len(cloud)} fixed points to {output}", flush=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            msg = pending.pop("msg", None)
            if msg is None:
                if tracker.map and time.monotonic() - last_save >= seconds_between_saves:
                    save()
                    last_save = time.monotonic()
                continue
            try:
                points = point_cloud2.read_points(
                    msg, field_names=["x", "y", "z", "intensity"], skip_nans=True
                )
                xyz = np.column_stack((points["x"], points["y"], points["z"]))
                intensity = np.asarray(points["intensity"], dtype=np.float64)
            except (AssertionError, ValueError):
                xyz = point_cloud2.read_points_numpy(
                    msg, field_names=["x", "y", "z"], skip_nans=True
                )
                intensity = np.ones(len(xyz))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            delta = None
            if last_stamp is not None:
                delta = imu.rotation_since(last_stamp, stamp)
            status = tracker.add_scan(xyz, intensity, rotation_hint=delta)
            if status != "skipped":
                last_stamp = stamp
            counts[status] += 1
            place = tracker.translation
            fit = "  --" if tracker.last_fit is None else f"{tracker.last_fit * 100:4.1f}cm"
            dimu = "  --" if delta is None else f"{heading_deg(delta):5.1f}°"
            print(
                f"{status}: map {len(tracker.map)} points, "
                f"lidar at {place[0]:.2f} {place[1]:.2f} {place[2]:.2f} m, "
                f"yaw={heading_deg(tracker.rotation):5.1f}° dIMU={dimu}, "
                f"wall fit {fit}, imu {'on' if len(imu.samples) >= 2 else 'off'}, "
                f"aligned {counts['aligned']}, skipped {counts['skipped']}",
                flush=True,
            )
            if tracker.map and time.monotonic() - last_save >= seconds_between_saves:
                save()
                last_save = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        save()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/unilidar/cloud")
    parser.add_argument("--imu-topic", default="/unilidar/imu")
    parser.add_argument("--output", default=str(Path.home() / "map.pcd"))
    parser.add_argument("--voxel", type=float, default=0.02, help="map cell size in meters")
    parser.add_argument(
        "--max-correspondence",
        type=float,
        default=0.4,
        help="how far a new sweep may start from the map and still be pulled onto it",
    )
    parser.add_argument("--save-every", type=float, default=5.0, help="seconds between PCD writes")
    args = parser.parse_args()
    if args.voxel <= 0 or args.max_correspondence <= 0 or args.save_every <= 0:
        raise SystemExit("voxel, max-correspondence, and save-every must be greater than 0")
    collect(
        args.save_every,
        args.topic,
        args.imu_topic,
        Path(args.output).expanduser(),
        args.voxel,
        args.max_correspondence,
    )


if __name__ == "__main__":
    main()

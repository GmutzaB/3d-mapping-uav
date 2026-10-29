#!/usr/bin/env python3
"""Build a map that stays put while the lidar moves.

The first cloud is the map. Each later cloud is pulled onto those surfaces
(so a wall stays one wall) and only points that see new space are added.
The IMU orientation is the direction the lidar is actually facing, measured
from that first cloud. The gyro turn is used only when the orientation is missing.
Lifting and walking are tracked too: the accelerometer supplies the move since
the last cloud, and the floor and walls pull that guess back into place.

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
# One stored point about every 2.5 mm. Coarser than this was collapsing a room to ~20k points.
DEFAULT_VOXEL = 0.0025


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


def quat_xyzw_to_rot(quat):
    """Body-to-parent rotation from a ROS quaternion (x, y, z, w).

    The Unitree driver copies quaternion[0..3] into orientation x, y, z, w.
    A zero quaternion means the orientation has not been reported yet.
    """
    quat = np.asarray(quat, dtype=np.float64).reshape(-1)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        return None
    norm = np.linalg.norm(quat)
    if norm < 1e-6:
        return None
    x, y, z, w = quat / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose_guess(previous_rotation, imu_delta):
    """Last ICP pose, plus this spin's gyro turn. Not an absolute heading."""
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
    current_translation = _shift_search(
        source, source_normals, target, target_normals, rotation, translation, max_distance
    )
    current_rotation, current_translation, yaw_ok = _yaw_search(
        source, source_normals, target, target_normals, rotation, current_translation, max_distance
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


def _floor_points(points, normals, min_range, rotation=None):
    """Horizontal surfaces. A vertical wall looks the same after a lift, so only these observe height."""
    points = np.asarray(points, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    if len(points) == 0:
        return None
    if rotation is not None:
        normals = (np.asarray(rotation, dtype=np.float64) @ normals.T).T
    keep = (np.abs(normals[:, 2]) > 0.7) & (np.linalg.norm(points, axis=1) >= min_range)
    if keep.sum() < MIN_PAIRS:
        return None
    return points[keep]


def _shift_search(source, source_normals, target, target_normals, rotation, translation, max_distance):
    """Move the guess up, down, and sideways until the floor and the walls agree.

    The prior is only where the lidar was. A lift larger than the match gate
    then has no correspondences, the sweep is dropped, and the cloud stays put.
    Ties keep the smaller move so a flat floor cannot slide the room.
    """
    gate = max(0.08, min(0.12, max_distance * 0.35))
    probe_wall = _wall_points(source, source_normals, MIN_RANGE)
    probe_floor = _floor_points(source, source_normals, MIN_RANGE, rotation)
    stored_wall = _wall_points(target, target_normals, 0.0)
    stored_floor = _floor_points(target, target_normals, 0.0)
    if probe_wall is not None:
        probe_wall = probe_wall[stride_index(len(probe_wall), 400)]
    if probe_floor is not None:
        probe_floor = probe_floor[stride_index(len(probe_floor), 400)]
    if stored_wall is not None:
        stored_wall = stored_wall[stride_index(len(stored_wall), 800)]
    if stored_floor is not None:
        stored_floor = stored_floor[stride_index(len(stored_floor), 800)]
    if probe_floor is None or stored_floor is None:
        return np.array(translation, dtype=np.float64, copy=True)

    def score(shift):
        total = 0
        if probe_wall is not None and stored_wall is not None:
            total += _inlier_count(probe_wall, stored_wall, rotation, shift, gate)
        total += _inlier_count(probe_floor, stored_floor, rotation, shift, gate)
        return total

    translation = np.asarray(translation, dtype=np.float64)
    best_shift = np.array(translation, copy=True)
    best_score = score(best_shift)
    best_size = 0.0
    offsets = [(0.0, 0.0, 0.0)]
    for meters in (0.12, 0.25, 0.40):
        offsets.extend((
            (0.0, 0.0, meters), (0.0, 0.0, -meters),
            (meters, 0.0, 0.0), (-meters, 0.0, 0.0),
            (0.0, meters, 0.0), (0.0, -meters, 0.0),
        ))
    for offset in offsets[1:]:
        shift = translation + np.array(offset)
        found = score(shift)
        size = float(np.linalg.norm(offset))
        closer = found == best_score and size < best_size
        if found > best_score or closer:
            best_score = found
            best_size = size
            best_shift = shift
    return best_shift


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


def novelty_gate(voxel):
    """How far a new point must sit from the map before it is stored.

    A fine map uses one cell. The old 5 cm floor was wider than the cell, so
    later sweeps could not fill anything closer than 5 cm and the cloud stayed sparse.
    Coarser maps keep a wider margin so a shaky sweep does not paint a second wall.
    """
    voxel = float(voxel)
    if voxel <= 0.01:
        return voxel
    return max(voxel * 2.5, 0.05)


class StableMap:
    """Voxel map. The first point to claim a voxel is kept, and nearby points are not layered on it."""

    def __init__(self, voxel):
        self.voxel = float(voxel)
        self.gate = novelty_gate(self.voxel)
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
    """Facing from the IMU quaternion, with gyro integration as the fallback."""

    def __init__(self):
        self.samples = []
        self.accel = []
        self.orientation = None
        self.origin = None
        self.gravity = None

    def push(self, stamp, gyro):
        gyro = np.asarray(gyro, dtype=np.float64)
        if gyro.shape != (3,) or not np.all(np.isfinite(gyro)):
            return
        self.samples.append((float(stamp), gyro))
        cutoff = float(stamp) - 2.0
        while len(self.samples) > 2 and self.samples[0][0] < cutoff:
            self.samples.pop(0)

    def push_orientation(self, quat):
        """Keep the latest orientation. Cloud and IMU stamps often do not overlap."""
        rotation = quat_xyzw_to_rot(quat)
        if rotation is None:
            return
        self.orientation = rotation

    def push_accel(self, stamp, accel):
        """Specific force in the body frame, with the orientation at that moment."""
        accel = np.asarray(accel, dtype=np.float64)
        if accel.shape != (3,) or not np.all(np.isfinite(accel)) or self.orientation is None:
            return
        self.accel.append((float(stamp), accel, np.array(self.orientation, copy=True)))
        cutoff = float(stamp) - 2.0
        while len(self.accel) > 2 and self.accel[0][0] < cutoff:
            self.accel.pop(0)

    def lock_origin(self):
        """Remember the facing at the first cloud. Later clouds are measured from it."""
        if self.origin is None and self.orientation is not None:
            self.origin = np.array(self.orientation, copy=True)

    def lock_gravity(self):
        """The still reading, in the map, is gravity. Later moves are measured from it."""
        if self.gravity is not None or self.origin is None or len(self.accel) < 5:
            return
        samples = [self.origin.T @ rotation @ accel for _, accel, rotation in self.accel]
        self.gravity = np.mean(samples, axis=0)

    def _map_accel(self, accel, rotation):
        return self.origin.T @ rotation @ np.asarray(accel, dtype=np.float64) - self.gravity

    def shift_since(self, start, end):
        """Map displacement from the accelerometer, or None when gravity or the gap is missing.

        Velocity starts at rest at ``start``. Steady speed is carried by the last
        accepted move; this integral is only the extra acceleration in the gap.
        """
        if self.gravity is None or self.origin is None or not self.accel or end < start:
            return None
        window = [sample for sample in self.accel if start - 0.05 <= sample[0] <= end + 0.05]
        if len(window) < 2 or window[-1][0] - window[0][0] < 0.5 * (end - start):
            return None
        position = np.zeros(3)
        velocity = np.zeros(3)
        previous_time = start
        accel = self._map_accel(window[0][1], window[0][2])
        for stamp, sample, rotation in window:
            if stamp < start:
                accel = self._map_accel(sample, rotation)
                continue
            if stamp > end:
                break
            step = stamp - previous_time
            if step > 0:
                position = position + velocity * step + 0.5 * accel * step * step
                velocity = velocity + accel * step
            previous_time = stamp
            accel = self._map_accel(sample, rotation)
        tail = end - previous_time
        if tail > 0:
            position = position + velocity * tail + 0.5 * accel * tail * tail
        if not np.all(np.isfinite(position)) or np.linalg.norm(position) > 0.75:
            return None
        return position

    def facing_in_map(self):
        """Map-from-lidar rotation since the first cloud, or None if it is not known."""
        if self.orientation is None or self.origin is None:
            return None
        return self.origin.T @ self.orientation

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

    def __init__(self, voxel=DEFAULT_VOXEL, max_correspondence=0.4):
        self.voxel = float(voxel)
        self.max_correspondence = float(max_correspondence)
        self.map = StableMap(self.voxel)
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self.step = np.zeros(3)
        self.last_fit = None

    def add_scan(self, xyz, intensity=None, rotation_hint=None, facing=None, translation_hint=None):
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
        # facing is where the lidar is pointed in the map, from the IMU
        # quaternion at the first cloud. It replaces the guess. Stacking it on
        # the last pose would apply the same turn twice and rotate the room.
        # rotation_hint is the gyro change since the last good spin, and it is
        # used only when that quaternion is missing.
        if facing is not None:
            guess_rotation = np.array(facing, dtype=np.float64, copy=True)
        elif rotation_hint is None:
            guess_rotation = np.array(self.rotation, dtype=np.float64, copy=True)
        else:
            guess_rotation = pose_guess(self.rotation, rotation_hint)
        # The last accepted move carries a steady walk. translation_hint is only
        # the extra acceleration since that cloud, so the two add.
        guess_translation = np.array(self.translation, dtype=np.float64) + self.step
        if translation_hint is not None:
            hint = np.asarray(translation_hint, dtype=np.float64)
            length = float(np.linalg.norm(hint))
            if length > 0.4:
                hint = hint * (0.4 / length)
            guess_translation = guess_translation + hint
        previous_translation = np.array(self.translation, dtype=np.float64, copy=True)
        rotation, translation, ok, fit = align_scan(
            xyz, normals, self.map.xyz(), self.map.normals(),
            guess_rotation, guess_translation, self.max_correspondence,
        )
        self.last_fit = fit
        if not ok:
            self.step = np.zeros(3)
            return "skipped"
        moved = apply_pose(rotation, translation, xyz)
        moved_normals = (rotation @ normals.T).T
        fresh = self.map.novel_mask(moved)
        self.step = translation - previous_translation
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
        imu.push_orientation([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        imu.push_accel(stamp, [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

    node.create_subscription(PointCloud2, topic, on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, imu_topic, on_imu, qos_profile_sensor_data)
    print(
        f"mapping {topic} into {output} with about {voxel * 1000:.1f} mm between points — walk slowly",
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
            shift = None
            if last_stamp is not None:
                delta = imu.rotation_since(last_stamp, stamp)
                shift = imu.shift_since(last_stamp, stamp)
            # The map frame is the IMU frame of the cloud that actually seeds it.
            # If that quaternion arrives one cloud late, adopt it only while the
            # lidar is still at the start. A later quaternion must not redefine
            # the room.
            if (
                imu.origin is None
                and len(tracker.map) > 0
                and counts["aligned"] == 0
                and np.allclose(tracker.rotation, np.eye(3))
            ):
                imu.lock_origin()
            facing = None if len(tracker.map) == 0 else imu.facing_in_map()
            status = tracker.add_scan(
                xyz, intensity, rotation_hint=delta, facing=facing, translation_hint=shift,
            )
            if status == "seed":
                imu.lock_origin()
            if imu.gravity is None and np.linalg.norm(tracker.translation) < 0.02:
                imu.lock_gravity()
            if status != "skipped":
                last_stamp = stamp
            counts[status] += 1
            place = tracker.translation
            fit = "  --" if tracker.last_fit is None else f"{tracker.last_fit * 100:4.1f}cm"
            dimu = "  --" if delta is None else f"{heading_deg(delta):5.1f}°"
            reported = imu.facing_in_map()
            face = "  --" if reported is None else f"{heading_deg(reported):5.1f}°"
            imu_on = imu.orientation is not None or len(imu.samples) >= 2
            print(
                f"{status}: map {len(tracker.map)} points, "
                f"lidar at {place[0]:.2f} {place[1]:.2f} {place[2]:.2f} m, "
                f"yaw={heading_deg(tracker.rotation):5.1f}° face={face} dIMU={dimu}, "
                f"wall fit {fit}, imu {'on' if imu_on else 'off'}, "
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
    parser.add_argument(
        "--voxel",
        type=float,
        default=DEFAULT_VOXEL,
        help="spacing between stored points, in meters (default 2.5 mm)",
    )
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

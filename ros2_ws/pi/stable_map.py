#!/usr/bin/env python3
"""Build a map that stays put while the lidar moves.

The first cloud is the map. Each later cloud is pulled onto those surfaces
(so a wall stays one wall) and only points that see new space are added.
A repeat of the same surface is not stored again: the lidar's range wanders
by about a centimetre, which is wider than the 2.5 mm spacing of new points.
The IMU orientation is the direction the lidar is actually facing, measured
from that first cloud. The gyro turn is used only when the orientation is missing.
The move since the last kept sweep comes from the IMU's own clock (the cloud
stamp is a different clock) plus the speed of the last sweep that lined up.
If that guess misses, the sweep is tried again within about 2 m. Holding the
lidar still at the new spot can lock on, instead of skipping forever.
The floor and walls pull that guess back into place.

Leave the lidar launch running, then:

    python3 ros2_ws/pi/stable_map.py

Hold still until the count levels off, then move. The same path is what runs
on the drone. Ctrl-C stops. ~/map.pcd is rewritten every few seconds.
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
# The L2's range on a wall wanders by about a centimetre. A gate tighter than
# that stores a second sheet of the same surface on every sweep.
SURFACE_MATCH = 0.02
# How far one sweep's guess may move. A drone covers this between scans; the
# walls still have to agree before the pose is kept.
STEP_LIMIT = 1.0
# After the guess misses, look this far around it. Holding the lidar still at
# the new spot has to be found again, or every later sweep stays skipped.
REACQUIRE_RADIUS = 2.0
IMU_HISTORY = 4.0
MOTION_GAP = 3.0


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
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    if len(sorted_keys) == 1:
        bounds = np.array([0, 1])
    else:
        changes = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
        bounds = np.concatenate((
            [0], np.flatnonzero(changes) + 1, [len(sorted_keys)]
        ))
    buckets = {
        (int(sorted_keys[start, 0]), int(sorted_keys[start, 1]), int(sorted_keys[start, 2])): order[start:end]
        for start, end in zip(bounds[:-1], bounds[1:])
    }
    offsets = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    for key, members in buckets.items():
        neighbor_ids = []
        for offset in offsets:
            found = buckets.get((key[0] + offset[0], key[1] + offset[1], key[2] + offset[2]))
            if found is not None:
                neighbor_ids.append(found)
        if not neighbor_ids:
            continue
        neighbors = np.concatenate(neighbor_ids)
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


# Cells around a stored point that still count as the same surface.
# Radius 2 is about 4–6 cm, which covers the lidar's range scatter.
# The point's own cell is checked first and is not in this list.
_NEIGHBOR_CELLS = tuple(
    (a, b, c)
    for radius in (1, 2)
    for a in range(-radius, radius + 1)
    for b in range(-radius, radius + 1)
    for c in range(-radius, radius + 1)
    if max(abs(a), abs(b), abs(c)) == radius
)


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
    If the guess misses, other positions within a couple of metres are tried.
    A sweep that then sits still at the new spot can lock on instead of skipping
    forever from the place the lidar used to be.
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
    found = _refine(
        source, source_normals, target, target_normals,
        rotation, translation, max_distance, iterations,
    )
    if found[2]:
        return found
    for shift in _reacquire_translations(
        source, source_normals, target, target_normals, rotation, translation
    ):
        found = _refine(
            source, source_normals, target, target_normals,
            rotation, shift, max_distance, iterations,
        )
        if found[2]:
            return found
    return rotation, translation, False, None


def _refine(source, source_normals, target, target_normals, rotation, translation, max_distance, iterations):
    """Yaw search, plane steps, then the wall and floor test. No position jump."""
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
    if fit is None or _floor_fit(
        source, source_normals, current_rotation, current_translation, target, target_normals, tight
    ) is None:
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


def _reacquire_translations(source, source_normals, target, target_normals, rotation, translation):
    """Positions around the guess worth a full alignment.

    The old search only tried a handful of points on the axes, and it gave up
    unless a floor was visible. A move that landed between those points never
    matched, and holding the lidar still there skipped forever. Walls pick the
    horizontal cell. The floor, when this sweep has one, picks the height.
    """
    probe_wall = _wall_points(source, source_normals, MIN_RANGE)
    stored_wall = _wall_points(target, target_normals, 0.0)
    if probe_wall is None or stored_wall is None:
        return []
    probe_wall = probe_wall[stride_index(len(probe_wall), 120)]
    stored_wall = stored_wall[stride_index(len(stored_wall), 400)]
    probe_floor = _floor_points(source, source_normals, MIN_RANGE, rotation)
    stored_floor = _floor_points(target, target_normals, 0.0)
    if probe_floor is not None:
        probe_floor = probe_floor[stride_index(len(probe_floor), 120)]
    if stored_floor is not None:
        stored_floor = stored_floor[stride_index(len(stored_floor), 400)]
    center = np.asarray(translation, dtype=np.float64)

    def wall_score(shift, gate):
        return _inlier_count(probe_wall, stored_wall, rotation, shift, gate)

    def floor_score(shift, gate):
        if probe_floor is None or stored_floor is None:
            return 0
        return _inlier_count(probe_floor, stored_floor, rotation, shift, gate)

    def closer(shift):
        return float(np.linalg.norm(shift - center))

    coarse = np.arange(-REACQUIRE_RADIUS, REACQUIRE_RADIUS + 1e-6, 0.5)
    ranked = []
    for x in coarse:
        for y in coarse:
            shift = center + np.array([x, y, 0.0])
            ranked.append((wall_score(shift, 0.45), closer(shift), shift))
    ranked.sort(key=lambda item: (-item[0], item[1]))
    seen = []
    picks = []
    for _, _, rough in ranked[:4]:
        local = []
        for x in np.arange(-0.5, 0.51, 0.25):
            for y in np.arange(-0.5, 0.51, 0.25):
                shift = rough + np.array([x, y, 0.0])
                # A wide gate calls every nearby cell a tie. The tighter gate
                # leaves the cell that actually sits on the walls.
                local.append((wall_score(shift, 0.22), closer(shift), shift))
        local.sort(key=lambda item: (-item[0], item[1]))
        shift = local[0][2]
        heights = (0.0,) if probe_floor is None or stored_floor is None else (
            -0.6, -0.3, 0.0, 0.3, 0.6
        )
        best_shift = np.array([shift[0], shift[1], center[2]])
        best_key = None
        for z in heights:
            trial = np.array([shift[0], shift[1], center[2] + z])
            score = wall_score(trial, 0.22) + floor_score(trial, 0.22)
            key = (-score, abs(z))
            if best_key is None or key < best_key:
                best_key = key
                best_shift = trial
        if np.linalg.norm(best_shift - center) < 0.08:
            continue
        if any(np.linalg.norm(best_shift - earlier) < 0.2 for earlier in seen):
            continue
        seen.append(best_shift)
        picks.append(best_shift)
        if len(picks) == 3:
            break
    return picks


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
    # A point or two of noise must not walk the heading off the IMU facing.
    # A real turn still wins: the tight gate makes the wrong heading miss the walls.
    margin = max(15, int(0.05 * max(best_score, 1)))
    for degrees in (-90, -60, -40, -25, -12, 12, 25, 40, 60, 90):
        candidate = rotation @ _yaw_matrix(np.deg2rad(degrees))
        score = _inlier_count(probe_source, probe_target, candidate, translation, gate)
        closer = score == best_score and abs(degrees) < abs(best_offset)
        if score >= best_score + margin or closer:
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
    stored_normals = target_normals[stored_keep]
    # The pose test is a fraction and a median, so a stride stands in for the
    # whole cloud. Matching every point of a growing map is what made each
    # sweep take so long that the next one had already moved out of range.
    chosen = stride_index(len(walls), SOURCE_CAP)
    walls = walls[chosen]
    chosen = stride_index(len(stored), TARGET_CAP)
    stored = stored[chosen]
    stored_normals = stored_normals[chosen]
    paired = correspondences(walls, stored, gate)
    hits = paired >= 0
    if hits.sum() < MIN_PAIRS or hits.sum() / len(walls) < 0.55:
        return None
    residual = np.sum(stored_normals[paired[hits]] * (walls[hits] - stored[paired[hits]]), axis=1)
    fit = float(np.median(np.abs(residual)))
    if fit > 0.03:
        return None
    return fit


def _floor_fit(source, source_normals, rotation, translation, target, target_normals, gate):
    """Height check. Walls stay a match after a lift, so a pose that misses the floor is rejected.

    Returns 0 when this sweep has no floor to check. Returns None when the floor misses.
    """
    source_normals = np.asarray(source_normals, dtype=np.float64)
    target_normals = np.asarray(target_normals, dtype=np.float64)
    posed = (np.asarray(rotation, dtype=np.float64) @ source_normals.T).T
    keep = (np.abs(posed[:, 2]) > 0.7) & (np.linalg.norm(source, axis=1) >= MIN_RANGE)
    stored_keep = np.abs(target_normals[:, 2]) > 0.7
    if keep.sum() < MIN_PAIRS or stored_keep.sum() < MIN_PAIRS:
        return 0.0
    floors = apply_pose(rotation, translation, source[keep])
    stored = target[stored_keep]
    stored_normals = target_normals[stored_keep]
    chosen = stride_index(len(floors), SOURCE_CAP)
    floors = floors[chosen]
    chosen = stride_index(len(stored), TARGET_CAP)
    stored = stored[chosen]
    stored_normals = stored_normals[chosen]
    paired = correspondences(floors, stored, gate)
    hits = paired >= 0
    if hits.sum() < MIN_PAIRS or hits.sum() / len(floors) < 0.45:
        return None
    residual = np.sum(
        stored_normals[paired[hits]] * (floors[hits] - stored[paired[hits]]),
        axis=1,
    )
    if float(np.median(np.abs(residual))) > 0.03:
        return None
    return float(np.median(np.abs(residual)))


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

    The voxel is still the spacing of points on the first sweep and on any
    surface that is actually new. On a fine map the gate is one 2 cm cell, and
    a later point within two cells (about 4–6 cm) is the same surface. That is
    wider than 2.5 mm because the range scatter is about a centimetre; it is
    not the old 5 cm floor on the first sweep. A surface about 8 cm farther is
    still new. Coarser maps keep a wider margin so a shaky sweep does not
    paint a second wall.
    """
    voxel = float(voxel)
    if voxel <= 0.01:
        return max(voxel, SURFACE_MATCH)
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
        """True for points that are not already sitting on a stored surface.

        A point in the same 2 cm cell as a stored point, or within two cells of
        it, is the same surface. That covers the lidar's range scatter, which
        is about a centimetre and was being saved as a second sheet. Checking
        cells, instead of the distance to every stored point, keeps a full
        sweep fast enough to follow a moving lidar.
        """
        xyz = np.asarray(xyz, dtype=np.float64)
        count = len(xyz)
        if self._count == 0 or count == 0:
            return np.ones(count, dtype=bool)
        keys = np.floor(xyz / self.gate).astype(np.int64)
        occupied = self._bins
        mask = np.ones(count, dtype=bool)
        for index in range(count):
            x = int(keys[index, 0])
            y = int(keys[index, 1])
            z = int(keys[index, 2])
            if (x, y, z) in occupied:
                mask[index] = False
                continue
            for offset in _NEIGHBOR_CELLS:
                if (x + offset[0], y + offset[1], z + offset[2]) in occupied:
                    mask[index] = False
                    break
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
        # IMU header stamps already consumed. The cloud header is a different
        # clock, so motion is measured from these cursors, not from the cloud.
        self._gyro_cursor = None
        self._accel_cursor = None

    def push(self, stamp, gyro):
        gyro = np.asarray(gyro, dtype=np.float64)
        if gyro.shape != (3,) or not np.all(np.isfinite(gyro)):
            return
        self.samples.append((float(stamp), gyro))
        cutoff = float(stamp) - IMU_HISTORY
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
        cutoff = float(stamp) - IMU_HISTORY
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

    def shift_since(self, start, end, max_gap=0.5, max_norm=0.75):
        """Map displacement from the accelerometer, or None when gravity or the gap is missing.

        Velocity starts at rest at ``start``. Steady speed is carried by the last
        accepted move; this integral is only the extra acceleration in the gap.
        ``max_gap`` keeps a stale clock from integrating for several seconds and
        shoving the pose off the map.
        """
        if self.gravity is None or self.origin is None or not self.accel or end < start:
            return None
        if end - start > max_gap:
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
        if not np.all(np.isfinite(position)) or np.linalg.norm(position) > max_norm:
            return None
        return position

    def advance(self):
        """Mark the IMU samples so far as already used by a kept sweep."""
        if self.samples:
            self._gyro_cursor = self.samples[-1][0]
        if self.accel:
            self._accel_cursor = self.accel[-1][0]

    def _preview_span(self, series, cursor):
        if len(series) < 2:
            return None
        end = series[-1][0]
        start = end if cursor is None else cursor
        if end <= start:
            return None
        if end - start > MOTION_GAP:
            start = end - MOTION_GAP
        return start, end

    def preview_shift(self):
        """Accel move since the last kept sweep, on the IMU clock."""
        span = self._preview_span(self.accel, self._accel_cursor)
        if span is None:
            return None
        return self.shift_since(span[0], span[1], max_gap=MOTION_GAP, max_norm=STEP_LIMIT)

    def preview_rotation(self):
        """Gyro turn since the last kept sweep, on the IMU clock."""
        span = self._preview_span(self.samples, self._gyro_cursor)
        if span is None:
            return None
        return self.rotation_since(span[0], span[1])

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
        self.speed = np.zeros(3)
        self.last_fit = None

    def note_elapsed(self, seconds):
        """Turn the last accepted move into a speed. The next guess carries it."""
        seconds = float(seconds)
        if seconds <= 0.05:
            return
        self.speed = self.step / seconds

    def add_scan(self, xyz, intensity=None, rotation_hint=None, facing=None, translation_hint=None, carried=None):
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
        if len(self.map) < MIN_PAIRS:
            # A short first packet used to become the whole map. Alignment needs
            # more than that, so every later sweep was skipped and the count froze.
            moved = apply_pose(self.rotation, self.translation, xyz)
            moved_normals = (self.rotation @ normals.T).T
            self.map.insert(moved, intensity, moved_normals)
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
        # the extra acceleration since that cloud, so the two add. If that guess
        # misses, try the last pose by itself before giving up on the sweep.
        previous_translation = np.array(self.translation, dtype=np.float64, copy=True)
        rotation = translation = None
        ok = False
        fit = None
        for use_motion in (True, False):
            guess_translation = np.array(previous_translation, copy=True)
            if use_motion:
                # carried is the speed of the last kept sweep times the time
                # since then. Tests that do not time the sweeps use the last
                # displacement instead.
                motion = self.step if carried is None else np.asarray(carried, dtype=np.float64)
                motion_length = float(np.linalg.norm(motion))
                if motion_length > STEP_LIMIT:
                    motion = motion * (STEP_LIMIT / motion_length)
                guess_translation = guess_translation + motion
                if translation_hint is not None:
                    hint = np.asarray(translation_hint, dtype=np.float64)
                    length = float(np.linalg.norm(hint))
                    if length > STEP_LIMIT:
                        hint = hint * (STEP_LIMIT / length)
                    guess_translation = guess_translation + hint
            rotation, translation, ok, fit = align_scan(
                xyz, normals, self.map.xyz(), self.map.normals(),
                guess_rotation, guess_translation, self.max_correspondence,
            )
            if ok:
                break
        self.last_fit = fit
        if not ok:
            # Keep the last speed. Zeroing it left the next guess at the start
            # after a single miss, and every later sweep then missed too.
            return "skipped"
        moved = apply_pose(rotation, translation, xyz)
        moved_normals = (rotation @ normals.T).T
        fresh = self.map.novel_mask(moved)
        delta = translation - previous_translation
        length = float(np.linalg.norm(delta))
        if length < 0.01:
            self.step = np.zeros(3)
        elif length > STEP_LIMIT:
            self.step = delta * (STEP_LIMIT / length)
        else:
            self.step = delta
        self.rotation = rotation
        self.translation = translation
        self.map.insert(moved[fresh], intensity[fresh], moved_normals[fresh])
        return "aligned"


def collect(seconds_between_saves, topic, imu_topic, output, voxel, max_correspondence):
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("stable_map")
    tracker = Tracker(voxel, max_correspondence)
    imu = ImuTrack()
    pending = {}
    last_save = time.monotonic()
    last_kept = None
    counts = {"seed": 0, "aligned": 0, "skipped": 0}
    # The sensor profile keeps five messages. A sweep used to take long enough
    # that the gyro and accel samples in between were thrown away.
    imu_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=400,
    )

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
    node.create_subscription(Imu, imu_topic, on_imu, imu_qos)
    print(
        f"mapping {topic} into {output} with about {voxel * 1000:.1f} mm between points"
        " — hold still until the count levels off, then move",
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
            # The cloud header and the IMU header do not share a clock, so a
            # window between those stamps is empty and the guess stays put.
            # Integrate the IMU samples on their own stamps instead.
            now = time.monotonic()
            delta = imu.preview_rotation()
            shift = imu.preview_shift()
            carried = None
            if last_kept is not None:
                elapsed = now - last_kept
                if 0.02 < elapsed <= MOTION_GAP:
                    carried = tracker.speed * elapsed
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
                xyz, intensity,
                rotation_hint=delta,
                facing=facing,
                translation_hint=shift,
                carried=carried,
            )
            if status == "seed":
                imu.lock_origin()
            if imu.gravity is None and np.linalg.norm(tracker.translation) < 0.02:
                imu.lock_gravity()
            if status != "skipped":
                if last_kept is not None:
                    tracker.note_elapsed(now - last_kept)
                imu.advance()
                last_kept = time.monotonic()
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

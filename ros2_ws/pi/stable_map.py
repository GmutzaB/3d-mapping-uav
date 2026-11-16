#!/usr/bin/env python3
"""Build a map that stays put while the lidar moves.

The first cloud is the map. Each later cloud is pulled onto those surfaces
(so a wall stays one wall) and only points that see new space are added.
A repeat of the same surface is not stored again: the lidar's range wanders
by about a centimetre, which is wider than the 2.5 mm spacing of new points.
The IMU orientation is the direction the lidar is facing, but its heading
wanders (no compass), so the offset between it and the map is re-measured
every time a sweep seats on the walls. Each sweep is also straightened for
the turning and travel that happened while it was being captured.
The gyro turn is used only when the orientation is missing.
The move since the last kept sweep comes from the IMU's own clock (the cloud
stamp is a different clock) plus the speed of the last sweep that lined up.
Walk, turn, and lift freely. A sweep that sits on the walls is stored.
A sweep that misses while the lidar is moving is not stored, but the pose
keeps following that motion so the next sweep can lock on. A miss with no
motion to follow stays put and is tried again within about 2 m.
The floor and walls pull that guess back into place.

Leave the lidar launch running, then:

    python3 ros2_ws/pi/stable_map.py

Hold still until the count levels off, then move. The same path is what runs
on the drone. Ctrl-C stops. ~/map.pcd is rewritten every few seconds.
"""

import argparse
import threading
import time
from bisect import bisect_left
from collections import deque
from pathlib import Path

import numpy as np

try:  # python3-scipy. Optional: everything still runs without it, only slower.
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None

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
# A point this close to a wall, but outside the tight gate, is the same wall
# with the wrong pose. Farther than this is a different surface.
NEAR_MISS = 0.35
IMU_HISTORY = 4.0
MOTION_GAP = 3.0
# How long the pose may coast on the accelerometer after the walls were last seen.
COAST_LIMIT = 1.5
# How far a match may turn the room away from the corrected IMU facing: this
# much, plus this much per second since the walls last agreed.
HEADING_TRUST_DEG = 8.0
HEADING_WANDER_DEG_S = 2.0
# A change in speed larger than this between two sweeps is a bad accelerometer reading.
SPEED_STEP_LIMIT = 4.0


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


def _spread_index(coords, limit):
    """Indexes spread across parallel planes, not bunched on the nearest one.

    A uniform stride of a sweep that is mostly a new wall keeps that wall and
    drops the older ones. The search then slides the new wall onto whatever
    it touches. Each plane along ``coords`` keeps a share of the sample.
    """
    coords = np.asarray(coords, dtype=np.float64)
    count = len(coords)
    if count <= limit:
        return np.arange(count)
    buckets = np.round(coords / 0.5).astype(np.int64)
    unique = np.unique(buckets)
    per = max(1, int(np.ceil(limit / len(unique))))
    chosen = []
    for bucket in unique:
        idx = np.flatnonzero(buckets == bucket)
        chosen.append(idx[stride_index(len(idx), per)])
    chosen = np.concatenate(chosen)
    if len(chosen) > limit:
        chosen = chosen[stride_index(len(chosen), limit)]
    return chosen


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
    """Outward-facing plane normals. Rows are zero where the neighborhood is not flat.

    Each point takes the plane through its own cell and the 26 around it.
    The sums are kept per cell and added up for all cells at once: one cell at a
    time in Python was the slowest part of every sweep.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    normals = np.zeros((len(xyz), 3))
    if len(xyz) < 8:
        return normals
    voxel = radius / 2
    keys = np.floor(xyz / voxel).astype(np.int64)
    low = keys.min(axis=0) - 1
    span = keys.max(axis=0) - low + 2
    if float(span[0]) * float(span[1]) * float(span[2]) >= 2 ** 62:
        return normals
    packed = ((keys[:, 0] - low[0]) * span[1] + (keys[:, 1] - low[1])) * span[2] + (keys[:, 2] - low[2])
    cells, member = np.unique(packed, return_inverse=True)
    count = np.bincount(member, minlength=len(cells)).astype(np.float64)
    total = np.zeros((len(cells), 3))
    np.add.at(total, member, xyz)
    outer = np.zeros((len(cells), 9))
    np.add.at(outer, member, (xyz[:, :, None] * xyz[:, None, :]).reshape(-1, 9))
    near_count = np.zeros(len(cells))
    near_total = np.zeros((len(cells), 3))
    near_outer = np.zeros((len(cells), 9))
    for a in (-1, 0, 1):
        for b in (-1, 0, 1):
            for c in (-1, 0, 1):
                wanted = cells + (a * span[1] + b) * span[2] + c
                at = np.searchsorted(cells, wanted)
                at[at >= len(cells)] = 0
                found = cells[at] == wanted
                near_count[found] += count[at[found]]
                near_total[found] += total[at[found]]
                near_outer[found] += outer[at[found]]
    enough = near_count >= 6
    if not np.any(enough):
        return normals
    mean = near_total[enough] / near_count[enough, None]
    cov = (near_outer[enough] / near_count[enough, None]).reshape(-1, 3, 3) - mean[:, :, None] * mean[:, None, :]
    eigvals, eigvecs = np.linalg.eigh(cov)
    flat = (eigvals[:, 1] >= 1e-8) & (eigvals[:, 0] <= 0.35 * np.maximum(eigvals[:, 1], 1e-300))
    normal = eigvecs[:, :, 0]
    normal = np.where((np.einsum("ij,ij->i", normal, mean) > 0)[:, None], -normal, normal)
    normal[~flat] = 0.0
    cell_normal = np.zeros((len(cells), 3))
    cell_normal[enough] = normal
    return cell_normal[member]


# Cells around a stored point that still count as the same surface.
# The pose wanders by a few centimetres while the lidar sits still, and the
# range wanders by about a centimetre. Radius 4 on a 2 cm cell covers that,
# so a still sweep does not paint another sheet. The point's own cell is
# checked first and is not in this list.
_NEIGHBOR_CELLS = tuple(
    (a, b, c)
    for radius in range(1, 5)
    for a in range(-radius, radius + 1)
    for b in range(-radius, radius + 1)
    for c in range(-radius, radius + 1)
    if max(abs(a), abs(b), abs(c)) == radius
)
# A gap longer than this between IMU samples is a dropped stretch. Holding the
# last rate across it turns a pause into a phantom spin (tens of degrees).
_MAX_IMU_STEP = 0.15


def _covariance(pts):
    center = pts.mean(axis=0)
    shifted = pts - center
    return center, (shifted.T @ shifted) / len(pts)


def correspondences(source, target, max_distance):
    """Nearest target index for each source point inside max_distance. -1 if none."""
    matched = np.full(len(source), -1, dtype=np.int64)
    if len(source) == 0 or len(target) == 0:
        return matched
    if cKDTree is not None:
        # Same answer as the comparison below, several times faster. A sweep has
        # to finish before the lidar has moved out of reach of the next one.
        distance, nearest = cKDTree(target).query(source, distance_upper_bound=max_distance * (1 + 1e-12))
        keep = np.isfinite(distance)
        matched[keep] = nearest[keep]
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


# How the four numbers of the IMU quaternion may be laid out: which slot holds w,
# and whether it turns the lidar into the world or the world into the lidar.
QUAT_LAYOUTS = (("xyzw", False), ("wxyz", False), ("xyzw", True), ("wxyz", True))


def quat_to_rot(quat, order="xyzw", inverse=False):
    """Lidar-to-world rotation from four quaternion numbers in the given layout. None if unusable."""
    quat = np.asarray(quat, dtype=np.float64).reshape(-1)
    if quat.shape != (4,):
        return None
    if order == "wxyz":
        quat = quat[[1, 2, 3, 0]]
    rotation = quat_xyzw_to_rot(quat)
    if rotation is not None and inverse:
        rotation = rotation.T
    return rotation


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
    # The source is not thinned here. A uniform stride drops the old walls
    # once most of the sweep is new space, and the pose test then sees no
    # overlap. The plane steps thin their own copy.
    found = _refine(
        source, source_normals, target, target_normals,
        rotation, translation, max_distance, iterations,
    )
    if found[2]:
        return found
    for shift in _reacquire_translations(
        source, source_normals, target, target_normals, rotation, translation
    ):
        # Far from the guess, a tight yaw gate cannot see the turn. Search yaw
        # more loosely here; the wall test below is still the tight one.
        yaw_gate = 0.35 if np.linalg.norm(shift - translation) > 0.3 else None
        found = _refine(
            source, source_normals, target, target_normals,
            rotation, shift, max_distance, iterations, yaw_gate=yaw_gate,
        )
        if found[2]:
            return found
    return rotation, translation, False, None


def _pose_fit(source, source_normals, rotation, translation, target, target_normals, gate):
    """Wall error when this pose sits on the walls and the floor. None when it does not."""
    fit = _wall_fit(
        source, source_normals, rotation, translation, target, target_normals, gate
    )
    if fit is None or _floor_fit(
        source, source_normals, rotation, translation, target, target_normals, gate
    ) is None:
        return None
    return fit


def _refine(source, source_normals, target, target_normals, rotation, translation, max_distance, iterations, yaw_gate=None):
    """Yaw search, plane steps, then the wall and floor test. No position jump.

    The pose that enters the plane steps is kept when those steps walk off a
    match. A sweep that already sits on the walls used to be thrown away, and
    every later sweep then skipped from the last place that had worked.
    """
    current_rotation, current_translation, yaw_ok = _yaw_search(
        source, source_normals, target, target_normals, rotation, translation, max_distance,
        yaw_gate=yaw_gate,
    )
    if not yaw_ok:
        return rotation, translation, False, None
    tight = max(0.08, min(0.12, max_distance * 0.35))
    held_fit = _pose_fit(
        source, source_normals, current_rotation, current_translation, target, target_normals, tight
    )
    held = (
        None if held_fit is None
        else (np.array(current_rotation, copy=True), np.array(current_translation, copy=True), held_fit)
    )
    icp_index = stride_index(len(source), SOURCE_CAP)
    icp_source = source[icp_index]
    for step in range(iterations):
        limit = max_distance if step < iterations // 2 else tight
        moved = apply_pose(current_rotation, current_translation, icp_source)
        paired = correspondences(moved, target, limit)
        use = _with_normals(paired, target_normals)
        if use.sum() < MIN_PAIRS:
            break
        # One new wall can be most of the correspondences. Left in, it drags
        # the pose onto the stored wall it happens to face, and the sweep that
        # was a few centimetres off never seats.
        seated, paired_target, paired_normals = _balanced_pairs(
            moved[use], target[paired[use]], target_normals[paired[use]]
        )
        if len(seated) < MIN_PAIRS:
            break
        delta_rotation, delta_translation, step_ok = _plane_step(
            seated, paired_target, paired_normals
        )
        if not step_ok:
            break
        current_rotation = delta_rotation @ current_rotation
        current_translation = delta_rotation @ current_translation + delta_translation
        if np.linalg.norm(delta_translation) < 1e-3 and _rotation_angle(delta_rotation) < 1e-3:
            break
    fit = _pose_fit(
        source, source_normals, current_rotation, current_translation, target, target_normals, tight
    )
    if fit is not None:
        return current_rotation, current_translation, True, fit
    if held is not None:
        return held[0], held[1], True, held[2]
    return rotation, translation, False, None


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


def _direction_pairs(source, source_normals, target, target_normals, rotation):
    """One sample of each wall direction, in the lidar frame and on the map.

    A new wall can be most of the sweep. Counting every upright point then
    follows that new wall onto whatever it happens to touch, and the old
    walls never get a vote. Each direction is sampled on its own, and the
    search keeps the pose where the weaker direction still lands.
    """
    keep = _upright_mask(source, source_normals, MIN_RANGE)
    stored_keep = _upright_mask(target, target_normals, 0.0)
    if keep.sum() < MIN_PAIRS or stored_keep.sum() < MIN_PAIRS:
        return []
    posed = (np.asarray(rotation, dtype=np.float64) @ np.asarray(source_normals, dtype=np.float64).T).T
    source_bins = _normal_bins(posed[keep])
    stored_bins = _normal_bins(target_normals[stored_keep])
    source_points = source[keep]
    stored_points = target[stored_keep]
    # Bin in the map, without the translation: parallel walls stay apart, and
    # the sample does not depend on the pose being searched.
    posed_source = (np.asarray(rotation, dtype=np.float64) @ source_points.T).T
    pairs = []
    for direction in (0, 1):
        src_idx = np.flatnonzero(source_bins == direction)
        dst_idx = np.flatnonzero(stored_bins == direction)
        if len(src_idx) < 15 or len(dst_idx) < 15:
            continue
        src_idx = src_idx[_spread_index(posed_source[src_idx, direction], 80)]
        dst_idx = dst_idx[_spread_index(stored_points[dst_idx, direction], 200)]
        pairs.append((source_points[src_idx], stored_points[dst_idx]))
    return pairs


def _reacquire_translations(source, source_normals, target, target_normals, rotation, translation):
    """Positions around the guess worth a full alignment.

    The old search only tried a handful of points on the axes, and it gave up
    unless a floor was visible. A move that landed between those points never
    matched, and holding the lidar still there skipped forever. Walls pick the
    horizontal cell. The floor, when this sweep has one, picks the height.
    """
    directions = _direction_pairs(source, source_normals, target, target_normals, rotation)
    probe_wall = _wall_points(source, source_normals, MIN_RANGE)
    stored_wall = _wall_points(target, target_normals, 0.0)
    if not directions and (probe_wall is None or stored_wall is None):
        return []
    if probe_wall is not None:
        probe_wall = probe_wall[stride_index(len(probe_wall), 120)]
    if stored_wall is not None:
        stored_wall = stored_wall[stride_index(len(stored_wall), 400)]
    probe_floor = _floor_points(source, source_normals, MIN_RANGE, rotation)
    stored_floor = _floor_points(target, target_normals, 0.0)
    if probe_floor is not None:
        probe_floor = probe_floor[stride_index(len(probe_floor), 120)]
    if stored_floor is not None:
        stored_floor = stored_floor[stride_index(len(stored_floor), 400)]
    center = np.asarray(translation, dtype=np.float64)

    def wall_score(shift, gate):
        if not directions:
            return _inlier_count(probe_wall, stored_wall, rotation, shift, gate)
        # The weaker direction decides. A new wall is one direction and can
        # touch the map by accident; the other direction still has to land.
        return min(
            _inlier_count(probe, stored, rotation, shift, gate)
            for probe, stored in directions
        )

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


def _yaw_search(source, source_normals, target, target_normals, rotation, translation, max_distance, yaw_gate=None):
    """Pick the body yaw, around the prior, that lands the walls on the walls.

    The gate is tight on purpose. A wide gate counts a point that merely reaches
    some wall, so a square room scores just as well a quarter-turn off, and the
    sweep is stored as another rotated copy.
    """
    gate = max(0.08, min(0.12, max_distance * 0.35)) if yaw_gate is None else float(yaw_gate)
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


def _normal_bins(normals):
    """Two wall directions: facing across x, or facing across y.

    Opposite normals are the same direction. A cut at 45 degree steps flips
    when a plane step turns the cloud by a fraction of a degree, and the old
    walls then look like a direction the map does not have.
    """
    return (np.abs(normals[:, 1]) > np.abs(normals[:, 0])).astype(np.int64)


def _wall_fit(source, source_normals, rotation, translation, target, target_normals, gate):
    """Plane error when the walls agree. None when they do not.

    A high score from the floor alone is a miss. Source points are still in the
    lidar frame, so the near-field blob around the sensor is dropped before the
    pose is applied.

    Most of a still sweep lands on the stored walls. After a move, most of the
    sweep can be new space. That new space must not veto a pose: each wall
    direction that still overlaps the map has to sit on that wall, and points
    farther out are the new view.
    """
    keep = _upright_mask(source, source_normals, MIN_RANGE)
    stored_keep = _upright_mask(target, target_normals, 0.0)
    if keep.sum() < MIN_PAIRS or stored_keep.sum() < MIN_PAIRS:
        return None
    walls = apply_pose(rotation, translation, source[keep])
    wall_normals = (np.asarray(rotation, dtype=np.float64) @ source_normals[keep].T).T
    stored = target[stored_keep]
    stored_normals = target_normals[stored_keep]
    # The pose test is a fraction and a median, so a stride stands in for the
    # whole cloud. Matching every point of a growing map is what made each
    # sweep take so long that the next one had already moved out of range.
    chosen = stride_index(len(walls), SOURCE_CAP)
    walls = walls[chosen]
    wall_normals = wall_normals[chosen]
    chosen = stride_index(len(stored), TARGET_CAP)
    stored = stored[chosen]
    stored_normals = stored_normals[chosen]
    paired = correspondences(walls, stored, gate)
    hits = paired >= 0
    if hits.sum() < MIN_PAIRS:
        return None
    residual = np.sum(stored_normals[paired[hits]] * (walls[hits] - stored[paired[hits]]), axis=1)
    fit = float(np.median(np.abs(residual)))
    if fit > 0.03:
        return None
    # A high overall score is not enough. Walls parallel to a slide still
    # touch the map, so each wall direction has to agree on its own.
    return fit if _overlap_agrees(walls, wall_normals, stored, stored_normals, hits) else None


def _overlap_agrees(walls, wall_normals, stored, stored_normals, hits):
    """True when every overlapping wall direction sits on the map.

    Points farther than a near miss are new space and do not vote. A slide
    along the room leaves one wall direction with nothing on the map, and
    that direction rejects the pose.
    """
    wall_bins = _normal_bins(wall_normals)
    stored_bins = _normal_bins(stored_normals)
    near_paired = correspondences(walls, stored, NEAR_MISS)
    near = near_paired >= 0
    same = np.zeros(len(walls), dtype=bool)
    if np.any(near):
        same[near] = stored_bins[near_paired[near]] == wall_bins[near]
    # A direction can be a small share of the sweep once most of the cloud is
    # new space. Twenty points on that wall is enough to see that it agrees,
    # and a near miss next to them still rejects a slid pose.
    overlap_min = 20
    checked = 0
    for direction in range(2):
        source_here = wall_bins == direction
        if source_here.sum() < overlap_min or (stored_bins == direction).sum() < overlap_min:
            continue
        checked += 1
        tight = int((hits & source_here).sum())
        contradictions = int((~hits & source_here & same).sum())
        overlapping = tight + contradictions
        if overlapping < overlap_min or tight / overlapping < 0.55:
            return False
    return checked > 0


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
    if hits.sum() < MIN_PAIRS:
        return None
    fraction = hits.sum() / len(floors)
    residual = np.sum(
        stored_normals[paired[hits]] * (floors[hits] - stored[paired[hits]]),
        axis=1,
    )
    fit = float(np.median(np.abs(residual)))
    if fit > 0.03:
        return None
    if fraction >= 0.45:
        return fit
    # Part of the floor can leave the view when the lidar moves. The part that
    # still hits has to sit tight, or a lifted pose would pass on the walls.
    if fraction >= 0.25 and fit <= 0.02:
        return fit
    return None


def _yaw_matrix(radians):
    cosine, sine = np.cos(radians), np.sin(radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def _inlier_count(source, target, rotation, translation, max_distance):
    moved = apply_pose(rotation, translation, source)
    return int((correspondences(moved, target, max_distance) >= 0).sum())


def _balanced_pairs(moved, matched, normals, cap=40):
    """Keep every surface that belongs to one pose, and cap how hard it pulls.

    Points are already paired. A second parallel wall sits farther off the
    plane than the walls that agree with each other. That farther wall is
    dropped. The floor is its own direction, so a lift is still observed.
    Each remaining surface is capped, so the biggest wall does not outvote
    the others.
    """
    moved = np.asarray(moved, dtype=np.float64)
    matched = np.asarray(matched, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    count = len(moved)
    if count < MIN_PAIRS:
        return moved, matched, normals
    residual = np.abs(np.einsum("ij,ij->i", normals, moved - matched))
    vertical = np.abs(normals[:, 2]) > 0.7
    facing_y = np.abs(normals[:, 1]) > np.abs(normals[:, 0])
    kind = np.where(vertical, 2, facing_y.astype(np.int64))
    axis_pos = np.where(vertical, moved[:, 2], np.where(facing_y, moved[:, 1], moved[:, 0]))
    bucket = np.round(axis_pos / 0.5).astype(np.int64)
    key = kind.astype(np.int64) * 100000 + (bucket + 5000)
    order = np.argsort(key, kind="mergesort")
    sorted_key = key[order]
    starts = np.concatenate(([0], np.flatnonzero(np.diff(sorted_key)) + 1))
    ends = np.concatenate((starts[1:], [count]))
    groups = []
    best = [None, None, None]
    for start, end in zip(starts, ends):
        idx = order[start:end]
        med = float(np.median(residual[idx]))
        this_kind = int(kind[idx[0]])
        groups.append((idx, med, this_kind))
        if len(idx) >= 8:
            current = best[this_kind]
            if current is None or med < current:
                best[this_kind] = med
    chosen = []
    for idx, med, this_kind in groups:
        baseline = best[this_kind]
        if baseline is None or med <= baseline + 0.15:
            if len(idx) > cap:
                # An even stride can stop a few short of the cap and leave the
                # plane step with nothing to solve. Take exactly cap points.
                pick = np.linspace(0, len(idx) - 1, cap).astype(np.int64)
                idx = idx[np.unique(pick)]
            chosen.append(idx)
    if not chosen:
        empty = slice(0, 0)
        return moved[empty], matched[empty], normals[empty]
    use = np.concatenate(chosen)
    return moved[use], matched[use], normals[use]


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
    surface that is actually new. On a fine map a later point within about
    8 cm is the same surface: the pose wanders by a few centimetres while the
    lidar sits still, and the range wanders by about a centimetre. It is not
    the old 5 cm floor on the first sweep. A surface about 15 cm farther is
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
        self._cell_tree = None
        self._cell_tree_size = 0
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

        A point in the same 2 cm cell as a stored point, or within four cells
        of it, is the same surface. That covers the pose wobble of a few
        centimetres plus the lidar's range scatter. Checking cells, instead of
        the distance to every stored point, keeps a full sweep fast enough to
        follow a moving lidar.
        """
        xyz = np.asarray(xyz, dtype=np.float64)
        count = len(xyz)
        if self._count == 0 or count == 0:
            return np.ones(count, dtype=bool)
        keys = np.floor(xyz / self.gate).astype(np.int64)
        occupied = self._bins
        if cKDTree is not None:
            # "Within four cells in every direction" is a largest-axis distance of 4.
            if self._cell_tree is None or self._cell_tree_size != len(occupied):
                self._cell_tree = cKDTree(np.array(list(occupied.keys()), dtype=np.float64))
                self._cell_tree_size = len(occupied)
            reach, _ = self._cell_tree.query(keys.astype(np.float64), p=np.inf, distance_upper_bound=4.5)
            return ~np.isfinite(reach)
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
    """Facing from the IMU quaternion, with gyro integration as the fallback.

    The L2's IMU has no compass, so the heading in its quaternion wanders away
    from the room over time. Roll and pitch do not, because gravity pins them.
    ``correction`` is the rotation that turns the IMU's orientation into the
    lidar's facing in the map. It starts as the facing of the first cloud and is
    then re-measured, about the vertical only, every time a sweep seats on the
    walls. Reading the raw quaternion against the first cloud forever (the old
    behaviour) hands every sweep the whole accumulated heading error again.
    """

    def __init__(self, quat_order="auto"):
        if quat_order not in ("auto", "xyzw", "wxyz"):
            raise ValueError("quat_order must be auto, xyzw or wxyz")
        # The Unitree driver fills the IMU message as if the quaternion were
        # x, y, z, w, and its own transform code as if it were w, x, y, z. Read
        # the wrong way, a turn of the lidar looks like the lidar rolling over,
        # and every turn redraws the room at a new angle. The gyro is not
        # ambiguous, so "auto" picks the layout that agrees with it.
        self.layout = ("xyzw", False) if quat_order == "auto" else (quat_order, False)
        self.layout_checked = quat_order != "auto"      # True once the gyro (or the user) has settled it
        self.layout_note = "set by hand" if quat_order != "auto" else "not checked yet"
        self.needs_anchor = False
        self._relock_gravity = False
        self._trial = None
        self._first_guess_done = quat_order != "auto"
        self.samples = []
        self.accel = []
        self.orientation = None
        self.origin = None
        self.correction = None
        self.gravity = None
        # (IMU stamp, rotation), so a sweep can ask where the lidar was facing
        # when each part of it was captured, not where it faces once the Pi
        # gets round to the sweep.
        self.facings = []
        self._facing_stamps = []
        # (IMU stamp, arrival on the Pi's clock). Clouds are timed by arrival.
        self._arrivals = []
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

    def learn_layout(self, stamp, quat, gyro, accel):
        """Work out how the quaternion is laid out. Call with every IMU message until layout_checked.

        At rest, the right layout turns the accelerometer reading to point
        straight up; that gives a first choice before anything has moved.
        The first real turn settles it: the change in the quaternion has to
        match the turn the gyro measured over the same moment.
        """
        if self.layout_checked:
            return
        quat = np.asarray(quat, dtype=np.float64).reshape(-1)
        gyro = np.asarray(gyro, dtype=np.float64).reshape(-1)
        accel = np.asarray(accel, dtype=np.float64).reshape(-1)
        if quat.shape != (4,) or not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-6:
            return
        if gyro.shape != (3,) or not np.all(np.isfinite(gyro)):
            return
        if not self._first_guess_done and accel.shape == (3,) and np.linalg.norm(accel) > 5.0:
            up = [
                (quat_to_rot(quat, order) @ accel)[2] / np.linalg.norm(accel) for order in ("xyzw", "wxyz")
            ]
            self._first_guess_done = True
            if up[1] > up[0] + 0.05:
                self._switch_layout(("wxyz", False), "from gravity, not yet confirmed by a turn")
            else:
                self.layout_note = "from gravity, not yet confirmed by a turn"
        stamp = float(stamp)
        trial = self._trial
        if trial is None or not 0 < stamp - trial["last"] <= _MAX_IMU_STEP or stamp - trial["start"] > 4.0:
            self._trial = {"start": stamp, "last": stamp, "quat": quat.copy(), "turn": np.eye(3)}
            return
        trial["turn"] = trial["turn"] @ rodrigues(gyro * (stamp - trial["last"]))
        trial["last"] = stamp
        if _rotation_angle(trial["turn"]) < np.radians(15.0):
            return
        errors = []
        for order, inverse in QUAT_LAYOUTS:
            change = quat_to_rot(trial["quat"], order, inverse).T @ quat_to_rot(quat, order, inverse)
            errors.append(np.degrees(_rotation_angle(change.T @ trial["turn"])))
        self._trial = None
        ranked = np.argsort(errors)
        if errors[ranked[0]] < 4.0 and errors[ranked[1]] > 8.0:
            best = QUAT_LAYOUTS[ranked[0]]
            note = f"confirmed by the gyro (off by {errors[ranked[0]]:.1f}° over a 15° turn)"
            if best != self.layout:
                self._switch_layout(best, note)
            self.layout_note = note
            self.layout_checked = True

    def _switch_layout(self, layout, note):
        """Everything remembered under the old layout is wrong. Drop it and ask to be re-anchored."""
        self.layout = layout
        self.layout_note = note
        self.facings.clear()
        self._facing_stamps.clear()
        self.accel.clear()
        self.orientation = None
        if self.origin is not None:
            self.needs_anchor = True

    def reanchor(self, map_rotation):
        """After a layout switch: take the lidar's current facing from the map and carry on from there."""
        if self.orientation is None:
            return
        self.origin = np.array(self.orientation, copy=True)
        self.correction = np.asarray(map_rotation, dtype=np.float64) @ self.orientation.T
        self.gravity = None
        self._relock_gravity = True
        self.needs_anchor = False

    def push_orientation(self, quat, stamp=None, arrival=None):
        """Keep the latest orientation, and its history when the IMU stamp is given.

        ``arrival`` is when the message reached the Pi, on the Pi's own clock.
        The cloud header is a different clock from the IMU header, so a sweep
        is placed on the IMU clock by when it arrived.
        """
        rotation = quat_to_rot(quat, *self.layout)
        if rotation is None:
            return
        self.orientation = rotation
        if stamp is None:
            return
        stamp = float(stamp)
        if self._facing_stamps and stamp <= self._facing_stamps[-1]:
            if stamp > self._facing_stamps[-1] - 1.0:
                return
            # The IMU clock stepped back (driver restarted). Old history is useless.
            self.facings.clear()
            self._facing_stamps.clear()
            self._arrivals.clear()
        self.facings.append(rotation)
        self._facing_stamps.append(stamp)
        if arrival is not None:
            self._arrivals.append((stamp, float(arrival)))
        cutoff = stamp - IMU_HISTORY
        drop = bisect_left(self._facing_stamps, cutoff)
        if drop > 256:
            del self.facings[:drop]
            del self._facing_stamps[:drop]
            self._arrivals = [pair for pair in self._arrivals if pair[0] >= cutoff]

    def imu_time(self, arrival):
        """IMU-clock time of something that reached the Pi at ``arrival``. None without IMU arrivals.

        Messages queue up while a sweep is being worked on, so one arrival can
        be late. The quickest delivery in the history is the true offset.
        """
        if not self._arrivals:
            return None
        return float(arrival) - min(arrived - stamp for stamp, arrived in self._arrivals)

    def orientation_at(self, stamp):
        """IMU orientation at an IMU stamp. Falls back to the latest when there is no history."""
        if stamp is None or not self._facing_stamps:
            return self.orientation
        index = bisect_left(self._facing_stamps, float(stamp))
        if index <= 0:
            return self.facings[0]
        if index >= len(self.facings):
            return self.facings[-1]
        before = self._facing_stamps[index - 1]
        after = self._facing_stamps[index]
        return self.facings[index if after - stamp < stamp - before else index - 1]

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
            self.correction = self.origin.T.copy()

    def correct(self, map_rotation, stamp=None):
        """Re-measure the heading offset from a sweep that seated on the walls.

        ``map_rotation`` is the lidar's facing in the map that the walls agreed
        on at IMU time ``stamp``. Only the turn about the vertical is taken from
        it: the IMU already knows which way is down, and letting a slightly
        tilted match overwrite that would lean every later sweep.
        Returns the heading change in degrees.
        """
        measured = self.orientation_at(stamp)
        if self.correction is None or measured is None:
            return 0.0
        error = np.asarray(map_rotation, dtype=np.float64) @ (self.correction @ measured).T
        angle = 0.0
        if self.gravity is not None and np.linalg.norm(self.gravity) > 1.0:
            up = self.gravity / np.linalg.norm(self.gravity)
            half_cos = 0.5 * np.sqrt(max(0.0, 1.0 + np.trace(error)))
            if half_cos > 1e-3:
                axis = np.array([
                    error[2, 1] - error[1, 2], error[0, 2] - error[2, 0], error[1, 0] - error[0, 1],
                ]) / (4.0 * half_cos)
                angle = 2.0 * np.arctan2(axis @ up, half_cos)
                error = rodrigues(up * angle)
        else:
            angle = _rotation_angle(error)
        fixed = error @ self.correction
        # Thousands of small products drift off a true rotation. Square it up.
        u, _, vt = np.linalg.svd(fixed)
        self.correction = u @ vt
        return float(np.degrees(angle))

    def lock_gravity(self):
        """The still reading, in the map, is gravity. Later moves are measured from it."""
        if self.gravity is not None or self.origin is None or len(self.accel) < 5:
            return
        samples = [self.correction @ rotation @ accel for _, accel, rotation in self.accel]
        self.gravity = np.mean(samples, axis=0)
        self._relock_gravity = False

    def _map_accel(self, accel, rotation):
        return self.correction @ rotation @ np.asarray(accel, dtype=np.float64) - self.gravity

    def shift_since(self, start, end, max_gap=0.5, max_norm=0.75):
        """Map displacement from the accelerometer, or None when gravity or the gap is missing.

        Velocity starts at rest at ``start``. Steady speed is carried by the last
        accepted move; this integral is only the extra acceleration in the gap.
        ``max_gap`` keeps a stale clock from integrating for several seconds and
        shoving the pose off the map.
        """
        moved = self.motion_since(start, end, max_gap)
        if moved is None or np.linalg.norm(moved[0]) > max_norm:
            return None
        return moved[0]

    def motion_since(self, start, end, max_gap=0.5):
        """(displacement from rest, change in velocity) between two IMU stamps, in the map. None if unknown.

        The change in velocity is what lets a carried speed slow down again.
        Without it a lidar that had been moving was assumed to keep moving.
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
            if 0 < step <= _MAX_IMU_STEP:
                position = position + velocity * step + 0.5 * accel * step * step
                velocity = velocity + accel * step
            previous_time = stamp
            accel = self._map_accel(sample, rotation)
        tail = end - previous_time
        if 0 < tail <= _MAX_IMU_STEP:
            position = position + velocity * tail + 0.5 * accel * tail * tail
            velocity = velocity + accel * tail
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return None
        return position, velocity

    def advance(self, until=None):
        """Mark the IMU samples up to ``until`` (default: all so far) as used by a kept sweep."""
        if self.samples:
            self._gyro_cursor = self.samples[-1][0] if until is None else min(self.samples[-1][0], until)
        if self.accel:
            self._accel_cursor = self.accel[-1][0] if until is None else min(self.accel[-1][0], until)

    def _preview_span(self, series, cursor, until=None):
        if len(series) < 2:
            return None
        end = series[-1][0] if until is None else min(series[-1][0], until)
        start = end if cursor is None else cursor
        if end <= start:
            return None
        if end - start > MOTION_GAP:
            start = end - MOTION_GAP
        return start, end

    def preview_shift(self, until=None):
        """Accel move since the last kept sweep, on the IMU clock."""
        span = self._preview_span(self.accel, self._accel_cursor, until)
        if span is None:
            return None
        return self.shift_since(span[0], span[1], max_gap=MOTION_GAP, max_norm=STEP_LIMIT)

    def preview_rotation(self, until=None):
        """Gyro turn since the last kept sweep, on the IMU clock."""
        span = self._preview_span(self.samples, self._gyro_cursor, until)
        if span is None:
            return None
        return self.rotation_since(span[0], span[1])

    def facing_in_map(self, stamp=None):
        """Lidar facing in the map at an IMU stamp (default: now), or None if it is not known."""
        measured = self.orientation_at(stamp)
        if measured is None or self.correction is None:
            return None
        return self.correction @ measured

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
            if 0 < step <= _MAX_IMU_STEP:
                rotation = rotation @ rodrigues(rate * step)
            previous_time = stamp
            rate = gyro
        tail = end - previous_time
        if 0 < tail <= _MAX_IMU_STEP:
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

    def add_scan(self, xyz, intensity=None, rotation_hint=None, facing=None, translation_hint=None, carried=None,
                 facing_tolerance=None):
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
        # Where an explicit motion measurement would put the lidar. Kept so a
        # sweep that cannot lock still follows the move instead of freezing.
        # The last step alone is not a measurement: coasting on it would walk
        # the pose while the lidar is sitting still.
        motion_guess = None
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
                explicit = carried is not None
                if translation_hint is not None:
                    hint = np.asarray(translation_hint, dtype=np.float64)
                    length = float(np.linalg.norm(hint))
                    if length > STEP_LIMIT:
                        hint = hint * (STEP_LIMIT / length)
                    guess_translation = guess_translation + hint
                    explicit = True
                if explicit:
                    motion_guess = np.array(guess_translation, copy=True)
            rotation, translation, ok, fit = align_scan(
                xyz, normals, self.map.xyz(), self.map.normals(),
                guess_rotation, guess_translation, self.max_correspondence,
            )
            if ok and facing_tolerance is not None and facing is not None:
                if _rotation_angle(rotation @ guess_rotation.T) > facing_tolerance:
                    ok, fit = False, None
            if ok:
                break
        self.last_fit = fit
        if not ok:
            if motion_guess is None:
                # Keep the last speed. Zeroing it left the next guess at the
                # start after a single miss, and every later sweep then missed too.
                return "skipped"
            delta = motion_guess - previous_translation
            length = float(np.linalg.norm(delta))
            if length < 0.02:
                return "skipped"
            if length > STEP_LIMIT:
                delta = delta * (STEP_LIMIT / length)
            # The walls did not agree, so nothing is stored. The pose still
            # moves, and the next sweep is guessed from here.
            self.rotation = guess_rotation
            self.translation = previous_translation + delta
            self.step = delta
            return "tracking"
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


def sweep_offsets(point_time):
    """Seconds before the end of the sweep at which each point was captured (zero or negative).

    The driver glues many packets into one cloud and restarts the ``time``
    field at zero in each packet, so the field alone only places a point inside
    its own line. The restarts mark the packets, and the packets are evenly
    spaced. Returns None when the cloud carries no usable time.
    """
    if point_time is None:
        return None
    point_time = np.asarray(point_time, dtype=np.float64).reshape(-1)
    if len(point_time) < 2 or not np.all(np.isfinite(point_time)):
        return None
    if point_time.min() < 0 or point_time.max() > 0.5:
        return None
    restart = np.diff(point_time) < 0
    packet = np.concatenate(([0], np.cumsum(restart)))
    steps = np.diff(point_time)[~restart]
    steps = steps[steps > 0]
    if len(steps) == 0:
        return None
    # A packet's last returns can be dropped as out of range, so the longest
    # packet seen is the best measure of the spacing between packets.
    period = float(point_time.max() + np.median(steps))
    offsets = (packet - packet[-1]) * period + point_time - period
    return offsets


def deskew(xyz, offsets, imu, end_stamp, velocity=None, facing=None):
    """Redraw a sweep as if every point had been captured at the end of it.

    A sweep takes about a third of a second. A lidar that turns or travels in
    that time smears the walls across the cloud, and the smeared cloud then
    seats on the map at the wrong angle. Each packet is turned by how far the
    IMU says the lidar rotated between that packet and the end of the sweep.
    With ``velocity`` (map frame) and ``facing`` (map from lidar at the end),
    the distance travelled is taken out too.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    if offsets is None or end_stamp is None or len(xyz) != len(offsets) or not imu.facings:
        return xyz
    end_rotation = imu.orientation_at(end_stamp)
    drift = None
    if velocity is not None and facing is not None and np.linalg.norm(velocity) > 1e-3:
        drift = np.asarray(facing, dtype=np.float64).T @ np.asarray(velocity, dtype=np.float64)
    out = np.empty_like(xyz)
    # About 5 ms per packet: one orientation per packet is as fine as the IMU reports.
    slot = np.round(offsets / 0.004).astype(np.int64)
    order = np.argsort(slot, kind="stable")
    bounds = np.concatenate(([0], np.flatnonzero(np.diff(slot[order])) + 1, [len(order)]))
    for first, last in zip(bounds[:-1], bounds[1:]):
        members = order[first:last]
        before_end = float(offsets[members].mean())
        turn = end_rotation.T @ imu.orientation_at(end_stamp + before_end)
        moved = xyz[members] @ turn.T
        if drift is not None:
            moved = moved + drift * before_end
        out[members] = moved
    return out


class Mapper:
    """The mapping loop without ROS: feed it IMU messages and clouds as they arrive.

    The Pi script and the simulator both drive this, so what is tested at a
    desk is what runs on the aircraft.
    """

    def __init__(self, voxel=DEFAULT_VOXEL, max_correspondence=0.4, deskew_sweeps=True,
                 correct_heading=True, quat_order="auto"):
        self.tracker = Tracker(voxel, max_correspondence)
        self.imu = ImuTrack(quat_order)
        self.deskew_sweeps = deskew_sweeps
        self.correct_heading = correct_heading
        self.counts = {"seed": 0, "aligned": 0, "tracking": 0, "skipped": 0}
        self.last_kept = None
        self.last_end = None           # IMU-clock end of the last sweep that moved the pose
        self.last_aligned = None       # arrival of the last sweep that seated on the walls
        self.misses = 0                # sweeps in a row that did not seat
        self.heading_fix_deg = 0.0     # running total the walls have corrected the IMU heading by
        self.last = {}

    def on_imu(self, stamp, quat, gyro, accel, arrival=None):
        """One IMU message. ``stamp`` is its header time; ``arrival`` is the Pi's clock."""
        self.imu.learn_layout(stamp, quat, gyro, accel)
        self.imu.push(stamp, gyro)
        self.imu.push_orientation(quat, stamp, stamp if arrival is None else arrival)
        self.imu.push_accel(stamp, accel)

    def on_cloud(self, arrival, xyz, intensity=None, point_time=None):
        """One cloud. ``arrival`` is when it reached the Pi; the sweep ended about then."""
        tracker, imu = self.tracker, self.imu
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        end = imu.imu_time(arrival)
        if imu.needs_anchor:
            imu.reanchor(tracker.rotation)
        facing = None if len(tracker.map) == 0 else imu.facing_in_map(end)
        # Without walls to check against, the accelerometer is only good for
        # about a second. Past that, coasting on it carries the pose out of the
        # room, where it can never find the walls again. Stay put and search.
        lost = (
            self.misses > 0 and self.last_aligned is not None
            and arrival - self.last_aligned > COAST_LIMIT
        )
        if lost:
            tracker.speed = np.zeros(3)
        if self.deskew_sweeps and end is not None:
            offsets = sweep_offsets(point_time)
            velocity = tracker.speed
            if offsets is not None and self.last_end is not None and not lost:
                # tracker.speed is the speed at the end of the last sweep. The
                # accelerometer brings it up to the middle of this one.
                gained = imu.motion_since(self.last_end, end + 0.5 * float(offsets.min()), MOTION_GAP)
                if gained is not None and np.linalg.norm(gained[1]) < SPEED_STEP_LIMIT:
                    velocity = velocity + gained[1]
            xyz = deskew(xyz, offsets, imu, end,
                         velocity=velocity, facing=facing if facing is not None else tracker.rotation)
        delta = imu.preview_rotation(end)
        shift = None if lost else imu.preview_shift(end)
        carried = None
        if self.last_kept is not None:
            elapsed = arrival - self.last_kept
            if lost:
                carried = np.zeros(3)
            elif 0.02 < elapsed <= MOTION_GAP:
                carried = tracker.speed * elapsed
        # The map frame is the IMU frame of the cloud that actually seeds it.
        # If that quaternion arrives one cloud late, adopt it only while the
        # lidar is still at the start. A later quaternion must not redefine
        # the room.
        if (
            imu.origin is None
            and len(tracker.map) > 0
            and self.counts["aligned"] == 0
            and np.allclose(tracker.rotation, np.eye(3))
        ):
            imu.lock_origin()
            facing = imu.facing_in_map(end)
        # The corrected IMU facing can only be off by what the heading has
        # wandered since the walls last agreed. A match that turns the room much
        # further than that is the wrong wall (a square room fits itself a
        # quarter turn round), and keeping it would also teach that quarter
        # turn to every later sweep.
        tolerance = None
        if facing is not None and self.correct_heading and self.counts["aligned"] > 0:
            since = 0.0 if self.last_aligned is None else max(0.0, arrival - self.last_aligned)
            tolerance = np.radians(min(HEADING_TRUST_DEG + HEADING_WANDER_DEG_S * since, 45.0))
        status = tracker.add_scan(
            xyz, intensity,
            rotation_hint=delta,
            facing=facing,
            translation_hint=shift,
            carried=carried,
            facing_tolerance=tolerance,
        )
        self.misses = 0 if status in ("seed", "aligned") else self.misses + 1
        if status == "seed":
            imu.lock_origin()
        if imu.gravity is None and (np.linalg.norm(tracker.translation) < 0.02 or imu._relock_gravity):
            imu.lock_gravity()
        if status == "aligned" and self.correct_heading:
            # The walls have just said where the lidar is really facing. Carry
            # that into the next sweep's starting guess instead of throwing it away.
            self.heading_fix_deg += imu.correct(tracker.rotation, end)
        if status in ("seed", "aligned"):
            self.last_aligned = arrival
        if status != "skipped":
            if self.last_kept is not None:
                tracker.note_elapsed(arrival - self.last_kept)
                # That is the average speed since the last sweep. Half of what
                # the accelerometer saw in between makes it the speed right now.
                gained = None
                if self.last_end is not None and end is not None:
                    gained = imu.motion_since(self.last_end, end, MOTION_GAP)
                if gained is not None and np.linalg.norm(gained[1]) < SPEED_STEP_LIMIT:
                    tracker.speed = tracker.speed + 0.5 * gained[1]
            imu.advance(end)
            self.last_kept = arrival
            self.last_end = end
        self.counts[status] += 1
        self.last = {"status": status, "gyro_turn": delta}
        return status

    def status_line(self):
        tracker, imu, counts = self.tracker, self.imu, self.counts
        place = tracker.translation
        fit = "  --" if tracker.last_fit is None else f"{tracker.last_fit * 100:4.1f}cm"
        reported = imu.facing_in_map()
        face = "  --" if reported is None else f"{heading_deg(reported):5.1f}°"
        imu_on = imu.orientation is not None or len(imu.samples) >= 2
        return (
            f"{self.last.get('status', 'waiting')}: map {len(tracker.map)} points, "
            f"lidar at {place[0]:.2f} {place[1]:.2f} {place[2]:.2f} m, "
            f"yaw={heading_deg(tracker.rotation):5.1f}° face={face} "
            f"imu heading fix={self.heading_fix_deg:+.1f}°, "
            f"wall fit {fit}, imu {'on' if imu_on else 'off'} "
            f"({imu.layout[0]}{' inverse' if imu.layout[1] else ''}{'' if imu.layout_checked else '?'}), "
            f"aligned {counts['aligned']}, tracking {counts['tracking']}, "
            f"skipped {counts['skipped']}"
        )


def collect(seconds_between_saves, topic, imu_topic, output, voxel, max_correspondence,
            deskew_sweeps=True, correct_heading=True, quat_order="auto"):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("stable_map")
    mapper = Mapper(voxel, max_correspondence, deskew_sweeps, correct_heading, quat_order)
    layout_said = None
    tracker = mapper.tracker
    # Messages are received on their own thread and stamped as they arrive.
    # Taking them one at a time between sweeps left the IMU reading seconds
    # stale whenever a sweep took longer than the gap between clouds.
    imu_inbox = deque(maxlen=4000)
    cloud_inbox = deque(maxlen=1)      # only the newest sweep is worth working on
    last_save = time.monotonic()
    imu_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=400,
    )

    def on_cloud(msg: PointCloud2) -> None:
        cloud_inbox.append((time.monotonic(), msg))

    def on_imu(msg: Imu) -> None:
        imu_inbox.append((
            time.monotonic(),
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
        ))

    node.create_subscription(PointCloud2, topic, on_cloud, qos_profile_sensor_data)
    node.create_subscription(Imu, imu_topic, on_imu, imu_qos)

    def receive():
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        except Exception as error:  # the context can already be gone at shutdown
            if rclpy.ok():
                print(f"receiver stopped: {error}", flush=True)

    threading.Thread(target=receive, daemon=True).start()
    print(
        f"mapping {topic} into {output} with about {voxel * 1000:.1f} mm between points"
        " — hold still until the count levels off, then move it freely",
        flush=True,
    )

    def save():
        cloud = tracker.map.cloud()
        if len(cloud) == 0:
            return
        write_pcd(output, cloud)
        print(f"wrote {len(cloud)} fixed points to {output}", flush=True)

    def read(msg):
        names = {field.name for field in msg.fields}
        wanted = ["x", "y", "z"] + [name for name in ("intensity", "time") if name in names]
        points = point_cloud2.read_points(msg, field_names=wanted, skip_nans=True)
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float64)
        intensity = (
            np.asarray(points["intensity"], dtype=np.float64) if "intensity" in names
            else np.ones(len(xyz))
        )
        stamps = np.asarray(points["time"], dtype=np.float64) if "time" in names else None
        return xyz, intensity, stamps

    try:
        while rclpy.ok():
            if not cloud_inbox:
                time.sleep(0.005)
                if tracker.map and time.monotonic() - last_save >= seconds_between_saves:
                    save()
                    last_save = time.monotonic()
                continue
            arrival, msg = cloud_inbox.popleft()
            while imu_inbox:
                arrived, stamp, quat, gyro, accel = imu_inbox.popleft()
                mapper.on_imu(stamp, quat, gyro, accel, arrival=arrived)
            try:
                xyz, intensity, stamps = read(msg)
            except (AssertionError, ValueError):
                xyz = point_cloud2.read_points_numpy(
                    msg, field_names=["x", "y", "z"], skip_nans=True
                )
                intensity, stamps = np.ones(len(xyz)), None
            started = time.monotonic()
            mapper.on_cloud(arrival, xyz, intensity, stamps)
            print(f"{mapper.status_line()}, {time.monotonic() - started:.2f} s", flush=True)
            layout_now = (mapper.imu.layout, mapper.imu.layout_note)
            if layout_now != layout_said and mapper.imu.orientation is not None:
                layout_said = layout_now
                order, inverse = mapper.imu.layout
                print(f"IMU quaternion read as {order}{' (inverse)' if inverse else ''}: "
                      f"{mapper.imu.layout_note}", flush=True)
            if tracker.map and time.monotonic() - last_save >= seconds_between_saves:
                save()
                last_save = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        save()
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
    parser.add_argument(
        "--quat-order", choices=("auto", "xyzw", "wxyz"), default="auto",
        help="layout of the IMU quaternion. auto checks it against the gyro (default)",
    )
    parser.add_argument(
        "--no-deskew", action="store_true",
        help="do not straighten sweeps captured while the lidar was moving (for comparison)",
    )
    parser.add_argument(
        "--no-heading-correction", action="store_true",
        help="read the raw IMU heading against the first cloud forever, as before (for comparison)",
    )
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
        deskew_sweeps=not args.no_deskew,
        correct_heading=not args.no_heading_correction,
        quat_order=args.quat_order,
    )


if __name__ == "__main__":
    main()

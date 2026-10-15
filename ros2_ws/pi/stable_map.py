#!/usr/bin/env python3
"""Build a map that stays put while the lidar moves.

The first cloud defines the map. Each later cloud is aligned to that map and
only empty voxels receive new points, so points already stored are not moved.
Stop with Ctrl-C. The latest map is rewritten to ~/map.pcd while this runs.

Leave the lidar launch running, then:

    python3 ros2_ws/pi/stable_map.py

Move slowly enough that each sweep still overlaps the map. A sweep that cannot
be aligned is skipped instead of dragging the old points with it.
"""

import argparse
import time
from pathlib import Path

import numpy as np

SOURCE_CAP = 1500
TARGET_CAP = 3000
MIN_PAIRS = 40


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


def stride(points, limit):
    if len(points) <= limit:
        return points
    step = int(np.ceil(len(points) / limit))
    return points[::step]


def rigid_transform(src, dst):
    """Rotation and translation that map src onto dst. Both are Nx3."""
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    src_center = src.mean(axis=0)
    dst_center = dst.mean(axis=0)
    covariance = (src - src_center).T @ (dst - dst_center)
    left, _, vh = np.linalg.svd(covariance)
    rotation = vh.T @ left.T
    if np.linalg.det(rotation) < 0:
        vh = vh.copy()
        vh[-1] *= -1
        rotation = vh.T @ left.T
    translation = dst_center - rotation @ src_center
    return rotation, translation


def apply_pose(rotation, translation, xyz):
    return (rotation @ np.asarray(xyz, dtype=np.float64).T).T + translation


def predict_pose(older_rotation, older_translation, rotation, translation):
    """Constant-velocity guess. With one previous pose, assume it stayed still."""
    if older_rotation is None:
        return rotation.copy(), translation.copy()
    inverse_rotation = older_rotation.T
    inverse_translation = -inverse_rotation @ older_translation
    delta_rotation = rotation @ inverse_rotation
    delta_translation = rotation @ inverse_translation + translation
    return delta_rotation @ rotation, delta_rotation @ translation + delta_translation


def correspondences(source, target, max_distance):
    """Pairs of source points and their nearest target point within max_distance."""
    if len(source) == 0 or len(target) == 0:
        return np.empty((0, 3)), np.empty((0, 3))
    limit = max_distance * max_distance
    matched_source = []
    matched_target = []
    chunk = 250
    for start in range(0, len(source), chunk):
        block = source[start:start + chunk]
        distance_sq = np.sum((block[:, None, :] - target[None, :, :]) ** 2, axis=2)
        nearest = np.argmin(distance_sq, axis=1)
        best = distance_sq[np.arange(len(block)), nearest]
        keep = best <= limit
        if np.any(keep):
            matched_source.append(block[keep])
            matched_target.append(target[nearest[keep]])
    if not matched_source:
        return np.empty((0, 3)), np.empty((0, 3))
    return np.vstack(matched_source), np.vstack(matched_target)


def align_scan(source, target, rotation, translation, max_distance, iterations=8):
    """Align source (lidar frame) to target (map frame). Returns pose and whether it held."""
    if len(source) < MIN_PAIRS or len(target) < MIN_PAIRS:
        return rotation, translation, False
    current_rotation = np.array(rotation, dtype=np.float64, copy=True)
    current_translation = np.array(translation, dtype=np.float64, copy=True)
    source = stride(np.asarray(source, dtype=np.float64), SOURCE_CAP)
    target = stride(np.asarray(target, dtype=np.float64), TARGET_CAP)
    tight = max(0.12, max_distance * 0.35)
    for step in range(iterations):
        limit = max_distance if step < iterations // 2 else tight
        moved = apply_pose(current_rotation, current_translation, source)
        paired_source, paired_target = correspondences(moved, target, limit)
        if len(paired_source) < MIN_PAIRS:
            return rotation, translation, False
        delta_rotation, delta_translation = rigid_transform(paired_source, paired_target)
        current_rotation = delta_rotation @ current_rotation
        current_translation = delta_rotation @ current_translation + delta_translation
        if np.linalg.norm(delta_translation) < 1e-3 and np.linalg.norm(delta_rotation - np.eye(3)) < 1e-3:
            break
    moved = apply_pose(current_rotation, current_translation, source)
    paired_source, _paired_target = correspondences(moved, target, tight)
    if len(paired_source) < MIN_PAIRS or len(paired_source) / len(source) < 0.3:
        return rotation, translation, False
    return current_rotation, current_translation, True


class StableMap:
    """Voxel map. The first point to claim a voxel is kept permanently."""

    def __init__(self, voxel):
        self.voxel = float(voxel)
        self.occupied = set()
        self._capacity = 0
        self._count = 0
        self._points = np.empty((0, 4), dtype=np.float32)

    def __len__(self):
        return self._count

    def cloud(self):
        return self._points[:self._count].copy()

    def xyz(self):
        return np.asarray(self._points[:self._count, :3], dtype=np.float64)

    def insert(self, xyz, intensity):
        xyz = np.asarray(xyz, dtype=np.float64)
        intensity = np.asarray(intensity, dtype=np.float64)
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
        self._append(block)
        return len(block)

    def _append(self, block):
        count = self._count
        need = count + len(block)
        if need > self._capacity:
            self._capacity = max(need, self._capacity * 2, 1024)
            grown = np.empty((self._capacity, 4), dtype=np.float32)
            if count:
                grown[:count] = self._points[:count]
            self._points = grown
        self._points[count:need] = block
        self._count = need


class Tracker:
    """Align each sweep to the map, then add only voxels that are still empty."""

    def __init__(self, voxel=0.03, max_correspondence=0.45):
        self.voxel = float(voxel)
        self.max_correspondence = float(max_correspondence)
        self.map = StableMap(self.voxel)
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self._older_rotation = None
        self._older_translation = None

    def add_scan(self, xyz, intensity=None):
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        if intensity is None:
            intensity = np.ones(len(xyz), dtype=np.float64)
        else:
            intensity = np.asarray(intensity, dtype=np.float64).reshape(-1)
        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        xyz, intensity = voxel_downsample(xyz[finite], intensity[finite], self.voxel)
        if len(xyz) == 0:
            return "skipped"
        if len(self.map) == 0:
            self.map.insert(xyz, intensity)
            return "seed"
        guess_rotation, guess_translation = predict_pose(
            self._older_rotation, self._older_translation, self.rotation, self.translation
        )
        rotation, translation, ok = align_scan(
            xyz, self.map.xyz(), guess_rotation, guess_translation, self.max_correspondence
        )
        if not ok:
            return "skipped"
        self._older_rotation = self.rotation
        self._older_translation = self.translation.copy()
        self.rotation = rotation
        self.translation = translation
        self.map.insert(apply_pose(rotation, translation, xyz), intensity)
        return "aligned"


def collect(seconds_between_saves, topic, output, voxel, max_correspondence):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("stable_map")
    tracker = Tracker(voxel, max_correspondence)
    pending = {}
    last_save = time.monotonic()
    counts = {"seed": 0, "aligned": 0, "skipped": 0}

    def on_cloud(msg: PointCloud2) -> None:
        pending["msg"] = msg

    node.create_subscription(PointCloud2, topic, on_cloud, qos_profile_sensor_data)
    print(
        f"mapping {topic} into {output} at {voxel:.2f} m voxels — move the lidar slowly",
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
            if msg is not None:
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
                status = tracker.add_scan(xyz, intensity)
                counts[status] += 1
                place = tracker.translation
                print(
                    f"{status}: map {len(tracker.map)} points, "
                    f"lidar at {place[0]:.2f} {place[1]:.2f} {place[2]:.2f} m, "
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
    parser.add_argument("--output", default=str(Path.home() / "map.pcd"))
    parser.add_argument("--voxel", type=float, default=0.03, help="map cell size in meters")
    parser.add_argument(
        "--max-correspondence",
        type=float,
        default=0.45,
        help="farthest a new point may be from the map and still count as the same surface",
    )
    parser.add_argument("--save-every", type=float, default=5.0, help="seconds between PCD writes")
    args = parser.parse_args()
    if args.voxel <= 0 or args.max_correspondence <= 0 or args.save_every <= 0:
        raise SystemExit("voxel, max-correspondence, and save-every must be greater than 0")
    collect(
        args.save_every,
        args.topic,
        Path(args.output).expanduser(),
        args.voxel,
        args.max_correspondence,
    )


if __name__ == "__main__":
    main()

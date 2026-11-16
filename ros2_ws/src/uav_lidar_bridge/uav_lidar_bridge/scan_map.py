"""Build a 3D scan from LiDAR clouds and aircraft poses. No ROS imports, so it tests anywhere.

Frames: LiDAR points -> body FLU (mount offset, same as obstacle sectors) -> world ENU
(MAVROS local_position/pose convention: x east, y north, z up). The saved map is ENU.

Poses are interpolated to each cloud's timestamp. With `deskew`, a cloud is split by
each point's own capture time so motion during one message doesn't smear the map.
"""
import json
import math
import os
from bisect import bisect_left
from collections import deque

import numpy as np

from .sectors import rotation_rpy


# --- rotations -----------------------------------------------------------------

def quat_to_matrix(q):
    """Unit quaternion (x, y, z, w), ROS order, to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def slerp(q0, q1, f):
    """Shortest-path spherical interpolation between (x, y, z, w) quaternions."""
    q0 = np.asarray(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float) / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:  # nearly equal: linear is accurate and avoids dividing by ~0
        q = q0 + f * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1 - f) * theta) * q0 + math.sin(f * theta) * q1) / math.sin(theta)


# --- pose history --------------------------------------------------------------

class PoseBuffer:
    """Recent aircraft poses; answers "where was the aircraft at time t?".

    Refuses to answer (returns None) outside the stored range or across a gap
    longer than max_gap_s, rather than guessing.
    """

    def __init__(self, max_gap_s=0.2, keep_s=5.0):
        self.max_gap_s = max_gap_s
        self.keep_s = keep_s
        self.times = deque()
        self.poses = deque()  # (position xyz, quaternion xyzw)

    def add(self, t, position, quat):
        if not (np.all(np.isfinite(position)) and np.all(np.isfinite(quat))
                and np.linalg.norm(quat) > 0.5):
            return False
        if self.times and t <= self.times[-1]:
            if t < self.times[-1] - 1.0:  # clock jumped backwards (e.g. bag restarted)
                self.times.clear()
                self.poses.clear()
            else:
                return False
        self.times.append(t)
        self.poses.append((np.asarray(position, dtype=float), np.asarray(quat, dtype=float)))
        while self.times and self.times[-1] - self.times[0] > self.keep_s:
            self.times.popleft()
            self.poses.popleft()
        return True

    def at(self, t):
        """(R world<-body, p world) at time t, or None if not known well enough."""
        if not self.times or t < self.times[0] or t > self.times[-1]:
            return None
        i = bisect_left(self.times, t)
        if self.times[i] == t:
            p, q = self.poses[i]
            return quat_to_matrix(q), p.copy()
        t0, t1 = self.times[i - 1], self.times[i]
        if t1 - t0 > self.max_gap_s:
            return None
        f = (t - t0) / (t1 - t0)
        (p0, q0), (p1, q1) = self.poses[i - 1], self.poses[i]
        return quat_to_matrix(slerp(q0, q1, f)), p0 + f * (p1 - p0)


# --- the map -------------------------------------------------------------------

_OFFSET = 1 << 20          # voxel indices from -2^20..2^20 fit in 21 bits each
_MASK = (1 << 21) - 1


def _pack(ijk):
    ijk = ijk + _OFFSET
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


class VoxelMap:
    """Point map that keeps one averaged point per voxel_size cube.

    Memory stays bounded by the scanned volume, not by flight time. Each voxel stores
    the mean position and mean intensity of every return that landed in it.
    """

    def __init__(self, voxel_size=0.05, max_voxels=4_000_000):
        if not voxel_size > 0:
            raise ValueError('voxel_size must be positive')
        self.voxel_size = float(voxel_size)
        self.max_voxels = int(max_voxels)
        self.index = {}
        self.sums = np.zeros((1024, 4))   # x, y, z, intensity
        self.counts = np.zeros(1024, dtype=np.int64)
        self.full = False

    def __len__(self):
        return len(self.index)

    def add(self, xyz, intensity=None):
        """Add world-frame points. Returns how many were stored (0 once full)."""
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        inten = np.zeros(len(xyz)) if intensity is None else np.asarray(intensity, float).ravel()
        ok = np.all(np.isfinite(xyz), axis=1) & np.isfinite(inten)
        xyz, inten = xyz[ok], inten[ok]
        if not len(xyz):
            return 0
        ijk = np.floor(xyz / self.voxel_size).astype(np.int64)
        in_range = np.all(np.abs(ijk) < _OFFSET, axis=1)
        xyz, inten, ijk = xyz[in_range], inten[in_range], ijk[in_range]
        keys, inverse = np.unique(_pack(ijk), return_inverse=True)

        if self.full:
            known = np.fromiter((self.index.get(k, -1) for k in keys.tolist()), np.int64, len(keys))
        else:
            room = self.max_voxels - len(self.index)
            new = [k for k in keys.tolist() if k not in self.index]
            if len(new) > room:
                self.full = True
                new = new[:room]
            for k in new:
                self.index[k] = len(self.index)
            known = np.fromiter((self.index.get(k, -1) for k in keys.tolist()), np.int64, len(keys))
            self._grow(len(self.index))

        rows = known[inverse]
        keep = rows >= 0
        np.add.at(self.sums, rows[keep], np.column_stack([xyz[keep], inten[keep]]))
        np.add.at(self.counts, rows[keep], 1)
        return int(keep.sum())

    def _grow(self, needed):
        if needed <= len(self.counts):
            return
        size = max(needed, 2 * len(self.counts))
        self.sums = np.vstack([self.sums, np.zeros((size - len(self.sums), 4))])
        self.counts = np.concatenate([self.counts, np.zeros(size - len(self.counts), np.int64)])

    def points(self):
        """(N,3) mean positions, (N,) mean intensity, (N,) return counts."""
        n = len(self.index)
        c = self.counts[:n]
        mean = self.sums[:n] / np.maximum(c, 1)[:, None]
        return mean[:, :3], mean[:, 3], c.copy()


# --- turning one cloud into world points -----------------------------------------

def lidar_to_world(points, R_wb, p_wb, R_bl, p_bl):
    """LiDAR-frame points -> world, given body pose (R_wb, p_wb) and mount (R_bl, p_bl)."""
    body = np.asarray(points, float).reshape(-1, 3) @ R_bl.T + p_bl
    return body @ R_wb.T + p_wb


class CloudIntegrator:
    """Filters each cloud, places it with the pose buffer, and adds it to a VoxelMap.

    pose_buffer=None means stationary capture: the body frame is the world frame.
    That is for bench scans with the sensor sitting still (no flight controller needed).
    """

    def __init__(self, vmap, *, mount_xyz=(0, 0, 0), mount_rpy=(0, 0, 0),
                 range_min=0.35, range_max=30.0, deskew_slices=8, pose_buffer=None):
        self.map = vmap
        self.R_bl = rotation_rpy(*mount_rpy)
        self.p_bl = np.asarray(mount_xyz, float)
        self.range_min, self.range_max = range_min, range_max
        self.deskew_slices = max(1, int(deskew_slices))
        self.poses = pose_buffer
        self.stats = dict(clouds=0, used=0, no_pose=0, points_in=0, points_added=0)

    def add_cloud(self, stamp, xyz, intensity=None, point_time=None):
        """stamp: cloud time (s). point_time: optional per-point offset from stamp (s)."""
        s = self.stats
        s['clouds'] += 1
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        n = len(xyz)
        inten = np.zeros(n) if intensity is None else np.asarray(intensity, float).ravel()
        r = np.linalg.norm(xyz, axis=1)
        keep = np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)
        xyz, inten = xyz[keep], inten[keep]
        s['points_in'] += int(keep.sum())

        if not len(xyz):  # everything was too close (props/frame) or too far
            return False

        if self.poses is None:
            world = lidar_to_world(xyz, np.eye(3), np.zeros(3), self.R_bl, self.p_bl)
            s['used'] += 1
            s['points_added'] += self.map.add(world, inten)
            return True

        times = np.full(len(xyz), float(stamp))
        if point_time is not None and self.deskew_slices > 1:
            dt = np.asarray(point_time, float).ravel()[keep]
            if np.all(np.isfinite(dt)) and len(dt) and 0 <= dt.min() and dt.max() < 1.0:
                times = stamp + dt

        slices = np.array_split(np.argsort(times, kind='stable'), self.deskew_slices) \
            if np.ptp(times) > 0 else [np.arange(len(xyz))]
        placed = 0
        for idx in slices:
            if not len(idx):
                continue
            pose = self.poses.at(float(np.median(times[idx])))
            if pose is None:
                continue
            world = lidar_to_world(xyz[idx], pose[0], pose[1], self.R_bl, self.p_bl)
            s['points_added'] += self.map.add(world, inten[idx])
            placed += 1
        if placed:
            s['used'] += 1
            return True
        s['no_pose'] += 1
        return False


# --- saving ------------------------------------------------------------------

def write_ply(path, xyz, intensity, counts):
    """Binary PLY that CloudCompare, MeshLab, Open3D and three.js PLYLoader all read."""
    n = len(xyz)
    rec = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                             ('intensity', '<f4'), ('count', '<u4')])
    rec['x'], rec['y'], rec['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rec['intensity'] = intensity
    rec['count'] = np.minimum(counts, np.iinfo(np.uint32).max)
    header = ('ply\nformat binary_little_endian 1.0\n'
              'comment 3d-mapping-uav scan, world frame ENU, meters\n'
              f'element vertex {n}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property float intensity\nproperty uint count\nend_header\n')
    tmp = path + '.part'
    with open(tmp, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(rec.tobytes())
    os.replace(tmp, path)  # readers never see a half-written file


def read_ply(path):
    """Read back what write_ply wrote (used by tests and quick checks)."""
    with open(path, 'rb') as f:
        n = None
        while (line := f.readline().decode('ascii').strip()) != 'end_header':
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
        rec = np.frombuffer(f.read(), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                             ('intensity', '<f4'), ('count', '<u4')], count=n)
    return np.column_stack([rec['x'], rec['y'], rec['z']]).astype(float), rec


def save_scan(folder, vmap, metadata):
    """Write map.ply and scan.json into folder. Returns the metadata that was written."""
    os.makedirs(folder, exist_ok=True)
    xyz, inten, counts = vmap.points()
    meta = dict(metadata)
    meta.update(points=int(len(xyz)), voxel_size_m=vmap.voxel_size, map_full=vmap.full,
                frame='ENU (x east, y north, z up), meters', file='map.ply')
    if len(xyz):
        meta['bounds_min'] = [round(float(v), 3) for v in xyz.min(axis=0)]
        meta['bounds_max'] = [round(float(v), 3) for v in xyz.max(axis=0)]
    write_ply(os.path.join(folder, 'map.ply'), xyz, inten, counts)
    tmp = os.path.join(folder, 'scan.json.part')
    with open(tmp, 'w') as f:
        json.dump(meta, f, indent=2)
    os.replace(tmp, os.path.join(folder, 'scan.json'))
    return meta

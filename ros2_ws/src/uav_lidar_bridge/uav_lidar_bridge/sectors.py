"""Reduce LiDAR points to 72 horizontal obstacle sectors (MAVLink OBSTACLE_DISTANCE).

Sector 0 points forward and angles increase clockwise seen from above (body FRD),
which is what ArduPilot expects. Input points are body FLU (x forward, y left, z up).
"""
from collections import deque
import math
import numpy as np

SECTOR_COUNT = 72
SECTOR_WIDTH_DEG = 360 / SECTOR_COUNT


def rotation_rpy(roll, pitch, yaw):
    """Fixed-axis roll, pitch, yaw in radians (same convention as ROS static TF)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def to_body_flu(points, mount_xyz, mount_rpy):
    """LiDAR-frame points to body FLU, given where the LiDAR sits on the airframe."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ rotation_rpy(*mount_rpy).T + np.asarray(mount_xyz, dtype=float)


def nearest_per_sector(points_flu, *, z_min, z_max, range_min, range_max):
    """Closest horizontal distance per sector in meters; NaN where nothing was seen."""
    p = np.asarray(points_flu, dtype=float).reshape(-1, 3)
    p = p[np.all(np.isfinite(p), axis=1)]
    p = p[(p[:, 2] >= z_min) & (p[:, 2] <= z_max)]
    distance = np.hypot(p[:, 0], p[:, 1])
    keep = (distance >= range_min) & (distance <= range_max)
    p, distance = p[keep], distance[keep]
    clockwise_deg = np.degrees(np.arctan2(-p[:, 1], p[:, 0])) % 360
    index = np.rint(clockwise_deg / SECTOR_WIDTH_DEG).astype(int) % SECTOR_COUNT
    nearest = np.full(SECTOR_COUNT, np.inf)
    np.minimum.at(nearest, index, distance)
    nearest[np.isinf(nearest)] = np.nan
    return nearest


class SectorMemory:
    """Closest reading per sector over the last hold_s seconds.

    One L2 cloud message covers only part of a sweep, so a single message leaves gaps.
    """

    def __init__(self, hold_s):
        if not (math.isfinite(hold_s) and hold_s > 0):
            raise ValueError('hold_s must be positive')
        self.hold_s = hold_s
        self.history = deque()

    def update(self, nearest, now):
        if self.history and now < self.history[-1][0]:
            self.history.clear()
        self.history.append((now, np.asarray(nearest, dtype=float)))
        while now - self.history[0][0] > self.hold_s:
            self.history.popleft()
        stacked = np.vstack([d for _, d in self.history])
        closest = np.where(np.isnan(stacked), np.inf, stacked).min(axis=0)
        closest[np.isinf(closest)] = np.nan
        return closest

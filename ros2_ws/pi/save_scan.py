#!/usr/bin/env python3
"""Stack /unilidar/cloud for a few seconds and write one CloudCompare PCD.

One published message is a single sweep. Keeping the lidar still and
collecting longer puts many sweeps on the same surfaces, so walls and
objects show up as solid clouds instead of a thin slice.
"""

import argparse
import time
from pathlib import Path

import numpy as np


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
    with path.open("wb") as out:
        out.write(header.encode("ascii"))
        out.write(pts.tobytes())


def collect_clouds(seconds: float, topic: str) -> np.ndarray:
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    rclpy.init()
    node = rclpy.create_node("save_scan")
    chunks = []

    def on_cloud(msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z", "intensity"], skip_nans=True
        )
        if len(pts):
            chunks.append(np.asarray(pts, dtype=np.float32))

    node.create_subscription(PointCloud2, topic, on_cloud, 10)
    deadline = time.monotonic() + seconds
    next_report = 0
    print(f"collecting {topic} for {seconds:g}s — keep the lidar still", flush=True)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            count = sum(len(chunk) for chunk in chunks)
            if count >= next_report:
                print(f"{count} points", flush=True)
                next_report = count + 50000
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not chunks:
        raise SystemExit(f"no points on {topic}; is the lidar launch still running?")
    return np.vstack(chunks)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seconds",
        type=float,
        default=15.0,
        help="how long to stack clouds (default 15)",
    )
    parser.add_argument("--topic", default="/unilidar/cloud")
    parser.add_argument(
        "--output",
        default=str(Path.home() / "scan.pcd"),
        help="PCD path (default ~/scan.pcd)",
    )
    args = parser.parse_args()
    if args.seconds <= 0:
        raise SystemExit("--seconds must be greater than 0")

    cloud = collect_clouds(args.seconds, args.topic)
    output = Path(args.output).expanduser()
    write_pcd(output, cloud)
    print(f"wrote {len(cloud)} points to {output}", flush=True)


if __name__ == "__main__":
    main()

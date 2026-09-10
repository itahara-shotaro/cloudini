#!/usr/bin/env python3
"""Synthetic PointCloud2 publisher for benchmarking the Cloudini ROS1 bridge.

Two generation modes, because they bracket the achievable compression ratio:

  lidar    - a rotating-LiDAR scan: rings x azimuths, ranges drawn from a smooth
             surface plus noise, points emitted in scan order. Neighbouring points
             are spatially correlated, which is what the delta/quantization stage
             is designed to exploit. Representative of real sensor data.
  uniform  - independent uniform points in a box. No spatial correlation at all,
             so this is the worst case for any delta-based codec.

Schema is x,y,z,intensity (FLOAT32) + ring (UINT16), tightly packed at 18 bytes
per point, matching what the bridge accepts (no row padding).
"""

import struct

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField

POINT_STEP = 18  # 4 floats + 1 uint16, tightly packed

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
]


def make_lidar_scan(rings, azimuths, rng, nan_fraction):
    """A ring x azimuth scan of a smoothly varying surface, in scan order."""
    ring_idx = np.repeat(np.arange(rings, dtype=np.float32), azimuths)
    az_idx = np.tile(np.arange(azimuths, dtype=np.float32), rings)

    azimuth = az_idx * (2.0 * np.pi / azimuths)
    elevation = np.deg2rad(-15.0 + 30.0 * ring_idx / max(rings - 1, 1))

    # Smooth pseudo-structure (walls / floor undulation) plus a little sensor noise.
    surface = 12.0 + 6.0 * np.sin(3.0 * azimuth) + 2.0 * np.cos(5.0 * azimuth + elevation)
    ranges = surface + rng.normal(0.0, 0.02, size=surface.shape)
    ranges = np.clip(ranges, 0.5, 100.0).astype(np.float32)

    horizontal = ranges * np.cos(elevation)
    x = (horizontal * np.cos(azimuth)).astype(np.float32)
    y = (horizontal * np.sin(azimuth)).astype(np.float32)
    z = (ranges * np.sin(elevation)).astype(np.float32)

    # Reflectivity as a 0..255 whole number stored in a float, as most drivers do.
    intensity = np.floor(rng.uniform(0.0, 255.0, size=x.shape)).astype(np.float32)
    ring = ring_idx.astype(np.uint16)

    if nan_fraction > 0.0:
        drop = rng.random(x.shape) < nan_fraction
        x[drop] = np.nan
        y[drop] = np.nan
        z[drop] = np.nan

    return x, y, z, intensity, ring


def make_uniform(count, rng):
    """Worst case: no spatial correlation between consecutive points."""
    x = rng.uniform(-50.0, 50.0, size=count).astype(np.float32)
    y = rng.uniform(-50.0, 50.0, size=count).astype(np.float32)
    z = rng.uniform(-5.0, 5.0, size=count).astype(np.float32)
    intensity = np.floor(rng.uniform(0.0, 255.0, size=count)).astype(np.float32)
    ring = rng.integers(0, 32, size=count).astype(np.uint16)
    return x, y, z, intensity, ring


def pack(x, y, z, intensity, ring):
    """Interleave the columns into the packed 18-byte point layout."""
    count = x.shape[0]
    buffer = np.zeros((count, POINT_STEP), dtype=np.uint8)
    floats = np.stack([x, y, z, intensity], axis=1).astype("<f4")
    buffer[:, 0:16] = floats.view(np.uint8).reshape(count, 16)
    buffer[:, 16:18] = ring.astype("<u2").view(np.uint8).reshape(count, 2)
    return buffer.tobytes()


def main():
    rospy.init_node("mock_cloud_publisher")

    topic = rospy.get_param("~topic", "/points")
    mode = rospy.get_param("~mode", "lidar")
    rate_hz = rospy.get_param("~rate", 10.0)
    rings = int(rospy.get_param("~rings", 32))
    azimuths = int(rospy.get_param("~azimuths", 1024))
    count = int(rospy.get_param("~count", rings * azimuths))
    nan_fraction = float(rospy.get_param("~nan_fraction", 0.0))
    frame_id = rospy.get_param("~frame_id", "lidar_link")
    seed = int(rospy.get_param("~seed", 0))

    if mode not in ("lidar", "uniform"):
        rospy.logfatal("~mode must be 'lidar' or 'uniform' (got '%s')", mode)
        return 1

    rng = np.random.default_rng(seed)
    pub = rospy.Publisher(topic, PointCloud2, queue_size=1)

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.fields = FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.is_dense = nan_fraction <= 0.0

    n = rings * azimuths if mode == "lidar" else count
    rospy.loginfo(
        "mock_cloud_publisher: mode=%s points=%d rate=%.1fHz -> %s (%.1f MB/s raw)",
        mode, n, rate_hz, topic, n * POINT_STEP * rate_hz / 1e6,
    )

    rate = rospy.Rate(rate_hz)
    seq = 0
    while not rospy.is_shutdown():
        if mode == "lidar":
            columns = make_lidar_scan(rings, azimuths, rng, nan_fraction)
        else:
            columns = make_uniform(count, rng)

        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        msg.width = columns[0].shape[0]
        msg.row_step = msg.width * msg.point_step
        msg.data = pack(*columns)

        pub.publish(msg)
        seq += 1
        rate.sleep()
    return 0


if __name__ == "__main__":
    main()

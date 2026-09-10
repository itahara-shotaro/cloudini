#!/usr/bin/env python3
"""Measures the Cloudini ROS1 bridge end to end.

Subscribes to all three topics (raw, compressed, decoded), measures on-the-wire
bandwidth for each, and verifies the decoded cloud against the raw one it came
from -- metadata, per-field error, and NaN placement.

Raw and decoded sizes are the true serialized message sizes, not just the data
array, so the ratio reported is what actually crosses the network.
"""

from io import BytesIO

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8MultiArray

FLOAT_NAMES = ("x", "y", "z", "intensity")

# The lossy encoders work in float32 and round twice: once on `value * (1/resolution)`
# before np.rint, and once on `quantized * resolution` when decoding. Each contributes up
# to |value| * 2^-24, so the achievable error bound is 0.5*resolution + 2*|value|*2^-24,
# not 0.5*resolution alone. Verified against a codec-free float32 round trip: over 2e6
# samples at |v| <= 20 and res=1e-3 the worst error is 5.01633e-4 and this bound is never
# violated.
FLOAT32_EPS = 2.0 ** -24


def serialized_size(msg):
    buf = BytesIO()
    msg.serialize(buf)
    return len(buf.getvalue())


def columns(msg):
    """Extract every declared field as a numpy column, honouring the point layout."""
    n = msg.width * msg.height
    if n == 0:
        return {}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    dtypes = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
    out = {}
    for field in msg.fields:
        dtype = np.dtype(dtypes[field.datatype])
        chunk = raw[:, field.offset:field.offset + dtype.itemsize]
        out[field.name] = np.ascontiguousarray(chunk).view(dtype).reshape(n)
    return out


class Stats:
    """Byte/message accumulator for one topic."""

    def __init__(self):
        self.messages = 0
        self.total_bytes = 0
        self.first = None
        self.last = None

    def add(self, size, now):
        self.messages += 1
        self.total_bytes += size
        if self.first is None:
            self.first = now
        self.last = now

    def duration(self):
        if self.first is None or self.last is None or self.messages < 2:
            return 0.0
        return self.last - self.first

    def mbps(self):
        d = self.duration()
        return (self.total_bytes / d / 1e6) if d > 0 else 0.0

    def hz(self):
        d = self.duration()
        return ((self.messages - 1) / d) if d > 0 else 0.0


class Benchmark:
    def __init__(self):
        self.raw = Stats()
        self.compressed = Stats()
        self.decoded = Stats()

        # Raw clouds held by timestamp so a decoded cloud can be checked against its source.
        self.pending = {}
        self.compared = 0
        self.mismatches = []
        self.max_error = {name: 0.0 for name in FLOAT_NAMES}
        self.max_excess = {name: -1.0 for name in FLOAT_NAMES}
        self.worst_at = {name: (0.0, 0.0) for name in FLOAT_NAMES}
        self.seq_offsets = set()
        self.exact_field_errors = 0
        self.nan_mismatches = 0
        self.latencies = []

        duration = rospy.get_param("~duration", 20.0)
        self.warmup = rospy.get_param("~warmup", 3.0)
        self.xyz_resolution = rospy.get_param("~xyz_resolution", 0.001)
        self.intensity_resolution = rospy.get_param("~intensity_resolution", 1.0)
        self.label = rospy.get_param("~label", "benchmark")

        raw_topic = rospy.get_param("~raw_topic", "/points")
        compressed_topic = rospy.get_param("~compressed_topic", "/points/cloudini")
        decoded_topic = rospy.get_param("~decoded_topic", "/points/decoded")

        self.start = None
        self.deadline = duration + self.warmup

        rospy.Subscriber(raw_topic, PointCloud2, self.on_raw, queue_size=50)
        rospy.Subscriber(compressed_topic, UInt8MultiArray, self.on_compressed, queue_size=50)
        rospy.Subscriber(decoded_topic, PointCloud2, self.on_decoded, queue_size=50)

    def counting(self):
        """True once the warmup window has elapsed (connections established, caches warm)."""
        now = rospy.Time.now().to_sec()
        if self.start is None:
            self.start = now
        return (now - self.start) >= self.warmup

    def finished(self):
        return self.start is not None and (rospy.Time.now().to_sec() - self.start) >= self.deadline

    def on_raw(self, msg):
        key = msg.header.stamp.to_nsec()
        self.pending[key] = (msg, rospy.Time.now().to_sec())
        if len(self.pending) > 200:
            for stale in sorted(self.pending)[:100]:
                del self.pending[stale]
        if self.counting():
            self.raw.add(serialized_size(msg), rospy.Time.now().to_sec())

    def on_compressed(self, msg):
        if self.counting():
            # 4-byte array length prefix + payload; layout is empty.
            self.compressed.add(len(msg.data) + 4, rospy.Time.now().to_sec())

    def on_decoded(self, msg):
        now = rospy.Time.now().to_sec()
        if self.counting():
            self.decoded.add(serialized_size(msg), now)

        entry = self.pending.pop(msg.header.stamp.to_nsec(), None)
        if entry is None or not self.counting():
            return
        original, sent_at = entry
        self.latencies.append((now - sent_at) * 1e3)
        self.compare(original, msg)

    def compare(self, original, decoded):
        self.compared += 1

        for attr in ("height", "width", "point_step", "row_step", "is_dense", "is_bigendian"):
            if getattr(original, attr) != getattr(decoded, attr):
                self.mismatches.append(
                    "%s: %r != %r" % (attr, getattr(original, attr), getattr(decoded, attr)))
        if original.header.stamp != decoded.header.stamp:
            self.mismatches.append("header.stamp changed")
        # header.seq is deliberately NOT compared: every ROS1 publisher stamps its own
        # sequence number at publish time (roscpp Publication::incrementSequence), so any
        # relay node renumbers it. The bridge does transport the original value inside the
        # envelope -- see the RoundTrip.PreservesAllMetadata unit test, which checks the
        # library directly without a publisher in the path.
        self.seq_offsets.add(int(decoded.header.seq) - int(original.header.seq))
        if original.header.frame_id != decoded.header.frame_id:
            self.mismatches.append("frame_id changed")
        if [(f.name, f.offset, f.datatype, f.count) for f in original.fields] != \
           [(f.name, f.offset, f.datatype, f.count) for f in decoded.fields]:
            self.mismatches.append("field schema changed")
            return

        before = columns(original)
        after = columns(decoded)
        for name, source in before.items():
            result = after[name]
            if name in FLOAT_NAMES:
                finite = np.isfinite(source)
                if not np.array_equal(np.isnan(source), np.isnan(result)):
                    self.nan_mismatches += 1
                if finite.any():
                    a = source[finite].astype(np.float64)
                    b = result[finite].astype(np.float64)
                    error = np.abs(a - b)
                    self.max_error[name] = max(self.max_error[name], float(error.max()))
                    # The codec's contract is 0.5 * resolution on the quantized value; the
                    # reconstruction q * resolution is then rounded to the nearest float32,
                    # which adds up to |value| * 2^-24. Bound each element by both terms.
                    resolution = (self.intensity_resolution if name == "intensity"
                                  else self.xyz_resolution)
                    bound = 0.5 * resolution + 2.0 * np.abs(a) * FLOAT32_EPS + 1e-9
                    self.max_excess[name] = max(
                        self.max_excess[name], float(np.max(error - bound)))
                    if error.max() >= self.worst_at[name][0]:
                        self.worst_at[name] = (float(error.max()), float(np.abs(a[np.argmax(error)])))
            else:
                # Not in the lossy whitelist: must be bit-exact.
                if not np.array_equal(source, result):
                    self.exact_field_errors += 1

    def report(self):
        print("")
        print("=" * 74)
        print("  Cloudini ROS1 bridge benchmark: %s" % self.label)
        print("=" * 74)
        print("  %-12s %8s %9s %11s %12s" % ("topic", "msgs", "Hz", "MB/s", "bytes/msg"))
        for name, stats in (("raw", self.raw), ("compressed", self.compressed), ("decoded", self.decoded)):
            per_msg = stats.total_bytes / stats.messages if stats.messages else 0
            print("  %-12s %8d %9.2f %11.3f %12.0f" % (name, stats.messages, stats.hz(), stats.mbps(), per_msg))

        if self.raw.total_bytes and self.compressed.total_bytes:
            raw_per = self.raw.total_bytes / self.raw.messages
            comp_per = self.compressed.total_bytes / self.compressed.messages
            print("")
            print("  compression ratio : %.2f%% of raw  (%.1fx reduction)"
                  % (100.0 * comp_per / raw_per, raw_per / comp_per))
            print("  bandwidth saved   : %.3f MB/s" % (self.raw.mbps() - self.compressed.mbps()))

        if self.latencies:
            lat = np.array(self.latencies)
            print("  round-trip latency: mean %.2f ms, p95 %.2f ms, max %.2f ms  (both nodes, same host)"
                  % (lat.mean(), np.percentile(lat, 95), lat.max()))

        print("")
        print("  fidelity over %d matched cloud pairs" % self.compared)
        for name in FLOAT_NAMES:
            resolution = self.intensity_resolution if name == "intensity" else self.xyz_resolution
            within = self.max_excess[name] <= 0.0
            print("    %-10s max |error| %.7f at |v|=%7.2f   bound %.7f (0.5*res + 2|v|*2^-24)  %s"
                  % (name, self.max_error[name], self.worst_at[name][1],
                     0.5 * resolution + 2.0 * self.worst_at[name][1] * FLOAT32_EPS,
                     "ok" if within else "OVER BOUND"))
        print("    lossless fields exact          : %s"
              % ("yes" if self.exact_field_errors == 0 else "NO (%d)" % self.exact_field_errors))
        print("    NaN placement preserved        : %s"
              % ("yes" if self.nan_mismatches == 0 else "NO (%d)" % self.nan_mismatches))
        print("    metadata preserved             : %s"
              % ("yes" if not self.mismatches else "NO"))
        if self.seq_offsets:
            print("    header.seq renumbered by        : %s (expected: every ROS1 publisher"
                  % sorted(self.seq_offsets))
            print("                                      stamps its own sequence number)")
        print("    delivery                       : %d decoded / %d raw in the measured window"
              % (self.decoded.messages, self.raw.messages))

        ok = (self.compared > 0 and not self.mismatches and self.exact_field_errors == 0
              and self.nan_mismatches == 0
              and all(self.max_excess[n] <= 0.0 for n in FLOAT_NAMES))
        if self.mismatches:
            print("")
            print("  METADATA MISMATCHES:")
            for text in sorted(set(self.mismatches))[:10]:
                print("    - %s" % text)
        print("")
        print("  RESULT: %s" % ("PASS" if ok else "FAIL"))
        print("=" * 74)
        return ok


def main():
    rospy.init_node("bridge_benchmark")
    bench = Benchmark()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and not bench.finished():
        rate.sleep()
    bench.report()


if __name__ == "__main__":
    main()

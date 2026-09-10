# cloudini_ros1

ROS1 / Noetic network bridge for `sensor_msgs/PointCloud2`. A compressor node encodes clouds
with the core [Cloudini](../cloudini_lib) codec and publishes them as
`std_msgs/UInt8MultiArray`; a decompressor node on another machine reconstructs a usable
`PointCloud2`. Only the compressed topic crosses the network.

No custom message package is needed, and the ROS2 package (`cloudini_ros`) is untouched.

```
 sensor host                          network                    base host
┌──────────────┐   PointCloud2   ┌──────────────────┐        ┌────────────────┐   PointCloud2
│ LiDAR driver │ ──────────────► │ compress_node    │ ─TCP─► │ decompress_node│ ─────────────►
└──────────────┘    /points      └──────────────────┘        └────────────────┘  /points/decoded
                                   /points/cloudini
                                   (UInt8MultiArray)
```

## Build

The core library is pulled in with `add_subdirectory(../cloudini_lib)` and built as a static
archive with tools, benchmarks and its own test suite disabled, using the system
`liblz4-dev` / `libzstd-dev` rather than downloading them.

`cloudini_lib` itself declares `<build_type>ament_cmake</build_type>` and cannot be built by
catkin, so it carries a `CATKIN_IGNORE` marker. If you prefer not to have that file in your
source tree, delete it and skiplist the package instead:

```bash
catkin config --skiplist cloudini_lib
```

Then:

```bash
cd <your catkin workspace>
catkin build cloudini_ros1
source devel/setup.bash
```

Requirements: ROS Noetic, CMake ≥ 3.16, a C++20-capable compiler (GCC 9.4 is enough —
CMake selects `-std=gnu++2a`), `liblz4-dev`, `libzstd-dev`.

## Running

```bash
roslaunch cloudini_ros1 cloudini_network_bridge.launch
```

In a real two-machine deployment, run only the compressor on the sensor host and only the
decompressor on the base host, both pointed at the same master:

```bash
# both machines
export ROS_MASTER_URI=http://<master-host>:11311
export ROS_IP=<this machine's IP on the shared network>

# sensor host
rosrun cloudini_ros1 cloudini_compress_node _input_topic:=/livox/lidar _output_topic:=/livox/lidar/cloudini

# base host
rosrun cloudini_ros1 cloudini_decompress_node _input_topic:=/livox/lidar/cloudini _output_topic:=/livox/lidar/decoded
```

Check the gain with:

```bash
rostopic bw /livox/lidar          # raw
rostopic bw /livox/lidar/cloudini # compressed
rostopic hz /livox/lidar/decoded
```

## Parameters

### `cloudini_compress_node`

| Parameter | Default | Meaning |
|---|---|---|
| `~input_topic` | `/points` | `sensor_msgs/PointCloud2` to compress |
| `~output_topic` | `/points/cloudini` | `std_msgs/UInt8MultiArray` output |
| `~queue_size` | `1` | Favors fresh clouds over backlog |
| `~xyz_resolution` | `0.001` | Quantization step for `x`/`y`/`z`, in metres |
| `~intensity_resolution` | `1.0` | Quantization step for `intensity`; `0` leaves it lossless |
| `~field_resolutions` | *(unset)* | Dict of explicit per-field overrides, e.g. `{x: 0.001, intensity: 0.01}` |
| `~compression` | `zstd` | `zstd`, `lz4` or `none` (case-insensitive) |
| `~encoding` | `lossy` | `lossy`, `lossless` or `none` |
| `~use_threads` | `true` | Run stage-2 compression on a worker thread |
| `~check_value_range` | `true` | Reject clouds holding non-finite or unquantizable values |
| `~skip_when_no_subscribers` | `true` | Do no work when nobody is listening |
| `~tcp_no_delay` | `true` | `TransportHints().tcpNoDelay()` on the input subscription |
| `~max_input_bytes` | `67108864` | Reject larger input clouds |
| `~max_compressed_bytes` | `33554432` | Payload size cap |
| `~log_period` | `10.0` | Seconds between throughput log lines |

### `cloudini_decompress_node`

| Parameter | Default | Meaning |
|---|---|---|
| `~input_topic` | `/points/cloudini` | Compressed input |
| `~output_topic` | `/points/decoded` | Reconstructed `sensor_msgs/PointCloud2` |
| `~queue_size` | `1` | |
| `~tcp_no_delay` | `true` | |
| `~max_compressed_bytes` | `33554432` | Reject larger payloads before decoding |
| `~max_decoded_bytes` | `67108864` | Bound on the allocation made for the decoded cloud |
| `~max_metadata_bytes` | `65536` | Bound on the transported ROS metadata |
| `~log_period` | `10.0` | |

Bad parameters are fatal at startup, not per-message. Both nodes refuse to start if
`~input_topic` and `~output_topic` resolve to the same name.

## What is lossy and what is not

Per field, in order:

1. An explicit `~field_resolutions` entry wins. It must be a float field and the value must
   be `> 0` — zero would mean "drop this field" in Cloudini, which would change the output
   schema, and field dropping is not supported here.
2. `x`, `y`, `z` (FLOAT32 or FLOAT64) → `~xyz_resolution`.
3. `intensity` (FLOAT32 or FLOAT64) → `~intensity_resolution`, unless it is `0`.
4. Everything else is **lossless**: packed `rgb`/`rgba` bit patterns, `ring`, `tag`,
   `curvature`, per-point timestamps, normals, and every integer field.

Quantized fields are reconstructed within `0.5 × resolution + 2 × |value| × 2⁻²⁴`. The
second term is float32 representation error, not codec loss: the encoders round once on
`value × (1/resolution)` and again on `quantized × resolution`, each contributing up to
`|value| × 2⁻²⁴`. At 1 mm resolution and 20 m range that is ~0.502 mm rather than exactly
0.5 mm. NaN survives a round trip exactly, so sparse/organized clouds with invalid returns
are fine.

`~intensity_resolution` defaults to `1.0` because LiDAR drivers typically store reflectivity
as a 0–255 float, where whole-number steps are effectively lossless and the bandwidth saving
is significant. **If your driver packs something else into `intensity`** — a timestamp, a
sub-unit reflectance — set `~intensity_resolution: 0`.

## Supported inputs

Rejected deterministically, with a throttled warning and no crash:

- big-endian clouds (`is_bigendian == true`)
- array-valued fields (`count != 1`)
- datatypes outside `PointField` 1–8
- `point_step == 0`, fields extending past `point_step`, overlapping fields
- padded rows (`row_step != width * point_step`)
- `data.size()` disagreeing with `height * row_step`
- clouds over `~max_input_bytes`
- values in a lossy field that are ±inf or so large they would saturate the codec's int32
  quantization lanes (disable with `~check_value_range: false`)

Undeclared padding bytes inside a point are **not** preserved. The decoder zero-fills them,
so they are deterministic but not necessarily identical to the sender's.

## Wire format

One `std_msgs/UInt8MultiArray` per cloud, `layout` left empty, `data` treated as opaque
bytes. Envelope version 1, all integers explicitly little-endian:

```
off  size  field
  0     8  magic "CLDROS1\0"
  8     2  envelope version (currently 1)
 10     2  flags, reserved, must be zero
 12     4  metadata length M
 16     4  Cloudini payload length C
 20     M  ROS1-serialized PointCloud2 metadata skeleton (data[] empty)
20+M    C  self-contained Cloudini payload
```

The received byte count must equal `20 + M + C` exactly. The skeleton carries what the
Cloudini header cannot: `header.seq`/`stamp`/`frame_id`, `is_dense`, and the original field
names.

Note on `header.seq`: the bridge transports the original value faithfully, but ROS1
publishers stamp their own sequence number at publish time
(`ros::Publication::incrementSequence`), so the decompressor's publisher renumbers it on the
way out. That is true of any ROS1 relay node, not specific to this bridge — the measured
offset is a constant equal to however many clouds were published before the bridge finished
connecting. **Do not rely on `header.seq` surviving the bridge; use `header.stamp`**, which
is preserved exactly. The receiver validates the skeleton and the decoded Cloudini header independently and
rejects any disagreement in dimensions, field count, name, offset or datatype before it
allocates anything.

There is no CRC in version 1 — TCPROS is reliable and ordered, and corruption surfaces
through the strict length checks and the codec's own error paths. A later version can add
one behind a flag without changing version 1 parsing.

## Compatibility

- **Both hosts must run the same `cloudini` checkout.** The envelope version is independent
  of the Cloudini wire version, but the payload is not: the core currently writes V5 and
  reads V2–V5, so a receiver on an older checkout will reject every message with an
  "Unsupported encoding version" warning. The bridge deliberately does not pin the codec
  version.
- **Little-endian hosts only.** The Cloudini payload is written in native byte order, and
  big-endian input clouds are rejected outright.
- Which internal codec path the core selects (V4 vs V5) depends on the encoding options and
  on whether a non-leading integer field is present, so compression ratios legitimately
  differ between topics with different schemas. Both ends derive this from the header, so it
  is always consistent.
- Running `catkin build --install` would also install `cloudini_lib`'s own headers and
  archive into the install space, since its `install()` rules come along with
  `add_subdirectory`. Harmless, and dormant while the profile has `install: false`.

## Tests

```bash
catkin run_tests cloudini_ros1 --no-deps
catkin_test_results build/cloudini_ros1
```

- `test_cloudini_ros1_wire` — envelope writer/parser, and the skeleton pre-scan against
  truncated, oversized and hostile input.
- `test_cloudini_ros1_validation` — the structural contract and the lossy value-range check.
- `test_cloudini_ros1_codec` — round trips for every ROS scalar datatype, organized and
  empty clouds, all compression modes, NaN, packed RGB, encoder-cache invalidation, plus
  tampered envelopes and per-byte corruption.
- `test_cloudini_ros1_roundtrip` — rostest: a live cloud through both nodes, and a check
  that garbage on the compressed topic does not take the bridge down.

## Benchmark

A self-contained harness — synthetic publisher, both bridge nodes, and a checker that
measures true serialized wire sizes and verifies every field — ships with the package:

```bash
roslaunch cloudini_ros1 mock_benchmark.launch                     # LiDAR-like scan
roslaunch cloudini_ros1 mock_benchmark.launch mode:=uniform       # worst case, no spatial structure
roslaunch cloudini_ros1 mock_benchmark.launch rings:=64 azimuths:=1800 rate:=20
```

`mode:=lidar` generates a rings × azimuths scan of a smooth surface in scan order, which has
the spatial correlation real sensors produce. `mode:=uniform` generates independent random
points and is the floor of what any delta-based codec can achieve — quote it as the worst
case, never as the expected result.

Measured on one host (GCC 9.4 Release, ZSTD unless noted), 32 × 1024 = 32768 points/cloud,
`x,y,z,intensity` FLOAT32 + `ring` UINT16, 18 B/point, 3% NaN:

| Configuration | Raw | Compressed | Ratio | Decoded rate | Latency (mean) |
|---|---|---|---|---|---|
| lidar, 1 mm, 10 Hz | 5.94 MB/s | 1.55 MB/s | 26.1% (3.8×) | 10.00 Hz | 5.9 ms |
| lidar, 1 mm, 10 Hz, LZ4 | 5.94 MB/s | 1.88 MB/s | 31.6% (3.2×) | 10.00 Hz | 4.6 ms |
| lidar, 1 cm, 10 Hz | 5.94 MB/s | 1.11 MB/s | 18.6% (5.4×) | 10.00 Hz | 5.0 ms |
| lidar, 1 mm, intensity lossless | 5.94 MB/s | 1.94 MB/s | 32.7% (3.1×) | 10.00 Hz | 5.6 ms |
| **lidar, 1 mm, 64×1800 @ 20 Hz** | **41.6 MB/s** | **9.92 MB/s** | **23.8% (4.2×)** | **20.00 Hz** | **8.4 ms** |
| uniform, 1 mm, 10 Hz | 5.94 MB/s | 3.02 MB/s | 51.0% (2.0×) | 10.00 Hz | 7.0 ms |

Every run reported no dropped clouds, bit-exact `ring`, preserved NaN placement and metadata,
and `x`/`y`/`z`/`intensity` errors inside the bound above. Latency is compress + transport +
decompress on a single machine, so it is codec cost plus loopback, not a network figure.

Two things worth noting for tuning: quantizing `intensity` is worth about 20% of the
compressed size (26.1% vs 32.7%), and dropping geometry precision from 1 mm to 1 cm buys
another 30%. LZ4 costs ~5 points of ratio and saves about 1 ms per cloud.

## Design notes

- `PointcloudEncoder` owns scratch buffers and a compression worker thread and is not safe
  for concurrent callbacks, so both nodes use a single-threaded `ros::spin()`. Do not switch
  to `AsyncSpinner` without giving each callback queue its own encoder.
- The encoder is cached and rebuilt whenever the full encoding configuration changes. Note
  that `width` is part of that configuration (it is baked into the pre-serialized header), so
  unorganized clouds with a varying point count rebuild every frame — tens of microseconds.
  Set `~use_threads: false` to drop the per-rebuild thread spawn entirely.
- The receiver never uses the codec's convenience `std::vector` decode overload: its internal
  size computation multiplies `uint32`s unchecked and is not a validation boundary for
  untrusted input.

### Not implemented

Nodelets (which would remove the intra-host serialization copy and are the biggest remaining
win for same-machine use), CRC32 in the envelope, UDPROS, multi-threaded callbacks, and
field dropping.

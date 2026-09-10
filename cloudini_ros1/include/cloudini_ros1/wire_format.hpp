#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cloudini_ros1 {

/**
 * Envelope version 1 layout (all integers explicitly little-endian):
 *
 *   off  size  field
 *     0     8  magic: "CLDROS1\0"
 *     8     2  envelope version (uint16)
 *    10     2  flags (uint16), reserved, must be zero
 *    12     4  metadata byte length M (uint32)
 *    16     4  Cloudini payload byte length C (uint32)
 *    20     M  ROS1-serialized PointCloud2 metadata skeleton (data[] empty)
 *  20+M     C  self-contained Cloudini payload (header + chunks)
 *
 * The received byte count must equal 20 + M + C exactly; trailing bytes are rejected.
 * The envelope version is deliberately independent of Cloudini::EncodingInfo::version.
 */
constexpr uint16_t kEnvelopeVersion = 1;
constexpr size_t kEnvelopeHeaderLen = 20;
constexpr size_t kEnvelopeMagicLen = 8;
constexpr char kEnvelopeMagic[kEnvelopeMagicLen] = {'C', 'L', 'D', 'R', 'O', 'S', '1', '\0'};

/// Bounds applied to untrusted network input before any allocation.
struct WireLimits {
  size_t max_total_bytes = 64u << 20;
  size_t max_metadata_bytes = 64u << 10;
  size_t max_payload_bytes = 32u << 20;
  size_t max_fields = 64;
  size_t max_name_bytes = 256;
};

/// Non-owning view into a parsed envelope. Valid only while the source buffer is alive.
struct EnvelopeView {
  uint16_t version = 0;
  uint16_t flags = 0;
  const uint8_t* metadata = nullptr;
  size_t metadata_len = 0;
  const uint8_t* payload = nullptr;
  size_t payload_len = 0;
};

// -------- explicit little-endian scalar helpers (no packed structs, no type punning) -----

inline void putU16LE(std::vector<uint8_t>& out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v & 0xFFu));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
}

inline void putU32LE(std::vector<uint8_t>& out, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
  }
}

/// Caller must guarantee 2 readable bytes at p.
inline uint16_t getU16LE(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

/// Caller must guarantee 4 readable bytes at p.
inline uint32_t getU32LE(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

// ---------------------------------- envelope --------------------------------------------

/// Build a version-1 envelope. Throws std::length_error if either section exceeds uint32.
void writeEnvelope(
    const std::vector<uint8_t>& metadata, const std::vector<uint8_t>& payload, std::vector<uint8_t>& out);

/// Parse and bounds-check an envelope. Never throws; returns false with a reason in `error`.
bool parseEnvelope(
    const uint8_t* data, size_t size, const WireLimits& limits, EnvelopeView& out, std::string& error);

// ------------------------------ metadata skeleton ---------------------------------------

/// Serialize the PointCloud2 metadata (everything but `data`) with ROS1 serialization.
void serializeSkeleton(const sensor_msgs::PointCloud2& input, std::vector<uint8_t>& out);

/**
 * Bounded pre-scan of skeleton bytes, mirroring the ROS1 PointCloud2 layout.
 *
 * This MUST run before ros::serialization::deserialize. The ROS1 array reader does
 * `v.resize(len)` before any bounds check (ros/serialization.h), so a hostile 24-byte
 * skeleton declaring fields.size() == 0xFFFFFFFF would attempt a huge allocation before
 * the stream overrun is ever detected. Capping the declared metadata length does not help.
 */
bool preflightSkeletonBytes(const uint8_t* data, size_t size, const WireLimits& limits, std::string& error);

/// Deserialize skeleton bytes that have already passed preflightSkeletonBytes(). Never throws.
bool deserializeSkeleton(const uint8_t* data, size_t size, sensor_msgs::PointCloud2& out, std::string& error);

}  // namespace cloudini_ros1

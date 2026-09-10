#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8MultiArray.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_ros1/validation.hpp"
#include "cloudini_ros1/wire_format.hpp"

namespace cloudini_ros1 {

/// Bridge configuration. Resolutions are doubles here because ROS parameters and
/// XmlRpcValue are doubles; they are narrowed to float exactly once, at EncodingInfo
/// construction, via toResolutionFloat().
struct CodecOptions {
  Cloudini::CompressionOption compression = Cloudini::CompressionOption::ZSTD;
  Cloudini::EncodingOptions encoding = Cloudini::EncodingOptions::LOSSY;

  double xyz_resolution = 0.001;        // metres
  double intensity_resolution = 1.0;    // 0 => leave `intensity` lossless
  std::map<std::string, double> field_resolutions;  // explicit per-field override

  bool use_threads = true;
  bool check_value_range = true;

  size_t max_input_bytes = 64u << 20;
  size_t max_decoded_bytes = 64u << 20;
  WireLimits wire;

  StructureLimits structureLimits() const {
    StructureLimits l;
    l.max_data_bytes = max_input_bytes;
    l.max_fields = wire.max_fields;
    l.max_name_bytes = wire.max_name_bytes;
    return l;
  }
};

/// Narrow a resolution to the float that Cloudini::PointField stores, rejecting values that
/// would underflow to zero (FieldEncoderFloat_Lossy throws on resolution <= 0).
bool toResolutionFloat(double in, float& out);

/// Apply the per-field lossy policy in place. See README for the rule order.
bool applyFieldResolutionPolicy(
    const CodecOptions& options, std::vector<Cloudini::PointField>& fields, std::string& error);

/**
 * Holds one PointcloudEncoder, rebuilt whenever the encoding configuration changes.
 *
 * Cloudini::EncodingInfo::operator== ignores encoding_config, use_threads and version, all
 * of which change the emitted bytes or the worker-thread topology, so it is NOT usable as a
 * cache key. sameConfig() compares every member explicitly.
 *
 * PointcloudEncoder owns a thread, a mutex and two condition variables: it is neither
 * copyable nor movable, hence the unique_ptr, and it is not safe for concurrent callbacks.
 */
class EncoderCache {
 public:
  Cloudini::PointcloudEncoder& get(const Cloudini::EncodingInfo& info);

  size_t rebuilds() const {
    return rebuilds_;
  }

  static bool sameConfig(const Cloudini::EncodingInfo& a, const Cloudini::EncodingInfo& b);

 private:
  std::unique_ptr<Cloudini::PointcloudEncoder> encoder_;
  size_t rebuilds_ = 0;
};

/// Build the EncodingInfo for a validated cloud, including the resolution policy.
bool buildEncodingInfo(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, Cloudini::EncodingInfo& out,
    std::string& error);

/**
 * Validate, compress and wrap a cloud into a version-1 envelope.
 * `payload_scratch` and `meta_scratch` are caller-owned so steady-state allocation is zero.
 */
bool encodePointCloud(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, EncoderCache& cache,
    std::vector<uint8_t>& payload_scratch, std::vector<uint8_t>& meta_scratch, std_msgs::UInt8MultiArray& out,
    std::string& error);

/// Parse, cross-check and decompress an envelope received from the network. Never throws.
bool decodePointCloud(
    const std_msgs::UInt8MultiArray& input, const CodecOptions& limits, Cloudini::PointcloudDecoder& decoder,
    sensor_msgs::PointCloud2& out, std::string& error);

}  // namespace cloudini_ros1

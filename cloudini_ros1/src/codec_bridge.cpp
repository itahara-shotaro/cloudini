#include "cloudini_ros1/codec_bridge.hpp"

#include <cmath>
#include <exception>

namespace cloudini_ros1 {

bool toResolutionFloat(double in, float& out) {
  if (!std::isfinite(in) || in <= 0.0) {
    return false;
  }
  const float narrowed = static_cast<float>(in);
  // A denormal-small parameter can narrow to 0.0f, and FieldEncoderFloat_Lossy throws on a
  // non-positive resolution deep inside the codec. Reject it here instead.
  if (!std::isfinite(narrowed) || narrowed <= 0.0f) {
    return false;
  }
  out = narrowed;
  return true;
}

namespace {

bool isFloatType(Cloudini::FieldType type) {
  return type == Cloudini::FieldType::FLOAT32 || type == Cloudini::FieldType::FLOAT64;
}

bool isValidEncodingOption(Cloudini::EncodingOptions opt) {
  return opt == Cloudini::EncodingOptions::NONE || opt == Cloudini::EncodingOptions::LOSSY ||
         opt == Cloudini::EncodingOptions::LOSSLESS;
}

bool isValidCompressionOption(Cloudini::CompressionOption opt) {
  return opt == Cloudini::CompressionOption::NONE || opt == Cloudini::CompressionOption::LZ4 ||
         opt == Cloudini::CompressionOption::ZSTD;
}

}  // namespace

bool applyFieldResolutionPolicy(
    const CodecOptions& options, std::vector<Cloudini::PointField>& fields, std::string& error) {
  for (auto& field : fields) {
    field.resolution = std::nullopt;

    // 1. Explicit override wins. Zero is rejected: in Cloudini a zero resolution means
    //    "drop the field", which would change the output schema, and field dropping is not
    //    implemented end-to-end here.
    const auto it = options.field_resolutions.find(field.name);
    if (it != options.field_resolutions.end()) {
      if (!isFloatType(field.type)) {
        error = "field_resolutions names '" + field.name + "', which is not a float field";
        return false;
      }
      float resolution = 0.0f;
      if (!toResolutionFloat(it->second, resolution)) {
        error = "field_resolutions['" + field.name + "'] must be finite and > 0";
        return false;
      }
      field.resolution = resolution;
      continue;
    }

    // 2. Geometry.
    if ((field.name == "x" || field.name == "y" || field.name == "z") && isFloatType(field.type)) {
      float resolution = 0.0f;
      if (!toResolutionFloat(options.xyz_resolution, resolution)) {
        error = "xyz_resolution must be finite and > 0";
        return false;
      }
      field.resolution = resolution;
      continue;
    }

    // 3. Intensity, when enabled. Deviates from the original spec (xyz-only) deliberately:
    //    on LiDAR clouds intensity is a large share of the payload, and reflectivity stored
    //    as a 0..255 float is unaffected by a resolution of 1.0.
    if (field.name == "intensity" && isFloatType(field.type) && options.intensity_resolution > 0.0) {
      float resolution = 0.0f;
      if (!toResolutionFloat(options.intensity_resolution, resolution)) {
        error = "intensity_resolution must be finite and > 0 (or exactly 0 to disable)";
        return false;
      }
      field.resolution = resolution;
      continue;
    }

    // 4. Everything else stays lossless: rgb/rgba packed bit patterns, normals, curvature,
    //    per-point timestamps, and all integer fields.
  }
  return true;
}

bool buildEncodingInfo(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, Cloudini::EncodingInfo& out,
    std::string& error) {
  out = Cloudini::EncodingInfo{};
  out.width = input.width;
  out.height = input.height;
  out.point_step = input.point_step;
  out.encoding_opt = options.encoding;
  out.compression_opt = options.compression;
  out.use_threads = options.use_threads;
  // out.version deliberately left at Cloudini::kEncodingVersion.

  out.fields.reserve(input.fields.size());
  for (const auto& msg_field : input.fields) {
    Cloudini::PointField field;
    field.name = msg_field.name;
    field.offset = msg_field.offset;
    if (!toCloudiniFieldType(msg_field.datatype, field.type)) {
      error = "field '" + msg_field.name + "' has unsupported datatype " + std::to_string(msg_field.datatype);
      return false;
    }
    out.fields.push_back(std::move(field));
  }

  return applyFieldResolutionPolicy(options, out.fields, error);
}

bool EncoderCache::sameConfig(const Cloudini::EncodingInfo& a, const Cloudini::EncodingInfo& b) {
  // Deliberately not EncodingInfo::operator==, which ignores encoding_config, use_threads
  // and version -- all of which change the emitted bytes or the thread topology.
  if (a.width != b.width || a.height != b.height || a.point_step != b.point_step) {
    return false;
  }
  if (a.encoding_opt != b.encoding_opt || a.compression_opt != b.compression_opt) {
    return false;
  }
  if (a.encoding_config != b.encoding_config || a.use_threads != b.use_threads || a.version != b.version) {
    return false;
  }
  if (a.fields.size() != b.fields.size()) {
    return false;
  }
  for (size_t i = 0; i < a.fields.size(); ++i) {
    const auto& fa = a.fields[i];
    const auto& fb = b.fields[i];
    if (fa.name != fb.name || fa.offset != fb.offset || fa.type != fb.type) {
      return false;
    }
    if (fa.resolution.has_value() != fb.resolution.has_value()) {
      return false;
    }
    if (fa.resolution.has_value() && *fa.resolution != *fb.resolution) {
      return false;
    }
  }
  return true;
}

Cloudini::PointcloudEncoder& EncoderCache::get(const Cloudini::EncodingInfo& info) {
  if (!encoder_ || !sameConfig(encoder_->getEncodingInfo(), info)) {
    // Note: width is part of the key and is baked into the pre-serialized header, so
    // unorganized clouds with a varying point count rebuild every message. That costs a
    // header re-encode and a thread spawn (tens of microseconds); set use_threads=false to
    // drop the thread entirely.
    encoder_ = std::make_unique<Cloudini::PointcloudEncoder>(info);
    ++rebuilds_;
  }
  return *encoder_;
}

bool encodePointCloud(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, EncoderCache& cache,
    std::vector<uint8_t>& payload_scratch, std::vector<uint8_t>& meta_scratch, std_msgs::UInt8MultiArray& out,
    std::string& error) {
  if (!validateCloudStructure(input, options.structureLimits(), /*require_data_size=*/true, error)) {
    return false;
  }
  if (input.data.size() > options.max_input_bytes) {
    error = "input cloud exceeds max_input_bytes";
    return false;
  }

  Cloudini::EncodingInfo info;
  if (!buildEncodingInfo(input, options, info, error)) {
    return false;
  }
  if (options.check_value_range && !checkQuantizableRange(input, info.fields, error)) {
    return false;
  }

  try {
    auto& encoder = cache.get(info);
    encoder.encode(Cloudini::ConstBufferView(input.data.data(), input.data.size()), payload_scratch);
  } catch (const std::exception& e) {
    error = std::string("cloudini encode failed: ") + e.what();
    return false;
  }

  serializeSkeleton(input, meta_scratch);

  try {
    writeEnvelope(meta_scratch, payload_scratch, out.data);
  } catch (const std::exception& e) {
    error = std::string("envelope write failed: ") + e.what();
    return false;
  }
  out.layout = std_msgs::MultiArrayLayout{};
  return true;
}

namespace {

/// Cross-check the decoded Cloudini header against the transported ROS skeleton. Both come
/// from the network, so neither is trusted on its own.
bool crossCheck(const Cloudini::EncodingInfo& info, const sensor_msgs::PointCloud2& skeleton, std::string& error) {
  if (info.width != skeleton.width || info.height != skeleton.height || info.point_step != skeleton.point_step) {
    error = "cloudini header dimensions disagree with the ROS skeleton";
    return false;
  }
  if (!isValidEncodingOption(info.encoding_opt) || !isValidCompressionOption(info.compression_opt)) {
    // DecodeHeader blind-casts these bytes for binary headers, so validate them explicitly.
    error = "cloudini header declares an out-of-range encoding/compression option";
    return false;
  }
  if (info.fields.size() != skeleton.fields.size()) {
    error = "cloudini header field count disagrees with the ROS skeleton";
    return false;
  }

  for (size_t i = 0; i < info.fields.size(); ++i) {
    const auto& cf = info.fields[i];
    const auto& sf = skeleton.fields[i];
    const std::string where = "field index " + std::to_string(i);

    if (cf.name != sf.name) {
      error = where + " name disagrees between header and skeleton";
      return false;
    }
    if (cf.offset == Cloudini::kDecodeButSkipStore) {
      error = where + " is marked skip-store; dropped fields are not supported";
      return false;
    }
    if (cf.offset != sf.offset) {
      error = where + " offset disagrees between header and skeleton";
      return false;
    }
    Cloudini::FieldType skeleton_type;
    if (!toCloudiniFieldType(sf.datatype, skeleton_type) || skeleton_type != cf.type) {
      error = where + " datatype disagrees between header and skeleton";
      return false;
    }
    if (cf.resolution.has_value() && (!std::isfinite(*cf.resolution) || *cf.resolution <= 0.0f)) {
      error = where + " declares a non-positive resolution";
      return false;
    }
  }
  return true;
}

}  // namespace

bool decodePointCloud(
    const std_msgs::UInt8MultiArray& input, const CodecOptions& limits, Cloudini::PointcloudDecoder& decoder,
    sensor_msgs::PointCloud2& out, std::string& error) {
  EnvelopeView envelope;
  if (!parseEnvelope(input.data.data(), input.data.size(), limits.wire, envelope, error)) {
    return false;
  }
  if (!preflightSkeletonBytes(envelope.metadata, envelope.metadata_len, limits.wire, error)) {
    return false;
  }

  sensor_msgs::PointCloud2 skeleton;
  if (!deserializeSkeleton(envelope.metadata, envelope.metadata_len, skeleton, error)) {
    return false;
  }

  StructureLimits structure = limits.structureLimits();
  structure.max_data_bytes = limits.max_decoded_bytes;
  if (!validateCloudStructure(skeleton, structure, /*require_data_size=*/false, error)) {
    return false;
  }

  // DecodeHeader advances this view past the header, leaving it at the first chunk --
  // exactly what PointcloudDecoder::decode requires (it throws if it still sees the magic).
  Cloudini::ConstBufferView payload(envelope.payload, envelope.payload_len);
  Cloudini::EncodingInfo info;
  try {
    info = Cloudini::DecodeHeader(payload);
  } catch (const std::exception& e) {
    error = std::string("cloudini header decode failed: ") + e.what();
    return false;
  }

  if (!crossCheck(info, skeleton, error)) {
    return false;
  }

  size_t total = 0;
  if (!checkedMul(static_cast<size_t>(skeleton.width), static_cast<size_t>(skeleton.height), total) ||
      !checkedMul(total, static_cast<size_t>(skeleton.point_step), total)) {
    error = "decoded size computation overflows";
    return false;
  }
  if (total > limits.max_decoded_bytes) {
    error = "decoded cloud would be " + std::to_string(total) + " bytes, over max_decoded_bytes";
    return false;
  }

  sensor_msgs::PointCloud2 result;
  result.header = skeleton.header;
  result.height = skeleton.height;
  result.width = skeleton.width;
  result.fields = skeleton.fields;
  result.is_dense = skeleton.is_dense;
  result.point_step = skeleton.point_step;
  result.is_bigendian = false;
  result.row_step = skeleton.width * skeleton.point_step;  // canonical; validated above
  // Zero-initialized so undeclared padding bytes are deterministic: Cloudini only restores
  // the bytes covered by declared scalar fields.
  result.data.assign(total, 0);

  try {
    // Never the std::vector overload: its internal resize multiplies uint32s unchecked and
    // is not the validation boundary for untrusted input.
    Cloudini::BufferView view(result.data.data(), result.data.size());
    decoder.decode(info, payload, view);
  } catch (const std::exception& e) {
    error = std::string("cloudini decode failed: ") + e.what();
    return false;
  }

  out = std::move(result);
  return true;
}

}  // namespace cloudini_ros1

#include "cloudini_ros1/wire_format.hpp"

#include <ros/exception.h>
#include <ros/serialization.h>

#include <cstring>
#include <stdexcept>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_ros1/validation.hpp"

namespace cloudini_ros1 {

void writeEnvelope(
    const std::vector<uint8_t>& metadata, const std::vector<uint8_t>& payload, std::vector<uint8_t>& out) {
  constexpr size_t kU32Max = 0xFFFFFFFFull;
  if (metadata.size() > kU32Max || payload.size() > kU32Max) {
    throw std::length_error("cloudini_ros1: envelope section exceeds uint32");
  }

  out.clear();
  out.reserve(kEnvelopeHeaderLen + metadata.size() + payload.size());
  out.insert(out.end(), kEnvelopeMagic, kEnvelopeMagic + kEnvelopeMagicLen);
  putU16LE(out, kEnvelopeVersion);
  putU16LE(out, 0);  // flags
  putU32LE(out, static_cast<uint32_t>(metadata.size()));
  putU32LE(out, static_cast<uint32_t>(payload.size()));
  out.insert(out.end(), metadata.begin(), metadata.end());
  out.insert(out.end(), payload.begin(), payload.end());
}

bool parseEnvelope(
    const uint8_t* data, size_t size, const WireLimits& limits, EnvelopeView& out, std::string& error) {
  out = EnvelopeView{};

  if (data == nullptr) {
    error = "null buffer";
    return false;
  }
  if (size > limits.max_total_bytes) {
    error = "message exceeds max_total_bytes";
    return false;
  }
  if (size < kEnvelopeHeaderLen) {
    error = "message shorter than envelope header";
    return false;
  }
  if (std::memcmp(data, kEnvelopeMagic, kEnvelopeMagicLen) != 0) {
    error = "bad envelope magic";
    return false;
  }

  const uint16_t version = getU16LE(data + 8);
  if (version != kEnvelopeVersion) {
    error = "unsupported envelope version " + std::to_string(version);
    return false;
  }
  const uint16_t flags = getU16LE(data + 10);
  if (flags != 0) {
    error = "reserved envelope flags are not zero";
    return false;
  }

  const size_t metadata_len = getU32LE(data + 12);
  const size_t payload_len = getU32LE(data + 16);

  size_t declared = 0;
  if (!checkedAdd(kEnvelopeHeaderLen, metadata_len, declared) || !checkedAdd(declared, payload_len, declared)) {
    error = "declared envelope lengths overflow";
    return false;
  }
  if (declared != size) {
    error = "declared envelope length " + std::to_string(declared) + " != received " + std::to_string(size);
    return false;
  }
  if (metadata_len > limits.max_metadata_bytes) {
    error = "metadata exceeds max_metadata_bytes";
    return false;
  }
  if (payload_len > limits.max_payload_bytes) {
    error = "payload exceeds max_payload_bytes";
    return false;
  }
  // A Cloudini payload cannot be shorter than its magic + 2 version digits.
  if (payload_len < static_cast<size_t>(Cloudini::kMagicHeaderLength) + 2) {
    error = "payload too short to hold a Cloudini header";
    return false;
  }

  out.version = version;
  out.flags = flags;
  out.metadata = data + kEnvelopeHeaderLen;
  out.metadata_len = metadata_len;
  out.payload = data + kEnvelopeHeaderLen + metadata_len;
  out.payload_len = payload_len;
  return true;
}

void serializeSkeleton(const sensor_msgs::PointCloud2& input, std::vector<uint8_t>& out) {
  // Built field-by-field rather than copy-then-clear: copying `input` would duplicate the
  // whole point buffer just to throw it away.
  sensor_msgs::PointCloud2 skeleton;
  skeleton.header = input.header;
  skeleton.height = input.height;
  skeleton.width = input.width;
  skeleton.fields = input.fields;
  skeleton.is_bigendian = input.is_bigendian;
  skeleton.point_step = input.point_step;
  skeleton.row_step = input.row_step;
  skeleton.is_dense = input.is_dense;
  // skeleton.data stays empty by construction.

  const uint32_t len = ros::serialization::serializationLength(skeleton);
  out.resize(len);
  ros::serialization::OStream stream(out.data(), len);
  ros::serialization::serialize(stream, skeleton);
}

namespace {

/// Bounded cursor over the skeleton bytes. Every read is checked before it happens.
class ByteCursor {
 public:
  ByteCursor(const uint8_t* data, size_t size) : data_(data), size_(size) {}

  bool readU8(uint8_t& v) {
    if (remaining() < 1) {
      return false;
    }
    v = data_[pos_++];
    return true;
  }

  bool readU32(uint32_t& v) {
    if (remaining() < 4) {
      return false;
    }
    v = getU32LE(data_ + pos_);
    pos_ += 4;
    return true;
  }

  /// Reads a length-prefixed string without materializing it.
  bool skipString(size_t max_len) {
    uint32_t len = 0;
    if (!readU32(len)) {
      return false;
    }
    if (len > max_len || len > remaining()) {
      return false;
    }
    pos_ += len;
    return true;
  }

  bool skip(size_t n) {
    if (remaining() < n) {
      return false;
    }
    pos_ += n;
    return true;
  }

  size_t remaining() const {
    return size_ - pos_;
  }

  bool atEnd() const {
    return pos_ == size_;
  }

 private:
  const uint8_t* data_;
  size_t size_;
  size_t pos_ = 0;
};

}  // namespace

bool preflightSkeletonBytes(const uint8_t* data, size_t size, const WireLimits& limits, std::string& error) {
  if (data == nullptr) {
    error = "null skeleton buffer";
    return false;
  }

  ByteCursor cur(data, size);
  uint32_t u32 = 0;

  // std_msgs/Header: seq, stamp.sec, stamp.nsec, frame_id
  if (!cur.readU32(u32) || !cur.readU32(u32) || !cur.readU32(u32)) {
    error = "truncated skeleton header";
    return false;
  }
  if (!cur.skipString(limits.max_name_bytes)) {
    error = "bad skeleton frame_id";
    return false;
  }

  // height, width
  if (!cur.readU32(u32) || !cur.readU32(u32)) {
    error = "truncated skeleton dimensions";
    return false;
  }

  // fields[]
  uint32_t field_count = 0;
  if (!cur.readU32(field_count)) {
    error = "truncated skeleton field count";
    return false;
  }
  if (field_count > limits.max_fields) {
    error = "skeleton declares " + std::to_string(field_count) + " fields, over max_fields";
    return false;
  }
  for (uint32_t i = 0; i < field_count; ++i) {
    uint8_t u8 = 0;
    // name, offset(u32), datatype(u8), count(u32)
    if (!cur.skipString(limits.max_name_bytes) || !cur.readU32(u32) || !cur.readU8(u8) || !cur.readU32(u32)) {
      error = "truncated skeleton field " + std::to_string(i);
      return false;
    }
  }

  // is_bigendian(u8), point_step(u32), row_step(u32)
  uint8_t is_bigendian = 0;
  if (!cur.readU8(is_bigendian) || !cur.readU32(u32) || !cur.readU32(u32)) {
    error = "truncated skeleton layout";
    return false;
  }

  // data[] must be declared empty
  uint32_t data_len = 0;
  if (!cur.readU32(data_len)) {
    error = "truncated skeleton data length";
    return false;
  }
  if (data_len != 0) {
    error = "skeleton carries a non-empty data array";
    return false;
  }

  uint8_t is_dense = 0;
  if (!cur.readU8(is_dense)) {
    error = "truncated skeleton is_dense";
    return false;
  }
  if (!cur.atEnd()) {
    error = "trailing bytes after skeleton";
    return false;
  }
  return true;
}

bool deserializeSkeleton(const uint8_t* data, size_t size, sensor_msgs::PointCloud2& out, std::string& error) {
  // IStream takes a non-const pointer; copy into a local buffer rather than const_cast.
  // preflightSkeletonBytes() has already bounded `size`, so this copy is small.
  std::vector<uint8_t> buffer(data, data + size);
  try {
    ros::serialization::IStream stream(buffer.data(), static_cast<uint32_t>(buffer.size()));
    ros::serialization::deserialize(stream, out);
  } catch (const ros::Exception& e) {
    error = std::string("skeleton deserialization failed: ") + e.what();
    return false;
  } catch (const std::exception& e) {
    error = std::string("skeleton deserialization failed: ") + e.what();
    return false;
  }

  if (!out.data.empty()) {
    error = "deserialized skeleton carries point data";
    return false;
  }
  return true;
}

}  // namespace cloudini_ros1

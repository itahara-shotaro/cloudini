#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include "cloudini_ros1/wire_format.hpp"

using namespace cloudini_ros1;

namespace {

/// Minimal payload that passes the "long enough to hold a Cloudini header" check.
std::vector<uint8_t> dummyPayload(size_t n = 32) {
  return std::vector<uint8_t>(n, 0xAB);
}

sensor_msgs::PointCloud2 makeSkeleton() {
  sensor_msgs::PointCloud2 msg;
  msg.header.seq = 42;
  msg.header.stamp = ros::Time(123, 456);
  msg.header.frame_id = "lidar_link";
  msg.height = 1;
  msg.width = 10;
  msg.point_step = 16;
  msg.row_step = 160;
  msg.is_bigendian = false;
  msg.is_dense = true;

  const char* names[] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::PointField f;
    f.name = names[i];
    f.offset = static_cast<uint32_t>(4 * i);
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  return msg;
}

/// A well-formed envelope, for tests that then corrupt one thing about it.
std::vector<uint8_t> makeEnvelope() {
  std::vector<uint8_t> meta;
  serializeSkeleton(makeSkeleton(), meta);
  std::vector<uint8_t> out;
  writeEnvelope(meta, dummyPayload(), out);
  return out;
}

}  // namespace

TEST(WireFormat, LittleEndianHelpersRoundTrip) {
  std::vector<uint8_t> buf;
  putU16LE(buf, 0xBEEF);
  putU32LE(buf, 0xDEADBEEF);
  ASSERT_EQ(buf.size(), 6u);
  EXPECT_EQ(buf[0], 0xEF);  // explicitly little-endian on the wire
  EXPECT_EQ(buf[1], 0xBE);
  EXPECT_EQ(getU16LE(buf.data()), 0xBEEF);
  EXPECT_EQ(getU32LE(buf.data() + 2), 0xDEADBEEFu);
}

TEST(WireFormat, EnvelopeRoundTrip) {
  std::vector<uint8_t> meta{1, 2, 3, 4, 5};
  const auto payload = dummyPayload(64);
  std::vector<uint8_t> buf;
  writeEnvelope(meta, payload, buf);

  ASSERT_EQ(buf.size(), kEnvelopeHeaderLen + meta.size() + payload.size());

  EnvelopeView view;
  std::string error;
  ASSERT_TRUE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error)) << error;
  EXPECT_EQ(view.version, kEnvelopeVersion);
  EXPECT_EQ(view.flags, 0);
  ASSERT_EQ(view.metadata_len, meta.size());
  ASSERT_EQ(view.payload_len, payload.size());
  EXPECT_EQ(std::memcmp(view.metadata, meta.data(), meta.size()), 0);
  EXPECT_EQ(std::memcmp(view.payload, payload.data(), payload.size()), 0);
}

TEST(WireFormat, RejectsBadMagic) {
  auto buf = makeEnvelope();
  buf[0] = 'X';
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
  EXPECT_NE(error.find("magic"), std::string::npos);
}

TEST(WireFormat, RejectsUnsupportedVersion) {
  auto buf = makeEnvelope();
  buf[8] = 2;  // version low byte
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
  EXPECT_NE(error.find("version"), std::string::npos);
}

TEST(WireFormat, RejectsNonZeroReservedFlags) {
  auto buf = makeEnvelope();
  buf[10] = 1;  // flags low byte
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
  EXPECT_NE(error.find("flags"), std::string::npos);
}

TEST(WireFormat, RejectsTruncationAtEveryBoundary) {
  const auto buf = makeEnvelope();
  // Every proper prefix of a valid envelope must be rejected.
  for (size_t n = 0; n < buf.size(); ++n) {
    EnvelopeView view;
    std::string error;
    EXPECT_FALSE(parseEnvelope(buf.data(), n, WireLimits{}, view, error)) << "accepted a " << n << "-byte prefix";
  }
}

TEST(WireFormat, RejectsTrailingBytes) {
  auto buf = makeEnvelope();
  buf.push_back(0);
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
}

TEST(WireFormat, RejectsLengthsThatOverflow) {
  auto buf = makeEnvelope();
  // Both declared lengths at uint32 max: 20 + M + C must not wrap, and cannot equal size.
  for (int i = 12; i < 20; ++i) {
    buf[i] = 0xFF;
  }
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
}

TEST(WireFormat, RejectsSectionsOverLimits) {
  std::vector<uint8_t> meta(4096, 0);
  std::vector<uint8_t> buf;
  writeEnvelope(meta, dummyPayload(), buf);

  WireLimits limits;
  limits.max_metadata_bytes = 128;
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), limits, view, error));
  EXPECT_NE(error.find("max_metadata_bytes"), std::string::npos);

  limits = WireLimits{};
  limits.max_payload_bytes = 8;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), limits, view, error));
  EXPECT_NE(error.find("max_payload_bytes"), std::string::npos);

  limits = WireLimits{};
  limits.max_total_bytes = 8;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), limits, view, error));
}

TEST(WireFormat, RejectsPayloadTooShortForCloudiniHeader) {
  std::vector<uint8_t> buf;
  writeEnvelope({1, 2, 3}, std::vector<uint8_t>(4, 0), buf);
  EnvelopeView view;
  std::string error;
  EXPECT_FALSE(parseEnvelope(buf.data(), buf.size(), WireLimits{}, view, error));
}

// ------------------------------- skeleton --------------------------------------------

TEST(Skeleton, RoundTripsAllMetadata) {
  const auto original = makeSkeleton();
  std::vector<uint8_t> bytes;
  serializeSkeleton(original, bytes);

  std::string error;
  ASSERT_TRUE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error)) << error;

  sensor_msgs::PointCloud2 restored;
  ASSERT_TRUE(deserializeSkeleton(bytes.data(), bytes.size(), restored, error)) << error;

  EXPECT_EQ(restored.header.seq, original.header.seq);
  EXPECT_EQ(restored.header.stamp, original.header.stamp);
  EXPECT_EQ(restored.header.frame_id, original.header.frame_id);
  EXPECT_EQ(restored.height, original.height);
  EXPECT_EQ(restored.width, original.width);
  EXPECT_EQ(restored.point_step, original.point_step);
  EXPECT_EQ(restored.row_step, original.row_step);
  EXPECT_EQ(restored.is_dense, original.is_dense);
  EXPECT_EQ(restored.is_bigendian, original.is_bigendian);
  ASSERT_EQ(restored.fields.size(), original.fields.size());
  for (size_t i = 0; i < restored.fields.size(); ++i) {
    EXPECT_EQ(restored.fields[i].name, original.fields[i].name);
    EXPECT_EQ(restored.fields[i].offset, original.fields[i].offset);
    EXPECT_EQ(restored.fields[i].datatype, original.fields[i].datatype);
    EXPECT_EQ(restored.fields[i].count, original.fields[i].count);
  }
  EXPECT_TRUE(restored.data.empty());
}

TEST(Skeleton, SerializedSkeletonNeverCarriesData) {
  auto cloud = makeSkeleton();
  cloud.data.assign(cloud.row_step * cloud.height, 0x7F);
  std::vector<uint8_t> bytes;
  serializeSkeleton(cloud, bytes);

  std::string error;
  ASSERT_TRUE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error)) << error;
  sensor_msgs::PointCloud2 restored;
  ASSERT_TRUE(deserializeSkeleton(bytes.data(), bytes.size(), restored, error)) << error;
  EXPECT_TRUE(restored.data.empty());
}

TEST(Skeleton, PreflightRejectsHugeFieldCountWithoutAllocating) {
  // The whole reason preflight exists: ros::serialization resizes the vector before it
  // bounds-checks, so this must be rejected by our own parser first.
  std::vector<uint8_t> bytes;
  putU32LE(bytes, 0);            // seq
  putU32LE(bytes, 0);            // stamp.sec
  putU32LE(bytes, 0);            // stamp.nsec
  putU32LE(bytes, 0);            // frame_id length
  putU32LE(bytes, 1);            // height
  putU32LE(bytes, 1);            // width
  putU32LE(bytes, 0xFFFFFFFFu);  // field count

  std::string error;
  EXPECT_FALSE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error));
  EXPECT_NE(error.find("max_fields"), std::string::npos);
}

TEST(Skeleton, PreflightRejectsHugeStringLength) {
  std::vector<uint8_t> bytes;
  putU32LE(bytes, 0);
  putU32LE(bytes, 0);
  putU32LE(bytes, 0);
  putU32LE(bytes, 0xFFFFFFFFu);  // frame_id length

  std::string error;
  EXPECT_FALSE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error));
}

TEST(Skeleton, PreflightRejectsNonEmptyDataArray) {
  auto cloud = makeSkeleton();
  cloud.data.assign(4, 1);
  // Serialize by hand (serializeSkeleton would strip data), to exercise the check.
  const uint32_t len = ros::serialization::serializationLength(cloud);
  std::vector<uint8_t> bytes(len);
  ros::serialization::OStream stream(bytes.data(), len);
  ros::serialization::serialize(stream, cloud);

  std::string error;
  EXPECT_FALSE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error));
  EXPECT_NE(error.find("non-empty data"), std::string::npos);
}

TEST(Skeleton, PreflightRejectsTruncationAndTrailingBytes) {
  std::vector<uint8_t> bytes;
  serializeSkeleton(makeSkeleton(), bytes);

  std::string error;
  for (size_t n = 0; n < bytes.size(); ++n) {
    EXPECT_FALSE(preflightSkeletonBytes(bytes.data(), n, WireLimits{}, error)) << "accepted prefix of " << n;
  }

  bytes.push_back(0);
  EXPECT_FALSE(preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error));
  EXPECT_NE(error.find("trailing"), std::string::npos);
}

TEST(Skeleton, PreflightRejectsRandomGarbage) {
  // Deterministic pseudo-random bytes; none of these should ever be accepted, and none
  // should crash or allocate wildly.
  uint32_t state = 12345;
  for (int iter = 0; iter < 500; ++iter) {
    std::vector<uint8_t> bytes(iter % 64);
    for (auto& b : bytes) {
      state = state * 1103515245u + 12345u;
      b = static_cast<uint8_t>(state >> 16);
    }
    std::string error;
    sensor_msgs::PointCloud2 out;
    if (preflightSkeletonBytes(bytes.data(), bytes.size(), WireLimits{}, error)) {
      // If preflight ever passes, deserialization must still be safe.
      deserializeSkeleton(bytes.data(), bytes.size(), out, error);
    }
  }
  SUCCEED();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

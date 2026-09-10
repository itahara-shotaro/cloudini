#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "cloudini_ros1/codec_bridge.hpp"

using namespace cloudini_ros1;

namespace {

struct FieldSpec {
  const char* name;
  uint8_t datatype;
};

/// Builds a cloud with the given schema, tightly packed, `n` points, data left zeroed.
sensor_msgs::PointCloud2 makeCloud(const std::vector<FieldSpec>& specs, uint32_t n, uint32_t height = 1) {
  sensor_msgs::PointCloud2 msg;
  msg.header.seq = 7;
  msg.header.stamp = ros::Time(1700000000, 250000000);
  msg.header.frame_id = "sensor_frame";
  msg.height = height;
  msg.width = n;
  msg.is_bigendian = false;
  msg.is_dense = true;

  uint32_t offset = 0;
  for (const auto& spec : specs) {
    sensor_msgs::PointField f;
    f.name = spec.name;
    f.offset = offset;
    f.datatype = spec.datatype;
    f.count = 1;
    msg.fields.push_back(f);
    offset += static_cast<uint32_t>(rosDatatypeSize(spec.datatype));
  }
  msg.point_step = offset;
  msg.row_step = msg.width * msg.point_step;
  msg.data.assign(static_cast<size_t>(msg.row_step) * msg.height, 0);
  return msg;
}

template <typename T>
void poke(sensor_msgs::PointCloud2& msg, uint32_t point, uint32_t offset, T value) {
  std::memcpy(msg.data.data() + static_cast<size_t>(point) * msg.point_step + offset, &value, sizeof(T));
}

template <typename T>
T peek(const sensor_msgs::PointCloud2& msg, uint32_t point, uint32_t offset) {
  T value;
  std::memcpy(&value, msg.data.data() + static_cast<size_t>(point) * msg.point_step + offset, sizeof(T));
  return value;
}

sensor_msgs::PointCloud2 makeXYZI(uint32_t n) {
  auto msg = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT32},
       {"y", sensor_msgs::PointField::FLOAT32},
       {"z", sensor_msgs::PointField::FLOAT32},
       {"intensity", sensor_msgs::PointField::FLOAT32}},
      n);
  for (uint32_t i = 0; i < n; ++i) {
    poke<float>(msg, i, 0, 0.001f * static_cast<float>(i) - 5.0f);
    poke<float>(msg, i, 4, 0.25f * static_cast<float>(i));
    poke<float>(msg, i, 8, -3.75f + 0.01f * static_cast<float>(i));
    poke<float>(msg, i, 12, static_cast<float>(i % 256));
  }
  return msg;
}

/// Full encode -> decode cycle through the envelope.
::testing::AssertionResult roundTrip(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, sensor_msgs::PointCloud2& out) {
  EncoderCache cache;
  std::vector<uint8_t> payload_scratch;
  std::vector<uint8_t> meta_scratch;
  std_msgs::UInt8MultiArray wire;
  std::string error;

  if (!encodePointCloud(input, options, cache, payload_scratch, meta_scratch, wire, error)) {
    return ::testing::AssertionFailure() << "encode failed: " << error;
  }
  Cloudini::PointcloudDecoder decoder;
  if (!decodePointCloud(wire, options, decoder, out, error)) {
    return ::testing::AssertionFailure() << "decode failed: " << error;
  }
  return ::testing::AssertionSuccess();
}

/// Encodes, then hands the raw envelope bytes to `mutate` before decoding.
template <typename Mutator>
bool roundTripWithTamper(
    const sensor_msgs::PointCloud2& input, const CodecOptions& options, Mutator mutate, sensor_msgs::PointCloud2& out,
    std::string& error) {
  EncoderCache cache;
  std::vector<uint8_t> payload_scratch;
  std::vector<uint8_t> meta_scratch;
  std_msgs::UInt8MultiArray wire;
  if (!encodePointCloud(input, options, cache, payload_scratch, meta_scratch, wire, error)) {
    ADD_FAILURE() << "encode failed: " << error;
    return false;
  }
  mutate(wire);
  Cloudini::PointcloudDecoder decoder;
  return decodePointCloud(wire, options, decoder, out, error);
}

/// Rebuilds an envelope around a modified skeleton, keeping the original Cloudini payload.
void replaceSkeleton(std_msgs::UInt8MultiArray& wire, const sensor_msgs::PointCloud2& skeleton) {
  EnvelopeView view;
  std::string error;
  ASSERT_TRUE(parseEnvelope(wire.data.data(), wire.data.size(), WireLimits{}, view, error)) << error;
  const std::vector<uint8_t> payload(view.payload, view.payload + view.payload_len);
  std::vector<uint8_t> meta;
  serializeSkeleton(skeleton, meta);
  writeEnvelope(meta, payload, wire.data);
}

sensor_msgs::PointCloud2 skeletonOf(const std_msgs::UInt8MultiArray& wire) {
  EnvelopeView view;
  std::string error;
  sensor_msgs::PointCloud2 skeleton;
  EXPECT_TRUE(parseEnvelope(wire.data.data(), wire.data.size(), WireLimits{}, view, error)) << error;
  EXPECT_TRUE(deserializeSkeleton(view.metadata, view.metadata_len, skeleton, error)) << error;
  return skeleton;
}

}  // namespace

// ------------------------------ resolution policy -------------------------------------

TEST(ResolutionPolicy, NarrowingRejectsUnrepresentableValues) {
  float out = 0.0f;
  EXPECT_TRUE(toResolutionFloat(0.001, out));
  EXPECT_FLOAT_EQ(out, 0.001f);
  EXPECT_FALSE(toResolutionFloat(0.0, out));
  EXPECT_FALSE(toResolutionFloat(-1.0, out));
  EXPECT_FALSE(toResolutionFloat(std::numeric_limits<double>::quiet_NaN(), out));
  EXPECT_FALSE(toResolutionFloat(std::numeric_limits<double>::infinity(), out));
  // Underflows to 0.0f, which would throw inside the codec's encoder constructor.
  // (1e-45 is still a valid float denormal; 1e-50 is not.)
  EXPECT_FALSE(toResolutionFloat(1e-50, out));
  EXPECT_TRUE(toResolutionFloat(1e-45, out));
}

TEST(ResolutionPolicy, DefaultsQuantizeXyzAndIntensityOnly) {
  auto msg = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT32},
       {"y", sensor_msgs::PointField::FLOAT32},
       {"z", sensor_msgs::PointField::FLOAT32},
       {"intensity", sensor_msgs::PointField::FLOAT32},
       {"rgb", sensor_msgs::PointField::FLOAT32},
       {"ring", sensor_msgs::PointField::UINT16}},
      2);

  CodecOptions options;
  Cloudini::EncodingInfo info;
  std::string error;
  ASSERT_TRUE(buildEncodingInfo(msg, options, info, error)) << error;

  ASSERT_EQ(info.fields.size(), 6u);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(info.fields[i].resolution.has_value()) << "field " << info.fields[i].name;
    EXPECT_FLOAT_EQ(*info.fields[i].resolution, 0.001f);
  }
  ASSERT_TRUE(info.fields[3].resolution.has_value());
  EXPECT_FLOAT_EQ(*info.fields[3].resolution, 1.0f);
  EXPECT_FALSE(info.fields[4].resolution.has_value()) << "packed rgb must not be quantized";
  EXPECT_FALSE(info.fields[5].resolution.has_value()) << "integers must not be quantized";
}

TEST(ResolutionPolicy, IntensityQuantizationCanBeDisabled) {
  auto msg = makeXYZI(2);
  CodecOptions options;
  options.intensity_resolution = 0.0;

  Cloudini::EncodingInfo info;
  std::string error;
  ASSERT_TRUE(buildEncodingInfo(msg, options, info, error)) << error;
  EXPECT_FALSE(info.fields[3].resolution.has_value());
}

TEST(ResolutionPolicy, ExplicitOverridesWin) {
  auto msg = makeXYZI(2);
  CodecOptions options;
  options.field_resolutions["x"] = 0.05;

  Cloudini::EncodingInfo info;
  std::string error;
  ASSERT_TRUE(buildEncodingInfo(msg, options, info, error)) << error;
  EXPECT_FLOAT_EQ(*info.fields[0].resolution, 0.05f);
  EXPECT_FLOAT_EQ(*info.fields[1].resolution, 0.001f);
}

TEST(ResolutionPolicy, RejectsZeroAndNonFloatOverrides) {
  auto msg = makeCloud({{"x", sensor_msgs::PointField::FLOAT32}, {"ring", sensor_msgs::PointField::UINT16}}, 2);
  Cloudini::EncodingInfo info;
  std::string error;

  CodecOptions options;
  options.field_resolutions["x"] = 0.0;  // zero would mean "drop the field" in Cloudini
  EXPECT_FALSE(buildEncodingInfo(msg, options, info, error));

  options.field_resolutions.clear();
  options.field_resolutions["ring"] = 1.0;
  EXPECT_FALSE(buildEncodingInfo(msg, options, info, error));
  EXPECT_NE(error.find("not a float field"), std::string::npos);
}

// ------------------------------- round trips ------------------------------------------

TEST(RoundTrip, PreservesAllMetadata) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));

  // The envelope transports seq faithfully. Note that at the ROS layer a republishing
  // Publisher overwrites it (ros::Publication::incrementSequence), so end users must key
  // off header.stamp instead; see README, "Wire format".
  EXPECT_EQ(out.header.seq, input.header.seq);
  EXPECT_EQ(out.header.stamp, input.header.stamp);
  EXPECT_EQ(out.header.frame_id, input.header.frame_id);
  EXPECT_EQ(out.height, input.height);
  EXPECT_EQ(out.width, input.width);
  EXPECT_EQ(out.point_step, input.point_step);
  EXPECT_EQ(out.row_step, input.width * input.point_step);
  EXPECT_EQ(out.is_dense, input.is_dense);
  EXPECT_FALSE(out.is_bigendian);
  ASSERT_EQ(out.fields.size(), input.fields.size());
  for (size_t i = 0; i < out.fields.size(); ++i) {
    EXPECT_EQ(out.fields[i].name, input.fields[i].name);
    EXPECT_EQ(out.fields[i].offset, input.fields[i].offset);
    EXPECT_EQ(out.fields[i].datatype, input.fields[i].datatype);
    EXPECT_EQ(out.fields[i].count, 1u);
  }
  EXPECT_EQ(out.data.size(), input.data.size());
}

TEST(RoundTrip, XyzWithinHalfResolution) {
  const auto input = makeXYZI(500);
  CodecOptions options;
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, options, out));

  const double tolerance = 0.5 * options.xyz_resolution + 1e-6;
  for (uint32_t i = 0; i < input.width; ++i) {
    for (uint32_t offset : {0u, 4u, 8u}) {
      const double expected = peek<float>(input, i, offset);
      const double actual = peek<float>(out, i, offset);
      EXPECT_NEAR(actual, expected, tolerance) << "point " << i << " offset " << offset;
    }
  }
}

TEST(RoundTrip, IntensityWithinHalfResolution) {
  const auto input = makeXYZI(300);
  CodecOptions options;
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, options, out));

  const double tolerance = 0.5 * options.intensity_resolution + 1e-6;
  for (uint32_t i = 0; i < input.width; ++i) {
    EXPECT_NEAR(peek<float>(out, i, 12), peek<float>(input, i, 12), tolerance) << "point " << i;
  }
}

TEST(RoundTrip, PackedRgbAndIntegersAreExact) {
  auto input = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT32},
       {"y", sensor_msgs::PointField::FLOAT32},
       {"z", sensor_msgs::PointField::FLOAT32},
       {"rgb", sensor_msgs::PointField::FLOAT32},
       {"ring", sensor_msgs::PointField::UINT16},
       {"label", sensor_msgs::PointField::INT32}},
      200);

  for (uint32_t i = 0; i < input.width; ++i) {
    poke<float>(input, i, 0, 0.123f * static_cast<float>(i));
    poke<float>(input, i, 4, -0.456f * static_cast<float>(i));
    poke<float>(input, i, 8, 1.5f);
    // A packed RGB bit pattern: quantizing this would corrupt the colour entirely.
    const uint32_t packed = 0xFF00FF00u | (i & 0xFFu);
    float as_float;
    std::memcpy(&as_float, &packed, sizeof(packed));
    poke<float>(input, i, 12, as_float);
    poke<uint16_t>(input, i, 16, static_cast<uint16_t>(i % 64));
    poke<int32_t>(input, i, 18, -static_cast<int32_t>(i) * 3);
  }

  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));

  for (uint32_t i = 0; i < input.width; ++i) {
    EXPECT_EQ(peek<uint32_t>(out, i, 12), peek<uint32_t>(input, i, 12)) << "rgb bits changed at point " << i;
    EXPECT_EQ(peek<uint16_t>(out, i, 16), peek<uint16_t>(input, i, 16));
    EXPECT_EQ(peek<int32_t>(out, i, 18), peek<int32_t>(input, i, 18));
  }
}

TEST(RoundTrip, PreservesNaN) {
  auto input = makeXYZI(50);
  input.is_dense = false;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  poke<float>(input, 10, 0, nan);
  poke<float>(input, 10, 4, nan);
  poke<float>(input, 10, 8, nan);

  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));
  EXPECT_TRUE(std::isnan(peek<float>(out, 10, 0)));
  EXPECT_TRUE(std::isnan(peek<float>(out, 10, 4)));
  EXPECT_TRUE(std::isnan(peek<float>(out, 10, 8)));
  EXPECT_FALSE(out.is_dense);
  EXPECT_FALSE(std::isnan(peek<float>(out, 11, 0)));
}

TEST(RoundTrip, EmptyCloud) {
  const auto input = makeXYZI(0);
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));
  EXPECT_EQ(out.width, 0u);
  EXPECT_TRUE(out.data.empty());
  EXPECT_EQ(out.fields.size(), 4u);
}

TEST(RoundTrip, OrganizedCloud) {
  auto input = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT32},
       {"y", sensor_msgs::PointField::FLOAT32},
       {"z", sensor_msgs::PointField::FLOAT32}},
      64, /*height=*/48);
  for (uint32_t i = 0; i < input.width * input.height; ++i) {
    poke<float>(input, i, 0, static_cast<float>(i) * 0.01f);
  }

  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));
  EXPECT_EQ(out.height, 48u);
  EXPECT_EQ(out.width, 64u);
  EXPECT_EQ(out.data.size(), input.data.size());
  EXPECT_NEAR(peek<float>(out, 1000, 0), peek<float>(input, 1000, 0), 0.001);
}

TEST(RoundTrip, EveryRosScalarDatatype) {
  auto input = makeCloud(
      {{"f_i8", sensor_msgs::PointField::INT8},
       {"f_u8", sensor_msgs::PointField::UINT8},
       {"f_i16", sensor_msgs::PointField::INT16},
       {"f_u16", sensor_msgs::PointField::UINT16},
       {"f_i32", sensor_msgs::PointField::INT32},
       {"f_u32", sensor_msgs::PointField::UINT32},
       {"f_f32", sensor_msgs::PointField::FLOAT32},
       {"f_f64", sensor_msgs::PointField::FLOAT64}},
      64);

  for (uint32_t i = 0; i < input.width; ++i) {
    poke<int8_t>(input, i, 0, static_cast<int8_t>(i - 32));
    poke<uint8_t>(input, i, 1, static_cast<uint8_t>(i));
    poke<int16_t>(input, i, 2, static_cast<int16_t>(i * 100 - 3000));
    poke<uint16_t>(input, i, 4, static_cast<uint16_t>(i * 500));
    poke<int32_t>(input, i, 6, static_cast<int32_t>(i) * -100000);
    poke<uint32_t>(input, i, 10, i * 70000u);
    poke<float>(input, i, 14, 1.0f / (static_cast<float>(i) + 1.0f));
    poke<double>(input, i, 18, 1.0e9 + static_cast<double>(i) * 1e-6);
  }

  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, CodecOptions{}, out));

  // None of these field names are in the lossy whitelist, so all must be bit-exact.
  for (uint32_t i = 0; i < input.width; ++i) {
    EXPECT_EQ(peek<int8_t>(out, i, 0), peek<int8_t>(input, i, 0));
    EXPECT_EQ(peek<uint8_t>(out, i, 1), peek<uint8_t>(input, i, 1));
    EXPECT_EQ(peek<int16_t>(out, i, 2), peek<int16_t>(input, i, 2));
    EXPECT_EQ(peek<uint16_t>(out, i, 4), peek<uint16_t>(input, i, 4));
    EXPECT_EQ(peek<int32_t>(out, i, 6), peek<int32_t>(input, i, 6));
    EXPECT_EQ(peek<uint32_t>(out, i, 10), peek<uint32_t>(input, i, 10));
    EXPECT_EQ(peek<float>(out, i, 14), peek<float>(input, i, 14));
    EXPECT_EQ(peek<double>(out, i, 18), peek<double>(input, i, 18));
  }
}

TEST(RoundTrip, Float64Geometry) {
  auto input = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT64},
       {"y", sensor_msgs::PointField::FLOAT64},
       {"z", sensor_msgs::PointField::FLOAT64}},
      100);
  for (uint32_t i = 0; i < input.width; ++i) {
    poke<double>(input, i, 0, 3.14159 * i);
    poke<double>(input, i, 8, -2.71828 * i);
    poke<double>(input, i, 16, 0.5 * i);
  }

  CodecOptions options;
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, options, out));
  const double tolerance = 0.5 * options.xyz_resolution + 1e-6;
  for (uint32_t i = 0; i < input.width; ++i) {
    EXPECT_NEAR(peek<double>(out, i, 0), peek<double>(input, i, 0), tolerance);
    EXPECT_NEAR(peek<double>(out, i, 16), peek<double>(input, i, 16), tolerance);
  }
}

TEST(RoundTrip, AllCompressionAndEncodingModes) {
  const auto input = makeXYZI(200);
  for (auto compression :
       {Cloudini::CompressionOption::ZSTD, Cloudini::CompressionOption::LZ4, Cloudini::CompressionOption::NONE}) {
    for (auto encoding : {Cloudini::EncodingOptions::LOSSY, Cloudini::EncodingOptions::NONE}) {
      CodecOptions options;
      options.compression = compression;
      options.encoding = encoding;
      sensor_msgs::PointCloud2 out;
      ASSERT_TRUE(roundTrip(input, options, out))
          << "compression=" << static_cast<int>(compression) << " encoding=" << static_cast<int>(encoding);
      EXPECT_EQ(out.data.size(), input.data.size());
    }
  }
}

TEST(RoundTrip, SingleThreadedEncoder) {
  const auto input = makeXYZI(200);
  CodecOptions options;
  options.use_threads = false;
  sensor_msgs::PointCloud2 out;
  ASSERT_TRUE(roundTrip(input, options, out));
  EXPECT_NEAR(peek<float>(out, 5, 0), peek<float>(input, 5, 0), 0.001);
}

TEST(RoundTrip, CompressesSubstantially) {
  // Sanity check that the bridge is actually compressing, not just wrapping bytes.
  const auto input = makeXYZI(20000);
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;
  ASSERT_TRUE(encodePointCloud(input, CodecOptions{}, cache, payload, meta, wire, error)) << error;
  EXPECT_LT(wire.data.size(), input.data.size() / 2)
      << "compressed " << wire.data.size() << " vs raw " << input.data.size();
}

// ------------------------------- encoder cache ----------------------------------------

TEST(EncoderCache, ReusesEncoderForIdenticalConfig) {
  const auto input = makeXYZI(64);
  CodecOptions options;
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(encodePointCloud(input, options, cache, payload, meta, wire, error)) << error;
  }
  EXPECT_EQ(cache.rebuilds(), 1u);
}

TEST(EncoderCache, RebuildsOnWidthAndSchemaChange) {
  CodecOptions options;
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;

  ASSERT_TRUE(encodePointCloud(makeXYZI(64), options, cache, payload, meta, wire, error)) << error;
  EXPECT_EQ(cache.rebuilds(), 1u);

  ASSERT_TRUE(encodePointCloud(makeXYZI(65), options, cache, payload, meta, wire, error)) << error;
  EXPECT_EQ(cache.rebuilds(), 2u) << "width is baked into the header and must invalidate the cache";

  const auto other_schema = makeCloud(
      {{"x", sensor_msgs::PointField::FLOAT32},
       {"y", sensor_msgs::PointField::FLOAT32},
       {"z", sensor_msgs::PointField::FLOAT32}},
      65);
  ASSERT_TRUE(encodePointCloud(other_schema, options, cache, payload, meta, wire, error)) << error;
  EXPECT_EQ(cache.rebuilds(), 3u);
}

TEST(EncoderCache, SameConfigDistinguishesMembersOperatorEqualsIgnores) {
  // EncodingInfo::operator== ignores use_threads and version, both of which change behavior,
  // which is exactly why sameConfig() exists.
  Cloudini::EncodingInfo a;
  a.width = 10;
  a.point_step = 12;
  Cloudini::EncodingInfo b = a;
  EXPECT_TRUE(EncoderCache::sameConfig(a, b));

  b.use_threads = !a.use_threads;
  EXPECT_TRUE(a == b) << "precondition: operator== ignores use_threads";
  EXPECT_FALSE(EncoderCache::sameConfig(a, b));

  b = a;
  b.version = static_cast<uint8_t>(a.version - 1);
  EXPECT_FALSE(EncoderCache::sameConfig(a, b));
}

// ---------------------------- adversarial decode --------------------------------------

TEST(Decode, RejectsGarbage) {
  std_msgs::UInt8MultiArray wire;
  wire.data.assign(256, 0xCD);
  Cloudini::PointcloudDecoder decoder;
  sensor_msgs::PointCloud2 out;
  std::string error;
  EXPECT_FALSE(decodePointCloud(wire, CodecOptions{}, decoder, out, error));
}

TEST(Decode, RejectsEmptyMessage) {
  std_msgs::UInt8MultiArray wire;
  Cloudini::PointcloudDecoder decoder;
  sensor_msgs::PointCloud2 out;
  std::string error;
  EXPECT_FALSE(decodePointCloud(wire, CodecOptions{}, decoder, out, error));
}

TEST(Decode, RejectsCorruptedPayload) {
  const auto input = makeXYZI(500);
  sensor_msgs::PointCloud2 out;
  std::string error;

  // Corrupt a byte well inside the compressed chunk data.
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        const size_t pos = wire.data.size() - wire.data.size() / 4;
        wire.data[pos] ^= 0xFFu;
      },
      out, error);
  EXPECT_FALSE(ok) << "a corrupted payload must not decode";
}

TEST(Decode, RejectsSkeletonWidthMismatch) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  std::string error;
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        auto skeleton = skeletonOf(wire);
        skeleton.width = 101;
        skeleton.row_step = skeleton.width * skeleton.point_step;
        replaceSkeleton(wire, skeleton);
      },
      out, error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("disagree"), std::string::npos) << error;
}

TEST(Decode, RejectsSkeletonFieldNameMismatch) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  std::string error;
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        auto skeleton = skeletonOf(wire);
        skeleton.fields[1].name = "not_y";
        replaceSkeleton(wire, skeleton);
      },
      out, error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("name disagrees"), std::string::npos) << error;
}

TEST(Decode, RejectsSkeletonDatatypeMismatch) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  std::string error;
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        auto skeleton = skeletonOf(wire);
        skeleton.fields[2].datatype = sensor_msgs::PointField::INT32;
        replaceSkeleton(wire, skeleton);
      },
      out, error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("datatype disagrees"), std::string::npos) << error;
}

TEST(Decode, RejectsSkeletonFieldCountMismatch) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  std::string error;
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        auto skeleton = skeletonOf(wire);
        skeleton.fields.pop_back();
        replaceSkeleton(wire, skeleton);
      },
      out, error);
  EXPECT_FALSE(ok);
}

TEST(Decode, RejectsSkeletonThatFailsStructuralValidation) {
  const auto input = makeXYZI(100);
  sensor_msgs::PointCloud2 out;
  std::string error;
  const bool ok = roundTripWithTamper(
      input, CodecOptions{},
      [](std_msgs::UInt8MultiArray& wire) {
        auto skeleton = skeletonOf(wire);
        skeleton.row_step += 4;  // padded rows are not supported
        replaceSkeleton(wire, skeleton);
      },
      out, error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("row_step"), std::string::npos) << error;
}

TEST(Decode, EnforcesMaxDecodedBytes) {
  const auto input = makeXYZI(1000);
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;
  ASSERT_TRUE(encodePointCloud(input, CodecOptions{}, cache, payload, meta, wire, error)) << error;

  CodecOptions limits;
  limits.max_decoded_bytes = 1024;
  Cloudini::PointcloudDecoder decoder;
  sensor_msgs::PointCloud2 out;
  EXPECT_FALSE(decodePointCloud(wire, limits, decoder, out, error));
}

TEST(Decode, SurvivesEverySingleByteCorruption) {
  // Not asserting rejection everywhere -- some byte flips land in ignorable places -- but no
  // input may crash, hang, or produce an over-large allocation.
  const auto input = makeXYZI(50);
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray original;
  std::string error;
  ASSERT_TRUE(encodePointCloud(input, CodecOptions{}, cache, payload, meta, original, error)) << error;

  Cloudini::PointcloudDecoder decoder;
  for (size_t i = 0; i < original.data.size(); i += 7) {
    std_msgs::UInt8MultiArray wire = original;
    wire.data[i] ^= 0xFFu;
    sensor_msgs::PointCloud2 out;
    std::string ignored;
    if (decodePointCloud(wire, CodecOptions{}, decoder, out, ignored)) {
      EXPECT_EQ(out.data.size(), static_cast<size_t>(out.height) * out.row_step);
    }
  }
  SUCCEED();
}

// ------------------------------ encode rejection --------------------------------------

TEST(Encode, RejectsInvalidClouds) {
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;

  auto bad = makeXYZI(10);
  bad.is_bigendian = true;
  EXPECT_FALSE(encodePointCloud(bad, CodecOptions{}, cache, payload, meta, wire, error));

  bad = makeXYZI(10);
  bad.fields[0].count = 3;
  EXPECT_FALSE(encodePointCloud(bad, CodecOptions{}, cache, payload, meta, wire, error));

  bad = makeXYZI(10);
  poke<float>(bad, 3, 0, std::numeric_limits<float>::infinity());
  EXPECT_FALSE(encodePointCloud(bad, CodecOptions{}, cache, payload, meta, wire, error));
}

TEST(Encode, RangeCheckCanBeDisabled) {
  auto cloud = makeXYZI(10);
  poke<float>(cloud, 3, 0, std::numeric_limits<float>::infinity());

  CodecOptions options;
  options.check_value_range = false;
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;
  EXPECT_TRUE(encodePointCloud(cloud, options, cache, payload, meta, wire, error)) << error;
}

TEST(Encode, EnforcesMaxInputBytes) {
  const auto input = makeXYZI(1000);
  CodecOptions options;
  options.max_input_bytes = 512;
  EncoderCache cache;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> meta;
  std_msgs::UInt8MultiArray wire;
  std::string error;
  EXPECT_FALSE(encodePointCloud(input, options, cache, payload, meta, wire, error));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

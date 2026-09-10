#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "cloudini_ros1/validation.hpp"

using namespace cloudini_ros1;

namespace {

/// A valid XYZI cloud with `n` points, all coordinates finite.
sensor_msgs::PointCloud2 makeXYZI(uint32_t n = 4) {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "map";
  msg.height = 1;
  msg.width = n;
  msg.point_step = 16;
  msg.row_step = msg.width * msg.point_step;
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

  msg.data.assign(msg.row_step * msg.height, 0);
  for (uint32_t i = 0; i < n; ++i) {
    const float values[4] = {1.0f * i, 2.0f * i, 3.0f * i, 100.0f};
    std::memcpy(msg.data.data() + i * msg.point_step, values, sizeof(values));
  }
  return msg;
}

void setFloat(sensor_msgs::PointCloud2& msg, uint32_t point, uint32_t offset, float value) {
  std::memcpy(msg.data.data() + point * msg.point_step + offset, &value, sizeof(value));
}

std::vector<Cloudini::PointField> lossyXYZ(float resolution = 0.001f) {
  std::vector<Cloudini::PointField> fields;
  const char* names[] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    Cloudini::PointField f;
    f.name = names[i];
    f.offset = static_cast<uint32_t>(4 * i);
    f.type = Cloudini::FieldType::FLOAT32;
    f.resolution = resolution;
    fields.push_back(f);
  }
  return fields;
}

}  // namespace

TEST(Validation, CheckedArithmetic) {
  size_t out = 0;
  EXPECT_TRUE(checkedAdd(2, 3, out));
  EXPECT_EQ(out, 5u);
  EXPECT_FALSE(checkedAdd(std::numeric_limits<size_t>::max(), 1, out));

  EXPECT_TRUE(checkedMul(6, 7, out));
  EXPECT_EQ(out, 42u);
  EXPECT_FALSE(checkedMul(std::numeric_limits<size_t>::max(), 2, out));
}

TEST(Validation, DatatypeSizesMatchRos) {
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::INT8), 1);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::UINT8), 1);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::INT16), 2);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::UINT16), 2);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::INT32), 4);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::UINT32), 4);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::FLOAT32), 4);
  EXPECT_EQ(rosDatatypeSize(sensor_msgs::PointField::FLOAT64), 8);
  EXPECT_EQ(rosDatatypeSize(0), 0);
  EXPECT_EQ(rosDatatypeSize(9), 0);
  EXPECT_EQ(rosDatatypeSize(255), 0);
}

TEST(Validation, FieldTypeMappingIsIdentityForRosRange) {
  Cloudini::FieldType type;
  for (uint8_t d = 1; d <= 8; ++d) {
    ASSERT_TRUE(toCloudiniFieldType(d, type)) << "datatype " << int(d);
    EXPECT_EQ(static_cast<uint8_t>(type), d);
  }
  EXPECT_FALSE(toCloudiniFieldType(0, type));
  EXPECT_FALSE(toCloudiniFieldType(9, type));  // Cloudini INT64: no ROS equivalent
  EXPECT_FALSE(toCloudiniFieldType(255, type));
}

TEST(Validation, AcceptsWellFormedCloud) {
  std::string error;
  EXPECT_TRUE(validateCloudStructure(makeXYZI(), StructureLimits{}, true, error)) << error;
}

TEST(Validation, AcceptsEmptyCloudWithValidSchema) {
  auto msg = makeXYZI(0);
  std::string error;
  EXPECT_TRUE(validateCloudStructure(msg, StructureLimits{}, true, error)) << error;
}

TEST(Validation, AcceptsOrganizedCloud) {
  auto msg = makeXYZI(12);
  msg.height = 3;
  msg.width = 4;
  msg.row_step = msg.width * msg.point_step;
  msg.data.assign(msg.row_step * msg.height, 0);
  std::string error;
  EXPECT_TRUE(validateCloudStructure(msg, StructureLimits{}, true, error)) << error;
}

TEST(Validation, RejectsZeroPointStep) {
  auto msg = makeXYZI();
  msg.point_step = 0;
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("point_step"), std::string::npos);
}

TEST(Validation, RejectsPaddedRows) {
  auto msg = makeXYZI();
  msg.row_step += 8;  // row padding: Cloudini has no way to preserve it
  msg.data.assign(msg.row_step * msg.height, 0);
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("row_step"), std::string::npos);
}

TEST(Validation, RejectsDataSizeMismatch) {
  auto msg = makeXYZI();
  msg.data.pop_back();
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("data.size()"), std::string::npos);
}

TEST(Validation, RejectsBigEndian) {
  auto msg = makeXYZI();
  msg.is_bigendian = true;
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("big-endian"), std::string::npos);
}

TEST(Validation, RejectsArrayValuedFields) {
  auto msg = makeXYZI();
  msg.fields[3].count = 2;
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("count"), std::string::npos);
}

TEST(Validation, RejectsUnsupportedDatatypes) {
  for (uint8_t bad : {uint8_t(0), uint8_t(9), uint8_t(255)}) {
    auto msg = makeXYZI();
    msg.fields[1].datatype = bad;
    std::string error;
    EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error)) << "accepted datatype " << int(bad);
  }
}

TEST(Validation, RejectsFieldPastPointStep) {
  auto msg = makeXYZI();
  msg.fields[3].offset = 14;  // 14 + 4 > 16
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("past point_step"), std::string::npos);
}

TEST(Validation, RejectsOverlappingFields) {
  auto msg = makeXYZI();
  msg.fields[1].offset = 2;  // overlaps x at [0,4)
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
  EXPECT_NE(error.find("overlap"), std::string::npos);
}

TEST(Validation, RejectsEmptyFieldName) {
  auto msg = makeXYZI();
  msg.fields[0].name.clear();
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
}

TEST(Validation, RejectsOverflowingDimensions) {
  auto msg = makeXYZI();
  msg.width = 0xFFFFFFFFu;
  msg.point_step = 0xFFFFFFFFu;
  msg.height = 0xFFFFFFFFu;
  msg.row_step = 0xFFFFFFFFu;
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, true, error));
}

TEST(Validation, EnforcesMaxDataBytes) {
  auto msg = makeXYZI(1000);
  StructureLimits limits;
  limits.max_data_bytes = 128;
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, limits, true, error));
  EXPECT_NE(error.find("over the configured limit"), std::string::npos);
}

TEST(Validation, SkeletonModeRequiresEmptyData) {
  auto msg = makeXYZI();
  std::string error;
  EXPECT_FALSE(validateCloudStructure(msg, StructureLimits{}, false, error));

  msg.data.clear();
  EXPECT_TRUE(validateCloudStructure(msg, StructureLimits{}, false, error)) << error;
}

// -------------------------- quantizable range ----------------------------------------

TEST(QuantizableRange, AcceptsFiniteValues) {
  std::string error;
  EXPECT_TRUE(checkQuantizableRange(makeXYZI(), lossyXYZ(), error)) << error;
}

TEST(QuantizableRange, AcceptsNaN) {
  // Cloudini encodes NaN as varint 0 and restores quiet_NaN on decode, so it must pass.
  auto msg = makeXYZI();
  setFloat(msg, 2, 0, std::numeric_limits<float>::quiet_NaN());
  std::string error;
  EXPECT_TRUE(checkQuantizableRange(msg, lossyXYZ(), error)) << error;
}

TEST(QuantizableRange, RejectsInfinity) {
  for (float bad : {std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
    auto msg = makeXYZI();
    setFloat(msg, 1, 4, bad);
    std::string error;
    EXPECT_FALSE(checkQuantizableRange(msg, lossyXYZ(), error));
    EXPECT_NE(error.find("non-finite or out-of-range"), std::string::npos);
  }
}

TEST(QuantizableRange, RejectsValuesThatWouldSaturateInt32) {
  // Finite, but |v| / resolution exceeds the int32 lanes the SIMD group encoder uses, where
  // the value would silently saturate rather than error.
  auto msg = makeXYZI();
  setFloat(msg, 0, 8, 1.0e9f);  // 1e9 m at 1mm => 1e12 quantized steps
  std::string error;
  EXPECT_FALSE(checkQuantizableRange(msg, lossyXYZ(0.001f), error));
}

TEST(QuantizableRange, LargeValuesAreFineAtCoarseResolution) {
  auto msg = makeXYZI();
  setFloat(msg, 0, 8, 1.0e9f);
  std::string error;
  EXPECT_TRUE(checkQuantizableRange(msg, lossyXYZ(1.0f), error)) << error;
}

TEST(QuantizableRange, IgnoresLosslessFields) {
  auto msg = makeXYZI();
  setFloat(msg, 0, 12, std::numeric_limits<float>::infinity());  // intensity, no resolution
  std::string error;
  EXPECT_TRUE(checkQuantizableRange(msg, lossyXYZ(), error)) << error;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

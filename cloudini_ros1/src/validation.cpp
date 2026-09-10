#include "cloudini_ros1/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace cloudini_ros1 {

int rosDatatypeSize(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool toCloudiniFieldType(uint8_t datatype, Cloudini::FieldType& out) {
  // Cloudini::FieldType values 1..8 are defined to match sensor_msgs/PointField exactly.
  // 9/10 (INT64/UINT64) are Cloudini extensions with no ROS counterpart.
  if (datatype < 1 || datatype > 8) {
    return false;
  }
  out = static_cast<Cloudini::FieldType>(datatype);
  return true;
}

bool validateCloudStructure(
    const sensor_msgs::PointCloud2& msg, const StructureLimits& limits, bool require_data_size, std::string& error) {
  if (msg.point_step == 0) {
    error = "point_step is zero";
    return false;
  }
  if (msg.fields.size() > limits.max_fields) {
    error = "field count " + std::to_string(msg.fields.size()) + " exceeds max_fields";
    return false;
  }
  if (msg.is_bigendian) {
    error = "big-endian point clouds are not supported";
    return false;
  }

  // Collect byte ranges as we validate each field, then check for overlap.
  std::vector<std::pair<size_t, size_t>> ranges;
  ranges.reserve(msg.fields.size());

  for (size_t i = 0; i < msg.fields.size(); ++i) {
    const auto& field = msg.fields[i];
    const std::string where = "field '" + field.name + "' (index " + std::to_string(i) + ")";

    if (field.name.empty()) {
      error = "field index " + std::to_string(i) + " has an empty name";
      return false;
    }
    if (field.name.size() > limits.max_name_bytes) {
      error = where + " name exceeds max_name_bytes";
      return false;
    }
    if (field.count != 1) {
      error = where + " has count " + std::to_string(field.count) + "; only count == 1 is supported";
      return false;
    }
    const int type_size = rosDatatypeSize(field.datatype);
    if (type_size == 0) {
      error = where + " has unsupported datatype " + std::to_string(field.datatype);
      return false;
    }
    size_t end = 0;
    if (!checkedAdd(static_cast<size_t>(field.offset), static_cast<size_t>(type_size), end) || end > msg.point_step) {
      error = where + " extends past point_step";
      return false;
    }
    ranges.emplace_back(static_cast<size_t>(field.offset), end);
  }

  std::sort(ranges.begin(), ranges.end());
  for (size_t i = 1; i < ranges.size(); ++i) {
    if (ranges[i].first < ranges[i - 1].second) {
      error = "fields overlap in the point layout";
      return false;
    }
  }

  size_t expected_row = 0;
  if (!checkedMul(static_cast<size_t>(msg.width), static_cast<size_t>(msg.point_step), expected_row)) {
    error = "width * point_step overflows";
    return false;
  }
  if (expected_row != msg.row_step) {
    error = "row_step " + std::to_string(msg.row_step) + " != width * point_step " + std::to_string(expected_row) +
            "; padded rows are not supported";
    return false;
  }

  size_t total = 0;
  if (!checkedMul(static_cast<size_t>(msg.height), static_cast<size_t>(msg.row_step), total)) {
    error = "height * row_step overflows";
    return false;
  }
  if (total > limits.max_data_bytes) {
    error = "cloud declares " + std::to_string(total) + " bytes, over the configured limit";
    return false;
  }

  if (require_data_size) {
    if (msg.data.size() != total) {
      error = "data.size() " + std::to_string(msg.data.size()) + " != height * row_step " + std::to_string(total);
      return false;
    }
  } else if (!msg.data.empty()) {
    error = "metadata skeleton must carry no point data";
    return false;
  }

  return true;
}

namespace {

/// Reads a scalar out of the point buffer without assuming alignment.
template <typename T>
T loadUnaligned(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

bool valueInQuantizableRange(double v, double resolution) {
  // NaN is fine: the codec encodes it as varint 0 and restores quiet_NaN on decode.
  if (std::isnan(v)) {
    return true;
  }
  if (!std::isfinite(v)) {
    return false;
  }
  return std::fabs(v) <= kMaxQuantizedMagnitude * resolution;
}

}  // namespace

bool checkQuantizableRange(
    const sensor_msgs::PointCloud2& msg, const std::vector<Cloudini::PointField>& fields, std::string& error) {
  const size_t point_count = msg.point_step > 0 ? msg.data.size() / msg.point_step : 0;

  for (const auto& field : fields) {
    if (!field.resolution.has_value()) {
      continue;
    }
    const double resolution = static_cast<double>(*field.resolution);
    const bool is_double = (field.type == Cloudini::FieldType::FLOAT64);
    if (!is_double && field.type != Cloudini::FieldType::FLOAT32) {
      error = "field '" + field.name + "' has a resolution but is not a floating point type";
      return false;
    }

    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t* p = msg.data.data() + i * msg.point_step + field.offset;
      const double v = is_double ? loadUnaligned<double>(p) : static_cast<double>(loadUnaligned<float>(p));
      if (!valueInQuantizableRange(v, resolution)) {
        error = "field '" + field.name + "' holds a non-finite or out-of-range value at point " + std::to_string(i);
        return false;
      }
    }
  }
  return true;
}

}  // namespace cloudini_ros1

#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "cloudini_lib/basic_types.hpp"

namespace cloudini_ros1 {

/// Bounds applied to a cloud's declared structure before any allocation or codec call.
struct StructureLimits {
  size_t max_data_bytes = 64u << 20;
  size_t max_fields = 64;
  size_t max_name_bytes = 256;
};

/**
 * Largest |value| / resolution the codec can represent.
 *
 * FieldEncoderFloatN_Lossy (the SIMD group path used for leading lossy floats) quantizes
 * into int32 lanes, so out-of-range values silently saturate rather than trapping. A plain
 * isfinite() check does not catch that; this range check does. At 1mm resolution it still
 * permits +/- 2100 km.
 */
constexpr double kMaxQuantizedMagnitude = 2147483000.0;

inline bool checkedAdd(size_t a, size_t b, size_t& out) {
  return !__builtin_add_overflow(a, b, &out);
}

inline bool checkedMul(size_t a, size_t b, size_t& out) {
  return !__builtin_mul_overflow(a, b, &out);
}

/// Size in bytes of a sensor_msgs/PointField datatype; 0 for anything outside 1..8.
int rosDatatypeSize(uint8_t datatype);

/// Map a ROS PointField datatype (1..8) to a Cloudini FieldType. False for 0 and 9+.
bool toCloudiniFieldType(uint8_t datatype, Cloudini::FieldType& out);

/**
 * Full structural contract for a PointCloud2, applied to both locally produced clouds and
 * to skeletons received from the network. First failure wins; `error` names the field.
 *
 * @param require_data_size  true  -> data.size() must equal height * row_step
 *                           false -> data must be empty (metadata skeleton)
 */
bool validateCloudStructure(
    const sensor_msgs::PointCloud2& msg, const StructureLimits& limits, bool require_data_size, std::string& error);

/// Reject values that would be UB or silently saturate in the lossy encoders. NaN is allowed
/// (Cloudini round-trips it exactly); +/-inf and out-of-range magnitudes are not.
bool checkQuantizableRange(
    const sensor_msgs::PointCloud2& msg, const std::vector<Cloudini::PointField>& fields, std::string& error);

}  // namespace cloudini_ros1

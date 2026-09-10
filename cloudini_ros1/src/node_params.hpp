#pragma once

// Shared parameter-reading helpers for the two bridge nodes. Header-only and node-local:
// nothing here belongs in the public bridge library.

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include "cloudini_ros1/codec_bridge.hpp"

namespace cloudini_ros1 {

/// Reads a size limit expressed in bytes. ROS parameters are ints, so this clamps at 0.
inline bool readByteLimit(const ros::NodeHandle& pnh, const std::string& name, size_t& value) {
  int raw = 0;
  if (!pnh.getParam(name, raw)) {
    return true;  // keep the default
  }
  if (raw <= 0) {
    ROS_FATAL("~%s must be a positive number of bytes", name.c_str());
    return false;
  }
  value = static_cast<size_t>(raw);
  return true;
}

/// Parses the `~field_resolutions` rosparam dictionary.
inline bool readFieldResolutions(const ros::NodeHandle& pnh, std::map<std::string, double>& out) {
  XmlRpc::XmlRpcValue raw;
  if (!pnh.getParam("field_resolutions", raw)) {
    return true;
  }
  if (raw.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_FATAL("~field_resolutions must be a dictionary, e.g. {x: 0.001, intensity: 0.01}");
    return false;
  }

  for (auto it = raw.begin(); it != raw.end(); ++it) {
    const std::string& name = it->first;
    XmlRpc::XmlRpcValue& value = it->second;
    double resolution = 0.0;
    switch (value.getType()) {
      case XmlRpc::XmlRpcValue::TypeDouble:
        resolution = static_cast<double>(value);
        break;
      case XmlRpc::XmlRpcValue::TypeInt:
        // `{intensity: 1}` parses as an int, and casting a TypeInt to double throws.
        resolution = static_cast<double>(static_cast<int>(value));
        break;
      default:
        ROS_FATAL("~field_resolutions['%s'] must be a number", name.c_str());
        return false;
    }
    if (!std::isfinite(resolution) || resolution <= 0.0) {
      ROS_FATAL("~field_resolutions['%s'] must be finite and > 0", name.c_str());
      return false;
    }
    out[name] = resolution;
  }
  return true;
}

/// Cloudini::CompressionOptionFromString / EncodingOptionsFromString accept only the exact
/// uppercase spellings and otherwise fall through to std::stoi, which throws "stoi" for a
/// word and silently accepts a bare enum number. Parse the ROS parameters ourselves so that
/// `zstd`, `ZSTD` and `Zstd` all work and a typo produces a message that names the options.
inline bool parseCompression(const std::string& text, Cloudini::CompressionOption& out) {
  std::string upper = text;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return std::toupper(c); });
  if (upper == "ZSTD") {
    out = Cloudini::CompressionOption::ZSTD;
  } else if (upper == "LZ4") {
    out = Cloudini::CompressionOption::LZ4;
  } else if (upper == "NONE") {
    out = Cloudini::CompressionOption::NONE;
  } else {
    ROS_FATAL("~compression must be one of zstd, lz4, none (got '%s')", text.c_str());
    return false;
  }
  return true;
}

inline bool parseEncoding(const std::string& text, Cloudini::EncodingOptions& out) {
  std::string upper = text;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return std::toupper(c); });
  if (upper == "LOSSY") {
    out = Cloudini::EncodingOptions::LOSSY;
  } else if (upper == "LOSSLESS") {
    out = Cloudini::EncodingOptions::LOSSLESS;
  } else if (upper == "NONE") {
    out = Cloudini::EncodingOptions::NONE;
  } else {
    ROS_FATAL("~encoding must be one of lossy, lossless, none (got '%s')", text.c_str());
    return false;
  }
  return true;
}

/// Guards against a remap that would make a node subscribe to its own output.
inline bool checkNoRemapLoop(const std::string& input_topic, const std::string& output_topic) {
  if (ros::names::resolve(input_topic) == ros::names::resolve(output_topic)) {
    ROS_FATAL(
        "~input_topic and ~output_topic both resolve to '%s'; that would feed the node its own output",
        ros::names::resolve(input_topic).c_str());
    return false;
  }
  return true;
}

}  // namespace cloudini_ros1

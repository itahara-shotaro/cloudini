#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8MultiArray.h>

#include <string>
#include <vector>

#include "cloudini_ros1/codec_bridge.hpp"
#include "node_params.hpp"

namespace cloudini_ros1 {

class CompressNode {
 public:
  bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("input_topic", input_topic_, "/points");
    pnh.param<std::string>("output_topic", output_topic_, "/points/cloudini");
    pnh.param("queue_size", queue_size_, 1);
    pnh.param("skip_when_no_subscribers", skip_when_no_subscribers_, true);
    pnh.param("tcp_no_delay", tcp_no_delay_, true);
    pnh.param("log_period", log_period_, 10.0);

    pnh.param("xyz_resolution", options_.xyz_resolution, 0.001);
    pnh.param("intensity_resolution", options_.intensity_resolution, 1.0);
    pnh.param("use_threads", options_.use_threads, true);
    pnh.param("check_value_range", options_.check_value_range, true);

    if (!checkNoRemapLoop(input_topic_, output_topic_)) {
      return false;
    }
    if (queue_size_ < 1) {
      ROS_FATAL("~queue_size must be >= 1");
      return false;
    }
    if (!std::isfinite(options_.xyz_resolution) || options_.xyz_resolution <= 0.0) {
      ROS_FATAL("~xyz_resolution must be finite and > 0");
      return false;
    }
    if (!std::isfinite(options_.intensity_resolution) || options_.intensity_resolution < 0.0) {
      ROS_FATAL("~intensity_resolution must be finite and >= 0 (0 disables intensity quantization)");
      return false;
    }
    if (!readFieldResolutions(pnh, options_.field_resolutions)) {
      return false;
    }

    std::string compression;
    pnh.param<std::string>("compression", compression, "zstd");
    std::string encoding;
    pnh.param<std::string>("encoding", encoding, "lossy");
    if (!parseCompression(compression, options_.compression) || !parseEncoding(encoding, options_.encoding)) {
      return false;
    }

    if (!readByteLimit(pnh, "max_input_bytes", options_.max_input_bytes) ||
        !readByteLimit(pnh, "max_compressed_bytes", options_.wire.max_payload_bytes)) {
      return false;
    }
    options_.wire.max_total_bytes = options_.wire.max_payload_bytes + options_.wire.max_metadata_bytes + 1024;

    pub_ = nh.advertise<std_msgs::UInt8MultiArray>(output_topic_, static_cast<uint32_t>(queue_size_));
    sub_ = nh.subscribe(
        input_topic_, static_cast<uint32_t>(queue_size_), &CompressNode::onCloud, this,
        ros::TransportHints().tcpNoDelay(tcp_no_delay_));

    ROS_INFO(
        "cloudini_compress: %s -> %s (xyz=%.4fm intensity=%.4f compression=%s encoding=%s)", input_topic_.c_str(),
        output_topic_.c_str(), options_.xyz_resolution, options_.intensity_resolution, compression.c_str(),
        encoding.c_str());
    return true;
  }

 private:
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (skip_when_no_subscribers_ && pub_.getNumSubscribers() == 0) {
      ++skipped_;
      return;
    }

    const ros::WallTime start = ros::WallTime::now();
    std::string error;
    if (!encodePointCloud(*msg, options_, cache_, payload_scratch_, meta_scratch_, out_msg_, error)) {
      ++rejected_;
      ROS_WARN_THROTTLE(5.0, "cloudini_compress: dropped a cloud from %s: %s", input_topic_.c_str(), error.c_str());
      return;
    }
    const double elapsed_ms = (ros::WallTime::now() - start).toSec() * 1e3;

    pub_.publish(out_msg_);

    ++published_;
    total_in_bytes_ += msg->data.size();
    total_out_bytes_ += out_msg_.data.size();
    total_encode_ms_ += elapsed_ms;
    report();
  }

  void report() const {
    const double ratio = total_in_bytes_ > 0
                             ? 100.0 * static_cast<double>(total_out_bytes_) / static_cast<double>(total_in_bytes_)
                             : 0.0;
    ROS_INFO_THROTTLE(
        log_period_,
        "cloudini_compress: %zu published, ratio %.1f%%, %.2f ms/cloud, %zu cache rebuilds, %zu skipped, %zu rejected",
        published_, ratio, total_encode_ms_ / static_cast<double>(published_), cache_.rebuilds(), skipped_, rejected_);
  }

  std::string input_topic_;
  std::string output_topic_;
  int queue_size_ = 1;
  bool skip_when_no_subscribers_ = true;
  bool tcp_no_delay_ = true;
  double log_period_ = 10.0;

  CodecOptions options_;
  EncoderCache cache_;
  std::vector<uint8_t> payload_scratch_;
  std::vector<uint8_t> meta_scratch_;
  std_msgs::UInt8MultiArray out_msg_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

  size_t published_ = 0;
  size_t skipped_ = 0;
  size_t rejected_ = 0;
  size_t total_in_bytes_ = 0;
  size_t total_out_bytes_ = 0;
  double total_encode_ms_ = 0.0;
};

}  // namespace cloudini_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloudini_compress_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cloudini_ros1::CompressNode node;
  if (!node.init(nh, pnh)) {
    return 1;
  }

  // Single-threaded on purpose: PointcloudEncoder owns scratch buffers and a compression
  // worker thread, and is not safe for concurrent callbacks.
  ros::spin();
  return 0;
}

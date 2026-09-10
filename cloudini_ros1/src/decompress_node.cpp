#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8MultiArray.h>

#include <string>

#include "cloudini_ros1/codec_bridge.hpp"
#include "node_params.hpp"

namespace cloudini_ros1 {

class DecompressNode {
 public:
  bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("input_topic", input_topic_, "/points/cloudini");
    pnh.param<std::string>("output_topic", output_topic_, "/points/decoded");
    pnh.param("queue_size", queue_size_, 1);
    pnh.param("tcp_no_delay", tcp_no_delay_, true);
    pnh.param("log_period", log_period_, 10.0);

    if (!checkNoRemapLoop(input_topic_, output_topic_)) {
      return false;
    }
    if (queue_size_ < 1) {
      ROS_FATAL("~queue_size must be >= 1");
      return false;
    }
    if (!readByteLimit(pnh, "max_compressed_bytes", limits_.wire.max_payload_bytes) ||
        !readByteLimit(pnh, "max_decoded_bytes", limits_.max_decoded_bytes) ||
        !readByteLimit(pnh, "max_metadata_bytes", limits_.wire.max_metadata_bytes)) {
      return false;
    }
    limits_.wire.max_total_bytes = limits_.wire.max_payload_bytes + limits_.wire.max_metadata_bytes + 1024;

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, static_cast<uint32_t>(queue_size_));
    sub_ = nh.subscribe(
        input_topic_, static_cast<uint32_t>(queue_size_), &DecompressNode::onCompressed, this,
        ros::TransportHints().tcpNoDelay(tcp_no_delay_));

    ROS_INFO("cloudini_decompress: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
    return true;
  }

 private:
  void onCompressed(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // Deliberately no zero-subscriber skip here: dropping silently would make `rostopic hz`
    // on the decoded topic report nothing and look like a bridge failure.
    const ros::WallTime start = ros::WallTime::now();
    std::string error;
    if (!decodePointCloud(*msg, limits_, decoder_, out_msg_, error)) {
      ++rejected_;
      ROS_WARN_THROTTLE(
          5.0, "cloudini_decompress: rejected a message on %s: %s", input_topic_.c_str(), error.c_str());
      return;
    }
    const double elapsed_ms = (ros::WallTime::now() - start).toSec() * 1e3;

    pub_.publish(out_msg_);

    ++published_;
    total_decode_ms_ += elapsed_ms;
    ROS_INFO_THROTTLE(
        log_period_, "cloudini_decompress: %zu published, %.2f ms/cloud, %zu rejected", published_,
        total_decode_ms_ / static_cast<double>(published_), rejected_);
  }

  std::string input_topic_;
  std::string output_topic_;
  int queue_size_ = 1;
  bool tcp_no_delay_ = true;
  double log_period_ = 10.0;

  CodecOptions limits_;
  Cloudini::PointcloudDecoder decoder_;
  sensor_msgs::PointCloud2 out_msg_;

  ros::Publisher pub_;
  ros::Subscriber sub_;

  size_t published_ = 0;
  size_t rejected_ = 0;
  double total_decode_ms_ = 0.0;
};

}  // namespace cloudini_ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloudini_decompress_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cloudini_ros1::DecompressNode node;
  if (!node.init(nh, pnh)) {
    return 1;
  }

  // Single-threaded: one PointcloudDecoder is reused sequentially across callbacks.
  ros::spin();
  return 0;
}

// End-to-end test through a live ROS graph: this process publishes a PointCloud2, the two
// bridge nodes compress and decompress it, and this process checks what comes back.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt8MultiArray.h>

#include <cmath>
#include <cstring>

namespace {

constexpr double kXyzResolution = 0.001;  // must match the .test file
constexpr uint32_t kNumPoints = 500;

sensor_msgs::PointCloud2 makeXYZI(uint32_t n) {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "roundtrip_frame";
  msg.header.stamp = ros::Time(1700000000, 123456789);
  msg.height = 1;
  msg.width = n;
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
  msg.point_step = 16;
  msg.row_step = msg.width * msg.point_step;
  msg.data.assign(static_cast<size_t>(msg.row_step) * msg.height, 0);

  for (uint32_t i = 0; i < n; ++i) {
    const float values[4] = {
        0.01f * static_cast<float>(i) - 2.0f, -0.02f * static_cast<float>(i), 1.5f + 0.005f * static_cast<float>(i),
        static_cast<float>(i % 200)};
    std::memcpy(msg.data.data() + static_cast<size_t>(i) * msg.point_step, values, sizeof(values));
  }
  return msg;
}

float pointValue(const sensor_msgs::PointCloud2& msg, uint32_t point, uint32_t offset) {
  float v;
  std::memcpy(&v, msg.data.data() + static_cast<size_t>(point) * msg.point_step + offset, sizeof(v));
  return v;
}

/// Publishes `msg` repeatedly until `received` becomes non-null or the deadline passes.
/// Retrying matters: a single publish can be lost while the TCPROS connection is still
/// being negotiated.
template <typename T>
bool pumpUntilReceived(
    ros::Publisher& pub, const T& msg, const boost::shared_ptr<const sensor_msgs::PointCloud2>& received,
    double timeout_s) {
  ros::Rate rate(10.0);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_s);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    pub.publish(msg);
    ros::spinOnce();
    if (received) {
      return true;
    }
    rate.sleep();
  }
  return static_cast<bool>(received);
}

class RoundTripFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::NodeHandle nh;
    decoded_.reset();
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        "/points/decoded", 1, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) { decoded_ = msg; });
    raw_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/points", 1);
    compressed_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("/points/cloudini", 1);
  }

  ros::Subscriber sub_;
  ros::Publisher raw_pub_;
  ros::Publisher compressed_pub_;
  sensor_msgs::PointCloud2::ConstPtr decoded_;
};

TEST_F(RoundTripFixture, CloudSurvivesTheBridge) {
  const auto input = makeXYZI(kNumPoints);
  ASSERT_TRUE(pumpUntilReceived(raw_pub_, input, decoded_, 20.0)) << "no decoded cloud arrived on /points/decoded";

  ASSERT_TRUE(decoded_);
  EXPECT_EQ(decoded_->header.frame_id, input.header.frame_id);
  EXPECT_EQ(decoded_->header.stamp, input.header.stamp);
  EXPECT_EQ(decoded_->height, input.height);
  EXPECT_EQ(decoded_->width, input.width);
  EXPECT_EQ(decoded_->point_step, input.point_step);
  EXPECT_EQ(decoded_->row_step, input.row_step);
  EXPECT_EQ(decoded_->is_dense, input.is_dense);
  EXPECT_FALSE(decoded_->is_bigendian);
  ASSERT_EQ(decoded_->fields.size(), input.fields.size());
  for (size_t i = 0; i < input.fields.size(); ++i) {
    EXPECT_EQ(decoded_->fields[i].name, input.fields[i].name);
    EXPECT_EQ(decoded_->fields[i].offset, input.fields[i].offset);
    EXPECT_EQ(decoded_->fields[i].datatype, input.fields[i].datatype);
  }
  ASSERT_EQ(decoded_->data.size(), input.data.size());

  const double tolerance = 0.5 * kXyzResolution + 1e-6;
  for (uint32_t i = 0; i < kNumPoints; ++i) {
    for (uint32_t offset : {0u, 4u, 8u}) {
      EXPECT_NEAR(pointValue(*decoded_, i, offset), pointValue(input, i, offset), tolerance)
          << "point " << i << " offset " << offset;
    }
  }
}

TEST_F(RoundTripFixture, MalformedCompressedMessagesDoNotKillTheNodes) {
  // Feed the decompressor garbage on its input topic...
  std_msgs::UInt8MultiArray garbage;
  garbage.data.assign(512, 0xA5);
  for (int i = 0; i < 20 && ros::ok(); ++i) {
    compressed_pub_.publish(garbage);
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  // ...then verify a real cloud still makes it all the way through.
  const auto input = makeXYZI(64);
  decoded_.reset();
  ASSERT_TRUE(pumpUntilReceived(raw_pub_, input, decoded_, 20.0)) << "bridge stopped working after malformed input";
  EXPECT_EQ(decoded_->width, input.width);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cloudini_roundtrip_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}

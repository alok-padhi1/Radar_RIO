#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "gps_denied_nav_msgs/msg/keyframe.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace gps_denied_nav
{

class KeyframeManagerNode : public rclcpp::Node
{
public:
  KeyframeManagerNode() : Node("keyframe_manager")
  {
    this->declare_parameter<double>("translation_threshold_m", 0.5);
    this->declare_parameter<double>("rotation_threshold_deg", 5.0);
    this->declare_parameter<double>("max_time_interval_sec", 3.0);
    this->declare_parameter<std::string>("keyframe_storage_path", "/mnt/nvme/maps/keyframes/");

    trans_thresh_ = this->get_parameter("translation_threshold_m").as_double();
    rot_thresh_rad_ = this->get_parameter("rotation_threshold_deg").as_double() * M_PI / 180.0;
    time_thresh_ = this->get_parameter("max_time_interval_sec").as_double();
    storage_path_ = this->get_parameter("keyframe_storage_path").as_string();

    std::filesystem::create_directories(storage_path_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "lio_odom", 10, std::bind(&KeyframeManagerNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 10, std::bind(&KeyframeManagerNode::cloudCallback, this, std::placeholders::_1));

    kf_pub_ = this->create_publisher<gps_denied_nav_msgs::msg::Keyframe>("keyframe", 10);
    
    RCLCPP_INFO(this->get_logger(), "Keyframe Manager Node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!latest_odom_) return;

    if (isFirst_) {
        saveKeyframe(msg, latest_odom_);
        isFirst_ = false;
        return;
    }

    double dx = latest_odom_->pose.pose.position.x - last_kf_odom_->pose.pose.position.x;
    double dy = latest_odom_->pose.pose.position.y - last_kf_odom_->pose.pose.position.y;
    double dz = latest_odom_->pose.pose.position.z - last_kf_odom_->pose.pose.position.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    tf2::Quaternion q_curr(
      latest_odom_->pose.pose.orientation.x,
      latest_odom_->pose.pose.orientation.y,
      latest_odom_->pose.pose.orientation.z,
      latest_odom_->pose.pose.orientation.w);
      
    tf2::Quaternion q_last(
      last_kf_odom_->pose.pose.orientation.x,
      last_kf_odom_->pose.pose.orientation.y,
      last_kf_odom_->pose.pose.orientation.z,
      last_kf_odom_->pose.pose.orientation.w);
      
    double angle = q_curr.angleShortestPath(q_last);

    auto now = this->now();
    double dt = (now - last_kf_time_).seconds();

    if (dist > trans_thresh_ || angle > rot_thresh_rad_ || dt > time_thresh_) {
        saveKeyframe(msg, latest_odom_);
    }
  }

  void saveKeyframe(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, 
                    const nav_msgs::msg::Odometry::SharedPtr odom)
  {
      last_kf_odom_ = odom;
      last_kf_time_ = this->now();
      kf_id_++;

      std::stringstream ss;
      ss << storage_path_ << "/kf_" << std::setfill('0') << std::setw(6) << kf_id_ << ".bin";
      std::string path = ss.str();

      // Write pointcloud as binary float32 (x,y,z)
      std::ofstream out(path, std::ios::binary);
      uint32_t point_count = 0;
      
      if (out.is_open()) {
          sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
          sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
          sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

          for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
              float pt[3] = {*iter_x, *iter_y, *iter_z};
              out.write(reinterpret_cast<char*>(&pt), sizeof(pt));
              point_count++;
          }
          out.close();
      } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", path.c_str());
          return;
      }

      gps_denied_nav_msgs::msg::Keyframe kf_msg;
      kf_msg.keyframe_id = kf_id_;
      kf_msg.submap_id = kf_id_ / 10; // group 10 keyframes per submap
      kf_msg.timestamp = cloud->header.stamp.sec * 1000000000ULL + cloud->header.stamp.nanosec;
      kf_msg.pose = odom->pose;
      kf_msg.twist = odom->twist;
      kf_msg.point_count = point_count;
      kf_msg.storage_path = path;
      
      kf_pub_->publish(kf_msg);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Saved Keyframe %d with %d points", kf_id_, point_count);
  }

  double trans_thresh_;
  double rot_thresh_rad_;
  double time_thresh_;
  std::string storage_path_;
  
  bool isFirst_ = true;
  uint32_t kf_id_ = 0;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_kf_odom_;
  rclcpp::Time last_kf_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<gps_denied_nav_msgs::msg::Keyframe>::SharedPtr kf_pub_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::KeyframeManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

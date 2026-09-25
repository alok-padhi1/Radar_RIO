#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "gps_denied_nav/frame_converter.hpp"

namespace gps_denied_nav
{

class Px4BridgeNode : public rclcpp::Node
{
public:
  Px4BridgeNode() : Node("px4_bridge")
  {
    this->declare_parameter<std::string>("px4_odom_topic", "/fmu/in/vehicle_visual_odometry");
    this->declare_parameter<double>("px4_odom_rate_hz", 30.0);
    this->declare_parameter<double>("watchdog_timeout_sec", 0.5);

    auto px4_topic = this->get_parameter("px4_odom_topic").as_string();
    watchdog_timeout_sec_ = this->get_parameter("watchdog_timeout_sec").as_double();
    
    double rate_hz = this->get_parameter("px4_odom_rate_hz").as_double();
    min_period_sec_ = 1.0 / rate_hz;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "lio_odom", 10, std::bind(&Px4BridgeNode::odomCallback, this, std::placeholders::_1));

    px4_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(px4_topic, 10);
    
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&Px4BridgeNode::watchdogCheck, this));

    last_odom_time_ = this->now();
    last_pub_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PX4 Bridge Node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto now = this->now();
    last_odom_time_ = now;

    // Rate limiting
    if ((now - last_pub_time_).seconds() < min_period_sec_) {
        return; 
    }
    last_pub_time_ = now;

    nav_msgs::msg::Odometry px4_odom;
    px4_odom.header.stamp = msg->header.stamp;
    px4_odom.header.frame_id = "odom_ned"; 
    px4_odom.child_frame_id = "base_link_frd";

    // Convert pose ENU->NED
    px4_odom.pose = FrameConverter::convertPoseEnuToNed(msg->pose);

    // Convert twist FLU->FRD
    px4_odom.twist = FrameConverter::convertTwistFluToFrd(msg->twist);

    px4_odom_pub_->publish(px4_odom);
  }

  void watchdogCheck()
  {
      auto now = this->now();
      if ((now - last_odom_time_).seconds() > watchdog_timeout_sec_) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "PX4 Bridge Watchdog: No odometry received for %.2f seconds!", 
              (now - last_odom_time_).seconds());
      }
  }

  double watchdog_timeout_sec_;
  double min_period_sec_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_pub_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odom_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::Px4BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

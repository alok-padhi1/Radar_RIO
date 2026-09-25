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
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "lio_odom", 10, std::bind(&Px4BridgeNode::odomCallback, this, std::placeholders::_1));

    px4_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("px4_visual_odom", 10);
    
    RCLCPP_INFO(this->get_logger(), "PX4 Bridge Node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry px4_odom;
    px4_odom.header.stamp = msg->header.stamp;
    px4_odom.header.frame_id = "odom_ned"; // or map_ned
    px4_odom.child_frame_id = "base_link_frd";

    // Convert pose
    px4_odom.pose = FrameConverter::convertPoseEnuToNed(msg->pose);

    // Convert twist
    px4_odom.twist = FrameConverter::convertTwistFluToFrd(msg->twist);

    px4_odom_pub_->publish(px4_odom);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odom_pub_;
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

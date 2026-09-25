#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "gps_denied_nav_msgs/msg/estimator_health.hpp"

namespace gps_denied_nav
{

class LioOutputAdapterNode : public rclcpp::Node
{
public:
  LioOutputAdapterNode() : Node("lio_output_adapter")
  {
    fastlio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "Odometry", 10, std::bind(&LioOutputAdapterNode::odomCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);
    health_pub_ = this->create_publisher<gps_denied_nav_msgs::msg::EstimatorHealth>("estimator_health", 10);
    
    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 Output Adapter Node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Republish odometry for downstream nodes (px4_bridge, keyframe_manager)
    odom_pub_->publish(*msg);

    // Generate Health Message
    gps_denied_nav_msgs::msg::EstimatorHealth health_msg;
    health_msg.timestamp = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    health_msg.state = 1; // LIO_VALID state placeholder
    health_msg.pose_valid = true;
    health_msg.velocity_valid = true;
    health_msg.map_tracking_valid = true;
    
    // Extract dummy sigmas from covariance (assuming diagonals)
    health_msg.translational_sigma = std::sqrt(msg->pose.covariance[0]);
    health_msg.rotational_sigma = std::sqrt(msg->pose.covariance[21]);
    health_msg.velocity_sigma = std::sqrt(msg->twist.covariance[0]);
    
    // Publish health
    health_pub_->publish(health_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<gps_denied_nav_msgs::msg::EstimatorHealth>::SharedPtr health_pub_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::LioOutputAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

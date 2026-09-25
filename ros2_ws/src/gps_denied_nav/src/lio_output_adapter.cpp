#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "gps_denied_nav_msgs/msg/estimator_health.hpp"
#include <cmath>

namespace gps_denied_nav
{

class LioOutputAdapterNode : public rclcpp::Node
{
public:
  LioOutputAdapterNode() : Node("lio_output_adapter")
  {
    fastlio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10, std::bind(&LioOutputAdapterNode::odomCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);
    health_pub_ = this->create_publisher<gps_denied_nav_msgs::msg::EstimatorHealth>("estimator_health", 10);
    
    RCLCPP_INFO(this->get_logger(), "FAST-LIO2 Output Adapter Node initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto now = this->now();
    rclcpp::Time msg_time(msg->header.stamp);
    double latency_ms = (now - msg_time).seconds() * 1000.0;

    // Republish odometry for downstream nodes (px4_bridge, keyframe_manager)
    odom_pub_->publish(*msg);

    // Generate Health Message
    gps_denied_nav_msgs::msg::EstimatorHealth health_msg;
    health_msg.timestamp = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    health_msg.state = 1; // 1 = LIO_VALID
    health_msg.pose_valid = true;
    health_msg.velocity_valid = true;
    health_msg.map_tracking_valid = true;
    health_msg.processing_latency_ms = latency_ms;
    
    // FAST-LIO2 publishes covariance in pose.covariance (36 elements, row-major 6x6)
    // Indexes: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
    
    // Translational sigma (magnitude of x,y,z variances)
    double var_x = std::max(0.0, msg->pose.covariance[0]);
    double var_y = std::max(0.0, msg->pose.covariance[7]);
    double var_z = std::max(0.0, msg->pose.covariance[14]);
    health_msg.translational_sigma = std::sqrt(var_x + var_y + var_z);
    
    // Rotational sigma
    double var_roll = std::max(0.0, msg->pose.covariance[21]);
    double var_pitch = std::max(0.0, msg->pose.covariance[28]);
    double var_yaw = std::max(0.0, msg->pose.covariance[35]);
    health_msg.rotational_sigma = std::sqrt(var_roll + var_pitch + var_yaw);

    // Velocity sigma
    double var_vx = std::max(0.0, msg->twist.covariance[0]);
    double var_vy = std::max(0.0, msg->twist.covariance[7]);
    double var_vz = std::max(0.0, msg->twist.covariance[14]);
    health_msg.velocity_sigma = std::sqrt(var_vx + var_vy + var_vz);

    // Some dummy metrics for now until we hook up to actual LIO internals
    health_msg.tracking_score = 100.0;
    
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

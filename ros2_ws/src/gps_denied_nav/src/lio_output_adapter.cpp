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
  // Check if a double value is finite and not NaN
  bool is_valid(double v) { return std::isfinite(v) && !std::isnan(v); }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto now = this->now();
    rclcpp::Time msg_time(msg->header.stamp);
    double latency_ms = (now - msg_time).seconds() * 1000.0;

    // Republish odometry for downstream nodes
    odom_pub_->publish(*msg);

    // ---- Build health message with REAL validation ----
    gps_denied_nav_msgs::msg::EstimatorHealth health_msg;
    health_msg.timestamp = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    health_msg.processing_latency_ms = latency_ms;
    
    // Validate pose: all position and orientation values must be finite
    bool pose_ok = is_valid(msg->pose.pose.position.x)
                && is_valid(msg->pose.pose.position.y)
                && is_valid(msg->pose.pose.position.z)
                && is_valid(msg->pose.pose.orientation.x)
                && is_valid(msg->pose.pose.orientation.y)
                && is_valid(msg->pose.pose.orientation.z)
                && is_valid(msg->pose.pose.orientation.w);

    // Validate velocity: twist values must be finite
    bool vel_ok = is_valid(msg->twist.twist.linear.x)
               && is_valid(msg->twist.twist.linear.y)
               && is_valid(msg->twist.twist.linear.z)
               && is_valid(msg->twist.twist.angular.x)
               && is_valid(msg->twist.twist.angular.y)
               && is_valid(msg->twist.twist.angular.z);

    // Validate covariance: diagonal elements must be finite and non-negative
    bool cov_ok = true;
    for (int i = 0; i < 6; i++) {
        double diag = msg->pose.covariance[i * 6 + i];
        if (!is_valid(diag) || diag < 0.0) {
            cov_ok = false;
            break;
        }
    }

    // Latency check
    bool latency_ok = (latency_ms < 500.0);  // 500ms max acceptable latency

    // Derive estimator state
    health_msg.pose_valid = pose_ok;
    health_msg.velocity_valid = vel_ok;

    if (pose_ok && cov_ok && latency_ok) {
        health_msg.state = 1;  // LIO_VALID
        health_msg.map_tracking_valid = true;
    } else if (pose_ok && !cov_ok) {
        health_msg.state = 2;  // DEGRADED (pose is there but covariance is bad)
        health_msg.map_tracking_valid = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "LIO DEGRADED: covariance invalid");
    } else if (!latency_ok) {
        health_msg.state = 2;  // DEGRADED
        health_msg.map_tracking_valid = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "LIO DEGRADED: latency %.1f ms", latency_ms);
    } else {
        health_msg.state = 0;  // LOST
        health_msg.map_tracking_valid = false;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "LIO LOST: pose contains NaN/Inf!");
    }
    
    // Compute real sigma values from covariance
    double var_x = std::max(0.0, msg->pose.covariance[0]);
    double var_y = std::max(0.0, msg->pose.covariance[7]);
    double var_z = std::max(0.0, msg->pose.covariance[14]);
    health_msg.translational_sigma = std::sqrt(var_x + var_y + var_z);
    
    double var_roll = std::max(0.0, msg->pose.covariance[21]);
    double var_pitch = std::max(0.0, msg->pose.covariance[28]);
    double var_yaw = std::max(0.0, msg->pose.covariance[35]);
    health_msg.rotational_sigma = std::sqrt(var_roll + var_pitch + var_yaw);

    double var_vx = std::max(0.0, msg->twist.covariance[0]);
    double var_vy = std::max(0.0, msg->twist.covariance[7]);
    double var_vz = std::max(0.0, msg->twist.covariance[14]);
    health_msg.velocity_sigma = std::sqrt(var_vx + var_vy + var_vz);

    // Tracking score: derived from translational sigma (lower sigma = higher score)
    // Score of 100 means sigma < 0.01m, score of 0 means sigma > 1.0m
    if (health_msg.translational_sigma > 0.0) {
        health_msg.tracking_score = std::max(0.0, std::min(100.0,
            100.0 * (1.0 - health_msg.translational_sigma)));
    } else {
        health_msg.tracking_score = 0.0;  // no covariance data = no confidence
    }
    
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

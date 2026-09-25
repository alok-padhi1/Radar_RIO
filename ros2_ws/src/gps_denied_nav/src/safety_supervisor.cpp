#include <rclcpp/rclcpp.hpp>
#include "gps_denied_nav_msgs/msg/estimator_health.hpp"
#include <fstream>
#include <string>
#include <unistd.h>

namespace gps_denied_nav
{

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode() : Node("safety_supervisor")
  {
    health_sub_ = this->create_subscription<gps_denied_nav_msgs::msg::EstimatorHealth>(
      "estimator_health", 10, std::bind(&SafetySupervisorNode::healthCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&SafetySupervisorNode::checkResources, this));
      
    RCLCPP_INFO(this->get_logger(), "Safety Supervisor Node initialized");
  }

private:
  void healthCallback(const gps_denied_nav_msgs::msg::EstimatorHealth::SharedPtr msg)
  {
    if (!msg->pose_valid) {
      RCLCPP_WARN(this->get_logger(), "Estimator pose is invalid!");
    }
  }

  void checkResources()
  {
    // A simplified check of available memory (Linux specific)
    std::ifstream meminfo("/proc/meminfo");
    std::string line;
    long mem_total = 0, mem_available = 0;
    
    while (std::getline(meminfo, line)) {
      if (line.find("MemTotal:") == 0) {
        sscanf(line.c_str(), "MemTotal: %ld kB", &mem_total);
      } else if (line.find("MemAvailable:") == 0) {
        sscanf(line.c_str(), "MemAvailable: %ld kB", &mem_available);
      }
    }
    
    if (mem_total > 0) {
      double percent_available = static_cast<double>(mem_available) / mem_total * 100.0;
      
      if (percent_available < 10.0) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Memory < 10%% (%.1f%% available). Stop logging buffers!", percent_available);
      } else if (percent_available < 15.0) {
        RCLCPP_WARN(this->get_logger(), "WARNING: Memory < 15%% (%.1f%% available). Stop loop closure!", percent_available);
      } else if (percent_available < 25.0) {
        RCLCPP_INFO(this->get_logger(), "NOTICE: Memory < 25%% (%.1f%% available). Reduce map cache.", percent_available);
      }
    }
  }

  rclcpp::Subscription<gps_denied_nav_msgs::msg::EstimatorHealth>::SharedPtr health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::SafetySupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

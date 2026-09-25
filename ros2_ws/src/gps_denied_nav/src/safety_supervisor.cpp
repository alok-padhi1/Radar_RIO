#include <rclcpp/rclcpp.hpp>
#include "gps_denied_nav_msgs/msg/estimator_health.hpp"
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/sysinfo.h>

namespace gps_denied_nav
{

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode() : Node("safety_supervisor")
  {
    this->declare_parameter<double>("cpu_max_percent", 90.0);
    this->declare_parameter<int>("mem_min_available_mb", 500);
    this->declare_parameter<double>("sensor_max_latency_ms", 100.0);
    this->declare_parameter<double>("estimator_timeout_sec", 1.0);

    this->declare_parameter<double>("mem_level_normal_pct", 25.0);
    this->declare_parameter<double>("mem_level_reduce_cache_pct", 15.0);
    this->declare_parameter<double>("mem_level_stop_loop_closure_pct", 10.0);
    this->declare_parameter<int>("max_log_buffer_mb", 200);

    health_sub_ = this->create_subscription<gps_denied_nav_msgs::msg::EstimatorHealth>(
      "estimator_health", 10, std::bind(&SafetySupervisorNode::healthCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), std::bind(&SafetySupervisorNode::checkResources, this));
      
    last_health_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Safety Supervisor Node initialized");
  }

private:
  void healthCallback(const gps_denied_nav_msgs::msg::EstimatorHealth::SharedPtr msg)
  {
    last_health_time_ = this->now();

    if (!msg->pose_valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Estimator pose is invalid!");
    }
    
    double max_latency = this->get_parameter("sensor_max_latency_ms").as_double();
    if (msg->processing_latency_ms > max_latency) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Estimator latency high! %.1f ms", msg->processing_latency_ms);
    }
  }

  void checkResources()
  {
    // Check estimator timeout
    double timeout = this->get_parameter("estimator_timeout_sec").as_double();
    if ((this->now() - last_health_time_).seconds() > timeout) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Estimator health timeout! No updates for %.1f seconds.", 
                     (this->now() - last_health_time_).seconds());
    }

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
      
      double stop_loop = this->get_parameter("mem_level_stop_loop_closure_pct").as_double();
      double reduce_cache = this->get_parameter("mem_level_reduce_cache_pct").as_double();
      double normal = this->get_parameter("mem_level_normal_pct").as_double();

      if (percent_available < stop_loop) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Memory < %.1f%% (%.1f%% available). Stop logging buffers!", stop_loop, percent_available);
      } else if (percent_available < reduce_cache) {
        RCLCPP_WARN(this->get_logger(), "WARNING: Memory < %.1f%% (%.1f%% available). Stop loop closure!", reduce_cache, percent_available);
      } else if (percent_available < normal) {
        RCLCPP_INFO(this->get_logger(), "NOTICE: Memory < %.1f%% (%.1f%% available). Reduce map cache.", normal, percent_available);
      }
    }

    // Checking CPU load avg as a proxy for CPU watchdog
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        // info.loads[0] is 1-minute load avg, scale is 65536
        double load1 = (double)info.loads[0] / 65536.0;
        int nprocs = get_nprocs();
        double cpu_percent = (load1 / nprocs) * 100.0;
        
        double max_cpu = this->get_parameter("cpu_max_percent").as_double();
        if (cpu_percent > max_cpu) {
            RCLCPP_WARN(this->get_logger(), "CPU Load High: %.1f%% (1-min avg)", cpu_percent);
        }
    }
  }

  rclcpp::Subscription<gps_denied_nav_msgs::msg::EstimatorHealth>::SharedPtr health_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_health_time_;
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

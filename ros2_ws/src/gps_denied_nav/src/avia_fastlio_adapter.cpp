#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

namespace gps_denied_nav
{

class AviaFastLioAdapterNode : public rclcpp::Node
{
public:
  AviaFastLioAdapterNode() : Node("avia_fastlio_adapter")
  {
    this->declare_parameter<std::string>("input_lidar_topic", "/livox/lidar");
    this->declare_parameter<std::string>("input_imu_topic", "/livox/imu");
    this->declare_parameter<std::string>("output_lidar_topic", "/fastlio/lidar");
    this->declare_parameter<std::string>("output_imu_topic", "/fastlio/imu");
    this->declare_parameter<std::string>("lidar_frame_id", "livox_frame");
    this->declare_parameter<std::string>("imu_frame_id", "livox_frame");
    
    this->declare_parameter<double>("max_lidar_age_ms", 200.0);
    this->declare_parameter<double>("max_imu_age_ms", 50.0);
    this->declare_parameter<int>("min_points_per_scan", 100);

    auto in_lidar = this->get_parameter("input_lidar_topic").as_string();
    auto in_imu = this->get_parameter("input_imu_topic").as_string();
    auto out_lidar = this->get_parameter("output_lidar_topic").as_string();
    auto out_imu = this->get_parameter("output_imu_topic").as_string();
    
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
    imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
    max_lidar_age_ms_ = this->get_parameter("max_lidar_age_ms").as_double();
    max_imu_age_ms_ = this->get_parameter("max_imu_age_ms").as_double();
    min_points_per_scan_ = this->get_parameter("min_points_per_scan").as_int();

    cloud_sub_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
      in_lidar, 10, std::bind(&AviaFastLioAdapterNode::cloudCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      in_imu, 10, std::bind(&AviaFastLioAdapterNode::imuCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<livox_interfaces::msg::CustomMsg>(out_lidar, 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(out_imu, 10);
    
    RCLCPP_INFO(this->get_logger(), "Avia -> FAST-LIO2 Adapter Node initialized");
  }

private:
  void cloudCallback(const livox_interfaces::msg::CustomMsg::SharedPtr msg)
  {
    auto now = this->now();
    rclcpp::Time msg_time(msg->header.stamp);
    double age_ms = (now - msg_time).seconds() * 1000.0;

    if (age_ms > max_lidar_age_ms_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "LiDAR scan too old! Age: %.1f ms", age_ms);
        // We still forward it, but warn. FAST-LIO might reject it if too old.
    }

    if ((int)msg->point_num < min_points_per_scan_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "LiDAR scan too sparse! Points: %d", msg->point_num);
        return; // Reject malformed or empty packets
    }

    auto msg_out = *msg;
    msg_out.header.frame_id = lidar_frame_id_;
    cloud_pub_->publish(msg_out);
  }
  
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto now = this->now();
    rclcpp::Time msg_time(msg->header.stamp);
    double age_ms = (now - msg_time).seconds() * 1000.0;

    if (age_ms > max_imu_age_ms_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "IMU data too old! Age: %.1f ms", age_ms);
    }

    auto msg_out = *msg;
    msg_out.header.frame_id = imu_frame_id_;
    imu_pub_->publish(msg_out);
  }

  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  double max_lidar_age_ms_;
  double max_imu_age_ms_;
  int min_points_per_scan_;

  rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  rclcpp::Publisher<livox_interfaces::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::AviaFastLioAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace gps_denied_nav
{

class AviaFastLioAdapterNode : public rclcpp::Node
{
public:
  AviaFastLioAdapterNode() : Node("avia_fastlio_adapter")
  {
    // The exact topic types depend on whether Livox CustomMsg or PointCloud2 is used.
    // Assuming PointCloud2 for the generic adapter.
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", 10, std::bind(&AviaFastLioAdapterNode::cloudCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "livox/imu", 10, std::bind(&AviaFastLioAdapterNode::imuCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("fastlio/lidar", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("fastlio/imu", 10);
    
    RCLCPP_INFO(this->get_logger(), "Avia -> FAST-LIO2 Adapter Node initialized");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Pass-through or frame id modification can happen here.
    auto msg_out = *msg;
    // msg_out.header.frame_id = "lidar_link";
    cloud_pub_->publish(msg_out);
  }
  
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Pass-through or frame id modification can happen here.
    auto msg_out = *msg;
    // msg_out.header.frame_id = "imu_link";
    imu_pub_->publish(msg_out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
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

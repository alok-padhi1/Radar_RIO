#include <rclcpp/rclcpp.hpp>
#include "gps_denied_nav_msgs/msg/keyframe.hpp"
#include <fstream>
#include <filesystem>
#include <map>

namespace gps_denied_nav
{

class MapBackendNode : public rclcpp::Node
{
public:
  MapBackendNode() : Node("map_backend")
  {
    this->declare_parameter<std::string>("manifest_path", "/mnt/nvme/maps/manifest.yaml");
    this->declare_parameter<std::string>("map_db_path", "/mnt/nvme/maps/map.csv");

    manifest_path_ = this->get_parameter("manifest_path").as_string();
    map_db_path_ = this->get_parameter("map_db_path").as_string();

    std::filesystem::create_directories(std::filesystem::path(manifest_path_).parent_path());

    kf_sub_ = this->create_subscription<gps_denied_nav_msgs::msg::Keyframe>(
      "keyframe", 10, std::bind(&MapBackendNode::keyframeCallback, this, std::placeholders::_1));

    // Initialize/append to CSV log of keyframes (simple substitute for sqlite3 for now)
    bool file_exists = std::filesystem::exists(map_db_path_);
    db_file_.open(map_db_path_, std::ios::app);
    if (!file_exists) {
        db_file_ << "id,timestamp,x,y,z,qx,qy,qz,qw,submap,points,path\n";
    }

    RCLCPP_INFO(this->get_logger(), "Map Backend Node initialized");
  }
  
  ~MapBackendNode() {
      if (db_file_.is_open()) db_file_.close();
  }

private:
  void keyframeCallback(const gps_denied_nav_msgs::msg::Keyframe::SharedPtr msg)
  {
      // Write to DB
      db_file_ << msg->keyframe_id << "," 
               << msg->timestamp << ","
               << msg->pose.pose.position.x << ","
               << msg->pose.pose.position.y << ","
               << msg->pose.pose.position.z << ","
               << msg->pose.pose.orientation.x << ","
               << msg->pose.pose.orientation.y << ","
               << msg->pose.pose.orientation.z << ","
               << msg->pose.pose.orientation.w << ","
               << msg->submap_id << ","
               << msg->point_count << ","
               << msg->storage_path << "\n";
      db_file_.flush();

      // Atomic manifest update
      updateManifest(msg->keyframe_id, msg->submap_id);
  }

  void updateManifest(uint32_t last_kf, uint32_t last_submap)
  {
      std::string temp_path = manifest_path_ + ".tmp";
      std::ofstream out(temp_path);
      out << "map_version: 1.0\n";
      out << "last_keyframe_id: " << last_kf << "\n";
      out << "last_submap_id: " << last_submap << "\n";
      out << "is_closed: false\n";
      out.close();

      // Atomic rename
      std::filesystem::rename(temp_path, manifest_path_);
  }

  std::string manifest_path_;
  std::string map_db_path_;
  std::ofstream db_file_;

  rclcpp::Subscription<gps_denied_nav_msgs::msg::Keyframe>::SharedPtr kf_sub_;
};

} // namespace gps_denied_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gps_denied_nav::MapBackendNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

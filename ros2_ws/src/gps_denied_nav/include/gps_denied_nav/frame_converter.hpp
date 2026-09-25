#pragma once

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <Eigen/Dense>

namespace gps_denied_nav
{

class FrameConverter
{
public:
  /**
   * @brief Converts a pose from ENU (ROS) to NED (PX4)
   * 
   * @param pose_enu Input pose in ENU frame
   * @return geometry_msgs::msg::PoseWithCovariance Output pose in NED frame
   */
  static geometry_msgs::msg::PoseWithCovariance convertPoseEnuToNed(
    const geometry_msgs::msg::PoseWithCovariance & pose_enu);

  /**
   * @brief Converts a twist from FLU (ROS body) to FRD (PX4 body)
   * 
   * @param twist_flu Input twist in FLU frame
   * @return geometry_msgs::msg::TwistWithCovariance Output twist in FRD frame
   */
  static geometry_msgs::msg::TwistWithCovariance convertTwistFluToFrd(
    const geometry_msgs::msg::TwistWithCovariance & twist_flu);
    
  /**
   * @brief Converts a covariance matrix from one frame to another given a rotation matrix
   * 
   * @param cov_in Input 6x6 covariance (pose or twist)
   * @param R Rotation matrix representing the frame transformation
   * @return std::array<double, 36> Output 6x6 covariance
   */
  static std::array<double, 36> transformCovariance(
    const std::array<double, 36> & cov_in,
    const Eigen::Matrix3d & R);
};

} // namespace gps_denied_nav

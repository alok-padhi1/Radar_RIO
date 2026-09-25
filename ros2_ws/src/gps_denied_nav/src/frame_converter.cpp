#include "gps_denied_nav/frame_converter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace gps_denied_nav
{

geometry_msgs::msg::PoseWithCovariance FrameConverter::convertPoseEnuToNed(
  const geometry_msgs::msg::PoseWithCovariance & pose_enu)
{
  geometry_msgs::msg::PoseWithCovariance pose_ned;

  // Position: ENU to NED
  // x_ned = y_enu
  // y_ned = x_enu
  // z_ned = -z_enu
  pose_ned.pose.position.x = pose_enu.pose.position.y;
  pose_ned.pose.position.y = pose_enu.pose.position.x;
  pose_ned.pose.position.z = -pose_enu.pose.position.z;

  // Orientation: ENU to NED
  // We need to rotate the quaternion.
  // NED is rotated from ENU by a 90 deg rotation around Z, then 180 deg around X.
  tf2::Quaternion q_enu(
    pose_enu.pose.orientation.x,
    pose_enu.pose.orientation.y,
    pose_enu.pose.orientation.z,
    pose_enu.pose.orientation.w);
    
  // R_enu_to_ned = R_z(pi/2) * R_x(pi) ... wait, standard ENU to NED is:
  // [0, 1, 0]
  // [1, 0, 0]
  // [0, 0, -1]
  // Let's build this rotation matrix directly.
  Eigen::Matrix3d R_enu_to_ned;
  R_enu_to_ned << 0, 1, 0,
                  1, 0, 0,
                  0, 0, -1;

  tf2::Matrix3x3 tf_R_enu_to_ned(
    0, 1, 0,
    1, 0, 0,
    0, 0, -1);
    
  tf2::Matrix3x3 tf_R_enu(q_enu);
  tf2::Matrix3x3 tf_R_ned = tf_R_enu_to_ned * tf_R_enu;
  
  tf2::Quaternion q_ned;
  tf_R_ned.getRotation(q_ned);
  q_ned.normalize();

  pose_ned.pose.orientation.x = q_ned.x();
  pose_ned.pose.orientation.y = q_ned.y();
  pose_ned.pose.orientation.z = q_ned.z();
  pose_ned.pose.orientation.w = q_ned.w();

  // Covariance
  pose_ned.covariance = transformCovariance(pose_enu.covariance, R_enu_to_ned);

  return pose_ned;
}

geometry_msgs::msg::TwistWithCovariance FrameConverter::convertTwistFluToFrd(
  const geometry_msgs::msg::TwistWithCovariance & twist_flu)
{
  geometry_msgs::msg::TwistWithCovariance twist_frd;

  // Linear velocity: FLU to FRD
  // x_frd = x_flu
  // y_frd = -y_flu
  // z_frd = -z_flu
  twist_frd.twist.linear.x = twist_flu.twist.linear.x;
  twist_frd.twist.linear.y = -twist_flu.twist.linear.y;
  twist_frd.twist.linear.z = -twist_flu.twist.linear.z;

  // Angular velocity: FLU to FRD
  // x_frd = x_flu
  // y_frd = -y_flu
  // z_frd = -z_flu
  twist_frd.twist.angular.x = twist_flu.twist.angular.x;
  twist_frd.twist.angular.y = -twist_flu.twist.angular.y;
  twist_frd.twist.angular.z = -twist_flu.twist.angular.z;

  Eigen::Matrix3d R_flu_to_frd;
  R_flu_to_frd << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;

  // Covariance
  twist_frd.covariance = transformCovariance(twist_flu.covariance, R_flu_to_frd);

  return twist_frd;
}

std::array<double, 36> FrameConverter::transformCovariance(
  const std::array<double, 36> & cov_in,
  const Eigen::Matrix3d & R)
{
  std::array<double, 36> cov_out;
  cov_out.fill(0.0);
  
  // A 6x6 covariance matrix can be seen as 4 3x3 blocks:
  // [ C_pp  C_po ]
  // [ C_op  C_oo ]
  // We transform each block: C' = R * C * R^T

  Eigen::MatrixXd C_in(6, 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      C_in(i, j) = cov_in[i * 6 + j];
    }
  }

  Eigen::MatrixXd R6(6, 6);
  R6.setZero();
  R6.block<3, 3>(0, 0) = R;
  R6.block<3, 3>(3, 3) = R;

  Eigen::MatrixXd C_out = R6 * C_in * R6.transpose();

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      cov_out[i * 6 + j] = C_out(i, j);
    }
  }

  return cov_out;
}

} // namespace gps_denied_nav

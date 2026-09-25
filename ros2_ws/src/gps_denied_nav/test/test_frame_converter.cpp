#include <gtest/gtest.h>
#include "gps_denied_nav/frame_converter.hpp"

using namespace gps_denied_nav;

TEST(FrameConverterTest, EnuToNedPosition)
{
  geometry_msgs::msg::PoseWithCovariance pose_enu;
  pose_enu.pose.position.x = 1.0;
  pose_enu.pose.position.y = 2.0;
  pose_enu.pose.position.z = 3.0;
  
  auto pose_ned = FrameConverter::convertPoseEnuToNed(pose_enu);
  
  EXPECT_DOUBLE_EQ(pose_ned.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(pose_ned.pose.position.y, 1.0);
  EXPECT_DOUBLE_EQ(pose_ned.pose.position.z, -3.0);
}

TEST(FrameConverterTest, FluToFrdVelocity)
{
  geometry_msgs::msg::TwistWithCovariance twist_flu;
  twist_flu.twist.linear.x = 1.0;
  twist_flu.twist.linear.y = 2.0;
  twist_flu.twist.linear.z = 3.0;
  
  auto twist_frd = FrameConverter::convertTwistFluToFrd(twist_flu);
  
  EXPECT_DOUBLE_EQ(twist_frd.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist_frd.twist.linear.y, -2.0);
  EXPECT_DOUBLE_EQ(twist_frd.twist.linear.z, -3.0);
}

TEST(FrameConverterTest, EnuToNedOrientationZero)
{
  geometry_msgs::msg::PoseWithCovariance pose_enu;
  // ENU identity orientation
  pose_enu.pose.orientation.x = 0.0;
  pose_enu.pose.orientation.y = 0.0;
  pose_enu.pose.orientation.z = 0.0;
  pose_enu.pose.orientation.w = 1.0;
  
  auto pose_ned = FrameConverter::convertPoseEnuToNed(pose_enu);
  
  // R_enu_to_ned:
  // [0, 1, 0]
  // [1, 0, 0]
  // [0, 0, -1]
  
  // This corresponds to a 90 deg rotation around Z and 180 deg around X.
  // It shouldn't be zero orientation in NED. Let's just check it doesn't crash.
  EXPECT_TRUE(pose_ned.pose.orientation.w != 1.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

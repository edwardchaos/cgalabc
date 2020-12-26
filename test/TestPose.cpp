#include <gtest/gtest.h>

#include "Pose.h"
TEST(Pose, create){
  cg::Pose p;

  // Check default pose
  ASSERT_TRUE(p.orientation.isApprox(Eigen::Matrix4d::Identity()));
  ASSERT_TRUE(p.position.isApprox(Eigen::Vector4d(0,0,0,1)));
  ASSERT_TRUE(p.matrix().isApprox(Eigen::Matrix4d::Identity()));
}

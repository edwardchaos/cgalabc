#pragma once

#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>

namespace cg{

class Pose{
 public:
  Pose()=default;
  Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation);

  // Homogenous coordinates for convenience
  Eigen::Matrix4d orientation_ = Eigen::Matrix4d::Identity();
  Eigen::Vector4d position_ = Eigen::Vector4d::Identity();
 private:
};

} // namespace cg
#pragma once

#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>

namespace cg{

struct Pose{
  // Homogenous coordinates for convenience
  Eigen::Matrix4d orientation = Eigen::Matrix4d::Identity();
  Eigen::Vector4d position = Eigen::Vector4d::Identity();

  Eigen::Matrix4d matrix() const;
  Pose()=default;
  Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation);
};

} // namespace cg
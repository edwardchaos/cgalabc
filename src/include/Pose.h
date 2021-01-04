#pragma once

#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>

namespace cg{

struct Pose{
  // Homogenous coordinates for convenience
  Eigen::Matrix4d orientation;
  Eigen::Vector4d position;

  [[nodiscard]] Eigen::Matrix4d matrix() const;
  Pose();
  Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation);

  bool operator==(const Pose&other);
};

} // namespace cg
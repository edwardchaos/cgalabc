#include "Pose.h"

namespace cg{

Pose::Pose(){
  orientation = Eigen::Matrix4d::Identity();
  position = Eigen::Vector4d::Zero();
  position(3) = 1;
}

Pose::Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation):
orientation(std::move(orientation)), position(std::move(position)){}

Eigen::Matrix4d Pose::matrix() const{
  Eigen::Matrix4d pose = orientation;
  pose.rightCols<1>() = position;
  return pose;
}

} // namespace cg

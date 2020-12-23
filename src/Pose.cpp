#include "Pose.h"

namespace cg{

Pose::Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation):
orientation_(std::move(orientation)), position_(std::move(position)){}

Eigen::Vector4d Pose::position()const{
  return position_;
}

Eigen::Matrix4d Pose::orientation()const{
  return orientation_;

}

} // namespace cg

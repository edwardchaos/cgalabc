#include "Pose.h"

namespace cg{

Pose::Pose(Eigen::Vector4d position, Eigen::Matrix4d orientation):
orientation_(std::move(orientation)), position_(std::move(position)){


}

} // namespace cg

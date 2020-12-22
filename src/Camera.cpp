#include "include/Camera.h"

#include <cmath>

namespace cg {

Camera::Camera(double aspect_ratio, double vertical_fov_rad, double near,
               double far): ar_(aspect_ratio), vertical_fov_
               (vertical_fov_rad), near_plane_dist_(near), far_plane_dist_(far){

  init();
}

void Camera::init(){
  pose_world_ = Eigen::Matrix4d::Ones();
  constructProjectionMatrix();
}

void Camera::constructProjectionMatrix(){
  projection_mat_ = Eigen::Matrix4d::Zero();
  // cos(x)/sin(x) numerically stable version of cot(x)
  double f = std::cos(vertical_fov_/2.0) / std::sin(vertical_fov_/2.0);

  projection_mat_(0,0) = ar_*f;
  projection_mat_(1,1) = f;
  projection_mat_(2,2) = (near_plane_dist_+far_plane_dist_)
      /(far_plane_dist_-near_plane_dist_);
  projection_mat_(3,2) = (2*far_plane_dist_*near_plane_dist_)
      /(far_plane_dist_-near_plane_dist_);
  projection_mat_(2,3) = -1;
}

Point2d_ptr Camera::projectPoint(const Vector4d &pt_homo) const{
  Eigen::RowVector4d row_pt(pt_homo);

  Eigen::RowVector4d pt = row_pt*projection_mat_;

  // TODO: Verify -y since in the image, y axis is positive downward.
  if(pt(3) != 0)
    return std::make_shared<Point2d>(pt(0)/pt(3), -pt(1)/pt(3));
  return nullptr;
}

} // namespace cg

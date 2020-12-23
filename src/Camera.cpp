#include "include/Camera.h"

#include <cmath>

#include "iostream"
namespace cg {

Camera::Camera(double aspect_ratio, double vertical_fov_rad, double near,
               double far): ar_(aspect_ratio), vertical_fov_
               (vertical_fov_rad), near_plane_dist_(near), far_plane_dist_(far){

  init();
}

void Camera::init(){
  pose_world_ = Eigen::Matrix4d::Identity();
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

  if(pt(3) != 0)
    return std::make_shared<Point2d>(pt(0)/pt(3), -pt(1)/pt(3));
  return nullptr;
}

bool Camera::isFacing(const Triangle& tri) const{
  // extract camera's position
  Vector3d cam_position = pose_world_.rightCols<1>().head<3>();
  Vector3d cam_2_tri = tri.points[0] - cam_position;
  cam_2_tri.normalize();
  return cam_2_tri.dot(tri.unit_normal()) < -EPS;
}

} // namespace cg

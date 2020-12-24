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

bool Camera::isFacing(const Triangle& tri_world) const{
  // extract camera's position
  Vector3d cam_position = pose_world.position.head<3>();
  Vector3d cam_2_tri = tri_world.points[0] - cam_position;
  cam_2_tri.normalize();
  return cam_2_tri.dot(tri_world.unit_normal()) < -EPS;
}

void Camera::moveTo(Vector4d position_world,
                    Vector4d look_dir,
                    Vector4d up){

  // Ensure they're normalized
  look_dir.normalize();
  up.normalize();

  Vector3d right_vec = look_dir.head<3>().cross(up.head<3>());
  right_vec.normalize();
  Vector3d up_vec = right_vec.cross(look_dir.head<3>());
  up_vec.normalize();

  Matrix4d new_cam_orientation= Matrix4d::Identity();
  new_cam_orientation.col(0).head<3>() = right_vec;
  new_cam_orientation.col(1).head<3>() = up_vec;
  new_cam_orientation.col(2) = look_dir;

  pose_world.orientation = new_cam_orientation;
  pose_world.position = position_world;
}

} // namespace cg

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

Vector4d Camera::tfPointWorldToCam(const Vector3d &pt_world) const{
  // Transform triangle in world coordinate to camera's coordinate
  Vector4d pt_cam;
  Vector4d pt_world_homo;
  pt_world_homo.head<3>() = pt_world;
  pt_world_homo(3) = 1;
  pt_cam = pose_world.matrix().inverse()*pt_world_homo;
  return pt_cam;
}

Point2d_ptr Camera::projectPoint(const Vector3d &pt_world) const{
  // Transform homogenous point in world coordinates to camera coordinate.
  Vector4d pt_cam = tfPointWorldToCam(pt_world);

  Eigen::RowVector4d row_pt(pt_cam);
  Eigen::RowVector4d pt = row_pt*projection_mat_;

  if(pt(3) != 0)
    return std::make_shared<Point2d>(pt(0)/pt(3), -pt(1)/pt(3));
  return nullptr;
}

bool Camera::isFacing(const Triangle& tri_world) const{
  // Transform triangle in world coordinate to camera's coordinate
  Vector4d pt0 = tfPointWorldToCam(tri_world.points[0]);
  Vector4d pt1 = tfPointWorldToCam(tri_world.points[1]);
  Vector4d pt2 = tfPointWorldToCam(tri_world.points[2]);

  // Triangle in camera coordinate frame
  Triangle tri_cam{pt0.head<3>(), pt1.head<3>(), pt2.head<3>()};

  Vector3d cam_position = pose_world.position.head<3>();
  Vector3d cam_2_tri = tri_cam.points[0] - cam_position;
  cam_2_tri.normalize();
  return cam_2_tri.dot(tri_cam.unit_normal()) < -EPS;
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
  new_cam_orientation.col(2) = -look_dir;

  pose_world.orientation = new_cam_orientation;
  pose_world.position = std::move(position_world);
}

void Camera::moveForward(double units){
  // Extract look dir unit vec (rotation matrix is orthonormal)
  Vector4d look_dir = -pose_world.orientation.col(2);

  // Add look dir to camera's position
  pose_world.position += look_dir*units;
}

} // namespace cg

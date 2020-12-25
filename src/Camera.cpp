#include "include/Camera.h"

#include <cassert>
#include <cmath>

#include <Utility.h>

namespace cg {

Camera::Camera(double aspect_ratio, double vertical_fov_rad, double near,
               double far, int screen_width_px, int screen_height_px):
               ar_(aspect_ratio),vertical_fov_(vertical_fov_rad),
               near_plane_dist_(near), far_plane_dist_(far),
               screen_width_(screen_width_px), screen_height_(screen_height_px){
  init();
}

void Camera::init(){
  // Camera's x-axis is in the opposite direction of the world's x axis at start
  pose_world.orientation(0,0) = -1;
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
  auto pt_cube = transformPoint(pt_world);
  return std::make_shared<Point2d>(pt_cube(0), -pt_cube(1));
}

Vector3d Camera::projectPointInCameraFrame(const Vector3d &pt_cam)const{
  Eigen::RowVector4d row_pt;
  row_pt.head<3>() = pt_cam;
  row_pt(3) = 1;
  Eigen::RowVector4d pt = row_pt*projection_mat_;

  assert(pt(3) != 0);
  Vector3d pt_cube(pt(0)/pt(3), pt(1)/pt(3), pt(2)/pt(3));
  return pt_cube;
}

Vector3d Camera::transformPoint(const Vector3d &pt_world) const{
  // Transform homogenous point in world coordinates to camera coordinate.
  Vector4d pt_cam = tfPointWorldToCam(pt_world);

  Eigen::RowVector4d row_pt(pt_cam);
  Eigen::RowVector4d pt = row_pt*projection_mat_;

  assert(pt(3) != 0);
  Vector3d pt_cube(pt(0)/pt(3), pt(1)/pt(3), pt(2)/pt(3));
  return pt_cube;
}

Triangle Camera::tfTriangleWorldToCam(const Triangle& tri_world) const{
  // Transform to camera coordinate
  Triangle tri_cam;
  for(int i = 0; i < 3; ++i)
    tri_cam.points[i] = tfPointWorldToCam(tri_world.points[i]).head<3>();

  return tri_cam;
}

std::vector<Triangle> Camera::clipNear(const Triangle& tri_cam) const{
  // Near plane normal vector pointed in camera view direction
  Vector3d near_plane_unit_normal(0,0,-1);
  // Point on the near plane in camera coordinates
  Vector3d near_plane_pt(0,0,-near_plane_dist_);

  // d holds dot products; Will be used for determining which side of the
  // plane the point is on.
  double d[3];
  d[0] = near_plane_unit_normal.dot(tri_cam.points[0]-near_plane_pt);
  d[1] = near_plane_unit_normal.dot(tri_cam.points[1]-near_plane_pt);
  d[2] = near_plane_unit_normal.dot(tri_cam.points[2]-near_plane_pt);

  // Triangle is completely on the 'out' side of near plane. Nothing to keep.
  if(d[0] <= EPS && d[1] <= EPS && d[2] <= EPS) return{};

  // Triangle is completely on the 'in' side of the near plane.
  if(d[0] >= -EPS && d[1] >= -EPS && d[2] >= -EPS) return{tri_cam};

  std::vector<Vector3d> in,out;

  for(int i = 0 ; i < 3; ++i){
    int cur_idx = i;
    int next_idx = (i+1)%3;

    auto cur_pt = tri_cam.points[cur_idx];
    auto next_pt = tri_cam.points[next_idx];

    // Add current point in 'In' or 'Out'?
    if(d[cur_idx] < -EPS) { // 'Out' side
      // Last point is not already there (for edge case when a point is
      // directly on the plane)
      if (out.empty() || (!out.empty() && !cur_pt.isApprox(out.back()))){
        out.push_back(cur_pt);
      }
    }
    // 'In' side
    else if(in.empty() || (!in.empty() && !cur_pt.isApprox(in.back()))){
      in.push_back(cur_pt);
    }

    // Is there an intersection point to add?
    if(d[cur_idx] * d[next_idx] < EPS){
      // current point on one side, next point on other side. There is an
      // intersection point.
      auto int_pt = planeLineIntersect(
          cur_pt, next_pt, near_plane_unit_normal, near_plane_pt);

      assert(int_pt!=nullptr);
      in.push_back(*int_pt);
      out.push_back(*int_pt);
    }
  }

  assert(in.size() == 3 || in.size() == 4);
  // Create triangles with 'In' points
  if(in.size() == 3) {
    return {Triangle(in[0], in[1], in[2])};
  }
  else
    return {
        Triangle(in[0],in[1],in[2]),
        Triangle(in[0],in[2],in[3])};
}

bool Camera::isFacing(const Triangle& tri_world) const{
  // Transform triangle in world coordinate to camera's coordinate
  auto tri_cam = tfTriangleWorldToCam(tri_world);

  Vector3d cam_2_tri = tri_cam.points[0];
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

void Camera::strafeRight(double units){
  // Extract right vector
  Vector4d right_vec = pose_world.orientation.col(0);

  // Add look dir to camera's position
  pose_world.position += right_vec*units;
}

void Camera::yawRight(double theta_radians){
  pose_world.orientation *= rotateY(-theta_radians);
}

} // namespace cg

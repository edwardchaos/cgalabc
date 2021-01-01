#include "include/Camera.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

#include <Utility.h>

namespace cg {

Camera::Camera(double vertical_fov_rad, double near,
               double far, int screen_width_px, int screen_height_px):
               ar_((double)screen_height_px/(double)screen_width_px),
               vertical_fov_(vertical_fov_rad),
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

[[nodiscard]] std::vector<Triangle2D>
Camera::projectTriangleInWorld(const Triangle& tri_world) const{
  // Backface culling
  if(!isFacing(tri_world)) return {};

  // Transform triangle from world to camera coordinate frame
  auto original_tri_cam = tfTriangleWorldToCam(tri_world);

  // Clip triangle in cam coordinate frame by near plane
  auto near_clipped_tris_cam = clipNear(original_tri_cam);

  std::vector<Triangle2D> finished_2d_triangles;

  for(const auto &tri_cam : near_clipped_tris_cam) {
    cg::Triangle2D tri_img;

    for (int i = 0; i < 3; ++i){
      auto pt_cam = tri_cam.points[i];
      // Perspective transformation
      auto pt_cube = tfPointCameraToCube(pt_cam);

      // Scale triangles to screen size
      double screen_x = (pt_cube.x()/pt_cube.w() + 1) * screen_width_ / 2.0;
      double screen_y = (-pt_cube.y()/pt_cube.w() + 1) * screen_height_ / 2.0;

      // Carry the w value with the points for correcting for perspective on the
      // texture as well. We're not making a PS1 game
      tri_img.points[i] = Vector3d(screen_x, screen_y, 1); //<- in cartesian

      // Same perspective transformation on the texture
      tri_img.t[i] = Vector3d(tri_cam.t[i].x()/pt_cube.w(),
                              tri_cam.t[i].y()/pt_cube.w(),
                              tri_cam.t[i].z()/pt_cube.w());
    }

    // Clip 2D triangle in screen space
    auto tris_screen_clipped = clipScreen2D(tri_img);
    finished_2d_triangles.insert(finished_2d_triangles.end(),
                                 tris_screen_clipped.begin(),
                                 tris_screen_clipped.end());
  }

  return finished_2d_triangles;
}

bool Camera::isFacing(const Triangle& tri_world) const{
  // Transform triangle in world coordinate to camera's coordinate
  auto tri_cam = tfTriangleWorldToCam(tri_world);

  Vector3d cam_2_tri = tri_cam.points[0].head<3>();
  cam_2_tri.normalize();
  return cam_2_tri.dot(tri_cam.face_unit_normal()) < -EPS;
}

Triangle Camera::tfTriangleWorldToCam(const Triangle& tri_world) const{
  Triangle tri_cam;
  for(int i = 0; i < 3; ++i) {
    tri_cam.points[i] = tfPointWorldToCam(tri_world.points[i]);
    tri_cam.t[i] = tri_world.t[i];
  }

  return tri_cam;
}

Vector4d Camera::tfPointCameraToCube(const Vector4d &pt_cam)const{
  Eigen::RowVector4d row_pt(pt_cam);
  Eigen::RowVector4d pt = row_pt*projection_mat_;

  if(pt(3) == 0)
    throw std::invalid_argument(
        "Cannot project a point with z = 0; Perhaps clip first by the near "
        "plane.");
  return Vector4d(pt);
}

std::vector<Triangle> Camera::clipNear(const Triangle& tri_cam) const{
  // Near plane normal vector pointed in camera view direction
  Vector3d near_plane_unit_normal(0,0,-1);
  // Point on the near plane in camera coordinates
  Vector3d near_plane_pt(0,0,-near_plane_dist_);

  // Triangle is completely on the 'out' side of near plane. Nothing to keep.
  if(tri_cam.points[0].z() >= -near_plane_dist_
  && tri_cam.points[1].z() >= -near_plane_dist_
  && tri_cam.points[2].z() >= -near_plane_dist_)return {};

  // Triangle is completely on the 'in' side of the near plane.
  if(tri_cam.points[0].z() <= -near_plane_dist_
  && tri_cam.points[1].z() <= -near_plane_dist_
  && tri_cam.points[2].z() <= -near_plane_dist_)return {tri_cam};

  std::vector<Vector3d> in,out;
  std::vector<Vector2d> in_t, out_t; // Texture coordinates

  for(int i = 0 ; i < 3; ++i){
    int cur_idx = i;
    int next_idx = (i+1)%3;

    auto cur_pt = tri_cam.points[cur_idx].head<3>();
    auto next_pt = tri_cam.points[next_idx].head<3>();
    auto cur_tx = tri_cam.t[cur_idx].head<2>();
    auto next_tx = tri_cam.t[next_idx].head<2>();

    // Add current point in 'In' or 'Out'?
    if(cur_pt.z() > -near_plane_dist_){
      // Last point is not already there (for edge case when a point is
      // directly on the plane)
      if (out.empty() || (!out.empty() && !cur_pt.isApprox(out.back()))){
        out.emplace_back(cur_pt);
        out_t.emplace_back(cur_tx);
      }
    }
      // 'In' side
    else if(in.empty() || (!in.empty() && !cur_pt.isApprox(in.back()))){
      in.emplace_back(cur_pt);
      in_t.emplace_back(cur_tx);
    }

    double t;
    auto int_pt = planeLineIntersect(
        cur_pt, next_pt, near_plane_unit_normal, near_plane_pt, t);
    // Is there an intersection point to add?
    if(int_pt!=nullptr){
      // There is an intersection point
      in.push_back(*int_pt);
      out.push_back(*int_pt);

      // Compute the corresponding intersect point in texel space
      auto int_tx = cur_tx + (next_tx - cur_tx)*t;
      in_t.emplace_back(int_tx);
      out_t.emplace_back(int_tx);
    }
  }

  assert(in_t.size()==in.size());
  assert(out_t.size()==out.size());
  assert(in.size() == 3 || in.size() == 4);
  // Create triangles with 'In' points
  if(in.size() == 3) {
    return {Triangle(in[0],in[1],in[2],in_t[0],in_t[1],in_t[2])};
  }
  else
    return {
        Triangle(in[0],in[1],in[2],in_t[0],in_t[1],in_t[2]),
        Triangle(in[0],in[2],in[3],in_t[0],in_t[2],in_t[3])};
}

std::vector<Triangle2D>
Camera::clipScreen2D(const Triangle2D &tri_img) const{
  std::vector<Triangle2D> triangles{tri_img};
  // Clip on Left edge
  Vector2d left_edge_normal(1,0);
  Vector2d pt_on_left_edge(0,0);
  auto tris_after_left_clip = clip2DEdge(left_edge_normal,
                                         pt_on_left_edge,
                                         triangles);
  // Clip on Right edge
  Vector2d right_edge_normal(-1,0);
  Vector2d pt_on_right_edge(screen_width_,0);
  auto tris_after_right_clip = clip2DEdge(right_edge_normal,
                                          pt_on_right_edge,
                                          tris_after_left_clip);
  // Clip on Top edge
  Vector2d top_edge_normal(0,1);
  Vector2d pt_on_top_edge(0,0);
  auto tris_after_top_clip = clip2DEdge(top_edge_normal,
                                        pt_on_top_edge,
                                        tris_after_right_clip);
  // Clip on Bottom edge
  Vector2d bottom_edge_normal(0,-1);
  Vector2d pt_on_bottom_edge(0,screen_height_);
  auto tris_after_bottom_clip = clip2DEdge(bottom_edge_normal,
                                           pt_on_bottom_edge,
                                           tris_after_top_clip);

  return tris_after_bottom_clip;
}

Vector4d Camera::tfPointWorldToCam(const Vector4d &pt_world) const{
  return pose_world.matrix().inverse()*pt_world;
}

Vector2d Camera::projectPointInWorld(const Vector4d &pt_world) const{
  auto pt_cube = tfPointWorldToCube(pt_world);
  return Vector2d(pt_cube.x()/pt_cube.w(), -pt_cube.y()/pt_cube.w());
}

Vector4d Camera::tfPointWorldToCube(const Vector4d &pt_world) const{
  Vector4d pt_cam = tfPointWorldToCam(pt_world);
  return tfPointCameraToCube(pt_cam);
}

std::vector<Triangle2D>
Camera::clip2DEdge(const Vector2d &edge_unit_normal,
                   const Vector2d &pt_on_edge,
                   const std::vector<Triangle2D> &tris) const{

  std::vector<Triangle2D> clipped_triangles;

  for(const auto& tri: tris){
    Vector2d pt_to_line1 = tri.points[0].head<2>()-pt_on_edge;
    Vector2d pt_to_line2 = tri.points[1].head<2>()-pt_on_edge;
    Vector2d pt_to_line3 = tri.points[2].head<2>()-pt_on_edge;

    // d holds dot products; Will be used for determining which side of the
    // line the point is on.
    double d[3];
    if(pt_to_line1.norm() > cg::EPS){
      pt_to_line1.normalize();
      d[0] = edge_unit_normal.dot(pt_to_line1);
    }else d[0] = 0;
    if(pt_to_line2.norm() > cg::EPS){
      pt_to_line2.normalize();
      d[1] = edge_unit_normal.dot(pt_to_line2);
    }else d[1] = 0;
    if(pt_to_line3.norm() > cg::EPS){
      pt_to_line3.normalize();
      d[2] = edge_unit_normal.dot(pt_to_line3);
    }else d[2] = 0;

    // Triangle is completely on the 'out' side. Nothing to keep.
    if(d[0] <= EPS && d[1] <= EPS && d[2] <= EPS) continue;

    // Triangle is completely on the 'in' side.
    if(d[0] >= -EPS && d[1] >= -EPS && d[2] >= -EPS){
      clipped_triangles.push_back(tri);
      continue;
    }

    std::vector<Vector3d> in,out;

    // Texture coordinates need clip too. Vector3d because we can't forget
    // about the w value which will be used for texel perspective.
    std::vector<Vector3d> in_t, out_t;

    for(int i = 0; i < 3; ++i){
      int cur_idx = i;
      int next_idx = (i+1)%3;

      auto cur_pt = tri.points[cur_idx];
      auto next_pt = tri.points[next_idx];

      // Texture coordinates
      auto cur_tx = tri.t[cur_idx];
      auto next_tx = tri.t[next_idx];

      // Add current point in 'In' or 'Out'?
      if(d[cur_idx] < -EPS) { // 'Out' side
        // Last point is not already there (for edge case when a point is
        // directly on the plane)
        if (out.empty() || (!out.empty() && !cur_pt.isApprox(out.back()))){
          out.emplace_back(cur_pt);
          out_t.emplace_back(cur_tx);
        }
      }
      // 'In' side
      else if(in.empty() || (!in.empty() && !cur_pt.isApprox(in.back()))){
        in.emplace_back(cur_pt);
        in_t.emplace_back(cur_tx);
      }

      double t;
      auto int_pt = lineLineIntersect2d(
          cur_pt.head<2>(), next_pt.head<2>(), edge_unit_normal,
          pt_on_edge,t);
      // Is there an intersection point to add?
      // note: additional checks for t in range [0,1] for floating point
      // numerical accuracy reasons
      if(int_pt!=nullptr && t>=0.0 && t<=1.0){
        // There is an intersection point.
        Vector3d intersection_point;
        intersection_point.head<2>() = *int_pt;
        intersection_point.z() = 1;
        in.emplace_back(intersection_point);
        out.emplace_back(intersection_point);

        // Compute intersection point in texel space
        auto int_tx = cur_tx + (next_tx-cur_tx)*t;
        in_t.emplace_back(int_tx);
        out_t.emplace_back(int_tx);
      }
    }

    assert(in_t.size() == in.size());
    assert(out_t.size() == out.size());
    assert(in.size() == 3 || in.size() == 4);
    // Add clipped triangles on the 'in' side of the line.
    if(in.size() == 3) {
      Triangle2D new_tri;
      new_tri.points[0] = in[0];
      new_tri.points[1] = in[1];
      new_tri.points[2] = in[2];
      new_tri.t[0] = in_t[0];
      new_tri.t[1] = in_t[1];
      new_tri.t[2] = in_t[2];
      clipped_triangles.push_back(std::move(new_tri));
    }
    else{
      Triangle2D new_tri1, new_tri2;
      new_tri1.points[0] = in[0];
      new_tri1.points[1] = in[1];
      new_tri1.points[2] = in[2];
      new_tri1.t[0] = in_t[0];
      new_tri1.t[1] = in_t[1];
      new_tri1.t[2] = in_t[2];

      new_tri2.points[0] = in[0];
      new_tri2.points[1] = in[2];
      new_tri2.points[2] = in[3];
      new_tri2.t[0] = in_t[0];
      new_tri2.t[1] = in_t[2];
      new_tri2.t[2] = in_t[3];
      clipped_triangles.emplace_back(std::move(new_tri1));
      clipped_triangles.emplace_back(std::move(new_tri2));
    }
  }

  return clipped_triangles;
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
  new_cam_orientation.col(0).head<3>() = -right_vec;
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
